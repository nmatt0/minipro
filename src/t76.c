/*
 * t76.c - Low level ops for T76
 *
 * This file is a part of Minipro.
 *
 * Minipro is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Minipro is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <time.h>

#include "database.h"
#include "minipro.h"
#include "bitbang.h"
#include "t76.h"
#include "usb.h"

#define T76_BEGIN_TRANS_LOGIC	  0x02 /* 64-byte NAND/logic FPGA-setup prelude */
#define T76_BEGIN_TRANS		  0x03
#define T76_END_TRANS		  0x04
#define T76_READID		  0x05
#define T76_READ_USER		  0x06
#define T76_WRITE_USER		  0x07
#define T76_READ_CFG		  0x08
#define T76_WRITE_CFG		  0x09
#define T76_WRITE_USER_DATA	  0x0A
#define T76_READ_USER_DATA	  0x0B
#define T76_WRITE_CODE		  0x0C
#define T76_READ_CODE		  0x0D
#define T76_ERASE		  0x0E
#define T76_TEST_RAM		  0x0F
#define T76_READ_DATA		  0x10
#define T76_WRITE_DATA		  0x11
#define T76_WRITE_LOCK		  0x14
#define T76_READ_LOCK		  0x15
#define T76_READ_CALIBRATION	  0x16
#define T76_PROTECT_OFF		  0x18
#define T76_PROTECT_ON		  0x19
#define T76_READ_JEDEC		  0x1D
#define T76_WRITE_JEDEC		  0x1E
#define T76_NAND_PROGRAM	  0x1F /* per-block NAND program (init + per-page EP05) */
#define T76_WRITE_BITSTREAM	  0x26
#define T76_LOGIC_IC_TEST_VECTOR  0x28
#define T76_AUTODETECT		  0x37
#define T76_UNLOCK_TSOP48	  0x38
#define T76_REQUEST_STATUS	  0x39
#define T76_NAND_BAD_BLOCK_CHECK  0x3A
#define T76_BOOTLOADER_WRITE	  0x3B
#define T76_BOOTLOADER_ERASE	  0x3C
#define T76_SWITCH		  0x3D
#define T76_PIN_DETECTION	  0x3E

/* Firmware */
#define T76_UPDATE_FILE_VERS_MASK 0xffff0000
#define T76_FIRMWARE_VERS_MASK	  0x0000ffff
#define T76_UPDATE_FILE_VERSION	  0xf0760000
#define T76_BTLDR_MAGIC		  0x049000

/* Device algorithm numbers used to autodetect 8/16 pin SPI devices.
 * This will select algorithm 'SPI25F11' and 'SPI25F21'
 * which are used for 25 SPI autodetection.
 * This is the high byte from the 'variant' field
 */
#define SPI_DEVICE_8P		  0x11
#define SPI_DEVICE_16P		  0x21
#define SPI_PROTOCOL		  0x03

/* T76_WRITE_BITSTREAM will take this as the first argument */
#define T76_BEGIN_BS		  0x00
#define T76_BS_BLOCK		  0x01
#define T76_END_BS		  0x02

/* Used to reset the FPGA */
#define T76_RESET_FPGA		  0xaf
#define T76_FPGA_MAGIC		  0xaa55ddee

/* We'll send the bistream algorithm in 512 bytes chunks */
#define BS_PACKET_SIZE		  0x200

/* Write FPGA */
static int t76_write_bitstream(minipro_handle_t *handle)
{
	/* Send the bitstream algorithm to the T76.
	 * First send the begin bitstream command.
	 * We must specify the packet size used and total bitstream size
	 */
	uint8_t msg[BS_PACKET_SIZE];
	algorithm_t *algorithm = &handle->device->algorithm;
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T76_WRITE_BITSTREAM;
	msg[1] = T76_BEGIN_BS;
	format_int(&msg[2], BS_PACKET_SIZE, 2, MP_LITTLE_ENDIAN);
	format_int(&msg[4], algorithm->length, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8)) {
		free(algorithm->bitstream);
		return EXIT_FAILURE;
	}

	/* Check if okay to begin write */
	if (msg_recv(handle->usb_handle, msg, 8) || msg[1])
		return EXIT_FAILURE;

	/* Now send the bitstream in 512 bytes chunks.
	 * Each packet will consist of 8 bytes header
	 * and 504 bytes payload
	 */
	size_t payload_size = BS_PACKET_SIZE - 8;
	memset(msg, 0x00, sizeof(msg));
	for (size_t i = 0; i < algorithm->length; i += payload_size) {

		/* Handle last block size */
		size_t block_size = ((i + payload_size) <= algorithm->length) ?
		                        payload_size :
		                        (algorithm->length - i);

		msg[0] = T76_WRITE_BITSTREAM;
		msg[1] = T76_BS_BLOCK;
		format_int(&msg[2], block_size, 2, MP_LITTLE_ENDIAN);
		memcpy(&msg[8], &algorithm->bitstream[i], block_size);
		if (msg_send(handle->usb_handle, msg, BS_PACKET_SIZE)) {
			free(algorithm->bitstream);
			return EXIT_FAILURE;
		}
	}

	/* Send the end of bitstream command. The vendor puts the size of the
	 * final (partial) block in msg[2..3] here (e.g. 0x169 = 361 for a
	 * 775513-byte / 504-chunk bitstream), so the FPGA finalizes the last
	 * config word. minipro sent 0, which leaves a NAND FPGA mis-finalized
	 * (READID/read return 0xFF). Carry the real last-block size for NAND.
	 * TODO: generalize to all T76 chip classes once validated. */
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T76_WRITE_BITSTREAM;
	msg[1] = T76_END_BS;
	if (handle->device->protocol_id == IC2_ALG_NAND) {
		size_t last_block = algorithm->length % payload_size;
		if (last_block == 0)
			last_block = payload_size;
		format_int(&msg[2], last_block, 2, MP_LITTLE_ENDIAN);
	}
	if (msg_send(handle->usb_handle, msg, 8)) {
		free(algorithm->bitstream);
		return EXIT_FAILURE;
	}

	/* Check if the bitstream transfer was okay. */
	if (msg_recv(handle->usb_handle, msg, 8) || msg[1]) {
		free(algorithm->bitstream);
		return EXIT_FAILURE;
	}

	free(algorithm->bitstream);
	return EXIT_SUCCESS;
}

/* Send the required bitstream algorithm to T76 */
/* Issue one 0x24 (FPGA register I/O) command. The command word carries the
 * response length in msg[2..3]; the device returns that many bytes on EP81
 * which MUST be read back, otherwise the next transfer desyncs (and a
 * 0xf0 power-down left undrained wedges the device until a USB replug).
 * Mirrors vendor t76_cmd_24_fpga_register_io @0x455ec0. */
static int t76_cmd_24(minipro_handle_t *handle, const uint8_t cmd[8])
{
	uint8_t msg[8], rsp[64];
	uint16_t recv_len = cmd[2] | ((uint16_t)cmd[3] << 8);
	memcpy(msg, cmd, 8);
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (recv_len) {
		if (recv_len > sizeof(rsp))
			recv_len = sizeof(rsp);
		if (msg_recv(handle->usb_handle, rsp, recv_len))
			return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

/* One-time socket-adapter power/init at session start. XGPro detects and
 * ENERGIZES the adapter via this 0x24 sequence before any chip op; the BGA
 * NAND adapter needs it (without it the NAND is never selected -> READID
 * reads 0xFF and the data read gets no EP82 data). Power-down (drains 8),
 * read-adapter-ID (drains 0x30), power-up. Bytes replayed from the read5
 * capture (W29N02GZ). TODO: derive from t76_adapter_detect @0x455670 /
 * t76_adapter_compat_check @0x455bd0 rather than hardcoding, and validate
 * the returned adapter ID. */
static int t76_adapter_init(minipro_handle_t *handle)
{
	static const uint8_t pwr_down[8] = { 0x24, 0xf0, 0x08, 0x00,
					     0x01, 0x00, 0x00, 0x00 };
	static const uint8_t read_id[8]  = { 0x24, 0xe4, 0x30, 0x00,
					     0x11, 0x01, 0x08, 0x00 };
	static const uint8_t pwr_up[8]   = { 0x24, 0xf1, 0x00, 0x00,
					     0x00, 0x00, 0x00, 0x00 };
	if (t76_cmd_24(handle, pwr_down) || t76_cmd_24(handle, read_id) ||
	    t76_cmd_24(handle, pwr_up))
		return EXIT_FAILURE;

	/* Pin detection (0x3e, 16-byte T76 form), run twice like XGPro right
	 * after adapter power-up and before the bitstream. On the T76 this
	 * configures the socket pin drivers for the selected package; without
	 * it the NAND lines are never driven and the chip reads as an open bus
	 * (0xFF). Returns a 32-byte bad-pin bitmask which we drain. */
	for (int i = 0; i < 2; i++) {
		uint8_t pd[16] = { 0 }, rsp[32];
		pd[0] = T76_PIN_DETECTION;
		if (msg_send(handle->usb_handle, pd, sizeof(pd)))
			return EXIT_FAILURE;
		if (msg_recv(handle->usb_handle, rsp, sizeof(rsp)))
			return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

static int t76_send_bitstream(minipro_handle_t *handle)
{
	/* Don't upload the bitstream again if we are in the same session */
	if (handle->bitstream_uploaded)
		return EXIT_SUCCESS;

	/* Get the required FPGA bitstream algorithm
	* Logic chips are not handled here
	*/
	device_t *device = handle->device;

	/* NAND: energize/init the socket adapter before the first bitstream. */
	if (device->protocol_id == IC2_ALG_NAND) {
		if (t76_adapter_init(handle))
			return EXIT_FAILURE;
	}

	if (get_algorithm(device, MP_T76, handle->cmdopts->algo_path,
			  handle->cmdopts->icsp, handle->cmdopts->vopt))
		return EXIT_FAILURE;

	fprintf(stderr, "Using T76 %s algorithm..\n", device->algorithm.name);

	if (t76_write_bitstream(handle)) {
		return EXIT_FAILURE;
	}

	handle->bitstream_uploaded = 1;
	return EXIT_SUCCESS;
}

int t76_begin_transaction(minipro_handle_t *handle)
{
	uint8_t msg[128] = { 0 };
	size_t msglen = 64;
	uint8_t ovc;
	device_t *device = handle->device;

	if (t76_send_bitstream(handle)) {
		fprintf(stderr, "An error occurred while sending bitstream.\n");
		return EXIT_FAILURE;
	}

	/* T76 FPGA was initialized, we can send the normal begin_transaction command */
	if (!handle->device->flags.custom_protocol) {
		msg[0] = T76_BEGIN_TRANS;
		msg[1] = device->protocol_id;
		msg[2] = (uint8_t)device->variant;
		msg[3] = handle->cmdopts->icsp;

		format_int(&(msg[4]), device->voltages.raw_voltages, 2,
			   MP_LITTLE_ENDIAN);
		msg[6] = (uint8_t)device->chip_info;
		msg[7] = (uint8_t)device->pin_map;
		format_int(&(msg[8]), device->data_memory_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[10]), device->page_size, 2, MP_LITTLE_ENDIAN);
		format_int(&(msg[12]), device->pulse_delay, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[14]), device->data_memory2_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[16]), device->code_memory_size, 4,
			   MP_LITTLE_ENDIAN);

		msg[20] = (uint8_t)(device->voltages.raw_voltages >> 16);

		if ((device->voltages.raw_voltages & 0xf0) == 0xf0) {
			msg[22] = (uint8_t)device->voltages.raw_voltages;
		} else {
			msg[21] = (uint8_t)device->voltages.raw_voltages & 0x0f;
			msg[22] = (uint8_t)device->voltages.raw_voltages & 0xf0;
		}
		if (device->voltages.raw_voltages & 0x80000000)
			msg[22] = (device->voltages.raw_voltages >> 16) & 0x0f;

		/* I2C address if supported or zero */
		if (device->flags.can_adjust_address) {
			msg[24] = device->i2c_address;
		}

		/* SPI clock  if supported or zero */
		if (device->flags.can_adjust_clock) {
			msg[28] = device->spi_clock;
		}

		format_int(&(msg[40]), device->package_details.packed_package,
			   4, MP_LITTLE_ENDIAN);
		format_int(&(msg[44]), device->read_buffer_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[56]), device->flags.raw_flags, 4,
			   MP_LITTLE_ENDIAN);

		/* Algorithm number */
		msg[63] = (uint8_t)(device->variant >> 8);

		/* The T76 BEGIN_TRANS is 128 bytes. Bytes 0x40..0x7f carry a
		 * chip-class geometry / SPI-setup block that the FPGA requires
		 * to drive memory access; minipro historically sent only 64
		 * bytes, which makes SPI-NOR reads clock out all zeros.
		 *
		 * For SPI 25-series NOR (protocol_id 3 / 0xF) the vendor's
		 * packer (sub_409850) selects a read-setup pair by adapter,
		 * then writes an SPI-timing dword and a clock-cfg byte. Values
		 * below are confirmed against real hardware:
		 *   8-pin  (ZB25VQ64A):   msg[0x40]=0x08000000 msg[0x50]=0x00800000
		 *   16-pin (MX25L12845E): msg[0x40]=0x00020000 msg[0x50]=0x02000000
		 *   both:                 msg[0x60]=0x0f05172f msg[0x65]=0x03
		 * All four are individually load-bearing (dropping any one
		 * reverts the read to all-zeros). The 8P/16P split is keyed off
		 * the algorithm/package nibble in `variant >> 8`. TODO: this is
		 * still adapter/clock-specific (only the standard adapter at
		 * 16 MHz is confirmed) — generalize from more chips. */
		if (device->protocol_id == IC2_ALG_SPI25F_1 ||
		    device->protocol_id == IC2_ALG_SPI25F_2) {
			if ((uint8_t)(device->variant >> 8) == SPI_DEVICE_16P) {
				format_int(&msg[0x40], 0x00020000, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x50], 0x02000000, 4,
					   MP_LITTLE_ENDIAN);
			} else {
				format_int(&msg[0x40], 0x08000000, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x50], 0x00800000, 4,
					   MP_LITTLE_ENDIAN);
			}
			format_int(&msg[0x60], 0x0f05172f, 4, MP_LITTLE_ENDIAN);
			msg[0x65] = 0x03;
			msglen = 128;
		}

		/* Parallel NOR / NAND / eMMC (protocol_id 0x12/0x14) use vendor
		 * packer sub_4b5a70 for the 0x40..0x7f BEGIN extension. Its
		 * inputs map to existing fields: chip-family signature =
		 * package_details (data_7aee14), adapter nibble = variant & 0xf0
		 * (data_7aede8), geometry select = variant & 0x0f.
		 *
		 * Implemented here: the parallel-NOR x16 family (package_details
		 * low byte 0x0b) with the >=8 geometry branch. Verified
		 * READ + ERASE bit-correct on an S29GL512N (512 Mbit, TSOP-56,
		 * adapter nibble 0x50 -> msg[0x48]=0x800). NOTE: parallel-NOR
		 * *program* (0x11) is still non-functional (needs its own
		 * per-command descriptor); read and erase work.
		 * TODO: other families/geometries and NAND/eMMC branches of
		 * sub_4b5a70 (need more chips + a vendor write capture). */
		if (device->protocol_id == IC2_ALG_T48 ||
		    device->protocol_id == IC2_ALG_T40B) {
			uint8_t family =
				(uint8_t)device->package_details.packed_package;
			uint8_t adapter = device->variant & 0xf0;
			uint8_t geom = device->variant & 0x0f;
			if (family == 0x0b && geom >= 8) {
				uint32_t b48;
				format_int(&msg[0x40], 0x01000000, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x44], 0x00000040, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x50], 0x10000000, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x54], 0x00008000, 4,
					   MP_LITTLE_ENDIAN);
				switch (adapter) {
				case 0x10: b48 = 0x0200; break;
				case 0x20: b48 = 0x1200; break;
				case 0x30: b48 = 0x0a00; break;
				case 0x40: b48 = 0x1000; break;
				case 0x50: b48 = 0x0800; break;
				case 0x60: b48 = 0x1800; break;
				case 0x70: b48 = 0x0400; break;
				default:   b48 = 0x0800; break;
				}
				format_int(&msg[0x48], b48, 4,
					   MP_LITTLE_ENDIAN);
				format_int(&msg[0x60], 0x0f05172f, 4,
					   MP_LITTLE_ENDIAN);
				msg[0x65] = 0x03;
				msglen = 128;
			}
		}

		/* NAND (protocol_id 0x2d): the FPGA algorithm bitstream drives
		 * the NAND command/address bus, so there is NO 0x40..0x5f
		 * geometry block (the vendor's chip-family signature is 0 here,
		 * so no BEGIN packer fires). The only required extension bytes
		 * are the clock/timing dword and its cfg byte, plus the 128-byte
		 * length. Values from a Cynthion capture of XGPro reading a
		 * W29N02GZ (pcaps/read5): msg[0x60]=0x0b09272f (a lower clock
		 * tier than SPI/NOR's 0x0f05172f), msg[0x65]=0x03. */
		if (device->protocol_id == IC2_ALG_NAND) {
			/* The NAND read engine needs the per-block transfer
			 * size (data + spare) at msg[0x10], NOT the total chip
			 * size, so it streams exactly one block per 0x0d. Also
			 * set the NAND flag bit 0x800 in raw_flags (vendor
			 * post-load adjustment). Matches the read5 capture. */
			if (device->pages_per_block)
				format_int(&msg[16],
					   (uint32_t)device->write_buffer_size *
						   device->pages_per_block,
					   4, MP_LITTLE_ENDIAN);
			format_int(&msg[56],
				   device->flags.raw_flags | 0x800, 4,
				   MP_LITTLE_ENDIAN);
			format_int(&msg[0x60], 0x0b09272f, 4,
				   MP_LITTLE_ENDIAN);
			msg[0x65] = 0x03;
			/* Remaining bytes that differ from the vendor BEGIN
			 * (read5): forced to match byte-for-byte while we
			 * determine which are load-bearing for the NAND read.
			 * 0x28 is the NAND pin/family byte (0xe2 in V13.19 DB
			 * vs minipro's stale 0x22); 0x0e/0x14/0x18/0x1c/0x30
			 * are voltage/reserved fields. TODO: source these
			 * properly (DB refresh / derive) once ablated. */
			msg[0x0e] = 0x20;
			msg[0x14] = 0x00;
			msg[0x18] = 0x03;
			msg[0x1c] = 0x03;
			format_int(&msg[0x28], 0xe2000000, 4,
				   MP_LITTLE_ENDIAN);
			msg[0x30] = 0x40;
			msglen = 128;

			/* NAND chips register as chip_type 0x2d, which makes the
			 * vendor emit a 64-byte opcode-0x02 "logic begin" prelude
			 * immediately BEFORE the 0x03 BEGIN_TRANS (vendor packer
			 * site t76_begin_transaction_full @ 0x443322). This prelude
			 * programs the FPGA's NAND page/block geometry and bus clock;
			 * without it the FPGA never clocks the NAND, so READID returns
			 * 00 FF FF and the first 0x0d read times out. minipro
			 * historically skipped it (confirmed by a same-machine USBPcap
			 * diff of minipro.exe vs XGPro on a W29N02GZ, 2026-05-29).
			 *
			 * Field map (vendor packer, the chip_type==0x2d block ending
			 * at the usb_msg_send_ep01 of 0x40 bytes @ 0x443322):
			 *   [0x00]   = 0x02 opcode
			 *   [0x08].w = data_7a5cc0 = pages_per_block
			 *   [0x0a].w = data_7a5cc2 = page_size (the value the
			 *              page-size-code switch keys off)
			 *   [0x0c].w = data_7a5cc4 = page_size
			 *   [0x0e].w = data_7a5cc6 = pages_per_block
			 *   [0x10].w = data_7a5cc8 } plane/LUN counts (1/1 for the
			 *   [0x12].w = data_7a5cca }  single-die W29N02GZ)
			 *   [0x14].d = data_7a5ccc = bus/width code (3)
			 *   [0x18].d = page-size code: <2048->4, ==2048->8,
			 *              ==4096->4, ==16384->1, else->2
			 *   [0x1c].d = (data_7a5cdc != 0) -> 0 here
			 *   [0x20].d = data_7a5cd4 (adapter-mode byte at +2 = 1)
			 *   [0x24].d = NAND bus clock, table[data_80dc6c & 7]
			 * Geometry comes straight from the device profile; the
			 * plane/width/mode/clock fields are programmer-fixed and kept
			 * as documented constants from the W29N02GZ capture (only one
			 * NAND part validated so far). Confirmed: with this prelude
			 * READID returns the real 0xEFAA and the full 264 MiB array
			 * reads back deterministically (md5 22a14421...); without it
			 * the FPGA never clocks the NAND (READID 00 FF FF, 0x0d
			 * timeout). TODO: validate plane/width/clock on more NAND
			 * parts and source the clock from the bus-clock table. */
			{
				uint8_t pre[64] = { 0 };
				uint16_t ps = (uint16_t)device->page_size;
				uint16_t ppb = device->pages_per_block;
				uint32_t ps_code = (ps < 0x800)   ? 4 :
						   (ps == 0x800)  ? 8 :
						   (ps == 0x1000) ? 4 :
						   (ps == 0x4000) ? 1 :
								    2;
				pre[0] = T76_BEGIN_TRANS_LOGIC; /* 0x02 */
				format_int(&pre[0x08], ppb, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x0a], ps, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x0c], ps, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x0e], ppb, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x10], 1, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x12], 1, 2, MP_LITTLE_ENDIAN);
				format_int(&pre[0x14], 3, 4, MP_LITTLE_ENDIAN);
				format_int(&pre[0x18], ps_code, 4, MP_LITTLE_ENDIAN);
				format_int(&pre[0x20], 0x00010000, 4, MP_LITTLE_ENDIAN);
				format_int(&pre[0x24], 0x27154f3b, 4, MP_LITTLE_ENDIAN);
				if (msg_send(handle->usb_handle, pre, sizeof(pre)))
					return EXIT_FAILURE;
			}
		}

		if (msg_send(handle->usb_handle, msg, msglen))
			return EXIT_FAILURE;
	} else {
		if (bb_begin_transaction(handle)) {
			return EXIT_FAILURE;
		}
	}

	if (t76_get_ovc_status(handle, NULL, &ovc))
		return EXIT_FAILURE;
	if (ovc) {
		fprintf(stderr, "Overcurrent protection!\007\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

int t76_end_transaction(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_end_transaction(handle);
	}
	uint8_t msg[8] = { 0 };
	msg[0] = T76_END_TRANS;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

/* Specific to T76. Sends a reset FPGA command */
int t76_reset_fpga(minipro_handle_t *handle)
{
	/* Reset FPGA */
	uint8_t msg[8] = { 0 };
	msg[0] = T76_WRITE_BITSTREAM;
	msg[1] = T76_RESET_FPGA;
	format_int(&msg[4], T76_FPGA_MAGIC, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8)) {
		return EXIT_FAILURE;
	}
	return (msg_recv(handle->usb_handle, msg, 8) || msg[1]);
}

/* T76 read block. Not compatible with previous versions. */
int t76_read_block(minipro_handle_t *handle, data_set_t *ds)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_block(handle, ds->type, ds->address, ds->data,
				     ds->size);
	}

	/* NAND read: one erase-block (data + spare) per command. The vendor
	 * (XGPro V13.19) issues a 0x0d per block with the 16-bit block index in
	 * msg[2..3] and a fixed NAND read-parameter header in msg[4..0xf]
	 * (captured from a W29N02GZ read, pcaps/read5). The whole block is then
	 * streamed raw on EP82. ds->size is set to the block-with-spare size by
	 * read_page_ram, and ds->address steps by ds->size, so block index =
	 * ds->address / ds->size.
	 * TODO: the fixed header (msg[4..0xf]) is replayed verbatim; derive it
	 * from the read-param setup (vendor sub_4f1fc0) instead of hardcoding. */
	if (handle->device->protocol_id == IC2_ALG_NAND && ds->type == MP_CODE) {
		uint8_t msg[16] = { 0 };
		static const uint8_t nand_read_hdr[12] = {
			0x10, 0x00, 0x04, 0x00, /* msg[4..7]  */
			0x08, 0x00, 0x08, 0x00, /* msg[8..b]  */
			0x69, 0x01, 0x00, 0x00, /* msg[c..f]  */
		};
		uint32_t block_index = ds->size ? (ds->address / ds->size) : 0;
		msg[0] = T76_READ_CODE;
		format_int(&msg[2], block_index, 2, MP_LITTLE_ENDIAN);
		memcpy(&msg[4], nand_read_hdr, sizeof(nand_read_hdr));
		if (msg_send(handle->usb_handle, msg, 16))
			return EXIT_FAILURE;
		return read_payload(handle->usb_handle, ds->data, ds->size);
	}

	uint8_t msg[64] = { 0 };
	format_int(&msg[2], ds->size, 2, MP_LITTLE_ENDIAN);
	format_int(&msg[4], ds->address, 4, MP_LITTLE_ENDIAN);

	/* MP_CODE handling */
	if (ds->type == MP_CODE) {
		msg[0] = T76_READ_CODE;
		format_int(&msg[8], ds->block_count, 4, MP_LITTLE_ENDIAN);

		/* If init is set then this is the first call */
		if (ds->init) {
			if (msg_send(handle->usb_handle, msg, 16)) {
				return EXIT_FAILURE;
			}
		}

		return read_payload(handle->usb_handle, ds->data, ds->size);

		/* MP_DATA handling */
	} else if (ds->type == MP_DATA) {
		msg[0] = T76_READ_DATA;

		if (msg_send(handle->usb_handle, msg, 16)) {
			return EXIT_FAILURE;
		}

		uint8_t *data = malloc(ds->size + 16);
		if (!data) {
			fprintf(stderr, "Out of memory!\n");
			return EXIT_FAILURE;
		}

		int ret = read_payload(handle->usb_handle, data, ds->size + 16);
		if (!ret) {
			memcpy(ds->data, data + 16, ds->size);
		}
		free(data);
		return ret;

		/* MP_USER handling */
	} else if (ds->type == MP_USER) {
		msg[0] = T76_READ_USER_DATA;

		format_int(&msg[2], ds->size, 2, MP_LITTLE_ENDIAN);
		format_int(&msg[4], ds->address, 4, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, msg, 16)) {
			return EXIT_FAILURE;
		}

		uint8_t *data = malloc(ds->size + 16);
		if (!data) {
			fprintf(stderr, "Out of memory!\n");
			return EXIT_FAILURE;
		}

		int ret = msg_recv(handle->usb_handle, data, ds->size + 16);
		if (!ret) {
			memcpy(ds->data, data + 16, ds->size);
		}
		free(data);
		return ret;
	}

	fprintf(stderr, "Unknown type for read_block (%d)\n", ds->type);
	return EXIT_FAILURE;
}

/* T76 write block. Not compatible with previous versions. */
int t76_write_block(minipro_handle_t *handle, data_set_t *ds)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_block(handle, ds->type, ds->address, ds->data,
				      ds->size);
	}

	/* NAND program (protocol_id 0x2d). The vendor T76 path (sub_4cf2c0)
	 * does NOT use the bulk 0x0C WRITE_CODE: it sends a per-block init with
	 * opcode 0x1F, then streams each page (page+spare) as a separate
	 * EP05 packet prefixed by a 16-byte header, reading a 0x20-byte status
	 * from EP83 after each page. read_page_ram/write_page_ram feed one
	 * erase-block (write_buffer_size * pages_per_block) per call here, so
	 * ds->data holds the whole block (pages interleaved with their spare,
	 * exactly as t76_read_block returns it) and block index =
	 * ds->address / ds->size.
	 *   Init (0x1F, 16B): msg[2..3]=page+spare, msg[4..7]=block,
	 *                     msg[8..b]=pages_per_block, msg[0xc..0xf]=page+spare.
	 *   Per page: [16B hdr (hdr[0]=0x1F) | page+spare] -> EP05.
	 *   Commit:   a plain 0x39 REQUEST_STATUS after the 64 pages.
	 *
	 * The firmware cache-programs: page N commits when page N+1's data
	 * arrives, so the block's LAST page (63) is left pending. The 0x39 poll
	 * after the page stream waits for that final program to complete (and
	 * returns block status), then the next block's 0x1F init follows. Without
	 * the 0x39, page 63 of every block reads back erased (observed). Confirmed
	 * by a capture of XGPro programming this part (pcaps/xgpro-2): per block it
	 * is 0x1F init, 64x EP05 page writes, then a plain `39 00..` (8B) before
	 * the next 0x1F. The EP83 reads the vendor also posts are overlapped and
	 * aborted (complete with 0 bytes) — telemetry, NOT the commit mechanism.
	 *
	 * NOTE: no host-side ECC here — the image is written raw (data+spare as
	 * read back), matching a raw read/restore. ECC-managed writes (vendor
	 * NandDLL) are a separate future path. */
	if (handle->device->protocol_id == IC2_ALG_NAND && ds->type == MP_CODE) {
		device_t *device = handle->device;
		uint16_t page_full = device->write_buffer_size;
		uint16_t ppb = device->pages_per_block;
		uint32_t block_index = ds->size ? (ds->address / ds->size) : 0;
		uint8_t imsg[16] = { 0 };

		imsg[0] = T76_NAND_PROGRAM;
		format_int(&imsg[2], page_full, 2, MP_LITTLE_ENDIAN);
		format_int(&imsg[4], block_index, 4, MP_LITTLE_ENDIAN);
		format_int(&imsg[8], ppb, 4, MP_LITTLE_ENDIAN);
		format_int(&imsg[12], page_full, 4, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, imsg, 16))
			return EXIT_FAILURE;

		uint8_t *pkt = malloc((size_t)page_full + 16);
		if (!pkt) {
			fprintf(stderr, "Out of memory!\n");
			return EXIT_FAILURE;
		}
		for (uint32_t p = 0; p < ppb; p++) {
			memset(pkt, 0, 16);
			pkt[0] = T76_NAND_PROGRAM;
			memcpy(pkt + 16, ds->data + (size_t)p * page_full,
			       page_full);
			if (write_payload(handle->usb_handle, pkt,
					   (size_t)page_full + 16)) {
				free(pkt);
				return EXIT_FAILURE;
			}
		}
		free(pkt);

		/* Commit the block: a plain 0x39 REQUEST_STATUS waits for the
		 * last page's program to finish and returns block status. */
		uint8_t st[32] = { 0 };
		st[0] = T76_REQUEST_STATUS;
		if (msg_send(handle->usb_handle, st, 8) ||
		    msg_recv(handle->usb_handle, st, sizeof(st)))
			return EXIT_FAILURE;
		return EXIT_SUCCESS;
	}

	uint8_t msg[64] = { 0 };
	format_int(&msg[2], ds->size, 2, MP_LITTLE_ENDIAN);
	format_int(&msg[4], ds->address, 4, MP_LITTLE_ENDIAN);
	format_int(&msg[12], ds->size, 4, MP_LITTLE_ENDIAN);

	/* MP_CODE handling */
	if (ds->type == MP_CODE) {
		msg[0] = T76_WRITE_CODE;

		/* If init is set then this is the first call */
		if (ds->init) {
			format_int(&msg[8], ds->block_count, 4,
				   MP_LITTLE_ENDIAN);
			if (msg_send(handle->usb_handle, msg, 16)) {
				return EXIT_FAILURE;
			}
		}

		memset(&msg[8], 0x00, 4);
		uint8_t *data = malloc(ds->size + 16);
		if (!data) {
			fprintf(stderr, "Out of memory!\n");
			return EXIT_FAILURE;
		}

		memcpy(data, msg, 16);
		memcpy(data + 16, ds->data, ds->size);
		int ret =
			write_payload(handle->usb_handle, data, ds->size + 16);
		free(data);
		return ret;

		/* MP_DATA handling */
	} else if (ds->type == MP_DATA) {
		msg[0] = T76_WRITE_DATA;

		if (msg_send(handle->usb_handle, msg, 16)) {
			return EXIT_FAILURE;
		}

		return write_payload(handle->usb_handle, ds->data, ds->size);

		/* MP_USER handling */
	} else if (ds->type == MP_USER) {
		msg[0] = T76_WRITE_USER_DATA;
		memcpy(&msg[16], ds->data, ds->size);
		return msg_send(handle->usb_handle, msg, 16 + ds->size);
	}

	fprintf(stderr, "Unknown type for write_block (%d)\n", ds->type);
	return EXIT_FAILURE;
}

int t76_read_fuses(minipro_handle_t *handle, uint8_t type, size_t length,
		   uint8_t items_count, uint8_t *buffer)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_fuses(handle, type, length, items_count, buffer);
	}
	uint8_t msg[64] = { 0 };

	if (type == MP_FUSE_USER) {
		type = T76_READ_USER;
	} else if (type == MP_FUSE_CFG) {
		type = T76_READ_CFG;
	} else if (type == MP_FUSE_LOCK) {
		type = T76_READ_LOCK;
	} else {
		fprintf(stderr, "Unknown type for read_fuses (%d)\n", type);
		return EXIT_FAILURE;
	}

	msg[0] = type;
	msg[1] = handle->device->protocol_id;
	msg[2] = items_count;
	format_int(&msg[4], handle->device->code_memory_size, 4,
		   MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;
	memcpy(buffer, &(msg[8]), length);
	return EXIT_SUCCESS;
}

int t76_write_fuses(minipro_handle_t *handle, uint8_t type, size_t length,
		    uint8_t items_count, uint8_t *buffer)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_fuses(handle, type, length, items_count,
				      buffer);
	}
	uint8_t msg[64] = { 0 };

	if (type == MP_FUSE_USER) {
		type = T76_WRITE_USER;
	} else if (type == MP_FUSE_CFG) {
		type = T76_WRITE_CFG;
	} else if (type == MP_FUSE_LOCK) {
		type = T76_WRITE_LOCK;
	} else {
		fprintf(stderr, "Unknown type for write_fuses (%d)\n", type);
	}

	msg[0] = type;
	if (buffer) {
		msg[1] = handle->device->protocol_id;
		msg[2] = items_count;
		format_int(&msg[4], handle->device->code_memory_size - 0x38, 4,
			   MP_LITTLE_ENDIAN); /* 0x38, firmware bug? */
		memcpy(&(msg[8]), buffer, length);
	}
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

int t76_read_calibration(minipro_handle_t *handle, uint8_t *buffer, size_t len)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_calibration(handle, buffer, len);
	}
	uint8_t msg[64] = { 0 };
	msg[0] = T76_READ_CALIBRATION;
	format_int(&(msg[2]), len, 2, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;
	return msg_recv(handle->usb_handle, buffer, len);
}

int t76_get_chip_id(minipro_handle_t *handle, uint8_t *type,
		    uint32_t *device_id)
{
	if (handle->device->flags.custom_protocol) {
		return bb_get_chip_id(handle, device_id);
	}
	uint8_t msg[32] = { 0 }, format, id_length;
	msg[0] = T76_READID;
	/* NOTE: XGPro leaves msg[1..7] as uninitialized stack garbage here
	 * (read5 sent dd ca 01, read7 sent db 11 07 — varies per run, both read
	 * the real ID), so these bytes are NOT load-bearing. minipro sends them
	 * zeroed, which is correct. */
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;

	*type = msg[0]; /* The Chip ID type (1-5) */

	format = (*type == MP_ID_TYPE3 || *type == MP_ID_TYPE4 ?
			  MP_LITTLE_ENDIAN :
			  MP_BIG_ENDIAN);

	/* The length byte is always 1-4 but never know,
	 * truncate to max. 4 bytes. */
	id_length = handle->device->chip_id_bytes_count > 4 ?
			    4 :
			    handle->device->chip_id_bytes_count;
	*device_id = (id_length ? load_int(&(msg[2]), id_length, format) :
				  0); /* Check for positive length. */
	return EXIT_SUCCESS;
}

int t76_spi_autodetect(minipro_handle_t *handle, uint8_t type,
		       uint32_t *device_id)
{
	if (handle->device != NULL && handle->device->flags.custom_protocol) {
		return bb_spi_autodetect(handle, type, device_id);
	}

	/* Create a device structure to search for required
	 * spi autodetection protocol
	 */
	device_t device;
	memset(&device, 0x00, sizeof(device_t));

	/* We need the protocol_id and high byte of the variant field to be set */
	device.protocol_id = SPI_PROTOCOL;
	device.variant = type ? SPI_DEVICE_16P : SPI_DEVICE_8P;
	device.variant <<= 8;
	handle->device = &device;

	/* Now search and send the required fpga bitstream
	 * used for autodetection ('SPI25F11' or 'SPI25F21')
	 */
	if (t76_send_bitstream(handle)) {
		free(handle->device->algorithm.bitstream);
		fprintf(stderr, "An error occurred while sending bitstream.\n");
		return EXIT_FAILURE;
	}

	uint8_t msg[64] = { 0 };
	msg[0] = T76_AUTODETECT;
	msg[8] = type;
	if (msg_send(handle->usb_handle, msg, 10))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, 16))
		return EXIT_FAILURE;
	*device_id = load_int(&(msg[2]), 3, MP_BIG_ENDIAN);
	return EXIT_SUCCESS;
}

int t76_protect_off(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_protect_off(handle);
	}
	uint8_t msg[8] = { 0 };
	msg[0] = T76_PROTECT_OFF;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

int t76_protect_on(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_protect_on(handle);
	}
	uint8_t msg[8] = { 0 };
	msg[0] = T76_PROTECT_ON;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

/* NAND erase. The T76 ERASE opcode (0x0E) erases ONE block, selected by the
 * 16-bit block index in msg[2..3]; a bare 0x0E (index 0) only ever erases
 * block 0. The vendor (t76_nand_erase_with_bad_block_skip @ 0x4c4e30) loops
 * over every block, first probing the factory bad-block marker with the 0x3A
 * check (t76_cmd_3a_nand_check_bad_block @ 0x4c62a6) and SKIPPING flagged
 * blocks so their markers are preserved, then issuing 0x0E for the rest.
 *   0x3A: msg[0]=3A, msg[2..3]=block, send 8 / recv 8; resp[1]!=0 => bad.
 *   0x0E: msg[0]=0E, msg[2..3]=block, send 16 / recv 8; resp[1]!=0 => bad.
 * Block count = code_memory_size / (write_buffer_size * pages_per_block). */
static int t76_nand_erase(minipro_handle_t *handle)
{
	device_t *device = handle->device;
	uint8_t msg[64];
	uint8_t resp[8];

	uint32_t block_size =
		(uint32_t)device->write_buffer_size * device->pages_per_block;
	if (!block_size) {
		fprintf(stderr, "NAND geometry missing (block size 0).\n");
		return EXIT_FAILURE;
	}
	uint32_t block_count = (uint32_t)(device->code_memory_size / block_size);
	uint32_t bad = 0;

	for (uint32_t blk = 0; blk < block_count; blk++) {
		/* 0x3A bad-block check: skip factory-marked bad blocks so the
		 * marker survives the erase. */
		memset(msg, 0, sizeof(msg));
		msg[0] = T76_NAND_BAD_BLOCK_CHECK;
		format_int(&msg[2], blk, 2, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, msg, 8) ||
		    msg_recv(handle->usb_handle, resp, sizeof(resp)))
			return EXIT_FAILURE;
		if (resp[1] != 0) {
			bad++;
			continue;
		}

		/* 0x0E erase this block. */
		memset(msg, 0, sizeof(msg));
		msg[0] = T76_ERASE;
		format_int(&msg[2], blk, 2, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, msg, 16) ||
		    msg_recv(handle->usb_handle, resp, sizeof(resp)))
			return EXIT_FAILURE;
		if (resp[1] != 0)
			bad++;
	}

	if (bad)
		fprintf(stderr, "(%u bad block%s skipped) ", bad,
			bad == 1 ? "" : "s");
	return EXIT_SUCCESS;
}

int t76_erase(minipro_handle_t *handle, uint8_t num_fuses, uint8_t pld)
{
	if (handle->device->flags.custom_protocol) {
		return bb_erase(handle);
	}

	if (handle->device->protocol_id == IC2_ALG_NAND)
		return t76_nand_erase(handle);

	uint8_t msg[64] = { 0 };
	msg[0] = T76_ERASE;
	msg[2] = num_fuses;
	msg[4] = pld;
	if (msg_send(handle->usb_handle, msg, 16)) {
		return EXIT_FAILURE;
	}

	memset(msg, 0x00, sizeof(msg));
	return msg_recv(handle->usb_handle, msg, sizeof(msg));
}

int t76_get_ovc_status(minipro_handle_t *handle, minipro_status_t *status,
		       uint8_t *ovc)
{
	uint8_t msg[32] = { 0 };
	msg[0] = T76_REQUEST_STATUS;
	/* For NAND the vendor reuses the BEGIN buffer for the 0x39 status poll,
	 * leaving the chip-parameter header in msg[1..7]
	 * (e.g. 39 2d 00 00 06 00 a0 70). A zeroed 0x39 appears to leave the
	 * NAND deselected so the following READID/read returns 0xFF. Mirror the
	 * vendor by repacking the same header for NAND. */
	if (handle->device->protocol_id == IC2_ALG_NAND) {
		device_t *device = handle->device;
		msg[1] = device->protocol_id;
		msg[2] = (uint8_t)device->variant;
		msg[3] = handle->cmdopts->icsp;
		format_int(&msg[4], device->voltages.raw_voltages, 2,
			   MP_LITTLE_ENDIAN);
		msg[6] = (uint8_t)device->chip_info;
		msg[7] = (uint8_t)device->pin_map;
	}
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;
	if (status && !handle->device->flags.custom_protocol) {
		/* This is verify while writing feature. */
		status->error = msg[0];
		status->address = load_int(&msg[8], 4, MP_LITTLE_ENDIAN);
		status->c1 = load_int(&msg[2], 2, MP_LITTLE_ENDIAN);
		status->c2 = load_int(&msg[4], 2, MP_LITTLE_ENDIAN);
	}
	*ovc = msg[12]; /* return the ovc status */
	return EXIT_SUCCESS;
}

int t76_write_jedec_row(minipro_handle_t *handle, jedec_set_t *js)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_jedec_row(handle, js->data, js->row, js->flags,
					  js->size);
	}
	uint8_t msg[64] = { 0 };
	msg[0] = T76_WRITE_JEDEC;
	msg[1] = js->type;
	msg[2] = js->size;
	msg[4] = js->row;
	msg[5] = js->flags;
	memcpy(&msg[8], js->data, (js->size + 7) / 8);
	return msg_send(handle->usb_handle, msg, 64);
}

int t76_read_jedec_row(minipro_handle_t *handle, jedec_set_t *js)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_jedec_row(handle, js->data, js->row, js->flags,
					 js->size);
	}
	uint8_t msg[32] = { 0 };
	msg[0] = T76_READ_JEDEC;
	msg[1] = js->type;
	msg[2] = js->size;
	msg[4] = js->row;
	msg[5] = js->flags;
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, 32))
		return EXIT_FAILURE;
	memcpy(js->data, msg, (js->size + 7) / 8);
	return EXIT_SUCCESS;
}

int t76_pin_test(minipro_handle_t *handle, pin_map_t *map)
{
	uint8_t msg[32] = { 0 };

	/* Do the pin detection in firmware */
	msg[0] = T76_PIN_DETECTION;
	if (msg_send(handle->usb_handle, msg, 8) ||
	    msg_recv(handle->usb_handle, msg, sizeof(msg)) ||
	    t76_end_transaction(handle))
		return EXIT_FAILURE;

	/* Now check for bad pin contact */
	int ret = EXIT_SUCCESS;
	int pin_count = handle->device->package_details.pin_count;
	uint8_t value = 0;
	for (int i = 0; i < map->mask_count; i++) {
		int pin = map->mask[i] <= pin_count / 2 ?
				  map->mask[i] :
				  (pin_count - (40 - map->mask[i]));
		if (!value) {
			fprintf(stderr, "Bad contact on pin:%u\n", pin);
			ret = EXIT_FAILURE;
		}
	}

	if (!ret)
		fprintf(stderr, "Pin test passed.\n");
	return ret;
}

/* Pull: 0=Pull-up, 1=Pull-down */
static uint8_t *do_ic_test(minipro_handle_t *handle, int pull)
{
	uint8_t *vector = handle->device->vectors;
	uint8_t msg[32];
	uint8_t *result;
	uint8_t pin_count = handle->device->package_details.pin_count;

	result = calloc(pin_count, handle->device->vector_count);
	if (!result){
		fprintf(stderr, "Out of memory!\n");
		return NULL;
	}

	device_t *device = handle->device;

	/* Get the TestLgcPull/TestLgcDown algo depending on the pull value */
	device->protocol_id = IC2_ALG_NONE;
	handle->device->variant = (pull ? T76_UTIL_ALG_TEST_LGC_DOWN << 8 :
					  T76_UTIL_ALG_TEST_LGC_PULL << 8);
	if (get_algorithm(device, MP_T76, handle->cmdopts->algo_path,
			  handle->cmdopts->icsp, handle->cmdopts->vopt)){
		free(result);
		return NULL;
	}

	/* Write the FPGA before testing */
	if (t76_write_bitstream(handle)) {
		fprintf(stderr, "An error occurred while sending bitstream.\n");
		free(result);
		return NULL;
	}

	uint8_t *out = result;
	int n;
	for (n = 0; n < handle->device->vector_count; n++) {
		memset(msg, 0xff, 32);

		msg[0] = T76_LOGIC_IC_TEST_VECTOR;
		msg[1] = handle->device->voltages.vcc;
		msg[1] |= pull << 7; /* Set the pull-up/pull-down */
		format_int(&msg[2], pin_count, 2, MP_LITTLE_ENDIAN);
		format_int(&msg[4], n, 4, MP_LITTLE_ENDIAN);

		int i;
		/* Pack the vector to 2 pin/byte */
		for (i = 0; i < handle->device->package_details.pin_count;
		     i++) {
			if (i & 1)
				msg[8 + i / 2] |= *vector << 4;
			else
				msg[8 + i / 2] = *vector;
			vector++;
		}

		/* Send the test vector and read the pin status */
		if (msg_send(handle->usb_handle, msg, sizeof(msg))) {
			free(result);
			return NULL;
		}
		if (msg_recv(handle->usb_handle, msg, sizeof(msg))) {
			free(result);
			return NULL;
		}

		if (msg[1]) {
			fprintf(stderr, "Overcurrent protection!\007\n");
			free(result);
			return NULL;
		}

		/* Unpack the result from 2 pin/byte to 1 pin/byte */
		for (i = 0; i < handle->device->package_details.pin_count; i++)
			*out++ = (msg[8 + i / 2] >> (4 * (i & 1))) & 0xf;
	}

	return result;
}

/* Performing a logic test. This is accomplished in two steps.
 * The first step will set a pull-up resistor on all chip outputs (L, H, Z).
 * The second step will set a pull-down resistor on all chip outputs.
 * According to the vector table then each output is compared against the two
 * result array. Considering the weak pull-up/pull-down resistors we can detect
 * L(low) state as 0 in both steps, H(high) state as 1 in both steps and
 * Z(high impedance) state as 1 in step 1 when the pull-up is activated and
 * 0 in step 2 when the pull-down is activated.
 * While for chips with open collector/open drain output we need to perform
 * these two steps to detect the Z state, for chips with totem-pole outputs
 * this is not really necessary but, sometimes internal issues can be
 * detected this way like burned H side or L side output transistors.
 * The C (clock) state is performed in firmware by first pulsing the pin
 * marked as C and then all pins are read back.
 * The X (don't care) state will leave the pin unconnected.
 * The V (VCC) and G (Ground) state will designate the power supply pins.
 */

int t76_logic_ic_test(minipro_handle_t *handle)
{
	uint8_t req[18] = { T76_TEST_RAM, 1, 4, 0,  0, 0, 0, 0,  1, 1, 1, 1, };
	uint8_t *vector = handle->device->vectors;
	uint8_t *first_step = NULL;
	uint8_t *second_step = NULL;
	int ret = EXIT_FAILURE;

	if (handle->device->chip_type == MP_SRAM)
		return minipro_test_ram_generic(handle, req, sizeof (req));

	fprintf(stderr, "Using T76 LOGIC algorithm..\n");

	if (!(first_step = do_ic_test(handle, 0))) { /* Pull-up active */
		fprintf(stderr,
			"Error running the first step of logic test.\n");
	} else if (!(second_step =
			     do_ic_test(handle, 1))) { /* Pull-down active */
		fprintf(stderr,
			"Error running the second step of logic test.\n");
	} else if (handle->cmdopts->logicic_out) {
		ret = write_logic_file(handle, first_step, second_step);
	} else {
		int errors = 0, err;
		static const char pst[] = "01LHCZXGV";
		size_t n = 0;

		printf("      ");
		for (int pin = 1;
		     pin <= handle->device->package_details.pin_count; pin++)
			printf("%-3d", pin);
		putchar('\n');

		for (int i = 0; i < handle->device->vector_count; i++) {
			printf("%04d: ", i);
			for (int pin = 0;
			     pin < handle->device->package_details.pin_count;
			     pin++) {
				err = 0;
				switch (vector[n]) {
				case LOGIC_L: /* Pin must be 0 in both steps */
					if (first_step[n] || second_step[n])
						err = 1;
					break;
				case LOGIC_H: /* Pin must be 1 in both steps */
					if (!first_step[n] || !second_step[n])
						err = 1;
					break;
				case LOGIC_Z: /* Pin must be 1 in step 1 and 0 in step 2 */
					if (!first_step[n] || second_step[n])
						err = 1;
					break;
				}
				printf("%s%c%c ", err ? "\e[0;91m" : "\e[0m",
				       pst[vector[n]], err ? '-' : ' ');
				errors += err;
				n++;
			}
			printf("\e[0m\n");
		}

		if (errors) {
			fprintf(stderr,
				"Logic test failed: %d errors encountered.\n",
				errors);
		} else {
			fprintf(stderr, "Logic test successful.\n");
			ret = EXIT_SUCCESS;
		}
	}

	free(second_step);
	free(first_step);
	if (t76_end_transaction(handle))
		return EXIT_FAILURE;

	return ret;
}

/*****************************************************************************
 * Firmware updater section
 *****************************************************************************

This is the updateT76.dat file structure.
It has a fixed 16 byte header, followed by a number of 284-byte blocks
|============|===========|============|==============|
|File version| File CRC  | Unknown    | Blocks count |
|============|===========|============|==============|
|  4 bytes   | 4 bytes   | 4 bytes    | 4 bytes      |
|============|===========|============|==============|
|  offset 0  | offset 4  | offset 8   | offset 12    |
|============|===========|============|==============|

*/

#define LAST_BLOCK_ADDR 0x049f00
#define LAST_BLOCK_CRC	0xcdef8668

/* Performing a firmware update */
int t76_firmware_update(minipro_handle_t *handle, const char *firmware)
{
	static uint8_t msg[284];

	struct stat st;
	if (stat(firmware, &st)) {
		fprintf(stderr, "%s open error!: ", firmware);
		perror("");
		return EXIT_FAILURE;
	}

	off_t file_size = st.st_size;
	/* Check the updateT76.dat size */
	if (file_size < 16 || file_size > 1048576) {
		fprintf(stderr, "%s file size error!\n", firmware);
		return EXIT_FAILURE;
	}

	/* Open the updateT76.dat firmware file */
	FILE *file = fopen(firmware, "rb");
	if (!file) {
		fprintf(stderr, "%s open error!: ", firmware);
		perror("");
		return EXIT_FAILURE;
	}
	uint8_t *update_dat = malloc(file_size);
	if (!update_dat) {
		fprintf(stderr, "Out of memory!\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	/* Read the updateT76.dat file */
	if (fread(update_dat, sizeof(char), st.st_size, file) != st.st_size) {
		fprintf(stderr, "%s file read error!\n", firmware);
		fclose(file);
		free(update_dat);
		return EXIT_FAILURE;
	}
	fclose(file);

	/* check for update file version */
	uint32_t version = load_int(update_dat, 4, MP_LITTLE_ENDIAN);
	if ((version & T76_UPDATE_FILE_VERS_MASK) != T76_UPDATE_FILE_VERSION) {
		fprintf(stderr, "%s file version error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}
	version &= T76_FIRMWARE_VERS_MASK;

	/* Read the blocks count and check if correct */
	uint32_t blocks = load_int(update_dat + 12, 4, MP_LITTLE_ENDIAN);
	if (blocks * 0x114 + 16 != file_size) {
		fprintf(stderr, "%s file size error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}

	/* Compute the file CRC and compare */
	uint32_t crc = crc_32(update_dat + 16, blocks * 0x114, 0xFFFFFFFF);
	if (crc != load_int(update_dat + 4, 4, MP_LITTLE_ENDIAN)) {
		fprintf(stderr, "%s file CRC error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "%s contains firmware version %02u.%u.%02u", firmware,
		0, (version >> 8) & 0xFF, (version & 0xFF));

	if (handle->firmware > version)
		fprintf(stderr, " (older)");
	else if (handle->firmware < version)
		fprintf(stderr, " (newer)");

	fprintf(stderr,
		"\n\nDo you want to continue with firmware update? y/n:");
	fflush(stderr);
	char c = getchar();
	if (c != 'Y' && c != 'y') {
		free(update_dat);
		fprintf(stderr, "Firmware update aborted.\n");
		return EXIT_FAILURE;
	}

	/* Switching to boot mode if necessary */
	if (handle->status == MP_STATUS_NORMAL) {
		fprintf(stderr, "Switching to bootloader... ");
		fflush(stderr);

		memset(msg, 0, sizeof(msg));
		msg[0] = T76_SWITCH;
		msg[1] = 0xaa;
		format_int(&(msg[4]), T76_BTLDR_MAGIC, 4, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, msg, 8)) {
			free(update_dat);
			return EXIT_FAILURE;
		}

		memset(msg, 0, sizeof(msg));
		if (msg_recv(handle->usb_handle, msg, 8)) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (msg[1]) {
			fprintf(stderr, "failed (code: %d)!\n", msg[0]);
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (minipro_reset(handle)) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		/* Wait for T76 to boot, otherwise minipro_open will fail */
		usleep(1000000);
		handle = minipro_open(VERBOSE);
		if (!handle) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (handle->status == MP_STATUS_NORMAL) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		fprintf(stderr, "OK\n");
	}

	/* Erase device */
	fprintf(stderr, "Erasing... ");
	fflush(stderr);

	memset(msg, 0, sizeof(msg));
	msg[0] = T76_BOOTLOADER_ERASE;
	msg[1] = 0xaa;
	if (msg_send(handle->usb_handle, msg, 8)) {
		fprintf(stderr, "\nErase failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	memset(msg, 0, sizeof(msg));
	if (msg_recv(handle->usb_handle, msg, 8)) {
		fprintf(stderr, "\nErase failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	if (msg[1]) {
		fprintf(stderr, "\nfailed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "OK\n");

	/* Reflash firmware */
	fprintf(stderr, "Reflashing... ");
	fflush(stderr);

	/* Begin write */
	msg[0] = T76_BOOTLOADER_WRITE;
	msg[1] = 0xaa;
	if (msg_send(handle->usb_handle, msg, 8)) {
		fprintf(stderr, "\nReflash failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	uint32_t address = 0;
	for (uint32_t i = 0; i < blocks; i++) {
		memset(msg, 0, sizeof(msg));
		msg[0] = T76_BOOTLOADER_WRITE;
		msg[1] = 0x00;
		msg[2] = 0x00; /* Data Length LSB */
		msg[3] = 0x01; /* Data length MSB */
		format_int(&(msg[4]), address, 4, MP_LITTLE_ENDIAN);
		memcpy(&msg[8], update_dat + 16 + i * 0x114, 0x114); /* Block */
		address += 256;

		/* Write firmware block */
		if (msg_send(handle->usb_handle, msg, 0x11c)) {
			fprintf(stderr, "\nReflash failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		/* Check if the firmware block was successfully written */
		memset(msg, 0, sizeof(msg));
		if (msg_recv(handle->usb_handle, msg, 8)) {
			fprintf(stderr, "\nReflash... Failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (msg[1]) {
			fprintf(stderr, "\nReflash... Failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		fprintf(stderr, "\r\e[KReflashing... %2d%%", i * 100 / blocks);
		fflush(stderr);
	}

	/* Write last block */
	memset(msg, 0, sizeof(msg));
	msg[0] = T76_BOOTLOADER_WRITE;
	msg[1] = 0x03;
	msg[2] = 0x00; /* Data Length LSB */
	msg[3] = 0x01; /* Data length MSB */
	format_int(&(msg[4]), LAST_BLOCK_ADDR, 4, MP_LITTLE_ENDIAN);
	format_int(&(msg[8]), LAST_BLOCK_CRC, 4, MP_LITTLE_ENDIAN);

	/* Write last firmware block */
	if (msg_send(handle->usb_handle, msg, 0x108)) {
		fprintf(stderr, "\nReflash failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	/* Check if the last firmware block was successfully written */
	memset(msg, 0, sizeof(msg));
	if (msg_recv(handle->usb_handle, msg, 8)) {
		fprintf(stderr, "\nReflash... Failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	if (msg[1]) {
		fprintf(stderr, "\nReflash... Failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "\r\e[KReflashing... 100%%\n");

	/* Switching back to normal mode */
	fprintf(stderr, "Resetting device... ");
	fflush(stderr);
	if (minipro_reset(handle)) {
		fprintf(stderr, "failed!\n");
		return EXIT_FAILURE;
	}

	/* Wait for T76 to boot, otherwise minipro_open will fail */
	usleep(1000000);
	handle = minipro_open(VERBOSE);
	if (!handle) {
		fprintf(stderr, "failed!\n");
		return EXIT_FAILURE;
	}
	fprintf(stderr, "OK\n");
	if (handle->status != MP_STATUS_NORMAL) {
		fprintf(stderr, "Reflash... failed\n");
		return EXIT_FAILURE;
	}

	fprintf(stderr, "Reflash... OK\n");
	return EXIT_SUCCESS;
}
