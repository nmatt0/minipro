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
#define T76_WRITE_BITSTREAM	  0x26
#define T76_LOGIC_IC_TEST_VECTOR  0x28
#define T76_AUTODETECT		  0x37
#define T76_UNLOCK_TSOP48	  0x38
#define T76_REQUEST_STATUS	  0x39
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

	/* Send the end of bitstream command */
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T76_WRITE_BITSTREAM;
	msg[1] = T76_END_BS;
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
static int t76_send_bitstream(minipro_handle_t *handle)
{
	/* Don't upload the bitstream again if we are in the same session */
	if (handle->bitstream_uploaded)
		return EXIT_SUCCESS;

	/* Get the required FPGA bitstream algorithm
	* Logic chips are not handled here
	*/
	device_t *device = handle->device;

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

int t76_erase(minipro_handle_t *handle, uint8_t num_fuses, uint8_t pld)
{
	if (handle->device->flags.custom_protocol) {
		return bb_erase(handle);
	}

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
