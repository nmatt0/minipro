/*
 * usb.c - Low level USB functions *nix libusb implementation.
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

#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usb.h"

#define MP_TL866_VID	    0x04d8
#define MP_TL866_PID	    0xe11c
#define MP_TL866II_VID	    0xa466
#define MP_TL866II_PID	    0x0a53
#define MP_T76_VID	        0xa466
#define MP_T76_PID	        0x1a86
#define MP_TL866IIPLUS	    5
#define MP_T56			    6
#define MP_T48			    7
#define MP_T76			    8
#define MP_USBTIMEOUT	    5000
#define MP_USB_READ_TIMEOUT 360000

typedef struct usb_id {
	uint16_t vid;
	uint16_t pid;
} usb_id_t;

typedef struct usb_handle {
	void *handle;
	usb_id_t usb_id;
} usb_handle_t;


/* Helper function to debug USB communication
 * Use make CFLAGS+=-DDEBUG_USB to activate this
 */
#ifdef DEBUG_USB
void print_hex(const uint8_t *buffer, int size, uint8_t endpoint)
{
	fprintf(stderr,
		"\x1b[33m%s %d bytes on endpoint 0x%02X \x1b[0m \n",
		(endpoint & 0x80) ? "Read" : "Write", size, endpoint);
	for (int i = 0; i < size; i += 16) {
		for (int j = 0; j < 16; j++) {
			if (i + j < size) {
				fprintf(stderr, "\x1b[36m%02X \x1b[0m",
					buffer[i + j]);
			} else {
				fprintf(stderr, "   ");
			}
		}
		fprintf(stderr, "  ");
		for (int j = 0; j < 16 && i + j < size; j++) {
			uint8_t c = buffer[i + j];
			if (c >= 32 && c <= 126) {
				fprintf(stderr, "\x1b[32m%c\x1b[0m", c);
			} else {
				fprintf(stderr, "\x1b[90m.\x1b[0m");
			}
		}
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");
}
#endif

/* Open usb device */
void *usb_open(uint8_t verbose)
{
	usb_id_t usb_ids[] = { { .vid = MP_TL866_VID, .pid = MP_TL866_PID },
			       { .vid = MP_TL866II_VID, .pid = MP_TL866II_PID },
			       { .vid = MP_T76_VID, .pid = MP_T76_PID } };

	int ret = libusb_init(NULL);
	if (ret < 0) {
		if (verbose)
			fprintf(stderr, "Error initializing libusb: %s\n",
				libusb_error_name(ret));
		return NULL;
	}

	/* Loop trough each known VID/PID */
	void *usb_handle = NULL;
	uint16_t vid;
	uint16_t pid;
	for (int i = 0;
	     i < sizeof(usb_ids) / sizeof(usb_ids[0]) && usb_handle == NULL;
	     i++) {
		vid = usb_ids[i].vid;
		pid = usb_ids[i].pid;
		usb_handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
	}

	/* If we don't get any connected device report error in connecting */
	if (usb_handle == NULL) {
		libusb_exit(NULL);
		if (verbose)
			fprintf(stderr, "No programmer found.\n");
		return NULL;
	}

	ret = libusb_claim_interface(usb_handle, 0);
	if (ret != 0) {
		if (verbose)
			fprintf(stderr, "\nIO error: claim_interface: %s\n",
				libusb_error_name(ret));
		libusb_close(usb_handle);
		libusb_exit(NULL);
		return NULL;
	}

	usb_handle_t *handle = calloc(1, sizeof(usb_handle_t));
	if (!handle) {
		if (verbose)
			fprintf(stderr, "Out of memory!\n");
		return NULL;
	}
	handle->handle = usb_handle;
	handle->usb_id.vid = vid;
	handle->usb_id.pid = pid;
	return handle;
}

/* Close usb device */
int usb_close(void *usb_handle)
{

	int ret = EXIT_SUCCESS;
	void *handle = ((usb_handle_t *)usb_handle)->handle;
	ret = libusb_release_interface(handle, 0);
	if (ret != 0 && ret != LIBUSB_ERROR_NO_DEVICE) {
		fprintf(stderr, "\nIO error: release_interface: %s\n",
			libusb_error_name(ret));
		ret = EXIT_FAILURE;
	}
	libusb_close(handle);
	libusb_exit(NULL);
	free(usb_handle);
	return ret;
}

/* Get no. of devices connected */
int minipro_get_devices_count(uint8_t version)
{
	libusb_device **devs;
	int i, devices = 0;

	uint16_t PID, VID;

	switch (version) {
	case MP_TL866IIPLUS:
	case MP_T48:
	case MP_T56:
		PID = MP_TL866II_PID;
		VID = MP_TL866II_VID;
		break;

	case MP_T76:
		PID = MP_T76_PID;
		VID = MP_T76_VID;
		break;

	default:
		PID = MP_TL866_PID;
		VID = MP_TL866_VID;
		break;
	}

	if (libusb_init(NULL) < 0)
		return 0;

	int count = libusb_get_device_list(NULL, &devs);
	if (count < 0) {
		libusb_exit(NULL);
		return 0;
	}

	for (i = 0; i < count; i++) {
		struct libusb_device_descriptor desc;
		int ret = libusb_get_device_descriptor(devs[i], &desc);
		if (ret < 0) {
			libusb_free_device_list(devs, 1);
			libusb_exit(NULL);
			return 0;
		}
		if (desc.idProduct == PID && desc.idVendor == VID) {
			devices++;
		}
	}
	libusb_free_device_list(devs, 1);
	libusb_exit(NULL);
	return devices;
}

static void payload_transfer_cb(struct libusb_transfer *transfer)
{
	int *completed = transfer->user_data;
	if (completed != NULL) {
		*completed = 1;
	}
}

static int msg_transfer(void *usb_handle, uint8_t *buffer, size_t size,
			uint8_t direction, uint8_t endpoint,
			int *bytes_transferred, uint32_t timeout)
{
	void *handle = ((usb_handle_t *)usb_handle)->handle;
	int ret = libusb_bulk_transfer(handle, (endpoint | direction), buffer,
				       size, bytes_transferred, timeout);

#ifdef DEBUG_USB
	print_hex(buffer, *bytes_transferred, (endpoint | direction));
#endif

	if (ret != LIBUSB_SUCCESS)
		fprintf(stderr, "\nIO error: bulk_transfer: %s\n",
			libusb_error_name(ret));

	return ret;
}

static int payload_transfer(void *usb_handle, uint8_t direction,
			    uint8_t *ep2_buffer, size_t ep2_length,
			    uint8_t *ep3_buffer, size_t ep3_length)
{
	void *handle = ((usb_handle_t *)usb_handle)->handle;
	struct libusb_transfer *ep2_urb;
	struct libusb_transfer *ep3_urb;
	int ret;
	int ep2_completed = 0;
	int ep3_completed = 0;

	ep2_urb = libusb_alloc_transfer(0);
	ep3_urb = libusb_alloc_transfer(0);
	if (!ep2_urb || !ep3_urb) {
		fprintf(stderr, "Out of memory!\n");
		return EXIT_FAILURE;
	}

	libusb_fill_bulk_transfer(ep2_urb, handle, (0x02 | direction),
				  ep2_buffer, ep2_length, payload_transfer_cb,
				  &ep2_completed, MP_USBTIMEOUT);
	libusb_fill_bulk_transfer(ep3_urb, handle, (0x03 | direction),
				  ep3_buffer, ep3_length, payload_transfer_cb,
				  &ep3_completed, MP_USBTIMEOUT);

	ret = libusb_submit_transfer(ep2_urb);
	if (ret < 0) {
		fprintf(stderr, "\nIO error: submit_transfer: %s\n",
			libusb_error_name(ret));
		return EXIT_FAILURE;
	}
	ret = libusb_submit_transfer(ep3_urb);
	if (ret < 0) {
		fprintf(stderr, "\nIO error: submit_transfer: %s\n",
			libusb_error_name(ret));
		return EXIT_FAILURE;
	}

	while (!ep2_completed) {
		ret = libusb_handle_events_completed(NULL, &ep2_completed);
		if (ret < 0) {
			if (ret == LIBUSB_ERROR_INTERRUPTED)
				continue;
			libusb_cancel_transfer(ep2_urb);
			libusb_cancel_transfer(ep3_urb);
			continue;
		}
	}
	while (!ep3_completed) {
		ret = libusb_handle_events_completed(NULL, &ep3_completed);
		if (ret < 0) {
			if (ret == LIBUSB_ERROR_INTERRUPTED)
				continue;
			libusb_cancel_transfer(ep2_urb);
			libusb_cancel_transfer(ep3_urb);
			continue;
		}
	}

	if (ep2_urb->status != 0 || ep3_urb->status != 0) {
		fprintf(stderr, "\nIO Error: Async transfer failed: %s\n",
			libusb_error_name(ep2_urb->status ? ep2_urb->status :
							    ep3_urb->status));
		libusb_free_transfer(ep2_urb);
		libusb_free_transfer(ep3_urb);
		return EXIT_FAILURE;
	}

#ifdef DEBUG_USB
	print_hex(ep2_buffer, ep2_urb->actual_length,  (0x02 | direction));
	print_hex(ep3_buffer, ep3_urb->actual_length,  (0x03 | direction));
#endif

	libusb_free_transfer(ep2_urb);
	libusb_free_transfer(ep3_urb);
	return EXIT_SUCCESS;
}

int write_payload2(void *usb_handle, uint8_t *buffer, size_t length, size_t limit)
{
	uint32_t ep2_length;
	uint32_t ep3_length;
	int status;
	int bytes_transferred;

	/* Handle T76 write payload to endpoint 0x05 */
	if (((usb_handle_t *)usb_handle)->usb_id.pid == MP_T76_PID) {
		return msg_transfer(usb_handle, buffer, length,
				    LIBUSB_ENDPOINT_OUT, 0x05,
				    &bytes_transferred, MP_USBTIMEOUT);
	}

	/* If the payload length is exactly 64 bytes send it over the
	 * endpoint2 only */
	if (!limit || length <= limit) {
		status = msg_transfer(usb_handle, buffer, length, LIBUSB_ENDPOINT_OUT,
				    0x02, &bytes_transferred, MP_USBTIMEOUT);
		if (bytes_transferred == length)
			return status;

		fprintf(stderr, "%s: short write %d/%zu\n", __func__, bytes_transferred, length);
		return EXIT_FAILURE;
	}

	/* This  is from XgPro */
	uint32_t j = length % 128;
	if (length % 128) {
		uint32_t k = (length - j) / 2;
		if (j > 64) {
			ep2_length = k + 64;
			ep3_length = j + k - 64;
		} else {
			ep2_length = k;
			ep3_length = j + k;
		}
	} else {
		ep3_length = length / 2;
		ep2_length = ep3_length;
	}

	return payload_transfer(usb_handle, LIBUSB_ENDPOINT_OUT, buffer, ep2_length,
				buffer + ep2_length, ep3_length);
}

int read_payload2(void *usb_handle, uint8_t *buffer, size_t length,
		  size_t limit)
{
	int i, bytes_transferred;

	/* Handle T76 read payload from endpoint 0x82 */
	if (((usb_handle_t *)usb_handle)->usb_id.pid == MP_T76_PID) {
		return msg_transfer(usb_handle, buffer, length,
				    LIBUSB_ENDPOINT_IN, 0x02,
				    &bytes_transferred, MP_USBTIMEOUT);
	}

	/* If the payload length is less than 64 bytes increase the
	 * buffer to 64 bytes and read it over the endpoint2 only.
	 * Submitting a buffer less than 64 bytes will cause an libusb
	 * overflow.
	 */
	if (length < 64) {
		uint8_t data[64];
		if (msg_transfer(usb_handle, data, sizeof(data),
				 LIBUSB_ENDPOINT_IN, 0x02, &bytes_transferred,
				 MP_USBTIMEOUT))
			return EXIT_FAILURE;
		memcpy(buffer, data, length);
		return EXIT_SUCCESS;
	}

	/* If the payload length < limit bytes read it over the endpoint2 only */
	if (length == 64 || !limit || length < limit)
		return msg_transfer(usb_handle, buffer, length,
				    LIBUSB_ENDPOINT_IN, 0x02,
				    &bytes_transferred, MP_USBTIMEOUT);

	/* More than limit bytes */
	uint8_t *data = malloc(length);
	if (!data) {
		fprintf(stderr, "\nOut of memory\n");
		return EXIT_FAILURE;
	}

	/* Async read of endpoints 2 and 3 */
	if (payload_transfer(usb_handle, LIBUSB_ENDPOINT_IN, data, length / 2,
			     data + length / 2, length / 2)) {
		free(data);
		return EXIT_FAILURE;
	}

	/* Deinterlacing the buffers */
	size_t blocks = length / 64;
	for (i = 0; i < blocks; ++i) {
		uint8_t *ep_buf;
		if (i % 2 == 0) {
			ep_buf = data;
		} else {
			ep_buf = data + length / 2;
		}
		memcpy(buffer + (i * 64), ep_buf + ((i / 2) * 64), 64);
	}
	free(data);
	return EXIT_SUCCESS;
}

int msg_send(void *usb_handle, uint8_t *buffer, size_t size)
{
	int bytes_transferred, ret;
	ret = msg_transfer(usb_handle, buffer, size, LIBUSB_ENDPOINT_OUT, 0x01,
			   &bytes_transferred, MP_USBTIMEOUT);
	if (bytes_transferred != (int)size) {
		fprintf(stderr,
			"IO error: expected %zu bytes but %u bytes transferred\n",
			size, bytes_transferred);
		return EXIT_FAILURE;
	}
	return ret;
}

int msg_recv(void *usb_handle, uint8_t *buffer, size_t size)
{
	int bytes_transferred;
	return msg_transfer(usb_handle, buffer, size, LIBUSB_ENDPOINT_IN, 0x01,
			    &bytes_transferred, MP_USB_READ_TIMEOUT);
}

/* Read a status packet from EP83 IN. Used by the T76 NAND program flow, which
 * emits a per-block status word on endpoint 0x83 once the block's last page has
 * finished programming. Short timeout: the status arrives within a page-program
 * time, so a missing one should fail fast rather than hang. */
int status_recv(void *usb_handle, uint8_t *buffer, size_t size)
{
	int bytes_transferred;
	return msg_transfer(usb_handle, buffer, size, LIBUSB_ENDPOINT_IN, 0x03,
			    &bytes_transferred, MP_USBTIMEOUT);
}
