/*
 * t76.h - Low level ops for T76 declarations and definitions
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

#ifndef __T76_H
#define __T76_H

#define T76_FIRMWARE_VERSION 0x111
#define T76_FIRMWARE_STRING  "00.1.17"

/* T76 low level functions. */
int t76_begin_transaction(minipro_handle_t *handle);
int t76_end_transaction(minipro_handle_t *handle);
int t76_reset_fpga(minipro_handle_t *handle);
int t76_get_chip_id(minipro_handle_t *handle, uint8_t *type,
		    uint32_t *device_id);
int t76_get_ovc_status(minipro_handle_t *handle, minipro_status_t *status,
		       uint8_t *ovc);
int t76_read_block(minipro_handle_t *handle, data_set_t *ds);
int t76_write_block(minipro_handle_t *handle, data_set_t *ds);
int t76_spi_autodetect(minipro_handle_t *handle, uint8_t type,
		       uint32_t *device_id);
int t76_read_fuses(minipro_handle_t *handle, uint8_t type, size_t size,
		   uint8_t items_count, uint8_t *buffer);
int t76_write_fuses(minipro_handle_t *handle, uint8_t type, size_t size,
		    uint8_t items_count, uint8_t *buffer);
int t76_read_calibration(minipro_handle_t *handle, uint8_t *buffer, size_t len);
int t76_erase(minipro_handle_t *handle, uint8_t num_fuses, uint8_t pld);
int t76_write_jedec_row(minipro_handle_t *handle, jedec_set_t *js);
int t76_read_jedec_row(minipro_handle_t *handle, jedec_set_t *js);
int t76_protect_off(minipro_handle_t *handle);
int t76_protect_on(minipro_handle_t *handle);
int t76_logic_ic_test(minipro_handle_t *handle);
int t76_firmware_update(minipro_handle_t *handle, const char *firmware);
int t76_pin_test(minipro_handle_t *handle, pin_map_t *map);
#endif
