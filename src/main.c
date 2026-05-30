/*
 * main.c - User interface and high-level operations.
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

#define _GNU_SOURCE

#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <getopt.h>
#include <unistd.h>

#include "database.h"
#include "jedec.h"
#include "ihex.h"
#include "srec.h"
#include "minipro.h"
#include "version.h"

#ifdef _WIN32
#include <shlwapi.h>
#include <fcntl.h>
#define STRCASESTR StrStrIA
#else
#define STRCASESTR strcasestr
#endif


#define READ_BUFFER_SIZE 65536
#define MIN(a, b)	 (((a) < (b)) ? (a) : (b))

static const char *user_id[] = {
	"user_id0", "user_id1", "user_id2", "user_id3",
	"user_id4", "user_id5", "user_id6", "user_id7"
};

/* Long options names */
static struct option long_options[] = {
	{ "pulse", required_argument, NULL, 2 },
	{ "vpp", required_argument, NULL, 2 },
	{ "vdd", required_argument, NULL, 2 },
	{ "vcc", required_argument, NULL, 2 },
	{ "spi_clock", required_argument, NULL, 2 },
	{ "address", required_argument, NULL, 2 },
	{ "fuses", no_argument, NULL, 1 },
	{ "uid", no_argument, NULL, 1 },
	{ "lock", no_argument, NULL, 1 },
	{ "infoic", required_argument, NULL, 3 },
	{ "logicic", required_argument, NULL, 4 },
	{ "logicic_out", required_argument, NULL, 5 },
	{ "algorithms", required_argument, NULL, 6 },
	{ "list", no_argument, NULL, 'l' },
	{ "search", required_argument, NULL, 'L' },
	{ "get_info", required_argument, NULL, 'd' },
	{ "device", required_argument, NULL, 'p' },
	{ "programmer", required_argument, NULL, 'q' },
	{ "presence_check", no_argument, NULL, 'k' },
	{ "query_supported", no_argument, NULL, 'Q' },
	{ "auto_detect", required_argument, NULL, 'a' },
	{ "write", required_argument, NULL, 'w' },
	{ "read", required_argument, NULL, 'r' },
	{ "verify", required_argument, NULL, 'm' },
	{ "blank_check", no_argument, NULL, 'b' },
	{ "erase", no_argument, NULL, 'E' },
	{ "read_id", no_argument, NULL, 'D' },
	{ "page", required_argument, NULL, 'c' },
	{ "skip_erase", no_argument, NULL, 'e' },
	{ "skip_verify", no_argument, NULL, 'v' },
	{ "skip_id", no_argument, NULL, 'x' },
	{ "no_size_error", no_argument, NULL, 's' },
	{ "no_size_warning", no_argument, NULL, 'S' },
	{ "no_id_error", no_argument, NULL, 'y' },
	{ "format", required_argument, NULL, 'f' },
	{ "version", no_argument, NULL, 'V' },
	{ "pin_check", no_argument, NULL, 'z' },
	{ "logic_test", no_argument, NULL, 'T' },
	{ "icsp_vcc", no_argument, NULL, 'i' },
	{ "icsp_no_vcc", no_argument, NULL, 'I' },
	{ "protect", no_argument, NULL, 'P' },
	{ "unprotect", no_argument, NULL, 'u' },
	{ "hardware_check", no_argument, NULL, 't' },
	{ "update", required_argument, NULL, 'F' },
	{ "help", no_argument, NULL, 'h' },
	{ NULL, 0, NULL, 0 }
};

#define SEP "----------------------------------------"
#define INFO "---------------Chip Info----------------"
static char signon[] = "minipro version %s     A free and open TL866 series programmer\n";


/* Get the connected programmer version */
int get_programmer_version(uint8_t *version)
{
	if (!(minipro_get_devices_count(MP_TL866A) +
	      minipro_get_devices_count(MP_TL866IIPLUS)+
		  minipro_get_devices_count(MP_T76))) {
		if (!(*version)) {
			fprintf(stderr,
				"No device found. Which database do you want to display?\n1) "
				"TL866A/CS\n2) TL866II+/T48/T56\n3) T76\n4) Abort\n");
			fflush(stderr);
			char c = getchar();
			switch (c) {
			case '1':
				*version = MP_TL866A;
				break;
			case '2':
				*version = MP_TL866IIPLUS;
				break;
			case '3':
				*version = MP_T76;
				break;
			default:
				fprintf(stderr, "Aborted.\n");
				return EXIT_FAILURE;
			}
		}
	} else if (!(*version)) {
		minipro_handle_t *tmp = minipro_open(VERBOSE);
		if (!tmp) {
			return EXIT_FAILURE;
		}
		minipro_print_system_info(tmp);
		fflush(stderr);
		*version = tmp->version;
		minipro_close(tmp);
	}
	return EXIT_SUCCESS;
}

/* Search the database xml and return a device_t structure */
int get_device(minipro_handle_t *handle)
{
	db_data_t db_data;
	memset(&db_data, 0, sizeof(db_data));
	db_data.device_name = handle->cmdopts->device_name;
	db_data.logicic_path = handle->cmdopts->logicic_path;
	db_data.infoic_path = handle->cmdopts->infoic_path;
	db_data.prog_version = handle->version;
	handle->device = get_device_by_name(&db_data);
	if (!handle->device) {
		fprintf(stderr, "\nDevice %s not found!\n",
			handle->cmdopts->device_name);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

/* Get the required parameters table */
const parameters_t *get_parameter_struct(device_t *device, uint8_t type)
{
	/* Switch by parameter type */
	switch (type) {
	case VPP_VOLTAGE:
		return device->vpp_table;
	case VCC_VOLTAGE:
		return device->vcc_table;
	case VPP_BB_VOLTAGE:
		return device->bb_vpp_table;
	case VCC_BB_VOLTAGE:
		return device->bb_vpp_table;
	case LOGIC_VOLTAGE:
		return device->vcc_logic_table;
	case SPI_CLOCK:
		return device->spi_clock_table;
	default:
		return NULL;
	}
}

/* Get a parameter string from an integer value */
const char *get_parameter(minipro_handle_t *handle, uint8_t value, uint8_t type)
{
	const parameters_t *table = get_parameter_struct(handle->device, type);
	while (table != NULL && table->name) {
		if (table->value == value) {
			return table->name;
		}
		table++;
	}
	return "-";
}

/* Get an integer value from a string parameter name */
int set_parameter(minipro_handle_t *handle, char *value, uint8_t *target,
		  uint8_t type)
{
	const parameters_t *table = get_parameter_struct(handle->device, type);
	while (table != NULL && table->name) {
		if (!strcasecmp(table->name, value)) {
			*target = table->value;
			return EXIT_SUCCESS;
		}
		table++;
	}
	return EXIT_FAILURE;
}

/* Helper function to dump a parameter table to console */
void print_parameters_table(const char *msg, const parameters_t *table)
{
	int wrap = strlen(msg);
	fprintf(stderr, "%s", msg);
	while (table->name != NULL) {
		const char *name = table->name;
		int len = strlen(name);
		int next = (table + 1)->name != NULL;
		int extra = next ? 2 : 0;
		if (wrap + len + extra > 40) {
			fprintf(stderr, "\n");
			wrap = 0;
		}
		fprintf(stderr, "%s", name);
		if (next) {
			fprintf(stderr, ", ");
			wrap += len + 2;
		} else {
			wrap += len;
		}
		table++;
	}
	fprintf(stderr, "\n");
}

/* Print programming options */
void print_options(minipro_handle_t *handle)
{
	device_t *device = handle->device;
	uint8_t bb = device->flags.custom_protocol;

	/* Print SPI clock if supported */
	if (device->flags.can_adjust_clock) {
		fprintf(stderr, "SPI Clock: %s MHz\n",
			get_parameter(handle, device->spi_clock, SPI_CLOCK));
	} else if (handle->cmdopts->set_spi_clock) {
		fprintf(stderr,
			"WARNING: Adjusting the SPI clock is not supported on this programmer!\n");
	}

	/* Print I2C slave address if supported */
	if (device->flags.can_adjust_address) {
		fprintf(stderr, "I2C slave address: 0x%02X\n",
			device->i2c_address);
	} else if (handle->cmdopts->set_i2c_addr) {
		fprintf(stderr,
			"WARNING: Changing the I2C slave address is not supported on this programmer!\n");
	}

	if (handle->cmdopts->action != WRITE || (!device->flags.can_adjust_vcc &&
	    !device->flags.can_adjust_vpp)) {
		return;
	}

	/* Print VPP */
	fprintf(stderr, "VPP: %s V",
		get_parameter(handle, device->voltages.vpp,
			      bb ? VPP_BB_VOLTAGE : VPP_VOLTAGE));

	/* Print VDD */
	if (handle->device->flags.can_adjust_vcc) {
		fprintf(stderr, ", VDD: %s V, ",
			get_parameter(handle, device->voltages.vdd,
				      bb ? VCC_BB_VOLTAGE : VCC_VOLTAGE));

		/* Print VCC */
		fprintf(stderr, "VCC: %s V, ",
			get_parameter(handle, device->voltages.vcc,
				      bb ? VCC_BB_VOLTAGE : VCC_VOLTAGE));

		/* Print pulse delay */
		fprintf(stderr, "Pulse: %u us\n", handle->device->pulse_delay);
	}
	fprintf(stderr, "\n");
}

/* Print the minipro info (-v option) */
void print_version_and_exit(cmdopts_t *cmdopts)
{
	fprintf(stderr, "Supported programmers: TL866A/CS, TL866II+, ");
	fprintf(stderr, "T48, T56, T76\n");
	minipro_handle_t *handle = minipro_open(VERBOSE);
	if (handle != NULL) {
		minipro_print_system_info(handle);
		minipro_close(handle);
	} /* No need to complain if this fails. */

	char output[] =
		"Commit date:\t%s\n"
		"Git commit:\t%s\n"
		"Git branch:\t%s\n"
		"Share dir:\t%s\n";

	fprintf(stderr, signon, VERSION);
	fprintf(stderr, output, GIT_DATE, GIT_HASH, GIT_BRANCH, SHARE_INSTDIR);
	db_data_t db_data;
	memset(&db_data, 0, sizeof(db_data));
	db_data.logicic_path = cmdopts->logicic_path;
	db_data.infoic_path = cmdopts->infoic_path;
	exit(print_chip_count(&db_data));
}

/* Add usage information to the manpage in man/minipro.1, not here. */
void print_help_and_exit(char *progname)
{
	char *myname;
	char usage[] =
		"Usage: %s [options]\n"
		"See the manual page (type \"man %s\" for documentation)\n\n";
	myname = basename(progname);
	fprintf(stderr, signon, VERSION);
	fprintf(stderr, usage, myname, myname);
	exit(EXIT_FAILURE);
}

/* Print supported programmers (-Q) */
void print_supported_programmers_and_exit(cmdopts_t *cmdopts)
{
	fprintf(stderr, "tl866a:  TL866CS/A\n"
			"tl866ii: TL866II+\n"
			"t48:     T48  (mostly complete)\n"
			"t56:     T56  (experimental)\n"
			"t76:     T76  (experimental)\n"
			);
	exit(EXIT_SUCCESS);
}

/* Print connected programmer version (-k) */
void print_connected_programmer_and_exit(cmdopts_t *cmdopts)
{
	minipro_handle_t *handle = minipro_open(NO_VERBOSE);
	if (!handle) {
		fprintf(stderr, "[No programmer found]\n");
	} else {
		switch (handle->version) {
		case MP_TL866A:
			fprintf(stderr, "tl866a: TL866A\n");
			break;
		case MP_TL866CS:
			fprintf(stderr, "tl866a: TL866CS\n");
			break;
		case MP_TL866IIPLUS:
			fprintf(stderr, "tl866ii: TL866II+\n");
			break;
		case MP_T48:
			fprintf(stderr, "t48: T48\n");
			break;
		case MP_T56:
			fprintf(stderr, "t56: T56\n");
			break;
		case MP_T76:
			fprintf(stderr, "t76: T76\n");
			break;
		default:
			fprintf(stderr, "[Unknown programmer version]\n");
		}
		free(handle);
	}
	exit(EXIT_SUCCESS);
}

/* List database devices. (-l and -L */
void print_devices_and_exit(cmdopts_t *cmdopts)
{
	db_data_t db_data;
	memset(&db_data, 0, sizeof(db_data));
	db_data.device_name = cmdopts->device_name;
	db_data.logicic_path = cmdopts->logicic_path;
	db_data.infoic_path = cmdopts->infoic_path;
	db_data.prog_version = cmdopts->version;
	if (get_programmer_version(&db_data.prog_version))
		exit(EXIT_FAILURE);

	/* If less is available under windows use it, otherwise just use more. */
	char *PAGER = "less";
	FILE *pager = NULL;
#ifdef _WIN32
	if (system("where less >nul 2>&1"))
		PAGER = "more";
#endif

	/* Detecting the mintty in windows
	 * The default isatty always return false */
	if (
#ifdef _WIN32
		_fileno(stdout)
#else
		isatty(STDOUT_FILENO)
#endif
		&& !cmdopts->device_name) {
		/* stdout is a terminal, opening pager */
		signal(SIGINT, SIG_IGN);
		char *pager_program = getenv("PAGER");
		if (!pager_program)
			pager_program = PAGER;
		pager = popen(pager_program, "w");
		dup2(fileno(pager), STDOUT_FILENO);
	}

	list_devices(&db_data);

	if (pager) {
		close(STDOUT_FILENO);
		pclose(pager);
	}

	exit(EXIT_SUCCESS);
}

/* Perform a hardware check (-t) */
void hardware_check_and_exit()
{
	minipro_handle_t *handle = minipro_open(VERBOSE);
	if (!handle) {
		exit(EXIT_FAILURE);
	}

	minipro_print_system_info(handle);
	if (handle->status == MP_STATUS_BOOTLOADER) {
		fprintf(stderr, "in bootloader mode!\nExiting...\n");
		exit(EXIT_FAILURE);
	}

	int ret = minipro_hardware_check(handle);
	minipro_close(handle);
	exit(ret);
}

/* Perform a firmware update (-F) */
void firmware_update_and_exit(const char *firmware)
{
	minipro_handle_t *handle = minipro_open(VERBOSE);
	if (!handle) {
		exit(EXIT_FAILURE);
	}
	minipro_print_system_info(handle);
	if (handle->status == MP_STATUS_BOOTLOADER)
		fprintf(stderr, "in bootloader mode!\n");
	int ret = minipro_firmware_update(handle, firmware);
	minipro_close(handle);
	exit(ret);
}

/* Autodetect 25xx SPI devices (-a) */
void spi_autodetect_and_exit(uint8_t package_type, cmdopts_t *cmdopts)
{
	minipro_handle_t *handle = minipro_open(VERBOSE);
	if (!handle) {
		exit(EXIT_FAILURE);
	}
	minipro_print_system_info(handle);
	if (handle->status == MP_STATUS_BOOTLOADER) {
		fprintf(stderr, "in bootloader mode!\n");
		exit(EXIT_FAILURE);
	}
	uint32_t chip_id, count = 0;
	handle->cmdopts = cmdopts;

	if (cmdopts->pincheck) {
		if (handle->version == MP_TL866IIPLUS) {
			device_t device;
			device.pin_map = (package_type == 8 ? 0x01 : 0x03);
			handle->device = &device;
			if (minipro_pin_test(handle)) {
				minipro_end_transaction(handle);
				handle->device = NULL;
				minipro_close(handle);
				exit(EXIT_FAILURE);
			}

			/*
			 * For the time being, device must be NULL
			 * before minipro_spi_autodetect() is called.
			 * Otherwise bb_autodetect() will be called,
			 * which is currently not implemented.
			 */
			handle->device = NULL;
		} else
			fprintf(stderr, "Pin test is not supported.\n");
	}

	if (minipro_spi_autodetect(handle, package_type >> 4, &chip_id)) {
		exit(EXIT_FAILURE);
	}

	db_data_t db_data;
	memset(&db_data, 0, sizeof(db_data));
	db_data.logicic_path = cmdopts->logicic_path;
	db_data.infoic_path = cmdopts->infoic_path;
	db_data.algo_path = cmdopts->algo_path;
	db_data.chip_id = chip_id;
	db_data.pin_count = package_type;
	db_data.prog_version = handle->version;
	db_data.count = &count;
	fprintf(stderr, "Autodetecting device (ID:0x%04X)\n", chip_id);
	if (list_devices(&db_data)) {
		minipro_close(handle);
		exit(EXIT_FAILURE);
	}

	fprintf(stderr, "%u device(s) found.\n", count);
	handle->device = NULL;
	minipro_close(handle);
	exit(EXIT_SUCCESS);
}

/* Print the chip info (-d) */
void print_device_info_and_exit(cmdopts_t *cmdopts)
{
	minipro_handle_t *handle = calloc(1, sizeof(minipro_handle_t));
	if (!handle) {
		fprintf(stderr, "Out of memory!\n");
	}
	handle->cmdopts = cmdopts;
	handle->version = cmdopts->version;
	if (get_programmer_version(&handle->version))
		exit(EXIT_FAILURE);
	if (get_device(handle)) {
		minipro_close(handle);
		exit(EXIT_FAILURE);
	}

	device_t *device = handle->device;
	fprintf(stderr, "\n%s\nName: %s\n", INFO, device->name);

	if (device->chip_type == MP_LOGIC) {
		fprintf(stderr,
			"Package:\t DIP%d\nVector count:\t %d\n%s\nDefault VCC voltage: %s V\n",
			device->package_details.pin_count, device->vector_count,
			SEP,
			get_parameter(handle, device->voltages.vcc,
				      LOGIC_VOLTAGE));
		print_parameters_table("Available VCC voltages [V]: ",
				       device->vcc_logic_table);
	} else {
		/* Availability */
        int c, all;
		fprintf(stderr, "Available on: ");
		switch (handle->version) {
		case MP_TL866A:
			fprintf(stderr, "TL866A/CS\n");
			break;
		case MP_T76:
			fprintf(stderr, "T76\n");
			break;
		default:
			c = 0;
			all = (!device->tl866_only && !device->t48_only &&
				   !device->t56_only);
			if (all || device->tl866_only) {
				fprintf(stderr, "TL866II");
				c++;
			}
			if (all || device->t48_only) {
				fprintf(stderr, "%sT48", c ? ", " : "");
				c++;
			}
			if (all || device->t56_only) {
				fprintf(stderr, "%sT56", c ? ", " : "");
				c++;
			}
			fprintf(stderr, "%s\n", --c ? "" : " only");
		}
		/* Memory shape */
		fprintf(stderr, "Memory: %u",
			device->code_memory_size / device->flags.word_size);
		switch (device->flags.data_org) {
		case MP_ORG_BYTES:
			fprintf(stderr, " Bytes");
			break;
		case MP_ORG_WORDS:
			fprintf(stderr, " Words");
			break;
		case MP_ORG_BITS:
			fprintf(stderr, " Bits");
			break;
		default:
			fprintf(stderr, "Unknown memory shape: 0x%x\n",
				device->flags.data_org);
			minipro_close(handle);
			exit(EXIT_FAILURE);
		}
		if (device->data_memory_size) {
			fprintf(stderr, " + %u Bytes",
				device->data_memory_size);
		}
		if (device->data_memory2_size) {
			fprintf(stderr, " + %u Bytes",
				device->data_memory2_size);
		}
		fprintf(stderr, "\n");

		/* Package info */
		fprintf(stderr, "Package: ");
		if (device->package_details.adapter) {
			fprintf(stderr, "Adapter%03d.JPG\n",
				device->package_details.adapter);
		} else if (device->package_details.pin_count) {
			fprintf(stderr, "%s%d\n",
				device->package_details.plcc ? "PLCC" : "DIP",
				device->package_details.pin_count);
		} else {
			fprintf(stderr, "ICSP only\n");
		}

		/* ICSP connection info if avialable */
		if (device->package_details.icsp) {
			fprintf(stderr, "ICSP: ICP%03d.JPG\n",
				device->package_details.icsp);
		}

		/* Protocol and algorithm info */
		fprintf(stderr, "Protocol: 0x%02x\n", device->protocol_id);
		if ((handle->version == MP_T56 || handle->version == MP_T76) &&
		    !get_algorithm(device, MP_T76, handle->cmdopts->algo_path,
				   handle->cmdopts->icsp,
				   handle->cmdopts->vopt)) {
			fprintf(stderr, "Algorithm: %s\n",
				device->algorithm.name);
			free(device->algorithm.bitstream);
		}

		/* Read/Write buffer size if available */
		if (device->read_buffer_size && device->write_buffer_size) {
			fprintf(stderr, "Read buffer size: %u Bytes\n",
				device->read_buffer_size);
			fprintf(stderr, "Write buffer size: %u Bytes\n",
				device->write_buffer_size);
		}

		/* Printing available SPI clock info */
		if (device->flags.can_adjust_clock && device->spi_clock_table) {
			fprintf(stderr, "%s\n", SEP);
			print_parameters_table(
				"Available SPI clock frequencies [MHz]: ",
				device->spi_clock_table);
		}

		/* Printing device programming voltages info */
		if (device->flags.can_adjust_vcc ||
		    device->flags.can_adjust_vpp) {
			fprintf(stderr, "%s\n", SEP);
			/* Print VPP */
			fprintf(stderr,
				"Default VPP programming voltage: %s V\n",
				get_parameter(handle, device->voltages.vpp,
					      VPP_VOLTAGE));
			print_parameters_table("Available VPP voltages [V]: ",
					       device->vpp_table);
			fprintf(stderr, "\n");

			if (device->flags.can_adjust_vcc) {
				/* Print VDD */
				fprintf(stderr,
					"Default VDD write voltage: %s V\n",
					get_parameter(handle,
						      device->voltages.vdd,
						      VCC_VOLTAGE));
				print_parameters_table(
					"Available VDD write voltages [V]: ",
					device->vcc_table);
				fprintf(stderr, "\n");

				/* Print VCC */
				fprintf(stderr,
					"Default VCC verify voltage: %s V\n",
					get_parameter(handle,
						      device->voltages.vcc,
						      VCC_VOLTAGE));
				print_parameters_table(
					"Available VCC verify voltages [V]: ",
					device->vcc_table);
				fprintf(stderr, "\n");

				/* Print pulse delay */
				fprintf(stderr,
					"Default write pulse: %u us\nAvailable write pulse[us]: 1-65535\n",
					device->pulse_delay);
			}
		}
		fprintf(stderr, "%s\n", SEP);
	}
	minipro_close(handle);
	exit(EXIT_SUCCESS);
}

/* Start, stop and report progress */
void progress_status(const char *label_fmt, int progress_percent, int done, ...)
{
	static struct {
		char label[64];
		struct timeval begin;
	} state;

	/* Start progress */
	if (label_fmt && !done && progress_percent < 0) {
		va_list args;
		va_start(args, done);
		vsnprintf(state.label, sizeof(state.label), label_fmt, args);
		va_end(args);
		gettimeofday(&state.begin, NULL);
		fprintf(stderr, "\r\e[K%s", state.label);
		fflush(stderr);
		return;
	}

	if (!done) {
		/* Progress update */
		fprintf(stderr, "\r\e[K%s%2d%%", state.label, progress_percent);
		fflush(stderr);
	} else {
		/* Done */
		struct timeval end;
		gettimeofday(&end, NULL);

		double elapsed =
			(end.tv_sec - state.begin.tv_sec) +
			(end.tv_usec - state.begin.tv_usec) / 1000000.0;

		fprintf(stderr, "\r\e[K%s", state.label);

		if (elapsed < 1.0) {
			double ms = elapsed * 1000;
			if (((int)(ms * 10)) % 10 == 0)
				fprintf(stderr, "%.0f ms  OK\n", ms);
			else
				fprintf(stderr, "%.1f ms  OK\n", ms);
		} else if (elapsed < 60.0) {
			if (((int)(elapsed * 100)) % 100 == 0)
				fprintf(stderr, "%.0f Sec  OK\n", elapsed);
			else if (((int)(elapsed * 10)) % 10 == 0)
				fprintf(stderr, "%.1f Sec  OK\n", elapsed);
			else
				fprintf(stderr, "%.2f Sec  OK\n", elapsed);
		} else {
			int minutes = (int)(elapsed / 60);
			int seconds = (int)(elapsed) % 60;
			fprintf(stderr, "%dm %d Sec  OK\n", minutes, seconds);
		}
		fflush(stderr);
	}
}

/* Parse command line options */
void parse_cmdline(int argc, char **argv, cmdopts_t *cmdopts)
{
	int8_t c;
	uint8_t package_type = 0;
	void (*p_func)(cmdopts_t *) = NULL;

	memset(cmdopts, 0, sizeof(cmdopts_t));
	long_options[6].flag = &cmdopts->filter_fuses;
	long_options[7].flag = &cmdopts->filter_uid;
	long_options[8].flag = &cmdopts->filter_locks;
	while ((c = getopt_long(argc, argv,
				"lL:q:Qkd:ea:zEbTuPvxyr:w:m:p:c:o:iIsSVhDtf:F:",
				long_options, NULL)) != -1) {
		switch (c) {
		case 0:
		case 2: /* Skip vdd, vcc, vpp, pulse and spi_clock here */
			break;
		case 3:
			cmdopts->infoic_path = optarg; /* Custom infoic.xml */
			break;
		case 4:
			cmdopts->logicic_path = optarg; /* Custom logicic.xml */
			break;
		case 5:
			cmdopts->logicic_out =
				optarg; /* Logic test output file */
			break;
		case 6:
			cmdopts->algo_path = optarg; /* Custom algorithm.xml */
			break;
		case 'q':
			if (!strcasecmp(optarg, "tl866a"))
				cmdopts->version = MP_TL866A;
			else if (!strcasecmp(optarg, "tl866ii"))
				cmdopts->version = MP_TL866IIPLUS;
			else if (!strcasecmp(optarg, "t48"))
				cmdopts->version = MP_T48;
			else if (!strcasecmp(optarg, "t56"))
				cmdopts->version = MP_T56;
			else if (!strcasecmp(optarg, "t76"))
				cmdopts->version = MP_T76;
			else {
				fprintf(stderr,
					"Unknown programmer version (%s).\n",
					optarg);
				print_help_and_exit(argv[0]);
			}
			break;

		case 'Q':
			p_func = print_supported_programmers_and_exit;
			break;

		case 'k':
			p_func = print_connected_programmer_and_exit;
			break;

		case 'l':
			p_func = print_devices_and_exit;
			break;

		case 'L':
			cmdopts->device_name = optarg;
			p_func = print_devices_and_exit;
			break;

		case 'd':
			cmdopts->device_name = optarg;
			p_func = print_device_info_and_exit;
			break;

		case 'e':
			cmdopts->no_erase = 1; /* 1= do not erase */
			break;

		case 'u':
			cmdopts->protect_off =
				1; /* 1 = disable write protect */
			break;

		case 'P':
			cmdopts->protect_on = 1; /* 1 =  enable write protect */
			break;

		case 'v':
			cmdopts->no_verify = 1; /* 1= do not verify */
			break;

		case 'x':
			cmdopts->idcheck_skip =
				1; /* 1= do not test id at all */
			break;

		case 'y':
			cmdopts->idcheck_continue =
				1; /* 1= do not stop on id mismatch */
			break;

		case 'z':
			cmdopts->pincheck =
				1; /* 1= Check for bad pin contact */
			break;

		case 'p':
			if (!strcasecmp(optarg, "help"))
				print_devices_and_exit(cmdopts);
			cmdopts->device_name = optarg;
			break;

		case 'c':
			if (!strcasecmp(optarg, "code"))
				cmdopts->page = CODE;
			else if (!strcasecmp(optarg, "data"))
				cmdopts->page = DATA;
			else if (!strcasecmp(optarg, "user"))
				cmdopts->page = USER;
			else if (!strcasecmp(optarg, "config"))
				cmdopts->page = CONFIG;
			else if (!strcasecmp(optarg, "calibration"))
				cmdopts->page = CALIBRATION;
			else {
				fprintf(stderr, "Unknown memory type\n");
				exit(EXIT_FAILURE);
			}
			break;

		case 'f':
			if (!strcasecmp(optarg, "ihex"))
				cmdopts->format = IHEX;
			if (!strcasecmp(optarg, "srec"))
				cmdopts->format = SREC;
			if (!cmdopts->format) {
				fprintf(stderr, "Unknown file format\n");
				exit(EXIT_FAILURE);
			}
			break;

		case 'r':
			cmdopts->action = READ;
			cmdopts->filename = optarg;
			break;

		case 'w':
			if (cmdopts->action == ERASE)
				cmdopts->force_erase = 1;
			cmdopts->action = WRITE;
			cmdopts->filename = optarg;
			break;

		case 'm':
			cmdopts->action = VERIFY;
			cmdopts->filename = optarg;
			break;

		case 'E':
			if (cmdopts->action != NO_ACTION)
				cmdopts->force_erase = 1;
			else
				cmdopts->action = ERASE;
			break;

		case 'b':
			cmdopts->action = BLANK_CHECK;
			break;

		case 'T':
			cmdopts->action = LOGIC_IC_TEST;
			break;

		case 'a':
			if (!strcasecmp(optarg, "8"))
				package_type = 8;
			else if (!strcasecmp(optarg, "16"))
				package_type = 16;
			else {
				fprintf(stderr, "Invalid argument.\n");
				print_help_and_exit(argv[0]);
			}
			break;

		case 'i':
			cmdopts->icsp = MP_ICSP_ENABLE | MP_ICSP_VCC;
			break;

		case 'I':
			cmdopts->icsp = MP_ICSP_ENABLE;
			break;

		case 'S':
			cmdopts->size_nowarn = 1;
			cmdopts->size_error = 1;
			break;

		case 's':
			cmdopts->size_error = 1;
			break;

		case 'D':
			cmdopts->idcheck_only = 1;
			break;

		case 'h':
			print_help_and_exit(argv[0]);
			break;

		case 'V':
			p_func = print_version_and_exit;
			break;

		case 't':
			hardware_check_and_exit();
			break;

		/*
       * Only check if the syntax is correct here.
       * The actual parsing of each 'o' option is done after the programmer
       * version is known.
       */
		case 'o':
			break;
		case 'F':
			firmware_update_and_exit(optarg);
			break;
		default:
			print_help_and_exit(argv[0]);
			break;
		}
	}

	if (optind < argc) {
		fprintf(stderr, "Extra argument: '%s'\n", argv[optind]);
		print_help_and_exit(argv[0]);
	}

	if (cmdopts->filter_fuses || cmdopts->filter_locks ||
	    cmdopts->filter_uid)
		cmdopts->page = CONFIG;
	if (cmdopts->version && !p_func) {
		fprintf(stderr,
			"-L, -l or -d command is required for this action.\n");
		print_help_and_exit(argv[0]);
	}
	if (p_func != NULL)
		p_func(cmdopts);
	if (package_type)
		spi_autodetect_and_exit(package_type, cmdopts);

	/* Check if a file name is required */
	switch (cmdopts->action) {
	case LOGIC_IC_TEST:
		break;
	case READ:
	case WRITE:
	case VERIFY:
		if (!cmdopts->filename && !cmdopts->idcheck_only) {
			fprintf(stderr,
				"A file name is required for this action.\n");
			print_help_and_exit(argv[0]);
		}
		break;
	default:
		break;
	}

	/* Check if a device name is required */
	if (!cmdopts->device_name) {
		fprintf(stderr,
			"Device required. Use -p <device> to specify a device.\n");
		print_help_and_exit(argv[0]);
	}

	/* don't permit skipping the ID read in write/erase-mode or ID
	 * only mode */
	if ((cmdopts->action == WRITE || cmdopts->action == ERASE ||
	     cmdopts->idcheck_only) &&
	    cmdopts->idcheck_skip) {
		fprintf(stderr,
			"Skipping the ID check is not permitted for this action.\n");
		print_help_and_exit(argv[0]);
	}

	/* Exit if no action is supplied */
	if (cmdopts->action == NO_ACTION && !cmdopts->idcheck_only &&
	    !cmdopts->pincheck) {
		fprintf(stderr, "No action to perform.\n");
		print_help_and_exit(argv[0]);
	}

	/* Set the pipe flag */
	if (cmdopts->filename) {
		cmdopts->is_pipe = (!strcmp(cmdopts->filename, "-"));
	}
}

/* Parse and set programming options for all supported programmers */
int parse_options(minipro_handle_t *handle, int argc, char **argv)
{
	uint32_t v;
	int8_t c;
	char *p_end, option[64], value[64];
	int opt_idx = 0;
	device_t *device = handle->device;
	uint8_t bb = device->flags.custom_protocol;

	/* Parse options first */
	optind = 1;
	opterr = 0;

	while ((c = getopt_long(argc, argv, "o:", long_options, &opt_idx)) !=
	       -1) {
		switch (c) {
		case 2:
			if (!strlen(optarg)) {
				fprintf(stderr,
					"%s: option '--%s' requires an argument\n",
					argv[0], long_options[opt_idx].name);
				return EXIT_FAILURE;
			}
			switch (opt_idx) {
			case 0:
				errno = 0;
				v = strtoul(optarg, &p_end, 0);
				if ((p_end == optarg) || errno)
					return EXIT_FAILURE;
				if (v > 0xffff)
					return EXIT_FAILURE;
				device->pulse_delay = (uint16_t)v;
				break;
			case 1:
				if (set_parameter(handle, optarg,
						  &device->voltages.vpp,
						  bb ? VPP_BB_VOLTAGE :
						       VPP_VOLTAGE))
					return EXIT_FAILURE;
				break;
			case 2:
				if (set_parameter(handle, optarg,
						  &device->voltages.vdd,
						  bb ? VCC_BB_VOLTAGE :
						       VCC_VOLTAGE))
					return EXIT_FAILURE;
				break;
			case 3:
				if (set_parameter(handle, optarg,
						  &device->voltages.vcc,
						  bb ? VCC_BB_VOLTAGE :
						       VCC_VOLTAGE))
					return EXIT_FAILURE;
				break;
			case 4:
				handle->cmdopts->set_spi_clock = 1;
				if (!device->flags.can_adjust_clock)
					break;
				if (set_parameter(handle, optarg,
						  &device->spi_clock,
						  SPI_CLOCK))
					return EXIT_FAILURE;
				break;
			case 5:
				handle->cmdopts->set_i2c_addr = 1;
				if (!device->flags.can_adjust_address)
					break;
				errno = 0;
				v = strtoul(optarg, &p_end, 0);
				if ((p_end == optarg) || errno)
					return EXIT_FAILURE;
				if (v > 0xff)
					return EXIT_FAILURE;
				device->i2c_address = (uint8_t)v;
				break;
			default:
				return EXIT_FAILURE;
			}
			break;
		case 'o':
			if (sscanf(optarg, "%[^=]=%[^=]", option, value) != 2)
				return EXIT_FAILURE;
			if (!strcasecmp(option, "pulse")) {
				/* Parse the numeric value */
				errno = 0;
				v = strtoul(value, &p_end, 0);
				if ((p_end == value) || errno)
					return EXIT_FAILURE;
				if (v > 0xffff)
					return EXIT_FAILURE;
				handle->device->pulse_delay = (uint16_t)v;
			}
			if (!strcasecmp(option, "address")) {
				handle->cmdopts->set_i2c_addr = 1;
				if (!device->flags.can_adjust_address)
					break;
				/* Parse the numeric value */
				errno = 0;
				v = strtoul(optarg, &p_end, 0);
				if ((p_end == optarg) || errno)
					return EXIT_FAILURE;
				if (v > 0xff)
					return EXIT_FAILURE;
				device->i2c_address = (uint8_t)v;
				handle->cmdopts->set_i2c_addr = 1;
			} else if (!strcasecmp(option, "vpp")) {
				if (set_parameter(handle, value,
						  &device->voltages.vpp,
						  bb ? VPP_BB_VOLTAGE :
						       VPP_VOLTAGE))
					return EXIT_FAILURE;
			} else if (!strcasecmp(option, "vdd")) {
				if (set_parameter(handle, value,
						  &device->voltages.vdd,
						  bb ? VCC_BB_VOLTAGE :
						       VCC_VOLTAGE))
					return EXIT_FAILURE;
			} else if (!strcasecmp(option, "vcc")) {
				if (set_parameter(handle, value,
						  &device->voltages.vcc,
						  bb ? VCC_BB_VOLTAGE :
						       VCC_VOLTAGE))
					return EXIT_FAILURE;
			} else if (!strcasecmp(option, "spi_clock")) {
				handle->cmdopts->set_spi_clock = 1;
				if (!device->flags.can_adjust_clock)
					break;
				if (set_parameter(handle, value,
						  &device->spi_clock,
						  SPI_CLOCK))
					return EXIT_FAILURE;
			} else
				return EXIT_FAILURE;
			break;
		}
	}
	return EXIT_SUCCESS;
}

/* Search for fuse name in buffer. */
int get_fuse_value(const char *buffer, size_t size, const char *key,
		   uint16_t *value)
{
	char *copy = strndup(buffer, size);
	if (!copy)
		return EXIT_FAILURE;

	char *token = strtok(copy, " \t\r\n");
	while (token) {
		char *eq = strchr(token, '=');
		if (eq) {
			*eq = '\0';
			const char *k = token;
			const char *v = eq + 1;

			if (!strcasecmp(k, key)) {
				char *endptr = NULL;
				unsigned long val = strtoul(v, &endptr, 0);
				if (*endptr == '\0' && val <= 0xFFFF) {
					*value = (uint16_t)val;
					free(copy);
					return EXIT_SUCCESS;
				}
			}
		}
		token = strtok(NULL, " \t\r\n");
	}

	free(copy);
	return EXIT_FAILURE;
}

/* Compare memory */
int compare_memory(uint8_t compare_mask, uint8_t *s1, uint8_t *s2, size_t size1,
		   size_t size2, uint32_t *address, uint8_t *c1, uint8_t *c2)
{
	size_t i;
	uint8_t v1, v2;
	size_t size = (size1 > size2) ? size2 : size1;
	for (i = 0; i < size; i++) {
		v1 = (i < size1) ?
			     s1[i] :
			     compare_mask; /* use mask value when buffer too short */
		v2 = (i < size2) ? s2[i] : compare_mask;
		v1 &= compare_mask;
		v2 &= compare_mask;
		if (v1 != v2) {
			if (address)
				*address = i;
			if (c1)
				*c1 = v1;
			if (c2)
				*c2 = v2;
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

/* returned value will be a byte offset
 * sizes are in bytes
 * replacement_value needs to be in native byte order
 * sizes can be odd */
int compare_word_memory(uint16_t replacement_value, uint16_t compare_mask,
			uint8_t little_endian, uint8_t *s1, uint8_t *s2,
			size_t size1, size_t size2, uint32_t *address,
			uint16_t *c1, uint16_t *c2)
{
	size_t i;
	uint16_t v1, v2;
	size_t size = (size1 > size2) ? size2 : size1;
	if (compare_mask == 0)
		compare_mask = 0xffff;
	uint8_t rvl = (replacement_value & compare_mask) & 0xff;
	uint8_t rvh = ((replacement_value & compare_mask) >> 8) & 0xff;

	for (i = 0; i < size; i += 2) {
		if (little_endian) {
			v1 = (i < size1) ? s1[i] : rvl;
			v1 |= (((i + 1) < size1) ? s1[i + 1] : rvh) << 8;
			v2 = (i < size2) ? s2[i] : rvl;
			v2 |= (((i + 1) < size2) ? s2[i + 1] : rvh) << 8;
		} else {
			v1 = ((i < size1) ? s1[i] : rvh) << 8;
			v1 |= ((i + 1) < size1) ? (s1[i + 1]) : rvl;
			v2 = ((i < size2) ? s2[i] : rvh) << 8;
			v2 |= ((i + 1) < size2) ? (s2[i + 1]) : rvl;
		}
		if ((v1 & compare_mask) != (v2 & compare_mask)) {
			if (address)
				*address = i;
			if (c1)
				*c1 = v1;
			if (c2)
				*c2 = v2;
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

/* RAM-centric IO operations */
int read_page_ram(minipro_handle_t *handle, uint8_t *buffer, uint8_t type,
		  size_t size)
{
	const char *name = (type == MP_DATA) ? "Data" :
			   (type == MP_USER) ? "User" :
					       "Code";

	data_set_t ds = { .data = buffer,
			  .type = type,
			  .size = (size < handle->device->read_buffer_size) ?
					  size :
					  handle->device->read_buffer_size,
			  .init = 1 };

	ds.block_count = (size + ds.size - 1) / ds.size;

	/* NAND (T76) is read one erase-block at a time (data + spare), so the
	 * transfer unit is write_buffer_size * pages_per_block (e.g. 0x840 *
	 * 0x40 = 0x21000), not the generic read_buffer_size. Each per-block
	 * read command carries the block index; see t76_read_block. */
	if (handle->version == MP_T76 && type == MP_CODE &&
	    handle->device->protocol_id == IC2_ALG_NAND &&
	    handle->device->pages_per_block) {
		ds.size = (size_t)handle->device->write_buffer_size *
			  handle->device->pages_per_block;
		ds.block_count = (size + ds.size - 1) / ds.size;
	}

	/* eMMC (T76): the vendor streams a region over a single 0x0D read as
	 * back-to-back 64 KiB (0x10000) blocks on EP82 (capture xgpro-3). Feed
	 * t76_read_block one 64 KiB block per call; it sends the 0x0D init once
	 * (with the start LBA and total 64 KiB-block count) then drains one block
	 * per call. read size must be a multiple of 0x10000. */
	if (handle->version == MP_T76 && type == MP_CODE &&
	    handle->device->protocol_id == IC2_ALG_EMMC) {
		ds.size = 0x10000;
		ds.block_count = (size + ds.size - 1) / ds.size;
	}

	/* Some controllers have data memory (eeprom) mapped to a
	 * different address than 0 in programming mode. For ex. AT89S8252 */
	uint32_t offset = handle->device->flags.has_data_offset ?
				  handle->device->page_size :
				  0;

	/* Initialize progress reporting */
	progress_status("Reading %s...  ", -1, 0, name);

	for (size_t i = 0; i < ds.block_count; i++) {
		/* Translating address to protocol-specific */
		ds.address = i * ds.size + offset;

		if (handle->device->flags.data_org == MP_ORG_WORDS &&
		    type == MP_CODE &&
		    handle->device->protocol_id != IC2_ALG_NAND) {
			ds.address >>= 1;
		}

		/* eMMC addresses by 512-byte LBA, not byte. A byte address would
		 * overflow uint32 past 4 GiB; the sector index fits (a 2 TiB part
		 * is still only 0xFFFFFFFF sectors). */
		if (handle->version == MP_T76 && type == MP_CODE &&
		    handle->device->protocol_id == IC2_ALG_EMMC)
			ds.address = (uint32_t)((uint64_t)i * (ds.size >> 9));

		if (minipro_read_block(handle, &ds))
			return EXIT_FAILURE;

		ds.data += ds.size;
		ds.init = 0;

		/* Report progress */
		progress_status(NULL, i * 100 / ds.block_count, 0);

		/* T76 doesn't support calling get_ovc_status while read/write */
		if (handle->version != MP_T76) {
			uint8_t ovc = 0;
			if (minipro_get_ovc_status(handle, NULL, &ovc))
				return EXIT_FAILURE;
			if (ovc) {
				fprintf(stderr,
					"\nOvercurrent protection!\007\n");
				return EXIT_FAILURE;
			}
		}
	}

	/* Stop progress and print elapsed time */
	progress_status(NULL, 0, 1);

	return EXIT_SUCCESS;
}

int write_page_ram(minipro_handle_t *handle, uint8_t *buffer, uint8_t type,
		   size_t size)
{
	const char *name = (type == MP_DATA) ? "Data" :
			   (type == MP_USER) ? "User" :
					       "Code";

	data_set_t ds = { .data = buffer,
			  .type = type,
			  .size = handle->device->write_buffer_size,
			  .init = 1 };
	ds.block_count = (size + ds.size - 1) / ds.size;

	/* NAND (T76) is programmed one erase-block at a time (data + spare):
	 * t76_write_block sends a per-block 0x1F init then streams each page.
	 * Feed it the block-with-spare size (write_buffer_size * pages_per_block,
	 * e.g. 0x21000), mirroring read_page_ram. */
	if (handle->version == MP_T76 && type == MP_CODE &&
	    handle->device->protocol_id == IC2_ALG_NAND &&
	    handle->device->pages_per_block) {
		ds.size = (size_t)handle->device->write_buffer_size *
			  handle->device->pages_per_block;
		ds.block_count = (size + ds.size - 1) / ds.size;
	}

	/* eMMC (T76): program a region as 64 KiB (0x10000) blocks streamed on
	 * EP05 after a single 0x1F init (mirrors the read path). */
	if (handle->version == MP_T76 && type == MP_CODE &&
	    handle->device->protocol_id == IC2_ALG_EMMC) {
		ds.size = 0x10000;
		ds.block_count = (size + ds.size - 1) / ds.size;
	}

	minipro_status_t status;

	/* Some controllers have data memory (eeprom) mapped to a
	 * different address than 0 in programming mode. For ex. AT89S8252 */
	uint32_t offset = handle->device->flags.has_data_offset ?
				  handle->device->page_size :
				  0;

	/* Initialize progress reporting */
	progress_status("Writing %s...  ", -1, 0, name);

	for (size_t i = 0; i < ds.block_count; i++) {
		/* Translating address to protocol-specific */
		ds.address = i * ds.size + offset;
		/* x16 chips (data_org == WORDS) use word addresses on the
		 * TL866-era programmers, so the byte address is halved. The
		 * T76's bulk WRITE_CODE (0x0C) addresses by BYTE, though: the
		 * per-block address must step by write_buffer_size, not half
		 * of it. Halving here made consecutive 256-byte blocks land
		 * 0x80 apart and overlap, ANDing the overlap region (a write
		 * of 0x91 over 0x11 read back as 0x11). Skip the halving for
		 * the T76. */
		if (handle->device->flags.data_org == MP_ORG_WORDS &&
		    type == MP_CODE && handle->version != MP_T76) {
			ds.address >>= 1;
		}

		/* eMMC: address by 512-byte LBA (sector index), not byte — fits
		 * uint32 at multi-GiB capacities. Computed from the full chunk
		 * before any last-block shrink below. */
		if (handle->version == MP_T76 && type == MP_CODE &&
		    handle->device->protocol_id == IC2_ALG_EMMC)
			ds.address = (uint32_t)((uint64_t)i * (ds.size >> 9));

		/* Last block */
		if ((i + 1) * ds.size > size)
			ds.size = size % ds.size;

		if (minipro_write_block(handle, &ds))
			return EXIT_FAILURE;

		ds.data += ds.size;
		ds.init = 0;

		/* Report progress */
		progress_status(NULL, i * 100 / ds.block_count, 0);

		/* T76 doesn't support calling get_ovc_status while read/write */
		if (handle->version != MP_T76) {
			uint8_t ovc = 0;
			if (minipro_get_ovc_status(handle, &status, &ovc))
				return EXIT_FAILURE;

			if (ovc) {
				fprintf(stderr,
					"\nOvercurrent protection!\007\n");
				return EXIT_FAILURE;
			}

			if (status.error && !handle->cmdopts->no_verify) {
				if (minipro_end_transaction(handle))
					return EXIT_FAILURE;

				uint32_t mask =
					handle->device->flags.word_size == 1 ?
						0xFF :
						0xFFFF;

				fprintf(stderr,
					"\nVerification failed at address 0x%04X: File=0x%02X, Device=0x%02X\n",
					status.address, status.c2 & mask,
					status.c1 & mask);
				return EXIT_FAILURE;
			}
		}
	}

	/* Stop progress and print elapsed time */
	progress_status(NULL, 0, 1);

	return EXIT_SUCCESS;
}

/* Read PLD device */
int read_jedec(minipro_handle_t *handle, jedec_t *jedec)
{
	uint8_t buffer[32];
	gal_config_t *config = (gal_config_t *)handle->device->config;

	uint8_t ovc = 0;
	if (minipro_get_ovc_status(handle, NULL, &ovc))
		return EXIT_FAILURE;
	if (ovc) {
		fprintf(stderr, "\nOvercurrent protection!\007\n");
		return EXIT_FAILURE;
	}

	jedec_set_t js = {
		.data = buffer, .flags = 0, .size = config->row_width, .type = 2
	};

	/* Initialize progress reporting */
	progress_status("Reading device... ", -1, 0);

	/* Read fuses */
	memset(jedec->fuses, 0, jedec->QF);
	for (js.row = 0; js.row < config->fuses_size; js.row++) {
		if (minipro_read_jedec_row(handle, &js))
			return EXIT_FAILURE;
		/* Unpacking the row */
		for (int j = 0; j < config->row_width; j++) {
			if (buffer[j / 8] & (0x80 >> (j & 0x07)))
				jedec->fuses[config->fuses_size * j + js.row] =
					1;
		}

		/* Report progress */
		progress_status(NULL, js.row * 100 / config->fuses_size, 0);
	}

	/* Read user electronic signature (UES)
	 * UES data can be missing in jedec, e.g. for db entry "ATF22V10C" */
	js.type = 1;
	if ((config->ues_address != 0) && (config->ues_size != 0) &&
	    ((config->ues_address + config->ues_size) <= jedec->QF) &&
	    !(handle->device->voltages.vdd & ATF_IN_PAL_COMPAT_MODE)) {
		js.size = config->ues_size;
		if (minipro_read_jedec_row(handle, &js))
			return EXIT_FAILURE;
		for (int j = 0; j < config->ues_size; j++) {
			if (buffer[j / 8] & (0x80 >> (j & 0x07)))
				jedec->fuses[config->ues_address + j] = 1;
		}
	}

	/* Read architecture control word (ACW) */
	js.type = 2;
	js.row = config->acw_address;
	js.flags = config->acw_address;
	js.size = config->acw_size;
	if (minipro_read_jedec_row(handle, &js))
		return EXIT_FAILURE;
	for (int i = 0; i < config->acw_size; i++) {
		if (buffer[i / 8] & (0x80 >> (i & 0x07)))
			jedec->fuses[config->acw_bits[i]] = 1;
	}

	/* Read Power-Down bit */
	js.row = config->powerdown_row;
	js.flags = 0;
	js.size = 1;
	if ((config->powerdown_row != 0) &&
	    (handle->device->flags.has_power_down)) {
		if (minipro_read_jedec_row(handle, &js))
			return EXIT_FAILURE;
		jedec->fuses[jedec->QF - 1] = (buffer[0] >> 7) & 0x01;
	}

	/* Stop progress and print elapsed time */
	progress_status(NULL, 0, 1);

	return EXIT_SUCCESS;
}

/* Write PLD device */
int write_jedec(minipro_handle_t *handle, jedec_t *jedec)
{
	uint8_t buffer[32];
	gal_config_t *config = (gal_config_t *)handle->device->config;

	uint8_t ovc = 0;
	if (minipro_get_ovc_status(handle, NULL, &ovc))
		return EXIT_FAILURE;
	if (ovc) {
		fprintf(stderr, "\nOvercurrent protection!\007\n");
		return EXIT_FAILURE;
	}

	jedec_set_t js = {
		.data = buffer, .flags = 0, .size = config->row_width, .type = 0
	};

	/* Initialize progress reporting */
	progress_status("Writing jedec file... ", -1, 0);

	/* Write fuses */
	js.type = 0;
	for (js.row = 0; js.row < config->fuses_size; js.row++) {
		memset(buffer, 0, sizeof(buffer));
		/* Building a row */
		for (int j = 0; j < config->row_width; j++) {
			if (jedec->fuses[config->fuses_size * j + js.row] == 1)
				buffer[j / 8] |= (0x80 >> (j & 0x07));
		}

		/* Report progress */
		progress_status(NULL, js.row * 100 / config->fuses_size, 0);

		if (minipro_write_jedec_row(handle, &js))
			return EXIT_FAILURE;
	}

	/* Write user electronic signature (UES) */
	memset(buffer, 0, sizeof(buffer));
	/* UES data can be missing in jedec, e.g. for db entry "ATF22V10C" */
	if ((config->ues_address != 0) && (config->ues_size != 0) &&
	    ((config->ues_address + config->ues_size) <= jedec->QF) &&
	    !(handle->device->voltages.vdd & ATF_IN_PAL_COMPAT_MODE)) {
		for (int j = 0; j < config->ues_size; j++) {
			if (jedec->fuses[config->ues_address + j] == 1)
				buffer[j / 8] |= (0x80 >> (j & 0x07));
		}
	}
	/* UES field is always written, even when not contained in JEDEC */
	js.size = config->ues_size;
	if (minipro_write_jedec_row(handle, &js))
		return EXIT_FAILURE;

	/* Write architecture control word (ACW) */
	js.type = 2;
	memset(buffer, 0, sizeof(buffer));
	for (int i = 0; i < config->acw_size; i++) {
		if (jedec->fuses[config->acw_bits[i]] == 1)
			buffer[i / 8] |= (0x80 >> (i & 0x07));
	}

	js.row = config->acw_address;
	js.flags = config->acw_address;
	js.size = config->acw_size;
	if (minipro_write_jedec_row(handle, &js))
		return EXIT_FAILURE;

	/* Disable Power-Down by writing to specific power-down row */
	js.row = config->powerdown_row;
	js.flags = 0;
	js.size = 1;
	if (config->powerdown_row != 0) {
		/* only '0' bits shall be written */
		if (((handle->device->flags.has_power_down) &&
		     (jedec->fuses[jedec->QF - 1] == 0)) ||
		    (handle->device->flags.is_powerdown_disabled)) {
			memset(buffer, 0, sizeof(buffer));
			if (minipro_write_jedec_row(handle, &js))
				return EXIT_FAILURE;
		}
	}

	/* Stop progress and print elapsed time */
	progress_status(NULL, 0, 1);

	return EXIT_SUCCESS;
}

/* Tweaks settings before minipro_begin_transaction */
int begin_transaction(minipro_handle_t *handle)
{
	pack_voltages(&handle->device->voltages);
	return minipro_begin_transaction(handle);
}

/* Perform a chip erase */
int erase_device(minipro_handle_t *handle)
{
	uint8_t num_fuses = 0, pld = 0;
	device_t *device = handle->device;
	uint8_t version = handle->version;

	/* Not all chips can be erased... */
	if (handle->cmdopts->no_erase == 0 && device->flags.can_erase) {
		/* Initialize progress reporting */
		progress_status("Erasing... ", -1, 0);

		/* Get the appropriate parameters for the erase command */
		if (device->config) {
			if (device->chip_type == MP_PLD) {
				if (version == MP_TL866IIPLUS ||
				    version == MP_T48 || version == MP_T56 ||
				    version == MP_T76) {
					pld = (device->protocol_id ==
					       IC2_ALG_GAL22) ?
						      ERASE_PLD1 :
						      ERASE_PLD2;
				}
			} else {
				fuse_decl_t *fuses =
					(fuse_decl_t *)device->config;
				num_fuses = (fuses->num_fuses > 4) ?
						    1 :
						    fuses->num_fuses;
			}
		}

		if (minipro_erase(handle, num_fuses, pld))
			return EXIT_FAILURE;

		/* Stop progress and print elapsed time */
		progress_status(NULL, 0, 1);
	}
	return EXIT_SUCCESS;
}

/* Check the chip ID if applicable */
int check_chip_id(minipro_handle_t *handle)
{
	uint8_t id_type;
	uint32_t chip_id, chip_id_temp;
	uint8_t shift = 0;
	device_t *device = handle->device;
	cmdopts_t *cmdopts = handle->cmdopts;
	fuse_decl_t *config = (fuse_decl_t *)device->config;

	/* Read the chip ID */
	if (begin_transaction(handle) ||
	    minipro_get_chip_id(handle, &id_type, &chip_id) ||
	    minipro_end_transaction(handle)) {
		return EXIT_FAILURE;
	}
	chip_id_temp = chip_id;
	/* Parse chip ID */
	switch (id_type) {
	case MP_ID_TYPE1:
	case MP_ID_TYPE2:
	case MP_ID_TYPE5:
		if (chip_id == device->chip_id) {
			fprintf(stderr, "Chip ID: 0x%04X  OK\n", chip_id);
			return EXIT_SUCCESS;
		}
		break;

	case MP_ID_TYPE3:
		if ((device->chip_id >> 5) == (chip_id >> 5)) {
			fprintf(stderr, "Chip ID: 0x%04X, Rev.0x%02X  OK\n",
				chip_id >> 5, chip_id & 0x1F);
			return EXIT_SUCCESS;
		}
		shift = 5;
		chip_id >>= 5;
		chip_id_temp = chip_id << 5;
		break;

	case MP_ID_TYPE4:
		if ((device->chip_id >> config->rev_bits) ==
		    (chip_id >> config->rev_bits)) {
			fprintf(stderr, "Chip ID: 0x%04X, Rev.0x%02X  OK\n",
				chip_id >> config->rev_bits,
				chip_id & ~(0xFF << config->rev_bits));
			return EXIT_SUCCESS;
		}
		shift = config->rev_bits;
		chip_id >>= shift;
		chip_id_temp = chip_id << shift;
		break;
	}

	/* If we reach this point, the chip ID didn't match.
	 * Attempt to identify the chip from the database.
	 */
	db_data_t db_data = { 0 };
	db_data.logicic_path = cmdopts->logicic_path;
	db_data.infoic_path = cmdopts->infoic_path;
	db_data.prog_version = handle->version;
	db_data.chip_id = chip_id_temp;
	db_data.protocol = device->protocol_id;

	const char *name = get_device_from_id(&db_data);

	if (cmdopts->idcheck_only) {
		fprintf(stderr,
			"Chip ID mismatch: expected 0x%04X, got 0x%04X (%s)\n",
			device->chip_id >> shift, chip_id_temp >> shift,
			name ? name : "unknown");
		if (name)
			free((char *)name);
		return EXIT_FAILURE;
	}

	if (cmdopts->idcheck_continue) {
		fprintf(stderr,
			"WARNING: Chip ID mismatch: expected 0x%04X, got 0x%04X (%s)\n",
			device->chip_id >> shift, chip_id_temp >> shift,
			name ? name : "unknown");
	} else {
		fprintf(stderr,
			"Invalid Chip ID: expected 0x%04X, got 0x%04X (%s)\n"
			"(use '-y' to continue anyway at your own risk)\n",
			device->chip_id >> shift, chip_id_temp >> shift,
			name ? name : "unknown");
		if (name)
			free((char *)name);
		return EXIT_FAILURE;
	}

	if (name)
		free((char *)name);
	return EXIT_SUCCESS;
}

/* Check for various adapters */
int check_adapter(minipro_handle_t *handle)
{
	/* Unlocking the TSOP48 adapter (if applicable) */
	uint8_t status;
	switch (handle->version) {
	case MP_TL866A:
	case MP_TL866CS:
	case MP_TL866IIPLUS:
		switch (handle->device->package_details.adapter) {
		case TSOP48_ADAPTER:
		case SOP44_ADAPTER:
		case SOP56_ADAPTER:
			if (minipro_unlock_tsop48(handle, &status)) {
				return EXIT_FAILURE;
			}
			switch (status) {
			case MP_TSOP48_TYPE_V3:
				fprintf(stderr, "Found TSOP adapter V3\n");
				break;
			case MP_TSOP48_TYPE_NONE:
				/* Needed to turn off the power on the ZIF socket. */
				minipro_end_transaction(handle);
				fprintf(stderr, "TSOP adapter not found!\n");
				return EXIT_FAILURE;
			case MP_TSOP48_TYPE_V0:
				fprintf(stderr, "Found TSOP adapter V0\n");
				break;
			case MP_TSOP48_TYPE_FAKE1:
			case MP_TSOP48_TYPE_FAKE2:
				fprintf(stderr, "Fake TSOP adapter found!\n");
				break;
			}
			minipro_end_transaction(handle);
			break;
		}
		break;
	case MP_T48:
		break;
	case MP_T56:
		break;
	case MP_T76:
		break;
	}
	return EXIT_SUCCESS;
}

/* Opens a physical file or a pipe if the pipe character is specified */
int open_file(minipro_handle_t *handle, uint8_t *data, size_t *file_size)
{
	FILE *file;
	struct stat st;

	/* Check if we are dealing with a pipe. */
	if (handle->cmdopts->is_pipe) {
		file = stdin;
		st.st_size = 0;
	} else {
		file = fopen(handle->cmdopts->filename, "rb");
		int ret = stat(handle->cmdopts->filename, &st);
		if (!file || ret) {
			fprintf(stderr, "Could not open file %s for reading.\n",
				handle->cmdopts->filename);
			perror("");
			if (file)
				fclose(file);
			return EXIT_FAILURE;
		}
	}

	/* Allocate a zero initialized buffer.
	 * If the file size is unknown (pipe) a default size will be used. */
	uint8_t *buffer = calloc(1, st.st_size ? st.st_size : READ_BUFFER_SIZE);
	if (!buffer) {
		fclose(file);
		fprintf(stderr, "Out of memory!\n");
		return EXIT_FAILURE;
	}

	/* Try to read the whole file.
	 * If we are reading from stdin, data will be read in small
	 * chunks of 64K each until EOF. */
	size_t br = 0;
	size_t sz = READ_BUFFER_SIZE;
	if (!st.st_size) {
		size_t ch;
		uint8_t *tmp;
		while (br < UINT32_MAX) {
			ch = fread(buffer + br, 1, READ_BUFFER_SIZE, file);
			br += ch;
			if (ch != READ_BUFFER_SIZE)
				break;
			sz += READ_BUFFER_SIZE;
			tmp = realloc(buffer, sz);
			if (!tmp) {
				free(buffer);
				fclose(file);
				fprintf(stderr, "Out of memory!\n");
				return EXIT_FAILURE;
			}
			buffer = tmp;
		}
	} else
		br = fread(buffer, 1, st.st_size, file);

	fclose(file);
	if (!br) {
		fprintf(stderr, "No data to read.\n");
		free(buffer);
		return EXIT_FAILURE;
	}

	/* If we are dealing with a jed file just return the data. */
	if (handle->device->chip_type == MP_PLD) {
		memcpy(data, buffer, br);
		free(buffer);
		*file_size = br;
		return EXIT_SUCCESS;
	}

	size_t chip_size = *file_size;
	*file_size = br;

	/* Probe for an Intel hex file */
	size_t hex_size = chip_size;
	int ret = read_hex_file(buffer, data, &hex_size);
	switch (ret) {
	case NOT_IHEX:
		break;
	case EXIT_FAILURE:
		free(buffer);
		return EXIT_FAILURE;
		break;
	case INTEL_HEX_FORMAT:
		*file_size = hex_size;
		fprintf(stderr, "Found Intel hex file.\n");
		free(buffer);
		return EXIT_SUCCESS;
	}

	/* Probe for a Motorola srec file */
	hex_size = chip_size;
	ret = read_srec_file(buffer, data, &hex_size);
	switch (ret) {
	case NOT_SREC:
		break;
	case EXIT_FAILURE:
		free(buffer);
		return EXIT_FAILURE;
		break;
	case SREC_FORMAT:
		*file_size = hex_size;
		fprintf(stderr, "Found Motorola S-Record file.\n");
		free(buffer);
		return EXIT_SUCCESS;
	}

	if (handle->cmdopts->format == IHEX) {
		fprintf(stderr, "This is not an Intel hex file.\n");
		free(buffer);
		return EXIT_FAILURE;
	}
	if (handle->cmdopts->format == SREC) {
		fprintf(stderr, "This is not an S-Record file.\n");
		free(buffer);
		return EXIT_FAILURE;
	}
	/* This must be a binary file */
	memcpy(data, buffer, *file_size > chip_size ? chip_size : *file_size);
	free(buffer);
	return EXIT_SUCCESS;
}

/* Open a JED file */
int open_jed_file(minipro_handle_t *handle, jedec_t *jedec)
{
	char *buffer = calloc(1, READ_BUFFER_SIZE);
	if (!buffer) {
		fprintf(stderr, "Out of memory!\n");
		return EXIT_FAILURE;
	}

	size_t file_size = handle->device->code_memory_size;
	if (open_file(handle, (uint8_t *)buffer, &file_size)) {
		free(buffer);
		return EXIT_FAILURE;
	}
	if (read_jedec_file(buffer, file_size, jedec))
		return EXIT_FAILURE;
	if (!jedec->fuses) {
		fprintf(stderr, "This file has no fuses (L) declaration!\n");
		free(buffer);
		return EXIT_FAILURE;
	}

	if (handle->device->code_memory_size != jedec->QF)
		fprintf(stderr,
			"\nWarning! JED file doesn't match the selected device!\n");

	fprintf(stderr,
		"\nDeclared fuse checksum: 0x%04X Calculated: 0x%04X ... %s\n",
		jedec->C, jedec->fuse_checksum,
		jedec->fuse_checksum == jedec->C ? "OK" : "Mismatch!");

	fprintf(stderr,
		"Declared file checksum: 0x%04X Calculated: 0x%04X ... %s\n",
		jedec->decl_file_checksum, jedec->calc_file_checksum,
		jedec->decl_file_checksum == jedec->calc_file_checksum ?
			"OK" :
			"Mismatch!");

	fprintf(stderr, "JED file parsed OK\n\n");
	free(buffer);
	return EXIT_SUCCESS;
}

/* Helper function to retrieve a file stream pointer */
FILE *get_file(minipro_handle_t *handle)
{
	FILE *file;
	if (handle->cmdopts->is_pipe)
		file = stdout;
	else {
		file = fopen(handle->cmdopts->filename, "wb");
		if (!file) {
			fprintf(stderr, "Could not open file %s for writing.\n",
				handle->cmdopts->filename);
			perror("");
			return NULL;
		}
	}
	return file;
}

/* Wrappers for operating with files */
int write_page_file(minipro_handle_t *handle, uint8_t type, size_t size)
{
	/* Allocate the buffer and clear it with default value */
	uint8_t *file_data = malloc(size);
	if (!file_data) {
		fprintf(stderr, "Out of memory!\n");
		return EXIT_FAILURE;
	}

	memset(file_data, handle->device->blank_value, size);
	size_t file_size = size;
	if (open_file(handle, file_data, &file_size))
		return EXIT_FAILURE;
	if (file_size != size) {
		if (!handle->cmdopts->size_error) {
			fprintf(stderr,
				"Incorrect file size: %zu (needed %zu, use -s/S to ignore)\n",
				file_size, size);
			free(file_data);
			return EXIT_FAILURE;
		} else if (handle->cmdopts->size_nowarn == 0)
			fprintf(stderr,
				"Warning: Incorrect file size: %zu (needed %zu)\n",
				file_size, size);

		/* The size of our array must be a multiple of
		 * handle->device->read_buffer_size, otherwise read_page_ram
		 * will try to access an out of bounds index. */
		const uint16_t buffer_size = handle->device->read_buffer_size;
		size = MIN(file_size, size);
		const uint16_t size_mod = size % buffer_size;
		if (size_mod)
			size += buffer_size - size_mod;
	}

	/* The XGPro host unprotects the chip BEFORE erasing it (its
	 * "Unprotect before programming" option, on by default), in a
	 * transaction of its own. The T76 firmware needs this ordering for
	 * protected parallel-NOR parts: unprotecting after the erase (the
	 * generic path below) is too late and the program silently does
	 * nothing. Mirror the vendor for the T76 whenever the chip carries
	 * the off_protect_before flag, regardless of -u. Unprotecting an
	 * already-unprotected chip is a no-op, so this is safe across T76
	 * chip classes. */
	if (handle->version == MP_T76 &&
	    handle->device->flags.off_protect_before) {
		if (minipro_protect_off(handle)) {
			free(file_data);
			return EXIT_FAILURE;
		}
		fprintf(stderr, "Protect off...OK\n");
		if (minipro_end_transaction(handle) ||
		    begin_transaction(handle)) {
			free(file_data);
			return EXIT_FAILURE;
		}
	}

	/* Perform an erase first */
	if (erase_device(handle)) {
		free(file_data);
		return EXIT_FAILURE;
	}
	/* We must reset the transaction after the erase */
	if (minipro_end_transaction(handle)) {
		free(file_data);
		return EXIT_FAILURE;
	}
	if (begin_transaction(handle)) {
		free(file_data);
		return EXIT_FAILURE;
	}

	if (handle->cmdopts->protect_off &&
	    handle->device->flags.off_protect_before &&
	    handle->version != MP_T76) {
		if (minipro_protect_off(handle)) {
			free(file_data);
			return EXIT_FAILURE;
		}
		fprintf(stderr, "Protect off...OK\n");
	}

	if (write_page_ram(handle, file_data, type, size)) {
		free(file_data);
		return EXIT_FAILURE;
	}

	/* Verify if data was written ok */
	if (handle->cmdopts->no_verify == 0) {
		/* We must reset the transaction for VCC verify to have effect */
		if (minipro_end_transaction(handle)) {
			free(file_data);
			return EXIT_FAILURE;
		}
		if (begin_transaction(handle)) {
			free(file_data);
			return EXIT_FAILURE;
		}

		/* There is an off by one bug in T56 firmware.
		 * Allocate couple extra bytes to prevent buffer overflow.
		 * We need only one byte but make it 16, we never know.
		 */
		uint8_t *chip_data = malloc(size + 16);
		if (!chip_data) {
			fprintf(stderr, "Out of memory\n");
			free(file_data);
			return EXIT_FAILURE;
		}
		if (read_page_ram(handle, chip_data, type, size)) {
			free(file_data);
			free(chip_data);
			return EXIT_FAILURE;
		}

		int ret;
		uint8_t c1 = 0, c2 = 0;
		uint16_t cw1 = 0, cw2 = 0;
		uint32_t address;
		uint16_t compare_mask =
			(type == MP_CODE) ? handle->device->compare_mask : 0xff;
		if (compare_mask > 0xff) {
			ret = compare_word_memory(0xffff, compare_mask, 1,
						  file_data, chip_data,
						  file_size, size, &address,
						  &cw1, &cw2);
		} else {
			ret = compare_memory(compare_mask, file_data, chip_data,
					     file_size, size, &address, &c1,
					     &c2);
		}

		free(chip_data);
		free(file_data);

		if (ret) {
			if (compare_mask > 0xff) {
				fprintf(stderr,
					"Verification failed at address 0x%04X: File=0x%04X, Device=0x%04X\n",
					address, cw1, cw2);
			} else {
				fprintf(stderr,
					"Verification failed at address 0x%04X: File=0x%02X, Device=0x%02X\n",
					address, c1, c2);
			}
			return EXIT_FAILURE;
		} else {
			fprintf(stderr, "Verification OK\n");
		}
	}
	return EXIT_SUCCESS;
}

int read_page_file(minipro_handle_t *handle, uint8_t type, size_t size)
{
	FILE *file = get_file(handle);
	if (!file)
		return EXIT_FAILURE;

	/* There is an off by one bug in T56 firmware.
	 * Allocate couple extra bytes to prevent buffer overflow.
	 * We need only one byte but make it 16, we never know.
	 */
	uint8_t *buffer = malloc(size + 16);
	if (!buffer) {
		fprintf(stderr, "Out of memory\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	memset(buffer, handle->device->blank_value, size);
	if (read_page_ram(handle, buffer, type, size)) {
		fclose(file);
		free(buffer);
		return EXIT_FAILURE;
	}

	switch (handle->cmdopts->format) {
	case IHEX:
		if (write_hex_file(file, buffer, 0, size, 1)) {
			fclose(file);
			free(buffer);
			return EXIT_FAILURE;
		}
		break;
	case SREC:
		if (write_srec_file(file, buffer, 0, size, 1)) {
			fclose(file);
			free(buffer);
			return EXIT_FAILURE;
		}
		break;
	default:
		fwrite(buffer, 1, size, file);
	}

	fclose(file);
	free(buffer);
	return EXIT_SUCCESS;
}

int verify_page_file(minipro_handle_t *handle, uint8_t type, size_t size)
{
	uint8_t *file_data;

	char *name;
	switch (type) {
	case MP_DATA:
		name = "Data";
		break;
	case MP_USER:
		name = "User";
		break;
	default:
		name = "Code";
	}
	size_t file_size = size;

	/* Allocate the buffer and clear it with default value */
	file_data = malloc(size);
	if (!file_data) {
		fprintf(stderr, "Out of memory!\n");
		return EXIT_FAILURE;
	}
	if (handle->cmdopts->filename) {
		memset(file_data, handle->device->blank_value, size);
		if (open_file(handle, file_data, &file_size)) {
			free(file_data);
			return EXIT_FAILURE;
		}

		if (file_size != size) {
			if (!handle->cmdopts->size_error) {
				fprintf(stderr,
					"Incorrect file size: %zu (needed %zu, use -s/S to ignore)\n",
					file_size, size);
				free(file_data);
				return EXIT_FAILURE;
			} else if (handle->cmdopts->size_nowarn == 0)
				fprintf(stderr,
					"Warning: Incorrect file size: %zu (needed %zu)\n",
					file_size, size);
		}

	}
	/* Blank check */
	else
		memset(file_data, handle->device->blank_value, size);

	/* Downloading data from chip*/
	uint8_t *chip_data = malloc(size + 128);
	if (!chip_data) {
		fprintf(stderr, "Out of memory!\n");
		free(file_data);
		return EXIT_FAILURE;
	}
	if (read_page_ram(handle, chip_data, type, size)) {
		free(file_data);
		free(chip_data);
		return EXIT_FAILURE;
	}

	int ret;
	uint8_t c1 = 0, c2 = 0;
	uint16_t cw1 = 0, cw2 = 0;
	uint32_t address;
	uint16_t compare_mask =
		(type == MP_CODE) ? handle->device->compare_mask : 0xff;
	if (compare_mask > 0xff) {
		ret = compare_word_memory(0xffff, compare_mask, 1, file_data,
					  chip_data, file_size, size, &address,
					  &cw1, &cw2);
	} else {
		ret = compare_memory(compare_mask, file_data, chip_data,
				     file_size, size, &address, &c1, &c2);
	}

	free(file_data);
	free(chip_data);

	if (ret) {
		if (compare_mask > 0xff) {
			fprintf(stderr,
				"Verification failed at address 0x%04X: File=0x%04X, Device=0x%04X\n",
				address, cw1, cw2);
		} else {
			fprintf(stderr,
				"Verification failed at address 0x%04X: File=0x%02X, Device=0x%02X\n",
				address, c1, c2);
		}
		return EXIT_FAILURE;
	} else {
		if (handle->cmdopts->filename) {
			fprintf(stderr, "Verification OK\n");
		} else {
			fprintf(stderr, "%s memory section is blank.\n", name);
		}
	}
	return EXIT_SUCCESS;
}

int read_fuses(minipro_handle_t *handle, fuse_decl_t *fuses)
{
	size_t i;
	uint8_t buffer[64] = { 0 };
	uint16_t value;

	FILE *file = get_file(handle);
	if (!file)
		return EXIT_FAILURE;

	/* Read calibration byte(s) if requested */
	if (handle->cmdopts->page == CALIBRATION) {
		if (minipro_read_calibration(handle, buffer,
					     fuses->num_calibytes)) {
			fclose(file);
			return EXIT_FAILURE;
		}

		fprintf(file, "calibration bytes: ");
		for (i = 0; i < fuses->num_calibytes; i++) {
			fprintf(file, "0x%02x%s", buffer[i],
				i < fuses->num_calibytes - 1 ? ", " : "");
		}
		fprintf(file, "\n");
		fclose(file);
		fprintf(stderr, "Reading calibration bytes... OK\n");
		return EXIT_SUCCESS;
	}

	cmdopts_t *cmdopts = handle->cmdopts;
	uint8_t filter = cmdopts->filter_fuses + cmdopts->filter_locks +
			 cmdopts->filter_uid;

	if (cmdopts->filter_fuses && !fuses->num_fuses) {
		fprintf(stderr, "No fuse section to read!\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	if (cmdopts->filter_uid && !fuses->num_uids) {
		fprintf(stderr, "No user id section to read!\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	if (cmdopts->filter_locks &&
	    (handle->device->flags.lock_bit_write_only || !fuses->num_locks)) {
		fprintf(stderr, "Can't read the lock byte for this device!\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	/* Initialize progress reporting */
	progress_status("Reading config... ", -1, 0);

	/* Read fuses section if requested */
	if (fuses->num_fuses && (!filter || cmdopts->filter_fuses)) {
		uint8_t items = fuses->num_fuses;

		if (minipro_read_fuses(handle, MP_FUSE_CFG,
				       fuses->num_fuses *
					       handle->device->flags.word_size,
				       items, buffer)) {
			fclose(file);
			return EXIT_FAILURE;
		}
		for (i = 0; i < fuses->num_fuses; i++) {
			value = load_int(
				&(buffer[i * handle->device->flags.word_size]),
				handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);

			/* Mask unused bits before compare */
			value |= ~(fuses->fuse[i].mask);
			if (handle->device->compare_mask > 0xff)
				value &= handle->device->compare_mask;
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			fprintf(file,
				handle->device->flags.word_size == 1 ?
					"%s=0x%02x\n" :
					"%s=0x%04x\n",
				fuses->fuse[i].name, value);
		}
	}

	/* Read user id section if requested */
	if (fuses->num_uids && (!filter || cmdopts->filter_uid)) {
		uint8_t item_size = handle->device->flags.data_org ? 2 : 1;
		if (minipro_read_fuses(handle, MP_FUSE_USER,
				       fuses->num_uids * item_size, 0,
				       buffer)) {
			fclose(file);
			return EXIT_FAILURE;
		}
		for (i = 0; i < fuses->num_uids; i++) {
			value = load_int(&(buffer[i * item_size]), item_size,
					 MP_LITTLE_ENDIAN);
			value &= (handle->device->compare_mask);
			fprintf(file,
				item_size == 1 ? "%s=0x%02x\n" : "%s=0x%04x\n",
				user_id[i], value);
		}
	}

	/* Read lock section if requested */
	if (fuses->num_locks && (!filter || cmdopts->filter_locks)) {
		if (minipro_read_fuses(
			    handle, MP_FUSE_LOCK,
			    fuses->num_locks * handle->device->flags.word_size,
			    handle->device->flags.word_size, buffer)) {
			fclose(file);
			return EXIT_FAILURE;
		}
		for (i = 0; i < fuses->num_locks; i++) {
			value = load_int(
				&(buffer[i * handle->device->flags.word_size]),
				handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
			value |= ~(fuses->lock[i].mask);
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			fprintf(file,
				handle->device->flags.word_size == 1 ?
					"%s=0x%02x\n" :
					"%s=0x%04x\n",
				fuses->lock[i].name, value);
		}
	}

	/* Stop progress and print elapsed time */
	progress_status(NULL, 0, 1);
	fclose(file);
	return EXIT_SUCCESS;
}

int write_fuses(minipro_handle_t *handle, fuse_decl_t *fuses)
{
	size_t i;
	uint8_t wbuffer[64], vbuffer[64];
	uint16_t value;
	char config[1024];

	memset(config, 0, sizeof(config));
	size_t file_size = sizeof(config);
	if (open_file(handle, (uint8_t *)config, &file_size))
		return EXIT_FAILURE;

	/* Perform an erase first if requested */
	if (handle->cmdopts->force_erase) {
		if (erase_device(handle)) {
			return EXIT_FAILURE;
		}
		/* We must reset the transaction after the erase */
		if (minipro_end_transaction(handle)) {
			return EXIT_FAILURE;
		}
		if (begin_transaction(handle)) {
			return EXIT_FAILURE;
		}
	}

	cmdopts_t *section = handle->cmdopts;
	uint8_t filter = section->filter_fuses + section->filter_locks +
			 section->filter_uid;
	if (section->filter_fuses && !fuses->num_fuses) {
		fprintf(stderr, "No fuse section to write!!\n");
		return EXIT_FAILURE;
	}

	if (section->filter_uid && !fuses->num_uids) {
		fprintf(stderr, "No user id section to write!\n");
		return EXIT_FAILURE;
	}

	if (section->filter_locks && !fuses->num_locks) {
		fprintf(stderr, "No lock section to write!\n");
		return EXIT_FAILURE;
	}

	/* Initialize progress reporting */
	progress_status("Writing fuses... ", -1, 0);

	/* Write Fuses section if requested */
	if (fuses->num_fuses && (!filter || section->filter_fuses)) {
		uint8_t items = fuses->num_fuses;
		for (i = 0; i < fuses->num_fuses; i++) {
			if (get_fuse_value(config, sizeof(config),
					   fuses->fuse[i].name,
					   &value) == EXIT_FAILURE) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					fuses->lock[i].name);
				return EXIT_FAILURE;
			}

			/* Mask unused bits before compare */
			value |= ~(fuses->fuse[i].mask);
			if (handle->device->compare_mask > 0xff)
				value &= handle->device->compare_mask;
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(wbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}
		if (minipro_write_fuses(handle, MP_FUSE_CFG,
					fuses->num_fuses *
						handle->device->flags.word_size,
					items, wbuffer))
			return EXIT_FAILURE;
		if (minipro_read_fuses(handle, MP_FUSE_CFG,
				       fuses->num_fuses *
					       handle->device->flags.word_size,
				       items, vbuffer))
			return EXIT_FAILURE;

		/* Mask vbuffer for compare */
		for (i = 0; i < fuses->num_fuses; i++) {
			value = load_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);

			/* Mask unused bits before compare */
			value |= ~(fuses->fuse[i].mask);
			if (handle->device->compare_mask > 0xff)
				value &= handle->device->compare_mask;
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}

		if (memcmp(wbuffer, vbuffer,
			   fuses->num_fuses *
				   handle->device->flags.word_size)) {
			fprintf(stderr, "\nFuses verify error!\n");
		}

		/* Stop progress and print elapsed time */
		progress_status(NULL, 0, 1);
	}

	/* Initialize progress reporting */
	progress_status("Writing user id... ", -1, 0);

	/* Write user id section if requested */
	if (fuses->num_uids && (!filter || section->filter_uid)) {
		uint8_t item_size = handle->device->flags.data_org ? 2 : 1;
		for (i = 0; i < fuses->num_uids; i++) {
			if (get_fuse_value(config, sizeof(config), user_id[i],
					     &value) == EXIT_FAILURE) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					user_id[i]);
				return EXIT_FAILURE;
			}
			value &= (handle->device->compare_mask);
			format_int(&(wbuffer[i * item_size]), value, item_size,
				   MP_LITTLE_ENDIAN);
		}
		if (minipro_write_fuses(handle, MP_FUSE_USER,
					fuses->num_uids * item_size, item_size,
					wbuffer))
			return EXIT_FAILURE;
		if (minipro_read_fuses(handle, MP_FUSE_USER,
				       fuses->num_uids * item_size, 0, vbuffer))
			return EXIT_FAILURE;

		/* Mask vbuffer for compare */
		for (i = 0; i < fuses->num_uids; i++) {
			value = load_int(&(vbuffer[i * item_size]), item_size,
					 MP_LITTLE_ENDIAN);
			value &= (handle->device->compare_mask);
			format_int(&(vbuffer[i * item_size]), value, item_size,
				   MP_LITTLE_ENDIAN);
		}

		if (memcmp(wbuffer, vbuffer, fuses->num_uids * item_size)) {
			fprintf(stderr, "\nUser ID verify error!\n");
		}

		/* Stop progress and print elapsed time */
		progress_status(NULL, 0, 1);
	}

	/* Initialize progress reporting */
	progress_status("Writing lock bits... ", -1, 0);

	/* Write lock section if requested */
	if (fuses->num_locks && (!filter || section->filter_locks)) {
		for (i = 0; i < fuses->num_locks; i++) {
			if (get_fuse_value(config, sizeof(config), fuses->lock[i].name,
					     &value) == EXIT_FAILURE) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					fuses->lock[i].name);
				return EXIT_FAILURE;
			}
			value |= ~(fuses->lock[i].mask);
			format_int(
				&(wbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}
		if (minipro_write_fuses(
			    handle, MP_FUSE_LOCK,
			    fuses->num_locks * handle->device->flags.word_size,
			    handle->device->flags.word_size, wbuffer))
			return EXIT_FAILURE;
		if (!handle->device->flags.lock_bit_write_only) {
			if (minipro_read_fuses(
				    handle, MP_FUSE_LOCK,
				    fuses->num_locks *
					    handle->device->flags.word_size,
				    handle->device->flags.word_size, vbuffer))
				return EXIT_FAILURE;

			/* Mask vbuffer for compare */
			for (i = 0; i < fuses->num_locks; i++) {
				value = load_int(
					&(vbuffer[i * handle->device->flags
							      .word_size]),
					handle->device->flags.word_size,
					MP_LITTLE_ENDIAN);
				value |= ~(fuses->lock[i].mask);
				if (handle->device->flags.word_size == 1)
					value &= 0xff;
				format_int(&(vbuffer[i * handle->device->flags
								 .word_size]),
					   value,
					   handle->device->flags.word_size,
					   MP_LITTLE_ENDIAN);
			}

			if (memcmp(wbuffer, vbuffer,
				   fuses->num_locks *
					   handle->device->flags.word_size)) {
				fprintf(stderr, "\nLock bits verify error!\n");
			}
		}

		/* Stop progress and print elapsed time */
		progress_status(NULL, 0, 1);
	}
	return EXIT_SUCCESS;
}

int verify_fuses(minipro_handle_t *handle, fuse_decl_t *fuses)
{
	uint8_t wbuffer[64], vbuffer[64];
	uint16_t value;
	size_t i;
	char config[1024];
	int ret = EXIT_SUCCESS;

	memset(config, 0, sizeof(config));
	size_t file_size = sizeof(config);
	if (handle->cmdopts->filename &&
	    open_file(handle, (uint8_t *)config, &file_size))
		return EXIT_FAILURE;

	if (begin_transaction(handle))
		return EXIT_FAILURE;

	cmdopts_t *section = handle->cmdopts;
	uint8_t filter = section->filter_fuses + section->filter_locks +
			 section->filter_uid;

	if (section->filter_fuses && !fuses->num_fuses) {
		fprintf(stderr, "No fuse section to read!\n");
		return EXIT_FAILURE;
	}

	if (section->filter_uid && !fuses->num_uids) {
		fprintf(stderr, "No user id section to read!\n");
		return EXIT_FAILURE;
	}

	if (section->filter_locks &&
	    (handle->device->flags.lock_bit_write_only || !fuses->num_locks)) {
		fprintf(stderr, "Can't read the lock byte for this device!\n");
		return EXIT_FAILURE;
	}

	/* Verify/Blank check fuses section if requested */
	if (fuses->num_fuses && (!filter || section->filter_fuses)) {
		uint8_t items = fuses->num_fuses;
		for (i = 0; i < fuses->num_fuses; i++) {
			value = fuses->fuse[i].def;
			if (handle->cmdopts->filename &&
			    get_fuse_value(config, sizeof(config), fuses->fuse[i].name,
					     &value)) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					fuses->fuse[i].name);
				return EXIT_FAILURE;
			}

			/* Mask unused bits before compare */
			value |= ~(fuses->fuse[i].mask);
			if (handle->device->compare_mask > 0xff)
				value &= handle->device->compare_mask;
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(wbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}
		if (minipro_read_fuses(handle, MP_FUSE_CFG,
				       fuses->num_fuses *
					       handle->device->flags.word_size,
				       items, vbuffer))
			return EXIT_FAILURE;

		/* Mask vbuffer for compare */
		for (i = 0; i < fuses->num_fuses; i++) {
			value = load_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);

			/* Mask unused bits before compare */
			value |= ~(fuses->fuse[i].mask);
			if (handle->device->compare_mask > 0xff)
				value &= handle->device->compare_mask;
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}

		if (memcmp(wbuffer, vbuffer,
			   fuses->num_fuses *
				   handle->device->flags.word_size)) {
			fprintf(stderr,
				handle->cmdopts->filename ?
					"Fuse bits verification error!\n" :
					"Fuse bits aren't in their default value!\n");

			ret = EXIT_FAILURE;
		} else
			fprintf(stderr,
				handle->cmdopts->filename ?
					"Fuse bits verification OK.\n" :
					"Fuse bits are in their default value.\n");
	}

	/* Verify/Blank check user id section if requested */
	if (fuses->num_uids && (!filter || section->filter_uid)) {
		uint8_t item_size = handle->device->flags.data_org ? 2 : 1;
		for (i = 0; i < fuses->num_uids; i++) {
			value = handle->device->compare_mask;
			if (handle->cmdopts->filename &&
			    get_fuse_value(config, sizeof(config), user_id[i], &value)) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					user_id[i]);
				return EXIT_FAILURE;
			}
			value &= (handle->device->compare_mask);
			format_int(&(wbuffer[i * item_size]), value, item_size,
				   MP_LITTLE_ENDIAN);
		}
		if (minipro_read_fuses(handle, MP_FUSE_USER,
				       fuses->num_uids * item_size, 0, vbuffer))
			return EXIT_FAILURE;

		/* Mask buffer for compare */
		for (i = 0; i < fuses->num_uids; i++) {
			value = load_int(&(vbuffer[i * item_size]), item_size,
					 MP_LITTLE_ENDIAN);
			value &= (handle->device->compare_mask);
			format_int(&(vbuffer[i * item_size]), value, item_size,
				   MP_LITTLE_ENDIAN);
		}

		if (memcmp(wbuffer, vbuffer, fuses->num_uids * item_size)) {
			fprintf(stderr,
				handle->cmdopts->filename ?
					"User ID verification error!\n" :
					"User ID section is not blank!\n");

			ret = EXIT_FAILURE;
		} else
			fprintf(stderr, handle->cmdopts->filename ?
						"User ID verification OK.\n" :
						"User ID section is blank.\n");
	}

	/* Verify/Blank check lock section if requested */
	if (fuses->num_locks && (!filter || section->filter_locks)) {
		for (i = 0; i < fuses->num_locks; i++) {
			value = fuses->lock[i].def;
			if (handle->cmdopts->filename &&
			    get_fuse_value(config, sizeof(config), fuses->lock[i].name,
					     &value)) {
				fprintf(stderr,
					"Could not read config %s value.\n",
					fuses->lock[i].name);
				return EXIT_FAILURE;
			}
			value |= ~(fuses->lock[i].mask);
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(wbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}
		if (minipro_read_fuses(
			    handle, MP_FUSE_LOCK,
			    fuses->num_locks * handle->device->flags.word_size,
			    handle->device->flags.word_size, vbuffer))
			return EXIT_FAILURE;

		/* Mask vbuffer for compare */
		for (i = 0; i < fuses->num_locks; i++) {
			value = load_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
			value |= ~(fuses->lock[i].mask);
			if (handle->device->flags.word_size == 1)
				value &= 0xff;
			format_int(
				&(vbuffer[i * handle->device->flags.word_size]),
				value, handle->device->flags.word_size,
				MP_LITTLE_ENDIAN);
		}

		if (memcmp(wbuffer, vbuffer,
			   fuses->num_locks *
				   handle->device->flags.word_size)) {
			fprintf(stderr,
				handle->cmdopts->filename ?
					"Lock bits verification error!\n" :
					"Lock bits aren't in their default value!\n");
			ret = EXIT_FAILURE;
		} else
			fprintf(stderr,
				handle->cmdopts->filename ?
					"Lock bits verification OK.\n" :
					"Lock bits are in their default value.\n");
	}
	return ret;
}

/* Code-memory size in bytes for the current device. For a T76 eMMC this is the
 * real capacity read from EXT_CSD SEC_COUNT during begin_transaction
 * (handle->emmc_capacity, 64-bit); minipro's grouped infoic.xml carries only a
 * placeholder code_memory_size for eMMC. Falls back to code_memory_size when the
 * EXT_CSD read hasn't run or returned 0. */
static size_t t76_code_size(minipro_handle_t *handle)
{
	if (handle->version == MP_T76 &&
	    handle->device->protocol_id == IC2_ALG_EMMC && handle->emmc_capacity)
		return (size_t)handle->emmc_capacity;
	return handle->device->code_memory_size;
}

/* Higher-level logic */
int action_read(minipro_handle_t *handle)
{
	jedec_t jedec;
	int ret = EXIT_SUCCESS;

	if (begin_transaction(handle))
		return EXIT_FAILURE;

	if (handle->device->chip_type == MP_PLD) {
		jedec.QF = handle->device->code_memory_size;
		if (!jedec.QF) {
			fprintf(stderr, "Unknown fuse size!\n");
			return EXIT_FAILURE;
		}

		jedec.fuses = malloc(jedec.QF);
		if (!jedec.fuses) {
			fprintf(stderr, "Out of memory\n");
			return EXIT_FAILURE;
		}
		memset(jedec.fuses, 0, jedec.QF);

		jedec.F = 0;
		jedec.G = 0;
		jedec.QP = handle->device->package_details.pin_count;
		jedec.device_name = handle->device->name;

		if (read_jedec(handle, &jedec)) {
			ret = EXIT_FAILURE;
		} else {
			FILE *file = get_file(handle);
			if (!file) {
				ret = EXIT_FAILURE;
			} else {
				if (write_jedec_file(file, &jedec))
					ret = EXIT_FAILURE;
				fclose(file);
			}
		}

		free(jedec.fuses);
		return ret;
	}

	/* No PLD */
	char *base = handle->cmdopts->filename;
	char *data_filename = base;
	char *user_filename = base;
	char *config_filename = base;
	char *default_data_filename = NULL;
	char *default_user_filename = NULL;
	char *default_config_filename = NULL;

	if (!handle->cmdopts->is_pipe) {
		size_t base_len = strlen(base);
		default_data_filename = malloc(base_len + 32);
		default_user_filename = malloc(base_len + 32);
		default_config_filename = malloc(base_len + 32);
		if (!default_data_filename || !default_user_filename ||
		    !default_config_filename) {
			ret = EXIT_FAILURE;
			goto end;
		}

		strcpy(default_data_filename, base);
		strcpy(default_user_filename, base);
		strcpy(default_config_filename, base);

		char *data_ext = NULL, *user_ext = NULL;
		switch (handle->cmdopts->format) {
		case IHEX:
			data_ext = ".eeprom.hex";
			user_ext = ".user.hex";
			break;
		case SREC:
			data_ext = ".eeprom.srec";
			user_ext = ".user.srec";
			break;
		default:
			data_ext = ".eeprom.bin";
			user_ext = ".user.bin";
		}
		strcat(default_data_filename, data_ext);
		strcat(default_user_filename, user_ext);
		strcat(default_config_filename, ".fuses.conf");
	}

	if (handle->cmdopts->page == UNSPECIFIED) {
		data_filename = default_data_filename;
		user_filename = default_user_filename;
		config_filename = default_config_filename;
	}

	if (handle->cmdopts->page == CODE ||
	    handle->cmdopts->page == UNSPECIFIED) {
		if (read_page_file(handle, MP_CODE,
				   t76_code_size(handle))) {
			ret = EXIT_FAILURE;
			goto end;
		}
	}

	if ((handle->cmdopts->page == DATA ||
	     (handle->cmdopts->page == UNSPECIFIED &&
	      !handle->cmdopts->is_pipe)) &&
	    handle->device->data_memory_size) {
/* T56 needs to restart transaction when reading code and data memory in sequence */
    if(handle->version == MP_T56 && 
      handle->cmdopts->page == UNSPECIFIED){
    if (minipro_end_transaction(handle)) {
      minipro_close(handle);
      return EXIT_FAILURE;
    }	
    if (begin_transaction(handle))
      return EXIT_FAILURE;
    }
		handle->cmdopts->filename = data_filename;
		if (read_page_file(handle, MP_DATA,
				   handle->device->data_memory_size)) {
			ret = EXIT_FAILURE;
			goto end;
		}
	}

	if ((handle->cmdopts->page == USER ||
	     (handle->cmdopts->page == UNSPECIFIED &&
	      !handle->cmdopts->is_pipe)) &&
	    handle->device->data_memory2_size) {
		handle->cmdopts->filename = user_filename;
		if (read_page_file(handle, MP_USER,
				   handle->device->data_memory2_size)) {
			ret = EXIT_FAILURE;
			goto end;
		}
	}

	if ((handle->cmdopts->page == CONFIG ||
	     (handle->cmdopts->page == CALIBRATION &&
	      handle->device->flags.has_calibration) ||
	     (handle->cmdopts->page == UNSPECIFIED &&
	      !handle->cmdopts->is_pipe)) &&
	    handle->device->config) {
		handle->cmdopts->filename = config_filename;
		if (read_fuses(handle, handle->device->config)) {
			ret = EXIT_FAILURE;
			goto end;
		}
	}

	if (handle->cmdopts->page == DATA &&
	    !handle->device->data_memory_size) {
		fprintf(stderr, "No data section found.\n");
		ret = EXIT_FAILURE;
	}

	if (handle->cmdopts->page == USER &&
	    !handle->device->data_memory2_size) {
		fprintf(stderr, "No user section found.\n");
		ret = EXIT_FAILURE;
	}

	if (handle->cmdopts->page == CONFIG && !handle->device->config) {
		fprintf(stderr, "No config section found.\n");
		ret = EXIT_FAILURE;
	}

	if (handle->cmdopts->page == CALIBRATION &&
	    !handle->device->flags.has_calibration) {
		fprintf(stderr,
			"This chip doesn't have any calibration bytes.\n");
		ret = EXIT_FAILURE;
	}

end:
	free(default_data_filename);
	free(default_user_filename);
	free(default_config_filename);
	return ret;
}

int action_write(minipro_handle_t *handle)
{
	jedec_t wjedec, rjedec;
	int ret = EXIT_SUCCESS;
	uint8_t c1, c2;
	uint32_t address;

	if (handle->device->chip_type == MP_PLD) {

		/* Open jedec file */
		if (open_jed_file(handle, &wjedec))
			return EXIT_FAILURE;

		/* Begin transaction */
		if (begin_transaction(handle)) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}

		/* Erase device */
		if (erase_device(handle)) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}

		/* Write jedec file */
		if (write_jedec(handle, &wjedec)) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}

		/* End transaction */
		if (minipro_end_transaction(handle)) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}

		/* Verify */
		if (handle->cmdopts->no_verify == 0) {
			rjedec.QF = handle->device->code_memory_size;
			rjedec.F = wjedec.F;
			rjedec.fuses = malloc(rjedec.QF);
			if (!rjedec.fuses) {
				free(wjedec.fuses);
				return EXIT_FAILURE;
			}
			/* compare fuses */
			if (begin_transaction(handle)) {
				free(wjedec.fuses);
				free(rjedec.fuses);
				return EXIT_FAILURE;
			}
			if (read_jedec(handle, &rjedec)) {
				free(wjedec.fuses);
				free(rjedec.fuses);
				return EXIT_FAILURE;
			}
			if (minipro_end_transaction(handle)) {
				free(wjedec.fuses);
				free(rjedec.fuses);
				return EXIT_FAILURE;
			}
			ret = compare_memory(0x01, wjedec.fuses, rjedec.fuses,
					     wjedec.QF, rjedec.QF, &address,
					     &c1, &c2);

			/* The error output is delayed until the security
			 * fuse has been written to avoid a 99% correctly
			 * programmed chip without the security fuse. */

			free(rjedec.fuses);
		}
		free(wjedec.fuses);

		if (handle->cmdopts->protect_on) {

			/* Initialize progress reporting */
			progress_status("Writing lock bit... ", -1, 0);

			if (begin_transaction(handle))
				return EXIT_FAILURE;
			if (minipro_write_fuses(handle, MP_FUSE_LOCK, 0, 0,
						NULL))
				return EXIT_FAILURE;
			if (minipro_end_transaction(handle))
				return EXIT_FAILURE;

			/* Stop progress and print elapsed time */
			progress_status(NULL, 0, 1);
		}

		/* handle error from verify */
		if (ret) {
			fprintf(stderr,
				"Verification failed at address 0x%04X: File=0x%02X, "
				"Device=0x%02X\n",
				address, c1, c2);
			return EXIT_FAILURE;
		} else {
			fprintf(stderr, "Verification OK\n");
		}

		return EXIT_SUCCESS;

		/* No GAL devices */
	} else {

		/* Begin transaction */
		if (begin_transaction(handle))
			return EXIT_FAILURE;

		/* Write specified page(s) */
		switch (handle->cmdopts->page) {
		case UNSPECIFIED:
		case CODE:
			if (write_page_file(handle, MP_CODE,
					    t76_code_size(handle)))
				return EXIT_FAILURE;
			break;
		case DATA:
			if (handle->cmdopts->page == DATA &&
			    !handle->device->data_memory_size) {
				fprintf(stderr, "No data section found.\n");
				return EXIT_FAILURE;
			}
			if (write_page_file(handle, MP_DATA,
					    handle->device->data_memory_size))
				return EXIT_FAILURE;
			break;
		case USER:
			if (handle->cmdopts->page == USER &&
			    !handle->device->data_memory2_size) {
				fprintf(stderr, "No user section found.\n");
				return EXIT_FAILURE;
			}
			if (write_page_file(handle, MP_USER,
					    handle->device->data_memory2_size))
				return EXIT_FAILURE;
			break;
		case CONFIG:
			if (handle->cmdopts->page == CONFIG &&
			    !handle->device->config) {
				fprintf(stderr, "No config section found.\n");
				return EXIT_FAILURE;
			}
			if (handle->device->config) {
				if (write_fuses(handle, handle->device->config))
					return EXIT_FAILURE;
			}
			break;
		case CALIBRATION:
			fprintf(stderr, "Calibration bytes are read only.\n");
			return EXIT_FAILURE;
		}
		if (handle->cmdopts->protect_on &&
		    handle->device->flags.protect_after) {
			fprintf(stderr, "Protect on...");
			fflush(stderr);
			if (minipro_protect_on(handle))
				return EXIT_FAILURE;
			fprintf(stderr, "OK\n");
		}
	}
	return EXIT_SUCCESS;
}

int action_verify(minipro_handle_t *handle)
{
	jedec_t wjedec, rjedec;
	int ret = EXIT_SUCCESS;

	if (handle->device->chip_type == MP_PLD) {
		if (handle->cmdopts->filename) {
			if (open_jed_file(handle, &wjedec))
				return EXIT_FAILURE;
		}
		/* Blank check */
		else {
			wjedec.QF = handle->device->code_memory_size;
			wjedec.F = 0x01;
			wjedec.fuses = malloc(wjedec.QF);
			memset(wjedec.fuses, 0x01, wjedec.QF);
		}

		if (begin_transaction(handle)) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}

		rjedec.QF = handle->device->code_memory_size;
		rjedec.F = wjedec.F;
		rjedec.fuses = malloc(rjedec.QF);
		if (!rjedec.fuses) {
			free(wjedec.fuses);
			return EXIT_FAILURE;
		}
		/* compare fuses */
		if (begin_transaction(handle)) {
			free(wjedec.fuses);
			free(rjedec.fuses);
			return EXIT_FAILURE;
		}
		if (read_jedec(handle, &rjedec)) {
			free(wjedec.fuses);
			free(rjedec.fuses);
			return EXIT_FAILURE;
		}
		if (minipro_end_transaction(handle)) {
			free(wjedec.fuses);
			free(rjedec.fuses);
			return EXIT_FAILURE;
		}
		uint8_t c1, c2;
		uint32_t address;

		if (compare_memory(0x01, wjedec.fuses, rjedec.fuses, wjedec.QF,
				   rjedec.QF, &address, &c1, &c2)) {
			if (handle->cmdopts->filename) {
				fprintf(stderr,
					"Verification failed at address 0x%04X: File=0x%02X, "
					"Device=0x%02X\n",
					address, c1, c2);
			} else {
				fprintf(stderr, "This device is not blank.\n");
			}
			free(rjedec.fuses);
			return EXIT_FAILURE;
		} else {
			if (handle->cmdopts->filename) {
				fprintf(stderr, "Verification OK\n");
			} else {
				fprintf(stderr, "This device is blank.\n");
			}
		}
		free(rjedec.fuses);
		free(wjedec.fuses);
	} else {
		/* No GAL devices */

		/* Verifying code memory section.
		 * If filename is null then a blank check is performed */
		if (handle->cmdopts->page == UNSPECIFIED ||
		    handle->cmdopts->page == CODE) {
			if (begin_transaction(handle))
				return EXIT_FAILURE;
			if (verify_page_file(handle, MP_CODE,
					     t76_code_size(handle)))
				ret = EXIT_FAILURE;
		}

		if (!handle->device->data_memory_size &&
		    handle->cmdopts->page == DATA) {
			fprintf(stderr, "No data section found.\n");
			return EXIT_FAILURE;
		}

		if (!handle->device->data_memory2_size &&
		    handle->cmdopts->page == USER) {
			fprintf(stderr, "No user section found.\n");
			return EXIT_FAILURE;
		}

		if (!handle->device->config &&
		    handle->cmdopts->page == CONFIG) {
			fprintf(stderr, "No config section found.\n");
			return EXIT_FAILURE;
		}

		/* Verifying data memory section.
		 * If filename is null then a blank check is performed */
		if (handle->device->data_memory_size &&
		    (handle->cmdopts->page == DATA ||
		     (handle->cmdopts->page == UNSPECIFIED &&
		      !handle->cmdopts->filename))) {
			if (begin_transaction(handle))
				return EXIT_FAILURE;
			if (verify_page_file(handle, MP_DATA,
					     handle->device->data_memory_size))
				ret = EXIT_FAILURE;
		}

		/* Verifying user memory section.
		 * If filename is null then a blank check is performed. */
		if (handle->device->data_memory2_size &&
		    (handle->cmdopts->page == USER ||
		     (handle->cmdopts->page == UNSPECIFIED &&
		      !handle->cmdopts->filename))) {
			if (begin_transaction(handle))
				return EXIT_FAILURE;
			if (verify_page_file(handle, MP_USER,
					     handle->device->data_memory2_size))
				ret = EXIT_FAILURE;
		}

		/* Verifying configuration bytes. */
		if (handle->device->config &&
		    (handle->cmdopts->page == CONFIG ||
		     (handle->cmdopts->page == UNSPECIFIED &&
		      !handle->cmdopts->filename))) {
			ret = verify_fuses(handle, handle->device->config);
		}
	}
	return ret;
}

/* Program Main entry point */
int main(int argc, char **argv)
{
/* If we are in windows start the VT100 support.
*  Set the Windows translation mode to binary.
*/
#ifdef _WIN32
	system(" ");
	setmode(STDOUT_FILENO, O_BINARY);
	setmode(STDIN_FILENO, O_BINARY);
#endif

	cmdopts_t cmdopts;

	/* Parse the command line first */
	parse_cmdline(argc, argv, &cmdopts);

	/* Get a minipro handle */
	minipro_handle_t *handle = minipro_open(VERBOSE);
	if (!handle)
		return EXIT_FAILURE;
	handle->cmdopts = &cmdopts;

	/* Exit if bootloader is active */
	minipro_print_system_info(handle);
	if (handle->status == MP_STATUS_BOOTLOADER) {
		fprintf(stderr, "Exiting...\n");
		minipro_close(handle);
		return EXIT_FAILURE;
	}
	fprintf(stderr, "\n");

	/* Get the requested device */
	if (get_device(handle)) {
		minipro_close(handle);
		return EXIT_FAILURE;
	}
	device_t *device = handle->device;

	/* Check for unsupported devices */
	switch (device->chip_type) {
	/* case MP_NAND:  -- bypassed: T76 NAND read/erase/program implemented */
	/* case MP_EMMC:  -- bypassed: T76 eMMC read implemented (first pass) */
	case MP_VGA:
		minipro_close(handle);
		fprintf(stderr, "This chip is not supported yet.\n");
		return EXIT_FAILURE;
	}

	/* T76 eMMC: minipro's infoic.xml groups many eMMC parts under one entry
	 * with a placeholder code_memory_size (0x200) and no real per-chip
	 * capacity. The true capacity is read from the device's EXT_CSD SEC_COUNT
	 * during begin_transaction (handle->emmc_capacity, 64-bit) and used by the
	 * code-size helper t76_emmc_size(); see t76_emmc_bring_up. No static cap
	 * is applied here anymore. */

	/* Parse programming options */
	if (parse_options(handle, argc, argv)) {
		if (strlen(optarg))
			fprintf(stderr, "\nInvalid option '%s'\n",
				argv[optind - 1]);
		minipro_close(handle);
		print_help_and_exit(argv[0]);
	}

	/* Run a bad pin contact test if requested. */
	if (cmdopts.pincheck) {
		if ((handle->version == MP_TL866IIPLUS ||
		     handle->version == MP_T76) &&
		    !cmdopts.icsp) {
			if (minipro_pin_test(handle)) {
				minipro_end_transaction(handle);
				minipro_close(handle);
				return EXIT_FAILURE;
			}
		} else
			fprintf(stderr, "Pin test is not supported.\n");
		if (cmdopts.action == NO_ACTION && !cmdopts.idcheck_only)
			return EXIT_SUCCESS;
	}

	/* Perform a logic chip test and exit */
	if (cmdopts.action == LOGIC_IC_TEST) {
		int ret = minipro_logic_ic_test(handle);
		minipro_close(handle);
		return ret;
	}

	/* Handle adapters if applicable */
	if(check_adapter(handle)){
		minipro_close(handle);
	}

	/* Activate ICSP if the chip can only be programmed via ICSP. */
	if (device->flags.prog_support == MP_ICSP_ONLY) {
		handle->cmdopts->icsp = MP_ICSP_ENABLE | MP_ICSP_VCC;
	} else if (device->flags.prog_support == MP_ZIF_ONLY)
		handle->cmdopts->icsp = 0x00;
	if (handle->cmdopts->icsp)
		fprintf(stderr, "Activating ICSP...\n");
	if (cmdopts.icsp && device->flags.prog_support == MP_ZIF_ONLY)
		fprintf(stderr,
			"Warning: ICSP is not supported by this chip.\n");

	/* Verifying Chip ID (if applicable) */
	if (cmdopts.idcheck_skip) {
		fprintf(stderr, "WARNING: skipping Chip ID test\n");
	} else if (device->flags.has_chip_id) {
		if (check_chip_id(handle)) {
			minipro_close(handle);
			return EXIT_FAILURE;
		} else if (cmdopts.idcheck_only) {
			minipro_close(handle);
			return EXIT_SUCCESS;
		}
	} else {
		if (cmdopts.idcheck_only) {
			fprintf(stderr, "This chip doesn't have a chip ID!\n");
			minipro_close(handle);
			return EXIT_FAILURE;
		}
	}

	/* Print programming options */
	print_options(handle);

	/* Performing requested action */
	int ret;
	switch (cmdopts.action) {

	/* Read action */
	case READ:
		ret = action_read(handle);
		break;

	/* Write action */
	case WRITE:
		if (device->flags.prog_support == MP_READ_ONLY) {
			fprintf(stderr, "Read-only chip.\n");
			minipro_close(handle);
			return EXIT_FAILURE;
		}
		/* Print a warning about write-protection */
		if (device->flags.protect_after &&
		    !handle->cmdopts->protect_on) {
			fprintf(stderr,
				"Use -P if you want to write-protect this chip.\n");
		}
		ret = action_write(handle);
		/* Print a warning about write-protection */
		if (ret == EXIT_FAILURE && device->flags.off_protect_before &&
		    !handle->cmdopts->protect_off) {
			fprintf(stderr,
				"This chip may be write-protected. Use -u and try again.\n");
		}
		break;

	/* Verify/Blank check action */
	case VERIFY:
	case BLANK_CHECK:
		ret = action_verify(handle);
		break;

	/* Erase action */
	case ERASE:
		if (!device->flags.can_erase) {
			fprintf(stderr, "This chip can't be erased!\n");
			minipro_close(handle);
			return EXIT_FAILURE;
		}
		if (begin_transaction(handle)) {
			minipro_close(handle);
			return EXIT_FAILURE;
		}
		ret = erase_device(handle);
		break;
	default:
		ret = EXIT_FAILURE;
		break;
	}

	/* End session */
	if (minipro_end_transaction(handle)) {
		minipro_close(handle);
		return EXIT_FAILURE;
	}
	minipro_close(handle);
	return ret;
}
