/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2020 The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "programmer.h"
#include "spi.h"
#include "i2c_helper.h"

#define REGISTER_ADDRESS	(0x94 >> 1)
#define PAGE_ADDRESS		(0x9e >> 1)
#define PAGE_SIZE		256
#define READ_SIZE		32
#define MAX_SPI_WAIT_RETRIES	1000
#define FUNCTION_ERR		-1

#define CLT2_SPI		0x82
#define SPIEDID_BASE_ADDR2	0x8d
#define ROMADDR_BYTE1		0x8e
#define ROMADDR_BYTE2		0x8f
#define SWSPI_WDATA		0x90
	#define SWSPI_WDATA_CLEAR_STATUS		0x00
	#define SWSPI_WDATA_WRITE_REGISTER		0x01
	#define SWSPI_WDATA_READ_REGISTER		0x05
	#define SWSPI_WDATA_ENABLE_REGISTER		0x06
	#define SWSPI_WDATA_SECTOR_ERASE		0x20
	#define SWSPI_WDATA_PROTECT_BP			0x8c
#define SWSPI_RDATA		0x91
#define SWSPI_LEN		0x92
#define SWSPICTL		0x93
	#define SWSPICTL_ACCESS_TRIGGER			1
	#define SWSPICTL_CLEAR_PTR			(1 << 1)
	#define SWSPICTL_NO_READ			(1 << 2)
	#define SWSPICTL_ENABLE_READBACK		(1 << 3)
	#define SWSPICTL_MOT				(1 << 4)
#define SPISTATUS		0x9e
	#define SPISTATUS_BYTE_PROGRAM_FINISHED		0
	#define SPISTATUS_BYTE_PROGRAM_IN_IF		1
	#define SPISTATUS_BYTE_PROGRAM_SEND_DONE	(1 << 1)
	#define SPISTATUS_SECTOR_ERASE_FINISHED		0
	#define SPISTATUS_SECTOR_ERASE_IN_IF		(1 << 2)
	#define SPISTATUS_SECTOR_ERASE_SEND_DONE	(1 << 3)
	#define SPISTATUS_CHIP_ERASE_FINISHED		0
	#define SPISTATUS_CHIP_ERASE_IN_IF		(1 << 4)
	#define SPISTATUS_CHIP_ERASE_SEND_DONE		(1 << 5)
	#define SPISTATUS_FW_UPDATE_ENABLE		(1 << 6)
#define WRITE_PROTECTION	0xb3
	#define WRITE_PROTECTION_ON			0
	#define WRITE_PROTECTION_OFF			0x10
#define MPU			0xbc
#define PAGE_HW_WRITE		0xda
	#define PAGE_HW_WRITE_DISABLE			0
	#define PAGE_HW_COFIG_REGISTER			0xaa
	#define PAGE_HW_WRITE_ENABLE			0x55

struct lspcon_i2c_spi_data {
	int fd;
};

typedef struct {
	uint8_t command;
	uint8_t data_size;
	const uint8_t *data;
	uint8_t control;
} packet_t;

static int lspcon_i2c_spi_write_data(int fd, uint16_t addr, void *buf, uint16_t len)
{
	i2c_buffer_t data;
	if (i2c_buffer_t_fill(&data, buf, len))
		return FUNCTION_ERR;

	return i2c_write(fd, addr, &data) == len ? 0 : FUNCTION_ERR;
}

static int lspcon_i2c_spi_read_data(int fd, uint16_t addr, void *buf, uint16_t len)
{
	i2c_buffer_t data;
	if (i2c_buffer_t_fill(&data, buf, len))
		return FUNCTION_ERR;

	return i2c_read(fd, addr, &data) == len ? 0 : FUNCTION_ERR;
}

static int get_fd_from_context(struct flashctx *flash)
{
	if (!flash || !flash->mst || !flash->mst->spi.data) {
		msg_perr("Unable to extract fd from flash context.\n");
		return FUNCTION_ERR;
	}

	return ((const struct lspcon_i2c_spi_data *)flash->mst->spi.data)->fd;
}

static int lspcon_i2c_spi_write_register(int fd, uint8_t i2c_register, uint8_t value)
{
	uint8_t command[] = { i2c_register, value };
	return lspcon_i2c_spi_write_data(fd, REGISTER_ADDRESS, command, 2);
}

static int lspcon_i2c_spi_read_register(int fd, uint8_t i2c_register, uint8_t *value)
{
	uint8_t command[] = { i2c_register };
	int ret = lspcon_i2c_spi_write_data(fd, REGISTER_ADDRESS, command, 1);
	ret |= lspcon_i2c_spi_read_data(fd, REGISTER_ADDRESS, value, 1);

	return ret ? FUNCTION_ERR : 0;
}

static int lspcon_i2c_spi_register_control(int fd, packet_t *packet)
{
	int ret = lspcon_i2c_spi_write_register(fd, SWSPI_WDATA, packet->command);
	if (ret)
		return ret;

	uint8_t write_len = packet->data_size & 0x0f;
	for (int i = 0; i < write_len; ++i) {
		ret |= lspcon_i2c_spi_write_register(fd, SWSPI_WDATA, packet->data[i]);
	}

	ret |= lspcon_i2c_spi_write_register(fd, SWSPI_LEN, packet->data_size);
	ret |= lspcon_i2c_spi_write_register(fd, SWSPICTL, packet->control);

	return ret;
}

static int lspcon_i2c_spi_wait_command_done(int fd, unsigned int offset, int mask)
{
	uint8_t val;
	int tried = 0;
	int ret = 0;
	do {
		ret |= lspcon_i2c_spi_read_register(fd, offset, &val);
	} while(!ret && (val & mask) && ++tried < MAX_SPI_WAIT_RETRIES);

	if (tried == MAX_SPI_WAIT_RETRIES) {
		msg_perr("Error: Time out on sending command.\n");
		return -MAX_SPI_WAIT_RETRIES;
	}

	return (val & mask) ? FUNCTION_ERR : ret;
}

static int lspcon_i2c_spi_wait_rom_free(int fd)
{
	uint8_t val;
	int tried = 0;
	int ret = 0;
	ret |= lspcon_i2c_spi_wait_command_done(fd, SPISTATUS,
		SPISTATUS_SECTOR_ERASE_IN_IF | SPISTATUS_SECTOR_ERASE_SEND_DONE);
	if (ret)
		return ret;

	do {
		packet_t packet = { SWSPI_WDATA_READ_REGISTER, 0, NULL, SWSPICTL_ACCESS_TRIGGER };
		ret |= lspcon_i2c_spi_register_control(fd, &packet);
		ret |= lspcon_i2c_spi_wait_command_done(fd, SWSPICTL, SWSPICTL_ACCESS_TRIGGER);
		ret |= lspcon_i2c_spi_read_register(fd, SWSPI_RDATA, &val);
	} while (!ret && (val & SWSPICTL_ACCESS_TRIGGER) && ++tried < MAX_SPI_WAIT_RETRIES);

	if (tried == MAX_SPI_WAIT_RETRIES) {
		msg_perr("Error: Time out on waiting ROM free.\n");
		return -MAX_SPI_WAIT_RETRIES;
	}

	return (val & SWSPICTL_ACCESS_TRIGGER) ? FUNCTION_ERR : ret;
}

static int lspcon_i2c_spi_toggle_register_protection(int fd, int toggle)
{
	return lspcon_i2c_spi_write_register(fd, WRITE_PROTECTION,
		toggle ? WRITE_PROTECTION_OFF : WRITE_PROTECTION_ON);
}

static int lspcon_i2c_spi_enable_write_status_register(int fd)
{
	int ret = lspcon_i2c_spi_toggle_register_protection(fd, 1);
	packet_t packet = {
		SWSPI_WDATA_ENABLE_REGISTER, 0, NULL, SWSPICTL_ACCESS_TRIGGER | SWSPICTL_NO_READ };
	ret |= lspcon_i2c_spi_register_control(fd, &packet);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 0);

	return ret;
}

static int lspcon_i2c_spi_enable_write_status_register_protection(int fd)
{
	int ret = lspcon_i2c_spi_toggle_register_protection(fd, 1);
	uint8_t data[] = { SWSPI_WDATA_PROTECT_BP };
	packet_t packet = {
		SWSPI_WDATA_WRITE_REGISTER, 1, data, SWSPICTL_ACCESS_TRIGGER | SWSPICTL_NO_READ };
	ret |= lspcon_i2c_spi_register_control(fd, &packet);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 0);

	return ret;
}

static int lspcon_i2c_spi_disable_protection(int fd)
{
	int ret = lspcon_i2c_spi_toggle_register_protection(fd, 1);
	uint8_t data[] = { SWSPI_WDATA_CLEAR_STATUS };
	packet_t packet = {
		SWSPI_WDATA_WRITE_REGISTER, 1, &data[0], SWSPICTL_ACCESS_TRIGGER | SWSPICTL_NO_READ };
	ret |= lspcon_i2c_spi_register_control(fd, &packet);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 0);

	return ret;
}

static int lspcon_i2c_spi_disable_hw_write(int fd)
{
	return lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, PAGE_HW_WRITE_DISABLE);
}

static int lspcon_i2c_spi_enable_write_protection(int fd)
{
	int ret = lspcon_i2c_spi_enable_write_status_register(fd);
	ret |= lspcon_i2c_spi_enable_write_status_register_protection(fd);
	ret |= lspcon_i2c_spi_wait_rom_free(fd);
	ret |= lspcon_i2c_spi_disable_hw_write(fd);

	return ret;
}

static int lspcon_i2c_spi_disable_all_protection(int fd)
{
	int ret = lspcon_i2c_spi_enable_write_status_register(fd);
	ret |= lspcon_i2c_spi_disable_protection(fd);
	ret |= lspcon_i2c_spi_wait_rom_free(fd);

	return ret;
}

static int lspcon_i2c_spi_send_command(struct flashctx *flash,
				unsigned int writecnt, unsigned int readcnt,
				const unsigned char *writearr,
				unsigned char *readarr)
{
	if (writecnt > 16 || readcnt > 16 || writecnt == 0) {
		msg_perr("Error: Invalid read/write count for send command.\n");
		return FUNCTION_ERR;
	}

	int fd = get_fd_from_context(flash);
	if (fd < 0)
		return FUNCTION_ERR;

	int ret = lspcon_i2c_spi_disable_all_protection(fd);
	ret |= lspcon_i2c_spi_enable_write_status_register(fd);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 1);

	// First bit of writearr shuld be the command value, followed by the value to write.
	// Read length occupies 4 bit and represents 16 level, thus if read 1 bit, read length should be 0.
	packet_t packet = {
		writearr[0], (writecnt - 1) | ((readcnt - 1) << 4), &writearr[1],
		SWSPICTL_ACCESS_TRIGGER | (readcnt ? 0 : SWSPICTL_NO_READ),
	};
	ret |= lspcon_i2c_spi_register_control(fd, &packet);
	ret |= lspcon_i2c_spi_wait_command_done(fd, SWSPICTL, SWSPICTL_ACCESS_TRIGGER);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 0);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < readcnt; ++i) {
		ret |= lspcon_i2c_spi_read_register(fd, SWSPI_RDATA, &readarr[i]);
	}

	ret |= lspcon_i2c_spi_wait_rom_free(fd);

	return ret;
}

static int lspcon_i2c_spi_enable_hw_write(int fd)
{
	int ret = 0;
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, PAGE_HW_COFIG_REGISTER);
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, PAGE_HW_WRITE_ENABLE);
	// Probably low page.
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, 0x50);
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, 0x41);
	// Probably high page.
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, 0x52);
	ret |= lspcon_i2c_spi_write_register(fd, PAGE_HW_WRITE, 0x44);

	return ret;
}

static int lspcon_i2c_clt2_spi_reset(int fd)
{
	int ret = 0;
	ret |= lspcon_i2c_spi_write_register(fd, CLT2_SPI, 0x20);
	struct timespec wait_100_ms = { 0, 100000000 };
	nanosleep(&wait_100_ms, NULL);
	ret |= lspcon_i2c_spi_write_register(fd, CLT2_SPI, 0x00);

	return ret;
}

static int lspcon_i2c_spi_reset_mpu_stop(int fd)
{
	int ret = 0;
	//					 ctrl stop
	// In address MPU space send instruction 0xc0 0x40
	ret |= lspcon_i2c_spi_write_register(fd, MPU, 0xc0); // cmd mode
	ret |= lspcon_i2c_spi_write_register(fd, MPU, 0x40); // stop mcu

	return ret;
}

static int lspcon_i2c_spi_map_page(int fd, unsigned int offset)
{
	// Page number byte, need to / PAGE_SIZE.
	uint8_t command_byte1[] = { ROMADDR_BYTE1, (offset >> 8) & 0xff };
	uint8_t command_byte2[] = { ROMADDR_BYTE2, (offset >> 16) };
	int ret = 0;
	// ret |= i2c_transmit(fd, REGISTER_ADDRESS, command_byte1, 2);
	// ret |= i2c_transmit(fd, REGISTER_ADDRESS, command_byte2, 2);
	ret |= lspcon_i2c_spi_write_data(fd, REGISTER_ADDRESS, command_byte1, 2);
	ret |= lspcon_i2c_spi_write_data(fd, REGISTER_ADDRESS, command_byte2, 2);

	return ret ? FUNCTION_ERR : 0;
}

// static int lspcon_i2c_spi_read_page(int fd, uint8_t *buf, unsigned int len)
// {
	// return lspcon_i2c_spi_read_data(fd, PAGE_ADDRESS, buf, len);
// }

static int lspcon_i2c_spi_read(struct flashctx *flash, uint8_t *buf,
			unsigned int start, unsigned int len)
{
	int ret = 0;
	if (start & 0xff)
		return default_spi_read(flash, buf, start, len);

	int fd = get_fd_from_context(flash);
	if (fd < 0)
		return FUNCTION_ERR;

	for (unsigned int i = 0; i < len; i += PAGE_SIZE) {
		ret |= lspcon_i2c_spi_map_page(fd, i + start);
		// ret |= lspcon_i2c_spi_read_page(fd, buf + i, min(len - i, PAGE_SIZE));
		ret |= lspcon_i2c_spi_read_data(fd, PAGE_ADDRESS, buf + i, min(len - i, PAGE_SIZE));
	}

	return ret;
}

static int lspcon_i2c_spi_write_page(int fd, const uint8_t *buf, unsigned int len)
{
	uint8_t write_buffer[len + 1];
	write_buffer[0] = 0;
	memcpy(write_buffer + 1, buf, len);
	return lspcon_i2c_spi_write_data(fd, PAGE_ADDRESS, write_buffer, len + 1);
}

static int lspcon_i2c_spi_write_256(struct flashctx *flash, const uint8_t *buf,
				 unsigned int start, unsigned int len)
{
	int ret = 0;
	if (start & 0xff)
		return default_spi_write_256(flash, buf, start, len);

	int fd = get_fd_from_context(flash);
	if (fd < 0)
		return FUNCTION_ERR;

	// ret |= lspcon_i2c_spi_disable_all_protection(fd);
	// Enable hardware write and reset clt2SPI interface.
	ret |= lspcon_i2c_spi_enable_hw_write(fd);
	ret |= lspcon_i2c_clt2_spi_reset(fd);

	for (unsigned int i = 0; i < len; i += PAGE_SIZE) {
		ret |= lspcon_i2c_spi_map_page(fd, start + i);
		ret |= lspcon_i2c_spi_write_page(fd, buf + i, min(len - i, PAGE_SIZE));
	}

	// ret |= lspcon_i2c_spi_enable_write_protection(fd);
	ret |= lspcon_i2c_spi_disable_hw_write(fd);
	if (false) {
		lspcon_i2c_spi_enable_hw_write(fd);
		lspcon_i2c_clt2_spi_reset(fd);
	}

	return ret;
}

static int lspcon_i2c_spi_write_aai(struct flashctx *flash, const uint8_t *buf,
				 unsigned int start, unsigned int len)
{
	msg_perr("Error: AAI write function is not supported.\n");
	return FUNCTION_ERR;
}

static const struct spi_master spi_master_i2c_lspcon = {
	.max_data_read = 16,
	.max_data_write = 8,
	.command = lspcon_i2c_spi_send_command,
	.multicommand = default_spi_send_multicommand,
	.read = lspcon_i2c_spi_read,
	.write_256 = lspcon_i2c_spi_write_256,
	.write_aai = lspcon_i2c_spi_write_aai,
};

// TODO: MPU still stop at this point, probably need to reset it.
static int lspcon_i2c_spi_shutdown(void *data)
{
	int ret = 0;
	int fd = ((struct lspcon_i2c_spi_data *)data)->fd;
	ret |= lspcon_i2c_spi_enable_write_protection(fd);
	ret |= lspcon_i2c_spi_toggle_register_protection(fd, 0);
	i2c_close(fd);
	free(data);

	return ret;
}

// TODO: remove this out of the specific spi master implementation.
static int get_bus()
{
	char *bus_str = extract_programmer_param("bus");
	if (bus_str) {
		char *bus_suffix;
		errno = 0;
		int bus = (int)strtol(bus_str, &bus_suffix, 10);
		if (errno != 0 || bus_str == bus_suffix) {
			msg_perr("Error: Cold no convert 'bus'.\n");
			goto err;
		}

		if (bus < 0 || bus > 255) {
			msg_perr("Error: Value for 'bus' is out of range(0-255).\n");
			goto err;
		}

		if (strlen(bus_suffix) > 0) {
			msg_perr("Error: Garbage following 'bus' value.\n");
			goto err;
		}

		msg_pinfo("Using i2c bus %i.\n", bus);
		return bus;
	} else {
		msg_perr("Error: Bus number not specified.\n");
	}
err:
	if (bus_str)
		free(bus_str);

	return FUNCTION_ERR;
}

static int register_lspcon_i2c_spi_master(void *ptr)
{
	struct spi_master mst = spi_master_i2c_lspcon;
	mst.data = ptr;

	return register_spi_master(&mst);
}

int lspcon_i2c_spi_init(void)
{
	int lspcon_i2c_spi_bus = get_bus();
	if (lspcon_i2c_spi_bus < 0)
		return FUNCTION_ERR;

	int ret = 0;
	int fd = i2c_open(lspcon_i2c_spi_bus, REGISTER_ADDRESS, 0);
	if (fd < 0)
		return fd;

	ret |= lspcon_i2c_spi_reset_mpu_stop(fd);
	if (ret)
		return ret;

	struct lspcon_i2c_spi_data data = { fd };
	void *data_ptr = calloc(1, sizeof(struct lspcon_i2c_spi_data));
	if (!data_ptr) {
		msg_perr("Unable to allocate space for extra spi master data.\n");
		return FUNCTION_ERR;
	}

	memcpy(data_ptr, &data, sizeof(struct lspcon_i2c_spi_data));
	ret |= register_shutdown(lspcon_i2c_spi_shutdown, data_ptr);
	ret |= register_lspcon_i2c_spi_master(data_ptr);

	return ret;
}
