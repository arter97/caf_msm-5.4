/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#ifndef __WCD_SPI_H__
#define __WCD_SPI_H__

struct mdss_spi_msg {
	/*
	 * Caller's buffer pointer that holds data to
	 * be transmitted in case of data_write and
	 * data to be copied to in case of data_read.
	 */
	void *data;

	/* Length of data to write/read */
	size_t len;

	/*
	 * Address in remote memory to write to
	 * or read from.
	 */
	u32 remote_addr;

	/* Bitmask of flags, currently unused */
	u32 flags;
};

#if 1

int mdss_spi_transfer(const void *buf, size_t len);
int mdss_spi_transfer_data(const void *buf, size_t len);

int mdss_spi_panel_init(void);

#else

int mdss_spi_data_write(struct spi_device *spi, struct mdss_spi_msg *msg)
{
	return -ENODEV;
}

int mdss_spi_data_read(struct spi_device *spi, struct mdss_spi_msg *msg)
{
	return -ENODEV;
}

#endif /* End of CONFIG_SND_SOC_WCD_SPI */

#endif /* End of __WCD_SPI_H__ */

