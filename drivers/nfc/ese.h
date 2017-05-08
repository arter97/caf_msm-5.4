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
 */

#define ESE_MAGIC 0xEA
#define ESE_SET_PWR _IOW(ESE_MAGIC, 0x01, unsigned int)
#define ESE_SET_DBG _IOW(ESE_MAGIC, 0x02, unsigned int)
#define ESE_SET_MODE _IOW(ESE_MAGIC, 0x03, unsigned int)

#define ESE_POLL_TIMEOUT (2*1000)/*max 2 seconds*/
#define ESE_SPI_CLOCK	 960000L

/* size of maximum read/write buffer supported by driver */
#define MAX_BUFFER_SIZE	258U
#define SOF_LENGTH	0x01
#define HEADER_LENGTH	0x02
#define HEADER_OFFSET	0x01
#define DATA_LEN_INDEX	0x02
#define DATA_READ_OFFSET	0x03
#define MAX_SOF_IREAD_COUNT	0x03

#define ESE_SECURE_TRANSFER _IOW(ESE_MAGIC, 0x04, unsigned int)
#define ESE_SECURE_TRANSFER_OFF _IOW(ESE_MAGIC, 0x05, unsigned int)
struct ese_spi_platform_data {
	unsigned int use_pwr_req;
	unsigned int pwr_req;
	unsigned int ese_intr;
};
