
/* Copyright (c) 2012 The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful;
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ADV7180_H__
#define __ADV7180_H__



/**
 * struct adv7180_platform_data
 * structure to pass board specific information to the adv7180
 * @rstb_gpio:		put active low to hold chip in reset state
 * @pwdnb_gpio:		put active low to allow chip to pwrdwn and disable I2C
 * @dev_num:		instance of the device
 * @pwr_on:		indicates whether power on sequence is required for chip
 */
struct adv7180_platform_data {
	int rstb_gpio;
	int pwdnb_gpio;
	int dev_num;
	int pwr_on;
};

#endif

