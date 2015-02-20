/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#ifndef __MIPI_DSI_I2C_H__
#define __MIPI_DSI_I2C_H__

#include "msm_fb_panel.h"

/*
 * client may use this info to implement its own i2c read and write
 */
struct mipi_dsi_i2c_resource {
	struct i2c_client *client;
	int gpio;
};

/*
 * two hook functions for the specific setup
 */
struct mipi_dsi_i2c_configure {
	int (*config_dsi)(void);
	int (*config_i2c)(struct msm_panel_info *pinfo);
	void (*config_reset_dev)(void);
};

extern struct mipi_dsi_i2c_resource dsi_i2c_rsc;
extern struct mipi_dsi_i2c_configure dsi_i2c_cfg;

int mipi_dsi_i2c_read_byte(u8 addr, u8 reg, u8 *buf);
int mipi_dsi_i2c_write_byte(u8 addr, u8 reg, u8 val);

int mipi_dsi_i2c_device_register(struct msm_panel_info *pinfo);

#endif /* __MIPI_DSI_I2C_H__ */
