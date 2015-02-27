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

#ifndef __ADV7533_H__
#define __ADV7533_H__

#include "mach/board.h"
struct mipi_dsi_i2c_reg_cfg {
	u8 i2c_addr;
	u8 reg;
	u8 val;
	int sleep_in_ms;
};

extern int mipi_dsi_i2c_read_byte(u8 addr, u8 reg, u8 *buf);
extern int mipi_dsi_i2c_write_byte(u8 addr, u8 reg, u8 val);
extern struct mipi_dsi_i2c_resource dsi_i2c_rsc;
extern struct mipi_dsi_i2c_configure dsi_i2c_cfg;
extern int msm_fb_detect_client(const char *name,
				struct platform_disp_info *info);

#define ADV7533_REG_CHIP_REVISION (0x00)
#define ADV7533_MAIN (0x3d) /* 7a main right shift 1 */
#define ADV7533_CEC_DSI (0x3c)

#endif	/* __ADV7533_H__ */
