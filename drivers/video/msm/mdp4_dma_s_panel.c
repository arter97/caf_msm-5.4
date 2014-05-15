/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include "msm_fb.h"
#include "msm_fb_panel.h"

static int __init mdp4_dma_s_panel_init(void)
{
	int ret = 0;

	if (!msm_fb_detect_client("mipi_dsi_i2c_video_wvga"))
		ret = mipi_dsi_i2c_video_wvga_device_register(DISPLAY_4);
	else if (!msm_fb_detect_client("mipi_dsi_i2c_video_xga"))
		ret = mipi_dsi_i2c_video_xga_device_register(DISPLAY_4);

	return ret;
}

module_init(mdp4_dma_s_panel_init);
