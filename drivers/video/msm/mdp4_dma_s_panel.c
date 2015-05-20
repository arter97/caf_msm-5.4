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
 */

#include "msm_fb.h"
#include "msm_fb_panel.h"

static int __init mdp4_dma_s_panel_init(void)
{
	int ret = 0;
	struct platform_disp_info info = {
		.id = DISPLAY_TERTIARY,
		.dest = DISPLAY_4
	};

	if (!msm_fb_detect_client("mipi_dsi_i2c_video_wvga", &info))
		ret = mipi_dsi_i2c_video_wvga_device_register(&info);
	else if (!msm_fb_detect_client("mipi_dsi_i2c_video_xga", &info))
		ret = mipi_dsi_i2c_video_xga_device_register(&info);

	return ret;
}

module_init(mdp4_dma_s_panel_init);
