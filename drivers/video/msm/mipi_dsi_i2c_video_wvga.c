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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_i2c.h"

static struct msm_panel_info pinfo;

/* timing, phy and pll ctl */
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* 480x720, RGB888, 3 Lane 60 fps video mode */
	/* regulator n/a */
	{0x00, 0x0a, 0x04, 0x00},
	/* timing 0x440*/
	{0x49, 0x0D, 0x09, 0x00,
	 0x2C, 0x33, 0x0D, 0x11,
	 0x0E, 0x03, 0x04},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},
	/* strength */
	{0xff, 0x00, 0x06, 0x00},
	/* pll control */
	{0x00, 0x9f, 0x49, 0xd9,
	 0x00, 0x50, 0x48, 0x63,
	 0x03, 0x1f, 0x03, 0x00,
	 0x14, 0x03, 0x00, 0x02,
	 0x00, 0x20, 0x00, 0x01,
	 0x00},
};

int mipi_dsi_i2c_video_wvga_device_register(struct platform_disp_info *info)
{
	int ret;

	pinfo.xres = 720;
	pinfo.yres = 480;

	pinfo.lcdc.xres_pad = 0;
	pinfo.lcdc.yres_pad = 0;

	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = info->dest;
	pinfo.disp_id = info->id;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 60;
	pinfo.lcdc.h_front_porch = 16;
	pinfo.lcdc.h_pulse_width = 62;
	pinfo.lcdc.v_back_porch = 30;
	pinfo.lcdc.v_front_porch = 9;
	pinfo.lcdc.v_pulse_width = 6;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;

	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = FALSE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	/*
	 * video_mode_data_ctrl 0x1c
	 * no config from test code
	 * DSI_RGB_SWAP_RGB
	 */
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;

	/*
	 * dsi_ctrl 0x0 = 0x177 and 0x175
	 * bit(6) = 1 dln2_en
	 * bit(5) = 1 dln1_en
	 * bit(4) = 1 dln0_en
	 */
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = FALSE;
	/*
	 * dsi1_clkout_timing_ctrl 0xc0 = 0x38
	 * bit(5:0) = 0x38 t_clk_pre
	 */
	pinfo.mipi.t_clk_post = 0x05;
	pinfo.mipi.t_clk_pre = 0x12;

	pinfo.mipi.stream = 0;
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;

	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.esc_byte_ratio = 2;

	ret = mipi_dsi_i2c_device_register(&pinfo);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

static int __init mipi_dsi_i2c_video_wvga_init(void)
{
	struct platform_disp_info info = {
		.id = DISPLAY_PRIMARY,
		.dest = DISPLAY_1
	};

	return (msm_fb_detect_client("mipi_dsi_video_wvga", &info)) ?
		0 : mipi_dsi_i2c_video_wvga_device_register(&info);

}

module_init(mipi_dsi_i2c_video_wvga_init);
