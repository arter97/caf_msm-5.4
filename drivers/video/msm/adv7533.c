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

#include "mipi_dsi_i2c.h"
#include "adv7533.h"
#include "msm_fb.h"

#define ADV7533_REG_CHIP_REVISION (0x00)
#define ADV7533_MAIN (0x3d) /* 7a main right shift 1 */
#define ADV7533_CEC_DSI (0x3c)

static struct mipi_dsi_i2c_reg_cfg setup_cfg[] = {
	{ADV7533_MAIN, 0xd6, 0x48, 5},		/* HPD overriden */
	{ADV7533_MAIN, 0x41, 0x10, 5},		/* HDMI normal */
	{ADV7533_CEC_DSI, 0x03, 0x89, 0},	/* HDMI enabled */
	{ADV7533_MAIN, 0x16, 0x20, 0},
	{ADV7533_MAIN, 0x9A, 0xE0, 0},
	{ADV7533_MAIN, 0xBA, 0x70, 0},
	{ADV7533_MAIN, 0xDE, 0x82, 0},
	{ADV7533_MAIN, 0xE4, 0xC0, 0},
	{ADV7533_MAIN, 0xE5, 0x80, 0},
	{ADV7533_CEC_DSI, 0x15, 0xD0, 0},
	{ADV7533_CEC_DSI, 0x17, 0xD0, 0},
	{ADV7533_CEC_DSI, 0x24, 0x20, 0},
	{ADV7533_CEC_DSI, 0x57, 0x11, 0},
	/* hdmi or dvi mode: hdmi */
	{ADV7533_MAIN, 0xAF, 0x06, 0},
	{ADV7533_MAIN, 0x40, 0x80, 0},
	{ADV7533_MAIN, 0x4C, 0x04, 0},
	{ADV7533_MAIN, 0x49, 0x02, 0},
	{ADV7533_MAIN, 0x0D, 1 << 6, 0},
	{ADV7533_CEC_DSI, 0x1C, 0x30, 0},
};

static int mipi_adv7533_read_device_rev(void)
{
	u8 rev = 0;
	int ret;

	ret = mipi_dsi_i2c_read_byte(ADV7533_MAIN, ADV7533_REG_CHIP_REVISION,
		&rev);

	if (!ret)
		pr_info("adv7533 revision %x\n", rev);
	else
		pr_err("adv7533 rev error\n");

	return ret;
}

static int mipi_dsi_i2c_write_regs(struct mipi_dsi_i2c_reg_cfg *cfg, int size)
{
	int ret = 0;
	int i;

	for (i = 0; i < size; i++) {
		ret = mipi_dsi_i2c_write_byte(cfg[i].i2c_addr, cfg[i].reg,
			cfg[i].val);
		if (ret) {
			pr_err("mipi_dsi reg writes failed\n");
			goto w_regs_fail;
		}
		if (cfg[i].sleep_in_ms) {
			if (cfg[i].sleep_in_ms < 20)
				usleep_range(cfg[i].sleep_in_ms*1000, 10000);
			else
				msleep(cfg[i].sleep_in_ms);
		}
	}

w_regs_fail:
	return ret;
}

static int mipi_adv7533_config_common(void)
{
	int ret;

	ret = mipi_adv7533_read_device_rev();
	if (ret)
		goto s_err;

	ret = mipi_dsi_i2c_write_regs(setup_cfg, ARRAY_SIZE(setup_cfg));

s_err:
	return ret;
}

static int mipi_adv7533_config_timing(struct msm_panel_info *pinfo)
{
	if (pinfo != NULL) {
		int ret;
		uint32 h_total = (pinfo->xres +
			pinfo->lcdc.h_back_porch  +
			pinfo->lcdc.h_pulse_width +
			pinfo->lcdc.h_front_porch);

		uint32 v_total = (pinfo->yres +
			pinfo->lcdc.v_back_porch  +
			pinfo->lcdc.v_pulse_width +
			pinfo->lcdc.v_front_porch);

		uint32 active_high =
		(pinfo->lcdc.is_sync_active_high == FALSE) ? 0x00 : 0x60;

		struct mipi_dsi_i2c_reg_cfg tg_cfg[] = {
			/* hsync and vsync active low */ /*60 or 00*/
			{ADV7533_MAIN, 0x17, active_high, 0},
			/* h_width 0x672 1650*/
			{ADV7533_CEC_DSI, 0x28, ((h_total & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x29, ((h_total & 0xF) << 4), 0},
			/* hsync_width 0x28 40*/
			{ADV7533_CEC_DSI, 0x2A,
				((pinfo->lcdc.h_pulse_width & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x2B,
				((pinfo->lcdc.h_pulse_width & 0xF) << 4), 0},
			/* hfp 0x6e 110 */
			{ADV7533_CEC_DSI, 0x2C,
				((pinfo->lcdc.h_front_porch & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x2D,
				((pinfo->lcdc.h_front_porch & 0xF) << 4), 0},
			/* hbp 0xdc 220 */
			{ADV7533_CEC_DSI, 0x2E,
				((pinfo->lcdc.h_back_porch & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x2F,
				((pinfo->lcdc.h_back_porch & 0xF) << 4), 0},
			/* v_total 0x2ee 750 */
			{ADV7533_CEC_DSI, 0x30, ((v_total & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x31, ((v_total & 0xF) << 4), 0},
			/* vsync_width 0x05 5*/
			{ADV7533_CEC_DSI, 0x32,
				((pinfo->lcdc.v_pulse_width & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x33,
				((pinfo->lcdc.v_pulse_width & 0xF) << 4), 0},
			/* vfp 0x05 5  */
			{ADV7533_CEC_DSI, 0x34,
				((pinfo->lcdc.v_front_porch & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x35,
				((pinfo->lcdc.v_front_porch & 0xF) << 4), 0},
			/* vbp 0x14 20 */
			{ADV7533_CEC_DSI, 0x36,
				((pinfo->lcdc.v_back_porch & 0xFF0) >> 4), 0},
			{ADV7533_CEC_DSI, 0x37,
				((pinfo->lcdc.v_back_porch & 0xF) << 4), 0},
			{ADV7533_CEC_DSI, 0x03, 0x09, 0},/* HDMI disabled */
			{ADV7533_CEC_DSI, 0x03, 0x89, 0},/* HDMI enabled */
		};

		ret = mipi_adv7533_config_common();
		if (!ret)
			ret = mipi_dsi_i2c_write_regs(
					tg_cfg, ARRAY_SIZE(tg_cfg));
		return ret;
	}
	return -EFAULT;
}

static int __init adv7533_init(void)
{
	struct mipi_dsi_i2c_configure *cfg = &dsi_i2c_cfg;
	cfg->config_i2c = mipi_adv7533_config_timing;
	return 0;
}

module_init(adv7533_init);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("adv7533 driver");
