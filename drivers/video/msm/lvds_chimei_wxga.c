/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/socinfo.h>
#include "mach/board.h"
#include <video/msm_dba.h>

#define LVDS_CHIMEI_PWM_FREQ_HZ 300
#define LVDS_CHIMEI_PWM_PERIOD_USEC (USEC_PER_SEC / LVDS_CHIMEI_PWM_FREQ_HZ)
#define LVDS_CHIMEI_PWM_LEVEL 255
#define LVDS_CHIMEI_PWM_DUTY_LEVEL \
	(LVDS_CHIMEI_PWM_PERIOD_USEC / LVDS_CHIMEI_PWM_LEVEL)
#define LVDS_CHIMEI_DBA_CLIENT_NAME "LVDS_CHIMEI_PANEL"
#define LVDS_CHIMEI_DBA_CHIP_NAME   "DS90UH927Q"

static struct lvds_panel_platform_data *cm_pdata;
static struct platform_device *cm_fbpdev;
static struct pwm_device *bl_lpm;
static struct lvds_dba_info {
	void *handle;
	struct msm_dba_ops ops;
} dba_info;
static panel_error_cb err_cb;

static void lvds_chimei_panel_dba_cb(void *data,
				enum msm_dba_callback_event event)
{
	struct platform_device *pdev = (struct platform_device *)data;
	boolean failure = false;
	if (event & MSM_DBA_CB_VIDEO_FAILURE) {
		pr_info("%s DBA error detected: video failure", __func__);
		failure = true;
	}
	if (event & MSM_DBA_CB_AUDIO_FAILURE) {
		pr_info("%s DBA error detected: audio failure", __func__);
		failure = true;
	}
	/* callback to notify MDP */
	if (failure && err_cb)
		err_cb(pdev);
}

static int lvds_chimei_panel_register_dba(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_dba_reg_info info;

	memset(&info, 0, sizeof(info));
	strlcpy(info.client_name, LVDS_CHIMEI_DBA_CLIENT_NAME,
		MSM_DBA_CLIENT_NAME_LEN);
	strlcpy(info.chip_name, LVDS_CHIMEI_DBA_CHIP_NAME,
		MSM_DBA_CHIP_NAME_MAX_LEN);
	info.cb = lvds_chimei_panel_dba_cb;
	info.cb_data = pdev;
	dba_info.handle = msm_dba_register_client(&info, &dba_info.ops);
	if (IS_ERR(dba_info.handle)) {
		pr_err("%s dba register client failed!", __func__);
		rc = PTR_ERR(dba_info.handle);
	}
	return rc;
}

static int lvds_chimei_panel_deregister_dba(struct platform_device *pdev)
{
	return msm_dba_deregister_client(dba_info.handle);
}

static int lvds_chimei_panel_reset(struct platform_device *pdev)
{
	if (dba_info.ops.force_reset)
		return dba_info.ops.force_reset(dba_info.handle, 0);
	else
		return -EINVAL;
}

static int lvds_chimei_panel_set_cb(struct platform_device *pdev,
				panel_error_cb cb)
{
	if (cb) {
		err_cb = cb;
		return 0;
	} else {
		pr_err("%s callback function is NULL!", __func__);
		return -EINVAL;
	}
}

static int lvds_chimei_panel_on(struct platform_device *pdev)
{
	return 0;
}

static int lvds_chimei_panel_off(struct platform_device *pdev)
{
	return 0;
}

static void lvds_chimei_set_backlight(struct msm_fb_data_type *mfd)
{
	int ret;

	pr_debug("%s: back light level %d\n", __func__, mfd->bl_level);

	if (bl_lpm) {
		ret = pwm_config(bl_lpm, LVDS_CHIMEI_PWM_DUTY_LEVEL *
			mfd->bl_level, LVDS_CHIMEI_PWM_PERIOD_USEC);
		if (ret) {
			pr_err("pwm_config on lpm failed %d\n", ret);
			return;
		}
		if (mfd->bl_level) {
			ret = pwm_enable(bl_lpm);
			if (ret)
				pr_err("pwm enable/disable on lpm failed"
					"for bl %d\n",	mfd->bl_level);
		} else {
			pwm_disable(bl_lpm);
		}
	}
}

static int __devinit lvds_chimei_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (pdev->id == 0) {
		cm_pdata = pdev->dev.platform_data;
		if (cm_pdata == NULL)
			pr_err("%s: no PWM gpio specified\n", __func__);
		return 0;
	}

	if (cm_pdata != NULL)
		bl_lpm = pwm_request(cm_pdata->gpio[0],
			"backlight");

	if (bl_lpm == NULL || IS_ERR(bl_lpm)) {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_lpm = NULL;
	}
	pr_debug("bl_lpm = %p lpm = %d\n", bl_lpm,
		cm_pdata->gpio[0]);

	cm_fbpdev = msm_fb_add_device(pdev);
	if (!cm_fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
		goto probe_exit;
	}
	lvds_chimei_panel_register_dba(pdev);

probe_exit:
	return rc;
}

static int lvds_chimei_remove(struct platform_device *pdev)
{
	int rc = 0;
	rc = lvds_chimei_panel_deregister_dba(pdev);
	return rc;
}

static struct platform_driver this_driver = {
	.probe  = lvds_chimei_probe,
	.remove = lvds_chimei_remove,
	.driver = {
		.name   = "lvds_chimei_wxga",
	},
};

static struct msm_fb_panel_data lvds_chimei_panel_data = {
	.on = lvds_chimei_panel_on,
	.off = lvds_chimei_panel_off,
	.set_backlight = lvds_chimei_set_backlight,
	.dba_reset = lvds_chimei_panel_reset,
	.set_error_cb = lvds_chimei_panel_set_cb,
};

static struct platform_device this_device = {
	.name   = "lvds_chimei_wxga",
	.id	= 1,
	.dev	= {
		.platform_data = &lvds_chimei_panel_data,
	}
};

static int __init lvds_chimei_wxga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
	struct platform_disp_info info = {
		.id = DISPLAY_PRIMARY,
		.dest = DISPLAY_1
	};

	if (msm_fb_detect_client("lvds_chimei_wxga", &info))
		return 0;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lvds_chimei_panel_data.panel_info;
	if (machine_is_apq8064_mplatform()) {
		pinfo->xres = 1280;
		pinfo->yres = 720;
		MSM_FB_SINGLE_MODE_PANEL(pinfo);
		pinfo->type = LVDS_PANEL;
		pinfo->pdest = info.dest;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 24;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 79400000;
		pinfo->bl_max = 255;
		pinfo->bl_min = 1;
		pinfo->lcdc.h_back_porch = 43;
		pinfo->lcdc.h_front_porch = 43;
		pinfo->lcdc.h_pulse_width = 32;
		pinfo->lcdc.v_back_porch = 8;
		pinfo->lcdc.v_front_porch = 3;

		pinfo->lcdc.v_pulse_width = 5;
		pinfo->lcdc.underflow_clr = 0xff;
		pinfo->lcdc.hsync_skew = 0;
		pinfo->lvds.channel_mode =
			LVDS_SINGLE_CHANNEL_MODE;

		/* Set border color,
		 * padding only for reducing active
		 * display region
		 */
		pinfo->lcdc.border_clr = 0x0;
		pinfo->lcdc.xres_pad = 0;
		pinfo->lcdc.yres_pad = 0;
		pinfo->disp_id = info.id;

	} else {
		pinfo->xres = 1366;
		pinfo->yres = 768;
		MSM_FB_SINGLE_MODE_PANEL(pinfo);
		pinfo->type = LVDS_PANEL;
		pinfo->pdest = info.dest;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 24;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 74958000;
		pinfo->bl_max = 255;
		pinfo->bl_min = 1;

		/*
		 * this panel is operated by de,
		 * vsycn and hsync are ignored
		 */
		pinfo->lcdc.is_den_active_high = TRUE;
		pinfo->lcdc.is_sync_active_high = TRUE;
		pinfo->lcdc.h_back_porch = 0;
		pinfo->lcdc.h_front_porch = 164;
		pinfo->lcdc.h_pulse_width = 20;
		pinfo->lcdc.v_back_porch = 0;
		pinfo->lcdc.v_front_porch = 28;
		pinfo->lcdc.v_pulse_width = 10;
		pinfo->lcdc.underflow_clr = 0xff;
		pinfo->lcdc.hsync_skew = 0;
		pinfo->lvds.channel_mode =
			LVDS_SINGLE_CHANNEL_MODE;

		/*
		 * Set border color,
		 * padding only for reducing active
		 * display region
		 */
		pinfo->lcdc.border_clr = 0x0;
		pinfo->lcdc.xres_pad = 0;
		pinfo->lcdc.yres_pad = 0;
		pinfo->disp_id = info.id;
	}
	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lvds_chimei_wxga_init);
