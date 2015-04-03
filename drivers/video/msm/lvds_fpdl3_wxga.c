/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <video/msm_dba.h>

struct fpdl3_info {
	struct lvds_fpdl3_platform_data pdata;
	struct msm_dba_ops ops;
	void *handle;
	bool hdcp_status;
	bool hdcp_en;
};

static struct fpdl3_info fpdl3;
static struct platform_device *cm_fbpdev;

static ssize_t hdcp_status_rda_attr(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "%d\n",
			       fpdl3.hdcp_status ? 1 : 0);
	return ret;
}

static ssize_t hdcp_en_rda_attr(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "%d\n",
			       fpdl3.hdcp_en ? 1 : 0);
	return ret;
}

static ssize_t hdcp_en_wta_attr(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int rc;
	long en = 0;

	rc = kstrtol(buf, 0, &en);
	if (!rc) {
		if (en)
			fpdl3.hdcp_en = true;
		else
			fpdl3.hdcp_en = false;

		pr_debug("%s: HDCP_EN = %d\n", __func__, fpdl3.hdcp_en);
		rc = fpdl3.ops.hdcp_enable(fpdl3.handle, fpdl3.hdcp_en,
					   fpdl3.hdcp_en, 0);
		if (rc)
			pr_err("%s: enable hdcp failed\n", __func__);
		else if (fpdl3.hdcp_en == false)
			fpdl3.hdcp_status = false;
	} else {
		pr_err("%s: invalid parameter\n", __func__);
	}

	return ret;
}

static DEVICE_ATTR(hdcp_en, S_IRUGO | S_IWUSR, hdcp_en_rda_attr,
		   hdcp_en_wta_attr);
static DEVICE_ATTR(hdcp_status, S_IRUGO, hdcp_status_rda_attr, NULL);

static struct attribute *lvds_fpdl3_fs_attrs[] = {
	&dev_attr_hdcp_en.attr,
	&dev_attr_hdcp_status.attr,
	NULL,
};
static struct attribute_group lvds_fpdl3_fs_attr_grp = {
	.attrs = lvds_fpdl3_fs_attrs,
};
static int lvds_fpdl3_sysfs_attrib_init(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		pr_err("%s: mfd not found\n", __func__);
		return -ENODEV;
	}

	if (!mfd->fbi) {
		pr_err("%s: mfd->fbi not found\n", __func__);
		return -ENODEV;
	}

	if (!mfd->fbi->dev) {
		pr_err("%s: mfd->fbi->dev not found\n", __func__);
		return -ENODEV;
	}

	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &lvds_fpdl3_fs_attr_grp);
	if (rc) {
		pr_err("%s: sysfs group creation failed, rc=%d\n", __func__,
		       rc);
	}

	return rc;
}

static int lvds_fpdl3_panel_on(struct platform_device *pdev)
{
	int rc;

	rc = fpdl3.ops.power_on(fpdl3.handle, true, 0x0);
	if (rc) {
		pr_err("%s: power on failed\n", __func__);
		return rc;
	}

	rc = fpdl3.ops.video_on(fpdl3.handle, true, NULL, 0x0);
	if (rc) {
		pr_err("%s: video on failed\n", __func__);
		return rc;
	}

	rc = fpdl3.ops.interrupts_enable(fpdl3.handle, true,
					 MSM_DBA_CB_HDCP_LINK_AUTHENTICATED |
					 MSM_DBA_CB_HDCP_LINK_UNAUTHENTICATED,
					 0x0);
	if (rc) {
		pr_err("%s: enable interrupts failed\n", __func__);
		return rc;
	}
	return 0;
}

static int lvds_fpdl3_panel_off(struct platform_device *pdev)
{
	int rc;

	rc = fpdl3.ops.video_on(fpdl3.handle, false, NULL, 0x0);
	if (rc) {
		pr_err("%s: video off failed\n", __func__);
		return rc;
	}

	rc = fpdl3.ops.power_on(fpdl3.handle, false, 0x0);
	if (rc) {
		pr_err("%s: power off failed\n", __func__);
		return rc;
	}

	return 0;
}

static void lvds_fpdl3_set_backlight(struct msm_fb_data_type *mfd)
{
	pr_debug("%s: back light level %d\n", __func__, mfd->bl_level);
}

static void lvds_fpdl3_event_cb(void *data, enum msm_dba_callback_event event)
{
	struct fpdl3_info *dev = data;

	pr_debug("%s: Received event %d\n", __func__, event);
	if (!data)
		return;

	if (event & MSM_DBA_CB_HDCP_LINK_AUTHENTICATED)
		dev->hdcp_status = true;
	if (event & MSM_DBA_CB_HDCP_LINK_UNAUTHENTICATED)
		dev->hdcp_status = false;
}

static int __devinit lvds_fpdl3_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_dba_reg_info reg_info;

	if (pdev->id == 0) {
		if (pdev->dev.platform_data == NULL) {
			pr_err("%s: no FPDL3 info specified\n", __func__);
			return -ENODEV;
		}

		fpdl3.pdata =
		   *(struct lvds_fpdl3_platform_data *)pdev->dev.platform_data;
		return 0;
	}

	memset(&reg_info, 0x0, sizeof(reg_info));
	strlcpy(reg_info.client_name, pdev->name,
				MSM_DBA_CLIENT_NAME_LEN);
	strlcpy(reg_info.chip_name, fpdl3.pdata.chip_id,
				MSM_DBA_CHIP_NAME_MAX_LEN);
	reg_info.instance_id = fpdl3.pdata.instance_id;
	reg_info.cb = lvds_fpdl3_event_cb;
	reg_info.cb_data = &fpdl3;

	fpdl3.handle = msm_dba_register_client(&reg_info, &fpdl3.ops);
	if (!fpdl3.handle) {
		pr_err("%s: fpdl3 driver register failed\n", __func__);
		return -ENODEV;
	}

	cm_fbpdev = msm_fb_add_device(pdev);
	if (!cm_fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
		goto probe_exit;
	}

	rc = lvds_fpdl3_sysfs_attrib_init(cm_fbpdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: sys fs attring init failed\n",
			__func__);
		goto probe_exit;
	}
probe_exit:
	return rc;
}

static struct platform_driver this_driver = {
	.probe  = lvds_fpdl3_probe,
	.driver = {
		.name   = "lvds_fpdl3_wxga",
	},
};

static struct msm_fb_panel_data lvds_fpdl3_panel_data = {
	.on = lvds_fpdl3_panel_on,
	.off = lvds_fpdl3_panel_off,
	.set_backlight = lvds_fpdl3_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lvds_fpdl3_wxga",
	.id	= 1,
	.dev	= {
		.platform_data = &lvds_fpdl3_panel_data,
	}
};

static int __init lvds_fpdl3_wxga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
	struct platform_disp_info info = {
		.id = DISPLAY_PRIMARY,
		.dest = DISPLAY_1
	};

	if (msm_fb_detect_client("lvds_fpdl3_wxga", &info))
		return 0;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lvds_fpdl3_panel_data.panel_info;
	pinfo->xres = 1366;
	pinfo->yres = 768;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LVDS_PANEL;
	pinfo->pdest = info.dest;
	pinfo->disp_id = info.id;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 79400000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	/*
	 * this panel is operated by de,
	 * vsycn and hsync are ignored
	 */
	pinfo->lcdc.h_back_porch = 0;
	pinfo->lcdc.h_front_porch = 194;
	pinfo->lcdc.h_pulse_width = 40;
	pinfo->lcdc.v_back_porch = 0;
	pinfo->lcdc.v_front_porch = 38;
	pinfo->lcdc.v_pulse_width = 20;
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
	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lvds_fpdl3_wxga_init);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LVDS FPDLink3 panel driver");
