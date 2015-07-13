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
 *
 */
#include "msm_sensor.h"
#include <media/msm_ba.h>

#define NUM_AV_DEVICES 2

DEFINE_MUTEX(avdevice_mut);
DEFINE_MUTEX(avdevice2_mut);

static struct msm_sensor_ctrl_t avdevice_a_s_ctrl[NUM_AV_DEVICES];
static void *ba_instance_handler_a[NUM_AV_DEVICES];

static struct v4l2_subdev_info avdevice_subdev_info[] = {
	{
	.code		= V4L2_MBUS_FMT_UYVY8_2X8,
	.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	.fmt		= 1,
	.order		= 0,
	},
	/* more can be supported, to be added later */
};

#define avdevice_TOTALWIDTH    (720)
#define avdevice_TOTALHEIGHT   (507)
#define avdevice_FPS   60
static struct msm_sensor_output_info_t avdevice_dimensions[] = {
	{
		.x_output = avdevice_TOTALWIDTH,
		.y_output = avdevice_TOTALHEIGHT,
		.line_length_pclk = avdevice_TOTALWIDTH,
		.frame_length_lines = avdevice_TOTALHEIGHT,
		.vt_pixel_clk = (avdevice_TOTALWIDTH*
				avdevice_TOTALHEIGHT*
				avdevice_FPS),
		.op_pixel_clk = 96 * 1000000,
		.binning_factor = 0,
	},
};

static enum v4l2_mbus_pixelcode avdevice_v4l2fmt_to_mbuspixfmt(u32 fmt)
{
	enum v4l2_mbus_pixelcode mbuspixfmt;
	switch (fmt) {
	case V4L2_PIX_FMT_YUYV:
		mbuspixfmt = V4L2_MBUS_FMT_YUYV8_2X8;
		break;
	case V4L2_PIX_FMT_YVYU:
		mbuspixfmt = V4L2_MBUS_FMT_YVYU8_2X8;
		break;
	case V4L2_PIX_FMT_VYUY:
		mbuspixfmt = V4L2_MBUS_FMT_VYUY8_2X8;
		break;
	case V4L2_PIX_FMT_UYVY:
		mbuspixfmt = V4L2_MBUS_FMT_UYVY8_2X8;
		break;
	default:
		pr_err("%s: Unknown v4l2 fmt 0x%x", __func__, fmt);
		mbuspixfmt = V4L2_PIX_FMT_UYVY;
		break;
	}

	return mbuspixfmt;
}

static int32_t avdevice_ba_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *p_sensor_ctrl;
	struct msm_camera_sensor_info *p_sensor_info = pdev->dev.platform_data;
	struct v4l2_format ba_fmt;
	struct v4l2_control ctrl;

	pr_debug("avdevice platform platform probe...\n");

	if (!p_sensor_info) {
		pr_err("%s - dev platform data not set\n",
						__func__);
		return -EFAULT;
	}

	if (pdev->id < 0 || pdev->id > 1) {
		pr_err("%s - device id %d currently not supported...\n",
				__func__, pdev->id);
		return -EINVAL;
	}

	p_sensor_ctrl = &avdevice_a_s_ctrl[pdev->id];
	pr_debug("csi device %d %p...\n", pdev->id, p_sensor_ctrl);

	ba_instance_handler_a[pdev->id] = msm_ba_open(NULL);
	if (!ba_instance_handler_a[pdev->id]) {
		pr_err("%s - msm_ba_open(%d) failed",
				__func__, pdev->id);
		return -EFAULT;
	}

	rc = msm_ba_s_output(ba_instance_handler_a[pdev->id],
				p_sensor_info->ba_idx);
	if (rc) {
		pr_err("%s - msm_ba_s_output failed %d\n",
				__func__, rc);
		goto ba_failed;
	}

	rc = msm_ba_g_fmt(ba_instance_handler_a[pdev->id], &ba_fmt);
	if (rc) {
		pr_err("%s - msm_ba_g_fmt failed %d\n",
				__func__, rc);
		goto ba_failed;
	}

	ctrl.id = MSM_BA_PRIV_FPS;
	rc = msm_ba_g_ctrl(ba_instance_handler_a[pdev->id], &ctrl);
	if (rc) {
		pr_err("%s - msm_ba_g_ctrl %x failed %d\n",
				__func__, ctrl.id, rc);
		goto ba_failed;
	}

	p_sensor_ctrl->sensor_v4l2_subdev_info->code =
			avdevice_v4l2fmt_to_mbuspixfmt(
					ba_fmt.fmt.pix.pixelformat);
	p_sensor_ctrl->sensor_v4l2_subdev_info->colorspace =
			ba_fmt.fmt.pix.colorspace;

	p_sensor_ctrl->msm_sensor_reg->output_settings->x_output =
			ba_fmt.fmt.pix.width;
	p_sensor_ctrl->msm_sensor_reg->output_settings->y_output =
			ba_fmt.fmt.pix.height;
	p_sensor_ctrl->msm_sensor_reg->output_settings->line_length_pclk =
			ba_fmt.fmt.pix.width;
	p_sensor_ctrl->msm_sensor_reg->output_settings->frame_length_lines =
			ba_fmt.fmt.pix.height;

	p_sensor_ctrl->msm_sensor_reg->output_settings->vt_pixel_clk =
			ba_fmt.fmt.pix.width * ba_fmt.fmt.pix.height *
			(ctrl.value >> 16);
	pr_debug("%s - %dx%d @ %d\n", __func__, ba_fmt.fmt.pix.width,
			ba_fmt.fmt.pix.height, (ctrl.value >> 16));

	p_sensor_ctrl->pdev = pdev;
	pr_debug("avdevice probe sctrl %p pdev %p...\n", p_sensor_ctrl, pdev);
	rc = msm_sensor_platform_dev_probe(pdev, p_sensor_ctrl);
	if (rc) {
		pr_err("%s - msm_sensor_platform_dev_probe failed %d\n",
				__func__, rc);
		goto ba_failed;
	}

	/* TO DO setup ba handler to map to actual device and video node*/
	pr_debug("avdevice platform probe exit...\n");
	return rc;

ba_failed:
	msm_ba_close(ba_instance_handler_a[pdev->id]);
	return rc;
}

static struct platform_driver avdevice_ba_platform_driver = {
	.probe = avdevice_ba_platform_probe,
	.driver = {
		.name = "avdevice",
		.owner = THIS_MODULE,
	},
};

static int __init avdevice_init_module(void)
{
	int32_t rc = 0;
	pr_debug("avdevice platform device init module...\n");

	rc = platform_driver_register(&avdevice_ba_platform_driver);
	return rc;
}

static void __exit avdevice_exit_module(void)
{
	int i = 0;
	for (i = 0; i < NUM_AV_DEVICES; i++) {
		if (avdevice_a_s_ctrl[i].pdev)
			msm_sensor_free_sensor_data(&avdevice_a_s_ctrl[i]);
	}

	platform_driver_unregister(&avdevice_ba_platform_driver);
	return;
}

static struct v4l2_subdev_core_ops avdevice_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops avdevice_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops avdevice_subdev_ops = {
	.core = &avdevice_subdev_core_ops,
	.video  = &avdevice_subdev_video_ops,
};

static void avdevice_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int device = 0;
	if (s_ctrl) {
		pr_debug("Starting av device id %p", s_ctrl);
		if (s_ctrl->pdev) {
			device = s_ctrl->pdev->id;
			pr_debug("av device pdev %p id %i",
				s_ctrl->pdev, device);
		} else {
			pr_err("%s Start pdev null", __func__);
			return;
		}
	} else {
		pr_err("%s Start sctrl null", __func__);
		return;
	}

	if (device >= NUM_AV_DEVICES) {
		pr_err("%s Stop failed invalid device id %d", __func__, device);
		return;
	}
	msm_ba_streamon(ba_instance_handler_a[device], 0);
}

static void avdevice_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int device = 0;
	pr_debug("Stop stream av device");
	if (s_ctrl) {
		pr_debug("Stopping av device id %p", s_ctrl);
		if (s_ctrl->pdev) {
			pr_debug("Stop av device pdev %p id %i",
				s_ctrl->pdev, device);
			device = s_ctrl->pdev->id;
		} else {
			pr_err("%s Stop pdev null", __func__);
			return;
		}
	} else {
		pr_err("%s Stop sctrl null", __func__);
		return;
	}

	if (device >= NUM_AV_DEVICES) {
		pr_err("%s Stop failed invalid device id %d", __func__, device);
		return;
	}
	msm_ba_streamoff(ba_instance_handler_a[device], 0);
}

static int32_t avdevice_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

static void avdevice_do_nothing_void(struct msm_sensor_ctrl_t *s_ctrl)
{
	/* do nothing! */
	pr_debug("Nothing to do!");
}

static int32_t avdevice_pwr_on(struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}
static int32_t avdevice_pwr_off(struct msm_sensor_ctrl_t *s_ctrl)
{
	return 0;
}

int32_t avdevice_set_fps(struct msm_sensor_ctrl_t *s_ctrl,
						struct fps_cfg *fps)
{
	return 0;
}

int32_t avdevice_write_settings(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	pr_debug("up type %d, res %d", update_type, res);
	return 0;
}
int32_t avdevice_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	pr_debug("Mode %d, res %d", mode, res);
	return 0;
}
int32_t avdevice_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	return 0;
}

static int32_t avdevice_get_output_info(struct msm_sensor_ctrl_t *s_ctrl,
		struct sensor_output_info_t *sensor_output_info)
{
	int32_t rc = 0;
	struct v4l2_format ba_fmt;
	struct v4l2_control ctrl;

	if (!s_ctrl || !sensor_output_info) {
		pr_err("%s - passed in null param (%p, %p)",
				__func__, s_ctrl, sensor_output_info);
		return -EINVAL;
	}


	rc = msm_ba_g_fmt(ba_instance_handler_a[s_ctrl->pdev->id], &ba_fmt);
	if (rc)	{
		pr_err("%s : msm_ba_g_fmt failed %d!\n",
				__func__, rc);
		return rc;
	}

	ctrl.id = MSM_BA_PRIV_FPS;
	rc = msm_ba_g_ctrl(ba_instance_handler_a[s_ctrl->pdev->id], &ctrl);
	if (rc) {
		pr_err("%s : msm_ba_g_ctrl failed %d!\n",
				__func__, rc);
		return rc;
	}

	s_ctrl->sensor_v4l2_subdev_info->code =
			avdevice_v4l2fmt_to_mbuspixfmt(
					ba_fmt.fmt.pix.pixelformat);
	s_ctrl->sensor_v4l2_subdev_info->colorspace =
			ba_fmt.fmt.pix.colorspace;

	s_ctrl->msm_sensor_reg->output_settings->x_output =
			ba_fmt.fmt.pix.width;
	s_ctrl->msm_sensor_reg->output_settings->y_output =
			ba_fmt.fmt.pix.height;
	s_ctrl->msm_sensor_reg->output_settings->line_length_pclk =
			ba_fmt.fmt.pix.width;
	s_ctrl->msm_sensor_reg->output_settings->frame_length_lines =
			ba_fmt.fmt.pix.height;

	s_ctrl->msm_sensor_reg->output_settings->vt_pixel_clk =
			ba_fmt.fmt.pix.width * ba_fmt.fmt.pix.height *
			(ctrl.value >> 16);

	pr_debug("%s - %dx%d @ %d", __func__, ba_fmt.fmt.pix.width,
			ba_fmt.fmt.pix.height, ctrl.value >> 16);

	return msm_sensor_get_output_info(s_ctrl, sensor_output_info);
}

static int32_t avdevice_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t gain, uint32_t line, int32_t luma_avg, uint16_t fgain)
{
	return 0;
}

static int32_t avdevice_hdr_update(struct msm_sensor_ctrl_t *s_ctrl,
	struct sensor_hdr_update_parm_t *update_parm)
{
	return 0;
}

static struct msm_sensor_fn_t avdevice_func_tbl = {
	.sensor_start_stream = avdevice_start_stream,
	.sensor_stop_stream = avdevice_stop_stream,
	.sensor_group_hold_on = avdevice_do_nothing_void,
	.sensor_group_hold_off = avdevice_do_nothing_void,
	.sensor_set_fps = avdevice_set_fps,
	.sensor_write_exp_gain = avdevice_write_exp_gain,
	.sensor_write_snapshot_exp_gain = avdevice_write_exp_gain,
	.sensor_setting = avdevice_write_settings,
	.sensor_csi_setting = avdevice_write_settings,
	.sensor_set_sensor_mode = avdevice_set_sensor_mode,
	.sensor_mode_init = avdevice_mode_init,
	.sensor_get_output_info = avdevice_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = avdevice_pwr_on,
	.sensor_power_down = avdevice_pwr_off,
	.sensor_match_id = avdevice_match_id,
	.sensor_adjust_frame_lines = avdevice_do_nothing_void,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_hdr_update = avdevice_hdr_update,
};

static struct msm_sensor_reg_t avdevice_regs = {
	.default_data_type        = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf        = 0,
	.start_stream_conf_size   = 0,
	.stop_stream_conf         = 0,
	.stop_stream_conf_size    = 0,
	.group_hold_on_conf       = 0,
	.group_hold_on_conf_size  = 0,
	.group_hold_off_conf      = 0,
	.group_hold_off_conf_size = 0,
	.init_settings            = 0,
	.init_size                = 0,
	.mode_settings            = 0,
	.output_settings          = &avdevice_dimensions[0],
	.num_conf                 = ARRAY_SIZE(avdevice_dimensions),
};

static struct msm_sensor_ctrl_t avdevice_a_s_ctrl[] = {
	{
	.msm_sensor_reg               = &avdevice_regs,
	.sensor_i2c_client            = NULL,
	.sensor_i2c_addr              = 0x0,
	.sensor_output_reg_addr       = 0,
	.sensor_id_info               = 0,
	.sensor_exp_gain_info         = 0,
	.cam_mode                     = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex             = &avdevice_mut,
	.sensor_i2c_driver            = NULL,
	.sensor_v4l2_subdev_info      = avdevice_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(avdevice_subdev_info),
	.sensor_v4l2_subdev_ops       = &avdevice_subdev_ops,
	.func_tbl                     = &avdevice_func_tbl,
	.clk_rate                     = MSM_SENSOR_MCLK_24HZ,
	},
	{
	.msm_sensor_reg               = &avdevice_regs,
	.sensor_i2c_client            = NULL,
	.sensor_i2c_addr              = 0x0,
	.sensor_output_reg_addr       = 0,
	.sensor_id_info               = 0,
	.sensor_exp_gain_info         = 0,
	.cam_mode                     = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex             = &avdevice2_mut,
	.sensor_i2c_driver            = NULL,
	.sensor_v4l2_subdev_info      = avdevice_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(avdevice_subdev_info),
	.sensor_v4l2_subdev_ops       = &avdevice_subdev_ops,
	.func_tbl                     = &avdevice_func_tbl,
	.clk_rate                     = MSM_SENSOR_MCLK_24HZ,
	},
};

module_init(avdevice_init_module);
module_exit(avdevice_exit_module);
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. avdevice sensor driver");
MODULE_LICENSE("GPL v2");
