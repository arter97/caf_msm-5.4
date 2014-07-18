/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

#include "av_mgr.h"

#define AV_MGR_MODULE_NAME "av_mgr"
#define ADV7282_INPUT_CVBS_AIN1 0x0
#define ADV7282_INPUT_CVBS_AIN2 0x1
#define ADP_RCV_INPUT	ADV7282_INPUT_CVBS_AIN1
#define RCV_SELECTED	g_av_mgr->stream.input == RVC_IP
#define RCV_DEC_INIT	(g_av_mgr->rvc_decoder_init == 1)

struct av_vid_strm {
	int input;
	int index;
	int std_det;
	int streaming;
	v4l2_std_id std;
	struct v4l2_dv_timings *timings;
	struct v4l2_pix_format *format;

};

enum av_ip {
	RVC_IP = 0, /* Rear View Camera */
	CAM_IP,
	CVBS_IP,
	TTL_IP,
	HDMI_IP,
	MHL_IP,
	NO_IP
};


struct av_mgr {

	uint32_t initialized;
	struct mutex mutex;

	/* V4L2 Framework */
	struct video_device *vdev;
	struct v4l2_device v4l2_dev;

	struct v4l2_ctrl_handler ctrl_handler;

	/* AV v4l2 sub devs */
	struct v4l2_subdev *av_sd[AV_MAX_V4L2_SUBDEV_NUM];
	uint32_t num_av_subdevs;


	int rvc_decoder_init;
	struct av_vid_strm stream;


};

static struct av_mgr *g_av_mgr;


int av_mgr_rvc_decoder_init(bool enable, int rvc_frmt)
{
	int ret = 0;
	struct v4l2_subdev *sd = NULL;
	v4l2_std_id std;
	pr_debug("RVC decoder init : Initializing Decoder %d\n", rvc_frmt);
	sd = g_av_mgr->av_sd[0];
	if (!sd)
		return 1;

	if (enable)
		ret = v4l2_subdev_call(sd, core, s_power, 1);
	else
		ret = v4l2_subdev_call(sd, core, s_power, 0);

	switch (rvc_frmt) {
	case PAL:
		std = V4L2_STD_PAL;
		break;
	case NTSC:
		std = V4L2_STD_NTSC;
		break;
	case AUTO:
	default:
		std = V4L2_STD_ALL;
		break;
	}

	ret = v4l2_subdev_call(sd, core, s_std, std);

	return ret;
}

int av_mgr_select_ip(int av_input)
{
	int ret = 0;
	struct v4l2_subdev *sd = NULL;

	sd = g_av_mgr->av_sd[0];


	switch (av_input) {
	case RVC_IP:
		ret = v4l2_subdev_call(sd, video, s_routing,
						ADP_RCV_INPUT, 0, 0);
		g_av_mgr->stream.input = av_input;
		break;
	default:
		pr_err("RVC: Select Input: ERR %s:\n", "Unknown input");
		break;
	}

	if (!ret)
		g_av_mgr->stream.input = av_input;
	return ret;
}

int av_mgr_enable_op(int av_input, int enable)
{
	int ret = 0;
	struct v4l2_subdev *sd = NULL;

	sd = g_av_mgr->av_sd[0];


	switch (av_input) {
	case RVC_IP:
		ret = v4l2_subdev_call(sd, video, s_stream, enable);
		break;
	default:
		ret = v4l2_subdev_call(sd, video, s_stream, enable);
		break;
	}

	if (!ret)
		g_av_mgr->stream.streaming = enable;
	return ret;
}

int av_mgr_get_status(int av_input)
{
	int ret = 0;
	struct v4l2_subdev *sd = NULL;
	v4l2_std_id new_std = V4L2_STD_UNKNOWN;
	sd = g_av_mgr->av_sd[0];

	if (g_av_mgr->stream.input != av_input)
		return 1;

	switch (av_input) {
	case RVC_IP:
		ret = v4l2_subdev_call(sd, video, querystd, &new_std);
		if (ret) {
			g_av_mgr->stream.std_det = 0;
		} else {
			g_av_mgr->stream.std = new_std;
			g_av_mgr->stream.std_det = 1;
		}
		break;
	default:
		ret = v4l2_subdev_call(sd, video, querystd, &new_std);
		break;
	}


	return ret;
}


int av_mgr_rvc_enable(int rvc_frmt)
{
	int ret = 0;

	if (!g_av_mgr)
		return 1;
	if (!g_av_mgr->rvc_decoder_init) {
		av_mgr_rvc_decoder_init(1, rvc_frmt);
		g_av_mgr->rvc_decoder_init = 1;
	}

	if (g_av_mgr->stream.input != RVC_IP)
		av_mgr_select_ip(RVC_IP);

	return ret;
}
int av_mgr_rvc_get_status(struct av_rvc_status *rvc_status)
{
	int ret = 0;

	if (!rvc_status)
		return 1;

	if (!(av_mgr_get_status(RVC_IP))) {

		rvc_status->signal_lock = 1;
		switch (g_av_mgr->stream.std) {
		case V4L2_STD_PAL:
			rvc_status->height = 507;
			rvc_status->width = 720;
			rvc_status->frame_rate = 60;
			break;
		case V4L2_STD_NTSC:
			rvc_status->height = 507;
			rvc_status->width = 720;
			rvc_status->frame_rate = 50;
			break;
		default:
			rvc_status->height = 507;
			rvc_status->width = 720;
			rvc_status->frame_rate = 50;
			break;
		}

	} else {
		rvc_status->signal_lock = 0;
	}


	return ret;

}
int av_mgr_rvc_stream_enable(bool enable)
{
	int ret = 0;
	av_mgr_enable_op(RVC_IP, enable);
	return ret;

}

void av_mgr_sd_event_hndlr(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{

}

int av_mgr_init(void)
{
	int ret = 0;
	struct video_device *vfd = NULL;

	if (g_av_mgr)
		return -EINVAL;

	g_av_mgr = kzalloc(sizeof(struct av_mgr), GFP_KERNEL);
	if (!g_av_mgr)
		return -ENOMEM;

	mutex_init(&g_av_mgr->mutex);


	g_av_mgr->num_av_subdevs = 0;


	strlcpy(g_av_mgr->v4l2_dev.name, AV_MGR_MODULE_NAME,
				sizeof(g_av_mgr->v4l2_dev.name));
	g_av_mgr->v4l2_dev.notify = av_mgr_sd_event_hndlr;
	ret = v4l2_device_register(NULL, &g_av_mgr->v4l2_dev);
	if (ret)
		goto free_av_mgr;


	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;

	strlcpy(vfd->name, AV_MGR_MODULE_NAME, sizeof(vfd->name));
	vfd->release = video_device_release_empty;
	vfd->v4l2_dev = &g_av_mgr->v4l2_dev;
	vfd->lock = &g_av_mgr->mutex;


	g_av_mgr->vdev = vfd;


	g_av_mgr->initialized = 1;
	g_av_mgr->stream.input = NO_IP;
	pr_debug(" %s(%d), AV Manager Init Complete ",
					__func__, __LINE__);

	return ret;

unreg_dev:
	v4l2_device_unregister(&g_av_mgr->v4l2_dev);

free_av_mgr:
	kfree(g_av_mgr);
	return ret;

}

int av_mgr_exit(void)
{

	if (!g_av_mgr || !g_av_mgr->initialized)
		return -EINVAL;

	g_av_mgr->initialized = 0;

	mutex_lock(&g_av_mgr->mutex);
	if (!g_av_mgr->num_av_subdevs) {

		v4l2_info(&g_av_mgr->v4l2_dev, "unregistering %s\n",
				video_device_node_name(g_av_mgr->vdev));
		video_unregister_device(g_av_mgr->vdev);
		v4l2_device_unregister(&g_av_mgr->v4l2_dev);
		mutex_unlock(&g_av_mgr->mutex);
		mutex_destroy(&g_av_mgr->mutex);
		kfree(g_av_mgr);
		g_av_mgr = NULL;
	} else {
		mutex_unlock(&g_av_mgr->mutex);
	}

	return 0;
}
static int av_mgr_register_v4l2_subdev(struct v4l2_device *v4l2_dev,
	struct v4l2_subdev *sd)
{
	int ret = 0;

	if (v4l2_dev == NULL || sd == NULL || !sd->name[0])
		return -EINVAL;

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		pr_err(KERN_ERR
			" %s(%d), AV Manager V4L2 Device Registering failed",
			__func__, __LINE__);
		return ret;
	}

	return ret;
}


static int __av_mgr_sd_register(struct v4l2_subdev *sub_dev)
{
	int i;
	int ret = 0;


	mutex_lock(&g_av_mgr->mutex);

	for (i = 0; i < AV_MAX_V4L2_SUBDEV_NUM; i++) {
		if (!g_av_mgr->av_sd[i])
			break;
	}

	if (i == AV_MAX_V4L2_SUBDEV_NUM) {
		ret = -EBUSY;
		goto error_1;
	}

	g_av_mgr->av_sd[i] = sub_dev;
	g_av_mgr->num_av_subdevs++;

	pr_debug(" %s(%d), Registering new sub device: num av devices %d : %s",
		__func__, __LINE__, g_av_mgr->num_av_subdevs, sub_dev->name);
	ret = av_mgr_register_v4l2_subdev(&g_av_mgr->v4l2_dev, sub_dev);
	if (ret < 0)
		pr_err(KERN_ERR
		" %s(%d), AV Mgr New Sub Device Reg FAILED: num av devices %d : %s",
		__func__, __LINE__,
		g_av_mgr->num_av_subdevs, sub_dev->name);

	mutex_unlock(&g_av_mgr->mutex);


	return 0;


error_1:
	mutex_unlock(&g_av_mgr->mutex);
	pr_err(KERN_ERR
	" %s(%d), AV Mgr Failed Reg new sub device: num avdevices %d : %s",
		__func__, __LINE__,
		g_av_mgr->num_av_subdevs, sub_dev->name);
	return ret;
}

int av_mgr_sd_register(struct v4l2_subdev *sub_dev)
{
	if (!sub_dev) {
		pr_err(KERN_ERR " %s(%d),AV Manager No Subdevice provided ",
							__func__, __LINE__);
		return -EINVAL;
	}
	if (!g_av_mgr || !g_av_mgr->initialized) {
		pr_debug(" %s(%d),AV Manager ADV Manager not initialised!",
							__func__, __LINE__);
		av_mgr_init();
	}
	return __av_mgr_sd_register(sub_dev);
}
EXPORT_SYMBOL(av_mgr_sd_register);


static void __av_mgr_sd_unregister(struct v4l2_subdev *sub_dev)
{
	int i;

	mutex_lock(&g_av_mgr->mutex);

	v4l2_device_unregister_subdev(sub_dev);
	for (i = 0; i < AV_MAX_V4L2_SUBDEV_NUM; i++) {
		if (sub_dev == (struct v4l2_subdev *) g_av_mgr->av_sd[i]) {
			g_av_mgr->av_sd[i] = NULL;
			g_av_mgr->num_av_subdevs--;
			break;
		}
	}

	pr_debug(" %s(%d), AV Mgr Unreg Sub Device : num av devices %d : %s",
		__func__, __LINE__, i, sub_dev->name);

	mutex_unlock(&g_av_mgr->mutex);

}

int av_mgr_sd_unregister(struct v4l2_subdev *sub_dev)
{
	if (!g_av_mgr || !g_av_mgr->initialized)
		return -ENODEV;
	if (!sub_dev)
		return -EINVAL;
	__av_mgr_sd_unregister(sub_dev);
	return 0;
}
EXPORT_SYMBOL(av_mgr_sd_unregister);





static int __init av_mgr_mod_init(void)
{
	av_mgr_init();
	return 0;
}

static void __exit av_mgr_mod_exit(void)
{

	av_mgr_exit();
}

module_init(av_mgr_mod_init);
module_exit(av_mgr_mod_exit);
