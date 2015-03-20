/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#include <media/msm_ba.h>

#include "msm_ba_internal.h"
#include "msm_ba_debug.h"
#include "msm_ba_common.h"

#define MSM_BA_DEV_NAME "msm_ba_8064"

#define BA_DRV_INPUT_CVBS_AIN1		BA_RVC_IP
#define BA_DRV_INPUT_CVBS_AIN2		BA_CVBS_IP
#define BA_DRV_INPUT_HDMI_RX		BA_HDMI_IP

#define MSM_BA_RVC_INPUT			BA_DRV_INPUT_CVBS_AIN1
#define MSM_BA_CVBS_INPUT			BA_DRV_INPUT_CVBS_AIN2
#define MSM_BA_HDMI_INPUT			BA_DRV_INPUT_HDMI_RX

#define MSM_BA_MAX_EVENTS			10

int msm_ba_poll(void *instance, struct file *filp,
		struct poll_table_struct *wait)
{
	struct msm_ba_inst *inst = instance;
	int rc = 0;

	if (!inst)
		return -EINVAL;

	poll_wait(filp, &inst->event_handler.wait, wait);
	if (v4l2_event_pending(&inst->event_handler))
		rc |= POLLPRI;

	return rc;
}
EXPORT_SYMBOL(msm_ba_poll);

int msm_ba_querycap(void *instance, struct v4l2_capability *cap)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !cap) {
		dprintk(BA_ERR,
			"Invalid input, inst = 0x%p, cap = 0x%p\n", inst, cap);
		return -EINVAL;
	}

	strlcpy(cap->driver, MSM_BA_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MSM_BA_DEV_NAME, sizeof(cap->card));
	cap->bus_info[0] = 0;
	cap->version = MSM_BA_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_STREAMING;
	memset(cap->reserved, 0x00, sizeof(cap->reserved));

	return 0;
}
EXPORT_SYMBOL(msm_ba_querycap);

int msm_ba_s_parm(void *instance, struct v4l2_streamparm *a)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !a)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_s_parm);

int msm_ba_enum_input(void *instance, struct v4l2_input *input)
{
	struct msm_ba_inst *inst = instance;
	int rc = 0;

	if (!inst || !input)
		return -EINVAL;
	if (input->index >= inst->dev_ctxt->num_inputs)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	if (MSM_BA_RVC_INPUT == input->index) {
		input->std = V4L2_STD_ALL;
		snprintf(input->name, sizeof(input->name), "CVBS%d",
					input->index);
		input->capabilities = V4L2_IN_CAP_STD;
	} else if (MSM_BA_CVBS_INPUT == input->index) {
		input->std = V4L2_STD_ALL;
		snprintf(input->name, sizeof(input->name), "CVBS%d",
					input->index);
		input->capabilities = V4L2_IN_CAP_STD;
	} else {
		input->std = 0;
		strlcpy(input->name, "HDMI0", sizeof(input->name));
		input->capabilities = V4L2_IN_CAP_CUSTOM_TIMINGS;
	}
	return rc;
}
EXPORT_SYMBOL(msm_ba_enum_input);

int msm_ba_g_input(void *instance, unsigned int *index)
{
	struct msm_ba_inst *inst = instance;
	int rc = 0;

	if (!inst || !index)
		return -EINVAL;

	*index = inst->dev_ctxt->inputs->input.index;

	return rc;
}
EXPORT_SYMBOL(msm_ba_g_input);

int msm_ba_s_input(void *instance, unsigned int index)
{
	struct msm_ba_inst *inst = instance;
	struct v4l2_subdev *sd = NULL;
	int rc = 0;

	if (!inst)
		return -EINVAL;
	if (index > inst->dev_ctxt->num_inputs)
		return -EINVAL;

	sd = inst->dev_ctxt->sd[inst->sd_id];
	if (!sd) {
		dprintk(BA_ERR, "No sd registered\n");
		return -EINVAL;
	}
	switch (index) {
	case BA_RVC_IP:
		rc = v4l2_subdev_call(sd, video, s_routing,
						MSM_BA_RVC_INPUT, 0, 0);
		msm_ba_queue_v4l2_event(inst,
			V4L2_EVENT_MSM_BA_DEVICE_AVAILABLE);
		break;
	case BA_HDMI_IP:
		rc = v4l2_subdev_call(sd, video, s_routing,
						MSM_BA_HDMI_INPUT, 0, 0);
		break;
	default:
		dprintk(BA_ERR, "(%s): Select Input: ERR %s: %d\n", __func__,
				  "Unknown input", index);
		rc = -EINVAL;
		break;
	}
	return rc;
}
EXPORT_SYMBOL(msm_ba_s_input);

int msm_ba_enum_fmt(void *instance, struct v4l2_fmtdesc *f)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !f)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_enum_fmt);

int msm_ba_s_fmt(void *instance, struct v4l2_format *f)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !f)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_s_fmt);

int msm_ba_g_fmt(void *instance, struct v4l2_format *f)
{
	struct msm_ba_inst *inst = instance;
	struct v4l2_subdev *sd = NULL;
	v4l2_std_id new_std = V4L2_STD_UNKNOWN;
	int rc = 0;

	if (!inst || !f)
		return -EINVAL;

	sd = inst->dev_ctxt->sd[inst->sd_id];
	if (!sd) {
		dprintk(BA_ERR, "No sd registered\n");
		return -EINVAL;
	}
	rc = v4l2_subdev_call(sd, video, querystd, &new_std);
	if (rc) {
		dprintk(BA_ERR, "querystd failed for sd: %s\n", sd->name);
	} else {
		inst->sd_input.std = new_std;
		switch (inst->sd_input.std) {
		case V4L2_STD_PAL:
			f->fmt.pix.height = DEFAULT_HEIGHT;
			f->fmt.pix.width = DEFAULT_WIDTH;
			f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
			break;
		case V4L2_STD_NTSC:
			f->fmt.pix.height = DEFAULT_HEIGHT;
			f->fmt.pix.width = DEFAULT_WIDTH;
			f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
			break;
		default:
			f->fmt.pix.height = DEFAULT_HEIGHT;
			f->fmt.pix.width = DEFAULT_WIDTH;
			f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
			break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(msm_ba_g_fmt);

int msm_ba_s_ctrl(void *instance, struct v4l2_control *control)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !control)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_s_ctrl);

int msm_ba_g_ctrl(void *instance, struct v4l2_control *control)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !control)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_g_ctrl);

int msm_ba_s_ext_ctrl(void *instance, struct v4l2_ext_controls *control)
{
	struct msm_ba_inst *inst = instance;

	if (!inst || !control)
		return -EINVAL;

	return -EINVAL;
}
EXPORT_SYMBOL(msm_ba_s_ext_ctrl);

int msm_ba_streamon(void *instance, enum v4l2_buf_type i)
{
	struct msm_ba_inst *inst = instance;
	struct v4l2_subdev *sd = NULL;
	int rc = 0;

	if (!inst)
		return -EINVAL;

	sd = inst->dev_ctxt->sd[inst->sd_id];
	if (!sd) {
		dprintk(BA_ERR, "No sd registered\n");
		return -EINVAL;
	}
	rc = v4l2_subdev_call(sd, video, s_stream, 1);
	if (rc)
		dprintk(BA_ERR, "streamon failed on port: %d\n", i);

	return rc;
}
EXPORT_SYMBOL(msm_ba_streamon);

int msm_ba_streamoff(void *instance, enum v4l2_buf_type i)
{
	struct msm_ba_inst *inst = instance;
	struct v4l2_subdev *sd = NULL;
	int rc = 0;

	if (!inst)
		return -EINVAL;

	sd = inst->dev_ctxt->sd[inst->sd_id];
	if (!sd) {
		dprintk(BA_ERR, "No sd registered\n");
		return -EINVAL;
	}
	rc = v4l2_subdev_call(sd, video, s_stream, 0);
	if (rc)
		dprintk(BA_ERR, "streamoff failed on port: %d\n", i);

	return rc;
}
EXPORT_SYMBOL(msm_ba_streamoff);

void msm_ba_release_subdev_node(struct video_device *vdev)
{
	struct v4l2_subdev *sd = video_get_drvdata(vdev);

	sd->devnode = NULL;
	kfree(vdev);
}

static int msm_ba_register_v4l2_subdev(struct v4l2_device *v4l2_dev,
				struct v4l2_subdev *sd)
{
	struct video_device *vdev;
	int rc = 0;

	dprintk(BA_DBG, "Enter %s: v4l2_dev 0x%p, v4l2_subdev 0x%p\n",
			  __func__, v4l2_dev, sd);
	if (NULL == v4l2_dev || NULL == sd || !sd->name[0]) {
		dprintk(BA_ERR, "Invalid input");
		return -EINVAL;
	}
	rc = v4l2_device_register_subdev(v4l2_dev, sd);
	if (rc < 0) {
		dprintk(BA_ERR,
			"%s(%d), V4L2 subdev register failed for %s rc: %d",
			__func__, __LINE__, sd->name, rc);
		return rc;
	}
	if (sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE) {
		vdev = video_device_alloc();
		if (NULL == vdev) {
			dprintk(BA_ERR, "%s Not enough memory", __func__);
			return -ENOMEM;
		}
		video_set_drvdata(vdev, sd);
		strlcpy(vdev->name, sd->name, sizeof(vdev->name));
		vdev->v4l2_dev = v4l2_dev;
		vdev->fops = &v4l2_subdev_fops;
		vdev->release = msm_ba_release_subdev_node;
		rc = __video_register_device(vdev, VFL_TYPE_SUBDEV, -1, 1,
								sd->owner);
		if (rc < 0) {
			dprintk(BA_ERR, "%s Error registering video device %s",
					__func__, sd->name);
			kfree(vdev);
		} else {
#if defined(CONFIG_MEDIA_CONTROLLER)
			sd->entity.info.v4l.major = VIDEO_MAJOR;
			sd->entity.info.v4l.minor = vdev->minor;
			sd->entity.name = video_device_node_name(vdev);
#endif
			sd->devnode = vdev;
		}
	}
	dprintk(BA_DBG, "Exit %s with rc: %d\n", __func__, rc);

	return rc;
}

int msm_ba_register_subdev_node(struct v4l2_subdev *sd,
				enum subdev_id sd_id)
{
	struct ba_ctxt *ba_ctxt;
	uint8_t sd_index;
	int rc = 0;

	ba_ctxt = msm_ba_get_ba_context();
	sd_index = sd_id;
	if (sd_index >= MSM_BA_MAX_V4L2_SUBDEV_NUM) {
		dprintk(BA_ERR, "%s Invalid sub device index %d",
			__func__, sd_index);
		return -EINVAL;
	}
	ba_ctxt->dev_ctxt->sd[sd_index] = sd;
	ba_ctxt->dev_ctxt->num_inputs = 2;
	ba_ctxt->dev_ctxt->inputs =
		kzalloc(sizeof(*ba_ctxt->dev_ctxt->inputs)*
					ba_ctxt->dev_ctxt->num_inputs,
					GFP_KERNEL);
	if (!ba_ctxt->dev_ctxt->inputs) {
		dprintk(BA_ERR, "Failed to allocate memory\n");
	} else {
		ba_ctxt->dev_ctxt->inputs->input.index = BA_RVC_IP;
		ba_ctxt->dev_ctxt->inputs->subdev_index = sd_index;
	}
	rc = msm_ba_register_v4l2_subdev(&ba_ctxt->dev_ctxt->v4l2_dev, sd);

	return rc;
}
EXPORT_SYMBOL(msm_ba_register_subdev_node);

static void __msm_ba_sd_unregister(struct v4l2_subdev *sub_dev)
{
	struct ba_ctxt *ba_ctxt;
	int i;

	ba_ctxt = msm_ba_get_ba_context();
	mutex_lock(&ba_ctxt->ba_cs);

	v4l2_device_unregister_subdev(sub_dev);
	for (i = 0; i < MSM_BA_MAX_V4L2_SUBDEV_NUM; i++) {
		if (sub_dev == ba_ctxt->dev_ctxt->sd[i]) {
			ba_ctxt->dev_ctxt->sd[i] = NULL;
			ba_ctxt->dev_ctxt->num_ba_subdevs--;
			break;
		}
	}

	dprintk(BA_DBG, "%s(%d), BA Unreg Sub Device : num ba devices %d : %s",
		__func__, __LINE__, i, sub_dev->name);

	mutex_unlock(&ba_ctxt->ba_cs);
}

int msm_ba_unregister_subdev_node(struct v4l2_subdev *sub_dev)
{
	struct ba_ctxt *ba_ctxt;

	ba_ctxt = msm_ba_get_ba_context();
	if (!ba_ctxt || !ba_ctxt->dev_ctxt)
		return -ENODEV;
	if (!sub_dev)
		return -EINVAL;
	__msm_ba_sd_unregister(sub_dev);

	return 0;
}
EXPORT_SYMBOL(msm_ba_unregister_subdev_node);

static int setup_event_queue(void *inst,
				struct video_device *pvdev)
{
	int rc = 0;
	struct msm_ba_inst *ba_inst = (struct msm_ba_inst *)inst;

	v4l2_fh_init(&ba_inst->event_handler, pvdev);
	v4l2_fh_add(&ba_inst->event_handler);

	return rc;
}

int msm_ba_subscribe_event(void *inst,
				struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_ba_inst *ba_inst = (struct msm_ba_inst *)inst;

	if (!inst || !sub)
		return -EINVAL;

	rc = v4l2_event_subscribe(&ba_inst->event_handler, sub,
						MSM_BA_MAX_EVENTS);
	return rc;
}
EXPORT_SYMBOL(msm_ba_subscribe_event);

int msm_ba_unsubscribe_event(void *inst,
				struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_ba_inst *ba_inst = (struct msm_ba_inst *)inst;

	if (!inst || !sub)
		return -EINVAL;

	rc = v4l2_event_unsubscribe(&ba_inst->event_handler, sub);
	return rc;
}
EXPORT_SYMBOL(msm_ba_unsubscribe_event);

void msm_ba_subdev_event_hndlr(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{
	dprintk(BA_INFO, "Enter %s\n", __func__);

	if (!sd || !arg)
		return;

	switch (notification) {
	default:
		break;
	}

	dprintk(BA_INFO, "Exit %s\n", __func__);
}

void *msm_ba_open(void)
{
	struct msm_ba_inst *inst = NULL;
	struct msm_ba_dev *dev_ctxt = NULL;
	enum subdev_id i;

	dev_ctxt = get_ba_dev();

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);

	if (!inst) {
		dprintk(BA_ERR, "Failed to allocate memory\n");
	} else {
		memset(inst, 0x00, sizeof(*inst));

		mutex_init(&inst->inst_cs);

		init_waitqueue_head(&inst->kernel_event_queue);
		inst->state = MSM_BA_DEV_UNINIT_DONE;
		inst->dev_ctxt = dev_ctxt;

		for (i = MSM_BA_SUBDEV_0;
				i < MSM_BA_MAX_V4L2_SUBDEV_NUM; i++) {
			if (NULL != dev_ctxt->sd[i]) {
				inst->sd_id = i;
				break;
			}
		}

		mutex_lock(&dev_ctxt->dev_cs);
		list_add_tail(&inst->list, &dev_ctxt->instances);
		mutex_unlock(&dev_ctxt->dev_cs);

		dev_ctxt->state = BA_DEV_INIT;
		dev_ctxt->state = BA_DEV_INIT_DONE;
		inst->state = MSM_BA_DEV_INIT_DONE;

		inst->debugfs_root =
			msm_ba_debugfs_init_inst(inst, dev_ctxt->debugfs_root);

		setup_event_queue(inst, dev_ctxt->vdev);
	}

	return inst;
}
EXPORT_SYMBOL(msm_ba_open);

int msm_ba_close(void *instance)
{
	struct msm_ba_inst *inst = instance;
	struct msm_ba_inst *temp;
	struct msm_ba_dev *dev_ctxt;
	struct list_head *ptr;
	struct list_head *next;
	int rc = 0;

	if (!inst)
		return -EINVAL;
	v4l2_fh_del(&inst->event_handler);

	dev_ctxt = inst->dev_ctxt;
	mutex_lock(&dev_ctxt->dev_cs);

	list_for_each_safe(ptr, next, &dev_ctxt->instances) {
		temp = list_entry(ptr, struct msm_ba_inst, list);
		if (temp == inst)
			list_del(&inst->list);
	}
	mutex_unlock(&dev_ctxt->dev_cs);

	dprintk(BA_DBG, "Closed BA instance: 0x%p\n", inst);
	kfree(inst);

	return rc;
}
EXPORT_SYMBOL(msm_ba_close);
