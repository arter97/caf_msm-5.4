/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_BA_H_
#define _MSM_BA_H_

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <linux/poll.h>

enum msm_ba_ip {
	BA_RVC_IP = 0, /* Rear View Camera */
	BA_CAM_IP,
	BA_CVBS_IP,
	BA_TTL_IP,
	BA_HDMI_IP,
	BA_MHL_IP,
	BA_NO_IP
};

enum subdev_id {
	MSM_BA_SUBDEV_0 = 0,
	MSM_BA_SUBDEV_1 = 1,
	MSM_BA_MAX_V4L2_SUBDEV_NUM
};

void *msm_ba_open(void);
int msm_ba_close(void *instance);
int msm_ba_querycap(void *instance, struct v4l2_capability *cap);
int msm_ba_enum_input(void *instance, struct v4l2_input *input);
int msm_ba_g_input(void *instance, unsigned int *index);
int msm_ba_s_input(void *instance, unsigned int index);
int msm_ba_enum_fmt(void *instance, struct v4l2_fmtdesc *f);
int msm_ba_s_fmt(void *instance, struct v4l2_format *f);
int msm_ba_g_fmt(void *instance, struct v4l2_format *f);
int msm_ba_s_ctrl(void *instance, struct v4l2_control *a);
int msm_ba_s_ext_ctrl(void *instance, struct v4l2_ext_controls *a);
int msm_ba_g_ctrl(void *instance, struct v4l2_control *a);
int msm_ba_streamon(void *instance, enum v4l2_buf_type i);
int msm_ba_streamoff(void *instance, enum v4l2_buf_type i);
int msm_ba_poll(void *instance, struct file *filp,
		struct poll_table_struct *pt);
int msm_ba_subscribe_event(void *instance,
					struct v4l2_event_subscription *sub);
int msm_ba_unsubscribe_event(void *instance,
					struct v4l2_event_subscription *sub);
int msm_ba_s_parm(void *instance, struct v4l2_streamparm *a);
int msm_ba_register_subdev_node(struct v4l2_subdev *sd,
		enum subdev_id sd_id);
int msm_ba_unregister_subdev_node(struct v4l2_subdev *sd);
#endif
