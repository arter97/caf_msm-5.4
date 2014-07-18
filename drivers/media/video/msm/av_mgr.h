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
 *
 */

#ifndef __AV_MGR_H
#define __AV_MGR_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-common.h>

#define AV_MAX_V4L2_SUBDEV_NUM   1
#define AV_MAX_MAX_ACT_STRM	 1




enum av_rvc_frmt {
	AUTO = 0,
	NTSC,
	PAL,
};

struct av_rvc_status {
	int signal_lock;
	int height;
	int width;
	int frame_rate;
};


int av_mgr_rvc_enable(int rvc_frmt);
int av_mgr_rvc_get_status(struct av_rvc_status *rvc_status);
int av_mgr_rvc_stream_enable(bool enable);

extern int av_mgr_sd_register(struct v4l2_subdev *sub_dev);
extern int av_mgr_sd_unregister(struct v4l2_subdev *sub_dev);



#endif
#endif
