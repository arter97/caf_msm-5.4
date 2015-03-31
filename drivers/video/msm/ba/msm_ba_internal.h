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

#ifndef _MSM_BA_INTERNAL_H_
#define _MSM_BA_INTERNAL_H_

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/ocmem.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/msm_ba.h>

#define MSM_BA_DRV_NAME "msm_ba_driver"

#define MSM_BA_VERSION KERNEL_VERSION(0, 0, 1);

#define MAX_NAME_LENGTH 64

#define MAX_DEBUGFS_NAME MAX_NAME_LENGTH

#define DEFAULT_WIDTH 720
#define DEFAULT_HEIGHT 507

enum ba_dev_state {
	BA_DEV_UNINIT = 0,
	BA_DEV_LOADED,
	BA_DEV_INIT,
	BA_DEV_INIT_DONE,
	BA_DEV_INVALID
};

enum instance_state {
	MSM_BA_DEV_UNINIT_DONE = 0x0001,
	MSM_BA_DEV_INIT,
	MSM_BA_DEV_INIT_DONE,
	MSM_BA_OPEN,
	MSM_BA_OPEN_DONE,
	MSM_BA_START,
	MSM_BA_START_DONE,
	MSM_BA_STOP,
	MSM_BA_STOP_DONE,
	MSM_BA_CLOSE,
	MSM_BA_CLOSE_DONE,
	MSM_BA_DEV_UNINIT,
	MSM_BA_DEV_INVALID
};

struct ba_ctxt {

	struct mutex ba_cs;

	struct msm_ba_dev *dev_ctxt;

	struct dentry *debugfs_root;
};

enum profiling_points {
	SYS_INIT = 0,
	SESSION_INIT,
	MAX_PROFILING_POINTS
};

struct profile_data {
	int start;
	int stop;
	int cumulative;
	char name[64];
	int sampling;
	int average;
};

struct msm_ba_debug {
	struct profile_data pdata[MAX_PROFILING_POINTS];
	int profile;
	int samples;
};

struct msm_ba_dev_capability {
	u32 capability_set;
};

struct msm_ba_input {
	struct list_head list;
	struct v4l2_input input;
	uint32_t subdev_index;
	const char *subdev_name;
};

struct msm_ba_dev {
	struct mutex dev_cs;

	enum ba_dev_state state;

	struct msm_ba_input *inputs;
	uint32_t num_inputs;

	/* V4L2 Framework */
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	struct media_device mdev;

	struct list_head instances;

	/* BA v4l2 sub devs */
	struct v4l2_subdev *sd[MSM_BA_MAX_V4L2_SUBDEV_NUM];
	uint32_t num_ba_subdevs;

	struct dentry *debugfs_root;
};

struct msm_ba_inst {
	struct list_head list;
	struct mutex inst_cs;
	struct msm_ba_dev *dev_ctxt;

	struct v4l2_input sd_input;
	char sd_name[V4L2_SUBDEV_NAME_SIZE];
	enum subdev_id sd_id;
	int state;

	struct v4l2_fh event_handler;
	wait_queue_head_t kernel_event_queue;

	struct msm_ba_debug debug;
	struct dentry *debugfs_root;
};

struct ba_ctxt *msm_ba_get_ba_context(void);

void msm_ba_subdev_event_hndlr(struct v4l2_subdev *sd,
					unsigned int notification, void *arg);

#endif

