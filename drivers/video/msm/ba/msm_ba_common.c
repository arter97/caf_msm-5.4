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

#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include "msm_ba_common.h"

struct msm_ba_dev *get_ba_dev(void)
{
	struct ba_ctxt *ba_ctxt;
	struct msm_ba_dev *dev_ctxt = NULL;

	ba_ctxt = msm_ba_get_ba_context();

	mutex_lock(&ba_ctxt->ba_cs);
	dev_ctxt = ba_ctxt->dev_ctxt;
	mutex_unlock(&ba_ctxt->ba_cs);

	return dev_ctxt;
}

void msm_ba_queue_v4l2_event(struct msm_ba_inst *inst, int event_type)
{
	struct v4l2_event event = {.id = 0, .type = event_type};
	v4l2_event_queue_fh(&inst->event_handler, &event);
	wake_up(&inst->kernel_event_queue);
}
