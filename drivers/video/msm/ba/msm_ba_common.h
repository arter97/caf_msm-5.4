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

#ifndef _MSM_BA_COMMON_H_
#define _MSM_BA_COMMON_H_

#include "msm_ba_internal.h"

struct msm_ba_dev *get_ba_dev(void);
struct v4l2_subdev *msm_ba_sd_find(const char *name);
void msm_ba_add_inputs(struct v4l2_subdev *sd);
void msm_ba_del_inputs(struct v4l2_subdev *sd);
struct msm_ba_input *msm_ba_find_input(int ba_input);
struct msm_ba_input *msm_ba_find_output(int ba_output);
void msm_ba_queue_v4l2_event(struct msm_ba_inst *inst, int event_type);

#endif
