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

#include "msm_ba_debug.h"
#include "msm_ba_common.h"

struct msm_ba_input_config msm_ba_inp_cfg[] = {
	/* type, index, name, adv inp, dev id, sd name, signal status */
	{BA_INPUT_CVBS, 0, "CVBS-0", BA_IP_CVBS_0, 0, "adv7180", 1},
	{BA_INPUT_CVBS, 1, "CVBS-1", BA_IP_CVBS_1, 0, "adv7180", 1},
	{BA_INPUT_COMPONENT, 0, "COMP-0", BA_IP_COMPONENT_0, 0, "adv7180", 1},
	{BA_INPUT_CVBS, 2, "CVBS-2", BA_IP_CVBS_0, 1, "adv7180", 1},
	{BA_INPUT_CVBS, 3, "CVBS-3", BA_IP_CVBS_1, 1, "adv7180", 1},
	{BA_INPUT_CVBS, 4, "CVBS-4", BA_IP_CVBS_2, 1, "adv7180", 1},
	{BA_INPUT_HDMI, 0, "HDMI-1", BA_IP_HDMI_1, 2, "adv7481", 1},
};

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

struct v4l2_subdev *msm_ba_sd_find(const char *name)
{
	struct v4l2_subdev *sd = NULL;
	struct v4l2_subdev *sd_out = NULL;
	struct msm_ba_dev *dev_ctxt = NULL;

	dev_ctxt = get_ba_dev();
	if (!list_empty(&(dev_ctxt->v4l2_dev.subdevs))) {
		list_for_each_entry(sd, &(dev_ctxt->v4l2_dev.subdevs), list)
			if (!strcmp(name, sd->name)) {
				sd_out = sd;
				break;
			}
	}

	return sd_out;
}

void msm_ba_add_inputs(struct v4l2_subdev *sd)
{
	struct msm_ba_input *input = NULL;
	struct msm_ba_dev *dev_ctxt = NULL;
	int i;
	int str_length = 0;
	int rc;
	int start_index = 0;
	int end_index = 0;
	int dev_id = 0;
	int status = 0;

	dev_ctxt = get_ba_dev();
	if (!list_empty(&dev_ctxt->inputs))
		start_index = dev_ctxt->num_inputs;

	dev_id = msm_ba_inp_cfg[start_index].ba_out;
	end_index = sizeof(msm_ba_inp_cfg)/sizeof(msm_ba_inp_cfg[0]);
	for (i = start_index; i < end_index; i++) {
		str_length = strlen(msm_ba_inp_cfg[i].sd_name);
		rc = memcmp(sd->name, msm_ba_inp_cfg[i].sd_name, str_length);
		if (!rc && dev_id == msm_ba_inp_cfg[i].ba_out) {
			input = kzalloc(sizeof(*input), GFP_KERNEL);

			if (!input) {
				dprintk(BA_ERR, "Failed to allocate memory");
				break;
			} else {
				memset(input, 0x00, sizeof(*input));
				input->inputType = msm_ba_inp_cfg[i].inputType;
				input->name_index = msm_ba_inp_cfg[i].index;
				strlcpy(input->name, msm_ba_inp_cfg[i].name,
					sizeof(input->name));
				input->bridge_chip_ip = msm_ba_inp_cfg[i].ba_ip;
				input->ba_out = msm_ba_inp_cfg[i].ba_out;
				input->ba_ip = i;
				input->sd = sd;
				rc = v4l2_subdev_call(
					sd, video, g_input_status, &status);
				if (rc)
					dprintk(BA_ERR,
						"g_input_status failed for sd: %s",
						sd->name);
				else
					input->signal_status = status;
				list_add_tail(&input->list, &dev_ctxt->inputs);
				dev_ctxt->num_inputs++;
			}
		}
	}
}

void msm_ba_del_inputs(struct v4l2_subdev *sd)
{
	struct msm_ba_input *input = NULL;
	struct list_head *ptr;
	struct list_head *next;
	struct msm_ba_dev *dev_ctxt = NULL;

	dev_ctxt = get_ba_dev();

	list_for_each_safe(ptr, next, &(dev_ctxt->inputs)) {
		input = list_entry(ptr, struct msm_ba_input, list);
		if (input->sd == sd) {
			list_del(&input->list);
			kfree(input);
		}
	}
}

struct msm_ba_input *msm_ba_find_input(int ba_input)
{
	struct msm_ba_input *input = NULL;
	struct msm_ba_input *input_out = NULL;
	struct msm_ba_dev *dev_ctxt = NULL;

	dev_ctxt = get_ba_dev();

	if (!list_empty(&(dev_ctxt->inputs))) {
		list_for_each_entry(input, &(dev_ctxt->inputs), list)
			if (input->ba_ip == ba_input) {
				input_out = input;
				break;
			}
	}

	return input_out;
}

struct msm_ba_input *msm_ba_find_output(int ba_output)
{
	struct msm_ba_input *input = NULL;
	struct msm_ba_input *input_out = NULL;
	struct msm_ba_dev *dev_ctxt = NULL;

	dev_ctxt = get_ba_dev();

	if (!list_empty(&(dev_ctxt->inputs))) {
		list_for_each_entry(input, &(dev_ctxt->inputs), list) {
			if (input->ba_out == ba_output) {
				input_out = input;
				break;
			}
		}
	}

	return input_out;
}
