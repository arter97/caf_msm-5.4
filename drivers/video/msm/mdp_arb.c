/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/uaccess.h>
#include "mdp_arb_internal.h"

static struct mdp_arb_device_info *arb;
static struct class *arb_class;
static int arb_major;
static struct platform_device this_device = {
	.name   = "mdp_arb",
	.id	= 0,
};
static char uevent_client_str[MDP_ARB_UEVENT_LEN];
static char uevent_event_str[MDP_ARB_UEVENT_LEN];
static char uevent_state_str[MDP_ARB_UEVENT_LEN];
static char uevent_layer_str[MDP_ARB_UEVENT_LEN];
static char uevent_fbidx_str[MDP_ARB_UEVENT_LEN];
/* Hardcode it in here until query API is available from the display driver.*/
static struct mdp_arb_pipe mdp_pipe[OVERLAY_PIPE_MAX] = {
	[OVERLAY_PIPE_VG1]
	{
		.support_by_hw = true,
		.type = OVERLAY_TYPE_VIDEO,
		.client = NULL,
	},
	[OVERLAY_PIPE_VG2]
	{
		.support_by_hw = true,
		.type = OVERLAY_TYPE_VIDEO,
		.client = NULL,
	},
	[OVERLAY_PIPE_RGB1]
	{
		.support_by_hw = true,
		.type = OVERLAY_TYPE_RGB,
		.client = NULL,
	},
	[OVERLAY_PIPE_RGB2]
	{
		.support_by_hw = true,
		.type = OVERLAY_TYPE_RGB,
		.client = NULL,
	},
	[OVERLAY_PIPE_DMAS]
	{
		.support_by_hw = true,
		.type = OVERLAY_TYPE_DMAS,
		.client = NULL,
	},
};

#define MDP_ARB_DISPLAY_ID_TERTIARY "TERTIARY"
#define COPY_EVENT_NUM(from, to) \
	(to)->event.get_event.num_of_states = \
		(from)->event.get_event.num_of_states;
#define COPY_EVENT_STATE_TO_USER(from, to) \
	copy_to_user((to)->event.get_event.value, \
		(from)->event.get_event.value, \
		sizeof(int) * (to)->event.get_event.num_of_states)
#define COPY_EVENT_STATE_TO_KERNEL(from, to) \
	memcpy((to)->event.get_event.value, \
		(from)->event.get_event.value, \
		sizeof(int) * (to)->event.get_event.num_of_states);
#define IS_CLIENT_SUPPORT_UP(client) \
	((client)->register_info.common.notification_support_mask & \
		MDP_ARB_NOTIFICATION_UP)
#define IS_CLIENT_SUPPORT_DOWN(client) \
	((client)->register_info.common.notification_support_mask & \
		MDP_ARB_NOTIFICATION_DOWN)
#define IS_CLIENT_SUPPORT_OPTIMIZE(client) \
	((client)->register_info.common.notification_support_mask & \
		MDP_ARB_NOTIFICATION_OPTIMIZE)
#define IS_EVENT_NUM_SAME(event1, event2) \
	((event1)->event.driver_register.num_of_states == \
		(event2)->event.driver_register.num_of_states)
#define IS_SAME_FB_IDX(client1, client2) \
	((client1)->register_info.common.fb_index == \
		(client2)->register_info.common.fb_index)
#define IS_TERTIARY_FB(client) \
	(strnstr((client)->display_id, MDP_ARB_DISPLAY_ID_TERTIARY, \
		MDP_ARB_NAME_LEN))
#define SYSFS_PRINT_CLIENT(c, b, l, s) { \
	l += snprintf(b + l, s - l, "name=%s,fb_idx=%d,num_of_events=%d,"\
		"priority=%d,mask=0x%x,num_of_client=%d,num_of_layer=%d,"\
		"display_id=%s\n", \
		c->register_info.common.name, \
		c->register_info.common.fb_index, \
		c->register_info.common.num_of_events, \
		c->register_info.common.priority, \
		c->register_info.common.notification_support_mask, \
		c->num_of_client, c->num_of_layer, c->display_id); \
	}
#define SYSFS_PRINT_CLIENT_EVENT(e, b, l, s) {\
	l += snprintf(b + l, s - l, "\tname=%s,num_of_up=%d,"\
		"num_of_down=%d\n", e->name, \
		e->event.register_state.num_of_up_state_value, \
		e->event.register_state.num_of_down_state_value); \
	}

static int mdp_arb_get_event_sub(struct mdp_arb_device_info *arb,
				struct mdp_arb_events *events, int user)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_event_db *temp_event;
	struct mdp_arb_event *from, *to;
	int i = 0, num = 0;
	void *backup;

	mutex_lock(&arb->dev_mutex);
	if (!events->num_of_events) {
		events->num_of_events = arb->num_of_events;
		goto out;
	}

	if (events->num_of_events > arb->num_of_events)
		events->num_of_events = arb->num_of_events;
	list_for_each(pos, &arb->event_db_list) {
		temp_event = list_entry(pos, struct mdp_arb_event_db, list);
		from = &(temp_event->event);
		to = events->event + i;
		if (!to->event.get_event.num_of_states) {
			COPY_EVENT_NUM(from, to);
		} else {
			num = to->event.get_event.num_of_states;
			if (user) {
				backup = to->event.get_event.value;
				if (copy_to_user(to, from,
					sizeof(struct mdp_arb_event))) {
					pr_err("%s events copy_to_user " \
						"event failed", __func__);
					rc = -EFAULT;
					goto out;
				}
				to->event.get_event.value = backup;
				if (COPY_EVENT_STATE_TO_USER(from, to)) {
					pr_err("%s events value copy_to_user "\
						"event failed", __func__);
					rc = -EFAULT;
					goto out;
				}
			} else {
				backup = to->event.get_event.value;
				memcpy(to, from, sizeof(struct mdp_arb_event));
				to->event.get_event.value = backup;
				COPY_EVENT_STATE_TO_KERNEL(from, to);
			}
		}
		i++;
		if (i >= events->num_of_events)
			break;
	}

out:
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static void mdp_arb_free_client(struct mdp_arb_client_db *client)
{
	int i = 0;
	struct mdp_arb_event *e = NULL;
	if (client) {
		if (client->register_info.common.event) {
			for (i = 0; i < client->register_info.common.\
				num_of_events; i++) {
				e = client->register_info.common.event + i;
				kfree(e->event.register_state.down_state_value);
				kfree(e->event.register_state.up_state_value);
				kfree(e->event.register_state.opt_state_value);
			}
			kfree(client->register_info.common.event);
		}
		kfree(client);
	}
}

static int mdp_arb_register_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_register_info *client, int user, int bind,
	void **handle)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_client_db *c = NULL;
	struct mdp_arb_event *e = NULL;
	struct mdp_arb_event *e1 = NULL;
	bool found = false;
	int idx = 0;
	int i = 0, num = 0;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	} else if (client->common.fb_index >= FB_MAX) {
		pr_err("%s fb_index=%d is out of the bound, name=%s, user=%d",
		__func__, client->common.fb_index, client->common.name, user);
		return -EINVAL;
	} else if (!strlen(client->common.name)) {
		pr_err("%s client name is empty", __func__);
		return -EINVAL;
	}

	idx = client->common.fb_index;
	mutex_lock(&arb->dev_mutex);
	/* check if there is same client */
	list_for_each(pos, &arb->client_db_list[idx]) {
		c = list_entry(pos, struct mdp_arb_client_db, list);
		if (!strcmp(c->register_info.common.name,
			client->common.name)) {
			found = true;
			break;
		}
	}
	if (!found) {
		c = kzalloc(sizeof(*c), GFP_KERNEL);
		if (!c) {
			pr_err("%s out of memory for client", __func__);
			rc = -ENOMEM;
			goto out;
		}
		memset(c, 0x00, sizeof(*c));
	} else {
		if (bind) {
			c->num_of_client++;
			*handle = c;
			goto out;
		} else {
			pr_err("%s client=%s has been registered.fb_idx=%d",
				__func__, c->register_info.common.name, idx);
			rc = -EFAULT;
		}
		goto out;
	}
	memcpy(&(c->register_info), client,
		sizeof(struct mdp_arb_client_register_info));
	if (c->register_info.common.num_of_events) {
		c->register_info.common.event = kzalloc(
			sizeof(struct mdp_arb_event)*
			c->register_info.common.num_of_events,
			GFP_KERNEL);
		if (!c->register_info.common.event) {
			pr_err("%s out of memory for event, num=%d", __func__,
				c->register_info.common.num_of_events);
			rc = -ENOMEM;
			goto out;
		}
		memset(c->register_info.common.event, 0x00,
			sizeof(struct mdp_arb_event)*
			c->register_info.common.num_of_events);
		if (user) {
			if (copy_from_user(
				c->register_info.common.event,
				client->common.event,
				sizeof(struct mdp_arb_event)*
				client->common.num_of_events)) {
				pr_err("%s register copy_from_user for event" \
				" failed", __func__);
				rc = -EFAULT;
				goto out;
			}
		} else {
			memcpy(c->register_info.common.event,
				client->common.event,
				sizeof(struct mdp_arb_event)*
				client->common.num_of_events);
		}
	}
	for (i = 0; i < c->register_info.common.num_of_events; i++) {
		e = c->register_info.common.event + i;
		e1 = client->common.event + i;
		num = e->event.register_state.num_of_down_state_value;
		if (num) {
			e->event.register_state.down_state_value = kzalloc(
				sizeof(int) * num, GFP_KERNEL);
			if (!e->event.register_state.down_state_value) {
				pr_err("%s out of memory for down value num=%d",
					__func__, num);
				rc = -ENOMEM;
				goto out;
			}
			memset(e->event.register_state.down_state_value, 0x00,
				sizeof(int) * num);
			if (user) {
				if (copy_from_user(
					e->event.register_state.\
						down_state_value,
					e1->event.register_state.\
						down_state_value,
					sizeof(int) * num)) {
					pr_err("%s register copy_from_user for"\
						" down failed", __func__);
					rc = -EFAULT;
					goto out;
				}
			} else {
				memcpy(e->event.register_state.down_state_value,
					e1->event.register_state.\
						down_state_value,
					sizeof(int) * num);
			}
		}
		num = e->event.register_state.num_of_up_state_value;
		if (num) {
			e->event.register_state.up_state_value = kzalloc(
				sizeof(int) * num, GFP_KERNEL);
			if (!e->event.register_state.up_state_value) {
				pr_err("%s out of memory for up value num=%d",
					__func__, num);
				rc = -ENOMEM;
				goto out;
			}
			memset(e->event.register_state.up_state_value, 0x00,
				sizeof(int) * num);
			if (user) {
				if (copy_from_user(
					e->event.register_state.up_state_value,
					e1->event.register_state.up_state_value,
					sizeof(int) * num)) {
					pr_err("%s register copy_from_user for"\
						" up failed", __func__);
					rc = -EFAULT;
					goto out;
				}
			} else {
				memcpy(e->event.register_state.up_state_value,
					e1->event.register_state.up_state_value,
					sizeof(int) * num);
			}
		}
		num = e->event.register_state.num_of_opt_state_value;
		if (num) {
			e->event.register_state.opt_state_value = kzalloc(
				sizeof(int) * num, GFP_KERNEL);
			if (!e->event.register_state.opt_state_value) {
				pr_err("%s out of memory for opt value num=%d",
					__func__, num);
				rc = -ENOMEM;
				goto out;
			}
			memset(e->event.register_state.opt_state_value, 0x00,
				sizeof(int) * num);
			if (user) {
				if (copy_from_user(
					e->event.register_state.opt_state_value,
					e1->event.register_state.\
					opt_state_value, sizeof(int) * num)) {
					pr_err("%s register copy_from_user for"\
						" opt failed", __func__);
					rc = -EFAULT;
					goto out;
				}
			} else {
				memcpy(e->event.register_state.opt_state_value,
					e1->event.register_state.\
					opt_state_value, sizeof(int) * num);
			}
		}
	}
	rc = mdpclient_msm_fb_get_id(c->register_info.common.fb_index,
		c->display_id, MDP_ARB_NAME_LEN);
	if (rc) {
		pr_err("%s fb_get_id error=%d, fb_idx=%d", __func__, rc,
			c->register_info.common.fb_index);
		goto out;
	}
	init_completion(&c->ack_comp);
	list_add_tail(&c->list, &arb->client_db_list[idx]);
	arb->num_of_clients[idx]++;
	c->num_of_client++;
	*handle = c;

out:
	if (!found && rc)
		mdp_arb_free_client(c);
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_deregister_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, int user)
{
	int rc = 0;
	struct mdp_arb_client_register_info *client = &client_db->register_info;
	struct list_head *pos, *q;
	struct mdp_arb_client_db *temp_client = NULL;
	bool found = false;
	int idx = 0;

	if (!strlen(client->common.name)) {
		pr_err("%s client name is empty", __func__);
		return -EINVAL;
	} else if (client->common.fb_index >= FB_MAX) {
		pr_err("%s fb_index=%d is out of the bound, name=%s, user=%d",
		__func__, client->common.fb_index, client->common.name, user);
		return -EINVAL;
	}
	idx = client->common.fb_index;
	mutex_lock(&arb->dev_mutex);
	/* check if there is same client */
	list_for_each_safe(pos, q, &arb->client_db_list[idx]) {
		temp_client = list_entry(pos,
			struct mdp_arb_client_db, list);
		if (!strcmp(temp_client->register_info.common.name,
			client->common.name)) {
			found = true;
			if (temp_client->num_of_client > 1) {
				pr_debug("%s still have client binded to it"\
					" num_of_client=%d", __func__,
					temp_client->num_of_client);
				temp_client->num_of_client--;
			} else {
				list_del(pos);
				kfree(temp_client->register_info.common.event);
				kfree(temp_client);
			}
			break;
		}
	}
	if (!found) {
		pr_err("%s can't find client[%s] from db", __func__,
			client->common.name);
		rc = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_acknowledge_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, int mask, int user)
{
	int rc = 0;
	struct mdp_arb_client_register_info *client = &client_db->register_info;

	if (!strlen(client->common.name)) {
		pr_err("%s client name is empty", __func__);
		return -EINVAL;
	} else if (client->common.fb_index >= FB_MAX) {
		pr_err("%s fb_index=%d is out of the bound, name=%s, user=%d",
		__func__, client->common.fb_index, client->common.name, user);
		return -EINVAL;
	}

	mutex_lock(&arb->dev_mutex);
	if (!client_db->ack) {
		client_db->ack = true;
		complete_all(&client_db->ack_comp);
	}
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_get_state_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, struct mdp_arb_event *event,
	int user)
{
	int rc = 0;
	struct mdp_arb_client_register_info *client = &client_db->register_info;
	struct list_head *pos;
	struct mdp_arb_event_db *temp_event;
	bool found = false;

	if (!strlen(client->common.name)) {
		pr_err("%s client name is empty", __func__);
		return -EINVAL;
	} else if (client->common.fb_index >= FB_MAX) {
		pr_err("%s fb_index=%d is out of the bound, name=%s, user=%d",
		__func__, client->common.fb_index, client->common.name, user);
		return -EINVAL;
	}

	mutex_lock(&arb->dev_mutex);
	list_for_each(pos, &arb->event_db_list) {
		temp_event = list_entry(pos, struct mdp_arb_event_db, list);
		if (!strcmp(temp_event->event.name, event->name)) {
			event->event.get_state = temp_event->cur_state_value;
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("%s can't find state for event=%s", __func__,
			event->name);
		rc = -EINVAL;
	}

	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_ioctl_get_event(struct mdp_arb_device_info *arb,
					struct file *file, void __user *p)
{
	int rc = 0;
	struct mdp_arb_events get_event;

	if (copy_from_user(&get_event, p, sizeof(get_event))) {
		pr_err("%s get_event copy_from_user failed", __func__);
		return -EFAULT;
	}

	rc = mdp_arb_get_event_sub(arb, &get_event, true);
	if (rc) {
		pr_err("%s get_event_sub fails", __func__);
	} else if (copy_to_user(p, &get_event, sizeof(get_event))) {
		pr_err("%s get_event copy_to_user failed", __func__);
		return -EFAULT;
	}
	return rc;
}

static int mdp_arb_ioctl_register(struct mdp_arb_device_info *arb,
				struct file *file, void __user *p, int bind)
{
	int rc = 0;
	struct mdp_arb_client_register_info arb_register;
	void *handle = NULL;

	if (!file) {
		pr_err("%s file is NULL", __func__);
		return -EINVAL;
	} else if (file->private_data) {
		pr_err("%s private_data is not NULL, client called register",
			__func__);
		return -EINVAL;
	}
	memset(&arb_register, 0x00, sizeof(arb_register));
	if (copy_from_user(&arb_register.common, p,
				sizeof(struct mdp_arb_register))) {
		pr_err("%s register copy_from_user failed", __func__);
		return -EFAULT;
	}

	rc = mdp_arb_register_sub(arb, &arb_register, true, bind, &handle);
	if (rc) {
		pr_err("%s register_sub fails", __func__);
	} else if (!handle) {
		pr_err("%s register_sub succeed, but handle is NULL!",
			__func__);
		rc = -EFAULT;
	} else {
		file->private_data = handle;
	}

	return rc;
}

static int mdp_arb_ioctl_deregister(struct mdp_arb_device_info *arb,
					struct file *file, void __user *p)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db;

	if (!file) {
		pr_err("%s file is NULL", __func__);
		return -EINVAL;
	} else if (!file->private_data) {
		pr_err("%s private_data in file is NULL", __func__);
		return -EINVAL;
	}
	client_db = file->private_data;

	rc = mdp_arb_deregister_sub(arb, client_db, true);
	if (rc)
		pr_err("%s register_sub fails", __func__);
	else
		file->private_data = NULL;

	return rc;
}

static int mdp_arb_ioctl_acknowledge(struct mdp_arb_device_info *arb,
					struct file *file, void __user *p)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db;
	int mask = 0;

	if (!file) {
		pr_err("%s file is NULL", __func__);
		return -EINVAL;
	} else if (!file->private_data) {
		pr_err("%s private_data in file is NULL", __func__);
		return -EINVAL;
	}
	client_db = file->private_data;
	if (copy_from_user(&mask, p, sizeof(mask))) {
		pr_err("%s acknowledge copy_from_user failed", __func__);
		return -EFAULT;
	}
	rc = mdp_arb_acknowledge_sub(arb, client_db, mask, true);
	if (rc)
		pr_err("%s register_sub fails", __func__);

	return rc;
}

static int mdp_arb_ioctl_get_state(struct mdp_arb_device_info *arb,
					struct file *file, void __user *p)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db;
	struct mdp_arb_event event;

	if (!file) {
		pr_err("%s file is NULL", __func__);
		return -EINVAL;
	} else if (!file->private_data) {
		pr_err("%s private_data in file is NULL", __func__);
		return -EINVAL;
	}
	client_db = file->private_data;
	if (copy_from_user(&event, p, sizeof(event))) {
		pr_err("%s acknowledge copy_from_user failed", __func__);
		return -EFAULT;
	}

	rc = mdp_arb_get_state_sub(arb, client_db, &event, true);
	if (rc) {
		pr_err("%s get_state fails", __func__);
		return rc;
	}

	if (copy_to_user(p, &event, sizeof(event))) {
		pr_err("%s get_state copy_to_user failed", __func__);
		return -EFAULT;
	}

	return rc;
}

static int mdp_arb_trigger_optimize_cb(struct mdp_arb_device_info *arb,
	struct mdp_arb_notify_list *notify, bool wait)
{
	int rc = 0, cb_rc = 0, comp_rc = 0;
	struct list_head *pos;
	struct mdp_arb_notify_info *temp = NULL;
	mdp_arb_cb cb = NULL;
	struct mdp_arb_cb_info info;
	bool uevent = false;
	char *envp[6];
	int env_offset = 0;
	unsigned long timeout = msecs_to_jiffies(MDP_ARB_ACK_TIMEOUT_MS);

	/* First triggers optimize list */
	if (!list_empty(&notify->optimize_list)) {
		memset(uevent_client_str, 0x00, sizeof(uevent_client_str));
		memset(uevent_event_str, 0x00, sizeof(uevent_event_str));
		memset(uevent_state_str, 0x00, sizeof(uevent_state_str));
		memset(uevent_layer_str, 0x00, sizeof(uevent_layer_str));
		memset(uevent_fbidx_str, 0x00, sizeof(uevent_layer_str));
		strlcpy(uevent_client_str, MDP_ARB_OPTIMIZE_EVENT_STR,
			MDP_ARB_UEVENT_LEN);
		strlcpy(uevent_layer_str, "layer=", MDP_ARB_UEVENT_LEN);
		strlcpy(uevent_fbidx_str, "fb_idx=", MDP_ARB_UEVENT_LEN);
	}
	list_for_each(pos, &notify->optimize_list) {
		temp = list_entry(pos, struct mdp_arb_notify_info,
					list);
		if (!temp->client) {
			pr_err("%s temp->client is NULL", __func__);
			rc = -EFAULT;
			goto out;
		}
		if (temp->client->register_info.cb) {
			/* Kernel clients */
			cb = temp->client->register_info.cb;
			memset(&info, 0x00, sizeof(info));
			info.event = MDP_ARB_NOTIFICATION_OPTIMIZE;
			info.info.optimize.max_num_of_layers =
				temp->num_of_layers - 1;
			strlcpy(info.info.optimize.common.client_name,
				temp->client->register_info.common.name,
				MDP_ARB_NAME_LEN);
			info.info.optimize.common.fb_index =
				temp->client->register_info.common.fb_index;
			if (temp->event) {
				strlcpy(info.info.optimize.common.client_name,
					temp->event->event.name,
					MDP_ARB_NAME_LEN);
				info.info.optimize.common.value =
					temp->event->cur_state_value;
			}
			temp->client->ack = false;
			cb_rc = cb((void *)temp->client, &info, 0);
			if (cb_rc)
				pr_warning("%s cb returns error=%d for " \
					"client %s", __func__, cb_rc,
					info.info.optimize.common.client_name);
		} else {
			/* User clients */
			snprintf(uevent_client_str+strlen(uevent_client_str),
				MDP_ARB_UEVENT_LEN-strlen(uevent_client_str),
				"%s,", temp->client->register_info.common.name);
			snprintf(uevent_fbidx_str+strlen(uevent_fbidx_str),
				MDP_ARB_UEVENT_LEN-strlen(uevent_fbidx_str),
				"%d,",
				temp->client->register_info.common.fb_index);
			snprintf(uevent_layer_str+strlen(uevent_layer_str),
				MDP_ARB_UEVENT_LEN-strlen(uevent_layer_str),
				"%d,", temp->num_of_layers);
			if (temp->event && !strlen(uevent_event_str))
				snprintf(uevent_event_str, MDP_ARB_UEVENT_LEN,
				"event=%s", temp->event->event.name);
			if (temp->event && !strlen(uevent_state_str))
				snprintf(uevent_state_str, MDP_ARB_UEVENT_LEN,
				"state=%d", temp->event->cur_state_value);
			if (!uevent)
				uevent = true;
			temp->client->ack = false;
		}
	}
	if (uevent) {
		envp[env_offset++] = uevent_client_str;
		envp[env_offset++] = uevent_fbidx_str;
		envp[env_offset++] = uevent_event_str;
		envp[env_offset++] = uevent_state_str;
		envp[env_offset++] = uevent_layer_str;
		envp[env_offset] = NULL;
		mutex_lock(&arb->dev_mutex);
		kobject_uevent_env(&arb->arb_dev->kobj, KOBJ_CHANGE, envp);
		mutex_unlock(&arb->dev_mutex);
	}
	if (wait) {
		list_for_each(pos, &notify->optimize_list) {
			temp = list_entry(pos, struct mdp_arb_notify_info,
						list);
			if (!temp->client->ack) {
				/* Wait for client's acknowledgement */
				INIT_COMPLETION(temp->client->ack_comp);
				comp_rc = wait_for_completion_killable_timeout(
					&temp->client->ack_comp, timeout);
				if (comp_rc <= 0) {
					pr_err("%s waiting ack for client=%s " \
					"timeout or killed! timeout=%lu, rc=%d",
					__func__, temp->client->register_info.\
					common.name, timeout, comp_rc);
					temp->client->ack = true;
				}
			}
		}
	}
out:
	return rc;
}

static int mdp_arb_trigger_common_cb(struct mdp_arb_device_info *arb,
	struct mdp_arb_notify_list *notify, int event, bool wait)
{
	int rc = 0, cb_rc = 0, comp_rc = 0;
	struct list_head *pos;
	struct mdp_arb_notify_info *temp = NULL;
	mdp_arb_cb cb = NULL;
	struct mdp_arb_cb_info info;
	bool uevent = false;
	char *envp[5];
	int env_offset = 0;
	struct mdp_arb_notification *notification = NULL;
	struct list_head *head = NULL;
	unsigned long timeout = msecs_to_jiffies(MDP_ARB_ACK_TIMEOUT_MS);
	bool is_up = false, init = false;

	switch (event) {
	case MDP_ARB_NOTIFICATION_DOWN:
		head = &notify->down_list;
		notification = &info.info.down;
		break;
	case MDP_ARB_NOTIFICATION_UP:
		head = &notify->up_list;
		notification = &info.info.up;
		is_up = true;
		break;
	default:
		pr_err("%s unsupported event=%d", __func__, event);
		rc = -EINVAL;
		goto out;
	}

	list_for_each(pos, head) {
		temp = list_entry(pos, struct mdp_arb_notify_info, list);
		if (!temp->client) {
			pr_err("%s temp->client is NULL", __func__);
			rc = -EFAULT;
			goto out;
		}
		if (temp->client->register_info.cb) {
			/* Kernel clients */
			cb = temp->client->register_info.cb;
			memset(&info, 0x00, sizeof(info));
			info.event = event;
			strlcpy(notification->client_name,
				temp->client->register_info.common.name,
				MDP_ARB_NAME_LEN);
			notification->fb_index =
				temp->client->register_info.common.fb_index;
			if (temp->event) {
				strlcpy(notification->client_name,
					temp->event->event.name,
					MDP_ARB_NAME_LEN);
				notification->value =
					temp->event->cur_state_value;
			}
			temp->client->ack = false;
			cb_rc = cb((void *)temp->client, &info, 0);
			if (cb_rc)
				pr_warning("%s cb returns error=%d for " \
					"client %s", __func__, cb_rc,
					notification->client_name);
		} else {
			/* User clients */
			if (!init) {
				memset(uevent_client_str, 0x00,
					sizeof(uevent_client_str));
				memset(uevent_fbidx_str, 0x00,
					sizeof(uevent_layer_str));
				memset(uevent_event_str, 0x00,
					sizeof(uevent_event_str));
				memset(uevent_state_str, 0x00,
					sizeof(uevent_state_str));
				if (is_up)
					strlcpy(uevent_client_str,
						MDP_ARB_UP_EVENT_STR,
						MDP_ARB_UEVENT_LEN);
				else
					strlcpy(uevent_client_str,
						MDP_ARB_DOWN_EVENT_STR,
						MDP_ARB_UEVENT_LEN);
				strlcpy(uevent_fbidx_str, "fb_idx=",
					MDP_ARB_UEVENT_LEN);
				init = true;
			}
			snprintf(uevent_client_str+strlen(uevent_client_str),
				MDP_ARB_UEVENT_LEN-strlen(uevent_client_str),
				"%s,", temp->client->register_info.common.name);
			snprintf(uevent_fbidx_str+strlen(uevent_fbidx_str),
				MDP_ARB_UEVENT_LEN-strlen(uevent_fbidx_str),
				"%d,",
				temp->client->register_info.common.fb_index);
			if (temp->event && !strlen(uevent_event_str))
				snprintf(uevent_event_str, MDP_ARB_UEVENT_LEN,
				"event=%s", temp->event->event.name);
			if (temp->event && !strlen(uevent_state_str))
				snprintf(uevent_state_str, MDP_ARB_UEVENT_LEN,
				"state=%d", temp->event->cur_state_value);
			if (!uevent)
				uevent = true;
			temp->client->ack = false;
		}
	}
	if (uevent) {
		envp[env_offset++] = uevent_client_str;
		envp[env_offset++] = uevent_fbidx_str;
		envp[env_offset++] = uevent_event_str;
		envp[env_offset++] = uevent_state_str;
		envp[env_offset] = NULL;
		mutex_lock(&arb->dev_mutex);
		kobject_uevent_env(&arb->arb_dev->kobj, KOBJ_CHANGE, envp);
		mutex_unlock(&arb->dev_mutex);
	}
	if (wait) {
		list_for_each(pos, head) {
			temp = list_entry(pos, struct mdp_arb_notify_info,
						list);
			if (!temp->client->ack) {
				/* Wait for client's acknowledgement */
				INIT_COMPLETION(temp->client->ack_comp);
				comp_rc = wait_for_completion_killable_timeout(
					&temp->client->ack_comp, timeout);
				if (comp_rc <= 0) {
					pr_err("%s waiting ack for client=%s " \
					"timeout or killed! timeout=%lu, rc=%d"\
					" event=0x%x fb_idx=%d",
					__func__, temp->client->register_info.\
					common.name, timeout, comp_rc, event,
					temp->client->register_info.common.\
					fb_index);
					temp->client->ack = true;
				}
			}
		}
	}
out:
	return rc;
}

static int mdp_arb_trigger_cb(struct mdp_arb_device_info *arb,
	struct mdp_arb_notify_list *notify, bool wait)
{
	int rc = 0;

	/* First triggers optimize list */
	if (!list_empty(&notify->optimize_list)) {
		rc = mdp_arb_trigger_optimize_cb(arb, notify, wait);
		if (rc) {
			pr_err("%s trigger optimize cb fails=%d", __func__, rc);
			goto out;
		}
	}
	if (!list_empty(&notify->down_list)) {
		rc = mdp_arb_trigger_common_cb(arb, notify,
			MDP_ARB_NOTIFICATION_DOWN, wait);
		if (rc) {
			pr_err("%s trigger down cb fails=%d", __func__, rc);
			goto out;
		}
	}
	if (!list_empty(&notify->up_list)) {
		rc = mdp_arb_trigger_common_cb(arb, notify,
			MDP_ARB_NOTIFICATION_UP, wait);
		if (rc) {
			pr_err("%s trigger up cb fails=%d", __func__, rc);
			goto out;
		}
	}
out:
	return rc;
}

static int mdp_arb_add_to_notify_list(struct mdp_arb_client_db *client,
	struct mdp_arb_event_db *event, struct list_head *list)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_notify_info *temp = NULL;
	bool found = false;

	list_for_each(pos, list) {
		temp = list_entry(pos, struct mdp_arb_notify_info, list);
		if (!strcmp(client->register_info.common.name,
			temp->client->register_info.common.name) &&
			client->register_info.common.fb_index ==
			temp->client->register_info.common.fb_index) {
			found = true;
			break;
		}
	}
	if (!found) {
		temp = kzalloc(sizeof(*temp), GFP_KERNEL);
		if (!temp) {
			pr_err("%s out of memory for client", __func__);
			return -ENOMEM;
		}
		memset(temp, 0x00, sizeof(*temp));
		temp->client = client;
		temp->num_of_layers = 1;
		temp->event = event;
		list_add_tail(&temp->list, list);
	}

	return rc;
}

static void mdp_arb_clear_notify_list(struct mdp_arb_notify_list *notify)
{
	struct list_head *pos, *q;
	struct mdp_arb_notify_info *temp;

	if (notify) {
		list_for_each_safe(pos, q, &notify->optimize_list) {
			temp = list_entry(pos, struct mdp_arb_notify_info,
				list);
			list_del(pos);
			kfree(temp);
		}
		list_for_each_safe(pos, q, &notify->down_list) {
			temp = list_entry(pos, struct mdp_arb_notify_info,
				list);
			list_del(pos);
			kfree(temp);
		}
		list_for_each_safe(pos, q, &notify->up_list) {
			temp = list_entry(pos, struct mdp_arb_notify_info,
				list);
			list_del(pos);
			kfree(temp);
		}
	}
}

static bool mdp_arb_client_support_notify(struct mdp_arb_event *event,
	int state, int notify)
{
	bool support = false;
	int i = 0, num = 0;
	int *state_list = NULL;

	switch (notify) {
	case MDP_ARB_NOTIFICATION_DOWN:
		num = event->event.register_state.num_of_down_state_value;
		state_list = event->event.register_state.down_state_value;
		break;
	case MDP_ARB_NOTIFICATION_UP:
		num = event->event.register_state.num_of_up_state_value;
		state_list = event->event.register_state.up_state_value;
		break;
	case MDP_ARB_NOTIFICATION_OPTIMIZE:
		num = event->event.register_state.num_of_opt_state_value;
		state_list = event->event.register_state.opt_state_value;
		break;
	default:
		pr_err("%s doesn't support notify=%d, event=%s", __func__,
			notify, event->name);
		goto out;
	}

	for (i = 0; i < num; i++) {
		if (state == state_list[i]) {
			support = true;
			break;
		}
	}

out:
	return support;
}

static int mdp_arb_event_create_notify_list(struct mdp_arb_device_info *arb,
	struct mdp_arb_event_db *event, struct mdp_arb_notify_list *notify)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_client_db *client = NULL;
	struct mdp_arb_register *c = NULL;
	struct mdp_arb_event *e = NULL;
	struct list_head *list = NULL;
	int i = 0, j = 0;
	bool found = false;

	for (i = 0; i < FB_MAX; i++) {
		list_for_each(pos, &(arb->client_db_list[i])) {
			client = list_entry(pos, struct mdp_arb_client_db,
				list);
			c = &client->register_info.common;
			found = false;
			for (j = 0; j < c->num_of_events; j++) {
				e = c->event + j;
				if (!strcmp(e->name, event->event.name)) {
					if (mdp_arb_client_support_notify(e,
					event->cur_state_value,
					MDP_ARB_NOTIFICATION_OPTIMIZE)) {
						found = true;
						list = &notify->optimize_list;
					} else if (
						mdp_arb_client_support_notify(e,
						event->cur_state_value,
						MDP_ARB_NOTIFICATION_DOWN)) {
						found = true;
						list = &notify->down_list;
					} else if (
						mdp_arb_client_support_notify(e,
						event->cur_state_value,
						MDP_ARB_NOTIFICATION_UP)) {
						found = true;
						list = &notify->up_list;
					}
					break;
				}
			}
			if (found) {
				rc = mdp_arb_add_to_notify_list(client, event,
					list);
				if (rc) {
					pr_err("%s add_to_notify_list fails=%d"\
						" client=%s, event=%s,%d",
						__func__, rc, c->name,
						event->event.name,
						event->cur_state_value);
					goto out;
				}
			}
		}
	}

out:
	return rc;
}

static void mdp_arb_event_work(struct work_struct *work)
{
	int rc = 0;
	struct list_head *pos, *q;
	struct mdp_arb_event_db *temp = NULL;
	struct mdp_arb_notify_list notify;

	if (!arb) {
		pr_err("%s arb is NULL", __func__);
		return;
	}

	mutex_lock(&arb->dev_mutex);
	memset(&notify, 0x00, sizeof(notify));
	INIT_LIST_HEAD(&notify.optimize_list);
	INIT_LIST_HEAD(&notify.down_list);
	INIT_LIST_HEAD(&notify.up_list);
	list_for_each_safe(pos, q, &arb->event_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		rc = mdp_arb_event_create_notify_list(arb, temp, &notify);
		if (rc) {
			pr_err("%s create_notify_list fails=%d, event=%s,%d",
				__func__, rc, temp->event.name,
				temp->cur_state_value);
			goto out;
		}
		mutex_unlock(&arb->dev_mutex);

		rc = mdp_arb_trigger_cb(arb, &notify, true);
		mutex_lock(&arb->dev_mutex);
		if (rc) {
			pr_err("%s trigger_cb fails=%d", __func__, rc);
			goto out;
		}
		mdp_arb_clear_notify_list(&notify);
		memset(&notify, 0x00, sizeof(notify));
		INIT_LIST_HEAD(&notify.optimize_list);
		INIT_LIST_HEAD(&notify.down_list);
		INIT_LIST_HEAD(&notify.up_list);
		list_del(pos);
		kfree(temp);
	}

out:
	mdp_arb_clear_notify_list(&notify);
	mutex_unlock(&arb->dev_mutex);
	return;
}

static int mdp_arb_add_event(struct mdp_arb_device_info *arb,
	struct mdp_arb_event *event)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_event_db *temp = NULL;
	bool found = false;
	int num = 0;
	int *value = NULL;

	mutex_lock(&arb->dev_mutex);
	/* check if there is same event */
	list_for_each(pos, &arb->event_db_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		if (!strcmp(temp->event.name, event->name)) {
			found = true;
			break;
		}
	}
	if (!found) {
		temp = kzalloc(sizeof(*temp), GFP_KERNEL);
		if (!temp) {
			pr_err("%s out of memory for event=%s",
			__func__, event->name);
			rc = -EFAULT;
			goto out;
		}
		memset(temp, 0x00, sizeof(*temp));
	} else if (!IS_EVENT_NUM_SAME(&(temp->event), event)) {
		kfree(temp->event.event.driver_register.value);
		temp->event.event.driver_register.value = NULL;
		temp->event.event.driver_register.num_of_states = 0;
	}
	num = event->event.driver_register.num_of_states;
	value = temp->event.event.driver_register.value;
	if (!value) {
		value = kzalloc(sizeof(int) * num, GFP_KERNEL);
		if (!value) {
			pr_err("%s out of memory event(%s) value(%d)",
				__func__, event->name, num);
			if (!found)
				kfree(temp);
			goto out;
		}
		memset(value, 0x00, sizeof(int) * num);
	}
	memcpy(&(temp->event), event, sizeof(*event));
	memcpy(value, event->event.driver_register.value,
		sizeof(int) * num);
	temp->event.event.driver_register.value = value;
	if (!found) {
		list_add_tail(&temp->list, &arb->event_db_list);
		arb->num_of_events++;
	}

out:
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_remove_event(struct mdp_arb_device_info *arb,
	struct mdp_arb_event *event)
{
	int rc = 0;
	struct list_head *pos, *q;
	struct mdp_arb_event_db *temp = NULL;
	bool found = false;

	mutex_lock(&arb->dev_mutex);
	/* check if there is same event */
	list_for_each_safe(pos, q, &arb->event_db_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		if (!strcmp(temp->event.name, event->name)) {
			found = true;
			list_del(pos);
			kfree(temp->event.event.driver_register.value);
			kfree(temp);
			if (arb->num_of_events)
				arb->num_of_events--;
			break;
		}
	}
	if (!found) {
		pr_err("%s can't find event=%s", __func__, event->name);
		rc = -EFAULT;
	}

	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_set_event_state(struct mdp_arb_device_info *arb,
	struct mdp_arb_event *event)
{
	int rc = 0;
	struct list_head *pos;
	struct mdp_arb_event_db *temp = NULL, *event_in_list = NULL;
	bool found = false;

	mutex_lock(&arb->dev_mutex);
	/* check if there is same event */
	list_for_each(pos, &arb->event_db_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		if (!strcmp(temp->event.name, event->name)) {
			found = true;
			event_in_list = temp;
			break;
		}
	}
	if (!found) {
		pr_err("%s can't find this event=%s", __func__, event->name);
		rc = -EFAULT;
		goto out;
	}

	found = false;
	list_for_each(pos, &arb->event_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		if (!strcmp(temp->event.name, event->name)) {
			found = true;
			event_in_list->cur_state_value =
				event->event.driver_set_event;
			break;
		}
	}
	if (!found) {
		temp = kzalloc(sizeof(*temp), GFP_KERNEL);
		if (!temp) {
			pr_err("%s out of memory", __func__);
			goto out;
		}
		memset(temp, 0x00, sizeof(*temp));
		memcpy(&temp->event, event, sizeof(*event));
		temp->cur_state_value = event->event.driver_set_event;
		list_add_tail(&temp->list, &arb->event_list);
		queue_work(arb->event_queue, &arb->event_work);
	}
out:
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

#ifdef CONFIG_FB_MSM_OVERLAY
static int mdp_arb_find_new_pipe(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client, struct fb_info *fb,
	struct mdp_overlay *req, bool *found,
	struct mdp_arb_notify_list *notify)
{
	int rc = 0;
	int ptype = 0;
	int i = 0;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fb->par;

	mutex_lock(&arb->dev_mutex);
	/* This condition check needs to be revisited when query API is
	 * available from the mdp driver */
	if (arb->num_of_pipes != OVERLAY_PIPE_MAX) {
		pr_err("%s num_of_pipes=%d is wrong",
			__func__, arb->num_of_pipes);
		goto out;
	} else if (!mfd) {
		pr_err("%s mfd is NULL", __func__);
		goto out;
	}
	*found = false;
	if (req->src.format == MDP_RGB_BORDERFILL) {
		*found = true;
		goto out;
	}
	if (mfd->panel_info.pdest == DISPLAY_4) {
		if (arb->pipe[OVERLAY_PIPE_DMAS].support_by_hw) {
			*found = true;
		} else {
			pr_err("%s DMAS is not supported by HW, panle_type=%d",
				__func__, mfd->panel_info.type);
			rc = -EINVAL;
		}
		goto out;
	}
	ptype = mdp4_overlay_format2type(req->src.format);
	if (ptype < 0) {
		pr_err("%s: unsupported format=%d", __func__, req->src.format);
		rc = -EFAULT;
		goto out;
	}
	if (req->flags & MDP_OV_PIPE_SHARE)
		ptype = OVERLAY_TYPE_VIDEO;
	for (i = 0; i < OVERLAY_PIPE_DMAS; i++) {
		/* VG pipe supports both RGB+YUV */
		if (((arb->pipe[i].type == ptype) ||
			(ptype == OVERLAY_TYPE_RGB &&
			arb->pipe[i].type == OVERLAY_TYPE_VIDEO)) &&
			(arb->pipe[i].support_by_hw)) {
			if (!arb->pipe[i].client) {
				*found = true;
				goto out;
			}
		}
	}
	/* Check for optimize list with same fb idx */
	for (i = 0; i < OVERLAY_PIPE_DMAS; i++) {
		/* VG pipe supports both RGB+YUV */
		if (((arb->pipe[i].type == ptype) ||
			(ptype == OVERLAY_TYPE_RGB &&
			arb->pipe[i].type == OVERLAY_TYPE_VIDEO)) &&
			arb->pipe[i].support_by_hw &&
			!arb->pipe[i].client->notified &&
			IS_SAME_FB_IDX(arb->pipe[i].client, client)) {
			/*
			 * Even though client only has one layer at this moment,
			 * it may acquire more later. We need to send
			 * notification for client to enter optimization mode.
			 */
			if (IS_CLIENT_SUPPORT_OPTIMIZE(
				arb->pipe[i].client)) {
				rc = mdp_arb_add_to_notify_list(
					arb->pipe[i].client, NULL,
					&notify->optimize_list);
				if (rc)
					pr_err("%s add to optimize notify list"\
						" fails=%d, client=%s,fb=%d",
						__func__, rc,
						arb->pipe[i].client->\
						register_info.common.name,
						arb->pipe[i].client->\
						register_info.common.fb_index);
				else
					arb->pipe[i].client->notified = true;
				goto out;
			}
		}
	}
	/* Check for optimize list with different fb idx */
	for (i = 0; i < OVERLAY_PIPE_DMAS; i++) {
		/* VG pipe supports both RGB+YUV */
		if (((arb->pipe[i].type == ptype) ||
			(ptype == OVERLAY_TYPE_RGB &&
			arb->pipe[i].type == OVERLAY_TYPE_VIDEO)) &&
			arb->pipe[i].support_by_hw &&
			!arb->pipe[i].client->notified &&
			!IS_SAME_FB_IDX(arb->pipe[i].client, client) &&
			!IS_TERTIARY_FB(arb->pipe[i].client)) {
			if (IS_CLIENT_SUPPORT_OPTIMIZE(
				arb->pipe[i].client)) {
				rc = mdp_arb_add_to_notify_list(
					arb->pipe[i].client, NULL,
					&notify->optimize_list);
				if (rc)
					pr_err("%s add to optimize notify list"\
						" fails=%d, client=%s,fb=%d",
						__func__, rc,
						arb->pipe[i].client->\
						register_info.common.name,
						arb->pipe[i].client->\
						register_info.common.fb_index);
				else
					arb->pipe[i].client->notified = true;
				goto out;
			}
		}
	}
	/* Check for down list with same fb idx */
	for (i = 0; i < OVERLAY_PIPE_DMAS; i++) {
		/* VG pipe supports both RGB+YUV */
		if (((arb->pipe[i].type == ptype) ||
			(ptype == OVERLAY_TYPE_RGB &&
			arb->pipe[i].type == OVERLAY_TYPE_VIDEO)) &&
			arb->pipe[i].support_by_hw &&
			IS_SAME_FB_IDX(arb->pipe[i].client, client)) {
			if (client->register_info.common.priority >
				arb->pipe[i].client->register_info.\
				common.priority) {
				rc = mdp_arb_add_to_notify_list(
					arb->pipe[i].client, NULL,
					&notify->down_list);
				if (rc)
					pr_err("%s add to optimize notify list"\
						" fails=%d, client=%s,fb=%d",
						__func__, rc,
						arb->pipe[i].client->\
						register_info.common.name,
						arb->pipe[i].client->\
						register_info.common.fb_index);
				else
					arb->pipe[i].client->notified = true;
				goto out;
			}
		}
	}
	/* Check for down list with different fb idx */
	for (i = 0; i < OVERLAY_PIPE_DMAS; i++) {
		/* VG pipe supports both RGB+YUV */
		if (((arb->pipe[i].type == ptype) ||
			(ptype == OVERLAY_TYPE_RGB &&
			arb->pipe[i].type == OVERLAY_TYPE_VIDEO)) &&
			arb->pipe[i].support_by_hw &&
			!IS_SAME_FB_IDX(arb->pipe[i].client, client) &&
			!IS_TERTIARY_FB(arb->pipe[i].client)) {
			if (client->register_info.common.priority >
				arb->pipe[i].client->register_info.\
				common.priority) {
				rc = mdp_arb_add_to_notify_list(
					arb->pipe[i].client, NULL,
					&notify->down_list);
				if (rc)
					pr_err("%s add to optimize notify list"\
						" fails=%d, client=%s,fb=%d",
						__func__, rc,
						arb->pipe[i].client->\
						register_info.common.name,
						arb->pipe[i].client->\
						register_info.common.fb_index);
				else
					arb->pipe[i].client->notified = true;
				goto out;
			}
		}
	}

	pr_debug("%s can't find supported pipe, format=%d,w=%d,h=%d",
		__func__, req->src.format, req->src.width,
		req->src.height);

out:
	mutex_unlock(&arb->dev_mutex);
	return rc;
}

static int mdp_arb_overlay_set_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, struct fb_info *fb, void *p,
	int user)
{
	int rc = 0;
	int fb_idx = client_db->register_info.common.fb_index;
	struct mdp_overlay req;
	int pipe_id = 0;
	bool found = false, loop = false;
	struct mdp_arb_notify_list notify;
	struct mdp4_overlay_pipe *pipe = NULL;
	int cmd = MSMFB_OVERLAY_SET;
	struct list_head *pos;
	struct mdp_arb_client_db *client = NULL;
	int i = 0;

	if (!fb->par) {
		pr_err("%s fb->par is NULL, fb idx=%d", __func__, fb_idx);
		return -EINVAL;
	} else if (!p) {
		pr_err("%s p is NULL, fb idx=%d", __func__, fb_idx);
		return -EINVAL;
	}

	if (user) {
		if (copy_from_user(&req, p, sizeof(req))) {
			pr_err("%s copy_from_user fails", __func__);
			return -EFAULT;
		}
	} else {
		memcpy(&req, p, sizeof(req));
	}

	pipe_id = req.id;
	memset(&notify, 0x00, sizeof(notify));
	INIT_LIST_HEAD(&notify.optimize_list);
	INIT_LIST_HEAD(&notify.down_list);
	INIT_LIST_HEAD(&notify.up_list);
	if (pipe_id == MSMFB_NEW_REQUEST) {
		do {
			loop = false;
			rc = mdp_arb_find_new_pipe(arb, client_db, fb, &req,
				&found, &notify);
			if (rc) {
				pr_err("%s find_new_pipe falils=%d", __func__,
					rc);
				goto out;
			}
			if (!found) {
				if (!list_empty(&notify.optimize_list) ||
					!list_empty(&notify.down_list))
					loop = true;
				else
					break;
				rc = mdp_arb_trigger_cb(arb, &notify, true);
				if (rc) {
					pr_err("%s trigger_cb fails=%d",
						__func__, rc);
					goto out;
				}
				mdp_arb_clear_notify_list(&notify);
				memset(&notify, 0x00, sizeof(notify));
				INIT_LIST_HEAD(&notify.optimize_list);
				INIT_LIST_HEAD(&notify.down_list);
				INIT_LIST_HEAD(&notify.up_list);
			} else {
				goto call_fb;
			}
		} while (loop);
		if (!loop) {
			pr_err("%s can't find overlay resource for client=%s"\
				" id=%d, fb_idx=%d", __func__,
				client_db->register_info.common.name, pipe_id,
				client_db->register_info.common.fb_index);
			rc = -EFAULT;
			goto out;
		}
	} else {
		goto call_fb;
	}

call_fb:
	rc = msm_fb_overlay_ioctl(fb, cmd, (unsigned long)p, user);
	if (rc) {
		pr_err("%s msm_fb_overlay_ioctl overlay_set fails=%d client=%s"\
			" id=%d, fb_idx=%d", __func__, rc,
			client_db->register_info.common.name, pipe_id,
			client_db->register_info.common.fb_index);
		goto out;
	} else {
		if (user) {
			if (copy_from_user(&req, p, sizeof(req))) {
				pr_err("%s copy_from_user fails", __func__);
				goto out;
			}
		} else {
			memcpy(&req, p, sizeof(req));
		}
		pipe = mdp4_overlay_ndx2pipe(req.id);
		if (!pipe) {
			pr_err("%s pipe is NULL after set, id=%d", __func__,
				req.id);
			goto out;
		} else if (pipe->pipe_num >= OVERLAY_PIPE_MAX) {
			pr_err("%s pipe num=%d is out of bound=%d, id=%d",
				__func__, pipe->pipe_num, OVERLAY_PIPE_MAX,
				req.id);
			goto out;
		} else {
			if ((pipe_id == MSMFB_NEW_REQUEST) &&
				(req.src.format != MDP_RGB_BORDERFILL)) {
				mutex_lock(&arb->dev_mutex);
				arb->pipe[pipe->pipe_num].client = client_db;
				client_db->num_of_layer++;
				mutex_unlock(&arb->dev_mutex);
			}
		}
	}

out:
	/* Clear the notified flag */
	for (i = 0; i < FB_MAX; i++) {
		list_for_each(pos, &(arb->client_db_list[i])) {
			client = list_entry(pos, struct mdp_arb_client_db,
				list);
			client->notified = false;
		}
	}
	mdp_arb_clear_notify_list(&notify);
	return rc;
}

static int mdp_arb_overlay_unset_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, struct fb_info *fb, void *p,
	int user)
{
	int rc = 0;
	int fb_idx = client_db->register_info.common.fb_index;
	int ndx = 0;
	struct mdp4_overlay_pipe *pipe = NULL;
	int cmd = MSMFB_OVERLAY_UNSET;

	if (!fb->par) {
		pr_err("%s fb->par is NULL, fb idx=%d", __func__, fb_idx);
		return -EINVAL;
	} else if (!p) {
		pr_err("%s p is NULL, fb idx=%d", __func__, fb_idx);
		return -EINVAL;
	}

	if (user) {
		if (copy_from_user(&ndx, p, sizeof(ndx))) {
			pr_err("%s copy_from_user fails", __func__);
			return -EFAULT;
		}
	} else {
		memcpy(&ndx, p, sizeof(ndx));
	}
	pipe = mdp4_overlay_ndx2pipe(ndx);
	if (!pipe) {
		pr_err("%s pipe is NULL after set, id=%d", __func__,
			ndx);
		goto out;
	} else if (pipe->pipe_num >= OVERLAY_PIPE_MAX) {
		pr_err("%s pipe num=%d is out of bound=%d, id=%d",
			__func__, pipe->pipe_num, OVERLAY_PIPE_MAX,
			ndx);
		goto out;
	} else {
		rc = msm_fb_overlay_ioctl(fb, cmd, (unsigned long)p, user);
		if (rc)
			pr_err("%s msm_fb_overlay_ioctl overlay_unset fails=%d"\
				" client=%s, id=%d fb_idx=%d", __func__, rc,
				client_db->register_info.common.name, ndx,
				client_db->register_info.common.fb_index);
		else {
			mutex_lock(&arb->dev_mutex);
			if (!arb->pipe[pipe->pipe_num].client)
				pr_warning("%s pipe=%d calling unset, but "\
					"hasn't set yet!", __func__,
					pipe->pipe_num);
			arb->pipe[pipe->pipe_num].client = NULL;
			if (!client_db->num_of_layer)
				pr_warning("%s num_of_layer is 0 already",
					__func__);
			if ((client_db->num_of_layer) &&
				(pipe->pipe_type != OVERLAY_TYPE_BF))
				client_db->num_of_layer--;
			mutex_unlock(&arb->dev_mutex);
		}
	}

out:
	return rc;
}

static int mdp_arb_overlay_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int rc = 0;
	void *argp = (void *)arg;
	struct mdp_arb_client_db *client_db = NULL;
	int fb_idx = 0;

	if (!file) {
		pr_err("%s file is NULL", __func__);
		return -EINVAL;
	} else if (!file->private_data) {
		pr_err("%s private_data in file is NULL", __func__);
		return -EINVAL;
	}
	client_db = file->private_data;
	fb_idx = client_db->register_info.common.fb_index;
	if (fb_idx >= FB_MAX) {
		pr_err("%s fb_idx=%d is out of bound", __func__, fb_idx);
		return -EINVAL;
	} else if (!registered_fb[fb_idx]) {
		pr_err("%s fb_idx=%d is not registerd", __func__, fb_idx);
		return -EFAULT;
	}

	switch (cmd) {
	case MSMFB_OVERLAY_GET:
	case MSMFB_OVERLAY_PLAY_ENABLE:
	case MSMFB_OVERLAY_PLAY_WAIT:
	case MSMFB_OVERLAY_BLT:
	case MSMFB_OVERLAY_3D:
	case MSMFB_OVERLAY_PLAY:
	case MSMFB_DISPLAY_COMMIT:
		rc = msm_fb_overlay_ioctl(registered_fb[fb_idx], cmd, arg,
			true);
		if (rc)
			pr_err("%s msm_fb_overlay_ioctl fails=%d, cmd=0x%x"\
				" client=%s, fb_idx=%d", __func__, rc, cmd,
				client_db->register_info.common.name, fb_idx);
		break;
	case MSMFB_OVERLAY_SET:
		rc = mdp_arb_overlay_set_sub(arb, client_db,
			registered_fb[fb_idx], argp, true);
		if (rc)
			pr_err("%s overlay_set_sub fails=%d, cmd=0x%x"\
				" client=%s, fb_idx=%d", __func__, rc, cmd,
				client_db->register_info.common.name, fb_idx);
		break;
	case MSMFB_OVERLAY_UNSET:
		rc = mdp_arb_overlay_unset_sub(arb, client_db,
			registered_fb[fb_idx], argp, true);
		if (rc)
			pr_err("%s overlay_set_sub fails=%d, cmd=0x%x"\
				" client=%s, fb_idx=%d", __func__, rc, cmd,
				client_db->register_info.common.name, fb_idx);
		break;
	default:
		pr_err("unsupported ioctl=0x%08x from mdp arb driver", cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}
#else
static int mdp_arb_overlay_set_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, struct fb_info *fb, void *p,
	int user)
{
	pr_err("overlay set is not supported!");
	return -ENXIO;
}

static int mdp_arb_overlay_unset_sub(struct mdp_arb_device_info *arb,
	struct mdp_arb_client_db *client_db, struct fb_info *fb, void *p,
	int user)
{
	pr_err("overlay unset is not supported!");
	return -ENXIO;
}

static int mdp_arb_overlay_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	pr_err("overlay ioctls are not supported! cmd=0x%08x", cmd);
	return -ENXIO;
}
#endif

static int mdp_arb_do_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int rc = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case MSMFB_ARB_GET_EVENT:
		rc = mdp_arb_ioctl_get_event(arb, file, argp);
		break;
	case MSMFB_ARB_REGISTER:
		rc = mdp_arb_ioctl_register(arb, file, argp, false);
		break;
	case MSMFB_ARB_BIND:
		rc = mdp_arb_ioctl_register(arb, file, argp, true);
		break;
	case MSMFB_ARB_DEREGISTER:
	case MSMFB_ARB_UNBIND:
		rc = mdp_arb_ioctl_deregister(arb, file, argp);
		break;
	case MSMFB_ARB_ACKNOWLEDGE:
		rc = mdp_arb_ioctl_acknowledge(arb, file, argp);
		break;
	case MSMFB_ARB_GET_STATE:
		rc = mdp_arb_ioctl_get_state(arb, file, argp);
		break;
	default:
		rc = mdp_arb_overlay_ioctl(file, cmd, arg);
		break;
	}

	return rc;
}

static long mdp_arb_fops_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	long rc = 0;

	if (!arb) {
		pr_err("arb is not invalid, arb probe has error");
		rc = -ENXIO;
	} else {
		rc = mdp_arb_do_ioctl(file, cmd, arg);
	}

	return rc;
}


static const struct file_operations arb_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	mdp_arb_fops_ioctl,
};

int mdp_arb_client_get_event(struct mdp_arb_events *events)
{
	int rc = 0;

	if (!events) {
		pr_err("%s events is NULL", __func__);
		return -EINVAL;
	}

	rc = mdp_arb_get_event_sub(arb, events, false);
	if (rc)
		pr_err("%s get_event_sub fails", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_get_event);

int mdp_arb_client_register(struct mdp_arb_client_register_info *info,
	void **handle)
{
	int rc = 0;

	if (!info || !handle) {
		pr_err("%s info=0x%08x or handle=0x%08x is NULL", __func__,
			(int)info, (int)handle);
		return -EINVAL;
	}

	rc = mdp_arb_register_sub(arb, info, false, false, handle);
	if (rc)
		pr_err("%s register_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_register);

int mdp_arb_client_deregister(void *handle)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	}

	rc = mdp_arb_deregister_sub(arb, client_db, false);
	if (rc)
		pr_err("%s register_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_deregister);

int mdp_arb_client_bind(struct mdp_arb_bind *info, void **handle)
{
	int rc = 0;
	struct mdp_arb_client_register_info client;

	if (!info || !handle) {
		pr_err("%s info=0x%08x or handle=0x%08x is NULL", __func__,
			(int)info, (int)handle);
		return -EINVAL;
	}

	memset(&client, 0x00, sizeof(client));
	strlcpy(client.common.name, info->name, MDP_ARB_NAME_LEN);
	client.common.fb_index = info->fb_index;
	rc = mdp_arb_register_sub(arb, &client, false, true, handle);
	if (rc)
		pr_err("%s register_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_bind);

int mdp_arb_client_unbind(void *handle)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	}

	rc = mdp_arb_deregister_sub(arb, client_db, false);
	if (rc)
		pr_err("%s register_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_unbind);

int mdp_arb_client_acknowledge(void *handle, int notification_mask)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	}

	rc = mdp_arb_acknowledge_sub(arb, client_db, notification_mask,
					false);
	if (rc)
		pr_err("%s register_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_acknowledge);

int mdp_arb_client_get_state(void *handle, struct mdp_arb_event *event)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	} else if (!event) {
		pr_err("%s event is NULL", __func__);
		return -EINVAL;
	}

	rc = mdp_arb_get_state_sub(arb, client_db, event, false);
	if (rc)
		pr_err("%s get_state_sub falis", __func__);

	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_get_state);

int mdp_arb_client_overlay_set(void *handle, struct mdp_overlay *req)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;
	int fb_idx = 0;

	if (!handle || !req) {
		pr_err("%s handle=0x%08x or req=0x%08x is NULL", __func__,
			(int)handle, (int)req);
		return -EINVAL;
	}

	fb_idx = client_db->register_info.common.fb_index;
	if (fb_idx >= FB_MAX) {
		pr_err("%s fb_idx=%d is out of bound", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	} else if (!registered_fb[fb_idx]) {
		pr_err("%s fb_idx=%d is not registerd", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	}

	rc = mdp_arb_overlay_set_sub(arb, client_db, registered_fb[fb_idx], req,
					false);
	if (rc)
		pr_err("%s overlay_set_sub fails=%d", __func__, rc);

out:
	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_overlay_set);

int mdp_arb_client_overlay_unset(void *handle, unsigned int id)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;
	int fb_idx = 0;

	if (!handle) {
		pr_err("%s handle is NULL", __func__);
		return -EINVAL;
	}

	fb_idx = client_db->register_info.common.fb_index;
	if (fb_idx >= FB_MAX) {
		pr_err("%s fb_idx=%d is out of bound", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	} else if (!registered_fb[fb_idx]) {
		pr_err("%s fb_idx=%d is not registerd", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	}

	rc = mdp_arb_overlay_unset_sub(arb, client_db, registered_fb[fb_idx],
					&id, false);
	if (rc)
		pr_err("%s overlay_set_sub fails=%d, id=%d", __func__, rc, id);

out:
	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_overlay_unset);

int mdp_arb_client_overlay_play(void *handle, struct msmfb_overlay_data *data)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;
	int fb_idx = 0;
	int cmd = MSMFB_OVERLAY_PLAY;

	if (!handle || !data) {
		pr_err("%s handle=0x%08x or data=0x%08x is NULL", __func__,
			(int)handle, (int)data);
		return -EINVAL;
	}

	fb_idx = client_db->register_info.common.fb_index;
	if (fb_idx >= FB_MAX) {
		pr_err("%s fb_idx=%d is out of bound", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	} else if (!registered_fb[fb_idx]) {
		pr_err("%s fb_idx=%d is not registerd", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	}

	rc = msm_fb_overlay_ioctl(registered_fb[fb_idx], cmd,
					(unsigned long)data, false);
	if (rc)
		pr_err("%s msm_fb_overlay_ioctl fails=%d, cmd=0x%x, id=%d",
			__func__, rc, cmd, data->id);

out:
	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_overlay_play);

int mdp_arb_client_overlay_commit(void *handle, struct mdp_display_commit *data)
{
	int rc = 0;
	struct mdp_arb_client_db *client_db =
		(struct mdp_arb_client_db *)handle;
	int fb_idx = 0;
	int cmd = MSMFB_DISPLAY_COMMIT;

	if (!handle || !data) {
		pr_err("%s handle=0x%08x or data=0x%08x is NULL", __func__,
			(int)handle, (int)data);
		return -EINVAL;
	}

	fb_idx = client_db->register_info.common.fb_index;
	if (fb_idx >= FB_MAX) {
		pr_err("%s fb_idx=%d is out of bound", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	} else if (!registered_fb[fb_idx]) {
		pr_err("%s fb_idx=%d is not registerd", __func__, fb_idx);
		rc = -EINVAL;
		goto out;
	}

	rc = msm_fb_overlay_ioctl(registered_fb[fb_idx], cmd,
					(unsigned long)data, false);
	if (rc)
		pr_err("%s msm_fb_overlay_ioctl fails=%d, cmd=0x%x, wait=%d",
			__func__, rc, cmd, data->wait_for_finish);

out:
	return rc;
}
EXPORT_SYMBOL(mdp_arb_client_overlay_commit);

int mdp_arb_event_register(struct mdp_arb_events *events)
{
	int rc = 0;
	int i = 0;
	struct mdp_arb_event *event = NULL;

	if (!events) {
		pr_err("%s events is NULL", __func__);
		return -EINVAL;
	}

	if (!arb) {
		pr_err("%s mdp arb hasn't probed yet!", __func__);
		return -EPROBE_DEFER;
	}

	for (i = 0; i < events->num_of_events; i++) {
		event = events->event + i;
		rc = mdp_arb_add_event(arb, event);
		if (rc) {
			pr_err("%s add_event fails=%d for event=%s, i=%d",
				__func__, rc, event->name, i);
			rc = -EPROBE_DEFER;
			break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(mdp_arb_event_register);

int mdp_arb_event_deregister(struct mdp_arb_events *events)
{
	int rc = 0;
	int i = 0;
	struct mdp_arb_event *event = NULL;

	if (!events) {
		pr_err("%s events is NULL", __func__);
		return -EINVAL;
	}
	if (!arb) {
		pr_err("%s mdp arb hasn't probed yet!", __func__);
		return -EINVAL;
	}

	for (i = 0; i < events->num_of_events; i++) {
		event = events->event + i;
		rc = mdp_arb_remove_event(arb, event);
		if (rc) {
			pr_err("%s remove_event fails=%d for event=%s, i=%d",
				__func__, rc, event->name, i);
			break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(mdp_arb_event_deregister);

int mdp_arb_event_set(struct mdp_arb_event *event)
{
	int rc = 0;

	if (!event) {
		pr_err("%s events is NULL", __func__);
		return -EINVAL;
	}
	rc = mdp_arb_set_event_state(arb, event);
	if (rc)
		pr_err("%s set_event fails=%d, event=%s",
			__func__, rc, event->name);
	return rc;
}
EXPORT_SYMBOL(mdp_arb_event_set);


static ssize_t mdp_arb_rda_clients_attr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct list_head *pos;
	struct mdp_arb_client_db *temp;
	struct mdp_arb_event *e;
	int i = 0, j = 0, k = 0;
	int num = 0, up_num = 0, down_num = 0, opt_num = 0;
	int *v = NULL;

	if (!arb) {
		pr_err("%s arb is NULL", __func__);
		return 0;
	}
	mutex_lock(&arb->dev_mutex);
	len = snprintf(buf, PAGE_SIZE, "Clients:\n");
	for (i = 0; i < FB_MAX; i++) {
		if (!arb->num_of_clients[i])
			continue;
		len += snprintf(buf + len, PAGE_SIZE - len, "idx=%d:\n",
			i);
		if (len >= PAGE_SIZE)
			goto out;
		list_for_each(pos, &arb->client_db_list[i]) {
			temp = list_entry(pos, struct mdp_arb_client_db, list);
			SYSFS_PRINT_CLIENT(temp, buf, len, PAGE_SIZE);
			if (len >= PAGE_SIZE)
				goto out;
			num = temp->register_info.common.num_of_events;
			for (j = 0; j < num; j++) {
				e = temp->register_info.common.event + j;
				SYSFS_PRINT_CLIENT_EVENT(e, buf, len,
					PAGE_SIZE);
				if (len >= PAGE_SIZE)
					goto out;
				up_num = e->event.register_state.\
					num_of_up_state_value;
				down_num = e->event.register_state.\
					num_of_down_state_value;
				opt_num = e->event.register_state.\
					num_of_opt_state_value;
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\t\tup=");
				if (len >= PAGE_SIZE)
					goto out;
				v = e->event.register_state.up_state_value;
				for (k = 0; k < up_num; k++) {
					len += snprintf(buf + len,
						PAGE_SIZE - len, "%d,", v[k]);
					if (len >= PAGE_SIZE)
						goto out;
				}
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\n");
				if (len >= PAGE_SIZE)
					goto out;
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\t\tdown=");
				if (len >= PAGE_SIZE)
					goto out;
				v = e->event.register_state.down_state_value ;
				for (k = 0; k < down_num; k++) {
					len += snprintf(buf + len,
						PAGE_SIZE - len, "%d,", v[k]);
					if (len >= PAGE_SIZE)
						goto out;
				}
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\n");
				if (len >= PAGE_SIZE)
					goto out;
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\t\topt=");
				if (len >= PAGE_SIZE)
					goto out;
				v = e->event.register_state.opt_state_value;
				for (k = 0; k < opt_num; k++) {
					len += snprintf(buf + len,
						PAGE_SIZE - len, "%d,", v[k]);
					if (len >= PAGE_SIZE)
						goto out;
				}
				len += snprintf(buf + len, PAGE_SIZE - len,
					"\n");
				if (len >= PAGE_SIZE)
					goto out;
			}
		}
	}
out:
	mutex_unlock(&arb->dev_mutex);
	return len;
}

static ssize_t mdp_arb_rda_events_attr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct list_head *pos;
	struct mdp_arb_event_db *temp;
	int i = 0, num = 0;
	int *v = NULL;

	if (!arb) {
		pr_err("%s arb is NULL", __func__);
		return 0;
	}
	mutex_lock(&arb->dev_mutex);
	len = snprintf(buf, PAGE_SIZE, "Events:\n");
	list_for_each(pos, &arb->event_db_list) {
		temp = list_entry(pos, struct mdp_arb_event_db, list);
		len += snprintf(buf + len, PAGE_SIZE - len,
			"name=%s,num_of_state=%d,cur_state=%d\n",
			temp->event.name,
			temp->event.event.driver_register.num_of_states,
			temp->cur_state_value);
		if (len >= PAGE_SIZE)
			goto out;
		num = temp->event.event.driver_register.num_of_states;
		v = temp->event.event.driver_register.value;
		len += snprintf(buf + len, PAGE_SIZE - len, "\tvalue=");
		if (len >= PAGE_SIZE)
			goto out;
		for (i = 0; i < num; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len, "%d,",
				v[i]);
			if (len >= PAGE_SIZE)
				goto out;
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
		if (len >= PAGE_SIZE)
			goto out;
	}
out:
	mutex_unlock(&arb->dev_mutex);
	return len;
}

static ssize_t mdp_arb_rda_layers_attr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct mdp_arb_pipe *p;
	int i = 0;

	if (!arb) {
		pr_err("%s arb is NULL", __func__);
		return 0;
	}
	mutex_lock(&arb->dev_mutex);
	len = snprintf(buf, PAGE_SIZE, "Layers:\n");
	for (i = 0; i < arb->num_of_pipes; i++) {
		p = arb->pipe + i;
		len += snprintf(buf + len, PAGE_SIZE - len,
			"id=%d,type=%d,support=%d,client=%s,fb=%d\n",
			i, p->type, p->support_by_hw,
			(p->client) ? (p->client->register_info.common.name) : \
			("NULL"),
			(p->client) ? \
			(p->client->register_info.common.fb_index) : \
			0);
	}
	mutex_unlock(&arb->dev_mutex);
	return len;
}

static DEVICE_ATTR(clients, S_IRUGO, mdp_arb_rda_clients_attr, NULL);
static DEVICE_ATTR(events, S_IRUGO, mdp_arb_rda_events_attr, NULL);
static DEVICE_ATTR(layers, S_IRUGO, mdp_arb_rda_layers_attr, NULL);
static struct attribute *mdp_arb_attrs[] = {
	&dev_attr_clients.attr,
	&dev_attr_events.attr,
	&dev_attr_layers.attr,
	NULL,
};
static struct attribute_group mdp_arb_attr_group = {
	.attrs = mdp_arb_attrs,
};

static void mdp_arb_init_pipe(struct mdp_arb_device_info *arb)
{
	/* Init pipe structure, need to read from fb driver when API is
	 * available*/
	arb->num_of_pipes = OVERLAY_PIPE_MAX;
	arb->pipe = mdp_pipe;
}

static void mdp_arb_release(struct mdp_arb_device_info *arb)
{
	struct list_head *pos, *q;
	struct mdp_arb_client_db *temp_client;
	struct mdp_arb_event_db *temp_event;
	int i = 0;

	if (arb) {
		mutex_lock(&arb->dev_mutex);
		if (arb->event_queue)
			destroy_workqueue(arb->event_queue);
		list_for_each_safe(pos, q, &arb->event_list) {
			temp_event = list_entry(pos,
				struct mdp_arb_event_db, list);
			list_del(pos);
			kfree(temp_event);
		}
		list_for_each_safe(pos, q, &arb->event_db_list) {
			temp_event = list_entry(pos,
				struct mdp_arb_event_db, list);
			list_del(pos);
			kfree(temp_event->event.event.get_event.value);
			kfree(temp_event);
		}
		for (i = 0; i < FB_MAX; i++)
			list_for_each_safe(pos, q,
					&arb->client_db_list[i]) {
				temp_client = list_entry(pos,
					struct mdp_arb_client_db, list);
				list_del(pos);
				mdp_arb_free_client(temp_client);
			}
		if (arb->sysfs_created)
			sysfs_remove_group(&arb->arb_dev->kobj,
				&mdp_arb_attr_group);
		if (arb->arb_dev) {
			device_destroy(arb_class, MKDEV(arb_major, 0));
			arb->arb_dev = NULL;
		}
		mutex_unlock(&arb->dev_mutex);
		mutex_destroy(&arb->dev_mutex);
		kfree(arb);
		arb = NULL;
	}
}

static int __devinit mdp_arb_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i = 0;
	struct mdp_arb_device_info *mdp_arb = NULL;

	mdp_arb = kzalloc(sizeof(struct mdp_arb_device_info), GFP_KERNEL);
	if (!mdp_arb) {
		pr_err("%s kzalloc error", __func__);
		return -ENOMEM;
	}

	memset(mdp_arb, 0x00, sizeof(*mdp_arb));
	mutex_init(&mdp_arb->dev_mutex);
	mutex_lock(&mdp_arb->dev_mutex);

	mdp_arb->arb_dev = device_create(arb_class, NULL,
				MKDEV(arb_major, 0), NULL, "mdp_arb");
	if (IS_ERR_OR_NULL(mdp_arb->arb_dev)) {
		pr_err("%s unable to create device for mdp arb; errno = %ld",
			__func__, PTR_ERR(mdp_arb->arb_dev));
		mdp_arb->arb_dev = NULL;
		rc = -ENODEV;
		goto err;
	}
	rc = sysfs_create_group(&mdp_arb->arb_dev->kobj, &mdp_arb_attr_group);
	if (rc) {
		pr_err("%s sysfs_create_group fails=%d", __func__, rc);
		goto err;
	} else {
		mdp_arb->sysfs_created = true;
	}

	mdp_arb_init_pipe(mdp_arb);
	for (i = 0; i < FB_MAX; i++)
		INIT_LIST_HEAD(&mdp_arb->client_db_list[i]);
	INIT_LIST_HEAD(&mdp_arb->event_db_list);
	INIT_LIST_HEAD(&mdp_arb->event_list);
	INIT_WORK(&mdp_arb->event_work, mdp_arb_event_work);
	mdp_arb->event_queue = create_workqueue("arb_event");
	if (IS_ERR_OR_NULL(mdp_arb->event_queue)) {
		pr_err("%s unable to create queue for mdp arb; errno = %ld",
			__func__, PTR_ERR(mdp_arb->event_queue));
		mdp_arb->event_queue = NULL;
		rc = -EFAULT;
		goto err;
	}
	platform_set_drvdata(&this_device, mdp_arb);
	arb = mdp_arb;
	mutex_unlock(&mdp_arb->dev_mutex);

	return rc;
err:
	mutex_unlock(&mdp_arb->dev_mutex);
	mdp_arb_release(mdp_arb);
	return rc;
}

static int mdp_arb_remove(struct platform_device *pdev)
{
	int rc = 0;
	struct mdp_arb_device_info *arb = platform_get_drvdata(pdev);

	mdp_arb_release(arb);

	return rc;
}

static struct platform_driver this_driver = {
	.probe = mdp_arb_probe,
	.remove = mdp_arb_remove,
	.driver = {
		.name = "mdp_arb",
		},
};

static int __init mdp_arb_init(void)
{
	int rc = 0;
	int chrdev_register = 0, driver_register = 0;

	arb_major = register_chrdev(UNNAMED_MAJOR, "mdp_arb", &arb_fops);
	if (arb_major < 0) {
		pr_err("%s unable to get major for mdp_arb dev, arb_major=%d",
			__func__, arb_major);
		rc = -ENODEV;
		goto err;
	} else {
		chrdev_register = 1;
	}

	/* Create class */
	arb_class = class_create(THIS_MODULE, "mdp_arb");
	if (IS_ERR_OR_NULL(arb_class)) {
		pr_err("%s unable to create mdp_arb class; errno = %ld",
			__func__, PTR_ERR(arb_class));
		arb_class = NULL;
		rc = -ENODEV;
		goto err;
	}


	rc = platform_driver_register(&this_driver);
	if (rc) {
		pr_err("driver register mdp arb fails, rc=%d", rc);
		goto err;
	} else {
		driver_register = 1;
	}

	rc = platform_device_register(&this_device);
	if (rc) {
		pr_err("device register mdp arb fails, rc=%d", rc);
		goto err;
	}

	return rc;

err:
	if (driver_register)
		platform_driver_unregister(&this_driver);
	if (!arb_class)
		class_destroy(arb_class);
	if (chrdev_register)
		unregister_chrdev(arb_major, "mdp_arb");
	return rc;
}

static void __exit mdp_arb_exit(void)
{
	platform_device_unregister(&this_device);
	platform_driver_unregister(&this_driver);
	class_destroy(arb_class);
	unregister_chrdev(arb_major, "mdp_arb");
}

module_init(mdp_arb_init);
module_exit(mdp_arb_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mdp arbitrator driver");
