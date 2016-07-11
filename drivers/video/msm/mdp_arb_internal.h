/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#ifndef _MDP_ARB_INTERNAL_H
#define _MDP_ARB_INTERNAL_H

#include <video/mdp_arb.h>
#include <linux/notifier.h>
#include "mdp.h"
#include "mdp4.h"
#include "msm_fb.h"

#define MDP_ARB_UEVENT_LEN (128)
#define MDP_ARB_ACK_TIMEOUT_MS (170) /*10 frames*/
#define MDP_ARB_UP_EVENT_STR "up="
#define MDP_ARB_DOWN_EVENT_STR "down="
#define MDP_ARB_OPTIMIZE_EVENT_STR "optimize="

extern int mdpclient_msm_fb_get_id(int idx, char *buf, int len);

/**
 * struct mdp_arb_client_db - client database.
 * @register_info: information that client registers.
 * @ack: if client has sent acknowledgement.
 * @ack_comp: completion for acknowledgement.
 * @list: list pointer.
 * @notified: if driver has sent notification to this client.
 * @num_of_layer: number of layers accquired for this client.
 * @num_of_client: number of clients registered and binded.
 * @display_id: display ID associated with this fb.
 */
struct mdp_arb_client_db {
	struct mdp_arb_client_register_info register_info;
	bool ack;
	struct completion ack_comp;
	struct list_head list;
	bool notified;
	int num_of_layer;
	int num_of_client;
	char display_id[MDP_ARB_NAME_LEN];
};

/**
 * struct mdp_arb_event_db - event database.
 * @event: event information that driver registers.
 * @cur_state_value: current state value of this event.
 * @list: list pointer.
 */
struct mdp_arb_event_db {
	struct mdp_arb_event event;
	int cur_state_value;
	struct list_head list;
};

/**
 * struct mdp_arb_pipe - information for pipe bookkeeping.
 * @support_by_hw: if this pipe is supported by MDP HW. It reads from fb driver.
 * @type: pipe type.
 * @client: pointer of client information which uses this pipe. NULL means this
 * pipe is free to use.
 */
struct mdp_arb_pipe {
	bool support_by_hw;
	int type;
	struct mdp_arb_client_db *client;
};

/**
 * struct mdp_arb_notify_info - information for each notification/cb event.
 * @client: client information.
 * @num_of_layers: number of layers this client holds.
 * @event: pointer of event information.
 * @list: list pointer.
 */
struct mdp_arb_notify_info {
	struct mdp_arb_client_db *client;
	int num_of_layers;
	struct mdp_arb_event_db *event;
	struct list_head list;
};

/**
 * struct mdp_arb_notify_list - list of notification/cb sent to customer.
 * @optimize_list: list of clients for optimize notifications.
 * @down_list: list of clients for down notifications.
 * @up_list: list of clients for up notifications.
 */
struct mdp_arb_notify_list {
	struct list_head optimize_list;
	struct list_head down_list;
	struct list_head up_list;
};

/**
 * struct mdp_arb_device_info - device info for arbitrator.
 * @arb_dev: arbitrator device handle.
 * @dev_mutex: device mutex used to protect this context.
 * @num_of_clients: number of clients in client_db_list.
 * @client_db_list: client database list head.
 * @num_of_events: number of events in event_db_list.
 * @event_db_list: event database list head.
 * @num_of_pipes: number of pipes in the display driver.
 * @pipe: pipe list for bookkeeping.
 * @event_work: event work task used to submit to event queue thread.
 * @event_queue: event queue thread used to process event work.
 * @event_list: event list to queue the data.
 * @sysfs_created: if sysfs is successfully created.
 * @fb_notif: callback to receive fb notification.
 * @fb_notif_registered: if fb notification is successfully registered.
 * @event_work_notify_list: notify list for event work thread.
 * @overlay_set_notify_list: notify list when during overlay set.
 */
struct mdp_arb_device_info {
	struct device *arb_dev;
	struct mutex dev_mutex;
	int num_of_clients[FB_MAX];
	struct list_head client_db_list[FB_MAX];
	int num_of_events;
	struct list_head event_db_list;
	int num_of_pipes;
	struct mdp_arb_pipe *pipe;
	struct work_struct event_work;
	struct workqueue_struct *event_queue;
	struct list_head event_list;
	bool sysfs_created;
	struct notifier_block fb_notif;
	bool fb_notif_registered;
	struct mdp_arb_notify_list event_work_notify_list;
	struct mdp_arb_notify_list overlay_set_notify_list;
};

#endif /*_MDP_ARB_INTERNAL_H*/
