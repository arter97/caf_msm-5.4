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

#ifndef _MDP_ARB_H
#define _MDP_ARB_H

#include <linux/module.h>
#include <linux/msm_mdp.h>

/**
 * struct mdp_arb_notification - common information for notifications such as
 * up, down, etc.
 * @client_name: client name who registers this callback.
 * @fb_index: fb index of current client.
 * @event_name: event name which generates this notification.
 * @value: current event state value.
 */
struct mdp_arb_notification {
	char client_name[MDP_ARB_NAME_LEN];
	int fb_index;
	char event_name[MDP_ARB_NAME_LEN];
	int value;
};

/**
 * struct mdp_arb_notification_optimize - information for optimize notification.
 * @common: common information reported in this notification.
 * @max_num_of_layers: maximum number of layers the client(s) can keep after
 * entering the optimized mode.
 */
struct mdp_arb_notification_optimize {
	struct mdp_arb_notification common;
	int max_num_of_layers;
};

/**
 * struct mdp_arb_cb_info - callback information MDP arbitrator driver triggers.
 * @event: notification event triggered in this callback.
 * @info: union of callback information.
 * @up: information for up notification.
 * @down: information for down notification.
 * @optimize: information for optimize notification.
 */
struct mdp_arb_cb_info {
	enum mdp_arb_notification_event event;
	union {
		struct mdp_arb_notification up;
		struct mdp_arb_notification down;
		struct mdp_arb_notification_optimize optimize;
	} info;
};

/**
 * typedef mdp_arb_cb - prototype for callback function.
 * @handle: MDP arbitrator handle.
 * @info: callback information.
 * @flag: reserved for future usage.
 */
typedef int (*mdp_arb_cb)(void *handle, struct mdp_arb_cb_info *info, int flag);

/**
 * struct mdp_arb_client_register_info - information for kernel client
 * registering to driver.
 * @common: common register information for both kernel and user space clients.
 * @cb: callback function pointer used to receive notifications.
 */
struct mdp_arb_client_register_info {
	struct mdp_arb_register common;
	mdp_arb_cb cb;
};

/**
 * mdp_arb_client_get_event() - for kernel client to get event information
 * supported by current platform.
 * @events: pointer of event information
 */
int mdp_arb_client_get_event(struct mdp_arb_events *events);

/**
 * mdp_arb_client_register() - for kernel client to register to MDP arbitrator
 * driver.
 * @info: information used for registering.
 * @handle: handle returned by driver.
 */
int mdp_arb_client_register(struct mdp_arb_client_register_info *info,
	void **handle);

/**
 * mdp_arb_client_deregister() - for kernel client to deregister from MDP
 * arbitrator driver.
 * @handle: driver handle.
 */
int mdp_arb_client_deregister(void *handle);

/**
 * mdp_arb_client_bind() - for kernel client to bind to a registered client.
 * @info: information used for registered client.
 * @handle: handle returned by driver.
 */
int mdp_arb_client_bind(struct mdp_arb_bind *info, void **handle);

/**
 * mdp_arb_client_unbind() - for kernel client to unbind from a registered
 * client.
 * @handle: driver handle.
 */
int mdp_arb_client_unbind(void *handle);

/**
 * mdp_arb_client_acknowledge() - for kernel client to acknowledge the response
 * after it receives the notifications and processes them.
 * @handle: driver handle.
 * @notification_mask: notification mask of mdp_arb_notification_event.
 */
int mdp_arb_client_acknowledge(void *handle, int notification_mask);

/**
 * mdp_arb_client_get_state() - for kernel client to get current event state
 * value.
 * @handle: driver handle.
 * @event: event pointer to retrieve the state value.
 */
int mdp_arb_client_get_state(void *handle, struct mdp_arb_event *event);

/**
 * mdp_arb_client_overlay_set() - for kernel client to set/accquire overlay
 * resources.
 * @handle: driver handle.
 * @req: mdp overlay information for set.
 */
int mdp_arb_client_overlay_set(void *handle, struct mdp_overlay *req);

/**
 * mdp_arb_client_overlay_unset() - for kernel client to unset/release overlay
 * resources.
 * @handle: driver handle.
 * @id: mdp overlay id for unset.
 */
int mdp_arb_client_overlay_unset(void *handle, unsigned int id);

/**
 * mdp_arb_client_overlay_play() - for kernel client to play overlay resources.
 * @handle: driver handle.
 * @data: mdp overlay data for play.
 */
int mdp_arb_client_overlay_play(void *handle, struct msmfb_overlay_data *data);

/**
 * mdp_arb_client_overlay_commit() - for kernel client to commit overlay
 * resources.
 * @handle: driver handle.
 * @data: mdp overlay data for commit.
 */
int mdp_arb_client_overlay_commit(void *handle,
					struct mdp_display_commit *data);

/**
 * mdp_arb_event_register() - for event generator drivers, such as switch, CAN
 * bus, etc, to register the event to the mdp arbitrator.
 * @events: events pointer for the registration.
 */
int mdp_arb_event_register(struct mdp_arb_events *events);

/**
 * mdp_arb_event_deregister() - for event generator drivers, such as switch, CAN
 * bus, etc, to deregister the event from the mdp arbitrator.
 * @events: events pointer for the deregistration.
 */
int mdp_arb_event_deregister(struct mdp_arb_events *events);

/**
 * mdp_arb_event_set() - for event generator drivers to set the event to the mdp
 * arbitrator.
 * @event: event pointer for set.
 */
int mdp_arb_event_set(struct mdp_arb_event *event);

#endif /*_MDP_ARB_H*/

