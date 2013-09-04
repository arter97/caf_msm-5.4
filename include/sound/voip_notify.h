/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __VOIP_NOTIFY_H__
#define __VOIP_NOTIFY_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#define VOIP_NOTIFY_IOCTL_NAME "voip_notify"

#define VOIP_EVT_STREAM_ACTIVITY  0x01
#define VOIP_EVT_RX_DELAY         0x02

enum voip_activity_state {
	VOIP_STREAM_INACTIVE = 0,
	VOIP_STREAM_ACTIVE
};

/* Struct used for registering/deregistering events */
struct voip_event_type {
	uint32_t vsid;
	uint32_t event;
};

/* Struct used for sending notifications */
struct voip_event {
	uint32_t vsid;
	uint32_t event;
	uint32_t value;
};

/* Struct to store event registration state and values */
struct voip_event_info {
	uint32_t state;
	uint32_t reg_state;
	uint32_t evt_stream_activity;
	uint32_t evt_rx_delay;
};

#define AUDIO_NOTIFY_MAGIC 'N'

#define SNDRV_VOIP_EVT_NOTIFY		_IOWR(AUDIO_NOTIFY_MAGIC, \
					0x00, struct voip_event)
#define SNDRV_VOIP_EVT_REGISTER		_IOWR(AUDIO_NOTIFY_MAGIC, \
					0x01, struct voip_event_type)
#define SNDRV_VOIP_EVT_DEREGISTER	_IOWR(AUDIO_NOTIFY_MAGIC, \
					0x02, struct voip_event_type)

#endif
