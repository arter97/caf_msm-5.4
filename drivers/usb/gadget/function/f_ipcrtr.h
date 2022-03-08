/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#ifndef _F_IPCRTR_H
#define _F_IPCRTR_H

#define IPCRTR_CTRL_NAME_LEN	40
#define	IPCRTR_FAKE_PKT_LEN	100

#define IPC_USB_XPRT_MAX_READ_SZ	(4 * 1024)
#define IPC_USB_XPRT_MAX_WRITE_SZ	(4 * 1024)

/**
 * struct ipcrtr_ctrl_pkt - Wrapper used to send/receive IPC Router messages
 * @buf:	Buffer to hold actual packet
 * @len:	Length of packet
 * @list:	List head to store link-list of packets to be send or received
 */
struct ipcrtr_ctrl_pkt {
	void			*buf;
	int			len;
	struct list_head	list;
};

/**
 * struct ipcrtr_function_bind_info - Holder struct used to pass info to bind()
 * @string_defs:	Name of the advertised interface desc (iInterface)
 * @ctrl_str_idx:	string_defs number
 * @ctrl_desc:		Complete interface descriptor
 * @fs_notify_desc:	Full speed USB endpoint descriptor
 * @hs_notify_desc:	High speed USB endpoint descriptor
 * @ss_notify_desc:	Super speed USB endpoint descriptor
 * @fs_desc_hdr:	Full speed USB function header descriptor
 * @hs_desc_hdr:	High speed USB function header descriptor
 * @ss_desc_hdr:	Super speed USB function header descriptor
 * @notify_buf_len:	Size of notifications send accross this endpoint
 */
struct ipcrtr_function_bind_info {
	struct usb_string *string_defs;
	int ctrl_str_idx;
	struct usb_interface_descriptor *ctrl_desc;
	struct usb_endpoint_descriptor *fs_notify_desc;
	struct usb_endpoint_descriptor *hs_notify_desc;
	struct usb_endpoint_descriptor *ss_notify_desc;

	struct usb_descriptor_header **fs_desc_hdr;
	struct usb_descriptor_header **hs_desc_hdr;
	struct usb_descriptor_header **ss_desc_hdr;

	u32 notify_buf_len;
};

/**
 * enum ipcrtr_ctrl_notify_state - Internal enumeration for CDC control msgs
 *                                 Currently we only handle 2 messages. Enum
 *                                 is added for future expansion if we handle
 *                                 additional CDC control messages.
 * @NONE:		Default
 * @RESPONSE_AVAILABLE:	Send to HOST from DEVICE to get it's attention
 */
enum ipcrtr_ctrl_notify_state {
	IPCRTR_CTRL_NOTIFY_NONE,
	IPCRTR_CTRL_NOTIFY_RESPONSE_AVAILABLE,
};

/**
 * struct f_ipcrtr - IPC Router function driver context structure
 * @function:		USB function that is registered with USB core
 * @interface_id:	Interface number for notification endpoint
 * @connected:		Active config set using 'SET_CONFIGURATION'
 * @notify:		USB interrupt notification endpoint
 * @notify_req:		USB interrupt notification ep request
 * @notify_state:	Current status of ipcrtr_ctrl_notify_state
 * @notify_count:	Pending notifications to be submitted to int ep
 * @read_wq:		Waiting queue for read events
 * @cpkt_req_q:		Queue for packet received from USB control ep
 * @cpkt_resp_q:	Queue for packets to be send to USB control ep
 * @lock:		Lock to serialize send/receive operations
 * @is_open:		Flag to indicate status of USB IPC XPRT
 * @ipcrtr_wq:		Local workqueue for init/exit work
 * @init_work:		Use to perform registration with USB IPC XPRT
 * @exit_work:		Use to perform de-registration with USB IPC XPRT
 *
 * Stats
 * @host_to_ipc_xprt:		Recvd packets from hosts
 * @copied_to_ipc_xprt:		Delivered recvd packets to ipc xprt layer
 * @copied_from_ipc_xprt:	Recvd packets from ipc xprt layer
 * @ipc_xprt_to_host:		Delivered packets to USB controller
 * @cpkt_drop_cnt:		Control packets drop in USB driver
 */
struct f_ipcrtr {
	struct usb_function function;
	int interface_id;
	atomic_t connected;

	struct usb_ep *notify;
	struct usb_request *notify_req;
	int notify_state;
	atomic_t notify_count;

	wait_queue_head_t read_wq;

	struct list_head cpkt_req_q;
	struct list_head cpkt_resp_q;

	spinlock_t lock;
	bool is_open;
	bool is_exiting;

	struct workqueue_struct *ipcrtr_wq;
	struct delayed_work init_work;
	struct delayed_work exit_work;

	unsigned host_to_ipc_xprt;
	unsigned copied_to_ipc_xprt;
	unsigned copied_from_ipc_xprt;
	unsigned ipc_xprt_to_host;
	unsigned cpkt_drop_cnt;
};

/* Helper function */
static inline struct f_ipcrtr *func_to_ipcrtr(struct usb_function *f)
{
	return container_of(f, struct f_ipcrtr, function);
}

#define IPCRTR_LOG2_STATUS_INTERVAL_MSEC 5
#define MAX_IPCRTR_NOTIFY_SIZE sizeof(struct usb_cdc_notification)

/* ipcrtr device descriptors */
static struct usb_interface_descriptor ipcrtr_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	/* .iInterface = DYNAMIC */
};

/* Full speed support */

/* Full speed endpoint descriptor */
static struct usb_endpoint_descriptor ipcrtr_fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(MAX_IPCRTR_NOTIFY_SIZE),
	.bInterval =		1 << IPCRTR_LOG2_STATUS_INTERVAL_MSEC,
};

/* Full speed function descriptor */
static struct usb_descriptor_header *ipcrtr_fs_function[] = {
	(struct usb_descriptor_header *) &ipcrtr_interface_desc,
	(struct usb_descriptor_header *) &ipcrtr_fs_notify_desc,
	NULL,
};

/* High speed support */

/* High speed endpoint descriptor */
static struct usb_endpoint_descriptor ipcrtr_hs_notify_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(MAX_IPCRTR_NOTIFY_SIZE),
	.bInterval =		IPCRTR_LOG2_STATUS_INTERVAL_MSEC,
};

/* High speed function descriptor */
static struct usb_descriptor_header *ipcrtr_hs_function[] = {
	(struct usb_descriptor_header *) &ipcrtr_interface_desc,
	(struct usb_descriptor_header *) &ipcrtr_hs_notify_desc,
	NULL,
};

/* Super speed support */

/* Super speed endpoint descriptor */
static struct usb_endpoint_descriptor ipcrtr_ss_notify_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(MAX_IPCRTR_NOTIFY_SIZE),
	.bInterval =		IPCRTR_LOG2_STATUS_INTERVAL_MSEC,
};

/* Super speed companion descriptor */
static struct usb_ss_ep_comp_descriptor ipcrtr_ss_notify_comp_desc = {
	.bLength =		sizeof(ipcrtr_ss_notify_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 3 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
	.wBytesPerInterval =	cpu_to_le16(MAX_IPCRTR_NOTIFY_SIZE),
};

/* Super speed function descriptor */
static struct usb_descriptor_header *ipcrtr_ss_function[] = {
	(struct usb_descriptor_header *) &ipcrtr_interface_desc,
	(struct usb_descriptor_header *) &ipcrtr_ss_notify_desc,
	(struct usb_descriptor_header *) &ipcrtr_ss_notify_comp_desc,
	NULL,
};

/* String descriptors */
static struct usb_string ipcrtr_string_defs[] = {
	[0].s = "IPC Router Control",
	{  } /* end of list */
};

static struct usb_gadget_strings ipcrtr_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		ipcrtr_string_defs,
};

static struct usb_gadget_strings *ipcrtr_strings[] = {
	&ipcrtr_string_table,
	NULL,
};

#endif
