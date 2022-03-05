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

#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/usb/composite.h>
#include <linux/debugfs.h>
#include <linux/usb/cdc.h>

#include <f_ipcrtr.h>
#include <debug.h>
#include <linux/module.h>
#include <linux/ipc_router_usb_xprt.h>

static struct dentry *debugfs_root;
static struct f_ipcrtr *ipcrtr_ctx;

static int ipcrtr_ctrl_send_notification(struct f_ipcrtr *ipcrtr,
		enum ipcrtr_ctrl_notify_state);

/**
 * usb_ipcrtr_debugfs_read() - Dump debugging information
 * @file:	Node created when debugfs was created
 *
 * @user_buf:	User space buffer
 * @count:	Total size of debug dump
 * @ppos:	Position within the buffer
 */
static ssize_t usb_ipcrtr_debugfs_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	struct f_ipcrtr *ipcrtr = (struct f_ipcrtr *)file->private_data;
	ssize_t ret_cnt;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "%25s\n",
		"USB IPCRTR Function Status");

	if (!ipcrtr) {
		log_event_err("%s: ipcrtr not initialized %p",
						__func__, ipcrtr);
	}

	if (ipcrtr) {
		len += scnprintf(buf + len, buf_len - len, "%55s\n",
		"==================================================");
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10d\n", "TX Notification State: ",
				ipcrtr->notify_state);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Pending Notification Count: ",
				ipcrtr->notify_count.counter);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "USB IPC XPRT Open: ",
				ipcrtr->is_open);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Read by USB (RX): ",
				ipcrtr->host_to_ipc_xprt);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Send to IPC XPRT (RX): ",
				ipcrtr->copied_to_ipc_xprt);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Submitted to USB controller(TX): ",
				ipcrtr->ipc_xprt_to_host);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Recevied from IPC XPRT (TX): ",
				ipcrtr->copied_from_ipc_xprt);
		len += scnprintf(buf + len, buf_len - len,
		"%35s %10u\n", "Cummulative control pkt drops: ",
				ipcrtr->cpkt_drop_cnt);
		len += scnprintf(buf + len, buf_len - len, "%25s\n",
		"=================================================");
	}

	if (len > buf_len)
		len = buf_len;

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret_cnt;
}

/* FileOps strucutre for debugfs file node */
static const struct file_operations fops_usb_ipcrtr = {
	.read = usb_ipcrtr_debugfs_read,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

/**
 * usb_ipcrtr_debugfs_init() - Initialize, allocate debugfs node and file node
 *
 */
static int usb_ipcrtr_debugfs_init(void)
{
	debugfs_root = debugfs_create_dir("usb_ipcrtr", 0);
	if (IS_ERR(debugfs_root)) {
		log_event_err("%s: Cannot create debugfs %p ", __func__,
						debugfs_root);
		return -ENOMEM;
	}

	debugfs_create_file("stats", S_IRUSR | S_IRGRP | S_IROTH, debugfs_root,
					ipcrtr_ctx, &fops_usb_ipcrtr);
	log_event_err("%s: debugfs created %p ", __func__, debugfs_root);
	return 0;
}

/**
 * usb_ipcrtr_debugfs_exit() - Cleanup debugfs node and file node
 *
 */
static void usb_ipcrtr_debugfs_exit(void)
{
	debugfs_remove_recursive(debugfs_root);
}

/**
 * ipcrtr_dump_pkt() - Debug function to dump the packet
 * @pkt:	Packet buffer to be dumped
 * @count:	Length of the buffer
 *
 */
static void ipcrtr_dump_pkt(void *pkt, int count)
{
	int i;
	char *buf = (char *)pkt;
	char init_cnt = count;

	if (!buf || (count <= 0)) {
		log_event_err("%s: Invalid arguments buf %p count %d",
							__func__, buf, count);
		return;
	}

	/* Ensure count is a multiple of 4s */
	count = count - (count % 4);

	pr_info("%s - Packet Dump \n",__func__);
	for (i = 0; i < count - 1; i += 4)
		pr_info("%02x %02x %02x %02x\n",
			buf[i], buf[i+1], buf[i+2], buf[i+3]);

	for ( ;i < init_cnt; i++)
		pr_info("%02x ", buf[i]);
	pr_info("\n");
}

/**
 * ipcrtr_ctrl_pkt_alloc() - Allocation of a control packet
 *
 * @len:	Length of the packet to be allocated
 * @flags:	GFP flags to be used for allocation
 */
static struct ipcrtr_ctrl_pkt *ipcrtr_ctrl_pkt_alloc(unsigned len, gfp_t flags)
{
	struct ipcrtr_ctrl_pkt *pkt;

	pkt = kzalloc(sizeof(struct ipcrtr_ctrl_pkt), flags);
	if (!pkt)
		return ERR_PTR(-ENOMEM);

	pkt->buf = kmalloc(len, flags);
	if (!pkt->buf) {
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}
	pkt->len = len;

	return pkt;
}

/**
 * ipcrtr_ctrl_pkt_free() - De-allocation function for control packet
 *
 * @pkt:	Packet to be de-allocated.
 */
static void ipcrtr_ctrl_pkt_free(struct ipcrtr_ctrl_pkt *pkt)
{
	if (pkt) {
		kfree(pkt->buf);
		kfree(pkt);
	}
}

/**
 * ipcrtr_ctrl_clear_cpkt_queues() - Clean TX or RX packet queues
 *
 * @ipcrtr:	IPCRTR context
 * @skip_req_q:	Skip cleaning RX queues
 */
static void ipcrtr_ctrl_clear_cpkt_queues(struct f_ipcrtr *ipcrtr, bool skip_req_q)
{
	struct ipcrtr_ctrl_pkt *cpkt = NULL;
	struct list_head *act, *tmp;

	spin_lock(&ipcrtr->lock);
	if (skip_req_q)
		goto clean_resp_q;

	list_for_each_safe(act, tmp, &ipcrtr->cpkt_req_q) {
		cpkt = list_entry(act, struct ipcrtr_ctrl_pkt, list);
		list_del(&cpkt->list);
		ipcrtr_ctrl_pkt_free(cpkt);
	}
clean_resp_q:
	list_for_each_safe(act, tmp, &ipcrtr->cpkt_resp_q) {
		cpkt = list_entry(act, struct ipcrtr_ctrl_pkt, list);
		list_del(&cpkt->list);
		ipcrtr_ctrl_pkt_free(cpkt);
	}
	spin_unlock(&ipcrtr->lock);
}

/**
 * ipcrtr_ctrl_send_cpkt_to_xprt() - Send recvd control packet to IPC XPRT
 *
 * @ipcrtr:	IPCRTR context
 * @buf:	Buffer containing the actual packet
 * @len:	Length of the buffer
 */
static int ipcrtr_ctrl_send_cpkt_to_xprt(struct f_ipcrtr *ipcrtr,
							void *buf, size_t len)
{
	unsigned long flags;
	struct ipcrtr_ctrl_pkt *cpkt;

	if (!ipcrtr) {
		log_event_err("%s: ipcrtr ctrl port %p \n", __func__, ipcrtr);
		return -ENODEV;
	}

	/* Dump RX Packet */
	// log_event_info("%s RX Packet \n",__func__);
	// ipcrtr_dump_pkt(buf, len);

	spin_lock_irqsave(&ipcrtr->lock, flags);
	/* drop cpkt if port is not open */
	if (!ipcrtr->is_open) {
		log_event_err("%s: xprt device is not open \n",__func__);
		ipcrtr->cpkt_drop_cnt++;
		spin_unlock_irqrestore(&ipcrtr->lock, flags);
		return -ENODEV;
	}

	cpkt = ipcrtr_ctrl_pkt_alloc(len, GFP_ATOMIC);
	if (IS_ERR(cpkt)) {
		log_event_err("%s: Reset func pkt allocation failed \n",
								 __func__);
		spin_unlock_irqrestore(&ipcrtr->lock, flags);
		return -ENOMEM;
	}

	memcpy(cpkt->buf, buf, len);
	cpkt->len = len;

	list_add_tail(&cpkt->list, &ipcrtr->cpkt_req_q);
	ipcrtr->host_to_ipc_xprt++;
	spin_unlock_irqrestore(&ipcrtr->lock, flags);

	log_event_dbg("%s: Wake up read queue, packet recvd of %d bytes\n",
								__func__, len);
	wake_up(&ipcrtr->read_wq);

	return 0;
}

/**
 * ipcrtr_ctrl_open() - Device Open: called from IPC USB XPRT layer
 *
 */
static int ipcrtr_ctrl_open(void)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;

	if (!ipcrtr) {
		log_event_err("%s: ipcrtr null pointer ipcrtr %p \n",
							__func__, ipcrtr);
		return -ENODEV;
	}

	log_event_dbg("%s: opening xprt device is_open %d \n",
						__func__, ipcrtr->is_open);

	if (ipcrtr->is_open == true) {
		log_event_err("%s: Already opened \n", __func__);
		return -EBUSY;
	}

	ipcrtr->is_open = true;

	return 0;
}

/**
 * ipcrtr_ctrl_close() - Device Closed: called from IPC USB XPRT layer
 *
 */
static int ipcrtr_ctrl_close(void)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;

	if (!ipcrtr) {
		log_event_err("%s: ipcrtr null pointer ipcrtr %p \n",
							__func__, ipcrtr);
		return -ENODEV;
	}

	log_event_dbg("%s: closing xprt device\n", __func__);

	if (!ipcrtr->is_open) {
		log_event_err("%s: Already closed \n", __func__);
		return -EBUSY;
	}

	ipcrtr->is_open = false;

	return 0;
}

/**
 * ipcrtr_ctrl_read() - Read Packet from USB layer
 *                      Called from IPC XPRT layer
 *
 * @buf:	Read buffer which would contain the read packet
 * @count:	Max length of the read buffer
 *
 */
static int ipcrtr_ctrl_read(char *buf, unsigned int count)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;
	struct ipcrtr_ctrl_pkt *cpkt = NULL;
	unsigned long flags;
	int ret = 0;

	log_event_dbg("%s - Enter %zu \n", __func__, count);
	if (!ipcrtr) {
		log_event_err("%s: ipcrtr ctrl port %p \n", __func__, ipcrtr);
		return -ENODEV;
	}

	if (count > IPC_USB_XPRT_MAX_READ_SZ) {
		log_event_err("Large buff size %zu, should be %d \n",
			count, IPC_USB_XPRT_MAX_READ_SZ);
		return -EINVAL;
	}

	/* block until a new packet is available */
	spin_lock_irqsave(&ipcrtr->lock, flags);
	while (list_empty(&ipcrtr->cpkt_req_q)) {
		log_event_dbg("Requests list is empty. Wait. \n");
		spin_unlock_irqrestore(&ipcrtr->lock, flags);
		ret = wait_event_interruptible(ipcrtr->read_wq,
				!list_empty(&ipcrtr->cpkt_req_q));
		if (ret < 0) {
			log_event_err("Waiting failed \n");
			return -ERESTARTSYS;
		}
		if (ipcrtr->is_exiting)
			return -ESHUTDOWN;
		log_event_dbg("Received request packet \n");
		spin_lock_irqsave(&ipcrtr->lock, flags);
	}

	cpkt = list_first_entry(&ipcrtr->cpkt_req_q, struct ipcrtr_ctrl_pkt,
							list);
	list_del(&cpkt->list);
	spin_unlock_irqrestore(&ipcrtr->lock, flags);

	if (cpkt->len > count) {
		log_event_err("cpkt size large:%d > buf size:%zu",
				cpkt->len, count);
		ipcrtr_ctrl_pkt_free(cpkt);
		return -EOVERFLOW;
	}

	memcpy(buf, cpkt->buf, cpkt->len);

	log_event_dbg("%s: copied %d bytes to ipc xprt \n", __func__,
							cpkt->len);
	ret = cpkt->len;
	ipcrtr->copied_to_ipc_xprt++;

	ipcrtr_ctrl_pkt_free(cpkt);

	return ret;
}

/**
 * ipcrtr_ctrl_write() - Submit packet to USB driver
 *                       Called from IPC XPRT layer
 *
 * @buf:	Buffer which would contain the packet
 * @count:	Length of the sent packet
 *
 */
static int ipcrtr_ctrl_write(char *buf, unsigned int count)
{
	int ret = 0;
	unsigned long flags;
	struct ipcrtr_ctrl_pkt *cpkt;
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;
	struct usb_request *req = ipcrtr->notify_req;

	log_event_dbg("%s - Enter %zu \n", __func__, count);

	if (!ipcrtr || !req || !req->buf) {
		log_event_err("%s: ipcrtr %p req %p req->buf %p",__func__,
				ipcrtr, ipcrtr, req, req ? req->buf : req);
		return -ENODEV;
	}

	if (!count || count > IPC_USB_XPRT_MAX_WRITE_SZ) {
		log_event_err("error: ctrl pkt length %zu", count);
		return -EINVAL;
	}

	if (!atomic_read(&ipcrtr->connected)) {
		log_event_err("USB cable not connected\n");
		return -ECONNRESET;
	}

	/* TODO: If power save is supported add power-save check here */

	/* Dump TX Packet */
	// log_event_info("%s -- TX Packet Received \n",__func__);
	// ipcrtr_dump_pkt(buf, count);

	cpkt = ipcrtr_ctrl_pkt_alloc(count, GFP_KERNEL);
	if (IS_ERR(cpkt)) {
		log_event_err("failed to allocate ctrl pkt");
		return -ENOMEM;
	}

	memcpy(cpkt->buf, buf, count);

	ipcrtr->copied_from_ipc_xprt++;

	spin_lock_irqsave(&ipcrtr->lock, flags);
	list_add_tail(&cpkt->list, &ipcrtr->cpkt_resp_q);
	spin_unlock_irqrestore(&ipcrtr->lock, flags);

	ret = ipcrtr_ctrl_send_notification(ipcrtr,
			IPCRTR_CTRL_NOTIFY_RESPONSE_AVAILABLE);

	ipcrtr->ipc_xprt_to_host++;

	return ret ? ret : count;
}

/**
 * xprt_ops - Struct that is passed to IPC XPRT layer on registration
 *
 */
static struct usb_ipc_xprt_ops xprt_ops = {
	.max_read_size = IPC_USB_XPRT_MAX_READ_SZ,
	.max_write_size = IPC_USB_XPRT_MAX_WRITE_SZ,
	.open = ipcrtr_ctrl_open,
	.close = ipcrtr_ctrl_close,
	.read = ipcrtr_ctrl_read,
	.write = ipcrtr_ctrl_write,
};

/**
 * ipcrtr_function_ctrl_port_init() - Initialize notification endpoint
 *                                    related variables
 *
 */
static int ipcrtr_function_ctrl_port_init(void)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;

	if (unlikely(!ipcrtr)) {
		log_event_err("%s: ipcrtr prot ctx is NULL", __func__);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&ipcrtr->cpkt_req_q);
	INIT_LIST_HEAD(&ipcrtr->cpkt_resp_q);

	spin_lock_init(&ipcrtr->lock);

	init_waitqueue_head(&ipcrtr->read_wq);

	ipcrtr->is_open = false;

	return 0;
}

/**
 * ipcrtr_queue_notification_request() - Queue a CDC notification on USB Int EP
 *
 * @ipcrtr:	IPCRTR context
 */
static int ipcrtr_queue_notification_request(struct f_ipcrtr *ipcrtr)
{
	int ret;
	unsigned long flags;
	struct usb_cdc_notification *event;
	struct ipcrtr_ctrl_pkt *cpkt;

	ret = usb_func_ep_queue(&ipcrtr->function, ipcrtr->notify,
			   ipcrtr->notify_req, GFP_ATOMIC);
	if (ret == -ENOTSUPP || (ret < 0 && ret != -EAGAIN)) {
		spin_lock_irqsave(&ipcrtr->lock, flags);
		/* check if device disconnected while we dropped lock */
		if (atomic_read(&ipcrtr->connected) &&
			!list_empty(&ipcrtr->cpkt_resp_q)) {
			cpkt = list_first_entry(&ipcrtr->cpkt_resp_q,
					struct ipcrtr_ctrl_pkt, list);
			list_del(&cpkt->list);
			atomic_dec(&ipcrtr->notify_count);
			log_event_err("%s: drop ctrl pkt of len %d error %d",
						__func__, cpkt->len, ret);
			ipcrtr_ctrl_pkt_free(cpkt);
		}
		ipcrtr->cpkt_drop_cnt++;
		spin_unlock_irqrestore(&ipcrtr->lock, flags);
	} else {
		ret = 0;
		event = ipcrtr->notify_req->buf;
		log_event_dbg("%s: Queued Notify type %02x \n", __func__,
				event->bNotificationType);
	}

	return ret;
}

/**
 * ipcrtr_ctrl_send_notification() - Prepare a CDC notification message and
 *                                   submit USB Int EP
 *
 * @ipcrtr:	IPCRTR context
 * @state:	CDC Notification type to be queued
 */
static int ipcrtr_ctrl_send_notification(struct f_ipcrtr *ipcrtr,
		enum ipcrtr_ctrl_notify_state state)
{
	__le32 *data;
	struct usb_cdc_notification *event;
	struct usb_request *req = ipcrtr->notify_req;
	struct usb_composite_dev *cdev = ipcrtr->function.config->cdev;

	if (!atomic_read(&ipcrtr->connected)) {
		log_event_dbg("%s: cable disconnect \n", __func__);
		return -ENODEV;
	}

	event = req->buf;

	switch (state) {
	case IPCRTR_CTRL_NOTIFY_NONE:
		if (atomic_read(&ipcrtr->notify_count) > 0)
			log_event_dbg("IPCRTR_CTRL_NOTIFY_NONE %d \n",
			atomic_read(&ipcrtr->notify_count));
		else
			log_event_dbg("No pending notifications \n");
		return 0;
	case IPCRTR_CTRL_NOTIFY_RESPONSE_AVAILABLE:
		event->bNotificationType = USB_CDC_NOTIFY_RESPONSE_AVAILABLE;
		event->wValue = cpu_to_le16(0);
		event->wLength = cpu_to_le16(0);
		ipcrtr->notify_state = IPCRTR_CTRL_NOTIFY_RESPONSE_AVAILABLE;
		break;
	default:
		log_event_err("%s:unknown notify state", __func__);
		return -EINVAL;
	}

	log_event_dbg("send Notify type %02x \n", event->bNotificationType);

	if (atomic_inc_return(&ipcrtr->notify_count) != 1) {
		log_event_dbg("delay ep_queue: notify req is busy %d \n",
			atomic_read(&ipcrtr->notify_count));
		return 0;
	}

	return ipcrtr_queue_notification_request(ipcrtr);
}

/**
 * ipcrtr_ctrl_notify_resp_complete() - Completion callback for submitting pkts
 *                                      to notification EP
 *
 * @ep:		Notification endpoint
 * @req:	Notification request upon completion
 */
static void ipcrtr_ctrl_notify_resp_complete(struct usb_ep *ep,
					struct usb_request *req)
{
	struct f_ipcrtr *ipcrtr = req->context;
	struct usb_cdc_notification *event = req->buf;
	int status = req->status;

	log_event_dbg("%s -- event %02x ep \n",
				__func__, event->bNotificationType);
	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		ipcrtr->notify_state = IPCRTR_CTRL_NOTIFY_NONE;
		atomic_set(&ipcrtr->notify_count, 0);
		log_event_err("ESHUTDOWN/ECONNRESET, connection gone \n");
		ipcrtr_ctrl_clear_cpkt_queues(ipcrtr, false);
		/* TODO: Do we need to notify IPC Router XPRT ?*/
		break;
	default:
		log_event_err("Unknown event %02x --> %d",
			event->bNotificationType, req->status);
		/* FALLTHROUGH */
	case 0:
		/*
		 * handle multiple pending resp available
		 * notifications by queuing same until we're done,
		 * rest of the notification require queuing new
		 * request.
		 */
		if (!atomic_dec_and_test(&ipcrtr->notify_count)) {
			log_event_dbg("notify_count = %d \n",
			atomic_read(&ipcrtr->notify_count));
			ipcrtr_queue_notification_request(ipcrtr);
		}
		break;
	}
}

/**
 * ipcrtr_ctrl_cmd_complete() - Completion callback getting pkts
 *                              from control endpoint
 *
 * @ep:		Control endpoint
 * @req:	Control request upon completion
 */
static void ipcrtr_ctrl_cmd_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_ipcrtr *ipcrtr = req->context;

	/* Send Packet to USB IPC XPRT */
	ipcrtr_ctrl_send_cpkt_to_xprt(ipcrtr, req->buf, req->actual);
}

/**
 * ipcrtr_setup() - USB setup packet callback
 *
 * @f:		USB Function
 * @ctrl:	USB control message received
 */
static int
ipcrtr_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_ipcrtr *ipcrtr = func_to_ipcrtr(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request *req = cdev->req;
	int id, value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	struct ipcrtr_ctrl_pkt *cpkt;

	if (!atomic_read(&ipcrtr->connected)) {
		log_event_dbg("usb cable is not connected \n");
		return -ENOTCONN;
	}

	id = ipcrtr->interface_id;

	/* composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 */
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		log_event_dbg("USB_CDC_SEND_ENCAPSULATED_COMMAND \n");

		if (w_value || w_index != id)
			goto invalid;
		/* read the request; process it later */
		value = w_length;
		req->complete = ipcrtr_ctrl_cmd_complete;
		break;
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		log_event_dbg("USB_CDC_GET_ENCAPSULATED_RESPONSE \n");
		if (w_value || w_index != id)
			goto invalid;

		spin_lock(&ipcrtr->lock);
		if (list_empty(&ipcrtr->cpkt_resp_q)) {
			log_event_dbg("ctrl resp queue empty \n");
			spin_unlock(&ipcrtr->lock);
			break;
		}

		cpkt = list_first_entry(&ipcrtr->cpkt_resp_q,
					struct ipcrtr_ctrl_pkt, list);
		list_del(&cpkt->list);
		spin_unlock(&ipcrtr->lock);

		value = min_t(unsigned, w_length, cpkt->len);
		memcpy(req->buf, cpkt->buf, value);
		ipcrtr_ctrl_pkt_free(cpkt);

		log_event_dbg("copied encap_resp %d bytes \n",
			value);
		break;
	default:
invalid:
		log_event_err("inval ctrl req%02x.%02x v%04x i%04x l%d",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		log_event_dbg("req%02x.%02x v%04x i%04x l%d \n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->context = ipcrtr;
		req->zero = (value < w_length);
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			log_event_err("response on err %d \n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

/**
 * ipcrtr_get_alt() - Get active setting for USB configuration. Since we only
 *                    have on active setting for control endpoints. We return 0
 *                    always if an active control endpoint exists.
 *
 * @f:		USB Function
 * @intf:	Interface on which the request was received
 */
static int ipcrtr_get_alt(struct usb_function *f, unsigned intf)
{
	struct f_ipcrtr *ipcrtr = func_to_ipcrtr(f);

	if (intf == ipcrtr->interface_id)
		return 0;

	return -EINVAL;
}

/**
 * ipcrtr_set_alt() - Set active setting for USB configuration. Since we only
 *                    have on one setting, the only value valid value is 0.
 *
 * @f:		USB Function
 * @intf:	Interface on which the request was received
 * @alt:	Active setting, that is requested to be set
 */
static int ipcrtr_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_ipcrtr	 *ipcrtr = func_to_ipcrtr(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	log_event_dbg("intf=%u, alt=%u \n", intf, alt);

	/* Control interface has only altsetting 0 */
	if (intf == ipcrtr->interface_id) {
		if (alt != 0)
			goto fail;

		if (!ipcrtr->notify)
			goto fail;

		if (ipcrtr->notify->driver_data) {
			log_event_dbg("reset ipcrtr control %d \n", intf);
			usb_ep_disable(ipcrtr->notify);
		}

		ret = config_ep_by_speed(cdev->gadget, f,
					ipcrtr->notify);
		if (ret) {
			ipcrtr->notify->desc = NULL;
			log_event_err("Config-fail notify ep %s: err %d \n",
				ipcrtr->notify->name, ret);
			goto fail;
		}

		ret = usb_ep_enable(ipcrtr->notify);
		if (ret) {
			log_event_err("usb ep#%s enable failed, err#%d \n",
				ipcrtr->notify->name, ret);
			goto fail;
		}
		ipcrtr->notify->driver_data = ipcrtr;
		ipcrtr->is_exiting = false;
		log_event_dbg(" Control active intf=%u, alt=%u \n", intf, alt);
	} else {
		log_event_err("usb ep#%s wrong interface activated, inf#%d \n",
			ipcrtr->notify->name, intf);
		goto fail;
	}

	atomic_set(&ipcrtr->connected, 1);
	queue_delayed_work(ipcrtr->ipcrtr_wq, &ipcrtr->init_work, 0);

	return 0;
fail:
	return -EINVAL;
}

/**
 * usb_xprt_init() - Init function used to register with USB IPC XPRT
 *
 * @work:	Related work structure
 */
static void usb_xprt_init(struct work_struct *work)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;
	int ret;

	log_event_dbg("%s: Enter \n", __func__);
	if (unlikely(!ipcrtr)) {
		log_event_err("%s: ipcrtr prot ctx is NULL \n", __func__);
		return;
	}

	if (!atomic_read(&ipcrtr->connected)) {
		log_event_err(" %s: usb cable is not connected \n",__func__);
		return;
	}

	/* Wake up read queues to ensure all pending packets have been read */

	/* Register with USB IPC XPRT layer */
	ret = msm_ipc_router_usb_ipc_xprt_register(&xprt_ops);
	if (ret) {
		log_event_err("usb ep#%s XPRT Registration Failed, err#%d \n",
			ipcrtr->notify->name, ret);
		return;
	}

	log_event_dbg("usb ep#%s, XPRT Registration success \n",
					ipcrtr->notify->name);
}

/**
 * usb_xprt_deinit() - Cleanup function used to de-register with USB IPC XPRT
 *
 * @work:	Related work structure
 */
static void usb_xprt_deinit(struct work_struct *work)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;
	int ret;

	log_event_dbg("%s: Enter \n", __func__);
	if (unlikely(!ipcrtr)) {
		log_event_err("%s: ipcrtr prot ctx is NULL \n", __func__);
		return;
	}

	if (atomic_read(&ipcrtr->connected)) {
		log_event_err(" %s: usb cable still connected \n",__func__);
		return;
	}

	/* Unregister from USB IPC XPRT Layer */
	ret = msm_ipc_router_usb_ipc_xprt_unregister(&xprt_ops);
	if (ret) {
		log_event_err("usb ep#%s XPRT Un-registration Failed, err#%d",
			ipcrtr->notify->name, ret);
		return;
	}

	ipcrtr_ctrl_clear_cpkt_queues(ipcrtr, false);

	log_event_dbg("usb ep#%s, XPRT De-registration success \n",
					ipcrtr->notify->name);
}

/**
 * ipcrtr_disable() - Disable function registered with USB framework
 *
 * @f:		Related USB function
 */
static void ipcrtr_disable(struct usb_function *f)
{
	struct f_ipcrtr *ipcrtr = func_to_ipcrtr(f);
	int ret;
	struct ipcrtr_ctrl_pkt *cpkt;

	atomic_set(&ipcrtr->connected, 0);

	log_event_err("%s: Enter %s", __func__, f->name);

	 /* Disable Control Path */
	if (ipcrtr->notify &&
		ipcrtr->notify->driver_data) {
		usb_ep_disable(ipcrtr->notify);
		ipcrtr->notify->driver_data = NULL;
		ipcrtr->notify_state = IPCRTR_CTRL_NOTIFY_NONE;
	}

	atomic_set(&ipcrtr->notify_count, 0);

	/* Add a fake packet at the end */
	cpkt = ipcrtr_ctrl_pkt_alloc(IPCRTR_FAKE_PKT_LEN, GFP_ATOMIC);
	if (IS_ERR(cpkt)) {
		log_event_err("%s: func pkt allocation failed \n", __func__);
		return -ENOMEM;
	}

	list_add_tail(&cpkt->list, &ipcrtr->cpkt_req_q);
	ipcrtr->is_exiting = true;
	/* WakeUp Read queues to ensure that read callbacks are released */
	wake_up(&ipcrtr->read_wq);
	log_event_err("%s: After waking up work queue", __func__);

	queue_delayed_work(ipcrtr->ipcrtr_wq, &ipcrtr->exit_work,
						msecs_to_jiffies(100));
}

/**
 * ipcrtr_get_status() - Get status function registered with USB framework
 *
 * @f:		Related USB function
 */
static int ipcrtr_get_status(struct usb_function *f)
{
	unsigned remote_wakeup_en_status = f->func_wakeup_allowed ? 1 : 0;

	return (remote_wakeup_en_status << FUNC_WAKEUP_ENABLE_SHIFT) |
		(1 << FUNC_WAKEUP_CAPABLE_SHIFT);
}

/**
 * ipcrtr_update_function_bind_params() - Bind and configure USB endpoint
 *
 * @ipcrtr:	IPCRTR context
 * @cdev:	USB composite device instance
 * @info:	Information related to binding parameters
 */
static int ipcrtr_update_function_bind_params(struct f_ipcrtr *ipcrtr,
	struct usb_composite_dev *cdev,
	struct ipcrtr_function_bind_info *info)
{
	struct usb_ep *ep;
	struct usb_cdc_notification *event;
	struct usb_function *f = &ipcrtr->function;
	int status;

	/* maybe allocate device-global string IDs */
	if (info->string_defs[0].id != 0)
		goto skip_string_id_alloc;

	/* ctrl interface label */
	status = usb_string_id(cdev);
	if (status < 0)
		return status;
	info->string_defs[info->ctrl_str_idx].id = status;
	info->ctrl_desc->iInterface = status;

skip_string_id_alloc:
	if (info->ctrl_desc)
		info->ctrl_desc->bInterfaceNumber = ipcrtr->interface_id;

	/* allocate instance-specific endpoints */
	if (info->fs_notify_desc) {
		ep = usb_ep_autoconfig(cdev->gadget, info->fs_notify_desc);
		if (!ep)
			goto fail;
		ipcrtr->notify = ep;
		ep->driver_data = cdev;	/* claim */

		atomic_set(&ipcrtr->notify_count, 0);

		/* allocate notification request and buffer */
		ipcrtr->notify_req = usb_ep_alloc_request(ep, GFP_KERNEL);
		if (!ipcrtr->notify_req)
			goto fail;

		ipcrtr->notify_req->buf =
			kmalloc(info->notify_buf_len, GFP_KERNEL);
		if (!ipcrtr->notify_req->buf)
			goto fail;

		ipcrtr->notify_req->length = info->notify_buf_len;
		ipcrtr->notify_req->context = ipcrtr;
		ipcrtr->notify_req->complete =
				ipcrtr_ctrl_notify_resp_complete;
		event = ipcrtr->notify_req->buf;
		event->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
				| USB_RECIP_INTERFACE;
		event->wIndex = cpu_to_le16(ipcrtr->interface_id);
		event->wLength = cpu_to_le16(0);

		ipcrtr->notify_state = IPCRTR_CTRL_NOTIFY_NONE;
	}

	/* copy descriptors, and track endpoint copies */
	f->fs_descriptors = usb_copy_descriptors(info->fs_desc_hdr);
	if (!ipcrtr->function.fs_descriptors)
		goto fail;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(cdev->gadget)) {
		if (info->fs_notify_desc)
			info->hs_notify_desc->bEndpointAddress =
					info->fs_notify_desc->bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(info->hs_desc_hdr);
		if (!f->hs_descriptors)
			goto fail;
	}

	if (gadget_is_superspeed(cdev->gadget)) {
		if (info->fs_notify_desc)
			info->ss_notify_desc->bEndpointAddress =
					info->fs_notify_desc->bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->ss_descriptors = usb_copy_descriptors(info->ss_desc_hdr);
		if (!f->ss_descriptors)
			goto fail;
	}

	return 0;

fail:
	if (gadget_is_superspeed(cdev->gadget) && f->ss_descriptors)
		usb_free_descriptors(f->ss_descriptors);
	if (gadget_is_dualspeed(cdev->gadget) && f->hs_descriptors)
		usb_free_descriptors(f->hs_descriptors);
	if (f->fs_descriptors)
		usb_free_descriptors(f->fs_descriptors);
	if (ipcrtr->notify_req) {
		kfree(ipcrtr->notify_req->buf);
		usb_ep_free_request(ipcrtr->notify, ipcrtr->notify_req);
	}

	/* we might as well release our claims on endpoints */
	if (ipcrtr->notify)
		ipcrtr->notify->driver_data = NULL;
	log_event_err("%s: bind failed for %s", __func__, f->name);
	return -ENOMEM;
}

/**
 * ipcrtr_bind() - USB function registered with USB framework
 *
 * @c:	USB configuration
 * @f:	USB function which is getting binded
 */
static int ipcrtr_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct ipcrtr_function_bind_info info = {0};
	struct f_ipcrtr *ipcrtr = func_to_ipcrtr(f);
	int status;

	status = ipcrtr->interface_id = usb_interface_id(c, f);
	if (status < 0)
		goto fail;

	info.string_defs = ipcrtr_string_defs;
	info.ctrl_desc = &ipcrtr_interface_desc;
	info.ctrl_str_idx = 0;
	info.fs_notify_desc = &ipcrtr_fs_notify_desc;
	info.hs_notify_desc = &ipcrtr_hs_notify_desc;
	info.ss_notify_desc = &ipcrtr_ss_notify_desc;
	info.fs_desc_hdr = ipcrtr_fs_function;
	info.hs_desc_hdr = ipcrtr_hs_function;
	info.ss_desc_hdr = ipcrtr_ss_function;
	info.notify_buf_len = sizeof(struct usb_cdc_notification);

	status = ipcrtr_update_function_bind_params(ipcrtr, cdev, &info);

	log_event_info("%s: %s speed NOTIFY/%s intf id %d\n",
			f->name,
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			ipcrtr->notify->name, ipcrtr->interface_id);
	return 0;

fail:
	return status;
}

/**
 * ipcrtr_unbind() - USB unbind function registered with USB framework
 *
 * @c:	USB configuration
 * @f:	USB function which is getting unbinded
 */
static void ipcrtr_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_ipcrtr *ipcrtr = func_to_ipcrtr(f);
	struct usb_composite_dev *cdev = c->cdev;

	if (gadget_is_superspeed(cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	if (gadget_is_dualspeed(cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->fs_descriptors);

	if (ipcrtr->notify) {
		kfree(ipcrtr->notify_req->buf);
		usb_ep_free_request(ipcrtr->notify, ipcrtr->notify_req);

	}
}

/**
 * ipcrtr_bind_config() - USB function called from the android gadget framework
 *
 * @c:	USB configuration
 */
static int ipcrtr_bind_config(struct usb_configuration *c)
{
	struct f_ipcrtr	*ipcrtr;
	int status = 0;

	log_event_dbg("%s: -- Bind from android framework \n", __func__);
	pr_info("%s -- Entry -- \n",__func__);

	ipcrtr = ipcrtr_ctx;

	if (!ipcrtr) {
		log_event_err("%s: ipcrtr ctx is NULL \n", __func__);
		return -EINVAL;
	}

	ipcrtr->function.name = "ipcrtr";
	ipcrtr->function.strings = ipcrtr_strings;

	/* descriptors are per-instance copies */
	ipcrtr->function.bind = ipcrtr_bind;
	ipcrtr->function.unbind = ipcrtr_unbind;
	ipcrtr->function.set_alt = ipcrtr_set_alt;
	ipcrtr->function.get_alt = ipcrtr_get_alt;
	ipcrtr->function.setup = ipcrtr_setup;
	ipcrtr->function.disable = ipcrtr_disable;
	ipcrtr->function.get_status = ipcrtr_get_status;

	/* Power Save Features Not supported due to latencies */
	// ipcrtr->function.suspend = NULL;
	// ipcrtr->function.func_suspend = NULL;
	// ipcrtr->function.resume = NULL;

	status = usb_add_function(c, &ipcrtr->function);

	return status;
}

/**
 * ipcrtr_init() - USB function called from the android gadget framework
 *
 */
static int ipcrtr_init(void)
{
	struct f_ipcrtr *ipcrtr;
	int ret = 0;

	log_event_dbg("%s: -- Init from android framework \n", __func__);
	ipcrtr = kzalloc(sizeof(*ipcrtr), GFP_KERNEL);
	if (!ipcrtr) {
		return -ENOMEM;
	}

	ipcrtr_ctx = ipcrtr;

	ret = ipcrtr_function_ctrl_port_init();
	if (ret) {
		ret = -1;
		goto error;
	}

	ipcrtr->ipcrtr_wq = create_singlethread_workqueue("f_ipcrtr");
	if (!ipcrtr->ipcrtr_wq) {
		log_event_err("%s: WQ creation failed for %s",
						__func__,"f_ipcrtr");
		ret -EFAULT;
		goto error;
	}

	INIT_DELAYED_WORK(&ipcrtr->init_work, usb_xprt_init);
	INIT_DELAYED_WORK(&ipcrtr->exit_work, usb_xprt_deinit);

	ret = usb_ipcrtr_debugfs_init();
	if (ret)
		log_event_err("%s: Debugfs not inited ", __func__);

	return ret;
error:
	kfree(ipcrtr);
	ipcrtr_ctx = NULL;
	return ret;
}

/**
 * ipcrtr_cleanup() - USB function called from the android gadget framework
 *
 */
static void ipcrtr_cleanup(void)
{
	struct f_ipcrtr *ipcrtr = ipcrtr_ctx;

	log_event_dbg("%s: -- Cleanup from android framework \n", __func__);
	if (unlikely(!ipcrtr)) {
		log_event_err("%s: Invalid pointer", __func__);
		return;
	}

	if (ipcrtr->ipcrtr_wq)
		destroy_workqueue(ipcrtr->ipcrtr_wq);

	kfree(ipcrtr_ctx);
	ipcrtr_ctx = NULL;
}

static int __init fipcrtr_init(void)
{
	log_event_info("%s: -- Init IPCRTR Module \n", __func__);

	return 0;
}

static void __exit fipcrtr_exit(void)
{
	usb_ipcrtr_debugfs_exit();
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IPC Router function driver");

module_init(fipcrtr_init);
module_exit(fipcrtr_exit);
