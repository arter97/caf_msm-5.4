/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
 * CAN driver for EMS Dr. Thomas Wuensche CPC-USB/ARM7
 *
 * Copyright (C) 2004-2009 EMS Dr. Thomas Wuensche
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/slab.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#define DEBUG_SAM4E		0
#if DEBUG_SAM4E == 1
#define LOGNI(...) netdev_info(netdev, __VA_ARGS__)
#else
#define LOGNI(...)
#endif

#define BULK_IN_EP		3
#define BULK_OUT_EP		4

#define MAX_RX_URBS		10
#define MAX_TX_URBS		10
#define RX_BUFFER_SIZE		256
#define RX_ASSEMBLY_BUFFER_SIZE	64

#define CMD_SYS_SET_BOARD_PWR	0x2
#define CMD_SYS_GET_BOARD_PWR	0x3
#define CMD_SYS_READ_U32		0x4
#define CMD_SYS_WRITE_U32		0x5
#define CMD_SYS_GET_FW_VERSION	0x6
#define CMD_CAN_GET_MID			0x52
#define CMD_CAN_SET_MID			0x53
#define CMD_CAN_GET_FILTER		0x54
#define CMD_CAN_SET_FILTER		0x55
#define CMD_CAN_GET_PRIORITY	0x56
#define CMD_CAN_SET_PRIORITY	0x57
#define CMD_CAN_GET_MODE		0x58
#define CMD_CAN_SET_MODE		0x59
#define CMD_CAN_GET_TIMESTAMP	0x5A
#define CMD_CAN_READ			0x5B
#define CMD_CAN_WRITE			0x5C
#define CMD_CAN_READ_ASYNC		0x5D
#define CMD_CAN_FULL_WRITE		0x5E
#define CMD_CAN_FULL_READ		0x5F
#define CMD_CAN_TS_READ_ASYNC	0x60
#define CMD_CAN_TS_FULL_READ	0x61


struct sam4e_usb {
	struct can_priv can;

	struct usb_device *udev;
	struct net_device *netdev;

	atomic_t active_tx_urbs;
	atomic_t msg_seq;
	struct usb_anchor tx_submitted;
	struct usb_anchor rx_submitted;
	char *assembly_buffer;
	u8 assembly_buffer_size;
};

#define SAM4E_USB_VENDOR_ID 0x05C6
#define SAM4E_USB_PRODUCT_ID 0x9460

/*
 * Table of devices that work with this driver
 * NOTE: This driver supports only SAM4E usb device.
 */
static struct usb_device_id sam4e_usb_table[] = {
		{USB_DEVICE(SAM4E_USB_VENDOR_ID, SAM4E_USB_PRODUCT_ID)},
		{} /* Terminating entry */
};

/* Messages structures */
struct sam4e_req {
	u8 cmd;
	u8 len;
	u16 seq;
	u8 data[0];
} __packed;

struct sam4e_resp {
	u8 cmd;
	u8 len;
	u16 seq;
	u8 err;
	u8 data[0];
} __packed;

struct sam4e_unsol_msg {
	u8 cmd;
	u8 len;
	u16 seq;
	u8 data[0];
} __packed;

struct sam4e_fw_resp {
	u8 maj;
	u8 min;
	u8 ver[32];
} __packed;

struct sam4e_set_mid {
	u8 can;
	u8 mailbox;
	u32 mid;
} __packed;

struct sam4e_set_mode {
	u8 can;
	u8 mailbox;
	u8 mode;
} __packed;

struct sam4e_set_prio {
	u8 can;
	u8 mailbox;
	u8 prio;
} __packed;

struct sam4e_can_write {
	u8 can;
	u8 mailbox;
	u8 dlc;
	u8 data[8];
} __packed;

struct sam4e_can_full_write {
	u8 can;
	u8 mailbox;
	u8 prio;
	u32 mid;
	u8 dlc;
	u8 data[8];
} __packed;

struct sam4e_can_full_read {
	u8 can;
	u8 mailbox;
	u32 mid;
	u32 filter;
} __packed;

struct sam4e_can_unsl_receive {
	u8 can;
	u8 mailbox;
	u32 mid;
	u8 dlc;
	u8 data[8];
} __packed;

struct sam4e_can_full_ts_read {
	u8 can;
	u8 mailbox;
	u8 ts_on;
	u32 mid;
	u32 filter;
} __packed;

struct sam4e_can_unsl_ts_receive {
	u8 can;
	u8 mailbox;
	u32 ts;
	u32 mid;
	u8 dlc;
	u8 data[8];
} __packed;

static void sam4e_usb_receive_frame(struct net_device *netdev,
		struct sam4e_can_unsl_ts_receive *frame)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct skb_shared_hwtstamps *skt;
	struct timeval tv;
	static int msec;
	int i;

	skb = alloc_can_skb(netdev, &cf);
	if (skb == NULL)
		return;

	LOGNI(" rcv frame %d %x %d %x %x %x %x %x %x %x %x\n",
			frame->ts, frame->mid, frame->dlc, frame->data[0],
			frame->data[1], frame->data[2], frame->data[3],
			frame->data[4], frame->data[5], frame->data[6],
			frame->data[7]);
	cf->can_id = le32_to_cpu(frame->mid);
	cf->can_dlc = get_can_dlc(frame->dlc);

	for (i = 0; i < cf->can_dlc; i++)
		cf->data[i] = frame->data[i];

	msec = le32_to_cpu(frame->ts);
	tv.tv_sec = msec / 1000;
	tv.tv_usec = (msec - tv.tv_sec * 1000) * 1000;
	skt = skb_hwtstamps(skb);
	skt->hwtstamp = timeval_to_ktime(tv);
	LOGNI("   hwtstamp %lld\n", ktime_to_ms(skt->hwtstamp));
	skb->tstamp = timeval_to_ktime(tv);
	netif_rx(skb);
}

static void sam4e_process_response(struct sam4e_usb *dev,
				   struct sam4e_resp *resp, int length)
{
	struct net_device *netdev = dev->netdev;
	LOGNI("<%x %2d [%d] %d buff:[%d]\n", resp->cmd,
			resp->len, resp->seq, resp->err,
			atomic_read(&dev->active_tx_urbs));
	if (resp->cmd == CMD_CAN_TS_READ_ASYNC) {
		/* v1.3 of the firmware uses different frame structure for
		   unsol messages. (the one without error code) */
		struct sam4e_unsol_msg *msg = (struct sam4e_unsol_msg *)resp;
		/* v2.2 of the firmware supports TS_READ_ASYNC */
		struct sam4e_can_unsl_ts_receive *frame =
				(struct sam4e_can_unsl_ts_receive *)&msg->data;
		if (msg->len > length) {
			LOGNI("process_response: Saving %d bytes of response\n",
					length);
			memcpy(dev->assembly_buffer, (char *)resp, length);
			dev->assembly_buffer_size = length;
		} else {
			sam4e_usb_receive_frame(netdev, frame);
		}
	} else if (resp->cmd == CMD_CAN_FULL_WRITE) {
		atomic_dec(&dev->active_tx_urbs);
		if (netif_queue_stopped(netdev) &&
		    atomic_read(&dev->active_tx_urbs) < MAX_TX_URBS) {
			LOGNI("Waking up queue. (%d)\n",
			      atomic_read(&dev->active_tx_urbs));
			netif_wake_queue(netdev);
		}
	}
}

static void sam4e_usb_read_bulk_callback(struct urb *urb)
{
	struct sam4e_usb *dev = urb->context;
	struct net_device *netdev = dev->netdev;
	int err, length_processed = 0;

	LOGNI("sam4e_usb_read_bulk_callback length: %d\n", urb->actual_length);
	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0:
		break;

	case -ENOENT:
		return;

	default:
		LOGNI("Rx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

	while (length_processed < urb->actual_length) {
		int length_left = urb->actual_length - length_processed;
		int length = 0; /* length of consumed chunk */
		void *data;
		if (dev->assembly_buffer_size > 0) {
			struct sam4e_resp *resp;
			LOGNI("callback: Reassembling %d bytes of response\n",
					dev->assembly_buffer_size);
			memcpy(dev->assembly_buffer + dev->assembly_buffer_size,
					urb->transfer_buffer, 2);
			data = dev->assembly_buffer;
			resp = (struct sam4e_resp *)data;
			length = resp->len - dev->assembly_buffer_size;
			if (length > 0) {
				memcpy(dev->assembly_buffer +
						dev->assembly_buffer_size,
						urb->transfer_buffer, length);
			}
			length_left += dev->assembly_buffer_size;
			dev->assembly_buffer_size = 0;
		} else {
			struct sam4e_resp *resp;
			data = urb->transfer_buffer + length_processed;
			resp = (struct sam4e_resp *)data;
			length = resp->len;
		}
		LOGNI("processing. p %d -> l %d (t %d)\n",
				length_processed, length_left,
				urb->actual_length);
		length_processed += length;
		if (length_left >= sizeof(struct sam4e_resp)) {
			struct sam4e_resp *resp =
					(struct sam4e_resp *)data;
			if (resp->len < sizeof(struct sam4e_resp)) {
				LOGNI("Error resp->len is %d). Abort.\n",
						resp->len);
				break;
			}
			sam4e_process_response(dev, resp, length_left);
		} else if (length_left > 0) {
			/* Not full message. Store however much we have for
			   later assembly */
			LOGNI("callback: Storing %d bytes of response\n",
					length_left);
			memcpy(dev->assembly_buffer, data, length_left);
			dev->assembly_buffer_size = length_left;
			break;
		} else {
			break;
		}
	}

	if (urb->actual_length > length_processed) {
		LOGNI("Error length > length_processed: %d > %d (needed: %d)\n",
				urb->actual_length, length_processed,
				sizeof(struct sam4e_resp));
	}

resubmit_urb:
	LOGNI("Resubmitting Rx Urb\n");
	usb_fill_bulk_urb(urb, dev->udev,
			usb_rcvbulkpipe(dev->udev, BULK_IN_EP),
			urb->transfer_buffer, RX_BUFFER_SIZE,
			sam4e_usb_read_bulk_callback, dev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, RX_BUFFER_SIZE,
				urb->transfer_buffer,
				urb->transfer_dma);
		LOGNI("Failed to resubmit Rx Urb: %d\n", err);
	}
}

static int sam4e_init_urbs(struct net_device *netdev)
{
	int err, i;
	struct sam4e_usb *dev = netdev_priv(netdev);
	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf = NULL;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			netdev_err(netdev, "No memory left for URBs\n");
			err = -ENOMEM;
			break;
		}

		buf = usb_alloc_coherent(dev->udev, RX_BUFFER_SIZE, GFP_KERNEL,
				&urb->transfer_dma);
		if (!buf) {
			netdev_err(netdev, "No memory left for USB buffer\n");
			usb_free_urb(urb);
			err = -ENOMEM;
			break;
		}

		usb_fill_bulk_urb(urb, dev->udev,
				usb_rcvbulkpipe(dev->udev, BULK_IN_EP),
				buf, RX_BUFFER_SIZE,
				sam4e_usb_read_bulk_callback, dev);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &dev->rx_submitted);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			netdev_err(netdev, "Failed to init RX urb %d\n", err);
			usb_unanchor_urb(urb);
			usb_free_coherent(dev->udev, RX_BUFFER_SIZE, buf,
					urb->transfer_dma);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}

	if (err) {
		if (err == -ENODEV)
			netif_device_detach(netdev);

		netdev_warn(netdev, "couldn't start device: %d\n", err);
		close_candev(netdev);
		return err;
	}

	return err;
}

static int sam4e_netdev_open(struct net_device *netdev)
{
	int err;

	LOGNI("sam4e_open");
	err = open_candev(netdev);
	if (err)
		return err;

	netif_start_queue(netdev);

	return 0;
}

static void unlink_all_urbs(struct sam4e_usb *dev)
{
	usb_kill_anchored_urbs(&dev->rx_submitted);
	usb_kill_anchored_urbs(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);
}

static int sam4e_netdev_close(struct net_device *netdev)
{
	struct sam4e_usb *dev = netdev_priv(netdev);
	LOGNI("sam4e_close");

	/* Stop polling */
	unlink_all_urbs(dev);

	netif_stop_queue(netdev);
	close_candev(netdev);
	return 0;
}

static void sam4e_usb_write_bulk_callback(struct urb *urb)
{
	struct sam4e_usb *dev = urb->context;
	struct net_device *netdev = dev->netdev;
	usb_unanchor_urb(urb);
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);

	if (urb->status)
		netdev_err(netdev, "Tx URB aborted (%d)\n", urb->status);

	netdev->stats.tx_packets++;
}

static netdev_tx_t sam4e_netdev_start_xmit(
		struct sk_buff *skb, struct net_device *netdev)
{
	struct sam4e_req *req;
	struct usb_device *udev;
	struct sam4e_usb *dev = netdev_priv(netdev);
	struct net_device_stats *stats = &netdev->stats;
	int result;
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct urb *urb;
	size_t size = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_write);
	struct sam4e_can_full_write *cfw;

	if (can_dropped_invalid_skb(netdev, skb)) {
		netdev_err(netdev, "Dropping invalid can frame");
		return NETDEV_TX_OK;
	}

	udev = dev->udev;
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		netdev_err(netdev, "No memory left for URBs\n");
		goto nomem;
	}

	req = usb_alloc_coherent(dev->udev, size, GFP_ATOMIC,
			&urb->transfer_dma);
	if (!req) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	/* Fill message data */
	cfw = (struct sam4e_can_full_write *)&req->data;
	req->cmd = CMD_CAN_FULL_WRITE;
	req->len = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_write);
	req->seq = atomic_inc_return(&dev->msg_seq);
	cfw->can = 0;
	cfw->mailbox = 0;
	cfw->prio = 0;
	cfw->mid = cf->can_id;
	cfw->dlc = cf->can_dlc;
	memcpy(cfw->data, cf->data, 8);

	LOGNI(">%x %2d [%d] send frame [%d] %x %d %x %x %x %x %x %x %x %x\n",
			req->cmd, req->len, req->seq,
			atomic_read(&dev->active_tx_urbs), cfw->mid,
			cfw->dlc, cfw->data[0], cfw->data[1], cfw->data[2],
			cfw->data[3], cfw->data[4], cfw->data[5],
			cfw->data[6], cfw->data[7]);

	usb_fill_bulk_urb(urb, dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP), req,
			size, sam4e_usb_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->tx_submitted);
	atomic_inc(&dev->active_tx_urbs);

	result = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(result)) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, req, urb->transfer_dma);
		dev_kfree_skb(skb);

		atomic_dec(&dev->active_tx_urbs);
		if (result == -ENODEV) {
			netif_device_detach(netdev);
		} else {
			netdev_err(netdev, "failed tx_urb %d\n", result);
			stats->tx_dropped++;
		}
	} else {
		/* Put on hold tx path */
		if (atomic_read(&dev->active_tx_urbs) >= MAX_TX_URBS) {
			LOGNI("Too many outstanding requests (%d). Stop queue",
					atomic_read(&dev->active_tx_urbs));
			netif_stop_queue(netdev);
		}
	}
	dev_kfree_skb(skb);
	usb_free_urb(urb);
	return NETDEV_TX_OK;

nomem:
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

static const struct net_device_ops sam4e_usb_netdev_ops = {
		.ndo_open = sam4e_netdev_open,
		.ndo_stop = sam4e_netdev_close,
		.ndo_start_xmit = sam4e_netdev_start_xmit,
};

/*
 * probe function for usb devices
 */
static int sam4e_usb_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_host_interface *usb_hi;
	struct usb_interface_descriptor *usb_if_desc;
	struct usb_device *udev;
	struct sam4e_req *req;
	struct sam4e_resp *resp;
	int actual_length;
	int result;
	int err = 0;
	u8 if_num;
	struct net_device *netdev;
	struct sam4e_usb *dev;

	usb_hi = intf->cur_altsetting;
	usb_if_desc = &usb_hi->desc;
	if_num = usb_if_desc->bInterfaceNumber;
	if (DEBUG_SAM4E) {
		dev_info(&intf->dev, "probe %p %p if_num %d id if_num %d\n",
			intf, id, if_num, id->bInterfaceNumber);
	}

	if (if_num != 1)
		return -ENODEV;

	netdev = alloc_candev(sizeof(struct sam4e_usb), MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "sam4e_usb: Couldn't alloc candev\n");
		return -ENOMEM;
	}

	dev = netdev_priv(netdev);
	dev->udev = interface_to_usbdev(intf);
	dev->netdev = netdev;
	dev->assembly_buffer = kzalloc(RX_ASSEMBLY_BUFFER_SIZE, GFP_KERNEL);

	netdev->netdev_ops = &sam4e_usb_netdev_ops;

	init_usb_anchor(&dev->rx_submitted);
	init_usb_anchor(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);
	atomic_set(&dev->msg_seq, 0);

	usb_set_intfdata(intf, dev);
	SET_NETDEV_DEV(netdev, &intf->dev);

	err = register_candev(netdev);
	if (err) {
		netdev_err(netdev, "couldn't register CAN device: %d\n", err);
		goto cleanup_candev;
	}

	udev = interface_to_usbdev(intf);

	/* TODO: This is probe function! Don't do lengthy stuff here */
	/* Set mailbox 7 to read all EFF msgs */
	req = kzalloc(sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_ts_read), GFP_KERNEL);
	if (req != 0) {
		struct sam4e_can_full_ts_read *cfr =
				(struct sam4e_can_full_ts_read *)&req->data;
		req->cmd = CMD_CAN_TS_FULL_READ;
		req->len = sizeof(struct sam4e_req) +
				sizeof(struct sam4e_can_full_ts_read);
		req->seq = atomic_inc_return(&dev->msg_seq);
		cfr->can = 0;
		cfr->mailbox = 7;
		/* Timestamping on */
		cfr->ts_on = 1;
		/* accept EFF frames here */
		cfr->mid = CAN_EFF_FLAG;
		cfr->filter = CAN_EFF_FLAG;

		result = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_EP),
				req,
				sizeof(struct sam4e_req) +
				sizeof(struct sam4e_can_full_ts_read),
				&actual_length, 1000);

		LOGNI("sent %x result %d, actual_length %d",
				req->cmd, result, actual_length);
		kfree(req);
	}

	resp = kzalloc(sizeof(struct sam4e_resp), GFP_KERNEL);
	result =  usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_EP),
			resp,
			sizeof(struct sam4e_resp),
			&actual_length, 1000);

	LOGNI("rcv? result %d, actual_length %d",
			result, actual_length);
	if (result == 0 && DEBUG_SAM4E) {
		dev_info(&intf->dev, "data %x %d %d %d\n", resp->cmd,
				resp->len, resp->seq, resp->err);
	}
	kfree(resp);

	/* Set mailbox 6 to read all SFF msgs */
	req = kzalloc(sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_ts_read), GFP_KERNEL);
	if (req != 0) {
		struct sam4e_can_full_ts_read *cfr =
				(struct sam4e_can_full_ts_read *)&req->data;
		req->cmd = CMD_CAN_TS_FULL_READ;
		req->len = sizeof(struct sam4e_req) +
				sizeof(struct sam4e_can_full_ts_read);
		req->seq = atomic_inc_return(&dev->msg_seq);
		cfr->can = 0;
		cfr->mailbox = 6;
		/* Timestamping on */
		cfr->ts_on = 1;
		/* accept SFF frames here */
		cfr->mid = 0;
		cfr->filter = CAN_EFF_FLAG;

		result =  usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_EP),
				req,
				sizeof(struct sam4e_req) +
				sizeof(struct sam4e_can_full_ts_read),
				&actual_length, 1000);

		LOGNI("sent %x result %d, actual_length %d",
				req->cmd, result, actual_length);
		kfree(req);
	}

	resp = kzalloc(sizeof(struct sam4e_resp), GFP_KERNEL);
	result =  usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_EP),
			resp,
			sizeof(struct sam4e_resp),
			&actual_length, 1000);
	LOGNI("rcv? result %d, actual_length %d",
			result, actual_length);
	if (result == 0)
		LOGNI("data %x %d %d %d\n",
				resp->cmd, resp->len, resp->seq, resp->err);
	kfree(resp);

	err = sam4e_init_urbs(netdev);
	if (err) {
		netdev_err(netdev, "couldn't init urbs: %d\n", err);
		goto unregister_candev;
	}

	return 0; /*ok. it's ours */

unregister_candev:
	unregister_netdev(netdev);
cleanup_candev:
	kfree(dev->assembly_buffer);
	free_candev(netdev);
	return err;
}

/*
 * called by the usb core when the device is removed from the system
 */
static void sam4e_usb_disconnect(struct usb_interface *intf)
{
	struct sam4e_usb *dev;
	struct net_device *netdev;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	if (dev) {
		netdev = dev->netdev;
		LOGNI("Disconnect sam4e\n");
		unregister_netdev(dev->netdev);
		kfree(dev->assembly_buffer);
		free_candev(dev->netdev);
		unlink_all_urbs(dev);
	}
}

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver sam4e_usb_driver = {
		.name = "sam4e_usb",
		.probe = sam4e_usb_probe,
		.disconnect = sam4e_usb_disconnect,
		.id_table = sam4e_usb_table,
};

module_usb_driver(sam4e_usb_driver);

MODULE_DESCRIPTION("SAM4E USB-CAN module");
MODULE_LICENSE("GPL v2");
