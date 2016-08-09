/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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

#define ENABLE_TESTS	0
#if ENABLE_TESTS == 1
#define LOGTEST(...) dev_info(&dev->udev->dev, __VA_ARGS__)
#else
#define LOGTEST(...)
#endif

#define DEBUG_SAM4E	0
#if DEBUG_SAM4E == 1
#define LOGDI(...) dev_info(&dev->udev->dev, __VA_ARGS__)
#define LOGNI(...) netdev_info(netdev, __VA_ARGS__)
#else
#define LOGDI(...)
#define LOGNI(...)
#endif
#define LOGDE(...) dev_err(&dev->udev->dev, __VA_ARGS__)
#define LOGNE(...) netdev_err(netdev, __VA_ARGS__)

#define BULK_IN_EP	3
#define BULK_OUT_EP	4

#define MAX_RX_URBS			10
#define MAX_TX_URBS			10
#define RX_BUFFER_SIZE			256
#define RX_ASSEMBLY_BUFFER_SIZE		64

#define CMD_SYS_SET_BOARD_PWR		0x2
#define CMD_SYS_GET_BOARD_PWR		0x3
#define CMD_SYS_READ_U32		0x4
#define CMD_SYS_WRITE_U32		0x5
#define CMD_SYS_GET_FW_VERSION		0x6
#define CMD_CAN_GET_MID			0x52
#define CMD_CAN_SET_MID			0x53
#define CMD_CAN_GET_FILTER		0x54
#define CMD_CAN_SET_FILTER		0x55
#define CMD_CAN_GET_PRIORITY		0x56
#define CMD_CAN_SET_PRIORITY		0x57
#define CMD_CAN_GET_MODE		0x58
#define CMD_CAN_SET_MODE		0x59
#define CMD_CAN_GET_TIMESTAMP		0x5A
#define CMD_CAN_READ			0x5B
#define CMD_CAN_WRITE			0x5C
#define CMD_CAN_READ_ASYNC		0x5D
#define CMD_CAN_FULL_WRITE		0x5E
#define CMD_CAN_FULL_READ		0x5F
#define CMD_CAN_TS_READ_ASYNC		0x60
#define CMD_CAN_TS_FULL_READ		0x61
#define MAX_CAN_INTF			0x02

#define CMD_CAN_RELEASE_BUFFER		0x62
#define CMD_CAN_ENABLE_BUFFERING	0x63
#define CMD_CAN_DISABLE_BUFFERING	0X64
#define CMD_CAN_DISABLE_ALL_BUFFERING	0x65
#define CMD_CAN_ADD_FRAME_FILTER	0x66
#define CMD_CAN_REMOVE_FRAME_FILTER	0x67
#define CMD_CAN_UPDATE_FRAME_FILTER	0x68


#define IOCTL_RELEASE_CAN_BUFFER	(SIOCDEVPRIVATE + 0)
#define IOCTL_ENABLE_BUFFERING		(SIOCDEVPRIVATE + 1)
#define IOCTL_ADD_FRAME_FILTER		(SIOCDEVPRIVATE + 2)
#define IOCTL_REMOVE_FRAME_FILTER	(SIOCDEVPRIVATE + 3)
#define IOCTL_UPDATE_FRAME_FILTER	(SIOCDEVPRIVATE + 4)

#define FRAME_FILTER_TYPE_ON_REFRESH    0
#define FRAME_FILTER_TYPE_ON_CHANGE     1

struct sam4e_usb {
	struct usb_device *udev;
	struct net_device *netdev1;
	struct net_device *netdev2;

	atomic_t active_tx_urbs;
	atomic_t msg_seq;
	struct usb_anchor tx_submitted;
	struct usb_anchor rx_submitted;
	char *assembly_buffer;
	u8 assembly_buffer_size;
	atomic_t netif_queue_stop;
};
struct sam4e_usb_handle {
	struct can_priv can;
	struct sam4e_usb *sam4e_dev;
	u8 owner_netdev_index;
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

/* IOCTL messages */
struct sam4e_release_can_buffer {
	u8 enable;
} __packed;

struct sam4e_enable_buffering {
	u8 can;
	u32 mid;
	u32 mask;
} __packed;

struct sam4e_frame_filter {
	u8 can;
	u32 mid;
	u32 mask;
	u8 type;
} __packed;

static void sam4e_usb_receive_frame(struct sam4e_usb *dev,
		struct sam4e_can_unsl_ts_receive *frame)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct skb_shared_hwtstamps *skt;
	struct timeval tv;
	static int msec;
	int i;
	if (frame->can == 0)
		skb = alloc_can_skb(dev->netdev1, &cf);
	else
		skb = alloc_can_skb(dev->netdev2, &cf);
	if (skb == NULL) {
		pr_err("skb failed..frame->can %d", frame->can);
		return;
	}

	LOGDI(" rcv frame %d %x %d %x %x %x %x %x %x %x %x\n",
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
	LOGDI("   hwtstamp %lld\n", ktime_to_ms(skt->hwtstamp));
	skb->tstamp = timeval_to_ktime(tv);
	netif_rx(skb);
}

static void sam4e_process_response(struct sam4e_usb *dev,
				   struct sam4e_resp *resp, int length)
{
	LOGDI("<%x %2d [%d] %d buff:[%d]\n", resp->cmd,
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
			LOGDI("process_response: Saving %d bytes of response\n",
					length);
			memcpy(dev->assembly_buffer, (char *)resp, length);
			dev->assembly_buffer_size = length;
		} else {
			sam4e_usb_receive_frame(dev, frame);
		}
	} else if (resp->cmd == CMD_CAN_FULL_WRITE) {
		atomic_dec(&dev->active_tx_urbs);
		if ((atomic_read(&dev->netif_queue_stop) == 1) &&
		    atomic_read(&dev->active_tx_urbs) < MAX_TX_URBS) {
			LOGDI("Waking up queue. (%d)\n",
			      atomic_read(&dev->active_tx_urbs));
			netif_wake_queue(dev->netdev1);
			netif_wake_queue(dev->netdev2);
			atomic_dec(&dev->netif_queue_stop);
		}
	} else if (resp->cmd == CMD_CAN_RELEASE_BUFFER) {
		LOGDI("Received CMD_CAN_RELEASE_BUFFER response\n");
	} else if (resp->cmd == CMD_CAN_ENABLE_BUFFERING) {
		LOGDI("Received CMD_CAN_ENABLE_BUFFERING response\n");
	} else if (resp->cmd == CMD_SYS_GET_FW_VERSION) {
		struct sam4e_fw_resp *fw;
		LOGDI("Received CMD_SYS_GET_FW_VERSION response\n");
		fw = (struct sam4e_fw_resp *)resp->data;
		dev_info(&dev->udev->dev, "sam4e fw %d.%d", fw->maj, fw->min);
		dev_info(&dev->udev->dev, "sam4e fw string %s", fw->ver);
	} else {
		LOGDI("Received unknown response %x\n", resp->cmd);
	}
}

static void sam4e_usb_read_bulk_callback(struct urb *urb)
{
	struct sam4e_usb *dev = urb->context;
	int err, length_processed = 0;

	LOGDI("sam4e_usb_read_bulk_callback length: %d\n", urb->actual_length);

	switch (urb->status) {
	case 0:
		break;

	case -ENOENT:
		return;

	default:
		pr_err("Rx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

	while (length_processed < urb->actual_length) {
		int length_left = urb->actual_length - length_processed;
		int length = 0; /* length of consumed chunk */
		void *data;
		if (dev->assembly_buffer_size > 0) {
			struct sam4e_resp *resp;
			LOGDI("callback: Reassembling %d bytes of response\n",
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
		LOGDI("processing. p %d -> l %d (t %d)\n",
				length_processed, length_left,
				urb->actual_length);
		length_processed += length;
		if (length_left >= sizeof(struct sam4e_resp)) {
			struct sam4e_resp *resp =
					(struct sam4e_resp *)data;
			if (resp->len < sizeof(struct sam4e_resp)) {
				LOGDE("Error resp->len is %d). Abort.\n",
						resp->len);
				break;
			}
			sam4e_process_response(dev, resp, length_left);
		} else if (length_left > 0) {
			/* Not full message. Store however much we have for
			   later assembly */
			LOGDI("callback: Storing %d bytes of response\n",
					length_left);
			memcpy(dev->assembly_buffer, data, length_left);
			dev->assembly_buffer_size = length_left;
			break;
		} else {
			break;
		}
	}

	if (urb->actual_length > length_processed) {
		LOGDE("Error length > length_processed: %d > %d (needed: %d)\n",
				urb->actual_length, length_processed,
				sizeof(struct sam4e_resp));
	}

resubmit_urb:
	LOGDI("Resubmitting Rx Urb\n");
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
		pr_err("Failed to resubmit Rx Urb: %d\n", err);
	}
}

static int sam4e_init_urbs(struct sam4e_usb *dev)
{
	int err, i;
	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf = NULL;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			pr_err("No memory left for URBs\n");
			err = -ENOMEM;
			break;
		}

		buf = usb_alloc_coherent(dev->udev, RX_BUFFER_SIZE, GFP_KERNEL,
				&urb->transfer_dma);
		if (!buf) {
			pr_err("No memory left for USB buffer\n");
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
			pr_err("Failed to init RX urb %d\n", err);
			usb_unanchor_urb(urb);
			usb_free_coherent(dev->udev, RX_BUFFER_SIZE, buf,
					urb->transfer_dma);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
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
	struct sam4e_usb_handle *sam4e_usb_hnd = netdev_priv(netdev);
	struct sam4e_usb *dev = sam4e_usb_hnd->sam4e_dev;
	LOGNI("sam4e_close");

	/* Stop polling */
	unlink_all_urbs(dev);

	netif_stop_queue(netdev);
	close_candev(netdev);
	return 0;
}

static void sam4e_usb_write_bulk_callback(struct urb *urb)
{
	struct net_device *netdev = urb->context;
	usb_unanchor_urb(urb);
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);

	if (urb->status)
		pr_err("Tx URB aborted (%d)\n", urb->status);

	netdev->stats.tx_packets++;
}

static netdev_tx_t sam4e_netdev_start_xmit(
		struct sk_buff *skb, struct net_device *netdev)
{
	struct sam4e_req *req;
	struct usb_device *udev;
	struct sam4e_usb_handle *sam4e_usb_hnd = netdev_priv(netdev);
	struct sam4e_usb *dev = sam4e_usb_hnd->sam4e_dev;
	struct net_device_stats *stats = &netdev->stats;
	int result;
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct urb *urb;
	size_t size = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_write);
	struct sam4e_can_full_write *cfw;

	if (can_dropped_invalid_skb(netdev, skb)) {
		pr_err("Dropping invalid can frame");
		return NETDEV_TX_OK;
	}

	udev = dev->udev;
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("No memory left for URBs\n");
		goto nomem;
	}

	req = usb_alloc_coherent(dev->udev, size, GFP_ATOMIC,
			&urb->transfer_dma);
	if (!req) {
		pr_err("No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	/* Fill message data */
	cfw = (struct sam4e_can_full_write *)&req->data;
	req->cmd = CMD_CAN_FULL_WRITE;
	req->len = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_write);
	req->seq = atomic_inc_return(&dev->msg_seq);
	cfw->can = sam4e_usb_hnd->owner_netdev_index;
	cfw->mailbox = 0;
	cfw->prio = 0;
	cfw->mid = cf->can_id;
	cfw->dlc = cf->can_dlc;
	memcpy(cfw->data, cf->data, 8);

	LOGDI(">%x %2d [%d] send frame [%d] %x %d %x %x %x %x %x %x %x %x\n",
			req->cmd, req->len, req->seq,
			atomic_read(&dev->active_tx_urbs), cfw->mid,
			cfw->dlc, cfw->data[0], cfw->data[1], cfw->data[2],
			cfw->data[3], cfw->data[4], cfw->data[5],
			cfw->data[6], cfw->data[7]);

	usb_fill_bulk_urb(urb, dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP), req,
			size, sam4e_usb_write_bulk_callback, netdev);
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
			pr_err("failed tx_urb %d\n", result);
			stats->tx_dropped++;
		}
	} else {
		/* Put on hold tx path */
		if (atomic_read(&dev->active_tx_urbs) >= MAX_TX_URBS) {
			LOGDI("Too many outstanding requests (%d). Stop queue",
					atomic_read(&dev->active_tx_urbs));
			atomic_inc(&dev->netif_queue_stop);
			if (dev->netdev1)
				netif_stop_queue(dev->netdev1);
			if (dev->netdev2)
				netif_stop_queue(dev->netdev2);
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

static int sam4e_send_release_can_buffer_cmd(struct net_device *netdev)
{
	struct urb *urb;
	struct sam4e_req *req;
	struct sam4e_release_can_buffer *release_can_buffer;
	struct sam4e_usb_handle *sam4e_usb_hnd = netdev_priv(netdev);
	struct sam4e_usb *dev = sam4e_usb_hnd->sam4e_dev;
	size_t size = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_release_can_buffer);
	int result;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		netdev_err(netdev, "No memory left for URBs\n");
		goto nomem;
	}

	req = usb_alloc_coherent(dev->udev, size, GFP_KERNEL,
			&urb->transfer_dma);
	if (!req) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	/* Fill message data */
	req->cmd = CMD_CAN_RELEASE_BUFFER;
	req->len = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_release_can_buffer);
	req->seq = atomic_inc_return(&dev->msg_seq);

	release_can_buffer = (struct sam4e_release_can_buffer *)&req->data;
	release_can_buffer->enable = 1;

	usb_fill_bulk_urb(urb, dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP), req,
			size, sam4e_usb_write_bulk_callback, netdev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->tx_submitted);

	result = usb_submit_urb(urb, GFP_KERNEL);
	if (unlikely(result)) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, req, urb->transfer_dma);
	}
	usb_free_urb(urb);
	return result;

nomem:
	return -ENOMEM;
}

static int sam4e_enable_buffering(struct net_device *netdev,
		struct ifreq *ifr)
{
	struct sam4e_enable_buffering *add_request;
	struct sam4e_enable_buffering *enable_buffering;
	struct urb *urb;
	struct sam4e_req *req;
	struct sam4e_usb_handle *sam4e_usb_hnd = netdev_priv(netdev);
	struct sam4e_usb *dev = sam4e_usb_hnd->sam4e_dev;
	size_t size = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_enable_buffering);
	int result;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		netdev_err(netdev, "No memory left for URBs\n");
		goto nomem;
	}

	req = usb_alloc_coherent(dev->udev, size, GFP_KERNEL,
			&urb->transfer_dma);
	if (!req) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	add_request = ifr->ifr_data;
	/* Fill message data */
	req->cmd = CMD_CAN_ENABLE_BUFFERING;
	req->len = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_enable_buffering);
	req->seq = atomic_inc_return(&dev->msg_seq);
	enable_buffering = (struct sam4e_enable_buffering *)&req->data;
	enable_buffering->can = add_request->can;
	enable_buffering->mid = add_request->mid;
	enable_buffering->mask = add_request->mask;

	LOGDI("sam4e_send_enable_buffering %d %x %x", enable_buffering->can,
			enable_buffering->mid,
			enable_buffering->mask);

	usb_fill_bulk_urb(urb, dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP), req,
			size, sam4e_usb_write_bulk_callback, netdev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->tx_submitted);

	result = usb_submit_urb(urb, GFP_KERNEL);
	if (unlikely(result)) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, req, urb->transfer_dma);
	}
	usb_free_urb(urb);
	return result;

nomem:
	return -ENOMEM;
}

static int sam4e_frame_filter(struct net_device *netdev,
		struct ifreq *ifr, int cmd)
{
	struct sam4e_frame_filter *add_request;
	struct sam4e_frame_filter *frame_filter;
	struct urb *urb;
	struct sam4e_req *req;
	struct sam4e_usb_handle *sam4e_usb_hnd = netdev_priv(netdev);
	struct sam4e_usb *dev = sam4e_usb_hnd->sam4e_dev;
	size_t size = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_frame_filter);
	int result;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		netdev_err(netdev, "No memory left for URBs\n");
		goto nomem;
	}

	req = usb_alloc_coherent(dev->udev, size, GFP_KERNEL,
			&urb->transfer_dma);
	if (!req) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	add_request = ifr->ifr_data;
	/* Fill message data */
	switch (cmd) {
	case IOCTL_ADD_FRAME_FILTER:
		req->cmd = CMD_CAN_ADD_FRAME_FILTER;
		break;
	case IOCTL_REMOVE_FRAME_FILTER:
		req->cmd = CMD_CAN_REMOVE_FRAME_FILTER;
		break;
	case IOCTL_UPDATE_FRAME_FILTER:
		req->cmd = CMD_CAN_UPDATE_FRAME_FILTER;
		break;
	}
	req->len = sizeof(struct sam4e_req) +
			sizeof(struct sam4e_frame_filter);
	req->seq = atomic_inc_return(&dev->msg_seq);
	frame_filter = (struct sam4e_frame_filter *)&req->data;
	frame_filter->can = add_request->can;
	frame_filter->mid = add_request->mid;
	frame_filter->mask = add_request->mask;
	frame_filter->type = add_request->type;

	LOGDI("sam4e_send_frame_filter cmd:%d %d %x %x %d", req->cmd,
			frame_filter->can,
			frame_filter->mid,
			frame_filter->mask,
			frame_filter->type);

	LOGTEST("sam4e_send_frame_filter cmd:%d %d %x %x %d",
			req->cmd, frame_filter->can, frame_filter->mid,
			frame_filter->mask, frame_filter->type);

	usb_fill_bulk_urb(urb, dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP), req,
			size, sam4e_usb_write_bulk_callback, netdev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->tx_submitted);

	result = usb_submit_urb(urb, GFP_KERNEL);
	if (unlikely(result)) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, req, urb->transfer_dma);
	}
	usb_free_urb(urb);
	return result;

nomem:
	return -ENOMEM;
}

static int sam4e_netdev_do_ioctl(struct net_device *netdev,
		struct ifreq *ifr, int cmd)
{
	LOGNI("sam4e_netdev_do_ioctl %d", cmd);
	if (cmd == IOCTL_RELEASE_CAN_BUFFER) {
		LOGNI("IOCTL_RELEASE_CAN_BUFFER");
		sam4e_send_release_can_buffer_cmd(netdev);
	} else if (cmd == IOCTL_ENABLE_BUFFERING) {
		LOGNI("IOCTL_ENABLE_BUFFERING");
		sam4e_enable_buffering(netdev, ifr);
	} else if (cmd == IOCTL_ADD_FRAME_FILTER ||
			cmd == IOCTL_REMOVE_FRAME_FILTER ||
			cmd == IOCTL_UPDATE_FRAME_FILTER) {
		LOGNI("IOCTL_ADD/REMOVE/UPDATE_FRAME_FILTER");
		sam4e_frame_filter(netdev, ifr, cmd);
	}

	return 0;
}

static const struct net_device_ops sam4e_usb_netdev_ops = {
		.ndo_open = sam4e_netdev_open,
		.ndo_stop = sam4e_netdev_close,
		.ndo_start_xmit = sam4e_netdev_start_xmit,
		.ndo_do_ioctl = sam4e_netdev_do_ioctl,
};

static int sam4e_read_fw_version(struct sam4e_usb *dev)
{
	struct sam4e_req *req;
	int actual_length;
	int result;
	struct net_device *netdev;

	netdev = dev->netdev1;
	LOGDI("Querying firmware version");
	req = kzalloc(sizeof(struct sam4e_req), GFP_KERNEL);
	if (req == NULL)
		return -ENOMEM;

	req->cmd = CMD_SYS_GET_FW_VERSION;
	req->len = sizeof(struct sam4e_req);
	req->seq = atomic_inc_return(&dev->msg_seq);

	result = usb_bulk_msg(dev->udev,
			usb_sndbulkpipe(dev->udev, BULK_OUT_EP),
			req,
			sizeof(struct sam4e_req),
			&actual_length, 1000);

	LOGDI("sent %x result %d, actual_length %d",
			req->cmd, result, actual_length);
	kfree(req);
	return 0;
}
/*
 * set up mail box for can frames.
 */
static void setup_mailbox(u8 mailbox, u8 canid,
		u32 filter, u32 mid,
		struct usb_interface *intf,
		struct usb_device *udev,
		struct sam4e_usb *dev)
{
	struct sam4e_req *req;
	struct sam4e_resp *resp;
	int actual_length;
	int result;
	req = kzalloc(sizeof(struct sam4e_req) +
			sizeof(struct sam4e_can_full_ts_read), GFP_KERNEL);
	if (req != 0) {
		struct sam4e_can_full_ts_read *cfr =
				(struct sam4e_can_full_ts_read *)&req->data;
		req->cmd = CMD_CAN_TS_FULL_READ;
		req->len = sizeof(struct sam4e_req) +
				sizeof(struct sam4e_can_full_ts_read);
		req->seq = atomic_inc_return(&dev->msg_seq);
		cfr->mailbox = mailbox;
		cfr->can = canid;
		/* Timestamping on */
		cfr->ts_on = 1;
		cfr->mid = mid;
		cfr->filter = filter;
		result = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_EP),
					req,
					sizeof(struct sam4e_req) +
					sizeof(struct sam4e_can_full_ts_read),
					&actual_length, 1000);
		LOGDI("sent %x result %d, actual_length %d",
				req->cmd, result, actual_length);
		kfree(req);
	}
	resp = kzalloc(sizeof(struct sam4e_resp), GFP_KERNEL);
	result =  usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_EP),
				resp,
				sizeof(struct sam4e_resp),
				&actual_length, 1000);
	LOGDI("rcv? result %d, actual_length %d", result, actual_length);
	if (result == 0 && DEBUG_SAM4E) {
		dev_info(&intf->dev, "data %x %d %d %d\n", resp->cmd,
			resp->len, resp->seq, resp->err);
	}
	kfree(resp);
}
/*
 * probe function for usb devices
 */
static int sam4e_usb_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_host_interface *usb_hi;
	struct usb_interface_descriptor *usb_if_desc;
	struct usb_device *udev;
	int err = 0;
	u8 if_num;
	struct net_device *netdev;
	struct sam4e_usb *dev;
	struct sam4e_usb_handle *sam4e_usb_hnd;
	int cnt = 0;

	usb_hi = intf->cur_altsetting;
	usb_if_desc = &usb_hi->desc;
	if_num = usb_if_desc->bInterfaceNumber;
	if (DEBUG_SAM4E) {
		dev_info(&intf->dev, "probe %p %p if_num %d id if_num %d\n",
			intf, id, if_num, id->bInterfaceNumber);
	}

	if (if_num != 1)
		return -ENODEV;

	dev = kzalloc(sizeof(struct sam4e_usb), GFP_KERNEL);
	if (!dev) {
		dev_err(&intf->dev, "Couldn't alloc sam4e_usb\n");
		err = -ENOMEM;
		goto cleanup_candev;
	}
	atomic_set(&dev->netif_queue_stop, 0);
	while (cnt < MAX_CAN_INTF) {
		netdev = alloc_candev(sizeof(struct sam4e_usb_handle),
				MAX_TX_URBS);
		if (!netdev) {
			dev_err(&intf->dev, "Couldn't alloc candev\n");
			err = -ENOMEM;
			goto cleanup_candev;
		}

		sam4e_usb_hnd = netdev_priv(netdev);
		sam4e_usb_hnd->sam4e_dev = dev;
		sam4e_usb_hnd->owner_netdev_index = cnt;
		dev->udev = interface_to_usbdev(intf);
		if (cnt == 0)
			dev->netdev1 = netdev;
		else
			dev->netdev2 = netdev;

		dev->assembly_buffer = kzalloc(RX_ASSEMBLY_BUFFER_SIZE,
				GFP_KERNEL);

		netdev->netdev_ops = &sam4e_usb_netdev_ops;

		init_usb_anchor(&dev->rx_submitted);
		init_usb_anchor(&dev->tx_submitted);
		atomic_set(&dev->active_tx_urbs, 0);
		atomic_set(&dev->msg_seq, 0);

		usb_set_intfdata(intf, dev);
		SET_NETDEV_DEV(netdev, &intf->dev);

		err = register_candev(netdev);
		if (err) {
			pr_err("fail reg.CAN device: %d", err);
			goto cleanup_candev;
		}

		udev = interface_to_usbdev(intf);

		if (cnt == 0) {
			/*use mail box 7 and 6 for e/s frames on can0*/
			setup_mailbox(7, 0, CAN_EFF_FLAG, CAN_EFF_FLAG,
					intf, udev, dev);
			setup_mailbox(6, 0, CAN_EFF_FLAG, 0,
					intf, udev, dev);
		} else {
			/*use mail box 5 and 4 for e/s frames on can1*/
			setup_mailbox(5, 1, CAN_EFF_FLAG, CAN_EFF_FLAG,
					intf, udev, dev);
			setup_mailbox(4, 1, CAN_EFF_FLAG, 0,
					intf, udev, dev);
		}
		cnt++;
	}
	sam4e_read_fw_version(dev);
	err = sam4e_init_urbs(dev);
	if (err) {
		pr_err("couldn't start device: %d\n", err);
		goto unregister_candev;
	}
	return 0; /*ok. it's ours */
unregister_candev:
	if (dev) {
		if (dev->netdev1)
			unregister_netdev(dev->netdev1);
		if (dev->netdev2)
			unregister_netdev(dev->netdev2);
	}
cleanup_candev:
	if (dev) {
		kfree(dev->assembly_buffer);
		if (dev->netdev1)
			free_candev(dev->netdev1);
		if (dev->netdev2)
			free_candev(dev->netdev2);
		kfree(dev);
	}
	return err;
}

/*
 * called by the usb core when the device is removed from the system
 */
static void sam4e_usb_disconnect(struct usb_interface *intf)
{
	struct sam4e_usb *dev;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	if (dev) {
		LOGDI("Disconnect sam4e\n");
		if (dev->netdev1) {
			unregister_netdev(dev->netdev1);
			free_candev(dev->netdev1);
		}
		if (dev->netdev2) {
			unregister_netdev(dev->netdev2);
			free_candev(dev->netdev2);
		}
		kfree(dev->assembly_buffer);
		unlink_all_urbs(dev);
		kfree(dev);
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
