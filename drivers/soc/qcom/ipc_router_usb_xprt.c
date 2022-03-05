/* Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
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

/*
 * IPC ROUTER USB XPRT module.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/ipc_router_xprt.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/debugfs.h>

#include <linux/ipc_router_usb_xprt.h>

static int msm_ipc_router_usb_xprt_debug_mask;
module_param_named(debug_mask, msm_ipc_router_usb_xprt_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DBG(x...) do { \
if (msm_ipc_router_usb_xprt_debug_mask) \
	pr_info(x); \
} while (0)

#define IPC_XPRT_NAME_LEN 32

/**
 * msm_ipc_router_usb_xprt - IPC Router's USB XPRT structure
 * @list: IPC router's USB XPRTs list.
 * @ch_name: Name of the USB endpoint exported.
 * @xprt_name: Name of the XPRT to be registered with IPC Router.
 * @driver: Platform drivers register by this XPRT.
 * @xprt: IPC Router XPRT structure to contain USB XPRT specific info.
 * @pdev: Platform device registered by IPC Bridge function driver.
 * @usb_xprt_wq: Workqueue to queue read & other XPRT related works.
 * @read_work: Read Work to perform read operation from USB.
 * @in_pkt: Pointer to any partially read packet.
 * @xprt_lock: Lock to protect access to the xprt_avail.
 * @xprt_avail: flag used to check XPRT is available.
 * @sft_close_complete: Variable to indicate completion of IPC Router reset.
 * @xprt_version: IPC Router header version supported by this XPRT.
 * @xprt_option: XPRT specific options to be handled by IPC Router.
 */
struct msm_ipc_router_usb_xprt {
	struct list_head list;
	char ch_name[IPC_XPRT_NAME_LEN];
	char xprt_name[IPC_XPRT_NAME_LEN];
	struct platform_driver driver;
	struct msm_ipc_router_xprt xprt;
	struct usb_ipc_xprt_ops *ops;
	struct platform_device *pdev;
	struct workqueue_struct *usb_xprt_wq;
	struct delayed_work read_work;
	struct rr_packet *in_pkt;
	struct mutex xprt_lock;
	int xprt_avail;
	struct completion sft_close_complete;
	unsigned xprt_version;
	unsigned xprt_option;
	unsigned int read_from_usb;
	unsigned int send_to_usb;
	unsigned int send_to_ipc_core;
	unsigned int recv_from_ipc_core;
};

static struct msm_ipc_router_usb_xprt *usb_xprt_ctx;

/* Debug FS Root node */
static struct dentry *debugfs_root;

static void usb_xprt_read_data(struct work_struct *work);

/**
 * msm_ipc_router_usb_xprt_config - Config. Info. of each USB IPC XPRT
 * @ch_name: Name of the USB endpoint.
 * @xprt_name: Name of the XPRT to be registered with IPC Router.
 * @usb_pdev_id: ID to differentiate among multiple endpoints.
 * @link_id: Network Cluster ID to which this XPRT belongs to.
 * @xprt_version: IPC Router header version supported by this XPRT.
 */
struct msm_ipc_router_usb_xprt_config {
	char ch_name[IPC_XPRT_NAME_LEN];
	char xprt_name[IPC_XPRT_NAME_LEN];
	int usb_pdev_id;
	uint32_t link_id;
	unsigned xprt_version;
};

#define MODULE_NAME "ipc_router_usb_xprt"

/**
 * ipc_router_usb_set_xprt_version() - Set IPC Router header version
 *                                          in the transport
 * @xprt: Reference to the transport structure.
 * @version: The version to be set in transport.
 */
static void ipc_router_usb_set_xprt_version(
	struct msm_ipc_router_xprt *xprt, unsigned version)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp;

	if (!xprt)
		return;
	usb_xprtp = container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);

	if (!usb_xprtp)
		return;
	usb_xprtp->xprt_version = version;
}

/**
 * msm_ipc_router_usb_get_xprt_version() - Get IPC Router header version
 *                                          supported by the XPRT
 * @xprt: XPRT for which the version information is required.
 *
 * @return: IPC Router header version supported by the XPRT.
 */
static int msm_ipc_router_usb_get_xprt_version(
	struct msm_ipc_router_xprt *xprt)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp;
	if (!xprt)
		return -EINVAL;
	usb_xprtp = container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);

	if (!usb_xprtp)
		return -EINVAL;

	return (int)usb_xprtp->xprt_version;
}

/**
 * msm_ipc_router_usb_get_xprt_option() - Get XPRT options
 * @xprt: XPRT for which the option information is required.
 *
 * @return: Options supported by the XPRT.
 */
static int msm_ipc_router_usb_get_xprt_option(
	struct msm_ipc_router_xprt *xprt)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp;
	if (!xprt)
		return -EINVAL;
	usb_xprtp = container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);

	if (!usb_xprtp)
		return -EINVAL;

	return (int)usb_xprtp->xprt_option;
}

/**
 * msm_ipc_router_usb_remote_write_avail() - Get available write space
 * @xprt: XPRT for which the available write space info. is required.
 *
 * @return: Write space in bytes on success, 0 on SSR.
 */
static int msm_ipc_router_usb_remote_write_avail(
	struct msm_ipc_router_xprt *xprt)
{
	int write_avail = 0;
	struct msm_ipc_router_usb_xprt *usb_xprtp =
		container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);

	if (!usb_xprtp)
		return -EINVAL;

	mutex_lock(&usb_xprtp->xprt_lock);
	if (usb_xprtp->ops && usb_xprtp->xprt_avail)
		write_avail = usb_xprtp->ops->max_write_size;
	mutex_unlock(&usb_xprtp->xprt_lock);
	return write_avail;
}

/**
 * msm_ipc_router_usb_remote_write() - Write to USB XPRT layer
 * @data: Data to be written to the XPRT.
 * @len: Length of the data to be written.
 * @xprt: XPRT to which the data has to be written.
 *
 * @return: Data Length on success, standard Linux error codes on failure.
 */
static int msm_ipc_router_usb_remote_write(void *data,
		uint32_t len, struct msm_ipc_router_xprt *xprt)
{
	struct rr_packet *pkt = (struct rr_packet *)data;
	struct sk_buff *skb;
	struct msm_ipc_router_usb_xprt *usb_xprtp;
	int ret;
	uint32_t bytes_written = 0;
	uint32_t bytes_to_write;
	unsigned char *tx_data;

	if (!pkt || pkt->length != len || !xprt) {
		IPC_RTR_ERR("%s: Invalid input parameters\n", __func__);
		return -EINVAL;
	}

	usb_xprtp = container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);
	if (!usb_xprtp)
		return -EINVAL;

	mutex_lock(&usb_xprtp->xprt_lock);
	if (!usb_xprtp->xprt_avail) {
		IPC_RTR_ERR("%s: Trying to write on a reset link\n", __func__);
		mutex_unlock(&usb_xprtp->xprt_lock);
		return -ENETRESET;
	}

	if (!usb_xprtp->pdev) {
		IPC_RTR_ERR("%s: Trying to write on a closed link\n", __func__);
		mutex_unlock(&usb_xprtp->xprt_lock);
		return -ENODEV;
	}

	if (!usb_xprtp->ops || !usb_xprtp->ops->write) {
		IPC_RTR_ERR("%s on a uninitialized link\n", __func__);
		mutex_unlock(&usb_xprtp->xprt_lock);
		return -EFAULT;
	}
	usb_xprtp->recv_from_ipc_core++;

	skb = skb_peek(pkt->pkt_fragment_q);
	if (!skb) {
		IPC_RTR_ERR("%s SKB is NULL\n", __func__);
		mutex_unlock(&usb_xprtp->xprt_lock);
		return -EINVAL;
	}
	DBG("%s: About to write %d bytes\n", __func__, len);

	while (bytes_written < len) {
		bytes_to_write = min_t(uint32_t, (skb->len - bytes_written),
				       usb_xprtp->ops->max_write_size);
		tx_data = skb->data + bytes_written;
		ret = usb_xprtp->ops->write(tx_data, bytes_to_write);
		if (ret < 0) {
			IPC_RTR_ERR("%s: Error writing data %d\n",
				    __func__, ret);
			break;
		}
		usb_xprtp->send_to_usb++;
		if (ret != bytes_to_write)
			IPC_RTR_ERR("%s: Partial write %d < %d, retrying...\n",
				    __func__, ret, bytes_to_write);
		bytes_written += bytes_to_write;
	}
	if (bytes_written == len) {
		ret = bytes_written;
	} else if (ret > 0 && bytes_written != len) {
		IPC_RTR_ERR("%s: Fault writing data %d != %d\n",
			    __func__, bytes_written, len);
		ret = -EFAULT;
	}
	mutex_unlock(&usb_xprtp->xprt_lock);
	DBG("%s: Finished writing %d bytes\n", __func__, len);
	return ret;
}

/**
 * msm_ipc_router_usb_remote_close() - Close the XPRT
 * @xprt: XPRT which needs to be closed.
 *
 * @return: 0 on success, standard Linux error codes on failure.
 */
static int msm_ipc_router_usb_remote_close(
	struct msm_ipc_router_xprt *xprt)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp;

	DBG("%s \n",__func__);
	if (!xprt)
		return -EINVAL;
	usb_xprtp = container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);
	if (!usb_xprtp)
		return -EINVAL;

	if (!usb_xprtp->xprt_avail) {
		IPC_RTR_ERR("%s: xprt is not available\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&usb_xprtp->xprt_lock);
	usb_xprtp->xprt_avail = 0;
	mutex_unlock(&usb_xprtp->xprt_lock);
	flush_workqueue(usb_xprtp->usb_xprt_wq);
	destroy_workqueue(usb_xprtp->usb_xprt_wq);
	usb_xprtp->usb_xprt_wq = NULL;
	if (usb_xprtp->ops && usb_xprtp->ops->close)
		usb_xprtp->ops->close();
	return 0;
}

/**
 * usb_xprt_read_data() - Read work to read from the XPRT
 * @work: Read work to be executed.
 *
 * This function is a read work item queued on a XPRT specific workqueue.
 * The work parameter contains information regarding the XPRT on which this
 * read work has to be performed. The work item keeps reading from the HSIC
 * endpoint, until the endpoint returns an error.
 */
static void usb_xprt_read_data(struct work_struct *work)
{
	int bytes_to_read;
	int bytes_read;
	int skb_size;
	struct sk_buff *skb = NULL;
	struct delayed_work *rwork = to_delayed_work(work);
	struct msm_ipc_router_usb_xprt *usb_xprtp =
		container_of(rwork, struct msm_ipc_router_usb_xprt, read_work);

	if (unlikely(!usb_xprtp)) {
		IPC_RTR_ERR("%s: USB Xprt NULL pointer\n", __func__);
		return;
	}

	while (1) {
		while (!usb_xprtp->in_pkt) {
			usb_xprtp->in_pkt = create_pkt(NULL);
			if (usb_xprtp->in_pkt)
				break;
			IPC_RTR_ERR("%s: packet allocation failure\n",
								__func__);
			/* Retry after 100ms */
			msleep(100);
		}
		DBG("%s: Allocated rr_packet\n", __func__);

		bytes_to_read = 0;
		skb_size = usb_xprtp->ops->max_read_size;
		do {
			do {
				skb = alloc_skb(skb_size, GFP_KERNEL);
				if (skb)
					break;
				IPC_RTR_ERR("%s: Couldn't alloc SKB\n",
					    __func__);
				/* Retry after 100ms */
				msleep(100);
			} while (!skb);
			mutex_lock(&usb_xprtp->xprt_lock);
			if (!usb_xprtp->xprt_avail) {
				mutex_unlock(&usb_xprtp->xprt_lock);
				IPC_RTR_ERR("%s: Trying to read on "
						"a reseted link\n", __func__);
				kfree_skb(skb);
				goto out_read_data;
			}
			mutex_unlock(&usb_xprtp->xprt_lock);
			bytes_read = usb_xprtp->ops->read(skb->data,
						usb_xprtp->ops->max_read_size);
			if (bytes_read < 0) {
				if (bytes_read == -ESHUTDOWN)
					IPC_RTR_ERR("%s:USB Cable disconnected",
								__func__);
				else
					IPC_RTR_ERR("%s: Error %d @"
							" read operation\n",
							__func__, bytes_read);
				kfree_skb(skb);
				goto out_read_data;
			}
			usb_xprtp->read_from_usb++;
			if (!bytes_to_read) {
				bytes_to_read = ipc_router_peek_pkt_size(
						skb->data);
				if (bytes_to_read < 0) {
					IPC_RTR_ERR("%s: Invalid size %d\n",
						__func__, bytes_to_read);
					kfree_skb(skb);
					goto out_read_data;
				}
			}
			bytes_to_read -= bytes_read;
			skb_put(skb, bytes_read);
			skb_queue_tail(usb_xprtp->in_pkt->pkt_fragment_q, skb);
			usb_xprtp->in_pkt->length += bytes_read;
			skb_size = min_t(uint32_t,
					usb_xprtp->ops->max_read_size,
					(uint32_t)bytes_to_read);
		} while (bytes_to_read > 0);

		DBG("%s: Packet read of size %d\n",__func__,
					usb_xprtp->in_pkt->length);
		msm_ipc_router_xprt_notify(&usb_xprtp->xprt,
			IPC_ROUTER_XPRT_EVENT_DATA, (void *)usb_xprtp->in_pkt);
		usb_xprtp->send_to_ipc_core++;
		release_pkt(usb_xprtp->in_pkt);
		usb_xprtp->in_pkt = NULL;
	}
out_read_data:
	release_pkt(usb_xprtp->in_pkt);
	usb_xprtp->in_pkt = NULL;
}

/**
 * usb_xprt_sft_close_done() - Completion of XPRT reset
 * @xprt: XPRT on which the reset operation is complete.
 *
 * This function is used by IPC Router to signal this USB IPC XPRT Abstraction
 * Layer(XAL) that the reset of XPRT is completely handled by IPC Router.
 */
static void usb_xprt_sft_close_done(struct msm_ipc_router_xprt *xprt)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp =
		container_of(xprt, struct msm_ipc_router_usb_xprt, xprt);

	if (unlikely(!usb_xprtp)) {
		IPC_RTR_ERR("%s: USB Xprt NULL pointer ",__func__);
		return;
	}

	complete_all(&usb_xprtp->sft_close_complete);
}

/**
 * msm_ipc_router_usb_ipc_xprt_unregister() - Unregister USB IPC XPRT
 *
 * @xprt_ops: USB related operations to send/receive actual messages.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function should be called when USB IPC endpoint deregisters and is
 * not needed, i.e. Cable disconnect or composition change, etc.
 */
int msm_ipc_router_usb_ipc_xprt_unregister(struct usb_ipc_xprt_ops *xprt_ops)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp = usb_xprt_ctx;

	DBG("%s \n",__func__);
	if (unlikely(!usb_xprtp)) {
		IPC_RTR_ERR("%s: USB Xprt NULL pointer ",__func__);
		return -EINVAL;
	}

	if (xprt_ops != usb_xprtp->ops) {
		IPC_RTR_ERR("%s: USB Xprt Unregister called in incorrect order"
				"or from incorrect module usb_xprtp->ops %p"
				"xprt_ops %p",__func__,
					usb_xprtp->ops, xprt_ops);
		return -EINVAL;
	}

	if (!usb_xprtp->xprt_avail) {
		IPC_RTR_ERR("%s: USB Xprt not available\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&usb_xprtp->xprt_lock);
	usb_xprtp->xprt_avail = 0;
	mutex_unlock(&usb_xprtp->xprt_lock);
	flush_workqueue(usb_xprtp->usb_xprt_wq);
	destroy_workqueue(usb_xprtp->usb_xprt_wq);
	usb_xprtp->usb_xprt_wq = NULL;
	init_completion(&usb_xprtp->sft_close_complete);
	msm_ipc_router_xprt_notify(&usb_xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_CLOSE, NULL);
	DBG("%s: Notified IPC Router of %s CLOSE\n",
	  __func__, usb_xprtp->xprt.name);
	wait_for_completion(&usb_xprtp->sft_close_complete);
	DBG("%s: IPC Router Core Close Completed \n",__func__);
	if (usb_xprtp->ops && usb_xprtp->ops->close) {
		usb_xprtp->ops->close();
	}
	return 0;
}

/**
 * msm_ipc_router_usb_ipc_xprt_register() - Register USB IPC XPRT
 *
 * @xprt_ops: USB related operations to send/receive actual messages.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * Should be called from USB layer when corresponding IPC Router
 * endpoint is detected and registered.
 */
int msm_ipc_router_usb_ipc_xprt_register(struct usb_ipc_xprt_ops *xprt_ops)
{
	int rc;
	struct msm_ipc_router_usb_xprt *usb_xprtp = usb_xprt_ctx;

	DBG("%s \n",__func__);

	if (unlikely(!usb_xprtp)) {
		IPC_RTR_ERR("%s: USB Xprt NULL pointer ",__func__);
		return -EINVAL;
	}

	if (!xprt_ops || !xprt_ops->open || !xprt_ops->read ||
	    !xprt_ops->write || !xprt_ops->close) {
		IPC_RTR_ERR("%s: xprt_ops or xprt_ops->operations is NULL\n",
								__func__);
		return -EINVAL;
	}

	usb_xprtp->ops = xprt_ops;

	usb_xprtp->usb_xprt_wq =
		create_singlethread_workqueue(MODULE_NAME);
	if (!usb_xprtp->usb_xprt_wq) {
		IPC_RTR_ERR("%s: WQ creation failed for %s\n",
		__func__, MODULE_NAME);
		return -EFAULT;
	}

	mutex_lock(&usb_xprtp->xprt_lock);
	usb_xprtp->xprt_avail = 1;
	mutex_unlock(&usb_xprtp->xprt_lock);

	rc = xprt_ops->open();
	if (rc < 0) {
		IPC_RTR_ERR("%s: Channel open failed for %s xprt\n",
			__func__, MODULE_NAME);
		destroy_workqueue(usb_xprtp->usb_xprt_wq);
		usb_xprtp->usb_xprt_wq = NULL;
		mutex_lock(&usb_xprtp->xprt_lock);
		usb_xprtp->xprt_avail = 0;
		mutex_unlock(&usb_xprtp->xprt_lock);
		return rc;
	}

	msm_ipc_router_xprt_notify(&usb_xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_OPEN, NULL);
	IPC_RTR_ERR("%s:%d \n", __func__,__LINE__);
	DBG("%s: Notified IPC Router of %s OPEN\n",
	  __func__, usb_xprtp->xprt.name);
	queue_delayed_work(usb_xprtp->usb_xprt_wq,
			   &usb_xprtp->read_work, 0);
	return 0;
}

/**
 * msm_ipc_router_usb_config_init() - init USB IPC xprt configs
 *
 * @usb_xprt_config: pointer to USB IPC xprt configurations.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function is called to initialize the USB IPC XPRT pointer with
 * the USB IPC XPRT configurations either from device tree or static arrays.
 */
static int msm_ipc_router_usb_config_init(
		struct msm_ipc_router_usb_xprt_config *usb_xprt_config,
		struct platform_device *pdev)
{
	struct msm_ipc_router_usb_xprt *usb_xprtp;

	DBG("%s: \n", __func__);
	if (unlikely(usb_xprt_ctx)) {
		IPC_RTR_ERR("%s: Already Inited \n", __func__);
		return -1;
	}

	usb_xprtp = kzalloc(sizeof(struct msm_ipc_router_usb_xprt),
							GFP_KERNEL);
	if (IS_ERR_OR_NULL(usb_xprtp)) {
		IPC_RTR_ERR("%s: kzalloc() failed for usb_xprtp id:%s\n",
				__func__, usb_xprt_config->ch_name);
		return -ENOMEM;
	}

	usb_xprt_ctx = usb_xprtp;

	usb_xprtp->xprt.link_id = usb_xprt_config->link_id;
	usb_xprtp->xprt_version = usb_xprt_config->xprt_version;

	strlcpy(usb_xprtp->ch_name, usb_xprt_config->ch_name,
					IPC_XPRT_NAME_LEN);

	strlcpy(usb_xprtp->xprt_name, usb_xprt_config->xprt_name,
						IPC_XPRT_NAME_LEN);
	usb_xprtp->xprt.name = usb_xprtp->xprt_name;

	usb_xprtp->xprt.set_version =
		ipc_router_usb_set_xprt_version;
	usb_xprtp->xprt.get_version =
		msm_ipc_router_usb_get_xprt_version;
	usb_xprtp->xprt.get_option =
		 msm_ipc_router_usb_get_xprt_option;
	usb_xprtp->xprt.read_avail = NULL;
	usb_xprtp->xprt.read = NULL;
	usb_xprtp->xprt.write_avail =
		msm_ipc_router_usb_remote_write_avail;
	usb_xprtp->xprt.write = msm_ipc_router_usb_remote_write;
	usb_xprtp->xprt.close = msm_ipc_router_usb_remote_close;
	usb_xprtp->xprt.sft_close_done = usb_xprt_sft_close_done;
	/* We don't use the priv data member to store our context since
	   its been used by ipc_router_core internally. */
	usb_xprtp->xprt.priv = NULL;

	usb_xprtp->in_pkt = NULL;
	INIT_DELAYED_WORK(&usb_xprtp->read_work, usb_xprt_read_data);
	mutex_init(&usb_xprtp->xprt_lock);
	usb_xprtp->xprt_avail = 0;
	usb_xprtp->xprt_option = 0;

	usb_xprtp->pdev = pdev;

	return 0;

}

/**
 * parse_devicetree() - parse device tree binding
 *
 * @node: pointer to device tree node
 * @usb_xprt_config: pointer to USB IPC XPRT configurations
 *
 * @return: 0 on success, -ENODEV on failure.
 */
static int parse_devicetree(struct device_node *node,
		struct msm_ipc_router_usb_xprt_config *usb_xprt_config)
{
	int ret;
	int link_id;
	int version;
	char *key;
	const char *ch_name;
	const char *remote_ss;

	key = "qcom,ch-name";
	ch_name = of_get_property(node, key, NULL);
	if (!ch_name)
		goto error;
	strlcpy(usb_xprt_config->ch_name, ch_name, IPC_XPRT_NAME_LEN);

	key = "qcom,xprt-remote";
	remote_ss = of_get_property(node, key, NULL);
	if (!remote_ss)
		goto error;

	IPC_RTR_ERR("%s: %s=%s \n", __func__,key,remote_ss);

	key = "qcom,xprt-linkid";
	ret = of_property_read_u32(node, key, &link_id);
	if (ret)
		goto error;
	usb_xprt_config->link_id = link_id;
	IPC_RTR_ERR("%s: %s=%d \n", __func__,key,link_id);

	key = "qcom,xprt-version";
	ret = of_property_read_u32(node, key, &version);
	if (ret)
		goto error;
	usb_xprt_config->xprt_version = version;
	IPC_RTR_ERR("%s: %s=%d \n", __func__,key,version);

	scnprintf(usb_xprt_config->xprt_name, IPC_XPRT_NAME_LEN, "%s_%s",
			remote_ss, usb_xprt_config->ch_name);

	return 0;

error:
	IPC_RTR_ERR("%s: missing key: %s\n", __func__, key);
	return -ENODEV;
}

/**
 * msm_ipc_router_usb_xprt_probe() - Probe an USB IPC xprt
 * @pdev: Platform device corresponding to USB IPC xprt.
 *
 * @return: 0 on success, standard Linux error codes on error.
 *
 * This function is called when the underlying device tree driver registers
 * a platform device, mapped to an USB IPC transport.
 */
static int msm_ipc_router_usb_xprt_probe(
				struct platform_device *pdev)
{
	int ret;
	struct msm_ipc_router_usb_xprt_config usb_xprt_config;

	IPC_RTR_ERR("%s: \n", __func__);
	if (pdev && pdev->dev.of_node) {
		ret = parse_devicetree(pdev->dev.of_node,
						&usb_xprt_config);
		if (ret) {
			IPC_RTR_ERR("%s: Failed to parse device tree\n",
								__func__);
			return ret;
		}

		ret = msm_ipc_router_usb_config_init(
						&usb_xprt_config, pdev);
		if (ret) {
			IPC_RTR_ERR(" %s init failed\n", __func__);
			return ret;
		}
	}
	return ret;
}

static ssize_t usb_ipc_xprt_debugfs_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	struct msm_ipc_router_usb_xprt *usb_xprtp =
			(struct msm_ipc_router_usb_xprt *)file->private_data;
	ssize_t ret_cnt;

	if (unlikely(!usb_xprtp)) {
		IPC_RTR_ERR("%s: USB Xprt NULL pointer ",__func__);
		return -EINVAL;
	}

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "%25s\n",
			"USB IPC Router XPRT Status");

	len += scnprintf(buf + len, buf_len - len, "%55s\n",
		"==================================================");
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10s\n", "XPRT - Name : ", usb_xprtp->xprt_name);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10s\n", "XPRT - CH Name : ", usb_xprtp->ch_name);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "XPRT - Link ID : ", usb_xprtp->xprt.link_id);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "XPRT - Version : ", usb_xprtp->xprt_version);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "Read from USB: ", usb_xprtp->read_from_usb);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "Writen to IPC Core: ",
					usb_xprtp->send_to_ipc_core);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "Submitted to USB: ", usb_xprtp->send_to_usb);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10u\n", "Read from IPC Core: ",
					usb_xprtp->recv_from_ipc_core);
	len += scnprintf(buf + len, buf_len - len,
		"%25s %10d\n", "USB XPRT Avail : ", usb_xprtp->xprt_avail);
	len += scnprintf(buf + len, buf_len - len, "%55s\n",
		"==================================================");

	if (len > buf_len)
		len = buf_len;

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret_cnt;
}

static const struct file_operations fops_usb_ipc_xprt = {
	.read = usb_ipc_xprt_debugfs_read,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int msm_ipc_usb_xprt_debugfs_init(void)
{
	debugfs_root = debugfs_create_dir("usb_ipc_xprt", 0);
	if (IS_ERR(debugfs_root)) {
		IPC_RTR_ERR("%s: Cannot create debugfs %p ", __func__,
				debugfs_root);
		return -ENOMEM;
	}

	debugfs_create_file("stats", S_IRUSR | S_IRGRP | S_IROTH, debugfs_root,
				usb_xprt_ctx, &fops_usb_ipc_xprt);
	DBG("%s: debugfs created %p \n",__func__, debugfs_root);

	return 0;
}

static void msm_ipc_usb_xprt_debugfs_exit(void)
{
	debugfs_remove_recursive(debugfs_root);
	DBG("%s: debugfs destroyed \n",__func__);
}

static struct of_device_id msm_ipc_router_usb_xprt_match_table[] = {
	{ .compatible = "qcom,ipc-router-usb-xprt" },
	{},
};

static struct platform_driver msm_ipc_router_usb_xprt_driver = {
	.probe = msm_ipc_router_usb_xprt_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_ipc_router_usb_xprt_match_table,
	},
};

static int __init msm_ipc_router_usb_xprt_init(void)
{
	int rc;

	IPC_RTR_ERR("%s: \n", __func__);
	rc = platform_driver_register(&msm_ipc_router_usb_xprt_driver);
	if (rc) {
		IPC_RTR_ERR(
		"%s: msm_ipc_router_usb_xprt_driver register failed %d\n",
								__func__, rc);
		return rc;
	}

	rc = msm_ipc_usb_xprt_debugfs_init();
	if (rc)
		IPC_RTR_ERR("%s: debugfs not inited \n", __func__);
	return 0;
}

static void __exit msm_ipc_router_usb_xprt_exit(void)
{
	msm_ipc_usb_xprt_debugfs_exit();
}

module_init(msm_ipc_router_usb_xprt_init);
module_exit(msm_ipc_router_usb_xprt_exit);
MODULE_DESCRIPTION("IPC Router USB XPRT");
MODULE_LICENSE("GPL v2");
