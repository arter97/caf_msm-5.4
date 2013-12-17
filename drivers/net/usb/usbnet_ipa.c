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

#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <linux/usb/hbm.h>
#include <mach/odu_ipa.h>
#include <mach/usb_bam.h>
#include <mach/ipa.h>

#define USBNET_IPA_DRV_NAME "usbnet_ipa"

#define USBNET_IPA_DBG(fmt, args...) \
	pr_debug(USBNET_IPA_DRV_NAME " %s:%d " fmt, \
		 __func__, __LINE__, ## args)
#define USBNET_IPA_DBG_FUNC_ENTRY() \
	pr_debug(USBNET_IPA_DRV_NAME " %s:%d begin\n", __func__, __LINE__)
#define USBNET_IPA_DBG_FUNC_EXIT() \
	pr_debug(USBNET_IPA_DRV_NAME " %s:%d end\n", __func__, __LINE__)
#define USBNET_IPA_ERR(fmt, args...) \
	pr_err(USBNET_IPA_DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)


#define USBNET_CORE "hsic"

struct usbnet_ipa_bam_info {
	u32 src_pipe;
	u32 dst_pipe;
	struct urb *tx_urb;
	struct urb *rx_urb;
	struct odu_ipa_hw_hdr_info hdr_cfg;
	struct usb_bam_connect_ipa_params ipa_params;
	struct odu_ipa_params		  params;
	struct net_device *net;
};

static struct usbnet_ipa_bam_info ctx;

/* Specific vendor header information to configure the IPA to remove/add */
#ifdef CONFIG_USB_NET_SMSC75XX
void usbnet_config_header(void)
{
	u8 tx_smsc_header[] = {0x00, 0x00, 0x40, 0x00,
				0x00, 0x00, 0x00, 0x00};

	/* RX contains 2 additional bytes */
	u8 rx_smsc_header[] = {0x00, 0xC0, 0x00, 0x1C,
				0x00, 0x00, 0xFF, 0xFF,
				0x00, 0x00};

	memset(&ctx.hdr_cfg.rx, 0, sizeof(ctx.hdr_cfg.rx));
	memset(&ctx.hdr_cfg.tx, 0, sizeof(ctx.hdr_cfg.tx));

	ctx.hdr_cfg.rx.hdr_len = sizeof(rx_smsc_header);
	ctx.hdr_cfg.rx.is_little_endian = 1;

	memcpy(ctx.hdr_cfg.tx.raw_hdr, tx_smsc_header, sizeof(tx_smsc_header));
	ctx.hdr_cfg.tx.hdr_len = sizeof(tx_smsc_header) ;
	ctx.hdr_cfg.tx.hdr_ofst_pkt_size = 0;
	ctx.hdr_cfg.tx.hdr_ofst_pkt_size_valid = 1;
	ctx.hdr_cfg.tx.is_little_endian = 1;
}
#else
#error no header information to configure IPA header removal/addition
#endif


static void usbnet_ipa_set_hw_rx_flags(int flags)
{
	const struct net_device_ops *ops;

	USBNET_IPA_DBG_FUNC_ENTRY();

	if (!ctx.net) {
		USBNET_IPA_ERR("No net device set");
		return;
	}

	ops = ctx.net->netdev_ops;
	ctx.net->flags = flags;
	if (ops->ndo_set_rx_mode)
		ops->ndo_set_rx_mode(ctx.net);

	USBNET_IPA_DBG_FUNC_EXIT();
}

int usbnet_ipa_init(struct net_device *net)
{
	int ret;

	USBNET_IPA_DBG_FUNC_ENTRY();

	USBNET_IPA_DBG("device_ethaddr=%pM\n", net->dev_addr);

	/* Called from usbnet_probe, shouldn't be a concurency problem
	 * with disconnect/cleanup */
	ctx.net = net;

	memcpy(ctx.params.device_ethaddr, net->dev_addr, ETH_ALEN);
	ctx.params.set_hw_rx_flags = usbnet_ipa_set_hw_rx_flags;

	ret = odu_ipa_init(&ctx.params);
	if (ret)
		USBNET_IPA_ERR("failed to initialize odu_ipa");
	else
		USBNET_IPA_DBG("odu_ipa successful created");

	usbnet_config_header();
	USBNET_IPA_DBG("Configuring header");

	odu_ipa_add_hw_hdr_info(&ctx.hdr_cfg, ctx.params.priv);

	USBNET_IPA_DBG_FUNC_EXIT();

	return ret;
}

static void usbnet_ipa_free_urb(struct urb *urb)
{
	if (urb == NULL) {
		USBNET_IPA_ERR("URB is NULL, can not free!\n");
		return;
	}

	kfree(urb->priv_data);

	usb_free_urb(urb);
}

static void usbnet_ipa_bam_pipe_cb_tx(struct urb *urb)
{
	u32 *pipe = (u32 *) urb->context;

	USBNET_IPA_DBG("Callback recieved for pipe=%d", *pipe);
}

static void usbnet_ipa_bam_pipe_cb_rx(struct urb *urb)
{
	u32 *pipe = (u32 *) urb->context;

	USBNET_IPA_DBG("Callback recieved for pipe=%d", *pipe);
}

/* enqueue infinite transfer urb */
static int usbnet_ipa_enqueue(struct usb_interface *intf,
	unsigned int usb_pipe_handle,
	void (*cb_func)(struct urb *urb),
	u32 pipe, enum usb_bam_pipe_dir dir)
{
	struct urb *urb = NULL;
	struct usb_device *dev = interface_to_usbdev(intf);
	struct usb_host_bam_type *bam_type = NULL;
	int ret;
	int length = 0; /* This is just a dummy urb so length is 0 */

	USBNET_IPA_DBG_FUNC_ENTRY();

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (urb == NULL) {
		USBNET_IPA_ERR("URB allocation failed");
		return -ENOMEM;
	}

	usb_fill_bulk_urb(urb, dev, usb_pipe_handle, NULL, length,
		cb_func, &pipe);

	urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;

	if (usb_pipein(usb_pipe_handle)) {
		urb->transfer_flags |= URB_SHORT_NOT_OK;
		USBNET_IPA_DBG("Submitting RX URB");
	} else
		USBNET_IPA_DBG("Submitting TX URB");

	bam_type = kzalloc(sizeof(struct usb_host_bam_type), GFP_KERNEL);
	if (bam_type == NULL) {
		USBNET_IPA_ERR("Failed to allocate memory for BAM type\n");
		usb_free_urb(urb);
		return -ENOMEM;
	}
	bam_type->dir = dir;

	if (bam_type->dir == USB_TO_PEER_PERIPHERAL) {
		bam_type->pipe_num = pipe;
		USBNET_IPA_DBG("Submitting RX URB");
		ctx.rx_urb = urb;
	} else {
		USBNET_IPA_DBG("Submitting TX URB");
		bam_type->pipe_num = pipe;
		ctx.tx_urb = urb;
	}

	urb->priv_data = bam_type;

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		USBNET_IPA_ERR("Failed to submit URB for BAM pipe: %d\n",
				pipe);
		usbnet_ipa_free_urb(urb);
		return -EINVAL;
	}

	USBNET_IPA_DBG_FUNC_EXIT();
	return ret;
}

int usbnet_ipa_connect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle)
{
	u8 src_connection_idx, dst_connection_idx;
	struct usb_bam_connect_ipa_params *ipa_params = &ctx.ipa_params;
	int ret;

	USBNET_IPA_DBG_FUNC_ENTRY();

	ipa_params->src_client = IPA_CLIENT_HSIC1_PROD;
	ipa_params->dst_client = IPA_CLIENT_HSIC1_CONS;

	src_connection_idx = usb_bam_get_connection_idx(USBNET_CORE, IPA_P_BAM,
					USB_TO_PEER_PERIPHERAL, 0);
	dst_connection_idx = usb_bam_get_connection_idx(USBNET_CORE, IPA_P_BAM,
					PEER_PERIPHERAL_TO_USB, 0);
	if (src_connection_idx < 0 || dst_connection_idx < 0) {
		pr_err("%s: usb_bam_get_connection_idx failed\n", __func__);
		return ret;
	}

	USBNET_IPA_DBG("Using pipes: src=%d dst=%d",
		src_connection_idx, dst_connection_idx);

	ipa_params->src_pipe = &(ctx.src_pipe);
	ipa_params->dst_pipe = &(ctx.dst_pipe);
	ipa_params->src_idx = src_connection_idx;
	ipa_params->dst_idx = dst_connection_idx;

	/* Open producer pipe (rx) */
	ipa_params->dir = USB_TO_PEER_PERIPHERAL;
	ipa_params->notify = ctx.params.odu_ipa_rx_dp_notify;
	ipa_params->priv = ctx.params.priv;

	ret = usb_bam_connect_ipa(ipa_params);
	if (ret) {
		USBNET_IPA_ERR("usb_bam_connect_ipa failed: err:%d\n", ret);
		return ret;
	}

	/* Open consumer pipe (tx) */
	ipa_params->dir = PEER_PERIPHERAL_TO_USB;
	ipa_params->notify = ctx.params.odu_ipa_tx_dp_notify;
	ipa_params->priv = ctx.params.priv;
	ret = usb_bam_connect_ipa(ipa_params);
	if (ret) {
		USBNET_IPA_ERR("usb_bam_connect_ipa failed: err:%d\n", ret);
		return ret;
	}

	/* Connect IPA */
	ret = odu_ipa_connect(ipa_params->cons_clnt_hdl,
				ipa_params->prod_clnt_hdl,
				ipa_params->priv);
	if (ret) {
		USBNET_IPA_ERR("failed to connect IPA: err:%d\n", ret);
		return ret;
	}

	/* Enqueue tx infinite transfer - consumer pipe */
	ret = usbnet_ipa_enqueue(intf, tx_usb_pipe_handle,
		usbnet_ipa_bam_pipe_cb_tx, ctx.dst_pipe,
		PEER_PERIPHERAL_TO_USB);
	if (ret) {
		ctx.tx_urb = NULL;
		USBNET_IPA_ERR("failed to enqueue tx ep: err:%d\n", ret);
		return ret;
	}

	/* Enqueue rx infinite transfer - producer pipe */
	ret = usbnet_ipa_enqueue(intf, rx_usb_pipe_handle,
		usbnet_ipa_bam_pipe_cb_rx, ctx.src_pipe,
		USB_TO_PEER_PERIPHERAL);
	if (ret) {
		USBNET_IPA_ERR("failed to enqueue rx ep: err:%d\n", ret);
		usbnet_ipa_free_urb(ctx.rx_urb);
		ctx.rx_urb = ctx.tx_urb = NULL;
		return ret;
	}

	USBNET_IPA_DBG_FUNC_EXIT();
	return ret;
}

int usbnet_ipa_disconnect(struct usb_interface *intf,
	unsigned int rx_usb_pipe_handle, unsigned int tx_usb_pipe_handle)
{
	int ret;

	USBNET_IPA_DBG_FUNC_ENTRY();

	/* Disconnect pipes */
	ret = usb_bam_disconnect_ipa(&ctx.ipa_params);
	if (ret) {
		USBNET_IPA_ERR("usb_bam_disconnect_ipa failed: err:%d\n", ret);
		return ret;
	}

	/* Notify the IPA about it */
	ret = odu_ipa_disconnect(ctx.ipa_params.priv);
	if (ret) {
		USBNET_IPA_ERR("odu_ipa_disconnect failed: err:%d\n", ret);
		return ret;
	}

	/*Stop the BAM2BAM transfer */
	if (ctx.tx_urb == NULL) {
		USBNET_IPA_ERR("tx URB is NULL, can not kill!\n");
		return -ENODEV;
	} else
		usb_kill_urb(ctx.tx_urb);

	if (ctx.rx_urb == NULL) {
		USBNET_IPA_ERR("tx URB is NULL, can not kill!\n");
		return -ENODEV;
	} else
		usb_kill_urb(ctx.rx_urb);

	/* Free URBs */
	usbnet_ipa_free_urb(ctx.tx_urb);
	usbnet_ipa_free_urb(ctx.rx_urb);

	USBNET_IPA_DBG_FUNC_EXIT();

	return ret;
}

void usbnet_ipa_cleanup(void)
{
	USBNET_IPA_DBG_FUNC_ENTRY();

	ctx.net = NULL;
	odu_ipa_cleanup(ctx.ipa_params.priv);

	USBNET_IPA_DBG_FUNC_EXIT();
}

