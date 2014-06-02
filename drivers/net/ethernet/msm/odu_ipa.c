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

#include <linux/debugfs.h>
#include <linux/ipv6.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <net/addrconf.h>
#include <mach/odu_ipa.h>

#define DRIVER_NAME "odu_ipa"
#define ODU_IPA_IPV4_HDR_NAME "odu_hsic_ipv4"
#define ODU_IPA_IPV6_HDR_NAME "odu_hsic_ipv6"
#define HSIC_TO_IPA_CLIENT		IPA_CLIENT_HSIC1_PROD
#define IPA_TO_HSIC_CLIENT		IPA_CLIENT_HSIC1_CONS
#define IPA_TO_SYS_CLIENT		IPA_CLIENT_HSIC2_CONS

#define INACTIVITY_MSEC_DELAY 1000
#define DEFAULT_OUTSTANDING_HIGH 64
#define DEFAULT_OUTSTANDING_LOW 32
#define DEBUGFS_TEMP_BUF_SIZE 4
#define IPA_ODU_SYS_DESC_FIFO_SZ 0x800

#define ODU_IPA_ERR(fmt, args...) \
	pr_err(DRIVER_NAME " %s:%d " fmt, \
			__func__, __LINE__, ## args)

#define ODU_IPA_DBG(fmt, args...) \
	pr_debug(DRIVER_NAME " %s:%d " fmt, \
			__func__, __LINE__, ## args)

#define NULL_CHECK(ptr) \
	do { \
		if (!(ptr)) { \
			ODU_IPA_ERR("null pointer #ptr\n"); \
			return -EINVAL; \
		} \
	} \
	while (0)

#define ODU_IPA_LOG_ENTRY() ODU_IPA_DBG("ENTER\n")
#define ODU_IPA_LOG_EXIT() ODU_IPA_DBG("EXIT\n")

/**
 * enum odu_ipa_state - specify the current driver internal state
 *  which is guarded by a state machine.
 *
 * The driver internal state changes due to its external API usage.
 * The driver saves its internal state to guard from caller illegal
 * call sequence.
 * states:
 * UNLOADED is the first state which is the default one and is also the state
 *  after the driver gets unloaded (cleanup).
 * INITIALIZED is the driver state once it finished registering
 *  the network device and all internal data struct were initialized
 * CONNECTED is the driver state once the HSIC pipes were connected to IPA
 * UP is the driver state after the interface mode was set to UP but the
 *  pipes are not connected yet - this state is meta-stable state.
 * CONNECTED_AND_UP is the driver state when the pipe were connected and
 *  the interface got UP request from the network stack. this is the driver
 *   idle operation state which allows it to transmit/receive data.
 * INVALID is a state which is not allowed.
 */
enum odu_ipa_state {
	ODU_IPA_UNLOADED = 0,
	ODU_IPA_INITIALIZED,
	ODU_IPA_CONFIGURED,
	ODU_IPA_CONNECTED,
	ODU_IPA_UP,
	ODU_IPA_CONNECTED_AND_UP,
	ODU_IPA_INVALID,
};

/**
 * enum odu_ipa_operation - enumerations used to descibe the API operation
 *
 * Those enums are used as input for the driver state machine.
 */
enum odu_ipa_operation {
	ODU_IPA_INITIALIZE,
	ODU_IPA_CONFIGURE,
	ODU_IPA_CONNECT,
	ODU_IPA_OPEN,
	ODU_IPA_STOP,
	ODU_IPA_DISCONNECT,
	ODU_IPA_CLEANUP,
};

#define ODU_IPA_STATE_DEBUG(odu_ipa_ctx) \
	ODU_IPA_DBG("Driver state - %s", \
			odu_ipa_state_string(odu_ipa_ctx->state))

/**
 * struct odu_ipa_dev - main driver context parameters
 * @net: network interface struct implemented by this driver
 * @hw_hdr_info: hardware header information
 * @set_hw_rx_flags: HW specific callback for setting RX flags.
 * @hw_nway_reset: HW specific callback for restart auto-negotiation
 * @tx_enable: flag that enable/disable Tx path to continue to IPA
 * @rx_enable: flag that enable/disable Rx path to continue to network stack
 * @rm_enable: flag that enable/disable Resource manager request prior to Tx
 * @directory: debugfs directory for various debugging switches
 * @odu_hsic_ipv4_hdr_hdl: saved handle for ipv4 header-insertion table
 * @odu_hsic_ipv6_hdr_hdl: saved handle for ipv6 header-insertion table
 * @hsic_to_ipa_hdl: save handle for IPA pipe operations
 * @ipa_to_hsic_hdl: save handle for IPA pipe operations
 * @endianess_swap_pipe_hdl: handle for endianess swap system pipe
 * @outstanding_pkts: number of packets sent to IPA without TX complete ACKed
 * @outstanding_high: number of outstanding packets allowed
 * @outstanding_low: number of outstanding packets which shall cause
 *  to netdev queue start (after stopped due to outstanding_high reached)
 * @endianess_swapped_pkts: number of packets that were endianess swapped
 *  via the endianess swap sys pipe
 * @state: current state of odu_ipa driver
 */
struct odu_ipa_dev {
	struct net_device *net;
	struct odu_ipa_hw_hdr_info hw_hdr_info;
	int (*tx_fixup)(struct sk_buff *skb);
	void (*set_hw_rx_flags)(int flags);
	void (*hw_nway_reset)(void);
	u32 tx_enable;
	u32 rx_enable;
	u32  rm_enable;
	struct dentry *directory;
	uint32_t odu_hsic_ipv4_hdr_hdl;
	uint32_t odu_hsic_ipv6_hdr_hdl;
	u32 hsic_to_ipa_hdl;
	u32 ipa_to_hsic_hdl;
	u32 endianess_swap_pipe_hdl;
	atomic_t outstanding_pkts;
	u8 outstanding_high;
	u8 outstanding_low;
	u32 endianess_swapped_pkts;
	enum odu_ipa_state state;
};

static int odu_ipa_open(struct net_device *net);
static void odu_ipa_set_rx_mode(struct net_device *net);
static void odu_ipa_packet_receive_notify(void *priv,
		enum ipa_dp_evt_type evt, unsigned long data);
static void odu_ipa_tx_complete_notify(void *priv,
		enum ipa_dp_evt_type evt, unsigned long data);
static void odu_ipa_sys_pipe_rx_cb(void *priv,
		enum ipa_dp_evt_type evt, unsigned long data);
static int odu_ipa_stop(struct net_device *net);
static int odu_ipa_rules_cfg(struct odu_ipa_dev *odu_ipa_ctx);
static int odu_ipa_setup_sys_pipe(struct odu_ipa_dev *odu_ipa_ctx);
static int odu_ipa_teardown_sys_pipe(struct odu_ipa_dev *odu_ipa_ctx);
static void odu_ipa_rules_destroy(struct odu_ipa_dev *odu_ipa_ctx);
static int odu_ipa_send_dl_skb(struct sk_buff *skb, void *priv);
static int odu_ipa_register_properties(struct odu_ipa_dev *odu_ipa_ctx);
static void odu_ipa_deregister_properties(struct odu_ipa_dev *odu_ipa_ctx);
static void odu_ipa_rm_notify(void *user_data, enum ipa_rm_event event,
		unsigned long data);
static int odu_ipa_create_rm_resource(struct odu_ipa_dev *odu_ipa_ctx);
static void odu_ipa_destory_rm_resource(struct odu_ipa_dev *odu_ipa_ctx);
static bool rx_filter(struct sk_buff *skb);
static bool tx_filter(struct sk_buff *skb);
static bool rm_enabled(struct odu_ipa_dev *odu_ipa_ctx);
static int resource_request(struct odu_ipa_dev *odu_ipa_ctx);
static void resource_release(struct odu_ipa_dev *odu_ipa_ctx);
static netdev_tx_t odu_ipa_start_xmit(struct sk_buff *skb,
					struct net_device *net);
static int odu_ipa_debugfs_atomic_open(struct inode *inode, struct file *file);
static ssize_t odu_ipa_debugfs_atomic_read(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos);
static int odu_ipa_debugfs_init(struct odu_ipa_dev *odu_ipa_ctx);
static void odu_ipa_debugfs_destroy(struct odu_ipa_dev *odu_ipa_ctx);
static int odu_ipa_ep_registers_cfg(struct odu_ipa_dev *odu_ipa_ctx);
static int odu_ipa_set_device_ethernet_addr(u8 *dev_ethaddr,
		u8 device_ethaddr[]);
static enum odu_ipa_state odu_ipa_next_state(enum odu_ipa_state current_state,
		enum odu_ipa_operation operation);
static const char *odu_ipa_state_string(enum odu_ipa_state state);
static bool odu_ipa_is_ofst_valid(bool valid, u32 ofst, u32 len);
static int odu_ipa_init_module(void);
static void odu_ipa_cleanup_module(void);

static const struct net_device_ops odu_ipa_netdev_ops = {
	.ndo_open		= odu_ipa_open,
	.ndo_stop		= odu_ipa_stop,
	.ndo_start_xmit = odu_ipa_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_set_rx_mode = odu_ipa_set_rx_mode,
};

const struct file_operations odu_ipa_debugfs_atomic_ops = {
	.open = odu_ipa_debugfs_atomic_open,
	.read = odu_ipa_debugfs_atomic_read,
};

/**
 * odu_ipa_init() - create network device and initializes internal
 *  data structures
 * @params: in/out parameters required for odu_ipa initialization
 *
 * Shall be called prior to pipe connection.
 * The callbacks out parameters shall be supplied to ipa_connect.
 * Detailed description:
 *  - allocate the network device
 *  - Initialize ODU bridge
 *  - set default values for driver internals
 *  - create debugfs folder and files
 *  - create IPA resource manager client
 *  - set the carrier state to "off" (until odu_ipa_connect is called)
 *  - set the out parameters
 *
 * Returns negative errno, or zero on success
 */
int odu_ipa_init(struct odu_ipa_params *params)
{
	int result = 0;
	struct net_device *net;
	struct odu_ipa_dev *odu_ipa_ctx;

	ODU_IPA_LOG_ENTRY();
	ODU_IPA_DBG("%s initializing\n", DRIVER_NAME);
	NULL_CHECK(params);
	NULL_CHECK(params->set_hw_rx_flags);

	ODU_IPA_DBG("device_ethaddr=%pM\n", params->device_ethaddr);

	net = alloc_etherdev(sizeof(struct odu_ipa_dev));
	if (!net) {
		result = -ENOMEM;
		ODU_IPA_ERR("fail to allocate etherdev\n");
		goto fail_alloc_etherdev;
	}
	ODU_IPA_DBG("network device was successfully allocated\n");

	odu_ipa_ctx = netdev_priv(net);
	memset(odu_ipa_ctx, 0, sizeof(*odu_ipa_ctx));

	result = odu_bridge_init(odu_ipa_send_dl_skb, odu_ipa_ctx,
				odu_ipa_packet_receive_notify,
				&params->odu_ipa_rx_dp_notify);
	if (result) {
		ODU_IPA_ERR("fail to init bridge\n");
		goto fail_init_bridge;
	}

	odu_ipa_ctx->net = net;
	odu_ipa_ctx->tx_enable = true;
	odu_ipa_ctx->rx_enable = true;
	odu_ipa_ctx->rm_enable = true;
	odu_ipa_ctx->outstanding_high = DEFAULT_OUTSTANDING_HIGH;
	odu_ipa_ctx->outstanding_low = DEFAULT_OUTSTANDING_LOW;
	atomic_set(&odu_ipa_ctx->outstanding_pkts, 0);
	snprintf(net->name, sizeof(net->name), "%s%%d", "odu");
	net->netdev_ops = &odu_ipa_netdev_ops;
	ODU_IPA_DBG("internal data structures intialized\n");

	result = odu_ipa_debugfs_init(odu_ipa_ctx);
	if (result)
		goto fail_debugfs;
	ODU_IPA_DBG("debugfs entries were created\n");

	result = odu_ipa_create_rm_resource(odu_ipa_ctx);
	if (result) {
		ODU_IPA_ERR("fail on RM create\n");
		goto fail_create_rm;
	}
	ODU_IPA_DBG("RM resource was created\n");

	result = odu_ipa_set_device_ethernet_addr(net->dev_addr,
			params->device_ethaddr);
	if (result) {
		ODU_IPA_ERR("set device MAC failed\n");
		goto fail_set_device_ethernet;
	}
	ODU_IPA_DBG("Device Ethernet address set %pM\n", net->dev_addr);

	netif_carrier_off(net);
	ODU_IPA_DBG("set carrier off\n");

	params->odu_ipa_tx_dp_notify = odu_ipa_tx_complete_notify;
	params->priv = (void *)odu_ipa_ctx;
	odu_ipa_ctx->tx_fixup = params->tx_fixup;
	odu_ipa_ctx->set_hw_rx_flags = params->set_hw_rx_flags;
	odu_ipa_ctx->hw_nway_reset = params->hw_nway_reset;
	odu_ipa_ctx->state = ODU_IPA_INITIALIZED;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	ODU_IPA_LOG_EXIT();

	return 0;

fail_set_device_ethernet:
	odu_ipa_destory_rm_resource(odu_ipa_ctx);
fail_create_rm:
	odu_ipa_debugfs_destroy(odu_ipa_ctx);
fail_debugfs:
	odu_bridge_cleanup();
fail_init_bridge:
	free_netdev(net);
fail_alloc_etherdev:
	return result;
}
EXPORT_SYMBOL(odu_ipa_init);


/**
 * odu_ipa_add_hw_hdr_info() - Add information regarding HW header that shall
 * be used on hsic pipes.
 * @hw_hdr_info: header information
 * @priv: same value that was set by odu_ipa_init(), this
 * parameter holds the network device pointer.
 *
 * Shall be called prior to pipe connection, and after initialization.
 * Detailed description:
 *  - add hardware header information to ODU bridge
 *  - add partial header insertion rules for IPA driver (based on hardware
 *    header and device's Ethernet address given in input params)
 *  - register tx/rx properties to IPA driver (will be later used
 *    by IPA configuration manager to configure the full header and
 *    reset of the IPA rules)
 *  - register the network device
 *
 * Returns negative errno, or zero on success
 */
int odu_ipa_add_hw_hdr_info(struct odu_ipa_hw_hdr_info *hw_hdr_info,
			    void *priv)
{
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int next_state;
	int result;

	ODU_IPA_LOG_ENTRY();
	NULL_CHECK(hw_hdr_info);
	NULL_CHECK(priv);

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_CONFIGURE);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't add header info before initialization\n");
		return -EPERM;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	if (hw_hdr_info->tx.hdr_len == 0 || hw_hdr_info->rx.hdr_len == 0) {
		ODU_IPA_ERR("zero length header\n");
		return -EINVAL;
	}

	if (!odu_ipa_is_ofst_valid(hw_hdr_info->tx.hdr_ofst_pkt_size_valid,
		hw_hdr_info->tx.hdr_ofst_pkt_size,
		hw_hdr_info->tx.hdr_len)) {
			ODU_IPA_ERR("offset larger than header\n");
			return -EINVAL;
	}

	if (!odu_ipa_is_ofst_valid(hw_hdr_info->rx.hdr_ofst_pkt_size_valid,
		hw_hdr_info->rx.hdr_ofst_pkt_size,
		hw_hdr_info->rx.hdr_len)) {
			ODU_IPA_ERR("offset larger than header\n");
			return -EINVAL;
	}

	if (!odu_ipa_is_ofst_valid(hw_hdr_info->rx.hdr_ofst_metadata_valid,
		hw_hdr_info->rx.hdr_ofst_metadata,
		hw_hdr_info->rx.hdr_len)) {
			ODU_IPA_ERR("offset larger than header\n");
			return -EINVAL;
	}

	result = odu_bridge_add_hw_hdr_info(hw_hdr_info);
	if (result) {
		ODU_IPA_ERR("failed to add header info to bridge\n");
		goto fail_add_hw_hdr_info;
	}

	memcpy(&odu_ipa_ctx->hw_hdr_info, hw_hdr_info, sizeof(*hw_hdr_info));

	result = odu_ipa_rules_cfg(odu_ipa_ctx);
	if (result) {
		ODU_IPA_ERR("fail on ipa rules set\n");
		goto fail_rules_cfg;
	}
	ODU_IPA_DBG("Ethernet header insertion set\n");

	/* setup SYS2BAM pipe for endianess swap */
	if (odu_ipa_ctx->tx_fixup || hw_hdr_info->tx.is_little_endian) {
		ODU_IPA_DBG("configure SYS pipe\n");
		result = odu_ipa_setup_sys_pipe(odu_ipa_ctx);
		if (result) {
			ODU_IPA_ERR("fail on setup sys pipe\n");
			goto fail_setup_sys_pipe;
		}
		ODU_IPA_DBG("SYS pipe setup done\n");
	}

	odu_ipa_ctx->net->needed_headroom += hw_hdr_info->tx.hdr_len;
	result = register_netdev(odu_ipa_ctx->net);
	if (result) {
		ODU_IPA_ERR("register_netdev failed: %d\n", result);
		goto fail_register_netdev;
	}
	ODU_IPA_DBG("register_netdev succeeded\n");

	result = odu_ipa_register_properties(odu_ipa_ctx);
	if (result) {
		ODU_IPA_ERR("fail on properties set\n");
		goto fail_register_tx;
	}
	ODU_IPA_DBG("odu_ipa 2 Tx and 2 Rx properties were registered\n");

	ODU_IPA_LOG_EXIT();
	return 0;

fail_register_tx:
	unregister_netdev(odu_ipa_ctx->net);
fail_register_netdev:
	if (odu_ipa_ctx->tx_fixup || hw_hdr_info->tx.is_little_endian)
		odu_ipa_teardown_sys_pipe(odu_ipa_ctx);
fail_setup_sys_pipe:
	odu_ipa_rules_destroy(odu_ipa_ctx);
fail_rules_cfg:
fail_add_hw_hdr_info:
	return result;

}
EXPORT_SYMBOL(odu_ipa_add_hw_hdr_info);

/**
 * odu_ipa_connect() - notify odu_ipa for IPA<->HSIC pipes connection
 * @hsic_to_ipa_hdl: handle of IPA driver client for HSIC->IPA
 * @ipa_to_hsic_hdl: handle of IPA driver client for IPA->HSIC
 * @priv: same value that was set by odu_ipa_init(), this
 *  parameter holds the network device pointer.
 *
 * Once USB driver finishes the pipe connection between IPA core
 * and USB core this method shall be called in order to
 * allow odu_ipa complete the data path configurations.
 * Detailed description:
 *  - configure the IPA end-points register
 *  - notify the Linux kernel for "carrier_on"
 *  After this function is done the driver state changes to "Connected".
 *  This API is expected to be called after odu_ipa_init() or
 *  after a call to odu_ipa_disconnect.
 */
int odu_ipa_connect(u32 hsic_to_ipa_hdl, u32 ipa_to_hsic_hdl,
		void *priv)
{
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int next_state;
	int result;

	ODU_IPA_LOG_ENTRY();
	NULL_CHECK(priv);
	ODU_IPA_DBG("hsic_to_ipa_hdl = %d, ipa_to_hsic_hdl = %d, priv=0x%p\n",
					hsic_to_ipa_hdl, ipa_to_hsic_hdl, priv);

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_CONNECT);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't call connect before adding header info\n");
		return -EPERM;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	if (!hsic_to_ipa_hdl || hsic_to_ipa_hdl >= IPA_CLIENT_MAX) {
		ODU_IPA_ERR("hsic_to_ipa_hdl(%d) is not a valid ipa handle\n",
				hsic_to_ipa_hdl);
		return -EINVAL;
	}
	if (!ipa_to_hsic_hdl || ipa_to_hsic_hdl >= IPA_CLIENT_MAX) {
		ODU_IPA_ERR("ipa_to_hsic_hdl(%d) is not a valid ipa handle\n",
				ipa_to_hsic_hdl);
		return -EINVAL;
	}

	result = odu_bridge_connect();
	if (result) {
		ODU_IPA_ERR("bridge connect failed\n");
		return result;
	}

	odu_ipa_ctx->ipa_to_hsic_hdl = ipa_to_hsic_hdl;
	odu_ipa_ctx->hsic_to_ipa_hdl = hsic_to_ipa_hdl;
	odu_ipa_ep_registers_cfg(odu_ipa_ctx);
	ODU_IPA_DBG("end-point configured\n");

	netif_carrier_on(odu_ipa_ctx->net);
	if (!netif_carrier_ok(odu_ipa_ctx->net)) {
		ODU_IPA_ERR("netif_carrier_ok error\n");
		return -EBUSY;
	}
	ODU_IPA_DBG("carrier_on notified, odu_ipa is operational\n");

	if (odu_ipa_ctx->state == ODU_IPA_CONNECTED_AND_UP) {
		netif_start_queue(odu_ipa_ctx->net);
		ODU_IPA_DBG("queue started\n");
	}

	ODU_IPA_LOG_EXIT();

	return 0;
}
EXPORT_SYMBOL(odu_ipa_connect);

/**
 * odu_ipa_open() - notify Linux network stack to start sending packets
 * @net: the network interface supplied by the network stack
 *
 * Linux uses this API to notify the driver that the network interface
 * transitions to the up state.
 * The driver will instruct the Linux network stack to start
 * delivering data packets.
 */
static int odu_ipa_open(struct net_device *net)
{
	struct odu_ipa_dev *odu_ipa_ctx;
	int next_state;

	ODU_IPA_LOG_ENTRY();

	odu_ipa_ctx = netdev_priv(net);

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_OPEN);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't bring driver up before initialize\n");
		return -EPERM;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	if (odu_ipa_ctx->state == ODU_IPA_CONNECTED_AND_UP) {
		netif_start_queue(net);
		ODU_IPA_DBG("queue started\n");
	} else {
		ODU_IPA_DBG("queue was not started since not connected\n");
	}

	ODU_IPA_LOG_EXIT();

	return 0;
}

/**
 * odu_ipa_set_rx_mode() - notify HW callback of new RX flags
 *
 * This function calls to hardware specific callback for setting
 * Rx flags given as parameter in odu_init()
 */
static void odu_ipa_set_rx_mode(struct net_device *net)
{
	struct odu_ipa_dev *odu_ipa_ctx;

	ODU_IPA_LOG_ENTRY();
	ODU_IPA_DBG("flags = 0x%x\n", net->flags);
	odu_ipa_ctx = netdev_priv(net);
	odu_ipa_ctx->set_hw_rx_flags(net->flags);
	ODU_IPA_LOG_EXIT();
}

/**
 * odu_ipa_start_xmit() - send data from APPs to USB core via IPA core
 * @skb: packet received from Linux network stack
 * @net: the network device being used to send this packet
 *
 * Several conditions needed in order to send the packet to IPA:
 * - Transmit queue for the network driver is currently
 *   in "send" state
 * - The driver internal state is in "UP" state.
 * - Filter Tx switch is turned off
 * - The IPA resource manager state for the driver producer client
 *   is "Granted" which implies that all the resources in the dependency
 *   graph are valid for data flow.
 * - outstanding high boundary did not reach.
 *
 * In case all of the above conditions are met, the network driver will
 * send the packet by using the IPA API for Tx.
 * In case the outstanding packet high boundary is reached, the driver will
 * stop the send queue until enough packet were proceeded by the IPA core.
 */
static netdev_tx_t odu_ipa_start_xmit(struct sk_buff *skb,
					struct net_device *net)
{
	int ret;
	netdev_tx_t status = NETDEV_TX_BUSY;
	struct odu_ipa_dev *odu_ipa_ctx = netdev_priv(net);
	struct ipv6hdr *ipv6hdr;

	if (unlikely(netif_queue_stopped(net))) {
		ODU_IPA_ERR("interface queue is stopped\n");
		goto out;
	}

	if (unlikely(odu_ipa_ctx->state != ODU_IPA_CONNECTED_AND_UP)) {
		ODU_IPA_ERR("Missing pipe connected and/or iface up\n");
		return NETDEV_TX_BUSY;
	}

	if (unlikely(tx_filter(skb))) {
		dev_kfree_skb_any(skb);
		ODU_IPA_DBG("packet got filtered out on Tx path\n");
		status = NETDEV_TX_OK;
		goto out;
	}
	ret = resource_request(odu_ipa_ctx);
	if (ret) {
		ODU_IPA_DBG("Waiting to resource\n");
		netif_stop_queue(net);
		goto resource_busy;
	}

	if (atomic_read(&odu_ipa_ctx->outstanding_pkts) >=
					odu_ipa_ctx->outstanding_high) {
		ODU_IPA_DBG("Outstanding high (%d)- stopping queue\n",
				odu_ipa_ctx->outstanding_high);
		netif_stop_queue(net);
		status = NETDEV_TX_BUSY;
		goto out;
	}

	/* odu_bridge need to get all ipv6 multicast packets */
	ipv6hdr = (struct ipv6hdr *)(skb->data + ETH_HLEN);
	if (ipv6hdr->version == 6 &&
		ipv6_addr_is_multicast(&ipv6hdr->daddr)) {
		ODU_IPA_DBG("Multicast packet - to bridge\n");
		ret = odu_bridge_handle_mcast_skb(skb);
		if (ret) {
			ODU_IPA_ERR("bridge_handle_mcast failed (%d)\n", ret);
			goto fail_tx_packet;
		}
	}

	ret = odu_ipa_send_dl_skb(skb, odu_ipa_ctx);
	if (ret) {
		ODU_IPA_DBG("send dl skb failed (%d)\n", ret);
		goto fail_tx_packet;
	}

	status = NETDEV_TX_OK;
	goto out;

fail_tx_packet:
out:
	resource_release(odu_ipa_ctx);
resource_busy:
	return status;
}

/**
 * odu_ipa_packet_receive_notify() - Rx notify
 *
 * @priv: odu driver context
 * @evt: event type
 * @data: data provided with event
 *
 * IPA will pass a packet to the Linux network stack with skb->data pointing
 * to Ethernet packet frame.
 */
static void odu_ipa_packet_receive_notify(void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int result;

	if (evt != IPA_RECEIVE)	{
		ODU_IPA_ERR("A none IPA_RECEIVE event in odu_ipa_receive\n");
		return;
	}

	skb->dev = odu_ipa_ctx->net;
	skb->protocol = eth_type_trans(skb, odu_ipa_ctx->net);
	if (rx_filter(skb)) {
		ODU_IPA_DBG("packet got filtered out on Rx path\n");
		dev_kfree_skb_any(skb);
		return;
	}

	result = netif_rx(skb);
	if (result)
		ODU_IPA_ERR("fail on netif_rx\n");
	odu_ipa_ctx->net->stats.rx_packets++;
	odu_ipa_ctx->net->stats.rx_bytes += skb->len;

	return;
}

/** odu_ipa_stop() - called when network device transitions to the down
 *     state.
 *  @net: the network device being stopped.
 *
 * This API is used by Linux network stack to notify the network driver that
 * its state was changed to "down"
 * The driver will stop the "send" queue and change its internal
 * state to "Connected".
 */
static int odu_ipa_stop(struct net_device *net)
{
	struct odu_ipa_dev *odu_ipa_ctx = netdev_priv(net);
	int next_state;

	ODU_IPA_LOG_ENTRY();

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_STOP);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't do network interface down without up\n");
		return -EPERM;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	netif_stop_queue(net);
	ODU_IPA_DBG("network device stopped\n");

	if (odu_ipa_ctx->hw_nway_reset) {
		odu_ipa_ctx->hw_nway_reset();
		ODU_IPA_DBG("restarted auto-negotiation on HW\n");
	}

	ODU_IPA_LOG_EXIT();
	return 0;
}

/** odu_ipa_disconnect() - called when the ethernet cable is unplugged.
 * @priv: same value that was set by odu_ipa_init(), this
 *  parameter holds the network device pointer.
 *
 * Once the ethernet cable is unplugged the USB driver will notify the network
 * interface driver.
 * The internal driver state will returned to its initialized state and
 * Linux network stack will be informed for carrier off and the send queue
 * will be stopped.
 */
int odu_ipa_disconnect(void *priv)
{
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int next_state;
	int result;

	ODU_IPA_LOG_ENTRY();
	NULL_CHECK(odu_ipa_ctx);
	ODU_IPA_DBG("priv=0x%p\n", priv);

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_DISCONNECT);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't disconnect before connect\n");
		return -EPERM;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	result = odu_bridge_disconnect();
	if (result) {
		ODU_IPA_ERR("bridge disconnect failed\n");
		return result;
	}

	netif_carrier_off(odu_ipa_ctx->net);
	ODU_IPA_DBG("carrier_off notifcation was sent\n");

	netif_stop_queue(odu_ipa_ctx->net);
	ODU_IPA_DBG("queue stopped\n");

	atomic_set(&odu_ipa_ctx->outstanding_pkts, 0);
	ODU_IPA_DBG("outstanding_pkts reset\n");

	ODU_IPA_LOG_EXIT();

	return 0;
}
EXPORT_SYMBOL(odu_ipa_disconnect);


/**
 * odu_ipa_cleanup() - unregister the network interface driver and free
 *  internal data structs.
 * @priv: same value that was set by odu_ipa_init(), this
 *   parameter holds the network device pointer.
 *
 * This function shall be called once the network interface is not
 * needed anymore.
 * This function shall be called after the pipes were disconnected.
 * Detailed description:
 *  - delete the driver dependency defined for IPA resource manager and
 *   destroy the producer resource.
 *  -  remove the debugfs entries
 *  - deregister the network interface from Linux network stack
 *  - free all internal data structs
 */
void odu_ipa_cleanup(void *priv)
{
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int next_state;

	ODU_IPA_LOG_ENTRY();

	ODU_IPA_DBG("priv=0x%p\n", priv);

	if (!odu_ipa_ctx) {
		ODU_IPA_ERR("odu_ipa_ctx NULL pointer\n");
		return;
	}

	next_state = odu_ipa_next_state(odu_ipa_ctx->state, ODU_IPA_CLEANUP);
	if (next_state == ODU_IPA_INVALID) {
		ODU_IPA_ERR("can't clean driver without cable disconnect\n");
		return;
	}
	odu_ipa_ctx->state = next_state;
	ODU_IPA_STATE_DEBUG(odu_ipa_ctx);

	odu_ipa_deregister_properties(odu_ipa_ctx);

	if (odu_ipa_ctx->tx_fixup ||
	    odu_ipa_ctx->hw_hdr_info.tx.is_little_endian)
		odu_ipa_teardown_sys_pipe(odu_ipa_ctx);

	odu_ipa_destory_rm_resource(odu_ipa_ctx);
	odu_ipa_debugfs_destroy(odu_ipa_ctx);

	unregister_netdev(odu_ipa_ctx->net);
	free_netdev(odu_ipa_ctx->net);

	odu_bridge_cleanup();

	ODU_IPA_DBG("cleanup done\n");
	ODU_IPA_LOG_EXIT();

	return;
}
EXPORT_SYMBOL(odu_ipa_cleanup);

/**
 * odu_ipa_rules_cfg() - set header insertion and register Tx/Rx properties
 *				Headers will be commited to HW
 * @odu_ipa_ctx: main driver context parameters
 *
 * Returns negative errno, or zero on success
 */
static int odu_ipa_rules_cfg(struct odu_ipa_dev *odu_ipa_ctx)
{
	struct ipa_ioc_add_hdr *hdrs;
	struct ipa_hdr_add *ipv4_hdr;
	struct ipa_hdr_add *ipv6_hdr;
	struct ethhdr *eth_ipv4;
	struct ethhdr *eth_ipv6;
	int result = 0;

	ODU_IPA_LOG_ENTRY();
	hdrs = kzalloc(sizeof(*hdrs) + sizeof(*ipv4_hdr) + sizeof(*ipv6_hdr),
			GFP_KERNEL);
	if (!hdrs) {
		result = -ENOMEM;
		goto out;
	}
	ipv4_hdr = &hdrs->hdr[0];
	memcpy(&ipv4_hdr->hdr, &odu_ipa_ctx->hw_hdr_info.tx.raw_hdr,
		odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
	eth_ipv4 = (struct ethhdr *)(ipv4_hdr->hdr +
		odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
	ipv6_hdr = &hdrs->hdr[1];
	memcpy(&ipv6_hdr->hdr, &odu_ipa_ctx->hw_hdr_info.tx.raw_hdr,
		odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
	eth_ipv6 = (struct ethhdr *)(ipv6_hdr->hdr +
		odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
	strlcpy(ipv4_hdr->name, ODU_IPA_IPV4_HDR_NAME, IPA_RESOURCE_NAME_MAX);
	memcpy(eth_ipv4->h_source, odu_ipa_ctx->net->dev_addr, ETH_ALEN);
	eth_ipv4->h_proto = htons(ETH_P_IP);
	ipv4_hdr->hdr_len = odu_ipa_ctx->hw_hdr_info.tx.hdr_len + ETH_HLEN;
	ipv4_hdr->is_partial = 1;
	strlcpy(ipv6_hdr->name, ODU_IPA_IPV6_HDR_NAME, IPA_RESOURCE_NAME_MAX);
	memcpy(eth_ipv6->h_source, odu_ipa_ctx->net->dev_addr, ETH_ALEN);
	eth_ipv6->h_proto = htons(ETH_P_IPV6);
	ipv6_hdr->hdr_len = odu_ipa_ctx->hw_hdr_info.tx.hdr_len + ETH_HLEN;
	ipv6_hdr->is_partial = 1;
	hdrs->commit = 1;
	hdrs->num_hdrs = 2;
	result = ipa_add_hdr(hdrs);
	if (result) {
		ODU_IPA_ERR("Fail on Header-Insertion(%d)\n", result);
		goto out_free_mem;
	}
	if (ipv4_hdr->status) {
		ODU_IPA_ERR("Fail on Header-Insertion ipv4(%d)\n",
				ipv4_hdr->status);
		result = ipv4_hdr->status;
		goto out_free_mem;
	}
	if (ipv6_hdr->status) {
		ODU_IPA_ERR("Fail on Header-Insertion ipv6(%d)\n",
				ipv6_hdr->status);
		result = ipv6_hdr->status;
		goto out_free_mem;
	}
	odu_ipa_ctx->odu_hsic_ipv4_hdr_hdl = ipv4_hdr->hdr_hdl;
	odu_ipa_ctx->odu_hsic_ipv6_hdr_hdl = ipv6_hdr->hdr_hdl;
	ODU_IPA_LOG_EXIT();
out_free_mem:
	kfree(hdrs);
out:
	return result;
}

/**
 * odu_ipa_setup_sys_pipe() - Setup system pipe for endianess swap in header.
 * @odu_ipa_ctx: main driver context parameters
 *
 * IPA core can handle header insertion in big endian only. for little endian
 * header, SW needs to swap length bytes in header.
 *
 * Returns negative errno, or zero on success
 */
static int odu_ipa_setup_sys_pipe(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result;
	struct ipa_sys_connect_params sys_connect_params;

	ODU_IPA_LOG_ENTRY();

	memset(&sys_connect_params, 0, sizeof(sys_connect_params));
	sys_connect_params.desc_fifo_sz = IPA_ODU_SYS_DESC_FIFO_SZ;
	sys_connect_params.client = IPA_TO_SYS_CLIENT;
	sys_connect_params.notify = odu_ipa_sys_pipe_rx_cb;
	sys_connect_params.priv = odu_ipa_ctx;
	sys_connect_params.ipa_ep_cfg.aggr.aggr_en = IPA_BYPASS_AGGR;
	sys_connect_params.ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid =
		odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size_valid;
	sys_connect_params.ipa_ep_cfg.hdr.hdr_ofst_pkt_size =
		odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size;
	sys_connect_params.ipa_ep_cfg.hdr.hdr_len =
		odu_ipa_ctx->hw_hdr_info.tx.hdr_len + ETH_HLEN;
	sys_connect_params.ipa_ep_cfg.hdr.hdr_additional_const_len = ETH_HLEN;
	sys_connect_params.ipa_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
	result = ipa_setup_sys_pipe(&sys_connect_params,
					&odu_ipa_ctx->endianess_swap_pipe_hdl);
	if (result) {
		ODU_IPA_ERR("setup sys pipe failed %d\n", result);
		return result;
	}

	ODU_IPA_LOG_EXIT();
	return 0;
}

/**
 * odu_ipa_teardown_sys_pipe() - Teardown system pipe that was setup by
 * odu_ipa_setup_sys_pipe().
 *
 * Returns negative errno, or zero on success
 */
static int odu_ipa_teardown_sys_pipe(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result;

	ODU_IPA_LOG_ENTRY();

	if (odu_ipa_ctx->endianess_swap_pipe_hdl) {
		result = ipa_teardown_sys_pipe(
				odu_ipa_ctx->endianess_swap_pipe_hdl);
		if (result) {
			ODU_IPA_ERR("teardown sys pipe failed %d\n", result);
			return result;
		}
		odu_ipa_ctx->endianess_swap_pipe_hdl = 0;
	}

	ODU_IPA_LOG_EXIT();
	return 0;
}

/**
 * odu_ipa_rules_destroy() - remove the IPA core configuration done for
 *  the driver data path.
 *  @odu_ipa_ctx: the driver context
 *
 *  Revert the work done on odu_ipa_rules_cfg.
 */
static void odu_ipa_rules_destroy(struct odu_ipa_dev *odu_ipa_ctx)
{
	struct ipa_ioc_del_hdr *del_hdr;
	struct ipa_hdr_del *ipv4;
	struct ipa_hdr_del *ipv6;
	int result;
	del_hdr = kzalloc(sizeof(*del_hdr) + sizeof(*ipv4) +
			sizeof(*ipv6), GFP_KERNEL);
	if (!del_hdr)
		return;
	del_hdr->commit = 1;
	del_hdr->num_hdls = 2;
	ipv4 = &del_hdr->hdl[0];
	ipv4->hdl = odu_ipa_ctx->odu_hsic_ipv4_hdr_hdl;
	ipv6 = &del_hdr->hdl[1];
	ipv6->hdl = odu_ipa_ctx->odu_hsic_ipv6_hdr_hdl;
	result = ipa_del_hdr(del_hdr);
	if (result || ipv4->status || ipv6->status)
		ODU_IPA_ERR("ipa_del_hdr failed");
}

/**
*  odu_ipa_send_dl_skb() - Adds HW header and send the packet to IPA
*  @skb: skb to be send
*  @priv: same value that was set by odu_ipa_init(), this
*  parameter holds the network device pointer.
*/
static int odu_ipa_send_dl_skb(struct sk_buff *skb, void *priv)
{
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	int ret;

	if (odu_ipa_ctx->tx_fixup) {
		ret = odu_ipa_ctx->tx_fixup(skb);
		if (ret) {
			ODU_IPA_ERR("tx_fixup failed (%d)\n", ret);
			return ret;
		}
	} else {
		/* add hw header to skb */
		memcpy(skb_push(skb, odu_ipa_ctx->hw_hdr_info.tx.hdr_len),
			odu_ipa_ctx->hw_hdr_info.tx.raw_hdr,
			odu_ipa_ctx->hw_hdr_info.tx.hdr_len);

		/* add the packet length to the header */
		if (odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size_valid) {
			u16 *pkt_len_ptr = (u16 *)(skb->data +
				odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size);
			*pkt_len_ptr = skb->len -
				odu_ipa_ctx->hw_hdr_info.tx.hdr_len;
			if (!odu_ipa_ctx->hw_hdr_info.tx.is_little_endian)
				*pkt_len_ptr = htons(*pkt_len_ptr);
		}
	}

	ret = ipa_tx_dp(IPA_TO_HSIC_CLIENT, skb, NULL);
	if (ret) {
		ODU_IPA_DBG("ipa transmit failed (%d)\n", ret);
		skb_pull(skb, odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
		return ret;
	}

	atomic_inc(&odu_ipa_ctx->outstanding_pkts);
	odu_ipa_ctx->net->stats.tx_packets++;
	odu_ipa_ctx->net->stats.tx_bytes += skb->len;

	return 0;
}

/* odu_ipa_register_properties() - set Tx/Rx properties for ipacm
 *
 * @name: name of odu_ipa network interface
 *
 * Register odu0 interface with 2 Tx properties and 2 Rx properties:
 * The 2 Tx properties are for data flowing from IPA to HSIC, they
 * have Header-Insertion properties both for Ipv4 and Ipv6 Ethernet framing.
 * The 2 Rx properties are for data flowing from HSIC to IPA, they have
 * simple rule which always "hit".
 *
 */
static int odu_ipa_register_properties(struct odu_ipa_dev *odu_ipa_ctx)
{
	struct ipa_tx_intf tx_properties = {0};
	struct ipa_ioc_tx_intf_prop properties[2] = { {0}, {0} };
	struct ipa_ioc_tx_intf_prop *ipv4_property;
	struct ipa_ioc_tx_intf_prop *ipv6_property;
	struct ipa_ioc_rx_intf_prop rx_ioc_properties[2] = { {0}, {0} };
	struct ipa_rx_intf rx_properties = {0};
	struct ipa_ioc_rx_intf_prop *rx_ipv4_property;
	struct ipa_ioc_rx_intf_prop *rx_ipv6_property;
	int result = 0;
	enum ipa_client_type dst_pipe;

	ODU_IPA_LOG_ENTRY();

	dst_pipe = (odu_ipa_ctx->tx_fixup ||
		odu_ipa_ctx->hw_hdr_info.tx.is_little_endian) ?
		IPA_TO_SYS_CLIENT :
		IPA_TO_HSIC_CLIENT;

	tx_properties.prop = properties;
	ipv4_property = &tx_properties.prop[0];
	ipv4_property->ip = IPA_IP_v4;
	ipv4_property->dst_pipe = dst_pipe;
	strlcpy(ipv4_property->hdr_name, ODU_IPA_IPV4_HDR_NAME,
			IPA_RESOURCE_NAME_MAX);
	ipv6_property = &tx_properties.prop[1];
	ipv6_property->ip = IPA_IP_v6;
	ipv6_property->dst_pipe = dst_pipe;
	strlcpy(ipv6_property->hdr_name, ODU_IPA_IPV6_HDR_NAME,
			IPA_RESOURCE_NAME_MAX);
	tx_properties.num_props = 2;

	rx_properties.prop = rx_ioc_properties;
	rx_ipv4_property = &rx_properties.prop[0];
	rx_ipv4_property->ip = IPA_IP_v4;
	rx_ipv4_property->attrib.attrib_mask = 0;
	rx_ipv4_property->src_pipe = HSIC_TO_IPA_CLIENT;
	rx_ipv6_property = &rx_properties.prop[1];
	rx_ipv6_property->ip = IPA_IP_v6;
	rx_ipv6_property->attrib.attrib_mask = 0;
	rx_ipv6_property->src_pipe = HSIC_TO_IPA_CLIENT;
	rx_properties.num_props = 2;

	result = ipa_register_intf(odu_ipa_ctx->net->name, &tx_properties,
		&rx_properties);
	if (result)
		ODU_IPA_ERR("fail on Tx/Rx properties registration\n");

	ODU_IPA_LOG_EXIT();

	return result;
}

static void odu_ipa_deregister_properties(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result;
	ODU_IPA_LOG_ENTRY();
	result = ipa_deregister_intf(odu_ipa_ctx->net->name);
	if (result)
		ODU_IPA_DBG("Fail on Tx prop deregister\n");
	ODU_IPA_LOG_EXIT();
	return;
}

static void odu_ipa_rm_notify(void *user_data, enum ipa_rm_event event,
		unsigned long data)
{
	struct odu_ipa_dev *odu_ipa_ctx = user_data;
	ODU_IPA_LOG_ENTRY();
	if (event == IPA_RM_RESOURCE_GRANTED &&
			netif_queue_stopped(odu_ipa_ctx->net)) {
		ODU_IPA_DBG("Resource Granted - waking queue\n");
		netif_wake_queue(odu_ipa_ctx->net);
	} else {
		ODU_IPA_DBG("Resource released\n");
	}
	ODU_IPA_LOG_EXIT();
}

static int odu_ipa_create_rm_resource(struct odu_ipa_dev *odu_ipa_ctx)
{
	struct ipa_rm_create_params create_params = {0};
	int result;
	ODU_IPA_LOG_ENTRY();
	create_params.name = IPA_RM_RESOURCE_ODU_PROD;
	create_params.reg_params.user_data = odu_ipa_ctx;
	create_params.reg_params.notify_cb = odu_ipa_rm_notify;
	result = ipa_rm_create_resource(&create_params);
	if (result) {
		ODU_IPA_ERR("Fail on ipa_rm_create_resource\n");
		goto fail_rm_create;
	}
	ODU_IPA_DBG("rm client was created");

	result = ipa_rm_inactivity_timer_init(IPA_RM_RESOURCE_ODU_PROD,
			INACTIVITY_MSEC_DELAY);
	if (result) {
		ODU_IPA_ERR("Fail on ipa_rm_inactivity_timer_init\n");
		goto fail_it;
	}
	ODU_IPA_DBG("rm_it client was created");

	result = ipa_rm_add_dependency(IPA_RM_RESOURCE_ODU_PROD,
				IPA_RM_RESOURCE_HSIC_CONS);
	if (result)
		ODU_IPA_ERR("unable to add dependency (%d)\n", result);

	ODU_IPA_DBG("rm dependency was set\n");

	ODU_IPA_LOG_EXIT();
	return 0;

fail_it:
fail_rm_create:
	return result;
}

static void odu_ipa_destory_rm_resource(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result;

	ODU_IPA_LOG_ENTRY();

	ipa_rm_delete_dependency(IPA_RM_RESOURCE_ODU_PROD,
			IPA_RM_RESOURCE_HSIC_CONS);
	ipa_rm_inactivity_timer_destroy(IPA_RM_RESOURCE_ODU_PROD);
	result = ipa_rm_delete_resource(IPA_RM_RESOURCE_ODU_PROD);
	if (result)
		ODU_IPA_ERR("resource deletion failed\n");

	ODU_IPA_LOG_EXIT();
}

static bool rx_filter(struct sk_buff *skb)
{
	struct odu_ipa_dev *odu_ipa_ctx = netdev_priv(skb->dev);
	return !odu_ipa_ctx->rx_enable;
}

static bool tx_filter(struct sk_buff *skb)
{
	struct odu_ipa_dev *odu_ipa_ctx = netdev_priv(skb->dev);
	return !odu_ipa_ctx->tx_enable;
}

static bool rm_enabled(struct odu_ipa_dev *odu_ipa_ctx)
{
	return odu_ipa_ctx->rm_enable;
}

static int resource_request(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result = 0;

	if (!rm_enabled(odu_ipa_ctx))
		goto out;
	result = ipa_rm_inactivity_timer_request_resource(
			IPA_RM_RESOURCE_ODU_PROD);
out:
	return result;
}

static void resource_release(struct odu_ipa_dev *odu_ipa_ctx)
{
	if (!rm_enabled(odu_ipa_ctx))
		goto out;
	ipa_rm_inactivity_timer_release_resource(IPA_RM_RESOURCE_ODU_PROD);
out:
	return;
}

/**
 * odu_ipa_tx_complete_notify() - Rx notify
 *
 * @priv: odu driver context
 * @evt: event type
 * @data: data provided with event
 *
 * Check that the packet is the one we sent and release it
 * This function will be called in defered context in IPA wq.
 */
static void odu_ipa_tx_complete_notify(void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct odu_ipa_dev *odu_ipa_ctx = priv;

	if (!odu_ipa_ctx) {
		ODU_IPA_ERR("odu_ipa_ctx is NULL pointer\n");
		return;
	}
	if (evt != IPA_WRITE_DONE) {
		ODU_IPA_ERR("unsupported event on Tx callback\n");
		return;
	}
	atomic_dec(&odu_ipa_ctx->outstanding_pkts);
	if (netif_queue_stopped(odu_ipa_ctx->net) &&
		atomic_read(&odu_ipa_ctx->outstanding_pkts) <
					(odu_ipa_ctx->outstanding_low)) {
		ODU_IPA_DBG("Outstanding low (%d) - waking up queue\n",
				odu_ipa_ctx->outstanding_low);
		netif_wake_queue(odu_ipa_ctx->net);
	}

	dev_kfree_skb_any(skb);
	return;
}

/**
 * odu_ipa_sys_pipe_rx_cb() - Rx handler for SYS pipe setup in init API.
 *
 * @priv: odu driver context
 * @evt: event type
 * @data: data provided with event
 *
 * Assumes packet arrives with little endian header. swap lentgth in packet
 * and sent it back to ipa
 */
static void odu_ipa_sys_pipe_rx_cb(void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct odu_ipa_dev *odu_ipa_ctx = priv;
	u16 *pkt_len_ptr;
	int result;

	if (!odu_ipa_ctx) {
		ODU_IPA_ERR("odu_ipa_ctx is NULL pointer\n");
		return;
	}
	if (evt != IPA_RECEIVE) {
		ODU_IPA_ERR("unsupported event on Tx callback\n");
		return;
	}

	if (odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size_valid) {
		pkt_len_ptr = (u16 *)(skb->data +
			odu_ipa_ctx->hw_hdr_info.tx.hdr_ofst_pkt_size);
		*pkt_len_ptr = (((*pkt_len_ptr) & 0xFF00) >> 8) |
			(((*pkt_len_ptr) & 0x00FF) << 8);
	}

	if (odu_ipa_ctx->tx_fixup) {
		/* first remove header added by IPA */
		skb_pull(skb, odu_ipa_ctx->hw_hdr_info.tx.hdr_len);
		result = odu_ipa_ctx->tx_fixup(skb);
		if (result) {
			ODU_IPA_ERR("tx_fixup failed (%d)\n", result);
			dev_kfree_skb_any(skb);
			return;
		}
	}

	result = ipa_tx_dp(IPA_TO_HSIC_CLIENT, skb, NULL);
	if (result) {
		ODU_IPA_DBG("ipa_tx_dp failed %d\n", result);
		dev_kfree_skb_any(skb);
		return;
	}
	atomic_inc(&odu_ipa_ctx->outstanding_pkts);
	odu_ipa_ctx->endianess_swapped_pkts++;
}

static int odu_ipa_debugfs_atomic_open(struct inode *inode, struct file *file)
{
	struct odu_ipa_dev *odu_ipa_ctx = inode->i_private;
	ODU_IPA_LOG_ENTRY();
	file->private_data = &(odu_ipa_ctx->outstanding_pkts);
	ODU_IPA_LOG_EXIT();
	return 0;
}

static ssize_t odu_ipa_debugfs_atomic_read(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	u8 atomic_str[DEBUGFS_TEMP_BUF_SIZE] = {0};
	atomic_t *atomic_var = file->private_data;
	nbytes = scnprintf(atomic_str, sizeof(atomic_str), "%d\n",
			atomic_read(atomic_var));
	return simple_read_from_buffer(ubuf, count, ppos, atomic_str, nbytes);
}


static int odu_ipa_debugfs_init(struct odu_ipa_dev *odu_ipa_ctx)
{
	const mode_t flags_read_write = S_IRUGO | S_IWUGO;
	const mode_t flags_read_only = S_IRUGO;
	struct dentry *file;

	ODU_IPA_LOG_ENTRY();

	if (!odu_ipa_ctx)
		return -EINVAL;

	odu_ipa_ctx->directory = debugfs_create_dir("odu_ipa", NULL);
	if (!odu_ipa_ctx->directory) {
		ODU_IPA_ERR("could not create debugfs directory entry\n");
		goto fail_directory;
	}
	file = debugfs_create_bool("tx_enable", flags_read_write,
			odu_ipa_ctx->directory, &odu_ipa_ctx->tx_enable);
	if (!file) {
		ODU_IPA_ERR("could not create debugfs tx file\n");
		goto fail_file;
	}
	file = debugfs_create_bool("rx_enable", flags_read_write,
			odu_ipa_ctx->directory, &odu_ipa_ctx->rx_enable);
	if (!file) {
		ODU_IPA_ERR("could not create debugfs rx file\n");
		goto fail_file;
	}
	file = debugfs_create_bool("rm_enable", flags_read_write,
			odu_ipa_ctx->directory, &odu_ipa_ctx->rm_enable);
	if (!file) {
		ODU_IPA_ERR("could not create debugfs rm file\n");
		goto fail_file;
	}
	file = debugfs_create_u8("outstanding_high", flags_read_write,
			odu_ipa_ctx->directory, &odu_ipa_ctx->outstanding_high);
	if (!file) {
		ODU_IPA_ERR("could not create outstanding_high file\n");
		goto fail_file;
	}
	file = debugfs_create_u8("outstanding_low", flags_read_write,
			odu_ipa_ctx->directory, &odu_ipa_ctx->outstanding_low);
	if (!file) {
		ODU_IPA_ERR("could not create outstanding_high file\n");
		goto fail_file;
	}
	file = debugfs_create_u32("endianess_swapped_pkts", flags_read_write,
			odu_ipa_ctx->directory,
			&odu_ipa_ctx->endianess_swapped_pkts);
	if (!file) {
		ODU_IPA_ERR("could not create outstanding_low file\n");
		goto fail_file;
	}
	file = debugfs_create_file("outstanding", flags_read_only,
			odu_ipa_ctx->directory,
			odu_ipa_ctx, &odu_ipa_debugfs_atomic_ops);
	if (!file) {
		ODU_IPA_ERR("could not create outstanding file\n");
		goto fail_file;
	}

	ODU_IPA_LOG_EXIT();

	return 0;
fail_file:
	debugfs_remove_recursive(odu_ipa_ctx->directory);
fail_directory:
	return -EFAULT;
}

static void odu_ipa_debugfs_destroy(struct odu_ipa_dev *odu_ipa_ctx)
{
	debugfs_remove_recursive(odu_ipa_ctx->directory);
}

static bool odu_ipa_is_ofst_valid(bool valid, u32 ofst, u32 len)
{
	if (!valid)
		return true;
	if (ofst >= len)
		return false;
	return true;
}

/**
 * odu_ipa_ep_cfg() - configure the HSC endpoints
 *
 *hsic_to_ipa_hdl: handle received from ipa_connect
 *ipa_to_hsic_hdl: handle received from ipa_connect
 *
 * HSIC to IPA pipe:
 *  - No de-aggregation
 *  - Remove HW + Ethernet header
 *  - SRC NAT
 *  - Default routing(0)
 * IPA to HSIC Pipe:
 *  - No aggregation
 *  - Add HW + Ethernet header
 */
static int odu_ipa_ep_registers_cfg(struct odu_ipa_dev *odu_ipa_ctx)
{
	int result = 0;
	struct ipa_ep_cfg hsic_to_ipa_ep_cfg;
	struct ipa_ep_cfg ipa_to_hsic_ep_cfg;

	ODU_IPA_LOG_ENTRY();
	/* configure RX (HSIC->IPA) EP */
	memset(&hsic_to_ipa_ep_cfg, 0 , sizeof(struct ipa_ep_cfg));
	hsic_to_ipa_ep_cfg.aggr.aggr_en = IPA_BYPASS_AGGR;
	hsic_to_ipa_ep_cfg.hdr.hdr_len = odu_ipa_ctx->hw_hdr_info.rx.hdr_len +
		ETH_HLEN;
	hsic_to_ipa_ep_cfg.nat.nat_en = IPA_SRC_NAT;
	hsic_to_ipa_ep_cfg.mode.mode = IPA_BASIC;
	result = ipa_cfg_ep(odu_ipa_ctx->hsic_to_ipa_hdl, &hsic_to_ipa_ep_cfg);
	if (result) {
		ODU_IPA_ERR("failed to configure HSIC to IPA point\n");
		goto out;
	}
	/* configure TX (IPA->HSIC) EP */
	memset(&ipa_to_hsic_ep_cfg, 0, sizeof(struct ipa_ep_cfg));
	if (!odu_ipa_ctx->tx_fixup &&
	    !odu_ipa_ctx->hw_hdr_info.tx.is_little_endian) {
		ipa_to_hsic_ep_cfg.aggr.aggr_en = IPA_BYPASS_AGGR;
		ipa_to_hsic_ep_cfg.hdr.hdr_len =
			odu_ipa_ctx->hw_hdr_info.tx.hdr_len + ETH_HLEN;
		hsic_to_ipa_ep_cfg.hdr.hdr_additional_const_len = ETH_HLEN;
		ipa_to_hsic_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
	}
	result = ipa_cfg_ep(odu_ipa_ctx->ipa_to_hsic_hdl, &ipa_to_hsic_ep_cfg);
	if (result) {
		ODU_IPA_ERR("failed to configure IPA to HSIC end-point\n");
		goto out;
	}
	ODU_IPA_DBG("end-point registers successfully configured\n");
out:
	ODU_IPA_LOG_EXIT();
	return result;
}

/**
 * odu_ipa_set_device_ethernet_addr() - set device etherenet address
 * @dev_ethaddr: device etherenet address
 *
 * Returns 0 for success, negative otherwise
 */
static int odu_ipa_set_device_ethernet_addr(u8 *dev_ethaddr,
		u8 device_ethaddr[])
{
	if (!is_valid_ether_addr(device_ethaddr))
		return -EINVAL;
	memcpy(dev_ethaddr, device_ethaddr, ETH_ALEN);
	ODU_IPA_DBG("device ethernet address: %pM\n", dev_ethaddr);
	return 0;
}

/** odu_ipa_next_state - return the next state of the driver
 * @current_state: the current state of the driver
 * @operation: an enum which represent the operation being made on the driver
 *  by its API.
 *
 * This function implements the driver internal state machine.
 * Its decisions are based on the driver current state and the operation
 * being made.
 * In case the operation is invalid this state machine will return
 * the value ODU_IPA_INVALID to inform the caller for a forbidden sequence.
 */
static enum odu_ipa_state odu_ipa_next_state(enum odu_ipa_state current_state,
		enum odu_ipa_operation operation)
{
	int next_state = ODU_IPA_INVALID;

	switch (current_state) {
	case ODU_IPA_UNLOADED:
		if (operation == ODU_IPA_INITIALIZE)
			next_state = ODU_IPA_INITIALIZED;
		break;
	case ODU_IPA_INITIALIZED:
		if (operation == ODU_IPA_CONFIGURE)
			next_state = ODU_IPA_CONFIGURED;
		else if (operation == ODU_IPA_CLEANUP)
			next_state = ODU_IPA_UNLOADED;
		break;
	case ODU_IPA_CONFIGURED:
		if (operation == ODU_IPA_CONNECT)
			next_state = ODU_IPA_CONNECTED;
		else if (operation == ODU_IPA_OPEN)
			next_state = ODU_IPA_UP;
		else if (operation == ODU_IPA_CLEANUP)
			next_state = ODU_IPA_UNLOADED;
		break;
	case ODU_IPA_CONNECTED:
		if (operation == ODU_IPA_DISCONNECT)
			next_state = ODU_IPA_CONFIGURED;
		else if (operation == ODU_IPA_OPEN)
			next_state = ODU_IPA_CONNECTED_AND_UP;
		break;
	case ODU_IPA_UP:
		if (operation == ODU_IPA_STOP)
			next_state = ODU_IPA_CONFIGURED;
		else if (operation == ODU_IPA_CONNECT)
			next_state = ODU_IPA_CONNECTED_AND_UP;
		else if (operation == ODU_IPA_CLEANUP)
			next_state = ODU_IPA_UNLOADED;
		break;
	case ODU_IPA_CONNECTED_AND_UP:
		if (operation == ODU_IPA_STOP)
			next_state = ODU_IPA_CONNECTED;
		else if (operation == ODU_IPA_DISCONNECT)
			next_state = ODU_IPA_UP;
		break;
	default:
		ODU_IPA_ERR("State is not supported\n");
		break;
	}

	ODU_IPA_DBG("state transition ( %s -> %s )- %s\n",
			odu_ipa_state_string(current_state),
			odu_ipa_state_string(next_state) ,
			next_state == ODU_IPA_INVALID ?
					"Forbidden" : "Allowed");

	return next_state;
}

/**
 * odu_ipa_state_string - return the state string representation
 * @state: enum which describe the state
 */
static const char *odu_ipa_state_string(enum odu_ipa_state state)
{
	switch (state) {
	case ODU_IPA_UNLOADED:
		return "ODU_IPA_UNLOADED";
	case ODU_IPA_INITIALIZED:
		return "ODU_IPA_INITIALIZED";
	case ODU_IPA_CONFIGURED:
		return "ODU_IPA_CONFIGURED";
	case ODU_IPA_CONNECTED:
		return "ODU_IPA_CONNECTED";
	case ODU_IPA_UP:
		return "ODU_IPA_UP";
	case ODU_IPA_CONNECTED_AND_UP:
		return "ODU_IPA_CONNECTED_AND_UP";
	default:
		return "Not supported";
	}
}

/**
 * odu_ipa_init_module() - module initialization
 *
 */
static int odu_ipa_init_module(void)
{
	ODU_IPA_LOG_ENTRY();
	ODU_IPA_LOG_EXIT();
	return 0;
}

/**
 * odu_ipa_cleanup_module() - module cleanup
 *
 */
static void odu_ipa_cleanup_module(void)
{
	ODU_IPA_LOG_ENTRY();
	ODU_IPA_LOG_EXIT();
	return;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ODU IPA network interface");

late_initcall(odu_ipa_init_module);
module_exit(odu_ipa_cleanup_module);
