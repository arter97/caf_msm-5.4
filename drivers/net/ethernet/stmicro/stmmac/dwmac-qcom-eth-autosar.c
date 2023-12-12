// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_clock.h>
#include "dwmac-qcom-ethqos.h"

// Define the structure
struct cnf_callback_structure {
	void (*g_txcnfcallback)(struct sk_buff *skb, u8 result);
	void (*g_rxcnfcallback)(struct sk_buff *skb);
};

// Create an instance of the structure
struct cnf_callback_structure cnf_callback_structure_t = {
	.g_txcnfcallback = NULL,
	.g_rxcnfcallback = NULL
};

/**
 *  EthWrapper_RegisterTxCnfCallback - Resgiter the callback
 *  @callback: function pointer.
 *  Description:
 *  Function to register the callback for Tx complete in the ETH wrapper API
 */
void ethwrapper_registertxcnfcallback(void(*callback) (struct sk_buff *skb, u8 result))
{
	cnf_callback_structure_t.g_txcnfcallback = callback;
}
EXPORT_SYMBOL(ethwrapper_registertxcnfcallback);

/**
 *  EthWrapper_HandleTxCompletion - Handle TX completion
 *  @skb: Pointer to the sk_buff structure
 *  @result: Result of the transmission
 *
 *  Description:
 *  This function handles the completion of a transmit operation. It invokes the registered
 *  callback function, if available, to notify the caller about the transmission result.
 */
void ethwrapper_handletxcompletion(struct sk_buff *skb, u8 result)
{
	if (cnf_callback_structure_t.g_txcnfcallback) {
		// Invoke the callback function if its registered
		cnf_callback_structure_t.g_txcnfcallback(skb, result);
	}
}
EXPORT_SYMBOL(ethwrapper_handletxcompletion);

/**
 *  EthWrapper_RegisterRxCnfCallback - Resgiter the callback
 *  @callback: function pointer.
 *  Description:
 *  Function to register the callback for Rx indication in the ETH wrapper API
 */
void ethwrapper_registerrxcnfcallback(void (*callback)(struct sk_buff *skb))
{
	cnf_callback_structure_t.g_rxcnfcallback = callback;
}
EXPORT_SYMBOL(ethwrapper_registerrxcnfcallback);

/**
 *  EthWrapper_HandleRICompletion - Handle RX indication completion
 *  @skb: Pointer to the sk_buff structure
 *
 *  Description:
 *  This function handles the completion of a receive indication. It invokes the registered
 *  callback function, if available, to notify the caller about the received packet.
 */
void ethwrapper_handlericompletion(struct sk_buff *skb)
{
	if (cnf_callback_structure_t.g_rxcnfcallback) {
		// Invoke the callback function if its registered
		cnf_callback_structure_t.g_rxcnfcallback(skb);
	}
}
EXPORT_SYMBOL(ethwrapper_handlericompletion);

/**
 *  EthDrv_allocateTxBuffer - Allocate transmit buffer
 *  @size: Size of the buffer to allocate
 *  Description-API to allocate Tx buffer.
 */
struct sk_buff *ethdrv_allocatetxbuffer(size_t size)
{
	struct sk_buff *skb;
	struct net_device *eth0_dev = dev_get_by_name(&init_net, "eth0");

	if (!eth0_dev)
		pr_err("\n eth0_dev is invalid\n");
	skb = netdev_alloc_skb(eth0_dev, size);
	if (!skb) {
		pr_err("\n alloc_skb is failed\n");
		return -EINVAL;
	}
	skb->len = size;
	skb->data = skb->head;
	eth0_dev->netdev_ops->ndo_select_queue(eth0_dev, skb, eth0_dev);
	if (skb->len == 0)
		return -EINVAL;

	if (!skb->data)
		return -EINVAL;
	return skb;
}
EXPORT_SYMBOL(ethdrv_allocatetxbuffer);

/**
 *  EthDrv_Transmit - Transmit the packet
 *  @skb: Pointer to the sk_buff structure
 *
 *  Description-
 *  API to transmit the packet. For the ethernet driver to transmit,
 *  the LG HAL layer needs to copy the Tx packet into SKB
 */
int ethdrv_transmit(struct sk_buff *skb)
{
	unsigned int ret = -1;
	struct net_device *eth0_dev = dev_get_by_name(&init_net, "eth0");

	if (!eth0_dev)
		pr_err("\n eth0_dev is invalid\n");

	ret = eth0_dev->netdev_ops->ndo_start_xmit(skb, eth0_dev);
	return ret;
}
EXPORT_SYMBOL(ethdrv_transmit);

/**
 *  EthDrv_GetCurSysTime - Get the current system time.
 *  @ts: Pointer to a struct timespec64 to store the current system time.
 *
 *  Description-
 *  API to get the current hardware time from Ethernet controller(ptp system time)
 */
void ethdrv_getcursystime(struct timespec64 *ts)
{
	qcom_ethqos_getcursystime(ts);
}
EXPORT_SYMBOL(ethdrv_getcursystime);

/**
 *  EthDrv_EnableTS - Enable hardware timestamping.
 *  @config: Pointer to a struct hwtstamp_config containing the configuration for
 *  hardware timestamping.
 *
 *  Description:
 *  This function enables hardware timestamping by calling the EthDrv_EnableTS_Wrapper function
 *  with the provided configuration.
 */
void ethdrv_enablets(struct hwtstamp_config *config)
{
	int ret = qcom_ethqos_enable_hw_timestamp(config);

	if (ret)
		pr_err("\nKERN_INFO HW TS enabled\n");
}
EXPORT_SYMBOL(ethdrv_enablets);
