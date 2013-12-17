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

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/if_ether.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msm_ipa.h>
#include <linux/mutex.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/ipv6.h>
#include <net/addrconf.h>
#include <mach/bam_dmux.h>
#include <mach/ipa.h>
#include <mach/sps.h>
#include "ipa_i.h"

#define ODU_BRIDGE_DRV_NAME "odu_ipa_bridge"

#define ODU_BRIDGE_DBG(fmt, args...) \
	pr_debug(ODU_BRIDGE_DRV_NAME " %s:%d " fmt, \
		 __func__, __LINE__, ## args)
#define ODU_BRIDGE_ERR(fmt, args...) \
	pr_err(ODU_BRIDGE_DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)
#define ODU_BRIDGE_FUNC_ENTRY() \
	ODU_BRIDGE_DBG("ENTRY\n")
#define ODU_BRIDGE_FUNC_EXIT() \
	ODU_BRIDGE_DBG("EXIT\n")


#define ODU_BRIDGE_IS_QMI_ADDR(daddr) \
	(memcmp(&(daddr), &odu_bridge_ctx->llv6_addr.addr, sizeof((daddr))) == 0)

#define HSIC_ETH_HDR_NAME_IPV4 "odu_br_hsic_ipv4"
#define HSIC_ETH_HDR_NAME_IPV6 "odu_br_hsic_ipv6"
#define A2_ETH_HDR_NAME_IPV4  "odu_br_a2_ipv4"
#define A2_ETH_HDR_NAME_IPV6  "odu_br_a2_ipv6"

#define HSIC_TO_A2_RT_TBL_NAME_IPV4 "odu_br_hsic_a2_rt4"
#define A2_TO_HSIC_RT_TBL_NAME_IPV4 "odu_br_a2_hsic_rt4"
#define HSIC_TO_A2_RT_TBL_NAME_IPV6 "odu_br_hsic_a2_rt6"
#define A2_TO_HSIC_RT_TBL_NAME_IPV6 "odu_br_a2_hsic_rt6"

#define ODU_INACTIVITY_TIME_MSEC (1000)

#define ODU_WORKQUEUE_NAME "odu_bridge_wq"

#define ODU_TOTAL_HDR_ENTRIES 4
#define ODU_TOTAL_RT_ENTRIES_IP 2
#define ODU_TOTAL_FLT_ENTRIES_IP 4

/**
 * struct mac_addresses_type - store host and device MAC addresses
 * @host_mac_addr: MAC address of the host (GW)
 * @host_mac_addr_known: is the MAC address of the host known ?
 * @device_mac_addr: MAC address of the device
 * @device_mac_addr_known: is the MAC address of the device known ?
 */
struct mac_addresses_type {
	u8 host_mac_addr[ETH_ALEN];
	bool host_mac_addr_known;
	u8 device_mac_addr[ETH_ALEN];
	bool device_mac_addr_known;
};

/**
 * struct link_local_ipv6_addr_type - store link local ipv6 address
 * @addr: link local ipv6 address
 * @addr_known: is the link local ipv6 address known ?
 */
struct link_local_ipv6_addr_type {
	struct in6_addr addr;
	bool addr_known;
};

/**
 * struct stats - driver statistics, viewable using debugfs
 * @num_ul_packets: number of packets bridged in uplink direction using the SW
 * bridge
 * @num_dl_packets: number of packets bridged in downink direction using the SW
 * bridge
 * @num_packets_during_resource_wakeup: number of packets bridged during a
 * resource wakeup period, there is a special treatment for these kind of
 * packets
 */
struct stats {
	u64 num_ul_packets;
	u64 num_dl_packets;
	u64 num_packets_during_resource_wakeup;
};

/**
 * struct odu_bridge_ctx - Tethering bridge driver context information
 * @class: kernel class pointer
 * @dev_num: kernel device number
 * @dev: kernel device struct pointer
 * @cdev: kernel character device struct
 * @a2_ipa_pipe_hdl: A2 to IPA pipe handle
 * @ipa_a2_pipe_hdl: IPA to A2 pipe handle
 * @handle_ul_skb_cb: callback to send skb in ullink direction to linux kernel
 * @send_dl_skb_cb: callback to send skb in downlink direction to HSIC
 * @cb_priv: opaque parameter for send skb ul/dl callbacks.
 * @hw_hdr_info: hardware header information for downlink direction
 * @mac_addresses: Struct which holds GW and device MAC addresses
 * @is_bridge_prod_up: completion object signaled when the bridge producer
 * finished its resource request procedure
 * @is_bridge_prod_down: completion object signaled when the bridge producer
 * finished its resource release procedure
 * @stats: statistics, how many packets were transmitted using the SW bridge
 * @wq: dedicated workqueue, used for setting up the HW bridge and for
 * sending packets using the SW bridge when the system is waking up from power
 * collapse
 * @hdr_del: array to store the headers handles in order to delete them later
 * @init_status: bridge initialization and connection status
 * @mode: working mode Router/Bridge
 * @lock: mutex for syncrhonization between WQ/driver/IOCTL contexts
 * @request_resource_mutex: for the odu_bridge_request_resource_sync
 * synchronization
 * @is_hw_bridge_complete: is HW bridge setup ?
 * @ch_info: array of logic_ch_info, used to hold channel information
 * @logic_ch_num: the total logical channels number
 * @ch_init_cnt: count the initialized channels
 * @is_conencted: is bridge connected ?
 * @mutex: for the initialization, connect and disconnect synchronization
 * @is_hw_bridge_complete: is HW bridge setup ?
 * @comp_hw_bridge_work: work item for completing HW bridge
 * @comp_hw_bridge_in_progress: true when the HW bridge setup is in progress
 * @llv6_addr: link local IPv6 address of ODU network interface
 * @hdr_del: array to store the headers handles in order to delete them later
 * @routing_del: array of routing rules handles, one array for IPv4 and one for
 * IPv6
 * @filtering_del: array of routing rules handles, one array for IPv4 and one
 * for IPv6
 */
struct odu_bridge_ctx {
	struct class *class;
	dev_t dev_num;
	struct device *dev;
	struct cdev cdev;
	u32 a2_ipa_pipe_hdl;
	u32 ipa_a2_pipe_hdl;
	odu_bridge_send_dl_skb_cb send_dl_skb_cb;
	ipa_notify_cb handle_ul_skb_cb;
	void *cb_priv;
	struct odu_ipa_hw_hdr_info hw_hdr_info;
	struct mac_addresses_type mac_addresses;
	struct completion is_bridge_prod_up;
	struct completion is_bridge_prod_down;
	struct stats stats;
	struct workqueue_struct *wq;
	bool is_connected;
	enum odu_bridge_mode mode;
	struct mutex lock;
	struct mutex request_resource_mutex;
	bool is_hw_bridge_complete;
	struct work_struct comp_hw_bridge_work;
	bool comp_hw_bridge_in_progress;
	struct link_local_ipv6_addr_type llv6_addr;
	struct ipa_ioc_del_hdr *hdr_del;
	struct ipa_ioc_del_rt_rule *routing_del[IPA_IP_MAX];
	struct ipa_ioc_del_flt_rule *filtering_del[IPA_IP_MAX];
};
static struct odu_bridge_ctx *odu_bridge_ctx;

enum odu_packet_direction {
	ODU_DIR_UL,
	ODU_DIR_DL,
};

/**
 * struct odu_work - wrapper for an skb which is sent using a workqueue
 * @work: used by the workqueue
 * @skb: pointer to the skb to be sent
 * @dir: direction of send, A2 to USB or USB to A2
 */
struct odu_work {
	struct work_struct work;
	struct sk_buff *skb;
	enum odu_packet_direction dir;
};


#ifdef CONFIG_DEBUG_FS
#define ODU_MAX_MSG_LEN 512
static char dbg_buff[ODU_MAX_MSG_LEN];
#endif

/**
 * add_eth_hdrs_internal() - add Ethernet headers to IPA
 * @hdr_name_ipv4: header name for IPv4
 * @hdr_name_ipv6: header name for IPv6
 * @src_mac_addr: source MAC address
 * @dst_mac_addr: destination MAC address
 * @add_hw_hdr: add hardware header on top of ethernet header ?
 */

static int add_eth_hdrs_internal(char *hdr_name_ipv4, char *hdr_name_ipv6,
			u8 *src_mac_addr, u8 *dst_mac_addr, bool add_hw_hdr)
{
	int res;
	struct ipa_ioc_add_hdr *hdrs;
	struct ethhdr hdr_ipv4;
	struct ethhdr hdr_ipv6;
	u32 eth_ofst = 0;
	int idx1;

	ODU_BRIDGE_FUNC_ENTRY();
	memcpy(hdr_ipv4.h_source, src_mac_addr, ETH_ALEN);
	memcpy(hdr_ipv4.h_dest, dst_mac_addr, ETH_ALEN);
	hdr_ipv4.h_proto = htons(ETH_P_IP);

	memcpy(hdr_ipv6.h_source, src_mac_addr, ETH_ALEN);
	memcpy(hdr_ipv6.h_dest, dst_mac_addr, ETH_ALEN);
	hdr_ipv6.h_proto = htons(ETH_P_IPV6);

	/* Add headers to the header insertion tables */
	hdrs = kzalloc(sizeof(struct ipa_ioc_add_hdr) +
		       2 * sizeof(struct ipa_hdr_add), GFP_KERNEL);
	if (hdrs == NULL) {
		ODU_BRIDGE_ERR("Failed allocating memory for headers !\n");
		return -ENOMEM;
	}

	hdrs->commit = 0;
	hdrs->num_hdrs = 2;

	/* Ethernet IPv4 header */
	strlcpy(hdrs->hdr[0].name, hdr_name_ipv4, IPA_RESOURCE_NAME_MAX);
	if (add_hw_hdr) {
		memcpy(hdrs->hdr[0].hdr,
			&odu_bridge_ctx->hw_hdr_info.tx.raw_hdr,
			odu_bridge_ctx->hw_hdr_info.tx.hdr_len);
		eth_ofst = odu_bridge_ctx->hw_hdr_info.tx.hdr_len;
	}
	memcpy(hdrs->hdr[0].hdr + eth_ofst, &hdr_ipv4, ETH_HLEN);
	hdrs->hdr[0].hdr_len = eth_ofst + ETH_HLEN;

	/* Ethernet IPv6 header */
	strlcpy(hdrs->hdr[1].name, hdr_name_ipv6, IPA_RESOURCE_NAME_MAX);
	if (add_hw_hdr) {
		memcpy(hdrs->hdr[1].hdr,
			&odu_bridge_ctx->hw_hdr_info.tx.raw_hdr,
			odu_bridge_ctx->hw_hdr_info.tx.hdr_len);
		eth_ofst = odu_bridge_ctx->hw_hdr_info.tx.hdr_len;
	}
	memcpy(hdrs->hdr[1].hdr + eth_ofst, &hdr_ipv6, ETH_HLEN);
	hdrs->hdr[1].hdr_len = eth_ofst + ETH_HLEN;

	res = ipa_add_hdr(hdrs);
	if (res || hdrs->hdr[0].status || hdrs->hdr[1].status)
		ODU_BRIDGE_ERR("Header insertion failed\n");

	/* Save the headers handles in order to delete them later */
	for (idx1 = 0; idx1 < hdrs->num_hdrs; idx1++) {
		int idx2 = odu_bridge_ctx->hdr_del->num_hdls++;
		odu_bridge_ctx->hdr_del->hdl[idx2].hdl =
			hdrs->hdr[idx1].hdr_hdl;
	}

	kfree(hdrs);
	ODU_BRIDGE_FUNC_EXIT();

	return res;
}

/**
 * add_eth_hdrs() - add Ethernet headers to IPA
 */
static int add_eth_hdrs(void)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	/* Add a header entry for HSIC */
	res = add_eth_hdrs_internal(HSIC_ETH_HDR_NAME_IPV4,
				HSIC_ETH_HDR_NAME_IPV6,
				odu_bridge_ctx->mac_addresses.device_mac_addr,
				odu_bridge_ctx->mac_addresses.host_mac_addr,
				true);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding HSIC Ethernet header\n");
		goto bail;
	}
	ODU_BRIDGE_DBG("Added HSIC Ethernet headers (IPv4 / IPv6)\n");

	/* Add a header entry for A2 */
	res = add_eth_hdrs_internal(A2_ETH_HDR_NAME_IPV4,
				A2_ETH_HDR_NAME_IPV6,
				odu_bridge_ctx->mac_addresses.host_mac_addr,
				odu_bridge_ctx->mac_addresses.device_mac_addr,
				false);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding A2 Ethernet header\n");
		goto bail;
	}
	ODU_BRIDGE_DBG("Added A2 Ethernet headers (IPv4 / IPv6\n");
bail:
	ODU_BRIDGE_FUNC_EXIT();
	return res;
}

/**
 * configure_ipa_header_block() - adds headers and configures endpoint registers
 */
static int configure_ipa_header_block(void)
{
	int res;
	struct ipa_ep_cfg ipa_ep_cfg;

	ODU_BRIDGE_FUNC_ENTRY();

	res = add_eth_hdrs();
	if (res) {
		ODU_BRIDGE_ERR("Failed adding Ethernet header\n");
		return res;
	}

	/* Configure A2 endpoints */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.hdr.hdr_len = ETH_HLEN;
	ipa_cfg_ep(odu_bridge_ctx->ipa_a2_pipe_hdl, &ipa_ep_cfg);
	ipa_cfg_ep(odu_bridge_ctx->a2_ipa_pipe_hdl, &ipa_ep_cfg);

	ODU_BRIDGE_FUNC_EXIT();
	return 0;
}

static int configure_routing_by_ip(char *hdr_name,
			    char *rt_tbl_name,
			    enum ipa_client_type dst,
			    enum ipa_ip_type ip_address_family)
{

	struct ipa_ioc_add_rt_rule *rt_rule;
	struct ipa_ioc_get_hdr hdr_info;
	int res;
	int idx;

	ODU_BRIDGE_FUNC_ENTRY();

	/* Get the header handle */
	memset(&hdr_info, 0, sizeof(hdr_info));
	strlcpy(hdr_info.name, hdr_name, IPA_RESOURCE_NAME_MAX);
	ipa_get_hdr(&hdr_info);

	rt_rule = kzalloc(sizeof(struct ipa_ioc_add_rt_rule) +
			  1 * sizeof(struct ipa_rt_rule_add),
			  GFP_KERNEL);
	if (!rt_rule) {
		ODU_BRIDGE_ERR("Memory allocation failure");
		return -ENOMEM;
	}

	/* Match all, do not commit to HW*/
	rt_rule->commit = 0;
	rt_rule->num_rules = 1;
	rt_rule->ip = ip_address_family;
	strlcpy(rt_rule->rt_tbl_name, rt_tbl_name, IPA_RESOURCE_NAME_MAX);
	rt_rule->rules[0].rule.dst = dst;
	rt_rule->rules[0].rule.hdr_hdl = hdr_info.hdl;
	rt_rule->rules[0].rule.attrib.attrib_mask = 0; /* Match all */
	res = ipa_add_rt_rule(rt_rule);
	if (res || rt_rule->rules[0].status)
		ODU_BRIDGE_ERR("Failed adding routing rule\n");

	/* Save the routing rule handle in order to delete it later */
	idx = odu_bridge_ctx->routing_del[ip_address_family]->num_hdls++;
	odu_bridge_ctx->routing_del[ip_address_family]->hdl[idx].hdl =
		rt_rule->rules[0].rt_rule_hdl;

	kfree(rt_rule);
	ODU_BRIDGE_FUNC_EXIT();

	return res;
}

static int configure_routing(char *hdr_name_ipv4,
			     char *rt_tbl_name_ipv4,
			     char *hdr_name_ipv6,
			     char *rt_tbl_name_ipv6,
			     enum ipa_client_type dst)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();
	/* Configure IPv4 routing table */
	res = configure_routing_by_ip(hdr_name_ipv4,
				      rt_tbl_name_ipv4,
				      dst,
				      IPA_IP_v4);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding IPv4 routing table\n");
		goto bail;
	}

	/* Configure IPv6 routing table */
	res = configure_routing_by_ip(hdr_name_ipv6,
				      rt_tbl_name_ipv6,
				      dst,
				      IPA_IP_v6);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding IPv6 routing table\n");
		goto bail;
	}
	ODU_BRIDGE_FUNC_EXIT();

bail:
	return res;
}

/**
 * configure_ipa_routing_block() - Configure the IPA routing block
 */
static int configure_ipa_routing_block(void)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	/* Configure HSIC -> A2 routing table */
	res = configure_routing(A2_ETH_HDR_NAME_IPV4,
				HSIC_TO_A2_RT_TBL_NAME_IPV4,
				A2_ETH_HDR_NAME_IPV6,
				HSIC_TO_A2_RT_TBL_NAME_IPV6,
				IPA_CLIENT_A2_TETHERED_CONS);
	if (res) {
		ODU_BRIDGE_ERR("HSIC to A2 routing block configure failed\n");
		return res;
	}

	/* Configure A2 -> USB routing table */
	res = configure_routing(HSIC_ETH_HDR_NAME_IPV4,
				A2_TO_HSIC_RT_TBL_NAME_IPV4,
				HSIC_ETH_HDR_NAME_IPV6,
				A2_TO_HSIC_RT_TBL_NAME_IPV6,
				IPA_CLIENT_HSIC2_CONS);
	if (res) {
		ODU_BRIDGE_ERR("A2 to HSIC routing block configure failed\n");
		return res;
	}

	ODU_BRIDGE_FUNC_EXIT();
	return 0;
}

/**
 * configure_filtering_by_ip() - Configures IPA filtering block for
 * address family: IPv4 or IPv6
 * @rt_tbl_name: routing table name
 * @src: which "clients" pipe does this rule apply to
 * @family: address family: IPv4 or IPv6
 * @dir: UL/DL direction
 */
static int configure_filtering_by_ip(char *rt_tbl_name,
			      enum ipa_client_type src,
			      enum ipa_ip_type family,
			      enum odu_packet_direction dir)
{
	struct ipa_ioc_add_flt_rule *flt_tbl;
	struct ipa_ioc_get_rt_tbl rt_tbl_info;
	int res;
	int idx;
	int idx2;
	int num_rules = 1;

	ODU_BRIDGE_FUNC_ENTRY();
	ODU_BRIDGE_DBG(
		"configure filter: routing table: %s src ep(client type):%d\n",
		rt_tbl_name, src);

	/* Get the needed routing table handle */
	rt_tbl_info.ip = family;
	strlcpy(rt_tbl_info.name, rt_tbl_name, IPA_RESOURCE_NAME_MAX);
	res = ipa_get_rt_tbl(&rt_tbl_info);
	if (res) {
		ODU_BRIDGE_ERR("Failed getting routing table handle\n");
		goto bail;
	}

	/* for UL ipv6 3 rules are needed */
	if (family == IPA_IP_v6 && dir == ODU_DIR_UL)
		num_rules = 3;

	flt_tbl = kzalloc(sizeof(struct ipa_ioc_add_flt_rule) +
			  num_rules * sizeof(struct ipa_flt_rule_add),
			  GFP_KERNEL);
	if (!flt_tbl) {
		ODU_BRIDGE_ERR("Filtering table memory allocation failure\n");
		return -ENOMEM;
	}

	flt_tbl->commit = 0;
	flt_tbl->ep = src;
	flt_tbl->global = 0;
	flt_tbl->ip = family;
	flt_tbl->num_rules = num_rules;
	idx = 0;

	if (family == IPA_IP_v6 && dir == ODU_DIR_UL) {
		/* all multicast packets to exception */
		flt_tbl->rules[idx].rule.action = IPA_PASS_TO_EXCEPTION;
		flt_tbl->rules[idx].rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr[0] = 0xff000000;
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr_mask[0] =
			0xff000000;
		flt_tbl->rules[idx].at_rear = true;
		idx++;

		/* all LLv6 packets to exception */
		flt_tbl->rules[idx].rule.action = IPA_PASS_TO_EXCEPTION;
		flt_tbl->rules[idx].rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr[0] =
			htonl(odu_bridge_ctx->llv6_addr.addr.s6_addr32[0]);
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr[1] =
			htonl(odu_bridge_ctx->llv6_addr.addr.s6_addr32[1]);
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr[2] =
			htonl(odu_bridge_ctx->llv6_addr.addr.s6_addr32[2]);
		flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr[3] =
			htonl(odu_bridge_ctx->llv6_addr.addr.s6_addr32[3]);
		memset(flt_tbl->rules[idx].rule.attrib.u.v6.dst_addr_mask,
			0xFF,
			sizeof(flt_tbl->
				rules[idx].rule.attrib.u.v6.dst_addr_mask));
		flt_tbl->rules[idx].at_rear = true;
		idx++;
	}

	/* all packets to routing block */
	flt_tbl->rules[idx].rule.action = IPA_PASS_TO_ROUTING;
	flt_tbl->rules[idx].rule.rt_tbl_hdl = rt_tbl_info.hdl;
	flt_tbl->rules[idx].rule.attrib.attrib_mask = 0; /* Match all */
	flt_tbl->rules[idx].at_rear = true;

	res = ipa_add_flt_rule(flt_tbl);
	if (res || flt_tbl->rules[0].status)
		ODU_BRIDGE_ERR("Failed adding filtering table\n");

	/* Save the filtering rule handle in order to delete it later */
	for (idx2 = 0; idx2 < num_rules; idx2++) {
		idx = odu_bridge_ctx->filtering_del[family]->num_hdls++;
		odu_bridge_ctx->filtering_del[family]->hdl[idx].hdl =
			flt_tbl->rules[idx2].flt_rule_hdl;
	}

	kfree(flt_tbl);
	ODU_BRIDGE_FUNC_EXIT();

bail:
	return res;
}

/**
 * configure_filtering() - Configures IPA filtering block
 * @rt_tbl_name_ipv4: IPv4 routing table name
 * @rt_tbl_name_ipv6: IPv6 routing table name
 * @src: which "clients" pipe does this rule apply to
 * @dir: UL/DL direction
 */
static int configure_filtering(char *rt_tbl_name_ipv4,
			char *rt_tbl_name_ipv6,
			enum ipa_client_type src,
			enum odu_packet_direction dir)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();
	res = configure_filtering_by_ip(rt_tbl_name_ipv4,
					src,
					IPA_IP_v4,
					dir);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding IPv4 filtering table\n");
		return res;
	}

	res = configure_filtering_by_ip(rt_tbl_name_ipv6,
					src,
					IPA_IP_v6,
					dir);
	if (res) {
		ODU_BRIDGE_ERR("Failed adding IPv6 filtering table\n");
		return res;
	}

	ODU_BRIDGE_FUNC_EXIT();

	return 0;
}

/**
 * configure_ipa_filtering_block() - Configures IPA filtering block
 * This function configures IPA for:
 * - Filter all ipv6 multicast packets from HSIC to exception
 * - Filter all link local ipv6 packets from HSIC which are designated to A5
 *   to exception
 * - Filter all other packets from HSIC to A2
 * - Filter all packets from A2 to HSIC
 */
static int configure_ipa_filtering_block(void)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	/* Filter traffic coming from HSIC to A2 */
	res = configure_filtering(HSIC_TO_A2_RT_TBL_NAME_IPV4,
				  HSIC_TO_A2_RT_TBL_NAME_IPV6,
				  IPA_CLIENT_HSIC1_PROD,
				  ODU_DIR_UL);
	if (res) {
		ODU_BRIDGE_ERR("UL filtering configuration failed\n");
		goto bail;
	}

	/* Filter traffic coming from A2 to HSIC */
	res = configure_filtering(A2_TO_HSIC_RT_TBL_NAME_IPV4,
				  A2_TO_HSIC_RT_TBL_NAME_IPV6,
				  IPA_CLIENT_A2_TETHERED_PROD,
				  ODU_DIR_DL);
	if (res) {
		ODU_BRIDGE_ERR("DL filtering configuration failed\n");
		goto bail;
	}

	ODU_BRIDGE_FUNC_EXIT();
bail:
	return res;
}

/**
 * odu_bridge_request_resource() - wrapper function to
 * ipa_rm_inactivity_timer_request_resource()
 */
static inline int odu_bridge_request_resource(void)
{
	return ipa_rm_inactivity_timer_request_resource(
					IPA_RM_RESOURCE_ODU_BRIDGE_PROD);
}

/**
 * odu_bridge_release_resource() - wrapper function to
 * ipa_rm_inactivity_timer_release_resource()
 */
static inline void odu_bridge_release_resource(void)
{
	ipa_rm_inactivity_timer_release_resource(
					IPA_RM_RESOURCE_ODU_BRIDGE_PROD);
}

/**
 * odu_bridge_request_resource_sync() - call to request resource and block
 * until resource is granted
 * ipa_rm_inactivity_timer_request_resource()
 *
 * - initialize the is_bridge_prod_up completion object
 * - request the resource
 * - error handling
 */
static int odu_bridge_request_resource_sync(void)
{
	int res;

	mutex_lock(&odu_bridge_ctx->request_resource_mutex);
	INIT_COMPLETION(odu_bridge_ctx->is_bridge_prod_up);
	res = odu_bridge_request_resource();
	if (res < 0) {
		if (res == -EINPROGRESS) {
			wait_for_completion(&odu_bridge_ctx->is_bridge_prod_up);
			res = 0;
		}
	} else {
		res = 0;
	}
	mutex_unlock(&odu_bridge_ctx->request_resource_mutex);
	return res;
}



/**
 * complete_hw_bridge() - setup the HW bridge between HSIC and A2
 */
static void complete_hw_bridge(struct work_struct *work)
{

	int res;

	ODU_BRIDGE_FUNC_ENTRY();
	ODU_BRIDGE_DBG("Host MAC: %pM\n",
		odu_bridge_ctx->mac_addresses.host_mac_addr);
	ODU_BRIDGE_DBG("device MAC: %pM\n",
		odu_bridge_ctx->mac_addresses.device_mac_addr);
	ODU_BRIDGE_DBG("LLV6 addr: %pI6c\n", &odu_bridge_ctx->llv6_addr.addr);

	res = odu_bridge_request_resource_sync();
	if (res) {
		ODU_BRIDGE_ERR("request_resource() failed.\n");
		goto bail;
	}

	res = configure_ipa_header_block();
	if (res) {
		ODU_BRIDGE_ERR("Configuration of IPA header block Failed\n");
		goto bail;
	}

	res = configure_ipa_routing_block();
	if (res) {
		ODU_BRIDGE_ERR("Configuration of IPA routing block Failed\n");
		goto bail;
	}

	res = configure_ipa_filtering_block();
	if (res) {
		ODU_BRIDGE_ERR("IPA filtering configuration block Failed\n");
		goto bail;
	}

	/*
	 * Commit all the data to HW, including header, routing and filtering
	 * blocks, IPv4 and IPv6
	 */
	res = ipa_commit_hdr();
	if (res) {
		ODU_BRIDGE_ERR("Failed committing headers\n");
		goto bail;
	}

	odu_bridge_ctx->is_hw_bridge_complete = true;

bail:
	odu_bridge_ctx->comp_hw_bridge_in_progress = false;
	odu_bridge_release_resource();
	ODU_BRIDGE_FUNC_EXIT();

	return;
}


/**
 * check_to_complete_hw_bridge() - can HW bridge be set up ?
 * @skb: pointer to socket buffer
 * @my_mac_addr: pointer to write 'my' extracted MAC address
 * @my_mac_addr_known: pointer to update whether 'my' extracted MAC
 * address is known
 * @peer_mac_addr_known: pointer to update whether the 'peer' extracted
 * MAC address is known
 *
 * This function is used by both A2 and uplink callback functions, therefore
 * the meaning of 'my' and 'peer' changes according to the context.
 * Extracts MAC address from the packet in Ethernet link protocol,
 * Sets up the HW bridge in case all conditions are met.
 */
static void check_to_complete_hw_bridge(struct sk_buff *skb,
					u8 *my_mac_addr,
					bool *my_mac_addr_known,
					bool *peer_mac_addr_known)
{
	bool both_mac_addresses_known;

	if (!(*my_mac_addr_known)) {
		memcpy(my_mac_addr, &skb->data[ETH_ALEN], ETH_ALEN);
		ODU_BRIDGE_DBG("Extracted MAC addr: %pM\n", my_mac_addr);
		*my_mac_addr_known = true;
	}

	both_mac_addresses_known = *my_mac_addr_known && *peer_mac_addr_known;
	if (both_mac_addresses_known && odu_bridge_ctx->llv6_addr.addr_known &&
				!odu_bridge_ctx->comp_hw_bridge_in_progress) {
		ODU_BRIDGE_DBG("completeing HW bridge\n");
		odu_bridge_ctx->comp_hw_bridge_in_progress = true;
		queue_work(odu_bridge_ctx->wq,
			&odu_bridge_ctx->comp_hw_bridge_work);
	}
}


/**
 * odu_send_skb_work() - workqueue function for sending a packet
 */
static void odu_send_skb_work(struct work_struct *work)
{
	struct odu_work *work_data =
		container_of(work, struct odu_work, work);
	int res;

	res = odu_bridge_request_resource_sync();
	if (res) {
		ODU_BRIDGE_ERR("Packet send failure, dropping packet !\n");
		goto bail;
	}

	switch (work_data->dir) {
	case ODU_DIR_UL:
		res = a2_mux_write(A2_MUX_TETHERED_0, work_data->skb);
		if (res) {
			ODU_BRIDGE_ERR("Packet send fail, dropping packet\n");
			goto bail;
		}
		odu_bridge_ctx->stats.num_ul_packets++;
		break;

	case ODU_DIR_DL:
		res = odu_bridge_ctx->send_dl_skb_cb(work_data->skb,
			odu_bridge_ctx->cb_priv);
		if (res) {
			ODU_BRIDGE_ERR("Packet send fail, dropping packet\n");
			goto bail;
		}
		odu_bridge_ctx->stats.num_dl_packets++;
		break;

	default:
		ODU_BRIDGE_ERR("Unsupported direction to send !\n");
		WARN_ON(1);
	}
	odu_bridge_release_resource();
	kfree(work_data);
	odu_bridge_ctx->stats.num_packets_during_resource_wakeup++;

	return;
bail:
	odu_bridge_release_resource();
	dev_kfree_skb(work_data->skb);
	kfree(work_data);
}

/**
 * defer_skb_send() - defer sending an skb using the SW bridge to a workqueue
 * @skb: pointer to the socket buffer
 * @dir: direction of send
 *
 * In case where during a packet send, the A2 or USB needs to wake up from power
 * collapse, defer the send and return the context to IPA driver. This is
 * important since IPA driver has a single threaded Rx path.
 */
static void defer_skb_send(struct sk_buff *skb, enum odu_packet_direction dir)
{
	struct odu_work *work = kmalloc(sizeof(struct odu_work), GFP_KERNEL);

	if (!work) {
		ODU_BRIDGE_ERR("No mem, dropping packet\n");
		dev_kfree_skb(skb);
		odu_bridge_release_resource();
		return;
	}

	/*
	 * Since IPA uses a single Rx thread, we don't
	 * want to wait for completion here
	 */
	INIT_WORK(&work->work, odu_send_skb_work);
	work->dir = dir;
	work->skb = skb;
	queue_work(odu_bridge_ctx->wq, &work->work);
}

/**
 * send_skb_to_a2() - Send skb in uplink direction
 * @skb: pointer to the socket buffer
 */
static int send_skb_to_a2(struct sk_buff *skb)
{
	int res;

	/*
	 * Request the BRIDGE_PROD resource, send the packet and
	 * release the resource
	 */
	res = odu_bridge_request_resource();
	if (res < 0) {
		if (res == -EINPROGRESS) {
			/* The resource is waking up */
			defer_skb_send(skb, ODU_DIR_UL);
		} else {
			ODU_BRIDGE_ERR("Packet send failure\n");
			return res;
		}
		odu_bridge_release_resource();
		return 0;
	}
	res = a2_mux_write(A2_MUX_TETHERED_0, skb);
	if (res) {
		ODU_BRIDGE_ERR("Packet send failure\n");
		odu_bridge_release_resource();
		return res;
	}
	odu_bridge_ctx->stats.num_ul_packets++;
	odu_bridge_release_resource();

	return 0;
}


/**
 * ul_notify_cb() - callback function for uplink packets
 * @priv: private data
 * @evt: event - RECEIVE or WRITE_DONE
 * @data: pointer to skb to be sent
 *
 * This callback function is installed by the IPA driver, it is invoked in 2
 * cases:
 * 1. When a packet comes from the HSIC pipe and is routed to A5
 * 2. After a packet has been bridged from USB to A2 and its skb should be freed
 *
 * Invocation: sps driver --> IPA driver --> bridge driver
 *
 * In the event of IPA_RECEIVE:
 *	In case of router mode, pass skb to ODU_IPA for handling
 *	In case of bridge mode:
 *		- Checks whether the HW bridge can be set up.
 *		- For QMI_IP packets, pass skb to ODU_IPA for handling
 *		- For multicast ipv6 packets, send skb both to A2 and ODU_IPA
 *		- For all other packets, send them to A2
 */
static void ul_notify_cb(void *priv,
			  enum ipa_dp_evt_type evt,
			  unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	struct sk_buff *skb_copied = NULL;
	struct ipv6hdr *ipv6hdr;
	int res;

	if (evt != IPA_RECEIVE) {
		ODU_BRIDGE_ERR("Unexpected event: %d\n", evt);
		return;
	}

	/* pull hw header */
	skb_pull(skb, odu_bridge_ctx->hw_hdr_info.rx.hdr_len);

	switch (odu_bridge_ctx->mode) {
	case ODU_BRIDGE_MODE_ROUTER:
		/* Router mode - pass skb to ODU_IPA */
		odu_bridge_ctx->handle_ul_skb_cb(priv, evt, data);
		break;

	case ODU_BRIDGE_MODE_BRIDGE:
		if (!odu_bridge_ctx->is_hw_bridge_complete)
			check_to_complete_hw_bridge(
			skb,
			odu_bridge_ctx->mac_addresses.host_mac_addr,
			&odu_bridge_ctx->mac_addresses.host_mac_addr_known,
			&odu_bridge_ctx->mac_addresses.device_mac_addr_known);

		ipv6hdr = (struct ipv6hdr *)(skb->data + ETH_HLEN);
		if (ODU_BRIDGE_IS_QMI_ADDR(ipv6hdr->daddr)) {
			ODU_BRIDGE_DBG("QMI packet\n");
			odu_bridge_ctx->handle_ul_skb_cb(priv, evt, data);
		} else  if (ipv6hdr->version == 6 &&
				ipv6_addr_is_multicast(&ipv6hdr->daddr)) {
			ODU_BRIDGE_DBG("Multicast packet, send to A2 and A5\n");
			skb_copied = skb_copy(skb, GFP_KERNEL);
			if (!skb_copied) {
				ODU_BRIDGE_ERR("No memory\n");
				dev_kfree_skb(skb);
				return;
			}
			odu_bridge_ctx->handle_ul_skb_cb(priv, evt, data);

			res = send_skb_to_a2(skb_copied);
			if (res) {
				ODU_BRIDGE_ERR("failed to send skb to A2\n");
				dev_kfree_skb(skb_copied);
				return;
			}
		} else {
			res = send_skb_to_a2(skb);
			if (res) {
				ODU_BRIDGE_ERR("failed to send skb to A2\n");
				dev_kfree_skb(skb);
				return;
			}
		}
		break;

	default:
		ODU_BRIDGE_ERR("Unsupported mode: %d\n", odu_bridge_ctx->mode);
		WARN_ON(1);

	}

	return;
}

/**
 * a2_notify_cb() - callback function for sending packets from A2 to HSIC
 * @user_data: private data
 * @event: event - RECEIVE or WRITE_DONE
 * @data: pointer to skb to be sent
 *
 * This callback function is installed by the IPA driver, it is invoked in 2
 * cases:
 * 1. When a packet comes from the A2 pipe and is routed to A5 (SW bridging)
 * 2. After a packet has been bridged from A2 to USB and its skb should be freed
 *
 * Invocation: sps driver --> IPA driver --> a2_service driver --> bridge driver
 *
 * In the event of A2_MUX_RECEIVE:
 * - Checks whether the HW bridge can be set up.
 * - Requests the BRIDGE_PROD resource so that A2 and USB are not in power
 * collapse. In case where the resource is waking up, defer the send operation
 * to a workqueue in order to not block the IPA driver single threaded Rx path.
 * - Sends the packets to USB using IPA driver's ipa_tx_dp() API.
 * - Releases the BRIDGE_PROD resource.
 *
 * In the event of A2_MUX_WRITE_DONE:
 * - Frees the skb memory
 */
static void a2_notify_cb(void *user_data,
			 enum a2_mux_event_type event,
			 unsigned long data)
{
	struct sk_buff *skb = (struct sk_buff *)data;
	int res;

	switch (event) {
	case A2_MUX_RECEIVE:
		if (!odu_bridge_ctx->is_hw_bridge_complete)
			check_to_complete_hw_bridge(
				skb,
				odu_bridge_ctx->mac_addresses.device_mac_addr,
				&odu_bridge_ctx->
					mac_addresses.device_mac_addr_known,
				&odu_bridge_ctx->
					mac_addresses.host_mac_addr_known);

		/*
		 * Request the BRIDGE_PROD resource, send the packet and release
		 * the resource
		 */
		res = odu_bridge_request_resource();
		if (res < 0) {
			if (res == -EINPROGRESS) {
				/* The resource is waking up */
				defer_skb_send(skb, ODU_DIR_DL);
			} else {
				ODU_BRIDGE_ERR(
					"Packet send fail, dropping packet\n");
				dev_kfree_skb(skb);
			}
			odu_bridge_release_resource();
			return;
		}

		res = odu_bridge_ctx->send_dl_skb_cb(skb,
						odu_bridge_ctx->cb_priv);
		if (res) {
			ODU_BRIDGE_ERR("Packet send fail, dropping packet\n");
			dev_kfree_skb(skb);
			odu_bridge_release_resource();
			return;
		}
		odu_bridge_ctx->stats.num_dl_packets++;
		odu_bridge_release_resource();
		break;

	case A2_MUX_WRITE_DONE:
		dev_kfree_skb(skb);
		break;

	default:
		ODU_BRIDGE_ERR("Unsupported IPA event !\n");
		WARN_ON(1);
	}

	return;
}

/**
 * bridge_prod_notify_cb() - IPA Resource Manager callback function
 * @param notify_cb_data: private data
 * @param event: RESOURCE_GRANTED / RESOURCE_RELEASED
 * @param data: not used in this case
 *
 * This callback function is called by IPA resource manager to notify the
 * BRIDGE_PROD entity of events like RESOURCE_GRANTED and RESOURCE_RELEASED.
 */
static void bridge_prod_notify_cb(void *notify_cb_data,
				  enum ipa_rm_event event,
				  unsigned long data)
{
	switch (event) {
	case IPA_RM_RESOURCE_GRANTED:
		complete(&odu_bridge_ctx->is_bridge_prod_up);
		break;

	case IPA_RM_RESOURCE_RELEASED:
		complete(&odu_bridge_ctx->is_bridge_prod_down);
		break;

	default:
		ODU_BRIDGE_ERR("Unsupported notification!\n");
		WARN_ON(1);
		break;
	}

	return;
}

/**
 * a2_prod_notify_cb() - IPA Resource Manager callback function for A2_PROD
 * @notify_cb_data: private data
 * @event: RESOURCE_GRANTED / RESOURCE_RELEASED
 * @data: not used in this case
 *
 * This callback function is called by IPA resource manager to notify the
 * state of A2_PROD. This is used for EP configuration since when A2_PROD is
 * granted, a2_service is connecting A2<->IPA pipes.
 */
static void a2_prod_notify_cb(void *notify_cb_data,
			      enum ipa_rm_event event,
			      unsigned long data)
{
	int res;
	struct ipa_ep_cfg ipa_ep_cfg;

	switch (event) {
	case IPA_RM_RESOURCE_GRANTED:
		res = a2_mux_get_client_handles(
					A2_MUX_TETHERED_0,
					&odu_bridge_ctx->ipa_a2_pipe_hdl,
					&odu_bridge_ctx->a2_ipa_pipe_hdl);
		if (res) {
			ODU_BRIDGE_ERR(
				"a2_mux_get_client_handles() failed, res = %d\n",
				res);
			return;
		}

		/* Reset the various endpoints configuration */
		memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
		ipa_ep_cfg.hdr.hdr_len = ETH_HLEN;
		ipa_cfg_ep(odu_bridge_ctx->ipa_a2_pipe_hdl, &ipa_ep_cfg);
		ipa_cfg_ep(odu_bridge_ctx->a2_ipa_pipe_hdl, &ipa_ep_cfg);
		break;

	case IPA_RM_RESOURCE_RELEASED:
		break;

	default:
		ODU_BRIDGE_ERR("Unsupported notification!\n");
		WARN_ON(1);
		break;
	}

	return;
}

static void free_del_hnds(void)
{
	ODU_BRIDGE_FUNC_ENTRY();

	kfree(odu_bridge_ctx->hdr_del);
	kfree(odu_bridge_ctx->routing_del[IPA_IP_v4]);
	kfree(odu_bridge_ctx->routing_del[IPA_IP_v6]);
	kfree(odu_bridge_ctx->filtering_del[IPA_IP_v4]);
	kfree(odu_bridge_ctx->filtering_del[IPA_IP_v6]);

	ODU_BRIDGE_FUNC_EXIT();
}

static int delete_hsic_dependencies(void)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	/*
	 * Delete part of IPA resource manager dependency graph. Only the
	 * BRIDGE_PROD <-> A2 dependency remains intact
	 */
	res = ipa_rm_delete_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
				       IPA_RM_RESOURCE_HSIC_CONS);
	if ((res != 0) && (res != -EINPROGRESS))
		ODU_BRIDGE_ERR(
			"Failed deleting ipa_rm dependency BRIDGE_PROD <-> HSIC_CONS\n");
	res = ipa_rm_delete_dependency(IPA_RM_RESOURCE_HSIC_PROD,
				       IPA_RM_RESOURCE_A2_CONS);
	if ((res != 0) && (res != -EINPROGRESS))
		ODU_BRIDGE_ERR(
			"Failed deleting ipa_rm dependency HSIC_PROD <-> A2_CONS\n");
	res = ipa_rm_delete_dependency(IPA_RM_RESOURCE_A2_PROD,
				       IPA_RM_RESOURCE_HSIC_CONS);
	if ((res != 0) && (res != -EINPROGRESS))
		ODU_BRIDGE_ERR(
			"Failed deleting ipa_rm dependency A2_PROD <-> HSIC_CONS\n");
	
	ODU_BRIDGE_FUNC_EXIT();
	return res;
}

static void teardown_hw_bridge(void)
{
	ODU_BRIDGE_FUNC_ENTRY();
	/* Delete header entries */
	if (ipa_del_hdr(odu_bridge_ctx->hdr_del))
		ODU_BRIDGE_ERR("ipa_del_hdr() failed\n");

	/* Delete installed routing rules */
	if (ipa_del_rt_rule(odu_bridge_ctx->routing_del[IPA_IP_v4]))
		ODU_BRIDGE_ERR("ipa_del_rt_rule() failed\n");
	if (ipa_del_rt_rule(odu_bridge_ctx->routing_del[IPA_IP_v6]))
		ODU_BRIDGE_ERR("ipa_del_rt_rule() failed\n");

	/* Delete installed filtering rules */
	if (ipa_del_flt_rule(odu_bridge_ctx->filtering_del[IPA_IP_v4]))
		ODU_BRIDGE_ERR("ipa_del_flt_rule() failed\n");
	if (ipa_del_flt_rule(odu_bridge_ctx->filtering_del[IPA_IP_v6]))
		ODU_BRIDGE_ERR("ipa_del_flt_rule() failed\n");

	/*
	 * Commit all the data to HW, including header, routing and
	 * filtering blocks, IPv4 and IPv6
	 */
	if (ipa_commit_hdr())
		ODU_BRIDGE_ERR("Failed committing delete rules\n");

	odu_bridge_ctx->is_hw_bridge_complete = false;
	
	ODU_BRIDGE_FUNC_EXIT();
	return;
}

/**
 * initialize_context() - Initialize the odu_bridge_ctx struct
 */
static void initialize_context(void)
{
	ODU_BRIDGE_FUNC_ENTRY();

	/* Initialize context variables */
	odu_bridge_ctx->ipa_a2_pipe_hdl = 0;
	odu_bridge_ctx->a2_ipa_pipe_hdl = 0;

	memset(&odu_bridge_ctx->mac_addresses, 0,
					sizeof(odu_bridge_ctx->mac_addresses));
	INIT_COMPLETION(odu_bridge_ctx->is_bridge_prod_up);
	INIT_COMPLETION(odu_bridge_ctx->is_bridge_prod_down);
	memset(odu_bridge_ctx->hdr_del,
		0,
		sizeof(struct ipa_ioc_del_hdr) + ODU_TOTAL_HDR_ENTRIES *
		sizeof(struct ipa_hdr_del));
	odu_bridge_ctx->is_connected = false;
	odu_bridge_ctx->is_hw_bridge_complete = false;
	odu_bridge_ctx->comp_hw_bridge_in_progress = false;
	memset(odu_bridge_ctx->routing_del[IPA_IP_v4],
		0,
		sizeof(struct ipa_ioc_del_rt_rule) +
		ODU_TOTAL_RT_ENTRIES_IP * sizeof(struct ipa_rt_rule_del));
	odu_bridge_ctx->routing_del[IPA_IP_v4]->ip = IPA_IP_v4;
	memset(odu_bridge_ctx->routing_del[IPA_IP_v6],
		0,
		sizeof(struct ipa_ioc_del_rt_rule) +
		ODU_TOTAL_RT_ENTRIES_IP * sizeof(struct ipa_rt_rule_del));
	odu_bridge_ctx->routing_del[IPA_IP_v6]->ip = IPA_IP_v6;
	memset(odu_bridge_ctx->filtering_del[IPA_IP_v4],
		0,
		sizeof(struct ipa_ioc_del_flt_rule) +
		ODU_TOTAL_FLT_ENTRIES_IP * sizeof(struct ipa_flt_rule_del));
	odu_bridge_ctx->filtering_del[IPA_IP_v4]->ip = IPA_IP_v4;
	memset(odu_bridge_ctx->filtering_del[IPA_IP_v6],
		0,
		sizeof(struct ipa_ioc_del_flt_rule) +
		ODU_TOTAL_FLT_ENTRIES_IP * sizeof(struct ipa_flt_rule_del));
	odu_bridge_ctx->filtering_del[IPA_IP_v6]->ip = IPA_IP_v6;
	ODU_BRIDGE_FUNC_EXIT();
}

/**
 * odu_bridge_disconnect() - Disconnect odu bridge
 */
int odu_bridge_disconnect()
{
	int res;
	struct ipa_rm_register_params a2_prod_reg_params;

	ODU_BRIDGE_FUNC_ENTRY();

	if (!odu_bridge_ctx) {
		ODU_BRIDGE_ERR("Not initialized\n");
		return -EFAULT;
	}

	if (!odu_bridge_ctx->is_connected) {
		ODU_BRIDGE_ERR("Not connected\n");
		return -EFAULT;
	}

	if (odu_bridge_ctx->mode == ODU_BRIDGE_MODE_BRIDGE) {
		res = delete_hsic_dependencies();
		if (res) {
			ODU_BRIDGE_ERR("delete_hsic_dependencies() failed.\n");
			return res;
		}

		/* Request the BRIDGE_PROD resource, power up A2 and IPA */
		res = odu_bridge_request_resource_sync();
		if (res) {
			ODU_BRIDGE_ERR("request_resource() failed.\n");
			return res;
		}

		/* Close the channel to A2 */
		if (a2_mux_close_channel(A2_MUX_TETHERED_0))
			ODU_BRIDGE_ERR("a2_mux_close_channel failed\n");
		/* Tear down the IPA HW bridge */
		if (odu_bridge_ctx->is_hw_bridge_complete)
			teardown_hw_bridge();

		odu_bridge_release_resource();

		/* Deregister from A2_PROD notifications */
		a2_prod_reg_params.user_data = NULL;
		a2_prod_reg_params.notify_cb = a2_prod_notify_cb;
		res = ipa_rm_deregister(IPA_RM_RESOURCE_A2_PROD,
							&a2_prod_reg_params);
		if (res)
			ODU_BRIDGE_ERR(
				"Failed deregister A2_prod notifications.\n");

		/* Delete the last ipa_rm dependency - BRIDGE_PROD <-> A2 */
		res = ipa_rm_delete_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
					IPA_RM_RESOURCE_A2_CONS);
		if ((res != 0) && (res != -EINPROGRESS))
			ODU_BRIDGE_ERR(
			"Failed deleting BRIDGE_PROD<->A2_CONS dependency\n");
	}

	initialize_context();

	odu_bridge_ctx->is_connected = false;

	ODU_BRIDGE_FUNC_EXIT();
	return 0;
}
EXPORT_SYMBOL(odu_bridge_disconnect);

/**
 * odu_bridge_handle_mcast_skb() - function handler for multicast packets
 * @skb: pointer to the socket buffer
 *
 * This function habdles multicast depends on the mode:
 * - For router mode, do nothing
 * - For bridge mode, send a copy of the skb to A2
 */
int odu_bridge_handle_mcast_skb(struct sk_buff *skb)
{
	struct sk_buff *skb_copied = NULL;
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	switch (odu_bridge_ctx->mode) {
	case ODU_BRIDGE_MODE_ROUTER:
		break;

	case ODU_BRIDGE_MODE_BRIDGE:
		skb_copied = skb_copy(skb, GFP_KERNEL);
		if (!skb_copied) {
			ODU_BRIDGE_ERR("No memory\n");
			return -ENOMEM;
		}
		break;

		res = send_skb_to_a2(skb_copied);
		if (res) {
			ODU_BRIDGE_ERR("failed to send skb to A2\n");
			dev_kfree_skb(skb_copied);
			return res;
		}

	default:
		ODU_BRIDGE_ERR("Unsupported mode: %d\n", odu_bridge_ctx->mode);
		WARN_ON(1);

	}

	ODU_BRIDGE_FUNC_EXIT();
	return 0;
}
EXPORT_SYMBOL(odu_bridge_handle_mcast_skb);

/**
 * odu_bridge_add_hw_hdr_info() - Add information regarding HW header that
 * shall be used on hsic pipes.
 * @hw_hdr_info: header information
 *
 *
 * Returns negative errno, or zero on success
 */
int odu_bridge_add_hw_hdr_info(struct odu_ipa_hw_hdr_info *hw_hdr_info)
{
	ODU_BRIDGE_FUNC_ENTRY();

	if (!odu_bridge_ctx) {
		ODU_BRIDGE_ERR("Not initialized\n");
		return -EFAULT;
	}

	memcpy(&odu_bridge_ctx->hw_hdr_info, hw_hdr_info,
						sizeof(*hw_hdr_info));

	ODU_BRIDGE_FUNC_EXIT();

	return 0;
}
EXPORT_SYMBOL(odu_bridge_add_hw_hdr_info);

/**
 * odu_bridge_connect_bridge() - Actual connection of odu bridge
 *
 * This function is called in bridge mode only, and it is responsible for
 * building dependency graph, and registering as a client to a2_service.
 *
 * Return codes: 0: success
 *		-EINVAL: invalid parameters
 *		-EPERM: Operation not permitted as the bridge is already
 *		connected
 */
static int odu_bridge_connect_bridge(void)
{
	int res;
	struct ipa_ep_cfg ipa_ep_cfg;
	struct ipa_rm_register_params a2_prod_reg_params;

	ODU_BRIDGE_FUNC_ENTRY();

	/* Build IPA Resource manager dependency graph */
	ODU_BRIDGE_DBG("build dependency graph\n");
	res = ipa_rm_add_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
					IPA_RM_RESOURCE_HSIC_CONS);
	if (res && res != -EINPROGRESS) {
		ODU_BRIDGE_ERR("ipa_rm_add_dependency() failed\n");
		goto fail_add_dependency_1;
	}

	res = ipa_rm_add_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
					IPA_RM_RESOURCE_A2_CONS);
	if (res && res != -EINPROGRESS) {
		ODU_BRIDGE_ERR("ipa_rm_add_dependency() failed\n");
		goto fail_add_dependency_2;
	}

	res = ipa_rm_add_dependency(IPA_RM_RESOURCE_HSIC_PROD,
					IPA_RM_RESOURCE_A2_CONS);
	if (res && res != -EINPROGRESS) {
		ODU_BRIDGE_ERR("ipa_rm_add_dependency() failed\n");
		goto fail_add_dependency_3;
	}

	res = ipa_rm_add_dependency(IPA_RM_RESOURCE_A2_PROD,
					IPA_RM_RESOURCE_HSIC_CONS);
	if (res && res != -EINPROGRESS) {
		ODU_BRIDGE_ERR("ipa_rm_add_dependency() failed\n");
		goto fail_add_dependency_4;
	}

	res = odu_bridge_request_resource_sync();
	if (res) {
		ODU_BRIDGE_ERR("request_resource() failed.\n");
		goto fail_request_resource;
	}

	res = a2_mux_open_channel(A2_MUX_TETHERED_0, NULL, a2_notify_cb);
	if (res) {
		ODU_BRIDGE_ERR("a2_mux_open_channel failed %d\n", res);
		goto fail_open_channel;
	}

	res = a2_mux_get_client_handles(A2_MUX_TETHERED_0,
					&odu_bridge_ctx->ipa_a2_pipe_hdl,
					&odu_bridge_ctx->a2_ipa_pipe_hdl);
	if (res) {
		ODU_BRIDGE_ERR(
			"a2_mux_get_client_handles() failed, res = %d\n", res);
			goto fail_get_handles;
	}

	ODU_BRIDGE_DBG("ipa_a2_pipe_hdl=0x%x, a2_ipa_pipe_hdl=0x%x\n",
			odu_bridge_ctx->ipa_a2_pipe_hdl,
			odu_bridge_ctx->a2_ipa_pipe_hdl);

	/* Reset the various endpoints configuration */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_cfg_ep(odu_bridge_ctx->ipa_a2_pipe_hdl, &ipa_ep_cfg);
	ipa_cfg_ep(odu_bridge_ctx->a2_ipa_pipe_hdl, &ipa_ep_cfg);

	/* Register for A2_PROD resource notifications */
	a2_prod_reg_params.user_data = NULL;
	a2_prod_reg_params.notify_cb = a2_prod_notify_cb;
	res = ipa_rm_register(IPA_RM_RESOURCE_A2_PROD, &a2_prod_reg_params);
	if (res) {
		ODU_BRIDGE_ERR("ipa_rm_register() failed\n");
		goto fail_rm_register;
	}

	odu_bridge_release_resource();
	ODU_BRIDGE_FUNC_EXIT();

	return 0;

fail_rm_register:
	odu_bridge_ctx->ipa_a2_pipe_hdl = 0;
	odu_bridge_ctx->a2_ipa_pipe_hdl = 0;
fail_get_handles:
	a2_mux_close_channel(A2_MUX_TETHERED_0);
fail_open_channel:
	odu_bridge_release_resource();
fail_request_resource:
	ipa_rm_delete_dependency(IPA_RM_RESOURCE_A2_PROD,
				IPA_RM_RESOURCE_HSIC_CONS);
fail_add_dependency_4:
	ipa_rm_delete_dependency(IPA_RM_RESOURCE_HSIC_PROD,
				IPA_RM_RESOURCE_A2_CONS);
fail_add_dependency_3:
	ipa_rm_delete_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
				IPA_RM_RESOURCE_A2_CONS);
fail_add_dependency_2:
	ipa_rm_delete_dependency(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
				IPA_RM_RESOURCE_HSIC_CONS);
fail_add_dependency_1:
	return res;
}

/**
 * odu_bridge_connect() - Connect odu bridge.
 *
 * Actual connect work is done in odu_bridge_connect_bridge(), and it is
 * called only for bridge mode.
 *
 * Return codes: 0: success
 *		-EINVAL: invalid parameters
 *		-EPERM: Operation not permitted as the bridge is already
 *		connected
 */
int odu_bridge_connect()
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	if (!odu_bridge_ctx) {
		ODU_BRIDGE_ERR("Not initialized\n");
		return -EFAULT;
	}

	if (odu_bridge_ctx->is_connected) {
		ODU_BRIDGE_ERR("already connected\n");
		return -EFAULT;
	}

	mutex_lock(&odu_bridge_ctx->lock);
	odu_bridge_ctx->is_connected = true;
	if (odu_bridge_ctx->mode == ODU_BRIDGE_MODE_BRIDGE) {
		res = odu_bridge_connect_bridge();
		if (res) {
			ODU_BRIDGE_ERR("connect_bridge failed\n");
			goto bail;
		}
	}

	res = 0;
bail:
	mutex_unlock(&odu_bridge_ctx->lock);
	ODU_BRIDGE_FUNC_EXIT();
	return res;
}
EXPORT_SYMBOL(odu_bridge_connect);

/**
 * odu_bridge_set_mode() - Set bridge mode to Router/Bridge
 * @mode: mode to be set
 */
static int odu_bridge_set_mode(enum odu_bridge_mode mode)
{
	int res;

	ODU_BRIDGE_FUNC_ENTRY();

	if (mode < 0 || mode >= ODU_BRIDGE_MODE_MAX) {
		ODU_BRIDGE_ERR("Unsupported mode: %d\n", mode);
		return -EFAULT;
	}

	ODU_BRIDGE_DBG("setting mode: %d\n", mode);
	mutex_lock(&odu_bridge_ctx->lock);
	odu_bridge_ctx->mode = mode;
	if (odu_bridge_ctx->is_connected &&
			odu_bridge_ctx->mode == ODU_BRIDGE_MODE_BRIDGE) {
		res = odu_bridge_connect_bridge();
		if (res) {
			ODU_BRIDGE_ERR("connect_bridge failed\n");
			goto bail;
		}
	}

	res = 0;
bail:
	mutex_unlock(&odu_bridge_ctx->lock);
	ODU_BRIDGE_FUNC_EXIT();
	return res;
};

/**
 * odu_bridge_set_llv6_addr() - Set link local ipv6 address
 * @llv6_addr: odu network interface link local address
 *
 * This function also complete the HW bridge, if MAC addresses are known and
 * bridge is not complete yet.
 */
static int odu_bridge_set_llv6_addr(struct in6_addr *llv6_addr)
{
	bool both_mac_addresses_known;
	struct in6_addr llv6_addr_host;

	ODU_BRIDGE_FUNC_ENTRY();

	llv6_addr_host.s6_addr32[0] = ntohl(llv6_addr->s6_addr32[0]);
	llv6_addr_host.s6_addr32[1] = ntohl(llv6_addr->s6_addr32[1]);
	llv6_addr_host.s6_addr32[2] = ntohl(llv6_addr->s6_addr32[2]);
	llv6_addr_host.s6_addr32[3] = ntohl(llv6_addr->s6_addr32[3]);

	/**
	 * In case that link local v6 address is different from the stored one,
	 * teardown the bridge and setup again.
	 */
	if (odu_bridge_ctx->is_hw_bridge_complete &&
			odu_bridge_ctx->llv6_addr.addr_known &&
			memcmp(&odu_bridge_ctx->llv6_addr.addr,
			&llv6_addr_host, sizeof(struct in6_addr)) != 0) {
		ODU_BRIDGE_DBG("link local address changed\n");
		ODU_BRIDGE_DBG("old LLV6 addr: %pI6c\n",
			&odu_bridge_ctx->llv6_addr.addr);

		teardown_hw_bridge();
	}

	memcpy(&odu_bridge_ctx->llv6_addr.addr, &llv6_addr_host,
				sizeof(odu_bridge_ctx->llv6_addr.addr));
	ODU_BRIDGE_DBG("LLV6 addr: %pI6c\n", &odu_bridge_ctx->llv6_addr.addr);


	odu_bridge_ctx->llv6_addr.addr_known = true;
	both_mac_addresses_known =
		odu_bridge_ctx->mac_addresses.device_mac_addr_known &&
		odu_bridge_ctx->mac_addresses.host_mac_addr_known;
	if (!odu_bridge_ctx->is_hw_bridge_complete &&
				!odu_bridge_ctx->comp_hw_bridge_in_progress &&
				both_mac_addresses_known) {
		ODU_BRIDGE_DBG("completeing HW bridge\n");
		odu_bridge_ctx->comp_hw_bridge_in_progress = true;
		queue_work(odu_bridge_ctx->wq,
			&odu_bridge_ctx->comp_hw_bridge_work);
	}

	ODU_BRIDGE_FUNC_EXIT();

	return 0;
};

static long odu_bridge_ioctl(struct file *filp,
			      unsigned int cmd,
			      unsigned long arg)
{
	int res = 0;
	struct in6_addr llv6_addr;

	ODU_BRIDGE_DBG("cmd=%x nr=%d\n", cmd, _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != ODU_BRIDGE_IOC_MAGIC) ||
	    (_IOC_NR(cmd) >= ODU_BRIDGE_IOCTL_MAX)) {
		ODU_BRIDGE_ERR("Invalid ioctl\n");
		return -ENOIOCTLCMD;
	}

	switch (cmd) {
	case ODU_BRIDGE_IOC_SET_MODE:
		ODU_BRIDGE_DBG("ODU_BRIDGE_IOC_SET_MODE ioctl called\n");
		res = odu_bridge_set_mode(arg);
		if (res) {
			ODU_BRIDGE_ERR("Error, res = %d\n", res);
			break;
		}
		break;

	case ODU_BRIDGE_IOC_SET_LLV6_ADDR:
		ODU_BRIDGE_DBG("ODU_BRIDGE_IOC_SET_LLV6_ADDR ioctl called\n");
		res = copy_from_user(&llv6_addr,
			(struct in6_addr *)arg,
			sizeof(llv6_addr));
		if (res) {
			ODU_BRIDGE_ERR("Error, res = %d\n", res);
			res = -EFAULT;
			break;
		}

		res = odu_bridge_set_llv6_addr(&llv6_addr);
		if (res) {
			ODU_BRIDGE_ERR("Error, res = %d\n", res);
			break;
		}
		break;

	default:
		ODU_BRIDGE_ERR("Unknown ioctl: %d\n", cmd);
		WARN_ON(1);
	}

	return res;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;
static struct dentry *dfile_stats;
static struct dentry *dfile_is_hw_bridge_complete;
static struct dentry *dfile_mode;

static ssize_t odu_debugfs_stats(struct file *file,
				  char __user *ubuf,
				  size_t count,
				  loff_t *ppos)
{
	int nbytes = 0;

	nbytes += scnprintf(&dbg_buff[nbytes],
			    ODU_MAX_MSG_LEN - nbytes,
			   "UL packets: %lld\n",
			    odu_bridge_ctx->stats.num_ul_packets);
	nbytes += scnprintf(&dbg_buff[nbytes],
			    ODU_MAX_MSG_LEN - nbytes,
			   "DL packets: %lld\n",
			    odu_bridge_ctx->stats.num_dl_packets);
	nbytes += scnprintf(
		&dbg_buff[nbytes],
		ODU_MAX_MSG_LEN - nbytes,
		"SW Tx packets sent during resource wakeup: %lld\n",
		odu_bridge_ctx->stats.num_packets_during_resource_wakeup);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t odu_debugfs_hw_bridge_status(struct file *file,
					     char __user *ubuf,
					     size_t count,
					     loff_t *ppos)
{
	int nbytes = 0;

	if (odu_bridge_ctx->is_hw_bridge_complete)
		nbytes += scnprintf(&dbg_buff[nbytes],
				    ODU_MAX_MSG_LEN - nbytes,
				   "HW bridge is in use.\n");
	else
		nbytes += scnprintf(&dbg_buff[nbytes],
			ODU_MAX_MSG_LEN - nbytes,
			"SW bridge is in use. HW bridge not complete yet.\n");

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t odu_debugfs_hw_bridge_mode_write(struct file *file,
					const char __user *ubuf,
					size_t count,
					loff_t *ppos)
{
	unsigned long missing;
	enum odu_bridge_mode mode;

	if (sizeof(dbg_buff) < count + 1)
		return -EFAULT;

	missing = copy_from_user(dbg_buff, ubuf, count);
	if (missing)
		return -EFAULT;

	if (count > 0)
		dbg_buff[count-1] = '\0';

	if (strcmp(dbg_buff, "router") == 0) {
		mode = ODU_BRIDGE_MODE_ROUTER;
	} else if (strcmp(dbg_buff, "bridge") == 0) {
		mode = ODU_BRIDGE_MODE_BRIDGE;
	} else {
		ODU_BRIDGE_ERR("Bad mode, got %s,\n"
			 "Use <router> or <bridge>.\n", dbg_buff);
		return count;
	}

	odu_bridge_set_mode(mode);
	return count;
}

static ssize_t odu_debugfs_hw_bridge_mode_read(struct file *file,
					     char __user *ubuf,
					     size_t count,
					     loff_t *ppos)
{
	int nbytes = 0;

	switch (odu_bridge_ctx->mode) {
	case ODU_BRIDGE_MODE_ROUTER:
		nbytes += scnprintf(&dbg_buff[nbytes],
			ODU_MAX_MSG_LEN - nbytes,
			"router\n");
		break;
	case ODU_BRIDGE_MODE_BRIDGE:
		nbytes += scnprintf(&dbg_buff[nbytes],
			ODU_MAX_MSG_LEN - nbytes,
			"bridge\n");
		break;
	default:
		nbytes += scnprintf(&dbg_buff[nbytes],
			ODU_MAX_MSG_LEN - nbytes,
			"mode error\n");
		break;

	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

const struct file_operations odu_stats_ops = {
	.read = odu_debugfs_stats,
};

const struct file_operations odu_hw_bridge_status_ops = {
	.read = odu_debugfs_hw_bridge_status,
};

const struct file_operations odu_hw_bridge_mode_ops = {
	.read = odu_debugfs_hw_bridge_mode_read,
	.write = odu_debugfs_hw_bridge_mode_write,
};

void odu_debugfs_init(void)
{
	const mode_t read_only_mode = S_IRUSR | S_IRGRP | S_IROTH;
	const mode_t read_write_mode = S_IRUSR | S_IRGRP | S_IROTH |
		S_IWUSR | S_IWGRP | S_IWOTH;

	dent = debugfs_create_dir("odu_ipa_bridge", 0);
	if (IS_ERR(dent)) {
		ODU_BRIDGE_ERR("fail to create folder odu_ipa_bridge\n");
		return;
	}

	dfile_stats =
		debugfs_create_file("stats", read_only_mode, dent,
				    0, &odu_stats_ops);
	if (!dfile_stats || IS_ERR(dfile_stats)) {
		ODU_BRIDGE_ERR("fail to create file stats\n");
		goto fail;
	}

	dfile_is_hw_bridge_complete =
		debugfs_create_file("is_hw_bridge_complete", read_only_mode,
				    dent, 0, &odu_hw_bridge_status_ops);
	if (!dfile_is_hw_bridge_complete ||
	    IS_ERR(dfile_is_hw_bridge_complete)) {
		ODU_BRIDGE_ERR("fail to create file is_hw_bridge_complete\n");
		goto fail;
	}

	dfile_mode =
		debugfs_create_file("mode", read_write_mode,
				    dent, 0, &odu_hw_bridge_mode_ops);
	if (!dfile_mode ||
	    IS_ERR(dfile_mode)) {
		ODU_BRIDGE_ERR("fail to create file dfile_mode\n");
		goto fail;
	}

	return;
fail:
	debugfs_remove_recursive(dent);
}

static void odu_debugfs_destroy(void)
{
	debugfs_remove_recursive(dent);
}

#else
static void odu_debugfs_init(void) {}
static void odu_debugfs_destroy(void) {}
#endif /* CONFIG_DEBUG_FS */


static const struct file_operations odu_bridge_drv_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = odu_bridge_ioctl,
};

static int alloc_del_hnds(void)
{
	ODU_BRIDGE_FUNC_ENTRY();

	odu_bridge_ctx->hdr_del = kzalloc(sizeof(struct ipa_ioc_del_hdr) +
					ODU_TOTAL_HDR_ENTRIES *
					sizeof(struct ipa_hdr_del),
					GFP_KERNEL);
	if (!odu_bridge_ctx->hdr_del) {
		ODU_BRIDGE_ERR("kzalloc err for hdr_del.\n");
		return -ENOMEM;
	}


	odu_bridge_ctx->routing_del[IPA_IP_v4] =
		kzalloc(sizeof(struct ipa_ioc_del_rt_rule) +
				ODU_TOTAL_RT_ENTRIES_IP *
				sizeof(struct ipa_rt_rule_del),
				GFP_KERNEL);
	if (!odu_bridge_ctx->routing_del[IPA_IP_v4]) {
		ODU_BRIDGE_ERR("kzalloc err for routing_del[IPA_IP_v4].\n");
		goto fail_alloc_routing_del_ipv4;
	}


	odu_bridge_ctx->routing_del[IPA_IP_v6] =
			kzalloc(sizeof(struct ipa_ioc_del_rt_rule) +
				ODU_TOTAL_RT_ENTRIES_IP *
				sizeof(struct ipa_rt_rule_del),
				GFP_KERNEL);
	if (!odu_bridge_ctx->routing_del[IPA_IP_v6]) {
		ODU_BRIDGE_ERR("kzalloc err for routing_del[IPA_IP_v6].\n");
		goto fail_alloc_routing_del_ipv6;
	}

	odu_bridge_ctx->filtering_del[IPA_IP_v4] =
		kzalloc(sizeof(struct ipa_ioc_del_flt_rule) +
			ODU_TOTAL_FLT_ENTRIES_IP *
			sizeof(struct ipa_flt_rule_del),
			GFP_KERNEL);
	if (!odu_bridge_ctx->filtering_del[IPA_IP_v4]) {
		ODU_BRIDGE_ERR("kzalloc err.\n");
		goto fail_alloc_filtering_del_ipv4;
	}


	odu_bridge_ctx->filtering_del[IPA_IP_v6] =
			kzalloc(sizeof(struct ipa_ioc_del_flt_rule) +
				ODU_TOTAL_FLT_ENTRIES_IP *
				sizeof(struct ipa_flt_rule_del),
				GFP_KERNEL);
	if (!odu_bridge_ctx->filtering_del[IPA_IP_v6]) {
		ODU_BRIDGE_ERR("kzalloc err.\n");
		goto fail_alloc_filtering_del_ipv6;
	}

	ODU_BRIDGE_FUNC_EXIT();
	return 0;

fail_alloc_filtering_del_ipv6:
	kfree(odu_bridge_ctx->filtering_del[IPA_IP_v6]);

fail_alloc_filtering_del_ipv4:
	kfree(odu_bridge_ctx->filtering_del[IPA_IP_v4]);

fail_alloc_routing_del_ipv6:
	kfree(odu_bridge_ctx->routing_del[IPA_IP_v6]);

fail_alloc_routing_del_ipv4:
	kfree(odu_bridge_ctx->routing_del[IPA_IP_v4]);

	kfree(odu_bridge_ctx->hdr_del);

	return -ENOMEM;
}

/**
 * odu_bridge_init() - Initialize the ODU bridge driver
 * @send_dl_skb_cb: [in] Callback function for sending skb to HSIC
 * @cb_priv: [in] Data for the callback function.
 * @handle_ul_skb_cb: [in] Callback function for sending skb to kernel
 * @ul_notify_cb_ptr: [out] callback to be used by the caller to be setup in
 * IPA uplink pipe.
 *
 * USB driver gets a pointer to a callback function (ul_notify_cb_ptr) and an
 * associated data. USB driver installs this callback function in the call to
 * ipa_connect().
 *
 * Return codes: 0: success,
 *		-EINVAL - Bad parameter
 *		Other negative value - Failure
 */
int odu_bridge_init(odu_bridge_send_dl_skb_cb send_dl_skb_cb,
		    void *cb_priv,
		    ipa_notify_cb handle_ul_skb_cb,
		    ipa_notify_cb *ul_notify_cb_ptr)
{
	int res;
	struct ipa_rm_create_params bridge_prod_params;
	res = -ENOMEM;

	ODU_BRIDGE_FUNC_ENTRY();

	if (!send_dl_skb_cb || !handle_ul_skb_cb || !ul_notify_cb_ptr) {
		ODU_BRIDGE_ERR("Bad parameter\n");
		return -EINVAL;
	}

	if (odu_bridge_ctx) {
		ODU_BRIDGE_ERR("Already initialized\n");
		return -EFAULT;
	}

	odu_bridge_ctx = kzalloc(sizeof(*odu_bridge_ctx), GFP_KERNEL);
	if (!odu_bridge_ctx) {
		ODU_BRIDGE_ERR("kzalloc err.\n");
		return -ENOMEM;
	}

	odu_bridge_ctx->send_dl_skb_cb = send_dl_skb_cb;
	odu_bridge_ctx->cb_priv = cb_priv;
	odu_bridge_ctx->handle_ul_skb_cb = handle_ul_skb_cb;

	odu_bridge_ctx->class = class_create(THIS_MODULE, ODU_BRIDGE_DRV_NAME);
	if (!odu_bridge_ctx->class) {
		ODU_BRIDGE_ERR("Class_create err.\n");
		goto fail_class_create;
	}

	res = alloc_chrdev_region(&odu_bridge_ctx->dev_num, 0, 1,
				  ODU_BRIDGE_DRV_NAME);
	if (res) {
		ODU_BRIDGE_ERR("alloc_chrdev_region err.\n");
		res = -ENODEV;
		goto fail_alloc_chrdev_region;
	}

	odu_bridge_ctx->dev = device_create(odu_bridge_ctx->class, NULL,
		odu_bridge_ctx->dev_num, odu_bridge_ctx, ODU_BRIDGE_DRV_NAME);
	if (IS_ERR(odu_bridge_ctx->dev)) {
		ODU_BRIDGE_ERR(":device_create err.\n");
		res = -ENODEV;
		goto fail_device_create;
	}

	cdev_init(&odu_bridge_ctx->cdev, &odu_bridge_drv_fops);
	odu_bridge_ctx->cdev.owner = THIS_MODULE;
	odu_bridge_ctx->cdev.ops = &odu_bridge_drv_fops;

	res = cdev_add(&odu_bridge_ctx->cdev, odu_bridge_ctx->dev_num, 1);
	if (res) {
		ODU_BRIDGE_ERR(":cdev_add err=%d\n", -res);
		res = -ENODEV;
		goto fail_cdev_add;
	}

	odu_debugfs_init();

	/* Create BRIDGE_PROD entity in IPA Resource Manager */
	bridge_prod_params.name = IPA_RM_RESOURCE_ODU_BRIDGE_PROD;
	bridge_prod_params.reg_params.user_data = NULL;
	bridge_prod_params.reg_params.notify_cb = bridge_prod_notify_cb;
	res = ipa_rm_create_resource(&bridge_prod_params);
	if (res) {
		ODU_BRIDGE_ERR("ipa_rm_create_resource() failed\n");
		goto fail_cdev_add;
	}
	init_completion(&odu_bridge_ctx->is_bridge_prod_up);
	init_completion(&odu_bridge_ctx->is_bridge_prod_down);

	res = ipa_rm_inactivity_timer_init(IPA_RM_RESOURCE_ODU_BRIDGE_PROD,
					   ODU_INACTIVITY_TIME_MSEC);
	if (res) {
		ODU_BRIDGE_ERR("ipa_rm_inactivity_timer_init() failed %d\n",
			 res);
		goto fail_cdev_add;
	}

	odu_bridge_ctx->wq = create_workqueue(ODU_WORKQUEUE_NAME);
	if (!odu_bridge_ctx->wq) {
		ODU_BRIDGE_ERR("workqueue creation failed\n");
		goto fail_cdev_add;
	}

	res = alloc_del_hnds();
	if (res) {
		ODU_BRIDGE_ERR("kzalloc err.\n");
		goto fail_cdev_add;
	}

	mutex_init(&odu_bridge_ctx->request_resource_mutex);
	mutex_init(&odu_bridge_ctx->lock);
	INIT_WORK(&odu_bridge_ctx->comp_hw_bridge_work, complete_hw_bridge);

	odu_bridge_ctx->mode = ODU_BRIDGE_MODE_ROUTER;

	*ul_notify_cb_ptr = ul_notify_cb;
	ODU_BRIDGE_FUNC_EXIT();

	return 0;

fail_cdev_add:
	device_destroy(odu_bridge_ctx->class, odu_bridge_ctx->dev_num);
fail_device_create:
	unregister_chrdev_region(odu_bridge_ctx->dev_num, 1);
fail_alloc_chrdev_region:
	class_destroy(odu_bridge_ctx->class);
fail_class_create:
	kfree(odu_bridge_ctx);
	odu_bridge_ctx = NULL;
	return res;
}
EXPORT_SYMBOL(odu_bridge_init);

/**
 * odu_bridge_cleanup() - De-Initialize the ODU bridge driver
 *
 * Return codes: 0: success,
 *		-EINVAL - Bad parameter
 *		Other negative value - Failure
 */
int odu_bridge_cleanup(void)
{
	ODU_BRIDGE_FUNC_ENTRY();

	if (!odu_bridge_ctx) {
		ODU_BRIDGE_ERR("Not initialized\n");
		return -EFAULT;
	}

	if (odu_bridge_ctx->is_connected) {
		ODU_BRIDGE_ERR("cannot deinit while bridge is conncetd\n");
		return -EFAULT;
	}

	free_del_hnds();
	destroy_workqueue(odu_bridge_ctx->wq);
	ipa_rm_inactivity_timer_destroy(IPA_RM_RESOURCE_ODU_BRIDGE_PROD);
	ipa_rm_delete_resource(IPA_RM_RESOURCE_ODU_BRIDGE_PROD);
	odu_debugfs_destroy();
	cdev_del(&odu_bridge_ctx->cdev);
	device_destroy(odu_bridge_ctx->class, odu_bridge_ctx->dev_num);
	unregister_chrdev_region(odu_bridge_ctx->dev_num, 1);
	class_destroy(odu_bridge_ctx->class);
	kfree(odu_bridge_ctx);
	odu_bridge_ctx = NULL;

	ODU_BRIDGE_FUNC_EXIT();
	return 0;
}
EXPORT_SYMBOL(odu_bridge_cleanup);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ODU bridge driver");
