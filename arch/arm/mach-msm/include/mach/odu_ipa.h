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

#ifndef _ODU_IPA_H_
#define _ODU_IPA_H_

#include <mach/ipa.h>
#include <linux/etherdevice.h>

/*
 * struct odu_ipa_params - parameters for odu_ipa initialization API
 *
 * @odu_ipa_rx_dp_notify: odu_ipa will set this callback (out parameter).
 * this callback shall be supplied for ipa_connect upon pipe
 * connection (HSIC->IPA), once IPA driver receive data packets
 * from HSIC pipe destined for Apps this callback will be called.
 * @odu_ipa_tx_dp_notify: odu_ipa will set this callback (out parameter).
 * this callback shall be supplied for ipa_connect upon pipe
 * connection (IPA->HSIC), once IPA driver send packets destined
 * for HSIC, IPA BAM will notify for Tx-complete.
 * @device_ethaddr: device Ethernet address in network order.
 * @priv: odu_ipa will set this pointer (out parameter).
 * This pointer will hold the network device for later interaction
 * with odu_ipa APIs. Also it shall be given to ipa_connect upon pipe
 * connection as private data.
 * @tx_fixup: callback for packet fixup before sending to IPA. In case HW
 * header is set, it will be pulled from the skb prior calling to this callback.
 * @set_hw_flags: HW specific callback for setting rx flags.
 * @hw_nway_reset: HW specific callback for restart auto-negotiation.
 * Shall be provided by USB driver.
 */
struct odu_ipa_params {
	ipa_notify_cb odu_ipa_rx_dp_notify;
	ipa_notify_cb odu_ipa_tx_dp_notify;
	u8 device_ethaddr[ETH_ALEN];
	void *priv;
	int (*tx_fixup)(struct sk_buff *skb);
	void (*set_hw_rx_flags)(int flags);
	void (*hw_nway_reset)(void);
};

#ifdef CONFIG_ODU_IPA

int odu_ipa_init(struct odu_ipa_params *params);

int odu_ipa_add_hw_hdr_info(struct odu_ipa_hw_hdr_info *hw_hdr_info,
			    void *priv);

int odu_ipa_connect(u32 hsic_to_ipa_hdl, u32 ipa_to_hsic_hdl, void *priv);

int odu_ipa_disconnect(void *priv);

void odu_ipa_cleanup(void *priv);

#else /* CONFIG_ODU_IPA*/

static inline int odu_ipa_init(struct odu_ipa_params *params)
{
	return 0;
}

static inline int odu_ipa_add_hw_hdr_info(
				struct odu_ipa_hw_hdr_info *hw_hdr_info,
				void *priv)
{
	return 0;
}

static inline int odu_ipa_connect(u32 hsic_to_ipa_hdl, u32 ipa_to_hsic_hdl,
				  void *priv)
{
	return 0;
}

static inline int odu_ipa_disconnect(void *priv)
{
	return 0;
}

static inline void odu_ipa_cleanup(void *priv)
{
	return;
}

#endif /* CONFIG_ODU_IPA*/

#endif /* _ODU_IPA_H_ */
