/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef	_DWMAC_QCOM_ETH_AUTOSAR_H
#define	_DWMAC_QCOM_ETH_AUTOSAR_H

struct sk_buff *ethdrv_allocatetxbuffer(size_t size);
int ethdrv_transmit(struct sk_buff *skb);
void ethwrapper_registertxcnfcallback(void(*callback) (struct sk_buff *skb, u8 result));
void ethwrapper_handletxcompletion(struct sk_buff *skb, u8 result);
void ethwrapper_handlericompletion(struct sk_buff *skb);
void ethwrapper_registerrxcnfcallback(void(*callback) (struct sk_buff *skb));
void ethdrv_getcursystime(struct timespec64 *ts);
void ethdrv_enablets(struct hwtstamp_config *config);
#endif /* _DWMAC_QCOM_ETH_AUTOSAR_H */

