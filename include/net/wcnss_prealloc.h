/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#ifndef _WCNSS_PRE_ALLOC_H_
#define _WCNSS_PRE_ALLOC_H_

#define WCNSS_PRE_ALLOC_GET_THRESHOLD (4*1024)
#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
#define WCNSS_PRE_SKB_ALLOC_GET_THRESHOLD (50*1024)
#endif
int wcnss_prealloc_init(void);
void wcnss_prealloc_deinit(void);
extern void *wcnss_prealloc_get(unsigned int size);
extern int wcnss_prealloc_put(void *ptr);

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
extern struct sk_buff *wcnss_skb_prealloc_get(unsigned int size);
extern int wcnss_skb_prealloc_put(struct sk_buff *skb);
extern int wcnss_skb_pre_alloc_reset(void);
#endif

#endif/* _WCNSS_PRE_ALLOC_H_ */
