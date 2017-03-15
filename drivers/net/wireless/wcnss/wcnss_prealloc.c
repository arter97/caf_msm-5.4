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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/wcnss_wlan.h>
#include <linux/spinlock.h>
#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
#include <linux/skbuff.h>
#endif

static DEFINE_SPINLOCK(alloc_lock);

struct wcnss_prealloc {
	int occupied;
	unsigned int size;
	void *ptr;
};

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
/* pre-alloced memory for skb */
static struct wcnss_prealloc wcnss_skb_allocs[] = {
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
};
#endif

/* pre-alloced mem for WLAN driver */
static struct wcnss_prealloc wcnss_allocs[] = {
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 16 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 128 * 1024, NULL},
	{0, 256 * 1024, NULL},
	{0, 256 * 1024, NULL},
};

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
int wcnss_skb_prealloc_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wcnss_skb_allocs); i++) {
		wcnss_skb_allocs[i].occupied = 0;
		wcnss_skb_allocs[i].ptr =
			dev_alloc_skb(wcnss_skb_allocs[i].size);
		if (wcnss_skb_allocs[i].ptr == NULL)
			return -ENOMEM;
	}

	return 0;
}
#else
int wcnss_skb_prealloc_init(void)
{
	return 0;
}
#endif

int wcnss_prealloc_init(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		wcnss_allocs[i].occupied = 0;
		wcnss_allocs[i].ptr = kmalloc(wcnss_allocs[i].size, GFP_KERNEL);
		if (wcnss_allocs[i].ptr == NULL)
			return -ENOMEM;
	}

	ret = wcnss_skb_prealloc_init();

	return ret;
}

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
void wcnss_skb_prealloc_deinit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wcnss_skb_allocs); i++) {
		dev_kfree_skb(wcnss_skb_allocs[i].ptr);
		wcnss_skb_allocs[i].ptr = NULL;
	}
}
#else
void wcnss_skb_prealloc_deinit(void) {}
#endif

void wcnss_prealloc_deinit(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++)
		kfree(wcnss_allocs[i].ptr);

	wcnss_skb_prealloc_deinit();
}

void *wcnss_prealloc_get(unsigned int size)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&alloc_lock, flags);
	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		if (wcnss_allocs[i].occupied)
			continue;

		if (wcnss_allocs[i].size >= size) {
			/* we found the slot */
			wcnss_allocs[i].occupied = 1;
			spin_unlock_irqrestore(&alloc_lock, flags);
			return wcnss_allocs[i].ptr;
		}
	}
	spin_unlock_irqrestore(&alloc_lock, flags);

	return NULL;
}
EXPORT_SYMBOL(wcnss_prealloc_get);

int wcnss_prealloc_put(void *ptr)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&alloc_lock, flags);
	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		if (wcnss_allocs[i].ptr == ptr) {
			wcnss_allocs[i].occupied = 0;
			spin_unlock_irqrestore(&alloc_lock, flags);
			return 1;
		}
	}
	spin_unlock_irqrestore(&alloc_lock, flags);

	return 0;
}
EXPORT_SYMBOL(wcnss_prealloc_put);

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
struct sk_buff *wcnss_skb_prealloc_get(unsigned int size)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&alloc_lock, flags);
	for (i = 0; i < ARRAY_SIZE(wcnss_skb_allocs); i++) {
		if (wcnss_skb_allocs[i].occupied)
			continue;

		if (wcnss_skb_allocs[i].size > size) {
			/* we found the slot */
			wcnss_skb_allocs[i].occupied = 1;
			spin_unlock_irqrestore(&alloc_lock, flags);
			return wcnss_skb_allocs[i].ptr;
		}
	}
	spin_unlock_irqrestore(&alloc_lock, flags);

	pr_err("wcnss: %s: prealloc not available for size: %d\n",
		__func__, size);

	return NULL;
}
EXPORT_SYMBOL(wcnss_skb_prealloc_get);

int wcnss_skb_prealloc_put(struct sk_buff *skb)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&alloc_lock, flags);
	for (i = 0; i < ARRAY_SIZE(wcnss_skb_allocs); i++) {
		if (wcnss_skb_allocs[i].ptr == skb) {
			wcnss_skb_allocs[i].occupied = 0;
			spin_unlock_irqrestore(&alloc_lock, flags);
			return 1;
		}
	}
	spin_unlock_irqrestore(&alloc_lock, flags);

	return 0;
}
EXPORT_SYMBOL(wcnss_skb_prealloc_put);
#endif

#ifdef CONFIG_WCNSS_SKB_PRE_ALLOC
/* Reset socket buffer pre-allock memory pool */
int wcnss_skb_pre_alloc_reset(void)
{
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_skb_allocs); i++) {
		if (!wcnss_skb_allocs[i].occupied)
			continue;

		wcnss_skb_allocs[i].occupied = 0;
		n++;
	}

	return n;
}
#else
int wcnss_skb_pre_alloc_reset(void)
{
	return 0;
}
#endif
