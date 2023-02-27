// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/sched.h>
#include <linux/usb/dwc3-msm.h>
#include <linux/usb/composite.h>
#include "core.h"
#include "gadget.h"

/**
 * dwc3_msm_ep_inc_trb - increment a trb index.
 * @index: Pointer to the TRB index to increment.
 *
 * The index should never point to the link TRB. After incrementing,
 * if it is point to the link TRB, wrap around to the beginning. The
 * link TRB is always at the last TRB entry.
 */
static void dwc3_msm_ep_inc_trb(u8 *index)
{
	(*index)++;
	if (*index == (DWC3_TRB_NUM - 1))
		*index = 0;
}

static unsigned long dwc3_pt_reg(struct pt_regs *regs, int reg)
{
#ifdef CONFIG_ARM64
	return regs->regs[reg];
#elif CONFIG_ARM
	return regs->uregs[reg];
#endif
}
static int entry_dwc3_gadget_endpoint_transfer_in_progress
			(struct kretprobe_instance *ri,
				struct pt_regs *regs)
{
	struct dwc3_ep *dep	= (struct dwc3_ep *)dwc3_pt_reg(regs, 0);
	struct usb_ep *ep	= &dep->endpoint;
	struct dwc3_trb		*trb;
	dma_addr_t		offset;

	if (dep->gsi) {
		/*
		 * Doorbell needs to be rung with the next TRB that is going to be
		 * processed by hardware.
		 * So, if 'n'th TRB got completed then ring doorbell with (n+1) TRB.
		 */

		dwc3_msm_ep_inc_trb(&dep->trb_dequeue);
		trb = &dep->trb_pool[dep->trb_dequeue];
		offset = dwc3_trb_dma_offset(dep, trb);
		usb_gsi_ep_op(ep, (void *)&offset, GSI_EP_OP_UPDATE_DB);
	}
	return 0;
}


#define ENTRY_EXIT(name) {\
	.handler = exit_##name,\
	.entry_handler = entry_##name,\
	.maxactive = 3,\
	.kp.symbol_name = #name,\
}

#define ENTRY(name) {\
	.entry_handler = entry_##name,\
	.maxactive = 3,\
	.kp.symbol_name = #name,\
}

static struct kretprobe dwc3_msm_probes[] = {
	ENTRY(dwc3_gadget_endpoint_transfer_in_progress),
};


int dwc3_msm_kretprobe_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_msm_probes) ; i++) {
		ret = register_kretprobe(&dwc3_msm_probes[i]);
		if (ret < 0)
			pr_err("register_kretprobe failed for %s, returned %d\n",
					dwc3_msm_probes[i].kp.symbol_name, ret);
	}

	return 0;
}

void dwc3_msm_kretprobe_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_msm_probes); i++)
		unregister_kretprobe(&dwc3_msm_probes[i]);
}

