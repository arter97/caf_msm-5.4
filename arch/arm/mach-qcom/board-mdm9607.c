// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include "board-dt.h"

static const char *mdm9607_dt_match[] __initconst = {
	"qcom,mdm9607",
	NULL
};

static void __init mdm9607_init(void)
{
	board_dt_populate(NULL);
}

DT_MACHINE_START(MDM9607_DT,
	"Qualcomm Technologies, Inc. MDM 9607 (Flattened Device Tree)")
	.init_machine		= mdm9607_init,
	.dt_compat		= mdm9607_dt_match,
MACHINE_END
