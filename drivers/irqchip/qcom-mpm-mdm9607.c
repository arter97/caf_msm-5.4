// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <soc/qcom/mpm.h>

const struct mpm_pin mpm_mdm9607_gic_chip_data[] = {
	{2, 184}, /* tsens_upper_lower_int */
	{49, 140}, /* usb1_hs_async_wakeup_irq */
	{51, 142}, /* usb2_hs_async_wakeup_irq */
	{53, 72}, /* mdss_irq */
	{58, 134}, /* usb_hs_irq */
	{62, 190}, /* ee0_apps_hlos_spmi_periph_irq */
	{-1},
};
