/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 OpenSynergy GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define COQOSHV_SMC_ARGS_NUM 8

/* 64 byte */
struct coqoshv_smc_mailbox {
        __u64 args[COQOSHV_SMC_ARGS_NUM];
} __packed;

void coqoshv_call_qcpe(const struct coqoshv_smc_mailbox *smc,
		       struct arm_smccc_res *res, const bool atomic);
