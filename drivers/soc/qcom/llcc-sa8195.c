// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018,2020 The Linux Foundation. All rights reserved.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/llcc-qcom.h>

/*
 * SCT entry contains of the following parameters
 * uid: Unique id for the client's use case
 * slice_id: llcc slice id for each client
 * max_cap: The maximum capacity of the cache slice provided in KB
 * priority: Priority of the client used to select victim line for replacement
 * fixed_size: Determine of the slice has a fixed capacity
 * bonus_ways: Bonus ways to be used by any slice, bonus way is used only if
 *             it't not a reserved way.
 * res_ways: Reserved ways for the cache slice, the reserved ways cannot be used
 *           by any other client than the one its assigned to.
 * cache_mode: Each slice operates as a cache, this controls the mode of the
 *             slice normal or TCM
 * probe_target_ways: Determines what ways to probe for access hit. When
 *                    configured to 1 only bonus and reseved ways are probed.
 *                    when configured to 0 all ways in llcc are probed.
 * write_scid_en: Bit enables write cache support for a given scid.
 * dis_cap_alloc: Disable capacity based allocation for a client
 * retain_on_pc: If this bit is set and client has maitained active vote
 *               then the ways assigned to this client are not flushed on power
 *               collapse.
 * activate_on_init: Activate the slice immidiately after the SCT is programmed
 */
#define SCT_ENTRY(uid, sid, mc, p, fs, bway, rway, cmod, ptw, dca, wse, rp, a) \
	{	.usecase_id = uid,		\
		.slice_id = sid,		\
		.max_cap = mc,			\
		.priority = p,			\
		.fixed_size = fs,		\
		.bonus_ways = bway,		\
		.res_ways = rway,		\
		.cache_mode = cmod,		\
		.probe_target_ways = ptw,	\
		.dis_cap_alloc = dca,		\
		.write_scid_en = wse,		\
		.retain_on_pc = rp,		\
		.activate_on_init = a,		\
	}

static struct llcc_slice_config sa8195_data[] =  {
	SCT_ENTRY(LLCC_CPUSS,    1, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 1),
	SCT_ENTRY(LLCC_VIDSC0,   2, 512, 2, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_VIDSC1,   3, 512, 2, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_ROTATOR,  4, 1024, 2, 1, 0xFFF, 0x0, 2, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_VOICE,    5, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_AUDIO,    6, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_MDMHPGRW, 7, 1024, 2, 0, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_MDM,      8, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_CMPT,     10, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_GPUHTW,   11, 1024, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_GPU,      12, 5120, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_MMUHWT,   13, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 0, 1),
	SCT_ENTRY(LLCC_CMPTDMA,  15, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_DISP,     16, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_VIDFW,    17, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_MDMHPFX,  20, 1024, 2, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_MDMPNG,   21, 1024, 0, 1, 0xF,  0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_AUDHW,    22, 1024, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_NPU,      23, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_WLANHW,   24, 6144, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
	SCT_ENTRY(LLCC_PIMEM,    25, 1024, 1, 1, 0xFFF, 0x0, 0, 0, 0, 0, 1, 0),
};

static int sa8195_qcom_llcc_probe(struct platform_device *pdev)
{
	return qcom_llcc_probe(pdev, sa8195_data,
				 ARRAY_SIZE(sa8195_data));
}

static const struct of_device_id sa8195_qcom_llcc_of_match[] = {
	{ .compatible = "qcom,sa8195-llcc", },
	{ },
};

static struct platform_driver sa8195_qcom_llcc_driver = {
	.driver = {
		.name = "sa8195-llcc",
		.owner = THIS_MODULE,
		.of_match_table = sa8195_qcom_llcc_of_match,
	},
	.probe = sa8195_qcom_llcc_probe,
	.remove = qcom_llcc_remove,
};

static int __init sa8195_init_qcom_llcc_init(void)
{
	return platform_driver_register(&sa8195_qcom_llcc_driver);
}
module_init(sa8195_init_qcom_llcc_init);

static void __exit sa8195_exit_qcom_llcc_exit(void)
{
	platform_driver_unregister(&sa8195_qcom_llcc_driver);
}
module_exit(sa8195_exit_qcom_llcc_exit);

MODULE_DESCRIPTION("Qualcomm Technologies Inc SA8195 LLCC driver");
MODULE_LICENSE("GPL v2");
