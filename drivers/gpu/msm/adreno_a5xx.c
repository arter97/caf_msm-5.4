/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/firmware.h>

#include "adreno.h"
#include "a5xx_reg.h"
#include "adreno_a3xx.h"
#include "adreno_cp_parser.h"
#include "adreno_trace.h"
#include "adreno_pm4types.h"
#include "kgsl_sharedmem.h"
#include "kgsl_log.h"
#include "kgsl.h"

static const struct adreno_vbif_data a530_vbif[] = {
	{0, 0},
};

static const struct adreno_vbif_platform a5xx_vbif_platforms[] = {
	{ adreno_is_a530, a530_vbif },
};

/**
 * a5xx_protect_init() - Initializes register protection on a5xx
 * @device: Pointer to the device structure
 * Performs register writes to enable protected access to sensitive
 * registers
 */
static void a5xx_protect_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	int index = 0;
	struct kgsl_protected_registers *iommu_regs;

	/* enable access protection to privileged registers */
	kgsl_regwrite(device, A5XX_CP_PROTECT_CNTL, 0x00000007);

	/* RBBM registers */
	adreno_set_protected_registers(adreno_dev, &index, 0x4, 2);
	adreno_set_protected_registers(adreno_dev, &index, 0x8, 3);
	adreno_set_protected_registers(adreno_dev, &index, 0x10, 4);
	adreno_set_protected_registers(adreno_dev, &index, 0x20, 5);
	adreno_set_protected_registers(adreno_dev, &index, 0x40, 6);
	adreno_set_protected_registers(adreno_dev, &index, 0x80, 6);

	/* Content protection registers */
	adreno_set_protected_registers(adreno_dev, &index,
		   A5XX_RBBM_SECVID_TSB_TRUSTED_BASE_LO, 4);
	adreno_set_protected_registers(adreno_dev, &index,
		   A5XX_RBBM_SECVID_TRUST_CNTL, 1);

	/* CP registers */
	adreno_set_protected_registers(adreno_dev, &index, 0x800, 6);
	adreno_set_protected_registers(adreno_dev, &index, 0x840, 3);
	adreno_set_protected_registers(adreno_dev, &index, 0x880, 5);
	adreno_set_protected_registers(adreno_dev, &index, 0x0AA0, 0);

	/* RB registers */
	adreno_set_protected_registers(adreno_dev, &index, 0xCC0, 0);
	adreno_set_protected_registers(adreno_dev, &index, 0xCF0, 1);

	/* HLSQ registers */
	adreno_set_protected_registers(adreno_dev, &index, 0xE00, 2);

	/* VPC registers */
	adreno_set_protected_registers(adreno_dev, &index, 0xE68, 3);
	adreno_set_protected_registers(adreno_dev, &index, 0xE70, 4);

	/* SMMU registers */
	iommu_regs = kgsl_mmu_get_prot_regs(&device->mmu);
	if (iommu_regs)
		adreno_set_protected_registers(adreno_dev, &index,
				iommu_regs->base, iommu_regs->range);
}

/*
 * a5xx_start() - Device start
 * @adreno_dev: Pointer to adreno device
 *
 * a5xx device start
 */
static void a5xx_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	uint val = 0;

	adreno_vbif_start(adreno_dev, a5xx_vbif_platforms,
			ARRAY_SIZE(a5xx_vbif_platforms));

	/* GPU comes up in secured mode, make it unsecured by default */
	kgsl_regwrite(device, A5XX_RBBM_SECVID_TRUST_CNTL, 0x0);

	/* Make all blocks contribute to the GPU BUSY perf counter */
	kgsl_regwrite(device, A5XX_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xFFFFFFFF);

	/*
	 * Enable the RBBM error reporting bits.  This lets us get
	 * useful information on failure
	 */
	kgsl_regwrite(device, A5XX_RBBM_AHB_CNTL0, 0x00000001);

	/* Enable AHB error reporting */
	kgsl_regwrite(device, A5XX_RBBM_AHB_CNTL1, 0xA6FFFFFF);
	kgsl_regwrite(device, A5XX_RBBM_AHB_CNTL2, 0x0000003F);

	/*
	 * Turn on hang detection - this spews a lot of useful information
	 * into the RBBM registers on a hang
	 */
	kgsl_regwrite(device, A5XX_RBBM_INTERFACE_HANG_INT_CNTL, 0xFFFF);

	/* Turn on performance counters */
	kgsl_regwrite(device, A5XX_RBBM_PERFCTR_CNTL, 0x01);

	memset(&adreno_dev->busy_data, 0, sizeof(adreno_dev->busy_data));

	/* Disable L2 bypass to avoid UCHE out of bounds errors */
	kgsl_regwrite(device, A5XX_UCHE_TRAP_BASE_LO, 0xffff0000);
	kgsl_regwrite(device, A5XX_UCHE_TRAP_BASE_HI, 0xffff0000);

	/*
	 * Below CP registers are 0x0 by default, program init
	 * values based on a5xx flavor.
	 */
	kgsl_regwrite(device, A5XX_CP_MEQ_THRESHOLDS, 0x40);
	kgsl_regwrite(device, A5XX_CP_MERCIU_SIZE, 0x40);
	kgsl_regwrite(device, A5XX_CP_ROQ_THRESHOLDS_2, 0x80000060);
	kgsl_regwrite(device, A5XX_CP_ROQ_THRESHOLDS_1, 0x40201B16);
	kgsl_regwrite(device, A5XX_PC_DBG_ECO_CNTL,
					(0x400 << 11 | 0x300 << 22));

	/*
	 * A5x USP LDST non valid pixel wrongly update read combine offset
	 * In A5xx we added optimization for read combine. There could be cases
	 * on a530 v1 there is no valid pixel but the active masks is not
	 * cleared and the offset can be wrongly updated if the invalid address
	 * can be combined. The wrongly latched value will make the returning
	 * data got shifted at wrong offset. workaround this issue by disabling
	 * LD combine, bit[25] of SP_DBG_ECO_CNTL (sp chicken bit[17]) need to
	 * be set to 1, default is 0(enable)
	 */
	if (adreno_is_a530v1(adreno_dev)) {
		kgsl_regread(device, A5XX_SP_DBG_ECO_CNTL, &val);
		val = (val | 1 << 25);
		kgsl_regwrite(device, A5XX_SP_DBG_ECO_CNTL, val);
	}

	a5xx_protect_init(adreno_dev);
}

/*
 * a5xx_rb_init() - Initialize ringbuffer
 * @adreno_dev: Pointer to adreno device
 * @rb: Pointer to the ringbuffer of device
 *
 * Submit commands for ME initialization,
 */
int a5xx_rb_init(struct adreno_device *adreno_dev,
			 struct adreno_ringbuffer *rb)
{
	unsigned int *cmds;
	cmds = adreno_ringbuffer_allocspace(rb, 8);
	if (IS_ERR(cmds))
		return PTR_ERR(cmds);
	if (cmds == NULL)
		return -ENOSPC;

	*cmds++ = cp_type7_packet(CP_ME_INIT, 8);
	/*
	 *  Mask -- look for all ordinals but drawcall
	 *  range and reset ucode scratch memory.
	 */
	*cmds++ = 0x0000000f;
	/* Multiple HW ctxs are unreliable on a530v1, use single hw context */
	if (adreno_is_a530v1(adreno_dev))
		*cmds++ = 0x00000000;
	else
		/* Use both contexts for 3D (bit0) 2D (bit1) */
		*cmds++ = 0x00000003;
	/* Enable register protection */
	*cmds++ = 0x20000000;
	/* Header dump address */
	*cmds++ = 0x00000000;
	/* Header dump enable and dump size */
	*cmds++ = 0x00000000;
	/* Below will be ignored by the CP unless bit4 in Mask is set */
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;

	adreno_ringbuffer_submit(rb, NULL);

	return 0;
}

static int _load_firmware(struct adreno_device *adreno_dev, const char *fwfile,
			  struct kgsl_memdesc *ucode)
{
	struct kgsl_device *device = &adreno_dev->dev;
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, fwfile, device->dev);

	if (ret) {
		KGSL_DRV_FATAL(device, "request_firmware(%s) failed: %d\n",
				fwfile, ret);
		return ret;
	}

	ret = kgsl_allocate_global(device, ucode, fw->size,
				KGSL_MEMFLAGS_GPUREADONLY, 0);

	if (ret)
		return ret;

	memcpy(ucode->hostptr, &fw->data[4], fw->size);

	release_firmware(fw);

	return 0;
}

/*
 * a5xx_microcode_read() - Read microcode
 * @adreno_dev: Pointer to adreno device
 */
void a5xx_microcode_read(struct adreno_device *adreno_dev)
{
	int ret;

	ret = _load_firmware(adreno_dev,
			 adreno_dev->gpucore->pm4fw_name, &adreno_dev->pm4);
	if (ret)
		return;

	ret = _load_firmware(adreno_dev,
			 adreno_dev->gpucore->pfpfw_name, &adreno_dev->pfp);
	if (ret)
		return;
}

/*
 * a5xx_microcode_load() - Load microcode
 * @adreno_dev: Pointer to adreno device
 * @start_type: type of device start cold/warm
 */
int a5xx_microcode_load(struct adreno_device *adreno_dev,
						unsigned int start_type)
{
	struct kgsl_device *device = &adreno_dev->dev;
	uint64_t gpuaddr;

	gpuaddr = adreno_dev->pm4.gpuaddr;
	kgsl_regwrite(device, A5XX_CP_PM4_INSTR_BASE_LO,
				(uint)gpuaddr);
	kgsl_regwrite(device, A5XX_CP_PM4_INSTR_BASE_HI,
				((uint64_t)(gpuaddr) >> 32));

	gpuaddr = adreno_dev->pfp.gpuaddr;
	kgsl_regwrite(device, A5XX_CP_PFP_INSTR_BASE_LO,
				(uint)gpuaddr);
	kgsl_regwrite(device, A5XX_CP_PFP_INSTR_BASE_HI,
				((uint64_t)(gpuaddr) >> 32));

	return 0;
}

static struct adreno_perfcount_register a5xx_perfcounters_cp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_0_LO,
		A5XX_RBBM_PERFCTR_CP_0_HI, 0, A5XX_CP_PERFCTR_CP_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_1_LO,
		A5XX_RBBM_PERFCTR_CP_1_HI, 0, A5XX_CP_PERFCTR_CP_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_2_LO,
		A5XX_RBBM_PERFCTR_CP_2_HI, 0, A5XX_CP_PERFCTR_CP_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_3_LO,
		A5XX_RBBM_PERFCTR_CP_3_HI, 0, A5XX_CP_PERFCTR_CP_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_4_LO,
		A5XX_RBBM_PERFCTR_CP_4_HI, 0, A5XX_CP_PERFCTR_CP_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_5_LO,
		A5XX_RBBM_PERFCTR_CP_5_HI, 0, A5XX_CP_PERFCTR_CP_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_6_LO,
		A5XX_RBBM_PERFCTR_CP_6_HI, 0, A5XX_CP_PERFCTR_CP_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CP_7_LO,
		A5XX_RBBM_PERFCTR_CP_7_HI, 0, A5XX_CP_PERFCTR_CP_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_rbbm[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RBBM_0_LO,
		A5XX_RBBM_PERFCTR_RBBM_0_HI, 8, A5XX_RBBM_PERFCTR_RBBM_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RBBM_1_LO,
		A5XX_RBBM_PERFCTR_RBBM_1_HI, 9, A5XX_RBBM_PERFCTR_RBBM_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RBBM_2_LO,
		A5XX_RBBM_PERFCTR_RBBM_2_HI, 10, A5XX_RBBM_PERFCTR_RBBM_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RBBM_3_LO,
		A5XX_RBBM_PERFCTR_RBBM_3_HI, 11, A5XX_RBBM_PERFCTR_RBBM_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_pc[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_0_LO,
		A5XX_RBBM_PERFCTR_PC_0_HI, 12, A5XX_PC_PERFCTR_PC_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_1_LO,
		A5XX_RBBM_PERFCTR_PC_1_HI, 13, A5XX_PC_PERFCTR_PC_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_2_LO,
		A5XX_RBBM_PERFCTR_PC_2_HI, 14, A5XX_PC_PERFCTR_PC_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_3_LO,
		A5XX_RBBM_PERFCTR_PC_3_HI, 15, A5XX_PC_PERFCTR_PC_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_4_LO,
		A5XX_RBBM_PERFCTR_PC_4_HI, 16, A5XX_PC_PERFCTR_PC_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_5_LO,
		A5XX_RBBM_PERFCTR_PC_5_HI, 17, A5XX_PC_PERFCTR_PC_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_6_LO,
		A5XX_RBBM_PERFCTR_PC_6_HI, 18, A5XX_PC_PERFCTR_PC_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_PC_7_LO,
		A5XX_RBBM_PERFCTR_PC_7_HI, 19, A5XX_PC_PERFCTR_PC_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_vfd[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_0_LO,
		A5XX_RBBM_PERFCTR_VFD_0_HI, 20, A5XX_VFD_PERFCTR_VFD_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_1_LO,
		A5XX_RBBM_PERFCTR_VFD_1_HI, 21, A5XX_VFD_PERFCTR_VFD_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_2_LO,
		A5XX_RBBM_PERFCTR_VFD_2_HI, 22, A5XX_VFD_PERFCTR_VFD_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_3_LO,
		A5XX_RBBM_PERFCTR_VFD_3_HI, 23, A5XX_VFD_PERFCTR_VFD_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_4_LO,
		A5XX_RBBM_PERFCTR_VFD_4_HI, 24, A5XX_VFD_PERFCTR_VFD_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_5_LO,
		A5XX_RBBM_PERFCTR_VFD_5_HI, 25, A5XX_VFD_PERFCTR_VFD_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_6_LO,
		A5XX_RBBM_PERFCTR_VFD_6_HI, 26, A5XX_VFD_PERFCTR_VFD_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VFD_7_LO,
		A5XX_RBBM_PERFCTR_VFD_7_HI, 27, A5XX_VFD_PERFCTR_VFD_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_hlsq[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_0_LO,
		A5XX_RBBM_PERFCTR_HLSQ_0_HI, 28, A5XX_HLSQ_PERFCTR_HLSQ_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_1_LO,
		A5XX_RBBM_PERFCTR_HLSQ_1_HI, 29, A5XX_HLSQ_PERFCTR_HLSQ_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_2_LO,
		A5XX_RBBM_PERFCTR_HLSQ_2_HI, 30, A5XX_HLSQ_PERFCTR_HLSQ_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_3_LO,
		A5XX_RBBM_PERFCTR_HLSQ_3_HI, 31, A5XX_HLSQ_PERFCTR_HLSQ_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_4_LO,
		A5XX_RBBM_PERFCTR_HLSQ_4_HI, 32, A5XX_HLSQ_PERFCTR_HLSQ_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_5_LO,
		A5XX_RBBM_PERFCTR_HLSQ_5_HI, 33, A5XX_HLSQ_PERFCTR_HLSQ_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_6_LO,
		A5XX_RBBM_PERFCTR_HLSQ_6_HI, 34, A5XX_HLSQ_PERFCTR_HLSQ_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_HLSQ_7_LO,
		A5XX_RBBM_PERFCTR_HLSQ_7_HI, 35, A5XX_HLSQ_PERFCTR_HLSQ_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_vpc[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VPC_0_LO,
		A5XX_RBBM_PERFCTR_VPC_0_HI, 36, A5XX_VPC_PERFCTR_VPC_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VPC_1_LO,
		A5XX_RBBM_PERFCTR_VPC_1_HI, 37, A5XX_VPC_PERFCTR_VPC_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VPC_2_LO,
		A5XX_RBBM_PERFCTR_VPC_2_HI, 38, A5XX_VPC_PERFCTR_VPC_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VPC_3_LO,
		A5XX_RBBM_PERFCTR_VPC_3_HI, 39, A5XX_VPC_PERFCTR_VPC_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_ccu[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CCU_0_LO,
		A5XX_RBBM_PERFCTR_CCU_0_HI, 40, A5XX_RB_PERFCTR_CCU_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CCU_1_LO,
		A5XX_RBBM_PERFCTR_CCU_1_HI, 41, A5XX_RB_PERFCTR_CCU_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CCU_2_LO,
		A5XX_RBBM_PERFCTR_CCU_2_HI, 42, A5XX_RB_PERFCTR_CCU_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CCU_3_LO,
		A5XX_RBBM_PERFCTR_CCU_3_HI, 43, A5XX_RB_PERFCTR_CCU_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_tse[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TSE_0_LO,
		A5XX_RBBM_PERFCTR_TSE_0_HI, 44, A5XX_GRAS_PERFCTR_TSE_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TSE_1_LO,
		A5XX_RBBM_PERFCTR_TSE_1_HI, 45, A5XX_GRAS_PERFCTR_TSE_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TSE_2_LO,
		A5XX_RBBM_PERFCTR_TSE_2_HI, 46, A5XX_GRAS_PERFCTR_TSE_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TSE_3_LO,
		A5XX_RBBM_PERFCTR_TSE_3_HI, 47, A5XX_GRAS_PERFCTR_TSE_SEL_3 },
};


static struct adreno_perfcount_register a5xx_perfcounters_ras[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RAS_0_LO,
		A5XX_RBBM_PERFCTR_RAS_0_HI, 48, A5XX_GRAS_PERFCTR_RAS_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RAS_1_LO,
		A5XX_RBBM_PERFCTR_RAS_1_HI, 49, A5XX_GRAS_PERFCTR_RAS_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RAS_2_LO,
		A5XX_RBBM_PERFCTR_RAS_2_HI, 50, A5XX_GRAS_PERFCTR_RAS_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RAS_3_LO,
		A5XX_RBBM_PERFCTR_RAS_3_HI, 51, A5XX_GRAS_PERFCTR_RAS_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_lrz[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_LRZ_0_LO,
		A5XX_RBBM_PERFCTR_LRZ_0_HI, 48, A5XX_GRAS_PERFCTR_LRZ_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_LRZ_1_LO,
		A5XX_RBBM_PERFCTR_LRZ_1_HI, 49, A5XX_GRAS_PERFCTR_LRZ_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_LRZ_2_LO,
		A5XX_RBBM_PERFCTR_LRZ_2_HI, 50, A5XX_GRAS_PERFCTR_LRZ_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_LRZ_3_LO,
		A5XX_RBBM_PERFCTR_LRZ_3_HI, 51, A5XX_GRAS_PERFCTR_LRZ_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_uche[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_0_LO,
		A5XX_RBBM_PERFCTR_UCHE_0_HI, 52, A5XX_UCHE_PERFCTR_UCHE_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_1_LO,
		A5XX_RBBM_PERFCTR_UCHE_1_HI, 53, A5XX_UCHE_PERFCTR_UCHE_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_2_LO,
		A5XX_RBBM_PERFCTR_UCHE_2_HI, 54, A5XX_UCHE_PERFCTR_UCHE_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_3_LO,
		A5XX_RBBM_PERFCTR_UCHE_3_HI, 55, A5XX_UCHE_PERFCTR_UCHE_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_4_LO,
		A5XX_RBBM_PERFCTR_UCHE_4_HI, 56, A5XX_UCHE_PERFCTR_UCHE_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_5_LO,
		A5XX_RBBM_PERFCTR_UCHE_5_HI, 57, A5XX_UCHE_PERFCTR_UCHE_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_6_LO,
		A5XX_RBBM_PERFCTR_UCHE_6_HI, 58, A5XX_UCHE_PERFCTR_UCHE_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_UCHE_7_LO,
		A5XX_RBBM_PERFCTR_UCHE_7_HI, 59, A5XX_UCHE_PERFCTR_UCHE_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_tp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_0_LO,
		A5XX_RBBM_PERFCTR_TP_0_HI, 60, A5XX_TPL1_PERFCTR_TP_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_1_LO,
		A5XX_RBBM_PERFCTR_TP_1_HI, 61, A5XX_TPL1_PERFCTR_TP_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_2_LO,
		A5XX_RBBM_PERFCTR_TP_2_HI, 62, A5XX_TPL1_PERFCTR_TP_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_3_LO,
		A5XX_RBBM_PERFCTR_TP_3_HI, 63, A5XX_TPL1_PERFCTR_TP_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_4_LO,
		A5XX_RBBM_PERFCTR_TP_4_HI, 64, A5XX_TPL1_PERFCTR_TP_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_5_LO,
		A5XX_RBBM_PERFCTR_TP_5_HI, 65, A5XX_TPL1_PERFCTR_TP_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_6_LO,
		A5XX_RBBM_PERFCTR_TP_6_HI, 66, A5XX_TPL1_PERFCTR_TP_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_TP_7_LO,
		A5XX_RBBM_PERFCTR_TP_7_HI, 67, A5XX_TPL1_PERFCTR_TP_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_sp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_0_LO,
		A5XX_RBBM_PERFCTR_SP_0_HI, 68, A5XX_SP_PERFCTR_SP_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_1_LO,
		A5XX_RBBM_PERFCTR_SP_1_HI, 69, A5XX_SP_PERFCTR_SP_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_2_LO,
		A5XX_RBBM_PERFCTR_SP_2_HI, 70, A5XX_SP_PERFCTR_SP_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_3_LO,
		A5XX_RBBM_PERFCTR_SP_3_HI, 71, A5XX_SP_PERFCTR_SP_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_4_LO,
		A5XX_RBBM_PERFCTR_SP_4_HI, 72, A5XX_SP_PERFCTR_SP_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_5_LO,
		A5XX_RBBM_PERFCTR_SP_5_HI, 73, A5XX_SP_PERFCTR_SP_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_6_LO,
		A5XX_RBBM_PERFCTR_SP_6_HI, 74, A5XX_SP_PERFCTR_SP_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_7_LO,
		A5XX_RBBM_PERFCTR_SP_7_HI, 75, A5XX_SP_PERFCTR_SP_SEL_7 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_8_LO,
		A5XX_RBBM_PERFCTR_SP_8_HI, 76, A5XX_SP_PERFCTR_SP_SEL_8 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_9_LO,
		A5XX_RBBM_PERFCTR_SP_9_HI, 77, A5XX_SP_PERFCTR_SP_SEL_9 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_10_LO,
		A5XX_RBBM_PERFCTR_SP_10_HI, 78, A5XX_SP_PERFCTR_SP_SEL_10 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_SP_11_LO,
		A5XX_RBBM_PERFCTR_SP_11_HI, 79, A5XX_SP_PERFCTR_SP_SEL_11 },
};

static struct adreno_perfcount_register a5xx_perfcounters_rb[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_0_LO,
		A5XX_RBBM_PERFCTR_RB_0_HI, 80, A5XX_RB_PERFCTR_RB_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_1_LO,
		A5XX_RBBM_PERFCTR_RB_1_HI, 81, A5XX_RB_PERFCTR_RB_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_2_LO,
		A5XX_RBBM_PERFCTR_RB_2_HI, 82, A5XX_RB_PERFCTR_RB_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_3_LO,
		A5XX_RBBM_PERFCTR_RB_3_HI, 83, A5XX_RB_PERFCTR_RB_SEL_3 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_4_LO,
		A5XX_RBBM_PERFCTR_RB_4_HI, 84, A5XX_RB_PERFCTR_RB_SEL_4 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_5_LO,
		A5XX_RBBM_PERFCTR_RB_5_HI, 85, A5XX_RB_PERFCTR_RB_SEL_5 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_6_LO,
		A5XX_RBBM_PERFCTR_RB_6_HI, 86, A5XX_RB_PERFCTR_RB_SEL_6 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_RB_7_LO,
		A5XX_RBBM_PERFCTR_RB_7_HI, 87, A5XX_RB_PERFCTR_RB_SEL_7 },
};

static struct adreno_perfcount_register a5xx_perfcounters_cmp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CMP_0_LO,
		A5XX_RBBM_PERFCTR_CMP_0_HI, 48, A5XX_RB_PERFCTR_CMP_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CMP_1_LO,
		A5XX_RBBM_PERFCTR_CMP_1_HI, 49, A5XX_RB_PERFCTR_CMP_SEL_1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CMP_2_LO,
		A5XX_RBBM_PERFCTR_CMP_2_HI, 50, A5XX_RB_PERFCTR_CMP_SEL_2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_CMP_3_LO,
		A5XX_RBBM_PERFCTR_CMP_3_HI, 51, A5XX_RB_PERFCTR_CMP_SEL_3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_vsc[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VSC_0_LO,
		A5XX_RBBM_PERFCTR_VSC_0_HI, 88, A5XX_VSC_PERFCTR_VSC_SEL_0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_PERFCTR_VSC_1_LO,
		A5XX_RBBM_PERFCTR_VSC_1_HI, 89, A5XX_VSC_PERFCTR_VSC_SEL_1 },
};

static struct adreno_perfcount_register a5xx_perfcounters_vbif[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_CNT_LOW0,
		A5XX_VBIF_PERF_CNT_HIGH0, -1, A5XX_VBIF_PERF_CNT_SEL0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_CNT_LOW1,
		A5XX_VBIF_PERF_CNT_HIGH1, -1, A5XX_VBIF_PERF_CNT_SEL1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_CNT_LOW2,
		A5XX_VBIF_PERF_CNT_HIGH2, -1, A5XX_VBIF_PERF_CNT_SEL2 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_CNT_LOW3,
		A5XX_VBIF_PERF_CNT_HIGH3, -1, A5XX_VBIF_PERF_CNT_SEL3 },
};

static struct adreno_perfcount_register a5xx_perfcounters_vbif_pwr[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_PWR_CNT_LOW0,
		A5XX_VBIF_PERF_PWR_CNT_HIGH0, -1, A5XX_VBIF_PERF_PWR_CNT_EN0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_PWR_CNT_LOW1,
		A5XX_VBIF_PERF_PWR_CNT_HIGH1, -1, A5XX_VBIF_PERF_PWR_CNT_EN1 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_VBIF_PERF_PWR_CNT_LOW2,
		A5XX_VBIF_PERF_PWR_CNT_HIGH2, -1, A5XX_VBIF_PERF_PWR_CNT_EN2 },
};

static struct adreno_perfcount_register a5xx_perfcounters_alwayson[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A5XX_RBBM_ALWAYSON_COUNTER_LO,
		A5XX_RBBM_ALWAYSON_COUNTER_HI, 0 },
};

#define A5XX_PERFCOUNTER_GROUP(offset, name) \
	ADRENO_PERFCOUNTER_GROUP(a5xx, offset, name)

#define A5XX_PERFCOUNTER_GROUP_FLAGS(offset, name, flags) \
	ADRENO_PERFCOUNTER_GROUP_FLAGS(a5xx, offset, name, flags)

static struct adreno_perfcount_group a5xx_perfcounter_groups
				[KGSL_PERFCOUNTER_GROUP_MAX] = {
	A5XX_PERFCOUNTER_GROUP(CP, cp),
	A5XX_PERFCOUNTER_GROUP(RBBM, rbbm),
	A5XX_PERFCOUNTER_GROUP(PC, pc),
	A5XX_PERFCOUNTER_GROUP(VFD, vfd),
	A5XX_PERFCOUNTER_GROUP(HLSQ, hlsq),
	A5XX_PERFCOUNTER_GROUP(VPC, vpc),
	A5XX_PERFCOUNTER_GROUP(CCU, ccu),
	A5XX_PERFCOUNTER_GROUP(CMP, cmp),
	A5XX_PERFCOUNTER_GROUP(TSE, tse),
	A5XX_PERFCOUNTER_GROUP(RAS, ras),
	A5XX_PERFCOUNTER_GROUP(LRZ, lrz),
	A5XX_PERFCOUNTER_GROUP(UCHE, uche),
	A5XX_PERFCOUNTER_GROUP(TP, tp),
	A5XX_PERFCOUNTER_GROUP(SP, sp),
	A5XX_PERFCOUNTER_GROUP(RB, rb),
	A5XX_PERFCOUNTER_GROUP(VSC, vsc),
	A5XX_PERFCOUNTER_GROUP(VBIF, vbif),
	A5XX_PERFCOUNTER_GROUP_FLAGS(VBIF_PWR, vbif_pwr,
		ADRENO_PERFCOUNTER_GROUP_FIXED),
	A5XX_PERFCOUNTER_GROUP_FLAGS(ALWAYSON, alwayson,
		ADRENO_PERFCOUNTER_GROUP_FIXED),
};

static struct adreno_perfcounters a5xx_perfcounters = {
	a5xx_perfcounter_groups,
	ARRAY_SIZE(a5xx_perfcounter_groups),
};

struct adreno_ft_perf_counters a5xx_ft_perf_counters[] = {
	{KGSL_PERFCOUNTER_GROUP_SP, A5XX_SP_ALU_ACTIVE_CYCLES},
	{KGSL_PERFCOUNTER_GROUP_SP, A5XX_SP0_ICL1_MISSES},
	{KGSL_PERFCOUNTER_GROUP_SP, A5XX_SP_FS_CFLOW_INSTRUCTIONS},
	{KGSL_PERFCOUNTER_GROUP_TSE, A5XX_TSE_INPUT_PRIM_NUM},
};

/* Register offset defines for A5XX, in order of enum adreno_regs */
static unsigned int a5xx_register_offsets[ADRENO_REG_REGISTER_MAX] = {
	ADRENO_REG_DEFINE(ADRENO_REG_CP_WFI_PEND_CTR, A5XX_CP_WFI_PEND_CTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_BASE, A5XX_CP_RB_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_BASE_HI, A5XX_CP_RB_BASE_HI),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_RPTR, A5XX_CP_RB_RPTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_WPTR, A5XX_CP_RB_WPTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ME_CNTL, A5XX_CP_ME_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_CNTL, A5XX_CP_RB_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BASE, A5XX_CP_IB1_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BASE_HI, A5XX_CP_IB1_BASE_HI),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BUFSZ, A5XX_CP_IB1_BUFSZ),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BASE, A5XX_CP_IB2_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BASE_HI, A5XX_CP_IB2_BASE_HI),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BUFSZ, A5XX_CP_IB2_BUFSZ),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ROQ_ADDR, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ROQ_DATA, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_MERCIU_ADDR, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_MERCIU_DATA, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_MERCIU_DATA2, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_MEQ_ADDR, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_MEQ_DATA, ADRENO_REG_UNUSED),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_PROTECT_REG_0, A5XX_CP_PROTECT_REG_0),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_STATUS, A5XX_RBBM_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_CTL, A5XX_RBBM_PERFCTR_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD0,
					A5XX_RBBM_PERFCTR_LOAD_CMD0),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD1,
					A5XX_RBBM_PERFCTR_LOAD_CMD1),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD2,
					A5XX_RBBM_PERFCTR_LOAD_CMD2),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD3,
					A5XX_RBBM_PERFCTR_LOAD_CMD3),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_0_MASK, A5XX_RBBM_INT_0_MASK),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_0_STATUS, A5XX_RBBM_INT_0_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_CLOCK_CTL, A5XX_RBBM_CLOCK_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_CLEAR_CMD,
				A5XX_RBBM_INT_CLEAR_CMD),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SW_RESET_CMD, A5XX_RBBM_SW_RESET_CMD),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_BLOCK_SW_RESET_CMD,
					  A5XX_RBBM_BLOCK_SW_RESET_CMD),
		ADRENO_REG_DEFINE(ADRENO_REG_RBBM_BLOCK_SW_RESET_CMD2,
					  A5XX_RBBM_BLOCK_SW_RESET_CMD2),
	ADRENO_REG_DEFINE(ADRENO_REG_UCHE_INVALIDATE0, A5XX_UCHE_INVALIDATE0),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_VALUE_LO,
				A5XX_RBBM_PERFCTR_LOAD_VALUE_LO),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_VALUE_HI,
				A5XX_RBBM_PERFCTR_LOAD_VALUE_HI),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SECVID_TRUST_CONTROL,
				A5XX_RBBM_SECVID_TRUST_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SECVID_TRUST_CONFIG,
				A5XX_RBBM_SECVID_TRUST_CONFIG),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SECVID_TSB_CONTROL,
				A5XX_RBBM_SECVID_TSB_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SECVID_TSB_TRUSTED_BASE,
				A5XX_RBBM_SECVID_TSB_TRUSTED_BASE_LO),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SECVID_TSB_TRUSTED_SIZE,
				A5XX_RBBM_SECVID_TSB_TRUSTED_SIZE),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_ALWAYSON_COUNTER_LO,
				A5XX_RBBM_ALWAYSON_COUNTER_LO),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_ALWAYSON_COUNTER_HI,
				A5XX_RBBM_ALWAYSON_COUNTER_HI),

};

const struct adreno_reg_offsets a5xx_reg_offsets = {
	.offsets = a5xx_register_offsets,
	.offset_0 = ADRENO_REG_REGISTER_MAX,
};

void a5xx_err_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = &adreno_dev->dev;
	unsigned int reg;

	switch (bit) {
	case A5XX_INT_RBBM_AHB_ERROR: {
		kgsl_regread(device, A5XX_RBBM_AHB_ERROR_STATUS, &reg);

		/*
		 * Return the word address of the erroring register so that it
		 * matches the register specification
		 */
		KGSL_DRV_CRIT(device,
			"RBBM | AHB bus error | %s | addr=%x | ports=%x:%x\n",
			reg & (1 << 28) ? "WRITE" : "READ",
			(reg & 0xFFFFF) >> 2, (reg >> 20) & 0x3,
			(reg >> 24) & 0xF);

		/* Clear the error */
		kgsl_regwrite(device, A5XX_RBBM_AHB_CMD, (1 << 4));
		return;
	}
	case A5XX_INT_RBBM_TRANSFER_TIMEOUT:
		KGSL_DRV_CRIT_RATELIMIT(device, "RBBM: AHB transfer timeout\n");
		break;
	case A5XX_INT_RBBM_ME_MS_TIMEOUT:
		kgsl_regread(device, A5XX_RBBM_AHB_ME_SPLIT_STATUS, &reg);
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM | ME master split timeout | status=%x\n", reg);
		break;
	case A5XX_INT_RBBM_PFP_MS_TIMEOUT:
		kgsl_regread(device, A5XX_RBBM_AHB_PFP_SPLIT_STATUS, &reg);
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM | PFP master split timeout | status=%x\n", reg);
		break;
	case A5XX_INT_RBBM_ETS_MS_TIMEOUT:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM: ME master split timeout\n");
		break;
	case A5XX_INT_RBBM_ATB_ASYNC_OVERFLOW:
		KGSL_DRV_CRIT_RATELIMIT(device, "RBBM: ATB ASYNC overflow\n");
		break;
	case A5XX_INT_RBBM_GPC_ERROR:
		KGSL_DRV_CRIT_RATELIMIT(device, "RBBM: GPC error\n");
		break;
	case A5XX_INT_CP_HW_ERROR:
		kgsl_regread(device, A5XX_CP_HW_FAULT, &reg);
		KGSL_DRV_CRIT_RATELIMIT(device,
			"CP | Ringbuffer HW fault | status=%x\n", reg);
		break;
	case A5XX_INT_RBBM_ATB_BUS_OVERFLOW:
		KGSL_DRV_CRIT_RATELIMIT(device, "RBBM: ATB bus overflow\n");
		break;
	case A5XX_INT_UCHE_OOB_ACCESS:
		KGSL_DRV_CRIT_RATELIMIT(device, "UCHE: Out of bounds access\n");
		break;
	default:
		KGSL_DRV_CRIT_RATELIMIT(device, "Unknown interrupt %d\n", bit);
	}
}

#define A5XX_INT_MASK \
	((1 << A5XX_INT_RBBM_AHB_ERROR) |		\
	 (1 << A5XX_INT_RBBM_TRANSFER_TIMEOUT) |		\
	 (1 << A5XX_INT_RBBM_ME_MS_TIMEOUT) |		\
	 (1 << A5XX_INT_RBBM_PFP_MS_TIMEOUT) |		\
	 (1 << A5XX_INT_RBBM_ETS_MS_TIMEOUT) |		\
	 (1 << A5XX_INT_RBBM_ATB_ASYNC_OVERFLOW) |		\
	 (1 << A5XX_INT_RBBM_GPC_ERROR) |		\
	 (1 << A5XX_INT_CP_HW_ERROR) |	\
	 (1 << A5XX_INT_CP_IB1) |			\
	 (1 << A5XX_INT_CP_IB2) |			\
	 (1 << A5XX_INT_CP_RB) |			\
	 (1 << A5XX_INT_RBBM_ATB_BUS_OVERFLOW) |	\
	 (1 << A5XX_INT_UCHE_OOB_ACCESS))


static struct adreno_irq_funcs a5xx_irq_funcs[] = {
	ADRENO_IRQ_CALLBACK(NULL),              /* 0 - RBBM_GPU_IDLE */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 1 - RBBM_AHB_ERROR */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 2 - RBBM_TRANSFER_TIMEOUT */
	/* 3 - RBBM_ME_MASTER_SPLIT_TIMEOUT  */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback),
	/* 4 - RBBM_PFP_MASTER_SPLIT_TIMEOUT */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback),
	 /* 5 - RBBM_ETS_MASTER_SPLIT_TIMEOUT */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback),
	/* 6 - RBBM_ATB_ASYNC_OVERFLOW */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback),
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 7 - GPC_ERR */
	ADRENO_IRQ_CALLBACK(NULL),				/* 8 - CP_SW */
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 9 - CP_HW_ERROR */
	/* 10 - CP_CCU_FLUSH_DEPTH_TS */
	ADRENO_IRQ_CALLBACK(NULL),
	 /* 11 - CP_CCU_FLUSH_COLOR_TS */
	ADRENO_IRQ_CALLBACK(NULL),
	 /* 12 - CP_CCU_RESOLVE_TS */
	ADRENO_IRQ_CALLBACK(NULL),
	ADRENO_IRQ_CALLBACK(adreno_cp_callback), /* 13 - CP_IB2_INT */
	ADRENO_IRQ_CALLBACK(adreno_cp_callback), /* 14 - CP_IB1_INT */
	ADRENO_IRQ_CALLBACK(adreno_cp_callback), /* 15 - CP_RB_INT */
	/* 16 - CCP_UNUSED_1 */
	ADRENO_IRQ_CALLBACK(NULL),
	ADRENO_IRQ_CALLBACK(NULL), /* 17 - CP_RB_DONE_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 18 - CP_WT_DONE_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 19 - UNKNOWN_1 */
	ADRENO_IRQ_CALLBACK(NULL), /* 20 - CP_CACHE_FLUSH_TS */
	/* 21 - UNUSED_2 */
	ADRENO_IRQ_CALLBACK(NULL),
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 22 - RBBM_ATB_BUS_OVERFLOW */
	/* 23 - MISC_HANG_DETECT */
	ADRENO_IRQ_CALLBACK(adreno_hang_int_callback),
	ADRENO_IRQ_CALLBACK(a5xx_err_callback), /* 24 - UCHE_OOB_ACCESS */
	ADRENO_IRQ_CALLBACK(NULL), /* 25 - UCHE_TRAP_INTR */
	ADRENO_IRQ_CALLBACK(NULL), /* 27 - DEBBUS_INTR_0 */
	ADRENO_IRQ_CALLBACK(NULL), /* 28 - DEBBUS_INTR_1 */
	ADRENO_IRQ_CALLBACK(NULL), /* 29 - GPMU_ERROR */
	ADRENO_IRQ_CALLBACK(NULL), /* 29 - GPMU_THERMAL */
	ADRENO_IRQ_CALLBACK(NULL), /* 30 - ISDB_CPU_IRQ */
	ADRENO_IRQ_CALLBACK(NULL), /* 31 - ISDB_UNDER_DEBUG */
};

static struct adreno_irq a5xx_irq = {
	.funcs = a5xx_irq_funcs,
	.funcs_count = ARRAY_SIZE(a5xx_irq_funcs),
	.mask = A5XX_INT_MASK,
};

struct adreno_gpudev adreno_a5xx_gpudev = {
	.reg_offsets = &a5xx_reg_offsets,
	.ft_perf_counters = a5xx_ft_perf_counters,
	.ft_perf_counters_count = ARRAY_SIZE(a5xx_ft_perf_counters),
	.start = a5xx_start,
	.irq = &a5xx_irq,
	.irq_trace = trace_kgsl_a5xx_irq_status,
	.num_prio_levels = 1,
	.rb_init = a5xx_rb_init,
	.microcode_read = a5xx_microcode_read,
	.microcode_load = a5xx_microcode_load,
	.perfcounters = &a5xx_perfcounters,
};

