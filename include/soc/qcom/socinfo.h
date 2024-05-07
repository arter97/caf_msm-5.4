/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */

#ifndef __SOC_QCOM_SOCINFO_H__
#define __SOC_QCOM_SOCINFO_H__

#include <linux/types.h>

enum socinfo_parttype {
	SOCINFO_PART_GPU = 1,
	SOCINFO_PART_VIDEO,
	SOCINFO_PART_CAMERA,
	SOCINFO_PART_DISPLAY,
	SOCINFO_PART_AUDIO,
	SOCINFO_PART_MODEM,
	SOCINFO_PART_WLAN,
	SOCINFO_PART_COMP,
	SOCINFO_PART_SENSORS,
	SOCINFO_PART_NPU,
	SOCINFO_PART_SPSS,
	SOCINFO_PART_NAV,
	SOCINFO_PART_COMPUTE_1,
	SOCINFO_PART_DISPLAY_1,
	SOCINFO_PART_NSP,
	SOCINFO_PART_EVA,
	SOCINFO_PART_MAX_PARTTYPE
};
enum subset_part_type {
	PART_UNKNOWN      = 0,
	PART_GPU          = 1,
	PART_VIDEO        = 2,
	PART_CAMERA       = 3,
	PART_DISPLAY      = 4,
	PART_AUDIO        = 5,
	PART_MODEM        = 6,
	PART_WLAN         = 7,
	PART_COMP         = 8,
	PART_SENSORS      = 9,
	PART_NPU          = 10,
	PART_SPSS         = 11,
	PART_NAV          = 12,
	PART_COMP1        = 13,
	PART_DISPLAY1     = 14,
	PART_NSP          = 15,
	PART_EVA,
	NUM_PARTS_MAX,
};
enum subset_cluster_type {
	CLUSTER_CPUSS      = 0,
	NUM_CLUSTERS_MAX,
};
#if IS_ENABLED(CONFIG_QCOM_SOCINFO)
uint32_t socinfo_get_id(void);
uint32_t socinfo_get_serial_number(void);
const char *socinfo_get_id_string(void);
char *socinfo_get_partinfo_part_name(unsigned int part_id);
uint32_t socinfo_get_cluster_info(enum subset_cluster_type cluster);
bool socinfo_get_part_info(enum subset_part_type part);
int socinfo_get_part_count(enum subset_part_type part);
int socinfo_get_subpart_info(enum subset_part_type part,
		u32 *part_info,
		u32 num_parts);
#else
static inline uint32_t socinfo_get_id(void)
{
	return 0;
}

static inline uint32_t socinfo_get_serial_number(void)
{
	return 0;
}

static inline const char *socinfo_get_id_string(void)
{
	return "N/A";
}
const char *socinfo_get_partinfo_part_name(unsigned int part_id)
{
	return NULL;
}
uint32_t socinfo_get_cluster_info(enum subset_cluster_type cluster)
{
	return 0;
}
bool socinfo_get_part_info(enum subset_part_type part)
{
	return false;
}
int socinfo_get_part_count(enum subset_part_type part)
{
	return -EINVAL;
}
int socinfo_get_subpart_info(enum subset_part_type part,
		u32 *part_info,
		u32 num_parts)
{
	return -EINVAL;
}
#endif /* CONFIG_QCOM_SOCINFO */

#endif /* __SOC_QCOM_SOCINFO_H__ */
