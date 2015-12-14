/* Copyright (c) 2010-2013, 2015 The Linux Foundation. All rights reserved.
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
#ifndef __EXTERNAL_COMMON_H__
#define __EXTERNAL_COMMON_H__
#include <linux/switch.h>
#include <video/msm_hdmi_modes.h>

#ifdef DEBUG
#ifndef DEV_DBG_PREFIX
#define DEV_DBG_PREFIX "EXT_INTERFACE: "
#endif
#define DEV_DBG(args...)	pr_debug(DEV_DBG_PREFIX args)
#else
#define DEV_DBG(args...)	(void)0
#endif /* DEBUG */
#define DEV_INFO(args...)	dev_info(external_common_state->dev, args)
#define DEV_WARN(args...)	dev_warn(external_common_state->dev, args)
#define DEV_ERR(args...)	dev_err(external_common_state->dev, args)

static inline const char *msm_hdmi_mode_2string(uint32_t mode)
{
	switch (mode) {
	case HDMI_VFRMT_UNKNOWN:		return "Unknown";
	case HDMI_VFRMT_640x480p60_4_3:		return "640x480 p60 4/3";
	case HDMI_VFRMT_720x480p60_4_3:		return "720x480 p60 4/3";
	case HDMI_VFRMT_720x480p60_16_9:	return "720x480 p60 16/9";
	case HDMI_VFRMT_1280x720p60_16_9:	return "1280x 720 p60 16/9";
	case HDMI_VFRMT_1920x1080i60_16_9:	return "1920x1080 i60 16/9";
	case HDMI_VFRMT_1440x480i60_4_3:	return "1440x480 i60 4/3";
	case HDMI_VFRMT_1440x480i60_16_9:	return "1440x480 i60 16/9";
	case HDMI_VFRMT_1440x240p60_4_3:	return "1440x240 p60 4/3";
	case HDMI_VFRMT_1440x240p60_16_9:	return "1440x240 p60 16/9";
	case HDMI_VFRMT_2880x480i60_4_3:	return "2880x480 i60 4/3";
	case HDMI_VFRMT_2880x480i60_16_9:	return "2880x480 i60 16/9";
	case HDMI_VFRMT_2880x240p60_4_3:	return "2880x240 p60 4/3";
	case HDMI_VFRMT_2880x240p60_16_9:	return "2880x240 p60 16/9";
	case HDMI_VFRMT_1440x480p60_4_3:	return "1440x480 p60 4/3";
	case HDMI_VFRMT_1440x480p60_16_9:	return "1440x480 p60 16/9";
	case HDMI_VFRMT_1920x1080p60_16_9:	return "1920x1080 p60 16/9";
	case HDMI_VFRMT_720x576p50_4_3:		return "720x576 p50 4/3";
	case HDMI_VFRMT_720x576p50_16_9:	return "720x576 p50 16/9";
	case HDMI_VFRMT_1280x720p50_16_9:	return "1280x720 p50 16/9";
	case HDMI_VFRMT_1920x1080i50_16_9:	return "1920x1080 i50 16/9";
	case HDMI_VFRMT_1440x576i50_4_3:	return "1440x576 i50 4/3";
	case HDMI_VFRMT_1440x576i50_16_9:	return "1440x576 i50 16/9";
	case HDMI_VFRMT_1440x288p50_4_3:	return "1440x288 p50 4/3";
	case HDMI_VFRMT_1440x288p50_16_9:	return "1440x288 p50 16/9";
	case HDMI_VFRMT_2880x576i50_4_3:	return "2880x576 i50 4/3";
	case HDMI_VFRMT_2880x576i50_16_9:	return "2880x576 i50 16/9";
	case HDMI_VFRMT_2880x288p50_4_3:	return "2880x288 p50 4/3";
	case HDMI_VFRMT_2880x288p50_16_9:	return "2880x288 p50 16/9";
	case HDMI_VFRMT_1440x576p50_4_3:	return "1440x576 p50 4/3";
	case HDMI_VFRMT_1440x576p50_16_9:	return "1440x576 p50 16/9";
	case HDMI_VFRMT_1920x1080p50_16_9:	return "1920x1080 p50 16/9";
	case HDMI_VFRMT_1920x1080p24_16_9:	return "1920x1080 p24 16/9";
	case HDMI_VFRMT_1920x1080p25_16_9:	return "1920x1080 p25 16/9";
	case HDMI_VFRMT_1920x1080p30_16_9:	return "1920x1080 p30 16/9";
	case HDMI_VFRMT_2880x480p60_4_3:	return "2880x480 p60 4/3";
	case HDMI_VFRMT_2880x480p60_16_9:	return "2880x480 p60 16/9";
	case HDMI_VFRMT_2880x576p50_4_3:	return "2880x576 p50 4/3";
	case HDMI_VFRMT_2880x576p50_16_9:	return "2880x576 p50 16/9";
	case HDMI_VFRMT_1920x1250i50_16_9:	return "1920x1250 i50 16/9";
	case HDMI_VFRMT_1920x1080i100_16_9:	return "1920x1080 i100 16/9";
	case HDMI_VFRMT_1280x720p100_16_9:	return "1280x720 p100 16/9";
	case HDMI_VFRMT_720x576p100_4_3:	return "720x576 p100 4/3";
	case HDMI_VFRMT_720x576p100_16_9:	return "720x576 p100 16/9";
	case HDMI_VFRMT_1440x576i100_4_3:	return "1440x576 i100 4/3";
	case HDMI_VFRMT_1440x576i100_16_9:	return "1440x576 i100 16/9";
	case HDMI_VFRMT_1920x1080i120_16_9:	return "1920x1080 i120 16/9";
	case HDMI_VFRMT_1280x720p120_16_9:	return "1280x720 p120 16/9";
	case HDMI_VFRMT_720x480p120_4_3:	return "720x480 p120 4/3";
	case HDMI_VFRMT_720x480p120_16_9:	return "720x480 p120 16/9";
	case HDMI_VFRMT_1440x480i120_4_3:	return "1440x480 i120 4/3";
	case HDMI_VFRMT_1440x480i120_16_9:	return "1440x480 i120 16/9";
	case HDMI_VFRMT_720x576p200_4_3:	return "720x576 p200 4/3";
	case HDMI_VFRMT_720x576p200_16_9:	return "720x576 p200 16/9";
	case HDMI_VFRMT_1440x576i200_4_3:	return "1440x576 i200 4/3";
	case HDMI_VFRMT_1440x576i200_16_9:	return "1440x576 i200 16/9";
	case HDMI_VFRMT_720x480p240_4_3:	return "720x480 p240 4/3";
	case HDMI_VFRMT_720x480p240_16_9:	return "720x480 p240 16/9";
	case HDMI_VFRMT_1440x480i240_4_3:	return "1440x480 i240 4/3";
	case HDMI_VFRMT_1440x480i240_16_9:	return "1440x480 i240 16/9";
	case HDMI_VFRMT_1280x720p24_16_9:	return "1280x720 p24 16/9";
	case HDMI_VFRMT_1280x720p25_16_9:	return "1280x720 p25 16/9";
	case HDMI_VFRMT_1280x720p30_16_9:	return "1280x720 p30 16/9";
	case HDMI_VFRMT_1920x1080p120_16_9:	return "1920x1080 p120 16/9";
	case HDMI_VFRMT_1920x1080p100_16_9:	return "1920x1080 p100 16/9";
	case HDMI_VFRMT_3840x2160p30_16_9:	return "3840x2160 p30 16/9";
	case HDMI_VFRMT_3840x2160p25_16_9:	return "3840x2160 p25 16/9";
	case HDMI_VFRMT_3840x2160p24_16_9:	return "3840x2160 p24 16/9";
	case HDMI_VFRMT_4096x2160p24_16_9:	return "4096x2160 p24 16/9";
	case HDMI_VFRMT_2560x1600p60_16_9:	return "2560x1600 p60 16/9";
	case HDMI_VFRMT_1280x1024p60_5_4:	return "1280x1042 p60 5/4";
	case HDMI_VFRMT_1280x800p60_16_9:	return "1280x800 p60 16/9";
	default:				return "???";
	}
}

#if defined(CONFIG_FB_MSM_HDMI_COMMON)
extern int ext_resolution;

/* A lookup table for all the supported display modes by the HDMI
 * hardware and driver.  Use HDMI_SETUP_LUT in the module init to
 * setup the LUT with the supported modes. */
extern struct msm_hdmi_mode_timing_info
	hdmi_common_supported_video_mode_lut[HDMI_VFRMT_MAX];

/* Structure that encapsulates all the supported display modes by the HDMI sink
 * device */
struct hdmi_disp_mode_list_type {
	uint32	disp_mode_list[HDMI_VFRMT_MAX];
#define TOP_AND_BOTTOM		0x10
#define FRAME_PACKING		0x20
#define SIDE_BY_SIDE_HALF	0x40
	uint32	disp_3d_mode_list[HDMI_VFRMT_MAX];
	uint32	disp_multi_3d_mode_list[16];
	uint32	disp_multi_3d_mode_list_cnt;
	uint32	num_of_elements;
};
#endif

/*
 * As per the CEA-861E spec, there can be a total of 10 short audio
 * descriptors with each SAD being 3 bytes long.
 * Thus, the maximum length of the audio data block would be 30 bytes
 */
#define MAX_AUDIO_DATA_BLOCK_SIZE	30
#define MAX_SPKR_ALLOC_DATA_BLOCK_SIZE	3

struct external_common_state_type {
	boolean hpd_state;
	boolean pre_suspend_hpd_state;
	boolean skip_edid;
	boolean edid_en;
	boolean hpd_en;
	struct kobject *uevent_kobj;
	uint32 video_resolution;
	struct device *dev;
	struct switch_dev sdev;
	struct switch_dev audio_sdev;
#ifdef CONFIG_FB_MSM_HDMI_3D
	boolean format_3d;
	void (*switch_3d)(boolean on);
#endif
#ifdef CONFIG_FB_MSM_HDMI_COMMON
	boolean hdcp_active;
	boolean hpd_feature_on;
	boolean hdmi_sink;
	struct hdmi_disp_mode_list_type disp_mode_list;
	uint16 video_latency, audio_latency;
	uint16 physical_address;
	uint32 preferred_video_format;
	uint8 pt_scan_info;
	uint8 it_scan_info;
	uint8 ce_scan_info;
	uint8 spd_vendor_name[9];
	uint8 spd_product_description[17];
	boolean present_3d;
	boolean present_hdcp;
	uint8 audio_data_block[MAX_AUDIO_DATA_BLOCK_SIZE];
	int adb_size;
	uint8 spkr_alloc_data_block[MAX_SPKR_ALLOC_DATA_BLOCK_SIZE];
	int sadb_size;
	int (*read_edid_block)(int block, uint8 *edid_buf);
	int (*hpd_feature)(int on);
#endif
};

/* The external interface driver needs to initialize the common state. */
extern struct external_common_state_type *external_common_state;
extern struct mutex external_common_state_hpd_mutex;
extern struct mutex hdmi_msm_state_mutex;

#ifdef CONFIG_FB_MSM_HDMI_COMMON
int hdmi_common_read_edid(void);
const char *video_format_2string(uint32 format);
bool hdmi_common_get_video_format_from_drv_data(struct msm_fb_data_type *mfd);
const struct msm_hdmi_mode_timing_info *hdmi_common_get_mode(uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_common_get_supported_mode(
	uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_mhl_get_mode(uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_mhl_get_supported_mode(
	uint32 mode);
void hdmi_common_init_panel_info(struct msm_panel_info *pinfo);

ssize_t video_3d_format_2string(uint32 format, char *buf, u32 size);
#endif

int external_common_state_create(struct platform_device *pdev);
void external_common_state_remove(void);

#endif /* __EXTERNAL_COMMON_H__ */
