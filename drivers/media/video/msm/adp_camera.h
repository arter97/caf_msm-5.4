/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_CAMERA_TEST_H
#define _MSM_CAMERA_TEST_H

#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/msm_mdp.h>
#include <media/msm_camera.h>
#include <media/msm_isp.h>
#include <media/videobuf2-core.h>
#include "csi/msm_csiphy.h"
#include "csi/msm_csid.h"
#include "sensors/msm_sensor_common.h"
#include "msm.h"
#include "vfe/msm_vfe32.h"
#include "../../../video/msm/msm_fb.h"

#define PREVIEW_BUFFER_COUNT 3
int PREVIEW_HEIGHT = 507;
int PREVIEW_WIDTH = 720;
int PREVIEW_BUFFER_LENGTH;
int PREVIEW_BUFFER_SIZE;
enum camera_preview_buffer_state {
	CAMERA_PREVIEW_BUFFER_STATE_UNUSED,
	CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED, /* free, can be used */
	CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG,/* used by ping pong */
	/* preview data dequeue from ping pong */
	CAMERA_PREVIEW_BUFFER_STATE_DEQUEUED_FROM_PINGPONG,
	CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_DISPLAY,/* used by display */
	/* dequeued by display, shall be set to initialized and re-use */
	CAMERA_PREVIEW_BUFFER_STATE_DEQUEUED_FROM_DISPLAY
};

/* Guidance lane variables */
#define GUIDANCE_LANE_BUFFER_COUNT 3
int GUIDANCE_LANE_HEIGHT;
int GUIDANCE_LANE_WIDTH;
int GUIDANCE_LANE_BUFFER_LENGTH;
int GUIDANCE_LANE_BUFFER_SIZE;

enum guidance_lane_buffer_state {
	GUIDANCE_LANE_BUFFER_STATE_UNUSED,
	GUIDANCE_LANE_BUFFER_STATE_INITIALIZED, /* free, can be used */
};

enum overlay_count {
	OVERLAY_CAMERA_PREVIEW = 0,
	OVERLAY_GUIDANCE_LANE,
	OVERLAY_COUNT,
};

/* use msm_free_buf, for easily called by ping pong buffer configuration */
struct preview_mem {
	struct msm_free_buf cam_preview;
	struct list_head list;
	enum camera_preview_buffer_state state;
	uint32_t offset;
};
struct msm_camera_preview_data {
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	struct preview_mem preview_buffer[PREVIEW_BUFFER_COUNT];
	struct list_head camera_preview_list;
	struct mutex preview_lock;
};
/* add for axi bus configuration */
#define AXI_MAX_WM_NUM	      7
#define MAX_OUTPUT_SUPPORTED 5 /* Max Number of scalers in VFE */
struct output_ch {
	int32_t   ch0:16;
	int32_t   ch1:16;
	int32_t   ch2:16;
	int32_t   /* reserved */ : 16;
	int32_t   inst_handle:32;
} __packed;

struct vfe_pixel_if_cfg {
	uint32_t input_select:2;
	uint32_t rdi_enable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdi_m0_select:4;
	uint32_t rdi_m1_select:4;
	uint32_t rdi_m2_select:4;
	uint32_t rdi_m0_frame_based_enable:1;
	uint32_t rdi_m1_frame_based_enable:1;
	uint32_t rdi_m2_frame_based_enable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdi_frame_skip:4;
	uint32_t rdi_frame_skip_enable:1;
	uint32_t /* reserved */ : 3;
	uint32_t rdi_stream_select:4;
} __packed;

struct vfe_rdi_cfg0 {
	uint32_t /*reserved*/ : 2;
	uint32_t rdi_enable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdi_m3_select:4;
	uint32_t rdi_m4_select:4;
	uint32_t rdi_m5_select:4;
	uint32_t /* reserved */ : 4;
	uint32_t rdi_frame_skip:4;
	uint32_t rdi_frame_skip_enable:1;
	uint32_t /*reserved*/ : 3;
	uint32_t rdi_stream_select1:4;
} __packed;

struct vfe_rdi_cfg1 {
	uint32_t /*reserved*/ : 2;
	uint32_t rdi_enable:1;
	uint32_t /* reserved */ : 1;
	uint32_t rdi_m6_select:4;
	uint32_t rdi_m7_select:4;
	uint32_t rdi_m8_select:4;
	uint32_t /* reserved */ : 4;
	uint32_t rdi_frame_skip:4;
	uint32_t rdi_frame_skip_enable:1;
	uint32_t /*reserved*/ : 3;
	uint32_t rdi_stream_select2:4;
} __packed;


struct output_path {
	struct output_ch out0;
	struct output_ch out1;
	struct output_ch out2;
	struct output_ch out3;
	struct output_ch out4;
} __packed;

struct vfe_wm_config {
	uint32_t wm_enable:1;
	uint32_t /* reserved */ : 31;
	uint32_t bus_ping_addr:32;
	uint32_t bus_pong_addr:32;
	uint32_t bus_ub_depth:10;
	uint32_t /* reserved */ : 6;
	uint32_t bus_ub_offset:10;
	uint32_t /* reserved */ : 6;
	uint32_t buslines_per_image:12;
	uint32_t /* reserved */ : 4;
	uint32_t busdwords_per_line:10;
	uint32_t /* reserved */ : 6;
	uint32_t busburst_length:2;
	uint32_t /* reserved */ : 2;
	uint32_t busbuffer_num_rows:12;
	uint32_t busrow_increment:13;
	uint32_t /* reserved */ : 3;
} __packed;

struct vfe_axi_output_config_cmd_type {
	uint32_t busio_format:32;
	uint32_t bus_cmd:32;
	/* bus_cfg's 31st bit config is as following
	* 7x30: OOO_WRITE_ENABLE  ---  Not Used
	* 8x60: Reserved          ---  Not Used
	* 8960: IMEM Mode disable (For Inline-JPEG only)
	*/
	uint32_t bus_cfg:32;
	uint32_t xbar_cfg0:32;
	uint32_t xbar_cfg1:32;
	uint32_t bus_wr_skip_cfg:32;
	struct vfe_wm_config wm[AXI_MAX_WM_NUM];
	struct output_path outpath;
	struct vfe_pixel_if_cfg pixel_if_cfg;
	struct vfe_rdi_cfg0 rdi_cfg0;
	struct vfe_rdi_cfg1 rdi_cfg1;
} __packed;
/* Guidance Lane data structure */
struct guidance_lane_mem {
	struct msm_free_buf guidance_lane;
	struct list_head list;
	enum guidance_lane_buffer_state state;
	uint32_t offset;
};
struct msm_guidance_lane_data {
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	struct guidance_lane_mem guidance_lane_buf[GUIDANCE_LANE_BUFFER_COUNT];
	struct list_head guidance_lane_list;
	struct mutex guidance_lane_lock;
};

extern struct csiphy_device *lsh_csiphy_dev;
extern struct csid_device *lsh_csid_dev;
extern struct ispif_device *lsh_ispif;
extern struct msm_sensor_ctrl_t mt9m114_s_ctrl;
extern struct msm_sensor_ctrl_t adv7481_s_ctrl;
extern struct v4l2_subdev *lsh_axi_ctrl;
extern struct axi_ctrl_t *my_axi_ctrl;
extern struct mdp4_overlay_pipe *pipe[OVERLAY_COUNT];

int msm_ispif_init(struct ispif_device *ispif, const uint32_t *csid_version);
int msm_ispif_config(struct ispif_device *ispif,
	struct msm_ispif_params_list *params_list);
int msm_csiphy_init(struct csiphy_device *csiphy_dev);
int msm_csiphy_lane_config(struct csiphy_device *csiphy_dev,
	struct msm_camera_csiphy_params *csiphy_params);
int msm_csid_init(struct csid_device *csid_dev,
	uint32_t *csid_version, uint32_t bypass);
int msm_csid_config(struct csid_device *csid_dev,
	struct msm_camera_csid_params *csid_params);
int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int msm_sensor_power_up_adv7481(struct msm_sensor_ctrl_t *s_ctrl);
int msm_sensor_power_down_adv7481(struct msm_sensor_ctrl_t *s_ctrl);
int msm_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl, int mode,
	struct sensor_init_cfg *init_info);
int msm_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
					int mode, int res);
void msm_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl);
void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl);
int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl);
void axi_start(struct msm_cam_media_controller *pmctl,
	struct axi_ctrl_t *axi_ctrl,
	struct msm_camera_vfe_params_t vfe_params);
int vfe32_config_axi(struct axi_ctrl_t *axi_ctrl, int mode,
	uint32_t *ao);
int msm_axi_subdev_init_rdi_only(struct v4l2_subdev *sd,
uint8_t dual_enabled, struct msm_sensor_ctrl_t *s_ctrl);
int msm_ispif_subdev_video_s_stream_rdi_only(
	struct ispif_device *ispif,
	int enable);
int axi_reset_rdi1_only(struct axi_ctrl_t *axi_ctrl,
	struct msm_camera_vfe_params_t vfe_params);
int configure_pingpong_buffers_rdi1_only(int id, int path,
	struct axi_ctrl_t *axi_ctrl);
void msm_axi_process_irq(struct v4l2_subdev *sd, void *arg);
void axi_start_rdi1_only(struct axi_ctrl_t *axi_ctrl,
	struct msm_sensor_ctrl_t *s_ctrl);
int vfe32_config_axi_rdi_only(struct axi_ctrl_t *axi_ctrl, int mode,
	uint32_t *ao);
void axi_stop_rdi1_only(struct axi_ctrl_t *axi_ctrl);

int mdpclient_overlay_set(struct mdp_overlay *ov);
int mdpclient_overlay_unset(struct mdp_overlay *ov);
int mdpclient_overlay_play(struct msmfb_overlay_data *ovdata);
int mdpclient_display_commit(void);
int mdpclient_msm_fb_open(void);
int mdpclient_msm_fb_close(void);
int mdpclient_msm_fb_blank(int blank_mode, bool op_enable);

int mdp_bus_scale_update_request(u64 ab_p0, u64 ib_p0, u64 ab_p1,
					u64 ib_p1);
int msm_csid_release(struct csid_device *csid_dev, uint32_t bypass);
int msm_csiphy_release(struct csiphy_device *csiphy_dev, void *arg);
void msm_ispif_release(struct ispif_device *ispif);
void msm_axi_subdev_release_rdi_only(struct v4l2_subdev *sd,
					struct msm_sensor_ctrl_t *s_ctrl);
int msm_axi_subdev_s_crystal_freq(struct v4l2_subdev *sd,
					u32 freq, u32 flags);
void msm_sensor_detect_std_adv7481(struct msm_sensor_ctrl_t *s_ctrl);

void msm_sensor_start_stream_adv7481(struct msm_sensor_ctrl_t *s_ctrl);

void preview_set_data_pipeline(void);
struct preview_mem *preview_buffer_find_free_for_ping_pong(void);
struct preview_mem *preview_buffer_find_free_for_ping_pong_mdp(void);
static void preview_configure_bufs(void);
static void guidance_lane_configure_bufs(void);
static void guidance_lane_pic_update(const unsigned char *pic,
					unsigned int pos_x,
					unsigned int pos_y,
					unsigned int image_w,
					unsigned int image_h,
					unsigned int buffer_index);
static void guidance_lane_pic_update_all(void);
static void guidance_lane_set_data_pipeline(void);
static void mdp_queue_overlay_buffers(struct work_struct *work);
static void guidance_lane_set_overlay_init(struct mdp_overlay *overlay);
static int adp_rear_camera_enable(void);

#endif /* _MSM_CAMERA_TEST_H */
