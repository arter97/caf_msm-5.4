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

#include <linux/msm_ion.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/swab.h>
#include <asm/mach-types.h>
#include <media/msm_ba.h>
#include "adp_camera.h"
#include <mach/board.h>
#include <video/mdp_arb.h>
#include <linux/reverse.h>
#include <media/v4l2-ctrls.h>
#include <linux/videodev2.h>

#define FRAME_DELAY 33333
#define RDI1_USE_WM 4
#define MM_CAM_USE_BYPASS 1
#ifdef CONFIG_MSM_S_PLATFORM
#define INIT_PIPELINE 0
#else
#define INIT_PIPELINE 1
#endif
static int init_pipeline;
static void *k_addr[OVERLAY_COUNT];
static int alloc_overlay_pipe_flag[OVERLAY_COUNT];
static struct mdp_overlay overlay_req[OVERLAY_COUNT];
static unsigned int overlay_fd[OVERLAY_COUNT];
static struct msmfb_overlay_data overlay_data[OVERLAY_COUNT];
static uint8_t dual_enabled;
static struct msm_sensor_ctrl_t *s_ctrl;
static struct msm_ispif_params_list params_list;
struct sensor_init_cfg *init_info;
static uint32_t csid_version;
static int rdi1_irq_count;
static struct vfe_axi_output_config_cmd_type vfe_axi_cmd_para;
static struct msm_camera_vfe_params_t vfe_para;
static struct work_struct wq_mdp_queue_overlay_buffers;

static int adp_rear_camera_enable(void);

static struct completion preview_enabled;
static struct completion preview_disabled;

static struct msm_camera_preview_data preview_data;
static struct mdp_buf_queue s_mdp_buf_queue;

static int g_preview_height = 507;
static int g_preview_width = 720;
static int g_preview_buffer_length;
static int g_preview_buffer_size;
#define DEFAULT_SRC_X_OFFSET 27

/* MDP buffer fifo */
#define MDP_BUF_QUEUE_LENGTH (PREVIEW_BUFFER_COUNT+1)
struct mdp_buf_queue {
	int read_idx;
	int write_idx;
	int bufq[MDP_BUF_QUEUE_LENGTH];
};

enum camera_states {
	CAMERA_UNINITIALIZED = 0,
	CAMERA_SUSPENDED,
	CAMERA_PREVIEW_DISABLED,
	CAMERA_PREVIEW_ENABLED,
	CAMERA_TRANSITION_SUSPEND,
	CAMERA_TRANSITION_PREVIEW,
	CAMERA_TRANSITION_OFF,
	CAMERA_RECOVERY_START,
	CAMERA_RECOVERY_RETRY,
	CAMERA_RECOVERY_IN_PROGRESS,
	CAMERA_RECOVERY_FAILED,
	CAMERA_UNKNOWN
};

static char *display_name[] = {
	"PRIMARY\n",
	"SECONDARY\n",
	"TERTIARY\n",
};
#define DISPLAY_ID_MAX ARRAY_SIZE(display_name)
#define DISPLAY_NAME_LENGTH_MAX 16

struct adp_camera_ctxt {
	void *ba_inst_hdlr;

	/* V4L2 Framework */
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct video_device *vdev;
	struct media_device mdev;

	/* state */
	enum camera_states state;

	/* v4l2 overlay configs */
	bool display_init[DISPLAY_ID_MAX];
	struct v4l2_cropcap crop_src_rect_cap;
	struct v4l2_cropcap crop_dst_rect_cap[DISPLAY_ID_MAX];
	struct v4l2_rect    crop_src_rect;
	struct v4l2_rect    crop_dst_rect[DISPLAY_ID_MAX];
	u32 z_order[DISPLAY_ID_MAX];
	u32 display_id;

#ifdef CONFIG_FB_MSM_MDP_ARB
	/* MDP Arbitrator */
	void *mdp_arb_handle[DISPLAY_ID_MAX];
#endif
	int fb_idx[DISPLAY_ID_MAX];

	/* recovery */
	struct delayed_work recovery_work;
	bool camera_stream_enabled;

	/* debug */
	struct dentry *debugfs_root;
};
static struct adp_camera_ctxt *adp_cam_ctxt;

#ifdef CONFIG_FB_MSM_MDP_ARB
#define MDP_ARB_CLIENT_NAME "adp_camera"
#define MDP_ARB_EVENT_NAME "switch-reverse"
#define MDP_ARB_NUM_OF_EVENT_STATE 2

static int enable_camera_preview(void);
static int disable_camera_preview(void);
static int adp_camera_enable_stream(void);
static int adp_camera_disable_stream(void);
static int mdp_enable_camera_preview(void);
static int mdp_disable_camera_preview(void);
#endif

static void adp_camera_event_callback(void *instance,
					unsigned int event, void *arg);

static const struct msm_ba_ext_ops adp_camera_ba_ext_ops = {
	.msm_ba_cb = adp_camera_event_callback,
};

static struct msm_cam_server_adp_cam adp_camera_msm_cam_ops = {
	.adp_cam_cb = adp_camera_event_callback,
};


static int axi_vfe_config_cmd_para(struct vfe_axi_output_config_cmd_type *cmd)
{
	/* configure the axi bus parameters here */
	int config;
	int ch_wm;
	int axi_output_ppw;
	int image_width;
	int image_height;
	int burst_length;
	image_width = g_preview_width*2;
	image_height = g_preview_height;
	cmd->busio_format = 0;
	cmd->bus_cmd = 0;
	cmd->bus_cfg = 0x2aaa771;
	cmd->bus_wr_skip_cfg  = 0;
	cmd->rdi_cfg0.rdi_enable = 0x1;
	cmd->rdi_cfg0.rdi_stream_select1 = 0x3;
	cmd->rdi_cfg0.rdi_m3_select = 0x0;
	config = 0x01 | (0x06 << 5);
	ch_wm = RDI1_USE_WM;

	cmd->xbar_cfg0 = 0;
	cmd->xbar_cfg1 = 0;
	if (ch_wm == 3 || ch_wm == 2 || ch_wm == 1 || ch_wm == 0) {
		cmd->xbar_cfg0 = (cmd->xbar_cfg0) &
			~(0x000000FF << (8 * (ch_wm % 4)));
		cmd->xbar_cfg0 = (cmd->xbar_cfg0) |
			(config << (8 * (ch_wm % 4)));
	} else if (ch_wm == 6 || ch_wm == 5 || ch_wm == 4) {
		cmd->xbar_cfg1 = (cmd->xbar_cfg1) &
			~(0x000000FF << (8 * (ch_wm % 4)));
		cmd->xbar_cfg1 = (cmd->xbar_cfg1) |
			(config << (8 * (ch_wm % 4)));
	}

	axi_output_ppw = 8;
	burst_length = 1;
	cmd->wm[ch_wm].busdwords_per_line = 89;
	cmd->wm[ch_wm].busrow_increment =
		(image_width+(axi_output_ppw-1))/(axi_output_ppw);
	cmd->wm[ch_wm].buslines_per_image = image_height - 1;
	cmd->wm[ch_wm].busbuffer_num_rows = image_height - 1;
	cmd->wm[ch_wm].busburst_length = burst_length;
	cmd->wm[ch_wm].bus_ub_offset = 931;
	cmd->wm[ch_wm].bus_ub_depth = 92;

	/* use frame base */
	cmd->pixel_if_cfg.rdi_m1_frame_based_enable = 1;
	cmd->outpath.out3.ch0 = ch_wm;

	return 0;
}

static int mdp_buf_queue_enq(int buf_idx)
{
	int rc = 0;
	int new_buf_idx = (s_mdp_buf_queue.write_idx+1)%MDP_BUF_QUEUE_LENGTH;

	if (new_buf_idx == s_mdp_buf_queue.read_idx) {
		/* Queue overflow. The queue is sized
		   to numberofbuffers+1 so this should
		   never occur */
		pr_err("Warning fifo overflow! enq buf_idx %d write_idx %d",
		buf_idx, s_mdp_buf_queue.write_idx);
		rc = -1;
	} else {
		s_mdp_buf_queue.bufq[s_mdp_buf_queue.write_idx] = buf_idx;
		pr_debug("enq buf_idx %d write_idx %d",
			buf_idx, s_mdp_buf_queue.write_idx);
		s_mdp_buf_queue.write_idx = new_buf_idx;
	}
	return rc;
}

static int mdp_buf_queue_deq(void)
{
	int buf_idx = -1;
	if (s_mdp_buf_queue.read_idx != s_mdp_buf_queue.write_idx) {
		buf_idx = s_mdp_buf_queue.bufq[s_mdp_buf_queue.read_idx];
		pr_debug("deq buf_idx %d read_idx %d",
			buf_idx, s_mdp_buf_queue.read_idx);
		s_mdp_buf_queue.read_idx =
			(s_mdp_buf_queue.read_idx+1)%MDP_BUF_QUEUE_LENGTH;
	}
	return buf_idx;
}

static void preview_set_data_pipeline(void)
{
	int ispif_stream_enable;
	u32 freq = 320000000;
	u32 flags = 0;
	int rc;

	pr_debug("%s:  kpi entry!!!\n", __func__);
	dual_enabled = 0; /* set according to log */
	/* power on and enable  clock for vfe and axi, before csi */
	rc = msm_axi_subdev_init_rdi_only(lsh_axi_ctrl, dual_enabled, s_ctrl);
	if (rc < 0)
		return;

	msm_axi_subdev_s_crystal_freq(lsh_axi_ctrl, freq,  flags);
	csid_version = 1;
	rc = msm_csid_init(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&csid_version, MM_CAM_USE_BYPASS);
	rc = msm_csiphy_init(
			lsh_csiphy_dev[adp_rvc_csi_lane_params.csi_phy_sel]);
	rc = msm_ispif_init_rdi(lsh_ispif, &csid_version); /* ISPIF_INIT */
	params_list.params[0].intftype =  RDI1; /* RDI1 */
	params_list.params[0].csid = adp_rvc_csi_lane_params.csi_phy_sel;
	params_list.params[0].vfe_intf =  VFE0;
	params_list.params[0].cid_mask = (1 << 0);
	params_list.len = 1;

	pr_debug("%s: config ispif\n", __func__);

	/* ISPIF_CFG,use csid 1,rdi1 */
	rc = msm_ispif_config(lsh_ispif, &params_list);
	rc = msm_csid_config(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&adp_rvc_csid_params); /* CSID_CFG */

	pr_debug("%s: config csid\n", __func__);

	/* CSIPHY_CFG */
	rc = msm_csiphy_lane_config(
			lsh_csiphy_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&adp_rvc_csiphy_params);
	ispif_stream_enable = 129;  /* configure to select RDI 1, VFE0 */
	msm_ispif_subdev_video_s_stream_rdi_only(lsh_ispif,
			ispif_stream_enable);
	pr_debug("%s: begin axi reset!!!\n", __func__);
	axi_reset_rdi1_only(my_axi_ctrl, vfe_para);
	pr_debug("%s: vfe32_config_axi now!!!\n", __func__);
	axi_vfe_config_cmd_para(&vfe_axi_cmd_para);
	rc = vfe32_config_axi_rdi_only(my_axi_ctrl, OUTPUT_TERT2,
			(uint32_t *)&vfe_axi_cmd_para);

	adp_camera_msm_cam_ops.interface = RDI1;
	adp_camera_msm_cam_ops.csid_sd = &lsh_csid_dev
			[adp_rvc_csi_lane_params.csi_phy_sel]->subdev;
	adp_camera_msm_cam_ops.csiphy_sd = &lsh_csiphy_dev
			[adp_rvc_csi_lane_params.csi_phy_sel]->subdev;
	msm_cam_server_adp_cam_register(&adp_camera_msm_cam_ops);

	pr_debug("%s: kpi exit!!!\n", __func__);
}

static void preview_buffer_alloc(void)
{
	int i, result;
	int offset = 0;
	int mem_len;
	unsigned long paddr;
	int cam_domain_num;

	memset(&preview_data, 0, sizeof(struct msm_camera_preview_data));
	preview_data.ion_client = msm_ion_client_create(-1, "camera");
	if (IS_ERR_OR_NULL((void *)preview_data.ion_client)) {
		pr_err("%s: ION create client failed\n", __func__);
		goto err;
	}
	/* ION_CP_MM_HEAP_ID size is 0x7800000 */
	preview_data.ion_handle = ion_alloc(preview_data.ion_client,
				(g_preview_buffer_size),
				SZ_4K,
				(0x1 << ION_CP_MM_HEAP_ID |
				 0x1 << ION_IOMMU_HEAP_ID),
				0);
	if (IS_ERR_OR_NULL((void *) preview_data.ion_handle)) {
		pr_err("%s: ION memory allocation failed\n", __func__);
		goto err_ion_client;
	}
	k_addr[OVERLAY_CAMERA_PREVIEW] = ion_map_kernel(preview_data.ion_client,
			preview_data.ion_handle);

	cam_domain_num = msm_cam_server_get_domain_num();
	pr_debug("%s cam domain num %d", __func__, cam_domain_num);

	result = ion_map_iommu(preview_data.ion_client, preview_data.ion_handle,
			cam_domain_num, 0, SZ_4K, 0,
			(unsigned long *)&paddr,
			(unsigned long *)&mem_len, 0, 1);
	if (result < 0) {
		pr_err("%s Could not get  address\n", __func__);
		goto err_ion_handle;
	}
	overlay_fd[OVERLAY_CAMERA_PREVIEW] = (unsigned int) ion_share_dma_buf(
						preview_data.ion_client,
						preview_data.ion_handle);
	/* Make all phys point to the correct address */
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		/* Y plane */
		preview_data.preview_buffer[i].cam_preview.ch_paddr[0] =
			(uint32_t)(paddr + offset);

		offset += g_preview_buffer_length;
	}
	return;
err_ion_handle:
	ion_free(preview_data.ion_client, preview_data.ion_handle);
err_ion_client:
	ion_client_destroy(preview_data.ion_client);
err:
	return;
}

static int preview_buffer_init(void)
{
	int i ;
	/* initialized the list head and add all nodes */
	pr_debug("%s: Begin to setup buffer list\n", __func__);

	INIT_LIST_HEAD(&preview_data.camera_preview_list);

	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		list_add(&(preview_data.preview_buffer[i].list),
				&(preview_data.camera_preview_list));
		preview_data.preview_buffer[i].state =
			CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
	}

	pr_debug("%s: setup buffer list\n", __func__);
	return 0;
}

static void preview_buffer_return_by_index(int i)
{
	preview_data.preview_buffer[i].state =
			CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
	pr_debug("%s: bufidx %i phys addr = 0x%x\n", __func__, i,
	preview_data.preview_buffer[i].cam_preview.ch_paddr[0]);
	return;
}

static int preview_find_buffer_index_by_paddr(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
	   set the buffer state as initialized and can be reused */
	int i;
	int index = 0;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++)	{
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0]
				== (uint32_t)(paddr)) {
			index = i;
			break;
		}
	}
	return index;
}

static struct preview_mem *preview_buffer_find_free_for_ping_pong(void)
{
	struct preview_mem *buf = NULL;
	bool found_buff = false;

	pr_debug("%s: begin to find free buffer for ping pong\n",
			__func__);
	list_for_each_entry(buf, &(preview_data.camera_preview_list), list) {
		if (buf->state == CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED) {
			found_buff = true;
			break;
		}
	}
	if (IS_ERR_OR_NULL((void *) buf))
		pr_err("%s: can not find free buffer\n", __func__);


	pr_debug("%s: free buff find finished, physical addr is %x buf 0x%p\n",
		__func__, buf->cam_preview.ch_paddr[0], buf);

	if (found_buff == false)
		buf = NULL;
	return buf;
}

static int preview_configure_ping_pong_buffer(
					struct preview_mem *ping_buf,
					struct preview_mem *pong_buf,
					struct preview_mem *free_buffer)
{
	int message_id;
	int path;
	message_id = 0;
	path = VFE_MSG_OUTPUT_TERTIARY2;
	pr_debug("%s: ping/pong configure enter!!, init the address\n",
			__func__);
	/* get three buffer from the list, use them to configure
	ping pong buffer and free buffer */
	my_axi_ctrl->share_ctrl->outpath.out3.ping.ch_paddr[0] =
			ping_buf->cam_preview.ch_paddr[0];
	my_axi_ctrl->share_ctrl->outpath.out3.pong.ch_paddr[0] =
			pong_buf->cam_preview.ch_paddr[0];
	my_axi_ctrl->share_ctrl->outpath.out3.free_buf.ch_paddr[0] =
			free_buffer->cam_preview.ch_paddr[0];
	pr_debug("%s: ping/pong configure enter~~~\n",
				__func__);
	configure_pingpong_buffers_rdi1_only(message_id, path, my_axi_ctrl);
	return 0;
}

static int preview_buffer_free(void)
{
	int ret;
	int cam_domain_num;
	cam_domain_num = msm_cam_server_get_domain_num();
	pr_debug("%s cam domain num %d", __func__, cam_domain_num);

	ion_unmap_iommu(preview_data.ion_client, preview_data.ion_handle,
			cam_domain_num, 0);
	ion_free(preview_data.ion_client, preview_data.ion_handle);
	ion_client_destroy(preview_data.ion_client);
	return ret = 0;
}

static void preview_configure_bufs(void)
{
	/* step 1, alloc ION buffer */
	pr_debug("%s: preview buffer allocate\n", __func__);
	preview_buffer_alloc();

	/*step2, buffer linked list */
	preview_buffer_init();
}

static void preview_set_overlay_params(struct mdp_overlay *overlay)
{
	overlay->src.width  = g_preview_width;
	overlay->src.height = g_preview_height;
	overlay->src.format = MDP_CBYCRY_H2V1;
	overlay->src_rect.x = adp_cam_ctxt->crop_src_rect.left;
	overlay->src_rect.y = adp_cam_ctxt->crop_src_rect.top;
	overlay->src_rect.w = adp_cam_ctxt->crop_src_rect.width;
	overlay->src_rect.h = adp_cam_ctxt->crop_src_rect.height;
	overlay->dst_rect.x = adp_cam_ctxt->
			crop_dst_rect[adp_cam_ctxt->display_id].left;
	overlay->dst_rect.y = adp_cam_ctxt->
			crop_dst_rect[adp_cam_ctxt->display_id].top;
	overlay->dst_rect.w = adp_cam_ctxt->
			crop_dst_rect[adp_cam_ctxt->display_id].width;
	overlay->dst_rect.h = adp_cam_ctxt->
			crop_dst_rect[adp_cam_ctxt->display_id].height;
	overlay->z_order =  adp_cam_ctxt->z_order[adp_cam_ctxt->display_id];
	overlay->alpha = MDP_ALPHA_NOP;
	overlay->transp_mask = MDP_TRANSP_NOP;
	overlay->blend_op = BLEND_OP_OPAQUE;
	overlay->flags = 0;
	overlay->is_fg = 0;

	return;
}

#ifdef CONFIG_FB_MSM_MDP_ARB
static int preview_overlay_update(void)
{
	int rc;
	struct mdp_display_commit commit_data;

	/* skip if preview is not running */
	if (CAMERA_PREVIEW_ENABLED != adp_cam_ctxt->state)
		return 0;

	preview_set_overlay_params(&overlay_req[OVERLAY_CAMERA_PREVIEW]);

	rc = mdp_arb_client_overlay_set(
			adp_cam_ctxt->mdp_arb_handle[adp_cam_ctxt->display_id],
			&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	if (rc) {
		pr_err("%s - mdp_arb_client_overlay_set failed %d",
				__func__, rc);
		return rc;
	}

	memset(&commit_data, 0, sizeof(commit_data));
	commit_data.wait_for_finish = true;
	commit_data.flags = MDP_DISPLAY_COMMIT_OVERLAY;
	rc = mdp_arb_client_overlay_commit(adp_cam_ctxt->\
			mdp_arb_handle[adp_cam_ctxt->display_id], &commit_data);
	if (rc) {
		pr_err("%s - mdp_arb_client_overlay_commit failed %d",
				__func__, rc);
		return rc;
	}

	return rc;
}

static void mdp_queue_overlay_buffers(struct work_struct *work)
{
	int buffer_index = 0;
	static int is_first_commit = true;
	int ret = 0;
	struct mdp_display_commit commit_data;
	/* dequeue next buffer to display */
	buffer_index = mdp_buf_queue_deq();

	if (buffer_index < 0) {
		pr_err("%s: mdp buf queue empty\n", __func__);
		return;
	} else if (buffer_index > PREVIEW_BUFFER_COUNT - 1) {
		pr_err("%s: mdp buf queue invalid buffer %d/n",
			__func__, buffer_index);
		return;
	}
	if (adp_cam_ctxt->state == CAMERA_PREVIEW_ENABLED) {
		overlay_data[OVERLAY_CAMERA_PREVIEW].id =
				overlay_req[OVERLAY_CAMERA_PREVIEW].id;
		overlay_data[OVERLAY_CAMERA_PREVIEW].data.flags =
			MSMFB_DATA_FLAG_ION_NOT_FD;
		overlay_data[OVERLAY_CAMERA_PREVIEW].data.offset =
			(buffer_index * g_preview_buffer_length);
		overlay_data[OVERLAY_CAMERA_PREVIEW].data.memory_id =
				overlay_fd[OVERLAY_CAMERA_PREVIEW];

		ret = mdp_arb_client_overlay_play(adp_cam_ctxt->
				mdp_arb_handle[adp_cam_ctxt->display_id],
				&overlay_data[OVERLAY_CAMERA_PREVIEW]);
		if (ret)
			pr_err("%s overlay play preview fails=%d", __func__,
				ret);

		if (is_first_commit == true)
			place_marker("FF RVC pre commit");
		/* perform display commit */
		memset(&commit_data, 0, sizeof(commit_data));
		commit_data.wait_for_finish = true;
		commit_data.flags = MDP_DISPLAY_COMMIT_OVERLAY;
		ret = mdp_arb_client_overlay_commit(adp_cam_ctxt->\
			mdp_arb_handle[adp_cam_ctxt->display_id], &commit_data);
		if (ret)
			pr_err("%s overlay commit fails=%d", __func__, ret);
		if (is_first_commit == true) {
			place_marker("FF RVC post commit");
			is_first_commit = false;
		}
	}
	preview_buffer_return_by_index(buffer_index);
}
#else
static int preview_overlay_update(void)
{
	int rc;

	/* skip if preview is not running */
	if (CAMERA_PREVIEW_ENABLED != adp_cam_ctxt->state)
		return 0;

	preview_set_overlay_params(&overlay_req[OVERLAY_CAMERA_PREVIEW]);

	rc = mdpclient_overlay_set(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
			&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	if (rc) {
		pr_err("%s - mdpclient_overlay_set failed %d", __func__, rc);
		return rc;
	}

	rc = mdpclient_display_commit(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	if (rc) {
		pr_err("%s - mdpclient_display_commit failed %d", __func__, rc);
		return rc;
	}

	return rc;
}

static void mdp_queue_overlay_buffers(struct work_struct *work)
{
	int buffer_index = 0;
	static int is_first_commit = true;
	/* dequeue next buffer to display */
	buffer_index = mdp_buf_queue_deq();

	if (buffer_index < 0) {
		pr_err("%s: mdp buf queue empty\n", __func__);
		return;
	} else if (buffer_index > PREVIEW_BUFFER_COUNT - 1) {
		pr_err("%s: mdp buf queue invalid buffer %d/n",
			__func__, buffer_index);
		return;
	}

	overlay_data[OVERLAY_CAMERA_PREVIEW].id =
			overlay_req[OVERLAY_CAMERA_PREVIEW].id;
	overlay_data[OVERLAY_CAMERA_PREVIEW].data.flags =
		MSMFB_DATA_FLAG_ION_NOT_FD;
	overlay_data[OVERLAY_CAMERA_PREVIEW].data.offset =
		(buffer_index * g_preview_buffer_length);
	overlay_data[OVERLAY_CAMERA_PREVIEW].data.memory_id =
			overlay_fd[OVERLAY_CAMERA_PREVIEW];

	if (is_first_commit == true)
		place_marker("FF RVC pre commit");
	/* perform display commit */
	mdpclient_display_commit(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	if (is_first_commit == true) {
		place_marker("FF RVC post commit");
		is_first_commit = false;
	}
	preview_buffer_return_by_index(buffer_index);
}
#endif

void vfe32_process_output_path_irq_rdi1_only(struct axi_ctrl_t *axi_ctrl)
{
	uint32_t ping_pong;
	uint32_t ch0_paddr = 0;
	int rc = 0;
	/* this must be rdi image output. */
	struct preview_mem *free_buf = NULL;
	int buffer_index;
	static bool is_camera_first_frame = true;

	pr_debug("%s:  enter into rdi1 irq now, rdi1_irq_count is %d!!!\n",
			__func__, rdi1_irq_count);

	if (adp_cam_ctxt->state == CAMERA_TRANSITION_OFF) {
		pr_debug("%s - received interrupt; skipping!\n", __func__);
		return;
	}

	rdi1_irq_count++;
	/*RDI1*/
	if (axi_ctrl->share_ctrl->operation_mode & VFE_OUTPUTS_RDI1) {
		/* check if there is free buffer,
		 * if there is, then put it to ping pong
		 */
		free_buf = preview_buffer_find_free_for_ping_pong();
		if (axi_ctrl->share_ctrl->outpath.out3.capture_cnt > 0 ||
			free_buf) {
			ping_pong = msm_camera_io_r(
				axi_ctrl->share_ctrl->vfebase +
				VFE_BUS_PING_PONG_STATUS);
			/* Y channel */
			/* read preview data from pingpong buffer */
			ch0_paddr = vfe32_get_ch_addr(ping_pong,
				axi_ctrl->share_ctrl->vfebase,
				axi_ctrl->share_ctrl->outpath.out3.ch0);
			/* convert camera domain address
			 * to mdp domain address
			 */
			buffer_index =
				preview_find_buffer_index_by_paddr(ch0_paddr);

			/* enq buffer to mdp buffers */
			rc = mdp_buf_queue_enq(buffer_index);
			if (rc) {
				/* enqueue failed, requeue the buffer to vfe */
				preview_buffer_return_by_index(buffer_index);
				pr_err("Failed to enq buf %d, dropped frame",
				      buffer_index);
			}

			/* Only log once for early camera kpi */
			pr_info_once("kpi cam rdi1 first frame %d\n",
				      rdi1_irq_count);

			if (is_camera_first_frame == true) {
				place_marker("First Camera Frame");
				is_camera_first_frame = false;
			}

			schedule_work(&wq_mdp_queue_overlay_buffers);
			if (free_buf) {
				vfe32_put_ch_addr(ping_pong,
					axi_ctrl->share_ctrl->vfebase,
					axi_ctrl->share_ctrl->outpath.out3.ch0,
					free_buf->cam_preview.ch_paddr[0]);
				/* shall add function to change the buffer state
				 * to be queued to ping pong
				 * preview_buffer_update_status_to_pingpong
				 * preview_free_buf() is needed?
				 */
				free_buf->state =
				CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
			}
			if (axi_ctrl->share_ctrl->
					outpath.out3.capture_cnt == 1)
				axi_ctrl->share_ctrl->
				outpath.out3.capture_cnt = 0;
		} else {
			axi_ctrl->share_ctrl->outpath.out3.frame_drop_cnt++;
			pr_debug("%s: no free buffer for rdi1!\n",
					__func__);
		}
	}
}

static int find_fb_idx(int display_id, int *fb_idx)
{
	int i;
	int ret = 0;
	char fb_name[DISPLAY_NAME_LENGTH_MAX];

	if (display_id >= DISPLAY_ID_MAX) {
		pr_err("%s display_id=%d is bigger than max=%d", __func__,
			display_id, DISPLAY_ID_MAX);
		return -EINVAL;
	}

	for (i = 0; i < FB_MAX; i++) {
		memset(fb_name, 0x00, sizeof(fb_name));
		ret = mdpclient_msm_fb_get_id(i, fb_name,
			DISPLAY_NAME_LENGTH_MAX);
		if (ret) {
			pr_err("%s fb_get_id fails=%d, idx=%d",
				__func__, ret, i);
			break;
		} else if (!strcmp(fb_name, display_name[display_id])) {
			break;
		}
	}
	if (i == FB_MAX) {
		pr_err("%s can't find fb_idx for display_id=%d",
			__func__, display_id);
		ret = -EFAULT;
	} else {
		if (fb_idx)
			*fb_idx = i;
	}
	return ret;
}

static int adp_camera_query_display(int display_id)
{
	int ret;
	struct fb_var_screeninfo screeninfo;

	if (adp_cam_ctxt->display_init[display_id])
		return 0;

	ret = mdpclient_msm_fb_open(adp_cam_ctxt->fb_idx[display_id]);
	if (ret) {
		pr_err("%s: can't open fb=%d ret=%d", __func__,
				adp_cam_ctxt->fb_idx[display_id], ret);
		return ret;
	}

	ret = mdpclient_msm_fb_get_vscreeninfo(adp_cam_ctxt->fb_idx[display_id],
			&screeninfo);

	mdpclient_msm_fb_close(adp_cam_ctxt->fb_idx[display_id]);
	if (ret) {
		pr_err("%s: can't get vscreen_info %d\n",
				__func__, ret);
		return ret;
	}

	adp_cam_ctxt->crop_dst_rect_cap[display_id].type =
			V4L2_BUF_TYPE_VIDEO_OUTPUT;
	adp_cam_ctxt->crop_dst_rect_cap[display_id].bounds.width =
			screeninfo.xres;
	adp_cam_ctxt->crop_dst_rect_cap[display_id].bounds.height =
			screeninfo.yres;
	adp_cam_ctxt->crop_dst_rect_cap[display_id].pixelaspect.numerator =
			adp_cam_ctxt->
			crop_dst_rect_cap[display_id].bounds.width;
	adp_cam_ctxt->crop_dst_rect_cap[display_id].pixelaspect.denominator =
			adp_cam_ctxt->
			crop_dst_rect_cap[display_id].bounds.height;

	/* set default rect and current region to fullscreen */
	adp_cam_ctxt->crop_dst_rect[display_id] =
			adp_cam_ctxt->crop_dst_rect_cap[display_id].bounds;
	adp_cam_ctxt->crop_dst_rect_cap[display_id].defrect =
			adp_cam_ctxt->crop_dst_rect_cap[display_id].bounds;

	adp_cam_ctxt->display_init[display_id] = true;

	return 0;
}

#ifdef CONFIG_FB_MSM_MDP_ARB

static int mdp_arb_set_event(int state)
{
	int rc = 0;
	struct mdp_arb_event event;

	memset(&event, 0, sizeof(event));
	strlcpy(event.name, MDP_ARB_EVENT_NAME, MDP_ARB_NAME_LEN);
	event.event.driver_set_event = state;
	rc = mdp_arb_event_set(&event);
	if (rc)
		pr_err("%s mdp_arb_event_set fails=%d, event=%s, state=%d",
			__func__, rc, event.name, state);

	return rc;
}

static int mdp_arb_register_event(void)
{
	int rc = 0;
	struct mdp_arb_event event;
	struct mdp_arb_events events;
	int state[MDP_ARB_NUM_OF_EVENT_STATE] = {0, 1};

	/* Register to MDP arbitrator*/
	strlcpy(event.name, MDP_ARB_EVENT_NAME, MDP_ARB_NAME_LEN);
	event.event.driver_register.num_of_states = MDP_ARB_NUM_OF_EVENT_STATE;
	event.event.driver_register.value = state;
	events.num_of_events = 1;
	events.event = &event;
	rc = mdp_arb_event_register(&events);
	if (rc) {
		pr_err("%s mdp_arb_event_register fails=%d", __func__, rc);
		return rc;
	}
	return rc;
}

static int mdp_arb_deregister_event(void)
{
	int rc = 0;
	struct mdp_arb_event event;
	struct mdp_arb_events events;

	strlcpy(event.name, MDP_ARB_EVENT_NAME, MDP_ARB_NAME_LEN);
	events.num_of_events = 1;
	events.event = &event;
	rc = mdp_arb_event_deregister(&events);
	if (rc)
		pr_err("%s mdp_arb_event_deregister fails=%d", __func__, rc);
	return rc;
}


static int arb_cb(void *handle, struct mdp_arb_cb_info *info, int flag)
{
	int ret = 0;
	int i, ack_ret;

	for (i = 0; i < DISPLAY_ID_MAX; i++) {
		if (handle == adp_cam_ctxt->mdp_arb_handle[i])
			break;
	}

	if (DISPLAY_ID_MAX == i) {
		pr_err("%s handle=0x%x != arb_handle=0x%x", __func__,
			(int)handle, (int)adp_cam_ctxt->mdp_arb_handle[0]);
		return -EINVAL;
	}

	if (!info) {
		pr_err("%s info is NULL", __func__);
		return -EINVAL;
	}

	/* only process callback for primary display */
	if (handle != adp_cam_ctxt->mdp_arb_handle[0])
		goto arb_ack;

	switch (info->event) {
	case MDP_ARB_NOTIFICATION_DOWN:
		ret = disable_camera_preview();
		if (ret)
			pr_err("%s disable_camera_preview fails=%d",
				__func__, ret);
		break;
	case MDP_ARB_NOTIFICATION_UP:
		ret = enable_camera_preview();
		if (ret)
			pr_err("%s enable_camera_preview fails=%d",
				__func__, ret);
		break;
	default:
		pr_err("%s doesn't support event=%x", __func__, info->event);
		ret = -EFAULT;
		break;
	}

arb_ack:
	ack_ret = mdp_arb_client_acknowledge(handle, info->event);
	if (ack_ret)
		pr_err("%s arb ack up fails=%d", __func__, ret);
	return ret ? ret : ack_ret;
}

static int mdp_init(int display_id)
{
	int ret = 0;
	struct mdp_arb_client_register_info info;
	struct mdp_arb_event event;
	int up_state = 1;
	int down_state = 0;

	memset(&info, 0x00, sizeof(info));
	memset(&event, 0x00, sizeof(event));

	info.cb = arb_cb;
	strlcpy(info.common.name, MDP_ARB_CLIENT_NAME, MDP_ARB_NAME_LEN);
	info.common.fb_index = adp_cam_ctxt->fb_idx[display_id];
	info.common.num_of_events = 1;
	strlcpy(event.name, MDP_ARB_EVENT_NAME, MDP_ARB_NAME_LEN);
	event.event.register_state.num_of_down_state_value = 1;
	event.event.register_state.down_state_value = &down_state;
	event.event.register_state.num_of_up_state_value = 1;
	event.event.register_state.up_state_value = &up_state;
	info.common.event = &event;
	info.common.priority = 2;
	info.common.notification_support_mask =
		MDP_ARB_NOTIFICATION_DOWN | MDP_ARB_NOTIFICATION_UP;
	ret = mdp_arb_client_register(&info,
			&adp_cam_ctxt->mdp_arb_handle[display_id]);
	if (ret)
		pr_err("%s arb_client_register fails=%d", __func__, ret);

	return ret;
}

static int mdp_exit(int display_id)
{
	int ret = 0;

	if (adp_cam_ctxt->mdp_arb_handle[display_id]) {
		ret = mdp_arb_client_deregister(
				adp_cam_ctxt->mdp_arb_handle[display_id]);
		if (ret)
			pr_err("%s arb_client_deregister fails=%d",
				__func__, ret);
	}
	return ret;
}
#else
static int mdp_arb_set_event(int state)
{
	int rc;

	if (state)
		rc = enable_camera_preview();
	else
		rc = disable_camera_preview();

	return rc;
}

static int mdp_arb_register_event(void)
{
	return 0;
}

static int mdp_arb_deregister_event(void)
{
	return 0;
}

static int mdp_init(int display_id)
{
	return 0;
}

static int mdp_exit(display_id)
{
	return 0;
}
#endif

static void adp_camera_v4l2_queue_event(int event)
{
	struct v4l2_event v4l2_ev;
	v4l2_ev.id = 0;
	v4l2_ev.type = event;
	ktime_get_ts(&v4l2_ev.timestamp);
	v4l2_event_queue(adp_cam_ctxt->vdev, &v4l2_ev);
}

/* Wait times in ms for delayed work
 * WAIT_RETRY : time to wait before retrying if we failed
 *                   to restart camera right away
 * WAIT_IN_PROGRESS : time to ensure no errors occur when we recovered
 * WAIT_FAILED_RETRY : time to wait before retrying if we failed recovery
 * */
#define CAMERA_RECOVERY_WAIT_RETRY 1000
#define CAMERA_RECOVERY_WAIT_IN_PROGRESS 1000
#define CAMERA_RECOVERY_WAIT_FAILED_RETRY 5000

static void adp_camera_recovery_work(struct work_struct *work)
{
	int rc;

	switch (adp_cam_ctxt->state) {
	case CAMERA_RECOVERY_START: {
		/* reset adp camera */
		pr_err("recover start");
		adp_camera_disable_stream();
		mdp_disable_camera_preview();

		rc = adp_camera_enable_stream();
		if (rc) {
			pr_err("%s - restart failed %d! Will try again...",
					__func__, rc);
			adp_cam_ctxt->state = CAMERA_RECOVERY_RETRY;
			schedule_delayed_work(&adp_cam_ctxt->recovery_work,
				msecs_to_jiffies(CAMERA_RECOVERY_WAIT_RETRY));
			goto cleanup;
		}

		adp_cam_ctxt->state = CAMERA_RECOVERY_IN_PROGRESS;
		schedule_delayed_work(&adp_cam_ctxt->recovery_work,
			msecs_to_jiffies(CAMERA_RECOVERY_WAIT_IN_PROGRESS));

		break;
	}
	case CAMERA_RECOVERY_RETRY: {
		rc = adp_camera_enable_stream();
		if (rc) {
			pr_err("%s - try restart failed %d!",
					__func__, rc);
			adp_cam_ctxt->state = CAMERA_RECOVERY_FAILED;
			schedule_delayed_work(&adp_cam_ctxt->recovery_work, 0);
		} else {
			/* make sure no more errors after restarting */
			pr_err("%s - restart success! Wait for clean pipe...",
					__func__);
			adp_cam_ctxt->state = CAMERA_RECOVERY_IN_PROGRESS;
			schedule_delayed_work(&adp_cam_ctxt->recovery_work,
				msecs_to_jiffies(
					CAMERA_RECOVERY_WAIT_IN_PROGRESS));
		}
		break;
	}
	case CAMERA_RECOVERY_IN_PROGRESS: {
		pr_err("%s - recovery success!", __func__);
		adp_camera_disable_stream();
		adp_cam_ctxt->state = CAMERA_PREVIEW_DISABLED;
		disable_camera_preview();
		adp_camera_v4l2_queue_event(ADP_CAMERA_EVENT_RECOVERY_SUCCESS);
		break;
	}
	case CAMERA_RECOVERY_FAILED:
		adp_camera_v4l2_queue_event(ADP_CAMERA_EVENT_RECOVERY_FAILED);
		adp_camera_disable_stream();
		msleep(CAMERA_RECOVERY_WAIT_FAILED_RETRY);

		pr_err("%s - recovery retry...", __func__);
		adp_cam_ctxt->state = CAMERA_RECOVERY_RETRY;
		schedule_delayed_work(&adp_cam_ctxt->recovery_work, 0);
		break;
	default:
		pr_err("%s - Invalid state %d",
				__func__, adp_cam_ctxt->state);
		break;
	}

cleanup:
	return;
}

static void adp_camera_event_callback(void *instance,
					unsigned int event, void *arg)
{
	pr_debug("%s - %x", __func__, event);

	switch (event) {
	case NOTIFY_VFE_WM_OVERFLOW_ERROR:
	case NOTIFY_ISPIF_OVERFLOW_ERROR:
	case NOTIFY_CSID_UNBOUNDED_FRAME_ERROR:
	case NOTIFY_CSID_STREAM_UNDERFLOW_ERROR:
	case NOTIFY_CSID_ECC_ERROR:
	case NOTIFY_CSID_CRC_ERROR:
	case NOTIFY_CSID_PHY_DL_OVERFLOW_ERROR:
	case NOTIFY_CSIPHY_ERROR:
	case V4L2_EVENT_MSM_BA_SIGNAL_LOST_LOCK:
	case V4L2_EVENT_MSM_BA_ERROR: {
		switch (adp_cam_ctxt->state) {
		case CAMERA_PREVIEW_ENABLED: {
			pr_err("%s - error event %x, start recovery...",
					__func__, event);
			adp_cam_ctxt->state = CAMERA_RECOVERY_START;

			adp_camera_v4l2_queue_event(ADP_CAMERA_EVENT_ERROR);

			schedule_delayed_work(&adp_cam_ctxt->recovery_work, 0);
			break;
		}
		case CAMERA_PREVIEW_DISABLED:
			pr_debug("%s ignore event %x while preview off",
					__func__, event);
			break;
		case CAMERA_RECOVERY_RETRY:
		case CAMERA_RECOVERY_START: {
			pr_debug("%s ignore event %x while starting recovery",
					__func__, event);
			break;
		}
		case CAMERA_RECOVERY_IN_PROGRESS:{
			pr_err("%s error event %x while recovering. recovery failed!",
					__func__, event);
			adp_cam_ctxt->state = CAMERA_RECOVERY_FAILED;
			cancel_delayed_work_sync(&adp_cam_ctxt->recovery_work);
			schedule_delayed_work(&adp_cam_ctxt->recovery_work, 0);
			break;
		}
		case CAMERA_RECOVERY_FAILED:
			pr_debug("%s ignore error event while failed already...",
					__func__);
			break;
		default:
			pr_err("%s not suppose to be here with state %d...",
					__func__, adp_cam_ctxt->state);
			break;
		}

		break;
	}
	default:
		break;
	}
}

static int adp_rear_camera_enable(void)
{
	struct preview_mem *ping_buffer, *pong_buffer, *free_buffer;
	struct v4l2_input input;
	struct v4l2_format fmt;
	int index = BA_IP_CVBS_0;
	enum v4l2_priority prio = V4L2_PRIORITY_RECORD;
	int ret = 0;
	int i;

	for (i = 0; i < DISPLAY_ID_MAX; i++) {

		ret = find_fb_idx(i, &adp_cam_ctxt->fb_idx[i]);
		if (ret) {
			if (i > 0) {
				pr_err("%s - couldn't find fb for display %d (%d)",
						__func__, i, ret);
				adp_cam_ctxt->fb_idx[i] = -1;
				continue;
			}

			pr_err("%s - couldn't find fb for primary display %d",
					__func__, ret);
			return ret;
		}

		ret = mdp_init(i);
		if (ret) {
			pr_err("%s mdp_init fails=%d", __func__, ret);
			return ret;
		}
	}

	pr_debug("%s: kpi entry\n", __func__);
	my_axi_ctrl->share_ctrl->current_mode = 4096;
	vfe_para.operation_mode = VFE_OUTPUTS_RDI1;

	/* Detect NTSC or PAL, get the preview width and height */

	adp_cam_ctxt->ba_inst_hdlr =
			msm_ba_open(&adp_camera_ba_ext_ops);
	pr_debug("%s: input index: %d\n", __func__, index);
	msm_ba_s_input(adp_cam_ctxt->ba_inst_hdlr, index);
	msm_ba_s_priority(adp_cam_ctxt->ba_inst_hdlr, prio);
	memset(&input, 0, sizeof(input));
	input.index = index;
	msm_ba_enum_input(adp_cam_ctxt->ba_inst_hdlr, &input);
	pr_debug("%s: input info: %s\n", __func__, input.name);
	msm_ba_g_fmt(adp_cam_ctxt->ba_inst_hdlr, &fmt);
	pr_debug("%s: format: %dx%d\n", __func__, fmt.fmt.pix.width,
			fmt.fmt.pix.height);

	g_preview_height = fmt.fmt.pix.height;
	g_preview_width = fmt.fmt.pix.width;

	adp_cam_ctxt->crop_src_rect_cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	adp_cam_ctxt->crop_src_rect_cap.bounds.width = g_preview_width;
	adp_cam_ctxt->crop_src_rect_cap.bounds.height = g_preview_height;
	adp_cam_ctxt->crop_src_rect_cap.pixelaspect.numerator =
			adp_cam_ctxt->crop_src_rect_cap.bounds.width;
	adp_cam_ctxt->crop_src_rect_cap.pixelaspect.denominator =
			adp_cam_ctxt->crop_src_rect_cap.bounds.height;

	/* set default rect and current region to fullscreen */
	adp_cam_ctxt->crop_src_rect =
			adp_cam_ctxt->crop_src_rect_cap.bounds;
	adp_cam_ctxt->crop_src_rect.top = DEFAULT_SRC_X_OFFSET;
	adp_cam_ctxt->crop_src_rect.height -= DEFAULT_SRC_X_OFFSET;
	adp_cam_ctxt->crop_src_rect_cap.defrect =
			adp_cam_ctxt->crop_src_rect;

	/* 4k align buffers */
	g_preview_buffer_length =  ALIGN(g_preview_width *
					g_preview_height*2, 4096);
	g_preview_buffer_size  =  PREVIEW_BUFFER_COUNT *
				g_preview_buffer_length;
	pr_debug("%s: width %d height %d length %d size %d\n", __func__,
		g_preview_width, g_preview_height, g_preview_buffer_length,
		g_preview_buffer_size);
	preview_configure_bufs();
	if (init_pipeline) {
		preview_set_data_pipeline();
		/* step 1, find free buffer from the list, then use it to
		configure ping/pong */
		ping_buffer = preview_buffer_find_free_for_ping_pong();
		ping_buffer->state =
			CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
		pong_buffer =
			preview_buffer_find_free_for_ping_pong();
		pong_buffer->state =
			CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
		free_buffer = preview_buffer_find_free_for_ping_pong();
		pr_debug("%s: find ping pong buffer end!!!\n", __func__);

		/* step 2, configure the free buffer to ping pong buffer */
		preview_configure_ping_pong_buffer(ping_buffer, pong_buffer,
			free_buffer);
	}
	/* init mdp buffer queue */
	memset(&s_mdp_buf_queue, 0, sizeof(s_mdp_buf_queue));

	my_axi_ctrl->share_ctrl->current_mode = 4096; /* BIT(12) */
	my_axi_ctrl->share_ctrl->operation_mode = 4096;

	pr_debug("%s: kpi exit\n", __func__);

	return 0;
}

static void adp_rear_camera_disable(void)
{
	int i;
	msm_cam_server_adp_cam_deregister();
	msm_axi_subdev_release_rdi_only(lsh_axi_ctrl, s_ctrl);
	msm_csid_release(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			MM_CAM_USE_BYPASS);
	msm_csiphy_release(lsh_csiphy_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&adp_rvc_csi_lane_params);
	msm_ispif_release_rdi(lsh_ispif);
	msm_ba_close(adp_cam_ctxt->ba_inst_hdlr);

	for (i = 0; i < DISPLAY_ID_MAX; i++)
		mdp_exit(i);
}

static int adp_camera_disable_stream(void)
{
	if (!adp_cam_ctxt->camera_stream_enabled) {
		pr_debug("%s - already disabled", __func__);
		return 0;
	}

	axi_stop_rdi1_only(my_axi_ctrl);

	/* delay to ensure sof for regupdate*/
	usleep(FRAME_DELAY);
	msm_ba_streamoff(adp_cam_ctxt->ba_inst_hdlr, 0);

	adp_cam_ctxt->camera_stream_enabled = false;

	return 0;
}

static int adp_camera_enable_stream(void)
{
	int rc = 0;

	if (adp_cam_ctxt->camera_stream_enabled) {
		pr_debug("%s - already enabled", __func__);
		goto exit;
	}

	msm_csid_reset(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel]);
	rc = msm_csid_config(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&adp_rvc_csid_params);
	if (rc) {
		pr_err("%s - msm_csid_config failed %d", __func__, rc);
		goto exit;
	}

	rc = msm_ba_streamon(adp_cam_ctxt->ba_inst_hdlr, 0);
	if (rc) {
		pr_err("%s - msm_ba_streamon failed %d", __func__, rc);
		goto exit;
	}

	axi_start_rdi1_only(my_axi_ctrl, s_ctrl);

	adp_cam_ctxt->camera_stream_enabled = true;

exit:
	return rc;
}

#ifdef CONFIG_FB_MSM_MDP_ARB
static int mdp_disable_camera_preview(void)
{
	int ret;
	struct mdp_display_commit commit_data;

	memset(&commit_data, 0, sizeof(commit_data));
	commit_data.wait_for_finish = true;
	commit_data.flags = MDP_DISPLAY_COMMIT_OVERLAY;

	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] == 0) {
		ret = mdp_arb_client_overlay_unset(
				adp_cam_ctxt->mdp_arb_handle
					[adp_cam_ctxt->display_id],
				overlay_req[OVERLAY_CAMERA_PREVIEW].id);
		if (ret)
			pr_err("%s overlay unset fails=%d", __func__, ret);

		pr_debug("%s: overlay_unset camera preview free pipe !\n",
			__func__);
	}

	ret = mdp_arb_client_overlay_commit(
			adp_cam_ctxt->mdp_arb_handle[adp_cam_ctxt->display_id],
			&commit_data);
	if (ret)
		pr_err("%s overlay commit fails=%d", __func__, ret);

	mdpclient_msm_fb_close(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);

	return ret;
}

static int mdp_enable_camera_preview(void)
{
	u64 mdp_max_bw_test = 2000000000;
	int ret;
	ret = mdpclient_msm_fb_open(
		adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	if (ret) {
		pr_err("%s: %d can't open fb=%d", __func__, ret,
				adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
		return ret;
	}

	ret = mdpclient_msm_fb_blank(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
			FB_BLANK_UNBLANK, true);
	if (ret) {
		pr_err("%s: can't turn on display!\n", __func__);
		goto mdp_failed;
	}

	mdp_bus_scale_update_request(mdp_max_bw_test, mdp_max_bw_test,
			mdp_max_bw_test, mdp_max_bw_test);

	/* configure pipe for reverse camera preview */
	overlay_req[OVERLAY_CAMERA_PREVIEW].id = MSMFB_NEW_REQUEST;
	preview_set_overlay_params(&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] =
			mdp_arb_client_overlay_set(
					adp_cam_ctxt->mdp_arb_handle
						[adp_cam_ctxt->display_id],
					&overlay_req[OVERLAY_CAMERA_PREVIEW]);

	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] != 0) {
		pr_err("%s: mdp_arb_client_overlay_set error!\n",
				__func__);
		goto mdp_failed;
	}

	return ret;

mdp_failed:
	mdpclient_msm_fb_close(
				adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	return ret;
}

#else
static int mdp_disable_camera_preview(void)
{
	mdpclient_display_commit(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);

	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] == 0) {
		mdpclient_overlay_unset(
				adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
				&overlay_req[OVERLAY_CAMERA_PREVIEW]);
		pr_debug("%s: overlay_unset camera preview free pipe !\n",
			__func__);
	}

	mdpclient_display_commit(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);

	mdpclient_msm_fb_close(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
}

static int mdp_enable_camera_preview(void)
{
	u64 mdp_max_bw_test = 2000000000;

	mdpclient_msm_fb_open(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	if (mdpclient_msm_fb_blank(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
			FB_BLANK_UNBLANK, true)) {
		pr_err("%s: can't turn on display!\n", __func__);
		goto mdp_failed;
	}

	mdp_bus_scale_update_request(mdp_max_bw_test, mdp_max_bw_test,
		mdp_max_bw_test, mdp_max_bw_test);

	/* configure pipe for reverse camera preview */
	overlay_req[OVERLAY_CAMERA_PREVIEW].id = MSMFB_NEW_REQUEST;
	preview_set_overlay_params(&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] =
		mdpclient_overlay_set(
				adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
				&overlay_req[OVERLAY_CAMERA_PREVIEW]);

	while (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] != 0 &&
				waitcounter_camera < max_wait_count) {
		msleep(100);
		waitcounter_camera++;
		alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] =
		mdpclient_overlay_set(
				adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id],
				&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	}

	if (waitcounter_camera > 1)
		pr_err("%s: mdpclient_overlay_set camera wait counter value is: %d",
			__func__, waitcounter_camera);

	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] != 0) {
		pr_err("%s: mdpclient_overlay_set error!\n",
			__func__);
		goto mdp_failed;
	}

	return ret;

mdp_failed:
	mdpclient_msm_fb_close(
			adp_cam_ctxt->fb_idx[adp_cam_ctxt->display_id]);
	return ret;
}
#endif

static int disable_camera_preview(void)
{
	int ret = 0;
	pr_debug("%s: kpi entry\n", __func__);

	if (!adp_cam_ctxt) {
		pr_err("%s - context is invalid",
						__func__);
		return -EPERM;
	}

	switch (adp_cam_ctxt->state) {
	case CAMERA_PREVIEW_ENABLED:
	case CAMERA_TRANSITION_PREVIEW:
		break;
	case CAMERA_PREVIEW_DISABLED:
		pr_warn("%s - preview already disabled", __func__);
		goto exit;
		break;
	default:
		pr_err("%s - cannot disable preview from uninitialized state",
				__func__);
		return -EPERM;
		break;
	}

	wait_for_completion(&preview_enabled);
	adp_cam_ctxt->state = CAMERA_TRANSITION_OFF;

	adp_camera_disable_stream();
	mdp_disable_camera_preview();

	adp_cam_ctxt->state = CAMERA_PREVIEW_DISABLED;

exit:
	complete_all(&preview_disabled);
	/* reset preview enable*/
	INIT_COMPLETION(preview_enabled);

	pr_debug("%s: kpi entry\n", __func__);
	return ret;
}

static int enable_camera_preview(void)
{
	int rc = 0;

	pr_debug("%s: kpi entry\n", __func__);

	if (!adp_cam_ctxt) {
		pr_err("%s - context is invalid",
						__func__);
		return -EPERM;
	}

	switch (adp_cam_ctxt->state) {
	case CAMERA_PREVIEW_DISABLED:
	case CAMERA_TRANSITION_OFF:
		break;
	case CAMERA_PREVIEW_ENABLED:
		pr_warn("%s - preview already enabled", __func__);
		goto exit;
		break;
	default:
		pr_err("%s - cannot enable preview from uninitialized state",
				__func__);
		return -EPERM;
		break;
	}

	if (adp_camera_query_display(adp_cam_ctxt->display_id))
		return -EINVAL;

	wait_for_completion(&preview_disabled);
	adp_cam_ctxt->state = CAMERA_TRANSITION_PREVIEW;

	rc = mdp_enable_camera_preview();
	if (rc)
		goto mdp_failed;

	rc = adp_camera_enable_stream();
	if (rc)
		goto camera_failed;

	adp_cam_ctxt->state = CAMERA_PREVIEW_ENABLED;

exit:
	complete_all(&preview_enabled);

	/* reset preview disable*/
	INIT_COMPLETION(preview_disabled);

	pr_debug("%s: kpi exit\n", __func__);
	return rc;

camera_failed:
	mdp_disable_camera_preview();
mdp_failed:
	adp_cam_ctxt->state = CAMERA_PREVIEW_DISABLED;
	complete_all(&preview_enabled);
	complete_all(&preview_disabled);
	pr_debug("%s: kpi exit\n", __func__);
	return rc;
}


static int adp_camera_v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	clear_bit(V4L2_FL_USES_V4L2_FH, &vdev->flags);
	return 0;
}

static int adp_camera_v4l2_close(struct file *filp)
{
	int rc = 0;
	return rc;
}

static int adp_camera_v4l2_querycap(struct file *filp, void *fh,
					struct v4l2_capability *cap)
{
	pr_debug("%s - enter", __func__);

	if (!cap) {
		pr_err("Invalid input cap = 0x%p", cap);
		return -EINVAL;
	}

	strlcpy(cap->driver, "adp_camera_driver", sizeof(cap->driver));
	strlcpy(cap->card, "adp_camera_8064", sizeof(cap->card));
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_STREAMING;
	memset(cap->reserved, 0x00, sizeof(cap->reserved));

	pr_debug("%s - exit", __func__);

	return 0;
}

static int adp_camera_v4l2_cropcap(struct file *file, void *fh,
				struct v4l2_cropcap *a)
{
	int rc;

	pr_debug("%s - enter", __func__);

	if (!a) {
		pr_err("%s - null param", __func__);
		return -EINVAL;
	}

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		*a = adp_cam_ctxt->crop_src_rect_cap;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		rc = adp_camera_query_display(adp_cam_ctxt->display_id);
		if (rc)
			return rc;

		*a = adp_cam_ctxt->crop_dst_rect_cap[adp_cam_ctxt->display_id];
		break;
	default:
		pr_err("%s - invalid type %d", __func__, a->type);
		return -EINVAL;
		break;
	}

	pr_debug("%s - exit", __func__);
	return 0;
}
static int adp_camera_v4l2_g_crop(struct file *file, void *fh,
				struct v4l2_crop *a)
{
	int rc;
	pr_debug("%s - enter", __func__);

	if (!a) {
		pr_err("%s - null param", __func__);
		return -EINVAL;
	}

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		a->c = adp_cam_ctxt->crop_src_rect;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		rc = adp_camera_query_display(adp_cam_ctxt->display_id);
		if (rc)
			return rc;

		a->c = adp_cam_ctxt->crop_dst_rect[adp_cam_ctxt->display_id];
		break;
	default:
		pr_err("%s - invalid type %d", __func__, a->type);
		return -EINVAL;
		break;
	}


	pr_debug("%s - exit", __func__);

	return 0;
}
static int adp_camera_v4l2_s_crop(struct file *file, void *fh,
				struct v4l2_crop *a)
{
	int rc = 0;

	pr_debug("%s - enter", __func__);

	if (!a) {
		pr_err("%s - null param", __func__);
		return -EINVAL;
	}

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		adp_cam_ctxt->crop_src_rect = a->c;
		pr_debug("%s - src_rect t-%d,l-%d w-%d,h-%d",
				__func__,
				a->c.top,
				a->c.left,
				a->c.width,
				a->c.height);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		rc = adp_camera_query_display(adp_cam_ctxt->display_id);
		if (rc)
			return rc;

		adp_cam_ctxt->crop_dst_rect[adp_cam_ctxt->display_id] = a->c;
		pr_debug("%s - dst_rect t-%d,l-%d w-%d,h-%d",
				__func__,
				a->c.top,
				a->c.left,
				a->c.width,
				a->c.height);
		break;
	default:
		pr_err("%s - invalid type %d", __func__, a->type);
		return -EINVAL;
		break;
	}

	rc = preview_overlay_update();
	if (rc)
		pr_err("%s - preview_overlay_update failed", __func__);


	pr_debug("%s - exit", __func__);

	return rc;
}

static int adp_camera_v4l2_streamon(struct file *file, void *fh,
					enum v4l2_buf_type i)
{
	int rc;
	INIT_COMPLETION(preview_enabled);
	rc = mdp_arb_set_event(1);
	if (!rc) {
		wait_for_completion(&preview_enabled);
		if (adp_cam_ctxt->state != CAMERA_PREVIEW_ENABLED)
			rc = -EFAULT;
	}
	return rc;
}

static int adp_camera_v4l2_streamoff(struct file *file, void *fh,
					enum v4l2_buf_type i)
{
	int rc;
	INIT_COMPLETION(preview_disabled);
	rc = mdp_arb_set_event(0);
	if (!rc) {
		wait_for_completion(&preview_disabled);
		if (adp_cam_ctxt->state != CAMERA_PREVIEW_DISABLED)
			rc = -EFAULT;
	}
	return rc;
}

static const struct v4l2_ioctl_ops adp_camera_v4l2_ioctl_ops = {
	.vidioc_querycap = adp_camera_v4l2_querycap,
	.vidioc_cropcap = adp_camera_v4l2_cropcap,
	.vidioc_g_crop = adp_camera_v4l2_g_crop,
	.vidioc_s_crop = adp_camera_v4l2_s_crop,
	.vidioc_streamon = adp_camera_v4l2_streamon,
	.vidioc_streamoff = adp_camera_v4l2_streamoff,
};


static void adp_camera_release_video_device(struct video_device *pvdev)
{
}

static const struct v4l2_file_operations adp_camera_v4l2_fops = {
	.owner = THIS_MODULE,
	.open = adp_camera_v4l2_open,
	.release = adp_camera_v4l2_close,
	.ioctl = video_ioctl2,
};

#define ADP_CAMERA_BASE_DEVICE_NUMBER 36
#define ADP_CAMERA_DRV_NAME "adp_camera_driver"

#define ADP_CAMERA_V4L2_CID_Z_ORDER ((V4L2_CID_USER_BASE | 0x7000)+1)
#define ADP_CAMERA_V4L2_CID_DISPLAY_ID ((V4L2_CID_USER_BASE | 0x7000)+2)

static int adp_camera_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	pr_debug("%s - Enter %d. %d", __func__, ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case ADP_CAMERA_V4L2_CID_Z_ORDER:
		adp_cam_ctxt->z_order[adp_cam_ctxt->display_id] = ctrl->val;
		rc = preview_overlay_update();
		if (rc)
			pr_err("%s - preview_overlay_update failed", __func__);
		break;
	case ADP_CAMERA_V4L2_CID_DISPLAY_ID:
		if (CAMERA_PREVIEW_ENABLED == adp_cam_ctxt->state) {
			pr_err("%s - cannot set display id while preview running",
					__func__);
			return -EBUSY;
		}
		if (adp_cam_ctxt->fb_idx[ctrl->val] < 0) {
			pr_err("%s - invalid display id %d. "\
					"Display may not be connected",
					__func__, ctrl->val);
			return -EINVAL;
		}

		if (adp_camera_query_display(adp_cam_ctxt->display_id))
			return -EINVAL;

		adp_cam_ctxt->display_id = ctrl->val;
		break;
	default:
		pr_err("%s - non supported ctrl id %d", __func__, ctrl->id);
		rc = -EINVAL;
		break;
	}

	pr_debug("%s - Exit", __func__);

	return rc;
}

static const struct v4l2_ctrl_ops adp_camera_ctrl_ops = {
	.s_ctrl = adp_camera_s_ctrl,
};


static const struct v4l2_ctrl_config adp_camera_ctrl_cfg[] = {
	{
		.ops = &adp_camera_ctrl_ops,
		.id = ADP_CAMERA_V4L2_CID_Z_ORDER,
		.name = "z_order",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 3,
		.def = 1,
		.step = 1,
		.is_private = 1,
	},
	{
		.ops = &adp_camera_ctrl_ops,
		.id = ADP_CAMERA_V4L2_CID_DISPLAY_ID,
		.name = "display_id",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = DISPLAY_ID_MAX-1,
		.def = 0,
		.step = 1,
		.is_private = 1,
	}
};

#define ADP_CAMERA_NUM_CTRLS ARRAY_SIZE(adp_camera_ctrl_cfg)

static int adp_camera_init_v4l2_ctrls(void)
{
	int rc, i;

	pr_debug("%s - Enter", __func__);

	rc = v4l2_ctrl_handler_init(&adp_cam_ctxt->ctrl_handler,
			ADP_CAMERA_NUM_CTRLS);

	if (rc || adp_cam_ctxt->ctrl_handler.error) {
		pr_err("Failed v4l2_ctrl_handler_init %d", rc);
		return rc;
	}
	adp_cam_ctxt->v4l2_dev.ctrl_handler = &adp_cam_ctxt->ctrl_handler;

	for (i = 0; i < ADP_CAMERA_NUM_CTRLS; i++)
		v4l2_ctrl_new_custom(&adp_cam_ctxt->ctrl_handler,
			&adp_camera_ctrl_cfg[i], NULL);

	if (adp_cam_ctxt->ctrl_handler.error) {
		int err = adp_cam_ctxt->ctrl_handler.error;
		pr_err("%s - failed to create custom ctrls %d", __func__, err);
		v4l2_ctrl_handler_free(&adp_cam_ctxt->ctrl_handler);
		return err;
	}

	/* set controls to default values */
	v4l2_ctrl_handler_setup(&adp_cam_ctxt->ctrl_handler);

	pr_debug("%s - Exit", __func__);

	return rc;
}

static void adp_camera_deinit_v4l2_ctrls(void)
{
	v4l2_ctrl_handler_free(&adp_cam_ctxt->ctrl_handler);
}

static int adp_camera_device_init(struct platform_device *pdev)
{
	int nr = ADP_CAMERA_BASE_DEVICE_NUMBER;
	int rc = 0;

	pr_debug("Enter %s\n", __func__);

	adp_cam_ctxt = kzalloc(sizeof(struct adp_camera_ctxt), GFP_KERNEL);
	if (NULL == adp_cam_ctxt) {
		pr_err("Failed to allocate adp context\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&adp_cam_ctxt->recovery_work,
						adp_camera_recovery_work);
	adp_cam_ctxt->state = CAMERA_UNINITIALIZED;

	rc = mdp_arb_register_event();
	if (rc) {
		pr_err("Failed mdp_arb_register_event %d\n", rc);
		goto mdp_arb_register_event_failed;
	}


	strlcpy(adp_cam_ctxt->v4l2_dev.name, ADP_CAMERA_DRV_NAME,
		sizeof(adp_cam_ctxt->v4l2_dev.name));
	adp_cam_ctxt->v4l2_dev.dev = &pdev->dev;

	rc = v4l2_device_register(adp_cam_ctxt->v4l2_dev.dev,
			&adp_cam_ctxt->v4l2_dev);
	if (rc) {
		pr_err("Failed to register v4l2 device\n");
		goto v4l2_device_register_failed;
	}

	rc = adp_camera_init_v4l2_ctrls();
	if (rc) {
		pr_err("adp_camera_init_v4l2_ctrls failed\n");
		goto adp_camera_init_v4l2_ctrls_failed;
	}

	adp_cam_ctxt->vdev = video_device_alloc();
	if (NULL == adp_cam_ctxt->vdev) {
		pr_err("%s - Failed video_device_alloc\n",
				__func__);
		rc = -ENOMEM;
		goto video_device_alloc_failed;
	}

	strlcpy(adp_cam_ctxt->vdev->name,
			pdev->name, sizeof(adp_cam_ctxt->vdev->name));
	adp_cam_ctxt->vdev->v4l2_dev =
			&adp_cam_ctxt->v4l2_dev;
	adp_cam_ctxt->vdev->release =
			adp_camera_release_video_device;
	adp_cam_ctxt->vdev->fops =
			&adp_camera_v4l2_fops;
	adp_cam_ctxt->vdev->ioctl_ops =
			&adp_camera_v4l2_ioctl_ops;
	adp_cam_ctxt->vdev->minor = nr;
	adp_cam_ctxt->vdev->vfl_type = VFL_TYPE_GRABBER;

	video_set_drvdata(adp_cam_ctxt->vdev, &adp_cam_ctxt);

	strlcpy(adp_cam_ctxt->mdev.model, ADP_CAMERA_DRV_NAME,
			sizeof(adp_cam_ctxt->mdev.model));
	adp_cam_ctxt->mdev.dev = &pdev->dev;
	rc = media_device_register(&adp_cam_ctxt->mdev);
	if (rc) {
		pr_err("%s - Failed media_device_register\n",
				__func__);
		goto media_device_register_failed;
	}

	adp_cam_ctxt->v4l2_dev.mdev = &adp_cam_ctxt->mdev;
	rc = media_entity_init(&adp_cam_ctxt->vdev->entity,
			0, NULL, 0);
	if (rc) {
		pr_err("%s - Failed media_entity_init\n", __func__);
		goto media_entity_init_failed;
	}

	adp_cam_ctxt->vdev->entity.type =
			MEDIA_ENT_T_DEVNODE_V4L;
	adp_cam_ctxt->vdev->entity.group_id = 2;

	rc = video_register_device(adp_cam_ctxt->vdev,
			VFL_TYPE_GRABBER, nr);
	if (rc) {
		pr_err("%s - Failed video_register_device\n",
				__func__);
		goto video_register_device_failed;
	}

	adp_cam_ctxt->vdev->entity.name =
			video_device_node_name(adp_cam_ctxt->vdev);

	pr_debug("Exit %s with error %d\n", __func__, rc);
	return 0;

video_register_device_failed:
	media_entity_cleanup(&adp_cam_ctxt->vdev->entity);
media_entity_init_failed:
	media_device_unregister(&adp_cam_ctxt->mdev);
media_device_register_failed:
	video_device_release(adp_cam_ctxt->vdev);
video_device_alloc_failed:
	adp_camera_deinit_v4l2_ctrls();
adp_camera_init_v4l2_ctrls_failed:
	v4l2_device_unregister(&adp_cam_ctxt->v4l2_dev);
v4l2_device_register_failed:
	mdp_arb_deregister_event();
mdp_arb_register_event_failed:
	kfree(adp_cam_ctxt);
	adp_cam_ctxt = NULL;
	return rc;
}

static int adp_camera_destroy(void)
{
	media_entity_cleanup(&adp_cam_ctxt->vdev->entity);
	media_device_unregister(&adp_cam_ctxt->mdev);
	video_unregister_device(adp_cam_ctxt->vdev);
	video_device_release(adp_cam_ctxt->vdev);
	adp_camera_deinit_v4l2_ctrls();
	v4l2_device_unregister(&adp_cam_ctxt->v4l2_dev);
	mdp_arb_deregister_event();
	kfree(adp_cam_ctxt);

	return 0;
}

static int init_camera_kthread(void)
{
	int ret;
	pr_debug("%s: entry\n", __func__);
	init_completion(&preview_enabled);
	init_completion(&preview_disabled);
	INIT_WORK(&wq_mdp_queue_overlay_buffers, mdp_queue_overlay_buffers);
	ret = adp_rear_camera_enable();
	if (ret)
		return ret;

	adp_cam_ctxt->state = CAMERA_PREVIEW_DISABLED;
	complete(&preview_disabled);

	pr_debug("%s: exit\n", __func__);
	place_marker("rvc thread enabled");

	return 0;
}

static void exit_camera_kthread(void)
{
	pr_debug("%s: entry\n", __func__);

	if (CAMERA_PREVIEW_ENABLED == adp_cam_ctxt->state)
		disable_camera_preview();

	wait_for_completion(&preview_disabled);
	preview_buffer_free();
	pr_debug("%s: begin axi release\n", __func__);

	adp_rear_camera_disable();
	cancel_work_sync(&wq_mdp_queue_overlay_buffers);

	adp_cam_ctxt->state = CAMERA_UNINITIALIZED;

	place_marker("rvc thread disabled");

	pr_debug("%s: exit\n", __func__);
}


static int32_t adp_camera_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;

	pr_debug("adp_camera platform platform probe...\n");

	init_pipeline = INIT_PIPELINE;

	rc = adp_camera_device_init(pdev);
	if (rc < 0) {
		pr_err("%s: Failed to init device %d", __func__, rc);
		return rc;
	}

	rc = init_camera_kthread();
	if (rc) {
		adp_camera_destroy();
		pr_err("%s: Failed to init the camera %d", __func__, rc);
		return rc;
	}

	pdev->dev.platform_data = adp_cam_ctxt;

	pr_debug("adp_camera platform probe exit %d..\n", rc);
	return rc;
}

static int adp_camera_remove(struct platform_device *pdev)
{
	pr_debug("Enter %s\n", __func__);

	exit_camera_kthread();
	adp_camera_destroy();

	pr_debug("Exit %s\n", __func__);

	return 0;
}

static int adp_camera_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	pr_debug("suspend rearview camera %s", __func__);

	switch (adp_cam_ctxt->state) {
	case CAMERA_PREVIEW_DISABLED:
	case CAMERA_TRANSITION_OFF:
		break;
	case CAMERA_SUSPENDED:
		pr_warn("%s - preview already suspended", __func__);
		return 0;
		break;
	default:
		pr_err("%s - cannot suspend from state %d",
				__func__, adp_cam_ctxt->state);
		return -EPERM;
		break;
	}

	wait_for_completion(&preview_disabled);
	adp_cam_ctxt->state = CAMERA_TRANSITION_SUSPEND;

	msm_axi_subdev_release_rdi_only(lsh_axi_ctrl, s_ctrl);
	msm_csid_release(lsh_csid_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			MM_CAM_USE_BYPASS);
	msm_csiphy_release(lsh_csiphy_dev[adp_rvc_csi_lane_params.csi_phy_sel],
			&adp_rvc_csi_lane_params);
	msm_ispif_release_rdi(lsh_ispif);

	INIT_COMPLETION(preview_disabled);
	adp_cam_ctxt->state = CAMERA_SUSPENDED;

	pr_debug("exit %s", __func__);

	return 0;
}

static int adp_camera_resume(struct platform_device *pdev)
{
	int rc = 0;
	pr_debug("resume rearview camera %s", __func__);

	switch (adp_cam_ctxt->state) {
	case CAMERA_SUSPENDED:
		break;
	case CAMERA_PREVIEW_DISABLED:
		pr_warn("%s - preview already resumed", __func__);
		return 0;
		break;
	default:
		pr_err("%s - cannot resume from state %d",
				__func__, adp_cam_ctxt->state);
		return -EPERM;
		break;
	}

	adp_cam_ctxt->state = CAMERA_TRANSITION_OFF;

	preview_set_data_pipeline();

	adp_cam_ctxt->state = CAMERA_PREVIEW_DISABLED;
	complete(&preview_disabled);

	return rc;
}

static struct platform_driver adp_camera_platform_driver = {
	.probe = adp_camera_platform_probe,
	.remove = adp_camera_remove,
	.suspend = adp_camera_suspend,
	.resume	= adp_camera_resume,
	.driver = {
		.name = "adp_camera",
		.owner = THIS_MODULE,
	},
};

static int __init adp_camera_init_module(void)
{
	int32_t rc = 0;
	pr_debug("adp_camera platform device init module...\n");

	rc = platform_driver_register(&adp_camera_platform_driver);
	return rc;
}

static void __exit adp_camera_exit_module(void)
{
	platform_driver_unregister(&adp_camera_platform_driver);
	return;
}

module_init(adp_camera_init_module);
module_exit(adp_camera_exit_module);
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. adp_camera sensor driver");
MODULE_LICENSE("GPL v2");
