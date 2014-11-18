/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include "adp_camera.h"
#include "background.h"
#include "left_lane.h"
#include "right_lane.h"
#include "av_mgr.h"

static void *k_addr[OVERLAY_COUNT];
static int alloc_overlay_pipe_flag[OVERLAY_COUNT];
static struct mdp_overlay overlay_req[OVERLAY_COUNT];
static unsigned int overlay_fd[OVERLAY_COUNT];
static struct msmfb_overlay_data overlay_data[OVERLAY_COUNT];
static int rc = -ENOIOCTLCMD;
static uint8_t dual_enabled;
static struct msm_sensor_ctrl_t *s_ctrl;
static struct msm_ispif_params_list params_list;
struct sensor_init_cfg *init_info;
static uint32_t csid_version;
static int rdi1_irq_count;
struct mdp4_overlay_pipe *pipe[OVERLAY_COUNT];
static struct vfe_axi_output_config_cmd_type vfe_axi_cmd_para;
static struct msm_camera_vfe_params_t vfe_para;
static struct work_struct wq_mdp_queue_overlay_buffers;
static int adp_rear_camera_enable(void);
struct completion camera_enabled;
struct completion preview_enabled;
struct completion preview_disabled;

struct msm_camera_csiphy_params csiphy_params = {
	.lane_cnt = 1,
	.settle_cnt = 0x14,
	.lane_mask = 0x1,
};
struct msm_camera_csi_lane_params csi_lane_params = {
	.csi_lane_assign = 0xe4,
	.csi_lane_mask = 0x1,
	.csi_phy_sel = 0,
};
struct msm_camera_csid_vc_cfg mt9m114_cid_cfg1[] = {
	{0, CSI_YUV422_8, CSI_DECODE_8BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};
struct msm_camera_csid_params csid_params = {
	.lane_cnt = 1,
	.lane_assign = 0xe4,
	.lut_params = {
		.num_cid = ARRAY_SIZE(mt9m114_cid_cfg1),
		.vc_cfg = mt9m114_cid_cfg1,
	},
};
static struct msm_camera_preview_data preview_data;

/* -------------------- Guidance Lane Data Structures ----------------------- */
static struct msm_guidance_lane_data guidance_lane_data;

int axi_vfe_config_cmd_para(struct vfe_axi_output_config_cmd_type *cmd)
{
	/* configure the axi bus parameters here */
	int config;
	int ch_wm;
	int axi_output_ppw;
	int ch0_wm;
	int image_width;
	int image_height;
	int burst_length;
	image_width = PREVIEW_WIDTH*2;
	image_height = PREVIEW_HEIGHT;
	cmd->busio_format = 0;
	cmd->bus_cmd = 0;
	cmd->bus_cfg = 0x2aaa771;
	cmd->bus_wr_skip_cfg  = 0;
	cmd->rdi_cfg0.rdi_enable = 0x1;
	cmd->rdi_cfg0.rdi_stream_select1 = 0x3;
	cmd->rdi_cfg0.rdi_m3_select = 0x0;
	config = 0x01 | (0x06 << 5);
	ch_wm = 3;
	cmd->xbar_cfg0 = 0;
	cmd->xbar_cfg0 = (cmd->xbar_cfg0) & ~(0x000000FF << (8 * (ch_wm % 4)));
	cmd->xbar_cfg0 = (cmd->xbar_cfg0) | (config << (8 * (ch_wm % 4)));
	ch0_wm = 3;
	axi_output_ppw = 8;
	burst_length = 1;
	cmd->wm[ch0_wm].busdwords_per_line = 89;
	cmd->wm[ch0_wm].busrow_increment =
		(image_width+(axi_output_ppw-1))/(axi_output_ppw);
	cmd->wm[ch0_wm].buslines_per_image = image_height - 1;
	cmd->wm[ch0_wm].busbuffer_num_rows = image_height - 1;
	cmd->wm[ch0_wm].busburst_length = burst_length;
	cmd->wm[ch0_wm].bus_ub_offset = 0;
	cmd->wm[ch0_wm].bus_ub_depth = 0x3f0;

	return 0;
}

void preview_set_data_pipeline()
{
	int ispif_stream_enable;
	u32 freq = 128000000;
	u32 flags = 0;

	pr_debug("%s:  kpi entry!!!\n", __func__);
	dual_enabled = 0; /* set according to log */
	/* power on and enable  clock for vfe and axi, before csi */
	rc = msm_axi_subdev_init_rdi_only(lsh_axi_ctrl, dual_enabled, s_ctrl);
	if (rc < 0)
		return;

	msm_axi_subdev_s_crystal_freq(lsh_axi_ctrl, freq,  flags);
	csid_version = 1;
	rc = msm_csid_init(lsh_csid_dev, &csid_version); /* CSID_INIT */
	rc = msm_csiphy_init(lsh_csiphy_dev); /* CSIPHY_INIT */
	rc = msm_ispif_init(lsh_ispif, &csid_version); /* ISPIF_INIT */
	params_list.params[0].intftype =  RDI1; /* RDI1 */
	params_list.params[0].csid = 0;
	params_list.params[0].vfe_intf =  VFE0;
	params_list.params[0].cid_mask = (1 << 0);
	params_list.len = 1;

	pr_debug("%s: config ispif\n", __func__);

	/* ISPIF_CFG,use csid 1,rdi1 */
	rc = msm_ispif_config(lsh_ispif, &params_list);
	rc = msm_csid_config(lsh_csid_dev, &csid_params); /* CSID_CFG */

	pr_debug("%s: config csid\n", __func__);

	/* CSIPHY_CFG */
	rc = msm_csiphy_lane_config(lsh_csiphy_dev, &csiphy_params);
	ispif_stream_enable = 129;  /* configure to select RDI 1, VFE0 */
	msm_ispif_subdev_video_s_stream_rdi_only(lsh_ispif,
			ispif_stream_enable);
	pr_debug("%s: begin axi reset!!!\n", __func__);
	axi_reset_rdi1_only(my_axi_ctrl, vfe_para);
	pr_debug("%s: vfe32_config_axi now!!!\n", __func__);
	axi_vfe_config_cmd_para(&vfe_axi_cmd_para);
	rc = vfe32_config_axi_rdi_only(my_axi_ctrl, OUTPUT_TERT2,
			(uint32_t *)&vfe_axi_cmd_para);

	pr_debug("%s: kpi exit!!!\n", __func__);
}

void preview_buffer_alloc(void)
{
	int i, result;
	int offset = 0;
	int mem_len;
	unsigned long paddr;

	memset(&preview_data, 0, sizeof(struct msm_camera_preview_data));
	preview_data.ion_client = msm_ion_client_create(-1, "camera");
	if (IS_ERR_OR_NULL((void *)preview_data.ion_client)) {
		pr_err("%s: ION create client failed\n", __func__);
		goto err;
	}
	/* ION_CP_MM_HEAP_ID size is 0x7800000 */
	preview_data.ion_handle = ion_alloc(preview_data.ion_client,
	/* add 32 to align with 8 for physical address align to 8 */
				(PREVIEW_BUFFER_SIZE + 32),
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
	result = ion_map_iommu(preview_data.ion_client, preview_data.ion_handle,
			CAMERA_DOMAIN, GEN_POOL, SZ_4K, 0,
			(unsigned long *)&paddr,
			(unsigned long *)&mem_len, 0, 1);
	if (result < 0) {
		pr_err("%s Could not get  address\n", __func__);
		goto err_ion_handle;
	}
	overlay_fd[OVERLAY_CAMERA_PREVIEW] = (unsigned int) ion_share_dma_buf(
						preview_data.ion_client,
						preview_data.ion_handle);
	paddr = ((paddr + 7) & 0xFFFFFFF8); /* to align with 8 */
	/* Make all phys point to the correct address */
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		/* Y plane */
		preview_data.preview_buffer[i].cam_preview.ch_paddr[0] =
			(uint32_t)(paddr + offset);
		/* add offset for display use */
		/* if paddr last byte is 5, we need to add 3 to align to 8 */
		preview_data.preview_buffer[i].offset =
			offset + (8 - (paddr & 7));
		offset += PREVIEW_BUFFER_LENGTH;
		/* this the offset from start address ion_handle,
		 * as align to 8
		 */
	}
	return;
err_ion_handle:
	ion_free(preview_data.ion_client, preview_data.ion_handle);
err_ion_client:
	ion_client_destroy(preview_data.ion_client);
err:
	return;
}

int preview_buffer_init(void)
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

void preview_buffer_return(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
	   set the buffer state as initialized and can be reused */
	int i;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0] ==
				(uint32_t)(paddr)) {
			preview_data.preview_buffer[i].state =
					CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED;
			pr_debug("%s: phys addr = 0x%x\n", __func__,
			preview_data.preview_buffer[i].cam_preview.ch_paddr[0]);
			break;
		}
	}
	return;
}

struct preview_mem *preview_find_buffer_by_paddr(uint32_t paddr)
{
	/* search in the 4 buffer list, if the physical address matches,
		set the buffer state as initialized and can be reused */
	int i;
	struct preview_mem *buf = NULL;
	for (i = 0; i < PREVIEW_BUFFER_COUNT; i++) {
		if (preview_data.preview_buffer[i].cam_preview.ch_paddr[0]
				== (uint32_t)(paddr)) {
			/* or we can return offset here directly
			 * we need not to define buffer here
			 */
			buf = &(preview_data.preview_buffer[i]);
			break;
			}
		}
	return buf;
}

int preview_find_buffer_index_by_paddr(uint32_t paddr)
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

struct preview_mem *preview_buffer_find_free_for_ping_pong()
{
	int ret = 0;
	struct preview_mem *buf = NULL;

	pr_debug("%s: begin to find free buffer for ping pong\n",
			__func__);
	list_for_each_entry(buf, &(preview_data.camera_preview_list), list) {
		if (buf->state == CAMERA_PREVIEW_BUFFER_STATE_INITIALIZED)
			break;
		ret = 1;
	}
	if (IS_ERR_OR_NULL((void *) buf))
		pr_err("%s: can not find free buffer\n", __func__);

	pr_debug("%s: free buff find finished, physical addr is %x\n",
		__func__, buf->cam_preview.ch_paddr[0]);
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
	ion_unmap_iommu(preview_data.ion_client, preview_data.ion_handle,
			CAMERA_DOMAIN, GEN_POOL);
	ion_free(preview_data.ion_client, preview_data.ion_handle);
	ion_client_destroy(preview_data.ion_client);
	return ret = 0;
}

static void preview_configure_bufs()
{
	/* step 1, alloc ION buffer */
	pr_debug("%s: preview buffer allocate\n", __func__);
	preview_buffer_alloc();

	/*step2, buffer linked list */
	preview_buffer_init();
}

static void preview_set_overlay_init(struct mdp_overlay *overlay)
{
	overlay->id = MSMFB_NEW_REQUEST;
	overlay->src.width  = PREVIEW_WIDTH;
	overlay->src.height = PREVIEW_HEIGHT;
	overlay->src.format = MDP_YCRYCB_H2V1;
	overlay->src_rect.x = 0;
	overlay->src_rect.y = 6;
	overlay->src_rect.w = PREVIEW_WIDTH;
	overlay->src_rect.h = 240;
	overlay->dst_rect.x = 0;
	overlay->dst_rect.y = 0;
	overlay->dst_rect.w = 1280;
	overlay->dst_rect.h = 720;
	overlay->z_order =  2;
	overlay->alpha = MDP_ALPHA_NOP;
	overlay->transp_mask = MDP_TRANSP_NOP;
	overlay->flags = 0;
	overlay->is_fg = 0;

	return;
}

void format_convert(int index)
{
	int i;
	int8 *data;
	data = (int8 *)k_addr[OVERLAY_CAMERA_PREVIEW];
	data += PREVIEW_WIDTH*PREVIEW_HEIGHT*2*index;
	for (i = 0; i < PREVIEW_WIDTH*PREVIEW_HEIGHT/4 ; i++)
		__swab32s((uint32 *)(data+4*i));
}

static void mdp_queue_overlay_buffers(struct work_struct *work)
{
	mdpclient_overlay_play(&overlay_data[OVERLAY_CAMERA_PREVIEW]);
	mdpclient_overlay_play(&overlay_data[OVERLAY_GUIDANCE_LANE]);

	/* perform display commit */
	mdpclient_display_commit();
}

void vfe32_process_output_path_irq_rdi1_only(struct axi_ctrl_t *axi_ctrl)
{
	uint32_t ping_pong;
	uint32_t ch0_paddr = 0;
	/* this must be rdi image output. */
	struct preview_mem *free_buf = NULL;
	int buffer_index;

	pr_debug("%s:  enter into rdi1 irq now, rdi1_irq_count is %d!!!\n",
			__func__, rdi1_irq_count);
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

			/* configure overlay data for guidance lane */
			overlay_data[OVERLAY_CAMERA_PREVIEW].id =
					overlay_req[OVERLAY_CAMERA_PREVIEW].id;
			overlay_data[OVERLAY_CAMERA_PREVIEW].data.flags =
				MSMFB_DATA_FLAG_ION_NOT_FD;
			overlay_data[OVERLAY_CAMERA_PREVIEW].data.offset =
				(buffer_index * PREVIEW_BUFFER_LENGTH);
			overlay_data[OVERLAY_CAMERA_PREVIEW].data.memory_id =
					overlay_fd[OVERLAY_CAMERA_PREVIEW];

			/*  configure overlay data for guidance lane */
			overlay_data[OVERLAY_GUIDANCE_LANE].id =
					overlay_req[OVERLAY_GUIDANCE_LANE].id;
			overlay_data[OVERLAY_GUIDANCE_LANE].data.flags =
				MSMFB_DATA_FLAG_ION_NOT_FD;
			overlay_data[OVERLAY_GUIDANCE_LANE].data.offset =
				(buffer_index * GUIDANCE_LANE_BUFFER_LENGTH);
			overlay_data[OVERLAY_GUIDANCE_LANE].data.memory_id =
					overlay_fd[OVERLAY_GUIDANCE_LANE];

			/* Only log once for early camera kpi */
			pr_info_once("kpi cam rdi1 first frame %d\n",
				      rdi1_irq_count);

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
			preview_buffer_return(ch0_paddr);
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

/* Guidance Bitmap Implememtation */

void guidance_lane_buffer_alloc(void)
{
	int i, result;
	int offset = 0;
	int mem_len;
	unsigned long paddr;
	memset(&guidance_lane_data, 0, sizeof(struct msm_guidance_lane_data));
	guidance_lane_data.ion_client = msm_ion_client_create(-1, "camera");
	if (IS_ERR_OR_NULL((void *)guidance_lane_data.ion_client)) {
		pr_err("%s: ION create client failed\n", __func__);
		goto err;
	}
	/* ION_CP_MM_HEAP_ID size is 0x7800000 */
	guidance_lane_data.ion_handle = ion_alloc(guidance_lane_data.ion_client,
	/* add 32 to align with 8 for physical address align to 8 */
			(GUIDANCE_LANE_BUFFER_SIZE + 32),
			SZ_4K,
			(0x1 << ION_CP_MM_HEAP_ID | 0x1 << ION_IOMMU_HEAP_ID),
			0);
	if (IS_ERR_OR_NULL((void *) guidance_lane_data.ion_handle)) {
		pr_err("%s: ION memory allocation failed\n", __func__);
		goto err_ion_client;
	}
	 k_addr[OVERLAY_GUIDANCE_LANE] =
			ion_map_kernel(guidance_lane_data.ion_client,
			guidance_lane_data.ion_handle);
	result = ion_map_iommu(guidance_lane_data.ion_client,
			 guidance_lane_data.ion_handle,
			 CAMERA_DOMAIN, GEN_POOL, SZ_4K, 0,
			(unsigned long *)&paddr,
			(unsigned long *)&mem_len, 0, 1);
	if (result < 0) {
			pr_err("%s Could not get  address\n", __func__);
			goto err_ion_handle;
	}
	overlay_fd[OVERLAY_GUIDANCE_LANE] = (unsigned int)ion_share_dma_buf(
			guidance_lane_data.ion_client,
			guidance_lane_data.ion_handle);
	paddr = ((paddr + 7) & 0xFFFFFFF8); /* to align with 8 */
	/* Make all phys point to the correct address */
	for (i = 0; i < GUIDANCE_LANE_BUFFER_COUNT; i++) {
		/* Y plane */
		guidance_lane_data.guidance_lane_buf[i].guidance_lane.\
			ch_paddr[0] = (uint32_t)(paddr + offset);
		/* add offset for display use */
		/* if paddr last byte is 5, we need to add 3 to align to 8 */
		guidance_lane_data.guidance_lane_buf[i].offset =
						offset + (8 - (paddr & 7));
		offset += GUIDANCE_LANE_BUFFER_LENGTH;
		/* this offset from start address ion_handle, as align to 8 */
	}
	return;
err_ion_handle:
	ion_free(guidance_lane_data.ion_client, guidance_lane_data.ion_handle);
err_ion_client:
	ion_client_destroy(guidance_lane_data.ion_client);
err:
	return;
}

int guidance_lane_buffer_init(void)
{
	int i ;
	/* initialized the list head and add all nodes */
	pr_debug("%s begin to setup buffer list\n", __func__);

	INIT_LIST_HEAD(&guidance_lane_data.guidance_lane_list);

	for (i = 0; i < GUIDANCE_LANE_BUFFER_COUNT; i++) {
		list_add(&(guidance_lane_data.guidance_lane_buf[i].list),
		 &(guidance_lane_data.guidance_lane_list));
		guidance_lane_data.guidance_lane_buf[i].state =
					GUIDANCE_LANE_BUFFER_STATE_INITIALIZED;
	}

	pr_debug("%s setup buffer list\n", __func__);
	return 0;
}

static int guidance_lane_buffer_free(void)
{
	int ret;
	ion_unmap_iommu(guidance_lane_data.ion_client,
			guidance_lane_data.ion_handle,
			CAMERA_DOMAIN, GEN_POOL);
	ion_free(guidance_lane_data.ion_client, guidance_lane_data.ion_handle);
	ion_client_destroy(guidance_lane_data.ion_client);
	return ret = 0;
}

static void guidance_lane_configure_bufs()
{
	GUIDANCE_LANE_WIDTH = (*(background + 1) << 8) + *(background + 0);
	GUIDANCE_LANE_HEIGHT = (*(background + 3) << 8) + *(background + 2);

	GUIDANCE_LANE_BUFFER_LENGTH =
		GUIDANCE_LANE_WIDTH * GUIDANCE_LANE_HEIGHT * 3;
	GUIDANCE_LANE_BUFFER_SIZE =
		GUIDANCE_LANE_BUFFER_COUNT * GUIDANCE_LANE_BUFFER_LENGTH;

	/* step 1, alloc ION buffer */
	guidance_lane_buffer_alloc();

	/*step2, buffer linked list */
	guidance_lane_buffer_init();
}


static void guidance_lane_set_overlay_init(struct mdp_overlay *overlay)
{
	overlay->id = MSMFB_NEW_REQUEST;
	overlay->src.width  = GUIDANCE_LANE_WIDTH;
	overlay->src.height = GUIDANCE_LANE_HEIGHT;
	overlay->src.format = MDP_RGB_888;
	overlay->src_rect.x = 0;
	overlay->src_rect.y = 0;
	overlay->src_rect.w = GUIDANCE_LANE_WIDTH;
	overlay->src_rect.h = GUIDANCE_LANE_HEIGHT;
	overlay->dst_rect.x = 0;
	overlay->dst_rect.y = 0;
	overlay->dst_rect.w = GUIDANCE_LANE_WIDTH;
	overlay->dst_rect.h = GUIDANCE_LANE_HEIGHT;
	overlay->z_order = 3;
	overlay->alpha = 0x80;
	overlay->transp_mask = 0x020202;
	overlay->flags = MDP_ROT_NOP;
	overlay->is_fg = 1;
	return;
}

static void guidance_lane_pic_update(const unsigned char *pic,
		unsigned int pos_x, unsigned int pos_y, unsigned int image_w,
		unsigned int image_h, unsigned int buffer_index)
{
	int i;
	unsigned int bpp = 3;
	unsigned char *buffer = k_addr[OVERLAY_GUIDANCE_LANE];
	buffer += (buffer_index * GUIDANCE_LANE_BUFFER_LENGTH);
	for (i = 0; i < image_h; i++)
		memcpy(buffer + GUIDANCE_LANE_WIDTH * bpp * i + pos_x*bpp +
				pos_y * GUIDANCE_LANE_WIDTH * bpp,
				pic + (image_w * bpp * i), (image_w * bpp));
}

static void guidance_lane_pic_update_all(void)
{
	const unsigned char *data = background + 4;
	int i;
	int w = (*(background + 1) << 8) + *(background + 0);
	int h = (*(background + 3) << 8) + *(background + 2);

	const unsigned char *data_a = left_lane + 4;
	int w_a = (*(left_lane + 1) << 8) + *(left_lane + 0);
	int h_a = (*(left_lane + 3) << 8) + *(left_lane + 2);

	const unsigned char *data_b = right_lane + 4;
	int w_b = (*(right_lane + 1) << 8) + *(right_lane + 0);
	int h_b = (*(right_lane + 3) << 8) + *(right_lane + 2);

	for (i = 0; i < GUIDANCE_LANE_BUFFER_COUNT; i++) {
		/* bg */
		guidance_lane_pic_update(data, 0, 0, w, h, i);
		/* 'A' */
		guidance_lane_pic_update(data_a, 170, 340, w_a, h_a, i);
		/* 'B' */
		guidance_lane_pic_update(data_b, 500, 340, w_b, h_b, i);
	}
}

static void guidance_lane_set_data_pipeline(void)
{
	pr_debug("%s entry\n", __func__);
	guidance_lane_pic_update_all();
	pr_debug("%s exit\n", __func__);
}

static int adp_rear_camera_enable(void)
{
	struct preview_mem *ping_buffer, *pong_buffer, *free_buffer;
	struct av_rvc_status rvc_status;

	pr_debug("%s: kpi entry\n", __func__);
	my_axi_ctrl->share_ctrl->current_mode = 4096;
	vfe_para.operation_mode = VFE_OUTPUTS_RDI1;

	/* Detect NTSC or PAL, get the preview width and height */
	av_mgr_rvc_enable(NTSC);
	av_mgr_rvc_get_status(&rvc_status);
	if (!rvc_status.signal_lock) {
		pr_err("%s: can't lock to video signals!\n", __func__);
		return -EAGAIN;
	}

	PREVIEW_HEIGHT = rvc_status.height;
	PREVIEW_WIDTH = rvc_status.width;

	PREVIEW_BUFFER_LENGTH =  PREVIEW_WIDTH * PREVIEW_HEIGHT*2;
	PREVIEW_BUFFER_SIZE  =  PREVIEW_BUFFER_COUNT * PREVIEW_BUFFER_LENGTH;
	preview_configure_bufs();
	preview_set_data_pipeline();
	/* step 1, find free buffer from the list, then use it to
	configure ping/pong */
	ping_buffer = preview_buffer_find_free_for_ping_pong();
	ping_buffer->state = CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
	pong_buffer = preview_buffer_find_free_for_ping_pong();
	pong_buffer->state = CAMERA_PREVIEW_BUFFER_STATE_QUEUED_TO_PINGPONG;
	free_buffer = preview_buffer_find_free_for_ping_pong();
	pr_debug("%s: find ping pong buffer end!!!\n", __func__);

	/* step 2, configure the free buffer to ping pong buffer */
	preview_configure_ping_pong_buffer(ping_buffer, pong_buffer,
		free_buffer);

	guidance_lane_configure_bufs();
	guidance_lane_set_data_pipeline();
	my_axi_ctrl->share_ctrl->current_mode = 4096; /* BIT(12) */
	my_axi_ctrl->share_ctrl->operation_mode = 4096;
	pr_debug("%s: kpi exit\n", __func__);

	return 0;
}

int  init_camera_kthread(void)
{
	int ret;
	pr_debug("%s: entry\n", __func__);
	init_completion(&camera_enabled);
	init_completion(&preview_enabled);
	init_completion(&preview_disabled);
	INIT_WORK(&wq_mdp_queue_overlay_buffers, mdp_queue_overlay_buffers);
	ret = adp_rear_camera_enable();
	complete(&camera_enabled);
	pr_debug("%s: exit\n", __func__);

	return 0;
}

void  exit_camera_kthread(void)
{
	pr_debug("%s: entry\n", __func__);
	wait_for_completion(&preview_disabled);
	guidance_lane_buffer_free();
	preview_buffer_free();
	pr_debug("%s: begin axi release\n", __func__);
	msm_axi_subdev_release_rdi_only(lsh_axi_ctrl, s_ctrl);
	msm_csid_release(lsh_csid_dev);
	msm_csiphy_release(lsh_csiphy_dev, &csi_lane_params);
	msm_ispif_release(lsh_ispif);
	cancel_work_sync(&wq_mdp_queue_overlay_buffers);

	pr_debug("%s: exit\n", __func__);
}

int disable_camera_preview(void)
{
	pr_debug("%s: kpi entry\n", __func__);

	wait_for_completion(&preview_enabled);
	axi_stop_rdi1_only(my_axi_ctrl);
	av_mgr_rvc_stream_enable(FALSE);
	mdpclient_display_commit();
	mdpclient_msm_fb_close();
	if (alloc_overlay_pipe_flag[OVERLAY_GUIDANCE_LANE] == 0) {
		mdpclient_overlay_unset(&overlay_req[OVERLAY_GUIDANCE_LANE]);
		pr_debug("%s: overlay_unset guidance lane free pipe !\n",
			__func__);
	}
	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] == 0) {
		mdpclient_overlay_unset(&overlay_req[OVERLAY_CAMERA_PREVIEW]);
		pr_debug("%s: overlay_unset camera preview free pipe !\n",
			__func__);
	}

	mdpclient_display_commit();

	complete(&preview_disabled);

	pr_debug("%s: kpi entry\n", __func__);
	return 0;
}

int enable_camera_preview(void)
{
	u64 mdp_max_bw_test = 2000000000;
	int waitcounter_camera = 0;
	int waitcounter_guidance = 0;
	int max_wait_count = 15;
	pr_debug("%s: kpi entry\n", __func__);

	wait_for_completion(&camera_enabled);
	mdpclient_msm_fb_open();
	if (mdpclient_msm_fb_blank(FB_BLANK_UNBLANK, true))
		pr_err("%s: can't turn on display!\n", __func__);
	mdp_bus_scale_update_request(mdp_max_bw_test, mdp_max_bw_test,
		mdp_max_bw_test, mdp_max_bw_test);

	/* configure pipe for reverse camera preview */
	preview_set_overlay_init(&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] =
		mdpclient_overlay_set(&overlay_req[OVERLAY_CAMERA_PREVIEW]);

	while (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] != 0 &&
				waitcounter_camera < max_wait_count) {
		msleep(100);
		waitcounter_camera++;
		alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] =
		mdpclient_overlay_set(&overlay_req[OVERLAY_CAMERA_PREVIEW]);
	}

	if (waitcounter_camera > 1)
		pr_err("%s: mdpclient_overlay_set camera wait counter value is: %d",
			__func__, waitcounter_camera);

	if (alloc_overlay_pipe_flag[OVERLAY_CAMERA_PREVIEW] != 0) {
		pr_err("%s: mdpclient_overlay_set error!1\n",
			__func__);
		complete(&camera_enabled);
		complete(&preview_enabled);
		return -EBUSY;
	}
	/* configure pipe for guidance lane */
	guidance_lane_set_overlay_init(&overlay_req[OVERLAY_GUIDANCE_LANE]);
	alloc_overlay_pipe_flag[OVERLAY_GUIDANCE_LANE] =
		mdpclient_overlay_set(&overlay_req[OVERLAY_GUIDANCE_LANE]);

	while (alloc_overlay_pipe_flag[OVERLAY_GUIDANCE_LANE] != 0 &&
				waitcounter_guidance < max_wait_count) {
		msleep(100);
		waitcounter_guidance++;
		alloc_overlay_pipe_flag[OVERLAY_GUIDANCE_LANE] =
		mdpclient_overlay_set(&overlay_req[OVERLAY_GUIDANCE_LANE]);
	}

	if (waitcounter_guidance > 1)
		pr_err("%s: mdpclient_overlay_set guidance lane wait counter value is: %d",
			__func__, waitcounter_guidance);

	if (alloc_overlay_pipe_flag[OVERLAY_GUIDANCE_LANE] != 0) {
		pr_err("%s: mdpclient_overlay_set error!1\n",
			__func__);
		complete(&camera_enabled);
		complete(&preview_enabled);
		return -EBUSY;
	}

	av_mgr_rvc_stream_enable(TRUE);
	axi_start_rdi1_only(my_axi_ctrl, s_ctrl);
	complete(&camera_enabled);
	complete(&preview_enabled);

	pr_debug("%s: kpi exit\n", __func__);
	return 0;
}
