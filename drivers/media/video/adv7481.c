/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/media.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <media/adv7481.h>

#include "adv7481_reg.h"
#include "msm/av_mgr.h"

#define DRIVER_NAME "adv7481"
#define I2C_RESET_DELAY		75000
#define I2C_RW_DELAY		75000
#define I2C_SW_DELAY		10000
#define GPIO_HW_DELAY_LOW	100000
#define GPIO_HW_DELAY_HI	10000
#define SDP_MIN_SLEEP		5000
#define SDP_MAX_SLEEP		6000
#define SDP_NUM_TRIES		30


struct adv7481_state {
	/* Platform Data*/
	struct adv7481_platform_data pdata;

	/* V4L2 Data*/
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_dv_timings timings;
	struct v4l2_ctrl *cable_det_ctrl;

	/*media entity controls*/
	struct media_pad pad;

	struct workqueue_struct *work_queues;
	struct mutex		mutex;

	struct i2c_client *client;
	struct i2c_client *i2c_csi_txa;
	struct i2c_client *i2c_csi_txb;
	struct i2c_client *i2c_hdmi;
	struct i2c_client *i2c_cp;
	struct i2c_client *i2c_sdp;
	struct i2c_client *i2c_rep;

	/* device status and Flags*/
	int powerup;
	/*routing configuration data*/
	int csia_src;
	int csib_src;
	int mode;
	/*CSI configuration data*/
	int txa_auto_params;
	int txa_lanes;
};

struct adv7481_hdmi_params {
	uint16_t pll_lock;
	uint16_t tmds_freq;
	uint16_t vert_lock;
	uint16_t horz_lock;
	uint16_t pix_rep;
	uint16_t color_depth;
};

struct adv7481_vid_params {
	uint16_t pix_clk;
	uint16_t act_pix;
	uint16_t act_lines;
	uint16_t tot_pix;
	uint16_t tot_lines;
	uint16_t fr_rate;
	uint16_t intrlcd;
};

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &(container_of(ctrl->handler,
			struct adv7481_state, ctrl_hdl)->sd);
}

static inline struct adv7481_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7481_state, sd);
}

/*I2C Rd/Rw Functions*/
static int adv7481_wr_byte(struct i2c_client *i2c_client, unsigned int reg,
	unsigned int value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c_client, reg & 0xFF, value);
	usleep(I2C_RW_DELAY);

	return ret;
}

static int adv7481_rd_byte(struct i2c_client *i2c_client, unsigned int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c_client, reg & 0xFF);
	usleep(I2C_RW_DELAY);

	return ret;
}

/*Initialize adv7481 I2C Settings*/
static int adv7481_dev_init(struct adv7481_state *state,
						struct i2c_client *client)
{
	int ret;
	mutex_lock(&state->mutex);

	/*Delay required following I2C reset and I2C transactions */
	/* soft reset */
	ret = adv7481_wr_byte(state->client, IO_REG_RST,
					IO_CTRL_MAIN_RST);
	usleep(I2C_SW_DELAY);

	/* power down controls */
	ret |= adv7481_wr_byte(state->client, IO_CTRL_MASTER_PWDN, 0x76);
	ret |= adv7481_wr_byte(state->client, IO_REG_CP_VID_STD, 0x4a);
	ret |= adv7481_wr_byte(state->client, IO_REG_I2C_CFG, 0x01);

	/*Configure I2C Maps and I2C Communication Settings*/
	ret |= adv7481_wr_byte(state->client,
					IO_REG_DPLL_ADDR, IO_REG_DPLL_SADDR);
	ret |= adv7481_wr_byte(state->client,
					IO_REG_CP_ADDR, IO_REG_CP_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_HDMI_ADDR,
					IO_REG_HDMI_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_EDID_ADDR,
					IO_REG_EDID_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_CSI_REP_ADDR,
					IO_REG_CSI_REP_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_HDMI_INF_ADDR,
					IO_REG_HDMI_INF_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_CBUS_ADDR,
					IO_REG_CBUS_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_CEC_ADDR,
					IO_REG_CEC_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_SDP_ADDR,
					IO_REG_SDP_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_CSI_TXA_ADDR,
					IO_REG_CSI_TXA_SADDR);
	ret |= adv7481_wr_byte(state->client, IO_REG_CSI_TXB_ADDR,
					IO_REG_CSI_TXB_SADDR);

	/*Configure i2c clients*/
	state->i2c_csi_txa = i2c_new_dummy(client->adapter, 0x94 >> 1);
	state->i2c_csi_txb = i2c_new_dummy(client->adapter, 0x88 >> 1);
	state->i2c_cp = i2c_new_dummy(client->adapter, 0x44 >> 1);
	state->i2c_hdmi = i2c_new_dummy(client->adapter, 0x74 >> 1);
	state->i2c_sdp = i2c_new_dummy(client->adapter, 0xf2 >> 1);
	state->i2c_rep = i2c_new_dummy(client->adapter, 0x64 >> 1);

	if (!state->i2c_csi_txa || !state->i2c_csi_txb || !state->i2c_cp ||
		!state->i2c_sdp || !state->i2c_hdmi || !state->i2c_rep) {
		pr_err("Additional I2C Client Fail\n");
		ret = EFAULT;
	}
	mutex_unlock(&state->mutex);

	return ret;
}

/*Initialize adv7481 hardware*/
static int adv7481_hw_init(struct adv7481_platform_data *pdata,
						struct adv7481_state *state)
{
	int ret;

	if (!pdata) {
		pr_err("PDATA is NULL\n");
		return -EFAULT;
	}

	mutex_lock(&state->mutex);
	if (gpio_is_valid(pdata->rstb_gpio)) {
		ret = gpio_request(pdata->rstb_gpio, "rstb_gpio");
		if (ret) {
			pr_err("Request GPIO Fail\n");
			return ret;
		}
		ret = gpio_direction_output(pdata->rstb_gpio, 0);
		usleep(GPIO_HW_DELAY_LOW);
		ret = gpio_direction_output(pdata->rstb_gpio, 1);
		usleep(GPIO_HW_DELAY_HI);
		if (ret) {
			pr_err("Set GPIO Fail\n");
			return ret;
		}
	}
	mutex_unlock(&state->mutex);

	return ret;
}

static int adv7481_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct adv7481_state *state = to_state(sd);
	int temp = 0x0;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		temp = adv7481_rd_byte(state->client, CP_REG_VID_ADJ);
		temp |= CP_CTR_VID_ADJ_EN;
		ret = adv7481_wr_byte(state->client, CP_REG_VID_ADJ, temp);
		ret |= adv7481_wr_byte(state->client,
				CP_REG_BRIGHTNESS, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		temp = adv7481_rd_byte(state->client, CP_REG_VID_ADJ);
		temp |= CP_CTR_VID_ADJ_EN;
		ret = adv7481_wr_byte(state->client, CP_REG_VID_ADJ, temp);
		ret |= adv7481_wr_byte(state->client,
				CP_REG_CONTRAST, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		temp = adv7481_rd_byte(state->client, CP_REG_VID_ADJ);
		temp |= CP_CTR_VID_ADJ_EN;
		ret = adv7481_wr_byte(state->client, CP_REG_VID_ADJ, temp);
		ret |= adv7481_wr_byte(state->client,
				CP_REG_SATURATION, ctrl->val);
		break;
	case V4L2_CID_HUE:
		temp = adv7481_rd_byte(state->client, CP_REG_VID_ADJ);
		temp |= CP_CTR_VID_ADJ_EN;
		ret = adv7481_wr_byte(state->client, CP_REG_VID_ADJ, temp);
		ret |= adv7481_wr_byte(state->client, CP_REG_HUE, ctrl->val);
		break;
	default:
		break;
	}
	return ret;
}

static int adv7481_powerup(struct adv7481_state *state, bool powerup)
{
	if (powerup)
		pr_debug("powered up\n");
	 else
		pr_debug("powered off\n");

	return 0;
}

static int adv7481_s_power(struct v4l2_subdev *sd, int on)
{
	struct adv7481_state *state = to_state(sd);
	int ret;

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return -EBUSY;

	ret = adv7481_powerup(state, on);
	if (ret == 0)
		state->powerup = on;

	mutex_unlock(&state->mutex);
	return ret;
}

static int adv7481_get_sd_timings(struct adv7481_state *state, int *sd_standard)
{
	int ret = 0;
	int sdp_stat, sdp_stat2;
	int timeout = 0;

	if (sd_standard == NULL)
		return -EINVAL;

	do {
		sdp_stat = adv7481_rd_byte(state->i2c_sdp, SDP_REG_STATUS1);
		usleep_range(SDP_MIN_SLEEP, SDP_MAX_SLEEP);
		timeout++;
		sdp_stat2 = adv7481_rd_byte(state->i2c_sdp, SDP_REG_STATUS1);
	} while ((sdp_stat != sdp_stat2) && (timeout < SDP_NUM_TRIES));

	if (sdp_stat != sdp_stat2) {
		pr_err("%s(%d), adv7481 SDP status unstable: 1 ",
							__func__, __LINE__);
		return -ETIMEDOUT;
	}

	if (!(sdp_stat & 0x01)) {
		pr_err("%s(%d), adv7481 SD Input NOT Locked: 1 ",
							__func__, __LINE__);
		return -EBUSY;
	}

	switch ((sdp_stat &= SDP_CTRL_ADRESLT) >> 4) {
	case AD_NTSM_M_J:
		*sd_standard = V4L2_STD_NTSC;
		break;
	case AD_NTSC_4_43:
		*sd_standard = V4L2_STD_NTSC_443;
		break;
	case AD_PAL_M:
		*sd_standard = V4L2_STD_PAL_M;
		break;
	case AD_PAL_60:
		*sd_standard = V4L2_STD_PAL_60;
		break;
	case AD_PAL_B_G:
		*sd_standard = V4L2_STD_PAL;
		break;
	case AD_SECAM:
		*sd_standard = V4L2_STD_SECAM;
		break;
	case AD_PAL_COMB:
		*sd_standard = V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
		break;
	case AD_SECAM_525:
		*sd_standard = V4L2_STD_SECAM;
		break;
	default:
		*sd_standard = V4L2_STD_UNKNOWN;
	}
	return ret;
}

int adv7481_set_cvbs_mode(struct adv7481_state *state)
{
	int ret;

	/* cvbs video settings ntsc etc*/
	ret = adv7481_wr_byte(state->client, 0x00, 0x30);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x0f, 0x00);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x00, 0x00);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x03, 0x42);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x04, 0x07);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x13, 0x00);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x17, 0x41);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x31, 0x12);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x52, 0xcd);
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x0e, 0xff);
	/*Enable TxA CSI 1-lane*/
	ret |= adv7481_wr_byte(state->client, 0x10, 0xa8);
	/*Enable autodetect*/
	ret |= adv7481_wr_byte(state->i2c_sdp, 0x0e, 0x81);

	return ret;
}

int adv7481_set_hdmi_mode(struct adv7481_state *state)
{
	int ret;
	int temp;
	/* Power up HDMI Rx */
	temp = adv7481_rd_byte(state->client, 0x00);
	temp |= IO_CTRL_RX_EN;
	temp &= ~IO_CTRL_RX_PWDN;
	adv7481_wr_byte(state->client, 0x00, temp);

	ret = adv7481_wr_byte(state->i2c_rep, 0x3D, 0x10);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x00, 0x08);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x3D, 0x10);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x3E, 0x69);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x3F, 0x46);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x4E, 0xFE);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x4F, 0x08);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x57, 0xA3);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x58, 0x04);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x85, 0x10);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x83, 0x00);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0xA3, 0x01);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0xBE, 0x00);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0xCB, 0x01);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x0F, 0x00);

	/* HPA Assert and termination*/
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x6C, 0x01);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0xF8, 0x01);
	ret |= adv7481_wr_byte(state->i2c_hdmi, 0x83, 0x00);

	return ret;
}

int adv7481_set_analog_mux(struct adv7481_state *state, int input)
{
	int ain_sel = 0x0;

	switch (input) {
	case ADV7481_IP_CVBS_1:
	case ADV7481_IP_CVBS_1_HDMI_SIM:
		ain_sel = 0x0;
		break;
	case ADV7481_IP_CVBS_2:
	case ADV7481_IP_CVBS_2_HDMI_SIM:
		ain_sel = 0x1;
		break;
	case ADV7481_IP_CVBS_3:
	case ADV7481_IP_CVBS_3_HDMI_SIM:
		ain_sel = 0x2;
		break;
	case ADV7481_IP_CVBS_4:
	case ADV7481_IP_CVBS_4_HDMI_SIM:
		ain_sel = 0x3;
		break;
	case ADV7481_IP_CVBS_5:
	case ADV7481_IP_CVBS_5_HDMI_SIM:
		ain_sel = 0x4;
		break;
	case ADV7481_IP_CVBS_6:
	case ADV7481_IP_CVBS_6_HDMI_SIM:
		ain_sel = 0x5;
		break;
	case ADV7481_IP_CVBS_7:
	case ADV7481_IP_CVBS_7_HDMI_SIM:
		ain_sel = 0x6;
		break;
	case ADV7481_IP_CVBS_8:
	case ADV7481_IP_CVBS_8_HDMI_SIM:
		ain_sel = 0x7;
		break;
	}
	return 0;
}

static int adv7481_set_ip_mode(struct adv7481_state *state, int input)
{
	int ret;
	switch (input) {
	case ADV7481_IP_HDMI:
		adv7481_set_hdmi_mode(state);
		break;
	case ADV7481_IP_CVBS_1:
	case ADV7481_IP_CVBS_2:
	case ADV7481_IP_CVBS_3:
	case ADV7481_IP_CVBS_4:
	case ADV7481_IP_CVBS_5:
	case ADV7481_IP_CVBS_6:
	case ADV7481_IP_CVBS_7:
	case ADV7481_IP_CVBS_8:
		adv7481_set_cvbs_mode(state);
		adv7481_set_analog_mux(state, input);
		break;
	case ADV7481_IP_CVBS_1_HDMI_SIM:
	case ADV7481_IP_CVBS_2_HDMI_SIM:
	case ADV7481_IP_CVBS_3_HDMI_SIM:
	case ADV7481_IP_CVBS_4_HDMI_SIM:
	case ADV7481_IP_CVBS_5_HDMI_SIM:
	case ADV7481_IP_CVBS_6_HDMI_SIM:
	case ADV7481_IP_CVBS_7_HDMI_SIM:
	case ADV7481_IP_CVBS_8_HDMI_SIM:
		adv7481_set_hdmi_mode(state);
		adv7481_set_cvbs_mode(state);
		adv7481_set_analog_mux(state, input);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int adv7482_set_op_src(struct adv7481_state *state,
						int output, int input)
{
	int ret = 0;
	int temp = 0, val = 0;
	switch (output) {
	case ADV7481_OP_CSIA:
		switch (input) {
		case ADV7481_IP_CVBS_1:
		case ADV7481_IP_CVBS_2:
		case ADV7481_IP_CVBS_3:
		case ADV7481_IP_CVBS_4:
		case ADV7481_IP_CVBS_5:
		case ADV7481_IP_CVBS_6:
		case ADV7481_IP_CVBS_7:
		case ADV7481_IP_CVBS_8:
			val = 0x10;
			break;
		case ADV7481_IP_CVBS_1_HDMI_SIM:
		case ADV7481_IP_CVBS_2_HDMI_SIM:
		case ADV7481_IP_CVBS_3_HDMI_SIM:
		case ADV7481_IP_CVBS_4_HDMI_SIM:
		case ADV7481_IP_CVBS_5_HDMI_SIM:
		case ADV7481_IP_CVBS_6_HDMI_SIM:
		case ADV7481_IP_CVBS_7_HDMI_SIM:
		case ADV7481_IP_CVBS_8_HDMI_SIM:
		case ADV7481_IP_HDMI:
			val = 0x00;
			break;
		case ADV7481_IP_TTL:
			val = 0x1;
			break;
		default:
			ret = -EINVAL;
		}
		temp = adv7481_rd_byte(state->client, 0x00);
		temp |= val;
		adv7481_wr_byte(state->client, 0x00, temp);
		break;
	case ADV7481_OP_CSIB:
		if (input != ADV7481_IP_HDMI && input != ADV7481_IP_TTL)
			state->csib_src = input;
		else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int adv7481_s_routing(struct v4l2_subdev *sd, u32 input,
				u32 output, u32 config)
{
	struct adv7481_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	ret = adv7482_set_op_src(state, output, input);
	if (ret) {
		pr_err("SRC Routing Error\n");
		return ret;
	}

	if (state->mode != input) {
		ret = adv7481_set_ip_mode(state, input);
		if (ret)
			state->mode = input;
	}


	mutex_unlock(&state->mutex);
	return ret;
}
static int adv7481_get_hdmi_timings(struct adv7481_state *state,
				struct adv7481_vid_params *vid_params,
				struct adv7481_hdmi_params *hdmi_params)
{
	int ret = 0, temp1 = 0, temp2 = 0, fieldfactor = 0;
	/*Check TMDS PLL Lock and Frequency*/
	hdmi_params->pll_lock = adv7481_rd_byte(state->i2c_hdmi, 0x04);
	hdmi_params->pll_lock = (hdmi_params->pll_lock & 0x2) >> 1;
	if (hdmi_params->pll_lock) {
		temp1 = adv7481_rd_byte(state->i2c_hdmi, 0x51);
		temp2 = adv7481_rd_byte(state->i2c_hdmi, 0x52);
		hdmi_params->tmds_freq &= 0xFF;
		hdmi_params->tmds_freq = (hdmi_params->tmds_freq << 1)
						+ (temp1 & 0x01);
		hdmi_params->tmds_freq = hdmi_params->tmds_freq * 1000000;

	} else {
		return -EBUSY;
	}
	/*Check HDMI Parameters*/
	temp1 = adv7481_rd_byte(state->i2c_hdmi, HDMI_REG_HDMI_PARAM6);
	hdmi_params->color_depth = (temp1 & 0xC0) >> 7;
	temp1 = adv7481_rd_byte(state->i2c_hdmi,  HDMI_REG_HDMI_PARAM5);
	hdmi_params->pix_rep = (temp1 & 0x0F);

	/*Check Interlaced and Field Factor*/
	vid_params->intrlcd = (temp1 & 0x20) >> 5;
	fieldfactor = (vid_params->intrlcd == 1) ? 2 : 1;


	/*Get Total Timing Data HDMI Map  V:0x26[5:0] + 0x27[7:0]*/
	temp1 = adv7481_rd_byte(state->i2c_hdmi, HDMI_REG_FLD0_TOT1);
	vid_params->tot_lines = ((temp1 & 0x3F) << 8);
	temp1 = adv7481_rd_byte(state->i2c_hdmi, HDMI_REG_FLD0_TOT2);
	vid_params->tot_lines = (vid_params->tot_lines & 0x1F00) |
				(temp1 & 0xFF);

	/*Get Active Timing Data HDMI Map  H:0x1E[5:0] + 0x1F[7:0]*/
	temp1 = adv7481_rd_byte(state->i2c_hdmi, HDMI_REG_LINE_TOT1);
	vid_params->tot_pix = ((temp1 & 0x3F) << 8);
	temp1 = adv7481_rd_byte(state->i2c_hdmi, HDMI_REG_LINE_TOT2);
	vid_params->tot_pix = ((vid_params->tot_pix & 0x3F00) | (temp1 & 0xFF));

	switch (hdmi_params->color_depth) {
	case CD_10BIT:
		vid_params->pix_clk =  ((vid_params->pix_clk*4)/5);
		break;
	case CD_12BIT:
		vid_params->pix_clk = ((vid_params->pix_clk*2)/3);
		break;
	case CD_16BIT:
		vid_params->pix_clk = (vid_params->pix_clk/2);
		break;
	case CD_8BIT:
	default:
		vid_params->pix_clk /= 1;
	}

	if ((vid_params->tot_pix != 0) && (vid_params->tot_lines != 0)) {
		vid_params->fr_rate = vid_params->pix_clk * fieldfactor
						/ vid_params->tot_lines;
		vid_params->fr_rate /= vid_params->tot_pix;
		vid_params->fr_rate /= (hdmi_params->pix_rep + 1);
	}

	pr_debug(" %s(%d), adv7481 TMDS Resolution: %d : %d @ %d\n",
			__func__, __LINE__,
			vid_params->act_lines, vid_params->act_pix,
			vid_params->fr_rate);
	return ret;
}

static int adv7481_query_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	int ret;
	struct adv7481_state *state = to_state(sd);
	struct adv7481_vid_params vid_params;
	struct adv7481_hdmi_params hdmi_params;
	struct v4l2_bt_timings *bt_timings = &timings->bt;

	if (!timings)
		return -EINVAL;

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));
	memset(&vid_params, 0, sizeof(struct adv7481_vid_params));
	memset(&hdmi_params, 0, sizeof(struct adv7481_hdmi_params));

	switch (state->mode) {
	case ADV7481_IP_HDMI:
	case ADV7481_IP_CVBS_1_HDMI_SIM:
		adv7481_get_hdmi_timings(state, &vid_params, &hdmi_params);
		timings->type = V4L2_DV_BT_656_1120;
		bt_timings->width = vid_params.act_pix;
		bt_timings->height = vid_params.act_lines;
		bt_timings->pixelclock = vid_params.pix_clk;
		bt_timings->interlaced = vid_params.intrlcd ?
				V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;
		if (bt_timings->interlaced == V4L2_DV_INTERLACED)
			bt_timings->height += bt_timings->height*2;
		break;
	default:
		return -EINVAL;
	}
	mutex_unlock(&state->mutex);
	return ret;
}

static int  adv7481_query_sd_std(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	int ret = 0;
	int temp = 0;
	struct adv7481_state *state = to_state(sd);
	int tStatus = 0x0;

	tStatus = adv7481_rd_byte(state->i2c_sdp, SDP_VDEC_LOCK);
	if (!(tStatus & 0x1))
		pr_err("SIGNAL NOT LOCKED\n");

	if (!std)
		return -EINVAL;

	switch (state->mode) {
	case ADV7481_IP_CVBS_1:
	case ADV7481_IP_CVBS_2:
	case ADV7481_IP_CVBS_3:
	case ADV7481_IP_CVBS_4:
	case ADV7481_IP_CVBS_5:
	case ADV7481_IP_CVBS_6:
	case ADV7481_IP_CVBS_7:
	case ADV7481_IP_CVBS_8:
	case ADV7481_IP_CVBS_1_HDMI_SIM:
	case ADV7481_IP_CVBS_2_HDMI_SIM:
	case ADV7481_IP_CVBS_3_HDMI_SIM:
	case ADV7481_IP_CVBS_4_HDMI_SIM:
	case ADV7481_IP_CVBS_5_HDMI_SIM:
	case ADV7481_IP_CVBS_6_HDMI_SIM:
	case ADV7481_IP_CVBS_7_HDMI_SIM:
	case ADV7481_IP_CVBS_8_HDMI_SIM:
		ret = adv7481_get_sd_timings(state, &temp);
		break;
	default:
		return -EINVAL;
	}

	if (!tStatus)
		*std = (v4l2_std_id) temp;
	else
		*std = V4L2_STD_UNKNOWN;

	return ret;
}

static int adv7481_csi_powerup(struct adv7481_state *state, bool pwr, int tx)
{
	int ret;
	struct i2c_client *csi_map;

	/*Select CSI TX to configure data*/
	if (tx != ADV7481_OP_CSIA || tx != ADV7481_OP_CSIB)
		/* for future if need to enable txa v txb */;
	else {
		csi_map = (tx == ADV7481_OP_CSIA) ?
				state->i2c_csi_txa : state->i2c_csi_txb;
	}

	/*TXA 1 lane settings for CSI*/
	ret = adv7481_wr_byte(state->i2c_csi_txa, 0x00, 0x81);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x00, 0xa1);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xd6, 0x07);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xc4, 0x0a);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xca, 0x02);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x71, 0x33);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x72, 0x11);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xf0, 0x00);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x31, 0x82);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x1e, 0x40);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xda, 0x01);
	/*adi Recommended power up sequence*/
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xda, 0x01);
	msleep(200);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x00, 0x21);
	msleep(100);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0xc1, 0x2b);
	msleep(100);
	ret |= adv7481_wr_byte(state->i2c_csi_txa, 0x31, 0x80);

	return ret;

}


static int adv7481_set_op_stream(struct adv7481_state *state, bool on)
{
	int ret;

	if (on && state->csia_src != ADV7481_IP_NONE)
		ret = adv7481_csi_powerup(state, on, ADV7481_OP_CSIA);
	else if (on && state->csib_src != ADV7481_IP_NONE)
		ret = adv7481_csi_powerup(state, on, ADV7481_OP_CSIB);
	else
		ret = adv7481_csi_powerup(state, on, ADV7481_OP_CSIA);

	return ret;
}

static int adv7481_s_stream(struct v4l2_subdev *sd, int on)
{
	struct adv7481_state *state = to_state(sd);
	int ret;

	ret = adv7481_set_op_stream(state, on);
	return ret;
}

static const struct v4l2_subdev_video_ops adv7481_video_ops = {
	.s_routing = adv7481_s_routing,
	.querystd = adv7481_query_sd_std,
	.g_dv_timings = adv7481_query_dv_timings,
	.s_stream = adv7481_s_stream,
};

static const struct v4l2_subdev_core_ops adv7481_core_ops = {
	.s_power = adv7481_s_power,
};

static const struct v4l2_ctrl_ops adv7481_ctrl_ops = {
	.s_ctrl = adv7481_s_ctrl,
};

static const struct v4l2_subdev_ops adv7481_ops = {
	.core = &adv7481_core_ops,
	.video = &adv7481_video_ops,
};

static int adv7481_init_v4l2_controls(struct adv7481_state *state)
{
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 4);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7481_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7481_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7481_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7481_ctrl_ops,
			  V4L2_CID_HUE, -127, 128, 1, 0);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	return 0;
}

static int adv7481_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv7481_state *state;
	struct adv7481_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *hdl;
	int ret;

	pr_debug("Attempting to probe...\n");
	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("Check i2c Functionality Fail\n");
		ret = -EIO;
		goto err;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			 client->addr, client->adapter->name);

	/*Create 7481 State  */
	state = devm_kzalloc(&client->dev,
				sizeof(struct adv7481_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		pr_err("Check Kzalloc Fail\n");
		goto err_mem;
	}
	state->client = client;
	mutex_init(&state->mutex);

	/* Get and Check Platform Data */
	pdata = (struct adv7481_platform_data *) client->dev.platform_data;
	if (!pdata) {
		ret = -ENOMEM;
		pr_err("Getting Platform data failed\n");
		goto err_mem;
	}

	/* Configure and Register V4L2 I2C Sub-device  */
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv7481_ops);
	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	/*Register as Media Entity*/
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	state->sd.entity.flags |= MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(&state->sd.entity, 1, &state->pad, 0);
	if (ret) {
		ret = -EIO;
		pr_err("Media entity init failed\n");
		goto err_media_entity;
	}

	/*Initialize HW Config */
	ret |= adv7481_hw_init(pdata, state);
	if (ret) {
		ret = -EIO;
		pr_err("HW Initialisation Failed\n");
		goto err_media_entity;
	}

	/*Register V4l2 Control Functions*/
	hdl = &state->ctrl_hdl;
	v4l2_ctrl_handler_init(hdl, 4);
	adv7481_init_v4l2_controls(state);

	/*Initials ADV7481 State Settings*/
	state->txa_auto_params = ADV7481_AUTO_PARAMS;
	state->txa_lanes = ADV7481_CSI_1LANE;

	/*Initialize SW Init Settings and I2C sub maps 7481*/
	ret |= adv7481_dev_init(state, client);
	if (ret) {
		ret = -EIO;
		pr_err("SW Initialisation Failed\n");
		goto err_media_entity;
	}

	/*Set cvbs settings*/
	ret |= adv7481_set_cvbs_mode(state);

	/*Av mgr registration*/
	ret |= av_mgr_sd_register(sd);
	if (ret) {
		ret = -EIO;
		pr_err("AV MGR INIT FAILED\n");
		goto err_media_entity;
	}
	pr_debug("Probe successful!\n");

	return ret;

err_media_entity:
	media_entity_cleanup(&sd->entity);
err_mem:
	kfree(state);
err:
	if (!ret)
		ret = 1;
	return ret;
}

static int adv7481_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7481_state *state = to_state(sd);

	av_mgr_sd_unregister(sd);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);

	i2c_unregister_device(state->i2c_csi_txa);
	i2c_unregister_device(state->i2c_csi_txb);
	i2c_unregister_device(state->i2c_hdmi);
	i2c_unregister_device(state->i2c_cp);
	i2c_unregister_device(state->i2c_sdp);
	i2c_unregister_device(state->i2c_rep);
	mutex_destroy(&state->mutex);
	kfree(state);

	return 0;
}

static const struct i2c_device_id adv7481_id[] = {
	{ DRIVER_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7481_id);


static struct i2c_driver adv7481_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = KBUILD_MODNAME,
	},
	.probe = adv7481_probe,
	.remove = adv7481_remove,
	.id_table = adv7481_id,
};

module_i2c_driver(adv7481_driver);

MODULE_DESCRIPTION("ADI ADV7481 HDMI/MHL/SD video receiver");
