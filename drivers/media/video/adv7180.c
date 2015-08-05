/*
 * adv7180.c Analog Devices ADV7180 video decoder driver
 * Copyright (c) 2009 Intel Corporation
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/adv7180.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/media.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/msm_ba.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define DRIVER_NAME "adv7180"

#define FRAME_START_DELAY 33333
#define I2C_RW_DELAY 1
#define I2C_RETRY_DELAY 17000
#define ADV7180_STD_AD_PAL_BG_NTSC_J_SECAM		0x0
#define ADV7180_STD_AD_PAL_BG_NTSC_J_SECAM_PED		0x1
#define ADV7180_STD_AD_PAL_N_NTSC_J_SECAM		0x2
#define ADV7180_STD_AD_PAL_N_NTSC_M_SECAM		0x3
#define ADV7180_STD_NTSC_J				0x4
#define ADV7180_STD_NTSC_M				0x5
#define ADV7180_STD_PAL60				0x6
#define ADV7180_STD_NTSC_443				0x7
#define ADV7180_STD_PAL_BG				0x8
#define ADV7180_STD_PAL_N				0x9
#define ADV7180_STD_PAL_M				0xa
#define ADV7180_STD_PAL_M_PED				0xb
#define ADV7180_STD_PAL_COMB_N				0xc
#define ADV7180_STD_PAL_COMB_N_PED			0xd
#define ADV7180_STD_PAL_SECAM				0xe
#define ADV7180_STD_PAL_SECAM_PED			0xf

#define ADV7180_REG_INPUT_CONTROL			0x0000
#define ADV7180_INPUT_CONTROL_INSEL_MASK		0x0f

#define ADV7182_REG_INPUT_VIDSEL			0x0002

#define ADV7180_REG_EXTENDED_OUTPUT_CONTROL		0x0004
#define ADV7180_EXTENDED_OUTPUT_CONTROL_NTSCDIS		0xC5

#define ADV7180_REG_AUTODETECT_ENABLE			0x07
#define ADV7180_AUTODETECT_DEFAULT			0x7f
/* Contrast */
#define ADV7180_REG_CON		0x0008	/* Unsigned */
#define ADV7180_CON_MIN		0
#define ADV7180_CON_DEF		128
#define ADV7180_CON_MAX		255
/* Brightness */
#define ADV7180_REG_BRI		0x000a	/* Signed */
#define ADV7180_BRI_MIN		-128
#define ADV7180_BRI_DEF		0
#define ADV7180_BRI_MAX		127
/* Hue */
#define ADV7180_REG_HUE		0x000b	/* Signed, inverted */
#define ADV7180_HUE_MIN		-127
#define ADV7180_HUE_DEF		0
#define ADV7180_HUE_MAX		128

#define ADV7180_REG_DEF_VAL_Y		0x000c
#define ADV7180_REG_DEF_VAL_C		0x000d

#define ADV7180_REG_CTRL		0x000e
#define ADV7180_CTRL_IRQ_SPACE		0x20

#define ADV7180_REG_PWR_MAN		0x0f
#define ADV7180_PWR_MAN_ON		0x04
#define ADV7180_PWR_MAN_OFF		0x24
#define ADV7180_PWR_MAN_RES		0x80

#define ADV7180_REG_STATUS1		0x0010
#define ADV7180_STATUS1_IN_LOCK		0x01
#define ADV7180_STATUS1_LOST_LOCK	0x02
#define ADV7180_STATUS1_FSC_LOCK	0x04
#define ADV7180_STATUS1_AUTOD_MASK	0x70
#define ADV7180_STATUS1_AUTOD_NTSM_M_J	0x00
#define ADV7180_STATUS1_AUTOD_NTSC_4_43 0x10
#define ADV7180_STATUS1_AUTOD_PAL_M	0x20
#define ADV7180_STATUS1_AUTOD_PAL_60	0x30
#define ADV7180_STATUS1_AUTOD_PAL_B_G	0x40
#define ADV7180_STATUS1_AUTOD_SECAM	0x50
#define ADV7180_STATUS1_AUTOD_PAL_COMB	0x60
#define ADV7180_STATUS1_AUTOD_SECAM_525	0x70

#define ADV7180_STATUS1_LOCKED	\
	(ADV7180_STATUS1_IN_LOCK|ADV7180_STATUS1_FSC_LOCK)

#define ADV7180_REG_STATUS3		0x13

#define ADV7180_REG_IDENT 0x0011
#define ADV7180_ID_7180 0x18

#define ADV7180_REG_ICONF1		0x2040

#define ADV7180_ICONF1_ACTIVE_LOW	0x01
#define ADV7180_ICONF1_PSYNC_ONLY	0x10
#define ADV7180_ICONF1_ACTIVE_TO_CLR	0xC0

/* Saturation */
#define ADV7180_REG_SD_SAT_CB	0x00e3	/* Unsigned */
#define ADV7180_REG_SD_SAT_CR	0x00e4	/* Unsigned */
#define ADV7180_SAT_MIN		0
#define ADV7180_SAT_DEF		128
#define ADV7180_SAT_MAX		255

#define ADV7180_IRQ1_LOCK	0x01
#define ADV7180_IRQ1_UNLOCK	0x02
#define ADV7180_IRQ1_MACROVISION 0x40

#define ADV7180_IRQ3_AD_CHANGE	0x08

#define ADV7180_REG_ISR1	0x2042
#define ADV7180_REG_ICR1	0x2043
#define ADV7180_REG_IMR1	0x2044

#define ADV7180_REG_ISR2	0x2046
#define ADV7180_REG_ICR2	0x2047
#define ADV7180_REG_IMR2	0x2048

#define ADV7180_REG_ISR3	0x204A
#define ADV7180_REG_ICR3	0x204B
#define ADV7180_REG_IMR3	0x204C

#define ADV7180_REG_ISR4	0x204E
#define ADV7180_REG_ICR4	0x204F
#define ADV7180_REG_IMR4	0x2050

#define ADV7180_REG_NTSC_V_BIT_END	0x00E6
#define ADV7180_NTSC_V_BIT_END_MANUAL_NVEND	0x4F

#define ADV7180_REG_VPP_SLAVE_ADDR	0xFD
#define ADV7180_REG_CSI_SLAVE_ADDR	0xFE

#define ADV7180_REG_FLCONTROL 0x40e0
#define ADV7180_FLCONTROL_FL_ENABLE 0x1

#define ADV7180_CSI_REG_PWRDN	0x00
#define ADV7180_CSI_PWRDN	0x80

#define ADV7180_INPUT_CVBS_AIN1 0x00
#define ADV7180_INPUT_CVBS_AIN2 0x01
#define ADV7180_INPUT_CVBS_AIN3 0x02
#define ADV7180_INPUT_CVBS_AIN4 0x03
#define ADV7180_INPUT_CVBS_AIN5 0x04
#define ADV7180_INPUT_CVBS_AIN6 0x05
#define ADV7180_INPUT_SVIDEO_AIN1_AIN2 0x06
#define ADV7180_INPUT_SVIDEO_AIN3_AIN4 0x07
#define ADV7180_INPUT_SVIDEO_AIN5_AIN6 0x08
#define ADV7180_INPUT_YPRPB_AIN1_AIN2_AIN3 0x09
#define ADV7180_INPUT_YPRPB_AIN4_AIN5_AIN6 0x0a

#define ADV7182_INPUT_CVBS_AIN1 0x00
#define ADV7182_INPUT_CVBS_AIN2 0x01
#define ADV7182_INPUT_CVBS_AIN3 0x02
#define ADV7182_INPUT_CVBS_AIN4 0x03
#define ADV7182_INPUT_CVBS_AIN5 0x04
#define ADV7182_INPUT_CVBS_AIN6 0x05
#define ADV7182_INPUT_CVBS_AIN7 0x06
#define ADV7182_INPUT_CVBS_AIN8 0x07
#define ADV7182_INPUT_SVIDEO_AIN1_AIN2 0x08
#define ADV7182_INPUT_SVIDEO_AIN3_AIN4 0x09
#define ADV7182_INPUT_SVIDEO_AIN5_AIN6 0x0a
#define ADV7182_INPUT_SVIDEO_AIN7_AIN8 0x0b
#define ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3 0x0c
#define ADV7182_INPUT_YPRPB_AIN4_AIN5_AIN6 0x0d
#define ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2 0x0e
#define ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4 0x0f
#define ADV7182_INPUT_DIFF_CVBS_AIN5_AIN6 0x10
#define ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8 0x11
#define ADV7182_INPUT_UNINITIALIZED 0xff

#define ADV7180_DEFAULT_CSI_I2C_ADDR 0x44
#define ADV7180_DEFAULT_VPP_I2C_ADDR 0x42

#define ADV7180_DEVICE2_CSI_I2C_ADDR 0x45
#define ADV7180_DEVICE2_VPP_I2C_ADDR 0x43

#define ADV7180_FLAG_V2			BIT(0)
#define ADV7180_FLAG_MIPI_CSI2		BIT(1)
#define ADV7180_FLAG_I2P		BIT(2)

#define ADV7180_INPUT_DISABLED (~0x00)

struct adv7180_state;

struct adv7180_chip_info {
	unsigned int valid_input_mask;
	int (*set_std)(struct adv7180_state *st, unsigned int std);
	int (*select_input)(struct adv7180_state *st, unsigned int input);
	int (*init)(struct adv7180_state *state);
	unsigned int flags;
};

struct adv7180_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct media_pad	pad;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	int			irq;
	v4l2_std_id		curr_norm;
	bool			autodetect;
	bool			powered;
	bool			force_free_run;
	u8			input;

	struct i2c_client	*client;
	unsigned int		register_page;
	struct i2c_client	*csi_client;
	struct i2c_client	*vpp_client;
	const struct adv7180_chip_info *chip_info;
	enum v4l2_field		field;
	/*
	 * Local variable used to keep track
	 * of csi programming.  Will be set to
	 * true the first time streaming is started.
	 * Will be set false when adv is powered off.
	 */
	int csi_configured;
	/* Keep track of current video standard */
	v4l2_std_id curr_mode;
	/* Keep track of current ain selected */
	int curr_input;
	int device_num;

	/* worker to handle interrupts */
	struct delayed_work irq_delayed_work;
};

static int adv7180_set_video_standard(struct adv7180_state *state,
	unsigned int std)
{
	return state->chip_info->set_std(state, std);
}

#define to_adv7180_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv7180_state,	\
					    ctrl_hdl)->sd)

static int adv7180_select_page(struct adv7180_state *state, unsigned int page)
{
	int rd_bk = 0x0;
	int ret = 0;
	int num_tries = 0;

	if (state->register_page != page) {
		ret = i2c_smbus_write_byte_data(state->client, ADV7180_REG_CTRL,
			page);
		while (ret < 0 && num_tries < 3) {
			pr_err("%s : write page failed num_tries %d ret %d\n",
				 __func__, num_tries, ret);
			ret = i2c_smbus_write_byte_data(state->client,
							ADV7180_REG_CTRL, page);
			num_tries++;
		}

		pr_debug("%s : write num_tries %d ret %d\n",
			__func__, num_tries, ret);

		rd_bk = i2c_smbus_read_byte_data(state->client,
						ADV7180_REG_CTRL);
		num_tries = 0;
		while (rd_bk < 0 && num_tries < 3) {
			pr_err("%s : read failed num_tries %d ret %d\n",
				__func__, num_tries, ret);
			num_tries++;
		}

		state->register_page = page;

		pr_debug("%s : rd_bk 0x%x\n", __func__, rd_bk);
	}
	return 0;
}

static int adv7180_write(struct adv7180_state *state, unsigned int reg,
	unsigned int value)
{
	int ret = -1;
	int num_tries = 0;
	pr_debug("%s : reg 0x%x state->client->addr 0x%x value 0x%x\n",
		__func__, reg, state->client->addr, value);

	lockdep_assert_held(&state->mutex);

	adv7180_select_page(state, reg >> 8);
	ret = i2c_smbus_write_byte_data(state->client,
					reg & 0xff, value);

	/* Retry the write if it failed */
	while (ret < 0 && num_tries < 3) {
		ret = i2c_smbus_write_byte_data(state->client,
		reg & 0xff, value);
		num_tries++;
		pr_err("%s : Retry addr 0x%x reg 0x%x value 0x%x ret 0x%x num_tries %d\n",
			__func__, state->client->addr,
			reg, value, ret, num_tries);
	}

	pr_debug("%s : write ret 0x%x num_tries %d\n",
			__func__, ret, num_tries);
	return ret;
}

static int adv7180_read(struct adv7180_state *state, unsigned int reg)
{
	int ret = 0;
	int num_tries = 0;
	lockdep_assert_held(&state->mutex);
	pr_debug("%s : reg 0x%x state->client->addr 0x%x\n",
		__func__, reg, state->client->addr);

	adv7180_select_page(state, reg >> 8);
	ret = i2c_smbus_read_byte_data(state->client, reg & 0xff);

	/* Retry the read if it failed */
	while (ret < 0 && num_tries < 10) {
		ret = i2c_smbus_read_byte_data(state->client,
			reg & 0xff);
		num_tries++;
		pr_err("%s : Retry addr 0x%x reg 0x%x ret 0x%x num_tries %d\n",
			__func__, state->client->addr,
			reg, ret, num_tries);
	}

	pr_debug("%s : read ret 0x%x num_tries %d\n", __func__, ret, num_tries);
	return ret;
}

static int adv7180_csi_write(struct adv7180_state *state, unsigned int reg,
	unsigned int value)
{
	int ret = -1;
	int num_tries = 0;
	pr_debug("%s : reg 0x%x state->client->addr 0x%x\n",
		__func__, reg, state->csi_client->addr);

	ret = i2c_smbus_write_byte_data(state->csi_client, reg, value);

	/* Retry the write if it failed */
	while (ret < 0 && num_tries < 3) {
		ret = i2c_smbus_write_byte_data(state->csi_client,
						reg, value);
		num_tries++;
		pr_err("%s : Retry addr 0x%x reg 0x%x val 0x%x ret 0x%x num_tries %d\n",
			__func__, state->csi_client->addr,
			reg, value, ret, num_tries);
	}

	pr_debug("%s :csi write ret 0x%x num_tries %d\n",
		__func__, ret, num_tries);
	return ret;
}

static int adv7180_vpp_write(struct adv7180_state *state, unsigned int reg,
	unsigned int value)
{
	int ret = -1;
	int num_tries = 0;
	pr_debug("%s : reg 0x%x state->client->addr 0x%x\n",
		__func__, reg, state->vpp_client->addr);

	ret = i2c_smbus_write_byte_data(state->vpp_client, reg, value);

	/* Retry the write if it failed */
	while (ret < 0 && num_tries < 3) {
		ret = i2c_smbus_write_byte_data(state->vpp_client,
						reg, value);
		num_tries++;
		pr_err("%s : Retry addr 0x%x reg 0x%x value 0x%x ret 0x%x num_tries %d\n",
			__func__, state->vpp_client->addr,
			reg, value, ret, num_tries);
	}

	pr_debug("%s :ret 0x%x num_tries %d\n", __func__, ret, num_tries);
	return ret;
}

static v4l2_std_id adv7180_std_to_v4l2(u8 status1)
{
	switch (status1 & ADV7180_STATUS1_AUTOD_MASK) {
	case ADV7180_STATUS1_AUTOD_NTSM_M_J:
		return V4L2_STD_NTSC;
	case ADV7180_STATUS1_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case ADV7180_STATUS1_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case ADV7180_STATUS1_AUTOD_PAL_60:
		return V4L2_STD_PAL_60;
	case ADV7180_STATUS1_AUTOD_PAL_B_G:
		return V4L2_STD_PAL;
	case ADV7180_STATUS1_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	case ADV7180_STATUS1_AUTOD_PAL_COMB:
		return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
	case ADV7180_STATUS1_AUTOD_SECAM_525:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static int v4l2_std_to_adv7180(v4l2_std_id std)
{
	if (std == V4L2_STD_PAL_60)
		return ADV7180_STD_PAL60;
	if (std == V4L2_STD_NTSC_443)
		return ADV7180_STD_NTSC_443;
	if (std == V4L2_STD_NTSC)
		return ADV7180_STD_NTSC_M;
	if (std == V4L2_STD_PAL_N)
		return ADV7180_STD_PAL_N;
	if (std == V4L2_STD_PAL_M)
		return ADV7180_STD_PAL_M;
	if (std == V4L2_STD_PAL_Nc)
		return ADV7180_STD_PAL_COMB_N;

	if (std & V4L2_STD_PAL)
		return ADV7180_STD_PAL_BG;
	if (std & V4L2_STD_NTSC)
		return ADV7180_STD_NTSC_M;
	if (std & V4L2_STD_SECAM)
		return ADV7180_STD_PAL_SECAM;

	return -EINVAL;
}

static u32 adv7180_status_to_v4l2(u8 status1)
{
	if (!(status1 & ADV7180_STATUS1_IN_LOCK))
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static int __adv7180_status(struct adv7180_state *state, u32 *status,
			    v4l2_std_id *std)
{
	int status1 = adv7180_read(state, ADV7180_REG_STATUS1);
	pr_debug("%s: status1 0x%x\n", __func__, status1);

	if (status1 < 0)
		return status1;

	if (status)
		*status = adv7180_status_to_v4l2(status1);
	if (std)
		*std = adv7180_std_to_v4l2(status1);

	return 0;
}

static inline struct adv7180_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7180_state, sd);
}

static int adv7180_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct adv7180_state *state = to_state(sd);
	int err  = 0;
	v4l2_std_id new_std = V4L2_STD_UNKNOWN;
	pr_debug("%s: entry!\n", __func__);

	err = mutex_lock_interruptible(&state->mutex);

	if (err)
		return err;

	/* when we are interrupt driven we know the state */
	if (state->irq > 0) {
		new_std = state->curr_norm;
	} else {
		usleep(10000);
		err = __adv7180_status(state, NULL, &new_std);
	}

	*std = new_std;

	mutex_unlock(&state->mutex);

	pr_debug("%s: exit!\n", __func__);
	return err;
}

static u32 ba_inp_to_adv7180(u32 input)
{
	u32 adv_input = ADV7180_INPUT_CVBS_AIN1;

	switch (input) {
	case BA_IP_CVBS_0:
		adv_input = ADV7180_INPUT_CVBS_AIN1;
		break;
	case BA_IP_CVBS_1:
		adv_input = ADV7180_INPUT_CVBS_AIN2;
		break;
	case BA_IP_CVBS_2:
		adv_input = ADV7180_INPUT_CVBS_AIN3;
		break;
	case BA_IP_CVBS_3:
		adv_input = ADV7180_INPUT_CVBS_AIN4;
		break;
	case BA_IP_CVBS_4:
		adv_input = ADV7180_INPUT_CVBS_AIN5;
		break;
	case BA_IP_CVBS_5:
		adv_input = ADV7180_INPUT_CVBS_AIN6;
		break;
	case BA_IP_SVIDEO_0:
		adv_input = ADV7180_INPUT_SVIDEO_AIN1_AIN2;
		break;
	case BA_IP_SVIDEO_1:
		adv_input = ADV7180_INPUT_SVIDEO_AIN3_AIN4;
		break;
	case BA_IP_SVIDEO_2:
		adv_input = ADV7180_INPUT_SVIDEO_AIN5_AIN6;
		break;
	case BA_IP_COMPONENT_0:
		adv_input = ADV7180_INPUT_YPRPB_AIN1_AIN2_AIN3;
		break;
	case BA_IP_COMPONENT_1:
		adv_input = ADV7180_INPUT_YPRPB_AIN4_AIN5_AIN6;
		break;
	case BA_IP_DVI_0:
		adv_input = ADV7180_INPUT_YPRPB_AIN1_AIN2_AIN3;
		break;
	case BA_IP_DVI_1:
		adv_input = ADV7180_INPUT_YPRPB_AIN4_AIN5_AIN6;
		break;
	default:
		adv_input = ADV7180_INPUT_CVBS_AIN1;
		break;
	}
	return adv_input;
}

static u32 adv7180_inp_to_ba(u32 input)
{
	u32 ba_input = BA_IP_CVBS_0;

	switch (input) {
	case ADV7180_INPUT_CVBS_AIN1:
		ba_input = BA_IP_CVBS_0;
		break;
	case ADV7180_INPUT_CVBS_AIN2:
		ba_input = BA_IP_CVBS_1;
		break;
	case ADV7180_INPUT_CVBS_AIN3:
		ba_input = BA_IP_CVBS_2;
		break;
	case ADV7180_INPUT_CVBS_AIN4:
		ba_input = BA_IP_CVBS_3;
		break;
	case ADV7180_INPUT_CVBS_AIN5:
		ba_input = BA_IP_CVBS_4;
		break;
	case ADV7180_INPUT_CVBS_AIN6:
		ba_input = BA_IP_CVBS_5;
		break;
	case ADV7180_INPUT_SVIDEO_AIN1_AIN2:
		ba_input = BA_IP_SVIDEO_0;
		break;
	case ADV7180_INPUT_SVIDEO_AIN3_AIN4:
		ba_input = BA_IP_SVIDEO_1;
		break;
	case ADV7180_INPUT_SVIDEO_AIN5_AIN6:
		ba_input = BA_IP_SVIDEO_2;
		break;
	case ADV7180_INPUT_YPRPB_AIN1_AIN2_AIN3:
		ba_input = BA_IP_COMPONENT_0;
		break;
	case ADV7180_INPUT_YPRPB_AIN4_AIN5_AIN6:
		ba_input = BA_IP_COMPONENT_1;
		break;
	default:
		ba_input = BA_IP_CVBS_0;
		break;
	}
	return ba_input;
}

static int adv7180_s_routing(struct v4l2_subdev *sd, u32 input,
				u32 output, u32 config)
{
	struct adv7180_state *state = to_state(sd);
	u32 adv_input = ba_inp_to_adv7180(input);
	struct v4l2_event event = {0};
	int *ptr = (int *)event.u.data;
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	if (adv_input > 31 ||
		!(BIT(adv_input) & state->chip_info->valid_input_mask)) {
		ret = -EINVAL;
		goto out;
	}

	ptr[0] = adv7180_inp_to_ba(state->input);

	if (state->force_free_run) {
		state->input = adv_input;
	} else {
		ret = state->chip_info->select_input(state, adv_input);
		if (ret == 0)
			state->input = adv_input;
	}

	/* On successful input switch and not same input
	 * send signal lost on previous input */
	if (ret == 0 &&
		ptr[0] != input) {
		event.type =  V4L2_EVENT_MSM_BA_SIGNAL_LOST_LOCK;
		v4l2_subdev_notify(&state->sd,
			event.type, &event);
	}
out:
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv7180_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct adv7180_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = __adv7180_status(state, status, NULL);
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv7180_program_std(struct adv7180_state *state)
{
	int ret;

	if (state->autodetect) {
		ret = adv7180_set_video_standard(state,
			ADV7180_STD_AD_PAL_BG_NTSC_J_SECAM);
		if (ret < 0)
			return ret;

		__adv7180_status(state, NULL, &state->curr_norm);

		state->curr_mode = state->curr_norm;
		pr_debug("%s: autodetect %d!!!\n",
			__func__,
			(int)state->curr_norm);
	} else {
		if (state->curr_mode == state->curr_norm) {
			pr_debug("%s: mode is the same!!!\n", __func__);
		} else {
			ret = v4l2_std_to_adv7180(state->curr_norm);
			if (ret < 0)
				return ret;

			ret = adv7180_set_video_standard(state, ret);
			if (ret < 0)
				return ret;

			state->curr_mode = state->curr_norm;
			pr_debug("%s: set mode %d!!!\n",
				__func__,
				(int)state->curr_norm);
		}
	}

	return 0;
}

static int adv7180_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct adv7180_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	/* all standards -> autodetect */
	if (std == V4L2_STD_ALL) {
		state->autodetect = true;
	} else {
		/* Make sure we can support this std */
		ret = v4l2_std_to_adv7180(std);
		if (ret < 0)
			goto out;

		state->curr_norm = std;
		state->autodetect = false;
	}

	ret = adv7180_program_std(state);
out:
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv7180_set_power(struct adv7180_state *state, bool on)
{
	int val;
	int ret;
	pr_debug("%s: entry!!!\n", __func__);

	if (on)
		val = ADV7180_PWR_MAN_ON;
	else
		val = ADV7180_PWR_MAN_OFF;

	ret = adv7180_write(state, ADV7180_REG_PWR_MAN, val);
	if (ret)
		return ret;

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
		if (on) {
			if (!state->csi_configured) {
				pr_debug("%s: config csi and vpp!!!\n",
					__func__);
				adv7180_vpp_write(state, 0xa3, 0x00);
				adv7180_vpp_write(state, 0x5b, 0x00);
				adv7180_vpp_write(state, 0x55, 0x80);

				adv7180_csi_write(state, 0x01, 0x20);
				adv7180_csi_write(state, 0x02, 0x28);
				adv7180_csi_write(state, 0x03, 0x38);
				adv7180_csi_write(state, 0x04, 0x30);
				adv7180_csi_write(state, 0x05, 0x30);
				adv7180_csi_write(state, 0x06, 0x80);
				adv7180_csi_write(state, 0x07, 0x70);
				adv7180_csi_write(state, 0x08, 0x50);

				adv7180_csi_write(state, 0xDE, 0x02);
				usleep(I2C_RW_DELAY);
				adv7180_csi_write(state, 0xD2, 0xF7);
				adv7180_csi_write(state, 0xD8, 0x65);
				adv7180_csi_write(state, 0xE0, 0x09);
				adv7180_csi_write(state, 0x2C, 0x00);
				adv7180_csi_write(state, 0x1D, 0x80);
				state->csi_configured = 1;
			} else {
				pr_debug("%s: csi/vpp already configured!!!\n",
					__func__);
			}
		} else {
			/* Reset local variables */
			state->csi_configured = 0;
			state->curr_mode = V4L2_STD_ALL;
			state->curr_input = ADV7182_INPUT_UNINITIALIZED;
		}
	}

	pr_debug("%s: exit!!!!\n", __func__);
	return 0;
}

static int adv7180_s_power(struct v4l2_subdev *sd, int on)
{
	struct adv7180_state *state = to_state(sd);
	int ret;

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = adv7180_set_power(state, on);
	if (ret == 0)
		state->powered = on;

	mutex_unlock(&state->mutex);
	return ret;
}

static bool adv7180_wait_for_lock(struct adv7180_state *state)
{
	int num_tries = 0;
	int vid_std = 0;
	int lock_status = 0;
	bool signal_std_locked = false;
	bool ret;
	int status1 = 0;
	int status3 = 0;

	do {
		num_tries++;

		status1 = adv7180_read(state, ADV7180_REG_STATUS1);

		vid_std = adv7180_std_to_v4l2(status1);
		lock_status = status1 & 0xf;

		if (state->autodetect) {
			/* Check for stable signal standard */
			pr_debug("%s: vid_std 0x%x lock_status 0x%x\n",
				__func__,
				vid_std,
				lock_status);
			signal_std_locked = true;
		} else {
			/* Check if std matches the expected value */
			if (state->curr_norm == vid_std) {
				pr_debug("%s: match vid_std 0x%x state->curr_norm 0x%x lock_status 0x%x\n",
							__func__, vid_std,
							(int)state->curr_norm,
							lock_status);
				signal_std_locked = true;
			} else {
				pr_err("%s: no match vid_std 0x%x state->curr_norm 0x%x lock_status 0x%x\n",
							__func__, vid_std,
							(int)state->curr_norm,
							lock_status);
			}
		}
		pr_debug("%s: status1 0x%x\n", __func__, status1);
		pr_debug("%s: vid_std 0x%x lock_status 0x%x\n",
			__func__,
			vid_std,
			lock_status);

		status3 = adv7180_read(state, ADV7180_REG_STATUS3);
		pr_debug("%s: status3 after 0x%x\n", __func__, status3);

		if (lock_status != ADV7180_STATUS1_LOCKED ||
			signal_std_locked == false) {
			pr_err("%s: status1 0x%x\n",
				__func__, status1);
			usleep(10000);
		}
	} while (((lock_status != ADV7180_STATUS1_LOCKED) ||
		(signal_std_locked == false)) && num_tries < 20);

	if ((lock_status == ADV7180_STATUS1_LOCKED) &&
		(signal_std_locked == true)) {
		ret = true;
	} else {
		ret = false;
		pr_err("%s: adv not locked status1 0x%x\n", __func__, status1);
	}
	return ret;
}

static int adv7180_set_op_stream(struct adv7180_state *state, bool on)
{
	int ret = -1;
	bool lock_status = false;
	pr_debug("%s: entry!!!\n", __func__);

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
		if (on) {
			/*
			 * Ensure signal lock before starting the
			 * csi transmitter
			 */
			lock_status = adv7180_wait_for_lock(state);

			if (lock_status == true) {
				usleep(FRAME_START_DELAY);
				/* Enable the csi */
				ret = adv7180_csi_write(state,
						0x00, 0x00);
			} else {
				pr_err("%s: adv sd failed to lock lock_status %d\n",
						__func__, lock_status);
			}
		} else {
			/* Stop csi transmitter */
			ret = adv7180_csi_write(state, 0x00, 0x80);
		}
	}
	pr_debug("%s: exit!!!\n", __func__);
	return ret;
}

static int adv7180_s_stream(struct v4l2_subdev *sd, int on)
{
	struct adv7180_state *state = to_state(sd);
	int ret;
	pr_debug("%s: entry!!!\n", __func__);
	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = adv7180_set_op_stream(state, on);

	mutex_unlock(&state->mutex);

	pr_debug("%s: exit!!!\n", __func__);
	return ret;
}

static int adv7180_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv7180_sd(ctrl);
	struct adv7180_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	int val;
	pr_debug("%s: entry!!!\n", __func__);

	if (ret)
		return ret;
	val = ctrl->val;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = adv7180_write(state, ADV7180_REG_BRI, val);
		break;
	case V4L2_CID_HUE:
		/* Hue is inverted according to HSL chart */
		ret = adv7180_write(state, ADV7180_REG_HUE, -val);
		break;
	case V4L2_CID_CONTRAST:
		ret = adv7180_write(state, ADV7180_REG_CON, val);
		break;
	case V4L2_CID_SATURATION:
		/*
		 *This could be V4L2_CID_BLUE_BALANCE/V4L2_CID_RED_BALANCE
		 *Let's not confuse the user, everybody understands saturation
		 */
		ret = adv7180_write(state, ADV7180_REG_SD_SAT_CB, val);
		if (ret < 0)
			break;
		ret = adv7180_write(state, ADV7180_REG_SD_SAT_CR, val);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&state->mutex);

	pr_debug("%s: exit!!!\n", __func__);
	return ret;
}

static const struct v4l2_ctrl_ops adv7180_ctrl_ops = {
	.s_ctrl = adv7180_s_ctrl,
};

static const char * const adv7180_free_run_pattern_strings[] = {
	"Solid",
	"Bars",
	"Luma Ramp",
	"Reserved",
	"Reserved",
	"Boundary Box",
};

static const char * const adv7180_free_run_mode_strings[] = {
	"Disabled",
	"Enabled",
	"Automatic",
};



static int adv7180_init_controls(struct adv7180_state *state)
{
	pr_debug("%s: entry\n", __func__);
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 4);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7180_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, ADV7180_BRI_MIN,
			  ADV7180_BRI_MAX, 1, ADV7180_BRI_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7180_ctrl_ops,
			  V4L2_CID_CONTRAST, ADV7180_CON_MIN,
			  ADV7180_CON_MAX, 1, ADV7180_CON_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7180_ctrl_ops,
			  V4L2_CID_SATURATION, ADV7180_SAT_MIN,
			  ADV7180_SAT_MAX, 1, ADV7180_SAT_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7180_ctrl_ops,
			  V4L2_CID_HUE, ADV7180_HUE_MIN,
			  ADV7180_HUE_MAX, 1, ADV7180_HUE_DEF);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	pr_debug("%s: exit\n", __func__);
	return 0;
}

static int adv7180_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	interval->interval.numerator = 1;
	interval->interval.denominator = 60;

	return 0;
}

static void adv7180_exit_controls(struct adv7180_state *state)
{
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
}

static int adv7180_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_UYVY8_2X8;

	return 0;
}

static int adv7180_g_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	struct adv7180_state *state = to_state(sd);

	if (!state || !fmt) {
		pr_err("%s - null params %p, %p", __func__, sd, fmt);
		return -EINVAL;
	}

	fmt->code = V4L2_MBUS_FMT_UYVY8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->width = 720;
	if (state->chip_info->flags & ADV7180_FLAG_I2P)
		fmt->height = state->curr_norm & V4L2_STD_525_60 ? 507 : 576;
	else
		fmt->height = state->curr_norm & V4L2_STD_525_60 ? 254 : 288;

	pr_debug("%s (%d) - %d x %d (%x)", __func__,
			state->device_num, fmt->width, fmt->height,
			(int)state->curr_norm);

	return 0;
}

static int adv7180_set_field_mode(struct adv7180_state *state)
{
	pr_debug("%s: entry!!!\n", __func__);

	if (!(state->chip_info->flags & ADV7180_FLAG_I2P))
		return 0;

	if (state->field == V4L2_FIELD_NONE) {
		if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
			adv7180_csi_write(state, 0x01, 0x20);
			adv7180_csi_write(state, 0x02, 0x28);
			adv7180_csi_write(state, 0x03, 0x38);
			adv7180_csi_write(state, 0x04, 0x30);
			adv7180_csi_write(state, 0x05, 0x30);
			adv7180_csi_write(state, 0x06, 0x80);
			adv7180_csi_write(state, 0x07, 0x70);
			adv7180_csi_write(state, 0x08, 0x50);
		}
		adv7180_vpp_write(state, 0xa3, 0x00);
		adv7180_vpp_write(state, 0x5b, 0x00);
		adv7180_vpp_write(state, 0x55, 0x80);
	} else {
		if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
			adv7180_csi_write(state, 0x01, 0x18);
			adv7180_csi_write(state, 0x02, 0x18);
			adv7180_csi_write(state, 0x03, 0x30);
			adv7180_csi_write(state, 0x04, 0x20);
			adv7180_csi_write(state, 0x05, 0x28);
			adv7180_csi_write(state, 0x06, 0x40);
			adv7180_csi_write(state, 0x07, 0x58);
			adv7180_csi_write(state, 0x08, 0x30);
		}
		adv7180_vpp_write(state, 0xa3, 0x70);
		adv7180_vpp_write(state, 0x5b, 0x80);
		adv7180_vpp_write(state, 0x55, 0x00);
	}

	pr_debug("%s: exit!!!\n", __func__);
	return 0;
}

static int adv7180_set_irq_config(struct adv7180_state *state)
{
	adv7180_write(state, ADV7180_REG_ICONF1,
			ADV7180_ICONF1_ACTIVE_LOW |
			ADV7180_ICONF1_PSYNC_ONLY |
			ADV7180_ICONF1_ACTIVE_TO_CLR);
	adv7180_write(state, ADV7180_REG_IMR1,
			ADV7180_IRQ1_LOCK |
			ADV7180_IRQ1_UNLOCK |
			ADV7180_IRQ1_MACROVISION);
	adv7180_write(state, ADV7180_REG_IMR2, 0);
	adv7180_write(state, ADV7180_REG_IMR3, ADV7180_IRQ3_AD_CHANGE);
	adv7180_write(state, ADV7180_REG_IMR4, 0);

	enable_irq(state->irq);

	return 0;
}

static int adv7180_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_format *format)
{
	struct adv7180_state *state = to_state(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format = *v4l2_subdev_get_try_format(fh, 0);
	} else {
		adv7180_g_mbus_fmt(sd, &format->format);
		format->format.field = state->field;
	}

	return 0;
}

static int adv7180_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_format *format)
{
	struct adv7180_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *framefmt;

	switch (format->format.field) {
	case V4L2_FIELD_NONE:
		if (!(state->chip_info->flags & ADV7180_FLAG_I2P))
			format->format.field = V4L2_FIELD_INTERLACED;
		break;
	default:
		format->format.field = V4L2_FIELD_INTERLACED;
		break;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		framefmt = &format->format;
		if (state->field != format->format.field) {
			state->field = format->format.field;
			adv7180_set_power(state, false);
			adv7180_set_field_mode(state);
			adv7180_set_power(state, true);
		}
	} else {
		framefmt = v4l2_subdev_get_try_format(fh, 0);
	}

	return adv7180_g_mbus_fmt(sd, framefmt);
}

static int adv7180_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	struct adv7180_state *state = to_state(sd);

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
		cfg->type = V4L2_MBUS_CSI2;
		cfg->flags = V4L2_MBUS_CSI2_1_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		/*
		 * The ADV7180 sensor supports BT.601/656 output modes.
		 * The BT.656 is default and not yet configurable by s/w.
		 */
		cfg->flags = V4L2_MBUS_MASTER | V4L2_MBUS_PCLK_SAMPLE_RISING |
				 V4L2_MBUS_DATA_ACTIVE_HIGH;
		cfg->type = V4L2_MBUS_BT656;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops adv7180_video_ops = {
	.querystd = adv7180_querystd,
	.g_input_status = adv7180_g_input_status,
	.s_routing = adv7180_s_routing,
	.g_frame_interval = adv7180_g_frame_interval,
	.g_mbus_fmt = adv7180_g_mbus_fmt,
	.g_mbus_config = adv7180_g_mbus_config,
	.s_stream = adv7180_s_stream,
};


static const struct v4l2_subdev_core_ops adv7180_core_ops = {
	.s_std = adv7180_s_std,
	.s_power = adv7180_s_power,
};

static const struct v4l2_subdev_pad_ops adv7180_pad_ops = {
	.enum_mbus_code = adv7180_enum_mbus_code,
	.set_fmt = adv7180_set_pad_format,
	.get_fmt = adv7180_get_pad_format,
};

static const struct v4l2_subdev_ops adv7180_ops = {
	.core = &adv7180_core_ops,
	.video = &adv7180_video_ops,
	.pad = &adv7180_pad_ops,
};

static irqreturn_t adv7180_irq(int irq, void *dev)
{
	struct adv7180_state *state = dev;

	schedule_delayed_work(&state->irq_delayed_work,
						msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static void adv7180_irq_delay_work(struct work_struct *work)
{
	struct adv7180_state *state;
	u8 isr1;
	u8 isr2;
	u8 isr3;
	u8 isr4;

	state = container_of(work, struct adv7180_state,
				irq_delayed_work.work);

	mutex_lock(&state->mutex);

	isr1 = adv7180_read(state, ADV7180_REG_ISR1);
	isr2 = adv7180_read(state, ADV7180_REG_ISR2);
	isr3 = adv7180_read(state, ADV7180_REG_ISR3);
	isr4 = adv7180_read(state, ADV7180_REG_ISR4);

	pr_debug("%s, dev %d got interrupt - %x %x %x %x", __func__,
			state->device_num, isr1, isr2, isr3, isr4);

	/* clear interrupts */
	adv7180_write(state, ADV7180_REG_ICR1, isr1);
	adv7180_write(state, ADV7180_REG_ICR2, isr2);
	adv7180_write(state, ADV7180_REG_ICR3, isr3);
	adv7180_write(state, ADV7180_REG_ICR4, isr4);

	if ((isr1 & ADV7180_IRQ1_LOCK) ||
		(isr1 & ADV7180_IRQ1_UNLOCK)) {
		int lock_status;
		struct v4l2_event event = {0};
		int *ptr = (int *)event.u.data;

		__adv7180_status(state, &lock_status, NULL);
		ptr[0] = adv7180_inp_to_ba(state->input);
		ptr[1] = lock_status;
		event.type = lock_status ?
			V4L2_EVENT_MSM_BA_SIGNAL_LOST_LOCK :
			V4L2_EVENT_MSM_BA_SIGNAL_IN_LOCK;
		v4l2_subdev_notify(&state->sd,
			event.type, &event);
		if (lock_status)
			goto cleanup;
	}
	if ((isr3 & ADV7180_IRQ3_AD_CHANGE) && state->autodetect) {
		struct v4l2_event event = {0};
		int *ptr = (int *)event.u.data;
		__adv7180_status(state, NULL, &state->curr_norm);
		ptr[0] = adv7180_inp_to_ba(state->input);
		ptr[1] = state->curr_norm;
		event.type = V4L2_EVENT_MSM_BA_SOURCE_CHANGE;
		v4l2_subdev_notify(&state->sd,
			event.type, &event);
	}
	if (isr1 & ADV7180_IRQ1_MACROVISION) {
		struct v4l2_event event = {0};
		int *ptr = (int *)event.u.data;
		ptr[0] = adv7180_inp_to_ba(state->input);
		event.type = V4L2_EVENT_MSM_BA_CP;
		v4l2_subdev_notify(&state->sd,
			event.type, &event);
	}

cleanup:
	mutex_unlock(&state->mutex);

	return;
}

static int adv7180_init(struct adv7180_state *state)
{
	int ret;

	/* ITU-R BT.656-4 compatible */
	ret = adv7180_write(state, ADV7180_REG_EXTENDED_OUTPUT_CONTROL,
			ADV7180_EXTENDED_OUTPUT_CONTROL_NTSCDIS);
	if (ret < 0)
		return ret;

	/* Manually set V bit end position in NTSC mode */
	return adv7180_write(state, ADV7180_REG_NTSC_V_BIT_END,
					ADV7180_NTSC_V_BIT_END_MANUAL_NVEND);
}

static int adv7180_set_std(struct adv7180_state *state, unsigned int std)
{
	return adv7180_write(state, ADV7180_REG_INPUT_CONTROL,
		(std << 4) | state->input);
}

static int adv7180_select_input(struct adv7180_state *state, unsigned int input)
{
	int ret;
	pr_debug("%s: entry\n", __func__);
	if (input == ADV7180_INPUT_DISABLED)
		input = 0x00;

	ret = adv7180_read(state, ADV7180_REG_INPUT_CONTROL);
	if (ret < 0)
		return ret;

	ret &= ~ADV7180_INPUT_CONTROL_INSEL_MASK;
	ret |= input;
	pr_debug("%s: exit\n", __func__);
	return adv7180_write(state, ADV7180_REG_INPUT_CONTROL, ret);
}

static int adv7182_init(struct adv7180_state *state)
{
	int ret = 0;
	pr_debug("%s: entry\n", __func__);

	/* Enable fast lock */
	adv7180_write(state, 0x40E0, 0x01);
	pr_debug("%s: fast lock enabled\n", __func__);

	adv7180_write(state, 0x809c, 0x00);
	adv7180_write(state, 0x809c, 0xff);

	if (state->chip_info->flags & ADV7180_FLAG_V2) {
		adv7180_write(state, 0x0080, 0x51);
		adv7180_write(state, 0x0081, 0x51);
		adv7180_write(state, 0x0082, 0x68);
	}

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
		adv7180_write(state, 0x0003, 0x4e);
		adv7180_write(state, 0x0004, 0x57);
		adv7180_write(state, 0x0013, 0x00);
		adv7180_write(state, 0x001d, 0xc0);
	} else {
		if (state->chip_info->flags & ADV7180_FLAG_V2)
			adv7180_write(state, 0x0004, 0x17);
		else
			adv7180_write(state, 0x0004, 0x07);
		adv7180_write(state, 0x0003, 0x0c);
		adv7180_write(state, 0x001d, 0x40);
	}

	if (state->field == V4L2_FIELD_NONE)
		ret = adv7180_csi_write(state, 0x1D, 0x80);
	usleep(I2C_RW_DELAY);

	if (!state->csi_configured) {
		pr_debug("%s: config csi and vpp!!!\n",
			__func__);
		adv7180_vpp_write(state, 0xa3, 0x00);
		adv7180_vpp_write(state, 0x5b, 0x00);
		adv7180_vpp_write(state, 0x55, 0x80);

		adv7180_csi_write(state, 0x01, 0x20);
		adv7180_csi_write(state, 0x02, 0x28);
		adv7180_csi_write(state, 0x03, 0x38);
		adv7180_csi_write(state, 0x04, 0x30);
		adv7180_csi_write(state, 0x05, 0x30);
		adv7180_csi_write(state, 0x06, 0x80);
		adv7180_csi_write(state, 0x07, 0x70);
		adv7180_csi_write(state, 0x08, 0x50);

		adv7180_csi_write(state, 0xDE, 0x02);
		usleep(I2C_RW_DELAY);
		adv7180_csi_write(state, 0xD2, 0xF7);
		adv7180_csi_write(state, 0xD8, 0x65);
		adv7180_csi_write(state, 0xE0, 0x09);
		adv7180_csi_write(state, 0x2C, 0x00);
		adv7180_csi_write(state, 0x1D, 0x80);
		state->csi_configured = 1;
	} else {
		pr_debug("%s: csi/vpp already configured!!!\n",
			__func__);
	}

	pr_debug("%s: exit\n", __func__);
	return 0;
}

static int adv7182_set_std(struct adv7180_state *state, unsigned int std)
{
	return adv7180_write(state, ADV7182_REG_INPUT_VIDSEL, std << 4);
}

enum adv7182_input_type {
	ADV7182_INPUT_TYPE_CVBS,
	ADV7182_INPUT_TYPE_DIFF_CVBS,
	ADV7182_INPUT_TYPE_SVIDEO,
	ADV7182_INPUT_TYPE_YPBPR,
};

static enum adv7182_input_type adv7182_get_input_type(unsigned int input)
{
	switch (input) {
	case ADV7182_INPUT_CVBS_AIN1:
	case ADV7182_INPUT_CVBS_AIN2:
	case ADV7182_INPUT_CVBS_AIN3:
	case ADV7182_INPUT_CVBS_AIN4:
	case ADV7182_INPUT_CVBS_AIN5:
	case ADV7182_INPUT_CVBS_AIN6:
	case ADV7182_INPUT_CVBS_AIN7:
	case ADV7182_INPUT_CVBS_AIN8:
		return ADV7182_INPUT_TYPE_CVBS;
	case ADV7182_INPUT_SVIDEO_AIN1_AIN2:
	case ADV7182_INPUT_SVIDEO_AIN3_AIN4:
	case ADV7182_INPUT_SVIDEO_AIN5_AIN6:
	case ADV7182_INPUT_SVIDEO_AIN7_AIN8:
		return ADV7182_INPUT_TYPE_SVIDEO;
	case ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3:
	case ADV7182_INPUT_YPRPB_AIN4_AIN5_AIN6:
		return ADV7182_INPUT_TYPE_YPBPR;
	case ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2:
	case ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4:
	case ADV7182_INPUT_DIFF_CVBS_AIN5_AIN6:
	case ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8:
		return ADV7182_INPUT_TYPE_DIFF_CVBS;
	default: /* Will never happen */
		return 0;
	}
}

static unsigned int adv7182_lbias_settings[][3] = {
	[ADV7182_INPUT_TYPE_CVBS] = { 0xCB, 0x4E, 0x80 },
	[ADV7182_INPUT_TYPE_DIFF_CVBS] = { 0xC0, 0x4E, 0x80 },
	[ADV7182_INPUT_TYPE_SVIDEO] = { 0x0B, 0xCE, 0x80 },
	[ADV7182_INPUT_TYPE_YPBPR] = { 0x0B, 0x4E, 0xC0 },
};

static unsigned int adv7280_lbias_settings[][3] = {
	[ADV7182_INPUT_TYPE_CVBS] = { 0xCD, 0x4E, 0x80 },
	[ADV7182_INPUT_TYPE_DIFF_CVBS] = { 0xC0, 0x4E, 0x80 },
	[ADV7182_INPUT_TYPE_SVIDEO] = { 0x0B, 0xCE, 0x80 },
	[ADV7182_INPUT_TYPE_YPBPR] = { 0x0B, 0x4E, 0xC0 },
};

static int adv7182_select_input(struct adv7180_state *state, unsigned int input)
{
	enum adv7182_input_type input_type;
	unsigned int *lbias;
	unsigned int i;
	int ret;
	pr_debug("%s: entry\n", __func__);
	if (input == ADV7180_INPUT_DISABLED)
		return adv7180_write(state, ADV7180_REG_INPUT_CONTROL, 0xff);

	if (state->curr_input == input) {
		pr_debug("%s: same input ain\n", __func__);
	} else {
		ret = adv7180_write(state, ADV7180_REG_INPUT_CONTROL, input);
		if (ret)
			return ret;

		state->curr_input = input;

		input_type = adv7182_get_input_type(input);

		switch (input_type) {
		case ADV7182_INPUT_TYPE_CVBS:
		case ADV7182_INPUT_TYPE_DIFF_CVBS:
			/* ADI recommands to use the SH1 filter */
			adv7180_write(state, 0x0017, 0x41);
			break;
		default:
			adv7180_write(state, 0x0017, 0x01);
			break;
		}

		if (state->chip_info->flags & ADV7180_FLAG_V2)
			lbias = adv7280_lbias_settings[input_type];
		else
			lbias = adv7182_lbias_settings[input_type];

		for (i = 0; i < ARRAY_SIZE(adv7182_lbias_settings[0]); i++)
			adv7180_write(state, 0x0052 + i, lbias[i]);

		if (input_type == ADV7182_INPUT_TYPE_DIFF_CVBS) {
			/* ADI required writes to make differential CVBS work */
			adv7180_write(state, 0x005f, 0xa8);
			adv7180_write(state, 0x005a, 0x90);
			adv7180_write(state, 0x0060, 0xb0);
			adv7180_write(state, 0x80b6, 0x08);
			adv7180_write(state, 0x80c0, 0xa0);
		} else {
			adv7180_write(state, 0x005f, 0xf0);
			adv7180_write(state, 0x005a, 0xd0);
			adv7180_write(state, 0x0060, 0x10);
		}
	}

	pr_debug("%s: exit\n", __func__);
	return 0;
}

static const struct adv7180_chip_info adv7180_info = {
	/* We cannot discriminate between LQFP and 40-pin LFCSP, so accept
	 * all inputs and let the card driver take care of validation
	 */
	.valid_input_mask = BIT(ADV7180_INPUT_CVBS_AIN1) |
		BIT(ADV7180_INPUT_CVBS_AIN2) |
		BIT(ADV7180_INPUT_CVBS_AIN3) |
		BIT(ADV7180_INPUT_CVBS_AIN4) |
		BIT(ADV7180_INPUT_CVBS_AIN5) |
		BIT(ADV7180_INPUT_CVBS_AIN6) |
		BIT(ADV7180_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7180_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7180_INPUT_SVIDEO_AIN5_AIN6) |
		BIT(ADV7180_INPUT_YPRPB_AIN1_AIN2_AIN3) |
		BIT(ADV7180_INPUT_YPRPB_AIN4_AIN5_AIN6),
	.init = adv7180_init,
	.set_std = adv7180_set_std,
	.select_input = adv7180_select_input,
};

static const struct adv7180_chip_info adv7182_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
};

static const struct adv7180_chip_info adv7280_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_I2P,
};

static const struct adv7180_chip_info adv7280_m_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_CVBS_AIN5) |
		BIT(ADV7182_INPUT_CVBS_AIN6) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN5_AIN6) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3) |
		BIT(ADV7182_INPUT_YPRPB_AIN4_AIN5_AIN6),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_MIPI_CSI2 | ADV7180_FLAG_I2P,
};

static const struct adv7180_chip_info adv7281_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_MIPI_CSI2,
};

static const struct adv7180_chip_info adv7281_m_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_MIPI_CSI2,
};

static const struct adv7180_chip_info adv7281_ma_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_CVBS_AIN5) |
		BIT(ADV7182_INPUT_CVBS_AIN6) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN5_AIN6) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_YPRPB_AIN1_AIN2_AIN3) |
		BIT(ADV7182_INPUT_YPRPB_AIN4_AIN5_AIN6) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN5_AIN6) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_MIPI_CSI2,
};

static const struct adv7180_chip_info adv7282_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_I2P,
};

static const struct adv7180_chip_info adv7282_m_info = {
	.valid_input_mask = BIT(ADV7182_INPUT_CVBS_AIN1) |
		BIT(ADV7182_INPUT_CVBS_AIN2) |
		BIT(ADV7182_INPUT_CVBS_AIN3) |
		BIT(ADV7182_INPUT_CVBS_AIN4) |
		BIT(ADV7182_INPUT_CVBS_AIN7) |
		BIT(ADV7182_INPUT_CVBS_AIN8) |
		BIT(ADV7182_INPUT_SVIDEO_AIN1_AIN2) |
		BIT(ADV7182_INPUT_SVIDEO_AIN3_AIN4) |
		BIT(ADV7182_INPUT_SVIDEO_AIN7_AIN8) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN1_AIN2) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN3_AIN4) |
		BIT(ADV7182_INPUT_DIFF_CVBS_AIN7_AIN8),
	.init = adv7182_init,
	.set_std = adv7182_set_std,
	.select_input = adv7182_select_input,
	.flags = ADV7180_FLAG_V2 | ADV7180_FLAG_MIPI_CSI2 | ADV7180_FLAG_I2P,
};

static int init_device(struct adv7180_state *state)
{
	int ret = 0;
	u16 csi_i2c_addr = 0;
	u16 vpp_i2c_addr = 0;
	pr_debug("%s : entry\n", __func__);

	mutex_lock(&state->mutex);

	ret = adv7180_write(state, ADV7180_REG_PWR_MAN, 0);
	if (ret)
		goto out_unlock;

	usleep(5000);

	if (state->device_num == 0) {
		csi_i2c_addr = ADV7180_DEFAULT_CSI_I2C_ADDR;
		vpp_i2c_addr = ADV7180_DEFAULT_VPP_I2C_ADDR;
	} else if (state->device_num == 1) {
		csi_i2c_addr = ADV7180_DEVICE2_CSI_I2C_ADDR;
		vpp_i2c_addr = ADV7180_DEVICE2_VPP_I2C_ADDR;
	} else {
		pr_err("%s : Unsupported adv device %d\n",
			__func__, state->device_num);
		return -EIO;
	}

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2)
		adv7180_write(state, ADV7180_REG_CSI_SLAVE_ADDR,
			csi_i2c_addr << 1);

	if (state->chip_info->flags & ADV7180_FLAG_I2P)
		adv7180_write(state, ADV7180_REG_VPP_SLAVE_ADDR,
			vpp_i2c_addr << 1);

	/* Default ain1 CVBS */
	adv7182_select_input(state, ADV7182_INPUT_CVBS_AIN1);

	ret = state->chip_info->init(state);
	if (ret)
		goto out_unlock;

	ret = adv7180_program_std(state);
	if (ret)
		goto out_unlock;

	adv7180_set_irq_config(state);

	pr_debug("%s : exit\n", __func__);
out_unlock:
	mutex_unlock(&state->mutex);

	return ret;
}

static int adv7180_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv7180_state *state;
	struct adv7180_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	int device_num = 0;
	u16 csi_i2c_addr = 0;
	u16 vpp_i2c_addr = 0;
	int ret = 0;
	pr_debug("%s : kpi entry\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	pdata = (struct adv7180_platform_data *) client->dev.platform_data;
	if (!pdata) {
		pr_err("%s : Getting Platform data : Failed\n", __func__);
		return -EIO;
	}

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		 client->addr, client->adapter->name);

	device_num = pdata->dev_num;

	if (device_num == 0) {
		csi_i2c_addr = ADV7180_DEFAULT_CSI_I2C_ADDR;
		vpp_i2c_addr = ADV7180_DEFAULT_VPP_I2C_ADDR;
	} else if (device_num == 1) {
		csi_i2c_addr = ADV7180_DEVICE2_CSI_I2C_ADDR;
		vpp_i2c_addr = ADV7180_DEVICE2_VPP_I2C_ADDR;
	} else {
		pr_err("%s : Unsupported ADV device %d\n",
			__func__, device_num);
		return -EIO;
	}

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	state->client = client;
	state->device_num = device_num;
	state->chip_info = (struct adv7180_chip_info *)id->driver_data;
	if (state->chip_info->flags & ADV7180_FLAG_I2P)
		state->field = V4L2_FIELD_NONE;
	else
		state->field = V4L2_FIELD_INTERLACED;

	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2) {
		state->csi_client = i2c_new_dummy(client->adapter,
				csi_i2c_addr);
		if (!state->csi_client)
			return -ENOMEM;
	}

	if (state->chip_info->flags & ADV7180_FLAG_I2P) {
		state->vpp_client = i2c_new_dummy(client->adapter,
				vpp_i2c_addr);
		if (!state->vpp_client) {
			ret = -ENOMEM;
			goto err_unregister_csi_client;
		}
	}

	mutex_init(&state->mutex);

	if (state->device_num == 0) {
		/* Default to NTSC for faster lock */
		state->autodetect = false;
		state->curr_norm = V4L2_STD_NTSC;
	} else {
		state->autodetect = true;
	}

	if (state->chip_info->flags & ADV7180_FLAG_V2)
		state->powered = false;
	else
		state->powered = true;
	state->input = 0;
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv7180_ops);

	if (gpio_is_valid(pdata->pwdnb_gpio)
				&& gpio_is_valid(pdata->rstb_gpio)
				&& pdata->pwr_on == 1) {
		ret = gpio_request(pdata->pwdnb_gpio, "pwdnb_gpio");
		if (ret)
			goto err_unregister_vpp_client;

		ret = gpio_request(pdata->rstb_gpio, "rstb_gpio");
		if (ret)
			goto err_unregister_vpp_client;

		ret = gpio_direction_output(pdata->pwdnb_gpio, 1);
		ret |= gpio_direction_output(pdata->rstb_gpio, 1);
		if (ret) {
			pr_err("%s : Failed gpio_direction %x", __func__, ret);
			goto err_unregister_vpp_client;
		}

		/* Required Control sequence for Powerdown and Reset Pins */
		/* Delay required following Reset before I2C is accessible */
		ret = gpio_direction_output(pdata->pwdnb_gpio, 0);
		ret |= gpio_direction_output(pdata->rstb_gpio, 0);
		usleep(2000);
		ret |= gpio_direction_output(pdata->pwdnb_gpio, 1);
		usleep(5000);
		ret |= gpio_direction_output(pdata->rstb_gpio, 1);

		if (ret) {
			pr_err("%s : Failed gpio_direction %x", __func__, ret);
			goto err_unregister_vpp_client;
		}

	} else {
		pr_debug("%s : ADV device Power up sequence not required\n",
			__func__);
	}


	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request(pdata->irq_gpio, "irq_gpio");
		if (ret) {
			pr_err("%s : Failed to request irq_gpio %x",
					__func__, ret);
			goto err_unregister_vpp_client;
		}

		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			pr_err("%s : Failed gpio_direction irq %x",
					__func__, ret);
			goto err_unregister_vpp_client;
		}

		state->irq = gpio_to_irq(pdata->irq_gpio);
		if (state->irq) {
			ret = request_irq(state->irq, adv7180_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					DRIVER_NAME, state);
			if (ret) {
				pr_err("%s : Failed request_irq %x",
						__func__, ret);
				goto err_unregister_vpp_client;
			}
		} else {
			pr_err("%s : Failed gpio_to_irq %x", __func__, ret);
			ret = -EINVAL;
			goto err_unregister_vpp_client;
		}

		/* disable irq until chip interrupts are programmed */
		disable_irq(state->irq);

		INIT_DELAYED_WORK(&state->irq_delayed_work,
				adv7180_irq_delay_work);
	}

	/* Initialize csi configurion flag */
	state->csi_configured = 0;
	/* Initialize current video standard mode */
	state->curr_mode = V4L2_STD_ALL;
	/* Initialize current ain input */
	state->curr_input = ADV7182_INPUT_UNINITIALIZED;

	ret = adv7180_init_controls(state);
	if (ret)
		goto err_free_irq;

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	state->sd.entity.flags |= MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(&state->sd.entity, 1, &state->pad, 0);
	if (ret)
		goto err_free_ctrl;

	ret = init_device(state);
	if (ret)
		goto err_media_entity_cleanup;


	if (device_num == 0) {
		ret = msm_ba_register_subdev_node(sd);
	} else if (device_num == 1) {
		ret = msm_ba_register_subdev_node(sd);
	} else {
		ret = -EIO;
		pr_err("%s : Unsupported adv device %d\n",
			__func__, device_num);
	}
	if (ret)
		goto err_media_entity_cleanup;

	pr_debug("%s : kpi exit\n", __func__);
	return 0;

err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err_free_ctrl:
	adv7180_exit_controls(state);
err_free_irq:
	if (state->irq > 0)
		free_irq(client->irq, state);
err_unregister_vpp_client:
	if (state->chip_info->flags & ADV7180_FLAG_I2P)
		i2c_unregister_device(state->vpp_client);
err_unregister_csi_client:
	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2)
		i2c_unregister_device(state->csi_client);
	mutex_destroy(&state->mutex);
err:
	return ret;
}

static int adv7180_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7180_state *state = to_state(sd);
	pr_debug("%s : entry\n", __func__);

	if (state->device_num == 0 || state->device_num == 1) {
		pr_debug("%s : deregister from ba dev_num %d\n",
			__func__, state->device_num);
		msm_ba_unregister_subdev_node(sd);
	} else {
		pr_err("%s : Unsupported ADV device %d\n",
			__func__, state->device_num);
		return -EIO;
	}

	if (state->irq > 0)
		free_irq(state->irq, state);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	adv7180_exit_controls(state);
	mutex_destroy(&state->mutex);

	if (state->chip_info->flags & ADV7180_FLAG_I2P)
		i2c_unregister_device(state->vpp_client);
	if (state->chip_info->flags & ADV7180_FLAG_MIPI_CSI2)
		i2c_unregister_device(state->csi_client);

	pr_debug("%s : exit\n", __func__);
	return 0;
}

static const struct i2c_device_id adv7180_id[] = {
	{ "adv7180", (kernel_ulong_t)&adv7180_info },
	{ "adv7182", (kernel_ulong_t)&adv7182_info },
	{ "adv7280", (kernel_ulong_t)&adv7280_info },
	{ "adv7280-m", (kernel_ulong_t)&adv7280_m_info },
	{ "adv7281", (kernel_ulong_t)&adv7281_info },
	{ "adv7281-m", (kernel_ulong_t)&adv7281_m_info },
	{ "adv7281-ma", (kernel_ulong_t)&adv7281_ma_info },
	{ "adv7282", (kernel_ulong_t)&adv7282_info },
	{ "adv7282-m", (kernel_ulong_t)&adv7282_m_info },
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7180_id);

#ifdef CONFIG_PM_SLEEP
static int adv7180_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7180_state *state = to_state(sd);

	return adv7180_set_power(state, false);
}

static int adv7180_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7180_state *state = to_state(sd);
	int ret;
	pr_debug("%s : entry\n", __func__);

	ret = init_device(state);
	if (ret < 0)
		return ret;

	ret = adv7180_set_power(state, state->powered);
	if (ret)
		return ret;

	pr_debug("%s : exit\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(adv7180_pm_ops, adv7180_suspend, adv7180_resume);
#define ADV7180_PM_OPS (&adv7180_pm_ops)

#else
#define ADV7180_PM_OPS NULL
#endif

static struct i2c_driver adv7180_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = KBUILD_MODNAME,
		   .pm = ADV7180_PM_OPS,
		   },
	.probe = adv7180_probe,
	.remove = adv7180_remove,
	.id_table = adv7180_id,
};

module_i2c_driver(adv7180_driver);

MODULE_DESCRIPTION("Analog Devices ADV7180 video decoder driver");
MODULE_AUTHOR("Mocean Laboratories");
MODULE_LICENSE("GPL v2");
