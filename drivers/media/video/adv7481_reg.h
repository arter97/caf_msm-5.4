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
#ifndef __ADV7481_REG_H__
#define __ADV7481_REG_H__

/*IO Map Registers*/
#define IO_REG_RST				0xFF
#define IO_CTRL_MAIN_RST		0xFF

#define IO_REG_I2C_CFG			0xF2
#define IO_REG_I2C_AUTOINC_EN	0x01

#define IO_REG_PWR1_REG		0x00
#define IO_CTRL_MASTER_PWDN	0x01
#define IO_CTRL_CORE_PWDN	0x02
#define IO_CTRL_XTAL_PWDN	0x04
#define IO_CTRL_RX_EN		0x02
#define IO_CTRL_RX_PWDN		0x04
#define IO_PAD_CTRLS		0x0E

#define IO_REG_PWR2_REG		0x01
#define IO_CTRL_CEC_PWDN	0xC0

#define IO_REG_CP_VID_STD	0x05

/*Offsets */
#define IO_REG_DPLL_ADDR		0xF3
#define IO_REG_CP_ADDR			0xF4
#define IO_REG_HDMI_ADDR		0xF5
#define IO_REG_EDID_ADDR		0xF6
#define IO_REG_CSI_REP_ADDR		0xF7
#define IO_REG_HDMI_INF_ADDR	0xF8
#define IO_REG_CBUS_ADDR		0xF9
#define IO_REG_CEC_ADDR			0xFA
#define IO_REG_SDP_ADDR			0xFB
#define IO_REG_CSI_TXB_ADDR		0xFC
#define IO_REG_CSI_TXA_ADDR		0xFD

/*Sub Address Map Locations */
#define IO_REG_DPLL_SADDR		0x4C
#define IO_REG_CP_SADDR			0x44
#define IO_REG_HDMI_SADDR		0x74
#define IO_REG_EDID_SADDR		0x6C
#define IO_REG_CSI_REP_SADDR	0x64
#define IO_REG_HDMI_INF_SADDR	0x62
#define IO_REG_CBUS_SADDR		0xF0
#define IO_REG_CEC_SADDR		0x82
#define IO_REG_SDP_SADDR		0xF2
#define IO_REG_CSI_TXA_SADDR	0x94
#define IO_REG_CSI_TXB_SADDR	0x88

/*HDMI Map Registers*/
#define HDMI_REG_HDMI_PARAM5	0x05
#define HDMI_REG_HDMI_PARAM6	0x0B
#define HDMI_REG_LINE_TOT1		0x1E
#define HDMI_REG_LINE_TOT2		0x1F
#define HDMI_REG_FLD0_TOT1		0x26
#define HDMI_REG_FLD0_TOT2		0x27

/*CP Map Registers*/
#define CP_REG_CONTRAST		0x3A
#define CP_REG_SATURATION	0x3B
#define CP_REG_BRIGHTNESS	0x3C
#define CP_REG_HUE			0x3D
#define CP_REG_VID_ADJ		0x3E
#define CP_CTR_VID_ADJ_EN	0x80

/*SDP Map Registers*/
#define SDP_REG_STATUS1		0x10
#define SDP_CTRL_ADRESLT	0x70

/*
 * CSI Map Registers
 */
#define CSI_REG_TX_CFG1			0x00
#define CSI_CTRL_LANE			0x07
#define CSI_CTRL_AUTO_PARAMS	0x20

/* Lock Signals */
/*bit 0 in lock, bit 1 lost lock*/
#define SDP_VDEC_LOCK			0x10

enum adv7481_adresult {
	AD_NTSM_M_J =	0x0,
	AD_NTSC_4_43 =	0x1,
	AD_PAL_M =		0x2,
	AD_PAL_60 =		0x3,
	AD_PAL_B_G =	0x4,
	AD_SECAM =		0x5,
	AD_PAL_COMB =	0x6,
	AD_SECAM_525 =	0x7,
};

enum adv7481_color_depth {
	CD_8BIT =	0x0,
	CD_10BIT =	0x1,
	CD_12BIT =	0x2,
	CD_16BIT =	0x3,
};

#endif
