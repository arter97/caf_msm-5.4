/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011-2012, 2014-2015, The Linux Foundation. All rights
 * reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>
#include <video/msm_dba.h>

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

/* MXT_TOUCH_KEYARRAY_T15 */
#define MXT_KEYARRAY_MAX_KEYS	32

/* Bootoader IDs */
#define MXT_BOOTLOADER_ID_224		0x0A
#define MXT_BOOTLOADER_ID_224E		0x06
#define MXT_BOOTLOADER_ID_1386		0x01
#define MXT_BOOTLOADER_ID_1386E		0x10
#define MXT_BOOTLOADER_ID_1386E_v2_4	0x24

/* Touchscreen absolute values */
#define MXT_MAX_FINGER		10
#define T7_DATA_SIZE		3
#define MXT_CFG_VERSION_LEN	3

enum mxt_device_state { INIT, APPMODE, BOOTLOADER };

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};

/* Config data for a given maXTouch controller with a specific firmware */
struct mxt_config_info {
	const u8 *config;
	size_t config_length;
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 bootldr_id;
	/* Points to the firmware name to be upgraded to */
	const char *fw_name;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const struct mxt_config_info *config_array;
	size_t config_array_size;

	/* touch panel's minimum and maximum coordinates */
	u32 panel_minx;
	u32 panel_maxx;
	u32 panel_miny;
	u32 panel_maxy;

	/* display's minimum and maximum coordinates */
	u32 disp_minx;
	u32 disp_maxx;
	u32 disp_miny;
	u32 disp_maxy;

	unsigned long irqflags;
	bool	i2c_pull_up;
	bool no_regulator_support;
	bool use_abs_reportid;
	bool	digital_pwr_regulator;
	int reset_gpio;
	u32 reset_gpio_flags;
	int irq_gpio;
	u32 irq_gpio_flags;
	int *key_codes;

	u8(*read_chg) (void);
	int (*init_hw) (bool);
	int (*power_on) (bool);

	bool iox_support;
	int iox_slave_id;
	struct msm_dba_reg_info *dba_host;
};

/* auxilary data to handle display abstraction layer */
struct mxt_data_dba_aux {
	void *handle;
	struct msm_dba_ops ops;
	void *mxt_info;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	const struct mxt_config_info *config_info;
	enum mxt_device_state state;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;
	struct delayed_work mxt_init_work;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	u8 t7_data[T7_DATA_SIZE];
	u16 t7_start_addr;
	u32 keyarray_old;
	u32 keyarray_new;
	u8 t9_max_reportid;
	u8 t9_min_reportid;
	u8 t15_max_reportid;
	u8 t15_min_reportid;
	u8 t42_max_reportid;
	u8 t42_min_reportid;
	u8 cfg_version[MXT_CFG_VERSION_LEN];
	int cfg_version_idx;
	int t38_start_addr;
	bool update_cfg;
	const char *fw_name;
	bool dev_on;
	struct mxt_data_dba_aux *dba_aux;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
