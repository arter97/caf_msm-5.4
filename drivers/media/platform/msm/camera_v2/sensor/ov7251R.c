/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include "msm_sensor.h"
#include "msm_camera_i2c.h"
#include "msm_cci.h"

#include <media/ov7251R_common.h>

#define OV7251R_SENSOR_NAME "ov7251R"
DEFINE_MSM_MUTEX(ov7251R_mut);


static struct msm_sensor_ctrl_t ov7251R_s_ctrl;

static struct v4l2_subdev_info ov7251R_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
};

static const struct i2c_device_id ov7251R_i2c_id[] = {
	{OV7251R_SENSOR_NAME, (kernel_ulong_t)&ov7251R_s_ctrl},
	{ }
};


static int32_t msm_ov7251R_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov7251R_s_ctrl);
}

static struct i2c_driver ov7251R_i2c_driver = {
	.id_table = ov7251R_i2c_id,
	.probe  = msm_ov7251R_i2c_probe,
	.driver = {
		.name = OV7251R_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov7251R_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov7251R_dt_match[] = {
	{.compatible = "qcom1,ov7251R", .data = &ov7251R_s_ctrl},
	{}
};


struct msm_camera_i2c_reg_array ov7251R_reg_array[1];
struct msm_camera_i2c_reg_setting ov7251R_reg_setting = {
	.reg_setting = ov7251R_reg_array,
	.size = ARRAY_SIZE(ov7251R_reg_array),
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0
};

MODULE_DEVICE_TABLE(of, ov7251R_dt_match);

static struct platform_driver ov7251R_platform_driver = {
	.driver = {
		.name = "qcom1,ov7251R",
		.owner = THIS_MODULE,
		.of_match_table = ov7251R_dt_match,
	},
};

static int32_t ov7251R_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov7251R_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}


static int __init ov7251R_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov7251R_platform_driver,
		ov7251R_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov7251R_i2c_driver);
}


static void __exit ov7251R_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov7251R_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov7251R_s_ctrl);
		platform_driver_unregister(&ov7251R_platform_driver);
	} else
		i2c_del_driver(&ov7251R_i2c_driver);
	return;
}

#ifdef OV7251R_DEBUG
static int ov7251R_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = msm_sensor_power_up(s_ctrl);
	return rc;
}

static int ov7251R_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	return msm_sensor_power_down(s_ctrl);
}

static struct msm_sensor_fn_t ov7251R_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov7251R_sensor_power_up,
	.sensor_power_down = ov7251R_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};
#endif

static struct msm_sensor_ctrl_t ov7251R_s_ctrl = {
	.sensor_i2c_client = &ov7251R_sensor_i2c_client,
	.power_setting_array.power_setting = ov7251R_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov7251R_power_setting),
	.msm_sensor_mutex = &ov7251R_mut,
	.sensor_v4l2_subdev_info = ov7251R_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov7251R_subdev_info),
#ifdef OV7251R_DEBUG
	.func_tbl = &ov7251R_sensor_func_tbl,
#endif
};


module_init(ov7251R_init_module);
module_exit(ov7251R_exit_module);
MODULE_DESCRIPTION("ov7251R");
MODULE_LICENSE("GPL v2");



