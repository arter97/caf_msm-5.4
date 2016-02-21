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

#include <linux/module.h>
#include <linux/of_gpio.h>
#include "msm_camera_dt_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "led-gpio"

static struct msm_led_flash_ctrl_t fctrl;

static int32_t msm_led_gpio_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
					  void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!fctrl || !subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	pr_debug("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static int32_t msm_led_gpio_config(struct msm_led_flash_ctrl_t *fctrl,
				   void *data)
{
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	struct gpio *gpio_led;
	int gpio_level;

	if (!fctrl || !cfg) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_debug("%s:%d called, cfgtype = %d\n", __func__, __LINE__,
							cfg->cfgtype);

	gpio_led = &fctrl->flashdata->power_info.gpio_conf->cam_gpio_req_tbl[0];

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
	case MSM_CAMERA_LED_OFF:
		gpio_level = GPIO_OUT_LOW;
		if (gpio_led->flags & GPIOF_INIT_HIGH)
			gpio_level = GPIO_OUT_HIGH;
		break;

	case MSM_CAMERA_LED_LOW:
	case MSM_CAMERA_LED_HIGH:
		gpio_level = GPIO_OUT_HIGH;
		if (gpio_led->flags & GPIOF_INIT_HIGH)
			gpio_level = GPIO_OUT_LOW;
		break;

	default:
		pr_err("%s:%d invalid cfgtype: %d\n", __func__, __LINE__,
								cfg->cfgtype);
		return -EFAULT;
	}

	gpio_set_value_cansleep(gpio_led->gpio, gpio_level);

	pr_debug("%s:%d flash_set_led_state: return 0\n", __func__, __LINE__);
	return 0;
}

enum msm_led_type {
	MSM_LED_FLASH,
	MSM_LED_IR
};

static const enum msm_led_type msm_led_flash = MSM_LED_FLASH;
static const enum msm_led_type msm_led_ir = MSM_LED_IR;

static const struct of_device_id msm_led_gpio_dt_match[] = {
	{.compatible = "qcom,led-gpio-flash", .data = &msm_led_flash},
	{.compatible = "qcom,led-gpio-ir", .data = &msm_led_ir},
	{}
};

MODULE_DEVICE_TABLE(of, msm_led_gpio_dt_match);

static struct platform_driver msm_led_gpio_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_gpio_dt_match,
	},
};

static int32_t msm_led_gpio_probe(struct platform_device *pdev)
{
	int32_t rc = 0, i = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	struct gpio *gpio_led;
	int gpio_level;
	const struct of_device_id *of_id;
	enum msm_led_type *msm_led_dev;
	int group_id;

	pr_debug("%s:%d called\n", __func__, __LINE__);

	if (!of_node) {
		pr_err("%s:%d of_node NULL\n", __func__, __LINE__);
		rc = -EINVAL;
		goto ERROR0;
	}

	fctrl.pdev = pdev;
	fctrl.flash_num_sources = 0;
	fctrl.torch_num_sources = 0;

	fctrl.flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl.flashdata) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR0;
	}

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_string(of_node, "label", &flashdata->sensor_name);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		goto ERROR1;
	}

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}
	gconf = power_info->gpio_conf;

	gpio_array_size = of_gpio_count(of_node);

	if (gpio_array_size != 1) {
		pr_err("%s:%d unsupported DTSI configuration\n", __func__,
								__LINE__);
		rc = -EINVAL;
		goto ERROR2;
	}

	gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			     GFP_KERNEL);
	if (!gpio_array) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR2;
	}

	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		pr_debug("%s:%d gpio_array[%d] = %d\n", __func__, __LINE__, i,
								gpio_array[i]);
	}

	/* This will allocate gconf->cam_gpio_req_tbl.             */
	/* Take care to free it if msm_led_gpio_probe fails later. */
	rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
					    gpio_array, gpio_array_size);
	kfree(gpio_array);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		goto ERROR2;
	}

	gpio_led = &gconf->cam_gpio_req_tbl[0];

	rc = gpio_request_one(gpio_led->gpio, gpio_led->flags, gpio_led->label);
	if (rc) {
		pr_err("%s:%d gpio %d:%s request fails\n",
				__func__, __LINE__,
				gpio_led->gpio, gpio_led->label);
		goto ERROR3;
	}

	gpio_level = GPIOF_INIT_LOW;
	if (gpio_led->flags & GPIOF_INIT_HIGH)
		gpio_level = GPIO_OUT_HIGH;

	rc = gpio_direction_output(gpio_led->gpio, gpio_level);
	if (rc) {
		pr_err("%s:%d gpio %d:%s set direction fails\n",
				__func__, __LINE__,
				gpio_led->gpio, gpio_led->label);
		goto ERROR4;
	}

	of_id = of_match_node(msm_led_gpio_dt_match, of_node);
	if (!of_id) {
		pr_err("%s:%d of_match_node fails\n", __func__, __LINE__);
		goto ERROR4;
	}
	msm_led_dev = (enum msm_led_type *) of_id->data;
	pr_debug("%s:%d *msm_led_dev = 0x%x\n", __func__, __LINE__,
						(unsigned int) *msm_led_dev);

	group_id = MSM_CAMERA_SUBDEV_LED_FLASH;
	if (*msm_led_dev == MSM_LED_IR)
		group_id = MSM_CAMERA_SUBDEV_LED_IR;

	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl, group_id);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		goto ERROR4;
	}

	return rc;

ERROR4:
	gpio_free(gpio_led->gpio);
ERROR3:
	kfree(gconf->cam_gpio_req_tbl);
ERROR2:
	kfree(gconf);
ERROR1:
	kfree(fctrl.flashdata);
ERROR0:
	return rc;
}

static int __init msm_led_gpio_add_driver(void)
{
	pr_debug("%s:%d called\n", __func__, __LINE__);
	return platform_driver_probe(&msm_led_gpio_driver, msm_led_gpio_probe);
}

static struct msm_flash_fn_t msm_led_gpio_func_tbl = {
	.flash_get_subdev_id = msm_led_gpio_get_subdev_id,
	.flash_led_config = msm_led_gpio_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_gpio_func_tbl,
};

module_init(msm_led_gpio_add_driver);
MODULE_DESCRIPTION("LED TRIGGER GPIO");
