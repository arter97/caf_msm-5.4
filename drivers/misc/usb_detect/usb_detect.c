/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

extern void phy_msm_otg_mode(int);
extern void pyh_msm_id_status(int);


static struct delayed_work mode_set_work;

enum usb_control_type {
	USB_NONE = 0,
	USB_PERIPHERAL,
	USB_HOST,
};

enum usb_control_type mode = USB_NONE;

struct priv_data {
	int gpio_id;
	int gpio_switch;
	enum usb_control_type mode;
};

struct priv_data usb_data = {
	.gpio_id = 0,
	.mode = USB_NONE,
};


static irqreturn_t irq_handler(int irq, void *_data)
{
	schedule_delayed_work(&mode_set_work, 2 * HZ);

	return IRQ_HANDLED;
}

static void work_handler(struct work_struct *work)
{
	int value;

	value = gpio_get_value(usb_data.gpio_id);
	if ( (0 == value) && usb_data.mode != USB_HOST) {
		usb_data.mode = USB_HOST;
		pyh_msm_id_status(usb_data.mode);
		msleep(1000);

		phy_msm_otg_mode(0);

		pr_notice("ID value = %d, HOST", value);

	} else if ( (1 == value) && usb_data.mode != USB_PERIPHERAL) {
		usb_data.mode = USB_PERIPHERAL;
		pyh_msm_id_status(usb_data.mode);
		msleep(1000);

		phy_msm_otg_mode(1);

		pr_notice("ID value = %d, PERIPHERAL", value);
	}

}

static const struct of_device_id usb_id_ids[] = {
	{ .compatible = "usb_id,usb_detect"},
	{},
};


static int usb_detect_probe(struct platform_device *dev)
{
	int ret, gpio_id, irq;

	gpio_id = of_get_named_gpio(dev->dev.of_node, "qcom,usb-id", 0);
	if (gpio_id < 0) {
		pr_err("of_get_named_gpio failed: %d" , gpio_id);
		return gpio_id;
	}

	ret = gpio_request(gpio_id, "usb_detect_gpio");
	if (ret) {
		pr_err("gpio_request failed: %d", ret);
		goto ERR1;
	}

	if ( gpio_direction_input(gpio_id) < 0) {

		pr_err("gpio_direction_input failed!");
		goto ERR1;
	}

	INIT_DELAYED_WORK(&mode_set_work, work_handler);

	usb_data.gpio_id = gpio_id;

	irq = gpio_to_irq(gpio_id);

	ret = request_irq(irq, irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"usb_detect", &usb_data);
	if (ret < 0) {
		pr_err("request_threaded_irq failed: %d", ret);
		goto ERR1;
	}

	schedule_delayed_work(&mode_set_work, 10 * HZ);

	return ret;

ERR1:

	gpio_free(gpio_id);

	return ret;
}

static int usb_detect_remove(struct platform_device *dev)
{

	return 0;
}

static struct platform_driver usb_detect_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "usb_detect_driver",
		.of_match_table	= usb_id_ids,
	},

	.probe			= usb_detect_probe,
	.remove 		= usb_detect_remove,
};
module_platform_driver(usb_detect_driver);

MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, usb_id_ids);

