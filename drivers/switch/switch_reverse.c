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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/reverse.h>
#include <linux/input.h>

struct reverse_reverse_data {
	struct switch_dev sdev;
	struct input_dev *idev;
	unsigned gpio;
	unsigned int key_code;
	unsigned debounce;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	struct delayed_work detect_delayed_work;
};

static int reverse_continue;

enum pic_status {
	PIC_SHOWING,
	SPLASH_LOGO_SHOWING,
	ALL_CLEAR,
};

static enum pic_status pic_status = ALL_CLEAR;

enum user_interface_status {
	SHOWING,
	NOT_SHOWING,
};

enum user_interface_status ui_status = NOT_SHOWING;

enum camera_states {
	CAMERA_POWERED_DOWN = 0,
	CAMERA_POWERED_UP,
	CAMERA_PREVIEW_ENABLED,
	CAMERA_PREVIEW_DISABLED,
	CAMERA_UNKNOWN
};
static enum camera_states camera_status = CAMERA_POWERED_DOWN;


static void show_pic_exit(void)
{
	if ((pic_status == PIC_SHOWING)) {
		if (ui_status == SHOWING)
			pic_status = ALL_CLEAR;
		else
			pic_status = SPLASH_LOGO_SHOWING;
	}
}

static void reverse_detection_work(struct work_struct *work)
{
	int state;
	struct reverse_reverse_data *data;

	data = container_of(work, struct reverse_reverse_data,
			detect_delayed_work.work);
	state = gpio_get_value(data->gpio);

	switch_set_state(&data->sdev, !state);

	if (!state && (camera_status == CAMERA_POWERED_UP
				|| camera_status == CAMERA_PREVIEW_DISABLED)) {
		if (enable_camera_preview() == 0)
			camera_status = CAMERA_PREVIEW_ENABLED;
	} else {
		if (camera_status == CAMERA_PREVIEW_ENABLED) {
			if (disable_camera_preview() == 0)
				camera_status = CAMERA_PREVIEW_DISABLED;
		}
		show_pic_exit();
	}

	input_report_key(data->idev, data->key_code, !state);
	input_sync(data->idev);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct reverse_reverse_data *data =
		(struct reverse_reverse_data *)dev_id;

	schedule_delayed_work(&data->detect_delayed_work,
					msecs_to_jiffies(data->debounce));
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct reverse_reverse_data	*data =
		container_of(sdev, struct reverse_reverse_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = data->state_on;
	else
		state = data->state_off;

	if (state)
		return snprintf(buf, sizeof(char), "%s\n", state);
	return -EPERM;
}

static ssize_t continues_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", reverse_continue);
}

static ssize_t continues_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;
	int rc;

	if (strcmp(buf, "show") == 0) {
		ui_status = SHOWING;
		if (pic_status == SPLASH_LOGO_SHOWING)
			pic_status = ALL_CLEAR;
	}

	rc = kstrtoul(buf, 0, &val);

	if (rc == 0)
		reverse_continue = val;
	else
		pr_err("Failed to convert the buffer to ulong = %d\n", rc);

	pr_debug("reverse_continue = %d\n", reverse_continue);
	return count;
}

static DEVICE_ATTR(continues, S_IRUGO | S_IWUSR,
		continues_show, continues_store);

static int switch_reverse_probe(struct platform_device *pdev)
{
	struct reverse_switch_platform_data *pdata = pdev->dev.platform_data;
	struct reverse_reverse_data *reverse_data;
	unsigned long irq_flags;
	int ret = 0;
	pr_debug("%s: kpi entry\n", __func__);

	if (!pdata)
		return -EBUSY;

	reverse_data = kzalloc(sizeof(struct reverse_reverse_data), GFP_KERNEL);
	if (!reverse_data)
		return -ENOMEM;

	reverse_data->sdev.name = pdata->name;
	reverse_data->gpio = pdata->gpio;
	reverse_data->key_code = pdata->key_code;
	reverse_data->debounce = pdata->debounce_time;
	reverse_data->name_on = pdata->name_on;
	reverse_data->name_off = pdata->name_off;
	reverse_data->state_on = pdata->state_on;
	reverse_data->state_off = pdata->state_off;
	reverse_data->sdev.print_state = switch_gpio_print_state;

	ret = switch_dev_register(&reverse_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	reverse_data->idev = input_allocate_device();
	if (!reverse_data->idev) {
		pr_err("Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto err_input_dev_register;
	}

	reverse_data->idev->name = pdata->name;
	reverse_data->idev->phys = "reverse_keys/input0";
	reverse_data->idev->id.bustype = BUS_HOST;
	reverse_data->idev->dev.parent = &pdev->dev;
	reverse_data->idev->evbit[0] = BIT_MASK(EV_KEY);
	reverse_data->idev->keybit[BIT_WORD(pdata->key_code)] =
						BIT_MASK(pdata->key_code);

	ret = input_register_device(reverse_data->idev);
	if (ret) {
		pr_err("Can't register input device: %d\n", ret);
		goto err_input_reg;
	}

	ret = gpio_request(reverse_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(reverse_data->gpio);
	if (ret < 0)
		goto err_free_gpio;

	reverse_data->irq = gpio_to_irq(reverse_data->gpio);
	if (reverse_data->irq < 0) {
		ret = reverse_data->irq;
		goto err_free_gpio;
	}

	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = request_irq(reverse_data->irq, gpio_irq_handler,
			  irq_flags, pdev->name, reverse_data);
	if (ret < 0)
		goto err_free_gpio;

	disable_irq(reverse_data->irq);

	ret = device_create_file(reverse_data->sdev.dev, &dev_attr_continues);

	INIT_DELAYED_WORK(&reverse_data->detect_delayed_work,
						reverse_detection_work);
	camera_status = CAMERA_POWERED_DOWN;

	pr_debug("%s: init_camera_kthread\n", __func__);

	if (camera_status == CAMERA_POWERED_DOWN) {
		int rc = 0;
		pr_debug("init camera configuration %s\n", __func__);
		rc = init_camera_kthread();
		if (rc < 0)
			pr_err("%s: Failed to init the camera", __func__);
		else
			camera_status = CAMERA_POWERED_UP;
	}

	pr_debug("%s: reverse detection work\n", __func__);

	reverse_detection_work(&reverse_data->detect_delayed_work.work);
	enable_irq(reverse_data->irq);

	pr_debug("%s: kpi exit\n", __func__);
	return 0;

err_free_gpio:
	gpio_free(reverse_data->gpio);
err_request_gpio:
	input_unregister_device(reverse_data->idev);
err_input_reg:
	input_free_device(reverse_data->idev);
err_input_dev_register:
	switch_dev_unregister(&reverse_data->sdev);
err_switch_dev_register:
	kfree(reverse_data);

	return ret;
}

static int __devexit switch_reverse_remove(struct platform_device *pdev)
{
	struct reverse_reverse_data *reverse_data = platform_get_drvdata(pdev);

	if ((camera_status == CAMERA_POWERED_UP
				|| camera_status == CAMERA_PREVIEW_DISABLED)) {
		exit_camera_kthread();
		camera_status = CAMERA_POWERED_DOWN;
	}

	cancel_delayed_work_sync(&reverse_data->detect_delayed_work);
	gpio_free(reverse_data->gpio);
	input_unregister_device(reverse_data->idev);
	switch_dev_unregister(&reverse_data->sdev);
	kfree(reverse_data);

	return 0;
}

static struct platform_driver switch_reverse_driver = {
	.probe		= switch_reverse_probe,
	.remove		= __devexit_p(switch_reverse_remove),
	.driver		= {
		.name	= "switch-reverse",
		.owner	= THIS_MODULE,
	},
};

static int __init switch_reverse_init(void)
{
	return platform_driver_register(&switch_reverse_driver);
}

static void __exit switch_reverse_exit(void)
{
	platform_driver_unregister(&switch_reverse_driver);
}

module_init(switch_reverse_init);
module_exit(switch_reverse_exit);

MODULE_DESCRIPTION("Switch Reverse driver");
MODULE_LICENSE("GPL v2");
