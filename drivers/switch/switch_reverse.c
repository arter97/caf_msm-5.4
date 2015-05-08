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

struct reverse_data {
	struct switch_dev *sdev;
	struct input_dev *idev;
	unsigned gpio;
	unsigned int key_code;
	unsigned debounce;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int active_low;
	int irq;
	struct work_struct work;
	struct delayed_work detect_delayed_work;
};

struct reverse_platform_data {
	struct reverse_data *reverse_data[REVERSE_MAX_GPIO];
	struct switch_dev *sdev;
	struct input_dev *idev;
};

static struct reverse_platform_data g_reverse_platform_data = {
		{NULL, NULL},
		NULL,
		NULL
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


static void show_pic_exit(void)
{
	if ((pic_status == PIC_SHOWING)) {
		if (ui_status == SHOWING)
			pic_status = ALL_CLEAR;
		else
			pic_status = SPLASH_LOGO_SHOWING;
	}
}

static int reverse_get_gpio_state(struct reverse_data *data)
{
	int state;

	state = gpio_get_value(data->gpio);

	/* Invert the state if active low*/
	if (data->active_low)
		state = (state == 0) ? 1 : 0;

	return state;
}

static void reverse_set_state(struct reverse_data *data, int state)
{
	switch_set_state(data->sdev, state);

	if (state)
		enable_camera_preview();
	else {
		disable_camera_preview();
		show_pic_exit();
	}

	input_report_key(data->idev, data->key_code, state);
	input_sync(data->idev);
}

static void reverse_detection_work(struct work_struct *work)
{
	int state;
	struct reverse_data *data;

	data = container_of(work, struct reverse_data,
			detect_delayed_work.work);
	state = gpio_get_value(data->gpio);

	/* Invert the state if active low*/
	if (data->active_low)
		state = (state == 0) ? 1 : 0;

	switch_set_state(data->sdev, state);

	if (state)
		enable_camera_preview();
	else {
		disable_camera_preview();
		show_pic_exit();
	}

	input_report_key(data->idev, data->key_code, state);
	input_sync(data->idev);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct reverse_data *data =
		(struct reverse_data *)dev_id;

	schedule_delayed_work(&data->detect_delayed_work,
					msecs_to_jiffies(data->debounce));
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct reverse_data *data = g_reverse_platform_data.reverse_data[0];

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

static int switch_reverse_setup_gpios(struct platform_device *pdev,
		struct reverse_data *reverse_data)
{
	unsigned long irq_flags;
	int ret = 0;
	pr_debug("%s: entry\n", __func__);

	if (!pdev || !reverse_data) {
		pr_err("%s: ERROR null params!", __func__);
		return -EINVAL;
	}

	ret = gpio_request(reverse_data->gpio, pdev->name);
	if (ret < 0)
		return ret;

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

	INIT_DELAYED_WORK(&reverse_data->detect_delayed_work,
						reverse_detection_work);

	pr_debug("%s: kpi exit\n", __func__);
	return 0;

err_free_gpio:
	gpio_free(reverse_data->gpio);

	pr_debug("%s: kpi exit %x\n", __func__, ret);

	return ret;
}

static int switch_reverse_probe(struct platform_device *pdev)
{
	int ret = 0;
	int index = 0;
	int state = 0;
	struct reverse_switch_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s: kpi entry\n", __func__);

	if (!pdata)
		return -EBUSY;

	/* register switch device */
	g_reverse_platform_data.sdev =
			kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	if (!g_reverse_platform_data.sdev) {
		pr_err("%s: failed to alloc for sdev", __func__);
		return -ENOMEM;
	}

	g_reverse_platform_data.sdev->name = pdata->name;
	g_reverse_platform_data.sdev->print_state = switch_gpio_print_state;

	ret = switch_dev_register(g_reverse_platform_data.sdev);
	if (ret < 0) {
		pr_err("%s: Failed to register switch dev\n", __func__);
		goto err_switch_dev_register;
	}

	/* register input device */
	g_reverse_platform_data.idev = input_allocate_device();
	if (!g_reverse_platform_data.idev) {
		pr_err("%s: Failed to allocate input dev\n", __func__);
		ret = -ENOMEM;
		goto err_input_dev_register;
	}

	g_reverse_platform_data.idev->name = pdata->name;
	g_reverse_platform_data.idev->phys = "reverse_keys/input0";
	g_reverse_platform_data.idev->id.bustype = BUS_HOST;
	g_reverse_platform_data.idev->dev.parent = &pdev->dev;
	g_reverse_platform_data.idev->evbit[0] = BIT_MASK(EV_KEY);
	g_reverse_platform_data.idev->keybit[BIT_WORD(pdata->key_code)] =
						BIT_MASK(pdata->key_code);

	ret = input_register_device(g_reverse_platform_data.idev);
	if (ret) {
		pr_err("%s: Can't register input device: %d\n", __func__, ret);
		goto err_input_reg;
	}

	ret = device_create_file(g_reverse_platform_data.sdev->dev,
			&dev_attr_continues);
	if (ret) {
		pr_err("%s: Failed device_create_file: %d\n", __func__, ret);
		goto err_device_create_file;
	}

	for (index = 0; index < REVERSE_MAX_GPIO; index++) {
		pr_debug("%s : setup reverse gpio(%d) index %d",
				__func__, pdata->gpio[index], index);

		g_reverse_platform_data.reverse_data[index] =
				kzalloc(sizeof(struct reverse_data),
						GFP_KERNEL);
		if (!g_reverse_platform_data.reverse_data[index]) {
			pr_err("%s: failed to alloc reverse_data for index %d",
					__func__, index);
			ret = -ENOMEM;
			goto err_setup_gpios;
		}

		g_reverse_platform_data.reverse_data[index]->sdev =
				g_reverse_platform_data.sdev;
		g_reverse_platform_data.reverse_data[index]->idev =
				g_reverse_platform_data.idev;
		g_reverse_platform_data.reverse_data[index]->gpio =
				pdata->gpio[index];
		g_reverse_platform_data.reverse_data[index]->key_code =
				pdata->key_code;
		g_reverse_platform_data.reverse_data[index]->debounce =
				pdata->debounce_time;
		g_reverse_platform_data.reverse_data[index]->name_on =
				pdata->name_on;
		g_reverse_platform_data.reverse_data[index]->name_off =
				pdata->name_off;
		g_reverse_platform_data.reverse_data[index]->state_on =
				pdata->state_on;
		g_reverse_platform_data.reverse_data[index]->state_off =
				pdata->state_off;
		g_reverse_platform_data.reverse_data[index]->active_low =
				pdata->active_low[index];

		if (pdata->gpio[index] == -1) {
			pr_debug("%s: invalid gpio %d for index %d",
					__func__, pdata->gpio[index], index);
			continue;
		}

		ret = switch_reverse_setup_gpios(pdev,
				g_reverse_platform_data.reverse_data[index]);
		if (ret < 0) {
			pr_err("%s: switch_reverse_setup_gpios failed with %d",
					__func__, ret);
			goto err_setup_gpios;
		}

		/* get initial states of gpios */
		state |= reverse_get_gpio_state(
				g_reverse_platform_data.reverse_data[index]);
	}

	/* set initial camera state */
	reverse_set_state(g_reverse_platform_data.reverse_data[0], state);

	/* enable interrupts */
	for (index = 0; index < REVERSE_MAX_GPIO; index++)
		if (g_reverse_platform_data.reverse_data[index]->irq)
			enable_irq(g_reverse_platform_data.
					reverse_data[index]->irq);

	platform_set_drvdata(pdev, &g_reverse_platform_data);

	pr_debug("%s: kpi exit", __func__);

	return 0;

err_setup_gpios:
	for (index = 0; index < REVERSE_MAX_GPIO; index++) {
		if (g_reverse_platform_data.reverse_data[index]) {
			gpio_free(g_reverse_platform_data.
					reverse_data[index]->gpio);
			kfree(g_reverse_platform_data.reverse_data[index]);
		}
	}
err_device_create_file:
	input_unregister_device(g_reverse_platform_data.idev);
err_input_reg:
	input_free_device(g_reverse_platform_data.idev);
err_input_dev_register:
	switch_dev_unregister(g_reverse_platform_data.sdev);
err_switch_dev_register:
	kfree(g_reverse_platform_data.sdev);

	pr_debug("%s: kpi exit %x", __func__, ret);

	return ret;
}


static int __devexit switch_reverse_remove(struct platform_device *pdev)
{
	int index = 0;
	struct reverse_platform_data *reverse_platform_data =
			platform_get_drvdata(pdev);

	disable_camera_preview();

	for (index = 0;
			index < REVERSE_MAX_GPIO &&
			reverse_platform_data->reverse_data[index];
			index++) {
		cancel_delayed_work_sync(&reverse_platform_data->
				reverse_data[index]->detect_delayed_work);
		gpio_free(reverse_platform_data->reverse_data[index]->gpio);
		kfree(reverse_platform_data->reverse_data[index]);
	}

	input_unregister_device(reverse_platform_data->idev);
	switch_dev_unregister(reverse_platform_data->sdev);
	kfree(reverse_platform_data->sdev);

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
