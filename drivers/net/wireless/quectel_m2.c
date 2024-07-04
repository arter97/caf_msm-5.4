// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved. */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/regulator/consumer.h>
#include <linux/errno.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct quectel_m2 {
	struct device *dev;
	int module_pwrkey;
	int reset_n;
	int rf_disable;
};

static int quectel_m2_poweron(struct quectel_m2 *qm, int on)
{
	if (!qm)
		return -EINVAL;

	if (on) {
		if (gpio_is_valid(qm->module_pwrkey)) {
			printk("quectel_m2 test --- %s --- %d\n", __func__, __LINE__);
			gpio_set_value(qm->module_pwrkey, 0);
		}
	} else if (!on) {
		if (gpio_is_valid(qm->module_pwrkey)) {
			printk("quectel_m2 test --- %s --- %d\n", __func__, __LINE__);
			gpio_set_value(qm->module_pwrkey, 1);
		}
	}

	return 0;
}

static int quectel_m2_parse_dt(struct device *dev, struct quectel_m2 *qm)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	pr_info("%s:ok\n", __func__);
	if (of_get_property(np, "module_pwrkey", NULL)) {
		qm->module_pwrkey = of_get_named_gpio(np, "module_pwrkey", 0);
		if (!gpio_is_valid(qm->module_pwrkey)) {
			dev_err(dev, "get module_pwrkey failed\n");
			goto err_gpio;
		}

		ret = gpio_request(qm->module_pwrkey, "module_pwrkey");
		if (ret) {
			dev_err(dev, "request module_pwrkey failed\n");
			goto err_gpio;
		}

		ret = gpio_direction_output(qm->module_pwrkey, 1);
		if (ret) {
			dev_err(dev, "set direction module_pwrkey failed\n");
			goto err_gpio;
		}
	}

	if (of_get_property(np, "rf_disable", NULL)) {
		qm->rf_disable = of_get_named_gpio(np, "rf_disable", 0);
		if (!gpio_is_valid(qm->rf_disable)) {
			dev_err(dev, "get rf_disable failed\n");
			goto err_gpio;
		}

		ret = gpio_request(qm->rf_disable, "rf_disable");
		if (ret) {
			dev_err(dev, "request rf_disable failed\n");
			goto err_gpio;
		}

		ret = gpio_direction_output(qm->rf_disable, 0);
		if (ret) {
			dev_err(dev, "set direction rf_disable failed\n");
			goto err_gpio;
		}
	}

	if (of_get_property(np, "module_reset", NULL)) {
		qm->reset_n = of_get_named_gpio(np, "module_reset", 0);
		if (!gpio_is_valid(qm->reset_n)) {
			dev_err(dev, "get module_reset failed\n");
			goto err_gpio;
		}

		ret = gpio_request(qm->reset_n, "module_reset");
		if (ret) {
			dev_err(dev, "request module_reset failed\n");
			goto err_gpio;
		}

		ret = gpio_direction_output(qm->reset_n, 0);
		if (ret) {
			dev_err(dev, "set direction module_reset failed\n");
			goto err_gpio;
		}
	}

	return 0;

err_gpio:
	gpio_free(qm->module_pwrkey);
	gpio_free(qm->rf_disable);
	gpio_free(qm->reset_n);
	return ret;
}

static struct of_device_id quectel_m2_of_match[] = {
	{ .compatible = "ext,quectel_m2", },
	{}
};

static int quectel_m2_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct quectel_m2 *qm;

	qm = devm_kzalloc(&pdev->dev, sizeof(struct quectel_m2),
						GFP_KERNEL);
	if (IS_ERR(qm)) {
		dev_err(&pdev->dev, "no memory\n");
		return PTR_ERR(qm);
	}

	if (pdev->dev.of_node) {
		ret = quectel_m2_parse_dt(&pdev->dev, qm);
		if (ret)
			goto err_parse_dt;
	}

	quectel_m2_poweron(qm, 1);
	msleep(70);
	gpio_set_value(qm->reset_n, 1);
	msleep(250);
	gpio_set_value(qm->reset_n, 0);
	quectel_m2_poweron(qm, 0);
	msleep(500);
	quectel_m2_poweron(qm, 1);

	qm->dev = &pdev->dev;

	platform_set_drvdata(pdev, qm);

	pr_info("%s:ok\n", __func__);
	return 0;

err_parse_dt:
	return ret;
}

static int quectel_m2_remove(struct platform_device *pdev)
{
	struct quectel_m2 *qm = platform_get_drvdata(pdev);
	if (!qm)
		return 0;
	quectel_m2_poweron(qm, 0);
	gpio_free(qm->module_pwrkey);
	gpio_free(qm->rf_disable);
	gpio_free(qm->reset_n);
	pr_info("%s:ok\n", __func__);

	return 0;
}


static struct platform_driver quectel_m2_driver = {
	.probe	= quectel_m2_probe,
	.remove	= quectel_m2_remove,
	.driver	= {
		.name		= "quectel_m2",
		.of_match_table	= quectel_m2_of_match,
	},
};

static int __init quectel_m2_init(void)
{
	return platform_driver_register(&quectel_m2_driver);
}

static void __exit quectel_m2_exit(void)
{
	platform_driver_unregister(&quectel_m2_driver);
}

module_init(quectel_m2_init);
module_exit(quectel_m2_exit);

