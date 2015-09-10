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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "board-9615.h"

#define GPIO_WLAN_RESET_N		21
#define GPIO_WLAN_PM_ENABLE		89

static struct gpio wlan_gpios[] = {
	{ GPIO_WLAN_RESET_N, GPIOF_OUT_INIT_LOW, "wlan_reset_n" },
	{ GPIO_WLAN_PM_ENABLE, GPIOF_OUT_INIT_LOW, "wlan_pm_enable" },
};

struct wlan_regulator {
	const char *vreg_name; /* Regulator Name */
	int min_uV; /* Minimum voltage at which QCA6174 can operate */
	int max_uV; /* Maximum voltage at which AR6003 can operate */
	int load_uA; /* Current which will be drawn from regulator */
	int delay_mT; /* Time from this operation to next */
	struct regulator *vreg; /* Regulator Handle */
};

static struct wlan_regulator wlan_regulators[] = {
	{"wlan_vreg", 1710000, 1890000, 86000, 5, NULL}
};

static void msm9615_wlan_power_down(struct wlan_regulator *regulators,
		size_t size)
{
	int rc = 0;
	int i;

	for (i = size - 1; i >= 0; i--) {

		if (!regulators[i].vreg) {
			pr_err("%s: vreg %s is NULL!\n", __func__,
					regulators[i].vreg_name);
			continue;
		}

		rc = regulator_disable(regulators[i].vreg);
		if (rc)
			pr_err("Failed to disable regulator: %s, rc: %d\n",
					regulators[i].vreg_name, rc);

		regulator_put(regulators[i].vreg);

		regulators[i].vreg = NULL;
	}
}

static int msm9615_wlan_power_up(struct platform_device *pdev,
		struct wlan_regulator *regulators, size_t num)
{
	int rc = 0;
	int i;

	for (i = 0; i < num; i++) {
		regulators[i].vreg = regulator_get(&pdev->dev,
				regulators[i].vreg_name);

		if (IS_ERR_OR_NULL(regulators[i].vreg)) {
			rc = PTR_ERR(regulators[i].vreg);
			pr_err("Failed to get regulator: %s, rc: %d\n",
					regulators[i].vreg_name, rc);
			goto out;
		}

		rc = regulator_set_voltage(regulators[i].vreg,
				regulators[i].min_uV, regulators[i].max_uV);

		if (rc) {
			pr_err("Failed to set regulator voltage: %s, rc: %d\n",
					regulators[i].vreg_name, rc);
			goto out;
		}

		rc = regulator_set_optimum_mode(regulators[i].vreg,
				regulators[i].load_uA);
		if (rc < 0) {
			pr_err("Failed to set regulator optimum mode: %s, rc: %d\n",
					regulators[i].vreg_name, rc);
			goto out;
		}

		rc = regulator_enable(regulators[i].vreg);
		if (rc) {
			pr_err("Failed to enable regulator: %s, rc: %d\n",
					regulators[i].vreg_name, rc);
			goto out;
		}

		mdelay(regulators[i].delay_mT);
	}

	return 0;
out:
	regulator_put(regulators[i].vreg);
	regulators[i].vreg = NULL;
	while(--i >= 0) {
		regulator_disable(regulators[i].vreg);
		regulator_put(regulators[i].vreg);
		regulators[i].vreg = NULL;
	}

	return rc;
}

static int msm9615_wlan_probe(struct platform_device *pdev)
{
	int rc = 0;

	rc = msm9615_wlan_power_up(pdev, wlan_regulators,
			ARRAY_SIZE(wlan_regulators));

	if (rc) {
		pr_err("%s: wlan power up failed: %d\n", __func__, rc);
		goto out;
	}

	rc = gpio_request_array(wlan_gpios, ARRAY_SIZE(wlan_gpios));
	if (rc) {
		pr_err("%s: gpio_request_array failed: %d\n", __func__, rc);
		goto gpio_request_failed;
	}

	rc = gpio_direction_output(GPIO_WLAN_PM_ENABLE, 1);
	if (rc) {
		pr_err("%s: wlan_pm_enable gpio_direction_output failed: %d\n",
			__func__, rc);
		goto pm_enable_failed;
	}

	mdelay(4);

	rc = gpio_direction_output(GPIO_WLAN_RESET_N, 1);
	if (rc) {
		pr_err("%s: wlan_reset_n gpio_direction_output failed: %d\n",
			__func__, rc);
		gpio_free(GPIO_WLAN_RESET_N);
		goto reset_n_failed;
	}

	return 0;

reset_n_failed:
	gpio_direction_output(GPIO_WLAN_PM_ENABLE, 0);
pm_enable_failed:
	gpio_free_array(wlan_gpios, ARRAY_SIZE(wlan_gpios));
gpio_request_failed:
	msm9615_wlan_power_down(wlan_regulators, ARRAY_SIZE(wlan_regulators));
out:
	return rc;
}

static int msm9615_wlan_remove(struct platform_device *pdev)
{
	int rc = 0;

	rc = gpio_direction_output(GPIO_WLAN_RESET_N, 0);
	if (rc) {
		pr_err("%s: wlan_reset_n gpio_direction_output failed: %d\n",
			__func__, rc);
	}

	rc = gpio_direction_output(GPIO_WLAN_PM_ENABLE, 0);
	if (rc) {
		pr_err("%s: wlan_pm_enable gpio_direction_output failed: %d\n",
			__func__, rc);
	}

	gpio_free_array(wlan_gpios, ARRAY_SIZE(wlan_gpios));

	msm9615_wlan_power_down(wlan_regulators, ARRAY_SIZE(wlan_regulators));

	return 0;
}

static struct platform_driver msm9615_wlan_driver = {
	.probe = msm9615_wlan_probe,
	.remove = msm9615_wlan_remove,
	.driver = {
		.name = "wlan-qcacld-platform",
		.owner = THIS_MODULE,
	},
};

int __init msm9615_init_wlan(void)
{
	return platform_driver_register(&msm9615_wlan_driver);
}

module_init(msm9615_init_wlan);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DEVICE "9615 WLAN Driver");
