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
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "board-9615.h"

#define GPIO_WLAN_RESET_N		84

static int msm9615_wlan_probe(struct platform_device *pdev)
{
	int rc = 0;

	rc = gpio_request(GPIO_WLAN_RESET_N, "wlan_reset_n");
	if (rc) {
		pr_err("%s: wlan_reset_n gpio_request failed: %d\n",
			__func__, rc);
		goto out;
	}

	rc = gpio_direction_output(GPIO_WLAN_RESET_N, 1);
	if (rc) {
		pr_err("%s: wlan_reset_n gpio_direction_output failed: %d\n",
			__func__, rc);
		gpio_free(GPIO_WLAN_RESET_N);
		goto out;
	}

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
	gpio_free(GPIO_WLAN_RESET_N);

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
