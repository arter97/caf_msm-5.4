/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <net/cnss.h>


static struct cnss_sdio_data {
	struct platform_device *pdev;
} *cnss_pdata;


void cnss_get_monotonic_boottime(struct timespec *ts)
{
	get_monotonic_boottime(ts);
}
EXPORT_SYMBOL(cnss_get_monotonic_boottime);

static int cnss_sdio_probe(struct platform_device *pdev)
{
	cnss_pdata = devm_kzalloc(&pdev->dev, sizeof(*cnss_pdata), GFP_KERNEL);
	if (!cnss_pdata)
		return -ENOMEM;

	cnss_pdata->pdev = pdev;
	dev_dbg(&pdev->dev, "CNSS SDIO Driver registered");
	return 0;
}

static int cnss_sdio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver cnss_sdio_driver = {
	.probe  = cnss_sdio_probe,
	.remove = cnss_sdio_remove,
	.driver = {
		.name = "cnss_sdio",
		.owner = THIS_MODULE,
	},
};

static int __init cnss_sdio_init(void)
{
	return platform_driver_register(&cnss_sdio_driver);
}

static void __exit cnss_sdio_exit(void)
{
	platform_driver_unregister(&cnss_sdio_driver);
}

module_init(cnss_sdio_init);
module_exit(cnss_sdio_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DEVICE "CNSS SDIO Driver");
