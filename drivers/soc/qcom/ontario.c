/* Qualcomm Technologies, Inc. Ontario Fingerprint driver
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <uapi/linux/ontario.h>
#include <linux/slab.h>
#include <linux/of.h>

#define ONTARIO_DEV "ontario"

static struct class *driver_class;
static dev_t ontario_device_no;
struct cdev cdev;
struct device *class_dev;
struct clk *spiclk;
struct clk *ahbclk;
unsigned spi_freq;
static int clock_state;
static int resume_state;
struct ontario_clk *clk_arg;

static void clocks_on(void)
{
	if (!clock_state) {
		clk_prepare_enable(spiclk);
		clk_set_rate(spiclk, /*25000000*/clk_arg->SpiFrequency);
		clk_prepare_enable(ahbclk);
		clock_state = 1;
	}

}

static void clocks_off(void)
{
	if (clock_state) {
		clk_disable_unprepare(spiclk);
		clk_disable_unprepare(ahbclk);
		clock_state = 0;
	}
}

static int ontario_open(struct inode *inode, struct file *file)
{
	pr_debug(KERN_INFO "ontario_open\n");
	return 0;
}


static int ontario_release(struct inode *inode, struct file *file)
{
	pr_debug(KERN_INFO "ontario_release\n");
	clocks_off();
	return 0;
}

long ontario_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	struct ontario_clk loc_clk;
	int rc;
	rc = copy_from_user((void *) &loc_clk, (void *) arg,
		sizeof(struct ontario_clk));

	pr_debug(KERN_INFO "ontario_ioctl 0x%x\n", cmd);
	switch (loc_clk.action) {
	case ONTARIO_SPI_CLKS_ON:
		clocks_on();
		break;
	case ONTARIO_SPI_CLKS_OFF:
		clocks_off();
		break;
	default:
		pr_err("Invalid IOCTL latest: spiclk 0x%x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations ontario_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ontario_ioctl,
	.open = ontario_open,
	.release = ontario_release
};


static int ontario_probe(struct platform_device *pdev)
{
	int rc = 0;

	pr_debug(KERN_INFO "ontario_probe\n");
	class_dev = &pdev->dev;

	rc = alloc_chrdev_region(&ontario_device_no, 0, 1, ONTARIO_DEV);
	if (rc < 0) {
		pr_err("alloc_chrdev_region failed %d\n", rc);
		return rc;
	}

	driver_class = class_create(THIS_MODULE, ONTARIO_DEV);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		pr_err("class_create failed %d\n", rc);
		unregister_chrdev_region(ontario_device_no, 1);
		return rc;
	}

	class_dev = device_create(driver_class, NULL, ontario_device_no, NULL,
			ONTARIO_DEV);
	if (!class_dev) {
		pr_err("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		unregister_chrdev_region(ontario_device_no, 1);
		class_destroy(driver_class);
		return rc;
	}

	cdev_init(&cdev, &ontario_fops);
	cdev.owner = THIS_MODULE;

	rc = cdev_add(&cdev, MKDEV(MAJOR(ontario_device_no), 0), 1);
	if (rc < 0) {
		pr_err("cdev_add failed %d\n", rc);
		device_destroy(driver_class, ontario_device_no);
		return rc;
	}

	spiclk = clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(spiclk)) {
		rc = -ENXIO;
		pr_err("get spiclk failed\n");
		return rc;
	}

	ahbclk = clk_get(&pdev->dev, "ahb_clk");
	if (IS_ERR(ahbclk)) {
		rc = -ENXIO;
		pr_err("get ahbclk failed\n");
		return rc;
	}

	return rc;
}

static int ontario_remove(struct platform_device *pdev)
{
	pr_debug(KERN_INFO "ontario_remove\n");
	clocks_off();
	return 0;
}

static int ontario_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_debug(KERN_INFO "ontario_suspend\n");
	resume_state = clock_state;
	clocks_off();
	return 0;
}

static int ontario_resume(struct platform_device *pdev)
{
	pr_debug(KERN_INFO "ontario_resume\n");
	if (resume_state) {
		clocks_on();
		resume_state = 0;
	}

	return 0;
}

static struct of_device_id ontario_match[] = {
	{
		.compatible = "qcom,ontario",
	},
	{}
};

static struct platform_driver ontario_plat_driver = {
	.probe = ontario_probe,
	.remove = ontario_remove,
	.suspend = ontario_suspend,
	.resume = ontario_resume,
	.driver = {
		.name = "ontario",
		.owner = THIS_MODULE,
		.of_match_table = ontario_match,
	},
};

static int ontario_init(void)
{
	return platform_driver_register(&ontario_plat_driver);
}

static void ontario_exit(void)
{
	pr_debug(KERN_INFO "ontario_exit\n");
	clocks_on();
	platform_driver_unregister(&ontario_plat_driver);
	unregister_chrdev_region(ontario_device_no, 1);
	class_destroy(driver_class);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Ontario driver");

module_init(ontario_init);
module_exit(ontario_exit);
