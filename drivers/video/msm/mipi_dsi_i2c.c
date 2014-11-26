/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <mach/gpio.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_i2c.h"

struct mipi_dsi_i2c_resource dsi_i2c_rsc;
EXPORT_SYMBOL(dsi_i2c_rsc);

struct mipi_dsi_i2c_configure dsi_i2c_cfg;
EXPORT_SYMBOL(dsi_i2c_cfg);

/*
 * If i2c read or write fails, wait for 100ms to try again, and try
 * max 3 times.
 */
#define MAX_WAIT_TIME (100)
#define MAX_RW_TRIES (3)

static int mipi_dsi_i2c_read(u8 addr, u8 reg, u8 *buf, u8 len)
{
	int ret = 0, i = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = dsi_i2c_rsc.client;

	if (!client) {
		pr_err("no mipi dsi i2c client\n");
		ret = -ENODEV;
		goto r_err;
	}

	client->addr = addr;

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2) {
			ret = 0;
			goto r_err;
		}
		msleep(MAX_WAIT_TIME);
	} while (++i < MAX_RW_TRIES);

	ret = -EIO;
	pr_err("i2c read failed after %d tries\n", MAX_RW_TRIES);

r_err:
	return ret;
}

int mipi_dsi_i2c_read_byte(u8 addr, u8 reg, u8 *buf)
{
	return mipi_dsi_i2c_read(addr, reg, buf, 1);
}
EXPORT_SYMBOL(mipi_dsi_i2c_read_byte);

int mipi_dsi_i2c_write_byte(u8 addr, u8 reg, u8 val)
{
	int ret = 0, i = 0;
	u8 buf[2] = {reg, val};
	struct i2c_msg msg[1];
	struct i2c_client *client = dsi_i2c_rsc.client;

	if (!client) {
		pr_err("no mipi dsi i2c client\n");
		ret = -ENODEV;
		goto w_err;
	}

	client->addr = addr;

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	do {
		if (i2c_transfer(client->adapter, msg, 1) >= 1) {
			ret = 0;
			goto w_err;
		}
		msleep(MAX_WAIT_TIME);
	} while (++i < MAX_RW_TRIES);

	ret = -EIO;
	pr_err("i2c write failed after %d tries\n", MAX_RW_TRIES);

w_err:
	return ret;
}
EXPORT_SYMBOL(mipi_dsi_i2c_write_byte);

static struct i2c_device_id mipi_dsi_i2c_id[] = {
	{ "mipi_dsi_i2c", 0 },
	{ }
};

static int __devinit mipi_dsi_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	const struct mipi_dsi_i2c_platform_data *pdata;
	int ret = 0;

	dsi_i2c_rsc.client = client;

	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -ENODEV;
		goto p_err;
	}

	ret = gpio_request(pdata->pd_gpio, "mipi_dsi_i2c_pd");
	if (ret) {
		pr_err("%s: gpio request %d failed\n", __func__,
			pdata->pd_gpio);
		dsi_i2c_rsc.gpio = -1;
		ret = -ENODEV;
		goto p_err;
	}

	dsi_i2c_rsc.gpio = pdata->pd_gpio;

p_err:
	return ret;
}

static int __devexit mipi_dsi_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver mipi_dsi_i2c_driver = {
	.driver = {
		.name = "mipi_dsi_i2c",
		.owner = THIS_MODULE,
	},
	.probe = mipi_dsi_i2c_probe,
	.remove = mipi_dsi_i2c_remove,
	.id_table = mipi_dsi_i2c_id,
};

static int mipi_dsi_i2c_lcd_on(struct platform_device *pdev)
{
	struct mipi_dsi_i2c_configure *cfg = &dsi_i2c_cfg;

	if (cfg->config_dsi)
		if (cfg->config_dsi())
			pr_err("mipi dsi i2c config dsi fails!\n");

	if (cfg->config_reset_dev)
		cfg->config_reset_dev();

	if (cfg->config_i2c)
		if (cfg->config_i2c())
			pr_err("mipi dsi i2c config i2c fails!\n");

	return 0;
}

static int mipi_dsi_i2c_lcd_off(struct platform_device *pdev)
{
	return 0;
}

static int mipi_dsi_i2c_lcd_late_init(struct platform_device *pdev)
{
	return 0;
}

static int mipi_dsi_i2c_lcd_early_off(struct platform_device *pdev)
{
	return 0;
}

static void mipi_dsi_i2c_set_backlight(struct msm_fb_data_type *mfd)
{
	return;
}

static struct msm_fb_panel_data mipi_dsi_i2c_panel_data = {
	.on             = mipi_dsi_i2c_lcd_on,
	.off            = mipi_dsi_i2c_lcd_off,
	.late_init      = mipi_dsi_i2c_lcd_late_init,
	.early_off      = mipi_dsi_i2c_lcd_early_off,
	.set_backlight  = mipi_dsi_i2c_set_backlight,
};

static int mipi_dsi_i2c_config_480p(void)
{
	int i;
	u32 offset = 0x40;
	u32 ln_cfg = 0x300;

	for (i = 0; i < 5; i++) { /* ln_cfg */
		MIPI_OUTP(MIPI_DSI_BASE + ln_cfg + 0x00, 0xc0);
		MIPI_OUTP(MIPI_DSI_BASE + ln_cfg + 0x04, 0x00);
		MIPI_OUTP(MIPI_DSI_BASE + ln_cfg + 0x14, 0x00);
		MIPI_OUTP(MIPI_DSI_BASE + ln_cfg + 0x18, 0x00);
		ln_cfg += offset;
	}

	MIPI_OUTP(MIPI_DSI_BASE + 0x0a8, 0x10000000);
	MIPI_OUTP(MIPI_DSI_BASE + 0x0c8, 0x01);
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);
	MIPI_OUTP(MIPI_DSI_BASE + 0x108, 0x13FF3BE0);
	MIPI_OUTP(MIPI_DSI_BASE + 0x10c, 0x00010002);
	MIPI_OUTP(MIPI_DSI_BASE + 0x118, 0x3f);
	MIPI_OUTP(MIPI_DSI_BASE + 0x11c, 0x3fd);

	MIPI_OUTP(MIPI_DSI_BASE + 0x48c, 0x00);
	MIPI_OUTP(MIPI_DSI_BASE + 0x490, 0x00);
	MIPI_OUTP(MIPI_DSI_BASE + 0x4b0, 0x25);
	MIPI_OUTP(MIPI_DSI_BASE + 0x510, 0x20);
	MIPI_OUTP(MIPI_DSI_BASE + 0x518, 0x01);
	MIPI_OUTP(MIPI_DSI_BASE + 0x51c, 0x00);

	return 0;
}

static void mipi_dsi_i2c_reset_dev(void)
{
	gpio_set_value_cansleep(dsi_i2c_rsc.gpio, 0);
	msleep(100);
	gpio_set_value_cansleep(dsi_i2c_rsc.gpio, 1);
}

static int __devinit mipi_dsi_i2c_lcd_probe(struct platform_device *pdev)
{
	struct mipi_dsi_i2c_configure *cfg = &dsi_i2c_cfg;

	cfg->config_dsi = mipi_dsi_i2c_config_480p;
	cfg->config_reset_dev = mipi_dsi_i2c_reset_dev;

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_dsi_i2c_lcd_probe,
	.driver = {
		.name   = "mipi_dsi_i2c_lcd",
	},
};

int mipi_dsi_i2c_device_register(struct msm_panel_info *pinfo)
{
	struct platform_device *pdev = NULL;
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret)
		goto reg_err;

	pdev = platform_device_alloc("mipi_dsi_i2c_lcd",
		((pinfo->pdest + 1) << 8));
	if (IS_ERR_OR_NULL(pdev)) {
		ret = -ENOMEM;
		goto reg_err;
	}

	mipi_dsi_i2c_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &mipi_dsi_i2c_panel_data,
		sizeof(mipi_dsi_i2c_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto reg_err;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto reg_err;
	}

	return ret;

reg_err:
	platform_device_put(pdev);

	return ret;
}

static int __init mipi_dsi_i2c_msm_init(void)
{
	return i2c_add_driver(&mipi_dsi_i2c_driver);
}

static void __exit mipi_dsi_i2c_msm_exit(void)
{
	i2c_del_driver(&mipi_dsi_i2c_driver);
}

module_init(mipi_dsi_i2c_msm_init);
module_exit(mipi_dsi_i2c_msm_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MIPI DSI VIDEO I2C driver");
