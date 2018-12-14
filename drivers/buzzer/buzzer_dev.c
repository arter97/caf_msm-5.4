/*
 *  buzzer_dev.c
 *
 *  GPIO driven buzzer device
 *
 *  Author: Leo Han <lhan@gopro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "buzzer_priv.h"

#define DEVICE_NAME "buzzer"

/* in tas buzzer gpio d4 value is gpio#66 */
#define BUZZER_GPIO 66

static struct buzzer_platform_data buz_platdata = {
    .gpio = BUZZER_GPIO,
};

static struct platform_device buzzer_dev = {
    .name = DEVICE_NAME,
    .id = -1,
    .dev = {
        .platform_data = &buz_platdata,
    },
};

static int __init buzzer_dev_init(void) {
    int ret;

    ret = platform_device_register(&buzzer_dev);
    if (ret < 0) {
        printk("Can't register buzzer_dev!\n");
        return ret;
    }

    return 0;
}

static void __exit buzzer_dev_exit(void) {
    platform_device_unregister(&buzzer_dev);
    return;
}

module_init(buzzer_dev_init);
module_exit(buzzer_dev_exit);

MODULE_AUTHOR("Leo Han <lhan@gopro.com>");
MODULE_LICENSE("GPL");

