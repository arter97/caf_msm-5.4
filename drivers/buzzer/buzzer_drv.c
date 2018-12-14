/*
 *  buzzer_drv.c
 *
 *  GPIO driven buzzer driver
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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/hrtimer.h>
#include <linux/buzzer.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>

#include "buzzer_priv.h"

#define DEVICE_NAME "buzzer"
#define DRIVER_NAME "buzzer"

/* the buzzer char device and its major */
static struct cdev  buzzer_dev;
static unsigned buz_major;

/* memory used for read/write */
#define BUF_LEN         (32)
static char             buz_mem[BUF_LEN];

static struct class *buz_class;
static struct device *buz_device;

static struct kobject *buz_kobj;
static struct hrtimer *buz_timer;
static struct buzzer_platform_data *buz_platdata;

static unsigned buz_on_ns;
static unsigned buz_off_ns;
static ktime_t  buz_on_ktime;
static ktime_t  buz_off_ktime;
static bool     buz_status;

static ssize_t show_freq(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_freq(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);
static ssize_t show_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);
static ssize_t show_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_status(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);
static ssize_t show_gpio_buzzer(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_gpio_buzzer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count);



static struct kobj_attribute freq_attribute =
__ATTR(freq, 0664, show_freq, store_freq);

static struct kobj_attribute duty_cycle_attribute =
__ATTR(dutycycle, 0664, show_duty_cycle, store_duty_cycle);

static struct kobj_attribute status_attribute =
__ATTR(status, 0664, show_status, store_status);

static struct kobj_attribute gpio_buzzer_attribute =
__ATTR(gpio_buzzer, 0664, show_gpio_buzzer, store_gpio_buzzer);


static struct attribute *buz_attrs[] = {
    &freq_attribute.attr,
    &duty_cycle_attribute.attr,
    &status_attribute.attr,
    &gpio_buzzer_attribute.attr,
    NULL
};

static struct attribute_group buz_attr_grp = {
    .attrs = buz_attrs,
};

static struct buzzer_config {
    unsigned freq;
    unsigned duty_cycle;
};

static struct buzzer_config buz_cfg = {
    .freq = 2700,
    .duty_cycle = 50,
};

static enum hrtimer_restart buzzer_hrtimer_handler(struct hrtimer *timer)
{
	//int gpio_val = gpio_get_value(buz_platdata->gpio);
	//gpio_val ^= 1;
    ktime_t buzz_delay;
    static bool gpio_val = 1;
    gpio_val ^= 1;
    gpio_set_value(buz_platdata->gpio, gpio_val);

    if (gpio_val){
        buzz_delay = ns_to_ktime(buz_on_ns);
    } else {
        buzz_delay = ns_to_ktime(buz_off_ns);
    }

    hrtimer_forward_now(buz_timer, buzz_delay);

    return HRTIMER_RESTART;
}

static ssize_t show_freq(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", buz_cfg.freq);
}

static ssize_t store_freq(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
    unsigned freq;

    freq = simple_strtoul(buf, NULL, 10);
    if((freq >= 200) && (freq <= 20000)) {
        buz_cfg.freq = freq;

        buz_on_ns = (1000000000/buz_cfg.freq) * buz_cfg.duty_cycle / 100;
        buz_off_ns = (1000000000/buz_cfg.freq) - buz_on_ns;
        buz_on_ktime = ns_to_ktime(buz_on_ns);
        buz_off_ktime = ns_to_ktime(buz_off_ns);
    }

	return strlen(buf);
}

static ssize_t show_gpio_buzzer(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", buz_platdata->gpio);
}

static ssize_t store_gpio_buzzer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
	unsigned gpio_buz;
	int ret;

	gpio_buz = simple_strtoul(buf, NULL, 10);
	buz_platdata->gpio = gpio_buz;
	ret = gpio_request(buz_platdata->gpio, "gpio toggling  for  driving buzzer ");
	if(ret)
		return 0;

	gpio_direction_output(buz_platdata->gpio, 0);
	return strlen(buf);

}



static ssize_t show_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", buz_cfg.duty_cycle);
}

static ssize_t store_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
    unsigned duty_cycle;

    duty_cycle = simple_strtoul(buf, NULL, 10);
    if((duty_cycle > 0) && (duty_cycle < 100)) {
        buz_cfg.duty_cycle = duty_cycle;

        buz_on_ns = (1000000000/buz_cfg.freq) * buz_cfg.duty_cycle / 100;
        buz_off_ns = (1000000000/buz_cfg.freq) - buz_on_ns;
        buz_on_ktime = ns_to_ktime(buz_on_ns);
        buz_off_ktime = ns_to_ktime(buz_off_ns);
    }

	return strlen(buf);
}
static ssize_t show_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return (buz_status == 1) ? sprintf(buf, "on\n") : sprintf(buf, "off\n");
}

static ssize_t store_status(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,  size_t count)
{
	if(!strncmp(buf, "on", 2)) {
        buz_status = 1;
        hrtimer_start(buz_timer, buz_on_ktime, HRTIMER_MODE_REL);
        return count;
	}
	else if (!strncmp(buf, "off", 3)) {
        buz_status = 0;
        hrtimer_cancel(buz_timer);
	gpio_set_value(buz_platdata->gpio, 0);
        return count;
	} else {
        return -EINVAL;
	}
}
static int buzzer_setup_hrtimer(void)
{
    buz_timer = kzalloc(sizeof(struct hrtimer), GFP_KERNEL);
    if(buz_timer == NULL)
        return -1;

    hrtimer_init(buz_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    buz_timer->function = buzzer_hrtimer_handler;

    return 0;
}

static int buzzer_open(struct inode *inode, struct file *filp) {
    unsigned int major, minor;

    major = imajor(inode);
    minor = iminor(inode);

    printk("%s, major = %d, minor = %d!\n", __func__, major, minor);
    return 0;
}

static ssize_t buzzer_read(struct file *filp, char *buf, size_t len, loff_t *offset) {
    printk("%s, len = %d\n", __func__, len);

    if (filp == NULL || buf == NULL || len == 0) {
        printk("Params error!\n");
        return -1;
    }

    if (len > BUF_LEN) {
        len = BUF_LEN;
    }

    copy_to_user(buf, buz_mem, len);

    return len;
}

static ssize_t buzzer_write(struct file *filp, const char *buf, size_t len, loff_t *offset)
{
    printk("%s, len = %d\n", __func__, len);

    if (filp == NULL || buf == NULL || len == 0) {
        printk("Params error!\n");
        return -1;
    }

    if (len > BUF_LEN) {
        len = BUF_LEN;
    }

    /* skip the offset variable */
    copy_from_user(buz_mem, buf, len);

        /* Thread Synchronization Testing */
    if (strcmp(buf, "start\n") == 0) {
        printk("%s, can also start buzzer this way\n", __func__);
        buz_status = 1;
        hrtimer_start(buz_timer, buz_on_ktime, HRTIMER_MODE_REL);
    }

    if (strcmp(buf, "stop\n") == 0) {
        printk("%s, can also stop buzzer this way\n", __func__);
        buz_status = 0;
        hrtimer_cancel(buz_timer);
	gpio_set_value(buz_platdata->gpio, 0);
    }

    return len;
}

static long buzzer_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int ret;
    unsigned freq, duty_cycle;

    switch (cmd) {
        case BUZZER_FREQ_GET:
        {
            printk("BUZZER_FREQ_GET..\n");
            ret = copy_to_user((void __user *)arg,&buz_cfg.freq, sizeof(buz_cfg.freq));
            break;
        }
        case BUZZER_FREQ_SET:
        {
            printk("BUZZER_FREQ_SET..\n");

            ret = copy_from_user(&freq, (void __user *)arg, sizeof(freq));
            if((ret == 0) && (freq >= 200) && (freq <= 20000)) {
                buz_cfg.freq = freq;
                buz_on_ns = (1000000000/buz_cfg.freq) * buz_cfg.duty_cycle / 100;
                buz_off_ns = (1000000000/buz_cfg.freq) - buz_on_ns;
                buz_on_ktime = ns_to_ktime(buz_on_ns);
                buz_off_ktime = ns_to_ktime(buz_off_ns);
            }
            break;
        }
        case BUZZER_DUTYCYCLE_GET:
        {
            printk("BUZZER_DUTYCYCLE_GET..\n");
            ret = copy_to_user((void __user *)arg,&buz_cfg.duty_cycle, sizeof(buz_cfg.duty_cycle));
            break;
        }
        case BUZZER_DUTYCYCLE_SET:
        {
            printk("BUZZER_DUTYCYCLE_SET..\n");

            ret = copy_from_user(&duty_cycle, (void __user *)arg, sizeof(duty_cycle));
            if((ret == 0) && (duty_cycle > 0) && (duty_cycle < 100)) {
                buz_cfg.duty_cycle = duty_cycle;
                buz_on_ns = (1000000000/buz_cfg.freq) * buz_cfg.duty_cycle / 100;
                buz_off_ns = (1000000000/buz_cfg.freq) - buz_on_ns;
                buz_on_ktime = ns_to_ktime(buz_on_ns);
                buz_off_ktime = ns_to_ktime(buz_off_ns);
            }

            break;
        }
        case BUZZER_START:
        {
            printk("BUZZER_START..\n");
            buz_status = 1;
            hrtimer_start(buz_timer, buz_on_ktime, HRTIMER_MODE_REL);
            break;
        }
        case BUZZER_STOP:
        {
            printk("BUZZER_STOP..\n");
            buz_status = 0;
            hrtimer_cancel(buz_timer);
            break;
        }
    }
    return 0;
}

static int buzzer_close(struct inode *inode, struct file *filp)
{
    printk("%s\n", __func__);
    return 0;
}

static struct file_operations buzzer_fops = {
    .open   = buzzer_open,
    .read   = buzzer_read,
    .write  = buzzer_write,
    .unlocked_ioctl = buzzer_ioctl,
    .release = buzzer_close,
};

static int buzzer_probe(struct platform_device *pdev)
{
    int     ret = 0;

    buz_major = register_chrdev(0, "buzzer", &buzzer_fops);
    if ( buz_major < 0 ) {
        printk("register_chrdev(buzzer): FAIL\n");
    } else {
        printk("register_chrdev(buzzer): OK(major = %d)\n", buz_major);
    }
    buz_class = class_create(THIS_MODULE, "buzzer");
    if(IS_ERR(buz_class)){
        printk("buzzer class create: FAIL\n");
        return PTR_ERR(buz_class);
    } else {
        printk("buzzer class create: OK\n");
    }

    buz_device = device_create(buz_class, NULL, MKDEV(buz_major, 0), NULL, "buzzer");
    if(unlikely(IS_ERR(buz_device))){
        printk("buzzer device create: FAIL\n");
        return PTR_ERR(buz_device);
    } else {
        printk("buzzer device create: OK\n");
    }

    buz_on_ns = (1000000000/buz_cfg.freq) * buz_cfg.duty_cycle / 100;
    buz_off_ns = (1000000000/buz_cfg.freq) - buz_on_ns;
    buz_on_ktime = ns_to_ktime(buz_on_ns);
    buz_off_ktime = ns_to_ktime(buz_off_ns);

    buz_platdata = pdev->dev.platform_data;
    ret = gpio_request(buz_platdata->gpio, "gpio to drive buzzer");
    if(ret)
        return ret;

    gpio_direction_output(buz_platdata->gpio, 0);

    /* High resolution timer setup */
    if(buzzer_setup_hrtimer())
        return -1;

    buz_kobj = kobject_create_and_add(DRIVER_NAME, kernel_kobj);
    if (!buz_kobj)
        return -ENOMEM;

    /* Create the files associated with this kobject */
    ret = sysfs_create_group(buz_kobj, &buz_attr_grp);
    if (ret)
        kobject_put(buz_kobj);

    return ret;
}

static int buzzer_remove(struct platform_device *pdev)
{
    printk("%s!\n", __func__);

    device_destroy(buz_class, MKDEV(buz_major, 0));
    class_destroy(buz_class);
    unregister_chrdev(buz_major, "buzzer");
    return 0;
}

static struct platform_driver buzzer_drv = {
    .driver= {
        .name = DRIVER_NAME,
    },

    .probe  = buzzer_probe,
    .remove = buzzer_remove,
};

static int __init buzzer_drv_init(void)
{
    int ret;

    ret = platform_driver_register(&buzzer_drv);
    if (ret < 0) {
        printk("Can't register buz_drv!\n");
        return ret;
    }

    return 0;
}

static void __exit buzzer_drv_exit(void)
{
    platform_driver_unregister(&buzzer_drv);
    return;
}

module_init(buzzer_drv_init);
module_exit(buzzer_drv_exit);

MODULE_AUTHOR("Leo Han <lhan@gopro.com>");
MODULE_LICENSE("GPL");

