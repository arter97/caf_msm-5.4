// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/mount.h>
#include <linux/syscalls.h>

static int gsi_boot;
static struct kobject *gsi_kobj;

static ssize_t gsi_boot_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", gsi_boot);
}

static ssize_t gsi_boot_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf,
		size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	gsi_boot = (int) val;

	return count;
}

static struct kobj_attribute gsi_boot_attr =
	__ATTR(boot, 0664, gsi_boot_show, gsi_boot_store);

static int gsi_sysfs_init(void)
{
	int ret;

	gsi_kobj = kobject_create_and_add("boot_gsi", kernel_kobj);
	if (!gsi_kobj) {
		pr_err("gsi: Could not create kobject\n");
		ret = -ENOMEM;
		goto kobj_err;
	}

	ret = sysfs_create_file(gsi_kobj, &gsi_boot_attr.attr);
	if (ret) {
		pr_err("gsi: Could not create sysfs file\n");
		goto err;
	}

	return 0;
err:
	kobject_del(gsi_kobj);
kobj_err:
	return ret;
}

static int __init early_init(void)
{
	int ret = 0;
	static char init_prog[128] = "/early_services/init_early";
	static char *init_prog_argv[2] = { init_prog, NULL };
	static char *init_envp[] = {
		"HOME=/",
		"TERM=linux",
		"PATH=/sbin:/usr/sbin:/bin:/usr/bin:/system/sbin:/system/bin:"
			"early_services/sbin:early_services/system/bin",
		NULL};

	devtmpfs_mount("dev");

	if (ksys_access((const char __user *) init_prog, 0) == 0) {
		ret = call_usermodehelper(init_prog, init_prog_argv, init_envp, 0);
		if (!ret)
			pr_info("%s launched\n", __func__);
		else
			pr_err("%s failed\n", __func__);
	} else {
		pr_err("%s: %s does not exist\n", __func__, init_prog);
	}

	gsi_sysfs_init();

	return ret;
}

late_initcall(early_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Early user space launch driver");
