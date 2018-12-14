/* Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/qpnp/power-on.h>
#include <linux/of_address.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>

#include <asm/cacheflush.h>
#include <asm/system_misc.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/restart.h>
#include <soc/qcom/watchdog.h>



#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777
#define EMMC_DLOAD_TYPE		0x2

#define SCM_IO_DISABLE_PMIC_ARBITER	1
#define SCM_IO_DEASSERT_PS_HOLD		2
#define SCM_WDOG_DEBUG_BOOT_PART	0x9
#define SCM_DLOAD_MODE			0X10
#define SCM_EDLOAD_MODE			0X01
#define SCM_DLOAD_CMD			0x10

#define MSM_PM_BUF_SIZE    80



#define REBOOT_COLD 0
#define REBOOT_WARM 1
#define REBOOT_HARD 2
#define REBOOT_SOFT 3
#define REBOOT_GPIO 4

#define RESTART_IOCTL_MAGIC            100


//#define RESTART_REBOOT_COLD _IOW(RESTART_IOCTL_MAGIC, 0, char*)

#define RESTART_REBOOT_COLD _IO(RESTART_IOCTL_MAGIC, 0)
#define RESTART_REBOOT_WARM _IOW(RESTART_IOCTL_MAGIC, 1, char*)
#define RESTART_REBOOT_HARD _IOW(RESTART_IOCTL_MAGIC, 2, char*)
#define RESTART_REBOOT_SOFT _IOW(RESTART_IOCTL_MAGIC, 3, char*)
#define RESTART_REBOOT_GPIO _IOW(RESTART_IOCTL_MAGIC, 4, char*)



static int restart_mode;
static void *restart_reason;
static bool scm_pmic_arbiter_disable_supported;
static bool scm_deassert_ps_hold_supported;
/* Download mode master kill-switch */
static void __iomem *msm_ps_hold;
static phys_addr_t tcsr_boot_misc_detect;
static void scm_disable_sdi(void);

static struct class *msm_pm_class;
static struct device *msm_pm_device;
char* msm_pm_buf;
static int major;

static struct kobject *msm_pm_kobject;
static int gp_poweroff;


#ifdef CONFIG_MSM_DLOAD_MODE
/* Runtime could be only changed value once.
* There is no API from TZ to re-enable the registers.
* So the SDI cannot be re-enabled when it already by-passed.
*/
static int download_mode = 1;
#else
static const int download_mode;
#endif

#ifdef CONFIG_MSM_DLOAD_MODE
#define EDL_MODE_PROP "qcom,msm-imem-emergency_download_mode"
#define DL_MODE_PROP "qcom,msm-imem-download_mode"

static int in_panic;
static void *dload_mode_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;
static bool scm_dload_supported;
static struct kobject dload_kobj;
static void *dload_type_addr;

static int dload_set(const char *val, struct kernel_param *kp);
/* interface for exporting attributes */
struct reset_attribute {
	struct attribute        attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);
	size_t (*store)(struct kobject *kobj, struct attribute *attr,
			const char *buf, size_t count);
};
#define to_reset_attr(_attr) \
	container_of(_attr, struct reset_attribute, attr)
#define RESET_ATTR(_name, _mode, _show, _store)	\
	static struct reset_attribute reset_attr_##_name = \
			__ATTR(_name, _mode, _show, _store)

module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

int scm_set_dload_mode(int arg1, int arg2)
{
	struct scm_desc desc = {
		.args[0] = arg1,
		.args[1] = arg2,
		.arginfo = SCM_ARGS(2),
	};

	if (!scm_dload_supported) {
		if (tcsr_boot_misc_detect)
			return scm_io_write(tcsr_boot_misc_detect, arg1);

		return 0;
	}

	if (!is_scm_armv8())
		return scm_call_atomic2(SCM_SVC_BOOT, SCM_DLOAD_CMD, arg1,
					arg2);

	return scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT, SCM_DLOAD_CMD),
				&desc);
}

static void set_dload_mode(int on)
{
	int ret;

	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
	}

	ret = scm_set_dload_mode(on ? SCM_DLOAD_MODE : 0, 0);
	if (ret)
		pr_err("Failed to set secure DLOAD mode: %d\n", ret);

	dload_mode_enabled = on;
}

static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}

static void enable_emergency_dload_mode(void)
{
	int ret;

	if (emergency_dload_mode_addr) {
		__raw_writel(EMERGENCY_DLOAD_MAGIC1,
				emergency_dload_mode_addr);
		__raw_writel(EMERGENCY_DLOAD_MAGIC2,
				emergency_dload_mode_addr +
				sizeof(unsigned int));
		__raw_writel(EMERGENCY_DLOAD_MAGIC3,
				emergency_dload_mode_addr +
				(2 * sizeof(unsigned int)));

		/* Need disable the pmic wdt, then the emergency dload mode
		 * will not auto reset. */
		qpnp_pon_wd_config(0);
		mb();
	}

	ret = scm_set_dload_mode(SCM_EDLOAD_MODE, 0);
	if (ret)
		pr_err("Failed to set secure EDLOAD mode: %d\n", ret);
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
static void set_dload_mode(int on)
{
	return;
}

static void enable_emergency_dload_mode(void)
{
	pr_err("dload mode is not enabled on target\n");
}

static bool get_dload_mode(void)
{
	return false;
}
#endif

static void scm_disable_sdi(void)
{
	int ret;
	struct scm_desc desc = {
		.args[0] = 1,
		.args[1] = 0,
		.arginfo = SCM_ARGS(2),
	};

	/* Needed to bypass debug image on some chips */
	if (!is_scm_armv8())
		ret = scm_call_atomic2(SCM_SVC_BOOT,
			       SCM_WDOG_DEBUG_BOOT_PART, 1, 0);
	else
		ret = scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_BOOT,
			  SCM_WDOG_DEBUG_BOOT_PART), &desc);
	if (ret)
		pr_err("Failed to disable secure wdog debug: %d\n", ret);
}

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

/*
 * Force the SPMI PMIC arbiter to shutdown so that no more SPMI transactions
 * are sent from the MSM to the PMIC.  This is required in order to avoid an
 * SPMI lockup on certain PMIC chips if PS_HOLD is lowered in the middle of
 * an SPMI transaction.
 */
static void halt_spmi_pmic_arbiter(void)
{
	struct scm_desc desc = {
		.args[0] = 0,
		.arginfo = SCM_ARGS(1),
	};

	if (scm_pmic_arbiter_disable_supported) {
		pr_crit("Calling SCM to disable SPMI PMIC arbiter\n");
		if (!is_scm_armv8())
			scm_call_atomic1(SCM_SVC_PWR,
					 SCM_IO_DISABLE_PMIC_ARBITER, 0);
		else
			scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_PWR,
				  SCM_IO_DISABLE_PMIC_ARBITER), &desc);
	}
}

static void msm_restart_prepare(const char *cmd)
{
	bool need_warm_reset = false;

#ifdef CONFIG_MSM_DLOAD_MODE

	/* Write download mode flags if we're panic'ing
	 * Write download mode flags if restart_mode says so
	 * Kill download mode if master-kill switch is set
	 */

	set_dload_mode(download_mode &&
			(in_panic || restart_mode == RESTART_DLOAD));
#endif

	if (qpnp_pon_check_hard_reset_stored()) {
		/* Set warm reset as true when device is in dload mode */
		if (get_dload_mode() ||
			((cmd != NULL && cmd[0] != '\0') &&
			!strcmp(cmd, "edl")))
			need_warm_reset = true;
	} else {
		need_warm_reset = (get_dload_mode() ||
				((cmd != NULL && cmd[0] != '\0') &&
				strcmp(cmd, "userrequested")));
	}

	/* Hard reset the PMIC unless memory contents must be maintained. */
	if (need_warm_reset) {
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
	} else {
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);
	}

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_BOOTLOADER);
			__raw_writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_RECOVERY);
			__raw_writel(0x77665502, restart_reason);
		} else if (!strcmp(cmd, "rtc")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_RTC);
			__raw_writel(0x77665503, restart_reason);
		} else if (!strcmp(cmd, "dm-verity device corrupted")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_DMVERITY_CORRUPTED);
			__raw_writel(0x77665508, restart_reason);
		} else if (!strcmp(cmd, "dm-verity enforcing")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_DMVERITY_ENFORCE);
			__raw_writel(0x77665509, restart_reason);
		} else if (!strcmp(cmd, "keys clear")) {
			qpnp_pon_set_restart_reason(
				PON_RESTART_REASON_KEYS_CLEAR);
			__raw_writel(0x7766550a, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			int ret;
			ret = kstrtoul(cmd + 4, 16, &code);
			if (!ret)
				__raw_writel(0x6f656d00 | (code & 0xff),
					     restart_reason);
		} else if (!strncmp(cmd, "edl", 3)) {
			enable_emergency_dload_mode();
		} else {
			__raw_writel(0x77665501, restart_reason);
		}
	}

	flush_cache_all();

	/*outer_flush_all is not supported by 64bit kernel*/
#ifndef CONFIG_ARM64
	outer_flush_all();
#endif

}

/*
 * Deassert PS_HOLD to signal the PMIC that we are ready to power down or reset.
 * Do this by calling into the secure environment, if available, or by directly
 * writing to a hardware register.
 *
 * This function should never return.
 */
static void deassert_ps_hold(void)
{
	struct scm_desc desc = {
		.args[0] = 0,
		.arginfo = SCM_ARGS(1),
	};

	if (scm_deassert_ps_hold_supported) {
		/* This call will be available on ARMv8 only */
		scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_PWR,
				 SCM_IO_DEASSERT_PS_HOLD), &desc);
	}

	/* Fall-through to the direct write in case the scm_call "returns" */
	__raw_writel(0, msm_ps_hold);
}

static void do_msm_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	pr_notice("Going down for restart now\n");

	msm_restart_prepare(cmd);

#ifdef CONFIG_MSM_DLOAD_MODE
	/*
	 * Trigger a watchdog bite here and if this fails,
	 * device will take the usual restart path.
	 */

	if (WDOG_BITE_ON_PANIC && in_panic)
		msm_trigger_wdog_bite();
#endif

	scm_disable_sdi();
	halt_spmi_pmic_arbiter();
	deassert_ps_hold();

	mdelay(10000);
}

static void do_msm_poweroff(void)
{
	pr_notice("Powering off the SoC\n");

	set_dload_mode(0);
	scm_disable_sdi();
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	halt_spmi_pmic_arbiter();
	deassert_ps_hold();

	mdelay(10000);
	pr_err("Powering off has failed\n");
	return;
}

#ifdef CONFIG_MSM_DLOAD_MODE
static ssize_t attr_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->show)
		ret = reset_attr->show(kobj, attr, buf);

	return ret;
}

static ssize_t attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct reset_attribute *reset_attr = to_reset_attr(attr);
	ssize_t ret = -EIO;

	if (reset_attr->store)
		ret = reset_attr->store(kobj, attr, buf, count);

	return ret;
}

static const struct sysfs_ops reset_sysfs_ops = {
	.show	= attr_show,
	.store	= attr_store,
};

static struct kobj_type reset_ktype = {
	.sysfs_ops	= &reset_sysfs_ops,
};

static ssize_t show_emmc_dload(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	uint32_t read_val, show_val;

	read_val = __raw_readl(dload_type_addr);
	if (read_val == EMMC_DLOAD_TYPE)
		show_val = 1;
	else
		show_val = 0;

	return snprintf(buf, sizeof(show_val), "%u\n", show_val);
}

static size_t store_emmc_dload(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	uint32_t enabled;
	int ret;

	ret = kstrtouint(buf, 0, &enabled);
	if (ret < 0)
		return ret;

	if (!((enabled == 0) || (enabled == 1)))
		return -EINVAL;

	if (enabled == 1)
		__raw_writel(EMMC_DLOAD_TYPE, dload_type_addr);
	else
		__raw_writel(0, dload_type_addr);

	return count;
}
RESET_ATTR(emmc_dload, 0644, show_emmc_dload, store_emmc_dload);

static struct attribute *reset_attrs[] = {
	&reset_attr_emmc_dload.attr,
	NULL
};

static struct attribute_group reset_attr_group = {
	.attrs = reset_attrs,
};
#endif

static int msm_restart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct device_node *np;
	int ret = 0;

#ifdef CONFIG_MSM_DLOAD_MODE
	if (scm_is_call_available(SCM_SVC_BOOT, SCM_DLOAD_CMD) > 0)
		scm_dload_supported = true;

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	np = of_find_compatible_node(NULL, NULL, DL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem DLOAD mode node\n");
	} else {
		dload_mode_addr = of_iomap(np, 0);
		if (!dload_mode_addr)
			pr_err("unable to map imem DLOAD offset\n");
	}

	np = of_find_compatible_node(NULL, NULL, EDL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem EDLOAD mode node\n");
	} else {
		emergency_dload_mode_addr = of_iomap(np, 0);
		if (!emergency_dload_mode_addr)
			pr_err("unable to map imem EDLOAD mode offset\n");
	}

	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-dload-type");
	if (!np) {
		pr_err("unable to find DT imem dload-type node\n");
		goto skip_sysfs_create;
	} else {
		dload_type_addr = of_iomap(np, 0);
		if (!dload_type_addr) {
			pr_err("unable to map imem dload-type offset\n");
			goto skip_sysfs_create;
		}
	}

	ret = kobject_init_and_add(&dload_kobj, &reset_ktype,
			kernel_kobj, "%s", "dload");
	if (ret) {
		pr_err("%s:Error in creation kobject_add\n", __func__);
		kobject_put(&dload_kobj);
		goto skip_sysfs_create;
	}

	ret = sysfs_create_group(&dload_kobj, &reset_attr_group);
	if (ret) {
		pr_err("%s:Error in creation sysfs_create_group\n", __func__);
		kobject_del(&dload_kobj);
	}
skip_sysfs_create:
#endif
	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_reason");
	if (!np) {
		pr_err("unable to find DT imem restart reason node\n");
	} else {
		restart_reason = of_iomap(np, 0);
		if (!restart_reason) {
			pr_err("unable to map imem restart reason offset\n");
			ret = -ENOMEM;
			goto err_restart_reason;
		}
	}

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pshold-base");
	msm_ps_hold = devm_ioremap_resource(dev, mem);
	if (IS_ERR(msm_ps_hold))
		return PTR_ERR(msm_ps_hold);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "tcsr-boot-misc-detect");
	if (mem)
		tcsr_boot_misc_detect = mem->start;

	pm_power_off = do_msm_poweroff;
	arm_pm_restart = do_msm_restart;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER) > 0)
		scm_pmic_arbiter_disable_supported = true;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DEASSERT_PS_HOLD) > 0)
		scm_deassert_ps_hold_supported = true;

	set_dload_mode(download_mode);
	if (!download_mode)
		scm_disable_sdi();

	return 0;

err_restart_reason:
#ifdef CONFIG_MSM_DLOAD_MODE
	iounmap(emergency_dload_mode_addr);
	iounmap(dload_mode_addr);
#endif
	return ret;
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};

static int __init msm_restart_init(void)
{
	return platform_driver_register(&msm_restart_driver);
}
device_initcall(msm_restart_init);


static ssize_t store_msm_poweroff(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
    pr_debug("Shutdown Command poweroff\n");
    do_msm_poweroff();
    return 1;
}

static ssize_t show_msm_poweroff(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf, size_t count)
{
        pr_debug("Shutdown Command Status\n");
        return sprintf(buf, "%s\n", "shutdown->echo store_msm_poweroff ");
}

static struct kobj_attribute msm_pm_attribute =__ATTR(gp_poweroff, 0660, show_msm_poweroff,
                                                   store_msm_poweroff);


static ssize_t msm_pm_write(struct file *file,
                            const char __user * buffer, size_t length, loff_t * offset)
{
#if 0
    for (i = 0; i < length && i < MSM_PM_BUF_SIZE; i++)
        get_user(msm_pm_buf[i], buffer + i);
#endif
   copy_from_user(msm_pm_buf, buffer, length);
   do_msm_restart(REBOOT_SOFT, msm_pm_buf);
    /*
     * Again, return the number of input characters used
     */
    return 0;
}


static long msm_pm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = -1;
    switch (cmd)
    {
    case RESTART_REBOOT_COLD:
    {
        //copy_from_user(msm_pm_buf,(void __user *)arg, sizeof(msm_pm_buf));

        //do_msm_restart(REBOOT_COLD, msm_pm_buf);
        do_msm_restart(REBOOT_COLD, NULL);
        ret = 0;
        break;
    default:
            break;
        }
    }

    return ret;
}

static struct file_operations msm_pm_ops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = msm_pm_ioctl,
    .write          = msm_pm_write,
};

static int msm_pm_remove(void)
{
    device_destroy(msm_pm_class, MKDEV(major, 0));
    class_destroy(msm_pm_class);
    unregister_chrdev(major, "msm_pm_reboot");
    kfree(msm_pm_buf);
    return 0;
}


static int __init gp_msm_pm_init (void)
{
        int error = 0;

        pr_debug("Module initialized successfully \n");

        msm_pm_kobject = kobject_create_and_add("gp_pmic_ctrl",
                                                 kernel_kobj);
        if(!msm_pm_kobject)
                return -ENOMEM;

        error = sysfs_create_file(msm_pm_kobject, &msm_pm_attribute.attr);
        if (error) {
                pr_debug("failed to create the poweroff file in /sys/kernel/gp_pmic_ctrl\n");
        }

        msm_pm_buf = kmalloc(MSM_PM_BUF_SIZE, GFP_KERNEL);

        major = register_chrdev(0, "msm_pm_reboot", &msm_pm_ops);
        if ( major < 0 ) {
            printk("register_chrdev(msm_pm): FAIL\n");
        } else {
            printk("register_chrdev(msm_pm): OK(major = %d)\n", major);
        }
        msm_pm_class = class_create(THIS_MODULE, "msm_pm_reboot");
        if(IS_ERR(msm_pm_class)){
            printk("msm_pm_class fail: FAIL\n");
            return PTR_ERR(msm_pm_class);
        } else {
            printk("msm pm create: OK\n");
        }


        msm_pm_device = device_create(msm_pm_class, NULL, MKDEV(major,0), NULL, "msm_pm_reboot");
        if(unlikely(IS_ERR(msm_pm_device))){
            printk("msm pm device create: FAIL\n");
            return PTR_ERR(msm_pm_device);
        } else {
            printk("msm pm device create: OK\n");
        }

        return error;
}
static void __exit gp_msm_pm_exit(void)
{
        pr_debug ("Module un initialized successfully \n");
        kobject_put(msm_pm_kobject);
        msm_pm_remove();
}


subsys_initcall(gp_msm_pm_init);

module_exit(gp_msm_pm_exit);
