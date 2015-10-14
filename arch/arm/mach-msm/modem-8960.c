/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/mdm2.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/tty.h>

#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/peripheral-loader.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>
#include <mach/socinfo.h>
#include <mach/msm_smsm.h>
#include "sysmon.h"

#include "smd_private.h"
#include "modem_notifier.h"
#include "ramdump.h"
#include <linux/fdtable.h>
#include "../../../fs/internal.h"
#include <linux/namei.h>

static int crash_shutdown;

static struct subsys_device *modem_8960_dev;

#define MAX_SSR_REASON_LEN 81U

static void log_modem_sfr(void)
{
	u32 size;
	char *smem_reason, reason[MAX_SSR_REASON_LEN];

	smem_reason = smem_get_entry(SMEM_SSR_REASON_MSS0, &size);
	if (!smem_reason || !size) {
		pr_err("modem subsystem failure reason: (unknown, smem_get_entry failed).\n");
		return;
	}
	if (!smem_reason[0]) {
		pr_err("modem subsystem failure reason: (unknown, init string found).\n");
		return;
	}

	size = min(size, MAX_SSR_REASON_LEN-1);
	memcpy(reason, smem_reason, size);
	reason[size] = '\0';
	pr_err("modem subsystem failure reason: %s.\n", reason);

	smem_reason[0] = '\0';
	wmb();
}

static void restart_modem(void)
{
	log_modem_sfr();
	subsystem_restart_dev(modem_8960_dev);
}

#define MODEM_WAIT_BEFORE_OFF  3000

static inline int build_open_flags(int flags, umode_t mode,
				   struct open_flags *op)
{
	int lookup_flags = 0;
	int acc_mode;

	if (!(flags & O_CREAT))
		mode = 0;
	op->mode = mode;
	flags &= ~FMODE_NONOTIFY;
	if (flags & __O_SYNC)
		flags |= O_DSYNC;
	if (flags & O_PATH) {
		flags &= O_DIRECTORY | O_NOFOLLOW | O_PATH;
		acc_mode = 0;
	} else {
		acc_mode = MAY_OPEN | ACC_MODE(flags);
	}
	op->open_flag = flags;
	if (flags & O_TRUNC)
		acc_mode |= MAY_WRITE;
	if (flags & O_APPEND)
		acc_mode |= MAY_APPEND;
	op->acc_mode = acc_mode;
	op->intent = flags & O_PATH ? 0 : LOOKUP_OPEN;
	if (flags & O_CREAT) {
		op->intent |= LOOKUP_CREATE;
		if (flags & O_EXCL)
			op->intent |= LOOKUP_EXCL;
	}
	if (flags & O_DIRECTORY)
		lookup_flags |= LOOKUP_DIRECTORY;
	if (!(flags & O_NOFOLLOW))
		lookup_flags |= LOOKUP_FOLLOW;
	return lookup_flags;
}

static void modem_off(void)
{
	struct file *serial_fd;
	mm_segment_t oldfs;
	unsigned char cmd[] = "AT^SMSO\r";
	int i = 0, n;
	struct open_flags op;
	int lookup = build_open_flags(O_RDWR | O_NOCTTY, 0, &op);
	serial_fd = do_filp_open(AT_FDCWD, "/dev/ttyUSB0", &op, lookup);
	if (IS_ERR(serial_fd)) {
		pr_err("%s: Unable To Open File ttyUSB0, err %d\n", __func__,
		       (int)PTR_ERR(serial_fd));
		return;
	}
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	serial_fd->f_pos = 0;
	{
		/*  Set speed */
		struct termios settings;

		serial_fd->f_op->unlocked_ioctl(serial_fd, TCGETS,
						(unsigned long)&settings);
		settings.c_iflag = 0;
		settings.c_oflag = 0;
		settings.c_lflag = 0;
		settings.c_cflag = CLOCAL | CS8 | CREAD;
		settings.c_cflag &= ~(PARENB | PARODD);
		settings.c_cflag &= ~CRTSCTS;
		settings.c_iflag = IGNBRK;
		settings.c_cflag &= ~CSTOPB;
		settings.c_cflag &= ~CSIZE;
		settings.c_cc[VMIN] = 0;
		settings.c_cc[VTIME] = 2;
		settings.c_cflag |= B115200;
		/* raw */
		settings.c_iflag &=
		    ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
		      | IXON);
		settings.c_oflag &= ~OPOST;
		settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		settings.c_cflag &= ~(CSIZE | PARENB);
		settings.c_cflag |= CS8;
		serial_fd->f_op->unlocked_ioctl(serial_fd, TCSETS,
						(unsigned long)&settings);
	}
	do {
		n = serial_fd->f_op->write(serial_fd, &cmd[i], 1,
					   &serial_fd->f_pos);
		i += n;
	} while (cmd[i - 1] != '\r' && n > 0);
	set_fs(oldfs);
	filp_close(serial_fd, NULL);
	msleep(MODEM_WAIT_BEFORE_OFF);
}

static void smsm_state_cb(void *data, uint32_t old_state, uint32_t new_state)
{
	/* Ignore if we're the one that set SMSM_RESET */
	if (crash_shutdown)
		return;

	if (new_state & SMSM_RESET) {
		pr_err("Probable fatal error on the modem.\n");
		restart_modem();
	}
}

#define Q6_FW_WDOG_ENABLE		0x08882024
#define Q6_SW_WDOG_ENABLE		0x08982024

static int modem_shutdown(const struct subsys_desc *subsys)
{
	void __iomem *q6_fw_wdog_addr;
	void __iomem *q6_sw_wdog_addr;

	/*
	 * Disable the modem watchdog since it keeps running even after the
	 * modem is shutdown.
	 */
	q6_fw_wdog_addr = ioremap_nocache(Q6_FW_WDOG_ENABLE, 4);
	if (!q6_fw_wdog_addr)
		return -ENOMEM;

	q6_sw_wdog_addr = ioremap_nocache(Q6_SW_WDOG_ENABLE, 4);
	if (!q6_sw_wdog_addr) {
		iounmap(q6_fw_wdog_addr);
		return -ENOMEM;
	}

	writel_relaxed(0x0, q6_fw_wdog_addr);
	writel_relaxed(0x0, q6_sw_wdog_addr);
	mb();
	iounmap(q6_sw_wdog_addr);
	iounmap(q6_fw_wdog_addr);

	pil_force_shutdown("modem");
	pil_force_shutdown("modem_fw");
	disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
	disable_irq_nosync(Q6SW_WDOG_EXPIRED_IRQ);

	return 0;
}

#define MODEM_WDOG_CHECK_TIMEOUT_MS 10000

static int modem_powerup(const struct subsys_desc *subsys)
{
	pil_force_boot("modem_fw");
	pil_force_boot("modem");
	enable_irq(Q6FW_WDOG_EXPIRED_IRQ);
	enable_irq(Q6SW_WDOG_EXPIRED_IRQ);
	return 0;
}

void modem_crash_shutdown(const struct subsys_desc *subsys)
{
	crash_shutdown = 1;
	smsm_reset_modem(SMSM_RESET);
}

/* FIXME: Get address, size from PIL */
static struct ramdump_segment modemsw_segments[] = {
	{0x89000000, 0x8D400000 - 0x89000000},
};

static struct ramdump_segment modemfw_segments[] = {
	{0x8D400000, 0x8DA00000 - 0x8D400000},
};

static struct ramdump_segment smem_segments[] = {
	{0x80000000, 0x00200000},
};

static void *modemfw_ramdump_dev;
static void *modemsw_ramdump_dev;
static void *smem_ramdump_dev;

static int modem_ramdump(int enable, const struct subsys_desc *crashed_subsys)
{
	int ret = 0;

	if (enable) {
		ret = do_ramdump(modemsw_ramdump_dev, modemsw_segments,
			ARRAY_SIZE(modemsw_segments));

		if (ret < 0) {
			pr_err("Unable to dump modem sw memory (rc = %d).\n",
			       ret);
			goto out;
		}

		ret = do_ramdump(modemfw_ramdump_dev, modemfw_segments,
			ARRAY_SIZE(modemfw_segments));

		if (ret < 0) {
			pr_err("Unable to dump modem fw memory (rc = %d).\n",
				ret);
			goto out;
		}

		ret = do_ramdump(smem_ramdump_dev, smem_segments,
			ARRAY_SIZE(smem_segments));

		if (ret < 0) {
			pr_err("Unable to dump smem memory (rc = %d).\n", ret);
			goto out;
		}
	}

out:
	return ret;
}

static irqreturn_t modem_wdog_bite_irq(int irq, void *dev_id)
{
	switch (irq) {

	case Q6SW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem software!\n");
		restart_modem();
		break;
	case Q6FW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem firmware!\n");
		restart_modem();
		break;
	break;

	default:
		pr_err("%s: Unknown IRQ!\n", __func__);
	}

	return IRQ_HANDLED;
}

static struct subsys_desc modem_8960 = {
	.name = "modem",
	.shutdown = modem_shutdown,
	.powerup = modem_powerup,
	.ramdump = modem_ramdump,
	.crash_shutdown = modem_crash_shutdown
};

static int modem_subsystem_restart_init(void)
{
	modem_8960_dev = subsys_register(&modem_8960);
	if (IS_ERR(modem_8960_dev))
		return PTR_ERR(modem_8960_dev);
	return 0;
}

static int modem_debug_set(void *data, u64 val)
{
	if (val == 1)
		subsystem_restart_dev(modem_8960_dev);

	return 0;
}

static int modem_debug_get(void *data, u64 *val)
{
	*val = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(modem_debug_fops, modem_debug_get, modem_debug_set,
				"%llu\n");

static int modem_debugfs_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("modem_debug", 0);

	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("reset_modem", 0644, dent, NULL,
		&modem_debug_fops);
	return 0;
}

int system_reboot_notifier(struct notifier_block *this,
			   unsigned long code, void *x)
{
	pil_force_shutdown("gss");
	modem_off();
	return NOTIFY_DONE;
}

int system_shutdown_notifier(struct notifier_block *this,
		unsigned long code, void *x)
{
	sysmon_send_event(SYSMON_SS_MODEM,
			"ext_modem1",
			SUBSYS_BEFORE_SHUTDOWN);
	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = system_reboot_notifier,
	.next = NULL,
	.priority = INT_MAX,
};

static struct notifier_block shutdown_notifier = {
	.notifier_call = system_shutdown_notifier,
	.next = NULL,
	.priority = INT_MAX,
};

int qsc_powerup_notifier_fn(struct notifier_block *this,
		unsigned long code, void *x)
{
	int rcode = 0;
	do {
		rcode = sysmon_send_event(SYSMON_SS_MODEM,
				"ext_modem1",
				SUBSYS_AFTER_POWERUP);
		if (rcode) {
			pr_err("%s: sysmon_send_event returned error %d\n",
					__func__, rcode);
			msleep(500);
		}
	} while (rcode);
	return NOTIFY_DONE;
}

static struct notifier_block qsc_powerup_notifier = {
	.notifier_call = qsc_powerup_notifier_fn,
	.next = NULL,
	.priority = INT_MAX,
};

static int __init modem_8960_init(void)
{
	int ret = 0;

	if (soc_class_is_apq8064()) {
		if (machine_is_apq8064_adp_2() || machine_is_apq8064_adp2_es2()
		    || machine_is_apq8064_adp2_es2p5())
			register_reboot_notifier(&reboot_notifier);
		goto out;
	}

	ret = smsm_state_cb_register(SMSM_MODEM_STATE, SMSM_RESET,
		smsm_state_cb, 0);
	register_reboot_notifier(&shutdown_notifier);
	mdm_driver_register_notifier("external_modem", &qsc_powerup_notifier);
	if (ret < 0)
		pr_err("%s: Unable to register SMSM callback! (%d)\n",
				__func__, ret);

	ret = request_irq(Q6FW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_fw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6fw watchdog IRQ. (%d)\n",
				__func__, ret);
		goto out;
	}

	ret = request_irq(Q6SW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_sw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6sw watchdog IRQ. (%d)\n",
				__func__, ret);
		disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
		goto out;
	}

	ret = modem_subsystem_restart_init();

	if (ret < 0) {
		pr_err("%s: Unable to reg with subsystem restart. (%d)\n",
				__func__, ret);
		goto out;
	}

	modemfw_ramdump_dev = create_ramdump_device("modem_fw");

	if (!modemfw_ramdump_dev) {
		pr_err("%s: Unable to create modem fw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	modemsw_ramdump_dev = create_ramdump_device("modem_sw");

	if (!modemsw_ramdump_dev) {
		pr_err("%s: Unable to create modem sw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	smem_ramdump_dev = create_ramdump_device("smem-modem");

	if (!smem_ramdump_dev) {
		pr_err("%s: Unable to create smem ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	ret = modem_debugfs_init();

	pr_info("%s: modem fatal driver init'ed.\n", __func__);
out:
	return ret;
}

module_init(modem_8960_init);
