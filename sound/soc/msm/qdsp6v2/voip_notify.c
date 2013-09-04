/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <sound/voip_notify.h>

#include "q6voice.h"

static int voip_notify_ioctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int voip_notify_ioctl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long voip_notify_ioctl_process(struct file *file, unsigned int cmd,
				      unsigned long u_arg)
{
	int ret = 0;
	bool reg = false;
	struct voip_event evt;
	struct voip_event_type evt_type;
	void __user *arg = (void __user *)u_arg;

	pr_debug("%s: cmd: %u\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_VOIP_EVT_REGISTER:
		reg = true;
	case SNDRV_VOIP_EVT_DEREGISTER:
		if (copy_from_user(&evt_type, arg,
				   sizeof(evt_type))) {
			pr_err("%s: copy_from_user failed\n", __func__);

			ret = -EINVAL;
			goto done;
		}
		voc_client_reg_evt(evt_type, reg);
		break;

	case SNDRV_VOIP_EVT_NOTIFY:
		if (copy_from_user(&evt, arg,
				   sizeof(evt))) {
			pr_err("%s: copy_from_user failed\n", __func__);

			ret = -EINVAL;
			goto done;
		}
		ret = voc_get_voip_evt(&evt);
		if (ret) {
			pr_err("%s: voc_get_voip_evt failed\n", __func__);

			ret = -EINVAL;
			goto done;
		}
		if (copy_to_user((struct voip_event *) arg, &evt,
				 sizeof(struct voip_event))) {
			pr_err("%s: copy_to_user failed\n", __func__);

			ret = -EINVAL;
		}
		break;

	default:
		pr_err("%s: Invalid cmd: %u\n", __func__, cmd);

		ret = -EINVAL;
	}

done:
	pr_debug("%s: return: %d\n", __func__, ret);

	return ret;
}


static const struct file_operations voip_notify_fops = {
	.open =                 voip_notify_ioctl_open,
	.unlocked_ioctl =       voip_notify_ioctl_process,
	.release =              voip_notify_ioctl_release,
};

struct miscdevice voip_notify_misc = {
	.minor  =       MISC_DYNAMIC_MINOR,
	.name   =       VOIP_NOTIFY_IOCTL_NAME,
	.fops   =       &voip_notify_fops,
};

static int __init voip_notify_init(void)
{
	return misc_register(&voip_notify_misc);
}

static void __exit voip_notify_exit(void)
{
}

module_init(voip_notify_init);
module_exit(voip_notify_exit);

MODULE_DESCRIPTION("Soc QDSP6v2 Voip Notify driver");
MODULE_LICENSE("GPL v2");
