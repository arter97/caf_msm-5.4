/*
 * TTY driver for Bluetooth DUN terminal
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
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>

#include <net/bluetooth/bluetooth.h>

/* TODO: Allow more than one port later */
#define DUN_TTY_PORTS		1
/* TODO: Figure out correct major/minor numbers later */
#define DUN_TTY_MAJOR		240
#define DUN_TTY_MINOR		0

#define BT_DUN_MAX_BUFFER_SIZE	PAGE_SIZE

#define dun_tx_lock(d)    mutex_lock_interruptible(&d->tx_lock)
#define dun_tx_unlock(d)  mutex_unlock(&d->tx_lock)
#define dun_rx_lock(d)    mutex_lock_interruptible(&d->rx_lock)
#define dun_rx_unlock(d)  mutex_unlock(&d->rx_lock)

static struct tty_driver *dun_tty_drv;

/* TODO: add support for more then one DUN devices */
/* private data for both misc device and tty driver */
struct dun_dev {
	struct sk_buff_head bt_txq;
	struct sk_buff_head bt_rxq;
	struct mutex tx_lock;
	struct mutex rx_lock;
	wait_queue_head_t wait;
};

/* ---- DUN misc device driver operations ---- */
static ssize_t dun_bt_read(struct file *file, char __user *buf,
				size_t count, loff_t *off)
{
	struct dun_dev *dev = (struct dun_dev *)file->private_data;
	struct sk_buff *skb;
	int read = 0, size = 0;
	int ret = 0;

	if (!dev)
		return -EINVAL;

	while (count) {
		/* TODO: do we need to lock to check empty */
		dun_rx_lock(dev);
		if (skb_queue_empty(&dev->bt_rxq)) {
			dun_rx_unlock(dev);
			ret = -ENODATA;
			break;
		}
		skb = skb_dequeue(&dev->bt_rxq);
		dun_rx_unlock(dev);

		BT_DBG("skb data %p len %d", skb->data, skb->len);
		size = min_t(uint, count, skb->len);
		if (copy_to_user(buf, skb->data, size)) {
			dun_rx_lock(dev);
			skb_queue_head(&dev->bt_rxq, skb);
			dun_rx_unlock(dev);
			ret =  -EFAULT;
			break;
		}

		if (skb->len > count) {
			skb->data += count;
			skb->len -= count;

			dun_rx_lock(dev);
			skb_queue_head(&dev->bt_rxq, skb);
			dun_rx_unlock(dev);
		} else
			kfree_skb(skb);

		read += size;
		buf += size;
		count -= size;
	}

	return read ? read : ret;
}

static ssize_t dun_bt_write(struct file *file, const char __user *buf,
				size_t count, loff_t *off)
{
	return 0;
}

static unsigned int dun_bt_poll(struct file *file,
					struct poll_table_struct *wait)
{
	struct dun_dev *dev = (struct dun_dev *)file->private_data;
	unsigned int mask = 0;

	if (!dev)
		return POLLERR;

	poll_wait(file, &dev->wait, wait);

	if (!skb_queue_empty(&dev->bt_rxq))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static long dun_bt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int dun_bt_open(struct inode *inode, struct file *file)
{
	struct dun_dev *dev = NULL;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	skb_queue_head_init(&dev->bt_txq);
	skb_queue_head_init(&dev->bt_txq);
	mutex_init(&dev->tx_lock);
	mutex_init(&dev->rx_lock);
	init_waitqueue_head(&dev->wait);

	file->private_data = dev;
	return 0;
}

static int dun_bt_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* TODO: Figure out compat_ioctl later */
static const struct file_operations dun_bt_ops = {
	.owner = THIS_MODULE,
	.read = dun_bt_read,
	.write = dun_bt_write,
	.poll = dun_bt_poll,
	.open = dun_bt_open,
	.release = dun_bt_release,
	.unlocked_ioctl = dun_bt_ioctl,
};

/*
 * TODO: figure out diff between name and
 * nodename and give description in one
*/
static struct miscdevice dun_bt = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dunBT",
	.nodename = "dunBT",
	.fops = &dun_bt_ops,
};

/* ---- DUN TTY driver operations ---- */
static int dun_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct dun_dev *dev = (struct dun_dev *)tty->driver_data;

	if (!dev)
		return -ENODEV;

	return 0;
}

static void dun_tty_close(struct tty_struct *tty, struct file *filp)
{
}

static int dun_tty_write(struct tty_struct *tty,
			  const unsigned char *buf, int count)
{
	struct dun_dev *dev = (struct dun_dev *)tty->driver_data;
	struct sk_buff *skb;

	BT_DBG("tty %p count %d", tty, count);

	if (!dev)
		return -ENODEV;

	if (!count)
		return count;

	skb = alloc_skb(count, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	memcpy(skb_put(skb, count), buf, count);

	dun_rx_lock(dev);
	skb_queue_tail(&dev->bt_rxq, skb);
	dun_rx_unlock(dev);

	wake_up_interruptible_sync(&dev->wait);
	return count;
}

static int dun_tty_write_room(struct tty_struct *tty)
{
	return 0;
}

static int dun_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static int dun_tty_ioctl(struct tty_struct *tty,
			unsigned int cmd, unsigned long arg)
{
	return 0;
}

static void dun_tty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
}

static void dun_tty_throttle(struct tty_struct *tty)
{
}

static void dun_tty_unthrottle(struct tty_struct *tty)
{
}

static void dun_tty_hangup(struct tty_struct *tty)
{
}

static void dun_tty_flush_buffer(struct tty_struct *tty)
{
}

static void dun_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
}

static void dun_tty_send_xchar(struct tty_struct *tty, char ch)
{
}

static int dun_tty_tiocmget(struct tty_struct *tty)
{
	return 0;
}

static int dun_tty_tiocmset(struct tty_struct *tty,
				unsigned int set, unsigned int clear)
{
	return 0;
}

/*
 * TODO: wake_peer is not declared. Do we need it?
 * wake_peer is called from write func which can be called in
 * interrupt context
 */
static const struct tty_operations dun_tty_ops = {
	.open			= dun_tty_open,
	.close			= dun_tty_close,
	.write			= dun_tty_write,
	.write_room		= dun_tty_write_room,
	.chars_in_buffer	= dun_tty_chars_in_buffer,
	.flush_buffer		= dun_tty_flush_buffer,
	.ioctl			= dun_tty_ioctl,
	.throttle		= dun_tty_throttle,
	.unthrottle		= dun_tty_unthrottle,
	.set_termios		= dun_tty_set_termios,
	.send_xchar		= dun_tty_send_xchar,
	.hangup 		= dun_tty_hangup,
	.wait_until_sent	= dun_tty_wait_until_sent,
	.tiocmget		= dun_tty_tiocmget,
	.tiocmset		= dun_tty_tiocmset,
};

static int __init dun_tty_init(void)
{
	int ret;

	dun_tty_drv = alloc_tty_driver(DUN_TTY_PORTS);
	if (!dun_tty_drv) {
		BT_ERR("Failed to alloc DUN TTY driver");
		return -ENOMEM;
	}

	dun_tty_drv->driver_name = "dun-tty";
	dun_tty_drv->name = "ttyDUN";
	dun_tty_drv->major = DUN_TTY_MAJOR;
	dun_tty_drv->minor_start = DUN_TTY_MINOR;
	dun_tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	dun_tty_drv->subtype = SERIAL_TYPE_NORMAL;
	dun_tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	dun_tty_drv->init_termios = tty_std_termios;
	dun_tty_drv->init_termios.c_cflag =
					B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	dun_tty_drv->init_termios.c_lflag &= ~ICANON;
	tty_set_operations(dun_tty_drv, &dun_tty_ops);

	ret = tty_register_driver(dun_tty_drv);
	if (ret) {
		BT_ERR("Failed to register DUN TTY driver");
		put_tty_driver(dun_tty_drv);
		return ret;
	}
	BT_INFO("DUN TTY layer initialized");

	ret = misc_register(&dun_bt);
	if (ret) {
		BT_ERR("Failed to register DUN BT misc device");
		return ret;
	}
	BT_INFO("DUN BT device registered");

	return 0;
}

static void __exit dun_tty_exit(void)
{
	tty_unregister_driver(dun_tty_drv);
	put_tty_driver(dun_tty_drv);
	BT_INFO("DUN TTY layer uninitialized");

	misc_deregister(&dun_bt);
	BT_INFO("DUN BT device unregistered");
}

module_init(dun_tty_init);
module_exit(dun_tty_exit);

MODULE_DESCRIPTION("Bluetooth DUN serial driver");
MODULE_LICENSE("GPL");
