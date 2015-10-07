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
/*
 * TODO list:
 * 1. review locks - especially on socket buffers.
 * 2. Need to be decided how port settings will be handled in
 *    set_termios routine
 * 3. set low_latency for tty port
 */

/* TODO: Allow more than one port later */
#define NR_DUN_TTY_PORTS		1
/* TODO: Figure out correct major/minor numbers later */
#define DUN_TTY_MAJOR		240
#define DUN_TTY_MINOR		0

#define BT_DUN_MAX_BUFFER_SIZE	PAGE_SIZE

static struct dun_dev **dun_devs;
static struct tty_driver *dun_tty_drv;

/*
 * TODO: add support for more then one DUN devices
 * private data for both misc device and tty driver
 * All the Tx/Rx are from Bluetooth point of view
 */
struct dun_dev {
	struct sk_buff_head txq;
	struct sk_buff_head rxq;
	struct mutex tx_lock;
	struct mutex rx_lock;
	wait_queue_head_t wait;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	char name[8];
	int id;
	struct tty_port port;
	struct device *tty_dev;
};

static void dun_dev_destruct(struct tty_port *port)
{
	struct dun_dev *dev = container_of(port, struct dun_dev, port);

	tty_unregister_device(dun_tty_drv, dev->id);
	kfree(dev);
}

static struct tty_port_operations dun_tty_port_ops = {
	.destruct = dun_dev_destruct,
};

static void dun_tx_work(struct work_struct *work)
{
	struct dun_dev *dev = container_of(work, struct dun_dev, tx_work);
	struct sk_buff *skb;
	struct tty_struct *tty = dev->port.tty;
	int c;

	while (1) {
		if (mutex_lock_interruptible(&dev->tx_lock))
			return;

		skb = skb_dequeue(&dev->txq);
		mutex_unlock(&dev->tx_lock);

		if (!skb) {
			BT_ERR("nothing to dequeue");
			break;
		}

		c = tty_buffer_request_room(tty, skb->len);
		if (c < skb->len)
			return;

		/* TODO: check the return; skb->len - 1 ?? */
		if (skb->len > 1)
			tty_insert_flip_string(tty, skb->data, skb->len);

		tty_flip_buffer_push(tty);
		kfree_skb(skb);
	}
}

/* ---- DUN misc device helper functions ---- */
static struct dun_dev *dun_dev_add(unsigned index)
{
	struct dun_dev *dev;
	struct tty_struct *tty;

	/*
	 * TODO: for now we don't need anything in argument. If we need
	 * something later, check for NULL and then do copy_from_user
	 */

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	tty = dev->port.tty;

	skb_queue_head_init(&dev->txq);
	skb_queue_head_init(&dev->rxq);
	mutex_init(&dev->tx_lock);
	mutex_init(&dev->rx_lock);
	init_waitqueue_head(&dev->wait);

	/* TODO: harcoded to 1 for now */
	dev->id = index;
	snprintf(dev->name, sizeof(dev->name), "dunBT");

	dev->wq = alloc_workqueue(dev->name, WQ_HIGHPRI | WQ_UNBOUND |
					WQ_MEM_RECLAIM, 1);
	if (!dev->wq) {
		kfree(dev);
		return NULL;
	}
	INIT_WORK(&dev->tx_work, dun_tx_work);

	tty_port_init(&dev->port);
	dev->port.ops = &dun_tty_port_ops;

	dev->tty_dev = tty_register_device(dun_tty_drv, dev->id, NULL);
	if (IS_ERR(dev->tty_dev))
		return NULL;

	return dev;
}

static struct dun_dev *dun_dev_get(unsigned index)
{
	return dun_devs[index];
}

static void dun_dev_del(unsigned index)
{
	struct dun_dev *dev = dun_dev_get(index);

	BT_DBG("dev %p", dev);

	destroy_workqueue(dev->wq);

	tty_port_put(&dev->port);
}

/* ---- DUN misc device driver operations ---- */
static int dun_bt_read(struct file *file, char __user *buf,
				size_t count, loff_t *off)
{
	struct dun_dev *dev = (struct dun_dev *)file->private_data;
	struct sk_buff *skb;
	int ret = 0;
	unsigned int size = 0;
	int read = 0;

	if (!dev)
		return -ENODEV;

	while (count) {
		/*
		 * Lock is not required for empty check as skb_dequeue() on
		 * rxq is performed only in this execution context. We need to
		 * acquire lock if empty check and dequeue operations can be
		 * performed concurrently on rxq.
		 */
		if (skb_queue_empty(&dev->rxq)) {
			ret = -ENODATA;
			break;
		}
		if (mutex_lock_interruptible(&dev->rx_lock))
			return -ERESTARTSYS;

		skb = skb_dequeue(&dev->rxq);
		mutex_unlock(&dev->rx_lock);

		BT_DBG("skb data %p len %d", skb->data, skb->len);
		size = min_t(uint, count, skb->len);
		if (copy_to_user(buf, skb->data, size)) {
			if (mutex_lock_interruptible(&dev->rx_lock))
				return -ERESTARTSYS;

			skb_queue_head(&dev->rxq, skb);
			mutex_unlock(&dev->rx_lock);
			ret =  -EFAULT;
			break;
		}

		if (skb->len > count) {
			skb->data += count;
			skb->len -= count;

			if (mutex_lock_interruptible(&dev->rx_lock))
				return -ERESTARTSYS;

			skb_queue_head(&dev->rxq, skb);
			mutex_unlock(&dev->rx_lock);
		} else {
			kfree_skb(skb);
		}

		read += size;
		buf += size;
		count -= size;
	}
	return read ? read : ret;
}

static int dun_bt_write(struct file *file, const char __user *buf,
				size_t count, loff_t *off)
{
	struct dun_dev *dev = (struct dun_dev *)file->private_data;
	struct sk_buff *skb;

	if (!dev)
		return -ENODEV;

	if (!count)
		return count;

	skb = alloc_skb(count, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	if (copy_from_user(skb_put(skb, count), buf, count))
		return -EFAULT;

	if (mutex_lock_interruptible(&dev->tx_lock))
		return -ERESTARTSYS;

	skb_queue_tail(&dev->txq, skb);
	mutex_unlock(&dev->tx_lock);

	queue_work(dev->wq, &dev->tx_work);

	return count;
}

static unsigned int dun_bt_poll(struct file *file,
				struct poll_table_struct *wait)
{
	struct dun_dev *dev = (struct dun_dev *)file->private_data;
	unsigned int mask = 0;

	if (!dev)
		return POLLERR;

	if (!skb_queue_empty(&dev->rxq))
		goto done;

	poll_wait(file, &dev->wait, wait);

	/* TODO: return error masks as well like POLLHUP */
	if (!skb_queue_empty(&dev->rxq))
		goto done;

	return mask;

done:
	mask |= POLLIN | POLLRDNORM;
	return mask;
}

static long dun_bt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int dun_bt_open(struct inode *inode, struct file *file)
{
	struct dun_dev *dev;

	dev = dun_dev_get(0);
	if (!dev) {
		BT_ERR("Failed to allocate DUN device");
		return -ENOMEM;
	}

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
	struct tty_port *port;
	struct dun_dev *dev = dun_dev_get(0);

	if (!dev)
		return -ENODEV;

	port = &dev->port;
	tty_port_tty_set(port, tty);

	return 0;
}

static void dun_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct tty_port *port;
	struct dun_dev *dev = dun_dev_get(0);

	if (!dev)
		return;

	port = &dev->port;
	tty_port_tty_set(port, NULL);
}

static int dun_tty_write(struct tty_struct *tty, const unsigned char *buf,
				int count)
{
	struct dun_dev *dev = dun_dev_get(tty->index);
	struct sk_buff *skb;

	if (!dev)
		return -ENODEV;

	if (!count)
		return count;

	skb = alloc_skb(count, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	memcpy(skb_put(skb, count), buf, count);

	if (mutex_lock_interruptible(&dev->rx_lock))
		return -ERESTARTSYS;

	skb_queue_tail(&dev->rxq, skb);
	mutex_unlock(&dev->rx_lock);

	wake_up_interruptible_sync(&dev->wait);
	return count;
}

static int dun_tty_write_room(struct tty_struct *tty)
{
	struct dun_dev *dev = dun_dev_get(tty->index);

	if (!dev)
		return -ENODEV;

	/* We are virtual driver, send MAX SIZE as available */
	return BT_DUN_MAX_BUFFER_SIZE;
}

static int dun_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static int dun_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
				unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static void dun_tty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	/* TODO: need to be decided how port settings will be handled */
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

static int dun_tty_tiocmset(struct tty_struct *tty, unsigned int set,
				unsigned int clear)
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
	int ret, i;
	size_t size;

	size = NR_DUN_TTY_PORTS * sizeof(struct dun_dev);
	dun_devs = kzalloc(size, GFP_KERNEL);
	if (!dun_devs) {
		BT_ERR("Failed to allocate DUN TTY ports");
		ret = -ENOMEM;
		goto fail;
	}

	dun_tty_drv = alloc_tty_driver(NR_DUN_TTY_PORTS);
	if (!dun_tty_drv) {
		BT_ERR("Failed to alloc DUN TTY driver");
		ret = -ENOMEM;
		goto fail;
	}

	dun_tty_drv->driver_name = "ttyDUN";
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
		goto fail;
	}

	for (i = 0; i < NR_DUN_TTY_PORTS; i++)
		dun_devs[i] = dun_dev_add(i);

	ret = misc_register(&dun_bt);
	if (ret) {
		BT_ERR("Failed to register DUN BT misc device");
		goto fail;
	}

	BT_INFO("DUN BT device registered");

	return 0;

fail:
	kfree(dun_devs);
	if (dun_tty_drv)
		put_tty_driver(dun_tty_drv);

	return ret;
}

static void __exit dun_tty_exit(void)
{
	int i;

	for (i = 0; i < NR_DUN_TTY_PORTS; i++)
		dun_dev_del(i);

	misc_deregister(&dun_bt);
	BT_INFO("DUN BT device unregistered");

	tty_unregister_driver(dun_tty_drv);
	put_tty_driver(dun_tty_drv);
	BT_INFO("DUN TTY layer uninitialized");
}

module_init(dun_tty_init);
module_exit(dun_tty_exit);

MODULE_DESCRIPTION("Bluetooth DUN serial driver");
MODULE_LICENSE("GPL");
