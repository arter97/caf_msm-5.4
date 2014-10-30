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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#define AUDIO_SPISLAVE_NAME    "audio_spislave"
#define AUDIO_SPI_NAME    "audio_spi"

static struct spi_device *spislave_client;

static int audio_spi_open(struct inode *inode, struct file *file)
{
	spi_setup(spislave_client);

	return 0;
};

static int audio_spi_release(struct inode *inode, struct file *file)
{
	return 0;
};

static ssize_t audio_spi_read(struct file *file,
				 char __user *buf,
				 size_t len,
				 loff_t *ppos)
{
	int rc;
	char *rx_buf_ptr;

	if ((buf != NULL) && (len > 0)) {
		rx_buf_ptr = kcalloc(len, 1, GFP_USER);
		if (!rx_buf_ptr)
			return -ENOMEM;

		rc = spi_read(spislave_client, rx_buf_ptr, len);

		if (rc)
			goto read_done;
		if (copy_to_user(buf, rx_buf_ptr, len))
			rc = -EFAULT;
	} else {
		return -EINVAL;
	}

read_done:
	kfree(rx_buf_ptr);
	return rc;
}

static ssize_t audio_spi_write(struct file *file,
				 const char __user *buf,
				 size_t len,
				 loff_t *ppos)
{
	int rc;
	char *tx_buf_ptr;

	if ((buf != NULL) && (len > 0)) {
		tx_buf_ptr = kcalloc(len, 1, GFP_USER);
		if (!tx_buf_ptr)
			return -ENOMEM;

		if (copy_from_user(tx_buf_ptr, buf, len)) {
			rc = -EFAULT;
			goto write_done;
		}
		rc = spi_write(spislave_client, tx_buf_ptr, len);
	} else {
		return -EINVAL;
	}

write_done:
	kfree(tx_buf_ptr);
	return rc;
}

static const struct file_operations audio_spislave_fops = {
	.open = audio_spi_open,
	.read = audio_spi_read,
	.write = audio_spi_write,
	.release = audio_spi_release,
};

struct miscdevice audio_spislave_misc = {
	.minor  =       MISC_DYNAMIC_MINOR,
	.name   =       AUDIO_SPISLAVE_NAME,
	.fops   =       &audio_spislave_fops,
};

static int __devinit audio_spi_probe(struct spi_device *spi)
{
	pr_debug("%s, spi_ptr = %p\n", __func__, spi);

	if (misc_register(&audio_spislave_misc)) {
		dev_err(&spi->dev, "Unable to register misc device!\n");
		return -EFAULT;
	}

	spislave_client = spi;
	spislave_client->bits_per_word = 8;

	return 0;
}

static int __devexit audio_spi_remove(struct spi_device *spi)
{
	pr_debug("%s\n", __func__);

	misc_deregister(&audio_spislave_misc);

	spislave_client = NULL;

	return 0;
}

static const struct of_device_id audio_spislave_dt_match[] = {
	{	.compatible = "qcom,audio-spislave",
	},
	{}
};

static struct spi_driver audio_spi_driver = {
	.probe = audio_spi_probe,
	.remove = __devexit_p(audio_spi_remove),
	.driver = {
		.name = AUDIO_SPISLAVE_NAME,
		.of_match_table = audio_spislave_dt_match,
	},

};

static int __init audio_spislave_init(void){
	int ret = 0;

	ret = spi_register_driver(&audio_spi_driver);
	if (ret)
		pr_err("%s: spi register failed: rc=%d\n", __func__, ret);

	return ret;
}
static void __exit audio_spislave_exit(void){
	spi_unregister_driver(&audio_spi_driver);
}

module_init(audio_spislave_init);
module_exit(audio_spislave_exit);

/* Module information */
MODULE_DESCRIPTION("Audio side SPI slave driver");
MODULE_LICENSE("GPL v2");

