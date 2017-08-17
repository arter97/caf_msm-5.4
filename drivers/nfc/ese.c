/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "ese.h"
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <soc/qcom/scm.h>

static struct of_device_id ese_match_table[] = {
	{.compatible = "qcom,ese"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

/* Device specific macro and structure */
struct ese_dev {
	wait_queue_head_t read_wq; /* wait queue for read interrupt */
	struct mutex read_mutex; /* read mutex */
	struct mutex write_mutex; /* write mutex */
	struct spi_device *spi; /* spi device structure */
	struct miscdevice ese_device; /* char device as misc driver */
	unsigned int pwr_req; /* SE Power request gpio */
	unsigned int use_pwr_req;/*duse SE Power request gpio or use DWP*/
	unsigned int ese_intr;
	unsigned int use_ese_intr; /* flag to indicate irq is used */
	bool	irq_enabled;
	unsigned char enable_poll_mode; /* enable the poll mode */
	spinlock_t irq_enabled_lock; /*spin lock for read irq */
};

/* T==1 protocol specific global data */
const unsigned char SOF = 0xA5u;

/**
 * \ingroup spi_driver
 * \brief To disable IRQ
 *
 * \param[in]       struct ese_dev *
 *
 * \retval void
 *
*/

static void ese_disable_irq(struct ese_dev *ese_dev)
{
	unsigned long flags;

	dev_dbg(&ese_dev->spi->dev, "Entry : %s\n", __func__);
	if (ese_dev->use_ese_intr) {
		spin_lock_irqsave(&ese_dev->irq_enabled_lock, flags);
		if (ese_dev->irq_enabled) {
			disable_irq_nosync(ese_dev->spi->irq);
			ese_dev->irq_enabled = false;
		}
		spin_unlock_irqrestore(&ese_dev->irq_enabled_lock, flags);
	}

	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
}
/**
 * \ingroup spi_driver
 * \brief To enable IRQ
 *
 * \param[in]       struct ese_dev *
 *
 * \retval void
 *
*/
static void ese_enable_irq(struct ese_dev *ese_dev)
{
	unsigned long flags;

	dev_dbg(&ese_dev->spi->dev, "Entry : %s\n", __func__);
	if (ese_dev->use_ese_intr) {
		spin_lock_irqsave(&ese_dev->irq_enabled_lock, flags);
		if (!ese_dev->irq_enabled) {
			ese_dev->irq_enabled = true;
			enable_irq(ese_dev->spi->irq);
		}
		spin_unlock_irqrestore(&ese_dev->irq_enabled_lock, flags);
	}
	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
}
/**
 * \ingroup spi_driver
 * \brief Will get called when interrupt line asserted from ese
 *
 * \param[in]       int
 * \param[in]       void *
 *
 * \retval IRQ handle
 *
*/
static irqreturn_t ese_dev_irq_handler(int irq, void *dev_id)
{
	struct ese_dev *ese_dev = dev_id;

	if (ese_dev->use_ese_intr)
		wake_up(&ese_dev->read_wq);

	return IRQ_HANDLED;
}

/**
 * \ingroup spi_driver
 * \brief Called from SPI LibEse to initilaize the ese device
 *
 * \param[in]		struct inode *
 * \param[in]		struct file *
 *
 * \retval 0 if ok.
 *
*/
static int ese_dev_open(struct inode *inode, struct file *filp)
{
	struct ese_dev
	*ese_dev = container_of(filp->private_data,
			struct ese_dev,
			ese_device);

	filp->private_data = ese_dev;
	dev_dbg(&ese_dev->spi->dev,
		"%s : Major No: %d, Minor No: %d\n", __func__,
		imajor(inode), iminor(inode));
	return 0;
}
/**
 * \ingroup spi_driver
 * \brief To configure the ESE_SET_PWR/ESE_SET_DBG/ese_SET_POLL
 * \n	ese_SET_PWR - hard reset (arg=2), soft reset (arg=1)
 * \n	ese_SET_DBG - Enable/Disable (based on arg value) the driver logs
 * \n	ese_SET_POLL - Configure the driver in poll (arg = 1), interrupt
	(arg = 0) based read operation
 * \param[in]	struct file *
 * \param[in]	unsigned int
 * \param[in]	unsigned long
 *
 * \retval 0 if ok.
 *
*/
static long ese_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	struct ese_dev *ese_dev = NULL;

	ese_dev = filp->private_data;

	dev_dbg(&ese_dev->spi->dev,
		"ese_dev_ioctl-Enter %u arg = %ld ese power req en= %d\n",
		cmd, arg, ese_dev->use_pwr_req);
	switch (cmd) {
	case ESE_SET_PWR:

	if (arg == 2) {
		if (ese_dev->use_pwr_req) {
			gpio_set_value(ese_dev->pwr_req, 1);
			dev_info(&ese_dev->spi->dev, "[eSE] -  inside ese_SET_PWR=2\n");
		}
		if (ese_dev->use_ese_intr)
			ese_enable_irq(ese_dev);
		dev_info(&ese_dev->spi->dev, "[eSE] -  ese_SET_PWR=2\n");
	} else if (arg == 3) {
		if (ese_dev->use_pwr_req)
			gpio_set_value(ese_dev->pwr_req, 0);
		if (ese_dev->use_ese_intr)
			ese_disable_irq(ese_dev);
	} else if (arg == 1) {
		dev_info(&ese_dev->spi->dev, "[eSE] -  ese_SET_PWR=1\n");
		if (ese_dev->use_pwr_req) {
			gpio_set_value(ese_dev->pwr_req, 0);
			msleep(50);
			gpio_set_value(ese_dev->pwr_req, 1);
			msleep(20);
			dev_info(&ese_dev->spi->dev, "[eSE] -  inside ese_SET_PWR=1\n");
		if (ese_dev->use_ese_intr)
			ese_enable_irq(ese_dev);
		}
	}
	break;

	case ESE_SET_DBG:

	dev_info(&ese_dev->spi->dev, "[eSE]-enable debug using dynamic debugging\n");
	break;

	case ESE_SET_MODE:

	ese_dev->enable_poll_mode = (unsigned char)arg;
	if (ese_dev->enable_poll_mode == 0) {
		if (ese_dev->use_ese_intr) {
			dev_dbg(&ese_dev->spi->dev, "[eSE-ese] - IRQ Mode is set\n");
		} else {
			dev_info(&ese_dev->spi->dev,
			"[eSE-ese] - IRQ Mode is NOT set in DT\n");
			dev_info(&ese_dev->spi->dev, "[eSE-ese] - Forcing Poll Mode\n");
			ese_dev->enable_poll_mode = 1;
		}
	} else {
		dev_dbg(&ese_dev->spi->dev, "[eSE-ese] - Poll Mode is set\n");
		ese_dev->enable_poll_mode = 1;
	}
	break;

	default:
	ret = -EINVAL;
	}

	return ret;
}

/**
 * \ingroup spi_driver
 * \brief Write data to ese on SPI
 *
 * \param[in]		struct file *
 * \param[in]		const char *
 * \param[in]		size_t
 * \param[in]		loff_t *
 *
 * \retval data size
 *
*/
static ssize_t ese_dev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *offset)
{
	int ret = -1;
	struct ese_dev *ese_dev;
	unsigned char tx_buffer[MAX_BUFFER_SIZE];


	ese_dev = filp->private_data;

	dev_dbg(&ese_dev->spi->dev,
		"ese_dev_write -Enter count %d\n", (int)count);
	mutex_lock(&ese_dev->write_mutex);
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(&tx_buffer[0], 0, sizeof(tx_buffer));
	if (copy_from_user(&tx_buffer[0], &buf[0], count)) {
		dev_err(&ese_dev->spi->dev,
			"%s : failed to copy from user space\n", __func__);
		mutex_unlock(&ese_dev->write_mutex);
		return -EFAULT;
	}
	/* Write data */
	ret = spi_write(ese_dev->spi, &tx_buffer[0], count);
	if (ret < 0)
		ret = -EIO;
	else
		ret = count;

	mutex_unlock(&ese_dev->write_mutex);
	dev_dbg(&ese_dev->spi->dev, "ese_dev_write ret %d- Exit\n", ret);
	return ret;
}
/**
 * \ingroup spi_driver
 * \brief Used to read data from ese in Poll/interrupt mode
 * configured using ioctl call
 * \param[in]		struct file *
 * \param[in]		char *
 * \param[in]		size_t
 * \param[in]		loff_t *
 *
 * \retval read size
 *
*/
static ssize_t ese_dev_read(struct file *filp, char *buf, size_t count,
	loff_t *offset)
{
	int ret = -EIO;
	struct ese_dev *ese_dev = filp->private_data;
	unsigned char sof = 0x00;
	int total_count = 0;
	int sof_counter = 0;/* one read may take 1 ms*/
	unsigned char rx_buffer[MAX_BUFFER_SIZE+20];

	dev_dbg(&ese_dev->spi->dev,
		"ese_dev_read count %d-Enter\n", (int)count);

	if (count < MAX_BUFFER_SIZE) {
		dev_err(&ese_dev->spi->dev,
		"Invalid length (min : 258) [%d]\n", (int)count);
		return -EINVAL;
	}
	mutex_lock(&ese_dev->read_mutex);
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(&rx_buffer[0], 0x00, sizeof(rx_buffer));
	if (ese_dev->enable_poll_mode) {
		dev_dbg(&ese_dev->spi->dev, "%s Poll Mode Enabled\n", __func__);
		do {
			sof = 0x00;
			sof_counter++;
			ret = spi_read(ese_dev->spi, (void *)&sof, SOF_LENGTH);
			dev_dbg(&ese_dev->spi->dev,
				"SPI_READ ret=%d0x%x\n", sof, ret);

			if (0 > ret) {
				dev_err(&ese_dev->spi->dev, "spi_read failed [SOF]\n");
				goto fail;
			}
			/* if SOF not received, give some time to ese */
			/* RC put the conditional delay only if SOF not received */
			if (sof != SOF)
				usleep_range(1000, 1500);
		} while ((sof != SOF) && (sof_counter < ESE_POLL_TIMEOUT));

		if (sof != SOF) {
			dev_err(&ese_dev->spi->dev, "spi_read SOF time out\n");
			ret = -ETIME;
			goto fail;
		}
	} else {
		if (ese_dev->use_ese_intr) {
			dev_dbg(&ese_dev->spi->dev,
				" %s Interrupt Mode Enabled\n", __func__);
			if (!gpio_get_value(ese_dev->ese_intr)) {
				if (filp->f_flags & O_NONBLOCK) {
					ret = -EAGAIN;
					goto fail;
				}
			}
			while (1) {
				ret = wait_event_interruptible(ese_dev->read_wq,
				ese_dev->irq_enabled);
				if (ret) {
					dev_err(&ese_dev->spi->dev,
					"wait_event_interruptible() : Failed\n");
					goto fail;
				}
				ese_disable_irq(ese_dev);
				if (gpio_get_value(ese_dev->ese_intr))
					break;

				dev_err(&ese_dev->spi->dev,
				"%s: spurious interrupt detected\n", __func__);
			}

		} else
			dev_err(&ese_dev->spi->dev, "%s IRQ not Enabled\n", __func__);

		sof_counter = 0;
		do {
			sof = 0x00;
			ret = spi_read(ese_dev->spi, (void *)&sof, SOF_LENGTH);
			dev_dbg(&ese_dev->spi->dev, "READ returned 0x%x", sof);
			sof_counter++;
			if ((0 > ret)) {
				dev_err(&ese_dev->spi->dev,
					"read SOF Failed 0x%x\n", ret);
				ret = -EIO;
				goto fail;
			}
		} while ((sof != SOF) && (sof_counter < MAX_SOF_IREAD_COUNT));
	}
	total_count = SOF_LENGTH;
	rx_buffer[0] = sof;
	/* Read the HEADER of Two bytes*/
	ret = spi_read(ese_dev->spi, (void *)&rx_buffer[HEADER_OFFSET],
					HEADER_LENGTH);
	dev_dbg(&ese_dev->spi->dev, "SPI_READ 2 returned 0x%x\n", ret);
	if (ret < 0) {
		dev_err(&ese_dev->spi->dev, "spi_read fails after [PCB]\n");
		ret = -EIO;
		goto fail;
	}
	total_count += HEADER_LENGTH;
	/* Get the data length */
	count = rx_buffer[DATA_LEN_INDEX];
	dev_dbg(&ese_dev->spi->dev, "Data Length = %d", (int)count);
	/* Read the available data along with one byte LRC */
	ret = spi_read(ese_dev->spi, (void *)&rx_buffer[DATA_READ_OFFSET],
				(count+1));
	if (ret < 0) {
		dev_err(&ese_dev->spi->dev, "spi_read failed\n");
		ret = -EIO;
		goto fail;
	}
	total_count = (total_count + (count+1));
	dev_dbg(&ese_dev->spi->dev, "total_count = %d", total_count);
	if (copy_to_user(buf, &rx_buffer[0], total_count)) {
		dev_err(&ese_dev->spi->dev,
			"%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto fail;
	}
	ret = total_count;
	if (ese_dev->use_ese_intr)
		ese_enable_irq(ese_dev);
	dev_dbg(&ese_dev->spi->dev, "ese_dev_read ret %d Exit\n", ret);

	mutex_unlock(&ese_dev->read_mutex);

	return ret;

fail:
	dev_err(&ese_dev->spi->dev, "Error ese_dev_read ret %d Exit\n", ret);
	mutex_unlock(&ese_dev->read_mutex);
	return ret;
}
/**
 * \ingroup spi_driver
 * \brief It will configure the GPIOs required for soft reset, read interrupt
 *		  to ese.
 *
 * \param[in]		struct ese_dev *
 * \param[in]		struct spi_device *
 *
 * \retval 0 if ok.
 *
*/
static int ese_hw_setup(struct ese_dev *ese_dev, struct spi_device *spi)
{
	int ret = 0;
	unsigned int irq_flags;

	dev_dbg(&ese_dev->spi->dev, "Entry : %s\n", __func__);
	if (ese_dev->use_pwr_req) {
		ret = gpio_request(ese_dev->pwr_req, "ese reset");
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev, "gpio pwr request failed = 0x%x\n",
				ese_dev->pwr_req);
			goto fail_gpio;
		}

		ret = gpio_direction_output(ese_dev->pwr_req, 0);
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev, "gpio pwr direction request failed = 0x%x\n",
				ese_dev->pwr_req);
			goto fail_gpiop;
		}
	}
	if (ese_dev->use_ese_intr) {
		ese_dev->irq_enabled = true;
		ret = gpio_request(ese_dev->ese_intr, "ese intr");
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev, "gpio intr request failed = 0x%x\n",
				ese_dev->ese_intr);
			goto fail_gpiop;
		}
		ret = gpio_direction_input(ese_dev->ese_intr);
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev, "gpio intr direction failed = 0x%x\n",
				ese_dev->ese_intr);
			goto fail_gpioi;
		}
		ese_dev->spi->irq = gpio_to_irq(ese_dev->ese_intr);
		if (ese_dev->spi->irq < 0) {
			dev_err(&ese_dev->spi->dev, "gpio_to_irq request failed gpio = 0x%x\n",
				ese_dev->ese_intr);
			goto fail_gpioi;
		}

		/* request irq.  the irq is set whenever the chip has data available
		* for reading.  it is cleared when all data has been read.
		*/
		irq_flags =  IRQF_TRIGGER_RISING;

		ret = request_irq(ese_dev->spi->irq, ese_dev_irq_handler,
				irq_flags, ese_dev->ese_device.name, ese_dev);
		if (ret) {
			dev_err(&ese_dev->spi->dev, "gpio_to_irq request failed gpio = 0x%x\n",
				ese_dev->spi->irq);
			goto fail_gpioi;
		}
		ese_disable_irq(ese_dev);
	}

	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
	return ret;

fail_gpioi:
	if (ese_dev->ese_intr)
		gpio_free(ese_dev->ese_intr);
fail_gpiop:
	if (ese_dev->use_pwr_req)
		gpio_free(ese_dev->pwr_req);
fail_gpio:
	return ret;
}
/**
 * \ingroup spi_driver
 * \brief Set the ese device specific context for future use.
 *
 * \param[in]		struct spi_device *
 * \param[in]		void *
 *
 * \retval void
 *
*/
static inline void ese_set_data(struct spi_device *spi, void *data)
{
	dev_set_drvdata(&spi->dev, data);
}
/**
 * \ingroup spi_driver
 * \brief Get the ese device specific context.
 *
 * \param[in]		const struct spi_device *
 *
 * \retval Device Parameters
 *
*/
static inline void *ese_get_data(const struct spi_device *spi)
{
	return dev_get_drvdata(&spi->dev);
}
/* possible fops on the ese device */
static const struct file_operations ese_dev_fops = {
		.owner = THIS_MODULE,
		.read = ese_dev_read,
		.write = ese_dev_write,
		.open = ese_dev_open,
		.unlocked_ioctl = ese_dev_ioctl,
};

static int ese_parse_dt(struct device *dev,
	struct ese_dev *ese_dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	dev_dbg(&ese_dev->spi->dev, "ENTER : %s\n", __func__);
	ese_dev->use_pwr_req = 0;
	ese_dev->use_ese_intr = 0;

	ret = of_property_read_u32(np, "qcom,use_pwr_req",
						&ese_dev->use_pwr_req);
	if (ret != 0)
		ese_dev->use_pwr_req = 0;

	ret = of_property_read_u32(np, "qcom,use_interrupt",
						&ese_dev->use_ese_intr);
	if (ret != 0)
		ese_dev->use_ese_intr = 0;
	if (ese_dev->use_pwr_req) {
		ese_dev->pwr_req = of_get_named_gpio(np, "qcom,ese_pwr_req", 0);
		if (!gpio_is_valid(ese_dev->pwr_req)) {
			dev_err(&ese_dev->spi->dev, "ese pwr_req not valid\n");
			ese_dev->pwr_req = -EINVAL;
			return -EINVAL;
		}
	}
	if (ese_dev->use_ese_intr) {
		ese_dev->ese_intr = of_get_named_gpio(np, "qcom,ese_intr", 0);
		if (!gpio_is_valid(ese_dev->ese_intr)) {
			dev_err(&ese_dev->spi->dev, "ese ese_intr not valid\n");
			ese_dev->ese_intr = -EINVAL;
			return -EINVAL;
		}
	}
	ret = 0;
	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
	return ret;
}
/**
 * \ingroup spi_driver
 * \brief To probe for ese SPI interface. If found initialize
 * the SPI clock, bit rate & SPI mode. It will create the dev
 * entry (ese) for user space.
 * \param[in]		struct spi_device *
 *
 * \retval 0 if ok.
 *
*/
static int ese_probe(struct spi_device *spi)

{
	int ret = -1;
	struct ese_spi_platform_data *platform_data = NULL;
	struct ese_dev *ese_dev = NULL;

	if (spi->dev.of_node) {
		platform_data = devm_kzalloc(&spi->dev,
			sizeof(struct ese_spi_platform_data), GFP_KERNEL);
		if (!platform_data)
			return -ENOMEM;
	} else {
		platform_data = spi->dev.platform_data;
	}

	ese_dev = devm_kzalloc(&spi->dev, sizeof(struct ese_dev), GFP_KERNEL);
	if (ese_dev == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = ESE_SPI_CLOCK;
	dev_dbg(&ese_dev->spi->dev, "%s:freq =%u", __func__, spi->max_speed_hz);
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&ese_dev->spi->dev, "failed to do spi_setup()\n");
		goto ese_register_err;
	}

	ese_dev->spi = spi;
	ese_dev->ese_device.minor = MISC_DYNAMIC_MINOR;
	ese_dev->ese_device.name = "ese";
	ese_dev->ese_device.fops = &ese_dev_fops;
	ese_dev->ese_device.parent = &spi->dev;

	dev_set_drvdata(&spi->dev, ese_dev);

	/* init mutex and queues */
	init_waitqueue_head(&ese_dev->read_wq);
	mutex_init(&ese_dev->read_mutex);
	mutex_init(&ese_dev->write_mutex);

	ret = misc_register(&ese_dev->ese_device);
	if (ret < 0) {
		dev_err(&ese_dev->spi->dev, "misc_register failed! %d\n", ret);
		goto ese_register_err;
	}
	dev_dbg(&ese_dev->spi->dev, "%s chip select : %d , bus number = %d\n",
		__func__, spi->chip_select, spi->master->bus_num);
	ret = ese_parse_dt(&spi->dev, ese_dev);
	if (ret) {
				dev_err(&ese_dev->spi->dev, "Failed to parse DT\n");
				goto ese_setup_err;
	}
	ret = ese_hw_setup(ese_dev, spi);
	if (ret < 0) {
		dev_err(&ese_dev->spi->dev, "Failed ese_hw_setup\n");
		goto ese_setup_err;
	}
	ese_dev->enable_poll_mode = 0; /* Default IRQ read mode */
	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
	return ret;
ese_setup_err:
	misc_deregister(&ese_dev->ese_device);
ese_register_err:
	mutex_destroy(&ese_dev->read_mutex);
	mutex_destroy(&ese_dev->write_mutex);
	devm_kfree(&spi->dev, ese_dev);
err_exit:
	if (ese_dev != NULL)
		dev_dbg(&ese_dev->spi->dev,
			"ERROR: Exit : %s ret %d\n", __func__, ret);
	if (spi->dev.of_node)
		devm_kfree(&spi->dev, platform_data);
	spi_set_drvdata(spi, NULL);
	return ret;
}
/**
 * \ingroup spi_driver
 * \brief Will get called when the device is removed to release the resources.
 *
 * \param[in]		struct spi_device
 *
 * \retval 0 if ok.
 *
*/
static int ese_remove(struct spi_device *spi)

{
	struct ese_dev *ese_dev = ese_get_data(spi);

	dev_dbg(&ese_dev->spi->dev, "Entry : %s\n", __func__);
	if (ese_dev->use_pwr_req)
		gpio_free(ese_dev->pwr_req);
	mutex_destroy(&ese_dev->read_mutex);
	mutex_destroy(&ese_dev->write_mutex);
	misc_deregister(&ese_dev->ese_device);
	dev_dbg(&ese_dev->spi->dev, "Exit : %s\n", __func__);
	devm_kfree(&spi->dev, ese_dev);
	return 0;
}

static struct spi_driver ese_driver = {
		.driver = {
				.name = "ese",
				.bus = &spi_bus_type,
				.owner = THIS_MODULE,
				.of_match_table = ese_match_table,
		},
		.probe =  ese_probe,
		.remove = ese_remove,
};

/**
 * \ingroup spi_driver
 * \brief Module init interface
 *
 * \param[in]		void
 *
 * \retval handle
 *
*/
static int __init ese_dev_init(void)
{
	int x = 1;

	x = spi_register_driver(&ese_driver);
	return x;
}
/**
 * \ingroup spi_driver
 * \brief Module exit interface
 *
 * \param[in]		void
 *
 * \retval void
 *
*/
static void __exit ese_dev_exit(void)
{
	spi_unregister_driver(&ese_driver);
}
module_init(ese_dev_init);
module_exit(ese_dev_exit);

MODULE_DESCRIPTION("eSE SPI driver");
MODULE_LICENSE("GPLv2");
