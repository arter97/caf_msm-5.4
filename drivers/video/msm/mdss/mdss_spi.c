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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/msm_mdp.h>
#include <linux/vmalloc.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

//#include "mdss_panel.h"
//#include "mdss_spi.h"
struct spi_device *mdss_spi;
struct completion spi_te;

#define BUFFER_SIZE 4<<10
//u8 *tx_buf;
atomic_t dspi_panel_te;
int reset_gpio;
int te_gpio;
int dc_gpio;
int panel_init_done = 0;
static int mdss_spi_tx(struct spi_device *spi, const void *buf, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spi_sync(spi, &m);
}

static int spidev_write_cmd(struct spi_device *spi, u8 cmd)
{
    u8 buf[2];

    buf[0] = cmd;

    gpio_set_value(dc_gpio, 0);
    mdss_spi_tx(spi, &buf[0], 1);
    gpio_set_value(dc_gpio, 1);

    return 0;
}

static int spidev_write_data(struct spi_device *spi, u8 data)
{
    u8 buf[2];
    gpio_set_value(dc_gpio, 1);

    buf[0] = data;
    return mdss_spi_tx(spi, &buf[0], 1);
}

int mdss_spi_panel_init(void)
{
	
	if (!gpio_is_valid(reset_gpio)) {
		pr_err("lei: %s:%d, reset gpio not configured\n",__func__, __LINE__);
	}

    if (gpio_request(reset_gpio, "reset-gpios")) {
		pr_err("lei: %s:%d, gpio_request failed\n",__func__, __LINE__);
    }

	if (!gpio_is_valid(dc_gpio)) {
		pr_err("lei: %s:%d, reset gpio not configured\n",__func__, __LINE__);
	}

    if (gpio_request(dc_gpio, "dc-gpios")) {
		pr_err("lei: %s:%d, gpio_request failed\n",__func__, __LINE__);
    }
	gpio_direction_output(reset_gpio, 1);
	gpio_direction_output(dc_gpio, 1);
	gpio_set_value(reset_gpio, 1);
	mdelay(120);
	gpio_set_value(reset_gpio, 0);
	mdelay(120);
	gpio_set_value(reset_gpio, 1);
	mdelay(120);

	mdss_spi->bits_per_word = 8;
	spidev_write_cmd(mdss_spi, 0xfe);
	spidev_write_cmd(mdss_spi, 0xef);
	spidev_write_cmd(mdss_spi, 0x36);
	spidev_write_data(mdss_spi,0x48);
	spidev_write_cmd(mdss_spi, 0x3a);
	spidev_write_data(mdss_spi,0x05);//RGB565
	//spidev_write_data(spi,0x06);//RGB666
	spidev_write_cmd(mdss_spi, 0x35);
	spidev_write_data(mdss_spi,0x00);
	//------------------------------end display control setting--------------------------------//
	//------------------------------Power Control Registers Initial------------------------------//
	spidev_write_cmd(mdss_spi, 0xa4);
	spidev_write_data(mdss_spi,0x44);
	spidev_write_data(mdss_spi,0x44);
	spidev_write_cmd(mdss_spi, 0xa5);
	spidev_write_data(mdss_spi,0x42);
	spidev_write_data(mdss_spi,0x42);
	spidev_write_cmd(mdss_spi, 0xaa);
	spidev_write_data(mdss_spi,0x88);
	spidev_write_data(mdss_spi,0x88);
	spidev_write_cmd(mdss_spi, 0xe8);
	//spidev_write_data(spi,0x11);
	//spidev_write_data(spi,0x0b);
	spidev_write_data(mdss_spi,0x12);//54.1
	//spidev_write_data(spi,0x13);//46.9
	spidev_write_data(mdss_spi,0x40);
	spidev_write_cmd(mdss_spi, 0xe3);
	spidev_write_data(mdss_spi,0x01);
	spidev_write_data(mdss_spi,0x10);
	spidev_write_cmd(mdss_spi, 0xff);
	spidev_write_data(mdss_spi,0x61);
	spidev_write_cmd(mdss_spi, 0xAC);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_cmd(mdss_spi, 0xa6);
	spidev_write_data(mdss_spi,0x2a);
	spidev_write_data(mdss_spi,0x2a);
	spidev_write_cmd(mdss_spi, 0xa7);
	spidev_write_data(mdss_spi,0x2b);
	spidev_write_data(mdss_spi,0x2b);
	spidev_write_cmd(mdss_spi, 0xa8);
	spidev_write_data(mdss_spi,0x18);
	spidev_write_data(mdss_spi,0x18);
	spidev_write_cmd(mdss_spi, 0xa9);
	spidev_write_data(mdss_spi,0x2a);
	spidev_write_data(mdss_spi,0x2a);
	spidev_write_cmd(mdss_spi, 0xad);
	spidev_write_data(mdss_spi,0x33);
	spidev_write_cmd(mdss_spi, 0xaf);
	spidev_write_data(mdss_spi,0x55);
	spidev_write_cmd(mdss_spi, 0xae);
	spidev_write_data(mdss_spi,0x2b);
	//------------------------end Power Control Registers Initial------------------------------//
	//----------------------------display window 240X320------------------------------------//
	spidev_write_cmd(mdss_spi, 0x2a);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_data(mdss_spi,0xef);
	spidev_write_cmd(mdss_spi, 0x2b);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_data(mdss_spi,0x00);
	spidev_write_data(mdss_spi,0x01);
	spidev_write_data(mdss_spi,0x3f);
	spidev_write_cmd(mdss_spi, 0x2c);
	//----------------------------------end display window ----------------------------------------//
	//----------------------------------------gamma setting-----------------------------------------//
	spidev_write_cmd(mdss_spi, 0xf0);
	spidev_write_data(mdss_spi,0x2);
	spidev_write_data(mdss_spi,0x2);
	spidev_write_data(mdss_spi,0x0);
	spidev_write_data(mdss_spi,0x8);
	spidev_write_data(mdss_spi,0xC);
	spidev_write_data(mdss_spi,0x10);
	spidev_write_cmd(mdss_spi, 0xf1);
	spidev_write_data(mdss_spi,0x1);
	spidev_write_data(mdss_spi,0x0);
	spidev_write_data(mdss_spi,0x0);
	spidev_write_data(mdss_spi,0x14);
	spidev_write_data(mdss_spi,0x1D);
	spidev_write_data(mdss_spi,0xE);
	spidev_write_cmd(mdss_spi, 0xf2);
	spidev_write_data(mdss_spi,0x10);
	spidev_write_data(mdss_spi,0x9);
	spidev_write_data(mdss_spi,0x37);
	spidev_write_data(mdss_spi,0x4);
	spidev_write_data(mdss_spi,0x4);
	spidev_write_data(mdss_spi,0x48);
	spidev_write_cmd(mdss_spi, 0xf3);
	spidev_write_data(mdss_spi,0x10);
	spidev_write_data(mdss_spi,0xB);
	spidev_write_data(mdss_spi,0x3F);
	spidev_write_data(mdss_spi,0x5);
	spidev_write_data(mdss_spi,0x5);
	spidev_write_data(mdss_spi,0x4E);
	spidev_write_cmd(mdss_spi, 0xf4);
	spidev_write_data(mdss_spi,0xD);
	spidev_write_data(mdss_spi,0x19);
	spidev_write_data(mdss_spi,0x17);
	spidev_write_data(mdss_spi,0x1D);
	spidev_write_data(mdss_spi,0x1E);
	spidev_write_data(mdss_spi,0xF);
	spidev_write_cmd(mdss_spi, 0xf5);
	spidev_write_data(mdss_spi,0x6);
	spidev_write_data(mdss_spi,0x12);
	spidev_write_data(mdss_spi,0x13);
	spidev_write_data(mdss_spi,0x1A);
	spidev_write_data(mdss_spi,0x1B);
	spidev_write_data(mdss_spi,0xF);
	//------------------------------------end gamma setting-----------------------------------------//
	spidev_write_cmd(mdss_spi, 0x11);
	mdelay(120);
	spidev_write_cmd(mdss_spi, 0x29);
	spidev_write_cmd(mdss_spi, 0x2c);

	return 0;
}
EXPORT_SYMBOL(mdss_spi_panel_init);
#define FB "/data/fb_data"
char *tx_buf1;
int mdss_spi_transfer_data(void *buf, size_t len)
{
//	int wait_count = 50;
	char *tx_buf2;
	int ret;

	tx_buf2 = buf;

	//memcpy((tx_buf1), (tx_buf2), 240*2*320);	

	{
		int i =0;
		int j =0;
		for (i = 0; i < 320*256*2;) {
			 memcpy((tx_buf1+j),(tx_buf2+i), 240*2);
			 i = i + 256*2;
			 j = j + 240*2;
		}	
	}

#if 0
	if(0){
		struct file *file = NULL;
		mm_segment_t fs;
		loff_t pos = 0;
		file = filp_open(FB, O_RDWR | O_CREAT, 0644);
		if (IS_ERR(file)) {
				printk("error occured while opening file exiting...\n");
				return 0;
		}
		fs = get_fs(); 
		set_fs(KERNEL_DS); 

		vfs_write(file, tx_buf1, 240*320*2, &pos);

		filp_close(file, NULL);  
		set_fs(fs);
		file = NULL;
	}
#endif
	INIT_COMPLETION(spi_te);
	ret = wait_for_completion_timeout(&spi_te,
				   msecs_to_jiffies(200));
	
	mdss_spi->bits_per_word = 16;
	mdss_spi_tx(mdss_spi, tx_buf1, 240*320*2);

	return 0;
}
EXPORT_SYMBOL(mdss_spi_transfer_data);

int mdss_spi_transfer(const void *buf, size_t len)
{

	char *tx_buf1;
	char *tx_buf2;
	char *tx_buf3;
	int count = 20;
	int wait_count;
	int data_size = 240*320*2;
	tx_buf1 = kmalloc(data_size, GFP_KERNEL);
	tx_buf2 = kmalloc(data_size, GFP_KERNEL);
	tx_buf3 = kmalloc(data_size, GFP_KERNEL);

	memset(tx_buf1,0,data_size);
	memset(tx_buf2,0,data_size);
	memset(tx_buf3,0,data_size);

	mdss_spi_panel_init();

	mdelay(120);
	
	//set_addr_win(mdss_spi, 0, 0, 239, 319);
    gpio_set_value(dc_gpio, 1);
	{
		    int i,j,k;   
			k = 0;
//			set_addr_win(mdss_spi, 0, 0, 239, 319);    
			for (i = 0; i < 320; i++) {        
				for (j = 0; j < 240; j++) {            
					//tx_buf1[k] = 0xF8;
					//tx_buf1[k+1] = 0x00;

					//tx_buf2[k] = 0x07;
					//tx_buf2[k+1] = 0xE0;

					//tx_buf3[k] = 0x00;
					//tx_buf3[k+1] = 0x1F;
					tx_buf1[k] = 0xF8;
					tx_buf1[k+1] = 0x00;

					tx_buf2[k] = 0x07;
					tx_buf2[k+1] = 0xE0;

					tx_buf3[k] = 0x00;
					tx_buf3[k+1] = 0x1F;

					k = k+2;
				}    
			}
	}

	mdelay(120);

    gpio_set_value(dc_gpio, 1);

	while(count--){
		wait_count = 100;
		atomic_set(&dspi_panel_te,0);
		while((!(atomic_read(&dspi_panel_te)))&&(wait_count)){
			mdelay(1);
			wait_count --;
			if(!wait_count)
				pr_err("lei: TE1 timeout\n");
		}
		spi_write(mdss_spi, tx_buf1, data_size);

		atomic_set(&dspi_panel_te,0);	
		wait_count = 100;
		while((!(atomic_read(&dspi_panel_te)))&&(wait_count)){
			mdelay(1);
			wait_count --;
			if(!wait_count)
				pr_err("lei: TE2 timeout\n");
		}
		spi_write(mdss_spi, tx_buf2, data_size);
	}
	panel_init_done = 1;

	kfree(tx_buf1);
	kfree(tx_buf2);
	kfree(tx_buf3);

	return 0;
}

EXPORT_SYMBOL(mdss_spi_transfer);

irqreturn_t dsi_te_handler(int irq, void *data)
{
	complete(&spi_te);

	return IRQ_HANDLED;
}
static int mdss_spi_probe(struct spi_device *spi)
{
	int irq,rc;
	int cs;
	int cpha,cpol,cs_high;
	u32 max_speed;
	
	tx_buf1 = kmalloc(240*320*2, GFP_KERNEL);

	pr_err("mdss_spi_probe lei:spi\n");


	mdss_spi = spi;
	init_completion(&spi_te);

	reset_gpio = of_get_named_gpio(spi->dev.of_node, "panel-reset-gpio", 0);
	dc_gpio = of_get_named_gpio(spi->dev.of_node, "panel-dc-gpio", 0);
	te_gpio = of_get_named_gpio(spi->dev.of_node, "panel-te-gpio", 0);
    irq = spi->irq;
    cs = spi->chip_select;
    cpha = (spi->mode & SPI_CPHA) ? 1:0;
    cpol = (spi->mode & SPI_CPOL) ? 1:0;
    cs_high = (spi->mode & SPI_CS_HIGH) ? 1:0;
    max_speed = spi->max_speed_hz;
	pr_err("lei: reset_gpio = %d,dc_gpio = %d,te_gpio = %d \n",reset_gpio,dc_gpio,te_gpio);
    pr_err("lei: cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x] Max_speed[%d] bits_per_word[%d]\n",
                        cs, cpha, cpol, cs_high, max_speed, spi->bits_per_word);

	//mdss_spi_panel_init(spi);

	atomic_set(&dspi_panel_te,0);

	rc = devm_request_irq(&spi->dev,
		gpio_to_irq(te_gpio),
		dsi_te_handler, IRQF_TRIGGER_RISING,
		"VSYNC_GPIO", NULL);
	if (rc) {
		pr_err("TE request_irq failed.\n");
	}

	//return mdss_spi_transfer(spi, buf, 2);
	return 0;
	
}

static int mdss_spi_remove(struct spi_device *pdev)
{
	return 0;
}

static const struct of_device_id mdss_spi_dt_match[] = {
    { .compatible = "qcom,mdss-spi-ctrl" },
    {},
};


static struct spi_driver mdss_spi_platform_driver = {
	.probe = mdss_spi_probe,
	.remove = mdss_spi_remove,
	.driver = {
	    .name = "mdss-spi-ctrl",
        .owner  = THIS_MODULE,
        .of_match_table = mdss_spi_dt_match,
    },
};

static int __init mdss_spi_init(void)
{
    int ret;

	ret = spi_register_driver(&mdss_spi_platform_driver);
    printk("%s %d %d\n", __func__, __LINE__, ret);

	return 0;
}

static void __exit mdss_spi_exit(void)
{

}

//module_spi_driver(wcd_spi_driver);

module_init(mdss_spi_init);
module_exit(mdss_spi_exit);

