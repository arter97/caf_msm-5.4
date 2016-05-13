/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#ifndef BTFM_SLIM_H
#define BTFM_SLIM_H
#include <linux/slimbus/slimbus.h>

#define BTFMSLIM_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)
#define BTFMSLIM_INFO(fmt, arg...) pr_warn("%s: " fmt "\n" , __func__ , ## arg)
#define BTFMSLIM_ERR(fmt, arg...)  pr_err("%s: " fmt "\n" , __func__ , ## arg)

/* type predefine */
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

/* Vendor specific defines
 This should redefines in slimbus slave specific header
*/
#define SLIM_SLAVE_COMPATIBLE_STR	"btfmslim_slave"
#define SLIM_SLAVE_REG_OFFSET		0x0000
#define SLIM_SLAVE_RXPORT 			NULL
#define SLIM_SLAVE_TXPORT 			NULL
#define SLIM_SLAVE_INIT 				NULL
#define SLIM_SLAVE_PORT_EN 			NULL

/* Misc defines */
#define SLIM_SLAVE_RW_MAX_TRIES		3
#define SLIM_SLAVE_PRESENT_TIMEOUT	100

#define PGD	1
#define IFD	0


/* Codec driver defines */
enum {
	BTFM_FM_SLIM_TX = 0,
	BTFM_BT_SCO_SLIM_TX,
	BTFM_BT_SCO_A2DP_SLIM_RX,
	BTFM_BT_SPLIT_A2DP_SLIM_RX,
	BTFM_SLIM_NUM_CODEC_DAIS
};

/* Slimbus Port defines - This should be redefined in specific device file */
#define BTFM_SLIM_PGD_PORT_LAST				0xFF

struct btfmslim_ch {
	int id;
	char *name;
	u32 port_hdl;	/* slimbus port handler */
	u16 port;		/* slimbus port number */

	u8 ch;		/* slimbus channel number */
	u16 ch_hdl;	/* slimbus channel handler */
	u16 grph;	/* slimbus group channel handler */
};

struct btfmslim {
	struct device *dev;
	struct slim_device *slim_pgd;
	struct slim_device slim_ifd;
	struct mutex io_lock;
	struct mutex xfer_lock;
	u8 enabled;

	u32 num_rx_port;
	u32 num_tx_port;

	struct btfmslim_ch *rx_chs;
	struct btfmslim_ch *tx_chs;

	int (*vendor_init)(struct btfmslim *btfmslim);
	int (*vendor_port_en)(struct btfmslim *btfmslim, u8 port_num, u8 rxport, u8 enable);
};


/*
 * btfm_slim_get_data: Return btfm slimbus data pointer
 * Returns:
 * Null: invalid data pointer
 * btfmslim: slimbus data information.
 */
extern struct btfmslim * btfm_slim_get_data(void );

/*
 * btfm_slim_hw_init: Initialize slimbus slave device
 * Returns:
 * 0: Sucess
 * else: Fail
 */
extern int btfm_slim_hw_init(struct btfmslim *btfmslim);

/*
 * btfm_slim_hw_deinit: Deinitialize slimbus slave device
 * Returns:
 * 0: Sucess
 * else: Fail
 */
extern int btfm_slim_hw_deinit(struct btfmslim *btfmslim);

/*
 * btfm_slim_write: write value to pgd or ifd device
 * @btfmslim: slimbus slave device data pointer.
 * @reg: slimbus slave register address
 * @bytes: length of data
 * @src: data pointer to write
 * @pgd: selection for device: either PGD or IFD
 * Returns:
 * -EINVAL
 * -ETIMEDOUT
 * -ENOMEM
 */
extern int btfm_slim_write(struct btfmslim *btfmslim,
	u16 reg, int bytes, void *src, u8 pgd);



/*
 * btfm_slim_read: read value from pgd or ifd device
 * @btfmslim: slimbus slave device data pointer.
 * @reg: slimbus slave register address
 * @bytes: length of data
 * @dest: data pointer to read
 * @pgd: selection for device: either PGD or IFD
 * Returns:
 * -EINVAL
 * -ETIMEDOUT
 * -ENOMEM
 */
extern int btfm_slim_read(struct btfmslim *btfmslim,
	u16 reg, int bytes, void *dest, u8 pgd);


/*
 * btfm_slim_enable_ch: enable channel for slimbus slave port
 * @btfmslim: slimbus slave device data pointer.
 * @ch: slimbus slave channel pointer
 * @rxport: rxport or txport
 * Returns:
 * -EINVAL
 * -ETIMEDOUT
 * -ENOMEM
 */
extern int btfm_slim_enable_ch(struct btfmslim *btfmslim, struct btfmslim_ch *ch,
	u8 rxport, u32 rates, u8 grp, u8 nchan);

/*
 * btfm_slim_disable_ch: disable channel for slimbus slave port
 * @btfmslim: slimbus slave device data pointer.
 * @ch: slimbus slave channel pointer
 * @rxport: rxport or txport
 * Returns:
 * -EINVAL
 * -ETIMEDOUT
 * -ENOMEM
 */
extern int btfm_slim_disable_ch(struct btfmslim *btfmslim, struct btfmslim_ch *ch,
	u8 rxport, u8 grp, u8 nchan);

#endif /* BTFM_SLIM_H */
