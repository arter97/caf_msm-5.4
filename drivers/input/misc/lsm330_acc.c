/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V.1.2.6.1
 * Date			: 2013/Oct/02
 * Description		: LSM330 accelerometer driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manages int1
			and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3		Ch state program codes and state prog parameters defines
	V.1.2.5		Changes create_sysfs_interfaces
	V.1.2.6		Changes resume and suspend functions
	V.1.2.6.1	Introduce SignMotion feat implementation and solves
			acc suspend/resume issue;
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>

/* #include <linux/input/lsm330.h> */
#include <linux/lsm330.h>

#define DEBUG			1

#define LOAD_SM1_PROGRAM	0
#define LOAD_SM1_PARAMETERS	0
#define LOAD_SM2_PROGRAM	1
#define LOAD_SM2_PARAMETERS	1


#if ENABLE_SIGNIFICANT_MOTION > 0
#define ABS_SIGN_MOTION ABS_WHEEL
/* customized thresholds for significant motion program*/
#define THRS1_2_02G 0x09
#define THRS1_2_04G 0x05
#define THRS1_2_06G 0x03
#define THRS1_2_08G 0x02
#define THRS1_2_16G 0x01
#endif

#define G_MAX			23920640	/* ug */
#define I2C_RETRY_DELAY		5		/* Waiting for signals [ms] */
#define I2C_RETRIES		5		/* Number of retries */
#define I2C_AUTO_INCREMENT	0x00		/* Autoincrement i2c address */
#define MS_TO_NS(x)		(x*1000000L)

#define SENSITIVITY_2G		1		/* ug/LSB	*/
#define SENSITIVITY_4G		2		/* ug/LSB	*/
#define SENSITIVITY_6G		3		/* ug/LSB	*/
#define SENSITIVITY_8G		4		/* ug/LSB	*/
#define SENSITIVITY_16G		12		/* ug/LSB	*/

#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define LSM330_ODR_MASK		(0xF0)
#define LSM330_PM_OFF		(0x00)		/* OFF */
#define LSM330_ODR3_125		(0x10)		/*    3.125 Hz */
#define LSM330_ODR6_25		(0x20)		/*    6.25  Hz */
#define LSM330_ODR12_5		(0x30)		/*   12.5   Hz */
#define LSM330_ODR25		(0x40)		/*   25     Hz */
#define LSM330_ODR50		(0x50)		/*   50     Hz */
#define LSM330_ODR100		(0x60)		/*  100     Hz */
#define LSM330_ODR400		(0x70)		/*  400     Hz */
#define LSM330_ODR800		(0x80)		/*  800     Hz */
#define LSM330_ODR1600		(0x90)		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREGx */
#define LSM330_INTEN_MASK		(0x01)
#define LSM330_INTEN_OFF		(0x00)
#define LSM330_INTEN_ON			(0x01)

/* CTRLREG1 */
#define LSM330_HIST1_MASK		(0xE0)
#define LSM330_SM1INT_PIN_MASK		(0x08)
#define LSM330_SM1INT_PININT2		(0x08)
#define LSM330_SM1INT_PININT1		(0x00)
#define LSM330_SM1_EN_MASK		(0x01)
#define LSM330_SM1_EN_ON		(0x01)
#define LSM330_SM1_EN_OFF		(0x00)
/* */

/* CTRLREG2 */
#define LSM330_HIST2_MASK		(0xE0)
#define LSM330_SM2INT_PIN_MASK		(0x08)
#define LSM330_SM2INT_PININT2		(0x08)
#define LSM330_SM2INT_PININT1		(0x00)
#define LSM330_SM2_EN_MASK		(0x01)
#define LSM330_SM2_EN_ON		(0x01)
#define LSM330_SM2_EN_OFF		(0x00)
/* */

/* CTRLREG3 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		(0x00)

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		~(LSM330_INT2_EN_ON)

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		~(LSM330_INT1_EN_ON)
/* */

/*Data Ready INT*/

#define LSM330_DRDYINT1_EN_ON		(0x01 << 7)
#define LSM330_DRDYINT1_EN_OFF		~(LSM330_DRDYINT1_EN_ON)

/* CTRLREG4 */
#define LSM330_BDU_EN			(0x08)
#define LSM330_ALL_AXES			(0x07)
/* */

/* STATUS REG BITS */
#define LSM330_STAT_INTSM1_BIT		(0x01 << 3)
#define LSM330_STAT_INTSM2_BIT		(0x01 << 2)
#define LSM330_STAT_INTDRDY_BIT		(0x01)

#define OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		(0x40)	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define LSM330_WHO_AM_I			(0x0F)	/* WhoAmI register Address */

#define LSM330_OUTX_L			(0x28)	/* Output X LSByte */
#define LSM330_OUTX_H			(0x29)	/* Output X MSByte */
#define LSM330_OUTY_L			(0x2A)	/* Output Y LSByte */
#define LSM330_OUTY_H			(0x2B)	/* Output Y MSByte */
#define LSM330_OUTZ_L			(0x2C)	/* Output Z LSByte */
#define LSM330_OUTZ_H			(0x2D)	/* Output Z MSByte */
#define LSM330_LC_L			(0x16)	/* LSByte Long Counter Status */
#define LSM330_LC_H			(0x17)	/* MSByte Long Counter Status */

#define LSM330_INTERR_STAT		(0x18)	/* Interrupt Status */

#define LSM330_STATUS_REG		(0x27)	/* Status */

#define LSM330_CTRL_REG1		(0x21)	/* control reg 1 */
#define LSM330_CTRL_REG2		(0x22)	/* control reg 2 */
#define LSM330_CTRL_REG3		(0x23)	/* control reg 3 */
#define LSM330_CTRL_REG4		(0x20)	/* control reg 4 */
#define LSM330_CTRL_REG5		(0x24)	/* control reg 5 */
#define LSM330_CTRL_REG6		(0x25)	/* control reg 6 */

#define LSM330_OFF_X			(0x10)	/* Offset X Corr */
#define LSM330_OFF_Y			(0x11)	/* Offset Y Corr */
#define LSM330_OFF_Z			(0x12)	/* Offset Z Corr */

#define LSM330_CS_X			(0x13)	/* Const Shift X */
#define LSM330_CS_Y			(0x14)	/* Const Shift Y */
#define LSM330_CS_Z			(0x15)	/* Const Shift Z */

#define LSM330_VFC_1			(0x1B)	/* Vect Filter Coeff 1 */
#define LSM330_VFC_2			(0x1C)	/* Vect Filter Coeff 2 */
#define LSM330_VFC_3			(0x1D)	/* Vect Filter Coeff 3 */
#define LSM330_VFC_4			(0x1E)	/* Vect Filter Coeff 4 */


	/* state machine 1 program */
#define LSM330_STATEPR1		(0x40)	/*	State Program 1 16 bytes */
	/* state machine 1 params */
#define LSM330_TIM4_1		(0x50)	/*	SPr1 Timer4		*/
#define LSM330_TIM3_1		(0x51)	/*	SPr1 Timer3		*/
#define LSM330_TIM2_1		(0x52)	/*	SPr1 Timer2	2bytes	*/
//
#define LSM330_TIM1_1		(0x54)	/*	SPr1 Timer1	2bytes	*/
//
#define LSM330_THRS2_1		(0x56)	/*	SPr1 Threshold2		*/
#define LSM330_THRS1_1		(0x57)	/*	SPr1 Threshold1		*/
#define LSM330_SA_1		(0x59)	/*	SPr1 Swap Axis Sign Msk	*/
#define LSM330_MA_1		(0x5A)	/*	SPr1 Axis Sign Msk	*/
#define LSM330_SETT_1		(0x5B)	/*	SPr1 			*/
#define LSM330_PPRP_1		(0x5C)	/*	SPr1 ProgPointer ResetPointer */
#define LSM330_TC_1		(0x5D)	/*	SPr1 		2bytes	*/
#define LSM330_OUTS_1		(0x5F)	/*	SPr1 			*/

	/* state machine 2 program */
#define LSM330_STATEPR2	(0x60)	/*	State Program 2 16 bytes */
	/* state machine 2 params */
#define LSM330_TIM4_2		(0x70)	/*	SPr2 Timer4		*/
#define LSM330_TIM3_2		(0x71)	/*	SPr2 Timer3		*/
#define LSM330_TIM2_2		(0x72)	/*	SPr2 Timer2	2bytes	*/
//
#define LSM330_TIM1_2		(0x74)	/*	SPr2 Timer1	2bytes	*/
//
#define LSM330_THRS2_2		(0x76)	/*	SPr2 Threshold2		*/
#define LSM330_THRS1_2		(0x77)	/*	SPr2 Threshold1		*/
#define LSM330_DES_2		(0x78)	/*	SPr2 Decimation		*/
#define LSM330_SA_2		(0x79)	/*	SPr2 Swap Axis Sign Msk	*/
#define LSM330_MA_2		(0x7A)	/*	SPr2 Axis Sign Msk	*/
#define LSM330_SETT_2		(0x7B)	/*	SPr2 			*/
#define LSM330_PPRP_2		(0x7C)	/*	SPr2 ProgPointer ResetPointer */
#define LSM330_TC_2		(0x7D)	/*	SPr2 		2bytes	*/
//
#define LSM330_OUTS_2		(0x7F)	/*	SPr2 			*/
/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define RES_LSM330_LC_L				0
#define RES_LSM330_LC_H				1

#define RES_LSM330_CTRL_REG4			2
#define RES_LSM330_CTRL_REG1			3
#define RES_LSM330_CTRL_REG2			4
#define RES_LSM330_CTRL_REG3			5
#define RES_LSM330_CTRL_REG5			6
#define RES_LSM330_CTRL_REG6			7

#define RES_LSM330_OFF_X			8
#define RES_LSM330_OFF_Y			9
#define RES_LSM330_OFF_Z			10

#define RES_LSM330_CS_X				11
#define RES_LSM330_CS_Y				12
#define RES_LSM330_CS_Z				13

#define RES_LSM330_VFC_1			14
#define RES_LSM330_VFC_2			15
#define RES_LSM330_VFC_3			16
#define RES_LSM330_VFC_4			17

#define RES_LSM330_THRS3			18

#define RES_LSM330_TIM4_1			20
#define RES_LSM330_TIM3_1			21
#define RES_LSM330_TIM2_1_L			22
#define RES_LSM330_TIM2_1_H			23
#define RES_LSM330_TIM1_1_L			24
#define RES_LSM330_TIM1_1_H			25

#define RES_LSM330_THRS2_1			26
#define RES_LSM330_THRS1_1			27
#define RES_LSM330_SA_1				28
#define RES_LSM330_MA_1				29
#define RES_LSM330_SETT_1			30

#define RES_LSM330_TIM4_2			31
#define RES_LSM330_TIM3_2			32
#define RES_LSM330_TIM2_2_L			33
#define RES_LSM330_TIM2_2_H			34
#define RES_LSM330_TIM1_2_L			35
#define RES_LSM330_TIM1_2_H			36

#define RES_LSM330_THRS2_2			37
#define RES_LSM330_THRS1_2			38
#define RES_LSM330_DES_2			39
#define RES_LSM330_SA_2				40
#define RES_LSM330_MA_2				41
#define RES_LSM330_SETT_2			42

#define LSM330_RESUME_ENTRIES			43



#define LSM330_STATE_PR_SIZE			16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define LSM330_SM1_DIS_SM2_DIS			(0x00)
#define LSM330_SM1_EN_SM2_DIS			(0x01)
#define LSM330_SM1_DIS_SM2_EN			(0x02)
#define LSM330_SM1_EN_SM2_EN			(0x03)

/* INTERRUPTS ENABLE CONTROLS */
#define LSM330_INT1_DIS_INT2_DIS		(0x00)
#define LSM330_INT1_EN_INT2_DIS			(0x01)
#define LSM330_INT1_DIS_INT2_EN			(0x02)
#define LSM330_INT1_EN_INT2_EN			(0x03)

#define LSM330_SENSOR_ACC_MIN_POLL_PERIOD_MS 10
#define POLL_MS_100HZ 10

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330_acc_odr_table[] = {
		{    1, LSM330_ODR1600 },
		{    3, LSM330_ODR400  },
		{   10, LSM330_ODR100  },
		{   20, LSM330_ODR50   },
		{   40, LSM330_ODR25   },
		{   80, LSM330_ODR12_5 },
		{  160, LSM330_ODR6_25 },
		{  320, LSM330_ODR3_125},
};

static struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
	.fs_range = LSM330_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 10,
	.min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

static int int1_gpio = LSM330_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);
MODULE_PARM_DESC(int1_gpio, "integer: gpio number being assined to interrupt PIN1");
MODULE_PARM_DESC(int2_gpio, "integer: gpio number being assined to interrupt PIN2");

struct lsm330_acc_data {
	struct i2c_client *client;
	struct lsm330_acc_platform_data *pdata;

	struct mutex lock;
	struct hrtimer hr_timer_acc;
	ktime_t ktime_acc;

	struct input_dev *input_dev;

#ifdef CUSTOM_SYSFS_PATH
	struct class *acc_class;
	struct device *acc_dev;
#endif

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	atomic_t sign_mot_enabled;
	int enable_polling;
	int use_interrupt;
	int on_before_suspend;
	int use_smbus;

	u16 sensitivity;

	u8 resume_state[LSM330_RESUME_ENTRIES];
	u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
	u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct sensors_classdev accel_cdev;
	struct regulator *vdd;
	struct regulator *vddio;
	bool power_enabled;
	int acc_wkp_flag;
	bool acc_delay_change;
	struct task_struct *acc_task;
	wait_queue_head_t acc_wq;
	ktime_t timestamp;
#ifdef DEBUG
	u8 reg_addr;
#endif
};

/* Accelerometer information read by HAL */
static struct sensors_classdev lsm330_acc_cdev = {
	.name = LSM330_ACC_DEV_NAME,
	.vendor = "STMicro",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",	/* m/s^2 */
	.resolution = "0.000598144",	/* m/s^2 */
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = LSM330_SENSOR_ACC_MIN_POLL_PERIOD_MS,
	.delay_msec = 100,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int acc_poll_thread(void *data);


static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc,
				u8 *buf, int len);

static void lsm330_acc_set_drdyint_register_values(
		struct lsm330_acc_data *acc, int en)
{
	if (en) {
		if (acc->pdata->gpio_int1 >= 0)
			acc->resume_state[RES_LSM330_CTRL_REG3] =
				acc->resume_state[RES_LSM330_CTRL_REG3] | \
							LSM330_INT1_EN_ON |  \
							LSM330_DRDYINT1_EN_ON;

		if (acc->pdata->gpio_int2 >= 0)
			acc->resume_state[RES_LSM330_CTRL_REG3] =
				acc->resume_state[RES_LSM330_CTRL_REG3] | \
						LSM330_INT2_EN_ON | \
						LSM330_DRDYINT1_EN_ON;
		dev_dbg(&acc->client->dev, "INT ON settings\n");
	} else {
		dev_dbg(&acc->client->dev, "INT OFF settings\n");
		acc->resume_state[RES_LSM330_CTRL_REG3] &=
						LSM330_INT1_EN_OFF;
		acc->resume_state[RES_LSM330_CTRL_REG3] &=
						LSM330_INT2_EN_OFF;
		acc->resume_state[RES_LSM330_CTRL_REG3] &=
						LSM330_DRDYINT1_EN_OFF;
	}
}

static void lsm330_acc_drdyint(struct lsm330_acc_data *acc)
{
	int err = -1;
	u8 buf[2];

	buf[0] = (LSM330_CTRL_REG3);
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG3];
	err = lsm330_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "lsm330_acc_set_drdyint fail\n");

}

/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
	acc->resume_state[RES_LSM330_LC_L] = 0x00;
	acc->resume_state[RES_LSM330_LC_H] = 0x00;

	acc->resume_state[RES_LSM330_CTRL_REG3] = LSM330_INT_ACT_H;

	lsm330_acc_set_drdyint_register_values(acc, acc->use_interrupt);

	acc->resume_state[RES_LSM330_CTRL_REG4] = (LSM330_BDU_EN |
							LSM330_ALL_AXES);
	acc->resume_state[RES_LSM330_CTRL_REG5] = 0x00;
	acc->resume_state[RES_LSM330_CTRL_REG6] = 0x10;

	acc->resume_state[RES_LSM330_THRS3] = 0x00;
	acc->resume_state[RES_LSM330_OFF_X] = 0x00;
	acc->resume_state[RES_LSM330_OFF_Y] = 0x00;
	acc->resume_state[RES_LSM330_OFF_Z] = 0x00;

	acc->resume_state[RES_LSM330_CS_X] = 0x00;
	acc->resume_state[RES_LSM330_CS_Y] = 0x00;
	acc->resume_state[RES_LSM330_CS_Z] = 0x00;

	acc->resume_state[RES_LSM330_VFC_1] = 0x00;
	acc->resume_state[RES_LSM330_VFC_2] = 0x00;
	acc->resume_state[RES_LSM330_VFC_3] = 0x00;
	acc->resume_state[RES_LSM330_VFC_4] = 0x00;
}

static void lsm330_acc_set_init_statepr1_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program1 */
#if LOAD_SM1_PROGRAM > 0
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0x00;
	acc->resume_stmach_program1[3] = 0x00;
	acc->resume_stmach_program1[4] = 0x00;
	acc->resume_stmach_program1[5] = 0x00;
	acc->resume_stmach_program1[6] = 0x00;
	acc->resume_stmach_program1[7] = 0x00;
	acc->resume_stmach_program1[8] = 0x00;
	acc->resume_stmach_program1[9] = 0x00;
	acc->resume_stmach_program1[10] = 0x00;
	acc->resume_stmach_program1[11] = 0x00;
	acc->resume_stmach_program1[12] = 0x00;
	acc->resume_stmach_program1[13] = 0x00;
	acc->resume_stmach_program1[14] = 0x00;
	acc->resume_stmach_program1[15] = 0x00;
#else /* loads default state program1 */
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0x00;
	acc->resume_stmach_program1[3] = 0x00;
	acc->resume_stmach_program1[4] = 0x00;
	acc->resume_stmach_program1[5] = 0x00;
	acc->resume_stmach_program1[6] = 0x00;
	acc->resume_stmach_program1[7] = 0x00;
	acc->resume_stmach_program1[8] = 0x00;
	acc->resume_stmach_program1[9] = 0x00;
	acc->resume_stmach_program1[10] = 0x00;
	acc->resume_stmach_program1[11] = 0x00;
	acc->resume_stmach_program1[12] = 0x00;
	acc->resume_stmach_program1[13] = 0x00;
	acc->resume_stmach_program1[14] = 0x00;
	acc->resume_stmach_program1[15] = 0x00;
#endif /* LOAD_SM1_PROGRAM */
}

static void lsm330_acc_set_init_statepr2_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program2 */
#if LOAD_SM2_PROGRAM > 0
#if ENABLE_SIGNIFICANT_MOTION > 0
	acc->resume_stmach_program2[0] = 0x05;
	acc->resume_stmach_program2[1] = 0x03;
	acc->resume_stmach_program2[2] = 0x04;
	acc->resume_stmach_program2[3] = 0x45;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#else
	acc->resume_stmach_program2[0] = 0x00;
	acc->resume_stmach_program2[1] = 0x00;
	acc->resume_stmach_program2[2] = 0x00;
	acc->resume_stmach_program2[3] = 0x00;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#endif /* ENABLE_SIGNIFICANT_MOTION */
#else /* loads default state program2 */
	acc->resume_stmach_program2[0] = 0x00;
	acc->resume_stmach_program2[1] = 0x00;
	acc->resume_stmach_program2[2] = 0x00;
	acc->resume_stmach_program2[3] = 0x00;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#endif /* LOAD_SM2_PROGRAM */
}

static void lsm330_acc_set_init_statepr1_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog1 parameters */
#if LOAD_SM1_PARAMETERS > 0
	acc->resume_state[RES_LSM330_TIM4_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_1] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LSM330_SA_1] = 0x00;
	acc->resume_state[RES_LSM330_MA_1] = 0x00;
	acc->resume_state[RES_LSM330_SETT_1] = 0x00;
#else 	/* loads default state prog1 parameters */
	acc->resume_state[RES_LSM330_TIM4_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_1] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LSM330_SA_1] = 0x00;
	acc->resume_state[RES_LSM330_MA_1] = 0x00;
	acc->resume_state[RES_LSM330_SETT_1] = 0x00;
#endif
}

static void lsm330_acc_set_init_statepr2_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog2 parameters */
#if LOAD_SM2_PARAMETERS > 0
#if ENABLE_SIGNIFICANT_MOTION > 0
	acc->resume_state[RES_LSM330_TIM4_2] = 0x64;
	acc->resume_state[RES_LSM330_TIM3_2] = 0xC8;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = THRS1_2_02G;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0xA8;
	acc->resume_state[RES_LSM330_MA_2] = 0xA8;
	acc->resume_state[RES_LSM330_SETT_2] = 0x13;
#else
	acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = 0x00;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0x00;
	acc->resume_state[RES_LSM330_MA_2] = 0x00;
	acc->resume_state[RES_LSM330_SETT_2] = 0x00;
#endif  /* ENABLE_SIGNIFICANT_MOTION */
#else	/* loads default state prog2 parameters */
	acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = 0x00;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0x00;
	acc->resume_state[RES_LSM330_MA_2] = 0x00;
	acc->resume_state[RES_LSM330_SETT_2] = 0x00;
#endif
}

static int lsm330_acc_i2c_read(struct lsm330_acc_data *acc,
				u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc, u8 * buf,
								int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_update(struct lsm330_acc_data *acc,
				u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1] = { reg_address };
	u8 wrbuf[2] = { reg_address , 0x00 };

	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_i2c_read(acc, rdbuf, 1);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[1] = updated_val;
		err = lsm330_acc_i2c_write(acc, wrbuf, 1);
	}
	return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
	int i;
	int err = -1;
	u8 buf[17];

	dev_dbg(&acc->client->dev, "hw init start\n");

	buf[0] = LSM330_WHO_AM_I;
	err = lsm330_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
		dev_warn(&acc->client->dev,
			"Error reading WHO_AM_I: is device "
			"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_LSM330_ACC) {
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%02x,"
		" Replies: 0x%02x\n", WHOAMI_LSM330_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
	buf[1] = acc->resume_state[RES_LSM330_LC_L];
	buf[2] = acc->resume_state[RES_LSM330_LC_H];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_1);
	buf[1] = acc->resume_state[RES_LSM330_TIM4_1];
	buf[2] = acc->resume_state[RES_LSM330_TIM3_1];
	buf[3] = acc->resume_state[RES_LSM330_TIM2_1_L];
	buf[4] = acc->resume_state[RES_LSM330_TIM2_1_H];
	buf[5] = acc->resume_state[RES_LSM330_TIM1_1_L];
	buf[6] = acc->resume_state[RES_LSM330_TIM1_1_H];
	buf[7] = acc->resume_state[RES_LSM330_THRS2_1];
	buf[8] = acc->resume_state[RES_LSM330_THRS1_1];
	err = lsm330_acc_i2c_write(acc, buf, 8);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_SA_1);
	buf[1] = acc->resume_state[RES_LSM330_SA_1];
	buf[2] = acc->resume_state[RES_LSM330_MA_1];
	buf[3] = acc->resume_state[RES_LSM330_SETT_1];
	err = lsm330_acc_i2c_write(acc, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_TIM4_2);
	buf[1] = acc->resume_state[RES_LSM330_TIM4_2];
	buf[2] = acc->resume_state[RES_LSM330_TIM3_2];
	buf[3] = acc->resume_state[RES_LSM330_TIM2_2_L];
	buf[4] = acc->resume_state[RES_LSM330_TIM2_2_H];
	buf[5] = acc->resume_state[RES_LSM330_TIM1_2_L];
	buf[6] = acc->resume_state[RES_LSM330_TIM1_2_H];
	buf[7] = acc->resume_state[RES_LSM330_THRS2_2];
	buf[8] = acc->resume_state[RES_LSM330_THRS1_2];
	buf[9] = acc->resume_state[RES_LSM330_DES_2];
	buf[10] = acc->resume_state[RES_LSM330_SA_2];
	buf[11] = acc->resume_state[RES_LSM330_MA_2];
	buf[12] = acc->resume_state[RES_LSM330_SETT_2];
	err = lsm330_acc_i2c_write(acc, buf, 12);
	if (err < 0)
		goto err_resume_state;

	/*	state program 1 */
	buf[0] = (I2C_AUTO_INCREMENT | LSM330_STATEPR1);
	for (i = 1; i <= LSM330_STATE_PR_SIZE; i++) {
		buf[i] = acc->resume_stmach_program1[i-1];
		pr_debug("i=%d,sm pr1 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lsm330_acc_i2c_write(acc, buf, LSM330_STATE_PR_SIZE);
	if (err < 0)
		goto err_resume_state;

	/*	state program 2 */
	buf[0] = (I2C_AUTO_INCREMENT | LSM330_STATEPR2);
	for(i = 1; i <= LSM330_STATE_PR_SIZE; i++){
		buf[i] = acc->resume_stmach_program2[i-1];
		pr_debug("i=%d,sm pr2 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lsm330_acc_i2c_write(acc, buf, LSM330_STATE_PR_SIZE);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG5);
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG5];
	buf[2] = acc->resume_state[RES_LSM330_CTRL_REG6];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG1);
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG1];
	buf[2] = acc->resume_state[RES_LSM330_CTRL_REG2];
	buf[3] = acc->resume_state[RES_LSM330_CTRL_REG3];
	err = lsm330_acc_i2c_write(acc, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (LSM330_CTRL_REG4);
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG4];
	err = lsm330_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	dev_dbg(&acc->client->dev, "%s: hw init done\n", LSM330_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
	int err;

	err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,
					LSM330_ODR_MASK, LSM330_PM_OFF);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	dev_dbg(&acc->client->dev, "%s:polling:%d\n",
					__func__, acc->enable_polling);

	if (acc->pdata->power_off) {
		if (!acc->enable_polling) {
			lsm330_acc_set_drdyint_register_values(acc, 0);
			lsm330_acc_drdyint(acc);
		}
		if (acc->pdata->gpio_int1)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off(acc->client);
		acc->hw_initialized = 0;
	}

	dev_dbg(&acc->client->dev, "%s:hw_initialized:%d\n",
					__func__, acc->hw_initialized);

	if (acc->hw_initialized) {
		if (!acc->enable_polling) {
			lsm330_acc_set_drdyint_register_values(acc, 0);
			lsm330_acc_drdyint(acc);
		}
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on(acc->client);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
	}
	dev_dbg(&acc->client->dev, "%s:hw_initialized:%d\n",
					__func__, acc->hw_initialized);


	if (!acc->hw_initialized) {
		err = lsm330_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lsm330_acc_device_power_off(acc);
			return err;
		}
	}

	dev_dbg(&acc->client->dev, "%s:polling:%d\n",
					__func__, acc->enable_polling);

	if (acc->hw_initialized) {
		if (!acc->enable_polling) {
			lsm330_acc_set_drdyint_register_values(acc, 1);
			lsm330_acc_drdyint(acc);
		}
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

#if ENABLE_SIGNIFICANT_MOTION > 0
static void lsm330_acc_signMotion_interrupt_action(struct lsm330_acc_data *status)
{
	input_report_abs(status->input_dev, ABS_SIGN_MOTION, 1);
	input_sync(status->input_dev);
	input_report_abs(status->input_dev, ABS_SIGN_MOTION, 0);
	input_sync(status->input_dev);
	pr_debug("%s: sign Motion event\n", LSM330_ACC_DEV_NAME);
	atomic_set(&status->sign_mot_enabled, 0);
}
#endif

static irqreturn_t lsm330_acc_isr1(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	pr_debug("%s: isr1 queued\n", LSM330_ACC_DEV_NAME);
	acc->timestamp = ktime_get_boottime();
	acc->acc_wkp_flag = 1;
	wake_up_interruptible(&acc->acc_wq);
	return IRQ_HANDLED;
}

static irqreturn_t lsm330_acc_isr2(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	pr_debug("%s: isr2 queued\n", LSM330_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{

	int err = -1;
	u8 rbuf[2], status;
	struct lsm330_acc_data *acc;

	acc = container_of(work, struct lsm330_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_int_source(acc); */
	pr_debug("%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);

	rbuf[0] = LSM330_INTERR_STAT;
	err = lsm330_acc_i2c_read(acc, rbuf, 1);
	pr_debug("%s: INTERR_STAT_REG: 0x%02x\n",
				LSM330_ACC_DEV_NAME, rbuf[0]);
	status = rbuf[0];
	if (status & LSM330_STAT_INTSM1_BIT) {
		pr_debug("%s: SM1 interrupt\n",
				LSM330_ACC_DEV_NAME);
	/* mandatory to unlatch SM2 interrupt */
		rbuf[0] = LSM330_OUTS_1;
		err = lsm330_acc_i2c_read(acc, rbuf, 1);
		pr_debug("%s: OUTS_1: 0x%02x\n",
				LSM330_ACC_DEV_NAME, rbuf[0]);

	}
	if (status & LSM330_STAT_INTSM2_BIT) {
		pr_debug("%s: SM2 interrupt\n",
				LSM330_ACC_DEV_NAME);
#if ENABLE_SIGNIFICANT_MOTION > 0
		lsm330_acc_signMotion_interrupt_action(acc);
#endif
	/* mandatory to unlatch SM2 interrupt */
		rbuf[0] = LSM330_OUTS_2;
		err = lsm330_acc_i2c_read(acc, rbuf, 1);
		pr_debug("%s: OUTS_2: 0x%02x\n",
					LSM330_ACC_DEV_NAME, rbuf[0]);
	}
	pr_debug("%s: IRQ1 served\n", LSM330_ACC_DEV_NAME);
	enable_irq(acc->irq1);
	pr_debug("%s: IRQ1 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static void lsm330_acc_irq2_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	acc = container_of(work, struct lsm330_acc_data, irq2_work);
	pr_debug("%s: IRQ2 triggered\n", LSM330_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_stat_source(acc); */
	/* ; */
	pr_debug("%s: IRQ2 served\n", LSM330_ACC_DEV_NAME);
	enable_irq(acc->irq2);
	pr_debug("%s: IRQ2 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,
		u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
	u8 config[2] = {0};
	u8 init_val, updated_val;
	int err;
	int step = 0;

	config[0] = reg_address;
	err = lsm330_acc_i2c_read(acc, config, 1);
	if (err < 0)
		goto error;
	init_val = config[0];
	acc->resume_state[resume_index] = init_val;
	step = 1;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	config[0] = reg_address;
	config[1] = updated_val;
	err = lsm330_acc_i2c_write(acc, config, 1);
	if (err < 0)
		goto error;
	acc->resume_state[resume_index] = updated_val;

	return err;
	error:
		dev_err(&acc->client->dev,
			"register 0x%02x update failed at step %d, error: %d\n",
				config[0], step, err);
	return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc,
								u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;
	u8 sigmot_threshold;
	u8 init_val, updated_val;

	switch (new_fs_range) {
	case LSM330_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		sigmot_threshold = THRS1_2_02G;
		break;
	case LSM330_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		sigmot_threshold = THRS1_2_04G;
		break;
	case LSM330_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		sigmot_threshold = THRS1_2_06G;
		break;
	case LSM330_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		sigmot_threshold = THRS1_2_08G;
		break;
	case LSM330_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		sigmot_threshold = THRS1_2_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}
	/* Updates configuration register 5,
	* which contains odr range setting if device is enabled,
	* otherwise updates just RES_CTRL5 for when it will */
	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,
			LSM330_ACC_FS_MASK, new_fs_range, RES_LSM330_CTRL_REG5);
		if (err < 0) {
			dev_err(&acc->client->dev, "update g range failed\n");
			return err;
		} else
			acc->sensitivity = sensitivity;

#if ENABLE_SIGNIFICANT_MOTION > 0
		err = lsm330_acc_register_masked_update(acc, LSM330_THRS1_2,
			0xFF, sigmot_threshold, RES_LSM330_THRS1_2);
		if (err < 0)
			dev_err(&acc->client->dev, "update sign motion theshold"
								" failed\n");
		return err;
#endif
	} else {
		init_val = acc->resume_state[RES_LSM330_CTRL_REG5];
		updated_val = ((LSM330_ACC_FS_MASK & new_fs_range) | ((~LSM330_ACC_FS_MASK) & init_val));
		acc->resume_state[RES_LSM330_CTRL_REG5] = updated_val;

#if ENABLE_SIGNIFICANT_MOTION > 0
		acc->resume_state[RES_LSM330_THRS1_2] = sigmot_threshold;
#endif
		return 0;
	}

	return err;
}


static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 new_odr;
	u8 updated_val;
	u8 init_val;
	u8 mask = LSM330_ODR_MASK;
	int match_odr = 0;

#if ENABLE_SIGNIFICANT_MOTION > 0
	if (poll_interval_ms == 0)
		poll_interval_ms = 10;
#endif

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
		if (lsm330_acc_odr_table[i].cutoff_ms == poll_interval_ms) {
				match_odr = 1;
				break;
		}
	}

	if (match_odr && acc->use_interrupt) {
		/*found the exact match sampling rate use the interrupt mode*/
		dev_dbg(&acc->client->dev,
			"%s:poll_delay matched for int generation\n",
				__func__);
		new_odr = lsm330_acc_odr_table[i].mask;
		acc->enable_polling = 0;
	} else {
		/*intoption not available or match_odr is not avaiable , run the
		sensor at maz ODR and poll on the required frequency to get the
		accurate physical sample in polling mode*/
		dev_dbg(&acc->client->dev,
			"%s:poll_delay *not* matched for int generation\n",
			__func__);
		acc->enable_polling = 1;
		new_odr = LSM330_ODR1600;
	}

	/* Updates configuration register 4,
	* which contains odr range setting if device is enabled,
	* otherwise updates just RES_CTRL4 for when it will */
	if (atomic_read(&acc->enabled)) {
		err = lsm330_acc_register_masked_update(acc,
			LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,
							RES_LSM330_CTRL_REG4);
		if (err < 0) {
			dev_err(&acc->client->dev, "update odr failed\n");
			return err;
		}
		acc->ktime_acc = ktime_set(poll_interval_ms / 1000,
					MS_TO_NS(poll_interval_ms % 1000));
	} else {
		init_val = acc->resume_state[RES_LSM330_CTRL_REG4];
		updated_val = ((mask & new_odr) | ((~mask) & init_val));
		acc->resume_state[RES_LSM330_CTRL_REG4] = updated_val;
		return 0;
	}

	return err;
}


static int lsm330_acc_get_data(struct lsm330_acc_data *acc, int *xyz)
{

	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	if (acc->enable_polling)
		acc->timestamp = ktime_get_boottime();

	acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
	err = lsm330_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	pr_debug("%s read x=%d, y=%d, z=%d\n",
			LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

	return err;
}

static void lsm330_acc_report_values(struct lsm330_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_event(acc->input_dev, EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(acc->timestamp).tv_sec);
	input_event(acc->input_dev, EV_SYN, SYN_TIME_NSEC,
				ktime_to_timespec(acc->timestamp).tv_nsec);
	input_sync(acc->input_dev);
}

static void lsm330_acc_polling_manage(struct lsm330_acc_data *acc)
{
	if ((acc->enable_polling)) {
		if (atomic_read(&acc->enabled)) {
				hrtimer_start(&acc->hr_timer_acc,
					acc->ktime_acc, HRTIMER_MODE_REL);
		} else
			hrtimer_cancel(&acc->hr_timer_acc);
	}
}

static int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		lsm330_acc_polling_manage(acc);
	}

	return 0;
}

static int lsm330_acc_disable(struct lsm330_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		lsm330_acc_polling_manage(acc);
		lsm330_acc_device_power_off(acc);
	}

	return 0;
}



static ssize_t attr_get_enable_polling(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->enable_polling;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_disable_IntLine(
				struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long disable;

	if (kstrtoul(buf, 10, &disable))
		return -EINVAL;

	mutex_lock(&acc->lock);
	if (disable)
		acc->use_interrupt = 0;
	else
		acc->use_interrupt = 1;
	mutex_unlock(&acc->lock);

	return size;
}


static ssize_t attr_set_enable_polling(struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long enable;

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;
	mutex_lock(&acc->lock);
	if (enable)
		acc->enable_polling = 1;
	else
		acc->enable_polling = 0;
	mutex_unlock(&acc->lock);
	lsm330_acc_polling_manage(acc);
	return size;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = lsm330_acc_update_odr(acc, interval_ms);
	if (err >= 0)
		acc->pdata->poll_interval = interval_ms;
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LSM330_ACC_G_2G:
		range = 2;
		break;
	case LSM330_ACC_G_4G:
		range = 4;
		break;
	case LSM330_ACC_G_6G:
		range = 6;
		break;
	case LSM330_ACC_G_8G:
		range = 8;
		break;
	case LSM330_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LSM330_ACC_G_2G;
			break;
		case 4:
			range = LSM330_ACC_G_4G;
			break;
		case 6:
			range = LSM330_ACC_G_6G;
			break;
		case 8:
			range = LSM330_ACC_G_8G;
			break;
		case 16:
			range = LSM330_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_update_fs_range(acc, range);
	if (err >= 0)
		acc->pdata->fs_range = range;
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_acc_enable(acc);
	else
		lsm330_acc_disable(acc);

	return size;
}

static int lsm330_acc_state_progrs_enable_control(
				struct lsm330_acc_data *acc, u8 settings)
{
	u8 val1, val2;
	int err = -1;
	settings = settings & 0x03;

	switch ( settings ) {
	case LSM330_SM1_DIS_SM2_DIS:
		val1 = LSM330_SM1_EN_OFF;
		val2 = LSM330_SM2_EN_OFF;
		break;
	case LSM330_SM1_DIS_SM2_EN:
		val1 = LSM330_SM1_EN_OFF;
		val2 = LSM330_SM2_EN_ON;
		break;
	case LSM330_SM1_EN_SM2_DIS:
		val1 = LSM330_SM1_EN_ON;
		val2 = LSM330_SM2_EN_OFF;
		break;
	case LSM330_SM1_EN_SM2_EN:
		val1 = LSM330_SM1_EN_ON;
		val2 = LSM330_SM2_EN_ON;
		break;
	default :
		pr_err("invalid state program setting : 0x%02x\n",settings);
		return err;
	}
	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG1, LSM330_SM1_EN_MASK, val1,
							RES_LSM330_CTRL_REG1);
	if (err < 0 )
		return err;

	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG2, LSM330_SM2_EN_MASK, val2,
							RES_LSM330_CTRL_REG2);
	if (err < 0 )
			return err;

#if ENABLE_SIGNIFICANT_MOTION > 0
	if (val2 == LSM330_SM2_EN_ON)
		atomic_set(&acc->sign_mot_enabled, 1);
#endif

	pr_debug("state program setting : 0x%02x\n", settings);


	return err;
}

static ssize_t attr_set_enable_state_prog(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int err = -1;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;


	if ( val < 0x00 || val > LSM330_SM1_EN_SM2_EN){
		pr_warn("invalid state program setting, val: %ld\n",val);
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_state_progrs_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;
	return size;
}

static ssize_t attr_get_enable_state_prog(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	u8 val, val1 = 0, val2 = 0, config[2];
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);


	config[0] = LSM330_CTRL_REG1;
	lsm330_acc_i2c_read(acc, config, 1);
	val1 = (config[0] & LSM330_SM1_EN_MASK);

	config[0] = LSM330_CTRL_REG2;
	lsm330_acc_i2c_read(acc, config, 1);
	val2 = ((config[0] & LSM330_SM2_EN_MASK) << 1);

	val = (val1 | val2);

	mutex_unlock(&acc->lock);
	return sprintf(buf, "0x%02x\n", val);
}




#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lsm330_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lsm330_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}

#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0666, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(enable_polling, 0666, attr_get_enable_polling, attr_set_enable_polling),
	__ATTR(enable_state_prog, 0666, attr_get_enable_state_prog,
						attr_set_enable_state_prog),
#ifdef DEBUG
	__ATTR(disable_interrupt, 0666, NULL, attr_set_disable_IntLine),
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct lsm330_acc_data *acc)
{
	int i;

#ifdef CUSTOM_SYSFS_PATH
	acc->acc_class = class_create(THIS_MODULE, CUSTOM_SYSFS_CLASS_NAME_ACC);
	if (acc->acc_class == NULL)
		goto custom_class_error;

	acc->acc_dev = device_create(acc->acc_class, NULL, 0, "%s", "acc");
	if (acc->acc_dev == NULL)
		goto custom_class_error;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(acc->acc_dev, attributes + i))
			goto error;
#else
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(&acc->client->dev, attributes + i))
			goto error;
#endif
	return 0;

error:
	for ( ; i >= 0; i--)
#ifdef CUSTOM_SYSFS_PATH
		device_remove_file(acc->acc_dev, attributes + i);
#else
		device_remove_file(&acc->client->dev, attributes + i);
#endif

#ifdef CUSTOM_SYSFS_PATH
custom_class_error:
#endif
	dev_err(&acc->client->dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

int lsm330_acc_input_open(struct input_dev *input)
{
	struct lsm330_acc_data *acc = input_get_drvdata(input);

	return lsm330_acc_enable(acc);
}

void lsm330_acc_input_close(struct input_dev *dev)
{
	struct lsm330_acc_data *acc = input_get_drvdata(dev);

	lsm330_acc_disable(acc);
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
	int err;

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lsm330_acc_input_open;
	acc->input_dev->close = lsm330_acc_input_close;
	acc->input_dev->name = LSM330_ACC_DEV_NAME;

	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);

	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
#if ENABLE_SIGNIFICANT_MOTION > 0
	set_bit(ABS_SIGN_MOTION, acc->input_dev->absbit);
	input_set_abs_params(acc->input_dev, ABS_SIGN_MOTION, 0, 1, 0, 0);
#endif

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lsm330_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct lsm330_acc_data *stat = container_of(sensors_cdev,
			struct lsm330_acc_data, accel_cdev);
	int err;

	dev_dbg(&stat->client->dev,
				"enable %u client_data->enable %u\n",
				enable, atomic_read(&stat->enabled));

	mutex_lock(&stat->lock);
	if (enable)
			err = lsm330_acc_enable(stat);
		else
			err = lsm330_acc_disable(stat);
	mutex_unlock(&stat->lock);

	return err;

}

static int lsm330_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int interval_ms)
{
	struct lsm330_acc_data *stat = container_of(sensors_cdev,
			struct lsm330_acc_data, accel_cdev);
	int err;

	if (interval_ms < LSM330_SENSOR_ACC_MIN_POLL_PERIOD_MS)
		interval_ms = LSM330_SENSOR_ACC_MIN_POLL_PERIOD_MS;

	dev_dbg(&stat->client->dev, "%s sample_rate %u\n",
					__func__, interval_ms);

	mutex_lock(&stat->lock);
	if (atomic_read(&stat->enabled)) {
		if (stat->enable_polling) {
				hrtimer_cancel(&stat->hr_timer_acc);
		} else {
			lsm330_acc_set_drdyint_register_values(stat, 0);
			lsm330_acc_drdyint(stat);
		}
	}

	err = lsm330_acc_update_odr(stat, interval_ms);
	if (err >= 0)
		stat->pdata->poll_interval = interval_ms;
	stat->acc_delay_change = true;

	if (atomic_read(&stat->enabled)) {
		if (stat->enable_polling) {
			hrtimer_start(&stat->hr_timer_acc,
				stat->ktime_acc, HRTIMER_MODE_REL);
		} else {
			lsm330_acc_set_drdyint_register_values(stat, 1);
			lsm330_acc_drdyint(stat);
			if (stat->pdata->gpio_int1 >= 0)
				enable_irq(stat->irq1);
			if (stat->pdata->gpio_int2 >= 0)
				enable_irq(stat->irq2);
		}
	}
	mutex_unlock(&stat->lock);
	return 0;
}

static int lsm330_cdev_flush(struct sensors_classdev *sensors_cdev)
{
	return 0;
}

static int lsm330_cdev_set_latency(struct sensors_classdev *sensors_cdev,
					unsigned int max_latency)
{
	return 0;
}

static int lsm330_acc_power_init(struct i2c_client *client)
{
	int ret = 0;
	struct lsm330_acc_data *stat = i2c_get_clientdata(client);

	dev_dbg(&stat->client->dev, "power init");

	stat->vdd = regulator_get(&stat->client->dev, "vdd");
	if (IS_ERR(stat->vdd)) {
		ret = PTR_ERR(stat->vdd);
		dev_err(&stat->client->dev,
			"Regulator get failed(vdd) ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(stat->vdd) > 0) {
		ret = regulator_set_voltage(stat->vdd, 2400000,
					3600000);
		if (ret) {
			dev_err(&stat->client->dev,
				"regulator_count_voltages(vdd) failed  ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	stat->vddio = regulator_get(&stat->client->dev, "vddio");
	if (IS_ERR(stat->vddio)) {
		ret = PTR_ERR(stat->vddio);
		dev_err(&stat->client->dev,
			"Regulator get failed(vi2c) ret=%d\n", ret);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(stat->vddio) > 0) {
		ret = regulator_set_voltage(stat->vddio,
				0, 1800000);
		if (ret) {
			dev_err(&stat->client->dev,
				"regulator_count_voltages(vddio) failed ret=%d\n",
				ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(stat->vddio);

reg_vdd_set_vtg:
	if (regulator_count_voltages(stat->vdd) > 0)
			regulator_set_voltage(stat->vdd, 0,
						3600000);

reg_vdd_put:
	regulator_put(stat->vdd);

	return ret;
}

static void lsm330_acc_power_deinit(struct i2c_client *client)
{
	struct lsm330_acc_data *stat = i2c_get_clientdata(client);

	dev_dbg(&stat->client->dev, "power deinit");

	if (regulator_count_voltages(stat->vddio) > 0)
		regulator_set_voltage(stat->vddio, 0, 1800000);
	regulator_put(stat->vddio);

	if (regulator_count_voltages(stat->vdd) > 0)
		regulator_set_voltage(stat->vdd, 0, 3600000);
	regulator_put(stat->vdd);

	return ;
}


static int lsm330_acc_power_on(struct i2c_client *client)
{
	int ret = 0;
	struct lsm330_acc_data *stat = i2c_get_clientdata(client);

	dev_dbg(&stat->client->dev, "power on");

	if (!stat->power_enabled) {
		ret = regulator_enable(stat->vdd);
		if (ret) {
			dev_err(&stat->client->dev, "regulator_enable failed(vdd)");
			return ret;
		}
		ret = regulator_enable(stat->vddio);
		if (ret) {
			regulator_disable(stat->vdd);
			dev_err(&stat->client->dev, "regulator_enable failed(vddio)");
			return ret;
		}
		stat->power_enabled = true;
	} else {
		dev_dbg(&stat->client->dev,
			"already powered on");
	}
	return ret;
}
static int lsm330_acc_power_off(struct i2c_client *client)
{
	int ret = 0;
	struct lsm330_acc_data *stat = i2c_get_clientdata(client);

	dev_dbg(&stat->client->dev, "power off");

	if (stat->power_enabled) {
		ret = regulator_disable(stat->vdd);
		if (ret) {
			dev_err(&stat->client->dev, "regulator_disable failed(vdd)");
			return ret;
		}
		ret = regulator_disable(stat->vddio);
		if (ret) {
			regulator_enable(stat->vdd);
			dev_err(&stat->client->dev, "regulator_disable failed(vddio)");
			return ret;
		}
		stat->power_enabled = false;
	} else {
		dev_dbg(&stat->client->dev,
			"already powered off");
	}

	return ret;
}


static int acc_poll_thread(void *data)
{
	int xyz[3] = { 0 };
	int err;
	struct lsm330_acc_data *acc = data;
	u8 status;

	while (1) {
		wait_event_interruptible(acc->acc_wq,
					((acc->acc_wkp_flag != 0) ||
					kthread_should_stop()));
		acc->acc_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		mutex_lock(&acc->lock);
		if (acc->acc_delay_change) {
			if (acc->pdata->poll_interval <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
			acc->acc_delay_change = false;
		}

		if (acc->enable_polling) {
			err = lsm330_acc_get_data(acc, xyz);
			if (err < 0)
				dev_err(&acc->client->dev,
					"get_accelerometer_data failed\n");
			else
				lsm330_acc_report_values(acc, xyz);
			dev_dbg(&acc->client->dev, "using the poll option\n");
		} else {
			int err = -1;
			u8 rbuf[2];

			rbuf[0] = LSM330_INTERR_STAT;
			err = lsm330_acc_i2c_read(acc, rbuf, 1);
			dev_dbg(&acc->client->dev, "%s: INTERR_STAT_REG: 0x%02x\n",
						LSM330_ACC_DEV_NAME, rbuf[0]);
			status = rbuf[0];
			if (status & LSM330_STAT_INTDRDY_BIT) {
				dev_dbg(&acc->client->dev,
					"data ready interrupt generated\n");
				err = lsm330_acc_get_data(acc, xyz);
				enable_irq(acc->irq1);
				if (err < 0)
					dev_err(&acc->client->dev,
						"get_accelerometer_data failed\n");
				else
					lsm330_acc_report_values(acc, xyz);
			} else {
				enable_irq(acc->irq1);
			}
		}
		mutex_unlock(&acc->lock);
	}
	return 0;
}

enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm330_acc_data *acc;

	acc = container_of((struct hrtimer *)timer,
					struct lsm330_acc_data, hr_timer_acc);

	lsm330_acc_polling_manage(acc);

	acc->acc_wkp_flag = 1;
	wake_up_interruptible(&acc->acc_wq);

	return HRTIMER_NORESTART;
}

static int lsm330_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm330_acc_data *acc;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

	int err = -1;

	dev_info(&client->dev, "probe start.\n");

	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)){
			acc->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			acc->use_smbus = 0;
			goto exit_check_functionality_failed;
		}
	} else {
		acc->use_smbus = 0;
	}

	init_waitqueue_head(&acc->acc_wq);
	acc->acc_wkp_flag = 0;

	hrtimer_init(&acc->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->hr_timer_acc.function = &poll_function_read_acc;
	acc->acc_task = kthread_run(acc_poll_thread, acc, "lsm330_acc_sns");

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data == NULL) {
		default_lsm330_acc_pdata.gpio_int1 = int1_gpio;
		default_lsm330_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(acc->pdata, &default_lsm330_acc_pdata,
							sizeof(*acc->pdata));
		dev_info(&client->dev, "using default platform_data\n");
	} else {
		memcpy(acc->pdata, client->dev.platform_data,
							sizeof(*acc->pdata));
	}

	err = lsm330_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	acc->power_enabled = false;
	acc->pdata->init = lsm330_acc_power_init;
	acc->pdata->exit = lsm330_acc_power_deinit;
	acc->pdata->power_on = lsm330_acc_power_on;
	acc->pdata->power_off = lsm330_acc_power_off;

	if (acc->pdata->init) {
		err = acc->pdata->init(client);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	/*use the interrupt mode if INT1 or INT2 are available */
	acc->use_interrupt = 0;

	if (acc->pdata->gpio_int1 >= 0) {
		acc->use_interrupt = 1;
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		acc->use_interrupt = 1;
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lsm330_acc_set_init_register_values(acc);
	//init state program1 and params
	lsm330_acc_set_init_statepr1_param(acc);
	lsm330_acc_set_init_statepr1_inst(acc);
	//init state program2  and params
	lsm330_acc_set_init_statepr2_param(acc);
	lsm330_acc_set_init_statepr2_inst(acc);

	err = lsm330_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(acc);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LSM330_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

#ifdef CUSTOM_SYSFS_PATH
	dev_set_drvdata(acc->acc_dev, acc);
#endif
	lsm330_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

#if ENABLE_SIGNIFICANT_MOTION > 0
	atomic_set(&acc->sign_mot_enabled, 0);
#endif
	if (acc->pdata->gpio_int1 >= 0) {
		INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lsm330_acc_isr1,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lsm330_acc_isr2,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);

	acc->accel_cdev = lsm330_acc_cdev;
	acc->accel_cdev.delay_msec = LSM330_SENSOR_ACC_MIN_POLL_PERIOD_MS;
	acc->accel_cdev.sensors_enable = lsm330_cdev_enable;
	acc->accel_cdev.sensors_poll_delay = lsm330_cdev_poll_delay;
	acc->accel_cdev.fifo_reserved_event_count = 0;
	acc->accel_cdev.fifo_max_event_count = 0;
	acc->accel_cdev.sensors_set_latency = lsm330_cdev_set_latency;
	acc->accel_cdev.sensors_flush = lsm330_cdev_flush;

	err = sensors_classdev_register(&client->dev, &acc->accel_cdev);
	if (err) {
		dev_err(&client->dev,
			"create lsm330_acc class device file failed!\n");
		err = -EINVAL;
		goto err_destoyworkqueue2;
	}

	dev_info(&client->dev, "%s: probed\n", LSM330_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if (acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if (acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lsm330_acc_input_cleanup(acc);
err_power_off:
	lsm330_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit(client);
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	hrtimer_cancel(&acc->hr_timer_acc);
	kthread_stop(acc->acc_task);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM330_ACC_DEV_NAME);
	return err;
}

static int __devexit lsm330_acc_remove(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (acc->pdata->gpio_int1 >= 0) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	hrtimer_cancel(&acc->hr_timer_acc);
	kthread_stop(acc->acc_task);
	lsm330_acc_device_power_off(acc);
	lsm330_acc_input_cleanup(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit(client);

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lsm330_acc_resume(struct i2c_client *client)
{
	int err = 0;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend) {
		err = lsm330_acc_enable(acc);
	}

	return err;
}

static int lsm330_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);

#if ENABLE_SIGNIFICANT_MOTION > 0
	if (!atomic_read(&acc->sign_mot_enabled)) {
		err = lsm330_acc_disable(acc);
	} else {
		if (acc->on_before_suspend) {
			acc->enable_polling = 0;
			lsm330_acc_polling_manage(acc);
		}
	}
#else
	err = lsm330_acc_disable(acc);
#endif
	return err;
}
#else /* CONFIG_PM */
#define lsm330_acc_suspend	NULL
#define lsm330_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm330_acc_id[]
		= { { LSM330_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm330_acc_id);

static struct i2c_driver lsm330_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_ACC_DEV_NAME,
		  },
	.probe = lsm330_acc_probe,
	.remove = __devexit_p(lsm330_acc_remove),
	.suspend = lsm330_acc_suspend,
	.resume = lsm330_acc_resume,
	.id_table = lsm330_acc_id,
};

static int __init lsm330_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", LSM330_ACC_DEV_NAME);
	return i2c_add_driver(&lsm330_acc_driver);
}

static void __exit lsm330_acc_exit(void)
{
	pr_info("%s accelerometer driver exit\n", LSM330_ACC_DEV_NAME);
	i2c_del_driver(&lsm330_acc_driver);
	return;
}

module_init(lsm330_acc_init);
module_exit(lsm330_acc_exit);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

