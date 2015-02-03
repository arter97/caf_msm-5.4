/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c/i2c-qup.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/memory.h>
#include <linux/regulator/cpr-regulator.h>
#include <linux/regulator/fan53555.h>
#include <linux/regulator/onsemi-ncp6335d.h>
#include <linux/regulator/qpnp-regulator.h>
#include <linux/msm_tsens.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/board.h>
#include <mach/msm_bus.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/restart.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/socinfo.h>
#include <mach/board.h>
#include <mach/clk-provider.h>
#include <mach/msm_smd.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/msm_smem.h>
#include <linux/msm_thermal.h>
#include "board-dt.h"
#include "clock.h"
#include "platsmp.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#include "spm-regulator.h"
#include <linux/leds-lp5523.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-lp5521.h>
#include <linux/i2c/adp5588.h>




static	struct regulator *reg_lvs1_5523;
static	struct regulator *reg_lvs1_5521;
static	struct regulator *reg_lvs1_adp5588;


static struct lp5523_led_config lp5523_led_config[] = {
	{
		.chan_nr = 0,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 1,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 2,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 3,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 4,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 5,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 6,
		.led_current = 0,
		.max_current = 130,
	},
	{
		.chan_nr = 7,
		.led_current = 50,
		.max_current = 130,
	},
	{
		.chan_nr = 8,
		.led_current = 50,
		.max_current = 130,
	}
};

static struct lp5521_led_config lp5521_led_config[] = {
	{
		.name =	"red",
		.chan_nr = 0,
		.led_current = 50,
		.max_current = 200,
	},
	{
		.name = "green",
		.chan_nr = 1,
		.led_current = 50,
		.max_current = 200,
	},
	{
		.name = "blue",
		.chan_nr = 2,
		.led_current = 50,
		.max_current = 150,
	}
};


static int lp5523_setup(void)
{
	int rc;

	reg_lvs1_5523 = regulator_get(NULL,"vdd_led");
	rc = regulator_enable(reg_lvs1_5523);

	if (rc) {
		printk(" lp5523_enable- lvs1 fail\n");
	}

	return 0;
}

static void lp5523_release(void)
{
	int rc;

	rc = regulator_disable(reg_lvs1_5523);

	if (rc) {
		printk(" lp5523_release- lvs1 fail\n");
	}
}

static void lp5523_enable(bool state)
{

}

static struct lp5523_platform_data lp5523_platform_data = {
		.led_config     = lp5523_led_config,
		.num_channels   = ARRAY_SIZE(lp5523_led_config),
		.clock_mode     = LP5523_CLOCK_EXT,
		.setup_resources   = lp5523_setup,
		.release_resources = lp5523_release,
		.enable            = lp5523_enable,
};


static struct i2c_board_info lp5523_i2c_boardinfo[] __initdata = {
	{
	I2C_BOARD_INFO("lp55231", 0x33),
	.platform_data = &lp5523_platform_data,
	},
};

static int lp5521_setup(void)
{
	int rc;

	reg_lvs1_5521 = regulator_get(NULL,"vdd_led");
	rc = regulator_enable(reg_lvs1_5521);

	if (rc) {
		printk(" lp5521_enable- lvs1 fail\n");
	}

	return 0;
}

static void lp5521_release(void)
{
	int rc;

	rc = regulator_disable(reg_lvs1_5521);

	if (rc) {
		printk(" lp5521_release- lvs1 fail\n");
	}
}

static void lp5521_enable(bool state)
{

}

static u8 pattern_red[] = {
		0x40, 0xFF, 0x5F, 0x00, 0x40, 0x00, 0x5F, 0x00, 0xE0, 0x04, 0xE2, 0x00, 0x00, 0x00, 0xC0, 0x00,
		};
static u8 pattern_green[] = {
		0xE0, 0x80, 0x40, 0xFF, 0x5F, 0x00, 0x40, 0x00, 0x5F, 0x00, 0xE0, 0x08, 0x00, 0x00, 0xC0, 0x00,
		};
static u8 pattern_blue[] = {
		0xE1, 0x00, 0x40, 0xFF, 0x5F, 0x00, 0x40, 0x00, 0x5F, 0x00, 0xE0, 0x02, 0x00, 0x00, 0xC0, 0x00,
		};
static struct lp5521_led_pattern board_led_patterns[] = {
	{
		.r = pattern_red,
		.g = pattern_green,
		.b = pattern_blue,
		.size_r = ARRAY_SIZE(pattern_red),
		.size_g = ARRAY_SIZE(pattern_green),
		.size_b = ARRAY_SIZE(pattern_blue),
	},
};


#define LP5521_CONFIGS	( LP5521_CLOCK_INT | LP5521_PWRSAVE_EN | \
			LP5521_CP_MODE_AUTO | LP5521_R_TO_BATT )

static struct lp5521_platform_data lp5521_platform_data = {
		.led_config     = lp5521_led_config,
		.num_channels   = ARRAY_SIZE(lp5521_led_config),
		.clock_mode     = LP5521_CLOCK_INT,
		.setup_resources   = lp5521_setup,
		.release_resources = lp5521_release,
		.enable            = lp5521_enable,
		.update_config  = LP5521_CONFIGS,
		.patterns = board_led_patterns,
		.num_patterns = ARRAY_SIZE(board_led_patterns),
};

static struct i2c_board_info lp5521_i2c_boardinfo[] __initdata = {
{
	I2C_BOARD_INFO("lp5521", 0x32),
	.platform_data = &lp5521_platform_data,
	},
};


int adp5588_gpio_setup(struct i2c_client *client,
				int gpio, unsigned ngpio,
				void *context)
{
	int rc;

	reg_lvs1_adp5588= regulator_get(NULL,"vdd_led");
	rc = regulator_enable(reg_lvs1_adp5588);

	if (rc) {
		pr_err("reg_lvs1_adp5588 - lvs1 fail\n");
	}

	return rc;
}

int adp5588_gpio_teardown(struct i2c_client *client,
				int gpio, unsigned ngpio,
				void *context)
{
	int rc = 0;

	rc = regulator_disable(reg_lvs1_adp5588);

	if (rc) {
		pr_err("reg_lvs1_adp5588 - lvs1 fail\n");
	}

	return rc;
}


static struct adp5588_gpio_platform_data adp5588_gpio_data = {
	.gpio_start = 200,
	.pullup_dis_mask = 0,
	.setup = adp5588_gpio_setup,
	.teardown = adp5588_gpio_teardown,

};

static struct i2c_board_info adp5588_gpio_i2c_boardinfo[] __initdata = {
	{
	I2C_BOARD_INFO("adp5588-gpio", 0x34),
	.platform_data = &adp5588_gpio_data,
	},
};

static struct memtype_reserve msm8226_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm8226_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct of_dev_auxdata msm_hsic_host_adata[] = {
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	{}
};

static struct of_dev_auxdata msm8226_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	OF_DEV_AUXDATA("qcom,hsic-smsc-hub", 0, "msm_smsc_hub",
			msm_hsic_host_adata),

	{}
};

static struct reserve_info msm8226_reserve_info __initdata = {
	.memtype_reserve_table = msm8226_reserve_table,
	.paddr_to_memtype = msm8226_paddr_to_memtype,
};

static void __init msm8226_early_memory(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, msm8226_reserve_table);
}

static void __init msm8226_reserve(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8226_reserve_table);
	msm_reserve();
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8226_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	qpnp_regulator_init();
	spm_regulator_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8226_rumi_clock_init_data);
	else
		msm_clock_init(&msm8226_clock_init_data);
	msm_bus_fabric_init_driver();
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

void __init msm8226_init(void)
{
	struct of_dev_auxdata *adata = msm8226_auxdata_lookup;

	struct regulator *reg_l8;
	int rc;

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	if (socinfo_get_platform_type() == 5)
		qm8626_msm8226_init_gpiomux();
	else
		msm8226_init_gpiomux();

	board_dt_populate(adata);
	msm8226_add_drivers();

	if (socinfo_get_platform_type() == 5) {
		/* qm8626 need to keep L8 on */
		reg_l8 = regulator_get(NULL,"l8_keep_alive");
		rc = regulator_enable(reg_l8);
		if (rc) {
			printk(" msm8226_init - l8 fail\n");
		}
		/* qm8626 register the LED drivers LP5521/5523 with I2C */
		i2c_register_board_info( 1,lp5523_i2c_boardinfo , ARRAY_SIZE(lp5523_i2c_boardinfo) );
		i2c_register_board_info( 1,lp5521_i2c_boardinfo , ARRAY_SIZE(lp5521_i2c_boardinfo) );
		i2c_register_board_info( 1,adp5588_gpio_i2c_boardinfo , ARRAY_SIZE(adp5588_gpio_i2c_boardinfo) );
	}
}

static const char *msm8226_dt_match[] __initconst = {
	"qcom,msm8226",
	"qcom,msm8926",
	"qcom,apq8026",
	NULL
};

DT_MACHINE_START(MSM8226_DT, "Qualcomm MSM 8x26 / MSM 8x28 (Flattened Device Tree)")
	.map_io = msm_map_msm8226_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8226_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8226_dt_match,
	.reserve = msm8226_reserve,
	.init_very_early = msm8226_early_memory,
	.restart = msm_restart,
	.smp = &arm_smp_ops,
MACHINE_END
