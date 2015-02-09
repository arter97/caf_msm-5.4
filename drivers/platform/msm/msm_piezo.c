/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>


#define PIEZO_DRIVER_NAME "piezo"
#define PIEZO_COMMAND_ON  1
#define PIEZO_COMMAND_OFF 0

/* SW Interface document (SWI_80-NC832) explains all the definitions below */
#define SOURCE_CLOCK_FREQUENCY 4800000
#define MAX_TARGET_FREQUENCY   6000

#define PERIPH_SS_PDM_BASE    0xF9BFE040

/* The following address are for M, N and D values for MND counter */
#define GP_MN_CLK_MDIV_OFFSET  0x0C /* 0xF9BFE04C */
#define GP_MN_CLK_NDIV_OFFSET  0x10 /* 0xF9BFE050 */
#define GP_MN_CLK_DUTY_OFFSET  0x14 /* 0xF9BFE054 */

#define M_VALUE_MASK 0x1FF /* 9 bits */
#define N_VALUE_MASK 0x1FFFF /* 13 bits */
#define D_VALUE_MASK 0x1FFFF /* 13 bits */

/* Clock Registers to access to PERIPH_SS_PDM */
#define GCC_CC_PHYS    0xFC400000
#define GCC_CC_SIZE    SZ_16K

#define GCC_PDM_AHB_CBCR 0x0CC4
#define GCC_PDM_AHB_CLK_OFF_BIT 0x80000000
#define GCC_PDM_AHB_CLK_ENABLE_BIT 0x00000001

#define GCC_PDM_XO4_CBCR 0x0CC8
#define GCC_PDM_XO4_CLK_OFF_BIT 0x80000000
#define GCC_PDM_XO4_CLK_DIV_BIT 0x00030000
#define GCC_PDM_XO4_CLK_ENABLE_BIT 0x00000001

#define PERIPH_SS_PDM_TCXO4_CLK_DISABLE 0x00000080

/*
 * This module exposes the following attributes at /sys/kernel/piezo
 * Frequency, Duration and Duty cycle values used to calculate M and N values
 * These attributes are grouped under class attributes
 */
static struct kobject *piezo_kobj;

/* Global variables to store m, n, d register values */
static uint m_value;
static uint n_value;
static uint d_value;
static uint gp_mn_gpio;

/* Virtual address of Pheriphral SS PDM base */
static void __iomem *virtual_reg_base;
static void __iomem *virtual_clk_reg_base;

/* Used for debugfs /sys/kernel/debug/piezo-debug */
struct dentry *debugfs_dir;
struct dentry *debugfs_m;
struct dentry *debugfs_n;
struct dentry *debugfs_d;

/* User space input */
static uint frequency; /* in Hz */
static uint duration; /* in ms */
static uint duty_cycle; /* in percentage */
static uint command; /* on/off */

/* Delayed work definitions*/
static void piezo_work_queue_execute(struct work_struct *w);

static struct workqueue_struct *piezo_wq;
static DECLARE_DELAYED_WORK(piezo_clk_off_work, piezo_work_queue_execute);
static bool wq_pending;

static void piezo_mnd_finder(void)
{
	/* Functions to calculate M:N/D(finder) and set registers
	 * The followings are fixed
	 * Number of bits in M : 9 bits - Max 1FF, 511
	 * Number of bits in N : 13 bits - Max 1FFFF, 8191
	 * Number of bits in D : 13 bits - Max 1FFFF, 8191
	 * M:N/D counter mode : Dual Edge
	 * Source frequency : 19.2 MHz/4 = 4.8 MHz
	 * Pre-Divider : 1
	 * Target Clock Frequency : 1 KHz ~ 6 KHz
	 */

	pr_debug(PIEZO_DRIVER_NAME" freq : %d, duration :%d, duty_cycle :%d \n",
		frequency, duration, duty_cycle);

	/* With the assumtion above, M value never changes. So it is fixed to 1 */
	m_value = 1;

	/* Caculate N value. It is the value of source frequency divided by the
	 *target frequency
	 */
	if (frequency != 0) {
		if (frequency >  MAX_TARGET_FREQUENCY)
			frequency = MAX_TARGET_FREQUENCY;
		n_value = SOURCE_CLOCK_FREQUENCY / frequency;
	} else {
		pr_err(PIEZO_DRIVER_NAME" invalid frequency value \n");
	}

	/* Calculate D value. Duty cycle is in % */
	if (duty_cycle != 0)
		d_value = (n_value * duty_cycle) / 100;
	else
		pr_err(PIEZO_DRIVER_NAME" invalid duty_cycle value \n");

	pr_debug(PIEZO_DRIVER_NAME" m : %d, n : %d, d: %d \n",
			m_value, n_value, d_value);
}

static void piezo_reg_clk_on(void)
{
	u32 m_reg_val, n_reg_val, d_reg_val, tcxo_pdm_ctl_val;
	u32 __iomem *reg;
	uint bit_val;

	/* M, N and D value has assigned bits. So we need to mask the value
	 * M value has 9 bits assgined
	 * N value has 13 bits assgined but the value has to be -(N-M)
	 * D is number of low cycles and 13 bits assgined
	 * So the required bit operations are performed below
	 */
	m_reg_val = (M_VALUE_MASK & m_value);
	n_reg_val = ~(n_value - m_value);
	n_reg_val = N_VALUE_MASK & n_reg_val;
	d_reg_val = (D_VALUE_MASK & d_value);

	pr_debug(PIEZO_DRIVER_NAME" reg val m : %#x, n : %#x, d: %#x \n",
			m_reg_val, n_reg_val, d_reg_val);

	/* check TCXO_CLK_DISABLE
	 * If bit 7 is set to 1 ( which means DISABLED ), we will need
	 * to turn it on
	 */
	reg = virtual_reg_base;
	tcxo_pdm_ctl_val = readl_relaxed(reg);
	pr_debug(PIEZO_DRIVER_NAME" read pdm reg val TCXO_CLK_DISABLE: %#x \n",
			tcxo_pdm_ctl_val);

	bit_val = tcxo_pdm_ctl_val & PERIPH_SS_PDM_TCXO4_CLK_DISABLE;
	if (bit_val) {
		tcxo_pdm_ctl_val =
				(tcxo_pdm_ctl_val & ~PERIPH_SS_PDM_TCXO4_CLK_DISABLE);
		pr_debug(PIEZO_DRIVER_NAME" read pdm change TCXO_CLK_DISABLE:%#x\n",
			tcxo_pdm_ctl_val);
		writel_relaxed(tcxo_pdm_ctl_val, reg);
	}

	reg = virtual_reg_base + GP_MN_CLK_MDIV_OFFSET;
	writel_relaxed(m_reg_val, reg);
	reg = virtual_reg_base + GP_MN_CLK_NDIV_OFFSET;
	writel_relaxed(n_reg_val, reg);
	reg = virtual_reg_base + GP_MN_CLK_DUTY_OFFSET;
	writel_relaxed(d_reg_val, reg);
}

static void piezo_reg_clk_off(void)
{
	u32 tcxo_pdm_ctl_val;
	u32 __iomem *reg;

	/* Set TCXO_CLK_DISABLE to 1 to disable GP_MN signal */
	reg = virtual_reg_base;
	tcxo_pdm_ctl_val = readl_relaxed(reg);
	tcxo_pdm_ctl_val = (tcxo_pdm_ctl_val & ~PERIPH_SS_PDM_TCXO4_CLK_DISABLE)
						| PERIPH_SS_PDM_TCXO4_CLK_DISABLE;

	pr_debug(PIEZO_DRIVER_NAME" reg_clk_off : val: %#x \n",
				tcxo_pdm_ctl_val);
	writel_relaxed(tcxo_pdm_ctl_val, reg);
}

static void __init piezo_config_pdm_clk(void)
{
	u32 reg_val;
	u32 __iomem *reg;

	/* Enable PDM_AHB_CLK
	 * Bit 31 - CLK_OFF : 1
	 * Bit 0  - CLK_ENABLE : 1
	 */
	reg = virtual_clk_reg_base + GCC_PDM_AHB_CBCR;
	reg_val = readl_relaxed(reg);
	pr_debug(PIEZO_DRIVER_NAME" Read Reg PDM_AHB_CLK : %#x \n", reg_val);
	reg_val = (reg_val & (~GCC_PDM_AHB_CLK_OFF_BIT)) | GCC_PDM_AHB_CLK_OFF_BIT;
	reg_val = (reg_val & (~GCC_PDM_AHB_CLK_ENABLE_BIT))
				| GCC_PDM_AHB_CLK_ENABLE_BIT;
	writel_relaxed(reg_val, reg);
	pr_debug(PIEZO_DRIVER_NAME" Write Reg PDM_AHB_CLK : %#x \n", reg_val);

	/* Enable GCC_PDM_XO4_CBCR
	 * Bit 31 - CLK_OFF : 0
	 * Bit 17:16  - CLK_DIV : 11 DIV-4
	 * Bit 0  - CLK_ENABLE : 1
	 */
	reg = virtual_clk_reg_base + GCC_PDM_XO4_CBCR;
	reg_val = readl_relaxed(reg);
	pr_debug(PIEZO_DRIVER_NAME" Read Reg GCC_PDM_XO4 : %#x \n", reg_val);
	reg_val = reg_val & ~(GCC_PDM_XO4_CLK_OFF_BIT);
	reg_val = (reg_val & (~GCC_PDM_XO4_CLK_DIV_BIT)) | GCC_PDM_XO4_CLK_DIV_BIT;
	reg_val = (reg_val & (~GCC_PDM_XO4_CLK_ENABLE_BIT))
				| GCC_PDM_XO4_CLK_ENABLE_BIT;
	writel_relaxed(reg_val, reg);
	pr_debug(PIEZO_DRIVER_NAME" Write Reg GCC_PDM_XO4 : %#x \n", reg_val);
}

static void
piezo_work_queue_execute(struct work_struct *w)
{
	pr_debug(PIEZO_DRIVER_NAME" work queue entered \n");

	/* Turn off the clocks now - update register */
	piezo_reg_clk_off();

	/* Now update sysfs and let user space know the command is done */
	wq_pending = false;
	command = 0;
	sysfs_notify(piezo_kobj, NULL, "command");

	pr_debug(PIEZO_DRIVER_NAME" sysfs command notify \n");
}

static ssize_t piezo_command_run(void)
{
	int status;

	if (command == PIEZO_COMMAND_ON) {
		piezo_mnd_finder();
		piezo_reg_clk_on();
	} else {
		status = 2;
		pr_err(PIEZO_DRIVER_NAME" cmd attr wrong value (%d) \n", command);
	}

	return status;
}


static ssize_t piezo_value_show(struct kobject *kobj,
								struct kobj_attribute *attr,
								char *buf)
{
	int var;

	if (strcmp(attr->attr.name, "frequency") == 0)
		var = frequency;
	else if (strcmp(attr->attr.name, "duration") == 0)
		var = duration;
	else if (strcmp(attr->attr.name, "duty_cycle") == 0)
		var = duty_cycle;
	else if (strcmp(attr->attr.name, "command") == 0)
		var = command;

	return scnprintf(buf, PAGE_SIZE, "%s: %d\n", attr->attr.name, var);
}

static ssize_t piezo_value_store(struct kobject *kobj,
								struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	ssize_t status = count;
	uint var;
	unsigned long work_delay;

	if (strcmp(attr->attr.name, "command") == 0) {
		sscanf(buf, "%d", &var);
		if(wq_pending && (var == PIEZO_COMMAND_ON)) {
			pr_debug("piezo : ON command work queue pending \n");
			status = -EBUSY;
		} else if (wq_pending && (var == PIEZO_COMMAND_OFF)) {
			pr_debug("piezo : OFF command work queue pending \n");
			piezo_reg_clk_off();
			cancel_delayed_work(&piezo_clk_off_work);
			wq_pending = false;
			command = var;
		} else {
			command = var;
			if (var == PIEZO_COMMAND_ON) {
				piezo_command_run();
				work_delay = msecs_to_jiffies(duration);
				pr_debug("piezo : starting work queue (%ld) \n", work_delay);
				queue_delayed_work(piezo_wq, &piezo_clk_off_work, work_delay);
				wq_pending = true;
			}
		}
	} else {
		sscanf(buf, "%d", &var);
		if (strcmp(attr->attr.name, "frequency") == 0)
			frequency = var;
		else if (strcmp(attr->attr.name, "duration") == 0)
			duration = var;
		else if (strcmp(attr->attr.name, "duty_cycle") == 0)
			duty_cycle = var;

		pr_debug("piezo : buf = %s,store %s = %d \n",
				buf, attr->attr.name, var);
	}

	return status;
}

static struct kobj_attribute frequency_attr =
	__ATTR(frequency, 0666, piezo_value_show, piezo_value_store);
static struct kobj_attribute duration_attr =
	__ATTR(duration, 0666, piezo_value_show, piezo_value_store);
static struct kobj_attribute duty_cycle_attr =
	__ATTR(duty_cycle, 0666, piezo_value_show, piezo_value_store);
static struct kobj_attribute command_attr =
	__ATTR(command, 0666, piezo_value_show, piezo_value_store);

static struct attribute *piezo_attrs[] = {
	&frequency_attr.attr,
	&duration_attr.attr,
	&duty_cycle_attr.attr,
	&command_attr.attr,
	NULL,
};

static struct attribute_group piezo_attr_group = {
	.attrs = piezo_attrs,
};

static void __init piezo_reg_iomap(void)
{
	virtual_reg_base = ioremap((u32)PERIPH_SS_PDM_BASE, SZ_16);
	if (!virtual_reg_base) {
		pr_err(PIEZO_DRIVER_NAME" error virtual_reg_base \n");
		panic("Unable to ioremap GP_MN memory!");
	}

	pr_debug(PIEZO_DRIVER_NAME" : virtual addr - %#x \n",
			(u32)virtual_reg_base);

	virtual_clk_reg_base = ioremap(GCC_CC_PHYS, GCC_CC_SIZE);
	if (!virtual_clk_reg_base) {
		pr_err(PIEZO_DRIVER_NAME" error virtual_clk_base \n");
		panic("Unable to ioremap clock memory!");
	}

	pr_debug(PIEZO_DRIVER_NAME" : virtual clk base addr - %#x \n",
			(u32)virtual_clk_reg_base);

}

static int piezo_driver_probe(struct platform_device *pdev)
{
	int result;
	struct resource *resource;
	void __iomem *devm_base;

	pr_debug(PIEZO_DRIVER_NAME " %s : pr_debug probe \n", __func__);

	resource =
			platform_get_resource_byname(pdev,IORESOURCE_MEM,"periph_ss_pdm");
	devm_base = devm_request_and_ioremap(&pdev->dev, resource);
	if (!devm_base)
		return -ENOMEM;

	pr_debug(PIEZO_DRIVER_NAME" %s : devm_base %#x \n", __func__,
			(u32)devm_base);

	/* Read default values of piezo control information from device tree */
	result = of_property_read_u32(pdev->dev.of_node,
			"qcom,piezo-frequency", &frequency);
	if (result)
		dev_err(&pdev->dev, "%s: qcom,piezo-frequency missing in DT node\n",
				__func__);

	result = of_property_read_u32(pdev->dev.of_node,
			"qcom,piezo-duration", &duration);
	if (result)
		dev_err(&pdev->dev, "%s: qcom,piezo-duration missing in DT node\n",
				__func__);

	result = of_property_read_u32(pdev->dev.of_node,
			"qcom,piezo-duty-cycle", &duty_cycle);
	if (result)
		dev_err(&pdev->dev, "%s: qti,piezo-frequency missing in DT node\n",
				__func__);

	result = of_property_read_u32(pdev->dev.of_node,
			"qcom,piezo-command", &command);
	if (result)
		dev_err(&pdev->dev, "%s: qti,piezo-frequency missing in DT node\n",
				__func__);

	result = of_property_read_u32(pdev->dev.of_node,
			"qcom,piezo-gp-mn-gpio", &gp_mn_gpio);
	if (result)
		dev_err(&pdev->dev, "%s: qcom,piezo-gp-mn-gpio missing in DT node\n",
				__func__);

	pr_debug(PIEZO_DRIVER_NAME" %s : pr_debug gp_mn_gpio : %d \n", __func__,
			gp_mn_gpio);

	/* To update GP_MN, registers, call ioremap to get the virtual address*/
	piezo_reg_iomap();

	piezo_config_pdm_clk();

	return result;
}

static int piezo_driver_exit(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id piezo_match_table[] = {
	{ .compatible = "qcom,msm-piezo" },
	{}
};

static struct platform_driver piezo_driver = {
	.probe = piezo_driver_probe,
	.remove = __devexit_p(piezo_driver_exit),
	.driver = {
		.name = "msm-piezo",
		.of_match_table = piezo_match_table,
		.owner = THIS_MODULE,
	},
};

static void piezo_init_debugfs(void)
{
	/* The following debug directory will be created at /sys/kernel/debug */
	debugfs_dir = debugfs_create_dir("piezo-debug", NULL);
	if (!debugfs_dir)
		goto error_root;

	debugfs_m = debugfs_create_u32("m_value", S_IRUGO, debugfs_dir, &m_value);
	if (!debugfs_m)
		goto error_m;

	debugfs_n = debugfs_create_u32("n_value", S_IRUGO, debugfs_dir, &n_value);
	if (!debugfs_m)
		goto error_n;

	debugfs_d = debugfs_create_u32("d_value", S_IRUGO, debugfs_dir, &d_value);
	if (!debugfs_m)
		goto error_d;

	return;

error_d:
	debugfs_remove(debugfs_dir);
	debugfs_remove(debugfs_m);
	debugfs_remove(debugfs_n);
	return;
error_n:
	debugfs_remove(debugfs_dir);
	debugfs_remove(debugfs_m);
	return;
error_m:
	debugfs_remove(debugfs_dir);
	return;
error_root:
	return;
}


static int __init piezo_init(void)
{
	int result;

	result = platform_driver_register(&piezo_driver);
	pr_debug(PIEZO_DRIVER_NAME" %s : driver result %d \n", __func__, result);

	piezo_wq = create_singlethread_workqueue("piezo_workqueue");
	if (!piezo_wq) {
		pr_err(PIEZO_DRIVER_NAME" error creating work queue\n");
		result = -ENOMEM;
	}

	piezo_kobj = kobject_create_and_add(PIEZO_DRIVER_NAME, kernel_kobj);

	if (piezo_kobj) {
		result = sysfs_create_group(piezo_kobj, &piezo_attr_group);
		if (result) {
			result = -1;
			pr_err(PIEZO_DRIVER_NAME" sysfs_create_group() failed\n");
			kobject_put(piezo_kobj);
		}
	} else {
		result = -ENOMEM;
	}

	piezo_init_debugfs();

	return result;
}

static void __exit piezo_exit(void)
{
	if (piezo_kobj) {
		kobject_put(piezo_kobj);
		kfree(piezo_kobj);
	}
	destroy_workqueue(piezo_wq);
	platform_driver_unregister(&piezo_driver);
}

module_init(piezo_init);
module_exit(piezo_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("piezobuzzer control driver based on user space input");
