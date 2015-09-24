/*
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <mach/board.h>
#include <video/msm_dba.h>
#include "msm_dba_internal.h"

#define MAX_SLAVE_DEVICES 0x8
#define SERIALIZER_REG_START_ADDR   0x00
#define SERIALIZER_REG_END_ADDR     0xF5
#define DESERIALIZER_REG_START_ADDR 0x00
#define DESERIALIZER_REG_END_ADDR   0xF5

struct ds90uh92x {
	struct msm_dba_device_info dev;
	struct i2c_client *i2c_client;
	int reset_gpio;
	int irq_gpio;
	int irq;
	u8 i2c_addr;
	u8 slave_ids[MAX_SLAVE_DEVICES];
	u32 num_of_slaves;
	struct completion ksv_rdy;
	struct completion hdcp_done;
	bool hdcp_sync_kickoff;
	bool remote_comm_en;
};
static int ds90uh92x_write_reg(struct msm_dba_device_info *dev,
			       u32 reg, u32 val)
{
	int rc = 0;
	struct ds90uh92x *ds90uh92x = NULL;

	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   (u8)(reg & 0xFF),
					   (u8)(val & 0xFF));
	if (rc)
		pr_err("%s: i2c write failed\n", __func__);

	return rc;
}

static int ds90uh92x_read_reg(struct msm_dba_device_info *dev,
			      u32 reg, u32 *val)
{
	int rc = 0;
	u8 byte_val = 0;
	struct ds90uh92x *ds90uh92x = NULL;

	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     (u8)(reg & 0xFF),
				     &byte_val,
				     1);
	if (rc)
		pr_err("%s: i2c write failed\n", __func__);
	else
		*val = (u32)byte_val;

	return rc;
}

static int ds90uh92x_toggle_pdb(struct ds90uh92x *ds90uh92x, bool on)
{
	int rc = 0;
	int output = (on) ? (1) : (0);

	rc = gpio_direction_output(ds90uh92x->reset_gpio, output);
	if (rc) {
		pr_err("[%s:%d] Failed to get gpio output=%d rc=%d", __func__,
		       ds90uh92x->dev.instance_id, output, rc);
		return rc;
	}

	usleep_range(4000, 6000);

	return rc;
}

static int ds90uh92x_power_on(struct msm_dba_device_info *dev, bool on)
{
	int rc = 0;
	struct ds90uh92x *ds90uh92x = NULL;

	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	if (!on)
		disable_irq(ds90uh92x->irq);

	rc = ds90uh92x_toggle_pdb(ds90uh92x, on);
	if (rc) {
		pr_err("[%s:%d] Failed to get gpio output=%d rc=%d", __func__,
		       dev->instance_id, on, rc);
		return rc;
	}

	dev->power_status = on;

	pr_debug("[%s:%d] Power status = %d", __func__, dev->instance_id,
					      dev->power_status);

	if (on) {
		/* Enable all interrupts */
		rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   0xc6,
						   0x3F);
		if (rc)
			pr_err("%s: failed to enable interrupts\n", __func__);

		enable_irq(ds90uh92x->irq);
	} else {
		/* After power off, need to reset remote device information.*/
		ds90uh92x->num_of_slaves = 0;
	}
	return rc;
}

static int ds90uh92x_video_on(struct msm_dba_device_info *dev,
			      struct msm_dba_video_cfg *cfg, bool on)
{
	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	dev->video_status = on;
	pr_debug("[%s:%d] Video status = %d", __func__, dev->instance_id,
					      dev->video_status);
	return 0;
}

static int ds90uh92x_handle_interrupts(struct msm_dba_device_info *dev,
				       u32 *mask)
{
	int rc = 0;
	struct ds90uh92x *ds90uh92x = NULL;
	u32 int_mask = 0;
	u8 val = 0;

	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     0xC7,
				     &val,
				     1);

	if (rc) {
		pr_err("[%s:%d]: failed to read isr\n", __func__,
						       dev->instance_id);
		*mask = 0;
		return rc;
	}

	pr_debug("[%s:%d] Int register = 0x%x", __func__,
						dev->instance_id, val);

	if (val & 0x2) {
		int_mask |= MSM_DBA_CB_HDCP_LINK_AUTHENTICATED;
		complete(&ds90uh92x->hdcp_done);
		dev->hdcp_status = true;
		if (!ds90uh92x->hdcp_sync_kickoff && dev->enc_on) {
			rc = msm_dba_helper_i2c_write_byte(
						   ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   0xC3,
						   BIT(2)
						   );
			if (rc) {
				pr_err("[%s:%d] i2c write failed\n", __func__,
							   dev->instance_id);
			}
		}
	}
	if (val & 0x4) {
		int_mask |= MSM_DBA_CB_HDCP_LINK_UNAUTHENTICATED;
		complete(&ds90uh92x->hdcp_done);
		dev->hdcp_status = false;
	}
	if (val & 0x20)
		int_mask |= MSM_DBA_CB_REMOTE_INT;

	if (val & BIT(3)) {
		if (ds90uh92x->hdcp_sync_kickoff) {
			complete(&ds90uh92x->ksv_rdy);
		} else {
			/* Kick off remaining HDCP sequence */
			rc =
			   msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
							 ds90uh92x->i2c_addr,
							 0xC3,
							 BIT(4));
			if (rc) {
				pr_err("[%s:%d] i2c write for HDCP failed\n",
				       __func__,  dev->instance_id);
			}
		}
	}
	*mask = int_mask;

	return rc;
}

static int ds90uh92x_unmask_interrupts(struct msm_dba_device_info *dev,
				       u32 mask)
{
	return 0;
}
static int ds90uh92x_hdcp_kickoff(struct ds90uh92x *ds90uh92x, u32 flags)
{
	int rc = 0;
	u8 reg_val;
	struct msm_dba_device_info *device = &ds90uh92x->dev;

	pr_debug("[%s:%d] HDCP kick off started", __func__,
						  device->instance_id);
	/*
	 * Setup HDCP_CFG register for configurations
	 *	1. Disable repeater mode
	 *	2. Use OESS mode
	 *	3. Encryption is cotrolled through register
	 *	4. AV Mute is not enabled
	 *	5. Receiver detected when there is a lock
	 */
	reg_val = 0x1 << 3;

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0xC2,
					   reg_val);

	if (rc) {
		pr_err("[%s:%d], i2c write failed for HDCP_CFG\n", __func__,
		       device->instance_id);
		goto fail;
	}

	rc = msm_dba_register_hdcp_monitor(device, true);
	if (rc) {
		pr_err("[%s:%d] failed to enable hdcp monitor\n", __func__,
		       device->instance_id);
		goto fail;
	}
	/*
	 * Kick off authentication and wait for controller to set the KSV RDY
	 * interrupt. Once this interrupt is received, the driver will set the
	 * KSV_VALID bit before authentication can proceed to the next step.
	 */

	INIT_COMPLETION(ds90uh92x->ksv_rdy);
	if (flags & MSM_DBA_ASYNC_FLAG)
		ds90uh92x->hdcp_sync_kickoff = false;
	else
		ds90uh92x->hdcp_sync_kickoff = true;

	reg_val = BIT(0);

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0xC3,
					   reg_val);

	if (rc) {
		pr_err("[%s:%d], i2c write failed for HDCP_CTL\n", __func__,
		       device->instance_id);
		goto fail;
	}

	if (flags & MSM_DBA_ASYNC_FLAG) {
		pr_debug("%s: Async kick off requested\n", __func__);
		goto fail;
	}

	pr_debug("[%s:%d] Wait for KSV RDY", __func__, device->instance_id);
	mutex_unlock(&device->dev_mutex);
	/* Wait for KSV RDY interrupt */
	rc = wait_for_completion_timeout(&ds90uh92x->ksv_rdy,
					 msecs_to_jiffies(100));
	mutex_lock(&device->dev_mutex);
	if (!rc) {
		pr_err("[%s:%d], Timeout waiting for KSV RDY\n", __func__,
		       device->instance_id);
		rc = -EIO;
		goto fail;
	} else {
		rc = 0;
	}

	INIT_COMPLETION(ds90uh92x->hdcp_done);

	reg_val = BIT(4);

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0xC3,
					   reg_val);

	if (rc) {
		pr_err("[%s:%d], i2c write failed for HDCP DONE\n", __func__,
		       device->instance_id);
		goto fail;
	}
	pr_debug("[%s:%d] Wait for HDCP DONE", __func__, device->instance_id);
	mutex_unlock(&device->dev_mutex);
	/* Wait for HDCP STATUS interrupt */
	rc = wait_for_completion_timeout(&ds90uh92x->hdcp_done,
					 msecs_to_jiffies(100));
	mutex_lock(&device->dev_mutex);
	if (!rc) {
		pr_err("[%s:%d], Timeout waiting for HDCP DONE\n", __func__,
		       device->instance_id);
		rc = -EIO;
		goto fail;
	} else {
		rc = 0;
	}

	if (!device->hdcp_status) {
		pr_err("[%s:%d]: HDCP AUTH failed\n", __func__,
		       device->instance_id);
		rc = -EIO;
	}

fail:
	return rc;
}

static int ds90uh92x_hdcp_reset(struct msm_dba_device_info *dev)
{
	int rc = 0;
	struct ds90uh92x *ds90uh92x = NULL;
	u8 reg_val;


	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	reg_val = BIT(1) | BIT(3);

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0xC3,
					   reg_val);
	if (rc) {
		pr_err("[%s:%d] i2c read failed\n", __func__,
						   dev->instance_id);
		goto fail;
	}

fail:
	return rc;
}

static int ds90uh92x_hdcp_retry(struct msm_dba_device_info *dev, u32 flags)
{
	int rc = 0;
	struct ds90uh92x *ds90uh92x = NULL;

	if (!dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	rc = ds90uh92x_hdcp_kickoff(ds90uh92x, flags);
	return rc;
}
static int ds90uh92x_hdcp_enable(struct ds90uh92x *ds90uh92x,
				 bool hdcp_on,
				 bool enc_on,
				 u32 flags)
{
	struct msm_dba_device_info *device = &ds90uh92x->dev;
	u8 reg_val = 0;
	int rc = 0;

	if (hdcp_on && device->hdcp_status) {
		reg_val = (enc_on ? BIT(2) : BIT(3));

		rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   0xC3,
						   reg_val);
		if (rc) {
			pr_err("[%s:%d] i2c read failed\n", __func__,
							   device->instance_id);
			goto fail;
		}
	} else if (hdcp_on && !device->hdcp_status) {
		rc = ds90uh92x_hdcp_kickoff(ds90uh92x, flags);
		if (rc) {
			pr_err("[%s:%d] failed to enable hdcp\n", __func__,
							   device->instance_id);
			goto fail;
		}

		if (!(flags & MSM_DBA_ASYNC_FLAG)) {
			reg_val = (enc_on ? BIT(2) : BIT(3));
			rc = msm_dba_helper_i2c_write_byte(
							ds90uh92x->i2c_client,
							ds90uh92x->i2c_addr,
							0xC3,
							reg_val);
			if (rc) {
				pr_err("[%s:%d] i2c write failed\n", __func__,
							   device->instance_id);
				goto fail;
			}
		} else {
			device->enc_on = enc_on;
		}

	} else if (!hdcp_on) {
		(void)msm_dba_register_hdcp_monitor(device, false);
		rc = ds90uh92x_hdcp_reset(device);
		if (rc) {
			pr_err("[%s:%d] failed to reset hdcp\n", __func__,
							   device->instance_id);
			goto fail;
		}
		device->hdcp_status = false;
	}
fail:
	return rc;
}

static int hdcp_enable(void *client, bool hdcp_on, bool enc_on, u32 flags)
{
	int rc = 0;
	struct msm_dba_client_info *c = client;
	struct msm_dba_device_info *device;
	struct ds90uh92x *ds90uh92x;

	if (!c) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	device = c->dev;
	if (!device) {
		pr_err("%s: deviceis NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&device->dev_mutex);

	pr_debug("[%s:%d] hdcp_en(%d), enc_on(%d)", __func__,
			device->instance_id, hdcp_on, enc_on);
	ds90uh92x = container_of(device, struct ds90uh92x, dev);

	rc = ds90uh92x_hdcp_enable(ds90uh92x, hdcp_on, enc_on, flags);
	mutex_unlock(&device->dev_mutex);
	return rc;
}

static int hdcp_get_ksv_list_size(void *client, u32 *count, u32 flags)
{
	return 0;
}

static int hdcp_get_ksv_list(void *client, u32 count, char *buf, u32 flags)
{
	return 0;
}

static int enable_remote_comm(void *client, bool on, u32 flags)
{
	int rc = 0;
	struct msm_dba_client_info *c = client;
	struct msm_dba_device_info *device;
	struct ds90uh92x *ds90uh92x;
	u8 reg_val = 0;

	if (!c) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	device = c->dev;
	if (!device) {
		pr_err("%s: deviceis NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&device->dev_mutex);
	pr_debug("[%s:%d] enable i2c bypass(%d)", __func__,
						 device->instance_id, on);

	ds90uh92x = container_of(device, struct ds90uh92x, dev);

	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     0x03,
				     &reg_val,
				     1);
	if (rc) {
		pr_err("[%s:%d] i2c read failed (0x%x, 0x%x)\n", __func__,
			device->instance_id, ds90uh92x->i2c_addr, 0x03);
		goto fail;
	}

	if (on)
		reg_val |= BIT(3);
	else
		reg_val &= ~BIT(3);

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0x03,
					   reg_val);
	if (rc) {
		pr_err("[%s:%d] i2c write failed (0x%x, 0x%x)\n", __func__,
			device->instance_id, ds90uh92x->i2c_addr, 0x03);
		goto fail;
	}
	ds90uh92x->remote_comm_en = on;
fail:
	mutex_unlock(&device->dev_mutex);
	return rc;
}

static int add_remote_device(void *client, u32 *slave_ids, u32 count, u32 flags)
{
	int rc = 0;
	struct msm_dba_client_info *c = client;
	struct msm_dba_device_info *device;
	struct ds90uh92x *ds90uh92x;
	u8 reg_val = 0;
	u8 reg = 0;
	int i = 0;

	if (!c || !slave_ids) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	device = c->dev;
	if (!device) {
		pr_err("%s: deviceis NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&device->dev_mutex);
	pr_debug("[%s:%d] add (%d) remote devices", __func__,
						   device->instance_id, count);

	ds90uh92x = container_of(device, struct ds90uh92x, dev);

	if ((ds90uh92x->num_of_slaves + count) > MAX_SLAVE_DEVICES) {
		pr_err("[%s:%d] Exceeds maximum slave devices (%d, %d)\n",
		       __func__, device->instance_id,
		       ds90uh92x->num_of_slaves, count);
		rc = -EINVAL;
		goto fail;
	}

	for (i = ds90uh92x->num_of_slaves;
		i < (ds90uh92x->num_of_slaves + count); i++) {

		reg_val = (slave_ids[i - ds90uh92x->num_of_slaves]) << 1;
		ds90uh92x->slave_ids[i] =
			slave_ids[i - ds90uh92x->num_of_slaves];
		if (i == 0)
			reg = 0x07;
		else
			reg = 0x6F + i;

		rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   reg,
						   reg_val);
		if (rc) {
			pr_err("[%s:%d] i2c write failed (%d, %d)\n", __func__,
			       device->instance_id, ds90uh92x->i2c_addr, reg);
			goto fail;
		}

		if (i == 0)
			reg = 0x08;
		else
			reg = 0x76 + i;

		rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   reg,
						   reg_val);
		if (rc) {
			pr_err("[%s:%d] i2c write failed (%d, %d)\n", __func__,
			       device->instance_id, ds90uh92x->i2c_addr, reg);
			goto fail;
		}

	}

	ds90uh92x->num_of_slaves += count;
fail:
	mutex_unlock(&device->dev_mutex);
	return rc;
}

static int ds90uh92x_reprogram_device(struct ds90uh92x *ds90uh92x)
{
	int rc = 0;
	u8 reg_val = 0;
	int i = 0;
	u8 reg = 0;
	struct msm_dba_device_info *device = &ds90uh92x->dev;
	/*
	 * Reprogramming sequence:
	 *	1. Setup interrupt mask
	 *	2. Enable remote dev communication
	 *	3. Add remote devices.
	 *	4. enable hdcp if required.
	 */

	rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
					   ds90uh92x->i2c_addr,
					   0xc6,
					   0x3F);

	if (rc) {
		pr_err("%s: i2c write failed for int enable\n", __func__);
		goto fail;
	}

	if (ds90uh92x->remote_comm_en) {
		rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
					     ds90uh92x->i2c_addr,
					     0x03,
					     &reg_val,
					     1);
		if (rc) {
			pr_err("[%s:%d] i2c read failed (0x%x, 0x%x)\n",
				__func__, device->instance_id,
				ds90uh92x->i2c_addr, 0x03);
			goto fail;
		}

		reg_val |= BIT(3);

		rc = msm_dba_helper_i2c_write_byte(ds90uh92x->i2c_client,
						   ds90uh92x->i2c_addr,
						   0x03,
						   reg_val);
		if (rc) {
			pr_err("[%s:%d] i2c write failed (0x%x, 0x%x)\n",
				 __func__, device->instance_id,
				 ds90uh92x->i2c_addr, 0x03);
			goto fail;
		}

		for (i = 0; i < ds90uh92x->num_of_slaves; i++) {
			reg_val = (ds90uh92x->slave_ids[i]) << 1;
			if (i == 0)
				reg = 0x07;
			else
				reg = 0x6F + i;

			rc = msm_dba_helper_i2c_write_byte(
							ds90uh92x->i2c_client,
							ds90uh92x->i2c_addr,
							reg,
							reg_val);
			if (rc) {
				pr_err("[%s:%d] i2c write failed (%d, %d)\n",
				       __func__, device->instance_id,
				       ds90uh92x->i2c_addr, reg);
				goto fail;
			}

			if (i == 0)
				reg = 0x08;
			else
				reg = 0x76 + i;

			rc = msm_dba_helper_i2c_write_byte(
							ds90uh92x->i2c_client,
							ds90uh92x->i2c_addr,
							reg,
							reg_val);
			if (rc) {
				pr_err("[%s:%d] i2c write failed (%d, %d)\n",
				       __func__, device->instance_id,
				       ds90uh92x->i2c_addr, reg);
				goto fail;
			}
		}
	}

	device->hdcp_status = false;
	rc = ds90uh92x_hdcp_enable(ds90uh92x,
				   device->hdcp_on,
				   device->enc_on,
				   0);
	if (rc)
		pr_err("%s: hdcp enable failed\n", __func__);
fail:
	return rc;
}

static int ds90uh92x_force_reset(struct msm_dba_device_info *dev, u32 flags)
{
	struct ds90uh92x *ds90uh92x = NULL;
	int rc = 0;

	if (!dev) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	if (dev->power_status) {
		rc = ds90uh92x_toggle_pdb(ds90uh92x, false);
		if (rc) {
			pr_err("%s: toggle pdb off failed", __func__);
			return rc;
		}
		rc = ds90uh92x_toggle_pdb(ds90uh92x, true);
		if (rc) {
			pr_err("%s: toggle pdb on failed", __func__);
			return rc;
		}
		rc = ds90uh92x_reprogram_device(ds90uh92x);
		if (rc)
			pr_err("%s: ds90uh92x reprogram failed\n", __func__);
	}

	return rc;
}

static int ds90uh92x_dump_info(struct msm_dba_device_info *dev, u32 flags)
{
	struct ds90uh92x *ds90uh92x = NULL;
	u8 reg_val = 0, deserial_addr = 0;
	int rc = 0;
	int i = 0;

	if (!dev) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	if (!ds90uh92x) {
		pr_err("%s: ds90uh92x is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&dev->dev_mutex);
	pr_info("------------%s:%d STATUS--------------\n", dev->chip_name,
							  dev->instance_id);
	pr_info("POWER_STATUS = %d\n", dev->power_status);
	pr_info("VIDEO_STATUS = %d\n", dev->video_status);
	pr_info("HDCP_STATUS = %d\n", dev->hdcp_status);

	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     0xC3,
				     &reg_val,
				     1);
	if (rc)
		goto fail;

	pr_info("HDCP_CTL = 0x%x\n", reg_val);

	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     0xC4,
				     &reg_val,
				     1);
	if (rc)
		goto fail;

	pr_info("HDCP_STS = 0x%x\n", reg_val);
	pr_info("Serializer Reg Dump I2C=[0x%02x]:\n", ds90uh92x->i2c_addr);
	for (i = SERIALIZER_REG_START_ADDR; i <= SERIALIZER_REG_END_ADDR; i++) {
		rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
					     ds90uh92x->i2c_addr,
					     i,
					     &reg_val,
					     1);
		if (rc)
			goto fail;
		pr_info("[0x%02x]=0x%02x\n", i, reg_val);
	}
	rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
				     ds90uh92x->i2c_addr,
				     0x06,
				     &deserial_addr,
				     1);
	if (rc)
		goto fail;
	deserial_addr >>= 1;
	pr_info("De-Serializer Reg Dump I2C=[0x%02x]:\n", deserial_addr);
	for (i = DESERIALIZER_REG_START_ADDR;
		i <= DESERIALIZER_REG_END_ADDR; i++) {
		rc = msm_dba_helper_i2c_read(ds90uh92x->i2c_client,
					     deserial_addr,
					     i,
					     &reg_val,
					     1);
		if (rc)
			goto fail;
		pr_info("[0x%02x]=0x%02x\n", i, reg_val);
	}
fail:
	pr_info("---------------------------------------\n");
	mutex_unlock(&dev->dev_mutex);
	return rc;
}

static int dump_debug_info(void *client, u32 flags)
{
	int rc = 0;
	struct msm_dba_client_info *c = client;

	if (!c) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = ds90uh92x_dump_info(c->dev, flags);

	return rc;
}

static void ds90uh92x_get_capabilities(char *chip_name,
				       struct msm_dba_capabilities *caps)
{
	if (!strcmp(chip_name, "DS90UH927Q") ||
	    !strcmp(chip_name, "DS90UH929")) {

		caps->av_mute_support = false;
		caps->deferred_commit_support = false;

		caps->aud_caps.audio_support = false;
		caps->aud_caps.audio_rates = 0;
		caps->aud_caps.audio_fmts = 0;

		caps->vid_caps.hdcp_support = true;
		caps->vid_caps.edid_support = false;
		caps->vid_caps.data_lanes_lp_support = false;
		caps->vid_caps.clock_lanes_lp_support = false;
		caps->vid_caps.max_pclk_khz = 80000;
		caps->vid_caps.num_of_input_lanes = 1;
	}

	if (!strcmp(chip_name, "DS90UB927Q"))
		caps->vid_caps.hdcp_support = false;

}

static int ds90uh92x_init_device_info(struct ds90uh92x *info,
				   const struct ds90uh92x_platform_data *pdata)
{
	struct msm_dba_device_info *dev = &info->dev;

	if (!dev) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	strlcpy(dev->chip_name, pdata->chip_id, MSM_DBA_CHIP_NAME_MAX_LEN);
	dev->instance_id = pdata->instance_id;

	mutex_init(&dev->dev_mutex);
	INIT_LIST_HEAD(&dev->client_list);
	init_completion(&info->ksv_rdy);
	init_completion(&info->hdcp_done);

	info->reset_gpio = pdata->reset_gpio;
	info->irq_gpio = pdata->irq_gpio;
	info->i2c_addr = pdata->slave_addr;

	ds90uh92x_get_capabilities(dev->chip_name, &dev->caps);

	dev->client_ops.get_caps = msm_dba_helper_get_caps;

	dev->client_ops.power_on = msm_dba_helper_power_on;
	dev->dev_ops.dev_power_on = ds90uh92x_power_on;

	dev->client_ops.video_on = msm_dba_helper_video_on;
	dev->dev_ops.dev_video_on = ds90uh92x_video_on;

	dev->client_ops.interrupts_enable = msm_dba_helper_interrupts_enable;
	dev->client_ops.hdcp_enable = hdcp_enable;
	dev->client_ops.hdcp_get_ksv_list_size = hdcp_get_ksv_list_size;
	dev->client_ops.hdcp_get_ksv_list = hdcp_get_ksv_list;
	dev->client_ops.enable_remote_comm = enable_remote_comm;
	dev->client_ops.add_remote_device = add_remote_device;
	dev->client_ops.dump_debug_info = dump_debug_info;
	dev->client_ops.force_reset = msm_dba_helper_force_reset;

	dev->dev_ops.handle_interrupts = ds90uh92x_handle_interrupts;
	dev->dev_ops.unmask_interrupts = ds90uh92x_unmask_interrupts;
	dev->dev_ops.hdcp_reset = ds90uh92x_hdcp_reset;
	dev->dev_ops.hdcp_retry = ds90uh92x_hdcp_retry;
	dev->dev_ops.write_reg = ds90uh92x_write_reg;
	dev->dev_ops.read_reg = ds90uh92x_read_reg;
	dev->dev_ops.force_reset = ds90uh92x_force_reset;
	dev->dev_ops.dump_debug_info = ds90uh92x_dump_info;

	return 0;
}

static int ds90uh92x_attach_irq(struct ds90uh92x *ds90uh92x)
{
	int rc = 0;

	rc = gpio_request(ds90uh92x->irq_gpio, "ds90uh92x");
	if (rc) {
		pr_err("%s: gpio request failed\n", __func__);
		return rc;
	}

	rc = gpio_direction_input(ds90uh92x->irq_gpio);
	if (rc) {
		pr_err("%s: Failed to configure irq gpio %d\n",  __func__, rc);
		return rc;
	}

	ds90uh92x->irq = gpio_to_irq(ds90uh92x->irq_gpio);

	rc = msm_dba_helper_register_irq(&ds90uh92x->dev, ds90uh92x->irq,
					 IRQF_TRIGGER_FALLING | IRQF_ONESHOT);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__, rc);
		return rc;
	}
	disable_irq(ds90uh92x->irq);
	return rc;
}

static int __devinit ds90uh92x_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	const struct ds90uh92x_platform_data *pdata;
	struct ds90uh92x *ds90uh92x;
	int rc = 0;

	if (!client) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	pdata = client->dev.platform_data;
	pr_debug("%s: probe called %s, %d\n", __func__, pdata->chip_id,
		 pdata->instance_id);
	ds90uh92x = kzalloc(sizeof(*ds90uh92x), GFP_KERNEL);
	if (!ds90uh92x) {
		pr_err("%s: Not enough memory\n", __func__);
		return -ENOMEM;
	}

	memset(ds90uh92x, 0x0, sizeof(*ds90uh92x));

	rc = ds90uh92x_init_device_info(ds90uh92x, pdata);
	if (rc) {
		pr_err("%s: Device not compatible\n", __func__);
		kfree(ds90uh92x);
		return rc;
	}
	ds90uh92x->i2c_client = client;

	rc = msm_dba_add_probed_device(&ds90uh92x->dev);
	if (rc) {
		pr_err("%s: could not add device\n", __func__);
		kfree(ds90uh92x);
		return rc;
	}

	rc = gpio_request(ds90uh92x->reset_gpio, "ds90uh92x");
	if (rc) {
		pr_err("%s: could not request gpio\n", __func__);
		kfree(ds90uh92x);
		return rc;
	}

	rc = ds90uh92x_attach_irq(ds90uh92x);
	if (rc) {
		pr_err("%s: could not attach irq\n", __func__);
		kfree(ds90uh92x);
		return rc;
	}

	rc = msm_dba_helper_sysfs_init(&client->dev);
	if (rc) {
		pr_err("%s: sysfs init failed\n", __func__);
		kfree(ds90uh92x);
		return rc;
	}

	pr_debug("%s: probe successful %s, %d\n", __func__, pdata->chip_id,
		 pdata->instance_id);
	dev_set_drvdata(&client->dev, &ds90uh92x->dev);
	return 0;
}

static int ds90uh92x_i2c_remove(struct i2c_client *client)
{
	struct msm_dba_device_info *dev;
	struct ds90uh92x *ds90uh92x;

	if (!client) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	dev = dev_get_drvdata(&client->dev);
	ds90uh92x = container_of(dev, struct ds90uh92x, dev);
	mutex_destroy(&ds90uh92x->dev.dev_mutex);
	kfree(ds90uh92x);

	return 0;
}

static struct i2c_device_id ds90uh92x_i2c_id[] = {
	{ "ds90uh92x_i2c", 0 },
	{ }
};

static struct i2c_driver ds90uh92x_i2c_driver = {
	.driver = {
		.name = "ds90uh92x_i2c",
		.owner = THIS_MODULE,
	},
	.probe = ds90uh92x_i2c_probe,
	.remove = ds90uh92x_i2c_remove,
	.id_table = ds90uh92x_i2c_id,
};
static int __init ds90uh92x_init(void)
{
	pr_debug("%s: init called for ds90uh92x\n", __func__);
	return i2c_add_driver(&ds90uh92x_i2c_driver);
}

static void __exit ds90uh92x_exit(void)
{
	i2c_del_driver(&ds90uh92x_i2c_driver);
}

module_init(ds90uh92x_init);
module_exit(ds90uh92x_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver for ds90uh92x devices");
