/* Copyright (c) 2010-2012, 2014 The Linux Foundation. All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/usb/msm_hsusb_hw.h>
#include <linux/usb/ulpi.h>
#include <linux/gpio.h>

#include "ci13xxx_udc.c"

#define MSM_USB_BASE		(udc->regs)
#define MAX_USB_CORE_NUM	4

/* device index 1 is used for USB1 Controller,3 for USB3 and id 4 for USB4 */
struct ci13xxx_udc_context {
	int irq;
	void __iomem *regs;
	int wake_gpio;
	int wake_irq;
	bool wake_irq_state;
};

static struct ci13xxx_udc_context _udc_ctxt[MAX_USB_CORE_NUM+1];

static irqreturn_t msm_udc_irq(int irq, void *data)
{
	struct platform_device *pdev = data;

	return udc_irq(irq, pdev);
}

static void ci13xxx_msm_suspend(struct ci13xxx *udc)
{
	struct device *dev = udc->gadget.dev.parent;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	dev_dbg(dev, "ci13xxx_msm_suspend\n");

	if (_udc_ctxt[pdev->id].wake_irq &&
			!_udc_ctxt[pdev->id].wake_irq_state) {
		enable_irq_wake(_udc_ctxt[pdev->id].wake_irq);
		enable_irq(_udc_ctxt[pdev->id].wake_irq);
		_udc_ctxt[pdev->id].wake_irq_state = true;
	}
}

static void ci13xxx_msm_resume(struct ci13xxx *udc)
{
	struct device *dev = udc->gadget.dev.parent;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	dev_dbg(dev, "ci13xxx_msm_resume\n");

	if (_udc_ctxt[pdev->id].wake_irq &&
		_udc_ctxt[pdev->id].wake_irq_state) {
		disable_irq_wake(_udc_ctxt[pdev->id].wake_irq);
		disable_irq_nosync(_udc_ctxt[pdev->id].wake_irq);
		_udc_ctxt[pdev->id].wake_irq_state = false;
	}
}

static void ci13xxx_msm_disconnect(struct ci13xxx *udc)
{
	struct usb_phy *phy = udc->transceiver;

	if (phy && (phy->flags & ENABLE_DP_MANUAL_PULLUP))
		usb_phy_io_write(phy,
				ULPI_MISC_A_VBUSVLDEXT |
				ULPI_MISC_A_VBUSVLDEXTSEL,
				ULPI_CLR(ULPI_MISC_A));
}

static void ci13xxx_msm_connect(struct ci13xxx *udc)
{
	struct usb_phy *phy = udc->transceiver;

	if (phy && (phy->flags & ENABLE_DP_MANUAL_PULLUP)) {
		int	temp;

		usb_phy_io_write(phy,
			ULPI_MISC_A_VBUSVLDEXT |
			ULPI_MISC_A_VBUSVLDEXTSEL,
			ULPI_SET(ULPI_MISC_A));

		temp = readl_relaxed(USB_GENCONFIG2);
		temp |= GENCFG2_SESS_VLD_CTRL_EN;
		writel_relaxed(temp, USB_GENCONFIG2);

		temp = readl_relaxed(USB_USBCMD);
		temp |= USBCMD_SESS_VLD_CTRL;
		writel_relaxed(temp, USB_USBCMD);

		/*
		 * Add memory barrier as it is must to complete
		 * above USB PHY and Link register writes before
		 * moving ahead with USB peripheral mode enumeration,
		 * otherwise USB peripheral mode may not work.
		 */
		mb();
	}
}

static void ci13xxx_msm_notify_event(struct ci13xxx *udc, unsigned event)
{
	struct device *dev = udc->gadget.dev.parent;

	switch (event) {
	case CI13XXX_CONTROLLER_RESET_EVENT:
		dev_info(dev, "CI13XXX_CONTROLLER_RESET_EVENT received\n");
		writel(0, USB_AHBBURST);
		writel_relaxed(0x08, USB_AHBMODE);
		break;
	case CI13XXX_CONTROLLER_DISCONNECT_EVENT:
		dev_info(dev, "CI13XXX_CONTROLLER_DISCONNECT_EVENT received\n");
		ci13xxx_msm_disconnect(udc);
		ci13xxx_msm_resume(udc);
		break;
	case CI13XXX_CONTROLLER_CONNECT_EVENT:
		dev_info(dev, "CI13XXX_CONTROLLER_CONNECT_EVENT received\n");
		ci13xxx_msm_connect(udc);
		break;
	case CI13XXX_CONTROLLER_SUSPEND_EVENT:
		dev_info(dev, "CI13XXX_CONTROLLER_SUSPEND_EVENT received\n");
		ci13xxx_msm_suspend(udc);
		break;
	case CI13XXX_CONTROLLER_RESUME_EVENT:
		dev_info(dev, "CI13XXX_CONTROLLER_RESUME_EVENT received\n");
		ci13xxx_msm_resume(udc);
		break;

	default:
		dev_dbg(dev, "unknown ci13xxx_udc event\n");
		break;
	}
}

static irqreturn_t ci13xxx_msm_resume_irq(int irq, void *data)
{
	struct ci13xxx *udc = data;

	if (udc->transceiver && udc->vbus_active && udc->suspended)
		usb_phy_set_suspend(udc->transceiver, 0);
	else if (!udc->suspended)
		ci13xxx_msm_resume(udc);

	return IRQ_HANDLED;
}

static struct ci13xxx_udc_driver ci13xxx_msm_udc_driver = {
	.name			= "ci13xxx_msm",
	.flags			= CI13XXX_REGS_SHARED |
				  CI13XXX_REQUIRE_TRANSCEIVER |
				  CI13XXX_PULLUP_ON_VBUS |
				  CI13XXX_ZERO_ITC |
				  CI13XXX_DISABLE_STREAMING |
				  CI13XXX_IS_OTG,

	.notify_event		= ci13xxx_msm_notify_event,
};

static int ci13xxx_msm_install_wake_gpio(struct platform_device *pdev,
				struct resource *res)
{
	struct ci13xxx *udc = _udc[pdev->id];
	int wake_irq;
	int ret;

	dev_dbg(&pdev->dev, "ci13xxx_msm_install_wake_gpio\n");

	_udc_ctxt[pdev->id].wake_gpio = res->start;
	gpio_request(_udc_ctxt[pdev->id].wake_gpio, "USB_RESUME");
	gpio_direction_input(_udc_ctxt[pdev->id].wake_gpio);
	wake_irq = gpio_to_irq(_udc_ctxt[pdev->id].wake_gpio);
	if (wake_irq < 0) {
		dev_err(&pdev->dev, "could not register USB_RESUME GPIO.\n");
		return -ENXIO;
	}

	dev_dbg(&pdev->dev, "_udc_ctxt.gpio_irq = %d and irq = %d\n",
			_udc_ctxt[pdev->id].wake_gpio, wake_irq);
	ret = request_irq(wake_irq, ci13xxx_msm_resume_irq,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, "usb resume", udc);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register USB_RESUME IRQ.\n");
		goto gpio_free;
	}
	disable_irq(wake_irq);
	_udc_ctxt[pdev->id].wake_irq = wake_irq;

	return 0;

gpio_free:
	gpio_free(_udc_ctxt[pdev->id].wake_gpio);
	_udc_ctxt[pdev->id].wake_gpio = 0;
	return ret;
}

static void ci13xxx_msm_uninstall_wake_gpio(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "ci13xxx_msm_uninstall_wake_gpio\n");

	if (_udc_ctxt[pdev->id].wake_gpio) {
		gpio_free(_udc_ctxt[pdev->id].wake_gpio);
		_udc_ctxt[pdev->id].wake_gpio = 0;
	}
}

static int ci13xxx_msm_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	dev_dbg(&pdev->dev, "ci13xxx_msm_probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform resource mem\n");
		return -ENXIO;
	}

	_udc_ctxt[pdev->id].regs = ioremap(res->start, resource_size(res));
	if (!_udc_ctxt[pdev->id].regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	ret = udc_probe(&ci13xxx_msm_udc_driver, pdev,
					_udc_ctxt[pdev->id].regs);
	if (ret < 0) {
		dev_err(&pdev->dev, "udc_probe failed\n");
		goto iounmap;
	}

	_udc_ctxt[pdev->id].irq = platform_get_irq(pdev, 0);
	if (_udc_ctxt[pdev->id].irq < 0) {
		dev_err(&pdev->dev, "IRQ not found\n");
		ret = -ENXIO;
		goto udc_remove;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, "USB_RESUME");
	if (res) {
		ret = ci13xxx_msm_install_wake_gpio(pdev, res);
		if (ret < 0) {
			dev_err(&pdev->dev, "gpio irq install failed\n");
			goto udc_remove;
		}
	}

	ret = request_irq(_udc_ctxt[pdev->id].irq, msm_udc_irq, IRQF_SHARED,
			pdev->name, pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto gpio_uninstall;
	}

	pm_runtime_no_callbacks(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

gpio_uninstall:
	ci13xxx_msm_uninstall_wake_gpio(pdev);
udc_remove:
	udc_remove(pdev);
iounmap:
	iounmap(_udc_ctxt[pdev->id].regs);

	return ret;
}

int ci13xxx_msm_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	free_irq(_udc_ctxt[pdev->id].irq, pdev);
	ci13xxx_msm_uninstall_wake_gpio(pdev);
	udc_remove(pdev);
	iounmap(_udc_ctxt[pdev->id].regs);
	return 0;
}

void ci13xxx_msm_shutdown(struct platform_device *pdev)
{
	ci13xxx_pullup(&_udc[pdev->id]->gadget, 0);
}

static struct platform_driver ci13xxx_msm_driver = {
	.probe = ci13xxx_msm_probe,
	.driver = { .name = "msm_hsusb", },
	.remove = ci13xxx_msm_remove,
	.shutdown = ci13xxx_msm_shutdown,
};
MODULE_ALIAS("platform:msm_hsusb");

static int __init ci13xxx_msm_init(void)
{
	return platform_driver_register(&ci13xxx_msm_driver);
}
module_init(ci13xxx_msm_init);

static void __exit ci13xxx_msm_exit(void)
{
	platform_driver_unregister(&ci13xxx_msm_driver);
}
module_exit(ci13xxx_msm_exit);

MODULE_LICENSE("GPL v2");
