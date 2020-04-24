// SPDX-License-Identifier: GPL-2.0+
/*
 * Wrapper for hot plugging virtio memory mapped device
 * Copyright (C) 2020  OpenSynergy GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * DEVICE TREE BINDINGS
 * ====================
 *
 * In simple case, it's enough to replace original virtio-mmio device node name
 * and compatible string. If the node name is not changed, subsequent attachment
 * of the target virtio-mmio node will fail due to a sysfs name conflict.
 *
 * In general case, the stub device can be instantiated in two possible ways.
 *
 * 1. Replacing only original virtio-mmio device node
 * --------------------------------------------------
 *
 * In this case, you need to change only the device node name and compatible
 * string.
 *
 * Example:
 *
 *     virtio_mmio_stub@4b003000 {
 *         compatible = "virtio,mmio-stub";
 *         reg = <0x0 0x4b003000 0x0 0x1000>;
 *         interrupts = <0 134 4>;
 *     };
 *
 * 2. Replacing all virtio-mmio device nodes with a single stub device node
 * ------------------------------------------------------------------------
 *
 * In this case, you need to change the device node name and compatible string,
 * and add two additional properties:
 *
 * virtio-mmio-count (u32)
 *   Specifies the number of virtio-mmio devices to be hot plugged.
 *
 * virtio-mmio-irq-cell (u32)
 *   Specifies the cell offset of the IRQ line in the interrupts property.
 *
 * If we assume that ADDRESS0 and SIZE is the address and size from the stub
 * device reg property, and IRQ0 is the interrupt number from the stub device
 * interrupts property, then for each virtio-mmio device:
 *
 *   - The address in the reg property is set to ADDRESS0 + SIZE * device_index,
 *   - The IRQ line in the interrupts property is set to IRQ0 + device_index.
 *
 * Example:
 *
 *     virtio_mmio_stub@4b004000 {
 *         compatible = "virtio,mmio-stub";
 *         reg = <0x0 0x4b004000 0x0 0x1000>;
 *         interrupts = <0 135 4>;
 *         virtio-mmio-count = <4>;
 *         virtio-mmio-irq-cell = <1>;
 *     };
 */

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <uapi/linux/virtio_mmio.h>

/*******************************************************************************
 * VIRTIO MMIO HOTPLUG STATUS BITS
 */
/* Request device side to be ready for hot plugging */
#define VIRTIO_CONFIG_S_HP_REQUEST	0x40000000
/* Indicates the device is ready for hot plugging */
#define VIRTIO_CONFIG_S_HP_READY	0x80000000

/**
 * struct virtio_mmio_ctx - virtio mmio device context
 * @pdev: Stub platform device.
 * @work: Kernel worker for a virtio mmio device probing.
 * @node_lock: Mutex protecting the device node.
 * @node: Target virtio mmio device node.
 * @mem_start: Physical address of the memory mapped region.
 * @mem_size: Size of the memory mapped region.
 * @iomem: Virtual address of the memory mapped region.
 * @irq: IRQ line.
 */
struct virtio_mmio_ctx {
	struct platform_device *pdev;
	struct work_struct work;
	struct mutex node_lock;
	struct device_node *node;
	resource_size_t mem_start;
	resource_size_t mem_size;
	u8 *iomem;
	int irq;
};

/**
 * struct virtio_mmio_stub_ctx - virtio mmio stub device context
 * @devices: virtio-mmio device contexts to be hot plugged.
 * @ndevices: The number of virtio-mmio device contexts.
 */
struct virtio_mmio_stub_ctx {
	struct virtio_mmio_ctx *devices;
	unsigned int ndevices;
};

/*
 * Defined in the drivers/of/of_private.h header file, which cannot be included
 * here.
 */
struct device_node *__of_node_dup(const struct device_node *np,
				  const char *fmt, ...);

static irqreturn_t virtio_mmio_stub_isr(int irq, void *opaque)
{
	struct virtio_mmio_ctx *ctx = opaque;

	mb(); /* TODO: sync VIRTIO_MMIO_STATUS value */

	if (readl(ctx->iomem + VIRTIO_MMIO_STATUS) & VIRTIO_CONFIG_S_HP_READY) {
		schedule_work(&ctx->work);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void virtio_mmio_stub_unmap_resources(struct virtio_mmio_ctx *ctx)
{
	struct platform_device *pdev = ctx->pdev;

	if (ctx->iomem) {
		devm_ioremap_release(&pdev->dev, ctx->iomem);
		devm_release_mem_region(&pdev->dev, ctx->mem_start,
					ctx->mem_size);
	}

	free_irq(ctx->irq, ctx);

	ctx->iomem = NULL;
}

static int virtio_mmio_stub_map_resources(struct virtio_mmio_ctx *ctx)
{
	struct platform_device *pdev = ctx->pdev;
	int code;

	if (!devm_request_mem_region(&pdev->dev, ctx->mem_start, ctx->mem_size,
				     pdev->name))
		return -EBUSY;

	ctx->iomem = devm_ioremap(&pdev->dev, ctx->mem_start, ctx->mem_size);
	if (!ctx->iomem) {
		devm_release_mem_region(&pdev->dev, ctx->mem_start,
					ctx->mem_size);
		return -EFAULT;
	}

	/*
	 * Must be written before activating IRQ line!
	 *
	 * IRQ line can be already triggered:
	 *   request_irq
	 *     -> virtio_mmio_stub_isr
	 *          -> virtio_mmio_stub_unmap_resources
	 *               -> ctx->iomem = NULL;
	 */
	writel(VIRTIO_CONFIG_S_HP_REQUEST, ctx->iomem + VIRTIO_MMIO_STATUS);

	code = request_irq(ctx->irq, virtio_mmio_stub_isr, IRQF_SHARED,
			   dev_name(&pdev->dev), ctx);
	if (code)
		virtio_mmio_stub_unmap_resources(ctx);

	return code;
}

static void virtio_mmio_stub_work(struct work_struct *work)
{
	struct virtio_mmio_ctx *ctx =
		container_of(work, struct virtio_mmio_ctx, work);

	/*
	 * Since the callback can be invoked in any moment, we need to sync with
	 * possible invocation of the remove or the suspend/resume callbacks.
	 */
	mutex_lock(&ctx->node_lock);

	/*
	 * Hypothetically, there may be another interrupt during this callback.
	 */
	if (ctx->node) {
		virtio_mmio_stub_unmap_resources(ctx);

		if (of_attach_node(ctx->node)) {
			dev_err(&ctx->pdev->dev, "Failed to probe a device");

			if (virtio_mmio_stub_map_resources(ctx))
				dev_err(&ctx->pdev->dev,
					"Failed to remap resources");
		} else {
			/* Mark the device node as populated */
			ctx->node = NULL;
		}
	}

	mutex_unlock(&ctx->node_lock);
}

static int virtio_mmio_ctx_init(struct platform_device *pdev,
				struct virtio_mmio_ctx *ctx, unsigned int index,
				int irq_cell)
{
	struct resource *mem;
	struct property *property;

	ctx->pdev = pdev;
	mutex_init(&ctx->node_lock);
	INIT_WORK(&ctx->work, virtio_mmio_stub_work);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	ctx->mem_size = resource_size(mem);
	ctx->mem_start = mem->start + ctx->mem_size * index;

	/* Clone original device node */
	ctx->node = __of_node_dup(pdev->dev.of_node, "%s/virtio_mmio@%llx",
				  pdev->dev.of_node->parent->full_name,
				  ctx->mem_start);
	if (!ctx->node)
		return -ENOMEM;

	ctx->node->parent = pdev->dev.of_node->parent;

	/* Replace name property */
	property = of_find_property(ctx->node, "name", NULL);
	if (!property)
		return -EINVAL;

	kfree(property->value);

	property->value = kstrdup("virtio_mmio", GFP_KERNEL);
	property->length = strlen(property->value);

	/* Replace compatible property */
	property = of_find_property(ctx->node, "compatible", NULL);
	if (!property)
		return -EINVAL;

	kfree(property->value);

	property->value = kstrdup("virtio,mmio", GFP_KERNEL);
	property->length = strlen(property->value);

	if (irq_cell != -1) {
		struct device_node *node;
		u32 i;
		u32 address_cells = 0;
		u64 address = ctx->mem_start;
		struct resource resource;
		__be32 *values;

		/* Replace reg property */
		for (node = ctx->node; node; node = node->parent)
			if (!of_property_read_u32(node, "#address-cells",
						  &address_cells))
				break;

		if (!address_cells) {
			dev_err(&pdev->dev,
				"Failed to read #address-cells value");
			return -EINVAL;
		}

		property = of_find_property(ctx->node, "reg", NULL);
		if (!property)
			return -EINVAL;

		values = property->value;
		for (i = 0; i < address_cells; ++i) {
			values[address_cells - i - 1] =
				cpu_to_be32((u32)(address & 0xffffffff));
			address >>= 32;
		}

		/* Replace interrupts property */
		property = of_find_property(ctx->node, "interrupts", NULL);
		if (!property)
			return -EINVAL;

		if (irq_cell >= property->length / sizeof(__be32)) {
			dev_err(&pdev->dev,
				"interrupts contains invalid value");
			return -EINVAL;
		}

		values = property->value;
		values[irq_cell] =
			cpu_to_be32(be32_to_cpu(values[irq_cell]) + index);

		ctx->irq = of_irq_to_resource(ctx->node, 0, &resource);
	} else {
		ctx->irq = platform_get_irq(pdev, 0);
	}

	if (ctx->irq < 0)
		return ctx->irq;

	return virtio_mmio_stub_map_resources(ctx);
}

static int virtio_mmio_stub_read_properties(struct platform_device *pdev,
					    u32 *count, u32 *irq_cell)
{
	struct property *property;

	if (of_property_read_u32(pdev->dev.of_node, "virtio-mmio-count", count))
		return 0;

	if (*count > 1 && of_property_read_u32(pdev->dev.of_node,
					       "virtio-mmio-irq-cell",
					       irq_cell)) {
		dev_err(&pdev->dev,
			"virtio-mmio-irq-cell contains invalid value");
		return -EINVAL;
	}

	property = of_find_property(pdev->dev.of_node,
				    "virtio-mmio-count", NULL);
	if (property)
		of_remove_property(pdev->dev.of_node, property);

	property = of_find_property(pdev->dev.of_node,
				    "virtio-mmio-irq-cell", NULL);
	if (property)
		of_remove_property(pdev->dev.of_node, property);

	return 0;
}

static int virtio_mmio_stub_probe(struct platform_device *pdev)
{
	struct virtio_mmio_stub_ctx *ctx;
	int irq_cell = -1;
	unsigned int i;
	int code;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->ndevices = 1;

	code = virtio_mmio_stub_read_properties(pdev, &ctx->ndevices,
						&irq_cell);
	if (code)
		return code;

	if (!ctx->ndevices)
		return 0;

	ctx->devices = devm_kcalloc(&pdev->dev, ctx->ndevices,
				    sizeof(*ctx->devices), GFP_KERNEL);
	if (!ctx->devices)
		return -ENOMEM;

	for (i = 0; i < ctx->ndevices; ++i) {
		code = virtio_mmio_ctx_init(pdev, &ctx->devices[i], i,
					    irq_cell);
		if (code)
			return code;
	}

	platform_set_drvdata(pdev, ctx);

	return 0;
}

static int virtio_mmio_stub_remove(struct platform_device *pdev)
{
	struct virtio_mmio_stub_ctx *ctx = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ctx->ndevices; ++i) {
		struct virtio_mmio_ctx *device_ctx = &ctx->devices[i];

		mutex_lock(&device_ctx->node_lock);
		if (device_ctx->node)
			virtio_mmio_stub_unmap_resources(device_ctx);
		mutex_unlock(&device_ctx->node_lock);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int virtio_mmio_stub_freeze(struct device *dev)
{
	return virtio_mmio_stub_remove(to_platform_device(dev));
}

static int virtio_mmio_stub_restore(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct virtio_mmio_stub_ctx *ctx = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ctx->ndevices; ++i) {
		struct virtio_mmio_ctx *device_ctx = &ctx->devices[i];

		mutex_lock(&device_ctx->node_lock);
		if (device_ctx->node)
			if (virtio_mmio_stub_map_resources(device_ctx))
				dev_err(&pdev->dev,
					"Failed to remap resources");
		mutex_unlock(&device_ctx->node_lock);
	}

	return 0;
}

static const struct dev_pm_ops virtio_mmio_stub_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(virtio_mmio_stub_freeze,
				virtio_mmio_stub_restore)
};
#endif /* CONFIG_PM_SLEEP */

static struct of_device_id virtio_mmio_stub_match[] = {
	{ .compatible = "virtio,mmio-stub", },
	{},
};
MODULE_DEVICE_TABLE(of, virtio_mmio_stub_match);

static struct platform_driver virtio_mmio_stub_driver = {
	.probe		= virtio_mmio_stub_probe,
	.remove		= virtio_mmio_stub_remove,
	.driver		= {
		.name	= "virtio-mmio-stub",
		.of_match_table	= virtio_mmio_stub_match
	},
#ifdef CONFIG_PM_SLEEP
	.driver.pm = &virtio_mmio_stub_pm_ops,
#endif
};

static int __init virtio_mmio_stub_init(void)
{
	return platform_driver_register(&virtio_mmio_stub_driver);
}
subsys_initcall(virtio_mmio_stub_init);

static void __exit virtio_mmio_stub_exit(void)
{
	platform_driver_unregister(&virtio_mmio_stub_driver);
}
module_exit(virtio_mmio_stub_exit);

MODULE_DESCRIPTION("Platform stub driver for virtio-mmio devices");
MODULE_LICENSE("GPL");
