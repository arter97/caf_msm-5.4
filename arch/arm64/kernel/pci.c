/*
 * Code borrowed from powerpc/kernel/pci-common.c
 *
 * Copyright (C) 2003 Anton Blanchard <anton@xxxxxxxxxx>, IBM
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#include <asm/pci-bridge.h>

struct ioresource {
	struct list_head list;
	phys_addr_t start;
	resource_size_t size;
};

static LIST_HEAD(io_list);

int pci_register_io_range(phys_addr_t address, resource_size_t size)
{
	struct ioresource *res;
	resource_size_t allocated_size = 0;

	/* find if the range has not been already allocated */
	list_for_each_entry(res, &io_list, list) {
		if (address >= res->start &&
			address + size <= res->start + size)
			return 0;
		allocated_size += res->size;
	}

	/* range not already registered, check for space */
	if (allocated_size + size > IO_SPACE_LIMIT)
		return -E2BIG;

	/* add the range in the list */
	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;
	res->start = address;
	res->size = size;

	list_add_tail(&res->list, &io_list);

	return 0;
}
EXPORT_SYMBOL_GPL(pci_register_io_range);

unsigned long pci_address_to_pio(phys_addr_t address)
{
	struct ioresource *res;

	list_for_each_entry(res, &io_list, list) {
		if (address >= res->start &&
			address < res->start + res->size) {
			return res->start - address;
		}
	}

	return (unsigned long)-1;
}
EXPORT_SYMBOL_GPL(pci_address_to_pio);

/*
 * Called after each bus is probed, but before its children are examined
 */
void pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *dev;
	struct resource *res;
	int i;

	if (!pci_is_root_bus(bus)) {
		pci_read_bridge_bases(bus);

		pci_bus_for_each_resource(bus, res, i) {
			if (!res || !res->flags || res->parent)
				continue;

			/*
			 * If we are going to reassign everything, we can
			 * shrink the P2P resource to have zero size to
			 * save space
			 */
			if (pci_has_flag(PCI_REASSIGN_ALL_RSRC)) {
				res->flags |= IORESOURCE_UNSET;
				res->start = 0;
				res->end = -1;
				continue;
			}
		}
	}

	list_for_each_entry(dev, &bus->devices, bus_list) {
		/* Ignore fully discovered devices */
		if (dev->is_added)
			continue;

		set_dev_node(&dev->dev, pcibus_to_node(dev->bus));

		/* Read default IRQs and fixup if necessary */
		dev->irq = of_irq_parse_and_map_pci(dev, 0, 0);
	}
}
EXPORT_SYMBOL(pcibios_fixup_bus);

/*
 * We don't have to worry about legacy ISA devices, so nothing to do here
 */
resource_size_t pcibios_align_resource(void *data, const struct resource *res,
				resource_size_t size, resource_size_t align)
{
	return res->start;
}

int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	return pci_enable_resources(dev, mask);
}

#define IO_SPACE_PAGES	((IO_SPACE_LIMIT + 1) / PAGE_SIZE)
static DECLARE_BITMAP(pci_iospace, IO_SPACE_PAGES);

unsigned long pci_ioremap_io(const struct resource *res, phys_addr_t phys_addr)
{
	unsigned long start, len, virt_start;
	int err;

	if (res->end > IO_SPACE_LIMIT)
		return -EINVAL;

	/*
	 * try finding free space for the whole size first,
	 * fall back to 64K if not available
	 */
	len = resource_size(res);
	start = bitmap_find_next_zero_area(pci_iospace, IO_SPACE_PAGES,
				res->start / PAGE_SIZE, len / PAGE_SIZE, 0);
	if (start == IO_SPACE_PAGES && len > SZ_64K) {
		len = SZ_64K;
		start = 0;
		start = bitmap_find_next_zero_area(pci_iospace, IO_SPACE_PAGES,
					start, len / PAGE_SIZE, 0);
	}

	/* no 64K area found */
	if (start == IO_SPACE_PAGES)
		return -ENOMEM;

	/* ioremap physical aperture to virtual aperture */
	virt_start = start * PAGE_SIZE + (unsigned long)PCI_IOBASE;
	err = ioremap_page_range(virt_start, virt_start + len,
				phys_addr, __pgprot(PROT_DEVICE_nGnRE));
	if (err)
		return err;

	bitmap_set(pci_iospace, start, len / PAGE_SIZE);

	/* return io_offset */
	return start * PAGE_SIZE - res->start;
}
