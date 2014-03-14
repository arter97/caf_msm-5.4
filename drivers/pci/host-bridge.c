/*
 * host bridge related code
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/slab.h>

#include "pci.h"

static atomic_t domain_nr = ATOMIC_INIT(-1);

static struct pci_bus *find_pci_root_bus(struct pci_bus *bus)
{
	while (bus->parent)
		bus = bus->parent;

	return bus;
}

struct pci_host_bridge *find_pci_host_bridge(struct pci_bus *bus)
{
	struct pci_bus *root_bus = find_pci_root_bus(bus);

	return to_pci_host_bridge(root_bus->bridge);
}
EXPORT_SYMBOL_GPL(find_pci_host_bridge);

void pci_set_host_bridge_release(struct pci_host_bridge *bridge,
				 void (*release_fn)(struct pci_host_bridge *),
				 void *release_data)
{
	bridge->release_fn = release_fn;
	bridge->release_data = release_data;
}

static bool resource_contains(struct resource *res1, struct resource *res2)
{
	return res1->start <= res2->start && res1->end >= res2->end;
}

void pcibios_resource_to_bus(struct pci_bus *bus, struct pci_bus_region *region,
			     struct resource *res)
{
	struct pci_host_bridge *bridge = find_pci_host_bridge(bus);
	struct pci_host_bridge_window *window;
	resource_size_t offset = 0;

	list_for_each_entry(window, &bridge->windows, list) {
		if (resource_type(res) != resource_type(window->res))
			continue;

		if (resource_contains(window->res, res)) {
			offset = window->offset;
			break;
		}
	}

	region->start = res->start - offset;
	region->end = res->end - offset;
}
EXPORT_SYMBOL(pcibios_resource_to_bus);

static bool region_contains(struct pci_bus_region *region1,
			    struct pci_bus_region *region2)
{
	return region1->start <= region2->start && region1->end >= region2->end;
}

void pcibios_bus_to_resource(struct pci_bus *bus, struct resource *res,
			     struct pci_bus_region *region)
{
	struct pci_host_bridge *bridge = find_pci_host_bridge(bus);
	struct pci_host_bridge_window *window;
	resource_size_t offset = 0;

	list_for_each_entry(window, &bridge->windows, list) {
		struct pci_bus_region bus_region;

		if (resource_type(res) != resource_type(window->res))
			continue;

		bus_region.start = window->res->start - window->offset;
		bus_region.end = window->res->end - window->offset;

		if (region_contains(&bus_region, region)) {
			offset = window->offset;
			break;
		}
	}

	res->start = region->start + offset;
	res->end = region->end + offset;
}
EXPORT_SYMBOL(pcibios_bus_to_resource);

#ifdef CONFIG_OF
/**
 * Simple version of the platform specific code for filtering the list
 * of resources obtained from the ranges declaration in DT.
 *
 * Platforms can override this function in order to impose stronger
 * constraints onto the list of resources that a host bridge can use.
 * The filtered list will then be used to create a root bus and associate
 * it with the host bridge.
 *
 */
int __weak pcibios_fixup_bridge_ranges(struct list_head *resources)
{
	return 0;
}

/**
 * pci_host_bridge_of_get_ranges - Parse PCI host bridge resources from DT
 * @dev: device node of the host bridge having the range property
 * @resources: list where the range of resources will be added after DT parsing
 * @io_base: pointer to a variable that will contain the physical address for
 * the start of the I/O range.
 *
 * It is the callers job to free the @resources list if an error is returned.
 *
 * This function will parse the "ranges" property of a PCI host bridge device
 * node and setup the resource mapping based on its content. It is expected
 * that the property conforms with the Power ePAPR document.
 *
 * Each architecture is then offered the chance of applying their own
 * filtering of pci_host_bridge_windows based on their own restrictions by
 * calling pcibios_fixup_bridge_ranges(). The filtered list of windows
 * can then be used when creating a pci_host_bridge structure.
 */
static int pci_host_bridge_of_get_ranges(struct device_node *dev,
		struct list_head *resources, resource_size_t *io_base)
{
	struct resource *res;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	int err;

	pr_info("PCI host bridge %s ranges:\n", dev->full_name);

	/* Check for ranges property */
	err = of_pci_range_parser_init(&parser, dev);
	if (err)
		return err;

	pr_debug("Parsing ranges property...\n");
	for_each_of_pci_range(&parser, &range) {
		/* Read next ranges element */
		pr_debug("pci_space: 0x%08x pci_addr:0x%016llx ",
				range.pci_space, range.pci_addr);
		pr_debug("cpu_addr:0x%016llx size:0x%016llx\n",
					range.cpu_addr, range.size);

		/*
		 * If we failed translation or got a zero-sized region
		 * then skip this range
		 */
		if (range.cpu_addr == OF_BAD_ADDR || range.size == 0)
			continue;

		res = kzalloc(sizeof(struct resource), GFP_KERNEL);
		if (!res)
			return -ENOMEM;

		err = of_pci_range_to_resource(&range, dev, res);
		if (err)
			return err;

		if (resource_type(res) == IORESOURCE_IO)
			*io_base = range.cpu_addr;

		pci_add_resource_offset(resources, res,
				res->start - range.pci_addr);
	}

	/* Apply architecture specific fixups for the ranges */
	return pcibios_fixup_bridge_ranges(resources);
}

/**
 * of_create_pci_host_bridge - Create a PCI host bridge structure using
 * information passed in the DT.
 * @parent: device owning this host bridge
 * @ops: pci_ops associated with the host controller
 * @host_data: opaque data structure used by the host controller.
 *
 * returns a pointer to the newly created pci_host_bridge structure, or
 * NULL if the call failed.
 *
 * This function will try to obtain the host bridge domain number by
 * using of_alias_get_id() call with "pci-domain" as a stem. If that
 * fails, a local allocator will be used that will put each host bridge
 * in a new domain.
 */
struct pci_host_bridge *
of_create_pci_host_bridge(struct device *parent, struct pci_ops *ops, void *host_data)
{
	int err, domain, busno;
	struct resource *bus_range;
	struct pci_bus *root_bus;
	struct pci_host_bridge *bridge;
	resource_size_t io_base;
	LIST_HEAD(res);

	bus_range = kzalloc(sizeof(*bus_range), GFP_KERNEL);
	if (!bus_range)
		return ERR_PTR(-ENOMEM);

	domain = of_alias_get_id(parent->of_node, "pci-domain");
	if (domain == -ENODEV)
		domain = atomic_inc_return(&domain_nr);

	err = of_pci_parse_bus_range(parent->of_node, bus_range);
	if (err) {
		dev_info(parent, "No bus range for %s, using default [0-255]\n",
			parent->of_node->full_name);
		bus_range->start = 0;
		bus_range->end = 255;
		bus_range->flags = IORESOURCE_BUS;
	}
	busno = bus_range->start;
	pci_add_resource(&res, bus_range);

	/* now parse the rest of host bridge bus ranges */
	err = pci_host_bridge_of_get_ranges(parent->of_node, &res, &io_base);
	if (err)
		goto err_create;

	/* then create the root bus */
	root_bus = pci_create_root_bus_in_domain(parent, domain, busno,
						ops, host_data, &res);
	if (IS_ERR(root_bus)) {
		err = PTR_ERR(root_bus);
		goto err_create;
	}

	bridge = to_pci_host_bridge(root_bus->bridge);
	bridge->io_base = io_base;

	return bridge;

err_create:
	pci_free_resource_list(&res);
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(of_create_pci_host_bridge);

#endif /* CONFIG_OF */
