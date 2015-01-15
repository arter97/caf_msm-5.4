#ifndef __ASM_PCI_H
#define __ASM_PCI_H
#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm-generic/pci-bridge.h>
#include <asm-generic/pci-dma-compat.h>

#define PCIBIOS_MIN_IO		0x1000
#define PCIBIOS_MIN_MEM		0

struct pci_host_bridge *find_pci_host_bridge(struct pci_bus *bus);

/*
 * Set to 1 if the kernel should re-assign all PCI bus numbers
 */
#define pcibios_assign_all_busses() \
	(pci_has_flag(PCI_REASSIGN_ALL_BUS))

/*
 * PCI address space differs from physical memory address space
 */
#define PCI_DMA_BUS_IS_PHYS	(0)

extern int isa_dma_bridge_buggy;

#ifdef CONFIG_PCI
static inline int pci_domain_nr(struct pci_bus *bus)
{
	struct pci_host_bridge *bridge = find_pci_host_bridge(bus);

	if (bridge)
		return bridge->domain_nr;

	return 0;
}

static inline int pci_proc_domain(struct pci_bus *bus)
{
	return 1;
}
#endif

extern unsigned long pci_ioremap_io(const struct resource *res, phys_addr_t phys_addr);

#endif  /* __KERNEL__ */
#endif  /* __ASM_PCI_H */
