/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * rp1 -- the Raspberry Pi RP1 south bridge as a PCI device.
 *
 * On the FDT lane, RP1 (1de4:0001) is enumerated behind bcm2712_pcib, and
 * PCI places its BARs.  RP1's peripherals -- GPIO, PWM, clocks, Ethernet
 * and the rest -- are in BAR1, which does not land where the device tree's
 * rp1 ranges assume (PCIe 0).  This driver owns BAR1, enables bus mastering
 * so that RP1's DMA masters can reach RAM, and publishes BAR1 to the RP1
 * drivers through bcm2712_rp1_publish().  Drivers that were waiting for RP1
 * run then (rpi5_modules.git doc/M2_PCIE_HOST.md, phase 2).
 *
 * Linux drivers/mfd/rp1.c also maps BAR1 (pci_resource_start(pdev, 1)) and
 * is the parent of RP1's functions.  Here they remain separate drivers that
 * find their registers through bcm2712_fdt.h.
 *
 * On the ACPI lane EDK2 does not expose RP1 as a PCI device, so this
 * driver never attaches there.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <arm64/broadcom/bcm2712/bcm2712_var.h>

#define	RP1_VENDOR	0x1de4
#define	RP1_DEVICE	0x0001

struct rp1_softc {
	device_t	dev;
	struct resource	*bar1;
	int		bar1_rid;
};

static int
rp1_probe(device_t dev)
{

	if (pci_get_vendor(dev) != RP1_VENDOR ||
	    pci_get_device(dev) != RP1_DEVICE)
		return (ENXIO);
	device_set_desc(dev, "Raspberry Pi RP1 south bridge");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_attach(device_t dev)
{
	struct rp1_softc *sc;
	bus_addr_t pa;
	bus_size_t size;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->bar1_rid = PCIR_BAR(1);
	sc->bar1 = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->bar1_rid,
	    RF_ACTIVE);
	if (sc->bar1 == NULL) {
		device_printf(dev, "cannot allocate BAR1\n");
		return (ENXIO);
	}

	/*
	 * BAR1's CPU physical address, for the RP1 drivers that map their
	 * windows with pmap_mapdev_attr().  The resource holds the PCI bus
	 * address; the mapping the activation made gives the CPU one.
	 */
	pa = pmap_kextract((vm_offset_t)rman_get_bushandle(sc->bar1));
	size = rman_get_size(sc->bar1);

	/* RP1's GEM, xHCI and DMA controllers master the bus. */
	pci_enable_busmaster(dev);

	device_printf(dev, "BAR1 (peripherals): PCIe 0x%jx -> CPU 0x%jx, "
	    "%ju KB\n", (uintmax_t)rman_get_start(sc->bar1), (uintmax_t)pa,
	    (uintmax_t)size / 1024);

	bcm2712_rp1_publish(pa, size);
	return (0);
}

static int
rp1_detach(device_t dev)
{

	/* The RP1 drivers keep mappings of BAR1; RP1 cannot go away. */
	return (EBUSY);
}

static device_method_t rp1_methods[] = {
	DEVMETHOD(device_probe,		rp1_probe),
	DEVMETHOD(device_attach,	rp1_attach),
	DEVMETHOD(device_detach,	rp1_detach),

	DEVMETHOD_END
};

/* "rp1pci0", not "rp10": a unit number run onto a name ending in a digit. */
static driver_t rp1_driver = {
	"rp1pci",
	rp1_methods,
	sizeof(struct rp1_softc),
};

DRIVER_MODULE(rp1, pci, rp1_driver, NULL, NULL);
MODULE_DEPEND(rp1, pci, 1, 1, 1);
MODULE_DEPEND(rp1, bcm2712, 1, 1, 1);
MODULE_VERSION(rp1, 1);
