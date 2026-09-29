/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * bcm2712_mip -- the BCM2712 MSI-X Interrupt Peripheral (brcm,bcm2712-mip).
 *
 * The BCM2712's PCIe1 and PCIe2 controllers have no MSI receiver of their
 * own; their DT msi-parent is a MIP (PCIe0's internal one is not used on
 * the Pi 5).  A device raises MIP vector v by writing v to the doorbell, a
 * PCIe address (reg[1]) that the controller's last inbound window
 * (dma-ranges) sends to the MIP's registers.  Each vector drives its own
 * GIC SPI, edge-triggered.  From the DT and Linux irq-bcm2712-mip.c:
 *
 *	mip0 (PCIe2, RP1):   vectors  0-63 -> SPI 128-191
 *	mip1 (PCIe1, NVMe):  vectors  8-15 -> SPI 255-262
 *
 * msi-ranges = <&gic GIC_SPI base type count> names the SPI of vector 0
 * (SPI = base + vector) and the number of usable vectors; brcm,msi-offset
 * is the first usable vector.  mip1's other outputs reach the GIC too, but
 * not at consecutive SPIs, so Linux uses only those eight.
 *
 * That is the GICv2m model, so this driver follows arm_gicv2m in
 * sys/arm/arm/gic.c: it reserves its SPIs in the GIC as MSIs, allocates
 * from them with GIC_ALLOC_MSI()/GIC_ALLOC_MSIX(), and the interrupts are
 * the GIC's own, so they need no dispatch here.  Only the message data
 * differs: GICv2m's is the SPI's interrupt ID, the MIP's is the vector.
 *
 * rpi5_modules.git doc/M2_PCIE_HOST.md, phase 4.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/arm/gic.h>
#include <arm/arm/gic_common.h>

#include "gic_if.h"
#include "msi_if.h"

/* Registers (Linux irq-bcm2712-mip.c); bit n is vector n. */
#define	MIP_INT_RAISE		0x00	/* the doorbell's target */
#define	MIP_INT_CLEAR		0x10
#define	MIP_INT_CFGL_HOST	0x20	/* 1 = edge */
#define	MIP_INT_CFGH_HOST	0x30
#define	MIP_INT_MASKL_HOST	0x40	/* 1 = masked */
#define	MIP_INT_MASKH_HOST	0x50
#define	MIP_INT_MASKL_VPU	0x60
#define	MIP_INT_MASKH_VPU	0x70
#define	MIP_INT_STATUSL_HOST	0x80
#define	MIP_INT_STATUSH_HOST	0x90

#define	GIC_SPI_CELL		0	/* first cell of a GIC specifier */

struct bcm2712_mip_softc {
	device_t	dev;
	device_t	gic;
	struct resource	*mem;
	uint64_t	doorbell;	/* PCIe address */
	u_int		spi_base;	/* SPI of vector 0 */
	u_int		offset;		/* first usable vector */
	u_int		count;		/* usable vectors */
	u_int		irq_start;	/* GIC interrupt ID of vector offset */
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2712-mip",	1},
	{NULL,			0}
};

/*
 * The message data for a GIC interrupt taken from our range: its vector.
 */
static uint32_t
bcm2712_mip_vector(struct bcm2712_mip_softc *sc, struct intr_irqsrc *isrc)
{
	struct gic_irqsrc *gi = (struct gic_irqsrc *)isrc;

	return (gi->gi_irq - sc->irq_start + sc->offset);
}

static int
bcm2712_mip_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "BCM2712 MSI-X interrupt peripheral");
	return (BUS_PROBE_DEFAULT);
}

/*
 * reg[1], the doorbell, is a PCIe address and is not translated through
 * the parent's ranges (Linux of_property_read_reg() does not either).
 */
static int
bcm2712_mip_doorbell(phandle_t node, uint64_t *addr)
{
	pcell_t acells, scells, reg[8];
	int i, n;

	if (OF_getencprop(OF_parent(node), "#address-cells", &acells,
	    sizeof(acells)) <= 0)
		acells = 2;
	if (OF_getencprop(OF_parent(node), "#size-cells", &scells,
	    sizeof(scells)) <= 0)
		scells = 1;
	if (acells < 1 || acells > 2 || scells > 2)
		return (ENXIO);
	n = OF_getencprop(node, "reg", reg, sizeof(reg));
	if (n < (int)(2 * (acells + scells) * sizeof(pcell_t)))
		return (ENXIO);
	*addr = 0;
	for (i = 0; i < acells; i++)
		*addr = (*addr << 32) | reg[acells + scells + i];
	return (0);
}

static int
bcm2712_mip_attach(device_t dev)
{
	struct bcm2712_mip_softc *sc;
	phandle_t node, gicnode;
	pcell_t ranges[8], icells, offset;
	intptr_t xref;
	int n, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/* msi-ranges = <&gic specifier... count>: one range only. */
	n = OF_getencprop(node, "msi-ranges", ranges, sizeof(ranges));
	if (n < (int)(2 * sizeof(pcell_t))) {
		device_printf(dev, "no msi-ranges\n");
		return (ENXIO);
	}
	n /= sizeof(pcell_t);
	gicnode = OF_node_from_xref(ranges[0]);
	if (OF_getencprop(gicnode, "#interrupt-cells", &icells,
	    sizeof(icells)) <= 0 || icells < 2 || n != (int)icells + 2) {
		device_printf(dev, "cannot parse msi-ranges (%d cells)\n", n);
		return (ENXIO);
	}
	if (ranges[1 + GIC_SPI_CELL] != 0) {
		device_printf(dev, "msi-ranges is not a GIC SPI range\n");
		return (ENXIO);
	}
	sc->spi_base = ranges[2];
	sc->count = ranges[n - 1];
	if (OF_getencprop(node, "brcm,msi-offset", &offset,
	    sizeof(offset)) <= 0)
		offset = 0;
	sc->offset = offset;
	if (sc->count == 0 || sc->offset + sc->count > 64) {
		device_printf(dev, "bad range: vectors %u-%u\n", sc->offset,
		    sc->offset + sc->count - 1);
		return (ENXIO);
	}
	sc->irq_start = GIC_FIRST_SPI + sc->spi_base + sc->offset;

	/* The GIC_*() methods are arm_gic's (sys/arm/arm/gic.c), not v3's. */
	if (!ofw_bus_node_is_compatible(gicnode, "arm,gic-400")) {
		device_printf(dev, "msi-ranges parent is not a GIC-400\n");
		return (ENXIO);
	}
	sc->gic = OF_device_from_xref(ranges[0]);
	if (sc->gic == NULL) {
		device_printf(dev, "the GIC has not attached\n");
		return (ENXIO);
	}

	if (bcm2712_mip_doorbell(node, &sc->doorbell) != 0) {
		device_printf(dev, "cannot read the doorbell address (reg[1])\n");
		return (ENXIO);
	}

	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}

	/*
	 * As Linux: every vector unmasked for the host, masked for the VPU,
	 * and edge-triggered.
	 */
	bus_write_4(sc->mem, MIP_INT_MASKL_HOST, 0);
	bus_write_4(sc->mem, MIP_INT_MASKH_HOST, 0);
	bus_write_4(sc->mem, MIP_INT_MASKL_VPU, ~0u);
	bus_write_4(sc->mem, MIP_INT_MASKH_VPU, ~0u);
	bus_write_4(sc->mem, MIP_INT_CFGL_HOST, ~0u);
	bus_write_4(sc->mem, MIP_INT_CFGH_HOST, ~0u);

	GIC_RESERVE_MSI_RANGE(sc->gic, sc->irq_start, sc->count);

	xref = OF_xref_from_node(node);
	if (intr_msi_register(dev, xref) != 0) {
		device_printf(dev, "cannot register as an MSI controller\n");
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
		return (ENXIO);
	}

	device_printf(dev, "vectors %u-%u -> SPI %u-%u, doorbell PCIe 0x%jx\n",
	    sc->offset, sc->offset + sc->count - 1,
	    sc->spi_base + sc->offset, sc->spi_base + sc->offset + sc->count - 1,
	    (uintmax_t)sc->doorbell);
	return (0);
}

static int
bcm2712_mip_alloc_msi(device_t dev, device_t child, int count, int maxcount,
    device_t *pic, struct intr_irqsrc **srcs)
{
	struct bcm2712_mip_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = GIC_ALLOC_MSI(sc->gic, sc->irq_start, sc->count, count,
	    maxcount, srcs);
	if (error != 0)
		return (error);

	/*
	 * A multi-message MSI device puts the message number in the low bits
	 * of the data, so the first vector must be aligned to the count.  The
	 * GIC aligns the interrupt ID, which differs from the vector by
	 * GIC_FIRST_SPI + spi_base: 160 for mip0, 279 for mip1.  Refuse a
	 * block that is aligned there but not here; PCI then asks for fewer.
	 */
	if ((bcm2712_mip_vector(sc, srcs[0]) & (count - 1)) != 0) {
		GIC_RELEASE_MSI(sc->gic, count, srcs);
		return (ENXIO);
	}

	*pic = dev;
	return (0);
}

static int
bcm2712_mip_release_msi(device_t dev, device_t child, int count,
    struct intr_irqsrc **isrc)
{
	struct bcm2712_mip_softc *sc = device_get_softc(dev);

	return (GIC_RELEASE_MSI(sc->gic, count, isrc));
}

static int
bcm2712_mip_alloc_msix(device_t dev, device_t child, device_t *pic,
    struct intr_irqsrc **isrcp)
{
	struct bcm2712_mip_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = GIC_ALLOC_MSIX(sc->gic, sc->irq_start, sc->count, isrcp);
	if (error != 0)
		return (error);

	*pic = dev;
	return (0);
}

static int
bcm2712_mip_release_msix(device_t dev, device_t child,
    struct intr_irqsrc *isrc)
{
	struct bcm2712_mip_softc *sc = device_get_softc(dev);

	return (GIC_RELEASE_MSIX(sc->gic, isrc));
}

static int
bcm2712_mip_map_msi(device_t dev, device_t child, struct intr_irqsrc *isrc,
    uint64_t *addr, uint32_t *data)
{
	struct bcm2712_mip_softc *sc = device_get_softc(dev);

	*addr = sc->doorbell;
	*data = bcm2712_mip_vector(sc, isrc);
	return (0);
}

static device_method_t bcm2712_mip_methods[] = {
	DEVMETHOD(device_probe,		bcm2712_mip_probe),
	DEVMETHOD(device_attach,	bcm2712_mip_attach),

	DEVMETHOD(msi_alloc_msi,	bcm2712_mip_alloc_msi),
	DEVMETHOD(msi_release_msi,	bcm2712_mip_release_msi),
	DEVMETHOD(msi_alloc_msix,	bcm2712_mip_alloc_msix),
	DEVMETHOD(msi_release_msix,	bcm2712_mip_release_msix),
	DEVMETHOD(msi_map_msi,		bcm2712_mip_map_msi),

	DEVMETHOD_END
};

static DEFINE_CLASS_0(bcm2712_mip, bcm2712_mip_driver, bcm2712_mip_methods,
    sizeof(struct bcm2712_mip_softc));

/* After the GIC, and before the PCIe controllers enumerate. */
EARLY_DRIVER_MODULE(bcm2712_mip, simplebus, bcm2712_mip_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
