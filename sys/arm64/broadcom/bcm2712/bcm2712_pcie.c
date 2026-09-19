/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Contributors
 * All rights reserved.
 *
 * bcm2712_pcie — BCM2712 PCIe2→RP1 interrupt router (Milestone 3)
 *
 * Routes the RP1 GEM Ethernet MAC's interrupt to rp1_eth.  This is not a PCIe
 * host controller driver: it exists only to own a device_t that can legally
 * call bus_setup_intr() on the GIC line the RP1 MSI arrives on, and to map
 * enough of the GEM to tell whether the GEM was the source.
 *
 * The interrupt is shared with xhci0/xhci1.  This driver acts as a
 * filter-only handler: it reads CGEM_INT_STATUS and dispatches to
 * rp1_eth's ISR if the GEM fired.
 *
 * Discovery is by Device Tree, and registers are mapped by physical address,
 * the same way every other driver in this set reaches RP1.  The driver
 * registers under nexus rather than simplebus because, until there is a
 * brcm,bcm2712-pcie host controller driver, nothing enumerates the RP1
 * subtree of the device tree and there is no simplebus over it.  The FDT is
 * still available for discovery: machdep.c installs and initialises OFW
 * unconditionally, before bus_probe() picks a bus method.
 *
 * This replaces an earlier ACPI attachment, which matched a _HID of "BCM2712"
 * injected into the RP1B scope by a hand-written DSDT override in
 * /boot/acpi_dsdt.aml.  That override lived in no repository, and the FDT the
 * firmware already publishes describes the same hardware.
 *
 * KPI exported for rp1_eth:
 *   void bcm2712_pcie_register_rp1_intr(driver_filter_t *filter, void *arg)
 *   void bcm2712_pcie_deregister_rp1_intr(void)
 *   void bcm2712_pcie_gem_iack(void)
 *
 * References:
 *   sys/dev/cadence/if_cgem.c (CGEM_INT_STATUS definition)
 *   sys/arm64/broadcom/rp1/rp1_eth_var.h (RP1 physical address derivation)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include "opt_acpi.h"

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>
#endif

#include "bcm2712_pcie.h"

/* GEM interrupt status register — checked in the filter to confirm GEM fired */
#define CGEM_INT_STATUS		0x024
#define CGEM_INT_RX_COMPLETE	(1u << 1)
#define CGEM_INT_RX_USED_READ	(1u << 2)
#define CGEM_INT_TX_COMPLETE	(1u << 7)
#define CGEM_INT_TX_USED_READ	(1u << 3)
#define CGEM_INT_HRESP_NOT_OK	(1u << 11)
#define CGEM_INT_RX_OVERRUN	(1u << 10)
#define CGEM_INT_ANY		(CGEM_INT_RX_COMPLETE | CGEM_INT_RX_USED_READ | \
				 CGEM_INT_TX_COMPLETE | CGEM_INT_TX_USED_READ | \
				 CGEM_INT_HRESP_NOT_OK | CGEM_INT_RX_OVERRUN)

/*
 * RP1 register windows, as CPU physical addresses seen through the PCIe2
 * outbound window the VPU firmware programmed.  These match
 * RP1_ETH_MAC_BASE_PHYS / RP1_ETH_CFG_BASE_PHYS in rp1_eth_var.h; the
 * derivation is documented there.
 */
#define GEM_MAC_PHYS		0x1f00100000UL	/* ethernet@100000 */
#define GEM_MAC_SIZE		0x1000
#define ETH_CFG_PHYS		0x1f00104000UL	/* eth_cfg@104000 */
#define ETH_CFG_SIZE		0x1000

/* RP1 PCIE_CFG registers for MSIx IACK re-arm (RP-008370-DS-1 ss6.2) */
#define PCIE_CFG_PHYS       0x1f00108000UL  /* BCM2712 CPU physical address */
#define PCIE_CFG_SIZE       0x200
#define PCIE_CFG_MSIX_CFG_0 0x008   /* MSIX_CFG_n base; vector n at offset +n*4 */
#define MSIX_CFG_IACK       (1u << 2) /* SC: write 1 to re-arm vector */
#define PCIE_CFG_INTSTATL   0x108   /* vectors 0-31 assertion status (RO) */
#define PCIE_CFG_INTSTATH   0x10c   /* vectors 32-63 assertion status (RO) */
#define RP1_INT_ETH          6      /* RP1 GEM MSI vector (INTSTATL bit 6 = 0x40, verified at runtime) */

/*
 * The shared line, as a GIC interrupt specifier.
 *
 * pcie@1000120000 in the Pi 5 device tree routes INTA to GIC SPI 229:
 *   interrupt-map = <0 0 0 1 &gicv2 GIC_SPI 229 IRQ_TYPE_LEVEL_HIGH>, ...
 * We do not walk that map, because without a host controller driver there is
 * no PCI device to map an INTx pin for.  We name the line directly, which is
 * what the ACPI DSDT override also did (GSI 261 = the same SPI).
 *
 * GIC-400 uses three interrupt cells: <type, number, flags>.
 */
#define GIC_ICELLS		3
#define GIC_TYPE_SPI		0
#define GIC_IRQ_LEVEL_HIGH	4
#define RP1_GEM_GIC_SPI		229

#ifndef DEV_ACPI
/* Where the GEM lives in the device trees this board is known to publish. */
static const char *bcm2712_pcie_gem_paths[] = {
	"/axi/pcie@1000120000/rp1/ethernet@100000",
	"/soc/rp1/ethernet@100000",
	NULL
};
#endif /* !DEV_ACPI */

struct bcm2712_pcie_softc {
	device_t	 dev;
	struct resource	*mac_res;	/* SYS_RES_MEMORY rid 0: GEM MAC */
	struct resource	*cfg_res;	/* SYS_RES_MEMORY rid 1: eth_cfg */
	struct resource	*irq_res;	/* SYS_RES_IRQ    rid 0: shared */
	void		*intr_cookie;
	bus_space_tag_t	 pciecfg_bst;
	bus_space_handle_t pciecfg_bsh;
	int		 pciecfg_mapped;
};

/*
 * Module-level callback storage for rp1_eth's interrupt filter.
 *
 * These are intentionally NOT in bcm2712_pcie_softc.  The rp1_eth module
 * is often loaded from the boot loader and calls
 * bcm2712_pcie_register_rp1_intr() before bcm2712_pcie0 has probed/attached.
 * By storing the callback here, the registration succeeds at any time, and
 * the interrupt filter picks it up as soon as bcm2712_pcie0 hooks the GIC
 * line.
 *
 * Ordering contract (both store and load use rel/acq barriers):
 *   register:   store arg first, then filter (filter == NULL ⇒ arg ignored)
 *   deregister: clear filter first, then arg (filter == NULL ⇒ arg never read)
 */
static volatile uintptr_t g_rp1_filter;	/* atomic: driver_filter_t * */
static volatile uintptr_t g_rp1_arg;	/* atomic: void * */

/* Module-level PCIE_CFG handle for bcm2712_pcie_gem_iack() KPI */
static bus_space_tag_t    g_pciecfg_bst;
static bus_space_handle_t g_pciecfg_bsh;
static int                g_pciecfg_mapped;

/*
 * KPI: rp1_eth calls this to register its GEM interrupt filter.
 * Safe to call before or after bcm2712_pcie0 attaches.
 */
void
bcm2712_pcie_register_rp1_intr(driver_filter_t *filter, void *arg)
{
	/* Store arg before filter so the ISR never sees a stale arg. */
	atomic_store_rel_ptr(&g_rp1_arg, (uintptr_t)arg);
	atomic_store_rel_ptr(&g_rp1_filter, (uintptr_t)filter);
}

void
bcm2712_pcie_deregister_rp1_intr(void)
{
	/* Clear filter before arg so the ISR never fires with a stale arg. */
	atomic_store_rel_ptr(&g_rp1_filter, (uintptr_t)NULL);
	atomic_store_rel_ptr(&g_rp1_arg, (uintptr_t)NULL);
}

/*
 * Interrupt filter: called at interrupt level, no sleeping.
 * Read GEM INT_STATUS directly (the MAC resource is mapped at attach).
 * If GEM bits are set, dispatch to rp1_eth's filter.
 * Return FILTER_STRAY if GEM is not the source so xhci handlers run.
 *
 * This deliberately does not touch MSIX IACK; see bcm2712_pcie_gem_iack().
 */
static int
bcm2712_pcie_filter(void *arg)
{
	struct bcm2712_pcie_softc *sc = arg;
	driver_filter_t *filter;
	void *filter_arg;
	uint32_t istat;

	istat = bus_read_4(sc->mac_res, CGEM_INT_STATUS);
	if ((istat & CGEM_INT_ANY) == 0)
		return (FILTER_STRAY);

	filter = (driver_filter_t *)atomic_load_acq_ptr(&g_rp1_filter);
	if (filter == NULL)
		return (FILTER_STRAY);
	filter_arg = (void *)atomic_load_acq_ptr(&g_rp1_arg);

	return (filter(filter_arg));
}

/*
 * KPI: called by rp1_eth after reading (clearing) GEM INT_STATUS and
 * re-enabling GEM interrupts.  At that point the GEM interrupt source has
 * de-asserted, so IACK re-arms the vector safely.  A new MSI fires only if a
 * packet arrived after INT_STATUS was cleared.
 * MUST NOT be called from the filter -- INT_STATUS still set there causes
 * an immediate re-fire loop (interrupt storm).
 */
void
bcm2712_pcie_gem_iack(void)
{
	if (g_pciecfg_mapped)
		bus_space_write_4(g_pciecfg_bst, g_pciecfg_bsh,
		    PCIE_CFG_MSIX_CFG_0 + RP1_INT_ETH * 4, MSIX_CFG_IACK);
}

/*
 * Is this board's device tree describing an RP1 GEM?  Used as the presence
 * test; the registers themselves are reached by physical address.
 */
#ifndef DEV_ACPI
static phandle_t
bcm2712_pcie_find_gem_node(void)
{
	phandle_t node;
	int i;

	for (i = 0; bcm2712_pcie_gem_paths[i] != NULL; i++) {
		node = OF_finddevice(bcm2712_pcie_gem_paths[i]);
		if (node != -1 &&
		    ofw_bus_node_is_compatible(node, "raspberrypi,rp1-gem"))
			return (node);
	}
	return (-1);
}
#endif /* !DEV_ACPI */

#ifdef DEV_ACPI
/*
 * ACPI attachment.  Matches the "BCM2712" device injected into the RP1B scope
 * by the DSDT override in /boot/acpi_dsdt.aml, which supplies both MMIO
 * windows and the shared interrupt (GSI 261 = GIC SPI 229) as _CRS resources.
 */
static int
bcm2712_pcie_probe(device_t dev)
{
	static char *ids[] = { "BCM2712", NULL };

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, ids, NULL) > 0)
		return (ENXIO);
	device_set_desc(dev, "BCM2712 PCIe2/RP1 GEM interrupt router");
	return (BUS_PROBE_DEFAULT);
}
#else
static void
bcm2712_pcie_identify(driver_t *driver, device_t parent)
{
	if (bcm2712_pcie_find_gem_node() == -1)
		return;
	if (device_find_child(parent, "bcm2712_pcie", -1) != NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "bcm2712_pcie", -1) == NULL)
		device_printf(parent,
		    "bcm2712_pcie: BUS_ADD_CHILD failed\n");
}

static int
bcm2712_pcie_probe(device_t dev)
{
	if (bcm2712_pcie_find_gem_node() == -1)
		return (ENXIO);
	device_set_desc(dev, "BCM2712 PCIe2/RP1 GEM interrupt router");
	return (BUS_PROBE_DEFAULT);
}
#endif /* DEV_ACPI */

/*
 * Map GIC SPI 229 to an interrupt number we can allocate.  Returns 0 on
 * failure.  The GIC is found by compatible rather than by path, and its xref
 * is what intr_map_irq() keys on.
 */
#ifndef DEV_ACPI
static u_int
bcm2712_pcie_map_gic_spi(device_t dev, u_int spi)
{
	phandle_t gic;
	pcell_t cells[GIC_ICELLS];

	gic = ofw_bus_find_compatible(OF_peer(0), "arm,gic-400");
	if (gic == 0 || gic == -1) {
		device_printf(dev, "cannot find arm,gic-400 node in FDT\n");
		return (0);
	}

	cells[0] = GIC_TYPE_SPI;
	cells[1] = spi;
	cells[2] = GIC_IRQ_LEVEL_HIGH;

	return (ofw_bus_map_intr(dev, OF_xref_from_node(gic), GIC_ICELLS,
	    cells));
}
#endif /* !DEV_ACPI */

static int
bcm2712_pcie_attach(device_t dev)
{
	struct bcm2712_pcie_softc *sc = device_get_softc(dev);
	int rid, error;
#ifndef DEV_ACPI
	u_int irq;
#endif

	sc->dev = dev;

#ifndef DEV_ACPI
	/*
	 * Nothing enumerated this device from the device tree, so it has no
	 * resources yet; name them here.  nexus keeps a resource list per
	 * child (bus_generic_rl_set_resource), so bus_alloc_resource_any()
	 * below finds them.  Under ACPI the _CRS supplies both windows and the
	 * interrupt instead, and all of this is skipped.
	 */
	error = bus_set_resource(dev, SYS_RES_MEMORY, 0, GEM_MAC_PHYS,
	    GEM_MAC_SIZE);
	if (error != 0) {
		device_printf(dev, "cannot set GEM MAC resource: %d\n", error);
		return (error);
	}
	error = bus_set_resource(dev, SYS_RES_MEMORY, 1, ETH_CFG_PHYS,
	    ETH_CFG_SIZE);
	if (error != 0)
		device_printf(dev, "warning: cannot set eth_cfg resource: %d\n",
		    error);
#endif

	/* Map GEM MAC registers (memory resource 0) */
	rid = 0;
	sc->mac_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mac_res == NULL) {
		device_printf(dev, "cannot map GEM MAC registers\n");
		return (ENXIO);
	}

	/* Map RP1 PCIE_CFG for MSIx IACK re-arm (direct physical map) */
	sc->pciecfg_bst = rman_get_bustag(sc->mac_res);
	if (bus_space_map(sc->pciecfg_bst, PCIE_CFG_PHYS, PCIE_CFG_SIZE, 0,
	    &sc->pciecfg_bsh) == 0) {
		sc->pciecfg_mapped = 1;
		g_pciecfg_bst = sc->pciecfg_bst;
		g_pciecfg_bsh = sc->pciecfg_bsh;
		g_pciecfg_mapped = 1;
	} else {
		device_printf(dev, "warning: cannot map PCIE_CFG, IACK disabled\n");
		sc->pciecfg_mapped = 0;
	}

	/* Map eth_cfg registers (memory resource 1) — optional for now */
	rid = 1;
	sc->cfg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->cfg_res == NULL)
		device_printf(dev, "warning: cannot map eth_cfg registers\n");

	/* Allocate the shared interrupt (GIC SPI 229). */
#ifndef DEV_ACPI
	irq = bcm2712_pcie_map_gic_spi(dev, RP1_GEM_GIC_SPI);
	if (irq == 0) {
		device_printf(dev, "cannot map GIC SPI %d\n", RP1_GEM_GIC_SPI);
		error = ENXIO;
		goto fail_mem;
	}
	error = bus_set_resource(dev, SYS_RES_IRQ, 0, irq, 1);
	if (error != 0) {
		device_printf(dev, "cannot set IRQ resource: %d\n", error);
		goto fail_mem;
	}
#endif

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		error = ENXIO;
		goto fail_mem;
	}

	error = bus_setup_intr(dev, sc->irq_res,
	    INTR_TYPE_NET | INTR_MPSAFE,
	    bcm2712_pcie_filter, NULL, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt: %d\n", error);
		goto fail_irq;
	}

	device_printf(dev,
	    "GEM MAC mapped at %#jx, IRQ hooked (shared GIC SPI %d)\n",
	    (uintmax_t)rman_get_start(sc->mac_res), RP1_GEM_GIC_SPI);
	return (0);

fail_irq:
	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
fail_mem:
	if (sc->pciecfg_mapped) {
		g_pciecfg_mapped = 0;
		bus_space_unmap(sc->pciecfg_bst, sc->pciecfg_bsh,
		    PCIE_CFG_SIZE);
		sc->pciecfg_mapped = 0;
	}
	if (sc->cfg_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->cfg_res);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mac_res);
	return (error);
}

static int
bcm2712_pcie_detach(device_t dev)
{
	struct bcm2712_pcie_softc *sc = device_get_softc(dev);

	bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->pciecfg_mapped) {
		g_pciecfg_mapped = 0;
		bus_space_unmap(sc->pciecfg_bst, sc->pciecfg_bsh,
		    PCIE_CFG_SIZE);
		sc->pciecfg_mapped = 0;
	}
	if (sc->cfg_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->cfg_res);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mac_res);
	return (0);
}

static device_method_t bcm2712_pcie_methods[] = {
#ifndef DEV_ACPI
	DEVMETHOD(device_identify,	bcm2712_pcie_identify),
#endif
	DEVMETHOD(device_probe,		bcm2712_pcie_probe),
	DEVMETHOD(device_attach,	bcm2712_pcie_attach),
	DEVMETHOD(device_detach,	bcm2712_pcie_detach),
	DEVMETHOD_END
};

static driver_t bcm2712_pcie_driver = {
	"bcm2712_pcie",
	bcm2712_pcie_methods,
	sizeof(struct bcm2712_pcie_softc),
};

/*
 * Which bus this driver rides is decided when the kernel is configured, not at
 * run time.  A kernel with "device acpi" (RPI5) takes the ACPI attachment and
 * still needs the DSDT override; a kernel without it (RPI5-FDT) takes the
 * device tree attachment and needs no override.
 */
#ifdef DEV_ACPI
DRIVER_MODULE(bcm2712_pcie, acpi, bcm2712_pcie_driver, NULL, NULL);
MODULE_DEPEND(bcm2712_pcie, acpi, 1, 1, 1);
#else
DRIVER_MODULE(bcm2712_pcie, nexus, bcm2712_pcie_driver, NULL, NULL);
#endif
MODULE_VERSION(bcm2712_pcie, 1);
