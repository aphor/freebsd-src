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
 * the same way every other driver in this set reaches RP1.  On the FDT lane
 * RP1 is a PCI device behind bcm2712_pcib, and this driver attaches below
 * rp1pci, the RP1 PCI driver, once it has published BAR1: RP1's windows are
 * found relative to BAR1 (bcm2712_fdt.h), and the interrupt is RP1's INTA,
 * found through the root complex's interrupt-map (rpi5_modules.git
 * doc/M2_PCIE_HOST.md, phase 2).  The FDT is available for discovery on
 * both lanes: machdep.c installs and initialises OFW unconditionally, before
 * bus_probe() picks a bus method.
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

#include "bcm2712_fdt.h"
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
 * RP1 register windows, as offsets into RP1's peripheral BAR (BAR1).  On the
 * ACPI lane the GEM window comes from _CRS instead (EDK2 places BAR1 at CPU
 * 0x1f_00000000; see RP1_ETH_MAC_BASE_PHYS in rp1_eth_var.h).  On the FDT
 * lane the GEM is resolved from its node against the published BAR1, and
 * GEM_MAC_OFFSET is the fallback.
 *
 * eth_cfg, at 0x104000, is not mapped here: nothing in this driver uses it.
 * No device tree describes it; see rp1_eth_cfg.c.
 */
#define GEM_MAC_OFFSET		0x100000	/* ethernet@100000 */
#define GEM_MAC_SIZE		0x1000

/*
 * RP1 PCIE_CFG registers for MSIx IACK re-arm (RP-008370-DS-1 ss6.2).
 *
 * ToDo: absent from every device tree.  RP1's PCIe endpoint config block at
 * 0x108000 has no node in the vendor DTB or in Linux's rp1.dtsi: on Linux,
 * drivers/mfd/rp1.c is RP1's interrupt controller and reaches the block
 * through BAR1 itself, so nothing needs to name it.  Until rp1pci does the
 * same, the address is derived: BAR1 + 0x108000, where BAR1 is EDK2's
 * 0x1f_00000000 on the ACPI lane and the published BAR1 on the FDT lane.
 */
#define PCIE_CFG_PHYS       0x1f00108000UL  /* ACPI lane; see above */
#define PCIE_CFG_OFFSET     0x108000        /* in RP1's peripheral BAR */
#define PCIE_CFG_SIZE       0x200
#define PCIE_CFG_MSIX_CFG_0 0x008   /* MSIX_CFG_n base; vector n at offset +n*4 */
#define MSIX_CFG_IACK       (1u << 2) /* SC: write 1 to re-arm vector */
#define PCIE_CFG_INTSTATL   0x108   /* vectors 0-31 assertion status (RO) */
#define PCIE_CFG_INTSTATH   0x10c   /* vectors 32-63 assertion status (RO) */
/*
 * RP1 GEM MSI vector.  This one *is* in the device tree: ethernet@100000 has
 * interrupts = <6 4> against rp1's #interrupt-cells = <2>, so cell 0 is the
 * RP1 interrupt number and cell 1 the trigger type.  Resolved at attach into
 * rp1_int_eth; the constant is the fallback and matches INTSTATL bit 6
 * (0x40), verified at runtime.
 */
#define RP1_INT_ETH          6      /* fallback; see rp1_int_eth */

/*
 * The shared line, as a GIC interrupt specifier.
 *
 * pcie@1000120000 in the Pi 5 device tree routes INTA to GIC SPI 229:
 *   interrupt-map = <0 0 0 1 &gicv2 GIC_SPI 229 IRQ_TYPE_LEVEL_HIGH>, ...
 * The ACPI DSDT override named the same line as GSI 261, and the GEM's
 * interrupts arrive on it there, so RP1 raises them as INTA.  RP1 advertises
 * INTA in its Interrupt Pin register, and its MSI-X capability is left
 * disabled on both lanes.  This driver is not RP1's PCI driver, so it looks
 * the INTA entry up itself rather than through pci_alloc_resource().
 *
 * GIC-400 uses three interrupt cells: <type, number, flags>.
 */
#define GIC_ICELLS		3
#define GIC_TYPE_SPI		0
#define GIC_IRQ_LEVEL_HIGH	4
#define RP1_GEM_GIC_SPI		229	/* fallback; see the interrupt-map walk */

/*
 * interrupt-map layout on pcie@1000120000, with #address-cells = <3> and
 * #interrupt-cells = <1> on the PCI side and 3 GIC cells on the other:
 *
 *   child-addr[3]  child-irq[1]  parent-phandle[1]  parent-irq[3]
 *
 * so eight cells per entry, and the entry whose child-irq is 1 (INTA) is the
 * one the GEM's MSI is routed onto -- <0 0 0 1 &gicv2 0 229 4>.
 */
#define PCIE_IMAP_ENTRY_CELLS	8
#define PCIE_IMAP_CHILD_IRQ	3	/* index of child-irq within an entry */
#define PCIE_IMAP_IPARENT	4
#define PCIE_IMAP_PARENT_IRQ	5
#define PCIE_INTA		1

/* Where the GEM lives in the device trees this board is known to publish. */
static const char * const bcm2712_pcie_gem_paths[] = {
	"/axi/pcie@1000120000/rp1/ethernet@100000",
	"/soc/rp1/ethernet@100000",
	NULL
};

#ifndef DEV_ACPI
/* The PCIe2 root complex the RP1 hangs off, for its interrupt-map. */
static const char * const bcm2712_pcie_rc_paths[] = {
	"/axi/pcie@1000120000",
	"/soc/pcie@1000120000",
	NULL
};
#endif /* !DEV_ACPI */

/* RP1 MSI-X vector for the GEM, from the device tree where available. */
static u_int rp1_int_eth = RP1_INT_ETH;

struct bcm2712_pcie_softc {
	device_t	 dev;
	struct resource	*mac_res;	/* SYS_RES_MEMORY rid 0: GEM MAC (ACPI) */
	struct resource	*cfg_res;	/* SYS_RES_MEMORY rid 1: eth_cfg (ACPI) */
	struct resource	*irq_res;	/* SYS_RES_IRQ    rid 0: shared */
	void		*intr_cookie;
	bus_space_tag_t	 mac_bst;	/* GEM MAC, however it was mapped */
	bus_space_handle_t mac_bsh;
	int		 mac_mapped;	/* by bus_space_map (FDT) */
	bus_addr_t	 mac_phys;
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

	istat = bus_space_read_4(sc->mac_bst, sc->mac_bsh, CGEM_INT_STATUS);
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
		    PCIE_CFG_MSIX_CFG_0 + rp1_int_eth * 4, MSIX_CFG_IACK);
}

/*
 * Is this board's device tree describing an RP1 GEM?  Used as the presence
 * test; the registers themselves are reached by physical address.
 */
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

/*
 * Resolve the GEM's RP1 interrupt number from its interrupts property.  Used
 * on both bus methods: OFW is initialised before bus_probe() chooses one, so
 * the device tree is readable even on an ACPI boot.
 */
static void
bcm2712_pcie_resolve_int_eth(device_t dev)
{
	phandle_t gem;
	pcell_t cells[2];
	int len;

	gem = bcm2712_pcie_find_gem_node();
	if (gem == -1)
		return;
	len = OF_getencprop(gem, "interrupts", cells, sizeof(cells));
	if (len < (int)sizeof(cells[0])) {
		device_printf(dev, "no interrupts property on the GEM node, "
		    "using RP1 vector %d\n", RP1_INT_ETH);
		return;
	}
	rp1_int_eth = cells[0];
	if (rp1_int_eth != RP1_INT_ETH)
		device_printf(dev, "RP1 GEM vector %u from FDT (built-in "
		    "default is %d)\n", rp1_int_eth, RP1_INT_ETH);
}

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
/* Called by rp1pci, after it has published BAR1. */
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
/*
 * Map the shared RP1 line to an interrupt number we can allocate.  Returns 0
 * on failure.
 *
 * Preferred source is pcie@1000120000's interrupt-map: its INTA entry names
 * both the interrupt parent and the three GIC cells, so neither the SPI
 * number nor the trigger type has to be hardcoded.  If the map is missing or
 * has no INTA entry, fall back to naming GIC SPI 229 against the
 * arm,gic-400 node -- which is what the ACPI DSDT override did as GSI 261.
 */
static u_int
bcm2712_pcie_map_gic_spi(device_t dev, u_int spi)
{
	phandle_t rc, gic;
	pcell_t imap[PCIE_IMAP_ENTRY_CELLS * 8];
	pcell_t cells[GIC_ICELLS];
	int len, n, i;

	rc = bcm2712_fdt_find(bcm2712_pcie_rc_paths, "brcm,bcm2712-pcie");
	if (rc != -1) {
		len = OF_getencprop(rc, "interrupt-map", imap, sizeof(imap));
		n = (len > 0) ? len / (int)sizeof(imap[0]) : 0;
		for (i = 0; i + PCIE_IMAP_ENTRY_CELLS <= n;
		    i += PCIE_IMAP_ENTRY_CELLS) {
			if (imap[i + PCIE_IMAP_CHILD_IRQ] != PCIE_INTA)
				continue;
			device_printf(dev, "INTA -> GIC <%u %u %u> from the "
			    "pcie interrupt-map\n",
			    imap[i + PCIE_IMAP_PARENT_IRQ + 0],
			    imap[i + PCIE_IMAP_PARENT_IRQ + 1],
			    imap[i + PCIE_IMAP_PARENT_IRQ + 2]);
			return (ofw_bus_map_intr(dev,
			    imap[i + PCIE_IMAP_IPARENT], GIC_ICELLS,
			    &imap[i + PCIE_IMAP_PARENT_IRQ]));
		}
	}

	gic = ofw_bus_find_compatible(OF_peer(0), "arm,gic-400");
	if (gic == 0 || gic == -1) {
		device_printf(dev, "cannot find arm,gic-400 node in FDT\n");
		return (0);
	}
	device_printf(dev, "no INTA entry in the pcie interrupt-map; "
	    "naming GIC SPI %u\n", spi);

	cells[0] = GIC_TYPE_SPI;
	cells[1] = spi;
	cells[2] = GIC_IRQ_LEVEL_HIGH;

	return (ofw_bus_map_intr(dev, OF_xref_from_node(gic), GIC_ICELLS,
	    cells));
}
#endif /* !DEV_ACPI */

/*
 * Map the GEM MAC window and RP1's PCIE_CFG block.  Under ACPI the MAC window
 * is the device's first _CRS memory resource.  On the FDT lane both are
 * inside BAR1, which rp1pci owns, so they are mapped directly by address.
 */
static int
bcm2712_pcie_map_regs(device_t dev, struct bcm2712_pcie_softc *sc)
{
	bus_addr_t pciecfg_phys;
#ifdef DEV_ACPI
	int rid;
#else
	bus_addr_t bar_pa;
#endif

#ifdef DEV_ACPI
	rid = 0;
	sc->mac_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mac_res == NULL) {
		device_printf(dev, "cannot map GEM MAC registers\n");
		return (ENXIO);
	}
	sc->mac_bst = rman_get_bustag(sc->mac_res);
	sc->mac_bsh = rman_get_bushandle(sc->mac_res);
	sc->mac_phys = rman_get_start(sc->mac_res);

	/* Map eth_cfg registers (memory resource 1) — optional for now */
	rid = 1;
	sc->cfg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->cfg_res == NULL)
		device_printf(dev, "warning: cannot map eth_cfg registers\n");

	pciecfg_phys = PCIE_CFG_PHYS;
#else
	if (!bcm2712_rp1_bar(&bar_pa, NULL)) {
		device_printf(dev, "RP1's BAR1 has not been published\n");
		return (ENXIO);
	}
	sc->mac_phys = bar_pa + GEM_MAC_OFFSET;
	if (!bcm2712_fdt_rp1(bcm2712_pcie_gem_paths, "raspberrypi,rp1-gem", 0,
	    &sc->mac_phys, NULL))
		device_printf(dev, "GEM reg not resolvable from FDT, "
		    "using BAR1 + %#x\n", GEM_MAC_OFFSET);
	sc->mac_bst = bus_get_bus_tag(dev);
	if (bus_space_map(sc->mac_bst, sc->mac_phys, GEM_MAC_SIZE, 0,
	    &sc->mac_bsh) != 0) {
		device_printf(dev, "cannot map GEM MAC registers at %#jx\n",
		    (uintmax_t)sc->mac_phys);
		return (ENXIO);
	}
	sc->mac_mapped = 1;

	pciecfg_phys = bar_pa + PCIE_CFG_OFFSET;
#endif

	/* Map RP1 PCIE_CFG for MSIx IACK re-arm (direct physical map) */
	sc->pciecfg_bst = sc->mac_bst;
	if (bus_space_map(sc->pciecfg_bst, pciecfg_phys, PCIE_CFG_SIZE, 0,
	    &sc->pciecfg_bsh) == 0) {
		sc->pciecfg_mapped = 1;
		g_pciecfg_bst = sc->pciecfg_bst;
		g_pciecfg_bsh = sc->pciecfg_bsh;
		g_pciecfg_mapped = 1;
		/*
		 * As found: whether the GEM's vector is enabled, and which of
		 * RP1's interrupt sources are asserted.  Read only.
		 */
		device_printf(dev, "RP1 PCIE_CFG at %#jx: MSIX_CFG_%u=%#x "
		    "INTSTATL=%#x\n", (uintmax_t)pciecfg_phys, rp1_int_eth,
		    bus_space_read_4(sc->pciecfg_bst, sc->pciecfg_bsh,
		    PCIE_CFG_MSIX_CFG_0 + rp1_int_eth * 4),
		    bus_space_read_4(sc->pciecfg_bst, sc->pciecfg_bsh,
		    PCIE_CFG_INTSTATL));
	} else {
		device_printf(dev, "warning: cannot map PCIE_CFG, IACK disabled\n");
		sc->pciecfg_mapped = 0;
	}
	return (0);
}

static void
bcm2712_pcie_unmap_regs(device_t dev, struct bcm2712_pcie_softc *sc)
{

	if (sc->pciecfg_mapped) {
		g_pciecfg_mapped = 0;
		bus_space_unmap(sc->pciecfg_bst, sc->pciecfg_bsh,
		    PCIE_CFG_SIZE);
		sc->pciecfg_mapped = 0;
	}
	if (sc->cfg_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->cfg_res);
		sc->cfg_res = NULL;
	}
	if (sc->mac_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mac_res);
		sc->mac_res = NULL;
	}
	if (sc->mac_mapped) {
		bus_space_unmap(sc->mac_bst, sc->mac_bsh, GEM_MAC_SIZE);
		sc->mac_mapped = 0;
	}
}

static int
bcm2712_pcie_attach(device_t dev)
{
	struct bcm2712_pcie_softc *sc = device_get_softc(dev);
	int rid, error;
#ifndef DEV_ACPI
	u_int irq;
#endif

	sc->dev = dev;

	bcm2712_pcie_resolve_int_eth(dev);

	error = bcm2712_pcie_map_regs(dev, sc);
	if (error != 0)
		goto fail_mem;

	/* Allocate the shared interrupt (GIC SPI 229). */
	rid = 0;
#ifdef DEV_ACPI
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
#else
	/*
	 * rp1pci keeps no resource list for its children, so the interrupt is
	 * allocated by number; pci and the host bridge pass it up to nexus.
	 */
	irq = bcm2712_pcie_map_gic_spi(dev, RP1_GEM_GIC_SPI);
	if (irq == 0) {
		device_printf(dev, "cannot map GIC SPI %d\n", RP1_GEM_GIC_SPI);
		error = ENXIO;
		goto fail_mem;
	}
	sc->irq_res = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, irq, irq, 1,
	    RF_SHAREABLE | RF_ACTIVE);
#endif
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
	    (uintmax_t)sc->mac_phys, RP1_GEM_GIC_SPI);
	return (0);

fail_irq:
	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
fail_mem:
	bcm2712_pcie_unmap_regs(dev, sc);
	return (error);
}

static int
bcm2712_pcie_detach(device_t dev)
{
	struct bcm2712_pcie_softc *sc = device_get_softc(dev);

	bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	bcm2712_pcie_unmap_regs(dev, sc);
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
 * still needs the DSDT override; a kernel without it (RPI5-FDT) boots only
 * by FDT, where RP1 is a PCI device, and attaches below rp1pci.
 */
#ifdef DEV_ACPI
DRIVER_MODULE(bcm2712_pcie, acpi, bcm2712_pcie_driver, NULL, NULL);
MODULE_DEPEND(bcm2712_pcie, acpi, 1, 1, 1);
#else
DRIVER_MODULE(bcm2712_pcie, rp1pci, bcm2712_pcie_driver, NULL, NULL);
MODULE_DEPEND(bcm2712_pcie, rp1, 1, 1, 1);
#endif
MODULE_VERSION(bcm2712_pcie, 1);
MODULE_DEPEND(bcm2712_pcie, bcm2712, 1, 1, 1);	/* bcm2712_fdt.h */
