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
 * is the parent of RP1's functions.  Here the function drivers still find
 * their registers through bcm2712_fdt.h, but those that are newbus drivers
 * attach below rp1pci on the FDT lane (their identify methods pick nexus
 * or rp1pci by lane), so that they come up after BAR1 is published.
 *
 * It is also RP1's interrupt controller, as Linux drivers/mfd/rp1.c is.
 * Every RP1 peripheral interrupt is one of RP1's 61 MSI-X vectors: the DT
 * rp1 node is an interrupt-controller with #interrupt-cells = <2>, <vector
 * trigger>, and its children name it as their interrupt-parent.  This
 * driver allocates all 61 vectors (through the MIP, bcm2712_mip), gives
 * each a filter that dispatches to an interrupt source of its own, and
 * registers those sources under the rp1 node's xref, so ofw_bus_map_intr()
 * against rp1 resolves here.  Each vector also has an MSIX_CFG register in
 * RP1's PCIe endpoint block (BAR1 + 0x108000): ENABLE gates it, and for a
 * level-triggered source IACK_EN holds further messages until software
 * writes IACK -- after the handler, as Linux's rp1_chained_handle_irq()
 * does.  A handler must therefore have quieted its source by then
 * (rp1_eth's filter masks the GEM).  rpi5_modules.git doc/M2_PCIE_HOST.md,
 * phase 4b.
 *
 * And it is the parent of the RP1 functions that stock FreeBSD FDT
 * drivers can run, through a simplebus over the rp1 node (rp1_simplebus,
 * below): the DT's rp1 ranges assume BAR1 at PCIe 0, so the subclass
 * translates them to where PCI put BAR1, and this driver sub-allocates
 * BAR1 for those children.  Only whitelisted compatibles get a child
 * (rp1_ofw_compat), so no stock driver takes a function one of ours
 * drives.  First: RP1's two xHCI controllers, snps,dwc3 (phase 4c).
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
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <arm64/broadcom/bcm2712/bcm2712_var.h>

#include "ofw_bus_if.h"
#include "pic_if.h"

#define	RP1_VENDOR	0x1de4
#define	RP1_DEVICE	0x0001

/* RP1_INT_END in dt-bindings/mfd/rp1.h; RP1's MSI-X table has this many. */
#define	RP1_NIRQS	61

/*
 * RP1's PCIe endpoint block in BAR1 (Linux rp1.c RP1_PCIE_APBS_BASE), with
 * the usual RP1 atomic aliases: +0x800 sets bits, +0xc00 clears them.
 */
#define	RP1_CLOCKS		0x018000	/* clocks@18000 */
#define	RP1_PCIE_APBS		0x108000
#define	RP1_REG_SET		0x800
#define	RP1_REG_CLR		0xc00
#define	RP1_MSIX_CFG(v)		(RP1_PCIE_APBS + 0x008 + 4 * (v))
#define	 MSIX_CFG_IACK_EN	(1u << 3)
#define	 MSIX_CFG_IACK		(1u << 2)
#define	 MSIX_CFG_TEST		(1u << 1)
#define	 MSIX_CFG_ENABLE	(1u << 0)

/* The rp1 node, in the trees this board is known to publish. */
static const char * const rp1_node_paths[] = {
	"/axi/pcie@1000120000/rp1",
	"/soc/rp1",
	NULL
};

struct rp1_softc;

struct rp1_irqsrc {
	struct intr_irqsrc	isrc;
	struct rp1_softc	*sc;
	u_int			vector;
	bool			level;	/* IACK_EN set */
	struct resource		*res;	/* the MSI-X vector's interrupt */
	void			*cookie;
};

struct rp1_softc {
	device_t	dev;
	struct resource	*bar1;
	int		bar1_rid;
	struct resource	*bar0;		/* MSI-X table and PBA */
	int		bar0_rid;
	int		nirqs;		/* 0: no interrupts, children poll */
	struct rp1_irqsrc irqs[RP1_NIRQS];
	struct rman	mem_rman;	/* BAR1, in PCIe bus addresses */
	device_t	sbus;		/* rp1_simplebus, or NULL */
	struct ofw_bus_devinfo sbus_obd;
};

/*
 * The RP1 functions the rp1 simplebus gives a device: those a stock
 * FreeBSD FDT driver handles and none of ours does.
 */
static const char * const rp1_ofw_compat[] = {
	"snps,dwc3",		/* USB 3 hosts, usb@200000 and usb@300000 */
	NULL
};

static void
rp1_msix_cfg(struct rp1_softc *sc, u_int v, bus_size_t alias, uint32_t bits)
{

	bus_write_4(sc->bar1, alias + RP1_MSIX_CFG(v), bits);
}

/*
 * One MSI-X vector's filter: hand the interrupt to whoever set up the
 * matching RP1 source.  With nobody there, turn the vector off.
 */
static int
rp1_vector_filter(void *arg)
{
	struct rp1_irqsrc *ri = arg;

	if (intr_isrc_dispatch(&ri->isrc, curthread->td_intr_frame) != 0) {
		rp1_msix_cfg(ri->sc, ri->vector, RP1_REG_CLR, MSIX_CFG_ENABLE);
		device_printf(ri->sc->dev, "stray interrupt on vector %u, "
		    "disabled\n", ri->vector);
	}
	return (FILTER_HANDLED);
}

static phandle_t
rp1_find_node(void)
{
	phandle_t node;
	int i;

	for (i = 0; rp1_node_paths[i] != NULL; i++) {
		node = OF_finddevice(rp1_node_paths[i]);
		if (node != -1 &&
		    OF_hasprop(node, "interrupt-controller"))
			return (node);
	}
	return (-1);
}

/*
 * Take RP1's MSI-X vectors and become the interrupt controller for the rp1
 * node.  On failure RP1's drivers still attach, without interrupts.
 */
static void
rp1_intr_attach(struct rp1_softc *sc)
{
	device_t dev = sc->dev;
	struct rp1_irqsrc *ri;
	phandle_t node;
	int error, n, rid, v;

	node = rp1_find_node();
	if (node == -1) {
		device_printf(dev, "no rp1 interrupt-controller node; "
		    "no interrupts\n");
		return;
	}
	n = pci_msix_count(dev);
	if (n < RP1_NIRQS) {
		device_printf(dev, "MSI-X has %d vectors, not %d; no "
		    "interrupts\n", n, RP1_NIRQS);
		return;
	}

	/* pci_alloc_msix() needs the table's and PBA's BARs active. */
	sc->bar0_rid = pci_msix_table_bar(dev);
	if (sc->bar0_rid != sc->bar1_rid) {
		sc->bar0 = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &sc->bar0_rid, RF_ACTIVE);
		if (sc->bar0 == NULL) {
			device_printf(dev, "cannot allocate the MSI-X table "
			    "BAR; no interrupts\n");
			return;
		}
	}
	if (pci_msix_pba_bar(dev) != sc->bar0_rid &&
	    pci_msix_pba_bar(dev) != sc->bar1_rid) {
		device_printf(dev, "MSI-X PBA in an unexpected BAR; no "
		    "interrupts\n");
		goto fail_bar;
	}

	n = RP1_NIRQS;
	error = pci_alloc_msix(dev, &n);
	if (error != 0 || n != RP1_NIRQS) {
		device_printf(dev, "cannot allocate %d MSI-X vectors (%d, got "
		    "%d); no interrupts\n", RP1_NIRQS, error, n);
		if (error == 0)
			pci_release_msi(dev);
		goto fail_bar;
	}

	/* Every vector off until a driver sets it up. */
	for (v = 0; v < RP1_NIRQS; v++)
		rp1_msix_cfg(sc, v, RP1_REG_CLR,
		    MSIX_CFG_ENABLE | MSIX_CFG_IACK_EN | MSIX_CFG_TEST);

	/*
	 * Each vector's filter is set up now, not when a driver asks: PIC
	 * methods run under INTRNG's isrc_table_lock, where bus_setup_intr()
	 * cannot be called.
	 */
	for (v = 0; v < RP1_NIRQS; v++) {
		ri = &sc->irqs[v];
		ri->sc = sc;
		ri->vector = v;
		error = intr_isrc_register(&ri->isrc, dev, 0, "%s,%u",
		    device_get_nameunit(dev), v);
		if (error != 0) {
			device_printf(dev, "cannot register source %d: %d\n",
			    v, error);
			goto fail_vectors;
		}
		rid = v + 1;
		ri->res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
		    RF_ACTIVE);
		if (ri->res == NULL) {
			device_printf(dev, "cannot allocate vector %d\n", v);
			goto fail_vectors;
		}
		error = bus_setup_intr(dev, ri->res, INTR_TYPE_MISC |
		    INTR_MPSAFE, rp1_vector_filter, NULL, ri, &ri->cookie);
		if (error != 0) {
			device_printf(dev, "cannot set up vector %d: %d\n", v,
			    error);
			goto fail_vectors;
		}
		bus_describe_intr(dev, ri->res, ri->cookie, "v%d", v);
	}

	if (intr_pic_register(dev, OF_xref_from_node(node)) == NULL) {
		device_printf(dev, "cannot register as interrupt controller\n");
		goto fail_vectors;
	}
	sc->nirqs = RP1_NIRQS;
	device_printf(dev, "interrupt controller for %s: %d MSI-X vectors\n",
	    rp1_node_paths[0], RP1_NIRQS);
	return;

fail_vectors:
	/* No driver can have set any of them up: the PIC is not registered. */
	for (v = 0; v < RP1_NIRQS; v++) {
		ri = &sc->irqs[v];
		if (ri->cookie != NULL)
			bus_teardown_intr(dev, ri->res, ri->cookie);
		if (ri->res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ, v + 1, ri->res);
		if (ri->isrc.isrc_dev != NULL)
			intr_isrc_deregister(&ri->isrc);
		ri->cookie = NULL;
		ri->res = NULL;
	}
	pci_release_msi(dev);
fail_bar:
	if (sc->bar0 != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->bar0_rid,
		    sc->bar0);
	sc->bar0 = NULL;
}

/*
 * PIC interface: RP1's sources, named by the rp1 node's two-cell
 * specifiers <vector trigger>.
 */
static int
rp1_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct rp1_softc *sc = device_get_softc(dev);
	struct intr_map_data_fdt *daf;

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);
	daf = (struct intr_map_data_fdt *)data;
	if (daf->ncells != 2 || daf->cells[0] >= (u_int)sc->nirqs)
		return (EINVAL);
	*isrcp = &sc->irqs[daf->cells[0]].isrc;
	return (0);
}

static int
rp1_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct rp1_softc *sc = device_get_softc(dev);
	struct rp1_irqsrc *ri = (struct rp1_irqsrc *)isrc;
	struct intr_map_data_fdt *daf;
	bool level;

	if (data == NULL || data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);
	daf = (struct intr_map_data_fdt *)data;
	/* IRQ_TYPE_LEVEL_HIGH (4) or IRQ_TYPE_LEVEL_LOW (8). */
	level = (daf->cells[1] & 0xc) != 0;
	if (isrc->isrc_handlers != 0)
		return (level == ri->level ? 0 : EINVAL);

	ri->level = level;
	rp1_msix_cfg(sc, ri->vector, level ? RP1_REG_SET : RP1_REG_CLR,
	    MSIX_CFG_IACK_EN);
	return (0);
}

static int
rp1_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct rp1_softc *sc = device_get_softc(dev);
	struct rp1_irqsrc *ri = (struct rp1_irqsrc *)isrc;

	if (isrc->isrc_handlers == 0)
		rp1_msix_cfg(sc, ri->vector, RP1_REG_CLR,
		    MSIX_CFG_ENABLE | MSIX_CFG_IACK_EN);
	return (0);
}

static void
rp1_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct rp1_irqsrc *ri = (struct rp1_irqsrc *)isrc;

	rp1_msix_cfg(ri->sc, ri->vector, RP1_REG_SET, MSIX_CFG_ENABLE);
}

static void
rp1_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct rp1_irqsrc *ri = (struct rp1_irqsrc *)isrc;

	rp1_msix_cfg(ri->sc, ri->vector, RP1_REG_CLR, MSIX_CFG_ENABLE);
}

/* A level source sends no further message until acknowledged. */
static void
rp1_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct rp1_irqsrc *ri = (struct rp1_irqsrc *)isrc;

	if (ri->level)
		rp1_msix_cfg(ri->sc, ri->vector, RP1_REG_SET, MSIX_CFG_IACK);
}

static void
rp1_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	/* Nothing: an unacknowledged level source is already held. */
}

static void
rp1_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	rp1_pic_post_filter(dev, isrc);
}

static int
rp1_probe(device_t dev)
{

	if (pci_get_vendor(dev) != RP1_VENDOR ||
	    pci_get_device(dev) != RP1_DEVICE)
		return (ENXIO);
	device_set_desc(dev, "Raspberry Pi RP1 south bridge");
	return (BUS_PROBE_DEFAULT);
}

/*
 * Memory for the rp1 simplebus's children: sub-ranges of BAR1, in PCIe bus
 * addresses (the simplebus translates to those), mapped through BAR1.
 * Everything else, interrupts included, goes up as before.
 */
static struct rman *
rp1_get_rman(device_t dev, int type, u_int flags)
{
	struct rp1_softc *sc = device_get_softc(dev);

	if (type == SYS_RES_MEMORY && sc->sbus != NULL)
		return (&sc->mem_rman);
	return (NULL);
}

static struct resource *
rp1_alloc_resource(device_t dev, device_t child, int type, int rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{

	if (rp1_get_rman(dev, type, flags) != NULL)
		return (bus_generic_rman_alloc_resource(dev, child, type, rid,
		    start, end, count, flags));
	return (bus_generic_alloc_resource(dev, child, type, rid, start, end,
	    count, flags));
}

static bool
rp1_is_ours(device_t dev, struct resource *r)
{
	struct rp1_softc *sc = device_get_softc(dev);

	return (rman_get_type(r) == SYS_RES_MEMORY && sc->sbus != NULL &&
	    rman_is_region_manager(r, &sc->mem_rman));
}

static int
rp1_release_resource(device_t dev, device_t child, struct resource *r)
{

	if (rp1_is_ours(dev, r))
		return (bus_generic_rman_release_resource(dev, child, r));
	return (bus_generic_release_resource(dev, child, r));
}

static int
rp1_activate_resource(device_t dev, device_t child, struct resource *r)
{

	if (rp1_is_ours(dev, r))
		return (bus_generic_rman_activate_resource(dev, child, r));
	return (bus_generic_activate_resource(dev, child, r));
}

static int
rp1_deactivate_resource(device_t dev, device_t child, struct resource *r)
{

	if (rp1_is_ours(dev, r))
		return (bus_generic_rman_deactivate_resource(dev, child, r));
	return (bus_generic_deactivate_resource(dev, child, r));
}

static int
rp1_adjust_resource(device_t dev, device_t child, struct resource *r,
    rman_res_t start, rman_res_t end)
{

	if (rp1_is_ours(dev, r))
		return (bus_generic_rman_adjust_resource(dev, child, r, start,
		    end));
	return (bus_generic_adjust_resource(dev, child, r, start, end));
}

static int
rp1_map_resource(device_t dev, device_t child, struct resource *r,
    struct resource_map_request *argsp, struct resource_map *map)
{
	struct rp1_softc *sc = device_get_softc(dev);
	struct resource_map_request args;
	rman_res_t length, start;
	int error;

	if (!rp1_is_ours(dev, r))
		return (bus_generic_map_resource(dev, child, r, argsp, map));
	if ((rman_get_flags(r) & RF_ACTIVE) == 0)
		return (ENXIO);

	resource_init_map_request(&args);
	error = resource_validate_map_request(r, argsp, &args, &start,
	    &length);
	if (error != 0)
		return (error);
	args.offset = start - rman_get_start(sc->bar1);
	args.length = length;
	return (bus_map_resource(dev, sc->bar1, &args, map));
}

static int
rp1_unmap_resource(device_t dev, device_t child, struct resource *r,
    struct resource_map *map)
{
	struct rp1_softc *sc = device_get_softc(dev);

	if (!rp1_is_ours(dev, r))
		return (bus_generic_unmap_resource(dev, child, r, map));
	return (bus_unmap_resource(dev, sc->bar1, map));
}

/* OFW: only the rp1 simplebus has a node. */
static const struct ofw_bus_devinfo *
rp1_get_devinfo(device_t dev, device_t child)
{
	struct rp1_softc *sc = device_get_softc(dev);

	if (child != NULL && child == sc->sbus)
		return (&sc->sbus_obd);
	return (NULL);
}

/*
 * The rp1 node as a bus for stock FDT drivers.  Its BAR1 part is managed
 * here; the child is a simplebus subclass (rp1_simplebus).
 */
static void
rp1_sbus_attach(struct rp1_softc *sc)
{
	device_t dev = sc->dev;
	phandle_t node;
	int error;

	node = rp1_find_node();
	if (node == -1)
		return;

	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "RP1 peripherals (BAR1)";
	error = rman_init(&sc->mem_rman);
	if (error == 0)
		error = rman_manage_region(&sc->mem_rman,
		    rman_get_start(sc->bar1), rman_get_end(sc->bar1));
	if (error != 0) {
		device_printf(dev, "cannot manage BAR1 for the rp1 bus: %d\n",
		    error);
		return;
	}
	if (ofw_bus_gen_setup_devinfo(&sc->sbus_obd, node) != 0) {
		rman_fini(&sc->mem_rman);
		return;
	}
	sc->sbus = BUS_ADD_CHILD(dev, 0, "simplebus", DEVICE_UNIT_ANY);
	if (sc->sbus == NULL) {
		device_printf(dev, "cannot add the rp1 bus\n");
		ofw_bus_gen_destroy_devinfo(&sc->sbus_obd);
		rman_fini(&sc->mem_rman);
	}
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

	/*
	 * RP1's system PLL and clock, as found (Linux clk-rp1.c offsets).
	 * Our RP1 drivers assume what the firmware set up (sys 200 MHz);
	 * Linux reprograms it from the DT.  Logged to compare an adopted
	 * link with one brought up from reset (M2 phase 5).
	 */
	device_printf(dev, "clocks as found: PLL_SYS CS=%#x PWR=%#x "
	    "FBDIV=%u.%#x PRIM=%#x SEC=%#x; CLK_SYS CTRL=%#x DIV=%#x "
	    "SEL=%#x; CLK_ETH CTRL=%#x\n",
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x08000),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x08004),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x08008),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x0800c),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x08010),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x08014),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x00014),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x00018),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x00020),
	    bus_read_4(sc->bar1, RP1_CLOCKS + 0x00064));

	device_printf(dev, "BAR1 (peripherals): PCIe 0x%jx -> CPU 0x%jx, "
	    "%ju KB\n", (uintmax_t)rman_get_start(sc->bar1), (uintmax_t)pa,
	    (uintmax_t)size / 1024);

	/* RP1's bus masters' tags descend from this one. */
	bcm2712_rp1_publish(pa, size, bus_get_dma_tag(dev));

	/* Before the children, which set up their interrupts at attach. */
	rp1_intr_attach(sc);
	rp1_sbus_attach(sc);

	/* RP1's function drivers, now that their registers can be found. */
	bus_identify_children(dev);
	bus_attach_children(dev);
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

	/* Bus interface, for RP1's function drivers. */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_get_rman,		rp1_get_rman),
	DEVMETHOD(bus_alloc_resource,	rp1_alloc_resource),
	DEVMETHOD(bus_release_resource,	rp1_release_resource),
	DEVMETHOD(bus_activate_resource, rp1_activate_resource),
	DEVMETHOD(bus_deactivate_resource, rp1_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	rp1_adjust_resource),
	DEVMETHOD(bus_map_resource,	rp1_map_resource),
	DEVMETHOD(bus_unmap_resource,	rp1_unmap_resource),
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_get_dma_tag,	bus_generic_get_dma_tag),

	/* OFW, for the rp1 simplebus. */
	DEVMETHOD(ofw_bus_get_devinfo,	rp1_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	/* Interrupt controller interface, for the rp1 node's children. */
	DEVMETHOD(pic_map_intr,		rp1_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	rp1_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	rp1_pic_teardown_intr),
	DEVMETHOD(pic_enable_intr,	rp1_pic_enable_intr),
	DEVMETHOD(pic_disable_intr,	rp1_pic_disable_intr),
	DEVMETHOD(pic_post_filter,	rp1_pic_post_filter),
	DEVMETHOD(pic_pre_ithread,	rp1_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	rp1_pic_post_ithread),

	DEVMETHOD_END
};

/* "rp1pci0", not "rp10": a unit number run onto a name ending in a digit. */
static driver_t rp1_driver = {
	"rp1pci",
	rp1_methods,
	sizeof(struct rp1_softc),
};

DRIVER_MODULE(rp1, pci, rp1_driver, NULL, NULL);

/*
 * rp1_simplebus: simplebus over the rp1 node, with its ranges translated
 * to BAR1 and only rp1_ofw_compat children.  Named "simplebus" so that
 * stock drivers, which register on simplebus, attach below it.
 */
static int
rp1_simplebus_probe(device_t dev)
{

	if (ofw_bus_get_node(dev) == -1 ||
	    !ofw_bus_is_compatible(dev, "simple-bus"))
		return (ENXIO);
	device_set_desc(dev, "RP1 peripherals (FDT)");
	return (BUS_PROBE_SPECIFIC);
}

/*
 * rp1's ranges map its 2-cell child space to PCI space (3 cells:
 * phys.hi, then a 64-bit address).  The DT assumes BAR1 at PCIe 0; the
 * host side here is where PCI put BAR1, plus the DT's PCIe address,
 * as bcm2712_fdt_rp1_reg() computes for our own drivers.
 */
static int
rp1_simplebus_ranges(device_t dev, struct simplebus_softc *sc)
{
	struct rp1_softc *psc = device_get_softc(device_get_parent(dev));
	rman_res_t bar_start, bar_size;
	pcell_t *r;
	uint64_t pci;
	int e, i, k, len, n;

	e = sc->acells + 3 + sc->scells;
	len = OF_getencprop_alloc(sc->node, "ranges", (void **)&r);
	if (len <= 0)
		return (ENXIO);
	n = len / (int)sizeof(pcell_t) / e;
	if (n == 0) {
		OF_prop_free(r);
		return (ENXIO);
	}
	bar_start = rman_get_start(psc->bar1);
	bar_size = rman_get_size(psc->bar1);
	sc->ranges = malloc(n * sizeof(sc->ranges[0]), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	sc->nranges = n;
	for (i = 0; i < n; i++) {
		pcell_t *c = &r[i * e];

		for (k = 0; k < sc->acells; k++)
			sc->ranges[i].bus = (sc->ranges[i].bus << 32) | c[k];
		pci = ((uint64_t)c[sc->acells + 1] << 32) | c[sc->acells + 2];
		for (k = 0; k < sc->scells; k++)
			sc->ranges[i].size = (sc->ranges[i].size << 32) |
			    c[sc->acells + 3 + k];
		/* Only what BAR1 holds. */
		if (pci >= bar_size)
			sc->ranges[i].size = 0;
		else if (sc->ranges[i].size > bar_size - pci)
			sc->ranges[i].size = bar_size - pci;
		sc->ranges[i].host = bar_start + pci;
	}
	OF_prop_free(r);
	return (0);
}

static int
rp1_simplebus_attach(device_t dev)
{
	struct simplebus_softc *sc = device_get_softc(dev);
	phandle_t node;
	int i;

	simplebus_init(dev, 0);
	if (rp1_simplebus_ranges(dev, sc) != 0) {
		device_printf(dev, "cannot translate the rp1 ranges\n");
		return (ENXIO);
	}
	for (node = OF_child(sc->node); node > 0; node = OF_peer(node)) {
		if (!ofw_bus_node_status_okay(node))
			continue;
		for (i = 0; rp1_ofw_compat[i] != NULL; i++)
			if (ofw_bus_node_is_compatible(node, rp1_ofw_compat[i]))
				break;
		if (rp1_ofw_compat[i] != NULL)
			simplebus_add_device(dev, node, 0, NULL,
			    DEVICE_UNIT_ANY, NULL);
	}
	bus_attach_children(dev);
	return (0);
}

static device_method_t rp1_simplebus_methods[] = {
	DEVMETHOD(device_probe,		rp1_simplebus_probe),
	DEVMETHOD(device_attach,	rp1_simplebus_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(simplebus, rp1_simplebus_driver, rp1_simplebus_methods,
    sizeof(struct simplebus_softc), simplebus_driver);
DRIVER_MODULE(rp1_simplebus, rp1pci, rp1_simplebus_driver, NULL, NULL);
MODULE_DEPEND(rp1, pci, 1, 1, 1);
MODULE_DEPEND(rp1, bcm2712, 1, 1, 1);
MODULE_VERSION(rp1, 1);
