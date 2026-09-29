/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2020 Dr Robert Harvey Crowston <crowston@protonmail.com>
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * BCM2712 (Raspberry Pi 5) PCI-express host controller.
 *
 * A fork of sys/arm/broadcom/bcm2835/bcm2838_pci.c, the BCM2711 (Pi 4)
 * driver for the same Broadcom STB controller family, kept shaped for a
 * later merge back into it (rpi5_modules.git doc/M2_PCIE_HOST.md):
 *
 *  - Function names, order and flow follow bcm2838_pci.c, with the
 *    bcm2712_pcib_ prefix in place of bcm_pcib_.
 *  - Everything that differs by SoC is in struct bcm2712_pcib_cfg, which
 *    mirrors Linux pcie-brcmstb.c struct pcie_cfg_data.  The BCM2711
 *    values that bcm2838_pci.c hardcodes are given beside each field.
 *
 * Divergences from bcm2838_pci.c, as of phase 1:
 *
 *  BCM2712 differs:
 *   - Only controllers named in the loader tunable hw.bcm2712_pcib.adopt
 *     (DT unit addresses; default "1000120000", PCIe2) are touched at
 *     all.  The VPU firmware logs "PCI1 reset" at hand-off (and "PCI2
 *     reset" unless config.txt sets pciex4_reset=0), yet the bridge resets
 *     in brcm,brcmstb-reset all read deasserted, so no register tells us
 *     which controllers are safe to read.  Linux never reads one before
 *     resetting "rescal".  The list goes away in phase 3.
 *   - The DT "bridge" reset (via hwreset) is also checked before any
 *     controller register is read: asserted means unusable.  Necessary,
 *     not sufficient, as above.
 *   - UBUS/AXI error replies are suppressed so that failed reads return
 *     all ones (Linux brcm_pcie_post_setup_bcm2712).  Without this, config
 *     reads of empty slots return 0xdeaddead, which enumeration would take
 *     for a device.
 *   - Downstream config accesses are refused while the link is down, as
 *     Linux brcm_pcie_map_bus() does; such an access aborts the CPU.
 *   - MSI is not provided here: the DT's msi-parent is a separate
 *     brcm,bcm2712-mip controller (phase 4), not this node.
 *
 *   - Inbound (DMA) windows: one per dma-ranges entry, programmed as
 *     Linux set_inbound_win_registers() does for BCM2712 (RC_BARn size and
 *     PCIe address, UBUS_BARn remap to the CPU address), and the unused
 *     ones cleared, because we adopt the controller without a reset.
 *     FreeBSD does not translate dma-ranges, so RAM must be mapped 1:1;
 *     the freebsd-pcie2 overlay does that (rpi5_modules.git
 *     doc/DT_OVERLAYS.md).  Attach says so if it is not.
 *   - The bus DMA tag keeps DMA out of RAM that the outbound window
 *     shadows (bcm2838_pci.c limits DMA with its tag too, for another
 *     reason).
 *
 *  Phase 1 only -- adopts a link the firmware already trained:
 *   - No bridge reset, PHY/PLL set-up, PERST# or link training (phases 3
 *     and 5); if the link is not up, attach fails.
 *
 *  Duplicate, merge as-is: the config-space window, the outbound window
 *  encoders, the root port class fix-up, the bridge window relocation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/intr.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/hwreset/hwreset.h>

#include <dev/pci/pci_host_generic.h>
#include <dev/pci/pci_host_generic_fdt.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/md_var.h>

#include "pcib_if.h"

#define PCI_ID_VAL3		0x43c
#define CLASS_SHIFT		0x10
#define SUBCLASS_SHIFT		0x8

#define REG_CONTROLLER_HW_REV			0x406c
#define REG_BRIDGE_STATE			0x4068
#define BRIDGE_STATE_PHYLINKUP		0x10
#define BRIDGE_STATE_DL_ACTIVE		0x20
#define REG_BRIDGE_LINK_STATE			0x00bc
#define REG_BUS_WINDOW_LOW			0x400c
#define REG_BUS_WINDOW_HIGH			0x4010
#define REG_CPU_WINDOW_LOW			0x4070
#define REG_CPU_WINDOW_START_HIGH		0x4080
#define REG_CPU_WINDOW_END_HIGH			0x4084

/* BCM2712 (7712) only; Linux PCIE_MISC_UBUS_CTRL, _AXI_READ_ERROR_DATA. */
#define REG_UBUS_CTRL				0x40a4
#define UBUS_CTRL_REPLY_ERR_DIS		(1u << 13)
#define UBUS_CTRL_REPLY_DECERR_DIS	(1u << 19)
#define REG_AXI_READ_ERROR_DATA			0x4170

/*
 * Inbound windows, BCM2712 (7712) layout: Linux PCIE_MISC_RC_BAR{1,4}_*,
 * PCIE_MISC_UBUS_BAR{1,4}_CONFIG_REMAP, brcm_bar_reg_offset() and
 * brcm_ubus_reg_offset().
 */
#define REG_RC_BAR1_CONFIG_LO			0x402c
#define REG_RC_BAR4_CONFIG_LO			0x40d4
#define RC_BAR_CONFIG_LO_SIZE_MASK		0x1f
#define REG_UBUS_BAR1_CONFIG_REMAP		0x40ac
#define REG_UBUS_BAR4_CONFIG_REMAP		0x410c
#define UBUS_BAR_CONFIG_REMAP_ACCESS_EN		0x1
#define MAX_INBOUND_WINS			10

struct bcm2712_pcib_inbound_win {
	uint64_t	pci_base;
	uint64_t	cpu_base;
	uint64_t	size;
};

/*
 * Per-SoC differences, after Linux pcie-brcmstb.c struct pcie_cfg_data.
 * The BCM2711 value is what bcm2838_pci.c hardcodes.
 */
struct bcm2712_pcib_cfg {
	const char	*desc;
	bus_size_t	ext_cfg_index;	/* 2711: 0x9000 (REG_EP_CONFIG_CHOICE) */
	bus_size_t	ext_cfg_data;	/* 2711: 0x8000 (REG_EP_CONFIG_DATA) */
	bus_size_t	hard_debug;	/* 2711: 0x4204; 2712: 0x4304 */
	bus_size_t	intr2_cpu_base;	/* 2711: 0x4300; 2712: 0x4400 */
	u_int		num_inbound_wins; /* 2711: 3; 2712: 10 (UBUS BARs) */
	bool		ubus_err_suppress; /* 2711: no; 2712: yes */
};

static const struct bcm2712_pcib_cfg bcm2712_cfg = {
	.desc			= "BCM2712 PCI-express controller",
	.ext_cfg_index		= 0x9000,
	.ext_cfg_data		= 0x8000,
	.hard_debug		= 0x4304,
	.intr2_cpu_base		= 0x4400,
	.num_inbound_wins	= 10,
	.ubus_err_suppress	= true,
};

/*
 * Controllers phase 1 may adopt, by DT unit address, separated by spaces or
 * commas.  See the comment at the top.
 */
static char bcm2712_pcib_adopt[128] = "1000120000";
static SYSCTL_NODE(_hw, OID_AUTO, bcm2712_pcib, CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, "BCM2712 PCIe host controller");
SYSCTL_STRING(_hw_bcm2712_pcib, OID_AUTO, adopt, CTLFLAG_RDTUN,
    bcm2712_pcib_adopt, sizeof(bcm2712_pcib_adopt),
    "DT unit addresses of the controllers phase 1 may adopt");

struct bcm2712_pcib_softc {
	struct generic_pcie_fdt_softc	base;
	device_t			dev;
	const struct bcm2712_pcib_cfg	*cfg;
	hwreset_t			bridge_rst;
	struct mtx			config_mtx;
	bus_dma_tag_t			dmat;
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2712-pcie",			(uintptr_t)&bcm2712_cfg},
	{NULL,					0}
};

static int
bcm2712_pcib_probe(device_t dev)
{
	const struct bcm2712_pcib_cfg *cfg;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	cfg = (const struct bcm2712_pcib_cfg *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (cfg == NULL)
		return (ENXIO);

	device_set_desc(dev, cfg->desc);
	return (BUS_PROBE_DEFAULT);
}

static void
bcm2712_pcib_set_reg(struct bcm2712_pcib_softc *sc, uint32_t reg, uint32_t val)
{

	bus_write_4(sc->base.base.res, reg, htole32(val));
}

static uint32_t
bcm2712_pcib_read_reg(struct bcm2712_pcib_softc *sc, uint32_t reg)
{

	return (le32toh(bus_read_4(sc->base.base.res, reg)));
}

static bool
bcm2712_pcib_link_up(struct bcm2712_pcib_softc *sc)
{
	uint32_t val;

	val = bcm2712_pcib_read_reg(sc, REG_BRIDGE_STATE);
	return ((val & (BRIDGE_STATE_PHYLINKUP | BRIDGE_STATE_DL_ACTIVE)) ==
	    (BRIDGE_STATE_PHYLINKUP | BRIDGE_STATE_DL_ACTIVE));
}

static int
bcm2712_pcib_check_ranges(device_t dev)
{
	struct bcm2712_pcib_softc *sc;
	struct pcie_range *ranges;
	int error = 0, i;

	sc = device_get_softc(dev);
	ranges = &sc->base.base.ranges[0];

	/* The first range needs to be non-zero. */
	if (ranges[0].size == 0) {
		device_printf(dev, "error: first outbound memory range "
		    "(pci addr: 0x%jx, cpu addr: 0x%jx) has zero size.\n",
		    ranges[0].pci_base, ranges[0].phys_base);
		error = ENXIO;
	}

	/*
	 * The controller can handle several distinct ranges, but, as in
	 * bcm2838_pci.c, only the first is implemented.
	 *
	 * BCM2712 differs: said always, not only under bootverbose.  The
	 * generic FDT host has already put every range in its resource
	 * manager, so PCI may place BARs in one this driver never programs,
	 * and a second region there breaks growing a bridge window
	 * (INVARIANTS "next resource mismatch").  The freebsd-pcie2 overlay
	 * leaves one range (rpi5_modules.git doc/DT_OVERLAYS.md).
	 */
	for (i = 1; i < MAX_RANGES_TUPLES; ++i) {
		if (ranges[i].size > 0)
			device_printf(dev,
			    "WARNING: outbound memory range %d (pci addr: 0x%jx, "
			    "cpu addr: 0x%jx, size: 0x%jx) is not programmed, "
			    "but PCI may still place BARs in it.\n",
			    i, ranges[i].pci_base, ranges[i].phys_base,
			    ranges[i].size);
	}

	return (error);
}

static const char *
bcm2712_pcib_link_state_string(uint32_t mode)
{

	switch(mode & PCIEM_LINK_STA_SPEED) {
	case 0:
		return ("not up");
	case 1:
		return ("2.5 GT/s");
	case 2:
		return ("5.0 GT/s");
	case 4:
		return ("8.0 GT/s");
	default:
		return ("unknown");
	}
}

static bus_addr_t
bcm2712_get_offset_and_prepare_config(struct bcm2712_pcib_softc *sc,
    u_int bus, u_int slot, u_int func, u_int reg)
{
	/*
	 * Config for an end point is only available through a narrow window for
	 * one end point at a time. We first tell the controller which end point
	 * we want, then access it through the window.
	 */
	uint32_t func_index;

	if (bus == 0 && slot == 0 && func == 0)
		/*
		 * Special case for root device; its config is always available
		 * through the zero-offset.
		 */
		return (reg);

	/* Tell the controller to show us the config in question. */
	func_index = PCIE_ADDR_OFFSET(bus, slot, func, 0);
	bcm2712_pcib_set_reg(sc, sc->cfg->ext_cfg_index, func_index);

	return (sc->cfg->ext_cfg_data + reg);
}

static bool
bcm2712_pcib_is_valid_quad(struct bcm2712_pcib_softc *sc, u_int bus,
    u_int slot, u_int func, u_int reg)
{

	if ((bus < sc->base.base.bus_start) || (bus > sc->base.base.bus_end))
		return (false);
	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) || (reg > PCIE_REGMAX))
		return (false);

	if (bus == 0 && slot == 0 && func == 0)
		return (true);
	if (bus == 0)
		/*
		 * Probing other slots and funcs on bus 0 will lock up the
		 * memory controller.
		 */
		return (false);

	/* An access below the root port with the link down aborts the CPU. */
	if (!bcm2712_pcib_link_up(sc))
		return (false);

	return (true);
}

static uint32_t
bcm2712_pcib_read_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes)
{
	struct bcm2712_pcib_softc *sc;
	bus_addr_t offset;
	uint32_t data;

	sc = device_get_softc(dev);
	if (!bcm2712_pcib_is_valid_quad(sc, bus, slot, func, reg))
		return (~0U);

	mtx_lock(&sc->config_mtx);
	offset = bcm2712_get_offset_and_prepare_config(sc, bus, slot, func, reg);

	switch (bytes) {
	case 1:
		data = bus_read_1(sc->base.base.res, offset);
		break;
	case 2:
		data = le16toh(bus_read_2(sc->base.base.res, offset));
		break;
	case 4:
		data = le32toh(bus_read_4(sc->base.base.res, offset));
		break;
	default:
		data = ~0U;
		break;
	}

	mtx_unlock(&sc->config_mtx);
	return (data);
}

static void
bcm2712_pcib_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct bcm2712_pcib_softc *sc;
	uint32_t offset;

	sc = device_get_softc(dev);
	if (!bcm2712_pcib_is_valid_quad(sc, bus, slot, func, reg))
		return;

	mtx_lock(&sc->config_mtx);
	offset = bcm2712_get_offset_and_prepare_config(sc, bus, slot, func, reg);

	switch (bytes) {
	case 1:
		bus_write_1(sc->base.base.res, offset, val);
		break;
	case 2:
		bus_write_2(sc->base.base.res, offset, htole16(val));
		break;
	case 4:
		bus_write_4(sc->base.base.res, offset, htole32(val));
		break;
	default:
		break;
	}

	mtx_unlock(&sc->config_mtx);
}

static void
bcm2712_pcib_relocate_bridge_window(device_t dev)
{
	/*
	 * As in bcm2838_pci.c: move the root port's memory window to the
	 * start of the outbound range, where pcib_probe_windows() will find it,
	 * rather than leave one that allocation would reject.
	 */

	struct bcm2712_pcib_softc *sc;
	pci_addr_t base, limit, size, new_base, new_limit, range_end;
	uint16_t val;

	sc = device_get_softc(dev);

	/* uint32_t: PCI_PPBMEM*() shift val left 16, which overflows int. */
	val = bcm2712_pcib_read_config(dev, 0, 0, 0, PCIR_MEMBASE_1, 2);
	base = PCI_PPBMEMBASE(0, (uint32_t)val);

	val = bcm2712_pcib_read_config(dev, 0, 0, 0, PCIR_MEMLIMIT_1, 2);
	limit = PCI_PPBMEMLIMIT(0, (uint32_t)val);
	size = limit - base;

	new_base = sc->base.base.ranges[0].pci_base;
	val = (uint16_t) (new_base >> 16);
	bcm2712_pcib_write_config(dev, 0, 0, 0, PCIR_MEMBASE_1, val, 2);

	/*
	 * BCM2712 differs: keep the moved window inside the range.  An
	 * adopted link comes with the firmware's window, which need not fit
	 * the DT's range at its new place.
	 */
	range_end = new_base + sc->base.base.ranges[0].size - 1;
	new_limit = new_base + size;
	if (limit < base || new_limit > range_end)
		new_limit = range_end;
	val = (uint16_t) (new_limit >> 16);
	bcm2712_pcib_write_config(dev, 0, 0, 0, PCIR_MEMLIMIT_1, val, 2);

	device_printf(dev, "root port memory window 0x%jx-0x%jx as found, "
	    "now 0x%jx-0x%jx\n", (uintmax_t)base, (uintmax_t)limit,
	    (uintmax_t)new_base, (uintmax_t)(new_limit | 0xfffff));
}

static uint32_t
encode_cpu_window_low(pci_addr_t phys_base, bus_size_t size)
{

	return (((phys_base >> 0x10) & 0xfff0) |
	    ((phys_base + size - 1) & 0xfff00000));
}

static uint32_t
encode_cpu_window_start_high(pci_addr_t phys_base)
{

	return ((phys_base >> 0x20) & 0xff);
}

static uint32_t
encode_cpu_window_end_high(pci_addr_t phys_base, bus_size_t size)
{

	return (((phys_base + size - 1) >> 0x20) & 0xff);
}

static bus_size_t
bcm2712_pcib_rc_bar_reg(u_int bar)
{

	return (bar <= 3 ? REG_RC_BAR1_CONFIG_LO + 8 * (bar - 1) :
	    REG_RC_BAR4_CONFIG_LO + 8 * (bar - 4));
}

static bus_size_t
bcm2712_pcib_ubus_bar_reg(u_int bar)
{

	return (bar <= 3 ? REG_UBUS_BAR1_CONFIG_REMAP + 8 * (bar - 1) :
	    REG_UBUS_BAR4_CONFIG_REMAP + 8 * (bar - 4));
}

/*
 * RC_BARn size field; Linux brcm_pcie_encode_ibar_size().  0 disables the
 * window, and is also returned for a size the field cannot express.
 */
static uint32_t
bcm2712_pcib_encode_ibar_size(uint64_t size)
{
	int log2_in;

	if (size == 0 || !powerof2(size))
		return (0);
	log2_in = flsll(size) - 1;
	if (log2_in >= 12 && log2_in <= 15)
		return ((log2_in - 12) + 0x1c);
	if (log2_in >= 16 && log2_in <= 36)
		return (log2_in - 15);
	return (0);
}

/*
 * The DT's dma-ranges, as inbound windows.  Returns the number of windows,
 * or -1 if the property cannot be read as <3 PCI cells, 1-2 parent
 * address cells, 1-2 size cells>.
 */
static int
bcm2712_pcib_get_inbound_wins(device_t dev,
    struct bcm2712_pcib_inbound_win *wins, int max)
{
	phandle_t node;
	pcell_t acells, pacells, scells, *cells, *c;
	ssize_t len;
	int i, k, n, tuple;

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "#address-cells", &acells,
	    sizeof(acells)) <= 0)
		acells = 3;
	if (OF_getencprop(node, "#size-cells", &scells, sizeof(scells)) <= 0)
		scells = 2;
	if (OF_getencprop(OF_parent(node), "#address-cells", &pacells,
	    sizeof(pacells)) <= 0)
		pacells = 2;
	if (acells != 3 || pacells < 1 || pacells > 2 || scells < 1 ||
	    scells > 2)
		return (-1);

	len = OF_getencprop_alloc_multi(node, "dma-ranges", sizeof(*cells),
	    (void **)&cells);
	if (len <= 0)
		return (0);
	tuple = acells + pacells + scells;
	n = 0;
	for (i = 0; i + tuple <= len && n < max; i += tuple) {
		c = &cells[i];
		wins[n].pci_base = ((uint64_t)c[1] << 32) | c[2];
		wins[n].cpu_base = 0;
		for (k = 0; k < pacells; k++)
			wins[n].cpu_base = (wins[n].cpu_base << 32) | c[3 + k];
		wins[n].size = 0;
		for (k = 0; k < scells; k++)
			wins[n].size = (wins[n].size << 32) |
			    c[3 + pacells + k];
		n++;
	}
	OF_prop_free(cells);
	return (n);
}

/*
 * Report the inbound windows as the firmware left them; the enabled ones
 * only.  RC_BARn holds the PCIe address and the size code, UBUS_BARn the
 * CPU address it maps to.
 */
static void
bcm2712_pcib_log_inbound_wins(struct bcm2712_pcib_softc *sc)
{
	uint32_t lo, hi, ulo, uhi;
	u_int bar;

	for (bar = 1; bar <= sc->cfg->num_inbound_wins; bar++) {
		lo = bcm2712_pcib_read_reg(sc, bcm2712_pcib_rc_bar_reg(bar));
		hi = bcm2712_pcib_read_reg(sc, bcm2712_pcib_rc_bar_reg(bar) + 4);
		ulo = bcm2712_pcib_read_reg(sc, bcm2712_pcib_ubus_bar_reg(bar));
		uhi = bcm2712_pcib_read_reg(sc,
		    bcm2712_pcib_ubus_bar_reg(bar) + 4);
		if ((lo & RC_BAR_CONFIG_LO_SIZE_MASK) == 0 &&
		    (ulo & UBUS_BAR_CONFIG_REMAP_ACCESS_EN) == 0)
			continue;
		device_printf(sc->dev, "inbound window %u as found: "
		    "PCIe 0x%jx size code 0x%x -> CPU 0x%jx%s\n", bar,
		    (uintmax_t)(((uint64_t)hi << 32) |
		    (lo & ~RC_BAR_CONFIG_LO_SIZE_MASK)),
		    lo & RC_BAR_CONFIG_LO_SIZE_MASK,
		    (uintmax_t)(((uint64_t)uhi << 32) | (ulo & ~0xfffu)),
		    (ulo & UBUS_BAR_CONFIG_REMAP_ACCESS_EN) ? "" :
		    " (remap disabled)");
	}
}

/* Linux set_inbound_win_registers() for BCM2712, clearing the rest. */
static void
bcm2712_pcib_set_inbound_wins(struct bcm2712_pcib_softc *sc,
    const struct bcm2712_pcib_inbound_win *wins, int n)
{
	const struct bcm2712_pcib_inbound_win *w;
	bus_size_t reg, ureg;
	uint32_t lo;
	u_int bar;

	for (bar = 1; bar <= sc->cfg->num_inbound_wins; bar++) {
		reg = bcm2712_pcib_rc_bar_reg(bar);
		ureg = bcm2712_pcib_ubus_bar_reg(bar);
		if (bar > (u_int)n) {
			bcm2712_pcib_set_reg(sc, reg, 0);
			bcm2712_pcib_set_reg(sc, reg + 4, 0);
			bcm2712_pcib_set_reg(sc, ureg, 0);
			bcm2712_pcib_set_reg(sc, ureg + 4, 0);
			continue;
		}
		w = &wins[bar - 1];
		lo = ((uint32_t)w->pci_base & ~RC_BAR_CONFIG_LO_SIZE_MASK) |
		    bcm2712_pcib_encode_ibar_size(w->size);
		bcm2712_pcib_set_reg(sc, reg, lo);
		bcm2712_pcib_set_reg(sc, reg + 4, w->pci_base >> 32);
		bcm2712_pcib_set_reg(sc, ureg,
		    ((uint32_t)w->cpu_base & ~0xfffu) |
		    UBUS_BAR_CONFIG_REMAP_ACCESS_EN);
		bcm2712_pcib_set_reg(sc, ureg + 4, w->cpu_base >> 32);
		device_printf(sc->dev, "inbound window %u: PCIe 0x%jx -> "
		    "CPU 0x%jx, size 0x%jx\n", bar, (uintmax_t)w->pci_base,
		    (uintmax_t)w->cpu_base, (uintmax_t)w->size);
	}
}

/*
 * Program the inbound windows from dma-ranges and build the DMA tag.
 *
 * busdma gives devices CPU physical addresses, so all of RAM must be in a
 * window that maps PCIe X to CPU X.  The outbound window's PCIe range then
 * shadows the RAM at the same addresses -- a device's DMA to them would be
 * taken for a transfer to a device behind the bridge -- so the tag keeps
 * DMA out of it, the way a DMA limit would, with bounce buffers.
 */
static int
bcm2712_pcib_setup_inbound(struct bcm2712_pcib_softc *sc)
{
	struct bcm2712_pcib_inbound_win wins[MAX_INBOUND_WINS];
	uint64_t ram_end, ob_lo, ob_hi;
	bus_addr_t lowaddr, highaddr;
	bool ram_1to1;
	int error, i, n;

	bcm2712_pcib_log_inbound_wins(sc);

	n = bcm2712_pcib_get_inbound_wins(sc->dev, wins,
	    MIN(sc->cfg->num_inbound_wins, MAX_INBOUND_WINS));
	if (n <= 0) {
		device_printf(sc->dev, "no usable dma-ranges (%d); inbound "
		    "windows left as found\n", n);
		return (0);
	}
	for (i = 0; i < n; i++) {
		if (bcm2712_pcib_encode_ibar_size(wins[i].size) == 0 ||
		    (wins[i].pci_base & (wins[i].size - 1)) != 0) {
			device_printf(sc->dev, "dma-ranges entry %d (PCIe "
			    "0x%jx, size 0x%jx) cannot be an inbound window; "
			    "inbound windows left as found\n", i,
			    (uintmax_t)wins[i].pci_base,
			    (uintmax_t)wins[i].size);
			return (0);
		}
	}

	ram_end = ptoa((uint64_t)Maxmem);
	ram_1to1 = false;
	for (i = 0; i < n; i++)
		if (wins[i].pci_base == 0 && wins[i].cpu_base == 0 &&
		    wins[i].size >= ram_end)
			ram_1to1 = true;
	if (!ram_1to1)
		device_printf(sc->dev, "WARNING: dma-ranges do not map RAM "
		    "(0-0x%jx) 1:1, and FreeBSD does not translate them: DMA "
		    "by devices behind this bridge will not reach RAM.  "
		    "Expected the freebsd-pcie2 device-tree overlay "
		    "(rpi5_modules.git doc/DT_OVERLAYS.md).\n",
		    (uintmax_t)ram_end);

	bcm2712_pcib_set_inbound_wins(sc, wins, n);

	/* Keep DMA out of the RAM the outbound window shadows. */
	lowaddr = BUS_SPACE_MAXADDR;
	highaddr = BUS_SPACE_MAXADDR;
	ob_lo = sc->base.base.ranges[0].pci_base;
	ob_hi = ob_lo + sc->base.base.ranges[0].size - 1;
	if (ram_1to1 && ob_lo < ram_end) {
		lowaddr = ob_lo - 1;
		highaddr = ob_hi;
		device_printf(sc->dev, "DMA excluded from 0x%jx-0x%jx, "
		    "shadowed by the outbound window\n", (uintmax_t)ob_lo,
		    (uintmax_t)ob_hi);
	}
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), /* parent */
	    1, 0,				/* alignment, bounds */
	    lowaddr,				/* lowaddr */
	    highaddr,				/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    BUS_SPACE_MAXSIZE,			/* maxsize */
	    BUS_SPACE_UNRESTRICTED,		/* nsegments */
	    BUS_SPACE_MAXSIZE,			/* maxsegsize */
	    sc->base.base.coherent ? BUS_DMA_COHERENT : 0, /* flags */
	    NULL, NULL,				/* lockfunc, lockarg */
	    &sc->dmat);
	if (error != 0)
		device_printf(sc->dev, "cannot create the DMA tag (%d)\n",
		    error);
	return (error);
}

static bus_dma_tag_t
bcm2712_pcib_get_dma_tag(device_t dev, device_t child)
{
	struct bcm2712_pcib_softc *sc;

	sc = device_get_softc(dev);
	return (sc->dmat != NULL ? sc->dmat : sc->base.base.dmat);
}

/* Is this controller's DT unit address in hw.bcm2712_pcib.adopt? */
static bool
bcm2712_pcib_adopt_listed(device_t dev)
{
	const char *name, *ua, *p;
	size_t len, n;

	name = ofw_bus_get_name(dev);
	if (name == NULL || (ua = strchr(name, '@')) == NULL)
		return (false);
	ua++;
	len = strlen(ua);
	for (p = bcm2712_pcib_adopt; *p != '\0'; p += n) {
		if (*p == ' ' || *p == ',') {
			n = 1;
			continue;
		}
		for (n = 0; p[n] != '\0' && p[n] != ' ' && p[n] != ','; n++)
			;
		if (n == len && strncmp(p, ua, len) == 0)
			return (true);
	}
	return (false);
}

/*
 * Check the DT "bridge" reset before any controller register is read.  A
 * controller held in reset raises an SError on access, and bringing one out
 * of reset is phases 3 and 5 of M2_PCIE_HOST.md.
 */
static int
bcm2712_pcib_check_reset(device_t dev)
{
	struct bcm2712_pcib_softc *sc;
	bool asserted;
	int error;

	sc = device_get_softc(dev);
	error = hwreset_get_by_ofw_name(dev, 0, "bridge", &sc->bridge_rst);
	if (error != 0) {
		device_printf(dev, "no \"bridge\" reset in the DT (%d); "
		    "not touching the controller\n", error);
		return (ENXIO);
	}
	error = hwreset_is_asserted(sc->bridge_rst, &asserted);
	if (error != 0 || asserted) {
		if (error != 0)
			device_printf(dev, "cannot read the bridge reset (%d)\n",
			    error);
		else
			device_printf(dev, "held in reset by the firmware; "
			    "bringing it out of reset is not implemented yet\n");
		hwreset_release(sc->bridge_rst);
		return (ENXIO);
	}
	return (0);
}

static int
bcm2712_pcib_attach(device_t dev)
{
	struct bcm2712_pcib_softc *sc;
	pci_addr_t phys_base, pci_base;
	bus_size_t size;
	uint32_t hardware_rev, link_state, tmp;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->cfg = (const struct bcm2712_pcib_cfg *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	if (!bcm2712_pcib_adopt_listed(dev)) {
		device_printf(dev, "not in hw.bcm2712_pcib.adopt (\"%s\"); "
		    "left untouched\n", bcm2712_pcib_adopt);
		return (ENXIO);
	}

	error = bcm2712_pcib_check_reset(dev);
	if (error != 0)
		return (error);

	error = pci_host_generic_setup_fdt(dev);
	if (error != 0)
		return (error);

	error = bcm2712_pcib_check_ranges(dev);
	if (error != 0)
		goto failed;

	hardware_rev = bcm2712_pcib_read_reg(sc, REG_CONTROLLER_HW_REV) & 0xffff;
	device_printf(dev, "hardware identifies as revision 0x%x.\n",
	    hardware_rev);

	/*
	 * Phase 1 adopts the link the firmware trained (pciex4_reset=0 for
	 * PCIe2); it does not reset, set up the PHY or train one itself.
	 */
	if (!bcm2712_pcib_link_up(sc)) {
		device_printf(dev, "error: link is not up (status 0x%08x); "
		    "link training is not implemented yet.\n",
		    bcm2712_pcib_read_reg(sc, REG_BRIDGE_STATE));
		error = ENXIO;
		goto failed;
	}

	mtx_init(&sc->config_mtx, "bcm2712_pcib: config_mtx", NULL, MTX_DEF);

	link_state = bcm2712_pcib_read_reg(sc, REG_BRIDGE_LINK_STATE) >> 0x10;
	device_printf(dev, "link up at %s (adopted from the firmware).\n",
	    bcm2712_pcib_link_state_string(link_state));

	/* Failed reads return all ones, not 0xdeaddead, and do not abort. */
	if (sc->cfg->ubus_err_suppress) {
		tmp = bcm2712_pcib_read_reg(sc, REG_UBUS_CTRL);
		tmp |= UBUS_CTRL_REPLY_ERR_DIS | UBUS_CTRL_REPLY_DECERR_DIS;
		bcm2712_pcib_set_reg(sc, REG_UBUS_CTRL, tmp);
		bcm2712_pcib_set_reg(sc, REG_AXI_READ_ERROR_DATA, 0xffffffff);
	}

	/*
	 * Set the CPU->PCI memory window. The map in this direction is not 1:1.
	 * Addresses seen by the CPU need to be adjusted to make sense to the
	 * controller as they pass through the window.
	 */
	pci_base  = sc->base.base.ranges[0].pci_base;
	phys_base = sc->base.base.ranges[0].phys_base;
	size      = sc->base.base.ranges[0].size;

	bcm2712_pcib_set_reg(sc, REG_BUS_WINDOW_LOW, pci_base & 0xffffffff);
	bcm2712_pcib_set_reg(sc, REG_BUS_WINDOW_HIGH, pci_base >> 32);

	bcm2712_pcib_set_reg(sc, REG_CPU_WINDOW_LOW,
	    encode_cpu_window_low(phys_base, size));
	bcm2712_pcib_set_reg(sc, REG_CPU_WINDOW_START_HIGH,
	    encode_cpu_window_start_high(phys_base));
	bcm2712_pcib_set_reg(sc, REG_CPU_WINDOW_END_HIGH,
	    encode_cpu_window_end_high(phys_base, size));

	/*
	 * The controller starts up declaring itself an endpoint; readvertise it
	 * as a bridge.
	 */
	bcm2712_pcib_set_reg(sc, PCI_ID_VAL3,
	    PCIC_BRIDGE << CLASS_SHIFT | PCIS_BRIDGE_PCI << SUBCLASS_SHIFT);

	/* The PCI->CPU (DMA) direction, and the tag that goes with it. */
	error = bcm2712_pcib_setup_inbound(sc);
	if (error != 0) {
		mtx_destroy(&sc->config_mtx);
		goto failed;
	}

	bcm2712_pcib_relocate_bridge_window(dev);

	/* Done. */
	device_add_child(dev, "pci", DEVICE_UNIT_ANY);
	bus_attach_children(dev);
	return (0);
failed:
	pci_host_generic_destroy_fdt(dev);
	hwreset_release(sc->bridge_rst);
	return (error);
}

/*
 * Device method table.
 */
static device_method_t bcm2712_pcib_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,			bcm2712_pcib_probe),
	DEVMETHOD(device_attach,		bcm2712_pcib_attach),

	/* Bus interface. */
	DEVMETHOD(bus_get_dma_tag,		bcm2712_pcib_get_dma_tag),

	/* PCIB interface. */
	DEVMETHOD(pcib_read_config,		bcm2712_pcib_read_config),
	DEVMETHOD(pcib_write_config,		bcm2712_pcib_write_config),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, bcm2712_pcib_driver, bcm2712_pcib_methods,
    sizeof(struct bcm2712_pcib_softc), generic_pcie_fdt_driver);

DRIVER_MODULE(bcm2712_pcib, simplebus, bcm2712_pcib_driver, 0, 0);
