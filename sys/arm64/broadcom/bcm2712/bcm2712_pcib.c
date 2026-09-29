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
 * Divergences from bcm2838_pci.c, as of phase 3:
 *
 *  BCM2712 differs:
 *   - Each controller is either adopted with the link the firmware trained
 *     (phase 1; loader tunable hw.bcm2712_pcib.adopt, DT unit addresses,
 *     default "1000120000", PCIe2), or brought up from reset as Linux does
 *     (phase 3; hw.bcm2712_pcib.reset, default "1000110000", PCIe1), or
 *     left untouched.  The VPU firmware logs "PCI1 reset" at hand-off (and
 *     "PCI2 reset" unless config.txt sets pciex4_reset=0), yet the bridge
 *     resets in brcm,brcmstb-reset all read deasserted, so no register
 *     tells us which controllers are safe to read without a reset.
 *   - Adopting, the DT "bridge" reset (via hwreset) is checked before any
 *     controller register is read: asserted means unusable.  Necessary,
 *     not sufficient, as above.
 *   - From reset: the bridge reset, the 54 MHz refclk PLL set-up over MDIO,
 *     PERST# and link training follow Linux pcie-brcmstb.c for
 *     bcm2712_cfg (see "Phase 3" below).  The shared "rescal" calibration
 *     is run only if hw.bcm2712_pcib.rescal is set, because the adopted
 *     PCIe2 shares it.  PERST# is not released unless RAM is mapped 1:1
 *     for DMA.
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
 *   - The 32-bit outbound window shadows the RAM at the same addresses,
 *     because RAM is mapped 1:1.  The freebsd-pcieN overlays reserve that
 *     RAM (/reserved-memory, no-map), as EDK2 does on the ACPI lane, and
 *     then the DMA tag has nothing to exclude.  Without the reservation
 *     the tag keeps DMA out of the shadowed RAM, by bouncing
 *     (bcm2838_pci.c limits DMA with its tag too, for another reason).
 *
 *  Adopted controllers only -- a link the firmware already trained:
 *   - No bridge reset, PHY/PLL set-up, PERST# or link training (phase 5
 *     does that for PCIe2); if the link is not up, attach fails.
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

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_dumpset.h>

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

/*
 * Bring-up from reset (phase 3), BCM2712 (7712) layout.  Linux names:
 * PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, PCIE_RC_CFG_PRIV1_*,
 * PCIE_RC_DL_MDIO_*, PCIE_RC_PL_PHY_CTL_15, PCIE_MISC_*, and the
 * PCIE_MISC_HARD_PCIE_HARD_DEBUG_* bits of cfg->hard_debug.
 */
#define REG_VENDOR_SPECIFIC_REG1		0x0188
#define VENDOR_REG1_ENDIAN_MODE_BAR2_MASK	0xc	/* 0: little endian */
#define REG_PCIE_CAP				0x00ac	/* the RC's PCIe capability */
#define LINK_CTL2_TARGET_SPEED_MASK		0xf
#define REG_LINK_CAPABILITY			0x04dc
#define LINK_CAPABILITY_MAX_LINK_SPEED_MASK	0xf
#define LINK_CAPABILITY_ASPM_SUPPORT_SHIFT	10
#define LINK_CAPABILITY_ASPM_SUPPORT_MASK	0xc00
#define ASPM_SUPPORT_L0S			0x1
#define ASPM_SUPPORT_L1				0x2
#define REG_ROOT_CAP				0x04f8
#define ROOT_CAP_L1SS_MODE_SHIFT		3
#define ROOT_CAP_L1SS_MODE_MASK			0xf8
#define REG_MDIO_ADDR				0x1100
#define REG_MDIO_WR_DATA			0x1104
#define REG_MDIO_RD_DATA			0x1108
#define MDIO_PORT0				0x0
#define MDIO_PORT_SHIFT				16	/* MDIO_PORT_MASK 0xf0000 */
#define MDIO_CMD_WRITE				(0u << 20) /* MDIO_CMD_MASK */
#define MDIO_DATA_DONE				(1u << 31)
#define MDIO_SET_ADDR_OFFSET			0x1f
#define REG_PL_PHY_CTL_15			0x184c
#define PL_PHY_CTL_15_PM_CLK_PERIOD_MASK	0xff
#define REG_MISC_CTRL				0x4008
#define MISC_CTRL_RCB_64B_MODE			0x80
#define MISC_CTRL_RCB_MPS_MODE			0x400
#define MISC_CTRL_SCB_ACCESS_EN			0x1000
#define MISC_CTRL_CFG_READ_UR_MODE		0x2000
#define MISC_CTRL_MAX_BURST_SIZE_SHIFT		20
#define MISC_CTRL_MAX_BURST_SIZE_MASK		0x300000
#define MISC_CTRL_SCB0_SIZE_SHIFT		27
#define MISC_CTRL_SCB0_SIZE_MASK		0xf8000000
#define MAX_BURST_SIZE_512			0x2	/* not 2711, 7278 or BMIPS */
#define REG_RC_CONFIG_RETRY_TIMEOUT		0x405c
#define REG_PCIE_CTRL				0x4064
#define PCIE_CTRL_PERSTB			0x4	/* 7278 way: 0 asserts PERST# */
#define BRIDGE_STATE_PORT			0x80	/* PCIE_STATUS: RC, not EP */
#define REG_MISC_CTRL_1				0x40a0
#define MISC_CTRL_1_EN_VDM_QOS_CONTROL		(1u << 5)
#define REG_UBUS_TIMEOUT			0x40a8
#define REG_TC_QUEUE_TO_QOS_MAP(x)		(0x4160 - (x) * 4)
#define REG_AXI_INTF_CTRL			0x416c
#define AXI_EN_RCLK_QOS_ARRAY_FIX		(1u << 13)
#define AXI_EN_QOS_UPDATE_TIMING_FIX		(1u << 12)
#define AXI_DIS_QOS_GATING_IN_MASTER		(1u << 11)
#define AXI_REQFIFO_EN_QOS_PROPAGATION		(1u << 7)
#define AXI_MASTER_MAX_OUTSTANDING_MASK		0x3f
#define HARD_DEBUG_CLKREQ_DEBUG_ENABLE		0x2
#define HARD_DEBUG_PERST_ASSERT			0x8
#define HARD_DEBUG_REFCLK_OVRD_ENABLE		0x10000
#define HARD_DEBUG_REFCLK_OVRD_OUT		0x100000
#define HARD_DEBUG_L1SS_ENABLE			0x200000
#define HARD_DEBUG_SERDES_IDDQ			0x08000000
#define HARD_DEBUG_CLKREQ_MASK			(HARD_DEBUG_CLKREQ_DEBUG_ENABLE | \
						 HARD_DEBUG_REFCLK_OVRD_ENABLE | \
						 HARD_DEBUG_REFCLK_OVRD_OUT | \
						 HARD_DEBUG_L1SS_ENABLE)

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

/*
 * Controllers phase 3 brings up from reset, in the same form.  PCIe1, the
 * NVMe slot, by default.  It needs the freebsd-pcie1 overlay: without a
 * 1:1 RAM mapping, attach leaves the device in PERST#.  To leave PCIe1
 * untouched, set it at the loader to a value that names no controller, e.g.
 * "none".  (Its multi-page NVMe reads were wrong
 * until nvme(4) kept bounced page offsets and the overlays reserved the RAM
 * the windows shadow: rpi5_modules.git doc/M2_PCIE_HOST.md, phase 3.)
 */
static char bcm2712_pcib_reset[128] = "1000110000";
SYSCTL_STRING(_hw_bcm2712_pcib, OID_AUTO, reset, CTLFLAG_RDTUN,
    bcm2712_pcib_reset, sizeof(bcm2712_pcib_reset),
    "DT unit addresses of the controllers phase 3 brings up from reset");

/*
 * Run the shared SATA/PCIe resistor calibration before a bring-up from
 * reset, as Linux does.  Off by default: it is shared with the adopted
 * PCIe2, whose link the firmware trained after its own calibration.
 */
static int bcm2712_pcib_rescal;
SYSCTL_INT(_hw_bcm2712_pcib, OID_AUTO, rescal, CTLFLAG_RDTUN,
    &bcm2712_pcib_rescal, 0,
    "Run rescal before bringing a controller up from reset");

/*
 * MISC_CTRL fields a bring-up from reset writes, where Linux and EDK2 differ
 * (rpi5_modules.git doc/M2_PCIE_HOST.md, phase 3).  The defaults are Linux's
 * for 7712; EDK2's effective values are burst 0, rcb64 0, scb0_size 0x15.
 * -1 leaves a field as the bridge reset left it.  Added to find which of
 * them broke multi-page NVMe reads; none did (the cause was bounced page
 * offsets).  Kept for experiments; not a permanent interface.
 */
static int bcm2712_pcib_burst = MAX_BURST_SIZE_512;
SYSCTL_INT(_hw_bcm2712_pcib, OID_AUTO, burst, CTLFLAG_RDTUN,
    &bcm2712_pcib_burst, 0, "MISC_CTRL MAX_BURST_SIZE field from reset");
static int bcm2712_pcib_rcb64 = 1;
SYSCTL_INT(_hw_bcm2712_pcib, OID_AUTO, rcb64, CTLFLAG_RDTUN,
    &bcm2712_pcib_rcb64, 0, "MISC_CTRL RCB_64B_MODE from reset");
static int bcm2712_pcib_scb0_size = -1;
SYSCTL_INT(_hw_bcm2712_pcib, OID_AUTO, scb0_size, CTLFLAG_RDTUN,
    &bcm2712_pcib_scb0_size, 0, "MISC_CTRL SCB0_SIZE field from reset");

struct bcm2712_pcib_softc {
	struct generic_pcie_fdt_softc	base;
	device_t			dev;
	const struct bcm2712_pcib_cfg	*cfg;
	hwreset_t			bridge_rst;
	struct mtx			config_mtx;
	bus_dma_tag_t			dmat;
	bool				ram_1to1;
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
 * Does [lo, hi] hold RAM that the kernel may give a device for DMA?
 *
 * dump_avail[] is RAM less the regions the kernel was told to leave alone
 * (EXFLAG_NODUMP: /reserved-memory no-map nodes, the EFI memreserve table).
 * Unlike phys_avail[], it keeps the kernel image, whose static buffers can
 * be DMA targets too.
 */
static bool
bcm2712_pcib_ram_in(vm_paddr_t lo, vm_paddr_t hi)
{
	int i;

	for (i = 0; dump_avail[i + 1] != 0; i += 2)
		if (dump_avail[i] <= hi && dump_avail[i + 1] > lo)
			return (true);
	return (false);
}

/*
 * Program the inbound windows from dma-ranges and build the DMA tag.
 *
 * busdma gives devices CPU physical addresses, so all of RAM must be in a
 * window that maps PCIe X to CPU X.  The outbound window's PCIe range then
 * shadows the RAM at the same addresses: once the root port's memory
 * window (which bcm2712_pcib_relocate_bridge_window() keeps inside that
 * range) decodes an address, a device's DMA to it is taken for a transfer
 * to a device behind the bridge, and never reaches RAM.
 *
 * The freebsd-pcieN overlays reserve the shadowed RAM, so nothing can be
 * allocated there, and the tag needs no exclusion.  That matters beyond
 * the shadow itself: a child tag's exclusion is the parent's widened to
 * the child's highaddr (common_bus_dma_tag_create() takes the MIN of the
 * lowaddrs and the MAX of the highaddrs), so for nvme(4) or rp1_eth, whose
 * highaddr is BUS_SPACE_MAXADDR, excluding the shadow here means bouncing
 * every buffer above it.  If the RAM is not reserved, it is excluded
 * anyway, as a safety net, and bouncing is the cost.
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
		    "Expected a freebsd-pcieN device-tree overlay "
		    "(rpi5_modules.git doc/DT_OVERLAYS.md).\n",
		    (uintmax_t)ram_end);
	sc->ram_1to1 = ram_1to1;

	bcm2712_pcib_set_inbound_wins(sc, wins, n);

	/* Keep DMA out of the RAM the outbound window shadows, if any. */
	lowaddr = BUS_SPACE_MAXADDR;
	highaddr = BUS_SPACE_MAXADDR;
	ob_lo = sc->base.base.ranges[0].pci_base;
	ob_hi = ob_lo + sc->base.base.ranges[0].size - 1;
	if (ram_1to1 && ob_lo < ram_end) {
		if (bcm2712_pcib_ram_in(ob_lo, ob_hi)) {
			lowaddr = ob_lo - 1;
			highaddr = ob_hi;
			device_printf(sc->dev, "DMA excluded from 0x%jx-0x%jx, "
			    "RAM shadowed by the outbound window and not "
			    "reserved: devices below may bounce any buffer "
			    "from 0x%jx up\n", (uintmax_t)ob_lo,
			    (uintmax_t)ob_hi, (uintmax_t)ob_lo);
		} else
			device_printf(sc->dev, "outbound window 0x%jx-0x%jx "
			    "shadows no RAM the kernel uses (reserved); no DMA "
			    "exclusion\n", (uintmax_t)ob_lo, (uintmax_t)ob_hi);
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

/*
 * Is this controller's DT unit address in list (hw.bcm2712_pcib.adopt or
 * .reset)?
 */
static bool
bcm2712_pcib_listed(device_t dev, const char *list)
{
	const char *name, *ua, *p;
	size_t len, n;

	name = ofw_bus_get_name(dev);
	if (name == NULL || (ua = strchr(name, '@')) == NULL)
		return (false);
	ua++;
	len = strlen(ua);
	for (p = list; *p != '\0'; p += n) {
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
 * Phase 3: bringing a controller up from reset, as Linux pcie-brcmstb.c
 * does with bcm2712_cfg: brcm_pcie_probe(), brcm_pcie_setup(),
 * brcm_pcie_post_setup_bcm2712() and brcm_pcie_start_link().  bcm2712_cfg
 * has no PHY to start (brcm_phy_start() does nothing without has_phy),
 * drives PERST# the 7278 way, has the NO_SSC quirk, and no SCB sizes to
 * program (brcm_pcie_get_inbound_wins() returns before them for 7712).
 */

/* Linux brcm_pcie_mdio_write(), port 0 only.  0 or ETIMEDOUT. */
static int
bcm2712_pcib_mdio_write(struct bcm2712_pcib_softc *sc, u_int port,
    u_int regad, uint16_t data)
{
	int us;

	bcm2712_pcib_set_reg(sc, REG_MDIO_ADDR, (port << MDIO_PORT_SHIFT) |
	    (regad & 0xffff) | MDIO_CMD_WRITE);
	(void)bcm2712_pcib_read_reg(sc, REG_MDIO_ADDR);
	bcm2712_pcib_set_reg(sc, REG_MDIO_WR_DATA, MDIO_DATA_DONE | data);
	for (us = 0; us <= 100; us += 10) {
		if ((bcm2712_pcib_read_reg(sc, REG_MDIO_WR_DATA) &
		    MDIO_DATA_DONE) == 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

/*
 * Linux brcm_pcie_probe() up to brcm_pcie_setup()'s first register access:
 * the bridge out of reset, the shared calibration if asked for, then
 * brcm_pcie_setup()'s bridge reset pulse.  Touches only the reset
 * controllers.
 */
static int
bcm2712_pcib_reset_bridge(device_t dev)
{
	struct bcm2712_pcib_softc *sc;
	hwreset_t rescal;
	int error;

	sc = device_get_softc(dev);
	error = hwreset_get_by_ofw_name(dev, 0, "bridge", &sc->bridge_rst);
	if (error != 0) {
		device_printf(dev, "no \"bridge\" reset in the DT (%d); "
		    "not touching the controller\n", error);
		return (ENXIO);
	}
	error = hwreset_deassert(sc->bridge_rst);
	if (error != 0) {
		device_printf(dev, "cannot deassert the bridge reset (%d)\n",
		    error);
		goto fail;
	}

	if (bcm2712_pcib_rescal != 0) {
		error = hwreset_get_by_ofw_name(dev, 0, "rescal", &rescal);
		if (error == 0) {
			error = hwreset_deassert(rescal);
			hwreset_release(rescal);
		}
		if (error != 0) {
			device_printf(dev, "rescal failed (%d)\n", error);
			goto fail;
		}
	} else
		device_printf(dev, "rescal left as the firmware set it "
		    "(hw.bcm2712_pcib.rescal=0)\n");

	/* brcm_pcie_setup(): reset the bridge, then take it out of reset. */
	error = hwreset_assert(sc->bridge_rst);
	if (error == 0) {
		DELAY(200);
		error = hwreset_deassert(sc->bridge_rst);
	}
	if (error != 0) {
		device_printf(dev, "cannot pulse the bridge reset (%d)\n",
		    error);
		goto fail;
	}
	return (0);
fail:
	hwreset_release(sc->bridge_rst);
	return (ENXIO);
}

/*
 * Linux brcm_pcie_setup() after the bridge reset, less the windows and the
 * class code, which attach programs on both paths.
 */
static int
bcm2712_pcib_setup(struct bcm2712_pcib_softc *sc)
{
	uint32_t aspm, tmp;

	/* SerDes out of IDDQ, then let it settle. */
	tmp = bcm2712_pcib_read_reg(sc, sc->cfg->hard_debug);
	tmp &= ~HARD_DEBUG_SERDES_IDDQ;
	bcm2712_pcib_set_reg(sc, sc->cfg->hard_debug, tmp);
	DELAY(200);

	tmp = bcm2712_pcib_read_reg(sc, REG_MISC_CTRL);
	tmp |= MISC_CTRL_SCB_ACCESS_EN | MISC_CTRL_CFG_READ_UR_MODE |
	    MISC_CTRL_RCB_MPS_MODE;
	if (bcm2712_pcib_rcb64 == 0)
		tmp &= ~MISC_CTRL_RCB_64B_MODE;
	else if (bcm2712_pcib_rcb64 > 0)
		tmp |= MISC_CTRL_RCB_64B_MODE;
	if (bcm2712_pcib_burst >= 0) {
		tmp &= ~MISC_CTRL_MAX_BURST_SIZE_MASK;
		tmp |= (bcm2712_pcib_burst << MISC_CTRL_MAX_BURST_SIZE_SHIFT) &
		    MISC_CTRL_MAX_BURST_SIZE_MASK;
	}
	if (bcm2712_pcib_scb0_size >= 0) {
		tmp &= ~MISC_CTRL_SCB0_SIZE_MASK;
		tmp |= (bcm2712_pcib_scb0_size << MISC_CTRL_SCB0_SIZE_SHIFT) &
		    MISC_CTRL_SCB0_SIZE_MASK;
	}
	bcm2712_pcib_set_reg(sc, REG_MISC_CTRL, tmp);
	device_printf(sc->dev, "MISC_CTRL now 0x%08x (burst %d, rcb64 %d, "
	    "scb0_size %d; -1 = as the reset left it)\n", tmp,
	    bcm2712_pcib_burst, bcm2712_pcib_rcb64, bcm2712_pcib_scb0_size);

	if ((bcm2712_pcib_read_reg(sc, REG_BRIDGE_STATE) &
	    BRIDGE_STATE_PORT) == 0) {
		device_printf(sc->dev, "misconfigured as an endpoint\n");
		return (ENXIO);
	}

	/* Always advertise L1; L0s too unless aspm-no-l0s. */
	aspm = ASPM_SUPPORT_L1;
	if (!OF_hasprop(ofw_bus_get_node(sc->dev), "aspm-no-l0s"))
		aspm |= ASPM_SUPPORT_L0S;
	tmp = bcm2712_pcib_read_reg(sc, REG_LINK_CAPABILITY);
	tmp &= ~LINK_CAPABILITY_ASPM_SUPPORT_MASK;
	tmp |= aspm << LINK_CAPABILITY_ASPM_SUPPORT_SHIFT;
	bcm2712_pcib_set_reg(sc, REG_LINK_CAPABILITY, tmp);

	/* PCIe->SCB endian mode for inbound window: little endian. */
	tmp = bcm2712_pcib_read_reg(sc, REG_VENDOR_SPECIFIC_REG1);
	tmp &= ~VENDOR_REG1_ENDIAN_MODE_BAR2_MASK;
	bcm2712_pcib_set_reg(sc, REG_VENDOR_SPECIFIC_REG1, tmp);

	return (0);
}

/* Linux brcm_pcie_post_setup_bcm2712(). */
static int
bcm2712_pcib_post_setup(struct bcm2712_pcib_softc *sc)
{
	static const uint16_t data[] =
	    { 0x50b9, 0xbda1, 0x0094, 0x97b4, 0x5030, 0x5030, 0x0007 };
	static const uint8_t regs[] =
	    { 0x16, 0x17, 0x18, 0x19, 0x1b, 0x1c, 0x1e };
	phandle_t node;
	uint8_t qos_map[4];
	uint32_t tmp;
	int error, i;

	/* Allow a 54 MHz (xosc) refclk source. */
	error = bcm2712_pcib_mdio_write(sc, MDIO_PORT0, MDIO_SET_ADDR_OFFSET,
	    0x1600);
	for (i = 0; error == 0 && i < nitems(regs); i++)
		error = bcm2712_pcib_mdio_write(sc, MDIO_PORT0, regs[i],
		    data[i]);
	if (error != 0) {
		device_printf(sc->dev, "refclk PLL set-up over MDIO timed "
		    "out\n");
		return (error);
	}
	DELAY(200);

	/* L1SS sub-state timers: PM clock period 18.52 ns (1/54 MHz). */
	tmp = bcm2712_pcib_read_reg(sc, REG_PL_PHY_CTL_15);
	tmp &= ~PL_PHY_CTL_15_PM_CLK_PERIOD_MASK;
	tmp |= 0x12;
	bcm2712_pcib_set_reg(sc, REG_PL_PHY_CTL_15, tmp);

	/* UBUS-AXI bridge: failed reads return all ones, not an AXI error. */
	tmp = bcm2712_pcib_read_reg(sc, REG_UBUS_CTRL);
	tmp |= UBUS_CTRL_REPLY_ERR_DIS | UBUS_CTRL_REPLY_DECERR_DIS;
	bcm2712_pcib_set_reg(sc, REG_UBUS_CTRL, tmp);
	bcm2712_pcib_set_reg(sc, REG_AXI_READ_ERROR_DATA, 0xffffffff);

	/*
	 * UBUS timeout 250 ms, then the RC config retry timeout ~240 ms,
	 * in clocks of 750 MHz.
	 */
	bcm2712_pcib_set_reg(sc, REG_UBUS_TIMEOUT, 0xb2d0000);
	bcm2712_pcib_set_reg(sc, REG_RC_CONFIG_RETRY_TIMEOUT, 0xaba0000);

	/* Disable broken forwarding search; chicken bits for 2712D0. */
	tmp = bcm2712_pcib_read_reg(sc, REG_AXI_INTF_CTRL);
	tmp &= ~AXI_REQFIFO_EN_QOS_PROPAGATION;
	tmp |= AXI_EN_RCLK_QOS_ARRAY_FIX | AXI_EN_QOS_UPDATE_TIMING_FIX |
	    AXI_DIS_QOS_GATING_IN_MASTER;
	bcm2712_pcib_set_reg(sc, REG_AXI_INTF_CTRL, tmp);

	/*
	 * QOS_UPDATE_TIMING_FIX reads as 0 on a 2712C1 or a single-lane RC:
	 * throttle AXI requests in flight instead.
	 */
	tmp = bcm2712_pcib_read_reg(sc, REG_AXI_INTF_CTRL);
	if ((tmp & AXI_EN_QOS_UPDATE_TIMING_FIX) == 0) {
		tmp &= ~AXI_MASTER_MAX_OUTSTANDING_MASK;
		tmp |= 15;
		bcm2712_pcib_set_reg(sc, REG_AXI_INTF_CTRL, tmp);
	}

	/* VDM reception off. */
	tmp = bcm2712_pcib_read_reg(sc, REG_MISC_CTRL_1);
	tmp &= ~MISC_CTRL_1_EN_VDM_QOS_CONTROL;
	bcm2712_pcib_set_reg(sc, REG_MISC_CTRL_1, tmp);

	/*
	 * brcm,fifo-qos-map: a QoS for each quartile of FIFO level, the same
	 * for every TC.  Linux's alternative, brcm,vdm-qos-map, is not
	 * implemented; it is only reported.
	 */
	node = ofw_bus_get_node(sc->dev);
	if (OF_getprop(node, "brcm,fifo-qos-map", qos_map,
	    sizeof(qos_map)) == sizeof(qos_map)) {
		tmp = 0;
		for (i = 0; i < 4; i++)
			tmp |= (uint32_t)(qos_map[i] & 0x0f) << (i * 4);
		for (i = 0; i < 8; i++)
			bcm2712_pcib_set_reg(sc, REG_TC_QUEUE_TO_QOS_MAP(i), tmp);
	} else if (OF_hasprop(node, "brcm,vdm-qos-map"))
		device_printf(sc->dev, "brcm,vdm-qos-map is not implemented; "
		    "ignored\n");

	return (0);
}

/* Linux brcm_config_clkreq(), for the DT's brcm,clkreq-mode. */
static void
bcm2712_pcib_config_clkreq(struct bcm2712_pcib_softc *sc)
{
	char mode[16];
	uint32_t hd, tmp;
	bool no_l1ss;

	memset(mode, 0, sizeof(mode));
	if (OF_getprop(ofw_bus_get_node(sc->dev), "brcm,clkreq-mode", mode,
	    sizeof(mode) - 1) <= 0)
		strlcpy(mode, "default", sizeof(mode));

	/*
	 * Read-modify-write, starting from the register with the CLKREQ bits
	 * cleared.  The Raspberry Pi Linux source ORs the mode's bits into an
	 * uninitialised clkreq_cntl and writes that whole; this keeps the
	 * other HARD_DEBUG bits (SERDES_IDDQ among them) as they are.
	 */
	hd = bcm2712_pcib_read_reg(sc, sc->cfg->hard_debug);
	hd &= ~HARD_DEBUG_CLKREQ_MASK;
	no_l1ss = true;
	if (strcmp(mode, "no-l1ss") == 0)
		hd |= HARD_DEBUG_CLKREQ_DEBUG_ENABLE;
	else if (strcmp(mode, "default") == 0) {
		/* brcm_extend_rbus_timeout() does nothing on 7712. */
		hd |= HARD_DEBUG_L1SS_ENABLE;
		no_l1ss = false;
	} else {
		if (strcmp(mode, "safe") != 0)
			device_printf(sc->dev, "invalid brcm,clkreq-mode "
			    "\"%s\"\n", mode);
		strlcpy(mode, "safe", sizeof(mode));
		hd |= HARD_DEBUG_REFCLK_OVRD_OUT | HARD_DEBUG_REFCLK_OVRD_ENABLE;
	}
	if (no_l1ss) {
		/* Un-advertise L1 substates. */
		tmp = bcm2712_pcib_read_reg(sc, REG_ROOT_CAP);
		tmp &= ~ROOT_CAP_L1SS_MODE_MASK;
		tmp |= 2 << ROOT_CAP_L1SS_MODE_SHIFT;
		bcm2712_pcib_set_reg(sc, REG_ROOT_CAP, tmp);
	}
	bcm2712_pcib_set_reg(sc, sc->cfg->hard_debug, hd);
	device_printf(sc->dev, "clkreq-mode set to %s\n", mode);
}

static void
bcm2712_pcib_set_perst(struct bcm2712_pcib_softc *sc, bool assert)
{
	uint32_t tmp;

	/* The 7278 way: PERSTB, where 0 asserts PERST#. */
	tmp = bcm2712_pcib_read_reg(sc, REG_PCIE_CTRL);
	if (assert)
		tmp &= ~PCIE_CTRL_PERSTB;
	else
		tmp |= PCIE_CTRL_PERSTB;
	bcm2712_pcib_set_reg(sc, REG_PCIE_CTRL, tmp);
}

/* Linux brcm_pcie_start_link(), without SSC (the NO_SSC quirk). */
static int
bcm2712_pcib_start_link(struct bcm2712_pcib_softc *sc)
{
	phandle_t node;
	pcell_t gen, tperst_ms;
	uint32_t tmp;
	uint16_t val;
	int ms;

	node = ofw_bus_get_node(sc->dev);

	/* Limit the generation to max-link-speed: brcm_pcie_set_gen(). */
	if (OF_getencprop(node, "max-link-speed", &gen, sizeof(gen)) ==
	    sizeof(gen) && gen > 0) {
		tmp = bcm2712_pcib_read_reg(sc, REG_LINK_CAPABILITY);
		tmp &= ~LINK_CAPABILITY_MAX_LINK_SPEED_MASK;
		tmp |= gen & LINK_CAPABILITY_MAX_LINK_SPEED_MASK;
		bcm2712_pcib_set_reg(sc, REG_LINK_CAPABILITY, tmp);
		val = le16toh(bus_read_2(sc->base.base.res,
		    REG_PCIE_CAP + PCIER_LINK_CTL2));
		val &= ~LINK_CTL2_TARGET_SPEED_MASK;
		val |= gen & LINK_CTL2_TARGET_SPEED_MASK;
		bus_write_2(sc->base.base.res, REG_PCIE_CAP + PCIER_LINK_CTL2,
		    htole16(val));
	}

	/* CLKREQ# input off before link-up. */
	tmp = bcm2712_pcib_read_reg(sc, sc->cfg->hard_debug);
	tmp &= ~HARD_DEBUG_CLKREQ_MASK;
	bcm2712_pcib_set_reg(sc, sc->cfg->hard_debug, tmp);

	/*
	 * Deassert PERST#.  brcm,tperst-clk-ms keeps the PERST# output low
	 * for that long after the internal reset is released, so that the
	 * refclk is stable sooner.
	 */
	if (OF_getencprop(node, "brcm,tperst-clk-ms", &tperst_ms,
	    sizeof(tperst_ms)) == sizeof(tperst_ms) && tperst_ms > 0) {
		tmp = bcm2712_pcib_read_reg(sc, sc->cfg->hard_debug);
		bcm2712_pcib_set_reg(sc, sc->cfg->hard_debug,
		    tmp | HARD_DEBUG_PERST_ASSERT);
		bcm2712_pcib_set_perst(sc, false);
		DELAY(tperst_ms * 1000);
		tmp = bcm2712_pcib_read_reg(sc, sc->cfg->hard_debug);
		bcm2712_pcib_set_reg(sc, sc->cfg->hard_debug,
		    tmp & ~HARD_DEBUG_PERST_ASSERT);
	} else
		bcm2712_pcib_set_perst(sc, false);

	/*
	 * 100 ms after PERST# (PCIe CEM 2.2, PCIe r5.0 6.6.1), then up to
	 * 100 ms more for the link.
	 */
	DELAY(100 * 1000);
	for (ms = 0; ms < 100 && !bcm2712_pcib_link_up(sc); ms += 5)
		DELAY(5 * 1000);
	if (!bcm2712_pcib_link_up(sc)) {
		device_printf(sc->dev, "link down (status 0x%08x)\n",
		    bcm2712_pcib_read_reg(sc, REG_BRIDGE_STATE));
		return (ENXIO);
	}

	bcm2712_pcib_config_clkreq(sc);

	/* Root Control is reset by PERST#: re-enable CRS visibility. */
	val = le16toh(bus_read_2(sc->base.base.res,
	    REG_PCIE_CAP + PCIER_ROOT_CAP));
	if ((val & PCIEM_ROOT_CAP_CRS_VIS) != 0) {
		val = le16toh(bus_read_2(sc->base.base.res,
		    REG_PCIE_CAP + PCIER_ROOT_CTL));
		bus_write_2(sc->base.base.res, REG_PCIE_CAP + PCIER_ROOT_CTL,
		    htole16(val | PCIEM_ROOT_CTL_CRS_VIS));
	}
	return (0);
}

/*
 * The controller's own registers that bear on DMA, where Linux, EDK2 and the
 * firmware differ (rpi5_modules.git doc/M2_PCIE_HOST.md, phase 3): logged as
 * found at attach, and readable later under dev.pcib.N.regs.
 */
static const struct {
	const char	*name;
	bus_size_t	reg;	/* 0: cfg->hard_debug */
} bcm2712_pcib_regs[] = {
	{ "misc_ctrl",		REG_MISC_CTRL },
	{ "hard_debug",		0 },
	{ "axi_intf_ctrl",	REG_AXI_INTF_CTRL },
	{ "misc_ctrl_1",	REG_MISC_CTRL_1 },
	{ "ubus_ctrl",		REG_UBUS_CTRL },
	{ "ubus_timeout",	REG_UBUS_TIMEOUT },
	{ "rc_config_retry_timeout", REG_RC_CONFIG_RETRY_TIMEOUT },
	{ "pcie_ctrl",		REG_PCIE_CTRL },
	{ "pl_phy_ctl_15",	REG_PL_PHY_CTL_15 },
	{ "vendor_specific_reg1", REG_VENDOR_SPECIFIC_REG1 },
};

static bus_size_t
bcm2712_pcib_regs_offset(struct bcm2712_pcib_softc *sc, u_int i)
{

	return (bcm2712_pcib_regs[i].reg != 0 ? bcm2712_pcib_regs[i].reg :
	    sc->cfg->hard_debug);
}

static int
bcm2712_pcib_reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct bcm2712_pcib_softc *sc;
	uint32_t val;

	sc = arg1;
	val = bcm2712_pcib_read_reg(sc, bcm2712_pcib_regs_offset(sc, arg2));
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static void
bcm2712_pcib_regs_report(struct bcm2712_pcib_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	u_int i;

	device_printf(sc->dev, "as found:");
	for (i = 0; i < nitems(bcm2712_pcib_regs); i++)
		printf(" %s=0x%08x", bcm2712_pcib_regs[i].name,
		    bcm2712_pcib_read_reg(sc, bcm2712_pcib_regs_offset(sc, i)));
	printf("\n");

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO, "regs",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Controller registers (live)");
	for (i = 0; i < nitems(bcm2712_pcib_regs); i++)
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    bcm2712_pcib_regs[i].name,
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, i,
		    bcm2712_pcib_reg_sysctl, "IU", "");
}

/*
 * The DT "bridge" reset's state, for a controller left untouched.  Reading
 * it touches only the reset controller.
 */
static const char *
bcm2712_pcib_bridge_reset_state(device_t dev)
{
	hwreset_t rst;
	bool asserted;
	int error;

	if (hwreset_get_by_ofw_name(dev, 0, "bridge", &rst) != 0)
		return ("not in the DT");
	error = hwreset_is_asserted(rst, &asserted);
	hwreset_release(rst);
	if (error != 0)
		return ("unreadable");
	return (asserted ? "asserted" : "deasserted");
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
	bool adopt;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->cfg = (const struct bcm2712_pcib_cfg *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	/* Adopt a trained link (phase 1), or bring one up from reset. */
	adopt = bcm2712_pcib_listed(dev, bcm2712_pcib_adopt);
	if (!adopt && !bcm2712_pcib_listed(dev, bcm2712_pcib_reset)) {
		device_printf(dev, "not in hw.bcm2712_pcib.adopt (\"%s\") or "
		    ".reset (\"%s\"); left untouched, bridge reset %s\n",
		    bcm2712_pcib_adopt, bcm2712_pcib_reset,
		    bcm2712_pcib_bridge_reset_state(dev));
		return (ENXIO);
	}

	if (adopt)
		error = bcm2712_pcib_check_reset(dev);
	else
		error = bcm2712_pcib_reset_bridge(dev);
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
	/* Adopted: as the firmware left it.  From reset: after the bridge reset. */
	bcm2712_pcib_regs_report(sc);

	if (adopt) {
		/*
		 * Phase 1 adopts the link the firmware trained (pciex4_reset=0
		 * for PCIe2); it does not reset, set up the PHY or train one.
		 */
		if (!bcm2712_pcib_link_up(sc)) {
			device_printf(dev, "error: link is not up (status "
			    "0x%08x); not in hw.bcm2712_pcib.reset, so not "
			    "trained here.\n",
			    bcm2712_pcib_read_reg(sc, REG_BRIDGE_STATE));
			error = ENXIO;
			goto failed;
		}
	} else {
		error = bcm2712_pcib_setup(sc);
		if (error != 0)
			goto failed;
	}

	mtx_init(&sc->config_mtx, "bcm2712_pcib: config_mtx", NULL, MTX_DEF);

	if (adopt) {
		link_state = bcm2712_pcib_read_reg(sc, REG_BRIDGE_LINK_STATE) >>
		    0x10;
		device_printf(dev, "link up at %s (adopted from the firmware).\n",
		    bcm2712_pcib_link_state_string(link_state));

		/* Failed reads return all ones, not 0xdeaddead, and do not abort. */
		if (sc->cfg->ubus_err_suppress) {
			tmp = bcm2712_pcib_read_reg(sc, REG_UBUS_CTRL);
			tmp |= UBUS_CTRL_REPLY_ERR_DIS | UBUS_CTRL_REPLY_DECERR_DIS;
			bcm2712_pcib_set_reg(sc, REG_UBUS_CTRL, tmp);
			bcm2712_pcib_set_reg(sc, REG_AXI_READ_ERROR_DATA,
			    0xffffffff);
		}
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

	if (!adopt) {
		/*
		 * A device's DMA would miss RAM, and an NVMe controller would
		 * fetch garbage for commands: do not release PERST#.
		 */
		if (!sc->ram_1to1) {
			device_printf(dev, "not starting the link: RAM is not "
			    "mapped 1:1 for DMA\n");
			error = ENXIO;
		}
		if (error == 0)
			error = bcm2712_pcib_post_setup(sc);
		if (error == 0)
			error = bcm2712_pcib_start_link(sc);
		if (error != 0) {
			bcm2712_pcib_set_perst(sc, true);
			if (sc->dmat != NULL) {
				bus_dma_tag_destroy(sc->dmat);
				sc->dmat = NULL;
			}
			mtx_destroy(&sc->config_mtx);
			goto failed;
		}
		tmp = bcm2712_pcib_read_reg(sc, REG_BRIDGE_LINK_STATE) >> 0x10;
		device_printf(dev, "link up at %s x%u (trained from reset).\n",
		    bcm2712_pcib_link_state_string(tmp),
		    (tmp & PCIEM_LINK_STA_WIDTH) >> 4);
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
