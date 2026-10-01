/*-
 * rpi_pcie.c -- just enough of PCIe2 to reach RP1, for the USB keyboard.
 *
 * RP1, the Pi 5's I/O chip, carries the USB controllers (and Ethernet,
 * GPIO, the fan PWM), and sits behind the SoC's PCIe2 root complex.  This
 * file does not train the link: with config.txt's pciex4_reset=0 the
 * firmware leaves it trained, and with the default (1) it resets PCIe2 at
 * hand-off, in which case there is no RP1 here and the loader goes without
 * USB.  What the firmware does not leave is the device tree's address
 * layout (measured, doc/LOADER_ZIMAGE.md "pciex4_reset=0"): its outbound
 * window maps CPU 0x1c_0000_0000 to PCIe 0x8000_0000, and RP1's BARs sit
 * somewhere in there.  So this sets up what the kernel's bcm2712_pcib and
 * rp1pci set up, from the same device tree values (pcie@1000120000):
 *
 *	ranges	   PCIe 0xc000_0000 -> CPU 0x1f_0000_0000, 16 MiB (window 0)
 *	RP1 BAR1   PCIe 0xc000_0000, so RP1's peripherals are at CPU
 *		   0x1f_0000_0000 and its USB controllers at +0x200000 and
 *		   +0x300000 ("rp1pci0: BAR1 (peripherals): PCIe 0xc0000000
 *		   -> CPU 0x1f00000000" in every kernel boot)
 *	dma-ranges PCIe 0 -> CPU 0, 64 GiB: the firmware already maps
 *		   inbound 1:1 with pciex4_reset=0 (doc/M2_PCIE_HOST.md), so
 *		   a DMA address is a physical address
 *
 * The kernel resets PCIe2 at boot by default (hw.bcm2712_pcib.reset), so
 * none of this outlives the loader; rpi_pcie_shutdown() still takes RP1's
 * bus mastering away first, so nothing on RP1 can write to memory the
 * kernel is about to use.
 *
 * Register offsets and encodings: sys/arm64/broadcom/bcm2712/bcm2712_pcib.c.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"
#include "librpiboot.h"

#define	PCIE2_BASE		0x1000120000UL

#define	REG_BRIDGE_STATE	0x4068
#define	  BRIDGE_STATE_PHYLINKUP	0x10
#define	  BRIDGE_STATE_DL_ACTIVE	0x20
#define	  BRIDGE_STATE_PORT		0x80	/* a root complex */
#define	REG_BUS_WINDOW_LOW	0x400c
#define	REG_BUS_WINDOW_HIGH	0x4010
#define	REG_CPU_WINDOW_LOW	0x4070
#define	REG_CPU_WINDOW_START_HIGH 0x4080
#define	REG_CPU_WINDOW_END_HIGH	0x4084
#define	REG_EXT_CFG_DATA	0x8000
#define	REG_EXT_CFG_INDEX	0x9000

/* The device tree's window 0 and RP1's BAR1 in it. */
#define	WIN_PCI_BASE		0xc0000000UL
#define	WIN_CPU_BASE		0x1f00000000UL
#define	WIN_SIZE		0x01000000UL

/* Standard config space. */
#define	PCIR_DEVVENDOR		0x00
#define	PCIR_COMMAND		0x04
#define	  PCIM_CMD_MEMEN	0x0002
#define	  PCIM_CMD_BUSMASTEREN	0x0004
#define	PCIR_BAR1		0x14
#define	PCIR_PRIBUS_1		0x18	/* primary, secondary, subordinate */
#define	PCIR_MEMBASE_1		0x20	/* memory base and limit */
#define	PCIR_PMBASEL_1		0x24	/* prefetchable base and limit */
#define	PCIR_PMBASEH_1		0x28
#define	PCIR_PMLIMITH_1		0x2c

#define	RC_ID			0x271214e4	/* 14e4:2712 */
#define	RP1_ID			0x00011de4	/* 1de4:0001 */
#define	RP1_SYSINFO_CHIP_ID	0x20001927	/* SYSINFO at BAR1 + 0 */

static bool rp1_up;

static inline uint32_t
pcie_rd(uint32_t reg)
{
	return (*(volatile uint32_t *)(PCIE2_BASE + reg));
}

static inline void
pcie_wr(uint32_t reg, uint32_t v)
{
	*(volatile uint32_t *)(PCIE2_BASE + reg) = v;
}

/* The root port's own config space is at the controller's base. */
static uint32_t
rc_cfg_rd(uint32_t reg)
{
	return (pcie_rd(reg));
}

static void
rc_cfg_wr(uint32_t reg, uint32_t v)
{
	pcie_wr(reg, v);
}

/*
 * RP1 is bus 1, device 0, function 0: select it in the index register,
 * then use the data window (PCIE_ADDR_OFFSET(1, 0, 0, 0) = 1 << 20).
 * Only valid with the link up; an access below the root port with the link
 * down aborts the CPU.
 */
static uint32_t
rp1_cfg_rd(uint32_t reg)
{
	pcie_wr(REG_EXT_CFG_INDEX, 1U << 20);
	return (pcie_rd(REG_EXT_CFG_DATA + reg));
}

static void
rp1_cfg_wr(uint32_t reg, uint32_t v)
{
	pcie_wr(REG_EXT_CFG_INDEX, 1U << 20);
	pcie_wr(REG_EXT_CFG_DATA + reg, v);
}

static bool
link_up(void)
{
	uint32_t st = pcie_rd(REG_BRIDGE_STATE);

	return ((st & (BRIDGE_STATE_PHYLINKUP | BRIDGE_STATE_DL_ACTIVE)) ==
	    (BRIDGE_STATE_PHYLINKUP | BRIDGE_STATE_DL_ACTIVE));
}

/*
 * Map RP1 at CPU 0x1f_0000_0000.  Returns 0, or ENXIO with a one-line
 * reason in *why.
 */
int
rpi_pcie_rp1_init(const char **why)
{
	uint32_t bar, id, st, chip;

	if (rp1_up)
		return (0);
	st = pcie_rd(REG_BRIDGE_STATE);
	if ((st & BRIDGE_STATE_PORT) == 0) {
		*why = "PCIe2 is not configured as a root complex";
		return (ENXIO);
	}
	if (!link_up()) {
		*why = "PCIe2 link down (config.txt pciex4_reset=1?)";
		return (ENXIO);
	}
	if ((id = rc_cfg_rd(PCIR_DEVVENDOR)) != RC_ID) {
		*why = "PCIe2 root port has an unexpected ID";
		return (ENXIO);
	}

	/* Outbound window 0, encoded as bcm2712_pcib's encode_cpu_window_*(). */
	pcie_wr(REG_BUS_WINDOW_LOW, (uint32_t)WIN_PCI_BASE);
	pcie_wr(REG_BUS_WINDOW_HIGH, (uint32_t)(WIN_PCI_BASE >> 32));
	pcie_wr(REG_CPU_WINDOW_LOW,
	    ((WIN_CPU_BASE >> 16) & 0xfff0) |
	    ((WIN_CPU_BASE + WIN_SIZE - 1) & 0xfff00000));
	pcie_wr(REG_CPU_WINDOW_START_HIGH, (WIN_CPU_BASE >> 32) & 0xff);
	pcie_wr(REG_CPU_WINDOW_END_HIGH,
	    ((WIN_CPU_BASE + WIN_SIZE - 1) >> 32) & 0xff);

	/*
	 * The root port as a bridge to bus 1, forwarding the window: memory
	 * base/limit in 1 MiB units, prefetchable range closed.
	 */
	rc_cfg_wr(PCIR_PRIBUS_1, (rc_cfg_rd(PCIR_PRIBUS_1) & 0xff000000) |
	    (1U << 16) | (1U << 8) | 0);
	rc_cfg_wr(PCIR_MEMBASE_1,
	    (((WIN_PCI_BASE + WIN_SIZE - 1) >> 16) & 0xfff0) << 16 |
	    ((WIN_PCI_BASE >> 16) & 0xfff0));
	rc_cfg_wr(PCIR_PMBASEL_1, 0x0000fff0);
	rc_cfg_wr(PCIR_PMBASEH_1, 0);
	rc_cfg_wr(PCIR_PMLIMITH_1, 0);
	rc_cfg_wr(PCIR_COMMAND, (rc_cfg_rd(PCIR_COMMAND) & 0xffff0000) |
	    PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);

	if ((id = rp1_cfg_rd(PCIR_DEVVENDOR)) != RP1_ID) {
		*why = "no RP1 (1de4:0001) on bus 1";
		return (ENXIO);
	}
	bar = rp1_cfg_rd(PCIR_BAR1);
	if ((bar & 0x7) != 0) {
		/* A 64-bit or I/O BAR1 is not the RP1 this was written for. */
		*why = "RP1 BAR1 is not a 32-bit memory BAR";
		return (ENXIO);
	}
	rp1_cfg_wr(PCIR_BAR1, (uint32_t)WIN_PCI_BASE);
	rp1_cfg_wr(PCIR_COMMAND, (rp1_cfg_rd(PCIR_COMMAND) & 0xffff0000) |
	    PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);

	chip = *(volatile uint32_t *)WIN_CPU_BASE;
	if (chip != RP1_SYSINFO_CHIP_ID) {
		*why = "RP1 SYSINFO does not read back its chip ID";
		return (ENXIO);
	}
	rp1_up = true;
	return (0);
}

uint64_t
rpi_pcie_rp1_base(void)
{
	return (WIN_CPU_BASE);
}

/* Before the kernel: RP1 may no longer master the bus. */
void
rpi_pcie_shutdown(void)
{
	if (!rp1_up || !link_up())
		return;
	rp1_cfg_wr(PCIR_COMMAND, rp1_cfg_rd(PCIR_COMMAND) &
	    ~(uint32_t)PCIM_CMD_BUSMASTEREN & 0xffff);
	rp1_up = false;
}
