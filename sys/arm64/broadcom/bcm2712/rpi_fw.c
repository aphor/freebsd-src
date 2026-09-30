/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * rpi_fw -- the Raspberry Pi 5 VPU property mailbox, and TryBoot from the OS.
 *
 * WHAT IT IS FOR
 *
 * The VPU firmware reads one of two files when the board boots: config.txt
 * normally, or tryboot.txt when a one-shot flag was set before the reset.  It
 * clears the flag as it consumes it, so exactly one boot uses tryboot.txt.
 * The stand/arm64/rpiboot loader can already set that flag from its prompt.
 * This driver lets the running OS set it too:
 *
 *	sysctl hw.rpi_fw.tryboot=1 && shutdown -r now
 *
 * With config.txt booting the known-good lane and tryboot.txt booting
 * something under test, a failed test costs one power cycle and never a
 * physical jumper.
 *
 * The sequence is vendor Linux's rpi_firmware_notify_reboot()
 * (drivers/firmware/raspberrypi.c): SET_REBOOT_FLAGS with bit 0 set, then
 * NOTIFY_REBOOT, then reset.  Here the SET happens when the sysctl is written
 * and is read back at once; NOTIFY_REBOOT is sent from shutdown_final, and
 * only when the flag is armed and the system is rebooting, so an ordinary
 * reboot behaves exactly as it did before this driver was loaded.
 *
 * The reset itself is arm64's cpu_reset_hook, which is psci_reset(): PSCI
 * SYSTEM_RESET to BL31.  That is the same path the loader's "tryboot" was
 * measured on (rpi5_modules.git/doc/LOADER_ZIMAGE.md, 2026-09-25), and the
 * flag survived it.  EDK2's ResetSystem is not involved, even on the ACPI
 * lane.
 *
 * WHY NOT bcm2835_mbox
 *
 * sys/arm/broadcom/bcm2835/bcm2835_mbox.c is an FDT simplebus driver gated
 * in files.arm64 on soc_brcm_bcm2837/2838, which neither RPI5 kernel carries,
 * and it computes a 0xC0000000-style VideoCore alias for the buffer through
 * bcm283x_dmabus_peripheral_lowaddr(), which has no BCM2712 entry.  On an
 * ACPI boot it would not attach at all.
 *
 * ADDRESSING, MEASURED
 *
 * mailbox@7c013880, "brcm,bcm2835-mbox", reg = <0x7c013880 0x40> under
 * /soc@107c000000, whose ranges put it at CPU physical 0x107c013880.  The
 * property buffer is handed to the VPU as a plain physical address: the
 * firmware node carries an empty dma-ranges, and loader/mboxtest.bin
 * confirmed it on this board (board revision and serial matched the
 * firmware's own log).  The message is a 32-bit address with the channel
 * in its low four bits, but the VPU only processes buffers below 1 GB --
 * measured on both lanes; see the allocation in rpi_fw_attach().
 *
 * Found through the device tree like the other drivers in this directory,
 * and attached to nexus for the same reason (see bcm2712_fdt.h): OFW is
 * initialised before the bus method is chosen, so this works on the ACPI
 * lane as well as the FDT one.
 *
 * THE RTC
 *
 * The Pi 5's battery-backed RTC is in the PMIC, and only the VPU talks to
 * the PMIC.  Vendor Linux reaches it through this same mailbox
 * (drivers/rtc/rtc-rpi.c, "raspberrypi,rpi-rtc"): GET_RTC_REG and
 * SET_RTC_REG, register 0 being seconds since the epoch.  So when the device
 * tree has that node and the firmware answers, this driver registers itself
 * as a clock(9) device, and the kernel has a time of day before the network
 * is up.  As rtc-rpi.c does at probe, it also sets the backup battery's
 * trickle-charge voltage to the node's trickle-charge-microvolt (0, which
 * disables charging, when absent).  No alarm support.
 *
 * CONCURRENCY
 *
 * The mailbox is shared with anything else that talks to the VPU.  On the
 * ACPI lane that may include EDK2 runtime services called by the kernel.
 * This driver only touches the mailbox when a sysctl is read or written, or
 * at shutdown with the flag armed, and it takes only replies on its own
 * channel.  A collision is possible in principle and has not been observed.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/reboot.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/atomic.h>
#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/broadcom/bcm2712/bcm2712_fdt.h>

#include "clock_if.h"

#define	RPI_FW_MBOX_PHYS	0x107c013880UL	/* fallback only */
#define	RPI_FW_MBOX_SIZE	0x40

#define	MBOX_REG_READ		0x00	/* mailbox 0: VPU -> ARM */
#define	MBOX_REG_STATUS		0x18
#define	  MBOX_STATUS_FULL	0x80000000U
#define	  MBOX_STATUS_EMPTY	0x40000000U
#define	MBOX_REG_WRITE		0x20	/* mailbox 1: ARM -> VPU */

#define	MBOX_CHAN_PROPERTY	8
#define	MBOX_MSG(chan, data)	(((data) & ~0xfU) | ((chan) & 0xfU))

#define	MBOX_TIMEOUT_US		1000000

#define	RESP_SUCCESS		0x80000000U

#define	TAG_GET_FIRMWARE_REVISION	0x00000001U
#define	TAG_GET_TEMPERATURE		0x00030006U
#define	TAG_GET_THROTTLED		0x00030046U
#define	TAG_NOTIFY_REBOOT		0x00030048U
#define	TAG_GET_REBOOT_FLAGS		0x00030064U
#define	TAG_SET_REBOOT_FLAGS		0x00038064U
#define	TAG_GET_RTC_REG			0x00030087U
#define	TAG_SET_RTC_REG			0x00038087U

/* RTC registers, as rtc-rpi.c numbers them. */
#define	RTC_REG_TIME			0	/* seconds since the epoch */
#define	RTC_REG_BBAT_CHG_VOLTS		4	/* trickle charge, microvolts */

#define	REBOOT_FLAG_TRYBOOT		0x1U

#define	RPI_FW_BUF_WORDS	32

static const char * const rpi_fw_fdt_paths[] = {
	"/soc@107c000000/mailbox@7c013880",
	"/soc/mailbox@7c013880",
	NULL
};

struct rpi_fw_softc {
	device_t		sc_dev;
	struct mtx		sc_mtx;
	volatile uint32_t	*sc_regs;	/* mailbox registers */
	uint32_t		*sc_buf;	/* property buffer, uncached */
	vm_paddr_t		sc_buf_pa;
	bus_addr_t		sc_regs_pa;
	bus_size_t		sc_regs_size;
	eventhandler_tag	sc_shutdown_tag;
	bool			sc_tryboot_armed;
	bool			sc_rtc;		/* registered with clock(9) */
	uint32_t		sc_last_code;	/* buf[1] of the last reply */
};

static inline uint32_t
mbox_read(struct rpi_fw_softc *sc, bus_size_t off)
{
	return (sc->sc_regs[off / 4]);
}

static inline void
mbox_write(struct rpi_fw_softc *sc, bus_size_t off, uint32_t v)
{
	sc->sc_regs[off / 4] = v;
}

/* Wait until (STATUS & mask) == want. */
static int
mbox_wait(struct rpi_fw_softc *sc, uint32_t mask, uint32_t want)
{
	int us;

	for (us = 0; us < MBOX_TIMEOUT_US; us++) {
		if ((mbox_read(sc, MBOX_REG_STATUS) & mask) == want)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

/*
 * One tag, one transaction.  inlen bytes of val are sent; vallen bytes of
 * reply are copied back.  Same buffer layout as the loader and
 * tools/vcio_test.c.  Caller holds sc_mtx.
 *
 * The buffer is mapped uncached, so the VPU sees the ARM's writes and the ARM
 * sees the VPU's reply without cache maintenance; the dsb orders the buffer
 * writes before the doorbell and the reply read after it.
 */
static int
rpi_fw_tag_locked(struct rpi_fw_softc *sc, uint32_t tag, uint32_t *val,
    uint32_t vallen, uint32_t inlen)
{
	uint32_t *buf = sc->sc_buf;
	uint32_t i, n, words, msg;
	int us;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	words = roundup2(vallen, 4) / 4;
	if (words > RPI_FW_BUF_WORDS - 6)
		return (EINVAL);

	n = 0;
	buf[n++] = 0;			/* total size, below */
	buf[n++] = 0;			/* process request */
	buf[n++] = tag;
	buf[n++] = words * 4;		/* value buffer size */
	buf[n++] = inlen;		/* request length */
	for (i = 0; i < words; i++)
		buf[n++] = (val != NULL && i * 4 < inlen) ? val[i] : 0;
	buf[n++] = 0;			/* end tag */
	buf[0] = n * 4;

	/* Discard anything stale in the VPU -> ARM mailbox. */
	for (us = 0; us < MBOX_TIMEOUT_US; us++) {
		if ((mbox_read(sc, MBOX_REG_STATUS) & MBOX_STATUS_EMPTY) != 0)
			break;
		(void)mbox_read(sc, MBOX_REG_READ);
	}

	if (mbox_wait(sc, MBOX_STATUS_FULL, 0) != 0)
		return (ETIMEDOUT);
	dsb(sy);
	mbox_write(sc, MBOX_REG_WRITE,
	    MBOX_MSG(MBOX_CHAN_PROPERTY, (uint32_t)sc->sc_buf_pa));

	for (;;) {
		if (mbox_wait(sc, MBOX_STATUS_EMPTY, 0) != 0)
			return (ETIMEDOUT);
		msg = mbox_read(sc, MBOX_REG_READ);
		if ((msg & 0xf) == MBOX_CHAN_PROPERTY)
			break;
	}
	dsb(sy);

	sc->sc_last_code = buf[1];
	if (buf[1] != RESP_SUCCESS)
		return (EIO);
	if ((buf[4] & RESP_SUCCESS) == 0)
		return (EOPNOTSUPP);
	for (i = 0; val != NULL && i < words; i++)
		val[i] = buf[5 + i];
	return (0);
}

static int
rpi_fw_tag(struct rpi_fw_softc *sc, uint32_t tag, uint32_t *val,
    uint32_t vallen, uint32_t inlen)
{
	int error;

	mtx_lock(&sc->sc_mtx);
	error = rpi_fw_tag_locked(sc, tag, val, vallen, inlen);
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

/*
 * hw.rpi_fw.tryboot: reads the flag that will apply to the next boot; writing
 * 1 arms TryBoot, 0 disarms it.  The flag is read back after every write, and
 * a write that did not stick is an error, not a silent success.
 */
static int
rpi_fw_sysctl_tryboot(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_softc *sc = arg1;
	uint32_t flags;
	int error, val;

	if ((error = rpi_fw_tag(sc, TAG_GET_REBOOT_FLAGS, &flags, 4, 0)) != 0)
		return (error);
	val = (flags & REBOOT_FLAG_TRYBOOT) != 0;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 0 && val != 1)
		return (EINVAL);

	mtx_lock(&sc->sc_mtx);
	flags = val ? REBOOT_FLAG_TRYBOOT : 0;
	error = rpi_fw_tag_locked(sc, TAG_SET_REBOOT_FLAGS, &flags, 4, 4);
	if (error == 0)
		error = rpi_fw_tag_locked(sc, TAG_GET_REBOOT_FLAGS, &flags, 4,
		    0);
	if (error == 0) {
		sc->sc_tryboot_armed = (flags & REBOOT_FLAG_TRYBOOT) != 0;
		if (sc->sc_tryboot_armed != (val != 0))
			error = EIO;
	}
	mtx_unlock(&sc->sc_mtx);

	if (error == 0)
		device_printf(sc->sc_dev, "TryBoot %s: the next boot reads %s\n",
		    val ? "armed" : "cleared",
		    val ? "tryboot.txt, once" : "config.txt");
	else
		device_printf(sc->sc_dev, "setting TryBoot failed: %d\n",
		    error);
	return (error);
}

static int
rpi_fw_sysctl_reboot_flags(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_softc *sc = arg1;
	uint32_t v;
	int error;

	if ((error = rpi_fw_tag(sc, TAG_GET_REBOOT_FLAGS, &v, 4, 0)) != 0)
		return (error);
	return (sysctl_handle_32(oidp, &v, 0, req));
}

static int
rpi_fw_sysctl_revision(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_softc *sc = arg1;
	uint32_t v;
	int error;

	if ((error = rpi_fw_tag(sc, TAG_GET_FIRMWARE_REVISION, &v, 4, 0)) != 0)
		return (error);
	return (sysctl_handle_32(oidp, &v, 0, req));
}

/* The firmware's own reading of the SoC sensor, in millidegrees C. */
/*
 * Throttling and under-voltage flags, the value `vcgencmd get_throttled`
 * prints.  Linux raspberrypi-hwmon asks with 0xffff, which also clears the
 * sticky "has occurred" bits (16 and up, bit 16 being under-voltage); this
 * asks with 0 so that reading does not erase the history.
 */
static int
rpi_fw_sysctl_throttled(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_softc *sc = arg1;
	uint32_t v = 0;
	int error;

	if ((error = rpi_fw_tag(sc, TAG_GET_THROTTLED, &v, 4, 4)) != 0)
		return (error);
	return (sysctl_handle_32(oidp, &v, 0, req));
}

static int
rpi_fw_sysctl_temperature(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_softc *sc = arg1;
	uint32_t v[2] = { 0, 0 };	/* request: sensor id 0 */
	int error, t;

	if ((error = rpi_fw_tag(sc, TAG_GET_TEMPERATURE, v, 8, 4)) != 0)
		return (error);
	t = (int)v[1];
	return (sysctl_handle_int(oidp, &t, 0, req));
}

/*
 * As rpi_firmware_notify_reboot() does, tell the VPU a reboot is coming --
 * but only when TryBoot is armed and this is a reboot, so ordinary reboots
 * and power-offs are exactly what they were.  Runs before the PSCI reset
 * handler (SHUTDOWN_PRI_FIRST), and never after a panic: the mailbox is not
 * worth risking a hang on the way to a crash dump.
 */
static void
rpi_fw_shutdown_final(void *arg, int howto)
{
	struct rpi_fw_softc *sc = arg;
	int error;

	if (!sc->sc_tryboot_armed || KERNEL_PANICKED() ||
	    (howto & (RB_HALT | RB_POWEROFF)) != 0)
		return;
	if (!mtx_trylock(&sc->sc_mtx))
		return;
	error = rpi_fw_tag_locked(sc, TAG_NOTIFY_REBOOT, NULL, 0, 0);
	mtx_unlock(&sc->sc_mtx);
	printf("rpi_fw: TryBoot armed; this reboot reads tryboot.txt%s\n",
	    error == 0 ? "" : " (NOTIFY_REBOOT failed)");
}

/*
 * clock(9): the PMIC RTC through the firmware.  A request carries the
 * register number and a value, and the reply the register number and its
 * value, as in rtc-rpi.c.
 */
static int
rpi_fw_gettime(device_t dev, struct timespec *ts)
{
	struct rpi_fw_softc *sc = device_get_softc(dev);
	uint32_t v[2] = { RTC_REG_TIME, 0 };
	int error;

	if ((error = rpi_fw_tag(sc, TAG_GET_RTC_REG, v, 8, 8)) != 0)
		return (error);
	ts->tv_sec = v[1];
	ts->tv_nsec = 0;
	return (0);
}

static int
rpi_fw_settime(device_t dev, struct timespec *ts)
{
	struct rpi_fw_softc *sc = device_get_softc(dev);
	uint32_t v[2] = { RTC_REG_TIME, (uint32_t)ts->tv_sec };

	return (rpi_fw_tag(sc, TAG_SET_RTC_REG, v, 8, 8));
}

static void
rpi_fw_rtc_attach(struct rpi_fw_softc *sc)
{
	phandle_t node;
	pcell_t uv;
	uint32_t v[2];
	int error;

	node = ofw_bus_find_compatible(OF_finddevice("/"),
	    "raspberrypi,rpi-rtc");
	if (node <= 0)
		return;

	v[0] = RTC_REG_TIME;
	v[1] = 0;
	if ((error = rpi_fw_tag(sc, TAG_GET_RTC_REG, v, 8, 8)) != 0) {
		device_printf(sc->sc_dev, "RTC: firmware did not answer (%d)\n",
		    error);
		return;
	}

	uv = 0;
	(void)OF_getencprop(node, "trickle-charge-microvolt", &uv, sizeof(uv));
	v[0] = RTC_REG_BBAT_CHG_VOLTS;
	v[1] = uv;
	error = rpi_fw_tag(sc, TAG_SET_RTC_REG, v, 8, 8);
	if (error != 0)
		device_printf(sc->sc_dev,
		    "RTC: setting trickle charge to %u uV failed (%d)\n", uv,
		    error);

	clock_register(sc->sc_dev, 1000000);
	sc->sc_rtc = true;
	device_printf(sc->sc_dev, "RTC in the PMIC registered%s\n",
	    uv != 0 ? ", battery trickle charging on" : "");
}

static phandle_t
rpi_fw_find_node(void)
{
	return (bcm2712_fdt_find(rpi_fw_fdt_paths, "brcm,bcm2835-mbox"));
}

static void
rpi_fw_identify(driver_t *driver, device_t parent)
{
	if (rpi_fw_find_node() == -1)
		return;
	if (device_find_child(parent, "rpi_fw", -1) != NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "rpi_fw", -1) == NULL)
		device_printf(parent, "rpi_fw: BUS_ADD_CHILD failed\n");
}

static int
rpi_fw_probe(device_t dev)
{
	device_set_desc(dev, "Raspberry Pi VPU firmware mailbox");
	return (BUS_PROBE_DEFAULT);
}

static int rpi_fw_detach(device_t dev);

static int
rpi_fw_attach(device_t dev)
{
	struct rpi_fw_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	bus_addr_t pa = RPI_FW_MBOX_PHYS;
	bus_size_t sz = RPI_FW_MBOX_SIZE;
	uint32_t rev, flags;
	bool from_fdt;
	int error;

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "rpi_fw", NULL, MTX_DEF);

	from_fdt = bcm2712_fdt_reg(rpi_fw_fdt_paths, "brcm,bcm2835-mbox", 0,
	    &pa, &sz);
	if (sz < RPI_FW_MBOX_SIZE)
		sz = RPI_FW_MBOX_SIZE;
	sc->sc_regs_pa = pa;
	sc->sc_regs_size = sz;
	sc->sc_regs = pmap_mapdev_attr(pa, sz, VM_MEMATTR_DEVICE);

	/*
	 * One page, below 1 GB, uncached.  Uncached is what makes it safe to
	 * share with the VPU without cache maintenance.
	 *
	 * Below 1 GB is measured, not documented.  Every buffer the VPU has
	 * processed was under 0x40000000 -- 0x2f282000, 0x3cc000, 0x3b0d3000
	 * and 0x4cd000 on the ACPI lane, and the loader's near 0x200000 --
	 * while the one allocated at 0x40233000 on an FDT boot was answered
	 * on our channel with buf[1] still 0: the VPU never read it.  So the
	 * device tree's identity dma-ranges does not mean the VPU can reach
	 * all of it.  The 32-bit mailbox message alone would allow 4 GB.
	 */
	sc->sc_buf = kmem_alloc_contig(PAGE_SIZE, M_WAITOK | M_ZERO, 0,
	    0x3fffffffUL, PAGE_SIZE, 0, VM_MEMATTR_UNCACHEABLE);
	if (sc->sc_buf == NULL) {
		device_printf(dev, "cannot allocate the property buffer\n");
		rpi_fw_detach(dev);
		return (ENOMEM);
	}
	sc->sc_buf_pa = pmap_kextract((vm_offset_t)sc->sc_buf);

	/* Prove the channel before exposing anything that depends on it. */
	error = rpi_fw_tag(sc, TAG_GET_FIRMWARE_REVISION, &rev, 4, 0);
	if (error == 0)
		error = rpi_fw_tag(sc, TAG_GET_REBOOT_FLAGS, &flags, 4, 0);
	if (error != 0) {
		/*
		 * Say where the buffer was and what the VPU wrote back.  EIO
		 * means the VPU replied on our channel but left buf[1] other
		 * than 0x80000000: it answered without processing the buffer
		 * it was given, which is the address question, not a dead
		 * channel.  Seen on the FDT lane on 2026-09-27, where the
		 * same code works on the ACPI lane.
		 */
		device_printf(dev, "mailbox at 0x%jx did not answer (%d): "
		    "buffer PA 0x%jx, response code 0x%08x\n", (uintmax_t)pa,
		    error, (uintmax_t)sc->sc_buf_pa, sc->sc_last_code);
		rpi_fw_detach(dev);
		return (ENXIO);
	}
	sc->sc_tryboot_armed = (flags & REBOOT_FLAG_TRYBOOT) != 0;

	device_printf(dev, "mailbox at 0x%jx (%s), buffer at PA 0x%jx, "
	    "firmware 0x%08x, reboot flags 0x%08x\n", (uintmax_t)pa,
	    from_fdt ? "from FDT" : "hardcoded", (uintmax_t)sc->sc_buf_pa,
	    rev, flags);

	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_ADD_NODE(ctx, SYSCTL_STATIC_CHILDREN(_hw), OID_AUTO,
	    "rpi_fw", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Raspberry Pi VPU firmware");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "tryboot",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    rpi_fw_sysctl_tryboot, "I",
	    "1 = next boot reads tryboot.txt, once (write 1 to arm, 0 to clear)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "reboot_flags",
	    CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rpi_fw_sysctl_reboot_flags, "IU",
	    "Raw reboot flags the firmware will apply to the next boot");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "firmware_revision", CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, rpi_fw_sysctl_revision, "IU",
	    "VPU firmware revision (build timestamp)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "temperature",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rpi_fw_sysctl_temperature, "I",
	    "SoC temperature as the firmware reads it, millidegrees C");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "throttled",
	    CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rpi_fw_sysctl_throttled, "IU",
	    "Throttling flags as vcgencmd get_throttled reports them "
	    "(bit 16: under-voltage has occurred)");

	rpi_fw_rtc_attach(sc);

	sc->sc_shutdown_tag = EVENTHANDLER_REGISTER(shutdown_final,
	    rpi_fw_shutdown_final, sc, SHUTDOWN_PRI_FIRST);
	return (0);
}

static int
rpi_fw_detach(device_t dev)
{
	struct rpi_fw_softc *sc = device_get_softc(dev);

	if (sc->sc_rtc) {
		clock_unregister(dev);
		sc->sc_rtc = false;
	}
	if (sc->sc_shutdown_tag != NULL) {
		EVENTHANDLER_DEREGISTER(shutdown_final, sc->sc_shutdown_tag);
		sc->sc_shutdown_tag = NULL;
	}
	/* The sysctl tree is on the device context and goes with it. */
	if (sc->sc_buf != NULL) {
		kmem_free(sc->sc_buf, PAGE_SIZE);
		sc->sc_buf = NULL;
	}
	if (sc->sc_regs != NULL) {
		pmap_unmapdev(__DEVOLATILE(void *, sc->sc_regs),
		    sc->sc_regs_size);
		sc->sc_regs = NULL;
	}
	if (mtx_initialized(&sc->sc_mtx))
		mtx_destroy(&sc->sc_mtx);
	return (0);
}

static device_method_t rpi_fw_methods[] = {
	DEVMETHOD(device_identify,	rpi_fw_identify),
	DEVMETHOD(device_probe,		rpi_fw_probe),
	DEVMETHOD(device_attach,	rpi_fw_attach),
	DEVMETHOD(device_detach,	rpi_fw_detach),

	DEVMETHOD(clock_gettime,	rpi_fw_gettime),
	DEVMETHOD(clock_settime,	rpi_fw_settime),
	DEVMETHOD_END
};

static driver_t rpi_fw_driver = {
	"rpi_fw",
	rpi_fw_methods,
	sizeof(struct rpi_fw_softc),
};

DRIVER_MODULE(rpi_fw, nexus, rpi_fw_driver, 0, 0);
MODULE_VERSION(rpi_fw, 1);
