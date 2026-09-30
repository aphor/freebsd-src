/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Contributors
 * All rights reserved.
 *
 * bcm2712_sdhci — BCM2712 SD/SDIO host controller, device tree attachment.
 *
 * The Pi 5 has two of these, both native BCM2712 peripherals rather than
 * anything behind RP1:
 *
 *   mmc@fff000    the microSD slot
 *   mmc@1100000   the SDIO bus the CYW43455 radio sits on
 *
 * Both declare compatible = "brcm,bcm2712-sdhci", "brcm,sdhci-brcmstb", and
 * before this driver nothing in FreeBSD matched either string: sdhci_fdt.c
 * covers marvell/qcom/xlnx, and bcm2835_sdhci.c only bcm2835 and
 * bcm2711-emmc2.  That made the SD card unreachable on an FDT boot, which is
 * what made an FDT-only kernel unable to find a root filesystem at all.
 *
 * The controller needs remarkably little.  Under ACPI the *generic*
 * sdhci_acpi driver drives this same hardware with no Broadcom-specific code,
 * announcing itself as an "Intel Bay Trail/Braswell SDXC Controller", and its
 * capability register reports everything sdhci(4) needs:
 *
 *	sdhci_acpi0-slot0: 200MHz 4bits VDD: VCCQ: 3.3V 1.8V DRV: BACD DMA removable
 *
 * so the base clock comes from SDHCI_CAPABILITIES and the device tree's
 * clocks/clock-names (clk-emmc2, a 200 MHz fixed-clock) do not have to be
 * plumbed through the clock framework -- which matters, because on this board
 * clk_fixed(4) cannot attach to the vendor DTB's fixed-clock nodes at all
 * ("Cannot FDT parameters", attach returned 6).
 *
 * Quirks are the two the working ACPI attachment applies to this controller,
 * SDHCI_QUIRK_WAIT_WHILE_BUSY | SDHCI_QUIRK_PRESET_VALUE_BROKEN.  Linux's
 * sdhci-brcmstb.c independently sets SDHCI_QUIRK2_PRESET_VALUE_BROKEN for its
 * bcm2712 match, so the second one is corroborated from two directions.
 *
 * ToDo, in rough order of how much they are likely to matter:
 *
 *  - The "cfg" register window (reg[1], 0x200 bytes at +0x400) is mapped but
 *    otherwise unused.  On brcmstb parts it carries the capability override
 *    and pin-select registers; the device tree offers sdhci-caps and
 *    sdhci-caps-mask on mmc@1100000, which a full driver would apply through
 *    it.  Nothing needs them while the CAPS register already reads correctly.
 *  - cd-gpios is ignored, so card detect comes from the controller's own
 *    present-state bit.  The pin is on the always-on BCM2712 controller
 *    (gio_aon, line 5), which brcmstb_gpio(4) now drives; it is simply not
 *    wired up here.
 *  - vqmmc-supply is not consumed, so there is no 1.8 V switching for UHS.
 *  - sd-uhs-sdr50/ddr50/sdr104, mmc-ddr-3_3v and supports-cqe are not
 *    consumed.  The controller is left in the speed modes sdhci(4) derives
 *    from CAPS, which is how the ACPI path runs today.
 *
 * POWER AND PINS
 *
 * pinctrl-0 is applied and vmmc-supply is enabled before the slot starts
 * looking for a card.  An earlier version of this file ignored both, on the
 * assumption that the firmware had muxed the pins and that the supplies were
 * always-on.  That holds for the microSD slot and is false for the SDIO slot.
 * The loader's peek, before any kernel ran, found gpio30..35 still muxed as
 * plain GPIO, and WL_ON -- the GPIO behind wl-on-reg, mmc@1100000's
 * vmmc-supply -- an input reading 0.  So the WiFi chip had no power and no
 * bus.  Both are now done here, as Linux does in its mmc core.
 *
 * Holding vmmc-supply matters for the microSD slot too, for a different
 * reason.  Its supply, sd-vcc-reg, is regulator-boot-on but not always-on.
 * Once brcmstb_gpio(4) gives it a GPIO it registers, and
 * regulator_shutdown() at SI_SUB_LAST -- before root is mounted -- turns off
 * every enabled regulator that is not always-on and has no users.  Unheld,
 * that would cut power to the card holding the root filesystem.
 *
 * References:
 *   sys/dev/sdhci/sdhci_acpi.c  the working attachment for this same hardware
 *   ../raspbian_linux.git/drivers/mmc/host/sdhci-brcmstb.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/regulator/regulator.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>

#include <dev/sdhci/sdhci.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

#define	BCM2712_SDHCI_QUIRKS						\
	(SDHCI_QUIRK_WAIT_WHILE_BUSY | SDHCI_QUIRK_PRESET_VALUE_BROKEN)

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2712-sdhci",		1 },
	{ "brcm,sdhci-brcmstb",		1 },
	{ NULL,				0 }
};

struct bcm2712_sdhci_softc {
	struct sdhci_slot	slot;
	struct resource		*mem_res;	/* reg[0], "host" */
	struct resource		*cfg_res;	/* reg[1], "cfg" -- see ToDo */
	struct resource		*irq_res;
	void			*intrhand;
	regulator_t		vmmc;		/* held while attached */
};

static void bcm2712_sdhci_intr(void *arg);
static int bcm2712_sdhci_detach(device_t dev);

/*
 * Register accessors.  The barriers mirror sdhci_acpi.c: this controller is
 * sensitive to read/write ordering around its 8- and 16-bit registers.
 */
static uint8_t
bcm2712_sdhci_read_1(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	return (bus_read_1(sc->mem_res, off));
}

static void
bcm2712_sdhci_write_1(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off, uint8_t val)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	bus_write_1(sc->mem_res, off, val);
}

static uint16_t
bcm2712_sdhci_read_2(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	return (bus_read_2(sc->mem_res, off));
}

static void
bcm2712_sdhci_write_2(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off, uint16_t val)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	bus_write_2(sc->mem_res, off, val);
}

static uint32_t
bcm2712_sdhci_read_4(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	return (bus_read_4(sc->mem_res, off));
}

static void
bcm2712_sdhci_write_4(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off, uint32_t val)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_barrier(sc->mem_res, 0, 0xFF,
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);
	bus_write_4(sc->mem_res, off, val);
}

static void
bcm2712_sdhci_read_multi_4(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_read_multi_4(sc->mem_res, off, data, count);
}

static void
bcm2712_sdhci_write_multi_4(device_t dev, struct sdhci_slot *slot __unused,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	bus_write_multi_4(sc->mem_res, off, data, count);
}

static int
bcm2712_sdhci_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "BCM2712 SD/SDIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm2712_sdhci_attach(device_t dev)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);
	int rid, err;

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		return (ENOMEM);
	}

	rid = 0;		/* reg[0], "host" */
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate the host register window\n");
		bcm2712_sdhci_detach(dev);
		return (ENOMEM);
	}

	/*
	 * reg[1] is the brcmstb "cfg" window.  Mapped so that a later driver
	 * can apply sdhci-caps/sdhci-caps-mask through it without changing the
	 * device tree binding, and so that nothing else claims it meanwhile.
	 * Not required: absent is only a notice, not a failure.
	 */
	rid = 1;
	sc->cfg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->cfg_res == NULL && bootverbose)
		device_printf(dev, "no cfg register window\n");

	/*
	 * Pins, then power, before sdhci_start_slot() looks for a card: see
	 * POWER AND PINS in the file comment.  Neither is fatal when absent,
	 * because the microSD slot works without both.  A supply that exists
	 * but cannot be enabled is reported.
	 */
	err = fdt_pinctrl_configure_by_name(dev, "default");
	if (err != 0 && err != ENOENT)
		device_printf(dev, "pinctrl-0 not applied: %d\n", err);

	err = regulator_get_by_ofw_property(dev, 0, "vmmc-supply", &sc->vmmc);
	if (err == 0) {
		err = regulator_enable(sc->vmmc);
		if (err != 0) {
			device_printf(dev, "cannot enable vmmc-supply: %d\n",
			    err);
			regulator_release(sc->vmmc);
			sc->vmmc = NULL;
		} else
			device_printf(dev, "vmmc-supply enabled\n");
	} else if (err != ENOENT)
		device_printf(dev, "vmmc-supply unavailable: %d\n", err);

	sc->slot.quirks = BCM2712_SDHCI_QUIRKS;

	/*
	 * max_clk is left at zero on purpose: sdhci_init_slot() takes the base
	 * clock from SDHCI_CAPABILITIES, which this controller fills in
	 * correctly (200 MHz).  See the file comment.
	 */
	err = sdhci_init_slot(dev, &sc->slot, 0);
	if (err != 0) {
		device_printf(dev, "cannot initialise the slot\n");
		bcm2712_sdhci_detach(dev);
		return (err);
	}

	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, bcm2712_sdhci_intr, sc, &sc->intrhand);
	if (err != 0) {
		device_printf(dev, "cannot set up the interrupt\n");
		bcm2712_sdhci_detach(dev);
		return (err);
	}

	sdhci_start_slot(&sc->slot);
	return (0);
}

static int
bcm2712_sdhci_detach(device_t dev)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);

	if (sc->intrhand != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->intrhand);
		sc->intrhand = NULL;
	}
	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(sc->irq_res), sc->irq_res);
		sc->irq_res = NULL;
	}
	if (sc->cfg_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->cfg_res), sc->cfg_res);
		sc->cfg_res = NULL;
	}
	if (sc->mem_res != NULL) {
		sdhci_cleanup_slot(&sc->slot);
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
		sc->mem_res = NULL;
	}
	if (sc->vmmc != NULL) {
		regulator_disable(sc->vmmc);
		regulator_release(sc->vmmc);
		sc->vmmc = NULL;
	}
	return (0);
}

static int
bcm2712_sdhci_suspend(device_t dev)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);
	int err;

	err = bus_generic_suspend(dev);
	if (err != 0)
		return (err);
	sdhci_generic_suspend(&sc->slot);
	return (0);
}

static int
bcm2712_sdhci_resume(device_t dev)
{
	struct bcm2712_sdhci_softc *sc = device_get_softc(dev);
	int err;

	sdhci_generic_resume(&sc->slot);
	err = bus_generic_resume(dev);
	if (err != 0)
		return (err);
	return (0);
}

static void
bcm2712_sdhci_intr(void *arg)
{
	struct bcm2712_sdhci_softc *sc = arg;

	sdhci_generic_intr(&sc->slot);
}

static device_method_t bcm2712_sdhci_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		bcm2712_sdhci_probe),
	DEVMETHOD(device_attach,	bcm2712_sdhci_attach),
	DEVMETHOD(device_detach,	bcm2712_sdhci_detach),
	DEVMETHOD(device_suspend,	bcm2712_sdhci_suspend),
	DEVMETHOD(device_resume,	bcm2712_sdhci_resume),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,	sdhci_generic_write_ivar),
	DEVMETHOD(bus_add_child,	bus_generic_add_child),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_update_ios,	sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_switch_vccq,	sdhci_generic_switch_vccq),
	DEVMETHOD(mmcbr_tune,		sdhci_generic_tune),
	DEVMETHOD(mmcbr_retune,		sdhci_generic_retune),
	DEVMETHOD(mmcbr_request,	sdhci_generic_request),
	DEVMETHOD(mmcbr_get_ro,		sdhci_generic_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host,	sdhci_generic_release_host),

	/* SDHCI accessors */
	DEVMETHOD(sdhci_read_1,		bcm2712_sdhci_read_1),
	DEVMETHOD(sdhci_read_2,		bcm2712_sdhci_read_2),
	DEVMETHOD(sdhci_read_4,		bcm2712_sdhci_read_4),
	DEVMETHOD(sdhci_read_multi_4,	bcm2712_sdhci_read_multi_4),
	DEVMETHOD(sdhci_write_1,	bcm2712_sdhci_write_1),
	DEVMETHOD(sdhci_write_2,	bcm2712_sdhci_write_2),
	DEVMETHOD(sdhci_write_4,	bcm2712_sdhci_write_4),
	DEVMETHOD(sdhci_write_multi_4,	bcm2712_sdhci_write_multi_4),
	DEVMETHOD(sdhci_set_uhs_timing,	sdhci_generic_set_uhs_timing),

	DEVMETHOD_END
};

static driver_t bcm2712_sdhci_driver = {
	"sdhci_bcm2712",
	bcm2712_sdhci_methods,
	sizeof(struct bcm2712_sdhci_softc),
};

DRIVER_MODULE(sdhci_bcm2712, simplebus, bcm2712_sdhci_driver, NULL, NULL);
SDHCI_DEPEND(sdhci_bcm2712);

#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_bcm2712);
#endif
