/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * brcmstb_rescal -- the Broadcom STB SATA/PCIe resistor calibration block
 * ("brcm,bcm7216-pcie-sata-rescal"), as an hwreset(9) provider.
 *
 * It is not a reset line: "deasserting" it runs the calibration, and
 * asserting it does nothing.  Calibration sets START, waits for STATUS,
 * and clears START again.  Behaviour from Linux
 * drivers/reset/reset-brcmstb-rescal.c.
 *
 * On the Raspberry Pi 5 all three PCIe controllers name it as their
 * "rescal" reset (#reset-cells = <0>), and Linux gets it as a shared reset,
 * which reset_control_reset() runs once however many consumers there are.
 * hwreset(9) has no shared resets, so this provider calibrates on the first
 * deassert only.
 *
 * Attach reads the registers and logs them as found: whether the VPU
 * firmware has already calibrated is not otherwise known.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/hwreset/hwreset.h>

#include "hwreset_if.h"

#define	RESCAL_START		0x0
#define	RESCAL_START_BIT	(1u << 0)
#define	RESCAL_CTRL		0x4
#define	RESCAL_STATUS		0x8
#define	RESCAL_STATUS_BIT	(1u << 0)

/* reset-brcmstb-rescal.c polls every 100 us for up to 1 ms. */
#define	RESCAL_POLL_US		100
#define	RESCAL_TIMEOUT_US	1000

struct brcmstb_rescal_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;
	bool			calibrated;
};

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm7216-pcie-sata-rescal",	1 },
	{ NULL,					0 }
};

static int
brcmstb_rescal_calibrate(struct brcmstb_rescal_softc *sc)
{
	uint32_t reg;
	int us;

	reg = bus_read_4(sc->res, RESCAL_START);
	bus_write_4(sc->res, RESCAL_START, reg | RESCAL_START_BIT);
	reg = bus_read_4(sc->res, RESCAL_START);
	if ((reg & RESCAL_START_BIT) == 0) {
		device_printf(sc->dev, "failed to start SATA/PCIe rescal\n");
		return (EIO);
	}

	for (us = 0; us < RESCAL_TIMEOUT_US; us += RESCAL_POLL_US) {
		if ((bus_read_4(sc->res, RESCAL_STATUS) & RESCAL_STATUS_BIT) != 0)
			break;
		DELAY(RESCAL_POLL_US);
	}
	if ((bus_read_4(sc->res, RESCAL_STATUS) & RESCAL_STATUS_BIT) == 0) {
		device_printf(sc->dev, "time out on SATA/PCIe rescal\n");
		return (ETIMEDOUT);
	}

	reg = bus_read_4(sc->res, RESCAL_START);
	bus_write_4(sc->res, RESCAL_START, reg & ~RESCAL_START_BIT);
	device_printf(sc->dev, "SATA/PCIe rescal done\n");
	return (0);
}

static int
brcmstb_rescal_assert(device_t dev, intptr_t id, bool reset)
{
	struct brcmstb_rescal_softc *sc;
	int error;

	sc = device_get_softc(dev);
	if (id != 0)
		return (EINVAL);
	if (reset)
		return (0);

	error = 0;
	mtx_lock(&sc->mtx);
	if (!sc->calibrated) {
		error = brcmstb_rescal_calibrate(sc);
		if (error == 0)
			sc->calibrated = true;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

static int
brcmstb_rescal_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Broadcom STB SATA/PCIe rescal");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_rescal_attach(device_t dev)
{
	struct brcmstb_rescal_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	device_printf(dev, "as found: START=%#x CTRL=%#x STATUS=%#x\n",
	    bus_read_4(sc->res, RESCAL_START), bus_read_4(sc->res, RESCAL_CTRL),
	    bus_read_4(sc->res, RESCAL_STATUS));

	hwreset_register_ofw_provider(dev);
	return (0);
}

static device_method_t brcmstb_rescal_methods[] = {
	DEVMETHOD(device_probe,		brcmstb_rescal_probe),
	DEVMETHOD(device_attach,	brcmstb_rescal_attach),

	DEVMETHOD(hwreset_assert,	brcmstb_rescal_assert),

	DEVMETHOD_END
};

static driver_t brcmstb_rescal_driver = {
	"brcmstb_rescal",
	brcmstb_rescal_methods,
	sizeof(struct brcmstb_rescal_softc),
};

EARLY_DRIVER_MODULE(brcmstb_rescal, simplebus, brcmstb_rescal_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(brcmstb_rescal, 1);
