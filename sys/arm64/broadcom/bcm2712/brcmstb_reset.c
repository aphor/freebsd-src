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
 * brcmstb_reset -- the Broadcom STB "SW_INIT" reset controller
 * ("brcm,brcmstb-reset"), as an hwreset(9) provider.
 *
 * The block is a run of banks, 0x18 bytes apart, of three registers each:
 * SW_INIT_SET (write 1 to assert), SW_INIT_CLEAR (write 1 to deassert) and
 * SW_INIT_STATUS (1 = asserted).  Reset id n is bit n % 32 of bank n / 32.
 * Layout from Linux drivers/reset/reset-brcmstb.c.
 *
 * On the Raspberry Pi 5 it holds the PCIe bridge resets: ids 42, 43 and 44
 * for pcie0, pcie1 and pcie2 (bcm2712.dtsi).
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

#define	SW_INIT_SET		0x00
#define	SW_INIT_CLEAR		0x04
#define	SW_INIT_STATUS		0x08
#define	SW_INIT_BANK_SIZE	0x18
#define	SW_INIT_BANK(id)	((id) >> 5)
#define	SW_INIT_BIT(id)		(1u << ((id) & 0x1f))

/*
 * Maximum delay between deasserting a line and the block operating is
 * typically 14 us (reset-brcmstb.c); wait longer.
 */
#define	SW_INIT_DEASSERT_US	100

struct brcmstb_reset_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;
	intptr_t		nresets;
};

static struct ofw_compat_data compat_data[] = {
	{ "brcm,brcmstb-reset",		1 },
	{ NULL,				0 }
};

static int
brcmstb_reset_assert(device_t dev, intptr_t id, bool reset)
{
	struct brcmstb_reset_softc *sc;
	bus_size_t off;

	sc = device_get_softc(dev);
	if (id < 0 || id >= sc->nresets)
		return (EINVAL);

	off = SW_INIT_BANK(id) * SW_INIT_BANK_SIZE +
	    (reset ? SW_INIT_SET : SW_INIT_CLEAR);
	mtx_lock(&sc->mtx);
	bus_write_4(sc->res, off, SW_INIT_BIT(id));
	mtx_unlock(&sc->mtx);
	if (!reset)
		DELAY(SW_INIT_DEASSERT_US);

	return (0);
}

static int
brcmstb_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct brcmstb_reset_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	if (id < 0 || id >= sc->nresets)
		return (EINVAL);

	mtx_lock(&sc->mtx);
	val = bus_read_4(sc->res,
	    SW_INIT_BANK(id) * SW_INIT_BANK_SIZE + SW_INIT_STATUS);
	mtx_unlock(&sc->mtx);
	*reset = (val & SW_INIT_BIT(id)) != 0;

	return (0);
}

static int
brcmstb_reset_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Broadcom STB SW_INIT reset controller");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_reset_attach(device_t dev)
{
	struct brcmstb_reset_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	/* Whole banks only, as reset-brcmstb.c counts them. */
	sc->nresets = (rman_get_size(sc->res) / SW_INIT_BANK_SIZE) * 32;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	hwreset_register_ofw_provider(dev);
	return (0);
}

static device_method_t brcmstb_reset_methods[] = {
	DEVMETHOD(device_probe,		brcmstb_reset_probe),
	DEVMETHOD(device_attach,	brcmstb_reset_attach),

	DEVMETHOD(hwreset_assert,	brcmstb_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	brcmstb_reset_is_asserted),

	DEVMETHOD_END
};

static driver_t brcmstb_reset_driver = {
	"brcmstb_reset",
	brcmstb_reset_methods,
	sizeof(struct brcmstb_reset_softc),
};

EARLY_DRIVER_MODULE(brcmstb_reset, simplebus, brcmstb_reset_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(brcmstb_reset, 1);
