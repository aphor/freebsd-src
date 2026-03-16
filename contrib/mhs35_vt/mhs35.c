/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 The FreeBSD Foundation
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
 * MHS35 SPI TFT LCD driver for Raspberry Pi.
 *
 * Drives a 3.5" 480x320 ILI9486-based SPI display, providing a vt(4)
 * compatible framebuffer console via a shadow buffer in RAM.
 */

#include <sys/cdefs.h>
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/gpio/gpiobusvar.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include <dev/vt/vt.h>
#include <dev/vt/hw/fb/vt_fb.h>
#include <dev/vt/colors/vt_termcolors.h>

#include "mhs35reg.h"
#include "mhs35var.h"

#include "spibus_if.h"

static MALLOC_DEFINE(M_MHS35, "mhs35", "MHS35 framebuffer");

static struct ofw_compat_data compat_data[] = {
	{ "mhs35",		1 },
	{ "ilitek,ili9486",	1 },
	{ NULL,			0 }
};

SPIBUS_FDT_PNP_INFO(compat_data);

/*
 * Low-level SPI helpers.
 */
static int
mhs35_spi_xfer(struct mhs35_softc *sc, bool is_data, void *buf, uint32_t len)
{
	struct spi_command cmd = SPI_COMMAND_INITIALIZER;
	uint8_t dummy;
	int error;

	gpio_pin_set_active(sc->sc_dc_pin, is_data);

	cmd.tx_cmd = buf;
	cmd.tx_cmd_sz = len;
	cmd.rx_cmd = &dummy;
	cmd.rx_cmd_sz = len;

	error = SPIBUS_TRANSFER(device_get_parent(sc->sc_dev), sc->sc_dev,
	    &cmd);
	if (error != 0)
		device_printf(sc->sc_dev, "SPI transfer failed: %d\n", error);

	return (error);
}

static int
mhs35_write_cmd(struct mhs35_softc *sc, uint8_t cmd)
{

	return (mhs35_spi_xfer(sc, false, &cmd, 1));
}

static int
mhs35_write_data(struct mhs35_softc *sc, void *data, uint32_t len)
{

	return (mhs35_spi_xfer(sc, true, data, len));
}

static int
mhs35_write_data8(struct mhs35_softc *sc, uint8_t val)
{

	return (mhs35_write_data(sc, &val, 1));
}

/*
 * Set the ILI9486 column/page address window for a subsequent RAMWR.
 */
static int
mhs35_set_window(struct mhs35_softc *sc, uint32_t x, uint32_t y,
    uint32_t w, uint32_t h)
{
	uint8_t buf[4];
	uint32_t x1, y1;
	int error;

	x1 = x + w - 1;
	y1 = y + h - 1;

	error = mhs35_write_cmd(sc, ILI9486_CASET);
	if (error != 0)
		return (error);
	buf[0] = (x >> 8) & 0xff;
	buf[1] = x & 0xff;
	buf[2] = (x1 >> 8) & 0xff;
	buf[3] = x1 & 0xff;
	error = mhs35_write_data(sc, buf, 4);
	if (error != 0)
		return (error);

	error = mhs35_write_cmd(sc, ILI9486_PASET);
	if (error != 0)
		return (error);
	buf[0] = (y >> 8) & 0xff;
	buf[1] = y & 0xff;
	buf[2] = (y1 >> 8) & 0xff;
	buf[3] = y1 & 0xff;
	error = mhs35_write_data(sc, buf, 4);
	if (error != 0)
		return (error);

	return (0);
}

/*
 * Flush a rectangular region from the shadow framebuffer to the display.
 * Called from the vt backend wrappers after vt_fb renders to the shadow buffer.
 */
void
mhs35_flush_region(struct mhs35_softc *sc, uint32_t x, uint32_t y,
    uint32_t w, uint32_t h)
{
	uint8_t *fb;
	uint32_t row;
	int error;

	if (w == 0 || h == 0)
		return;

	/* Clamp to display bounds */
	if (x + w > MHS35_WIDTH)
		w = MHS35_WIDTH - x;
	if (y + h > MHS35_HEIGHT)
		h = MHS35_HEIGHT - y;

	error = mhs35_set_window(sc, x, y, w, h);
	if (error != 0)
		return;

	error = mhs35_write_cmd(sc, ILI9486_RAMWR);
	if (error != 0)
		return;

	fb = (uint8_t *)sc->sc_fb.fb_vbase;

	/*
	 * Send pixel data row by row.  Each row segment is
	 * w * 2 bytes (RGB565, 2 bytes per pixel).
	 */
	for (row = y; row < y + h; row++) {
		error = mhs35_write_data(sc,
		    fb + row * MHS35_STRIDE + x * (MHS35_BPP / 8),
		    w * (MHS35_BPP / 8));
		if (error != 0)
			return;
	}
}

/*
 * ILI9486 initialization sequence.
 */
static int
mhs35_hw_reset(struct mhs35_softc *sc)
{

	gpio_pin_set_active(sc->sc_rst_pin, true);
	DELAY(MHS35_RESET_US);
	gpio_pin_set_active(sc->sc_rst_pin, false);
	DELAY(MHS35_RESET_US);
	gpio_pin_set_active(sc->sc_rst_pin, true);
	DELAY(MHS35_RESET_WAIT_US);

	return (0);
}

static int
mhs35_hw_init(struct mhs35_softc *sc)
{
	int error;

	/* Software reset */
	error = mhs35_write_cmd(sc, ILI9486_SWRESET);
	if (error != 0)
		return (error);
	DELAY(MHS35_RESET_WAIT_US);

	/* Sleep out */
	error = mhs35_write_cmd(sc, ILI9486_SLPOUT);
	if (error != 0)
		return (error);
	DELAY(MHS35_SLPOUT_WAIT_US);

	/* Pixel format: RGB565 */
	error = mhs35_write_cmd(sc, ILI9486_COLMOD);
	if (error != 0)
		return (error);
	error = mhs35_write_data8(sc, COLMOD_RGB565);
	if (error != 0)
		return (error);

	/* Memory access control: landscape, BGR */
	error = mhs35_write_cmd(sc, ILI9486_MADCTL);
	if (error != 0)
		return (error);
	error = mhs35_write_data8(sc, MHS35_MADCTL_LANDSCAPE);
	if (error != 0)
		return (error);

	/* Normal display mode */
	error = mhs35_write_cmd(sc, ILI9486_NORON);
	if (error != 0)
		return (error);

	/* Display on */
	error = mhs35_write_cmd(sc, ILI9486_DISPON);
	if (error != 0)
		return (error);

	return (0);
}

/*
 * Newbus methods.
 */
static int
mhs35_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MHS35 3.5\" SPI TFT LCD (ILI9486)");
	return (BUS_PROBE_DEFAULT);
}

static int
mhs35_attach(device_t dev)
{
	struct mhs35_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	node = ofw_bus_get_node(dev);

	/* Acquire GPIO pins from device tree */
	error = gpio_pin_get_by_ofw_property(dev, node, "dc-gpios",
	    &sc->sc_dc_pin);
	if (error != 0) {
		device_printf(dev, "cannot get dc-gpios: %d\n", error);
		return (error);
	}
	error = gpio_pin_setflags(sc->sc_dc_pin, GPIO_PIN_OUTPUT);
	if (error != 0) {
		device_printf(dev, "cannot set dc-gpios as output: %d\n",
		    error);
		goto fail;
	}

	error = gpio_pin_get_by_ofw_property(dev, node, "reset-gpios",
	    &sc->sc_rst_pin);
	if (error != 0) {
		device_printf(dev, "cannot get reset-gpios: %d\n", error);
		goto fail;
	}
	error = gpio_pin_setflags(sc->sc_rst_pin, GPIO_PIN_OUTPUT);
	if (error != 0) {
		device_printf(dev, "cannot set reset-gpios as output: %d\n",
		    error);
		goto fail;
	}

	/* Hardware reset and ILI9486 initialization */
	mhs35_hw_reset(sc);
	error = mhs35_hw_init(sc);
	if (error != 0) {
		device_printf(dev, "display init failed: %d\n", error);
		goto fail;
	}

	/* Allocate shadow framebuffer */
	sc->sc_fb.fb_vbase = (vm_offset_t)malloc(MHS35_FBSIZE, M_MHS35,
	    M_WAITOK | M_ZERO);

	/* Populate fb_info */
	sc->sc_fb.fb_name = device_get_nameunit(dev);
	sc->sc_fb.fb_width = MHS35_WIDTH;
	sc->sc_fb.fb_height = MHS35_HEIGHT;
	sc->sc_fb.fb_bpp = MHS35_BPP;
	sc->sc_fb.fb_depth = MHS35_BPP;
	sc->sc_fb.fb_stride = MHS35_STRIDE;
	sc->sc_fb.fb_size = MHS35_FBSIZE;
	sc->sc_fb.fb_flags = FB_FLAG_NOMMAP;
	sc->sc_fb.fb_cmsize = NCOLORS;

	/*
	 * Configure 16-color console palette for RGB565.
	 * RGB565: 5 bits red at offset 11, 6 bits green at offset 5,
	 * 5 bits blue at offset 0.
	 */
	vt_config_cons_colors(&sc->sc_fb, COLOR_FORMAT_RGB,
	    0x1f, 11, 0x3f, 5, 0x1f, 0);

	/* Register with vt(4) */
	vt_allocate(&vt_mhs35_driver, &sc->sc_fb);
	sc->sc_have_vt = true;

	/* Initial full-screen flush to clear display */
	mhs35_flush_region(sc, 0, 0, MHS35_WIDTH, MHS35_HEIGHT);

	device_printf(dev, "%dx%d %d-bpp framebuffer attached\n",
	    MHS35_WIDTH, MHS35_HEIGHT, MHS35_BPP);

	return (0);

fail:
	if (sc->sc_fb.fb_vbase != 0)
		free((void *)sc->sc_fb.fb_vbase, M_MHS35);
	return (error);
}

static int
mhs35_detach(device_t dev)
{
	struct mhs35_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_have_vt)
		vt_deallocate(&vt_mhs35_driver, &sc->sc_fb);

	/* Display off */
	mhs35_write_cmd(sc, ILI9486_DISPOFF);
	mhs35_write_cmd(sc, ILI9486_SLPIN);

	if (sc->sc_fb.fb_vbase != 0)
		free((void *)sc->sc_fb.fb_vbase, M_MHS35);

	return (0);
}

static device_method_t mhs35_methods[] = {
	DEVMETHOD(device_probe,		mhs35_probe),
	DEVMETHOD(device_attach,	mhs35_attach),
	DEVMETHOD(device_detach,	mhs35_detach),
	DEVMETHOD_END
};

static driver_t mhs35_driver = {
	"mhs35",
	mhs35_methods,
	sizeof(struct mhs35_softc)
};

DRIVER_MODULE(mhs35_vt, spibus, mhs35_driver, NULL, NULL);
MODULE_VERSION(mhs35_vt, 1);
MODULE_DEPEND(mhs35_vt, spibus, 1, 1, 1);
