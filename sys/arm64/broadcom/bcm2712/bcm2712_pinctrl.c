/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * bcm2712_pinctrl -- pin function and pull configuration for the BCM2712 D0
 * SoC GPIO block ("brcm,bcm2712d0-pinctrl", pinctrl@7d504100).
 *
 * WHY IT IS NEEDED
 *
 * The WiFi chip's SDIO bus is on gpio30..35, which must be muxed to the
 * "sd2" function for mmc@1100000 to reach it.  EDK2 does that on the ACPI
 * lane.  On an FDT boot nothing does: measured with the loader's peek before
 * any kernel ran, mux words 2 and 3 both read 0, i.e. every one of those pins
 * was still plain GPIO, with pull-downs on all of them.  So even a powered
 * chip would be unreachable.  The device tree says what is wanted --
 * sdio2_30_pins: sd2 on 30..35, no pull on the clock, pull-up on CMD and
 * DAT0..3 -- and consumers apply it through fdt_pinctrl(4).
 *
 * REGISTERS
 *
 * Tables from Linux drivers/pinctrl/bcm/pinctrl-bcm2712.c, D0 variant
 * (bcm2712_d0_gpio_pin_regs and bcm2712_d0_gpio_pin_funcs).  Eight 32-bit
 * words:
 *
 *	words 0..3  function select, 4 bits per pin: 0 = gpio, n = the n-th
 *	            alternate function in the pin's table
 *	words 4..6  pad pull, 2 bits per pin: 0 none, 1 down, 2 up
 *
 * The EMMC pins (36..46) have pulls but no function select.
 *
 * The decoding was checked against measured state before being trusted:
 * emmc_sd_pulls asks for pull-up on emmc_cmd and emmc_dat0..3, and word 6 as
 * the firmware left it (0x0aaaa595) holds 2 in exactly those fields.
 *
 * Nothing here is applied at attach.  There is no fdt_pinctrl_configure_tree()
 * call: that would apply every enabled node's pinctrl-0, including devices
 * nothing on this lane drives.  Consumers ask for their own configuration.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "fdt_pinctrl_if.h"

#define	BCM2712_PULL_NONE	0
#define	BCM2712_PULL_DOWN	1
#define	BCM2712_PULL_UP		2
#define	BCM2712_PULL_MASK	0x3
#define	BCM2712_FSEL_MASK	0xf
#define	BCM2712_NALT		8

#define	NOREG			0xff

struct bcm2712_pin {
	const char	*name;
	uint8_t		mux_reg, mux_nib;	/* NOREG: no function select */
	uint8_t		pad_reg, pad_fld;
	const char	*alt[BCM2712_NALT];	/* alt1..alt8, NULL = none */
};

#define	GPIO(n, mr, mb, pr, pb, ...) \
	{ "gpio" #n, mr, mb, pr, pb, { __VA_ARGS__ } }
#define	EMMC(nm, pr, pb) \
	{ nm, NOREG, 0, pr, pb, { NULL } }

/* bcm2712_d0_gpio_pin_regs + bcm2712_d0_gpio_pin_funcs, merged per pin. */
static const struct bcm2712_pin bcm2712_d0_pins[] = {
	GPIO(1, 0, 0, 4, 5, "vc_i2c0", "usb_pwr", "gpclk0", "sd_card_e",
	    "vc_spi3", "sr_edm_sense", "vc_spi0", "vc_uart0"),
	GPIO(2, 0, 1, 4, 6, "vc_i2c0", "usb_pwr", "gpclk1", "sd_card_e",
	    "vc_spi3", "clk_observe", "vc_spi0", "vc_uart0"),
	GPIO(3, 0, 2, 4, 7, "vc_i2c3", "usb_vbus", "gpclk2", "sd_card_e",
	    "vc_spi3", "vc_spi0", "vc_uart0", NULL),
	GPIO(4, 0, 3, 4, 8, "vc_i2c3", "vc_pwm1", "vc_spi3", "sd_card_e",
	    "vc_spi3", "vc_spi0", "vc_uart0", NULL),
	GPIO(10, 0, 4, 4, 9, "bsc_m3", "vc_pwm1", "vc_spi3", "sd_card_e",
	    "vc_spi3", "gpclk0", NULL, NULL),
	GPIO(11, 0, 5, 4, 10, "bsc_m3", "vc_spi3", "clk_observe", "sd_card_c",
	    "gpclk1", NULL, NULL, NULL),
	GPIO(12, 0, 6, 4, 11, "spi_s", "vc_spi3", "sd_card_c", "sd_card_d",
	    NULL, NULL, NULL, NULL),
	GPIO(13, 0, 7, 4, 12, "spi_s", "vc_spi3", "sd_card_c", "sd_card_d",
	    NULL, NULL, NULL, NULL),
	GPIO(14, 1, 0, 4, 13, "spi_s", "uui", "arm_jtag", "vc_pwm0", "vc_i2c0",
	    "sd_card_d", NULL, NULL),
	GPIO(15, 1, 1, 4, 14, "spi_s", "uui", "arm_jtag", "vc_pwm0", "vc_i2c0",
	    "gpclk0", NULL, NULL),
	GPIO(18, 1, 2, 5, 0, "sd_card_f", "vc_pwm1", NULL, NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(19, 1, 3, 5, 1, "sd_card_f", "usb_pwr", "vc_pwm1", NULL, NULL,
	    NULL, NULL, NULL),
	GPIO(20, 1, 4, 5, 2, "vc_i2c3", "uui", "vc_uart0", "arm_jtag",
	    "vc_uart2", NULL, NULL, NULL),
	GPIO(21, 1, 5, 5, 3, "vc_i2c3", "uui", "vc_uart0", "arm_jtag",
	    "vc_uart2", NULL, NULL, NULL),
	GPIO(22, 1, 6, 5, 4, "sd_card_f", "vc_uart0", "vc_i2c3", NULL, NULL,
	    NULL, NULL, NULL),
	GPIO(23, 1, 7, 5, 5, "vc_uart0", "vc_i2c3", NULL, NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(24, 2, 0, 5, 6, "sd_card_b", "vc_spi0", "arm_jtag", "uart0",
	    "usb_pwr", "vc_uart2", "vc_uart0", NULL),
	GPIO(25, 2, 1, 5, 7, "sd_card_b", "vc_spi0", "arm_jtag", "uart0",
	    "usb_pwr", "vc_uart2", "vc_uart0", NULL),
	GPIO(26, 2, 2, 5, 8, "sd_card_b", "vc_spi0", "arm_jtag", "uart0",
	    "usb_vbus", "vc_uart2", "vc_spi0", NULL),
	GPIO(27, 2, 3, 5, 9, "sd_card_b", "vc_spi0", "arm_jtag", "uart0",
	    "vc_uart2", "vc_spi0", NULL, NULL),
	GPIO(28, 2, 4, 5, 10, "sd_card_b", "vc_spi0", "arm_jtag", "vc_i2c0",
	    "vc_spi0", NULL, NULL, NULL),
	GPIO(29, 2, 5, 5, 11, "arm_jtag", "vc_i2c0", "vc_spi0", NULL, NULL,
	    NULL, NULL, NULL),
	GPIO(30, 2, 6, 5, 12, "sd2", "gpclk0", "vc_pwm0", NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(31, 2, 7, 5, 13, "sd2", "vc_spi3", "vc_pwm0", NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(32, 3, 0, 5, 14, "sd2", "vc_spi3", "vc_uart3", NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(33, 3, 1, 6, 0, "sd2", "vc_spi3", "vc_uart3", NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(34, 3, 2, 6, 1, "sd2", "vc_spi3", "vc_i2c5", NULL, NULL, NULL,
	    NULL, NULL),
	GPIO(35, 3, 3, 6, 2, "sd2", "vc_spi3", "vc_i2c5", NULL, NULL, NULL,
	    NULL, NULL),
	EMMC("emmc_cmd", 6, 3),
	EMMC("emmc_ds", 6, 4),
	EMMC("emmc_clk", 6, 5),
	EMMC("emmc_dat0", 6, 6),
	EMMC("emmc_dat1", 6, 7),
	EMMC("emmc_dat2", 6, 8),
	EMMC("emmc_dat3", 6, 9),
	EMMC("emmc_dat4", 6, 10),
	EMMC("emmc_dat5", 6, 11),
	EMMC("emmc_dat6", 6, 12),
	EMMC("emmc_dat7", 6, 13),
};

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2712d0-pinctrl",	1 },
	{ NULL,				0 }
};

struct bcm2712_pinctrl_softc {
	device_t		sc_dev;
	struct mtx		sc_mtx;
	struct resource		*sc_mem_res;
};

static const char *const pull_names[] = { "none", "down", "up", "?" };

static const struct bcm2712_pin *
bcm2712_pinctrl_lookup(const char *name)
{
	size_t i;

	for (i = 0; i < nitems(bcm2712_d0_pins); i++)
		if (strcmp(bcm2712_d0_pins[i].name, name) == 0)
			return (&bcm2712_d0_pins[i]);
	return (NULL);
}

/* 0 = gpio, 1..8 = the pin's alternates; -1 if the pin cannot do it. */
static int
bcm2712_pinctrl_fsel(const struct bcm2712_pin *pin, const char *func)
{
	int i;

	if (strcmp(func, "gpio") == 0)
		return (0);
	for (i = 0; i < BCM2712_NALT; i++)
		if (pin->alt[i] != NULL && strcmp(pin->alt[i], func) == 0)
			return (i + 1);
	/* The binding also allows the raw names alt1..alt8. */
	if (strncmp(func, "alt", 3) == 0 && func[3] >= '1' && func[3] <= '8' &&
	    func[4] == '\0')
		return (func[3] - '0');
	return (-1);
}

static void
bcm2712_pinctrl_rmw(struct bcm2712_pinctrl_softc *sc, u_int reg, u_int shift,
    uint32_t mask, uint32_t val, uint32_t *oldp)
{
	uint32_t v;

	mtx_lock_spin(&sc->sc_mtx);
	v = bus_read_4(sc->sc_mem_res, reg * 4);
	*oldp = (v >> shift) & mask;
	v &= ~(mask << shift);
	v |= (val & mask) << shift;
	bus_write_4(sc->sc_mem_res, reg * 4, v);
	mtx_unlock_spin(&sc->sc_mtx);
}

/* Apply one node that carries "pins" (and optionally function / bias). */
static int
bcm2712_pinctrl_apply(struct bcm2712_pinctrl_softc *sc, phandle_t node)
{
	const struct bcm2712_pin *pin;
	char *pins, *func, *p;
	ssize_t plen;
	uint32_t old;
	int fsel, pull, error;

	plen = OF_getprop_alloc(node, "pins", (void **)&pins);
	if (plen <= 0)
		return (0);
	func = NULL;
	(void)OF_getprop_alloc(node, "function", (void **)&func);

	pull = -1;
	if (OF_hasprop(node, "bias-disable"))
		pull = BCM2712_PULL_NONE;
	else if (OF_hasprop(node, "bias-pull-up"))
		pull = BCM2712_PULL_UP;
	else if (OF_hasprop(node, "bias-pull-down"))
		pull = BCM2712_PULL_DOWN;

	error = 0;
	for (p = pins; p < pins + plen; p += strlen(p) + 1) {
		pin = bcm2712_pinctrl_lookup(p);
		if (pin == NULL) {
			device_printf(sc->sc_dev, "unknown pin \"%s\"\n", p);
			error = EINVAL;
			continue;
		}
		if (func != NULL) {
			fsel = bcm2712_pinctrl_fsel(pin, func);
			if (fsel < 0 || pin->mux_reg == NOREG) {
				device_printf(sc->sc_dev,
				    "%s cannot be \"%s\"\n", p, func);
				error = EINVAL;
				continue;
			}
			bcm2712_pinctrl_rmw(sc, pin->mux_reg, pin->mux_nib * 4,
			    BCM2712_FSEL_MASK, fsel, &old);
			if (old != (uint32_t)fsel)
				device_printf(sc->sc_dev, "%s: function %u -> "
				    "%u (%s)\n", p, old, fsel, func);
		}
		if (pull >= 0) {
			bcm2712_pinctrl_rmw(sc, pin->pad_reg, pin->pad_fld * 2,
			    BCM2712_PULL_MASK, pull, &old);
			if (old != (uint32_t)pull)
				device_printf(sc->sc_dev, "%s: pull %s -> %s\n",
				    p, pull_names[old], pull_names[pull]);
		}
	}
	OF_prop_free(func);
	OF_prop_free(pins);
	return (error);
}

/*
 * A configuration node either carries "pins" itself (wl_on_pins) or groups
 * children that do (sdio2_30_pins: pin_clk, pin_cmd, pins_dat).
 */
static int
bcm2712_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct bcm2712_pinctrl_softc *sc = device_get_softc(dev);
	phandle_t node, child;
	int error, e;

	node = OF_node_from_xref(cfgxref);
	if (OF_hasprop(node, "pins"))
		return (bcm2712_pinctrl_apply(sc, node));

	error = 0;
	for (child = OF_child(node); child != 0; child = OF_peer(child))
		if ((e = bcm2712_pinctrl_apply(sc, child)) != 0)
			error = e;
	return (error);
}

static int
bcm2712_pinctrl_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "BCM2712 D0 pin controller");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm2712_pinctrl_attach(device_t dev)
{
	struct bcm2712_pinctrl_softc *sc = device_get_softc(dev);
	int rid = 0;

	sc->sc_dev = dev;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}
	if (rman_get_size(sc->sc_mem_res) < 7 * 4) {
		device_printf(dev, "register window too small\n");
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->sc_mem_res);
		return (ENXIO);
	}
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "bcm2712_pinctrl",
	    MTX_SPIN);

	device_printf(dev, "mux %08x %08x %08x %08x  pads %08x %08x %08x "
	    "(firmware state)\n",
	    bus_read_4(sc->sc_mem_res, 0x00), bus_read_4(sc->sc_mem_res, 0x04),
	    bus_read_4(sc->sc_mem_res, 0x08), bus_read_4(sc->sc_mem_res, 0x0c),
	    bus_read_4(sc->sc_mem_res, 0x10), bus_read_4(sc->sc_mem_res, 0x14),
	    bus_read_4(sc->sc_mem_res, 0x18));

	/* NULL: every descendant node may be referenced by a pinctrl-N. */
	fdt_pinctrl_register(dev, NULL);
	return (0);
}

static device_method_t bcm2712_pinctrl_methods[] = {
	DEVMETHOD(device_probe,		bcm2712_pinctrl_probe),
	DEVMETHOD(device_attach,	bcm2712_pinctrl_attach),

	DEVMETHOD(fdt_pinctrl_configure, bcm2712_pinctrl_configure),

	DEVMETHOD_END
};

static driver_t bcm2712_pinctrl_driver = {
	"bcm2712_pinctrl",
	bcm2712_pinctrl_methods,
	sizeof(struct bcm2712_pinctrl_softc),
};

/* Ahead of every consumer, which all attach in later passes. */
EARLY_DRIVER_MODULE(bcm2712_pinctrl, simplebus, bcm2712_pinctrl_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(bcm2712_pinctrl, 1);
