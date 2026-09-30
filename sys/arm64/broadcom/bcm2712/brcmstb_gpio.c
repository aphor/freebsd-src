/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * brcmstb_gpio -- the Broadcom STB GPIO block ("brcm,brcmstb-gpio"), as used
 * by the BCM2712 for its SoC-native, non-RP1 GPIO controllers.
 *
 * WHY THIS MATTERS ON A RASPBERRY PI 5
 *
 * Every GPIO FreeBSD could drive on this board so far was behind RP1, and so
 * behind PCIe.  But the board's own housekeeping pins are on the SoC, in two
 * of these controllers, and the firmware's device tree hangs real function
 * off them:
 *
 *   gpio@7d508500  banks 32 + 4  WL_ON (28), BT_ON, the SDIO2 lines, ...
 *   gpio@7d517c00  banks 15 + 6  (always-on) RP1_RUN, SD_IOVDD_SEL,
 *                                SD_PWR_ON, the status LED, ...
 *
 * In particular wl-on-reg -- the vmmc-supply of the WiFi SDIO slot -- is a
 * regulator-fixed switched by WL_ON.  With no driver here, regfix(4) can
 * never find its GPIO, the WiFi chip stays unpowered, and cyw(4) cannot
 * attach on an FDT boot.  Measured with the loader's peek command before any
 * kernel ran (rpi5_modules.git doc/LOADER_ZIMAGE.md): bank 0 IODIR read
 * 0xffffffff (every pin an input) and DATA bit 28 read 0.
 *
 * REGISTERS
 *
 * From Linux drivers/gpio/gpio-brcmstb.c.  Banks of up to 32 lines at a
 * 0x20-byte stride, each with eight registers:
 *
 *	0x00 ODEN   0x04 DATA   0x08 IODIR (1 = input)   0x0c EC
 *	0x10 EI     0x14 MASK   0x18 LEVEL               0x1c STAT
 *
 * The device tree numbers lines globally, bank * 32 + bit, so a bank narrower
 * than 32 leaves a gap: on gpio@7d508500, line 32 is bank 1 bit 0.
 * "brcm,gpio-direct" only tells Linux to read the hardware rather than a
 * shadow copy, which is all this driver ever does.
 *
 * ONE RULE: A DIRECTION CHANGE NEVER WRITES DATA
 *
 * The always-on controller carries SD_IOVDD_SEL, which selects 1.8 V or
 * 3.3 V signalling for the SD card that holds the root filesystem, and
 * gpioregulator(4) switches its pin to output at attach.  Measured before
 * boot, that pin is already an output driving 0 (3.3 V), so the switch is a
 * no-op -- provided setflags only touches IODIR.  It does.
 *
 * Not implemented: interrupts (the EC/EI/MASK/LEVEL/STAT registers), and
 * open-drain via ODEN.  No consumer this driver was written for needs them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "gpio_if.h"

#define	BRCMSTB_GPIO_BANK_SIZE	0x20
#define	BRCMSTB_GPIO_ODEN	0x00
#define	BRCMSTB_GPIO_DATA	0x04
#define	BRCMSTB_GPIO_IODIR	0x08	/* 1 = input */

#define	BRCMSTB_GPIO_MAX_BANKS	8
#define	BRCMSTB_GPIO_PER_BANK	32

#define	BRCMSTB_GPIO_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

static struct ofw_compat_data compat_data[] = {
	{ "brcm,brcmstb-gpio",	1 },
	{ NULL,			0 }
};

struct brcmstb_gpio_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct mtx		sc_mtx;
	struct resource		*sc_mem_res;
	int			sc_nbanks;
	uint32_t		sc_width[BRCMSTB_GPIO_MAX_BANKS];
	char			**sc_names;	/* per global line, may be NULL */
	char			*sc_names_buf;
};

#define	BRCMSTB_LOCK(sc)	mtx_lock_spin(&(sc)->sc_mtx)
#define	BRCMSTB_UNLOCK(sc)	mtx_unlock_spin(&(sc)->sc_mtx)

#define	PIN_BANK(pin)		((pin) / BRCMSTB_GPIO_PER_BANK)
#define	PIN_MASK(pin)		(1u << ((pin) % BRCMSTB_GPIO_PER_BANK))
#define	PIN_REG(pin, off)	(PIN_BANK(pin) * BRCMSTB_GPIO_BANK_SIZE + (off))

static int brcmstb_gpio_detach(device_t dev);

static inline uint32_t
rd4(struct brcmstb_gpio_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->sc_mem_res, off));
}

static inline void
wr4(struct brcmstb_gpio_softc *sc, bus_size_t off, uint32_t v)
{
	bus_write_4(sc->sc_mem_res, off, v);
}

/* A line exists if its bank exists and its bit is inside the bank's width. */
static bool
brcmstb_gpio_valid(struct brcmstb_gpio_softc *sc, uint32_t pin)
{
	int bank = PIN_BANK(pin);

	return (bank < sc->sc_nbanks &&
	    (pin % BRCMSTB_GPIO_PER_BANK) < sc->sc_width[bank]);
}

static void
brcmstb_gpio_parse_names(struct brcmstb_gpio_softc *sc, phandle_t node)
{
	ssize_t len;
	char *p, *end;
	int i, npins;

	npins = sc->sc_nbanks * BRCMSTB_GPIO_PER_BANK;
	len = OF_getprop_alloc(node, "gpio-line-names",
	    (void **)&sc->sc_names_buf);
	if (len <= 0)
		return;
	sc->sc_names = malloc(npins * sizeof(char *), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	/*
	 * gpio-line-names is indexed by global line number, gaps included:
	 * the always-on controller has 15 + 6 lines and 38 names, bank 0
	 * padded out to 32 with empty strings.  "" and "-" mean unnamed.
	 */
	p = sc->sc_names_buf;
	end = sc->sc_names_buf + len;
	for (i = 0; i < npins && p < end; i++) {
		if (*p != '\0' && strcmp(p, "-") != 0)
			sc->sc_names[i] = p;
		p += strnlen(p, end - p) + 1;
	}
}

static int
brcmstb_gpio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Broadcom STB GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_gpio_attach(device_t dev)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	pcell_t widths[BRCMSTB_GPIO_MAX_BANKS];
	ssize_t len;
	int b, rid;

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "brcmstb_gpio",
	    MTX_SPIN);

	len = OF_getencprop(node, "brcm,gpio-bank-widths", widths,
	    sizeof(widths));
	if (len <= 0 || (len % sizeof(pcell_t)) != 0) {
		device_printf(dev, "missing brcm,gpio-bank-widths\n");
		brcmstb_gpio_detach(dev);
		return (ENXIO);
	}
	sc->sc_nbanks = len / sizeof(pcell_t);
	for (b = 0; b < sc->sc_nbanks; b++) {
		if (widths[b] > BRCMSTB_GPIO_PER_BANK) {
			device_printf(dev, "bank %d width %u is invalid\n", b,
			    widths[b]);
			brcmstb_gpio_detach(dev);
			return (ENXIO);
		}
		sc->sc_width[b] = widths[b];
	}

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		brcmstb_gpio_detach(dev);
		return (ENXIO);
	}
	if (rman_get_size(sc->sc_mem_res) <
	    (rman_res_t)sc->sc_nbanks * BRCMSTB_GPIO_BANK_SIZE) {
		device_printf(dev, "register window too small for %d banks\n",
		    sc->sc_nbanks);
		brcmstb_gpio_detach(dev);
		return (ENXIO);
	}

	brcmstb_gpio_parse_names(sc, node);

	/*
	 * Say what the firmware left, before anything here writes a register.
	 * Every consumer that then attaches changes state from this baseline,
	 * and without it a change of behaviour is not attributable.
	 */
	for (b = 0; b < sc->sc_nbanks; b++)
		if (bootverbose || sc->sc_width[b] != 0)
			device_printf(dev, "bank %d: %u lines, DATA 0x%08x "
			    "IODIR 0x%08x (firmware state)\n", b,
			    sc->sc_width[b],
			    rd4(sc, b * BRCMSTB_GPIO_BANK_SIZE +
			    BRCMSTB_GPIO_DATA),
			    rd4(sc, b * BRCMSTB_GPIO_BANK_SIZE +
			    BRCMSTB_GPIO_IODIR));

	/* Also registers this node as a GPIO provider for consumers. */
	sc->sc_busdev = gpiobus_add_bus(dev);
	if (sc->sc_busdev == NULL) {
		device_printf(dev, "cannot add gpiobus\n");
		brcmstb_gpio_detach(dev);
		return (ENXIO);
	}
	bus_attach_children(dev);
	return (0);
}

static int
brcmstb_gpio_detach(device_t dev)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	if (sc->sc_busdev != NULL) {
		gpiobus_detach_bus(dev);
		sc->sc_busdev = NULL;
	}
	if (sc->sc_mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->sc_mem_res), sc->sc_mem_res);
		sc->sc_mem_res = NULL;
	}
	free(sc->sc_names, M_DEVBUF);
	sc->sc_names = NULL;
	OF_prop_free(sc->sc_names_buf);
	sc->sc_names_buf = NULL;
	if (mtx_initialized(&sc->sc_mtx))
		mtx_destroy(&sc->sc_mtx);
	return (0);
}

static device_t
brcmstb_gpio_get_bus(device_t dev)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	return (sc->sc_busdev);
}

static int
brcmstb_gpio_pin_max(device_t dev, int *maxpin)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	*maxpin = sc->sc_nbanks * BRCMSTB_GPIO_PER_BANK - 1;
	return (0);
}

static int
brcmstb_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	if (sc->sc_names != NULL && sc->sc_names[pin] != NULL)
		strlcpy(name, sc->sc_names[pin], GPIOMAXNAME);
	else
		snprintf(name, GPIOMAXNAME, "gpio%u", pin);
	return (0);
}

static int
brcmstb_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	*caps = BRCMSTB_GPIO_CAPS;
	return (0);
}

static int
brcmstb_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t iodir;

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	BRCMSTB_LOCK(sc);
	iodir = rd4(sc, PIN_REG(pin, BRCMSTB_GPIO_IODIR));
	BRCMSTB_UNLOCK(sc);
	*flags = (iodir & PIN_MASK(pin)) ? GPIO_PIN_INPUT : GPIO_PIN_OUTPUT;
	return (0);
}

/* Direction only.  DATA is never written here; see the file comment. */
static int
brcmstb_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t iodir;

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	if ((flags & ~BRCMSTB_GPIO_CAPS) != 0)
		return (EOPNOTSUPP);
	if ((flags & BRCMSTB_GPIO_CAPS) == BRCMSTB_GPIO_CAPS)
		return (EINVAL);
	if ((flags & BRCMSTB_GPIO_CAPS) == 0)
		return (0);

	BRCMSTB_LOCK(sc);
	iodir = rd4(sc, PIN_REG(pin, BRCMSTB_GPIO_IODIR));
	if (flags & GPIO_PIN_INPUT)
		iodir |= PIN_MASK(pin);
	else
		iodir &= ~PIN_MASK(pin);
	wr4(sc, PIN_REG(pin, BRCMSTB_GPIO_IODIR), iodir);
	BRCMSTB_UNLOCK(sc);
	return (0);
}

static int
brcmstb_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	BRCMSTB_LOCK(sc);
	*val = (rd4(sc, PIN_REG(pin, BRCMSTB_GPIO_DATA)) & PIN_MASK(pin)) ? 1 : 0;
	BRCMSTB_UNLOCK(sc);
	return (0);
}

static int
brcmstb_gpio_pin_set(device_t dev, uint32_t pin, unsigned int val)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t data;

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	BRCMSTB_LOCK(sc);
	data = rd4(sc, PIN_REG(pin, BRCMSTB_GPIO_DATA));
	if (val)
		data |= PIN_MASK(pin);
	else
		data &= ~PIN_MASK(pin);
	wr4(sc, PIN_REG(pin, BRCMSTB_GPIO_DATA), data);
	BRCMSTB_UNLOCK(sc);
	return (0);
}

static int
brcmstb_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t data;

	if (!brcmstb_gpio_valid(sc, pin))
		return (EINVAL);
	BRCMSTB_LOCK(sc);
	data = rd4(sc, PIN_REG(pin, BRCMSTB_GPIO_DATA));
	wr4(sc, PIN_REG(pin, BRCMSTB_GPIO_DATA), data ^ PIN_MASK(pin));
	BRCMSTB_UNLOCK(sc);
	return (0);
}

/* Two cells, <line flags>, with the line numbered globally across banks. */
static int
brcmstb_gpio_map_gpios(device_t bus, phandle_t dev, phandle_t gparent,
    int gcells, pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{
	if (gcells != 2)
		return (EINVAL);
	*pin = gpios[0];
	*flags = gpios[1];
	return (0);
}

static phandle_t
brcmstb_gpio_get_node(device_t bus, device_t dev)
{
	/* The only child is the GPIO bus, which uses our node. */
	return (ofw_bus_get_node(bus));
}

static device_method_t brcmstb_gpio_methods[] = {
	DEVMETHOD(device_probe,		brcmstb_gpio_probe),
	DEVMETHOD(device_attach,	brcmstb_gpio_attach),
	DEVMETHOD(device_detach,	brcmstb_gpio_detach),

	DEVMETHOD(gpio_get_bus,		brcmstb_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		brcmstb_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	brcmstb_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	brcmstb_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	brcmstb_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	brcmstb_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		brcmstb_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		brcmstb_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	brcmstb_gpio_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	brcmstb_gpio_map_gpios),

	DEVMETHOD(ofw_bus_get_node,	brcmstb_gpio_get_node),

	DEVMETHOD_END
};

/*
 * Named "gpio" because gpiobus(4) and ofw_gpiobus attach to that devclass.
 * Attached in the interrupt pass, ahead of gpioregulator(4) (interrupt pass,
 * order last) and of every default-pass consumer; regfix(4), which attaches
 * earlier, retries on the next pass and finds us then.
 */
static driver_t brcmstb_gpio_driver = {
	"gpio",
	brcmstb_gpio_methods,
	sizeof(struct brcmstb_gpio_softc),
};

EARLY_DRIVER_MODULE(brcmstb_gpio, simplebus, brcmstb_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(brcmstb_gpio, 1);
MODULE_DEPEND(brcmstb_gpio, gpiobus, 1, 1, 1);
