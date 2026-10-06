/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * Written from a functional specification of the BCM2712 PM block.
 */

/*
 * BCM2712 PM block: the V3D power domain and the V3D reset.
 *
 * The PM block's register window is also the SoC's watchdog, reboot and
 * power-off controller.  This driver owns the whole window with a single
 * mapping, so that a watchdog and power-off function can share it, and
 * implements only what the V3D GPU needs:
 *
 *  - reset cell 0 (V3D), through hwreset(9);
 *  - power-domain cell 1 (GRAFX_V3D), through the interface declared in
 *    bcm2712_pm.h, because FreeBSD has no power-domain framework.
 *
 * On the BCM2712 both act on a single bit, V3DRSTN (bit 6 of the register
 * at offset 0x304), an active-low reset.  The domain is on while the bit is
 * set and off while it is clear; a reset clears it, waits, and sets it
 * again.  Nothing else is switched, polled or acknowledged: the BCM2712
 * node has no AXI bridge region and no clocks, and the V3D power switch of
 * older parts is not used.
 *
 * The V3D domain has a parent, the GRAFX domain (cell 0), whose power
 * switch on the BCM2835 family is the register at offset 0x10c.  It is
 * powered on before the V3D domain is switched on and off after the V3D
 * domain is switched off.
 *
 * Writes are confined to bit 6 of 0x304 and to the power-switch fields of
 * 0x10c.  The registers RSTC (0x1c), RSTS (0x20) and WDOG (0x24) reboot or
 * halt the board and are never written here.
 *
 * Every write to the window carries the password 0x5a in bits 31:24.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/syslog.h>

#include <machine/atomic.h>
#include <machine/bus.h>

#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <arm64/broadcom/bcm2712/bcm2712_pm.h>

#include "hwreset_if.h"

/* Bits 31:24 of every write. */
#define	PM_PASSWORD		0x5a000000u
#define	PM_PASSWORD_MASK	0xff000000u

/*
 * Registers this driver uses.  The watchdog, reboot and power-off
 * registers (RSTC 0x1c, RSTS 0x20, WDOG 0x24) share the window and belong
 * to a separate function.
 */
#define	PM_GRAFX		0x10c	/* parent domain's power switch */
#define	PM_V3D			0x304	/* V3D reset line */
#define	PM_WINDOW_SIZE		0x308

#define	PM_V3D_V3DRSTN		(1u << 6)	/* V3D reset, active low */

/* Fields of a power-switch register, as on the BCM2835 family. */
#define	PM_INRUSH_SHIFT		13		/* inrush current limit */
#define	PM_INRUSH_MASK		(3u << PM_INRUSH_SHIFT)
#define	PM_INRUSH_MAX		3
#define	PM_ISFUNC		(1u << 5)	/* functional isolation off */
#define	PM_MRDONE		(1u << 4)	/* memory repair finished */
#define	PM_MEMREP		(1u << 3)	/* start memory repair */
#define	PM_ISPOW		(1u << 2)	/* electrical isolation off */
#define	PM_POWOK		(1u << 1)	/* power good */
#define	PM_POWUP		(1u << 0)	/* power switch closed */

/* How long power good and memory repair are waited for, microseconds. */
#define	PM_POWOK_WAIT_US	3
#define	PM_MRDONE_WAIT_US	1

/*
 * Wait before a reset line is released.  After an assertion it is the
 * shortest time the line stays in reset.
 */
#define	PM_RSTN_HOLD_US		1

/*
 * A power domain or a reset line implemented by one active-low reset bit.
 * On the BCM2712 the V3D domain is switched on by releasing the V3D reset
 * and off by asserting it.
 */
struct bcm2712_pm_rstn {
	u_int		cell;		/* number in the binding */
	const char	*name;
	bus_size_t	reg;
	uint32_t	bit;
};

static const struct bcm2712_pm_rstn bcm2712_pm_domains[] = {
	{ BCM2712_PM_DOMAIN_GRAFX_V3D, "grafx_v3d", PM_V3D, PM_V3D_V3DRSTN },
};
#define	BCM2712_PM_NDOMAINS	nitems(bcm2712_pm_domains)

static const struct bcm2712_pm_rstn bcm2712_pm_resets[] = {
	{ BCM2712_PM_RESET_V3D, "v3d", PM_V3D, PM_V3D_V3DRSTN },
};
#define	BCM2712_PM_NRESETS	nitems(bcm2712_pm_resets)

struct bcm2712_pm_softc {
	device_t		dev;
	struct resource		*mem;
	/*
	 * Serialises every read-modify-write of the window and the domain
	 * counts below.
	 */
	struct mtx		mtx;
	u_int			handles;	/* domain handles in existence */
	u_int			users[BCM2712_PM_NDOMAINS]; /* enabled handles */
	uint32_t		v3d_attach;	/* 0x304 as read at attach */
	uint32_t		grafx_attach;	/* 0x10c as read at attach */
};

struct bcm2712_pm_domain {
	struct bcm2712_pm_softc		*sc;
	const struct bcm2712_pm_rstn	*def;
	u_int				idx;	/* in bcm2712_pm_domains[] */
	device_t			consumer;
	bool				enabled;
};

static MALLOC_DEFINE(M_BCM2712_PM, "bcm2712_pm", "BCM2712 PM domain handles");

static driver_t bcm2712_pm_driver;

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2712-pm",	1 },
	{ NULL,			0 }
};

/*
 * Change one active-low reset bit by read-modify-write.  The value written
 * is the password in bits 31:24 and the intended value in bits 23:0; bits
 * 31:24 as read are not carried over.  Reading the register back and the
 * barrier make the write complete at the block before the caller goes on,
 * so that a delay that follows is a lower bound on the time the line
 * spends in its new state.
 */
static void
bcm2712_pm_rstn_write(struct bcm2712_pm_softc *sc,
    const struct bcm2712_pm_rstn *line, bool set)
{
	uint32_t val;

	mtx_assert(&sc->mtx, MA_OWNED);
	KASSERT(line->reg == PM_V3D && line->bit == PM_V3D_V3DRSTN,
	    ("%s: register 0x%jx bit 0x%x is not writable", __func__,
	    (uintmax_t)line->reg, line->bit));

	val = bus_read_4(sc->mem, line->reg) & ~PM_PASSWORD_MASK;
	if (set)
		val |= line->bit;
	else
		val &= ~line->bit;
	bus_write_4(sc->mem, line->reg, PM_PASSWORD | val);
	(void)bus_read_4(sc->mem, line->reg);
	dsb(sy);
}

/*
 * The parent domain, GRAFX.
 *
 * What the register at 0x10c is on the BCM2712 is not established by any
 * source.  The fields and the two sequences below are those of the GRAFX
 * power switch of the BCM2835 family, and they are applied to the BCM2712
 * because the reference implementation applies them to it.  Each sequence
 * logs the register, at debug priority, when it starts and when it ends, so
 * that what the register does here can be seen.
 */

static void
bcm2712_pm_grafx_write(struct bcm2712_pm_softc *sc, uint32_t clear,
    uint32_t set)
{
	uint32_t val;

	mtx_assert(&sc->mtx, MA_OWNED);

	val = bus_read_4(sc->mem, PM_GRAFX) & ~PM_PASSWORD_MASK;
	val = (val & ~clear) | set;
	bus_write_4(sc->mem, PM_GRAFX, PM_PASSWORD | val);
}

/* Read the register until 'bit' is set, for 'us' microseconds at most. */
static bool
bcm2712_pm_grafx_poll(struct bcm2712_pm_softc *sc, uint32_t bit, u_int us)
{
	u_int i;

	for (i = 0; i < us; i++) {
		if ((bus_read_4(sc->mem, PM_GRAFX) & bit) != 0)
			return (true);
		DELAY(1);
	}
	return ((bus_read_4(sc->mem, PM_GRAFX) & bit) != 0);
}

/*
 * Power the parent domain on.  A switch that is closed already is left as
 * it is.  Otherwise the switch is closed with each inrush limit in turn,
 * lowest first; then the electrical isolation is removed, the memory is
 * repaired and the functional isolation is removed.  If power does not
 * become good, or the repair does not finish, what was done is undone and
 * the result is ETIMEDOUT.
 */
static int
bcm2712_pm_grafx_on(struct bcm2712_pm_softc *sc)
{
	uint32_t val;
	u_int inrush;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	val = bus_read_4(sc->mem, PM_GRAFX);
	device_log(sc->dev, LOG_DEBUG,
	    "parent domain power-on starts: 0x10c = 0x%08x\n", val);
	error = 0;
	if ((val & PM_POWUP) != 0)
		goto done;

	for (inrush = 0; inrush <= PM_INRUSH_MAX; inrush++) {
		bcm2712_pm_grafx_write(sc, PM_INRUSH_MASK,
		    PM_POWUP | inrush << PM_INRUSH_SHIFT);
		(void)bcm2712_pm_grafx_poll(sc, PM_POWOK, PM_POWOK_WAIT_US);
	}
	if ((bus_read_4(sc->mem, PM_GRAFX) & PM_POWOK) == 0) {
		val = bus_read_4(sc->mem, PM_GRAFX);
		bcm2712_pm_grafx_write(sc, PM_POWUP | PM_INRUSH_MASK, 0);
		device_printf(sc->dev, "parent domain: power did not become "
		    "good: 0x10c = 0x%08x\n", val);
		error = ETIMEDOUT;
		goto done;
	}

	bcm2712_pm_grafx_write(sc, 0, PM_ISPOW);
	bcm2712_pm_grafx_write(sc, 0, PM_MEMREP);
	if (!bcm2712_pm_grafx_poll(sc, PM_MRDONE, PM_MRDONE_WAIT_US)) {
		val = bus_read_4(sc->mem, PM_GRAFX);
		bcm2712_pm_grafx_write(sc, PM_ISPOW, 0);
		bcm2712_pm_grafx_write(sc, PM_POWUP | PM_INRUSH_MASK, 0);
		device_printf(sc->dev, "parent domain: memory repair did not "
		    "finish: 0x10c = 0x%08x\n", val);
		error = ETIMEDOUT;
		goto done;
	}
	bcm2712_pm_grafx_write(sc, 0, PM_ISFUNC);

done:
	device_log(sc->dev, LOG_DEBUG,
	    "parent domain power-on ends%s: 0x10c = 0x%08x\n",
	    error != 0 ? " in a timeout" : "",
	    bus_read_4(sc->mem, PM_GRAFX));
	return (error);
}

/* Power the parent domain off: three writes, nothing to wait for. */
static void
bcm2712_pm_grafx_off(struct bcm2712_pm_softc *sc)
{

	mtx_assert(&sc->mtx, MA_OWNED);

	device_log(sc->dev, LOG_DEBUG,
	    "parent domain power-off starts: 0x10c = 0x%08x\n",
	    bus_read_4(sc->mem, PM_GRAFX));
	bcm2712_pm_grafx_write(sc, PM_ISFUNC, 0);
	bcm2712_pm_grafx_write(sc, PM_ISPOW, 0);
	bcm2712_pm_grafx_write(sc, PM_POWUP, 0);
	device_log(sc->dev, LOG_DEBUG,
	    "parent domain power-off ends: 0x10c = 0x%08x\n",
	    bus_read_4(sc->mem, PM_GRAFX));
}

/* Assert a reset line: one write, nothing to wait for. */
static void
bcm2712_pm_rstn_assert(struct bcm2712_pm_softc *sc,
    const struct bcm2712_pm_rstn *line)
{

	bcm2712_pm_rstn_write(sc, line, false);
}

/* Release a reset line: wait, then one write. */
static void
bcm2712_pm_rstn_release(struct bcm2712_pm_softc *sc,
    const struct bcm2712_pm_rstn *line)
{

	mtx_assert(&sc->mtx, MA_OWNED);
	DELAY(PM_RSTN_HOLD_US);
	bcm2712_pm_rstn_write(sc, line, true);
}

/*
 * hwreset(9) provider.
 */

static const struct bcm2712_pm_rstn *
bcm2712_pm_reset_lookup(intptr_t id)
{
	u_int i;

	for (i = 0; i < BCM2712_PM_NRESETS; i++) {
		if (bcm2712_pm_resets[i].cell == id)
			return (&bcm2712_pm_resets[i]);
	}
	return (NULL);
}

static int
bcm2712_pm_hwreset_map(device_t dev, phandle_t xref, int ncells,
    pcell_t *cells, intptr_t *id)
{

	if (ncells != 1 || bcm2712_pm_reset_lookup(cells[0]) == NULL)
		return (EINVAL);
	*id = cells[0];
	return (0);
}

/*
 * The hardware's reset is a complete pulse: assert, wait, release.
 * hwreset(9) offers its two halves.  Asserting writes the line clear;
 * deasserting waits PM_RSTN_HOLD_US and then writes it set, so an assert
 * followed by a deassert is the complete reset.  A consumer whose block
 * needs a longer pulse, such as a number of cycles of its own clock, keeps
 * the reset asserted for that long itself.
 *
 * The reset does not use the power-domain counts: it leaves the line
 * released, that is the V3D domain on, whatever state it found.
 */
static int
bcm2712_pm_hwreset_assert(device_t dev, intptr_t id, bool assert)
{
	struct bcm2712_pm_softc *sc;
	const struct bcm2712_pm_rstn *line;

	sc = device_get_softc(dev);
	line = bcm2712_pm_reset_lookup(id);
	if (line == NULL)
		return (EINVAL);

	mtx_lock(&sc->mtx);
	if (assert)
		bcm2712_pm_rstn_assert(sc, line);
	else
		bcm2712_pm_rstn_release(sc, line);
	mtx_unlock(&sc->mtx);
	return (0);
}

/* Read from the register the reset drives. */
static int
bcm2712_pm_hwreset_is_asserted(device_t dev, intptr_t id, bool *value)
{
	struct bcm2712_pm_softc *sc;
	const struct bcm2712_pm_rstn *line;

	sc = device_get_softc(dev);
	line = bcm2712_pm_reset_lookup(id);
	if (line == NULL)
		return (EINVAL);

	*value = (bus_read_4(sc->mem, line->reg) & line->bit) == 0;
	return (0);
}

/*
 * Power-domain interface (bcm2712_pm.h).
 */

int
bcm2712_pm_domain_get_by_ofw_idx(device_t consumer, phandle_t cnode, int idx,
    bcm2712_pm_domain_t *domp)
{
	struct bcm2712_pm_softc *sc;
	struct bcm2712_pm_domain *dom;
	device_t pmdev;
	phandle_t xref;
	pcell_t *cells;
	int error, ncells;
	u_int cell, i;

	if (cnode <= 0)
		cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (ENXIO);

	error = ofw_bus_parse_xref_list_alloc(cnode, "power-domains",
	    "#power-domain-cells", idx, &xref, &ncells, &cells);
	if (error != 0)
		return (error);
	cell = ncells == 1 ? cells[0] : 0;
	OF_prop_free(cells);

	pmdev = OF_device_from_xref(xref);
	if (pmdev == NULL || device_get_driver(pmdev) != &bcm2712_pm_driver ||
	    !device_is_attached(pmdev))
		return (ENODEV);
	if (ncells != 1)
		return (EINVAL);
	for (i = 0; i < BCM2712_PM_NDOMAINS; i++) {
		if (bcm2712_pm_domains[i].cell == cell)
			break;
	}
	if (i == BCM2712_PM_NDOMAINS)
		return (EINVAL);

	sc = device_get_softc(pmdev);
	dom = malloc(sizeof(*dom), M_BCM2712_PM, M_WAITOK | M_ZERO);
	dom->sc = sc;
	dom->def = &bcm2712_pm_domains[i];
	dom->idx = i;
	dom->consumer = consumer;

	mtx_lock(&sc->mtx);
	sc->handles++;
	mtx_unlock(&sc->mtx);

	*domp = dom;
	return (0);
}

/*
 * Domain on: its parent domain first, then release its reset line, after
 * the hold time.  If the parent domain does not come on the domain is left
 * off and the handle is not enabled.
 */
int
bcm2712_pm_domain_enable(bcm2712_pm_domain_t dom)
{
	struct bcm2712_pm_softc *sc;
	int error;

	if (dom == NULL)
		return (EINVAL);
	sc = dom->sc;

	error = 0;
	mtx_lock(&sc->mtx);
	if (!dom->enabled) {
		/* The first user: the parent domain, then the domain. */
		if (sc->users[dom->idx] == 0)
			error = bcm2712_pm_grafx_on(sc);
		if (error == 0) {
			if (sc->users[dom->idx]++ == 0)
				bcm2712_pm_rstn_release(sc, dom->def);
			dom->enabled = true;
		}
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

/* Domain off: assert its reset line, then power its parent domain off. */
int
bcm2712_pm_domain_disable(bcm2712_pm_domain_t dom)
{
	struct bcm2712_pm_softc *sc;

	if (dom == NULL)
		return (EINVAL);
	sc = dom->sc;

	mtx_lock(&sc->mtx);
	if (dom->enabled) {
		KASSERT(sc->users[dom->idx] > 0,
		    ("%s: domain %s has no users", __func__, dom->def->name));
		/* The last user: the domain, then the parent domain. */
		if (--sc->users[dom->idx] == 0) {
			bcm2712_pm_rstn_assert(sc, dom->def);
			bcm2712_pm_grafx_off(sc);
		}
		dom->enabled = false;
	}
	mtx_unlock(&sc->mtx);
	return (0);
}

void
bcm2712_pm_domain_release(bcm2712_pm_domain_t dom)
{
	struct bcm2712_pm_softc *sc;

	if (dom == NULL)
		return;
	sc = dom->sc;

	(void)bcm2712_pm_domain_disable(dom);
	mtx_lock(&sc->mtx);
	KASSERT(sc->handles > 0, ("%s: no handles", __func__));
	sc->handles--;
	mtx_unlock(&sc->mtx);
	free(dom, M_BCM2712_PM);
}

/*
 * Sysctls: the registers this driver reads, now and at attach.
 */

static int
bcm2712_pm_sysctl_hex(struct sysctl_req *req, struct sysctl_oid *oidp,
    uint32_t val)
{
	char buf[sizeof("0x00000000")];

	snprintf(buf, sizeof(buf), "0x%08x", val);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

/* arg2 is the register offset. */
static int
bcm2712_pm_sysctl_reg(SYSCTL_HANDLER_ARGS)
{
	struct bcm2712_pm_softc *sc;

	sc = arg1;
	return (bcm2712_pm_sysctl_hex(req, oidp, bus_read_4(sc->mem, arg2)));
}

/* arg1 points at a value saved at attach. */
static int
bcm2712_pm_sysctl_saved(SYSCTL_HANDLER_ARGS)
{

	return (bcm2712_pm_sysctl_hex(req, oidp, *(uint32_t *)arg1));
}

static void
bcm2712_pm_sysctl_init(struct bcm2712_pm_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;
	char name[32], descr[64];
	u_int i;

	ctx = device_get_sysctl_ctx(sc->dev);
	list = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "reg_304",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, PM_V3D,
	    bcm2712_pm_sysctl_reg, "A", "Register 0x304 (V3D reset line)");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "reg_304_attach",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->v3d_attach, 0,
	    bcm2712_pm_sysctl_saved, "A",
	    "Register 0x304 as read at attach, before any write");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "reg_10c",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, PM_GRAFX,
	    bcm2712_pm_sysctl_reg, "A",
	    "Register 0x10c (parent domain's power switch)");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "reg_10c_attach",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->grafx_attach, 0,
	    bcm2712_pm_sysctl_saved, "A",
	    "Register 0x10c as read at attach, before any write");

	for (i = 0; i < BCM2712_PM_NDOMAINS; i++) {
		snprintf(name, sizeof(name), "%s_users",
		    bcm2712_pm_domains[i].name);
		snprintf(descr, sizeof(descr),
		    "Enabled handles of power domain %u",
		    bcm2712_pm_domains[i].cell);
		SYSCTL_ADD_UINT(ctx, list, OID_AUTO, name,
		    CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->users[i], 0, descr);
	}
}

/*
 * Newbus.
 */

static int
bcm2712_pm_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "BCM2712 power management");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm2712_pm_attach(device_t dev)
{
	struct bcm2712_pm_softc *sc;
	phandle_t node;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/* The BCM2712 node has the "pm" region only. */
	if (ofw_bus_find_string_index(node, "reg-names", "pm", &rid) != 0)
		rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map the register window\n");
		return (ENXIO);
	}
	if (rman_get_size(sc->mem) < PM_WINDOW_SIZE) {
		device_printf(dev,
		    "register window is 0x%jx bytes, 0x%x needed\n",
		    (uintmax_t)rman_get_size(sc->mem), PM_WINDOW_SIZE);
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Saved before this driver writes anything. */
	sc->v3d_attach = bus_read_4(sc->mem, PM_V3D);
	sc->grafx_attach = bus_read_4(sc->mem, PM_GRAFX);
	if (bootverbose)
		device_printf(dev, "0x304 = 0x%08x, 0x10c = 0x%08x\n",
		    sc->v3d_attach, sc->grafx_attach);

	bcm2712_pm_sysctl_init(sc);
	hwreset_register_ofw_provider(dev);
	return (0);
}

/*
 * hwreset(9) keeps no count of its consumers' handles; a consumer module
 * that uses the reset depends on this one, which keeps this module loaded
 * while the consumer is.  Domain handles are counted, and a held one keeps
 * the driver attached.
 */
static int
bcm2712_pm_detach(device_t dev)
{
	struct bcm2712_pm_softc *sc;
	bool busy;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	busy = sc->handles != 0;
	mtx_unlock(&sc->mtx);
	if (busy)
		return (EBUSY);

	hwreset_unregister_ofw_provider(dev);
	bus_release_resource(dev, SYS_RES_MEMORY, rman_get_rid(sc->mem),
	    sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t bcm2712_pm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2712_pm_probe),
	DEVMETHOD(device_attach,	bcm2712_pm_attach),
	DEVMETHOD(device_detach,	bcm2712_pm_detach),

	/* Reset interface */
	DEVMETHOD(hwreset_map,		bcm2712_pm_hwreset_map),
	DEVMETHOD(hwreset_assert,	bcm2712_pm_hwreset_assert),
	DEVMETHOD(hwreset_is_asserted,	bcm2712_pm_hwreset_is_asserted),

	DEVMETHOD_END
};

static driver_t bcm2712_pm_driver = {
	"bcm2712_pm",
	bcm2712_pm_methods,
	sizeof(struct bcm2712_pm_softc),
};

/* Before the devices it serves, which attach in the default pass. */
EARLY_DRIVER_MODULE(bcm2712_pm, simplebus, bcm2712_pm_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(bcm2712_pm, 1);
