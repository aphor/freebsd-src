/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * Written from a functional specification of the Raspberry Pi firmware
 * clock interface.
 */

/*
 * Raspberry Pi firmware clocks ("raspberrypi,firmware-clocks").
 *
 * The VideoCore firmware owns the SoC's clocks.  It changes them itself to
 * protect the SoC against overheating and under-voltage, so they are read
 * and set only through its mailbox (rpi_fw), never by programming the clock
 * hardware.  This driver is a clk(9) provider for the clocks the firmware
 * lists; the device-tree cell is the firmware clock id.
 *
 *  - Discovery.  At attach the firmware is asked which clocks exist.  Each
 *    listed clock that this driver exports gets a clock node with no
 *    parent: the firmware's parent ids are reported, not used.
 *  - Range.  Each exported clock's firmware minimum and maximum are read
 *    once, at attach, and bound the rates set on it.  M2MC (id 13) has a
 *    floor of 120 MHz: with that clock at 0 Hz, an access to the HDMI state
 *    machine it drives locks the bus.  The lower end of its range is the
 *    larger of the floor and the firmware minimum, but not above the
 *    firmware maximum, and a clock found below that lower end is set to it
 *    at attach, before any consumer can use it.
 *  - No enable or disable.  Nothing is sent to the firmware when a
 *    consumer enables or disables a clock.  The gate state reported is the
 *    firmware's.
 *  - No caching here.  Every rate read asks the firmware, and a failed
 *    read reports 0 Hz.  The clock framework keeps its own copy of the
 *    rate it last saw, so rpi_fw_clk_query() offers consumers a reading
 *    taken now.
 *  - No rounding.  The firmware rounds, and how is not part of its
 *    interface; the rate it applied is read back, not computed.
 *
 * Rate requests.  clk(9) has one rate per clock, not a range per consumer,
 * so a request is interpreted this way.  The clocks kept as low as their
 * consumers allow (every exported clock but ARM) treat the requested rate
 * as the consumer's minimum: the clock is set to the larger of that rate
 * and its lower bound, and a minimum above the firmware maximum is refused
 * with EINVAL.  ARM is set to the requested rate clamped into its range.
 * The firmware is sent a new rate only when it differs from the clock's
 * current rate, as read from the firmware.
 *
 * The mailbox may not be available when the clock node is probed during
 * boot.  The node is claimed then anyway, so that the generic driver for
 * nodes named "clocks" does not take it, and discovery is tried again at
 * each later bus pass and once more when interrupts are enabled.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <arm64/broadcom/bcm2712/rpi_fw.h>
#include <arm64/broadcom/bcm2712/rpi_fw_clk.h>

#include "clknode_if.h"

/* Mailbox property tags. */
#define	FWCLK_TAG_GET_CLOCKS		0x00010007
#define	FWCLK_TAG_GET_CLOCK_STATE	0x00030001
#define	FWCLK_TAG_GET_CLOCK_RATE	0x00030002
#define	FWCLK_TAG_GET_MAX_CLOCK_RATE	0x00030004
#define	FWCLK_TAG_GET_MIN_CLOCK_RATE	0x00030007
#define	FWCLK_TAG_SET_CLOCK_RATE	0x00038002

/* Clock ids run from 1 to 16; an id of 0 ends the firmware's list. */
#define	FWCLK_NIDS			17
/* "Get clocks" value buffer: up to 17 (parent id, clock id) pairs. */
#define	FWCLK_LIST_PAIRS		17

#define	FWCLK_M2MC_FLOOR		120000000u	/* Hz */

struct fwclk_def {
	const char	*name;		/* firmware name, in lower case */
	const char	*clkname;	/* clock node name, unique system-wide */
	bool		exported;	/* offered to consumers */
	bool		minimise;	/* kept as low as consumers allow */
	uint32_t	floor;		/* lowest rate allowed, Hz, or 0 */
};

static const struct fwclk_def fwclk_defs[FWCLK_NIDS] = {
	[1] =	{ "emmc",	"rpi_fw_emmc",		false,	false,	0 },
	[2] =	{ "uart",	"rpi_fw_uart",		false,	false,	0 },
	[3] =	{ "arm",	"rpi_fw_arm",		true,	false,	0 },
	[4] =	{ "core",	"rpi_fw_core",		true,	true,	0 },
	[5] =	{ "v3d",	"rpi_fw_v3d",		true,	true,	0 },
	[6] =	{ "h264",	"rpi_fw_h264",		false,	false,	0 },
	[7] =	{ "isp",	"rpi_fw_isp",		true,	true,	0 },
	[8] =	{ "sdram",	"rpi_fw_sdram",		false,	false,	0 },
	[9] =	{ "pixel",	"rpi_fw_pixel",		true,	true,	0 },
	[10] =	{ "pwm",	"rpi_fw_pwm",		false,	false,	0 },
	[11] =	{ "hevc",	"rpi_fw_hevc",		true,	true,	0 },
	[12] =	{ "emmc2",	"rpi_fw_emmc2",		false,	false,	0 },
	[13] =	{ "m2mc",	"rpi_fw_m2mc",		true,	true,
		  FWCLK_M2MC_FLOOR },
	[14] =	{ "pixel_bvb",	"rpi_fw_pixel_bvb",	true,	true,	0 },
	[15] =	{ "vec",	"rpi_fw_vec",		true,	true,	0 },
	[16] =	{ "disp",	"rpi_fw_disp",		true,	true,	0 },
};

enum fwclk_state {
	FWCLK_WAITING,		/* the mailbox is not attached yet */
	FWCLK_REGISTERED,	/* the clock domain is registered */
	FWCLK_FAILED,		/* discovery failed; no clocks */
};

struct rpi_fw_clk_softc {
	device_t		dev;
	enum fwclk_state	state;
	struct clkdom		*clkdom;
	struct intr_config_hook	ich;
	bool			ich_pending;
	struct timeval		set_err_last;

	/* What the firmware reported at discovery, by clock id. */
	bool			listed[FWCLK_NIDS];
	uint32_t		parent[FWCLK_NIDS];
	uint32_t		fw_min[FWCLK_NIDS];
	uint32_t		fw_max[FWCLK_NIDS];
	int			fw_range_error[FWCLK_NIDS];
	/* The range rates are clamped into, for exported clocks. */
	uint32_t		lower[FWCLK_NIDS];
	uint32_t		upper[FWCLK_NIDS];
	struct clknode		*clknode[FWCLK_NIDS];
};

struct fwclk_node_softc {
	struct rpi_fw_clk_softc	*sc;
	uint32_t		id;
	uint32_t		lower;
	uint32_t		upper;
	bool			minimise;
};

/*
 * The registered provider.  There is one firmware, and a registered clock
 * domain cannot be removed, so this is set once and never cleared.
 */
static struct rpi_fw_clk_softc *fwclk_sc;

static const struct timeval fwclk_err_interval = { 1, 0 };

/*
 * Mailbox messages.  All words are little-endian.
 */

/* A "get" tag: the request is the clock id; the reply is (id, value). */
static int
fwclk_get(uint32_t tag, uint32_t id, uint32_t *value)
{
	uint32_t buf[2];
	int error;

	buf[0] = htole32(id);
	buf[1] = 0;
	error = rpi_fw_property(tag, buf, sizeof(buf), sizeof(buf[0]));
	if (error != 0)
		return (error);
	*value = le32toh(buf[1]);
	return (0);
}

/* The reply of "set clock rate" is not used; the next read asks again. */
static int
fwclk_set_rate(uint32_t id, uint32_t rate)
{
	uint32_t buf[3];

	buf[0] = htole32(id);
	buf[1] = htole32(rate);
	buf[2] = 0;		/* "skip setting turbo": no */
	return (rpi_fw_property(FWCLK_TAG_SET_CLOCK_RATE, buf, sizeof(buf),
	    sizeof(buf)));
}

static uint32_t
fwclk_rate(uint32_t id)
{
	uint32_t rate;

	if (fwclk_get(FWCLK_TAG_GET_CLOCK_RATE, id, &rate) != 0)
		rate = 0;
	return (rate);
}

/*
 * Clock nodes.
 */

static int
fwclk_node_init(struct clknode *clk, device_t dev)
{

	/*
	 * No parent to select.  The node is created with no parent names,
	 * and clknode_init_parent_idx() has nothing to do for it.
	 */
	return (0);
}

static int
fwclk_node_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct fwclk_node_softc *csc;

	csc = clknode_get_softc(clk);
	*freq = fwclk_rate(csc->id);
	return (0);
}

static int
fwclk_node_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct fwclk_node_softc *csc;
	struct rpi_fw_clk_softc *sc;
	uint64_t rate, want;
	uint32_t cur;
	int error;

	csc = clknode_get_softc(clk);
	sc = csc->sc;
	want = *fout;

	if (csc->minimise) {
		if (want > csc->upper)
			return (EINVAL);
		rate = MAX(want, csc->lower);
	} else {
		rate = MIN(want, csc->upper);
		rate = MAX(rate, csc->lower);
	}
	*done = 1;

	cur = fwclk_rate(csc->id);
	if (csc->minimise && rate == 0) {
		/*
		 * No lower bound at all, so nothing to keep the clock down
		 * to: it keeps its current rate, within its range.
		 */
		rate = MIN(cur, csc->upper);
	}
	*fout = rate;
	if ((flags & CLK_SET_DRYRUN) != 0 || rate == cur)
		return (0);

	error = fwclk_set_rate(csc->id, (uint32_t)rate);
	if (error != 0) {
		if (ratecheck(&sc->set_err_last, &fwclk_err_interval))
			device_printf(sc->dev,
			    "cannot set clock %s to %ju Hz: error %d\n",
			    fwclk_defs[csc->id].name, (uintmax_t)rate, error);
		return (error);
	}
	return (0);
}

/* The firmware is sent nothing: it decides when its clocks run. */
static int
fwclk_node_set_gate(struct clknode *clk, bool enable)
{

	return (0);
}

static int
fwclk_node_get_gate(struct clknode *clk, bool *enabled)
{
	struct fwclk_node_softc *csc;
	uint32_t state;

	csc = clknode_get_softc(clk);
	if (fwclk_get(FWCLK_TAG_GET_CLOCK_STATE, csc->id, &state) != 0)
		state = 0;	/* a failed query reports the clock off */
	*enabled = (state & RPI_FW_CLK_STATE_ON) != 0;
	return (0);
}

static clknode_method_t fwclk_node_methods[] = {
	CLKNODEMETHOD(clknode_init,		fwclk_node_init),
	CLKNODEMETHOD(clknode_recalc_freq,	fwclk_node_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		fwclk_node_set_freq),
	CLKNODEMETHOD(clknode_set_gate,		fwclk_node_set_gate),
	CLKNODEMETHOD(clknode_get_gate,		fwclk_node_get_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rpi_fw_clknode, rpi_fw_clknode_class, fwclk_node_methods,
    sizeof(struct fwclk_node_softc), clknode_class);

/*
 * Consumer interface (rpi_fw_clk.h).
 */

/* The firmware id of one of this provider's clocks, or 0. */
static uint32_t
fwclk_id(clk_t clk)
{
	struct rpi_fw_clk_softc *sc;
	const char *name;
	uint32_t id;

	sc = fwclk_sc;
	if (sc == NULL || clk == NULL)
		return (0);
	name = clk_get_name(clk);
	for (id = 1; id < FWCLK_NIDS; id++) {
		if (sc->clknode[id] != NULL &&
		    strcmp(fwclk_defs[id].clkname, name) == 0)
			return (id);
	}
	return (0);
}

int
rpi_fw_clk_query(clk_t clk, uint32_t *state, uint32_t *rate)
{
	uint32_t id;
	int error;

	id = fwclk_id(clk);
	if (id == 0)
		return (ENXIO);

	error = fwclk_get(FWCLK_TAG_GET_CLOCK_STATE, id, state);
	if (error == 0)
		error = fwclk_get(FWCLK_TAG_GET_CLOCK_RATE, id, rate);
	return (error);
}

int
rpi_fw_clk_max_rate(clk_t clk, uint32_t *rate)
{
	uint32_t id;

	id = fwclk_id(clk);
	if (id == 0)
		return (ENXIO);
	return (fwclk_get(FWCLK_TAG_GET_MAX_CLOCK_RATE, id, rate));
}

/*
 * Discovery and registration.
 */

/* Returns EAGAIN if the mailbox driver has not attached yet. */
static int
fwclk_discover(struct rpi_fw_clk_softc *sc)
{
	uint32_t list[2 * FWCLK_LIST_PAIRS];
	const struct fwclk_def *def;
	uint32_t id, parent;
	int error;
	u_int i;

	memset(list, 0, sizeof(list));
	error = rpi_fw_property(FWCLK_TAG_GET_CLOCKS, list, sizeof(list), 0);
	if (error == ENXIO)
		return (EAGAIN);
	if (error != 0) {
		device_printf(sc->dev, "cannot list the clocks: error %d\n",
		    error);
		return (error);
	}

	/*
	 * The firmware does not promise a terminating pair: the list ends at
	 * a clock id of 0 or at the end of the buffer.
	 */
	for (i = 0; i < FWCLK_LIST_PAIRS; i++) {
		parent = le32toh(list[2 * i]);
		id = le32toh(list[2 * i + 1]);
		if (id == 0)
			break;
		if (id >= FWCLK_NIDS) {
			device_printf(sc->dev,
			    "firmware lists clock id %u, above %u\n", id,
			    FWCLK_NIDS - 1);
			return (EINVAL);
		}
		sc->listed[id] = true;
		sc->parent[id] = parent;
	}

	/*
	 * The range of every listed clock is read once.  Only an exported
	 * clock needs it; for the others it is reported and nothing else.
	 */
	for (id = 1; id < FWCLK_NIDS; id++) {
		if (!sc->listed[id])
			continue;
		def = &fwclk_defs[id];
		error = fwclk_get(FWCLK_TAG_GET_MIN_CLOCK_RATE, id,
		    &sc->fw_min[id]);
		if (error == 0)
			error = fwclk_get(FWCLK_TAG_GET_MAX_CLOCK_RATE, id,
			    &sc->fw_max[id]);
		sc->fw_range_error[id] = error;
		if (error != 0 && def->exported) {
			device_printf(sc->dev,
			    "cannot read the range of clock %s: error %d\n",
			    def->name, error);
			return (error);
		}
		sc->lower[id] = MAX(sc->fw_min[id], def->floor);
		sc->upper[id] = sc->fw_max[id];
		if (error == 0 && sc->lower[id] > sc->upper[id]) {
			/* The firmware allows nothing as high as the floor. */
			device_printf(sc->dev, "clock %s: the firmware maximum, "
			    "%u Hz, is below the floor of %u Hz; the maximum "
			    "is used\n", def->name, sc->upper[id], def->floor);
			sc->lower[id] = sc->upper[id];
		}
	}

	/*
	 * A clock with a floor that is below the lower end of its range is
	 * raised to it, before any consumer can reach the clock.
	 */
	for (id = 1; id < FWCLK_NIDS; id++) {
		def = &fwclk_defs[id];
		if (!sc->listed[id] || !def->exported || def->floor == 0)
			continue;
		if (fwclk_rate(id) >= sc->lower[id])
			continue;
		error = fwclk_set_rate(id, sc->lower[id]);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot raise clock %s to %u Hz: error %d\n",
			    def->name, sc->lower[id], error);
			return (error);
		}
	}
	return (0);
}

static int
fwclk_register(struct rpi_fw_clk_softc *sc)
{
	struct clknode_init_def idef;
	struct fwclk_node_softc *csc;
	const struct fwclk_def *def;
	struct clknode *clk;
	uint32_t id;
	int error;

	sc->clkdom = clkdom_create(sc->dev);
	if (sc->clkdom == NULL)
		return (ENXIO);

	for (id = 1; id < FWCLK_NIDS; id++) {
		def = &fwclk_defs[id];
		if (!sc->listed[id] || !def->exported)
			continue;

		/*
		 * No parent.  An empty parent name, or a parent index naming
		 * no parent, would not be accepted by the clock framework.
		 */
		memset(&idef, 0, sizeof(idef));
		idef.name = def->clkname;
		idef.id = id;
		idef.parent_names = NULL;
		idef.parent_cnt = 0;
		/* The firmware changes these clocks while they run. */
		idef.flags = CLK_NODE_STATIC_STRINGS | CLK_NODE_GLITCH_FREE;

		clk = clknode_create(sc->clkdom, &rpi_fw_clknode_class, &idef);
		if (clk == NULL)
			return (ENXIO);
		csc = clknode_get_softc(clk);
		csc->sc = sc;
		csc->id = id;
		csc->lower = sc->lower[id];
		csc->upper = sc->upper[id];
		csc->minimise = def->minimise;
		if (clknode_register(sc->clkdom, clk) == NULL)
			return (ENXIO);
		sc->clknode[id] = clk;
	}

	/* Consumers can get the clocks once the domain is finished. */
	fwclk_sc = sc;
	error = clkdom_finit(sc->clkdom);
	if (error != 0) {
		device_printf(sc->dev, "cannot register the clocks: error %d\n",
		    error);
		return (error);
	}
	return (0);
}

/*
 * Sysctls: per listed clock, the firmware's values.  The rate and state
 * are asked for on every read; the range is the one read at discovery.
 */

#define	FWCLK_SYSCTL_RATE	0
#define	FWCLK_SYSCTL_STATE	1
#define	FWCLK_SYSCTL_MIN	2
#define	FWCLK_SYSCTL_MAX	3
#define	FWCLK_SYSCTL_ARG(id, what)	((what) << 8 | (id))

static int
fwclk_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_clk_softc *sc;
	uint32_t id, val;
	int error;

	sc = arg1;
	id = arg2 & 0xff;
	switch (arg2 >> 8) {
	case FWCLK_SYSCTL_RATE:
		error = fwclk_get(FWCLK_TAG_GET_CLOCK_RATE, id, &val);
		break;
	case FWCLK_SYSCTL_STATE:
		error = fwclk_get(FWCLK_TAG_GET_CLOCK_STATE, id, &val);
		break;
	case FWCLK_SYSCTL_MIN:
		error = sc->fw_range_error[id];
		val = sc->fw_min[id];
		break;
	case FWCLK_SYSCTL_MAX:
		error = sc->fw_range_error[id];
		val = sc->fw_max[id];
		break;
	default:
		error = EINVAL;
		break;
	}
	if (error != 0)
		return (error);
	return (sysctl_handle_32(oidp, &val, 0, req));
}

static const char *
fwclk_state_name(enum fwclk_state state)
{

	switch (state) {
	case FWCLK_WAITING:
		return ("waiting for the firmware mailbox");
	case FWCLK_REGISTERED:
		return ("registered");
	case FWCLK_FAILED:
		return ("failed");
	}
	return ("unknown");
}

static int
fwclk_sysctl_status(SYSCTL_HANDLER_ARGS)
{
	struct rpi_fw_clk_softc *sc;
	const char *s;

	sc = arg1;
	s = fwclk_state_name(sc->state);
	return (sysctl_handle_string(oidp, __DECONST(char *, s), 0, req));
}

static void
fwclk_sysctl_add(struct rpi_fw_clk_softc *sc, struct sysctl_oid_list *list,
    const char *name, uint32_t id, int what, const char *descr)
{

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(sc->dev), list, OID_AUTO, name,
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FWCLK_SYSCTL_ARG(id, what), fwclk_sysctl, "IU", descr);
}

static void
fwclk_sysctl_clocks(struct rpi_fw_clk_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;
	struct sysctl_oid *node;
	const struct fwclk_def *def;
	uint32_t id;

	ctx = device_get_sysctl_ctx(sc->dev);
	for (id = 1; id < FWCLK_NIDS; id++) {
		if (!sc->listed[id])
			continue;
		def = &fwclk_defs[id];
		node = SYSCTL_ADD_NODE(ctx,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
		    def->name, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
		    "Firmware clock");
		list = SYSCTL_CHILDREN(node);

		SYSCTL_ADD_U32(ctx, list, OID_AUTO, "id", CTLFLAG_RD,
		    NULL, id, "Firmware clock id");
		SYSCTL_ADD_U32(ctx, list, OID_AUTO, "parent", CTLFLAG_RD,
		    &sc->parent[id], 0, "Parent id in the firmware's list");
		fwclk_sysctl_add(sc, list, "fw_min", id, FWCLK_SYSCTL_MIN,
		    "Firmware minimum rate, Hz, read at attach");
		fwclk_sysctl_add(sc, list, "fw_max", id, FWCLK_SYSCTL_MAX,
		    "Firmware maximum rate, Hz, read at attach");
		fwclk_sysctl_add(sc, list, "rate", id, FWCLK_SYSCTL_RATE,
		    "Rate, Hz, read from the firmware now");
		fwclk_sysctl_add(sc, list, "state", id, FWCLK_SYSCTL_STATE,
		    "State word, read from the firmware now "
		    "(bit 0 on, bit 1 absent)");
		if (sc->clknode[id] == NULL)
			continue;
		SYSCTL_ADD_U32(ctx, list, OID_AUTO, "lower", CTLFLAG_RD,
		    &sc->lower[id], 0, "Lowest rate set, Hz");
		SYSCTL_ADD_U32(ctx, list, OID_AUTO, "upper", CTLFLAG_RD,
		    &sc->upper[id], 0, "Highest rate set, Hz");
	}
}

/* Returns EAGAIN while the mailbox driver has not attached. */
static int
fwclk_start(struct rpi_fw_clk_softc *sc)
{
	int error;

	error = fwclk_discover(sc);
	if (error == EAGAIN)
		return (EAGAIN);
	if (error == 0)
		error = fwclk_register(sc);
	if (error != 0) {
		sc->state = FWCLK_FAILED;
		return (error);
	}
	sc->state = FWCLK_REGISTERED;
	fwclk_sysctl_clocks(sc);
	return (0);
}

/* Discovery again, after attach found no mailbox. */
static void
fwclk_retry(struct rpi_fw_clk_softc *sc, bool last)
{
	int error;

	error = fwclk_start(sc);
	if (error == EAGAIN && !last)
		return;
	if (error == EAGAIN) {
		device_printf(sc->dev,
		    "the firmware mailbox did not attach; no clocks\n");
		sc->state = FWCLK_FAILED;
	}
}

static void
fwclk_intrhook(void *arg)
{
	struct rpi_fw_clk_softc *sc;

	sc = arg;
	if (sc->state == FWCLK_WAITING)
		fwclk_retry(sc, true);
	config_intrhook_disestablish(&sc->ich);
	sc->ich_pending = false;
}

/*
 * Newbus.
 */

static int
rpi_fw_clk_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "raspberrypi,firmware-clocks"))
		return (ENXIO);
	device_set_desc(dev, "Raspberry Pi firmware clocks");
	/* Ahead of ofw_clkbus, which takes any node named "clocks". */
	return (BUS_PROBE_DEFAULT);
}

static int
rpi_fw_clk_attach(device_t dev)
{
	struct rpi_fw_clk_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->state = FWCLK_WAITING;

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "status",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    fwclk_sysctl_status, "A", "State of clock discovery");

	error = fwclk_start(sc);
	if (error != 0 && error != EAGAIN && sc->clkdom != NULL) {
		/*
		 * Clock nodes exist and point at this softc; they cannot be
		 * removed, so the device stays attached, without clocks.
		 */
		return (0);
	}
	if (error != EAGAIN)
		return (error);
	if (!cold) {
		device_printf(dev, "the firmware mailbox is not attached\n");
		return (ENXIO);
	}

	/* Retried at each later bus pass, and when interrupts are on. */
	sc->ich.ich_func = fwclk_intrhook;
	sc->ich.ich_arg = sc;
	if (config_intrhook_establish(&sc->ich) == 0)
		sc->ich_pending = true;
	if (bootverbose)
		device_printf(dev, "waiting for the firmware mailbox\n");
	return (0);
}

static void
rpi_fw_clk_new_pass(device_t dev)
{
	struct rpi_fw_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->state != FWCLK_WAITING)
		return;
	fwclk_retry(sc, false);
	if (sc->state != FWCLK_WAITING && sc->ich_pending) {
		config_intrhook_disestablish(&sc->ich);
		sc->ich_pending = false;
	}
}

/*
 * clk(9) cannot remove a clock domain or its nodes, so once discovery has
 * created one the driver stays attached.
 */
static int
rpi_fw_clk_detach(device_t dev)
{
	struct rpi_fw_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->clkdom != NULL)
		return (EBUSY);
	if (sc->ich_pending) {
		config_intrhook_drain(&sc->ich);
		sc->ich_pending = false;
	}
	return (0);
}

static device_method_t rpi_fw_clk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rpi_fw_clk_probe),
	DEVMETHOD(device_attach,	rpi_fw_clk_attach),
	DEVMETHOD(device_detach,	rpi_fw_clk_detach),

	/* Bus interface: called at each bus pass while attached */
	DEVMETHOD(bus_new_pass,		rpi_fw_clk_new_pass),

	DEVMETHOD_END
};

static driver_t rpi_fw_clk_driver = {
	"rpi_fw_clk",
	rpi_fw_clk_methods,
	sizeof(struct rpi_fw_clk_softc),
};

/*
 * The node is a child of the firmware node, which simple_mfd attaches.
 * ofw_clkbus competes for it from BUS_PASS_BUS on, so this driver has to
 * be eligible in that pass too.
 */
EARLY_DRIVER_MODULE(rpi_fw_clk, simple_mfd, rpi_fw_clk_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
EARLY_DRIVER_MODULE(rpi_fw_clk, simplebus, rpi_fw_clk_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(rpi_fw_clk, 1);
MODULE_DEPEND(rpi_fw_clk, rpi_fw, 1, 1, 1);
