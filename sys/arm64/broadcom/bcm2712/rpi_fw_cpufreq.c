/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * rpi_fw_cpufreq -- cpufreq(4) for the Raspberry Pi 5.
 *
 * The four cores share one clock, and the VPU firmware owns it: the
 * firmware sets the PLL and whatever voltage the rate needs, and it only
 * raises the clock above its minimum when the operating system asks.  This
 * driver asks.  It offers the range the firmware reports, in 100 MHz steps,
 * as absolute cpufreq levels, so that dev.cpu.N.freq and powerd(8) work.
 *
 * The clock is the firmware clock provider's "rpi_fw_arm".  Rates are set
 * through the clock framework, so that the provider stays the one owner of
 * the clock, and read back from the firmware itself: a rate the clock
 * framework remembers is not evidence, because the firmware lowers the
 * clock on its own when the board is hot or under-powered.
 *
 * The provider registers its clocks once the firmware mailbox has
 * attached, which may be after the cpu devices have.  The clock is
 * therefore looked up when it is first needed, not at attach.
 *
 * What is not known to this driver: the voltage and power of a level, and
 * how long a change takes.  They are reported as unknown.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/pcpu.h>
#include <sys/smp.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>

#include <arm64/broadcom/bcm2712/rpi_fw.h>
#include <arm64/broadcom/bcm2712/rpi_fw_clk.h>

#include "cpufreq_if.h"

/* Firmware property tags; each takes an id and answers with one value. */
#define	RFC_TAG_GET_CLOCK_RATE		0x00030002
#define	RFC_TAG_GET_VOLTAGE		0x00030003
#define	RFC_TAG_GET_MAX_CLOCK_RATE	0x00030004
#define	RFC_TAG_GET_MIN_CLOCK_RATE	0x00030007

#define	RFC_CLOCK_ARM		3
#define	RFC_VOLTAGE_CORE	1

#define	RFC_CLOCK_NAME		"rpi_fw_arm"

#define	RFC_STEP_HZ		100000000u
#define	RFC_MAX_LEVELS		32
/* A rate read back this close to the one asked for was taken. */
#define	RFC_TOLERANCE_HZ	(RFC_STEP_HZ / 2)

struct rpi_fw_cpufreq_softc {
	device_t	dev;
};

/*
 * One clock for every core, so one state for every instance.  The lock is
 * held across firmware calls, which sleep.
 */
static struct sx	rfc_lock;
SX_SYSINIT(rpi_fw_cpufreq, &rfc_lock, "rpi_fw_cpufreq");

static clk_t		rfc_clk;
static bool		rfc_ready;
static uint32_t		rfc_min;		/* Hz */
static uint32_t		rfc_max;		/* Hz */
static uint32_t		rfc_levels[RFC_MAX_LEVELS];	/* Hz, highest first */
static int		rfc_nlevels;
static u_int		rfc_sets;
static u_int		rfc_set_failures;
static struct timeval	rfc_err_last;
static const struct timeval rfc_err_interval = { 10, 0 };

static SYSCTL_NODE(_hw, OID_AUTO, rpi_fw_cpufreq, CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, "Raspberry Pi firmware CPU clock");

/* 0 leaves the clock as the firmware set it. */
static u_int rfc_attach_freq;
SYSCTL_UINT(_hw_rpi_fw_cpufreq, OID_AUTO, attach_freq, CTLFLAG_RDTUN,
    &rfc_attach_freq, 0,
    "CPU clock asked for once, when the clock is first available, MHz");

static int
rfc_fw_get(uint32_t tag, uint32_t id, uint32_t *value)
{
	uint32_t buf[2];
	int error;

	buf[0] = id;
	buf[1] = 0;
	error = rpi_fw_property(tag, buf, sizeof(buf), sizeof(buf[0]));
	if (error == 0)
		*value = buf[1];
	return (error);
}

static void
rfc_notify(uint32_t rate)
{
	struct pcpu *pc;
	int cpu;

	CPU_FOREACH(cpu) {
		pc = pcpu_find(cpu);
		pc->pc_clock = rate;
	}
}

static int	rfc_set_locked(device_t dev, uint32_t want);

/*
 * Find the clock and its range.  Returns ENXIO until the provider has
 * registered the clock; the caller tries again the next time it is asked.
 */
static int
rfc_init_locked(device_t dev)
{
	uint32_t hz, min, max, rate;
	int error, n;

	sx_assert(&rfc_lock, SA_XLOCKED);
	if (rfc_ready)
		return (0);

	if (rfc_clk == NULL &&
	    clk_get_by_name(dev, RFC_CLOCK_NAME, &rfc_clk) != 0) {
		rfc_clk = NULL;
		return (ENXIO);
	}
	error = rfc_fw_get(RFC_TAG_GET_MIN_CLOCK_RATE, RFC_CLOCK_ARM, &min);
	if (error == 0)
		error = rfc_fw_get(RFC_TAG_GET_MAX_CLOCK_RATE, RFC_CLOCK_ARM,
		    &max);
	if (error == 0)
		error = rfc_fw_get(RFC_TAG_GET_CLOCK_RATE, RFC_CLOCK_ARM,
		    &rate);
	if (error != 0)
		return (error);
	if (min == 0 || max < min) {
		device_printf(dev, "the firmware's range for the CPU clock, "
		    "%u to %u Hz, is not usable\n", min, max);
		return (ENXIO);
	}

	/*
	 * The maximum, then every multiple of the step below it, then the
	 * minimum.
	 */
	n = 0;
	rfc_levels[n++] = max;
	for (hz = rounddown(max - 1, RFC_STEP_HZ); hz > min &&
	    n < RFC_MAX_LEVELS - 1; hz -= RFC_STEP_HZ)
		rfc_levels[n++] = hz;
	if (min != max)
		rfc_levels[n++] = min;
	rfc_nlevels = n;
	rfc_min = min;
	rfc_max = max;
	rfc_ready = true;
	rfc_notify(rate);

	device_printf(dev, "CPU clock %u MHz, %u to %u MHz in %d levels\n",
	    rate / 1000000, min / 1000000, max / 1000000, n);

	if (rfc_attach_freq != 0) {
		hz = MIN(MAX((uint64_t)rfc_attach_freq * 1000000, min), max);
		error = rfc_set_locked(dev, hz);
		device_printf(dev, "hw.rpi_fw_cpufreq.attach_freq: %u MHz "
		    "asked for, error %d\n", hz / 1000000, error);
	}
	return (0);
}

/* The level nearest to a rate. */
static uint32_t
rfc_nearest(uint32_t rate)
{
	uint32_t best, d, dbest;
	int i;

	best = rfc_levels[0];
	dbest = UINT32_MAX;
	for (i = 0; i < rfc_nlevels; i++) {
		d = rate > rfc_levels[i] ? rate - rfc_levels[i] :
		    rfc_levels[i] - rate;
		if (d < dbest) {
			dbest = d;
			best = rfc_levels[i];
		}
	}
	return (best);
}

static int
rfc_set_locked(device_t dev, uint32_t want)
{
	uint32_t d, rate;
	int error;

	sx_assert(&rfc_lock, SA_XLOCKED);

	/*
	 * The clock is shared: the request made for the first core has
	 * already changed it for the others.
	 */
	error = rfc_fw_get(RFC_TAG_GET_CLOCK_RATE, RFC_CLOCK_ARM, &rate);
	if (error != 0)
		return (error);
	if (rate == want)
		return (0);

	rfc_sets++;
	error = clk_set_freq(rfc_clk, want, CLK_SET_ROUND_ANY);
	if (error == 0)
		error = rfc_fw_get(RFC_TAG_GET_CLOCK_RATE, RFC_CLOCK_ARM,
		    &rate);
	if (error == 0) {
		d = rate > want ? rate - want : want - rate;
		if (d > RFC_TOLERANCE_HZ)
			error = EIO;
		rfc_notify(rate);
	}
	if (error != 0) {
		rfc_set_failures++;
		if (ratecheck(&rfc_err_last, &rfc_err_interval))
			device_printf(dev, "%u MHz asked for, the firmware "
			    "reports %u MHz, error %d\n", want / 1000000,
			    rate / 1000000, error);
	}
	return (error);
}

static void
rfc_fill(device_t dev, uint32_t hz, struct cf_setting *set)
{

	memset(set, CPUFREQ_VAL_UNKNOWN, sizeof(*set));
	set->freq = hz / 1000000;
	set->volts = CPUFREQ_VAL_UNKNOWN;
	set->power = CPUFREQ_VAL_UNKNOWN;
	set->lat = CPUFREQ_VAL_UNKNOWN;
	set->dev = dev;
}

static int
rpi_fw_cpufreq_get(device_t dev, struct cf_setting *set)
{
	uint32_t rate;
	int error;

	if (set == NULL)
		return (EINVAL);
	sx_xlock(&rfc_lock);
	error = rfc_init_locked(dev);
	if (error == 0)
		error = rfc_fw_get(RFC_TAG_GET_CLOCK_RATE, RFC_CLOCK_ARM,
		    &rate);
	if (error == 0) {
		rfc_notify(rate);
		rfc_fill(dev, rfc_nearest(rate), set);
	}
	sx_xunlock(&rfc_lock);
	return (error);
}

static int
rpi_fw_cpufreq_set(device_t dev, const struct cf_setting *set)
{
	uint64_t want;
	int error;

	if (set == NULL || set->freq <= 0)
		return (EINVAL);
	want = (uint64_t)set->freq * 1000000;
	sx_xlock(&rfc_lock);
	error = rfc_init_locked(dev);
	if (error == 0 && (want < rfc_min || want > rfc_max))
		error = EINVAL;
	if (error == 0)
		error = rfc_set_locked(dev, (uint32_t)want);
	sx_xunlock(&rfc_lock);
	return (error);
}

static int
rpi_fw_cpufreq_type(device_t dev, int *type)
{

	if (type == NULL)
		return (EINVAL);
	*type = CPUFREQ_TYPE_ABSOLUTE;
	return (0);
}

static int
rpi_fw_cpufreq_settings(device_t dev, struct cf_setting *sets, int *count)
{
	int error, i;

	if (sets == NULL || count == NULL)
		return (EINVAL);
	sx_xlock(&rfc_lock);
	error = rfc_init_locked(dev);
	if (error == 0 && *count < rfc_nlevels)
		error = E2BIG;
	if (error == 0) {
		for (i = 0; i < rfc_nlevels; i++)
			rfc_fill(dev, rfc_levels[i], &sets[i]);
		*count = rfc_nlevels;
	}
	sx_xunlock(&rfc_lock);
	return (error);
}

#define	RFC_SYSCTL_RATE		0
#define	RFC_SYSCTL_MIN		1
#define	RFC_SYSCTL_MAX		2
#define	RFC_SYSCTL_VOLTAGE	3

static int
rfc_sysctl(SYSCTL_HANDLER_ARGS)
{
	device_t dev = arg1;
	uint32_t value;
	int error;

	value = 0;
	sx_xlock(&rfc_lock);
	error = rfc_init_locked(dev);
	if (error == 0) {
		switch (arg2) {
		case RFC_SYSCTL_RATE:
			error = rfc_fw_get(RFC_TAG_GET_CLOCK_RATE,
			    RFC_CLOCK_ARM, &value);
			break;
		case RFC_SYSCTL_MIN:
			value = rfc_min;
			break;
		case RFC_SYSCTL_MAX:
			value = rfc_max;
			break;
		case RFC_SYSCTL_VOLTAGE:
			error = rfc_fw_get(RFC_TAG_GET_VOLTAGE,
			    RFC_VOLTAGE_CORE, &value);
			break;
		}
	}
	sx_xunlock(&rfc_lock);
	if (error != 0)
		return (error);
	return (sysctl_handle_32(oidp, &value, 0, req));
}

static void
rfc_sysctl_add(device_t dev, const char *name, int what, const char *descr)
{

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, name,
	    CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE, dev, what, rfc_sysctl,
	    "IU", descr);
}

/* Runs once interrupts work; by then the provider has had its turn. */
static void
rfc_intrhook(void *arg)
{
	device_t dev = arg;

	sx_xlock(&rfc_lock);
	(void)rfc_init_locked(dev);
	sx_xunlock(&rfc_lock);
}

static void
rpi_fw_cpufreq_identify(driver_t *driver, device_t parent)
{

	if (!ofw_bus_node_is_compatible(OF_finddevice("/"), "brcm,bcm2712"))
		return;
	if (device_find_child(parent, "rpi_fw_cpufreq", DEVICE_UNIT_ANY) !=
	    NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "rpi_fw_cpufreq",
	    device_get_unit(parent)) == NULL)
		device_printf(parent, "add rpi_fw_cpufreq child failed\n");
}

static int
rpi_fw_cpufreq_probe(device_t dev)
{

	if (!ofw_bus_node_is_compatible(OF_finddevice("/"), "brcm,bcm2712"))
		return (ENXIO);
	device_set_desc(dev, "Raspberry Pi firmware CPU clock");
	return (BUS_PROBE_DEFAULT);
}

static int
rpi_fw_cpufreq_attach(device_t dev)
{
	struct rpi_fw_cpufreq_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (device_get_unit(dev) == 0) {
		ctx = device_get_sysctl_ctx(dev);
		list = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
		rfc_sysctl_add(dev, "rate", RFC_SYSCTL_RATE,
		    "CPU clock, Hz, read from the firmware now");
		rfc_sysctl_add(dev, "fw_min", RFC_SYSCTL_MIN,
		    "Lowest CPU clock the firmware allows, Hz");
		rfc_sysctl_add(dev, "fw_max", RFC_SYSCTL_MAX,
		    "Highest CPU clock the firmware allows, Hz");
		rfc_sysctl_add(dev, "core_voltage", RFC_SYSCTL_VOLTAGE,
		    "Core voltage as the firmware reports it now, its units");
		SYSCTL_ADD_UINT(ctx, list, OID_AUTO, "sets", CTLFLAG_RD,
		    &rfc_sets, 0, "Changes of the CPU clock asked for");
		SYSCTL_ADD_UINT(ctx, list, OID_AUTO, "set_failures",
		    CTLFLAG_RD, &rfc_set_failures, 0,
		    "Changes the firmware refused or did not make");
		config_intrhook_oneshot(rfc_intrhook, dev);
	}

	cpufreq_register(dev);
	return (0);
}

static int
rpi_fw_cpufreq_detach(device_t dev)
{

	return (cpufreq_unregister(dev));
}

static device_method_t rpi_fw_cpufreq_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	rpi_fw_cpufreq_identify),
	DEVMETHOD(device_probe,		rpi_fw_cpufreq_probe),
	DEVMETHOD(device_attach,	rpi_fw_cpufreq_attach),
	DEVMETHOD(device_detach,	rpi_fw_cpufreq_detach),

	/* cpufreq interface */
	DEVMETHOD(cpufreq_drv_get,	rpi_fw_cpufreq_get),
	DEVMETHOD(cpufreq_drv_set,	rpi_fw_cpufreq_set),
	DEVMETHOD(cpufreq_drv_type,	rpi_fw_cpufreq_type),
	DEVMETHOD(cpufreq_drv_settings,	rpi_fw_cpufreq_settings),

	DEVMETHOD_END
};

static driver_t rpi_fw_cpufreq_driver = {
	"rpi_fw_cpufreq",
	rpi_fw_cpufreq_methods,
	sizeof(struct rpi_fw_cpufreq_softc),
};

DRIVER_MODULE(rpi_fw_cpufreq, cpu, rpi_fw_cpufreq_driver, 0, 0);
MODULE_VERSION(rpi_fw_cpufreq, 1);
MODULE_DEPEND(rpi_fw_cpufreq, rpi_fw, 1, 1, 1);
MODULE_DEPEND(rpi_fw_cpufreq, rpi_fw_clk, 1, 1, 1);
