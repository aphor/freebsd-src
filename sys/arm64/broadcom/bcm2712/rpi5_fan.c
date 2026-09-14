/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Contributors
 * All rights reserved.
 *
 * Raspberry Pi 5 Board-Specific Module
 * Provides Pi 5-specific cooling fan control and thermal management
 * Depends on bcm2712 module for hardware access
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/kthread.h>
#include <sys/callout.h>
#include <sys/time.h>

#include "bcm2712_var.h"

/* Debug flag for verbose logging (set to 1 to enable, 0 to disable) */
static int rpi5_debug = 0;

/* Sysctl context for fan control */
static struct sysctl_ctx_list rpi5_sysctl_ctx;

/* Tunable limits: milli-Celsius, except speed which is a raw PWM level. */
#define	RPI5_FAN_TEMP_MAX	120000	/* 120C */
#define	RPI5_FAN_HYST_MAX	10000	/* 10C */
#define	RPI5_FAN_SPEED_MAX	255

/* RPi5 Cooling Fan Control Structure */
struct rpi5_cooling_fan {
	struct mtx mtx;

	/* PWM device (from bcm2712 module) */
	device_t pwm_dev;
	u_int pwm_channel;

	/* Temperature thresholds (in milli-celsius) */
	uint32_t fan_temp0;
	uint32_t fan_temp1;
	uint32_t fan_temp2;
	uint32_t fan_temp3;

	/* Hysteresis values (in milli-celsius) */
	uint32_t fan_temp0_hyst;
	uint32_t fan_temp1_hyst;
	uint32_t fan_temp2_hyst;
	uint32_t fan_temp3_hyst;

	/* PWM speeds (0-255) */
	uint32_t fan_temp0_speed;
	uint32_t fan_temp1_speed;
	uint32_t fan_temp2_speed;
	uint32_t fan_temp3_speed;

	/* Current fan state (0-4) */
	uint32_t fan_current_state;

	/* Current CPU temperature (mC) */
	uint32_t cpu_temp;

	/*
	 * Low and high water marks of the thermal sensor.  The minimum is
	 * the best available proxy for inlet air temperature: the coldest
	 * the die reaches, which happens with the fan running and no load.
	 * Calibration uses it to express results as a rise above ambient,
	 * so a cooler measured on a warm day compares with one measured on
	 * a cold day.
	 */
	uint32_t temp_min;
	uint32_t temp_max;
	bool temp_seen;

	/* Thermal management */
	int thermal_active;
	struct callout thermal_callout;
};

/* Global cooling fan state */
static struct rpi5_cooling_fan cooling_fan = {
	.pwm_channel = 3,	/* Fan uses PWM channel 3 on RP1 */

	/* Default values matching RPi5 defaults */
	.fan_temp0 = 50000,
	.fan_temp1 = 60000,
	.fan_temp2 = 67500,
	.fan_temp3 = 75000,

	.fan_temp0_hyst = 5000,
	.fan_temp1_hyst = 5000,
	.fan_temp2_hyst = 5000,
	.fan_temp3_hyst = 5000,

	.fan_temp0_speed = 75,
	.fan_temp1_speed = 125,
	.fan_temp2_speed = 175,
	.fan_temp3_speed = 250,

	.fan_current_state = 0,
	.cpu_temp = 50000,
	.temp_seen = false,
};

/* Forward declarations */
static void rpi5_thermal_tick(void *arg);
static void rpi5_update_fan_state(void);

/* sysctl handlers */
static int rpi5_sysctl_temp_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_hyst_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_speed_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_current_temp_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_current_state_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_fan_rpm_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_thresholds_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_watermark_handler(SYSCTL_HANDLER_ARGS);
static void rpi5_warn_collapsed(int level, uint32_t lo, uint32_t hi);

/*
 * Return the threshold slot for a fan level, or NULL when the level is
 * outside 0..3.  Callers pass level +/- 1 to reach a neighbour and rely
 * on NULL meaning "no neighbour on that side".
 */
static uint32_t *
rpi5_fan_temp_slot(int level)
{
	switch (level) {
	case 0:		return (&cooling_fan.fan_temp0);
	case 1:		return (&cooling_fan.fan_temp1);
	case 2:		return (&cooling_fan.fan_temp2);
	case 3:		return (&cooling_fan.fan_temp3);
	default:	return (NULL);
	}
}

/*
 * Return the hysteresis slot for a fan level, or NULL outside 0..3.
 */
static uint32_t *
rpi5_fan_hyst_slot(int level)
{
	switch (level) {
	case 0:		return (&cooling_fan.fan_temp0_hyst);
	case 1:		return (&cooling_fan.fan_temp1_hyst);
	case 2:		return (&cooling_fan.fan_temp2_hyst);
	case 3:		return (&cooling_fan.fan_temp3_hyst);
	default:	return (NULL);
	}
}

/*
 * Saturating "threshold - hysteresis".
 *
 * Both operands are uint32_t, so a hysteresis larger than the threshold it
 * applies to wraps the difference to just under UINT32_MAX -- a value no
 * plausible CPU temperature can exceed.  The comparison in
 * rpi5_update_fan_state() would then read false where it should read true,
 * dropping the fan out of its current level instead of holding it there.
 * Clamp the floor at 0 rather than wrapping.
 */
static uint32_t
rpi5_hyst_floor(uint32_t threshold, uint32_t hyst)
{
	return (hyst >= threshold ? 0 : threshold - hyst);
}

/*
 * Select the fan state for a temperature, given the state currently held.
 *
 * Rising edge acts at once: the state is the highest level whose entry
 * threshold the temperature has reached.
 *
 * Falling edge is sticky.  State N is entered at temp[N-1] and released only
 * once the temperature has fallen a full hysteresis below that same
 * threshold -- temp < temp[N-1] - hyst[N-1].  The gap between those two
 * points is the hold band, and it is what stops the fan hunting: a curve
 * whose threshold sits near the cooled equilibrium would otherwise toggle
 * every thermal time constant, because switching the fan on drops the
 * temperature back below the threshold that switched it on.
 *
 * Several levels can be shed in one tick if the temperature fell a long way,
 * but never below the level the rising-edge test alone would choose.
 *
 * The previous formulation tested the hold band only inside a branch already
 * guarded by "temp >= threshold", where it was always true, so no hysteresis
 * setting could change the outcome.
 */
static uint32_t
rpi5_next_state(uint32_t temp, uint32_t state)
{
	uint32_t *entry, *hyst;
	uint32_t up, down;

	if (temp >= cooling_fan.fan_temp3)
		up = 4;
	else if (temp >= cooling_fan.fan_temp2)
		up = 3;
	else if (temp >= cooling_fan.fan_temp1)
		up = 2;
	else if (temp >= cooling_fan.fan_temp0)
		up = 1;
	else
		up = 0;

	if (up >= state)
		return (up);

	down = state;
	while (down > up) {
		entry = rpi5_fan_temp_slot(down - 1);
		hyst = rpi5_fan_hyst_slot(down - 1);
		if (entry == NULL || hyst == NULL)
			break;
		if (temp >= rpi5_hyst_floor(*entry, *hyst))
			break;			/* still inside the hold band */
		down--;
	}
	return (down);
}

/* Check if bcm2712 module is available */
static int
rpi5_check_bcm2712(void)
{
	/* Check if bcm2712 thermal sensor is available */
	if (rpi5_debug)
		printf("rpi5_fan: Checking BCM2712 thermal sensor availability\n");

	/* For now, assume bcm2712 is loaded. We'll verify at runtime. */
	if (rpi5_debug)
		printf("rpi5_fan: BCM2712 thermal sensor detected\n");
	return (0);
}

/* Update fan state based on current temperature */
static void
rpi5_update_fan_state(void)
{
	uint32_t temp, new_state, speed;
	int period = 41566;  /* ~24 kHz period in ns */
	int duty;

	/* Called from thermal_tick which holds mutex via callout_init_mtx */
	temp = cooling_fan.cpu_temp;
	new_state = cooling_fan.fan_current_state;

	/* Thermal control logic with hysteresis */
	new_state = rpi5_next_state(temp, cooling_fan.fan_current_state);

	if (new_state != cooling_fan.fan_current_state) {
		if (rpi5_debug)
			printf("rpi5_fan: Fan state %u->%u (temp %u.%uC)\n",
			    cooling_fan.fan_current_state, new_state,
			    temp / 1000, (temp % 1000) / 100);
		cooling_fan.fan_current_state = new_state;
	}

	/*
	 * Apply PWM unconditionally every tick — not just on state transitions.
	 *
	 * This fixes two bugs:
	 * (a) No-initial-write: without this, the first callout sees
	 *     new_state == current_state (both start at 0) and never programs
	 *     the PWM peripheral, leaving the fan free-running at boot.
	 * (b) Same-state speed change: writing hw.rpi5.fan.speedN while
	 *     current_state == N previously had no effect until the next
	 *     state transition.  Now it takes effect within one second.
	 *
	 * State 0 idles at fan_temp0_speed (minimum always-on speed) rather
	 * than 0 so the fan is never completely stopped — matching Pi 5
	 * active-cooler design intent.
	 *
	 * Run at the highest speed any satisfied level calls for.  Reaching
	 * state S means every threshold below S has been crossed, so the
	 * speeds for levels 0..min(S,3) are all permitted by the current
	 * thresholds; take the largest.  Nothing validates that the speed
	 * table ascends, and with a descending entry the old
	 * "speed = speed[state]" mapping would slow the fan down as the CPU
	 * got hotter.  Choosing the maximum fails toward more cooling.
	 *
	 * For an ascending speed table this selects exactly what the old
	 * per-state mapping did.  States 3 and 4 share fan_temp3_speed:
	 * there are five states and only four speed knobs.
	 */
	speed = cooling_fan.fan_temp0_speed;
	if (cooling_fan.fan_current_state >= 1 &&
	    cooling_fan.fan_temp1_speed > speed)
		speed = cooling_fan.fan_temp1_speed;
	if (cooling_fan.fan_current_state >= 2 &&
	    cooling_fan.fan_temp2_speed > speed)
		speed = cooling_fan.fan_temp2_speed;
	if (cooling_fan.fan_current_state >= 3 &&
	    cooling_fan.fan_temp3_speed > speed)
		speed = cooling_fan.fan_temp3_speed;

	/* Convert speed (0-255) to duty cycle nanoseconds */
	duty = (speed * period) / 255;

	/* Program PWM peripheral via bcm2712 module */
	bcm2712_pwm_set_config(cooling_fan.pwm_channel, period, duty);
	bcm2712_pwm_enable(cooling_fan.pwm_channel, 1);

	if (rpi5_debug)
		printf("rpi5_fan: state=%u temp=%u.%uC speed=%u duty=%dns\n",
		    cooling_fan.fan_current_state,
		    temp / 1000, (temp % 1000) / 100, speed, duty);
}

/* Thermal management callout */
static void
rpi5_thermal_tick(void *arg)
{
	uint32_t temp;
	int error;

	/* Callout is invoked with mutex already held (callout_init_mtx) */
	/* Read CPU temperature from BCM2712 thermal sensor */
	error = bcm2712_read_cpu_temp(&temp);
	if (error) {
		/* Fallback to previous reading on error */
		temp = cooling_fan.cpu_temp;
	} else {
		cooling_fan.cpu_temp = temp;

		/*
		 * Seed both marks from the first real reading rather than the
		 * placeholder cpu_temp, or the minimum would report a
		 * temperature the sensor never produced.
		 */
		if (!cooling_fan.temp_seen) {
			cooling_fan.temp_seen = true;
			cooling_fan.temp_min = temp;
			cooling_fan.temp_max = temp;
		} else {
			if (temp < cooling_fan.temp_min)
				cooling_fan.temp_min = temp;
			if (temp > cooling_fan.temp_max)
				cooling_fan.temp_max = temp;
		}
	}

	/* Update fan based on new temperature */
	rpi5_update_fan_state();

	/* Schedule next tick if thermal management is active */
	if (cooling_fan.thermal_active) {
		callout_reset(&cooling_fan.thermal_callout, hz, rpi5_thermal_tick, NULL);
	}
}

/* sysctl handlers */
static int
rpi5_sysctl_temp_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t *temp_ptr = (uint32_t *)arg1;
	int level = (int)arg2;
	uint32_t *lower, *upper, *hyst;
	uint32_t temp;
	int error;

	mtx_lock(&cooling_fan.mtx);
	temp = *temp_ptr;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &temp, 0, req);
	if (error || !req->newptr)
		return (error);

	/* Validate range: 0-120°C */
	if (temp > RPI5_FAN_TEMP_MAX)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);

	/*
	 * Thresholds must stay non-decreasing.  rpi5_update_fan_state() tests
	 * them from the top down, so a threshold out of order relative to a
	 * neighbour does not misbehave so much as disappear: the level it
	 * guards can never be selected.  Genuinely out-of-order values are
	 * rejected; equal neighbours are permitted but warned about, since
	 * collapsing two thresholds is a legitimate way to disable a level
	 * (or the whole fan, by raising every threshold to the same value).
	 *
	 * Each knob is compared only against its immediate neighbours, both
	 * read under the lock so concurrent writers cannot race past one
	 * another.  That makes the write ORDER significant when moving
	 * several knobs: raise from the top down (temp3 first), lower from
	 * the bottom up (temp0 first).  The opposite order transiently
	 * inverts a pair and is refused.
	 */
	lower = rpi5_fan_temp_slot(level - 1);
	if (lower != NULL && temp < *lower) {
		mtx_unlock(&cooling_fan.mtx);
		return (EINVAL);
	}
	upper = rpi5_fan_temp_slot(level + 1);
	if (upper != NULL && temp > *upper) {
		mtx_unlock(&cooling_fan.mtx);
		return (EINVAL);
	}

	/*
	 * Accepted, but still check the stricter invariant and say what the
	 * operator has given up.  temp[i] == temp[i+1] makes state i+1
	 * unreachable: any temperature that satisfies the lower threshold
	 * satisfies the higher one too, and the top-down ladder picks the
	 * higher state.
	 */
	if (lower != NULL)
		rpi5_warn_collapsed(level - 1, *lower, temp);
	if (upper != NULL)
		rpi5_warn_collapsed(level, temp, *upper);

	hyst = rpi5_fan_hyst_slot(level);
	if (hyst != NULL && *hyst >= temp && temp != 0)
		printf("rpi5_fan: temp%d_hyst (%u mC) >= temp%d (%u mC): "
		    "hysteresis floor clamps to 0\n", level, *hyst, level, temp);

	*temp_ptr = temp;
	mtx_unlock(&cooling_fan.mtx);

	return (0);
}

static int
rpi5_sysctl_hyst_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t *hyst_ptr = (uint32_t *)arg1;
	int level = (int)arg2;
	uint32_t *threshold;
	uint32_t hyst;
	int error;

	mtx_lock(&cooling_fan.mtx);
	hyst = *hyst_ptr;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &hyst, 0, req);
	if (error || !req->newptr)
		return (error);

	/* Validate range: 0-10°C */
	if (hyst > RPI5_FAN_HYST_MAX)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);

	/*
	 * A hysteresis at or above its own threshold makes
	 * rpi5_hyst_floor() saturate at 0, so the level would be held down
	 * to absolute zero -- no lower bound at all.  Permitted, but say so.
	 */
	threshold = rpi5_fan_temp_slot(level);
	if (threshold != NULL && *threshold != 0 && hyst >= *threshold)
		printf("rpi5_fan: temp%d_hyst (%u mC) >= temp%d (%u mC): "
		    "hysteresis floor clamps to 0\n", level, hyst, level,
		    *threshold);

	*hyst_ptr = hyst;
	mtx_unlock(&cooling_fan.mtx);

	return (0);
}

static int
rpi5_sysctl_speed_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t *speed_ptr = (uint32_t *)arg1;
	uint32_t speed;
	int error;

	mtx_lock(&cooling_fan.mtx);
	speed = *speed_ptr;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &speed, 0, req);
	if (error || !req->newptr)
		return (error);

	/* Validate range: 0-255 */
	if (speed > RPI5_FAN_SPEED_MAX)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);
	*speed_ptr = speed;
	mtx_unlock(&cooling_fan.mtx);

	return (0);
}

static int
rpi5_sysctl_current_temp_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t temp;

	mtx_lock(&cooling_fan.mtx);
	temp = cooling_fan.cpu_temp;
	mtx_unlock(&cooling_fan.mtx);

	return (sysctl_handle_int(oidp, &temp, 0, req));
}

static int
rpi5_sysctl_current_state_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t state;

	mtx_lock(&cooling_fan.mtx);
	state = cooling_fan.fan_current_state;
	mtx_unlock(&cooling_fan.mtx);

	return (sysctl_handle_int(oidp, &state, 0, req));
}

static int
rpi5_sysctl_fan_rpm_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t rpm;

	rpm = bcm2712_read_fan_rpm();
	return (sysctl_handle_int(oidp, &rpm, 0, req));
}

/*
 * Warn about a threshold pair that collapses a fan state.  Caller holds the
 * softc mutex.
 */
static void
rpi5_warn_collapsed(int level, uint32_t lo, uint32_t hi)
{
	if (lo == hi)
		printf("rpi5_fan: temp%d == temp%d (%u mC): fan state %d is "
		    "now unreachable\n", level, level + 1, lo, level + 1);
}

/*
 * hw.rpi5.fan.thresholds — read or replace all four thresholds at once.
 *
 * The per-knob temp{0..3} nodes are each validated against their immediate
 * neighbours, which makes write ORDER significant: a target curve that
 * crosses the current one cannot be reached in a single ascending or
 * descending pass, because some intermediate step inverts a pair.  This
 * node takes the whole curve as one string, validates it as a set, and
 * applies it under a single lock — so it either takes effect completely or
 * not at all, whatever the current values happen to be.
 *
 * Format is four ascending milli-Celsius values, e.g.
 *	sysctl hw.rpi5.fan.thresholds="50000 60000 67500 75000"
 */
static int
rpi5_sysctl_thresholds_handler(SYSCTL_HANDLER_ARGS)
{
	char buf[64];
	uint32_t v[4];
	uint32_t *slot, *hyst;
	const char *cp;
	char *ep;
	int error, i;

	mtx_lock(&cooling_fan.mtx);
	snprintf(buf, sizeof(buf), "%u %u %u %u",
	    cooling_fan.fan_temp0, cooling_fan.fan_temp1,
	    cooling_fan.fan_temp2, cooling_fan.fan_temp3);
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error || req->newptr == NULL)
		return (error);

	/* Exactly four unsigned decimal values, whitespace separated. */
	cp = buf;
	for (i = 0; i < 4; i++) {
		while (*cp == ' ' || *cp == '\t')
			cp++;
		if (*cp < '0' || *cp > '9')
			return (EINVAL);
		v[i] = (uint32_t)strtoul(cp, &ep, 10);
		if (ep == cp)
			return (EINVAL);
		cp = ep;
	}
	while (*cp == ' ' || *cp == '\t' || *cp == '\n' || *cp == '\r')
		cp++;
	if (*cp != '\0')
		return (EINVAL);

	for (i = 0; i < 4; i++)
		if (v[i] > RPI5_FAN_TEMP_MAX)
			return (EINVAL);

	/* Non-decreasing, same rule the per-knob nodes enforce. */
	if (v[0] > v[1] || v[1] > v[2] || v[2] > v[3])
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);

	cooling_fan.fan_temp0 = v[0];
	cooling_fan.fan_temp1 = v[1];
	cooling_fan.fan_temp2 = v[2];
	cooling_fan.fan_temp3 = v[3];

	for (i = 0; i < 3; i++)
		rpi5_warn_collapsed(i, v[i], v[i + 1]);
	for (i = 0; i < 4; i++) {
		slot = rpi5_fan_temp_slot(i);
		hyst = rpi5_fan_hyst_slot(i);
		if (slot != NULL && hyst != NULL && *slot != 0 &&
		    *hyst >= *slot)
			printf("rpi5_fan: temp%d_hyst (%u mC) >= temp%d "
			    "(%u mC): hysteresis floor clamps to 0\n",
			    i, *hyst, i, *slot);
	}

	mtx_unlock(&cooling_fan.mtx);
	return (0);
}

/*
 * Low/high water marks.  Writable so a measurement run can restart the
 * tracking: write a high value to temp_min (or a low one to temp_max) and
 * the next tick pulls the mark to the live temperature.
 */
static int
rpi5_sysctl_watermark_handler(SYSCTL_HANDLER_ARGS)
{
	uint32_t *mark = (uint32_t *)arg1;
	uint32_t value;
	int error;

	mtx_lock(&cooling_fan.mtx);
	value = cooling_fan.temp_seen ? *mark : cooling_fan.cpu_temp;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (value > RPI5_FAN_TEMP_MAX)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);
	*mark = value;
	cooling_fan.temp_seen = true;
	mtx_unlock(&cooling_fan.mtx);

	return (0);
}

/* Module load handler */
static int
rpi5_modevent(module_t mod, int event, void *data)
{
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		printf("rpi5_fan: Raspberry Pi 5 board support loading\n");

		/* Initialize mutex and callout */
		mtx_init(&cooling_fan.mtx, "rpi5_cooling", NULL, MTX_DEF);
		callout_init_mtx(&cooling_fan.thermal_callout, &cooling_fan.mtx, 0);

		/* Check bcm2712 module */
		error = rpi5_check_bcm2712();
		if (error) {
			printf("rpi5_fan: Failed to access BCM2712 module\n");
			mtx_destroy(&cooling_fan.mtx);
			return (error);
		}

		/* Create sysctl tree for fan control parameters */
		{
			struct sysctl_oid *tree, *fan_tree;

			sysctl_ctx_init(&rpi5_sysctl_ctx);

			/* Create hw.rpi5 node */
			tree = SYSCTL_ADD_NODE(&rpi5_sysctl_ctx, SYSCTL_STATIC_CHILDREN(_hw),
			    OID_AUTO, "rpi5", CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
			    "Raspberry Pi 5 cooling fan control");

			if (tree != NULL) {
				/* Create hw.rpi5.fan node */
				fan_tree = SYSCTL_ADD_NODE(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(tree),
				    OID_AUTO, "fan", CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
				    "Cooling fan control");

				if (fan_tree != NULL) {
					/* Temperature thresholds (in milli-Celsius) */
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp0",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp0, 0, rpi5_sysctl_temp_handler, "IU",
					    "Level 0 temperature threshold (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp1",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp1, 1, rpi5_sysctl_temp_handler, "IU",
					    "Level 1 temperature threshold (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp2",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp2, 2, rpi5_sysctl_temp_handler, "IU",
					    "Level 2 temperature threshold (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp3",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp3, 3, rpi5_sysctl_temp_handler, "IU",
					    "Level 3 temperature threshold (mC)");

					/* Hysteresis values */
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp0_hyst",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp0_hyst, 0, rpi5_sysctl_hyst_handler, "IU",
					    "Level 0 hysteresis (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp1_hyst",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp1_hyst, 1, rpi5_sysctl_hyst_handler, "IU",
					    "Level 1 hysteresis (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp2_hyst",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp2_hyst, 2, rpi5_sysctl_hyst_handler, "IU",
					    "Level 2 hysteresis (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp3_hyst",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp3_hyst, 3, rpi5_sysctl_hyst_handler, "IU",
					    "Level 3 hysteresis (mC)");

					/* PWM speeds (0-255) */
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed0",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp0_speed, 0, rpi5_sysctl_speed_handler, "IU",
					    "Level 0 PWM speed (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed1",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp1_speed, 0, rpi5_sysctl_speed_handler, "IU",
					    "Level 1 PWM speed (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed2",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp2_speed, 0, rpi5_sysctl_speed_handler, "IU",
					    "Level 2 PWM speed (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed3",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp3_speed, 0, rpi5_sysctl_speed_handler, "IU",
					    "Level 3 PWM speed (0-255)");

					/* Read-only status */
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "cpu_temp",
					    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
					    NULL, 0, rpi5_sysctl_current_temp_handler, "IU",
					    "Current CPU temperature (mC)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "current_state",
					    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
					    NULL, 0, rpi5_sysctl_current_state_handler, "IU",
					    "Current fan state (0-4)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp_min",
					    CTLTYPE_UINT | CTLFLAG_RW |
					    CTLFLAG_MPSAFE,
					    &cooling_fan.temp_min, 0,
					    rpi5_sysctl_watermark_handler, "IU",
					    "Lowest temperature seen (mC); ambient proxy, writable to reset");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "temp_max",
					    CTLTYPE_UINT | CTLFLAG_RW |
					    CTLFLAG_MPSAFE,
					    &cooling_fan.temp_max, 0,
					    rpi5_sysctl_watermark_handler, "IU",
					    "Highest temperature seen (mC); writable to reset");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "thresholds",
					    CTLTYPE_STRING | CTLFLAG_RW |
					    CTLFLAG_MPSAFE,
					    NULL, 0,
					    rpi5_sysctl_thresholds_handler, "A",
					    "All four thresholds (mC), ascending, set atomically");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "rpm",
					    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
					    NULL, 0, rpi5_sysctl_fan_rpm_handler, "IU",
					    "RP1 PWM1 offset 0x3C (CHAN2_PHASE); firmware-preloaded static value, not live fan RPM");
				}
			}
		}

		/*
		 * Immediately assert fan-off before the first thermal tick.
		 * The boot firmware may have left PWM3 in an arbitrary state.
		 * With inverted polarity, duty=0 + enabled → output always HIGH
		 * → fan signal HIGH → fan off.  Doing this here (not in the
		 * callout) avoids the 1-second window where firmware controls
		 * the fan speed.
		 */
		bcm2712_pwm_set_config(cooling_fan.pwm_channel, 41566, 0);
		bcm2712_pwm_enable(cooling_fan.pwm_channel, true);

		/* Start thermal management */
		mtx_lock(&cooling_fan.mtx);
		cooling_fan.thermal_active = 1;
		callout_reset(&cooling_fan.thermal_callout, hz, rpi5_thermal_tick, NULL);
		mtx_unlock(&cooling_fan.mtx);

		printf("rpi5_fan: Cooling fan thermal management started\n");
		break;

	case MOD_UNLOAD:
		printf("rpi5_fan: Raspberry Pi 5 board support unloading\n");

		/* Stop thermal management */
		mtx_lock(&cooling_fan.mtx);
		cooling_fan.thermal_active = 0;
		mtx_unlock(&cooling_fan.mtx);
		callout_drain(&cooling_fan.thermal_callout);

		/* Turn off fan */
		bcm2712_pwm_enable(cooling_fan.pwm_channel, false);

		/* Clean up sysctl tree */
		sysctl_ctx_free(&rpi5_sysctl_ctx);

		/* Clean up synchronization primitives */
		mtx_destroy(&cooling_fan.mtx);
		break;

	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t rpi5_mod = {
	"rpi5_fan",
	rpi5_modevent,
	0
};

DECLARE_MODULE(rpi5_fan, rpi5_mod, SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_VERSION(rpi5_fan, 1);
MODULE_DEPEND(rpi5_fan, bcm2712, 1, 1, 1);
