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
#include "rpi5_nn.h"
#include "rpi5_nn_weights.h"

/*
 * Two controllers share the thermal tick.  The learned controller is the
 * default; the region curve remains selectable so the two can be compared on
 * the same board and kernel, and so a misbehaving learned policy is one
 * sysctl away from a known-good fallback rather than one reboot away.
 */
#define	RPI5_CTRL_CURVE	0
#define	RPI5_CTRL_NN	1

/* Inadequate-cooling messages are rate-limited on top of the edge logic. */
static const struct timeval rpi5_nn_log_interval = { 300, 0 };

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
	 * Low and high water marks of the thermal sensor, used by calibration
	 * to express results as a rise above ambient so a cooler measured on a
	 * warm day compares with one measured on a cold day.
	 *
	 * The minimum is only an ambient proxy while the fan is actually
	 * running.  An idle board sits in region 0 with the fan stopped, so
	 * the floor it reaches is bounded by temp0 - temp0_hyst rather than by
	 * airflow.  A caller wanting the fan-cooled floor must force the fan
	 * on first, for example thresholds="0 0 0 0".
	 */
	uint32_t temp_min;
	uint32_t temp_max;
	bool temp_seen;

	/*
	 * Set while the thermal sensor is not answering.  The fan runs at full
	 * for as long as it is set, because there is no temperature to control
	 * on; see the fail-safe in rpi5_update_fan_state().
	 */
	bool sensor_failed;

	/* Thermal management */
	int thermal_active;
	struct callout thermal_callout;

	/* Controller selection */
	int controller;

	/* Learned controller */
	struct rpi5_nn_state	nn;
	struct rpi5_nn_weights	nn_w;
	struct rpi5_nn_policy	nn_pol;
	struct rpi5_nn_result	nn_last;
	struct timeval		nn_warn_last;
	uint32_t		nn_warnings;
	uint32_t		nn_stalls;
	uint32_t		nn_gated_ticks;
	uint32_t		nn_supervised_ticks;
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
	.fan_temp3_speed = 255,

	.fan_current_state = 0,
	.cpu_temp = 50000,
	.temp_seen = false,

	.controller = RPI5_CTRL_NN,
	/*
	 * Policy defaults, chosen in simulation against 200 unseen plants with
	 * the measured sensor model in the loop (tools/rpi5_fan_nn/train.c).
	 *
	 * tol = 4000 and dd_shift = 3: safety tied with the region curve, 46%
	 * less mean duty, corrections 29 counts/min against the curve's 31, no
	 * reversals of 4 counts or more, and recovery from a supervisor trip to
	 * below duty 50 in 70 s.  tol = 2000 is 2% lower on duty but makes 24%
	 * larger corrections.  An earlier default of 14000 was a mistake: a hold
	 * band that wide left the fan latched at full speed on an idle board after
	 * a supervisor trip, which dunn demonstrated.  crit sits
	 * below the 80 C throttle point so the supervisor acts before the SoC
	 * clock-limits rather than in the same tick.  A 60 tick debounce on the
	 * inadequate-cooling warning caught 95.6% of sustained under-cooled
	 * episodes at 0.39 false alarms per hour, against 1.41 at 15 ticks; the
	 * warning is a log message, and emergencies belong to the supervisor.
	 */
	.nn_pol = {
		.target = 65000,
		.tol = 4000,
		.spec = 75000,
		.crit = 78000,
		.warn_margin = 2000,
		.debounce = 60,
		.rate_up = 24,
		.rate_down = 4,
		.dd_scale = RPI5_NN_DD_SCALE,
		.dd_shift = 3,
	},
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
static int rpi5_sysctl_controller_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_nn_bounded_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_nn_reset_handler(SYSCTL_HANDLER_ARGS);
static int rpi5_sysctl_nn_probe_handler(SYSCTL_HANDLER_ARGS);
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
 * Return the speed slot for a fan region's governing knob, or NULL outside
 * 0..3.  Region k (k >= 1) is governed by speed[k-1]; region 0 has none.
 */
static uint32_t *
rpi5_fan_speed_slot(int level)
{
	switch (level) {
	case 0:		return (&cooling_fan.fan_temp0_speed);
	case 1:		return (&cooling_fan.fan_temp1_speed);
	case 2:		return (&cooling_fan.fan_temp2_speed);
	case 3:		return (&cooling_fan.fan_temp3_speed);
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

/*
 * Duty for a fan region.
 *
 * Four thresholds cut the temperature axis into five regions, and each region
 * above the lowest is governed by one speed knob:
 *
 *	region 0   T <  temp0             fan off
 *	region 1   temp0 <= T < temp1     speed0
 *	region 2   temp1 <= T < temp2     speed1
 *	region 3   temp2 <= T < temp3     speed2
 *	region 4   T >= temp3             speed3
 *
 * The region number equals the state number from rpi5_next_state(), so the
 * hold band that governs leaving a state is also the hold band that governs
 * leaving its region.
 *
 * Take the maximum over every knob the region has passed, not just its own.
 * Nothing validates that the speed table ascends, and with a descending entry
 * a plain "speed = speed[region-1]" would slow the fan down as the die got
 * hotter.  A maximum over a growing prefix is non-decreasing in temperature
 * whatever order the table is in, which is the property actually worth
 * guaranteeing.  Region 0 needs no special case: its prefix is empty, so the
 * maximum is 0 and no knob can lift the fan off its stop.
 */
static uint32_t
rpi5_region_speed(uint32_t region)
{
	uint32_t *sp, speed;
	int i;

	speed = 0;
	for (i = 0; i < 4 && (uint32_t)i < region; i++) {
		sp = rpi5_fan_speed_slot(i);
		if (sp != NULL && *sp > speed)
			speed = *sp;
	}
	return (speed);
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

/*
 * One learned-controller tick.  All of the numeric work, including the gate,
 * the supervisor and the warning debounce, is rpi5_nn_step() -- the same
 * function the trainer evaluated.  This only gathers inputs and reports.
 * Called with the cooling mutex held.
 */
static uint32_t
rpi5_nn_tick(uint32_t temp)
{
	struct rpi5_nn_result *r = &cooling_fan.nn_last;
	int32_t rpm;

	rpm = (int32_t)bcm2712_read_fan_rpm();
	rpi5_nn_step(&cooling_fan.nn, &cooling_fan.nn_w, &cooling_fan.nn_pol,
	    (int32_t)temp, rpm, r);

	if (r->gated)
		cooling_fan.nn_gated_ticks++;
	if (r->supervised)
		cooling_fan.nn_supervised_ticks++;

	if (r->warn_edge) {
		cooling_fan.nn_warnings++;
		if (ratecheck(&cooling_fan.nn_warn_last, &rpi5_nn_log_interval))
			/*
			 * The warning asserts above spec - warn_margin, so the
			 * prediction may sit just under spec.  Say what was
			 * predicted and what the limit is; do not claim it
			 * "exceeds" a limit it may not have reached.
			 */
			printf("rpi5_fan: inadequate cooling: full fan is predicted "
			    "to settle at %d.%d C against a %d C limit; throttling "
			    "is likely under this load\n",
			    r->pred_max / 1000, (r->pred_max % 1000) / 100,
			    cooling_fan.nn_pol.spec / 1000);
	}
	if (r->stall_edge) {
		cooling_fan.nn_stalls++;
		printf("rpi5_fan: fan reports no rotation at duty %d; "
		    "forcing full duty\n", cooling_fan.nn.duty);
	}
	return ((uint32_t)r->duty);
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

	/*
	 * Fail-safe.  Ahead of both controllers, because neither can make a
	 * sound decision without a temperature: the learned controller's
	 * supervisor only trips on a reading at or above crit, and the region
	 * curve would sit in whatever region the stale value names.
	 */
	if (cooling_fan.sensor_failed) {
		speed = 255;
		goto program;
	}

	if (cooling_fan.controller == RPI5_CTRL_NN) {
		speed = rpi5_nn_tick(temp);
		goto program;
	}

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
	 * Below temp0 the fan stops outright.  See rpi5_region_speed() for the
	 * region-to-knob mapping.
	 */
	speed = rpi5_region_speed(cooling_fan.fan_current_state);

program:
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
		/*
		 * The sensor is not answering, so there is no temperature to
		 * control on.  Reusing the last reading -- which is what this
		 * used to do -- is the dangerous choice: it leaves the
		 * controller and its critical-temperature supervisor acting on
		 * a number that stopped tracking the die, and a board can
		 * overheat with every sysctl looking healthy.  Run the fan at
		 * full until the sensor comes back.
		 */
		temp = cooling_fan.cpu_temp;
		if (!cooling_fan.sensor_failed) {
			cooling_fan.sensor_failed = true;
			printf("rpi5_fan: temperature unreadable (error %d); "
			    "forcing the fan to full until it returns\n",
			    error);
		}
	} else {
		if (cooling_fan.sensor_failed) {
			cooling_fan.sensor_failed = false;
			printf("rpi5_fan: temperature readable again "
			    "(%u.%u C); returning to %s control\n",
			    temp / 1000, (temp % 1000) / 100,
			    cooling_fan.controller == RPI5_CTRL_NN ?
			    "learned" : "region curve");
		}
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
	 * collapsing two thresholds is a legitimate way to disable a level.
	 * Raising every threshold to the same value really does disable the
	 * whole fan now: everything below the lowest threshold is region 0.
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

/*
 * hw.rpi5.fan.controller -- 0 selects the region curve, 1 the learned
 * controller.  Switching to the learned controller re-initialises its state
 * starting from the duty the curve was driving, so the handover is bumpless:
 * the delta-sigma accumulator resumes from where the fan already is instead
 * of snapping to zero.
 */
static int
rpi5_sysctl_controller_handler(SYSCTL_HANDLER_ARGS)
{
	int val, error;

	mtx_lock(&cooling_fan.mtx);
	val = cooling_fan.controller;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	if (val != RPI5_CTRL_CURVE && val != RPI5_CTRL_NN)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);
	if (val == RPI5_CTRL_NN && cooling_fan.controller != RPI5_CTRL_NN)
		rpi5_nn_init(&cooling_fan.nn, cooling_fan.nn_pol.target,
		    (int32_t)rpi5_region_speed(cooling_fan.fan_current_state));
	cooling_fan.controller = val;
	mtx_unlock(&cooling_fan.mtx);
	return (0);
}

/*
 * A policy tunable, range-checked to [0, arg2].  arg1 points at the int32
 * field.  The supervisor's crit threshold gets a tighter bound from its
 * caller: set above the hard throttle point it would no longer protect
 * anything.
 */
static int
rpi5_sysctl_nn_bounded_handler(SYSCTL_HANDLER_ARGS)
{
	int32_t *field = (int32_t *)arg1;
	int val, error;

	mtx_lock(&cooling_fan.mtx);
	val = *field;
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	if (val < 0 || val > arg2)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);
	*field = val;
	mtx_unlock(&cooling_fan.mtx);
	return (0);
}

/*
 * hw.rpi5.fan.nn.probe -- run the kernel's forward pass on a supplied input.
 * Write fourteen Q16 integers; read back the three Q16 outputs computed with
 * the live weights.  This exists so the running kernel can be checked against
 * the userland trainer bit-for-bit, rather than trusting that compiling the
 * same source file twice produced the same arithmetic.
 */
static int32_t rpi5_nn_probe_out[NN_N_OUT];

static int
rpi5_sysctl_nn_probe_handler(SYSCTL_HANDLER_ARGS)
{
	char buf[256];
	int32_t in[NN_N_IN], out[NN_N_OUT];
	const char *cp;
	char *ep;
	int error, i;

	mtx_lock(&cooling_fan.mtx);
	snprintf(buf, sizeof(buf), "%d %d %d", rpi5_nn_probe_out[0],
	    rpi5_nn_probe_out[1], rpi5_nn_probe_out[2]);
	mtx_unlock(&cooling_fan.mtx);

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error || req->newptr == NULL)
		return (error);

	cp = buf;
	for (i = 0; i < NN_N_IN; i++) {
		while (*cp == ' ' || *cp == '\t')
			cp++;
		if (*cp == '\0')
			return (EINVAL);
		in[i] = (int32_t)strtol(cp, &ep, 10);
		if (ep == cp)
			return (EINVAL);
		cp = ep;
	}

	mtx_lock(&cooling_fan.mtx);
	rpi5_nn_forward(&cooling_fan.nn_w, in, out);
	for (i = 0; i < NN_N_OUT; i++)
		rpi5_nn_probe_out[i] = out[i];
	mtx_unlock(&cooling_fan.mtx);
	return (0);
}

/* Write 1 to restore the shipped weights. */
static int
rpi5_sysctl_nn_reset_handler(SYSCTL_HANDLER_ARGS)
{
	int val = 0, error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	if (val != 1)
		return (EINVAL);

	mtx_lock(&cooling_fan.mtx);
	cooling_fan.nn_w = rpi5_nn_default_weights;
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
					    "PWM speed for temp0 <= T < temp1 (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed1",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp1_speed, 1, rpi5_sysctl_speed_handler, "IU",
					    "PWM speed for temp1 <= T < temp2 (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed2",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp2_speed, 2, rpi5_sysctl_speed_handler, "IU",
					    "PWM speed for temp2 <= T < temp3 (0-255)");
					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "speed3",
					    CTLTYPE_UINT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    &cooling_fan.fan_temp3_speed, 3, rpi5_sysctl_speed_handler, "IU",
					    "PWM speed for T >= temp3 (0-255)");

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
					    "Current fan region (0-4); 0 = fan off");
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

					SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, SYSCTL_CHILDREN(fan_tree),
					    OID_AUTO, "controller",
					    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
					    NULL, 0, rpi5_sysctl_controller_handler, "I",
					    "Active controller: 0 = region curve, 1 = learned");

					struct sysctl_oid *nn_tree;
					nn_tree = SYSCTL_ADD_NODE(&rpi5_sysctl_ctx,
					    SYSCTL_CHILDREN(fan_tree), OID_AUTO, "nn",
					    CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
					    "Learned fan controller");
					if (nn_tree != NULL) {
						struct sysctl_oid_list *nl = SYSCTL_CHILDREN(nn_tree);
#define	NN_TUNE(name, field, max, desc)					\
	SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, nl, OID_AUTO, name,		\
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,			\
	    &cooling_fan.nn_pol.field, max,				\
	    rpi5_sysctl_nn_bounded_handler, "I", desc)
						NN_TUNE("target", target, 100000,
						    "Temperature the controller settles toward (mC)");
						NN_TUNE("tol", tol, 50000,
						    "Confidence gate half-width (mC)");
						NN_TUNE("spec", spec, 100000,
						    "Temperature never to exceed (mC)");
						NN_TUNE("crit", crit, 85000,
						    "Supervisor forces full fan at or above (mC)");
						NN_TUNE("warn_margin", warn_margin, 30000,
						    "Warn when predicted full-fan temp > spec-margin (mC)");
						NN_TUNE("debounce", debounce, 600,
						    "Ticks a warning must persist before it is logged");
						NN_TUNE("rate_up", rate_up, 255,
						    "Maximum duty rise per tick");
						NN_TUNE("rate_down", rate_down, 255,
						    "Maximum duty fall per tick");
						NN_TUNE("dd_shift", dd_shift, 8,
						    "Request smoothing: EMA alpha = 2^-dd_shift, 0 = off");
#undef NN_TUNE
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "duty", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_last.duty, 0,
						    "Duty commanded on the last tick");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "pred_now", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_last.pred_now, 0,
						    "Predicted equilibrium at current duty (mC)");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "pred_max", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_last.pred_max, 0,
						    "Predicted equilibrium at full duty (mC)");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "gated", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_last.gated, 0,
						    "Confidence gate held duty on the last tick");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "supervised", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_last.supervised, 0,
						    "Supervisor overrode the network on the last tick");
						SYSCTL_ADD_U32(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "warnings", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_warnings, 0,
						    "Inadequate-cooling warnings raised");
						SYSCTL_ADD_U32(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "stalls", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_stalls, 0,
						    "Fan stall faults raised");
						SYSCTL_ADD_U32(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "gated_ticks", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_gated_ticks, 0,
						    "Ticks the confidence gate held duty");
						SYSCTL_ADD_U32(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "supervised_ticks", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn_supervised_ticks, 0,
						    "Ticks the supervisor overrode the network");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "temp_p", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn.pid_temp.error, 0,
						    "Temperature P term: target - temp (mC)");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "temp_i", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn.pid_temp.integral, 0,
						    "Temperature I term (mC*s, clamped)");
						SYSCTL_ADD_INT(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "temp_d", CTLFLAG_RD | CTLFLAG_MPSAFE,
						    &cooling_fan.nn.pid_temp.derivative, 0,
						    "Temperature D term: change over NN_SLOPE_N ticks (mC)");
						SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "reset_weights",
						    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE,
						    NULL, 0, rpi5_sysctl_nn_reset_handler, "I",
						    "Write 1 to restore the shipped weights");
						SYSCTL_ADD_PROC(&rpi5_sysctl_ctx, nl, OID_AUTO,
						    "probe",
						    CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_MPSAFE,
						    NULL, 0, rpi5_sysctl_nn_probe_handler, "A",
						    "Forward pass on 14 written Q16 inputs; reads 3 Q16 outputs");
					}
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

		/*
		 * Learned controller: shipped weights, fresh state from fan-off.
		 * hw.rpi5.fan.controller can be set in loader.conf to boot on the
		 * region curve instead.
		 */
		TUNABLE_INT_FETCH("hw.rpi5.fan.controller", &cooling_fan.controller);
		if (cooling_fan.controller != RPI5_CTRL_CURVE &&
		    cooling_fan.controller != RPI5_CTRL_NN)
			cooling_fan.controller = RPI5_CTRL_NN;
		cooling_fan.nn_w = rpi5_nn_default_weights;
		rpi5_nn_init(&cooling_fan.nn, cooling_fan.nn_pol.target, 0);

		/* Start thermal management */
		mtx_lock(&cooling_fan.mtx);
		cooling_fan.thermal_active = 1;
		callout_reset(&cooling_fan.thermal_callout, hz, rpi5_thermal_tick, NULL);
		mtx_unlock(&cooling_fan.mtx);

		printf("rpi5_fan: Cooling fan thermal management started (%s controller)\n",
		    cooling_fan.controller == RPI5_CTRL_NN ? "learned" : "region curve");
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
MODULE_VERSION(rpi5_fan, 3);
MODULE_DEPEND(rpi5_fan, bcm2712, 1, 1, 1);
