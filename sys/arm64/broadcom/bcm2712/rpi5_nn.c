/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * Fixed-point multilayer perceptron for the Raspberry Pi 5 fan controller.
 * See rpi5_nn.h for the numeric contract, which the userland trainer
 * reproduces exactly by compiling this same file.
 */

#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#endif

#include "rpi5_nn.h"

static inline int32_t
rpi5_nn_sat(int64_t v)
{
	if (v > NN_ACT_MAX)
		return (NN_ACT_MAX);
	if (v < -NN_ACT_MAX)
		return (-NN_ACT_MAX);
	return ((int32_t)v);
}

/*
 * One dense layer.  The weight matrix is passed flat, row-major, so the same
 * routine serves every layer regardless of its shape.
 */
static void
rpi5_nn_dense(const int32_t *w, const int32_t *b, const int32_t *in,
    int n_in, int n_out, int32_t *out, int relu)
{
	int64_t acc;
	int32_t v;
	int i, j;

	for (j = 0; j < n_out; j++) {
		acc = 0;
		for (i = 0; i < n_in; i++)
			acc += (int64_t)w[j * n_in + i] * (int64_t)in[i];
		/* Q32 -> Q16.  Arithmetic shift: floor, on both builds. */
		acc >>= NN_Q;
		acc += b[j];
		v = rpi5_nn_sat(acc);
		if (relu && v < 0)
			v = 0;
		out[j] = v;
	}
}

void
rpi5_nn_forward(const struct rpi5_nn_weights *w, const int32_t *in,
    int32_t *out)
{
	int32_t h1[NN_N_H1], h2[NN_N_H2];

	rpi5_nn_dense((const int32_t *)w->w1, w->b1, in, NN_N_IN, NN_N_H1,
	    h1, 1);
	rpi5_nn_dense((const int32_t *)w->w2, w->b2, h1, NN_N_H1, NN_N_H2,
	    h2, 1);
	rpi5_nn_dense((const int32_t *)w->w3, w->b3, h2, NN_N_H2, NN_N_OUT,
	    out, 0);
}

/*
 * num/den as Q16, flooring like an arithmetic shift so that feature scaling
 * behaves identically wherever this file is built.  Division here is 64-bit
 * native on arm64; there is no 128-bit division anywhere in this file.
 */
int32_t
rpi5_nn_ratio(int64_t num, int64_t den)
{
	int64_t q, n;

	if (den == 0)
		return (0);
	n = num * NN_ONE;
	q = n / den;
	/* C truncates toward zero; correct to floor when signs differ. */
	if ((n % den != 0) && ((n < 0) != (den < 0)))
		q--;
	return (rpi5_nn_sat(q));
}

/*
 * The P/I/D lines of pidctrl_classic(), reproduced exactly.  See the comment
 * on struct rpi5_nn_pid for why this is not a call to it.
 */
static void
rpi5_nn_pid_update(struct rpi5_nn_pid *pc, int32_t input)
{
	int64_t integ;
	int32_t error;

	error = pc->setpoint - input;
	pc->olderror = pc->error;
	pc->error = error;
	integ = (int64_t)pc->integral + error;
	if (integ > pc->bound)
		integ = pc->bound;
	if (integ < -pc->bound)
		integ = -pc->bound;
	pc->integral = (int32_t)integ;
	pc->derivative = error - pc->olderror;
}

static void
rpi5_nn_pid_init(struct rpi5_nn_pid *pc, int32_t setpoint, int32_t bound)
{
	pc->setpoint = setpoint;
	pc->bound = bound;
	pc->error = pc->olderror = pc->integral = pc->derivative = 0;
}

void
rpi5_nn_init(struct rpi5_nn_state *s, int32_t target_mC, int32_t duty)
{
	int i;

	rpi5_nn_pid_init(&s->pid_temp, target_mC, NN_TEMP_I_BOUND);
	rpi5_nn_pid_init(&s->pid_wmin, target_mC, NN_TEMP_I_BOUND);
	rpi5_nn_pid_init(&s->pid_wmax, target_mC, NN_TEMP_I_BOUND);
	rpi5_nn_pid_init(&s->pid_rpm, 0, NN_RPM_I_BOUND);
	for (i = 0; i < NN_WINDOW; i++)
		s->win[i] = 0;
	s->win_head = 0;
	s->win_fill = 0;
	s->ambient = 0;
	s->ambient_seen = 0;
	if (duty < 0)
		duty = 0;
	if (duty > 255)
		duty = 255;
	s->duty = duty;
	s->acc_q8 = duty << 8;
}

/* Fan rpm the commanded duty should produce; zero inside the stall band. */
static int32_t
rpi5_nn_expected_rpm(int32_t duty)
{
	return (duty < NN_STALL_DUTY ? 0 : duty * NN_RPM_PER_DUTY);
}

/*
 * Build the fourteen visible units for one tick, in rpi5_nn_input order.
 * Updates the sliding window, the ambient low water mark and all four P/I/D
 * trackers as a side effect, so call it exactly once per tick.
 */
void
rpi5_nn_features(struct rpi5_nn_state *s, int32_t temp_mC, int32_t rpm,
    int32_t target_mC, int32_t *in)
{
	int32_t wmin, wmax;
	int i;

	s->win[s->win_head] = temp_mC;
	s->win_head = (s->win_head + 1) % NN_WINDOW;
	if (s->win_fill < NN_WINDOW)
		s->win_fill++;
	wmin = wmax = temp_mC;
	for (i = 0; i < s->win_fill; i++) {
		if (s->win[i] < wmin)
			wmin = s->win[i];
		if (s->win[i] > wmax)
			wmax = s->win[i];
	}

	if (!s->ambient_seen || temp_mC < s->ambient) {
		s->ambient = temp_mC;
		s->ambient_seen = 1;
	}

	/* A setpoint change takes effect on the next update. */
	s->pid_temp.setpoint = target_mC;
	s->pid_wmin.setpoint = target_mC;
	s->pid_wmax.setpoint = target_mC;
	s->pid_rpm.setpoint = rpi5_nn_expected_rpm(s->duty);

	rpi5_nn_pid_update(&s->pid_temp, temp_mC);
	rpi5_nn_pid_update(&s->pid_wmin, wmin);
	rpi5_nn_pid_update(&s->pid_wmax, wmax);
	rpi5_nn_pid_update(&s->pid_rpm, rpm);

	in[NN_IN_TEMP_P] = rpi5_nn_ratio(s->pid_temp.error, NN_TEMP_SCALE);
	in[NN_IN_TEMP_I] = rpi5_nn_ratio(s->pid_temp.integral, NN_TEMP_I_SCALE);
	in[NN_IN_TEMP_D] = rpi5_nn_ratio(s->pid_temp.derivative, NN_DERIV_SCALE);
	in[NN_IN_WMIN_P] = rpi5_nn_ratio(s->pid_wmin.error, NN_TEMP_SCALE);
	in[NN_IN_WMIN_I] = rpi5_nn_ratio(s->pid_wmin.integral, NN_TEMP_I_SCALE);
	in[NN_IN_WMIN_D] = rpi5_nn_ratio(s->pid_wmin.derivative, NN_DERIV_SCALE);
	in[NN_IN_WMAX_P] = rpi5_nn_ratio(s->pid_wmax.error, NN_TEMP_SCALE);
	in[NN_IN_WMAX_I] = rpi5_nn_ratio(s->pid_wmax.integral, NN_TEMP_I_SCALE);
	in[NN_IN_WMAX_D] = rpi5_nn_ratio(s->pid_wmax.derivative, NN_DERIV_SCALE);
	in[NN_IN_RPM_P] = rpi5_nn_ratio(s->pid_rpm.error, NN_RPM_SCALE);
	in[NN_IN_RPM_I] = rpi5_nn_ratio(s->pid_rpm.integral, NN_RPM_I_SCALE);
	in[NN_IN_RPM_D] = rpi5_nn_ratio(s->pid_rpm.derivative, NN_RPM_SCALE);
	in[NN_IN_AMBIENT] = rpi5_nn_ratio(s->ambient - NN_TEMP_OFFSET,
	    NN_TEMP_SCALE);
	in[NN_IN_DUTY] = rpi5_nn_ratio(s->duty, NN_DUTY_SCALE);
}

/*
 * Delta-sigma output stage.  The network's delta is added to an accumulator
 * held in Q8 and the commanded duty is its truncation.  Keeping the residue
 * in the accumulator rather than discarding it is first-order noise shaping:
 * a request for half a duty step is honoured on average over time instead of
 * being lost to rounding every tick.
 *
 * rate_max bounds the per-tick change in duty counts.  Returns the new duty.
 */
int32_t
rpi5_nn_apply_delta(struct rpi5_nn_state *s, int32_t ddelta_q16,
    int32_t rate_max)
{
	int64_t d_q8, acc;

	/* Q16 duty counts -> Q8, flooring. */
	d_q8 = (int64_t)ddelta_q16 >> 8;
	if (d_q8 > (int64_t)rate_max << 8)
		d_q8 = (int64_t)rate_max << 8;
	if (d_q8 < -((int64_t)rate_max << 8))
		d_q8 = -((int64_t)rate_max << 8);

	acc = (int64_t)s->acc_q8 + d_q8;
	/* Clamp to the representable duty range so the residue cannot wind up
	 * beyond it and delay a later reversal. */
	if (acc < 0)
		acc = 0;
	if (acc > (255 << 8) + 255)
		acc = (255 << 8) + 255;
	s->acc_q8 = (int32_t)acc;
	s->duty = s->acc_q8 >> 8;
	return (s->duty);
}
