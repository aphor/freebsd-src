/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 *
 * Fixed-point multilayer perceptron for the Raspberry Pi 5 fan controller.
 *
 * This file and rpi5_nn.c are compiled into BOTH the kernel module and the
 * userland trainer.  That is the point: the trainer evaluates candidate
 * weights through exactly the integer arithmetic the kernel will execute, so
 * a policy that passes in training cannot silently degrade when quantised.
 * Keep this code free of anything that differs between the two builds --
 * no kernel-only headers in the arithmetic, no floating point anywhere.
 *
 * Floating point is not merely avoided here, it is unavailable: arm64 kernel
 * code is built with -mgeneral-regs-only (sys/conf/kern.mk), which makes any
 * float or double operation a hard compile error.
 *
 * Numeric contract -- the trainer's quantiser depends on every line of it:
 *
 *   Q16 fixed point:  real value = int32 / 65536
 *   dense layer:      z_j = sat(((sum_i w_ji * a_i) >> 16) + b_j)
 *                     products and sum in int64; >> is arithmetic (floor)
 *   hidden layers:    ReLU
 *   output layer:     linear
 *   saturation:       every activation clamped to [-NN_ACT_MAX, NN_ACT_MAX]
 *
 * Overflow bound: weights are clamped to |w| <= NN_W_MAX (32.0, 2^21 in Q16)
 * and activations to |a| <= NN_ACT_MAX (64.0, 2^22), so one product is below
 * 2^43 and a sum over NN_N_IN terms below 2^47 -- far inside int64.  Nothing
 * here divides, so the absent 128-bit division helpers are never needed.
 */

#ifndef _RPI5_NN_H_
#define _RPI5_NN_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdint.h>
#endif

#define	NN_Q		16
#define	NN_ONE		(1 << NN_Q)
#define	NN_ACT_MAX	(64 * NN_ONE)	/* activation saturation, 64.0 */
#define	NN_W_MAX	(32 * NN_ONE)	/* weight magnitude bound, 32.0 */

/* Topology. */
#define	NN_N_IN		14
#define	NN_N_H1		12
#define	NN_N_H2		8
#define	NN_N_OUT	3

/*
 * Visible units, in order.  Each P/I/D triple comes from one pidctrl_classic()
 * instance, so P = setpoint - input: positive means below the setpoint.
 */
enum rpi5_nn_input {
	NN_IN_TEMP_P = 0,	/* current die temperature */
	NN_IN_TEMP_I,
	NN_IN_TEMP_D,
	NN_IN_WMIN_P,		/* sliding-window minimum temperature */
	NN_IN_WMIN_I,
	NN_IN_WMIN_D,
	NN_IN_WMAX_P,		/* sliding-window maximum temperature */
	NN_IN_WMAX_I,
	NN_IN_WMAX_D,
	NN_IN_RPM_P,		/* fan rpm against the rpm the duty implies */
	NN_IN_RPM_I,
	NN_IN_RPM_D,
	NN_IN_AMBIENT,		/* since-reset low water mark */
	NN_IN_DUTY		/* currently commanded duty */
};

/*
 * Outputs are predictions, not an opaque confidence score, so that each can
 * be checked against what the board subsequently does.
 */
enum rpi5_nn_output {
	NN_OUT_DDUTY = 0,	/* duty delta, in duty counts */
	NN_OUT_TEQ_NOW,		/* predicted equilibrium at current duty */
	NN_OUT_TEQ_MAX		/* predicted equilibrium at maximum duty */
};

/*
 * Input and output scaling.  Temperatures are carried in milli-Celsius by the
 * driver and normalised here so every visible unit sits near [-4, 4].
 */
#define	NN_TEMP_SCALE	10000		/* mC per unit: 10 C -> 1.0 */
#define	NN_TEMP_OFFSET	40000		/* mC mapped to 0.0 */
#define	NN_DERIV_SCALE	1000		/* mC/s per unit: 1 C/s -> 1.0 */
#define	NN_RPM_SCALE	10000		/* rpm per unit */
#define	NN_DUTY_SCALE	255		/* full duty -> 1.0 */

struct rpi5_nn_weights {
	int32_t	w1[NN_N_H1][NN_N_IN];
	int32_t	b1[NN_N_H1];
	int32_t	w2[NN_N_H2][NN_N_H1];
	int32_t	b2[NN_N_H2];
	int32_t	w3[NN_N_OUT][NN_N_H2];
	int32_t	b3[NN_N_OUT];
};

/*
 * Scaling of the integral terms.  Each tick adds the current error, so the
 * integral is in mC*s; it is clamped to +/- the bound before scaling.
 */
#define	NN_TEMP_I_BOUND	300000		/* ~30 s at a 10 C error */
#define	NN_TEMP_I_SCALE	100000
#define	NN_RPM_I_BOUND	300000
#define	NN_RPM_I_SCALE	100000

#define	NN_WINDOW	60		/* samples in the sliding window */
#define	NN_SLOPE_N	20		/* ticks the derivative is measured over */
#define	NN_RPM_PER_DUTY	39		/* measured: ~39 rpm per duty count */
#define	NN_STALL_DUTY	16		/* measured: lowest duty that spins */

/*
 * P/I/D state for one signal.  P and I reproduce pidctrl_classic()
 * (sys/kern/subr_pidctrl.c) exactly: error is setpoint - input and the
 * integral is clamped to [-bound, bound].  They are carried here rather than
 * calling pidctrl_classic() because the trainer must compute features
 * bit-exactly the way the kernel does and cannot link a kernel symbol.
 *
 * D deliberately does NOT follow pidctrl_classic(), which takes the change in
 * error since the last call.  The BCM2712 sensor quantises to 550 mC with
 * correlated noise of about 500 mC, so at 1 Hz a one-tick difference is
 * almost entirely noise: on dunn, a flat idle die produced one-tick changes
 * of up to +/-2200 mC on a third of all ticks, and the first network read
 * each flicker as a real 0.55 C/s slope.  D here is the change in error over
 * the last NN_SLOPE_N ticks, and derivative/dspan together give the slope.
 */
struct rpi5_nn_pid {
	int32_t	setpoint;
	int32_t	bound;
	int32_t	error;		/* P */
	int32_t	olderror;
	int32_t	integral;	/* I */
	int32_t	derivative;	/* change in error over dspan ticks */
	int32_t	dspan;		/* ticks the derivative spans, 1..NN_SLOPE_N */
	int32_t	ehist[NN_SLOPE_N];
	int	hpos;
	int	hfill;
};

/*
 * Everything the controller carries between ticks.  Sensor reading and PWM
 * writing are the only parts of a tick that are not in this file.
 */
struct rpi5_nn_state {
	struct rpi5_nn_pid	pid_temp;
	struct rpi5_nn_pid	pid_wmin;
	struct rpi5_nn_pid	pid_wmax;
	struct rpi5_nn_pid	pid_rpm;
	int32_t	win[NN_WINDOW];		/* recent temperatures, mC */
	int	win_head;
	int	win_fill;
	int32_t	ambient;		/* since-reset low water mark, mC */
	int	ambient_seen;
	int32_t	acc_q8;			/* delta-sigma accumulator, duty Q8 */
	int32_t	duty;			/* currently commanded duty, 0..255 */
	int	warn_run;		/* consecutive ticks predicting inadequate */
	int	warned;			/* warning currently asserted */
	int	stall_run;		/* consecutive ticks of a silent fan */
	int	stalled;		/* stall fault currently asserted */
};

/*
 * Tunables for one controller tick.  These are policy, not network: they are
 * applied around the forward pass and can change at runtime without
 * retraining.
 */
struct rpi5_nn_policy {
	int32_t	target;		/* mC the controller settles toward */
	int32_t	tol;		/* confidence gate half-width, mC */
	int32_t	spec;		/* never exceed, mC */
	int32_t	crit;		/* supervisor forces full fan at or above, mC */
	int32_t	warn_margin;	/* warn when predicted max-fan temp > spec-margin */
	int32_t	debounce;	/* ticks a warning must persist before it fires */
	int32_t	rate_up;	/* duty counts per tick, rising */
	int32_t	rate_down;	/* duty counts per tick, falling */
	int32_t	dd_scale;	/* output scale of the duty-delta head */
};

#define	NN_STALL_TICKS	10	/* silent fan at real duty -> fault */

struct rpi5_nn_result {
	int32_t	duty;		/* duty to program this tick */
	int32_t	pred_now;	/* predicted equilibrium at current duty, mC */
	int32_t	pred_max;	/* predicted equilibrium at full duty, mC */
	int	gated;		/* confidence gate held duty this tick */
	int	supervised;	/* supervisor overrode the network this tick */
	int	warn_edge;	/* inadequate-cooling warning just asserted */
	int	stall_edge;	/* fan stall fault just asserted */
};

void	rpi5_nn_forward(const struct rpi5_nn_weights *w, const int32_t *in,
	    int32_t *out);
void	rpi5_nn_init(struct rpi5_nn_state *s, int32_t target_mC,
	    int32_t duty);
void	rpi5_nn_features(struct rpi5_nn_state *s, int32_t temp_mC,
	    int32_t rpm, int32_t target_mC, int32_t *in);
int32_t	rpi5_nn_apply_delta(struct rpi5_nn_state *s, int32_t ddelta_q16,
	    int32_t rate_up, int32_t rate_down);
int32_t	rpi5_nn_temp_mC(int32_t q16);
void	rpi5_nn_step(struct rpi5_nn_state *s, const struct rpi5_nn_weights *w,
	    const struct rpi5_nn_policy *pol, int32_t temp_mC, int32_t rpm,
	    struct rpi5_nn_result *r);

/* Q16 value = scaled integer ratio, with the same floor semantics as >>. */
int32_t	rpi5_nn_ratio(int64_t num, int64_t den);

#endif /* _RPI5_NN_H_ */
