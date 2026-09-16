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

void	rpi5_nn_forward(const struct rpi5_nn_weights *w, const int32_t *in,
	    int32_t *out);

/* Q16 value = scaled integer ratio, with the same floor semantics as >>. */
int32_t	rpi5_nn_ratio(int64_t num, int64_t den);

#endif /* _RPI5_NN_H_ */
