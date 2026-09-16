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
