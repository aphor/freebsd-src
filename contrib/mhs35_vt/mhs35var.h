/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 The FreeBSD Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _MHS35VAR_H_
#define	_MHS35VAR_H_

#include <sys/types.h>
#include <sys/fbio.h>
#include <dev/gpio/gpiobusvar.h>

#include "mhs35reg.h"

struct mhs35_softc {
	/* Must be first so fb_info* can be cast to softc* */
	struct fb_info	sc_fb;

	device_t	sc_dev;
	gpio_pin_t	sc_dc_pin;	/* Data/Command GPIO */
	gpio_pin_t	sc_rst_pin;	/* Reset GPIO */
	bool		sc_have_vt;	/* vt_allocate() succeeded */
};

/* Defined in mhs35.c, used by mhs35_vt.c */
void	mhs35_flush_region(struct mhs35_softc *sc, uint32_t x, uint32_t y,
	    uint32_t width, uint32_t height);

/* Defined in mhs35_vt.c, used by mhs35.c */
extern struct vt_driver vt_mhs35_driver;

#endif /* _MHS35VAR_H_ */
