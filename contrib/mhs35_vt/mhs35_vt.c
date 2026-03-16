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

/*
 * vt(4) backend for the MHS35 SPI TFT display.
 *
 * This follows the virtio_gpu.c pattern: each vt drawing callback
 * delegates to the generic vt_fb implementation (which renders into the
 * shadow framebuffer), then flushes the affected region to the SPI display.
 */

#include <sys/cdefs.h>
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/fbio.h>
#include <sys/kernel.h>

#include <dev/vt/vt.h>
#include <dev/vt/hw/fb/vt_fb.h>
#include <dev/vt/colors/vt_termcolors.h>

#include "mhs35reg.h"
#include "mhs35var.h"

static vd_blank_t		mhs35_fb_blank;
static vd_bitblt_text_t		mhs35_fb_bitblt_text;
static vd_bitblt_bmp_t		mhs35_fb_bitblt_bitmap;
static vd_bitblt_argb_t		mhs35_fb_bitblt_argb;
static vd_drawrect_t		mhs35_fb_drawrect;
static vd_setpixel_t		mhs35_fb_setpixel;

struct vt_driver vt_mhs35_driver = {
	.vd_name = "mhs35",
	.vd_init = vt_fb_init,
	.vd_fini = vt_fb_fini,
	.vd_blank = mhs35_fb_blank,
	.vd_bitblt_text = mhs35_fb_bitblt_text,
	.vd_invalidate_text = vt_fb_invalidate_text,
	.vd_bitblt_bmp = mhs35_fb_bitblt_bitmap,
	.vd_bitblt_argb = mhs35_fb_bitblt_argb,
	.vd_drawrect = mhs35_fb_drawrect,
	.vd_setpixel = mhs35_fb_setpixel,
	.vd_postswitch = vt_fb_postswitch,
	.vd_fb_ioctl = vt_fb_ioctl,
	.vd_fb_mmap = NULL,
	.vd_suspend = vt_fb_suspend,
	.vd_resume = vt_fb_resume,
	.vd_priority = VD_PRIORITY_SPECIFIC,
};

VT_DRIVER_DECLARE(vt_mhs35, vt_mhs35_driver);

static void
mhs35_fb_blank(struct vt_device *vd, term_color_t color)
{
	struct mhs35_softc *sc;
	struct fb_info *info;

	info = vd->vd_softc;
	sc = (struct mhs35_softc *)info;

	vt_fb_blank(vd, color);

	mhs35_flush_region(sc, 0, 0, MHS35_WIDTH, MHS35_HEIGHT);
}

static void
mhs35_fb_bitblt_text(struct vt_device *vd, const struct vt_window *vw,
    const term_rect_t *area)
{
	struct mhs35_softc *sc;
	struct fb_info *info;
	int x, y, width, height;

	info = vd->vd_softc;
	sc = (struct mhs35_softc *)info;

	vt_fb_bitblt_text(vd, vw, area);

	x = area->tr_begin.tp_col * vw->vw_font->vf_width +
	    vw->vw_draw_area.tr_begin.tp_col;
	y = area->tr_begin.tp_row * vw->vw_font->vf_height +
	    vw->vw_draw_area.tr_begin.tp_row;
	width = area->tr_end.tp_col * vw->vw_font->vf_width +
	    vw->vw_draw_area.tr_begin.tp_col - x;
	height = area->tr_end.tp_row * vw->vw_font->vf_height +
	    vw->vw_draw_area.tr_begin.tp_row - y;

	mhs35_flush_region(sc, x, y, width, height);
}

static void
mhs35_fb_bitblt_bitmap(struct vt_device *vd, const struct vt_window *vw,
    const uint8_t *pattern, const uint8_t *mask,
    unsigned int width, unsigned int height,
    unsigned int x, unsigned int y, term_color_t fg, term_color_t bg)
{
	struct mhs35_softc *sc;
	struct fb_info *info;

	info = vd->vd_softc;
	sc = (struct mhs35_softc *)info;

	vt_fb_bitblt_bitmap(vd, vw, pattern, mask, width, height, x, y,
	    fg, bg);

	mhs35_flush_region(sc, x, y, width, height);
}

static int
mhs35_fb_bitblt_argb(struct vt_device *vd, const struct vt_window *vw,
    const uint8_t *argb,
    unsigned int width, unsigned int height,
    unsigned int x, unsigned int y)
{

	return (EOPNOTSUPP);
}

static void
mhs35_fb_drawrect(struct vt_device *vd, int x1, int y1, int x2, int y2,
    int fill, term_color_t color)
{
	struct mhs35_softc *sc;
	struct fb_info *info;
	int width, height;

	info = vd->vd_softc;
	sc = (struct mhs35_softc *)info;

	vt_fb_drawrect(vd, x1, y1, x2, y2, fill, color);

	width = x2 - x1 + 1;
	height = y2 - y1 + 1;
	mhs35_flush_region(sc, x1, y1, width, height);
}

static void
mhs35_fb_setpixel(struct vt_device *vd, int x, int y, term_color_t color)
{
	struct mhs35_softc *sc;
	struct fb_info *info;

	info = vd->vd_softc;
	sc = (struct mhs35_softc *)info;

	vt_fb_setpixel(vd, x, y, color);

	mhs35_flush_region(sc, x, y, 1, 1);
}
