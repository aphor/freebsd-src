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

#ifndef _MHS35REG_H_
#define	_MHS35REG_H_

/*
 * ILI9486 LCD controller register definitions.
 *
 * The MHS35 is a 3.5" 480x320 TFT LCD with ILI9486 controller,
 * typically connected via SPI on Raspberry Pi.
 */

/* Display dimensions */
#define	MHS35_WIDTH		480
#define	MHS35_HEIGHT		320
#define	MHS35_BPP		16	/* RGB565 */
#define	MHS35_STRIDE		(MHS35_WIDTH * (MHS35_BPP / 8))
#define	MHS35_FBSIZE		(MHS35_STRIDE * MHS35_HEIGHT)

/* ILI9486 commands */
#define	ILI9486_NOP		0x00
#define	ILI9486_SWRESET		0x01	/* Software Reset */
#define	ILI9486_SLPIN		0x10	/* Sleep In */
#define	ILI9486_SLPOUT		0x11	/* Sleep Out */
#define	ILI9486_NORON		0x13	/* Normal Display Mode On */
#define	ILI9486_INVOFF		0x20	/* Display Inversion Off */
#define	ILI9486_INVON		0x21	/* Display Inversion On */
#define	ILI9486_DISPOFF		0x28	/* Display Off */
#define	ILI9486_DISPON		0x29	/* Display On */
#define	ILI9486_CASET		0x2A	/* Column Address Set */
#define	ILI9486_PASET		0x2B	/* Page Address Set */
#define	ILI9486_RAMWR		0x2C	/* Memory Write */
#define	ILI9486_MADCTL		0x36	/* Memory Access Control */
#define	ILI9486_COLMOD		0x3A	/* Interface Pixel Format */

/* MADCTL bits */
#define	MADCTL_MY		0x80	/* Row Address Order */
#define	MADCTL_MX		0x40	/* Column Address Order */
#define	MADCTL_MV		0x20	/* Row/Column Exchange */
#define	MADCTL_ML		0x10	/* Vertical Refresh Order */
#define	MADCTL_BGR		0x08	/* BGR Order (vs RGB) */
#define	MADCTL_MH		0x04	/* Horizontal Refresh Order */

/* COLMOD pixel format */
#define	COLMOD_RGB565		0x55	/* 16-bit/pixel */
#define	COLMOD_RGB666		0x66	/* 18-bit/pixel */

/* Landscape orientation: swap rows/columns, flip X */
#define	MHS35_MADCTL_LANDSCAPE	(MADCTL_MV | MADCTL_MX | MADCTL_BGR)

/* SPI clock for pixel data (Hz) */
#define	MHS35_SPI_FREQ		32000000

/* Timing (microseconds) */
#define	MHS35_RESET_US		10000	/* Reset pulse width */
#define	MHS35_RESET_WAIT_US	120000	/* Wait after reset */
#define	MHS35_SLPOUT_WAIT_US	120000	/* Wait after sleep out */

#endif /* _MHS35REG_H_ */
