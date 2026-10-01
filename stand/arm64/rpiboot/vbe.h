/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * vbe.h -- what stand/common/gfx_fb.c's non-EFI branch takes from
 * stand/i386/libi386/vbe.h, for this loader.
 *
 * gfx_fb.c includes <vbe.h> whenever it is not built for EFI, for the pixel
 * type of its blit buffers (struct paletteentry, the same B,G,R,X layout as
 * EFI's EFI_GRAPHICS_OUTPUT_BLT_PIXEL) and for pe8, the palette of 8-bit VBE
 * modes.  This loader has neither BIOS nor VBE: the framebuffer is the one
 * the VPU firmware set up, 32 bits per pixel, so pe8 is never used (see
 * rpi_fb.c).  ptov() is the identity with the MMU off.
 */

#ifndef _RPIBOOT_VBE_H_
#define	_RPIBOOT_VBE_H_

struct paletteentry {
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
	uint8_t Reserved;
} __packed;

extern struct paletteentry *pe8;

#define	ptov(pa)	((void *)(uintptr_t)(pa))

#endif /* _RPIBOOT_VBE_H_ */
