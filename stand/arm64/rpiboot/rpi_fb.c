/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * rpi_fb.c -- the VPU firmware's framebuffer: a loader console on it, and a
 * simple-framebuffer node for the kernel.
 *
 * THE FRAMEBUFFER
 *
 * The VPU firmware sets up a framebuffer on the HDMI display before it
 * enters us (its log: "FB0 disp 1 max-fb 1 1920x1080 stride 7680 base
 * 0x3f400000"), and keeps the region out of /memory (whose first bank ends
 * at 0x3f400000; the root node's "memreserve" property names the hole), but
 * no device-tree node describes it.  rpi_fb_probe() asks the property
 * mailbox for it the way sys/arm/broadcom/bcm2835/bcm2835_mbox.c does for
 * the Pi 3/4: the current physical size, then one message that sets that
 * size again with 32 bits per pixel and allocates the buffer (with a
 * buffer already there the firmware returns it) and reads the pitch.  The
 * result is used only if the buffer is outside every /memory range and
 * inside a reservation the firmware declares (rpi_fb_reserved()): memory
 * the kernel will keep its hands off.
 *
 * Channel order: the mailbox's pixel order is 0, "BGR", on dunn: blue,
 * green, red in bytes 0-2 of a pixel, so red in bits 16-23 (x8r8g8b8).
 * The firmware says the same to Linux on the command line it puts in
 * /chosen/bootargs, "bcm2708_fb.fbswap=1", which vendor Linux's bcm2708_fb
 * reads as blue in bits 0-7, green 8-15, red 16-23.  The tag decides; the
 * flag is the fallback.
 *
 * THE LOADER CONSOLE
 *
 * "vidconsole" draws text with stand/common/gfx_fb.c and teken, as the EFI
 * loader's console does, but on the framebuffer directly: no EFI.  It is
 * output only (no keyboard driver in this loader); "uart" stays the
 * console for input, and is first in the console list.
 *
 * THE KERNEL CONSOLE
 *
 * rpi_fb_fdt_node() adds /chosen/framebuffer@<base>, compatible
 * "simple-framebuffer", to the device tree the kernel gets, which is what
 * vt_simplefb looks for.  loader.rc sets boot_serial and boot_multicons, so
 * the PL011 stays the kernel's primary console and HDMI is the second.
 */

#include <stand.h>
#include <sys/param.h>

#include <bootstrap.h>
#include <teken.h>
#include <gfx_fb.h>
#include <libfdt.h>

#include "librpiboot.h"

extern uint64_t rpi_dtb_pa;	/* from start.S: x0 at entry */

/* Property tags (vendor raspberrypi-firmware.h, bcm2835_mbox_prop.h). */
#define	TAG_ALLOCATE_BUFFER	0x00040001U
#define	TAG_GET_PHYSICAL_W_H	0x00040003U
#define	TAG_GET_DEPTH		0x00040005U
#define	TAG_GET_PIXEL_ORDER	0x00040006U
#define	TAG_GET_PITCH		0x00040008U
#define	TAG_SET_PHYSICAL_W_H	0x00048003U
#define	TAG_SET_VIRTUAL_W_H	0x00048004U
#define	TAG_SET_DEPTH		0x00048005U
#define	TAG_SET_ALPHA_MODE	0x00048007U
#define	TAG_SET_VIRTUAL_OFFSET	0x00048009U
#define	ALPHA_MODE_IGNORED	2
#define	TAG_RESPONSE		0x80000000U

#define	RPI_FB_BPP		32

struct rpi_fb {
	bool		probed;
	bool		valid;
	const char	*why;		/* when not valid */
	uint32_t	width, height, depth, pitch;
	uint32_t	cur_depth;	/* depth the firmware had */
	int		order;		/* GET_PIXEL_ORDER, -1 if unknown */
	bool		swap;		/* bcm2708_fb.fbswap=1 */
	uint64_t	base, size;
	uint64_t	rsv_base, rsv_size;
	uint32_t	mask_r, mask_g, mask_b;
	const char	*format;	/* simple-framebuffer name, or NULL */
};

static struct rpi_fb fb;

/*
 * Measurement (fbtest): the generic timer, and a trace of the console's
 * set-up steps printed on the UART as each one is reached, so that a step
 * that never returns is the one after the last line printed.
 */
static bool fb_trace;
static uint64_t fb_t0;

static uint64_t
fb_now_us(void)
{
	uint64_t cnt, frq;

	__asm __volatile("isb; mrs %0, cntpct_el0" : "=r" (cnt));
	__asm __volatile("mrs %0, cntfrq_el0" : "=r" (frq));
	return (frq != 0 ? cnt / (frq / 1000000) : 0);
}

#define	FB_TRACE(msg)	do {						\
	if (fb_trace)							\
		printf("  fbtest %8ju us  %s\n",			\
		    (uintmax_t)(fb_now_us() - fb_t0), (msg));		\
} while (0)

/* gfx_fb.c's non-EFI branch wants these; see vbe.h. */
struct paletteentry *pe8 = NULL;
struct vesa_edid_info *edid_info = NULL;

static const void *
rpi_fb_dtb(void)
{
	const void *dtb = (const void *)(uintptr_t)rpi_dtb_pa;

	if (rpi_dtb_pa == 0 || fdt_check_header(dtb) != 0)
		return (NULL);
	return (dtb);
}

static uint64_t
rpi_fb_cells(const fdt32_t *p, int n)
{
	uint64_t v = 0;

	while (n-- > 0)
		v = (v << 32) | fdt32_to_cpu(*p++);
	return (v);
}

/*
 * Is [base, base + size) memory the kernel will leave alone?  Two facts,
 * both from the firmware's device tree (measured on dunn, 2026-09-30):
 *
 * - it must overlap no /memory range (the first bank ends at 0x3f400000,
 *   where the framebuffer starts) -- this is what keeps the kernel off it;
 * - and it must be inside a reservation the firmware declares: the header's
 *   memory-reservation map (empty on this firmware) or the root node's
 *   "memreserve" property, <0x3f400000 0xc00000>, pairs of one address
 *   and one size cell.
 */
static bool
rpi_fb_reserved(uint64_t base, uint64_t size)
{
	const void *dtb;
	const fdt32_t *p;
	uint64_t a, s;
	int ac, sc, i, len, n, node;
	bool declared;

	if ((dtb = rpi_fb_dtb()) == NULL)
		return (false);

	/* Not RAM the kernel is given. */
	ac = fdt_address_cells(dtb, 0);
	sc = fdt_size_cells(dtb, 0);
	if (ac < 1 || ac > 2 || sc < 1 || sc > 2)
		return (false);
	for (node = fdt_first_subnode(dtb, 0); node >= 0;
	    node = fdt_next_subnode(dtb, node)) {
		p = fdt_getprop(dtb, node, "device_type", &len);
		if (p == NULL || strcmp((const char *)p, "memory") != 0)
			continue;
		p = fdt_getprop(dtb, node, "reg", &len);
		if (p == NULL)
			continue;
		n = len / (int)sizeof(*p) / (ac + sc);
		for (i = 0; i < n; i++, p += ac + sc) {
			a = rpi_fb_cells(p, ac);
			s = rpi_fb_cells(p + ac, sc);
			if (base < a + s && a < base + size)
				return (false);
		}
	}

	/* Reserved by the firmware. */
	declared = false;
	n = fdt_num_mem_rsv(dtb);
	for (i = 0; i < n && !declared; i++) {
		if (fdt_get_mem_rsv(dtb, i, &a, &s) == 0 &&
		    base >= a && base + size <= a + s) {
			fb.rsv_base = a;
			fb.rsv_size = s;
			declared = true;
		}
	}
	p = fdt_getprop(dtb, 0, "memreserve", &len);
	for (i = 0; p != NULL && i + 1 < len / (int)sizeof(*p) && !declared;
	    i += 2) {
		a = fdt32_to_cpu(p[i]);
		s = fdt32_to_cpu(p[i + 1]);
		if (base >= a && base + size <= a + s) {
			fb.rsv_base = a;
			fb.rsv_size = s;
			declared = true;
		}
	}
	return (declared);
}

static bool
rpi_fb_bootargs_swap(void)
{
	const void *dtb;
	const char *args;
	int len, node;

	if ((dtb = rpi_fb_dtb()) == NULL)
		return (false);
	if ((node = fdt_path_offset(dtb, "/chosen")) < 0)
		return (false);
	args = fdt_getprop(dtb, node, "bootargs", &len);
	if (args == NULL || len <= 0 || args[len - 1] != '\0')
		return (false);
	return (strstr(args, "bcm2708_fb.fbswap=1") != NULL);
}

/*
 * One message, as bcm2835_mbox_fb_init() builds it: set the size again,
 * 32 bpp, alpha ignored, allocate, get the pitch.
 */
static int
rpi_fb_setup(uint32_t w, uint32_t h)
{
	static uint32_t buf[48] __aligned(16);
	uint32_t *alloc, *depth, *pitch;
	int error, n;

	n = 2;
#define	TAG(t, len)	do { buf[n++] = (t); buf[n++] = (len); buf[n++] = (len); } while (0)
	TAG(TAG_SET_PHYSICAL_W_H, 8);	buf[n++] = w; buf[n++] = h;
	TAG(TAG_SET_VIRTUAL_W_H, 8);	buf[n++] = w; buf[n++] = h;
	TAG(TAG_SET_VIRTUAL_OFFSET, 8);	buf[n++] = 0; buf[n++] = 0;
	TAG(TAG_SET_DEPTH, 4);		depth = &buf[n]; buf[n++] = RPI_FB_BPP;
	TAG(TAG_SET_ALPHA_MODE, 4);	buf[n++] = ALPHA_MODE_IGNORED;
	buf[n++] = TAG_ALLOCATE_BUFFER; buf[n++] = 8; buf[n++] = 4;
	alloc = &buf[n];		buf[n++] = PAGE_SIZE; buf[n++] = 0;
	buf[n++] = TAG_GET_PITCH; buf[n++] = 4; buf[n++] = 0;
	pitch = &buf[n];		buf[n++] = 0;
	buf[n++] = 0;			/* end tag */
#undef TAG
	buf[0] = n * 4;
	buf[1] = 0;

	if ((error = rpi_mbox_property(buf)) != 0)
		return (error);
	if ((alloc[-1] & TAG_RESPONSE) == 0 || (pitch[-1] & TAG_RESPONSE) == 0)
		return (ENOTSUP);
	fb.base = alloc[0];
	fb.size = alloc[1];
	fb.pitch = pitch[0];
	fb.depth = *depth;		/* SET_DEPTH answers with the depth set */
	return (0);
}

/*
 * Find the framebuffer.  Once: cons_probe() calls every probe twice.
 * Prints nothing, because it may run before there is a console.
 */
int
rpi_fb_probe(void)
{
	uint32_t v[2];

	if (fb.probed)
		return (fb.valid ? 0 : ENXIO);
	fb.probed = true;
	fb.order = -1;

	v[0] = v[1] = 0;
	if (rpi_mbox_tag(TAG_GET_PHYSICAL_W_H, v, 8, 0) != 0) {
		fb.why = "the firmware did not report a display size";
		return (ENXIO);
	}
	fb.width = v[0];
	fb.height = v[1];
	if (fb.width == 0 || fb.height == 0) {
		fb.why = "no display (the firmware reports 0x0)";
		return (ENXIO);
	}
	v[0] = 0;
	if (rpi_mbox_tag(TAG_GET_DEPTH, v, 4, 0) == 0)
		fb.cur_depth = v[0];
	v[0] = 0;
	if (rpi_mbox_tag(TAG_GET_PIXEL_ORDER, v, 4, 0) == 0)
		fb.order = (int)v[0];

	if (rpi_fb_setup(fb.width, fb.height) != 0) {
		fb.why = "the firmware did not allocate the framebuffer";
		return (ENXIO);
	}
	if (fb.depth != RPI_FB_BPP) {
		fb.why = "the firmware did not take 32 bits per pixel";
		return (ENXIO);
	}
	if (fb.base == 0 || fb.pitch < fb.width * 4 ||
	    fb.size < (uint64_t)fb.pitch * fb.height) {
		fb.why = "the firmware's framebuffer geometry is inconsistent";
		return (ENXIO);
	}
	if (!rpi_fb_reserved(fb.base, fb.size)) {
		fb.why = "the framebuffer is not reserved memory outside /memory";
		return (ENXIO);
	}

	/*
	 * Byte order in memory: the firmware's pixel order 0, "BGR", is blue,
	 * green, red in bytes 0-2, i.e. red in bits 16-23 of the 32-bit
	 * pixel; the bootargs flag says the same thing to Linux.  The tag
	 * decides; the flag is the fallback when the tag is not answered.
	 */
	fb.swap = rpi_fb_bootargs_swap();
	if (fb.order == 0 || (fb.order < 0 && fb.swap)) {
		fb.mask_r = 0x00ff0000;
		fb.mask_g = 0x0000ff00;
		fb.mask_b = 0x000000ff;
		fb.format = "x8r8g8b8";
	} else {
		/* Red in the low byte: no simple-framebuffer name for it. */
		fb.mask_r = 0x000000ff;
		fb.mask_g = 0x0000ff00;
		fb.mask_b = 0x00ff0000;
		fb.format = NULL;
	}
	fb.valid = true;
	return (0);
}

/*
 * The device tree the kernel gets: /chosen/framebuffer@<base>.  Called with
 * a writable copy of the blob (see rpi_fdt.c).  Returns 0 when there is
 * nothing to add.
 */
int
rpi_fb_fdt_node(void *dtb)
{
	char name[32];
	fdt32_t reg[4];
	int chosen, node;

	if (!fb.valid || fb.format == NULL)
		return (0);
	if (getenv("rpi_fb_no_fdt") != NULL)
		return (0);

	if ((chosen = fdt_path_offset(dtb, "/chosen")) < 0 &&
	    (chosen = fdt_add_subnode(dtb, 0, "chosen")) < 0)
		return (chosen);
	/* Explicit, rather than the defaults ofw_reg_to_paddr() assumes. */
	if (fdt_getprop(dtb, chosen, "#address-cells", NULL) == NULL) {
		fdt_setprop_u32(dtb, chosen, "#address-cells", 2);
		fdt_setprop_u32(dtb, chosen, "#size-cells", 2);
		fdt_setprop(dtb, chosen, "ranges", NULL, 0);
	}

	snprintf(name, sizeof(name), "framebuffer@%jx", (uintmax_t)fb.base);
	if ((node = fdt_add_subnode(dtb, chosen, name)) < 0)
		return (node);
	reg[0] = cpu_to_fdt32(fb.base >> 32);
	reg[1] = cpu_to_fdt32(fb.base & 0xffffffff);
	reg[2] = cpu_to_fdt32(fb.size >> 32);
	reg[3] = cpu_to_fdt32(fb.size & 0xffffffff);
	fdt_setprop_string(dtb, node, "compatible", "simple-framebuffer");
	fdt_setprop(dtb, node, "reg", reg, sizeof(reg));
	fdt_setprop_u32(dtb, node, "width", fb.width);
	fdt_setprop_u32(dtb, node, "height", fb.height);
	fdt_setprop_u32(dtb, node, "stride", fb.pitch);
	fdt_setprop_string(dtb, node, "format", fb.format);
	return (fdt_setprop_string(dtb, node, "status", "okay"));
}

static void
rpi_fb_print_rsv(void)
{
	const void *dtb;
	uint64_t addr, size;
	int i, n;

	if ((dtb = rpi_fb_dtb()) == NULL)
		return;
	const fdt32_t *p;
	int len;

	n = fdt_num_mem_rsv(dtb);
	for (i = 0; i < n; i++)
		if (fdt_get_mem_rsv(dtb, i, &addr, &size) == 0)
			printf("rsvmap      0x%jx + 0x%jx\n", (uintmax_t)addr,
			    (uintmax_t)size);
	p = fdt_getprop(dtb, 0, "memreserve", &len);
	for (i = 0; p != NULL && i + 1 < len / (int)sizeof(*p); i += 2)
		printf("memreserve  0x%x + 0x%x (root property)\n",
		    fdt32_to_cpu(p[i]), fdt32_to_cpu(p[i + 1]));
}

static int
command_fbinfo(int argc __unused, char *argv[] __unused)
{

	(void)rpi_fb_probe();
	if (fb.width != 0)
		printf("display     %u x %u (firmware depth %u, pixel order %d"
		    " = %s)\n", fb.width, fb.height, fb.cur_depth, fb.order,
		    fb.order == 1 ? "RGB" : fb.order == 0 ? "BGR" : "?");
	if (!fb.valid) {
		printf("no framebuffer: %s\n", fb.why);
		if (fb.base != 0 || fb.size != 0)
			printf("firmware    base 0x%jx, size %ju, pitch %u, "
			    "depth %u\n", (uintmax_t)fb.base,
			    (uintmax_t)fb.size, fb.pitch, fb.depth);
		rpi_fb_print_rsv();
		return (CMD_OK);
	}
	printf("buffer      0x%jx, %ju bytes, pitch %u, %u bpp\n",
	    (uintmax_t)fb.base, (uintmax_t)fb.size, fb.pitch, fb.depth);
	printf("reserved    inside 0x%jx + 0x%jx, outside /memory\n",
	    (uintmax_t)fb.rsv_base, (uintmax_t)fb.rsv_size);
	printf("channels    order %d, fbswap=%d: R 0x%08x G 0x%08x B 0x%08x"
	    " (%s)\n", fb.order, fb.swap, fb.mask_r, fb.mask_g, fb.mask_b,
	    fb.format != NULL ? fb.format : "no simple-framebuffer format");
	if (gfx_state.tg_fb_type != FB_TEXT)
		printf("console     %u x %u characters, font %u x %u\n",
		    gfx_state.tg_tp.tp_col, gfx_state.tg_tp.tp_row,
		    gfx_state.tg_font.vf_width, gfx_state.tg_font.vf_height);
	return (CMD_OK);
}
COMMAND_SET(fbinfo, "fbinfo", "show the firmware framebuffer", command_fbinfo);

/*
 * fbtest: the framebuffer and the console set-up one step at a time, timed,
 * each step announced before it runs.
 */
static int	fbcons_init(int);

static void
fbtest_step(const char *msg)
{
	printf("  fbtest %8ju us  %s\n", (uintmax_t)(fb_now_us() - fb_t0),
	    msg);
}

static int
command_fbtest(int argc, char *argv[])
{
	volatile uint32_t *px;
	uint32_t v, sum, x, y;
	uint64_t t;

	if (rpi_fb_probe() != 0) {
		printf("no framebuffer: %s\n", fb.why);
		return (CMD_OK);
	}
	px = (volatile uint32_t *)(uintptr_t)fb.base;
	fb_t0 = fb_now_us();

	fbtest_step("write pixel (0,0) white");
	px[0] = 0x00ffffff;
	fbtest_step("read it back");
	v = px[0];
	printf("  fbtest              read 0x%08x\n", v);

	fbtest_step("fill row 0 red (1920 pixels)");
	t = fb_now_us();
	for (x = 0; x < fb.width; x++)
		px[x] = fb.mask_r;
	printf("  fbtest              %ju us per row written\n",
	    (uintmax_t)(fb_now_us() - t));

	fbtest_step("fill rows 1-99 green");
	t = fb_now_us();
	for (y = 1; y < 100; y++)
		for (x = 0; x < fb.width; x++)
			px[y * (fb.pitch / 4) + x] = fb.mask_g;
	printf("  fbtest              %ju us for 99 rows\n",
	    (uintmax_t)(fb_now_us() - t));

	fbtest_step("read row 50 (1920 pixels)");
	t = fb_now_us();
	for (sum = 0, x = 0; x < fb.width; x++)
		sum += px[50 * (fb.pitch / 4) + x];
	printf("  fbtest              %ju us per row read (sum 0x%08x)\n",
	    (uintmax_t)(fb_now_us() - t), sum);

	if (argc > 1 && strcmp(argv[1], "raw") == 0)
		return (CMD_OK);

	fbtest_step("fbcons_init (the console's own set-up, traced)");
	fb_trace = true;
	v = fbcons_init(0);
	fb_trace = false;
	fbtest_step(v == 0 ? "fbcons_init returned 0" :
	    "fbcons_init failed");
	if (v != 0)
		return (CMD_OK);

	fbtest_step("teken: one line of text");
	teken_input(&gfx_state.tg_teken, "fbtest: FreeBSD rpiboot HDMI\r\n",
	    31);
	fbtest_step("done");
	return (CMD_OK);
}
COMMAND_SET(fbtest, "fbtest", "time the framebuffer console set-up",
    command_fbtest);

/*
 * The console.
 */
static void	fbcons_probe(struct console *);
static void	fbcons_putchar(int);
static int	fbcons_getchar(void);
static int	fbcons_poll(void);

struct console rpi_fb_console = {
	.c_name = "vidconsole",
	.c_desc = "HDMI framebuffer",
	.c_flags = 0,
	.c_probe = fbcons_probe,
	.c_init = fbcons_init,
	.c_out = fbcons_putchar,
	.c_in = fbcons_getchar,
	.c_ready = fbcons_poll,
};

static tf_bell_t	fbcons_bell;
static tf_respond_t	fbcons_respond;

static teken_funcs_t tfx = {
	.tf_bell	= fbcons_bell,
	.tf_cursor	= gfx_fb_cursor,
	.tf_putchar	= gfx_fb_putchar,
	.tf_fill	= gfx_fb_fill,
	.tf_copy	= gfx_fb_copy,
	.tf_param	= gfx_fb_param,
	.tf_respond	= fbcons_respond,
};

static bool fbcons_ready;

static void
fbcons_bell(void *s __unused)
{
}

static void
fbcons_respond(void *s __unused, const void *buf __unused,
    size_t len __unused)
{
}

static void
fbcons_probe(struct console *cp)
{

	/*
	 * Input "present" too, though there is none: cons_change() counts a
	 * console as up only when both are, and would otherwise report this
	 * one failed.  fbcons_getchar() never has anything.
	 */
	if (rpi_fb_probe() == 0)
		cp->c_flags |= C_PRESENTIN | C_PRESENTOUT;
}

/*
 * gfx_fb.c calls this to mark the cells an image covers, so that scrolling
 * copies them; each console provides it (the EFI one in efi_console.c).
 */
void
term_image_display(teken_gfx_t *state, const teken_rect_t *r)
{
	teken_pos_t p;
	int idx;

	if (screen_buffer == NULL)
		return;
	for (p.tp_row = r->tr_begin.tp_row; p.tp_row < r->tr_end.tp_row;
	    p.tp_row++) {
		for (p.tp_col = r->tr_begin.tp_col;
		    p.tp_col < r->tr_end.tp_col; p.tp_col++) {
			idx = p.tp_col + p.tp_row * state->tg_tp.tp_col;
			if (idx >= state->tg_tp.tp_col * state->tg_tp.tp_row)
				return;
			screen_buffer[idx].a.ta_format |= TF_IMAGE;
		}
	}
}

/* The frame around the text area, in the background colour. */
static void
fbcons_draw_frame(teken_attr_t *a)
{
	teken_attr_t attr = *a;
	teken_color_t fg = a->ta_fgcolor;

	attr.ta_fgcolor = attr.ta_bgcolor;
	teken_set_defattr(&gfx_state.tg_teken, &attr);
	gfx_fb_drawrect(0, 0, gfx_state.tg_fb.fb_width,
	    gfx_state.tg_origin.tp_row, 1);
	gfx_fb_drawrect(0,
	    gfx_state.tg_fb.fb_height - gfx_state.tg_origin.tp_row - 1,
	    gfx_state.tg_fb.fb_width, gfx_state.tg_fb.fb_height, 1);
	gfx_fb_drawrect(0, gfx_state.tg_origin.tp_row,
	    gfx_state.tg_origin.tp_col,
	    gfx_state.tg_fb.fb_height - gfx_state.tg_origin.tp_row - 1, 1);
	gfx_fb_drawrect(
	    gfx_state.tg_fb.fb_width - gfx_state.tg_origin.tp_col - 1,
	    gfx_state.tg_origin.tp_row, gfx_state.tg_fb.fb_width,
	    gfx_state.tg_fb.fb_height, 1);
	attr.ta_fgcolor = fg;
	teken_set_defattr(&gfx_state.tg_teken, &attr);
}

/*
 * Called by gfx_fb.c when the font changes, and by fbcons_init().  The EFI
 * console's cons_update_mode() without EFI: an 80 x 25 terminal, the font
 * setup_font() picks for the display, centred.
 */
bool
cons_update_mode(bool use_gfx_mode __unused)
{
	const teken_attr_t *a;
	teken_attr_t attr;
	uint32_t fb_height, fb_width;
	teken_unit_t rows, cols;
	char env[16];

	if (!fb.valid)
		return (false);

	fb_height = gfx_state.tg_fb.fb_height;
	fb_width = gfx_state.tg_fb.fb_width;
	gfx_state.tg_tp.tp_row = 25;
	gfx_state.tg_tp.tp_col = 80;
	FB_TRACE("setup_font");
	setup_font(&gfx_state, fb_height, fb_width);
	FB_TRACE("setup_font done");
	rows = gfx_state.tg_tp.tp_row;
	cols = gfx_state.tg_tp.tp_col;
	gfx_state.tg_origin.tp_row = (fb_height -
	    rows * gfx_state.tg_font.vf_height) / 2;
	gfx_state.tg_origin.tp_col = (fb_width -
	    cols * gfx_state.tg_font.vf_width) / 2;

	gfx_state.tg_glyph_size = gfx_state.tg_font.vf_height *
	    gfx_state.tg_font.vf_width * 4;
	free(gfx_state.tg_glyph);
	gfx_state.tg_glyph = malloc(gfx_state.tg_glyph_size);
	if (gfx_state.tg_glyph == NULL)
		return (false);

	gfx_state.tg_functions = &tfx;
	snprintf(env, sizeof(env), "%u", fb_height);
	env_setenv("screen.height", EV_VOLATILE | EV_NOHOOK, env,
	    env_noset, env_nounset);
	snprintf(env, sizeof(env), "%u", fb_width);
	env_setenv("screen.width", EV_VOLATILE | EV_NOHOOK, env,
	    env_noset, env_nounset);
	snprintf(env, sizeof(env), "%u", gfx_state.tg_fb.fb_bpp);
	env_setenv("screen.depth", EV_VOLATILE | EV_NOHOOK, env,
	    env_noset, env_nounset);

	FB_TRACE("teken_init");
	teken_init(&gfx_state.tg_teken, gfx_state.tg_functions, &gfx_state);

	free(screen_buffer);
	screen_buffer = malloc(rows * cols * sizeof(*screen_buffer));
	if (screen_buffer == NULL)
		return (false);
	teken_set_winsize(&gfx_state.tg_teken, &gfx_state.tg_tp);
	a = teken_get_defattr(&gfx_state.tg_teken);
	attr = *a;
	FB_TRACE("gfx_fb_setcolors");
	gfx_fb_setcolors(&attr, NULL, env_nounset);
	if (attr.ta_bgcolor == TC_WHITE)
		attr.ta_bgcolor |= TC_LIGHT;
	teken_set_defattr(&gfx_state.tg_teken, &attr);

	FB_TRACE("draw frame");
	fbcons_draw_frame(&attr);
	FB_TRACE("clear (ESC [2J)");
	teken_input(&gfx_state.tg_teken, "\e[2J", 4);
	FB_TRACE("show cursor");
	gfx_state.tg_functions->tf_param(&gfx_state, TP_SHOWCURSOR, 1);
	FB_TRACE("console ready");
	return (true);
}

static int
fbcons_init(int arg __unused)
{
	int roff, goff, boff;

	if (fbcons_ready)
		return (0);
	if (rpi_fb_probe() != 0)
		return (1);

	FB_TRACE("gfx_framework_init");
	gfx_framework_init();
	gfx_state.tg_fb_type = FB_VBE;	/* any direct framebuffer: not GOP */
	gfx_state.tg_fb.fb_addr = fb.base;
	gfx_state.tg_fb.fb_size = fb.size;
	gfx_state.tg_fb.fb_width = fb.width;
	gfx_state.tg_fb.fb_height = fb.height;
	gfx_state.tg_fb.fb_stride = fb.pitch / 4;	/* in pixels */
	gfx_state.tg_fb.fb_bpp = RPI_FB_BPP;
	gfx_state.tg_fb.fb_mask_red = fb.mask_r;
	gfx_state.tg_fb.fb_mask_green = fb.mask_g;
	gfx_state.tg_fb.fb_mask_blue = fb.mask_b;
	gfx_state.tg_fb.fb_mask_reserved = 0xff000000;

	roff = ffs(fb.mask_r) - 1;
	goff = ffs(fb.mask_g) - 1;
	boff = ffs(fb.mask_b) - 1;
	(void)generate_cons_palette(cmap, COLOR_FORMAT_RGB,
	    fb.mask_r >> roff, roff, fb.mask_g >> goff, goff,
	    fb.mask_b >> boff, boff);

	/*
	 * Draw into RAM and copy dirty rectangles out, as the EFI loader
	 * does; it spares reading the framebuffer back when scrolling.
	 */
	FB_TRACE("malloc shadow");
	gfx_state.tg_shadow_fb = malloc(fb.width * fb.height *
	    sizeof(*gfx_state.tg_shadow_fb));
	FB_TRACE(gfx_state.tg_shadow_fb != NULL ? "shadow allocated" :
	    "shadow allocation failed");

	if (!cons_update_mode(true)) {
		gfx_state.tg_fb_type = FB_TEXT;
		return (1);
	}
	fbcons_ready = true;
	return (0);
}

static void
fbcons_putchar(int c)
{
	unsigned char ch = c;

	if (!fbcons_ready)
		return;
	teken_input(&gfx_state.tg_teken, &ch, sizeof(ch));
}

static int
fbcons_getchar(void)
{
	return (-1);
}

static int
fbcons_poll(void)
{
	return (0);
}
