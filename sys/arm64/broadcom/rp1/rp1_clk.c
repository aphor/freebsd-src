/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Contributors
 * All rights reserved.
 *
 * rp1_clk -- the RP1 clock manager ("raspberrypi,rp1-clocks") as a clk(9)
 * provider.
 *
 * RP1's clocks block, at BAR1 + 0x18000, holds three PLLs fed by the 50 MHz
 * crystal (xosc) and some thirty peripheral clock generators.  Each PLL is a
 * VCO ("core", FBDIV_INT/FBDIV_FRAC), a primary output with two cascaded
 * post-dividers (PRIM), a phase output at half the primary rate, and one or
 * two secondary dividers (SEC, TERN).  Each peripheral clock has a CTRL
 * register (enable, source, auxiliary source), an integer divider, on some a
 * 16.16 fractional divider, and a SEL register reporting the source in use,
 * one-hot.
 *
 * The register layout, the clock tree (names, parents, mux encodings, limits)
 * and the device-tree index of each clock follow vendor Linux's
 * drivers/clk/clk-rp1.c and include/dt-bindings/clock/rp1.h, which match the
 * in-tree dt-bindings/clock/raspberrypi,rp1-clocks.h.  This file is written
 * for FreeBSD's clk framework; it does not share code with the GPL driver.
 *
 * WHAT IT DOES AND DOES NOT DO (first stage)
 *
 * It registers every clock the Linux driver registers, with the same index,
 * so <&rp1_clocks N> in the device tree resolves, and it reports each
 * clock's rate, parent and gate state as the hardware has them.  Peripheral
 * clocks can be gated, re-muxed and have their dividers set (divider only:
 * no reparenting to reach a rate).  The PLLs are read-only, and
 * assigned-clock-rates in the node is not applied: the firmware has already
 * programmed them, and nothing on the FreeBSD side yet needs another rate.
 * Nothing is written at attach.
 *
 * Where the Linux table names a parent that does not exist here the entry is
 * NULL: its "" and "-" placeholders (the framework panics on an empty name),
 * and clksrc_gp0..5, fixed-factor loopbacks of the GPCLK outputs that the
 * vendor device tree defines with status = "disabled".
 *
 * dev.rp1_clk.0.measure runs RP1's frequency counter against every clock
 * that has a counter input and prints computed and measured rates side by
 * side; hw.clock.<name>.gate shows whether each is running.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <dev/clk/clk.h>

#include <dt-bindings/clock/raspberrypi,rp1-clocks.h>

#include "clkdev_if.h"
#include "clknode_if.h"

/* PLLs */
#define	PLL_SYS_CS			0x08000
#define	PLL_SYS_PWR			0x08004
#define	PLL_SYS_FBDIV_INT		0x08008
#define	PLL_SYS_FBDIV_FRAC		0x0800c
#define	PLL_SYS_PRIM			0x08010
#define	PLL_SYS_SEC			0x08014
#define	PLL_AUDIO_CS			0x0c000
#define	PLL_AUDIO_PWR			0x0c004
#define	PLL_AUDIO_FBDIV_INT		0x0c008
#define	PLL_AUDIO_FBDIV_FRAC		0x0c00c
#define	PLL_AUDIO_PRIM			0x0c010
#define	PLL_AUDIO_SEC			0x0c014
#define	PLL_AUDIO_TERN			0x0c018
#define	PLL_VIDEO_CS			0x10000
#define	PLL_VIDEO_PWR			0x10004
#define	PLL_VIDEO_FBDIV_INT		0x10008
#define	PLL_VIDEO_FBDIV_FRAC		0x1000c
#define	PLL_VIDEO_PRIM			0x10010
#define	PLL_VIDEO_SEC			0x10014

#define	PLL_CS_LOCK			(1U << 31)
#define	PLL_PWR_PD			(1U << 0)
#define	PLL_PWR_POSTDIVPD		(1U << 3)
#define	PLL_PWR_VCOPD			(1U << 5)
#define	PLL_PRIM_DIV1_SHIFT		16
#define	PLL_PRIM_DIV1_MASK		0x00070000
#define	PLL_PRIM_DIV2_SHIFT		12
#define	PLL_PRIM_DIV2_MASK		0x00007000
#define	PLL_SEC_DIV_SHIFT		8
#define	PLL_SEC_DIV_MASK		0x00001f00
#define	PLL_SEC_RST			(1U << 16)
#define	PLL_PH_EN			(1U << 4)
#define	PLL_PH_PHASE_MASK		0x00000003

/* Peripheral clocks: CTRL, DIV_INT, [DIV_FRAC], SEL */
#define	GPCLK_OE_CTRL			0x00000
#define	CLK_SYS_CTRL			0x00014
#define	CLK_SLOW_SYS_CTRL		0x00024
#define	CLK_DMA_CTRL			0x00044
#define	CLK_UART_CTRL			0x00054
#define	CLK_ETH_CTRL			0x00064
#define	CLK_PWM0_CTRL			0x00074
#define	CLK_PWM1_CTRL			0x00084
#define	CLK_AUDIO_IN_CTRL		0x00094
#define	CLK_AUDIO_OUT_CTRL		0x000a4
#define	CLK_I2S_CTRL			0x000b4
#define	CLK_MIPI0_CFG_CTRL		0x000c4
#define	CLK_MIPI1_CFG_CTRL		0x000d4
#define	CLK_ETH_TSU_CTRL		0x00134
#define	CLK_ADC_CTRL			0x00144
#define	CLK_SDIO_TIMER_CTRL		0x00154
#define	CLK_SDIO_ALT_SRC_CTRL		0x00164
#define	CLK_GP0_CTRL			0x00174
#define	CLK_GP1_CTRL			0x00184
#define	CLK_GP2_CTRL			0x00194
#define	CLK_GP3_CTRL			0x001a4
#define	CLK_GP4_CTRL			0x001b4
#define	CLK_GP5_CTRL			0x001c4
#define	VIDEO_CLK_VEC_CTRL		0x04000
#define	VIDEO_CLK_DPI_CTRL		0x04010
#define	VIDEO_CLK_MIPI0_DPI_CTRL	0x04020
#define	VIDEO_CLK_MIPI1_DPI_CTRL	0x04030

/*
 * Every generator is CTRL, DIV_INT at +4, DIV_FRAC (where there is one) at
 * +8, and SEL at +0xc.
 */
#define	DIV_INT(ctrl)			((ctrl) + 0x4)
#define	DIV_FRAC(ctrl)			((ctrl) + 0x8)
#define	SEL(ctrl)			((ctrl) + 0xc)

#define	CLK_CTRL_ENABLE			(1U << 11)
#define	CLK_CTRL_AUXSRC_MASK		0x000003e0
#define	CLK_CTRL_AUXSRC_SHIFT		5
#define	CLK_DIV_FRAC_BITS		16
#define	AUX_SEL				1

#define	DIV_INT_8BIT_MAX		0x000000ffU
#define	DIV_INT_16BIT_MAX		0x0000ffffU
#define	DIV_INT_24BIT_MAX		0x00ffffffU

/* Frequency counter */
#define	FC0_REF_KHZ			0x0021c
#define	FC0_MIN_KHZ			0x00220
#define	FC0_MAX_KHZ			0x00224
#define	FC0_DELAY			0x00228
#define	FC0_INTERVAL			0x0022c
#define	FC0_SRC				0x00230
#define	FC0_STATUS			0x00234
#define	FC0_RESULT			0x00238
#define	FC_SIZE				0x20
#define	FC_COUNT			8
#define	FC_NUM(idx, off)		((idx) * 32 + (off))
#define	FC0_STATUS_DONE			(1U << 4)
#define	FC0_STATUS_RUNNING		(1U << 8)
#define	FC0_RESULT_FRAC_SHIFT		5
#define	FC_TIMEOUT_US			100000

#define	KHZ				1000UL
#define	MHZ				(KHZ * KHZ)

struct rp1_clk_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;
	struct clkdom		*clkdom;
};

/*
 * clkdev(9) register access, for the clock nodes.
 */
static int
rp1_clk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);

	bus_write_4(sc->res, addr, val);
	return (0);
}

static int
rp1_clk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);

	*val = bus_read_4(sc->res, addr);
	return (0);
}

static int
rp1_clk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);
	uint32_t reg;

	reg = bus_read_4(sc->res, addr);
	reg &= ~clr;
	reg |= set;
	bus_write_4(sc->res, addr, reg);
	return (0);
}

static void
rp1_clk_device_lock(device_t dev)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
}

static void
rp1_clk_device_unlock(device_t dev)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);

	mtx_unlock(&sc->mtx);
}

#define	RD4(clk, off, val)	CLKDEV_READ_4(clknode_get_device(clk), off, val)
#define	WR4(clk, off, val)	CLKDEV_WRITE_4(clknode_get_device(clk), off, val)
#define	MD4(clk, off, c, s)	CLKDEV_MODIFY_4(clknode_get_device(clk), off, c, s)
#define	LOCK(clk)		CLKDEV_DEVICE_LOCK(clknode_get_device(clk))
#define	UNLOCK(clk)		CLKDEV_DEVICE_UNLOCK(clknode_get_device(clk))

static uint32_t
rd4(struct clknode *clk, bus_addr_t off)
{
	uint32_t v;

	RD4(clk, off, &v);
	return (v);
}

/*
 * PLL core (VCO): xosc * (FBDIV_INT + FBDIV_FRAC / 2^24).  Read-only here,
 * and never gated: everything on RP1 hangs off pll_sys.
 */
struct rp1_pll_core_def {
	struct clknode_init_def	clkdef;
	uint32_t		cs, pwr, fbdiv_int, fbdiv_frac;
};

struct rp1_pll_core_sc {
	uint32_t		cs, pwr, fbdiv_int, fbdiv_frac;
};

static int
rp1_pll_core_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
rp1_pll_core_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rp1_pll_core_sc *sc = clknode_get_softc(clk);
	uint64_t fb;

	LOCK(clk);
	fb = ((uint64_t)rd4(clk, sc->fbdiv_int) << 24) +
	    (rd4(clk, sc->fbdiv_frac) & 0xffffff);
	UNLOCK(clk);
	*freq = (*freq * fb + (1 << 23)) >> 24;
	return (0);
}

/*
 * Running means locked and none of the power-down bits set.  (clk-rp1.c's
 * is_prepared returns the opposite -- true when PD or POSTDIVPD is set --
 * while PLL_SYS on dunn reads CS 0x80000001, locked, PWR 0x4, and clocks
 * everything.)
 */
static int
rp1_pll_core_get_gate(struct clknode *clk, bool *enabled)
{
	struct rp1_pll_core_sc *sc = clknode_get_softc(clk);
	uint32_t cs, pwr;

	LOCK(clk);
	cs = rd4(clk, sc->cs);
	pwr = rd4(clk, sc->pwr);
	UNLOCK(clk);
	*enabled = (cs & PLL_CS_LOCK) != 0 &&
	    (pwr & (PLL_PWR_PD | PLL_PWR_VCOPD | PLL_PWR_POSTDIVPD)) == 0;
	return (0);
}

static clknode_method_t rp1_pll_core_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_pll_core_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_pll_core_recalc),
	CLKNODEMETHOD(clknode_get_gate,		rp1_pll_core_get_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_pll_core, rp1_pll_core_class, rp1_pll_core_methods,
    sizeof(struct rp1_pll_core_sc), clknode_class);

/*
 * PLL primary output: core / (DIV1 * DIV2), each 1..7.  Read-only here.
 */
struct rp1_pll_def {
	struct clknode_init_def	clkdef;
	uint32_t		prim;
	uint32_t		fc0;
};

struct rp1_pll_sc {
	uint32_t		prim;
};

static int
rp1_pll_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
rp1_pll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rp1_pll_sc *sc = clknode_get_softc(clk);
	uint32_t prim, d1, d2;

	LOCK(clk);
	prim = rd4(clk, sc->prim);
	UNLOCK(clk);
	d1 = (prim & PLL_PRIM_DIV1_MASK) >> PLL_PRIM_DIV1_SHIFT;
	d2 = (prim & PLL_PRIM_DIV2_MASK) >> PLL_PRIM_DIV2_SHIFT;
	if (d1 == 0 || d2 == 0) {
		*freq = 0;
		return (0);
	}
	*freq = (*freq + (d1 * d2) / 2) / (d1 * d2);
	return (0);
}

static clknode_method_t rp1_pll_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_pll_recalc),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_pll, rp1_pll_class, rp1_pll_methods,
    sizeof(struct rp1_pll_sc), clknode_class);

/*
 * PLL phase output: the primary output / 2, gated by PH_EN in PRIM.
 */
struct rp1_pll_ph_def {
	struct clknode_init_def	clkdef;
	uint32_t		ph;
	uint32_t		phase;
	uint32_t		fixed_div;
	uint32_t		fc0;
};

struct rp1_pll_ph_sc {
	uint32_t		ph, phase, fixed_div;
};

static int
rp1_pll_ph_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
rp1_pll_ph_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rp1_pll_ph_sc *sc = clknode_get_softc(clk);

	*freq /= sc->fixed_div;
	return (0);
}

static int
rp1_pll_ph_get_gate(struct clknode *clk, bool *enabled)
{
	struct rp1_pll_ph_sc *sc = clknode_get_softc(clk);

	LOCK(clk);
	*enabled = (rd4(clk, sc->ph) & PLL_PH_EN) != 0;
	UNLOCK(clk);
	return (0);
}

static int
rp1_pll_ph_set_gate(struct clknode *clk, bool enable)
{
	struct rp1_pll_ph_sc *sc = clknode_get_softc(clk);

	LOCK(clk);
	if (enable)
		MD4(clk, sc->ph, PLL_PH_PHASE_MASK, sc->phase | PLL_PH_EN);
	else
		MD4(clk, sc->ph, PLL_PH_EN, 0);
	UNLOCK(clk);
	return (0);
}

static clknode_method_t rp1_pll_ph_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_pll_ph_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_pll_ph_recalc),
	CLKNODEMETHOD(clknode_get_gate,		rp1_pll_ph_get_gate),
	CLKNODEMETHOD(clknode_set_gate,		rp1_pll_ph_set_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_pll_ph, rp1_pll_ph_class, rp1_pll_ph_methods,
    sizeof(struct rp1_pll_ph_sc), clknode_class);

/*
 * PLL secondary/tertiary divider: core / DIV, DIV 8..19 (other codes act as
 * 19), gated by holding it in reset.  Rates are reported rounded up, as
 * Linux's clk_divider does.  The divider is read-only here.
 */
struct rp1_pll_div_def {
	struct clknode_init_def	clkdef;
	uint32_t		sec;
	uint32_t		fc0;
};

struct rp1_pll_div_sc {
	uint32_t		sec;
};

static int
rp1_pll_div_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
rp1_pll_div_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rp1_pll_div_sc *sc = clknode_get_softc(clk);
	uint32_t div;

	LOCK(clk);
	div = (rd4(clk, sc->sec) & PLL_SEC_DIV_MASK) >> PLL_SEC_DIV_SHIFT;
	UNLOCK(clk);
	if (div < 8 || div > 19)
		div = 19;
	*freq = howmany(*freq, div);
	return (0);
}

static int
rp1_pll_div_get_gate(struct clknode *clk, bool *enabled)
{
	struct rp1_pll_div_sc *sc = clknode_get_softc(clk);

	LOCK(clk);
	*enabled = (rd4(clk, sc->sec) & PLL_SEC_RST) == 0;
	UNLOCK(clk);
	return (0);
}

static int
rp1_pll_div_set_gate(struct clknode *clk, bool enable)
{
	struct rp1_pll_div_sc *sc = clknode_get_softc(clk);

	LOCK(clk);
	MD4(clk, sc->sec, enable ? PLL_SEC_RST : 0, enable ? 0 : PLL_SEC_RST);
	UNLOCK(clk);
	return (0);
}

static clknode_method_t rp1_pll_div_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_pll_div_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_pll_div_recalc),
	CLKNODEMETHOD(clknode_get_gate,		rp1_pll_div_get_gate),
	CLKNODEMETHOD(clknode_set_gate,		rp1_pll_div_set_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_pll_div, rp1_pll_div_class, rp1_pll_div_methods,
    sizeof(struct rp1_pll_div_sc), clknode_class);

/*
 * Peripheral clock generator.  Parents are numbered as clk-rp1.c numbers
 * them: the std sources first (CTRL.SRC), then the aux sources
 * (CTRL.AUXSRC, selected by SRC == AUX_SEL); SEL reads back, one-hot, the
 * std source in use, and 0 until the generator has run.
 */
struct rp1_clock_def {
	struct clknode_init_def	clkdef;
	uint32_t		ctrl;
	bool			has_frac;
	int			num_std;
	int			num_aux;
	uint32_t		src_mask;
	uint32_t		div_int_max;
	uint64_t		max_freq;
	uint32_t		oe_mask;
	uint32_t		fc0;
};

struct rp1_clock_sc {
	const struct rp1_clock_def *def;
};

static int
rp1_clock_get_parent(struct clknode *clk)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;
	uint32_t sel, ctrl;
	int parent;

	LOCK(clk);
	sel = rd4(clk, SEL(d->ctrl));
	ctrl = rd4(clk, d->ctrl);
	UNLOCK(clk);

	if (sel != 0)
		parent = ffs(sel) - 1;
	else
		parent = ctrl & d->src_mask;
	if (parent >= d->num_std)
		parent = AUX_SEL;
	if (parent == AUX_SEL)
		parent = d->num_std +
		    ((ctrl & CLK_CTRL_AUXSRC_MASK) >> CLK_CTRL_AUXSRC_SHIFT);
	return (parent);
}

static int
rp1_clock_init(struct clknode *clk, device_t dev)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;
	int idx;

	idx = rp1_clock_get_parent(clk);
	if (idx >= d->num_std + d->num_aux) {
		device_printf(dev, "%s: source %d out of range\n",
		    d->clkdef.name, idx);
		idx = 0;
	}
	clknode_init_parent_idx(clk, idx);
	return (0);
}

/* The divider as 16.16 fixed point; an integer part of 0 means 2^16. */
static uint64_t
rp1_clock_div(struct clknode *clk, const struct rp1_clock_def *d)
{
	uint64_t div;
	uint32_t frac;

	LOCK(clk);
	div = rd4(clk, DIV_INT(d->ctrl));
	frac = d->has_frac ? rd4(clk, DIV_FRAC(d->ctrl)) : 0;
	UNLOCK(clk);
	if (div == 0)
		div = 1 << 16;
	return ((div << CLK_DIV_FRAC_BITS) | (frac >> (32 - CLK_DIV_FRAC_BITS)));
}

static int
rp1_clock_recalc(struct clknode *clk, uint64_t *freq)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;

	*freq = (*freq << CLK_DIV_FRAC_BITS) / rp1_clock_div(clk, d);
	return (0);
}

static int
rp1_clock_get_gate(struct clknode *clk, bool *enabled)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;

	LOCK(clk);
	*enabled = (rd4(clk, d->ctrl) & CLK_CTRL_ENABLE) != 0;
	UNLOCK(clk);
	return (0);
}

static int
rp1_clock_set_gate(struct clknode *clk, bool enable)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;

	LOCK(clk);
	MD4(clk, d->ctrl, enable ? 0 : CLK_CTRL_ENABLE,
	    enable ? CLK_CTRL_ENABLE : 0);
	if (d->oe_mask != 0)
		MD4(clk, GPCLK_OE_CTRL, enable ? 0 : d->oe_mask,
		    enable ? d->oe_mask : 0);
	UNLOCK(clk);
	return (0);
}

static int
rp1_clock_set_mux(struct clknode *clk, int idx)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;

	if (idx < 0 || idx >= d->num_std + d->num_aux)
		return (EINVAL);
	LOCK(clk);
	if (idx >= d->num_std)
		MD4(clk, d->ctrl, CLK_CTRL_AUXSRC_MASK | d->src_mask,
		    ((idx - d->num_std) << CLK_CTRL_AUXSRC_SHIFT) |
		    (AUX_SEL & d->src_mask));
	else
		MD4(clk, d->ctrl, d->src_mask, idx & d->src_mask);
	UNLOCK(clk);
	if (rp1_clock_get_parent(clk) != idx)
		return (EIO);
	return (0);
}

/*
 * Divider only, on the current parent.  The divider is chosen to the
 * nearest (16.16 when there is a fractional divider), clamped to 1..max,
 * and refused if the result would exceed the generator's maximum.
 */
static int
rp1_clock_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	const struct rp1_clock_def *d =
	    ((struct rp1_clock_sc *)clknode_get_softc(clk))->def;
	uint64_t div, rate;

	if (*fout == 0 || *fout > fin + (fin >> CLK_DIV_FRAC_BITS))
		return (ERANGE);
	if (d->has_frac)
		div = ((fin << CLK_DIV_FRAC_BITS) + *fout / 2) / *fout;
	else
		div = ((fin + *fout / 2) / *fout) << CLK_DIV_FRAC_BITS;
	div = MAX(div, 1ULL << CLK_DIV_FRAC_BITS);
	div = MIN(div, (uint64_t)d->div_int_max << CLK_DIV_FRAC_BITS);
	rate = (fin << CLK_DIV_FRAC_BITS) / div;
	if (rate > d->max_freq + (d->max_freq >> 25))
		return (ERANGE);

	*fout = rate;
	*done = 1;
	if ((flags & CLK_SET_DRYRUN) != 0)
		return (0);

	LOCK(clk);
	WR4(clk, DIV_INT(d->ctrl), div >> CLK_DIV_FRAC_BITS);
	if (d->has_frac)
		WR4(clk, DIV_FRAC(d->ctrl),
		    (uint32_t)(div << (32 - CLK_DIV_FRAC_BITS)));
	UNLOCK(clk);
	return (0);
}

static clknode_method_t rp1_clock_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_clock_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_clock_recalc),
	CLKNODEMETHOD(clknode_get_gate,		rp1_clock_get_gate),
	CLKNODEMETHOD(clknode_set_gate,		rp1_clock_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		rp1_clock_set_mux),
	CLKNODEMETHOD(clknode_set_freq,		rp1_clock_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_clock, rp1_clock_class, rp1_clock_methods,
    sizeof(struct rp1_clock_sc), clknode_class);

/*
 * Variable source: a rate some other driver sets (the MIPI DSI byte
 * clocks), so the dividers below it can compute theirs.
 */
struct rp1_varsrc_sc {
	uint64_t		rate;
};

static int
rp1_varsrc_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
rp1_varsrc_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rp1_varsrc_sc *sc = clknode_get_softc(clk);

	*freq = sc->rate;
	return (0);
}

static int
rp1_varsrc_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct rp1_varsrc_sc *sc = clknode_get_softc(clk);

	*done = 1;
	if ((flags & CLK_SET_DRYRUN) == 0)
		sc->rate = *fout;
	return (0);
}

static clknode_method_t rp1_varsrc_methods[] = {
	CLKNODEMETHOD(clknode_init,		rp1_varsrc_init),
	CLKNODEMETHOD(clknode_recalc_freq,	rp1_varsrc_recalc),
	CLKNODEMETHOD(clknode_set_freq,		rp1_varsrc_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(rp1_varsrc, rp1_varsrc_class, rp1_varsrc_methods,
    sizeof(struct rp1_varsrc_sc), clknode_class);

/*
 * The clock tree.  Names, parents, encodings and limits from clk-rp1.c;
 * NULL where it has "", "-" or clksrc_gpN (see the top of the file).
 */
#define	DEF(n, i, p)							\
	.clkdef = {							\
		.name = (n),						\
		.id = (i),						\
		.parent_names = (p),					\
		.parent_cnt = nitems(p),				\
		.flags = CLK_NODE_STATIC_STRINGS,			\
	}

static const char *xosc_parent[] = { "xosc" };
static const char *pll_sys_core_parent[] = { "pll_sys_core" };
static const char *pll_audio_core_parent[] = { "pll_audio_core" };
static const char *pll_video_core_parent[] = { "pll_video_core" };
static const char *pll_sys_parent[] = { "pll_sys" };
static const char *pll_audio_parent[] = { "pll_audio" };
static const char *pll_video_parent[] = { "pll_video" };

static struct rp1_pll_core_def pll_core_defs[] = {
	{ DEF("pll_sys_core", RP1_PLL_SYS_CORE, xosc_parent),
	  PLL_SYS_CS, PLL_SYS_PWR, PLL_SYS_FBDIV_INT, PLL_SYS_FBDIV_FRAC },
	{ DEF("pll_audio_core", RP1_PLL_AUDIO_CORE, xosc_parent),
	  PLL_AUDIO_CS, PLL_AUDIO_PWR, PLL_AUDIO_FBDIV_INT,
	  PLL_AUDIO_FBDIV_FRAC },
	{ DEF("pll_video_core", RP1_PLL_VIDEO_CORE, xosc_parent),
	  PLL_VIDEO_CS, PLL_VIDEO_PWR, PLL_VIDEO_FBDIV_INT,
	  PLL_VIDEO_FBDIV_FRAC },
};

static struct rp1_pll_def pll_defs[] = {
	{ DEF("pll_sys", RP1_PLL_SYS, pll_sys_core_parent),
	  PLL_SYS_PRIM, FC_NUM(0, 2) },
	{ DEF("pll_audio", RP1_PLL_AUDIO, pll_audio_core_parent),
	  PLL_AUDIO_PRIM, FC_NUM(4, 2) },
	{ DEF("pll_video", RP1_PLL_VIDEO, pll_video_core_parent),
	  PLL_VIDEO_PRIM, FC_NUM(3, 2) },
};

static struct rp1_pll_ph_def pll_ph_defs[] = {
	{ DEF("pll_sys_pri_ph", RP1_PLL_SYS_PRI_PH, pll_sys_parent),
	  PLL_SYS_PRIM, 0, 2, FC_NUM(1, 2) },
	{ DEF("pll_audio_pri_ph", RP1_PLL_AUDIO_PRI_PH, pll_audio_parent),
	  PLL_AUDIO_PRIM, 0, 2, FC_NUM(5, 1) },
	{ DEF("pll_video_pri_ph", RP1_PLL_VIDEO_PRI_PH, pll_video_parent),
	  PLL_VIDEO_PRIM, 0, 2, FC_NUM(4, 3) },
};

static struct rp1_pll_div_def pll_div_defs[] = {
	{ DEF("pll_sys_sec", RP1_PLL_SYS_SEC, pll_sys_core_parent),
	  PLL_SYS_SEC, FC_NUM(2, 2) },
	{ DEF("pll_audio_sec", RP1_PLL_AUDIO_SEC, pll_audio_core_parent),
	  PLL_AUDIO_SEC, FC_NUM(6, 2) },
	{ DEF("pll_video_sec", RP1_PLL_VIDEO_SEC, pll_video_core_parent),
	  PLL_VIDEO_SEC, FC_NUM(5, 3) },
	{ DEF("pll_audio_tern", RP1_PLL_AUDIO_TERN, pll_audio_core_parent),
	  PLL_AUDIO_TERN, FC_NUM(6, 2) },
};

/* Aux source lists shared by several generators. */
static const char *p_sys_video_xosc[] = { "pll_sys_pri_ph", "pll_video",
	"xosc", NULL, NULL, NULL, NULL, NULL, NULL };
static const char *p_pwm[] = { NULL, "pll_video_sec", "xosc", NULL, NULL,
	NULL, NULL, NULL, NULL };
static const char *p_video_pixel[] = { "pll_sys", "pll_video_sec",
	"pll_video", NULL, NULL, NULL, NULL, NULL };
static const char *p_vec[] = { "pll_sys_pri_ph", "pll_video_sec",
	"pll_video", NULL, NULL, NULL, NULL, NULL };

static const char *p_clk_sys[] = { "xosc", NULL, "pll_sys" };
static const char *p_eth[] = { "pll_sys_sec", "pll_sys", "pll_video_sec",
	NULL, NULL, NULL, NULL, NULL, NULL };
static const char *p_audio_in[] = { NULL, NULL, NULL, "pll_video_sec",
	"xosc", NULL, NULL, NULL, NULL, NULL, NULL };
static const char *p_audio_out[] = { NULL, "pll_audio_sec",
	"pll_video_sec", "xosc", NULL, NULL, NULL, NULL, NULL, NULL };
static const char *p_i2s[] = { "xosc", "pll_audio", "pll_audio_sec", NULL,
	NULL, NULL, NULL, NULL, NULL };
static const char *p_eth_tsu[] = { "xosc", "pll_video_sec", NULL, NULL,
	NULL, NULL, NULL, NULL };
static const char *p_adc[] = { "xosc", NULL, NULL, NULL, NULL, NULL, NULL,
	NULL };
static const char *p_sdio_alt[] = { "pll_sys" };
static const char *p_gp0[] = { "xosc", NULL, NULL, NULL, NULL, NULL,
	"pll_sys", NULL, NULL, NULL, "clk_i2s", "clk_adc", NULL, NULL, NULL,
	"clk_sys" };
static const char *p_gp1[] = { "clk_sdio_timer", NULL, NULL, NULL, NULL,
	NULL, "pll_sys_pri_ph", NULL, NULL, NULL, "clk_adc", "clk_dpi",
	"clk_pwm0", NULL, NULL, NULL };
static const char *p_gp2[] = { "clk_sdio_alt_src", NULL, NULL, NULL, NULL,
	NULL, "pll_sys_sec", NULL, "pll_video", "clk_audio_in", "clk_dpi",
	"clk_pwm0", "clk_pwm1", "clk_mipi0_dpi", "clk_mipi1_cfg", "clk_sys" };
static const char *p_gp3[] = { "xosc", NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, "pll_video_pri_ph", "clk_audio_out", NULL, NULL,
	"clk_mipi1_dpi", NULL, NULL, NULL };
static const char *p_gp4[] = { "xosc", NULL, NULL, NULL, NULL, NULL, NULL,
	"pll_video_sec", NULL, NULL, NULL, "clk_mipi0_cfg", "clk_uart", NULL,
	NULL, "clk_sys" };
static const char *p_gp5[] = { "xosc", NULL, NULL, NULL, NULL, NULL, NULL,
	"pll_video_sec", "clk_eth_tsu", NULL, "clk_vec", NULL, NULL, NULL, NULL,
	NULL };
static const char *p_mipi0_dpi[] = { "pll_sys", "pll_video_sec", "pll_video",
	"clksrc_mipi0_dsi_byteclk", NULL, NULL, NULL, NULL };
static const char *p_mipi1_dpi[] = { "pll_sys", "pll_video_sec", "pll_video",
	"clksrc_mipi1_dsi_byteclk", NULL, NULL, NULL, NULL };

#define	CLK(n, i, p, c, std, fr, sm, dmax, fmax, oe, fc)		\
	{ DEF(n, i, p), .ctrl = (c), .has_frac = (fr), .num_std = (std),	\
	  .num_aux = nitems(p) - (std), .src_mask = (sm),		\
	  .div_int_max = (dmax), .max_freq = (fmax), .oe_mask = (oe),	\
	  .fc0 = (fc) }

static struct rp1_clock_def clock_defs[] = {
	CLK("clk_sys", RP1_CLK_SYS, p_clk_sys, CLK_SYS_CTRL, 3, false, 0x3,
	    DIV_INT_24BIT_MAX, 200 * MHZ, 0, FC_NUM(0, 4)),
	CLK("clk_slow_sys", RP1_CLK_SLOW_SYS, xosc_parent, CLK_SLOW_SYS_CTRL,
	    1, false, 0x1, DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(1, 4)),
	CLK("clk_dma", RP1_CLK_DMA, p_sys_video_xosc, CLK_DMA_CTRL, 0, false,
	    0, DIV_INT_8BIT_MAX, 100 * MHZ, 0, FC_NUM(2, 2)),
	CLK("clk_uart", RP1_CLK_UART, p_sys_video_xosc, CLK_UART_CTRL, 0,
	    false, 0, DIV_INT_8BIT_MAX, 100 * MHZ, 0, FC_NUM(6, 7)),
	CLK("clk_eth", RP1_CLK_ETH, p_eth, CLK_ETH_CTRL, 0, false, 0,
	    DIV_INT_8BIT_MAX, 125 * MHZ, 0, FC_NUM(4, 6)),
	CLK("clk_pwm0", RP1_CLK_PWM0, p_pwm, CLK_PWM0_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 76800 * KHZ, 0, FC_NUM(0, 5)),
	CLK("clk_pwm1", RP1_CLK_PWM1, p_pwm, CLK_PWM1_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 76800 * KHZ, 0, FC_NUM(1, 5)),
	CLK("clk_audio_in", RP1_CLK_AUDIO_IN, p_audio_in, CLK_AUDIO_IN_CTRL, 0,
	    false, 0, DIV_INT_8BIT_MAX, 76800 * KHZ, 0, FC_NUM(2, 5)),
	CLK("clk_audio_out", RP1_CLK_AUDIO_OUT, p_audio_out,
	    CLK_AUDIO_OUT_CTRL, 0, false, 0, DIV_INT_8BIT_MAX, 153600 * KHZ, 0,
	    FC_NUM(3, 5)),
	CLK("clk_i2s", RP1_CLK_I2S, p_i2s, CLK_I2S_CTRL, 0, false, 0,
	    DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(4, 4)),
	CLK("clk_mipi0_cfg", RP1_CLK_MIPI0_CFG, xosc_parent, CLK_MIPI0_CFG_CTRL,
	    0, false, 0, DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(4, 5)),
	CLK("clk_mipi1_cfg", RP1_CLK_MIPI1_CFG, xosc_parent, CLK_MIPI1_CFG_CTRL,
	    0, false, 1, DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(5, 6)),
	CLK("clk_eth_tsu", RP1_CLK_ETH_TSU, p_eth_tsu, CLK_ETH_TSU_CTRL, 0,
	    false, 0, DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(5, 7)),
	CLK("clk_adc", RP1_CLK_ADC, p_adc, CLK_ADC_CTRL, 0, false, 0,
	    DIV_INT_8BIT_MAX, 50 * MHZ, 0, FC_NUM(5, 5)),
	CLK("clk_sdio_timer", RP1_CLK_SDIO_TIMER, xosc_parent,
	    CLK_SDIO_TIMER_CTRL, 0, false, 0, DIV_INT_8BIT_MAX, 50 * MHZ, 0,
	    FC_NUM(3, 4)),
	CLK("clk_sdio_alt_src", RP1_CLK_SDIO_ALT_SRC, p_sdio_alt,
	    CLK_SDIO_ALT_SRC_CTRL, 0, false, 0, DIV_INT_8BIT_MAX, 200 * MHZ, 0,
	    FC_NUM(5, 4)),
	CLK("clk_gp0", RP1_CLK_GP0, p_gp0, CLK_GP0_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 0, FC_NUM(0, 1)),
	CLK("clk_gp1", RP1_CLK_GP1, p_gp1, CLK_GP1_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 1, FC_NUM(1, 1)),
	CLK("clk_gp2", RP1_CLK_GP2, p_gp2, CLK_GP2_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 2, FC_NUM(2, 1)),
	CLK("clk_gp3", RP1_CLK_GP3, p_gp3, CLK_GP3_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 3, FC_NUM(3, 1)),
	CLK("clk_gp4", RP1_CLK_GP4, p_gp4, CLK_GP4_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 4, FC_NUM(4, 1)),
	CLK("clk_gp5", RP1_CLK_GP5, p_gp5, CLK_GP5_CTRL, 0, true, 0,
	    DIV_INT_16BIT_MAX, 100 * MHZ, 1U << 5, FC_NUM(5, 1)),
	CLK("clk_vec", RP1_CLK_VEC, p_vec, VIDEO_CLK_VEC_CTRL, 0,
	    false, 0, DIV_INT_8BIT_MAX, 108 * MHZ, 0, FC_NUM(0, 6)),
	CLK("clk_dpi", RP1_CLK_DPI, p_video_pixel, VIDEO_CLK_DPI_CTRL, 0,
	    false, 0, DIV_INT_8BIT_MAX, 200 * MHZ, 0, FC_NUM(1, 6)),
	CLK("clk_mipi0_dpi", RP1_CLK_MIPI0_DPI, p_mipi0_dpi,
	    VIDEO_CLK_MIPI0_DPI_CTRL, 0, true, 0, DIV_INT_8BIT_MAX, 200 * MHZ, 0,
	    FC_NUM(2, 6)),
	CLK("clk_mipi1_dpi", RP1_CLK_MIPI1_DPI, p_mipi1_dpi,
	    VIDEO_CLK_MIPI1_DPI_CTRL, 0, true, 0, DIV_INT_8BIT_MAX, 200 * MHZ, 0,
	    FC_NUM(3, 6)),
};

static struct clknode_init_def varsrc_defs[] = {
	{ .name = "clksrc_mipi0_dsi_byteclk",
	  .id = RP1_CLK_MIPI0_DSI_BYTECLOCK, .parent_names = xosc_parent,
	  .parent_cnt = 1, .flags = CLK_NODE_STATIC_STRINGS },
	{ .name = "clksrc_mipi1_dsi_byteclk",
	  .id = RP1_CLK_MIPI1_DSI_BYTECLOCK, .parent_names = xosc_parent,
	  .parent_cnt = 1, .flags = CLK_NODE_STATIC_STRINGS },
};

static int
rp1_clk_register(struct rp1_clk_softc *sc)
{
	struct clknode *clk;
	u_int i;

#define	CREATE(cls, def)						\
	do {								\
		clk = clknode_create(sc->clkdom, &(cls), &(def)->clkdef); \
		if (clk == NULL)					\
			return (ENXIO);					\
	} while (0)
#define	REGISTER()							\
	do {								\
		if (clknode_register(sc->clkdom, clk) == NULL)		\
			return (ENXIO);					\
	} while (0)

	for (i = 0; i < nitems(pll_core_defs); i++) {
		struct rp1_pll_core_sc *s;

		CREATE(rp1_pll_core_class, &pll_core_defs[i]);
		s = clknode_get_softc(clk);
		s->cs = pll_core_defs[i].cs;
		s->pwr = pll_core_defs[i].pwr;
		s->fbdiv_int = pll_core_defs[i].fbdiv_int;
		s->fbdiv_frac = pll_core_defs[i].fbdiv_frac;
		REGISTER();
	}
	for (i = 0; i < nitems(pll_defs); i++) {
		CREATE(rp1_pll_class, &pll_defs[i]);
		((struct rp1_pll_sc *)clknode_get_softc(clk))->prim =
		    pll_defs[i].prim;
		REGISTER();
	}
	for (i = 0; i < nitems(pll_ph_defs); i++) {
		struct rp1_pll_ph_sc *s;

		CREATE(rp1_pll_ph_class, &pll_ph_defs[i]);
		s = clknode_get_softc(clk);
		s->ph = pll_ph_defs[i].ph;
		s->phase = pll_ph_defs[i].phase;
		s->fixed_div = pll_ph_defs[i].fixed_div;
		REGISTER();
	}
	for (i = 0; i < nitems(pll_div_defs); i++) {
		CREATE(rp1_pll_div_class, &pll_div_defs[i]);
		((struct rp1_pll_div_sc *)clknode_get_softc(clk))->sec =
		    pll_div_defs[i].sec;
		REGISTER();
	}
	for (i = 0; i < nitems(varsrc_defs); i++) {
		clk = clknode_create(sc->clkdom, &rp1_varsrc_class,
		    &varsrc_defs[i]);
		if (clk == NULL)
			return (ENXIO);
		REGISTER();
	}
	for (i = 0; i < nitems(clock_defs); i++) {
		CREATE(rp1_clock_class, &clock_defs[i]);
		((struct rp1_clock_sc *)clknode_get_softc(clk))->def =
		    &clock_defs[i];
		REGISTER();
	}
#undef CREATE
#undef REGISTER
	return (0);
}

/*
 * The frequency counter.  fc0 packs a counter group (/32) and that group's
 * source number (%32); source 0 is "off".  The reference is clk_slow_sys.
 * The result is kHz with FC0_RESULT_FRAC_SHIFT fractional bits.
 */
static int
rp1_clk_fc_measure(struct rp1_clk_softc *sc, uint32_t fc0, uint64_t ref_hz,
    uint64_t *khz)
{
	bus_addr_t off;
	uint32_t src;
	int us, error;

	src = fc0 % 32;
	if (src == 0 || fc0 / 32 >= FC_COUNT)
		return (EINVAL);
	off = (fc0 / 32) * FC_SIZE;

	error = ETIMEDOUT;
	mtx_lock(&sc->mtx);
	for (us = 0; us < FC_TIMEOUT_US; us++) {
		if ((bus_read_4(sc->res, off + FC0_STATUS) &
		    FC0_STATUS_RUNNING) == 0)
			break;
		DELAY(1);
	}
	if (us == FC_TIMEOUT_US)
		goto out;
	bus_write_4(sc->res, off + FC0_REF_KHZ, ref_hz / KHZ);
	bus_write_4(sc->res, off + FC0_MIN_KHZ, 0);
	bus_write_4(sc->res, off + FC0_MAX_KHZ, 0x1ffffff);
	bus_write_4(sc->res, off + FC0_INTERVAL, 8);
	bus_write_4(sc->res, off + FC0_DELAY, 7);
	bus_write_4(sc->res, off + FC0_SRC, src);
	for (us = 0; us < FC_TIMEOUT_US; us++) {
		if ((bus_read_4(sc->res, off + FC0_STATUS) &
		    FC0_STATUS_DONE) != 0) {
			*khz = bus_read_4(sc->res, off + FC0_RESULT) >>
			    FC0_RESULT_FRAC_SHIFT;
			error = 0;
			break;
		}
		DELAY(1);
	}
	bus_write_4(sc->res, off + FC0_SRC, 0);
out:
	mtx_unlock(&sc->mtx);
	return (error);
}

static void
rp1_clk_measure_one(struct rp1_clk_softc *sc, struct sbuf *sb,
    const char *name, uint32_t fc0, uint64_t ref_hz)
{
	clk_t clk;
	uint64_t freq, khz;
	int error;

	if (clk_get_by_name(sc->dev, name, &clk) != 0) {
		sbuf_printf(sb, "%-18s not registered\n", name);
		return;
	}
	if (clk_get_freq(clk, &freq) != 0)
		freq = 0;
	clk_release(clk);

	error = rp1_clk_fc_measure(sc, fc0, ref_hz, &khz);
	if (error != 0)
		sbuf_printf(sb, "%-18s %12ju Hz  measure failed (%d)\n", name,
		    (uintmax_t)freq, error);
	else
		sbuf_printf(sb, "%-18s %12ju Hz  measured %9ju kHz\n", name,
		    (uintmax_t)freq, (uintmax_t)khz);
}

static int
rp1_clk_sysctl_measure(SYSCTL_HANDLER_ARGS)
{
	struct rp1_clk_softc *sc = arg1;
	struct sbuf *sb;
	clk_t ref;
	uint64_t ref_hz;
	u_int i;
	int error;

	if (clk_get_by_name(sc->dev, "clk_slow_sys", &ref) != 0)
		return (ENXIO);
	error = clk_get_freq(ref, &ref_hz);
	clk_release(ref);
	if (error != 0)
		return (error);

	sb = sbuf_new_for_sysctl(NULL, NULL, 2048, req);
	sbuf_printf(sb, "\nreference clk_slow_sys %ju Hz\n",
	    (uintmax_t)ref_hz);
	for (i = 0; i < nitems(pll_defs); i++)
		rp1_clk_measure_one(sc, sb, pll_defs[i].clkdef.name,
		    pll_defs[i].fc0, ref_hz);
	for (i = 0; i < nitems(pll_ph_defs); i++)
		rp1_clk_measure_one(sc, sb, pll_ph_defs[i].clkdef.name,
		    pll_ph_defs[i].fc0, ref_hz);
	for (i = 0; i < nitems(pll_div_defs); i++)
		rp1_clk_measure_one(sc, sb, pll_div_defs[i].clkdef.name,
		    pll_div_defs[i].fc0, ref_hz);
	for (i = 0; i < nitems(clock_defs); i++)
		rp1_clk_measure_one(sc, sb, clock_defs[i].clkdef.name,
		    clock_defs[i].fc0, ref_hz);
	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

static int
rp1_clk_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "raspberrypi,rp1-clocks"))
		return (ENXIO);
	device_set_desc(dev, "RP1 clock manager");
	return (BUS_PROBE_DEFAULT);
}

static int
rp1_clk_attach(device_t dev)
{
	struct rp1_clk_softc *sc = device_get_softc(dev);
	int error, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map the clocks block\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "cannot create the clock domain\n");
		error = ENXIO;
		goto fail;
	}
	if ((error = rp1_clk_register(sc)) != 0) {
		device_printf(dev, "cannot register the clocks (%d)\n", error);
		goto fail;
	}
	/*
	 * clkdom_finit() makes the domain visible whatever it returns; an
	 * error only means some clock's parent could not be resolved, and it
	 * has named which.
	 */
	error = clkdom_finit(sc->clkdom);
	if (error != 0)
		device_printf(dev, "clock domain finit: %d\n", error);
	if (bootverbose)
		clkdom_dump(sc->clkdom);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "measure",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    rp1_clk_sysctl_measure, "A",
	    "Each clock's computed rate beside the frequency counter's reading");
	return (0);

fail:
	/* Clock nodes cannot be unregistered; only reached before finit. */
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
	mtx_destroy(&sc->mtx);
	return (error);
}

static device_method_t rp1_clk_methods[] = {
	DEVMETHOD(device_probe,		rp1_clk_probe),
	DEVMETHOD(device_attach,	rp1_clk_attach),

	DEVMETHOD(clkdev_read_4,	rp1_clk_read_4),
	DEVMETHOD(clkdev_write_4,	rp1_clk_write_4),
	DEVMETHOD(clkdev_modify_4,	rp1_clk_modify_4),
	DEVMETHOD(clkdev_device_lock,	rp1_clk_device_lock),
	DEVMETHOD(clkdev_device_unlock,	rp1_clk_device_unlock),
	DEVMETHOD_END
};

static driver_t rp1_clk_driver = {
	"rp1_clk",
	rp1_clk_methods,
	sizeof(struct rp1_clk_softc),
};

DRIVER_MODULE(rp1_clk, simplebus, rp1_clk_driver, NULL, NULL);
MODULE_VERSION(rp1_clk, 1);
