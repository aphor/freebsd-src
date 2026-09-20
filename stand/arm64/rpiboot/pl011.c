/*-
 * pl011.c -- ARM PrimeCell PL011 UART driver for the Raspberry Pi 5 loader.
 *
 * Freestanding: no libc, no FreeBSD headers, nothing but <stdint.h>.  That is
 * deliberate, so this same file can be linked into
 *
 *   - a bare-metal test image loaded by the VPU firmware (constest.bin),
 *     which is how it gets verified on hardware, and
 *   - the loader proper, behind the thin struct console wrapper in
 *     pl011_console.c.
 *
 * Constraints that shape the code, all consequences of running with the MMU
 * off at EL2 straight out of the firmware:
 *
 *   - Every access is Device-nGnRnE, so unaligned accesses fault.  Build with
 *     -mstrict-align; do not let the compiler merge or widen anything.
 *   - No FP/SIMD: CPTR_EL2 may trap it.  Build with -mgeneral-regs-only.
 *   - Physical == virtual, so a uint64_t address can be cast to a pointer.
 *   - No memcpy/memset from a library exists; anything needed is written out.
 *
 * See pl011.h for which UART this is, why, and the byte-versus-word offset
 * trap in uart_dev_pl011.c.
 */

#include "pl011.h"

/*
 * MMIO accessors.  Separate functions rather than macros so the volatile
 * qualifier and the access width are stated exactly once each, and so a
 * misuse is a type error rather than a surprising expansion.
 */
static inline uint32_t
pl011_rd(const struct pl011 *sc, uint32_t reg)
{
	return (*(volatile uint32_t *)(uintptr_t)(sc->base + reg));
}

static inline void
pl011_wr(const struct pl011 *sc, uint32_t reg, uint32_t val)
{
	*(volatile uint32_t *)(uintptr_t)(sc->base + reg) = val;
}

void
pl011_attach(struct pl011 *sc, uint64_t base, uint32_t uartclk)
{
	sc->base = base;
	sc->uartclk = uartclk;
}

int
pl011_periphid(const struct pl011 *sc, uint32_t *out)
{
	uint32_t id;

	/*
	 * Assembled the way arm,primecell-periphid encodes it, so the result
	 * can be compared directly against the device-tree value:
	 * id3<<24 | id2<<16 | id1<<8 | id0.
	 */
	id = ((pl011_rd(sc, PL011_PERIPHID3) & 0xff) << 24) |
	     ((pl011_rd(sc, PL011_PERIPHID2) & 0xff) << 16) |
	     ((pl011_rd(sc, PL011_PERIPHID1) & 0xff) << 8) |
	      (pl011_rd(sc, PL011_PERIPHID0) & 0xff);

	*out = id;

	/*
	 * Zero means the registers are not exposed by this SoC's 0x200 reg
	 * window, which is the Raspberry Pi 5 case -- not an absent device.
	 * See the long comment in pl011.h before treating this as a fault.
	 */
	return (id == 0 ? -1 : 0);
}

void
pl011_drain(const struct pl011 *sc)
{
	/*
	 * TXFE says the FIFO is empty; BUSY stays set until the last bit has
	 * actually left the shift register.  Both matter before disabling the
	 * UART or halting the CPU, or the tail of the message is lost.
	 */
	while ((pl011_rd(sc, PL011_FR) & PL011_FR_TXFE) == 0)
		;
	while ((pl011_rd(sc, PL011_FR) & PL011_FR_BUSY) != 0)
		;
}

int
pl011_configure(const struct pl011 *sc, uint32_t baud)
{
	uint32_t div, ibrd, fbrd, cr;

	if (sc->uartclk == 0 || baud == 0)
		return (-1);

	/*
	 * The PL011 divisor is a 16.6 fixed-point value: BAUDDIV =
	 * uartclk / (16 * baud), with the fraction in 1/64ths.  Computing
	 * 64*BAUDDIV in one integer expression and then splitting it keeps
	 * the rounding in one place.
	 *
	 *	div = (4 * uartclk) / baud
	 *
	 * because 64 * uartclk / (16 * baud) == 4 * uartclk / baud.  The
	 * multiply cannot overflow 32 bits for any plausible uartclk here
	 * (4 * 44236800 = 176947200), but it is done in 64-bit anyway so the
	 * expression stays correct if this driver is reused on a part with a
	 * faster reference clock.
	 */
	div = (uint32_t)(((uint64_t)sc->uartclk * 4) / baud);
	ibrd = div >> 6;
	fbrd = div & 0x3f;

	/* IBRD is 16 bits, and 0 is not a legal divisor. */
	if (ibrd == 0 || ibrd > 0xffff)
		return (-1);

	/*
	 * Order matters.  The PL011 latches IBRD/FBRD only when LCR_H is
	 * written, and the manual requires the UART be disabled while the
	 * baud registers change.  Disabling mid-character would corrupt the
	 * line, so drain first.
	 */
	pl011_drain(sc);

	cr = pl011_rd(sc, PL011_CR);
	pl011_wr(sc, PL011_CR, cr & ~PL011_CR_UARTEN);

	pl011_wr(sc, PL011_IMSC, 0);			/* no interrupts */
	pl011_wr(sc, PL011_ICR, PL011_ICR_CLEAR_ALL);

	pl011_wr(sc, PL011_IBRD, ibrd);
	pl011_wr(sc, PL011_FBRD, fbrd);
	/* This write is what commits IBRD and FBRD. */
	pl011_wr(sc, PL011_LCR_H, PL011_LCR_H_8N1);

	pl011_wr(sc, PL011_CR,
	    PL011_CR_UARTEN | PL011_CR_TXE | PL011_CR_RXE);

	return (0);
}

uint32_t
pl011_enable_rx(const struct pl011 *sc)
{
	uint32_t cr;

	cr = pl011_rd(sc, PL011_CR);
	/*
	 * Set only what is needed and preserve the rest.  In particular this
	 * does not touch LCR_H or the baud divisors, so a console inherited
	 * from the firmware keeps working at whatever rate it was using even
	 * if that is not what we would have chosen.
	 */
	pl011_wr(sc, PL011_CR,
	    cr | PL011_CR_UARTEN | PL011_CR_TXE | PL011_CR_RXE);

	return (cr);
}

void
pl011_putc_raw(const struct pl011 *sc, int c)
{
	while ((pl011_rd(sc, PL011_FR) & PL011_FR_TXFF) != 0)
		;
	pl011_wr(sc, PL011_DR, (uint32_t)(c & 0xff));
}

void
pl011_putc(const struct pl011 *sc, int c)
{
	if (c == '\n')
		pl011_putc_raw(sc, '\r');
	pl011_putc_raw(sc, c);
}

void
pl011_puts(const struct pl011 *sc, const char *s)
{
	while (*s != '\0')
		pl011_putc(sc, (unsigned char)*s++);
}

int
pl011_rxready(const struct pl011 *sc)
{
	return ((pl011_rd(sc, PL011_FR) & PL011_FR_RXFE) == 0);
}

int
pl011_getc(const struct pl011 *sc)
{
	uint32_t dr;

	if (!pl011_rxready(sc))
		return (-1);

	dr = pl011_rd(sc, PL011_DR);
	if ((dr & PL011_DR_ERROR) != 0) {
		/*
		 * Clear the sticky error and return the byte anyway.  A
		 * framing error on the first character after the firmware
		 * hands over is common and is not worth losing input over;
		 * silently dropping it would look like a dead keyboard.
		 */
		pl011_wr(sc, PL011_RSR_ECR, 0);
	}

	return ((int)(dr & PL011_DR_DATA));
}

int
pl011_getc_wait(const struct pl011 *sc)
{
	int c;

	do {
		c = pl011_getc(sc);
	} while (c < 0);

	return (c);
}

static inline uint64_t
pl011_cntpct(void)
{
	uint64_t v;

	__asm__ __volatile__("isb; mrs %0, cntpct_el0" : "=r"(v));
	return (v);
}

int
pl011_loopback_byte(const struct pl011 *sc, uint8_t tx, uint32_t *prev_cr,
    uint64_t timeout_ticks)
{
	uint64_t deadline;
	uint32_t cr;
	int c;

	/*
	 * Everything already queued must reach the wire first: once LBE is
	 * set it would be delivered to our own receiver instead.
	 */
	pl011_drain(sc);

	cr = pl011_rd(sc, PL011_CR);
	*prev_cr = cr;

	pl011_wr(sc, PL011_CR, cr | PL011_CR_UARTEN | PL011_CR_TXE |
	    PL011_CR_RXE | PL011_CR_LBE);

	/* Discard anything already sitting in the receiver. */
	while ((pl011_rd(sc, PL011_FR) & PL011_FR_RXFE) == 0)
		(void)pl011_rd(sc, PL011_DR);

	/*
	 * Note: on this implementation LBE loops the transmitter back to the
	 * receiver but does NOT stop driving the TXD pad, so the test byte is
	 * also emitted on the wire.  Observed 2026-09-20: a stray 'Z' (0x5a)
	 * appears in the console log immediately before the result line.  It
	 * is cosmetic, but do not read it as the byte having escaped the
	 * loopback -- it arrives on both paths.
	 */
	pl011_putc_raw(sc, tx);

	deadline = pl011_cntpct() + timeout_ticks;
	do {
		c = pl011_getc(sc);
	} while (c < 0 && pl011_cntpct() < deadline);

	/*
	 * Restore before returning, on every path.  A return that left LBE
	 * set would take the console with it.
	 */
	pl011_drain(sc);
	pl011_wr(sc, PL011_CR, cr);

	return (c);
}

void
pl011_puthex(const struct pl011 *sc, uint64_t v, int digits)
{
	static const char hex[] = "0123456789abcdef";
	int i;

	if (digits < 1)
		digits = 1;
	if (digits > 16)
		digits = 16;

	for (i = digits - 1; i >= 0; i--)
		pl011_putc_raw(sc, hex[(v >> (i * 4)) & 0xf]);
}
