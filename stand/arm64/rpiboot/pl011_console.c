/*-
 * pl011_console.c -- struct console over the verified PL011 driver.
 *
 * The driver underneath this (pl011.c) was proven on hardware before any of
 * it was wired into the loader: transmit on the inherited configuration,
 * receive confirmed both by the PL011's internal loopback and by real bytes
 * arriving from the wire, and a full reprogram that reproduced the firmware's
 * own register values exactly.  See the constest results in
 * rpi5_modules.git/doc/LOADER_ZIMAGE.md.
 *
 * This file is only the adaptor, and every method maps one-for-one onto
 * something already tested.
 */

#include <stand.h>

#include "bootstrap.h"
#include "librpiboot.h"
#include "pl011.h"

static struct pl011 sc;

static void	pl011_cons_probe(struct console *cp);
static int	pl011_cons_init(int arg);
static void	pl011_cons_putchar(int c);
static int	pl011_cons_getchar(void);
static int	pl011_cons_poll(void);

struct console pl011_console = {
	.c_name = "uart",
	.c_desc = "PL011 serial port",
	.c_flags = 0,
	.c_probe = pl011_cons_probe,
	.c_init = pl011_cons_init,
	.c_out = pl011_cons_putchar,
	.c_in = pl011_cons_getchar,
	.c_ready = pl011_cons_poll,
};

static void
pl011_cons_probe(struct console *cp)
{
	/*
	 * Bind without touching a register.  Always present: this UART is
	 * native to the SoC, needs no bus brought up, and the firmware is
	 * demonstrably printing through it immediately before we are entered.
	 *
	 * Deliberately no hardware presence test.  The PrimeCell ID registers
	 * sit at 0xfe0, outside this SoC's 0x200 reg window, so they read as
	 * zero on a working UART -- an earlier version of the driver reported
	 * "wrong address?" about a console that was printing the message.  The
	 * device tree is authoritative for both address and identity.
	 */
	pl011_attach(&sc, PL011_RPI5_BASE, PL011_RPI5_UARTCLK);
	cp->c_flags |= C_PRESENTIN | C_PRESENTOUT;
}

static int
pl011_cons_init(int arg __unused)
{
	/*
	 * Ensure receive is possible without disturbing baud or line control.
	 *
	 * Measured: the firmware leaves CR at 0x301, so UARTEN, TXE and RXE
	 * are all already set and this changes nothing.  It is kept because
	 * that is a fact about one firmware version, and a clear RXE would
	 * make the loader prompt indistinguishable from a dead keyboard.
	 *
	 * Note what this does NOT do: reprogram the baud rate.  The firmware
	 * has it at exactly 115200 (IBRD 24, FBRD 0 against a 44,236,800 Hz
	 * clk-uart) and pl011_configure() was shown to reproduce those same
	 * values, so reprogramming is provably safe but also provably
	 * pointless -- and it opens a window with the UART disabled for no
	 * gain.  The console we do not reconfigure is the one that cannot be
	 * broken by reconfiguring it.
	 */
	pl011_enable_rx(&sc);
	return (0);
}

static void
pl011_cons_putchar(int c)
{
	pl011_putc(&sc, c);
}

static int
pl011_cons_getchar(void)
{
	return (pl011_getc(&sc));	/* -1 when nothing is waiting */
}

static int
pl011_cons_poll(void)
{
	return (pl011_rxready(&sc));
}
