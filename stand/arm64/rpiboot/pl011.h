/*-
 * pl011.h -- ARM PrimeCell PL011 UART, for the Raspberry Pi 5 FreeBSD loader.
 *
 * This is the loader's first and most important device: with no EFI console
 * and no kernel yet, it is the only way anything reports what it is doing.
 *
 * WHICH UART, AND WHY THIS ONE
 *
 * The SoC-native PL011 at physical 0x107d001000.  Established from the
 * firmware's own runtime device tree (doc/fdt/rpi5-firmware-runtime.dts):
 *
 *	/aliases: console  = "/soc@107c000000/serial@7d001000"
 *	          serial10 = "/soc@107c000000/serial@7d001000"
 *	/chosen:  stdout-path = "serial10:115200n8"
 *
 *	serial@7d001000 {
 *		compatible = "arm,pl011", "arm,primecell";
 *		reg = <0x7d001000 0x200>;
 *		arm,primecell-periphid = <0x341011>;
 *		clocks = <&clk_uart &clk_vpu>;
 *		clock-names = "uartclk", "apb_pclk";
 *	};
 *
 * translated through the soc bus, whose ranges = <0x00 0x10 0x00 0x80000000>
 * maps child 0x7d001000 to parent 0x1000000000 + 0x7d001000 = 0x107d001000,
 * size 0x200.  That agrees exactly with what the running kernel reports:
 *
 *	uart0: <PrimeCell UART (PL011)> iomem 0x107d001000-0x107d0011ff
 *
 * The RP1 UARTs (serial@30000 and up, "arm,pl011-axi") are deliberately not
 * used: they sit behind PCIe, which is not up when the loader starts, and the
 * boot spec's suggestion of RP1 UART0 for early console is wrong for that
 * reason.  The native PL011 needs no bus brought up, and the VPU firmware is
 * already printing its own boot log through it before we are entered.
 *
 * OFFSETS ARE BYTES HERE
 *
 * Note carefully: sys/dev/uart/uart_dev_pl011.c defines its offsets in 32-bit
 * WORD units, because it reads through a scaled bus accessor -- its UART_FR
 * is 0x06, not 0x18.  Anything copied from that file into direct MMIO code
 * must be multiplied by four.  The values below are byte offsets, matching the
 * PL011 TRM, so they can be used directly.
 *
 * CLOCK AND BAUD
 *
 * clk-uart in the firmware's device tree is a fixed-clock at
 * clock-frequency = <0x2a30000> = 44,236,800 Hz.  For 115200 baud:
 *
 *	44236800 / (16 * 115200) = 24.000000
 *
 * exactly, so IBRD = 24 and FBRD = 0 with no rounding error.  An exact
 * integer divisor is good evidence that both the clock rate and the intended
 * line rate are what they appear to be.
 */

#ifndef	_RPI5_PL011_H_
#define	_RPI5_PL011_H_

#include <stdint.h>

/* The console UART on a Raspberry Pi 5, physical address. */
#define	PL011_RPI5_BASE		0x107d001000UL
/* clk-uart, from the firmware's device tree. */
#define	PL011_RPI5_UARTCLK	44236800U
#define	PL011_RPI5_BAUD		115200U

/* Register byte offsets (PL011 TRM). */
#define	PL011_DR		0x00	/* data			*/
#define	PL011_RSR_ECR		0x04	/* receive status / error clear	*/
#define	PL011_FR		0x18	/* flag			*/
#define	PL011_IBRD		0x24	/* integer baud divisor	*/
#define	PL011_FBRD		0x28	/* fractional baud divisor	*/
#define	PL011_LCR_H		0x2c	/* line control		*/
#define	PL011_CR		0x30	/* control			*/
#define	PL011_IFLS		0x34	/* FIFO level select	*/
#define	PL011_IMSC		0x38	/* interrupt mask		*/
#define	PL011_RIS		0x3c	/* raw interrupt status	*/
#define	PL011_MIS		0x40	/* masked interrupt status	*/
#define	PL011_ICR		0x44	/* interrupt clear		*/
#define	PL011_PERIPHID0		0xfe0	/* 0x11 on a PL011		*/
#define	PL011_PERIPHID1		0xfe4	/* 0x10			*/
#define	PL011_PERIPHID2		0xfe8	/* revision in bits 7:4	*/
#define	PL011_PERIPHID3		0xfec	/* 0x00			*/

/* DR, read side: error bits live above the data byte. */
#define	PL011_DR_DATA		0x0ff
#define	PL011_DR_FE		(1 << 8)	/* framing error	*/
#define	PL011_DR_PE		(1 << 9)	/* parity error		*/
#define	PL011_DR_BE		(1 << 10)	/* break		*/
#define	PL011_DR_OE		(1 << 11)	/* overrun		*/
#define	PL011_DR_ERROR	\
	(PL011_DR_FE | PL011_DR_PE | PL011_DR_BE | PL011_DR_OE)

/* FR */
#define	PL011_FR_CTS		(1 << 0)
#define	PL011_FR_BUSY		(1 << 3)	/* transmitting		*/
#define	PL011_FR_RXFE		(1 << 4)	/* receive FIFO empty	*/
#define	PL011_FR_TXFF		(1 << 5)	/* transmit FIFO full	*/
#define	PL011_FR_RXFF		(1 << 6)	/* receive FIFO full	*/
#define	PL011_FR_TXFE		(1 << 7)	/* transmit FIFO empty	*/

/* LCR_H */
#define	PL011_LCR_H_BRK		(1 << 0)
#define	PL011_LCR_H_PEN		(1 << 1)	/* parity enable	*/
#define	PL011_LCR_H_EPS		(1 << 2)	/* even parity		*/
#define	PL011_LCR_H_STP2	(1 << 3)	/* two stop bits	*/
#define	PL011_LCR_H_FEN		(1 << 4)	/* FIFO enable		*/
#define	PL011_LCR_H_WLEN8	(3 << 5)
#define	PL011_LCR_H_8N1		(PL011_LCR_H_WLEN8 | PL011_LCR_H_FEN)

/* CR */
#define	PL011_CR_UARTEN		(1 << 0)
#define	PL011_CR_LBE		(1 << 7)	/* loopback enable	*/
#define	PL011_CR_TXE		(1 << 8)	/* transmit enable	*/
#define	PL011_CR_RXE		(1 << 9)	/* receive enable	*/

#define	PL011_IMSC_MASK_ALL	0x7ff
#define	PL011_ICR_CLEAR_ALL	0x7ff

/*
 * A handle rather than a global, so the same code can serve a second UART and
 * so the base address can come from the device tree instead of a constant.
 * Zero-initialised is "not configured"; pl011_attach() must be called first.
 */
struct pl011 {
	uint64_t	base;		/* physical == virtual, MMU off	*/
	uint32_t	uartclk;	/* Hz; 0 means "do not reprogram" */
};

/*
 * Bind to a UART without touching a single register.  Safe to call before
 * anything is known about the hardware state.
 */
void	pl011_attach(struct pl011 *sc, uint64_t base, uint32_t uartclk);

/*
 * Read the PrimeCell peripheral ID, if it is reachable at all.
 *
 * DO NOT USE THIS AS A PRESENCE TEST ON A RASPBERRY PI 5.  Measured on
 * hardware 2026-09-20: PERIPHID0..3 all read 0x00000000 on a UART that was
 * demonstrably working and printing.  The reason is in the device tree --
 *
 *	reg = <0x7d001000 0x200>;
 *
 * The window this SoC decodes is 0x200 bytes, but the PrimeCell ID registers
 * live at 0xfe0..0xfec, outside it.  Reads there are not a wrong address;
 * they are a correct address in an undecoded part of the window, and they
 * return zero.
 *
 * Corroboration that this is by design rather than a fault: the vendor device
 * tree carries
 *
 *	arm,primecell-periphid = <0x341011>;
 *
 * which encodes exactly the id3/id2/id1/id0 = 0x00/0x34/0x10/0x11 a readable
 * PL011 would report.  A device tree only needs to state the peripheral ID
 * when software cannot read it from the hardware.
 *
 * So the device tree is authoritative for both the address and the identity,
 * and probing the ID registers can only produce a false negative.  An earlier
 * version of this driver had pl011_present() return false here and print
 * "NO -- PERIPHID mismatch, wrong address?" about a perfectly good console,
 * which is worse than having no check: it sends the reader hunting for an
 * addressing bug that does not exist.
 *
 * Returns 0 and stores the ID if it is readable, -1 if the window does not
 * expose it (all-zero), in which case *out is set to 0.
 */
int	pl011_periphid(const struct pl011 *sc, uint32_t *out);

/* The peripheral ID a readable PL011 reports; see arm,primecell-periphid. */
#define	PL011_PERIPHID_EXPECTED	0x341011U

/*
 * Configure for 8N1 at the given baud, enabling transmit and receive.
 *
 * Only call this when the inherited configuration is unusable.  The firmware
 * has already set this UART up and is printing through it when we are
 * entered, so the safest console is the one we do not reconfigure -- that is
 * what probe-k.bin relied on, and it worked on the first boot.  Reprogramming
 * means a window with the UART disabled, and a wrong uartclk turns a working
 * console into silence, which is the one failure that cannot report itself.
 *
 * Returns 0 on success, -1 if sc->uartclk is 0 or the divisor does not fit.
 */
int	pl011_configure(const struct pl011 *sc, uint32_t baud);

/*
 * Ensure receive is possible, without disturbing baud or line control.
 *
 * The firmware only ever writes to this UART, so whether it left CR.RXE set
 * is not something we can know from documentation -- and if it is clear, a
 * read-only console looks exactly like a dead keyboard.  This sets UARTEN,
 * TXE and RXE and leaves every other field alone.  Returns the previous CR so
 * the caller can report what the firmware actually left behind.
 */
uint32_t pl011_enable_rx(const struct pl011 *sc);

/* Blocking single-character output.  Expands \n to \r\n. */
void	pl011_putc(const struct pl011 *sc, int c);
/* Blocking, no newline translation. */
void	pl011_putc_raw(const struct pl011 *sc, int c);
/* NUL-terminated string, via pl011_putc(). */
void	pl011_puts(const struct pl011 *sc, const char *s);
/* Wait until the transmitter has drained. */
void	pl011_drain(const struct pl011 *sc);

/* Nonzero if a character is waiting. */
int	pl011_rxready(const struct pl011 *sc);
/* Next character, or -1 if none is waiting.  Never blocks. */
int	pl011_getc(const struct pl011 *sc);
/* Blocking read. */
int	pl011_getc_wait(const struct pl011 *sc);

/*
 * Verify the receive datapath using the PL011's internal loopback, with no
 * external input and nothing attached.
 *
 * This exists because the obvious test is not actually available: the host
 * holds the serial port open, so there is no terminal for anyone to type
 * into, and an input test that waits for a human measures the test harness
 * rather than the hardware.  CR.LBE ties the transmitter to the receiver
 * inside the peripheral, so writing a byte and reading it back exercises
 * RX enable, the FR.RXFE poll, the DR read and the error bits -- everything
 * except the physical pin.
 *
 * The console is unusable while loopback is set, because output goes to the
 * receiver instead of the wire.  So this drains first, prints nothing while
 * the bit is set, restores CR unconditionally before returning, and bounds
 * the wait with the generic timer.  Leaving LBE set would silence the console
 * permanently, which is the one outcome that cannot report itself.
 *
 * Returns the byte read back, or -1 on timeout.  *prev_cr gets the CR value
 * that was in force beforehand.
 */
int	pl011_loopback_byte(const struct pl011 *sc, uint8_t tx,
	    uint32_t *prev_cr, uint64_t timeout_ticks);

/* Hex output helpers; the loader has no printf this early. */
void	pl011_puthex(const struct pl011 *sc, uint64_t v, int digits);

#endif	/* _RPI5_PL011_H_ */
