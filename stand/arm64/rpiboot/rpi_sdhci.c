/*-
 * rpi_sdhci.c -- SD card block reads for the Raspberry Pi 5 loader.
 *
 * Polled, PIO only.  See rpi_sdhci.h for which controller, the address
 * translation, the base clock, and why DMA and the cfg window are mostly
 * avoided.
 *
 * The single-block path and the initialisation are rpi5_modules.git
 * loader/sdhci.c as it ran on hardware; the multi-block read and the
 * recovery after a failed transfer are new here.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"
#include "rpi_sdhci.h"

/* Generous: a slow card answering a read can legitimately take a while. */
#define	CMD_TIMEOUT_US		500000
#define	DATA_TIMEOUT_US		2000000
#define	RESET_TIMEOUT_US	200000
#define	CLOCK_TIMEOUT_US	50000
#define	POWERUP_TIMEOUT_US	2000000

/* --- the generic timer, for bounded waits ------------------------------- */

static inline uint64_t
cntfrq(void)
{
	uint64_t v;

	__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(v));
	return (v);
}

static inline uint64_t
cntpct(void)
{
	uint64_t v;

	/* isb: the counter read can otherwise be speculated ahead. */
	__asm__ __volatile__("isb; mrs %0, cntpct_el0" : "=r"(v));
	return (v);
}

static inline uint64_t
deadline_us(uint64_t us)
{
	return (cntpct() + (cntfrq() / 1000000) * us);
}

static inline int
expired(uint64_t deadline)
{
	return (cntpct() >= deadline);
}

uint64_t
sd_timer_hz(void)
{
	return (cntfrq());
}

/* --- register access --------------------------------------------------- */

uint32_t
sdhci_read32(const struct sdhci *sc, uint32_t reg)
{
	return (*(volatile uint32_t *)(uintptr_t)(sc->host + reg));
}

uint16_t
sdhci_read16(const struct sdhci *sc, uint32_t reg)
{
	return (*(volatile uint16_t *)(uintptr_t)(sc->host + reg));
}

uint8_t
sdhci_read8(const struct sdhci *sc, uint32_t reg)
{
	return (*(volatile uint8_t *)(uintptr_t)(sc->host + reg));
}

static inline void
wr32(const struct sdhci *sc, uint32_t reg, uint32_t v)
{
	*(volatile uint32_t *)(uintptr_t)(sc->host + reg) = v;
}

static inline void
wr16(const struct sdhci *sc, uint32_t reg, uint16_t v)
{
	*(volatile uint16_t *)(uintptr_t)(sc->host + reg) = v;
}

static inline void
wr8(const struct sdhci *sc, uint32_t reg, uint8_t v)
{
	*(volatile uint8_t *)(uintptr_t)(sc->host + reg) = v;
}

static inline uint32_t
cfg_rd(const struct sdhci *sc, uint32_t reg)
{
	return (*(volatile uint32_t *)(uintptr_t)(sc->cfg + reg));
}

static inline void
cfg_wr(const struct sdhci *sc, uint32_t reg, uint32_t v)
{
	*(volatile uint32_t *)(uintptr_t)(sc->cfg + reg) = v;
}

void
sdhci_attach(struct sdhci *sc, uint64_t host, uint64_t cfg,
    uint32_t base_clock)
{
	memset(sc, 0, sizeof(*sc));
	sc->host = host;
	sc->cfg = cfg;
	sc->base_clock = base_clock;
}

/* --- command plumbing -------------------------------------------------- */

static int
wait_inhibit(const struct sdhci *sc, uint32_t mask)
{
	uint64_t d = deadline_us(CMD_TIMEOUT_US);

	while ((sdhci_read32(sc, SDHCI_PRESENT_STATE) & mask) != 0) {
		if (expired(d))
			return (-1);
	}
	return (0);
}

/*
 * Issue one command and wait for it to complete.
 *
 * `flags` carries the response type and the CRC/INDEX/DATA bits.  The command
 * index and flags are written as a single 16-bit store to SDHCI_COMMAND_FLAGS,
 * which is what actually starts the command: writing the two bytes separately
 * works on some controllers and starts a half-formed command on others, so it
 * is done the way the specification describes.
 */
static int
sdhci_cmd(struct sdhci *sc, uint8_t idx, uint32_t arg, uint16_t flags)
{
	uint32_t mask, is;
	uint64_t d;

	/* A data command must also wait for the data line to be free. */
	mask = SDHCI_CMD_INHIBIT;
	if ((flags & SDHCI_CMD_DATA) != 0 ||
	    (flags & 0x03) == SDHCI_CMD_RESP_SHORT_BUSY)
		mask |= SDHCI_DAT_INHIBIT;
	if (wait_inhibit(sc, mask) != 0) {
		sc->last_int = 0xdead0001;
		return (-1);
	}

	/* Clear stale status so the poll below cannot see a previous result. */
	wr32(sc, SDHCI_INT_STATUS, 0xffffffffU);

	wr32(sc, SDHCI_ARGUMENT, arg);
	wr16(sc, SDHCI_COMMAND_FLAGS,
	    (uint16_t)(((uint16_t)idx << 8) | (flags & 0xff)));

	d = deadline_us(CMD_TIMEOUT_US);
	for (;;) {
		is = sdhci_read32(sc, SDHCI_INT_STATUS);
		if ((is & SDHCI_INT_ERROR) != 0) {
			sc->last_int = is;
			/*
			 * An errored command leaves the command line needing a
			 * reset before the next one will be accepted.
			 */
			wr8(sc, SDHCI_SOFTWARE_RESET, SDHCI_RESET_CMD);
			return (-1);
		}
		if ((is & SDHCI_INT_RESPONSE) != 0)
			break;
		if (expired(d)) {
			sc->last_int = is | 0xdead0000;
			return (-1);
		}
	}

	/* Consume the response bit, leave data bits for the caller. */
	wr32(sc, SDHCI_INT_STATUS, SDHCI_INT_RESPONSE);
	return (0);
}

static uint32_t
resp32(const struct sdhci *sc)
{
	return (sdhci_read32(sc, SDHCI_RESPONSE));
}

static void
resp128(const struct sdhci *sc, uint32_t out[4])
{
	for (int i = 0; i < 4; i++)
		out[i] = sdhci_read32(sc, SDHCI_RESPONSE + i * 4);
}

/* --- controller setup -------------------------------------------------- */

static int
sdhci_do_reset(const struct sdhci *sc, uint8_t mask)
{
	uint64_t d = deadline_us(RESET_TIMEOUT_US);

	wr8(sc, SDHCI_SOFTWARE_RESET, mask);
	while ((sdhci_read8(sc, SDHCI_SOFTWARE_RESET) & mask) != 0) {
		if (expired(d))
			return (-1);
	}
	return (0);
}

/*
 * Set the card clock.
 *
 * SDHCI v3 10-bit divided mode: SDCLK = base / (2 * N), with N in the
 * divider field and N == 0 meaning "base clock, undivided".  The firmware's
 * own log is the check on this arithmetic:
 *
 *	SD HOST: 200000000 ... BUS: 50000000 Hz actual: 50000000 HZ div: 4 (2)
 *
 * -- a total division of 4 with 2 in the register, which is exactly
 * base / (2 * 2).  400 kHz from 200 MHz likewise wants N = 250.
 */
static int
sdhci_set_clock(const struct sdhci *sc, uint32_t hz)
{
	uint32_t n, reg;
	uint64_t d;

	/* Stop the clock before changing the divisor. */
	wr16(sc, SDHCI_CLOCK_CONTROL, 0);

	if (hz == 0)
		return (0);

	/*
	 * Pin behaviour follows the rate class.  The vendor driver writes this
	 * on every clock change (sdhci_bcm2712_set_clock); this driver only
	 * ever runs SD rates, so it always selects SD.
	 */
	reg = cfg_rd(sc, SDIO_CFG_SD_PIN_SEL);
	reg &= ~(uint32_t)SDIO_CFG_SD_PIN_SEL_MASK;
	reg |= SDIO_CFG_SD_PIN_SEL_SD;
	cfg_wr(sc, SDIO_CFG_SD_PIN_SEL, reg);

	if (hz >= sc->base_clock) {
		n = 0;
	} else {
		/* Smallest N whose resulting rate does not exceed hz. */
		for (n = 1; n < 1024; n++) {
			if (sc->base_clock / (2 * n) <= hz)
				break;
		}
		if (n >= 1024)
			n = 1023;
	}

	reg = ((n & 0xff) << SDHCI_DIVIDER_SHIFT) |
	      (((n >> 8) & 0x3) << SDHCI_DIVIDER_HI_SHIFT);
	wr16(sc, SDHCI_CLOCK_CONTROL, (uint16_t)(reg | SDHCI_CLOCK_INT_EN));

	d = deadline_us(CLOCK_TIMEOUT_US);
	while ((sdhci_read16(sc, SDHCI_CLOCK_CONTROL) &
	    SDHCI_CLOCK_INT_STABLE) == 0) {
		if (expired(d))
			return (-1);
	}

	wr16(sc, SDHCI_CLOCK_CONTROL,
	    (uint16_t)(reg | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN));
	return (0);
}

/* --- data transfer ----------------------------------------------------- */

/*
 * Wait for one INT_STATUS bit and consume it.  An error bit ends the wait;
 * `tag` marks a timeout in last_int.
 */
static int
wait_int(struct sdhci *sc, uint32_t bit, uint32_t tag)
{
	uint64_t d = deadline_us(DATA_TIMEOUT_US);
	uint32_t is;

	for (;;) {
		is = sdhci_read32(sc, SDHCI_INT_STATUS);
		if ((is & SDHCI_INT_ERROR) != 0) {
			sc->last_int = is;
			return (-1);
		}
		if ((is & bit) != 0)
			break;
		if (expired(d)) {
			sc->last_int = is | tag;
			return (-1);
		}
	}
	wr32(sc, SDHCI_INT_STATUS, bit);
	return (0);
}

/*
 * One block out of the data port: 128 32-bit reads.  memcpy, because buf
 * may be unaligned and this loader is built -mstrict-align.
 */
static void
read_fifo(const struct sdhci *sc, uint8_t *p)
{
	uint32_t w;
	int i;

	for (i = 0; i < SD_BLOCK_SIZE / 4; i++) {
		w = sdhci_read32(sc, SDHCI_BUFFER);
		memcpy(p, &w, sizeof(w));
		p += sizeof(w);
	}
}

static int
lba_to_arg(const struct sdhci *sc, uint64_t lba, uint32_t count,
    uint32_t *arg)
{
	/* Block versus byte addressing comes from the OCR's CCS bit. */
	if (!sc->block_addressed) {
		if (lba + count > (0xffffffffULL / SD_BLOCK_SIZE))
			return (-1);
		*arg = (uint32_t)(lba * SD_BLOCK_SIZE);
	} else {
		if (lba + count > 0xffffffffULL)
			return (-1);
		*arg = (uint32_t)lba;
	}
	return (0);
}

/* CMD17, as it ran in the bare-metal test. */
static int
sd_read_single(struct sdhci *sc, uint64_t lba, uint8_t *buf)
{
	uint32_t arg;

	if (lba_to_arg(sc, lba, 1, &arg) != 0)
		return (-1);

	wr16(sc, SDHCI_BLOCK_SIZE, SD_BLOCK_SIZE);
	wr16(sc, SDHCI_BLOCK_COUNT, 1);
	wr16(sc, SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ);

	if (sdhci_cmd(sc, SD_CMD_READ_SINGLE_BLOCK, arg,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA) != 0)
		return (-1);
	sc->rd_cmds++;

	/* The buffer fills, is read, and the transfer completes. */
	if (wait_int(sc, SDHCI_INT_DATA_AVAIL, 0xdead0002) != 0)
		return (-1);
	read_fifo(sc, buf);
	return (wait_int(sc, SDHCI_INT_DATA_END, 0xdead0003));
}

/*
 * CMD18 for `count` blocks, with the controller counting them and sending
 * CMD12 itself after the last (auto CMD12), as sys/dev/sdhci does for a
 * multi-block read.  Buffer Read Ready is raised once per block, and is
 * cleared before that block is read so that the next one cannot be missed.
 */
static int
sd_read_multi(struct sdhci *sc, uint64_t lba, uint32_t count, uint8_t *buf)
{
	uint32_t arg, i;

	if (lba_to_arg(sc, lba, count, &arg) != 0)
		return (-1);

	wr16(sc, SDHCI_BLOCK_SIZE, SD_BLOCK_SIZE);
	wr16(sc, SDHCI_BLOCK_COUNT, (uint16_t)count);
	wr16(sc, SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ | SDHCI_TRNS_MULTI |
	    SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_ACMD12);

	if (sdhci_cmd(sc, SD_CMD_READ_MULTIPLE_BLOCK, arg,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA) != 0)
		return (-1);
	sc->rd_cmds++;

	for (i = 0; i < count; i++) {
		if (wait_int(sc, SDHCI_INT_DATA_AVAIL, 0xdead0002) != 0)
			return (-1);
		read_fifo(sc, buf + (size_t)i * SD_BLOCK_SIZE);
	}
	return (wait_int(sc, SDHCI_INT_DATA_END, 0xdead0003));
}

/*
 * After a failed transfer: free the command and data lines, and take the
 * card out of a transfer it may still be in.  CMD12 to a card that is not
 * sending is an illegal command and goes unanswered; that is not an error
 * here, and last_int keeps the failure that brought us here.
 */
static void
sd_recover(struct sdhci *sc)
{
	uint32_t why = sc->last_int;

	(void)sdhci_do_reset(sc, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	(void)sdhci_cmd(sc, SD_CMD_STOP_TRANSMISSION, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_TYPE_ABORT);
	(void)sdhci_do_reset(sc, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	wr32(sc, SDHCI_INT_STATUS, 0xffffffffU);
	sc->last_int = why;
}

int
sd_read_blocks(struct sdhci *sc, uint64_t lba, uint32_t count, void *buf)
{
	uint8_t *p = buf;
	uint64_t t0 = cntpct();
	uint32_t i, n, tries;
	int error = 0;

	while (count > 0 && error == 0) {
		n = MIN(count, SD_MAX_MULTI);
		if (n > 1 && !sc->no_multi) {
			if (sd_read_multi(sc, lba, n, p) == 0)
				goto next;
			sd_recover(sc);
			/* A card or slot that keeps failing CMD18 gets CMD17. */
			if (++sc->multi_errors >= 4)
				sc->no_multi = 1;
		}
		for (i = 0; i < n && error == 0; i++) {
			for (tries = 0; ; tries++) {
				if (sd_read_single(sc, lba + i,
				    p + (size_t)i * SD_BLOCK_SIZE) == 0)
					break;
				sd_recover(sc);
				sc->single_errors++;
				if (tries >= 2) {
					error = -1;
					break;
				}
			}
		}
		if (error != 0)
			break;
next:
		sc->rd_blocks += n;
		lba += n;
		p += (size_t)n * SD_BLOCK_SIZE;
		count -= n;
	}
	sc->rd_ticks += cntpct() - t0;
	return (error);
}

/* --- card initialisation ----------------------------------------------- */

static int
sd_app_cmd(struct sdhci *sc, uint8_t acmd, uint32_t arg, uint16_t flags)
{
	if (sdhci_cmd(sc, SD_CMD_APP_CMD, (uint32_t)sc->rca << 16,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) != 0)
		return (-1);
	return (sdhci_cmd(sc, acmd, arg, flags));
}

int
sd_init(struct sdhci *sc)
{
	uint64_t d;
	uint32_t r;

	if (sdhci_do_reset(sc, SDHCI_RESET_ALL) != 0)
		return (-1);

	/* Power the slot at 3.3 V. */
	wr8(sc, SDHCI_POWER_CONTROL, SDHCI_POWER_330);
	wr8(sc, SDHCI_POWER_CONTROL, SDHCI_POWER_330 | SDHCI_POWER_ON);

	/* Poll rather than interrupt, but the status bits must still be set. */
	wr32(sc, SDHCI_INT_ENABLE, 0xffffffffU);
	wr32(sc, SDHCI_SIGNAL_ENABLE, 0);
	wr8(sc, SDHCI_TIMEOUT_CONTROL, 0x0e);

	/* Identification runs at 400 kHz or below, 1-bit. */
	wr8(sc, SDHCI_HOST_CONTROL, 0);
	if (sdhci_set_clock(sc, 400000) != 0)
		return (-1);
	delay(2000);

	sc->rca = 0;
	sc->ocr = 0;
	sc->block_addressed = 0;

	/* CMD0: to idle. */
	if (sdhci_cmd(sc, SD_CMD_GO_IDLE, 0, SDHCI_CMD_RESP_NONE) != 0)
		return (-1);
	delay(2000);

	/*
	 * CMD8: declare 2.7-3.6 V and a check pattern.  A card that echoes
	 * the pattern back is SD 2.0 or later, which is what lets ACMD41 ask
	 * for high capacity.  No answer means a version 1 card, or no card;
	 * both end here.
	 */
	if (sdhci_cmd(sc, SD_CMD_SEND_IF_COND, 0x1aa,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) != 0)
		return (-1);
	if ((resp32(sc) & 0xfff) != 0x1aa)
		return (-1);

	/* ACMD41 until the card leaves initialisation.  HCS asks for SDHC. */
	d = deadline_us(POWERUP_TIMEOUT_US);
	for (;;) {
		if (sd_app_cmd(sc, SD_ACMD_SD_SEND_OP_COND, 0x40ff8000,
		    SDHCI_CMD_RESP_SHORT) != 0)
			return (-1);
		r = resp32(sc);
		if ((r & SD_OCR_BUSY) != 0)
			break;
		if (expired(d))
			return (-1);
		delay(10000);
	}
	sc->ocr = r;
	sc->block_addressed = (r & SD_OCR_CCS) != 0 ? 1 : 0;

	/* CMD2: CID, and the card moves to identification state. */
	if (sdhci_cmd(sc, SD_CMD_ALL_SEND_CID, 0,
	    SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC) != 0)
		return (-1);
	resp128(sc, sc->cid);

	/* CMD3: the card publishes its relative address. */
	if (sdhci_cmd(sc, SD_CMD_SEND_REL_ADDR, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) != 0)
		return (-1);
	sc->rca = (uint16_t)(resp32(sc) >> 16);

	/* CMD9: CSD, while still in stand-by. */
	if (sdhci_cmd(sc, SD_CMD_SEND_CSD, (uint32_t)sc->rca << 16,
	    SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC) != 0)
		return (-1);
	resp128(sc, sc->csd);

	/* CMD7: select it, moving to transfer state. */
	if (sdhci_cmd(sc, SD_CMD_SELECT_CARD, (uint32_t)sc->rca << 16,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) != 0)
		return (-1);

	/* Four-bit bus, per bus-width = <4> in the device tree. */
	if (sd_app_cmd(sc, SD_ACMD_SET_BUS_WIDTH, 2,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) == 0)
		wr8(sc, SDHCI_HOST_CONTROL,
		    sdhci_read8(sc, SDHCI_HOST_CONTROL) | SDHCI_CTRL_4BITBUS);

	/*
	 * 512-byte blocks.  Harmless on a block-addressed card, which fixes
	 * the length at 512 regardless, but required on a byte-addressed one.
	 */
	if (sdhci_cmd(sc, SD_CMD_SET_BLOCKLEN, SD_BLOCK_SIZE,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX) != 0)
		return (-1);

	/*
	 * 25 MHz: default speed, no high-speed switch and no tuning.  The
	 * firmware's 50 MHz would need SDHCI_CTRL_HISPD and a CMD6 switch.
	 */
	if (sdhci_set_clock(sc, 25000000) != 0)
		return (-1);

	return (0);
}

/*
 * The controller's long response holds CSD[127:8] in bits [119:0], so CSD bit
 * n is response bit n - 8, and csd[i] holds response bits [32i+31:32i].
 */
uint64_t
sd_capacity_blocks(const struct sdhci *sc)
{
	uint32_t c_size, mult, bl_len;

	switch ((sc->csd[3] >> 22) & 0x3) {	/* CSD_STRUCTURE, CSD[127:126] */
	case 1:
		/* Version 2: C_SIZE is CSD[69:48]; (C_SIZE + 1) * 512 KiB. */
		c_size = (sc->csd[1] >> 8) & 0x3fffff;
		return (((uint64_t)c_size + 1) * 1024);
	case 0:
		/*
		 * Version 1: C_SIZE CSD[73:62], C_SIZE_MULT CSD[49:47],
		 * READ_BL_LEN CSD[83:80].
		 */
		c_size = ((sc->csd[2] & 0x3) << 10) | (sc->csd[1] >> 22);
		mult = (sc->csd[1] >> 7) & 0x7;
		bl_len = (sc->csd[2] >> 8) & 0xf;
		if (bl_len < 9)
			return (0);
		return (((uint64_t)c_size + 1) << (mult + 2 + bl_len - 9));
	default:
		return (0);
	}
}

void
sd_cid_product(const struct sdhci *sc, char out[6])
{
	/*
	 * The controller's long response holds CID[127:8] in bits [119:0].
	 * PNM is CID[103:64], so four characters come out of cid[2] and the
	 * fifth is the top byte of cid[1].  Checked against the CID the
	 * firmware printed for dunn's card, whose product name is "SD32G".
	 */
	out[0] = (char)((sc->cid[2] >> 24) & 0xff);
	out[1] = (char)((sc->cid[2] >> 16) & 0xff);
	out[2] = (char)((sc->cid[2] >> 8) & 0xff);
	out[3] = (char)(sc->cid[2] & 0xff);
	out[4] = (char)((sc->cid[1] >> 24) & 0xff);
	out[5] = '\0';

	for (int i = 0; i < 5; i++) {
		if (out[i] < 0x20 || out[i] > 0x7e)
			out[i] = '?';
	}
}
