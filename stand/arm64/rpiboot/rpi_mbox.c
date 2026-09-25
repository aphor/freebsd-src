/*-
 * rpi_mbox.c -- the VPU property mailbox, and TryBoot boot selection.
 *
 * WHY THE LOADER TALKS TO THE VPU
 *
 * The firmware reads one of two files when it boots: config.txt normally, or
 * tryboot.txt if a one-shot flag is set when the board resets.  The flag is
 * cleared as the firmware consumes it, so exactly one boot uses tryboot.txt
 * and every later one is back on config.txt.  Setting that flag from the
 * loader prompt selects the next boot without touching the GPIO4 jumper, and
 * because it cannot persist, a tryboot.txt that fails to boot costs one power
 * cycle and never more.
 *
 * The mechanism is the vendor Linux driver's, drivers/firmware/raspberrypi.c
 * rpi_firmware_notify_reboot(): SET_REBOOT_FLAGS with bit 0 set, then
 * NOTIFY_REBOOT, then reset.
 *
 * ADDRESSING, MEASURED
 *
 * mailbox@7c013880, "brcm,bcm2835-mbox", reg = <0x7c013880 0x40>, translated
 * through /soc's ranges to PA 0x107c013880.  The property buffer is handed to
 * the VPU as a plain physical address: no 0xC0000000-style VideoCore alias.
 * The device tree implied that (the firmware node carries an empty
 * dma-ranges), and rpi5_modules.git/loader/mboxtest.bin measured it on this
 * board: four GET tags answered, and the board revision and serial matched
 * what the firmware printed on the same boot.
 *
 * With the MMU off every access is Device-nGnRnE, so the buffer needs no cache
 * maintenance before the VPU reads it or after it writes back.
 *
 * Register layout and MBOX_MSG from sys/arm/broadcom/bcm2835/bcm2835_mbox.c;
 * tags from the vendor include/soc/bcm2835/raspberrypi-firmware.h.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"
#include "librpiboot.h"

#define	MBOX_BASE		0x107c013880UL
#define	MBOX_REG_READ		0x00	/* mailbox 0: VPU -> ARM */
#define	MBOX_REG_STATUS		0x18
#define	  MBOX_STATUS_FULL	0x80000000U
#define	  MBOX_STATUS_EMPTY	0x40000000U
#define	MBOX_REG_WRITE		0x20	/* mailbox 1: ARM -> VPU */

#define	MBOX_CHAN_PROPERTY	8
#define	MBOX_MSG(chan, data)	(((data) & ~0xfU) | ((chan) & 0xfU))

#define	MBOX_TIMEOUT_US		1000000

#define	RESP_SUCCESS		0x80000000U

#define	TAG_GET_FIRMWARE_REVISION	0x00000001U
#define	TAG_GET_BOARD_REVISION		0x00010002U
#define	TAG_GET_BOARD_SERIAL		0x00010004U
#define	TAG_GET_TEMPERATURE		0x00030006U
#define	TAG_NOTIFY_REBOOT		0x00030048U
#define	TAG_GET_REBOOT_FLAGS		0x00030064U
#define	TAG_SET_REBOOT_FLAGS		0x00038064U

#define	REBOOT_FLAG_TRYBOOT		0x1U

static inline uint32_t
mbox_read(uint32_t off)
{
	return (*(volatile uint32_t *)(MBOX_BASE + off));
}

static inline void
mbox_write(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(MBOX_BASE + off) = v;
}

/* Wait until (STATUS & mask) == want.  Returns 0, or ETIMEDOUT. */
static int
mbox_wait(uint32_t mask, uint32_t want)
{
	int us;

	for (us = 0; us < MBOX_TIMEOUT_US; us++) {
		if ((mbox_read(MBOX_REG_STATUS) & mask) == want)
			return (0);
		delay(1);
	}
	return (ETIMEDOUT);
}

/*
 * One property-channel transaction on a caller-built buffer.  The buffer must
 * be 16-byte aligned and below 4 GB: the message carries a 32-bit address
 * with the channel in its low four bits.
 */
static int
mbox_property(uint32_t *buf)
{
	uint64_t pa = (uint64_t)(uintptr_t)buf;
	uint32_t msg;
	int us;

	if ((pa & 0xf) != 0 || pa > 0xffffffffUL)
		return (EINVAL);

	/* Discard anything stale in the VPU -> ARM mailbox. */
	for (us = 0; us < MBOX_TIMEOUT_US; us++) {
		if ((mbox_read(MBOX_REG_STATUS) & MBOX_STATUS_EMPTY) != 0)
			break;
		(void)mbox_read(MBOX_REG_READ);
	}

	if (mbox_wait(MBOX_STATUS_FULL, 0) != 0)
		return (ETIMEDOUT);
	mbox_write(MBOX_REG_WRITE, MBOX_MSG(MBOX_CHAN_PROPERTY, (uint32_t)pa));

	/* Take the reply on our channel; anything else is not ours. */
	for (;;) {
		if (mbox_wait(MBOX_STATUS_EMPTY, 0) != 0)
			return (ETIMEDOUT);
		msg = mbox_read(MBOX_REG_READ);
		if ((msg & 0xf) == MBOX_CHAN_PROPERTY)
			break;
	}

	if (buf[1] != RESP_SUCCESS)
		return (EIO);
	return (0);
}

/*
 * A single tag.  inlen bytes of val are sent; up to vallen bytes of the reply
 * are copied back into val.  Same buffer layout as tools/vcio_test.c.
 */
static int
rpi_mbox_tag(uint32_t tag, uint32_t *val, uint32_t vallen, uint32_t inlen)
{
	static uint32_t buf[32] __aligned(16);
	uint32_t i, n, words;
	int error;

	words = roundup2(vallen, 4) / 4;
	if (words > 24)
		return (EINVAL);

	n = 0;
	buf[n++] = 0;			/* total size, below */
	buf[n++] = 0;			/* process request */
	buf[n++] = tag;
	buf[n++] = words * 4;		/* value buffer size */
	buf[n++] = inlen;		/* request length */
	for (i = 0; i < words; i++)
		buf[n++] = (val != NULL && i * 4 < inlen) ? val[i] : 0;
	buf[n++] = 0;			/* end tag */
	buf[0] = n * 4;

	if ((error = mbox_property(buf)) != 0)
		return (error);
	if ((buf[4] & RESP_SUCCESS) == 0)
		return (ENOTSUP);
	for (i = 0; val != NULL && i < words; i++)
		val[i] = buf[5 + i];
	return (0);
}

static int
rpi_mbox_get32(uint32_t tag, uint32_t *v)
{
	return (rpi_mbox_tag(tag, v, 4, 0));
}

static const char *
mbox_strerror(int error)
{
	switch (error) {
	case ETIMEDOUT:	return ("no reply from the VPU");
	case EIO:	return ("request not processed");
	case ENOTSUP:	return ("tag not answered by this firmware");
	case EINVAL:	return ("bad buffer");
	default:	return ("error");
	}
}

/* One line saying which config file the firmware booted us with. */
void
rpi_print_boot_config(void)
{
	int tb;

#ifdef LOADER_FDT_SUPPORT
	tb = rpi_fdt_tryboot();
#else
	tb = -1;
#endif
	printf("   This boot used:    %s\n",
	    tb == 1 ? "tryboot.txt (TryBoot one-shot)" :
	    tb == 0 ? "config.txt" :
	    "unknown (no /chosen/bootloader/tryboot)");
}

static int
command_vcinfo(int argc __unused, char *argv[] __unused)
{
	uint32_t v, serial[2], temp[2];
	int error;

	printf("VPU property mailbox at 0x%lx:\n", MBOX_BASE);

	if ((error = rpi_mbox_get32(TAG_GET_FIRMWARE_REVISION, &v)) != 0) {
		printf("   firmware revision: %s\n", mbox_strerror(error));
		return (CMD_ERROR);
	}
	printf("   Firmware revision: 0x%08x\n", v);

	if ((error = rpi_mbox_get32(TAG_GET_BOARD_REVISION, &v)) == 0)
		printf("   Board revision:    0x%08x\n", v);
	else
		printf("   Board revision:    %s\n", mbox_strerror(error));

	if ((error = rpi_mbox_tag(TAG_GET_BOARD_SERIAL, serial, 8, 0)) == 0)
		printf("   Board serial:      0x%08x%08x\n", serial[1],
		    serial[0]);
	else
		printf("   Board serial:      %s\n", mbox_strerror(error));

	/*
	 * SoC temperature, because nothing at this prompt controls the fan.
	 * Worth a look before resetting into EDK2, whose early boot is known
	 * to overheat a board that restarts warm.  Request: sensor id 0;
	 * reply: id, then millidegrees C.
	 */
	temp[0] = 0;
	temp[1] = 0;
	if ((error = rpi_mbox_tag(TAG_GET_TEMPERATURE, temp, 8, 4)) == 0)
		printf("   SoC temperature:   %u.%01u C\n", temp[1] / 1000,
		    (temp[1] % 1000) / 100);
	else
		printf("   SoC temperature:   %s\n", mbox_strerror(error));

	if ((error = rpi_mbox_get32(TAG_GET_REBOOT_FLAGS, &v)) == 0)
		printf("   Reboot flags:      0x%08x (tryboot %s for the "
		    "next boot)\n", v,
		    (v & REBOOT_FLAG_TRYBOOT) ? "SET" : "clear");
	else
		printf("   Reboot flags:      %s\n", mbox_strerror(error));

	rpi_print_boot_config();
	return (CMD_OK);
}
COMMAND_SET(vcinfo, "vcinfo",
    "show VPU firmware, board, temperature and TryBoot state",
    command_vcinfo);

/*
 * tryboot: make the next boot, and only the next boot, read tryboot.txt.
 *
 * The flag is read back before resetting.  A SET that the firmware answers
 * but does not act on would otherwise look exactly like a firmware that
 * ignores tryboot.txt, and those need different fixes.
 */
static int
command_tryboot(int argc, char *argv[])
{
	uint32_t v;
	int error, noreset;

	noreset = (argc > 1 && strcmp(argv[1], "-n") == 0);

	v = REBOOT_FLAG_TRYBOOT;
	if ((error = rpi_mbox_tag(TAG_SET_REBOOT_FLAGS, &v, 4, 4)) != 0) {
		printf("SET_REBOOT_FLAGS failed: %s\n", mbox_strerror(error));
		return (CMD_ERROR);
	}
	if ((error = rpi_mbox_get32(TAG_GET_REBOOT_FLAGS, &v)) != 0) {
		printf("GET_REBOOT_FLAGS failed: %s\n", mbox_strerror(error));
		return (CMD_ERROR);
	}
	printf("Reboot flags now 0x%08x.\n", v);
	if ((v & REBOOT_FLAG_TRYBOOT) == 0) {
		printf("The firmware accepted SET_REBOOT_FLAGS but the tryboot "
		    "bit did not stick;\nnot resetting.\n");
		return (CMD_ERROR);
	}

	if (noreset) {
		printf("TryBoot armed; it takes effect at the next reset.\n");
		return (CMD_OK);
	}

	/* As rpi_firmware_notify_reboot() does, before the reset itself. */
	if ((error = rpi_mbox_tag(TAG_NOTIFY_REBOOT, NULL, 0, 0)) != 0)
		printf("NOTIFY_REBOOT: %s (resetting anyway)\n",
		    mbox_strerror(error));

	printf("TryBoot armed: the next boot reads tryboot.txt, once.\n");
	rpi_psci_reset();

	/* Only reached if the reset failed; rpi_psci_reset() said so. */
	for (;;)
		__asm__ __volatile__("wfi");
	return (CMD_OK);
}
COMMAND_SET(tryboot, "tryboot",
    "reboot once using tryboot.txt (-n: arm only)", command_tryboot);

/*
 * untryboot: clear an armed flag, for "tryboot -n" followed by a change of
 * mind.  Without it the only way to disarm is to let the flag be consumed.
 */
static int
command_untryboot(int argc __unused, char *argv[] __unused)
{
	uint32_t v = 0;
	int error;

	if ((error = rpi_mbox_tag(TAG_SET_REBOOT_FLAGS, &v, 4, 4)) != 0 ||
	    (error = rpi_mbox_get32(TAG_GET_REBOOT_FLAGS, &v)) != 0) {
		printf("reboot flags: %s\n", mbox_strerror(error));
		return (CMD_ERROR);
	}
	printf("Reboot flags now 0x%08x.\n", v);
	return (CMD_OK);
}
COMMAND_SET(untryboot, "untryboot", "clear an armed TryBoot flag",
    command_untryboot);
