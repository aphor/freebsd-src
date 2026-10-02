/*-
 * rpi_sd.c -- the SD card as the loader's disk0.
 *
 * rpi_sdhci.c reads blocks from the card in the Pi 5's slot; this file makes
 * a libsa disk of it, the way stand/uboot/uboot_disk.c does for U-Boot's
 * storage: stand/common/disk.c and part.c find the MBR, GPT and BSD-label
 * partitions, and conf.c's filesystems (UFS, FAT) open them, so
 *
 *	disk0s1:/config.txt		the firmware's FAT
 *	disk0s2a:/boot/kernel/kernel	a UFS on the card
 *
 * work at the prompt, in loader.conf, and as currdev.  Reads go through the
 * loader's block cache.  There are no writes: nextboot(8), which has the
 * loader rewrite a file, does not work from this disk.
 *
 * The card is initialised from reset in dv_init, whatever state the firmware
 * left it in: that path gives the card's size, and it is the one that works
 * for a card the firmware did not boot from.  With no card it fails in about
 * a second and the loader carries on with its memory disk.  The kernel's own
 * driver resets the controller again when it attaches.
 */

#include <stand.h>
#include <sys/param.h>
#include <sys/disk.h>
#include <stdarg.h>

#include "bootstrap.h"
#include "disk.h"
#include "librpiboot.h"
#include "rpi_sdhci.h"

static struct sdhci sd;
static bool sd_present;
static uint64_t sd_blocks;
static void *sd_bcache;
static const char *sd_why = "not probed";

static int sd_dv_init(void);
static int sd_dv_strategy(void *, int, daddr_t, size_t, char *, size_t *);
static int sd_dv_open(struct open_file *, ...);
static int sd_dv_close(struct open_file *);
static int sd_dv_ioctl(struct open_file *, u_long, void *);
static int sd_dv_print(int);
static void sd_dv_cleanup(void);

/* "disk", because disk_parsedev() accepts no other name. */
struct devsw rpi_sd_dev = {
	.dv_name = "disk",
	.dv_type = DEVT_DISK,
	.dv_init = sd_dv_init,
	.dv_strategy = sd_dv_strategy,
	.dv_open = sd_dv_open,
	.dv_close = sd_dv_close,
	.dv_ioctl = sd_dv_ioctl,
	.dv_print = sd_dv_print,
	.dv_cleanup = sd_dv_cleanup,
	.dv_fmtdev = disk_fmtdev,
	.dv_parsedev = disk_parsedev,
};

static int
sd_dv_init(void)
{
	char name[6];

	sdhci_attach(&sd, SDHCI_RPI5_HOST_BASE, SDHCI_RPI5_CFG_BASE,
	    SDHCI_RPI5_BASE_CLOCK);
	if (sd_init(&sd) != 0) {
		sd_why = "no card, or it did not initialise";
		printf("SD card: none (INT_STATUS 0x%08x)\n", sd.last_int);
		return (ENXIO);
	}
	sd_blocks = sd_capacity_blocks(&sd);
	if (sd_blocks == 0) {
		sd_why = "the card's CSD gives no size";
		printf("SD card: unknown CSD %08x %08x %08x %08x\n",
		    sd.csd[3], sd.csd[2], sd.csd[1], sd.csd[0]);
		return (ENXIO);
	}
	sd_present = true;
	sd_why = NULL;
	bcache_add_dev(1);
	sd_cid_product(&sd, name);
	printf("SD card: %s, %lu MiB, 4-bit at 25 MHz, polled\n", name,
	    (unsigned long)(sd_blocks / 2048));
	return (0);
}

static void
sd_dv_cleanup(void)
{
	if (sd_bcache != NULL) {
		bcache_free(sd_bcache);
		sd_bcache = NULL;
	}
}

/* Below the block cache: absolute block numbers, whole blocks. */
static int
sd_realstrategy(void *devdata __unused, int rw, daddr_t blk, size_t size,
    char *buf, size_t *rsize)
{
	if (rsize != NULL)
		*rsize = 0;
	if ((rw & F_MASK) != F_READ)
		return (EROFS);
	if (size % SD_BLOCK_SIZE != 0)
		return (EIO);
	if (blk < 0 || (uint64_t)blk + size / SD_BLOCK_SIZE > sd_blocks)
		return (EIO);
	if (sd_read_blocks(&sd, (uint64_t)blk, size / SD_BLOCK_SIZE,
	    buf) != 0) {
		printf("SD card: read of %zu blocks at %jd failed "
		    "(INT_STATUS 0x%08x)\n", size / SD_BLOCK_SIZE,
		    (intmax_t)blk, sd.last_int);
		return (EIO);
	}
	if (rsize != NULL)
		*rsize = size;
	return (0);
}

static int
sd_dv_strategy(void *devdata, int rw, daddr_t blk, size_t size, char *buf,
    size_t *rsize)
{
	struct disk_devdesc *dev = devdata;
	struct bcache_devdata bcd;

	if (dev == NULL)
		return (EINVAL);
	if (!sd_present)
		return (ENXIO);
	bcd.dv_strategy = sd_realstrategy;
	bcd.dv_devdata = devdata;
	bcd.dv_cache = sd_bcache;
	return (bcache_strategy(&bcd, rw, blk + dev->d_offset, size, buf,
	    rsize));
}

static int
sd_opendev(struct disk_devdesc *dev)
{
	if (!sd_present || dev->dd.d_unit != 0)
		return (ENXIO);
	/* Kept for the life of the loader, as libefi keeps a disk's. */
	if (sd_bcache == NULL)
		sd_bcache = bcache_allocate();
	return (disk_open(dev, sd_blocks * SD_BLOCK_SIZE, SD_BLOCK_SIZE));
}

static int
sd_dv_open(struct open_file *f, ...)
{
	struct disk_devdesc *dev;
	va_list ap;

	va_start(ap, f);
	dev = va_arg(ap, struct disk_devdesc *);
	va_end(ap);
	return (sd_opendev(dev));
}

static int
sd_dv_close(struct open_file *f)
{
	return (disk_close((struct disk_devdesc *)f->f_devdata));
}

static int
sd_dv_ioctl(struct open_file *f, u_long cmd, void *data)
{
	struct disk_devdesc *dev = (struct disk_devdesc *)f->f_devdata;
	int rc;

	rc = disk_ioctl(dev, cmd, data);
	if (rc != ENOTTY)
		return (rc);

	switch (cmd) {
	case DIOCGSECTORSIZE:
		*(u_int *)data = SD_BLOCK_SIZE;
		break;
	case DIOCGMEDIASIZE:
		*(uint64_t *)data = sd_blocks * SD_BLOCK_SIZE;
		break;
	default:
		return (ENOTTY);
	}
	return (0);
}

static int
sd_dv_print(int verbose)
{
	struct disk_devdesc dev;
	char line[80];
	int ret;

	if (!sd_present)
		return (0);
	printf("%s devices:", rpi_sd_dev.dv_name);
	if ((ret = pager_output("\n")) != 0)
		return (ret);

	dev.dd.d_dev = &rpi_sd_dev;
	dev.dd.d_unit = 0;
	dev.d_slice = D_SLICENONE;
	dev.d_partition = D_PARTNONE;
	snprintf(line, sizeof(line), "    disk0:    SD card, %lu MiB\n",
	    (unsigned long)(sd_blocks / 2048));
	if ((ret = pager_output(line)) != 0)
		return (ret);
	if (sd_opendev(&dev) == 0) {
		ret = disk_print(&dev, "    disk0", verbose);
		disk_close(&dev);
	}
	return (ret);
}

/* Is there a card to read?  For main.c's choice of boot device. */
bool
rpi_sd_present(void)
{
	return (sd_present);
}

/*
 * One line on what the card's reads cost, printed before the kernel is
 * entered: nothing else measures a polled driver on every boot.
 */
void
rpi_sd_report(void)
{
	uint64_t hz = sd_timer_hz();
	uint64_t ms;

	if (!sd_present || sd.rd_blocks == 0 || hz == 0)
		return;
	ms = sd.rd_ticks * 1000 / hz;
	printf("SD card: %lu KiB read in %lu ms", (unsigned long)(sd.rd_blocks / 2),
	    (unsigned long)ms);
	if (ms > 0)
		printf(" (%lu KiB/s)", (unsigned long)(sd.rd_blocks * 500 / ms));
	printf(", %u commands", sd.rd_cmds);
	if (sd.multi_errors != 0 || sd.single_errors != 0)
		printf(", %u multi-block and %u single-block retries%s",
		    sd.multi_errors, sd.single_errors,
		    sd.no_multi ? " (multi-block given up)" : "");
	printf("\n");
}

COMMAND_SET(sdinfo, "sdinfo", "show the SD card and its controller",
    command_sdinfo);

static int
command_sdinfo(int argc __unused, char *argv[] __unused)
{
	char name[6];

	printf("SDHCI at 0x%lx: version 0x%04x, capabilities 0x%08x, "
	    "present state 0x%08x\n", (unsigned long)sd.host,
	    sdhci_read16(&sd, SDHCI_HOST_VERSION),
	    sdhci_read32(&sd, SDHCI_CAPABILITIES),
	    sdhci_read32(&sd, SDHCI_PRESENT_STATE));
	printf("    clock control 0x%04x, host control 0x%02x, "
	    "power control 0x%02x\n",
	    sdhci_read16(&sd, SDHCI_CLOCK_CONTROL),
	    sdhci_read8(&sd, SDHCI_HOST_CONTROL),
	    sdhci_read8(&sd, SDHCI_POWER_CONTROL));
	if (!sd_present) {
		printf("No disk0: %s (INT_STATUS 0x%08x).\n", sd_why,
		    sd.last_int);
		return (CMD_OK);
	}
	sd_cid_product(&sd, name);
	printf("Card \"%s\": OCR 0x%08x (%s addressed), RCA 0x%04x, "
	    "%lu blocks (%lu MiB)\n", name, sd.ocr,
	    sd.block_addressed ? "block" : "byte", sd.rca,
	    (unsigned long)sd_blocks, (unsigned long)(sd_blocks / 2048));
	printf("    CID %08x %08x %08x %08x\n", sd.cid[3], sd.cid[2],
	    sd.cid[1], sd.cid[0]);
	printf("    CSD %08x %08x %08x %08x\n", sd.csd[3], sd.csd[2],
	    sd.csd[1], sd.csd[0]);
	if (sd.rd_blocks == 0)
		printf("Nothing read yet.\n");
	else
		rpi_sd_report();
	return (CMD_OK);
}
