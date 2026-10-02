/*-
 * conf.c -- the device, filesystem and console tables for the Raspberry Pi 5
 * loader.
 *
 * These three arrays are the loader's whole view of the machine.  Order
 * matters in all of them: devices are probed in order, filesystems are tried
 * in order on open, and the first console with C_PRESENTIN|C_PRESENTOUT
 * becomes the default.
 */

#include <stand.h>

#include "bootstrap.h"
#include "librpiboot.h"

/*
 * Devices.
 *
 * The memory disk first: md needs no hardware driver, so the loader has a
 * filesystem whatever else is true.  Then the SD card, disk0 (rpi_sd.c).
 */
struct devsw *devsw[] = {
	&md_dev,
	&rpi_sd_dev,
	NULL
};

/*
 * Filesystems.
 *
 * UFS first because the embedded image is UFS, and so is a root or a /boot
 * on the card.  dosfs is for the firmware's FAT partition, where config.txt,
 * the loader itself and loader.env live.
 */
struct fs_ops *file_system[] = {
	&ufs_fsops,
#ifdef LOADER_MSDOS_SUPPORT
	&dosfs_fsops,
#endif
	NULL
};

/*
 * File formats live in exec.c, which defines file_formats[] alongside the
 * arm64 ELF handoff it implements.  Kept there rather than here because the
 * table and the l_exec it points at have to agree, and splitting them is how
 * they come to disagree.
 */

/*
 * Consoles.
 *
 * One, and it is the only reason any of this is debuggable: the SoC PL011 the
 * firmware is already printing through when we are entered.
 */
struct console *consoles[] = {
	&pl011_console,
	&rpi_fb_console,	/* HDMI, output only; see rpi_fb.c */
	NULL
};
