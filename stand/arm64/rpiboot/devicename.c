/*-
 * devicename.c -- device name parsing for the Raspberry Pi 5 loader.
 *
 * This is all of it, because libsa already does the work.  devparse() looks
 * the device up in devsw[], calls its dv_parsedev if it has one, and falls
 * back to default_parsedev() if it does not -- which is exactly the memory
 * disk's case, since md_dev declares no dv_parsedev.
 *
 * The first version of this file hand-rolled the parse and delegated
 * DEVT_DISK to disk_parsedev(), copying stand/uboot.  That was wrong here:
 * disk_parsedev() begins
 *
 *	if (strncmp(devspec, "disk", 4) == 0) ...
 *	else if (strncmp(devspec, "vdisk", 5) == 0) ...
 *	else return (EINVAL);
 *
 * so "md0:" matched neither name and every path failed with "bad path 'md0:'"
 * on hardware, even though lsdev listed md0 correctly.  The device switch was
 * fine; the parser was not.  stand/efi/libefi/devicename.c and
 * stand/libofw/devicename.c both just call devparse(), and so does this.
 */

#include <stand.h>
#include <string.h>

#include "bootstrap.h"
#include "librpiboot.h"

int
rpi_getdev(void **vdev, const char *devspec, const char **path)
{
	struct devdesc **dev = (struct devdesc **)vdev;
	int rv;

	/*
	 * A bare path, or anything with no device part, means "on the current
	 * device": parse currdev instead and hand the whole spec back as the
	 * path.
	 */
	if (devspec == NULL || *devspec == '/' || strchr(devspec, ':') == NULL) {
		rv = devparse(dev, getenv("currdev"), NULL);
		if (rv == 0 && path != NULL)
			*path = devspec;
		return (rv);
	}

	return (devparse(dev, devspec, path));
}
