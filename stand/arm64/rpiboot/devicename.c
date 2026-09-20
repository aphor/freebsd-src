/*-
 * devicename.c -- device name parsing for the Raspberry Pi 5 loader.
 *
 * Modelled on stand/uboot/devicename.c, which is the closest non-EFI
 * equivalent.  Shorter, because this loader has a small device switch: the
 * embedded memory disk today, and the SD card next.
 */

#include <stand.h>
#include <string.h>

#include "bootstrap.h"
#include "disk.h"
#include "librpiboot.h"

static int rpi_parsedev(struct devdesc **dev, const char *devspec,
    const char **path);

int
rpi_getdev(void **vdev, const char *devspec, const char **path)
{
	struct devdesc **dev = (struct devdesc **)vdev;
	int rv;

	/*
	 * A bare path, or anything with no device part, means "on the current
	 * device".  Parse currdev instead and hand the whole spec back as the
	 * path.
	 */
	if (devspec == NULL || devspec[0] == '/' ||
	    strchr(devspec, ':') == NULL) {
		rv = rpi_parsedev(dev, getenv("currdev"), NULL);
		if (rv == 0 && path != NULL)
			*path = devspec;
		return (rv);
	}

	return (rpi_parsedev(dev, devspec, path));
}

static int
rpi_parsedev(struct devdesc **dev, const char *devspec, const char **path)
{
	struct devdesc *idev;
	struct devsw *dv;
	const char *np;
	int i, err;

	if (devspec == NULL || strlen(devspec) < 2)
		return (EINVAL);

	for (i = 0, dv = NULL; devsw[i] != NULL; i++) {
		if (strncmp(devspec, devsw[i]->dv_name,
		    strlen(devsw[i]->dv_name)) == 0) {
			dv = devsw[i];
			break;
		}
	}
	if (dv == NULL)
		return (ENOENT);

	np = devspec + strlen(dv->dv_name);
	err = 0;

	switch (dv->dv_type) {
	case DEVT_NONE:
		idev = malloc(sizeof(*idev));
		if (idev == NULL)
			return (ENOMEM);
		idev->d_unit = 0;
		if (path != NULL)
			*path = (*np == ':') ? np + 1 : np;
		break;
#ifdef LOADER_DISK_SUPPORT
	case DEVT_DISK:
		/*
		 * md is DEVT_DISK but carries no partition table -- the
		 * embedded image is a bare filesystem, so disk_parsedev()
		 * leaves slice and partition unset and the whole device is
		 * opened.  That is why the image is built with makefs and not
		 * as a partitioned disk.
		 */
		idev = NULL;
		err = disk_parsedev(&idev, devspec, path);
		if (err != 0)
			return (err);
		break;
#endif
	default:
		return (EINVAL);
	}

	idev->d_dev = dv;

	if (dev == NULL)
		free(idev);
	else
		*dev = idev;

	return (0);
}

int
rpi_setcurrdev(struct env_var *ev, int flags, const void *value)
{
	struct devdesc *ncurr;
	int rv;

	rv = rpi_parsedev(&ncurr, value, NULL);
	if (rv != 0)
		return (rv);
	free(ncurr);

	return (mount_currdev(ev, flags, value));
}
