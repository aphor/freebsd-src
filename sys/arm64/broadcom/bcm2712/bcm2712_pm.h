/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * Written from a functional specification of the BCM2712 PM block.
 */

#ifndef _ARM64_BROADCOM_BCM2712_BCM2712_PM_H_
#define	_ARM64_BROADCOM_BCM2712_BCM2712_PM_H_

#include <dev/ofw/openfirm.h>

/*
 * Power domains of the BCM2712 PM block ("brcm,bcm2712-pm").
 *
 * FreeBSD has no power-domain framework, so the PM driver offers its
 * domains through the functions below.  A consumer names a domain the way
 * its device-tree node does: an entry of its "power-domains" property, a
 * phandle of the PM node followed by one cell, the domain number.
 *
 * Domain numbers are those of the "brcm,bcm2835-pm" binding.  The driver
 * accepts only the domains it implements; any other cell value, a cell
 * count other than one, or a phandle that is not an attached PM block is
 * refused when the handle is requested.
 *
 * A handle belongs to one consumer.  Each handle is either enabled or not;
 * the driver counts the enabled handles of each domain, switches the domain
 * on when that count leaves zero and off when it returns to zero.
 * Enabling an enabled handle, or disabling a disabled one, changes nothing.
 * Releasing an enabled handle disables it first.  The PM driver refuses to
 * detach while any handle exists.
 *
 * All four functions may sleep: call them from thread context.  Enabling
 * the V3D domain powers its parent domain on first, and fails with
 * ETIMEDOUT, leaving the handle disabled, if that does not complete.
 *
 * Reset lines of the same block are offered through hwreset(9), with the
 * reset numbers of the same binding.
 */

/* Power-domain cells. */
#define	BCM2712_PM_DOMAIN_GRAFX_V3D	1

/* Reset cells. */
#define	BCM2712_PM_RESET_V3D		0

struct bcm2712_pm_domain;
typedef struct bcm2712_pm_domain *bcm2712_pm_domain_t;

/*
 * Get a handle for entry 'idx' of the "power-domains" property of 'cnode',
 * or of the consumer's own node if 'cnode' is 0 or less.
 */
int	bcm2712_pm_domain_get_by_ofw_idx(device_t consumer, phandle_t cnode,
	    int idx, bcm2712_pm_domain_t *domp);
int	bcm2712_pm_domain_enable(bcm2712_pm_domain_t dom);
int	bcm2712_pm_domain_disable(bcm2712_pm_domain_t dom);
void	bcm2712_pm_domain_release(bcm2712_pm_domain_t dom);

#endif /* _ARM64_BROADCOM_BCM2712_BCM2712_PM_H_ */
