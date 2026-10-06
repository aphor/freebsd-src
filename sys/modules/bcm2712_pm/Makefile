#-
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2026 Jeremy McMillan
#

.PATH: ${SRCTOP}/sys/arm64/broadcom/bcm2712

KMOD=	bcm2712_pm
SRCS=	bcm2712_pm.c

SRCS+=	bus_if.h \
	device_if.h \
	hwreset_if.h \
	ofw_bus_if.h \
	opt_platform.h

EXPORT_SYMS=	bcm2712_pm_domain_get_by_ofw_idx \
		bcm2712_pm_domain_enable \
		bcm2712_pm_domain_disable \
		bcm2712_pm_domain_release

.include <bsd.kmod.mk>
