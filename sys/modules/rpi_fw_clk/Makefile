#-
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2026 Jeremy McMillan
#

.PATH: ${SRCTOP}/sys/arm64/broadcom/bcm2712

KMOD=	rpi_fw_clk
SRCS=	rpi_fw_clk.c

SRCS+=	bus_if.h \
	clknode_if.h \
	device_if.h \
	ofw_bus_if.h \
	opt_platform.h

EXPORT_SYMS=	rpi_fw_clk_max_rate \
		rpi_fw_clk_query

.include <bsd.kmod.mk>
