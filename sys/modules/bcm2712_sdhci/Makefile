.PATH: ${SRCTOP}/sys/arm64/broadcom/bcm2712

KMOD=	sdhci_bcm2712
SRCS=	bcm2712_sdhci.c
SRCS+=	bus_if.h device_if.h ofw_bus_if.h mmcbr_if.h sdhci_if.h
SRCS+=	fdt_pinctrl_if.h regnode_if.h
SRCS+=	opt_mmccam.h opt_platform.h

.include <bsd.kmod.mk>
