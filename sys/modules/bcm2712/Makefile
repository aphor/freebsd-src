.PATH: ${SRCTOP}/sys/arm64/broadcom/bcm2712

KMOD=	bcm2712
SRCS=	bcm2712.c
SRCS+=	bus_if.h device_if.h ofw_bus_if.h
SRCS+=	opt_platform.h

EXPORT_SYMS=	bcm2712_read_cpu_temp
EXPORT_SYMS+=	bcm2712_get_softc
EXPORT_SYMS+=	bcm2712_pwm_set_config
EXPORT_SYMS+=	bcm2712_pwm_enable
EXPORT_SYMS+=	bcm2712_read_fan_rpm
EXPORT_SYMS+=	bcm2712_rp1_publish
EXPORT_SYMS+=	bcm2712_rp1_bar
EXPORT_SYMS+=	bcm2712_rp1_dma_tag
EXPORT_SYMS+=	bcm2712_rp1_defer
EXPORT_SYMS+=	bcm2712_rp1_undefer
EXPORT_SYMS+=	bcm2712_rp1_needs_pci

.include <bsd.kmod.mk>
