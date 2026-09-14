.PATH: ${SRCTOP}/sys/arm64/broadcom/rp1

KMOD=	rp1_eth
SRCS=	rp1_eth.c rp1_eth_cfg.c
SRCS+=	bus_if.h device_if.h ofw_bus_if.h
SRCS+=	opt_inet.h opt_inet6.h opt_platform.h

.include <bsd.kmod.mk>
