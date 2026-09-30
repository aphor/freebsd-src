.PATH: ${SRCTOP}/sys/arm64/broadcom/rp1

KMOD=	rp1_clk
SRCS=	rp1_clk.c
SRCS+=	bus_if.h clkdev_if.h clknode_if.h device_if.h ofw_bus_if.h
SRCS+=	opt_platform.h

# dt-bindings/clock/raspberrypi,rp1-clocks.h: the kernel build has this path,
# a module build does not.
CFLAGS+=	-I${SRCTOP}/sys/contrib/device-tree/include

.include <bsd.kmod.mk>
