.PATH: ${SRCTOP}/sys/arm64/broadcom/rp1

KMOD=	rp1_gpio
SRCS=	rp1_gpio.c
SRCS+=	bus_if.h device_if.h fdt_pinctrl_if.h gpio_if.h ofw_bus_if.h
SRCS+=	opt_platform.h

.include <bsd.kmod.mk>
