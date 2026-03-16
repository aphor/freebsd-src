.PATH:	${SRCTOP}/contrib/mhs35_vt

KMOD=	mhs35_vt

SRCS=	mhs35.c mhs35_vt.c

SRCS+=	\
	bus_if.h \
	device_if.h \
	gpio_if.h \
	ofw_bus_if.h \
	spibus_if.h \
	opt_platform.h \

CFLAGS+=	-I${SRCTOP}/contrib/mhs35_vt

.include <bsd.kmod.mk>
