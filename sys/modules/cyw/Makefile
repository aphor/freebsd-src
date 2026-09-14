.PATH: ${SRCTOP}/sys/dev/cyw

KMOD=	cyw
SRCS=	cyw.c cyw_cfg.c cyw_events.c cyw_fw.c cyw_fwil.c
SRCS+=	cyw_scan.c cyw_sdio.c cyw_sdpcm.c cyw_security.c
SRCS+=	bus_if.h device_if.h sdio_if.h
SRCS+=	opt_inet.h opt_inet6.h opt_wlan.h

.include <bsd.kmod.mk>
