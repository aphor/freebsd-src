#
# stand/arm64/rpiboot -- FreeBSD loader for the Raspberry Pi 5, booted
# directly by the VPU firmware with no EFI anywhere.
#
# The firmware loads this as "the kernel" (config.txt kernel=rpiboot.bin,
# os_check=0) per the Linux arm64 boot protocol, which was measured on
# hardware rather than assumed -- see the entry contract in
# rpi5_modules.git/doc/LOADER_ZIMAGE.md:
#
#	EL2, MMU off, x0 = DTB physical address, x1..x3 = 0
#	image loaded at 0x200000
#
# Output is a raw binary carrying a 64-byte arm64 Image header, the same
# recipe stand/uboot uses for ubldr.bin: link, then objcopy -O binary.  It is
# linked at its load address so nothing needs relocating at runtime.
#

LOADER?=	rpiboot
PROG=		${LOADER}.sym
INTERNALPROG=
NEWVERSWHAT=	"Raspberry Pi 5 loader" ${MACHINE_CPUARCH}
INSTALLFLAGS=	-b

# The interpreter.  "simp" rather than lua or 4th on purpose: it needs no
# script files on a filesystem, so the loader reaches an interactive prompt
# even if the memory disk is empty or broken.  A prompt that survives its own
# filesystem being wrong is worth more during bring-up than a scripted menu.
LOADER_INTERP?=	simp

# Memory disk.  This is what gives the loader a filesystem without any
# storage driver at all: stand/common/md.c reserves MD_IMAGE_SIZE bytes of
# .data pre-filled with the marker string "MFS Filesystem goes here", and
# sys/tools/embed_mfs.sh patches a real image over it after the link.  That
# script has a non-ELF path using strings(1), so it works on the raw binary.
LOADER_MD_SUPPORT=	yes
#
# 16 MiB, so the image can carry a stripped kernel and the cyw43455
# firmware.
#
# This is what lets the handoff be tested with one new variable instead of
# two: the loader has no storage driver yet, so without an embedded kernel
# there would be nothing to boot, and wiring the SD card at the same time
# would leave a failure ambiguous between the two.  The firmware has to be
# here too, and preloaded by loader.rc, because cyw(4) attaches before root
# is mounted.
#
# 14 MiB held the 12.45 MB kernel alone; with the 0.61 MB of firmware added,
# makefs ran out of space.  16 MiB makes the loader about 17 MiB, which the
# firmware loads at 0x200000 and which ends near 0x1300000 -- still far below
# the device tree at 0x4000000 and the heap at 0x8000000.
#
# Must be a multiple of 512; stand/common/md.c #errors otherwise.  Changing it
# needs a clean build: md.o takes it through CFLAGS, which make does not track.
MD_IMAGE_SIZE?=		16777216

# What this loader does and does not carry.  Network and ZFS are off because
# there are no drivers for them here yet; MSDOS is on because the firmware
# partition this board boots from is FAT16 and reading it is the next step.
LOADER_NET_SUPPORT=	no
LOADER_NFS_SUPPORT=	no
LOADER_TFTP_SUPPORT=	no
LOADER_MSDOS_SUPPORT=	yes
LOADER_UFS_SUPPORT=	yes
LOADER_GZIP_SUPPORT=	no
LOADER_BZIP2_SUPPORT=	no

HELP_FILENAME=	loader.help.rpiboot

.include <bsd.init.mk>

SRCS=	start.S
SRCS+=	main.c conf.c copy.c time.c devicename.c
SRCS+=	pl011.c pl011_console.c
SRCS+=	exec.c bootinfo_arch.c
SRCS+=	rpi_mbox.c rpi_peek.c
SRCS+=	vers.c

# bi_load() builds the metadata the kernel reads, which is a contract with
# sys/arm64 -- so share the file rather than reimplement it.  It is already
# #ifdef EFI-guarded and stand/kboot reuses it exactly this way.  Compiled
# without -DEFI here, its non-EFI branch wants "kboot.h" and bi_loadsmap();
# see kboot.h and bootinfo_arch.c for why both are provided rather than
# patching a file shared with kboot.
.PATH:		${BOOTSRC}/efi/loader
SRCS+=		bootinfo.c

# cpu_flush_dcache()/cpu_inval_icache() for the handoff, the same two lines
# stand/efi/loader/arch/arm64/Makefile.inc uses.
.PATH:		${BOOTSRC}/arm64/libarm64
CFLAGS+=	-I${BOOTSRC}/arm64/libarm64
SRCS+=		cache.c

# FDT comes from x0, so the platform contract is three small functions.
.if ${MK_FDT} != "no"
SRCS+=		rpi_fdt.c
CFLAGS+=	-I${FDTSRC} -I${BOOTOBJ}/fdt -DLOADER_FDT_SUPPORT
LIBFDT=		${BOOTOBJ}/fdt/libfdt.a
.endif

.include "${BOOTSRC}/loader.mk"

CFLAGS+=	-I${.CURDIR}
CFLAGS+=	-I${SYSDIR}/contrib/libfdt

# Freestanding, and every bit of it load-bearing at EL2 with the MMU off:
# unaligned accesses fault because all memory is Device-nGnRnE, and CPTR_EL2
# may trap FP/SIMD.  Both were established by the bare-metal test images.
CFLAGS+=	-mstrict-align -mgeneral-regs-only
CFLAGS+=	-fno-stack-protector -ffreestanding -fno-builtin

LDSCRIPT=	${.CURDIR}/ldscript.arm64
LDFLAGS=	-nostdlib -static -T ${LDSCRIPT}

# libsa is the loader's libc; LIBFDT is the device-tree library when FDT is
# enabled.  LDR_INTERP is empty for the "simp" interpreter, which is one of
# the reasons it was chosen -- no ficl or liblua to build and link.
DPADD=		${LDR_INTERP} ${LIBSAFDT} ${LIBFDT} ${LIBSA} ${LDSCRIPT}
LDADD=		${LDR_INTERP} ${LIBSAFDT} ${LIBFDT} ${LIBSA}

CLEANFILES+=	${LOADER}.bin ${LOADER}.sym loader.help.rpiboot

${LOADER}.bin: ${LOADER}.sym
	${OBJCOPY} -S -O binary ${LOADER}.sym ${.TARGET}
.if defined(MFS_IMAGE)
#
# Patch the memory-disk image into the binary.
#
# sys/tools/embed_mfs.sh finds the marker string that stand/common/md.c leaves
# in .data ("MFS Filesystem goes here") and writes the image over it.  Its
# non-ELF path uses strings(1), so it works on the raw binary -- which is why
# this runs after objcopy rather than on the .sym.
#
# The image must be a bare filesystem, not a partitioned disk: md exposes no
# partition table, so build it with makefs and nothing else.
	@echo "embedding ${MFS_IMAGE} (`stat -f %z ${MFS_IMAGE}` bytes) into ${.TARGET}"
	@if [ `stat -f %z ${MFS_IMAGE}` -gt ${MD_IMAGE_SIZE} ]; then \
		echo "ERROR: image is larger than MD_IMAGE_SIZE=${MD_IMAGE_SIZE}"; \
		false; \
	fi
	sh ${SRCTOP}/sys/tools/embed_mfs.sh ${.TARGET} ${MFS_IMAGE}
.endif

all: ${LOADER}.bin

FILES+=	${LOADER}.bin

.include <bsd.prog.mk>
