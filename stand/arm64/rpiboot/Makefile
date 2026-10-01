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

# The interpreter: lua, for the boot menu (on the PL011 and, with the MMU on,
# the HDMI console).  It reads its scripts from /boot/lua on the memory disk,
# which tools/rpiboot_image_build.sh copies from this tree, and loader.conf
# rather than loader.rc.  The property simp was chosen for during bring-up
# survives: if loader.lua is missing or broken, interp_lua.c prints the error,
# cancels autoboot and leaves the prompt working.  LOADER_INTERP=simp still
# builds the script-free loader, which reads loader.rc.
LOADER_INTERP?=	lua

# Memory disk.  This is what gives the loader a filesystem without any
# storage driver at all: stand/common/md.c reserves MD_IMAGE_SIZE bytes of
# .data pre-filled with the marker string "MFS Filesystem goes here", and
# sys/tools/embed_mfs.sh patches a real image over it after the link.  That
# script has a non-ELF path using strings(1), so it works on the raw binary.
LOADER_MD_SUPPORT=	yes
#
# 20 MiB, so the image can carry a stripped kernel with ZFS compiled in
# (15.9 MB since M4) and the cyw43455 firmware (0.6 MB).
# rpiboot_image_build.sh passes the size it builds the image for (-m).
#
# This is what lets the handoff be tested with one new variable instead of
# two: the loader has no storage driver yet, so without an embedded kernel
# there would be nothing to boot, and wiring the SD card at the same time
# would leave a failure ambiguous between the two.  The firmware has to be
# here too, and preloaded by loader.rc, because cyw(4) attaches before root
# is mounted.
#
# 14 MiB held the 12.45 MB kernel alone; with the 0.61 MB of firmware added,
# makefs ran out of space, and 16 MiB followed until ZFS.  20 MiB makes the
# loader about 21 MiB, which the firmware loads at 0x200000 and which ends
# near 0x1650000 -- still far below the device tree at 0x4000000 and the
# heap at 0x8000000.  The firmware FAT (51 MB) is the tighter limit.
#
# Must be a multiple of 512; stand/common/md.c #errors otherwise.  Changing it
# needs a clean build: md.o takes it through CFLAGS, which make does not track.
MD_IMAGE_SIZE?=		20971520

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
SRCS+=	rpi_fb.c
SRCS+=	rpi_mmu.c rpi_mmu_asm.S
SRCS+=	rpi_pcie.c rpi_usbkbd.c

# The HDMI console: stand/common/gfx_fb.c on teken with the built-in 8x16
# font, as the EFI loader builds it, but without EFI (gfx_fb.c's non-EFI
# branch; its <vbe.h> is ./vbe.h).
SRCS+=	gfx_fb.c teken.c 8x16.c
.PATH:	${SYSDIR}/teken
GFX_CFLAGS=	-I${SYSDIR}/teken -I${SRCTOP}/contrib/pnglite
CFLAGS.gfx_fb.c+= ${GFX_CFLAGS} ${ZLIB_CFLAGS}
CFLAGS.gfx_fb.c+= -I${SRCTOP}/sys/cddl/contrib/opensolaris/common/lz4
CFLAGS.teken.c+= -I${SYSDIR}/teken
CFLAGS.rpi_fb.c+= ${GFX_CFLAGS}
CFLAGS.main.c+=	${GFX_CFLAGS}
CFLAGS.conf.c+=	${GFX_CFLAGS}
CLEANFILES+=	8x16.c
# The built-in font is lz4-compressed, and libsa's lz4.c is built without
# -mstrict-align: clang then defines __ARM_FEATURE_UNALIGNED, which makes
# lz4.c copy with unaligned 64-bit accesses, and those fault here (MMU off,
# all memory Device-nGnRnE; see below).  Measured: the console set-up hung
# in setup_font(), the step that decompresses the font.  This copy is built
# with this loader's flags, and as a program object it is linked in place
# of libsa's.
.PATH:	${SRCTOP}/sys/cddl/contrib/opensolaris/common/lz4
SRCS+=	lz4.c
CFLAGS.lz4.c+= -I${SRCTOP}/sys/cddl/contrib/opensolaris/common/lz4
8x16.c:		${SRCTOP}/contrib/terminus/ter-u16b.bdf
	vtfontcvt -f compressed-source -o ${.TARGET} ${.ALLSRC}
SRCS+=	vers.c

# bi_load() builds the metadata the kernel reads, which is a contract with
# sys/arm64 -- so share the file rather than reimplement it.  It is already
# #ifdef EFI-guarded and stand/kboot reuses it exactly this way.  Compiled
# without -DEFI here, its non-EFI branch wants "kboot.h" and bi_loadsmap();
# see kboot.h and bootinfo_arch.c for why both are provided rather than
# patching a file shared with kboot.
.PATH:		${BOOTSRC}/efi/loader
SRCS+=		bootinfo.c

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
# rpi_mmu.c makes RAM Normal memory early in main(), but start.S and the
# console probe run before that, so -mstrict-align stays.
CFLAGS+=	-mstrict-align -mgeneral-regs-only
CFLAGS+=	-fno-stack-protector -ffreestanding -fno-builtin

LDSCRIPT=	${.CURDIR}/ldscript.arm64
LDFLAGS=	-nostdlib -static -T ${LDSCRIPT}

# libsa is the loader's libc; LIBFDT is the device-tree library when FDT is
# enabled.  LDR_INTERP is liblua (empty for simp).
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
