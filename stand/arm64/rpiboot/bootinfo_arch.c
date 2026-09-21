/*-
 * bootinfo_arch.c -- the platform half of bi_load(), for this board.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"

/*
 * Supply the kernel's memory map.  There is nothing to supply.
 *
 * bi_load()'s non-EFI path calls this, and on aarch64 stand/kboot implements
 * it by forwarding to efi_bi_loadsmap() -- because kboot runs under Linux,
 * which hands it an EFI memory map to pass along.
 *
 * Booted straight from the VPU firmware there is no EFI memory map anywhere in
 * the system, and none is needed: memory reaches the kernel through
 * MODINFOMD_DTBP and the device tree's /memory node.  That is not a
 * convenient assumption, it is the measured fact the entire non-EFI approach
 * rests on -- initarm() panics "Cannot get physical memory regions" with
 * neither source, the EDK2-supplied tree has no /memory at all, and the
 * firmware's own tree carries this board's real 16 GB in 8 regions:
 *
 *	base 0x0000000000  size 0x003f400000   (1012 MiB)
 *	base 0x0040000000  size 0x00c0000000   (3072 MiB)
 *	six further 2 GiB regions to 0x0380000000
 *	total 0x3ff400000 = 15.99 GiB
 *
 * See rpi5_modules.git/doc/LOADER_ZIMAGE.md for how that was established.
 *
 * So this being empty is a positive result rather than a stub.  If a kernel
 * ever panics for want of memory regions, the fault is in the device tree
 * reaching it or in MODINFOMD_DTBP, not here.
 */
void
bi_loadsmap(struct preloaded_file *kfp __unused)
{
}
