/*-
 * copy.c -- module address space access, and the staging translation.
 *
 * WHY A TRANSLATION IS NEEDED AT ALL
 *
 * A FreeBSD arm64 kernel's ELF declares virtual addresses in *both* p_vaddr
 * and p_paddr.  Measured on the RPI5-FDT kernel:
 *
 *	Entry point:  0xffff000000000800
 *	LOAD  vaddr 0xffff000000000000  paddr 0xffff000000000000
 *	LOAD  vaddr 0xffff000000000800  paddr 0xffff000000000800
 *	...
 *
 * and stand/common/load_elf.c takes dest = (e_entry & ~PAGE_MASK) for an
 * aarch64 ET_EXEC with off = 0 ("other archs use direct mapped kernels").  So
 * the address the MI loader hands down is 0xffff000000000000, which is not a
 * physical address and cannot be written to.
 *
 * The EFI loader solves this with a staging area and efi_translate(), and this
 * file does the same thing for the same reason.  A physical staging window is
 * reserved, and on the first write the offset between the module address space
 * and that window is fixed:
 *
 *	stage_offset = staging - <first dest written>
 *	translate(va) = va + stage_offset
 *
 * The unsigned wraparound is deliberate and correct: with a first dest of
 * 0xffff000000000000, stage_offset is the two's-complement difference, and
 * adding it back to any module address lands in the staging window.
 *
 * WHAT THE KERNEL IS HANDED
 *
 * Note carefully that exec.c jumps to the *translated* entry but passes
 * modulep *untranslated*, as a module address above KERNBASE.  That is not an
 * oversight copied from the EFI loader -- it is the kernel's contract:
 * sys/arm64/arm64/locore.S discriminates on whether x0 is below KERNBASE, and
 * treats a high value as a modulep and a low one as a bare DTB pointer.  The
 * kernel then works out its own virtual-to-physical delta from where it finds
 * itself running.  Passing a translated modulep would make the kernel read
 * metadata through a second translation and find garbage.
 */

#include <stand.h>
#include <sys/param.h>

#include "bootstrap.h"
#include "librpiboot.h"

/*
 * Staging window: where the kernel and its modules are actually assembled in
 * physical memory.
 *
 * 0x10000000 is 256 MiB, chosen to clear everything else this loader knows
 * about, all of which is measured rather than assumed:
 *
 *	loader image	0x00200000 .. ~0x00348000  (linked there; _end printed
 *					            at startup)
 *	device tree	0x04000000 .. 0x04013834  (device_tree_address, and
 *					            the blob is 0x13834)
 *	loader heap	0x08000000 .. 0x0b000000
 *	staging		0x10000000 .. 0x18000000
 *
 * and the whole of it sits inside the first /memory region, 0x0 .. 0x3f400000,
 * which the firmware reports and the probe confirmed.  128 MiB is ample: the
 * RPI5-FDT kernel's segments span about 15.4 MiB, and modules and metadata go
 * after it.
 *
 * 2 MiB aligned because arm64 kernels expect to be loaded on a 2 MiB boundary.
 */
#define	RPI_STAGING_BASE	0x10000000UL
#define	RPI_STAGING_SIZE	(128UL * 1024 * 1024)

static vm_offset_t	stage_offset;
static bool		stage_offset_set;

/*
 * Fix the translation on the first write, and bounds-check every one.
 *
 * The bounds check is not defensive decoration.  Without it, a kernel larger
 * than the staging window would silently scribble past it -- over the loader's
 * own heap at 0x8000000 if it grew downward, or over whatever is above -- and
 * the failure would appear as a corrupted kernel much later, with nothing
 * pointing back here.
 */
static int
stage_check(vm_offset_t dest, size_t len)
{
	if (!stage_offset_set) {
		stage_offset = RPI_STAGING_BASE - dest;
		stage_offset_set = true;
	}

	if (dest + stage_offset < RPI_STAGING_BASE ||
	    dest + stage_offset + len >
	    RPI_STAGING_BASE + RPI_STAGING_SIZE) {
		printf("staging overflow: module address 0x%lx + %zu bytes "
		    "lands outside 0x%lx..0x%lx\n",
		    (unsigned long)dest, len,
		    (unsigned long)RPI_STAGING_BASE,
		    (unsigned long)(RPI_STAGING_BASE + RPI_STAGING_SIZE));
		errno = ENOMEM;
		return (-1);
	}

	return (0);
}

/*
 * Module address to physical address.  Only meaningful once something has been
 * copied in, because that is when the offset is fixed; saying so out loud
 * beats returning a plausible-looking wrong pointer.
 */
void *
rpi_translate(vm_offset_t va)
{
	if (!stage_offset_set) {
		printf("rpi_translate(0x%lx) before anything was staged; "
		    "the result would be meaningless\n", (unsigned long)va);
		return (NULL);
	}

	return ((void *)(va + stage_offset));
}

ssize_t
rpi_copyin(const void *src, vm_offset_t dest, const size_t len)
{
	if (stage_check(dest, len) != 0)
		return (-1);

	bcopy(src, (void *)(dest + stage_offset), len);
	return (len);
}

ssize_t
rpi_copyout(const vm_offset_t src, void *dest, const size_t len)
{
	if (stage_check(src, len) != 0)
		return (-1);

	bcopy((void *)(src + stage_offset), dest, len);
	return (len);
}

ssize_t
rpi_readin(readin_handle_t fd, vm_offset_t dest, const size_t len)
{
	if (stage_check(dest, len) != 0)
		return (-1);

	return (VECTX_READ(fd, (void *)(dest + stage_offset), len));
}
