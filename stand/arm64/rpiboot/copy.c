/*-
 * copy.c -- module address space access for the Raspberry Pi 5 loader.
 *
 * The MMU is off and stays off for the whole life of this loader, so a module
 * address is a physical address is a pointer.  Every one of these is therefore
 * a straight copy, with no translation to get wrong.
 *
 * The EFI loader needs efi_translate() here because it runs with EFI's page
 * tables; that is the difference, and it is why the arm64 exec path had to be
 * forked rather than shared.
 */

#include <stand.h>
#include "bootstrap.h"

ssize_t
rpi_copyin(const void *src, vm_offset_t dest, const size_t len)
{
	bcopy(src, (void *)dest, len);
	return (len);
}

ssize_t
rpi_copyout(const vm_offset_t src, void *dest, const size_t len)
{
	bcopy((void *)src, dest, len);
	return (len);
}

ssize_t
rpi_readin(readin_handle_t fd, vm_offset_t dest, const size_t len)
{
	return (VECTX_READ(fd, (void *)dest, len));
}
