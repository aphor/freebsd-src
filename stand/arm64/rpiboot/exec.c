/*-
 * exec.c -- hand control to an arm64 kernel.
 *
 * Adapted from stand/efi/loader/arch/arm64/exec.c, which is 105 lines and
 * whose only dependencies on EFI are:
 *
 *	efi_time_fini()/efi_time_init()  stopping and restarting the EFI timer
 *					 around the handoff -- nothing to do
 *					 here, the generic timer needs no
 *					 arrangement
 *	efi_translate()			 module address to physical address,
 *					 which this loader provides itself as
 *					 rpi_translate(); see copy.c for why a
 *					 translation is needed at all
 *
 * Everything else -- elf64_loadfile, bi_load, the cache maintenance -- is
 * shared with the EFI path, deliberately.  The metadata layout handed to the
 * kernel is fiddly and is a contract with sys/arm64, so reusing bi_load() is
 * safer than reimplementing it however much smaller the result might be.
 */

#include <sys/param.h>
#include <sys/linker.h>
#include <machine/elf.h>

#include <stand.h>
#include <bootstrap.h>

#include "cache.h"
#include "librpiboot.h"

static int	elf64_exec(struct preloaded_file *amp);
static int	elf64_obj_exec(struct preloaded_file *amp);

static struct file_format arm64_elf = {
	.l_load = elf64_loadfile,
	.l_exec = elf64_exec
};

struct file_format *file_formats[] = {
	&arm64_elf,
	NULL
};

static int
elf64_exec(struct preloaded_file *fp)
{
	vm_offset_t modulep, kernendp;
	vm_offset_t clean_addr;
	size_t clean_size;
	struct file_metadata *md;
	Elf_Ehdr *ehdr;
	void (*entry)(vm_offset_t);
	int err;

	if ((md = file_findmetadata(fp, MODINFOMD_ELFHDR)) == NULL)
		return (EFTYPE);

	ehdr = (Elf_Ehdr *)&(md->md_data);

	/*
	 * Close devices before the point of no return.  The EFI loader does
	 * this because net_cleanup() stops working after ExitBootServices;
	 * here it is simply good manners -- the memory disk and any future SD
	 * device should not be left half-open when the kernel takes over the
	 * hardware.
	 */
	dev_cleanup();

	err = bi_load(fp->f_args, &modulep, &kernendp, true);
	if (err != 0)
		return (err);

	entry = rpi_translate(ehdr->e_entry);
	if (entry == NULL)
		return (EINVAL);

	/*
	 * Clean the D-cache over the kernel image and invalidate the whole
	 * I-cache.
	 *
	 * This matters more here than it looks.  The kernel was written as
	 * data and is about to be executed, and we are about to jump to it
	 * with the MMU off, where accesses do not go through the caches the
	 * writes may still be sitting in.  Skipping this is the classic way
	 * to get a kernel that crashes in its first few instructions on some
	 * boots and not others.
	 */
	clean_addr = (vm_offset_t)rpi_translate(fp->f_addr);
	clean_size = (vm_offset_t)rpi_translate(kernendp) - clean_addr;

	printf("Jumping to kernel entry 0x%lx (module 0x%lx), modulep 0x%lx\n",
	    (unsigned long)entry, (unsigned long)ehdr->e_entry,
	    (unsigned long)modulep);
	printf("Flushing D-cache 0x%lx + 0x%lx and invalidating I-cache.\n",
	    (unsigned long)clean_addr, (unsigned long)clean_size);

	cpu_flush_dcache((void *)clean_addr, clean_size);
	cpu_inval_icache();

	/*
	 * modulep is passed UNTRANSLATED, as a module address above KERNBASE.
	 * sys/arm64/arm64/locore.S discriminates on exactly that: a high x0
	 * is a modulep, a low one a bare DTB pointer.  See copy.c.
	 */
	(*entry)(modulep);

	panic("exec returned");
}

static int
elf64_obj_exec(struct preloaded_file *fp)
{
	printf("%s called for preloaded file %p (=%s):\n", __func__, fp,
	    fp->f_name);
	return (ENOSYS);
}
