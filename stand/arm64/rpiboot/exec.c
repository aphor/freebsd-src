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
 * Everything else -- elf64_loadfile, bi_load -- is shared with the EFI path,
 * deliberately.  The metadata layout handed to the kernel is fiddly and is a
 * contract with sys/arm64, so reusing bi_load() is safer than reimplementing
 * it however much smaller the result might be.  The cache maintenance is the
 * exception; see elf64_exec().
 */

#include <sys/param.h>
#include <sys/linker.h>
#include <machine/elf.h>

#include <stand.h>
#include <bootstrap.h>

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
	vm_offset_t clean_addr, clean_end;
	struct file_metadata *md;
	Elf_Ehdr *ehdr;
	void *entry;
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
	 * Enter the kernel with the MMU and the D-cache off, and nothing in the
	 * caches for any memory this loader used.
	 *
	 * The kernel was written as data through the D-cache (rpi_mmu.c) and
	 * is about to be executed with the caches off, so it has to reach
	 * memory first.  Skipping that is the classic way to get a kernel that
	 * crashes in its first few instructions on some boots and not others.
	 * rpi_mmu_handoff() turns the D-cache off before cleaning, which is
	 * stricter than the EFI loader's clean-then-jump with the cache still
	 * on: see rpi_mmu_asm.S for why that order matters.
	 *
	 * The range is everything this loader wrote or read through the
	 * cache: its own image (with its stack and page tables), the
	 * firmware's device tree, the heap, and the staging area up to the end
	 * of the kernel, its modules and their metadata, all of which lie in
	 * that order between the load address and kernendp.
	 */
	clean_addr = (vm_offset_t)rpi_translate(fp->f_addr);
	clean_end = (vm_offset_t)rpi_translate(kernendp);
	if (clean_end < RPI_HEAP_START + RPI_HEAP_SIZE)
		clean_end = RPI_HEAP_START + RPI_HEAP_SIZE;

	printf("Jumping to kernel entry 0x%lx (module 0x%lx), modulep 0x%lx\n",
	    (unsigned long)entry, (unsigned long)ehdr->e_entry,
	    (unsigned long)modulep);
	printf("Kernel at 0x%lx + 0x%lx; MMU and D-cache off, then "
	    "0x%lx..0x%lx cleaned and invalidated.\n",
	    (unsigned long)clean_addr, (unsigned long)(clean_end - clean_addr),
	    (unsigned long)RPI_LOAD_ADDR, (unsigned long)clean_end);

	/*
	 * modulep is passed UNTRANSLATED, as a module address above KERNBASE.
	 * sys/arm64/arm64/locore.S discriminates on exactly that: a high x0
	 * is a modulep, a low one a bare DTB pointer.  See copy.c.
	 */
	rpi_mmu_handoff(entry, modulep, RPI_LOAD_ADDR, clean_end);
}

static int
elf64_obj_exec(struct preloaded_file *fp)
{
	printf("%s called for preloaded file %p (=%s):\n", __func__, fp,
	    fp->f_name);
	return (ENOSYS);
}
