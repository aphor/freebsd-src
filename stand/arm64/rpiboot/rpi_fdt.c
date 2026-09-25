/*-
 * rpi_fdt.c -- the stand FDT platform contract, for a firmware-supplied tree.
 *
 * Three functions, the same shape as stand/efi/fdt/efi_fdt.c.  The difference
 * is where the blob comes from: EFI hands it over as a configuration table,
 * whereas the VPU firmware passes its physical address in x0, which start.S
 * saves for us.
 *
 * What arrives is better than it might be, and this was measured rather than
 * hoped for (probe-k.bin, 2026-09-20):
 *
 *   - the blob is the vendor dtb with the bcm2712d0 overlay ALREADY MERGED by
 *     the firmware -- 78,377 bytes in, 79,924 out -- so this loader does not
 *     have to implement dtoverlay=/dtparam= merging to get a correct tree;
 *   - /memory is patched with the board's real 16 GB layout in 8 regions,
 *     where the static vendor file carries a 640 MiB placeholder.  That is
 *     the fact the whole non-EFI approach depends on: initarm() panics
 *     "Cannot get physical memory regions" with neither an EFI memory map nor
 *     a usable /memory, and the EDK2-supplied tree has no /memory at all.
 */

#include <stand.h>
#include <sys/param.h>

#include <fdt_platform.h>
#include <libfdt.h>

extern uint64_t rpi_dtb_pa;	/* from start.S: x0 at entry */

int
fdt_platform_load_dtb(void)
{
	struct fdt_header *hdr;

	if (rpi_dtb_pa == 0) {
		printf("No device tree: x0 was zero at entry.\n");
		return (1);
	}

	/* MMU off, so the physical address is directly usable. */
	hdr = (struct fdt_header *)(uintptr_t)rpi_dtb_pa;

	if (fdt_load_dtb_addr(hdr) != 0) {
		printf("Device tree at %p is not usable.\n", hdr);
		return (1);
	}

	printf("Using DTB provided by the VPU firmware at %p.\n", hdr);
	return (0);
}

void
fdt_platform_load_overlays(void)
{
	/*
	 * Loader-applied overlays still work through the fdt_overlays
	 * variable, but note they are not needed for the board's own
	 * hardware: the firmware merges dtoverlay= itself before handing the
	 * blob over.
	 */
	fdt_load_dtb_overlays(NULL);
}

void
fdt_platform_fixups(void)
{
	fdt_apply_overlays();
}

/*
 * Which config file did the firmware boot us with?  The bootloader records it
 * in /chosen/bootloader/tryboot, a single cell: 0 for config.txt, 1 for
 * tryboot.txt.  Read straight off the blob at x0 so it works before, and
 * without, the MI FDT code loading a copy.  Returns -1 when absent.
 */
int
rpi_fdt_tryboot(void)
{
	const void *fdt = (const void *)(uintptr_t)rpi_dtb_pa;
	const fdt32_t *p;
	int len, node;

	if (rpi_dtb_pa == 0 || fdt_check_header(fdt) != 0)
		return (-1);
	node = fdt_path_offset(fdt, "/chosen/bootloader");
	if (node < 0)
		return (-1);
	p = fdt_getprop(fdt, node, "tryboot", &len);
	if (p == NULL || len != sizeof(*p))
		return (-1);
	return ((int)fdt32_to_cpu(*p));
}
