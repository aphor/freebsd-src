/*-
 * main.c -- startup for the Raspberry Pi 5 FreeBSD loader.
 *
 * Entered from start.S with the stack set up and BSS zeroed, at EL2 with the
 * MMU off, and with the firmware's device tree physical address in the global
 * rpi_dtb_pa.
 *
 * What makes this loader different from the EFI one is what it does NOT have:
 * no EFI boot services, no EFI memory map, no runtime services, no wall clock,
 * and at this stage no storage driver.  The memory it may use therefore has to
 * come from the device tree's /memory node, which was proven to be present and
 * correct on this board -- 15.99 GiB in 8 regions, patched into the tree by the
 * firmware at runtime.  That measurement is the reason this loader can exist
 * at all; see rpi5_modules.git/doc/LOADER_ZIMAGE.md.
 */

#include <stand.h>
#include <sys/param.h>
#include <sys/reboot.h>

#include "bootstrap.h"
#include "librpiboot.h"

struct arch_switch archsw;

extern uint64_t	rpi_dtb_pa;	/* from start.S: x0 at entry */
extern char	_end[];

/*
 * Heap.
 *
 * Placed at a fixed physical address rather than immediately after _end, and
 * the reason is the device tree.  The firmware puts the blob at
 * device_tree_address, which tools/boot_config_install.sh sets to 0x4000000,
 * and the loader itself lives at 0x200000 and is around a megabyte with the
 * memory disk embedded.  Growing a heap up from _end would march straight into
 * the blob.  Starting at 128 MiB clears both, and stays inside the first
 * /memory region (0x0 .. 0x3f400000) with room to spare.
 *
 * This is deliberately not derived from /memory yet: the tree has to be parsed
 * before it can be consulted, and parsing needs a working allocator.  Fixing
 * that properly means an early bootstrap allocator, which is worth doing when
 * something actually needs it.
 */
#define	RPI_HEAP_START	0x08000000UL
#define	RPI_HEAP_SIZE	(48UL * 1024 * 1024)

/* Sanity values checked at startup and reported; see rpi_report_entry(). */
#define	RPI_LOAD_ADDR	0x00200000UL

static void
rpi_report_entry(void)
{
	uint64_t el;

	__asm__ __volatile__("mrs %0, CurrentEL" : "=r"(el));

	printf("Entered from the Raspberry Pi VPU firmware, no EFI.\n");
	printf("   Exception level: EL%lu\n", (unsigned long)(el >> 2) & 3);
	printf("   Load address:    0x%lx\n", RPI_LOAD_ADDR);
	printf("   Device tree:     0x%lx (from x0)\n",
	    (unsigned long)rpi_dtb_pa);
	printf("   Loader end:      %p\n", _end);
	printf("   Heap:            0x%lx + %lu MiB\n",
	    (unsigned long)RPI_HEAP_START,
	    (unsigned long)(RPI_HEAP_SIZE / (1024 * 1024)));

	/*
	 * A loader whose heap overlaps the device tree would corrupt the tree
	 * the moment it allocated anything, and the symptom would appear much
	 * later and look like a tree problem.  Say so here instead.
	 */
	if (rpi_dtb_pa != 0 &&
	    rpi_dtb_pa >= RPI_HEAP_START &&
	    rpi_dtb_pa < RPI_HEAP_START + RPI_HEAP_SIZE)
		printf("   WARNING: the device tree is inside the heap; "
		    "move device_tree_address.\n");
	if ((uint64_t)(uintptr_t)_end > RPI_HEAP_START)
		printf("   WARNING: the loader image overlaps the heap.\n");
}

int
main(void)
{
	int i;

	/*
	 * Heap first: nothing else here may allocate until this is done, and
	 * that includes the console probe.
	 */
	setheap((void *)RPI_HEAP_START,
	    (void *)(RPI_HEAP_START + RPI_HEAP_SIZE));

	/*
	 * Console next, so that everything after this point can report what
	 * it is doing.  cons_probe() walks the consoles[] array in conf.c.
	 */
	cons_probe();

	printf("\n%s", bootprog_info);
	printf("\n");
	rpi_report_entry();

	archsw.arch_getdev = rpi_getdev;
	archsw.arch_copyin = rpi_copyin;
	archsw.arch_copyout = rpi_copyout;
	archsw.arch_readin = rpi_readin;
	archsw.arch_autoload = rpi_autoload;

	/*
	 * Probe the device switch.  Today that is the embedded memory disk
	 * and nothing else, which is the point: it needs no hardware driver,
	 * so the loader has a filesystem before it has a disk.
	 */
	for (i = 0; devsw[i] != NULL; i++) {
		if (devsw[i]->dv_init == NULL)
			continue;
		if ((devsw[i]->dv_init)() != 0)
			continue;
		printf("Found device: %s\n", devsw[i]->dv_name);
	}

	/*
	 * Point currdev at the memory disk.
	 *
	 * set_currdev() is the MI helper in stand/common/misc.c: it sets both
	 * currdev and loaddev and installs gen_setcurrdev() as the hook, so a
	 * later assignment from the prompt re-parses and remounts properly.
	 * An earlier version of this file installed a local hook that
	 * duplicated gen_setcurrdev() badly enough to break every path.
	 */
	set_currdev("md0:");
	setenv("LINES", "24", 1);

	interact();			/* doesn't return */

	return (0);
}

/*
 * No hardware autodetection to drive module loading yet.  Returning 0 keeps
 * the MI code happy without pretending to have looked.
 */
int
rpi_autoload(void)
{
	return (0);
}

void
exit(int code)
{
	printf("\nLoader exit(%d), and there is nothing to exit to.\n", code);
	printf("Halting; power-cycle the board.\n");
	for (;;)
		__asm__ __volatile__("wfi");
}

/*
 * The firmware offers no reboot service we can call from here, and the
 * VideoCore mailbox is not reachable this early (see the TryBoot
 * investigation in doc/LOADER_ZIMAGE.md).  A PSCI SYSTEM_RESET via BL31 is
 * the right answer and BL31 is live -- "NOTICE: BL31: v2.6" appears just
 * before we are entered -- so this is a small, well-defined piece of work
 * rather than an unknown.  Until then, say so honestly rather than hanging
 * with no explanation.
 */
void
reboot(void)
{
	printf("reboot is not implemented yet: PSCI SYSTEM_RESET via BL31 is "
	    "the intended route.\n");
	printf("Power-cycle the board.\n");
	for (;;)
		__asm__ __volatile__("wfi");
}
