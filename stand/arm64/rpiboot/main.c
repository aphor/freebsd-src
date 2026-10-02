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
#include <sys/boot.h>

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
/* RPI_HEAP_START and RPI_HEAP_SIZE are in librpiboot.h, for exec.c. */

/*
 * PSCI, which is how this loader resets the board.
 *
 * There is no firmware reset service to call, but ARM Trusted Firmware is
 * live at EL3 -- its banner prints immediately before we are entered:
 *
 *	NOTICE:  BL31: v2.6(release):v2.6-240-gfc45bc492
 *
 * and the firmware's device tree advertises it:
 *
 *	/psci { compatible = "arm,psci-1.0", "arm,psci-0.2"; method = "smc"; }
 *	/cpus/cpu@0..3 { enable-method = "psci"; }
 *
 * so an SMC from EL2 reaches BL31.  Function IDs are from
 * sys/dev/psci/psci.h; SYSTEM_RESET and SYSTEM_OFF are both SMC32 calls, so
 * the 0x84 prefix rather than 0xc4.
 *
 * Note this is the ATF the VPU firmware carries, which is NOT the one the
 * EDK2 lane uses -- RPI_EFI.fd ships its own v2.10.0.  A PSCI difference
 * between the two lanes would show up here first.
 */
#define	PSCI_FNID_VERSION	0x84000000U
#define	PSCI_FNID_SYSTEM_OFF	0x84000008U
#define	PSCI_FNID_SYSTEM_RESET	0x84000009U

static uint64_t
psci_smc(uint32_t fnid, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = fnid;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;

	__asm__ __volatile__("smc #0"
	    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
	    :
	    : "memory");

	return (x0);
}

/*
 * Reset via PSCI.  Does not return when it works.
 *
 * SYSTEM_RESET is specified never to return, so reaching the code after it
 * means the call failed -- and the caller wants to know that rather than sit
 * in a silent hang.
 */
void
rpi_psci_reset(void)
{
	printf("Resetting via PSCI SYSTEM_RESET (SMC to BL31)...\n");

	/*
	 * Let the console drain before the world stops.  Without this the
	 * message above can be lost in the UART FIFO, which makes a failed
	 * reset look like a hang with no explanation.
	 */
	delay(100000);

	(void)psci_smc(PSCI_FNID_SYSTEM_RESET, 0, 0, 0);

	printf("PSCI SYSTEM_RESET returned, so it did not work.\n");
	printf("Power-cycle the board.\n");
}

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
	rpi_mmu_report();

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

	/*
	 * Ask PSCI its version.  Cheap, harmless, and it answers up front
	 * whether "reboot" will work rather than finding out when it is
	 * needed.  A sane reply is major:minor in bits 31:16 / 15:0; PSCI
	 * NOT_SUPPORTED is returned as -1.
	 */
	{
		uint64_t v = psci_smc(PSCI_FNID_VERSION, 0, 0, 0);

		if ((int64_t)v < 0)
			printf("   PSCI:            not supported; "
			    "reboot will not work\n");
		else
			printf("   PSCI:            v%lu.%lu via smc "
			    "(reboot available)\n",
			    (unsigned long)((v >> 16) & 0xffff),
			    (unsigned long)(v & 0xffff));
	}

	/*
	 * config.txt or tryboot.txt?  The firmware records which one in the
	 * tree, and after a "tryboot" it is the only proof the one-shot was
	 * honoured rather than merely requested.
	 */
	rpi_print_boot_config();
}

/*
 * Read loader variables from a file: name=value words separated by blanks
 * or newlines, as the EFI loader reads /efi/freebsd/loader.env from its ESP
 * (boot_parse_cmdline(); no comments, no quoting, no blanks in a value).
 */
static int
rpi_env_file(const char *path)
{
	struct stat st;
	char *buf;
	int fd, howto;

	if (stat(path, &st) != 0 || st.st_size <= 0 || st.st_size > 4096)
		return (-1);
	if ((fd = open(path, O_RDONLY)) < 0)
		return (-1);
	buf = malloc(st.st_size + 1);
	if (buf == NULL || read(fd, buf, st.st_size) != st.st_size) {
		free(buf);
		close(fd);
		return (-1);
	}
	close(fd);
	buf[st.st_size] = '\0';
	printf("Reading loader variables from %s\n", path);
	howto = boot_parse_cmdline(buf);
	if (howto != 0)
		boot_howto_to_env(howto);
	free(buf);
	return (0);
}

/*
 * What to boot from.
 *
 * The memory disk inside this image, unless the card says otherwise: the
 * firmware's FAT (the first partition of disk0) may hold loader.env, and its
 * rootdev names the filesystem that has /boot, for example
 *
 *	rootdev=disk0s2a:
 *
 * The loader then reads its Lua scripts, loader.conf, the kernel and the
 * modules from there, as it does on any other machine, and installkernel
 * on the running system reaches the next boot.  The EFI loader takes
 * rootdev the same way, from its own loader.env.
 *
 * A boot that the firmware started from tryboot.txt reads tryboot.env
 * first, so that one image can be tried against another root, once.
 *
 * A rootdev without /boot/lua/loader.lua is not used: the interpreter
 * would have nothing to run, where the memory disk still boots.
 */
static void
rpi_choose_currdev(void)
{
	static const char *fat[] = { "disk0s1:", "disk0p1:" };
	char dev[40], path[80];
	const char *rootdev;
	struct stat st;
	size_t i;

	for (i = 0; rpi_sd_present() && i < nitems(fat); i++) {
		if (rpi_fdt_tryboot() == 1) {
			snprintf(path, sizeof(path), "%s/tryboot.env", fat[i]);
			if (rpi_env_file(path) == 0)
				break;
		}
		snprintf(path, sizeof(path), "%s/loader.env", fat[i]);
		if (rpi_env_file(path) == 0)
			break;
	}

	rootdev = getenv("rootdev");
	if (rootdev != NULL && *rootdev != '\0') {
		snprintf(dev, sizeof(dev), "%s%s", rootdev,
		    strchr(rootdev, ':') == NULL ? ":" : "");
		snprintf(path, sizeof(path), "%s/boot/lua/loader.lua", dev);
		if (stat(path, &st) == 0) {
			printf("Booting from %s (rootdev)\n", dev);
			set_currdev(dev);
			return;
		}
		printf("rootdev %s has no /boot/lua/loader.lua; "
		    "using the memory disk\n", dev);
	}
	set_currdev("md0:");
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
	 *
	 * The PL011 alone at first: the HDMI console joins once the MMU is
	 * on, below.
	 */
	setenv("console", "uart", 1);
	cons_probe();

	printf("\n%s", bootprog_info);
	printf("\n");

	/*
	 * MMU and caches on, as soon as there is a console to report a
	 * failure on (rpi_mmu.c).  Everything up to here ran uncached.
	 */
	(void)rpi_mmu_init(RPI_HEAP_START, RPI_HEAP_START + RPI_HEAP_SIZE);

	/*
	 * Then the HDMI console, second: the PL011 stays first and is the
	 * only input, for the UART tooling.  Not with the MMU off -- the
	 * console's set-up alone took over 4 s uncached (fbtest), and
	 * scrolling a second per line.  Without a display, cons_change()
	 * reports "console vidconsole failed to initialize" and the PL011
	 * carries on alone.
	 */
	if (rpi_mmu_enabled())
		setenv("console", "uart,vidconsole", 1);
	else
		printf("HDMI console off: the MMU is off, and it is too slow "
		    "uncached.\n");
	rpi_report_entry();

	/*
	 * A USB keyboard for the HDMI console (rpi_usbkbd.c): RP1's xHCI
	 * controllers, reachable when the firmware leaves PCIe2 trained
	 * (config.txt pciex4_reset=0).  "usbinfo" says what it found.
	 */
	rpi_usbkbd_init();

	/*
	 * Tell the Lua scripts that ACPI was probed early, so that they
	 * believe acpi.rsdp, which is never set here: this board is described
	 * by the firmware's device tree alone.  Without the feature,
	 * core.lua assumes ACPI, asks for an "acpi" module that does not
	 * exist, and boots with hint.acpi.0.disabled=0.
	 */
	feature_enable(FEATURE_EARLY_ACPI);

	archsw.arch_getdev = rpi_getdev;
	archsw.arch_copyin = rpi_copyin;
	archsw.arch_copyout = rpi_copyout;
	archsw.arch_readin = rpi_readin;
	archsw.arch_autoload = rpi_autoload;

	/*
	 * Probe the device switch: the embedded memory disk, which needs no
	 * hardware driver, and the SD card (rpi_sd.c), whose reads go through
	 * the block cache.  8 MiB of cache, out of a 48 MiB heap.
	 */
	bcache_init(16384, 512);
	for (i = 0; devsw[i] != NULL; i++) {
		if (devsw[i]->dv_init == NULL)
			continue;
		if ((devsw[i]->dv_init)() != 0)
			continue;
		printf("Found device: %s\n", devsw[i]->dv_name);
	}

	/*
	 * The card, if it says so, else the memory disk.
	 *
	 * set_currdev() is the MI helper in stand/common/misc.c: it sets both
	 * currdev and loaddev and installs gen_setcurrdev() as the hook, so a
	 * later assignment from the prompt re-parses and remounts properly.
	 * An earlier version of this file installed a local hook that
	 * duplicated gen_setcurrdev() badly enough to break every path.
	 */
	rpi_choose_currdev();
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

/*
 * reboot and poweroff are registered per-platform, not by MI code: there is a
 * COMMAND_SET for them in stand/efi/loader/main.c, stand/uboot/main.c and so
 * on, and none in stand/common.  Their absence here is why the first build
 * reached its prompt with no way to reset the board, which on this hardware
 * meant a physical power cycle for every iteration.
 */
static int
command_reboot(int argc __unused, char *argv[] __unused)
{
	rpi_psci_reset();

	/* Only reached if the reset failed; rpi_psci_reset() has said so. */
	for (;;)
		__asm__ __volatile__("wfi");

	return (CMD_OK);
}
COMMAND_SET(reboot, "reboot", "reboot the system", command_reboot);

static int
command_poweroff(int argc __unused, char *argv[] __unused)
{
	printf("Powering off via PSCI SYSTEM_OFF...\n");
	delay(100000);
	(void)psci_smc(PSCI_FNID_SYSTEM_OFF, 0, 0, 0);

	printf("PSCI SYSTEM_OFF returned, so it did not work.\n");
	for (;;)
		__asm__ __volatile__("wfi");

	return (CMD_OK);
}
COMMAND_SET(poweroff, "poweroff", "power off the system", command_poweroff);

/*
 * exit() is what "quit" reaches.  Resetting is the useful thing to do: there
 * is no firmware menu to fall back to, so halting would just strand the
 * board.  This cannot loop, because nothing calls exit() except an explicit
 * quit from the prompt.
 */
void
exit(int code)
{
	printf("\nLoader exit(%d); there is nothing to exit to, so "
	    "resetting.\n", code);
	rpi_psci_reset();

	for (;;)
		__asm__ __volatile__("wfi");
}

void
reboot(void)
{
	rpi_psci_reset();

	for (;;)
		__asm__ __volatile__("wfi");
}
