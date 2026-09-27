/*-
 * rpi_peek.c -- read device registers from the loader prompt.
 *
 *	peek <physical address> [count]
 *
 * prints count (default 1, at most 64) 32-bit words starting at the given
 * physical address.
 *
 * The loader runs after the VPU firmware and before any kernel, with the MMU
 * off, so a read here is a plain Device-nGnRnE access to the state the kernel
 * will inherit.  That makes this the cheapest way to answer "what did the
 * firmware leave in this register?", and nothing in the running kernels can
 * answer it as safely: FreeBSD's /dev/mem maps pages outside the direct map
 * with an uninitialised memory attribute.
 *
 * It is a bring-up tool with no guard rails.  A read of an address with
 * nothing behind it -- RP1's window with PCIe reset, for example -- can raise
 * an SError and kill the loader.  Only peek at addresses the device tree
 * describes.
 */

#include <stand.h>

#include "bootstrap.h"

#define	PEEK_MAX	64

static int
command_peek(int argc, char *argv[])
{
	unsigned long pa, count, i;
	char *end;

	if (argc < 2 || argc > 3) {
		command_errmsg = "usage: peek <physical address> [count]";
		return (CMD_ERROR);
	}
	pa = strtoul(argv[1], &end, 0);
	if (*end != '\0' || (pa & 3) != 0) {
		command_errmsg = "address must be a 4-byte-aligned number";
		return (CMD_ERROR);
	}
	count = 1;
	if (argc == 3) {
		count = strtoul(argv[2], &end, 0);
		if (*end != '\0' || count == 0 || count > PEEK_MAX) {
			command_errmsg = "count must be 1..64";
			return (CMD_ERROR);
		}
	}

	for (i = 0; i < count; i++) {
		if (i % 4 == 0)
			printf("%s0x%010lx:", i == 0 ? "" : "\n", pa + i * 4);
		printf(" %08x", *(volatile uint32_t *)(uintptr_t)(pa + i * 4));
	}
	printf("\n");
	return (CMD_OK);
}
COMMAND_SET(peek, "peek", "read 32-bit device registers by physical address",
    command_peek);
