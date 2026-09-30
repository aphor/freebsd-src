/*-
 * start.S -- entry point for the Raspberry Pi 5 FreeBSD loader.
 *
 * The VPU firmware enters here per the Linux arm64 boot protocol, a contract
 * measured on this board rather than taken on faith (probe-k.bin, 2026-09-20):
 *
 *	EL2, MMU off, caches off
 *	x0 = DTB physical address
 *	x1 = x2 = x3 = 0
 *	PC = 0x200000, the address config.txt "kernel=" loads to
 *
 * Three jobs before C can run: keep x0 somewhere main() can find it, give the
 * C environment a stack, and zero BSS.
 *
 * BSS zeroing is not optional.  image_size in the header below covers BSS, so
 * the firmware has reserved the range, but its contents are whatever was in
 * DRAM.  An uninitialised static would then make the loader behave differently
 * from boot to boot, which is the worst class of bug to chase on hardware.
 */

	.section .text.boot,"ax",@progbits
	.globl	_start
_start:
	/*
	 * 64-byte arm64 Image header.  Field layout and values match
	 * sys/tools/arm_kernel_boothdr.awk with hdrtype=v8booti, which is what
	 * the in-tree kernel uses, so the firmware validates this the same way
	 * it validates a kernel.
	 */
	b	real_start		/* code0: executable branch	*/
	.long	0			/* code1			*/
	.quad	0			/* text_offset			*/
	.quad	_end - _start		/* image_size, covers BSS	*/
	.quad	8			/* flags			*/
	.quad	0, 0, 0			/* res1[0..2]			*/
	.long	0x644d5241		/* magic: "ARM\x64"		*/
	.long	0			/* res2				*/

real_start:
	/*
	 * Keep the DTB address in a register across BSS zeroing, and store it
	 * only afterwards.
	 *
	 * The first version stored it immediately and then zeroed BSS, which
	 * wiped it: rpi_dtb_pa lives in BSS, so the loader reported
	 * "Device tree: 0x0 (from x0)" on hardware even though the firmware
	 * had passed 0x4000000 -- a fact three earlier probes had confirmed.
	 * The variable now lives in .data as well, so neither reordering this
	 * nor moving the section can bring the bug back.
	 */
	mov	x19, x0

	/* Stack.  AArch64 checks SP alignment on every stack access. */
	adrp	x4, __stack_top
	add	x4, x4, :lo12:__stack_top
	mov	sp, x4

	/*
	 * Zero BSS.  Bounds are 8-byte aligned by the link script, which
	 * matters: with the MMU off all memory is Device-nGnRnE and an
	 * unaligned store faults rather than being fixed up.
	 */
	adrp	x4, __bss_start
	add	x4, x4, :lo12:__bss_start
	adrp	x5, __bss_end
	add	x5, x5, :lo12:__bss_end
1:	cmp	x4, x5
	b.hs	2f
	str	xzr, [x4], #8
	b	1b
2:

	/* Now that BSS is zeroed, publish the entry contract. */
	adrp	x4, rpi_dtb_pa
	add	x4, x4, :lo12:rpi_dtb_pa
	str	x19, [x4]

	bl	main

	/*
	 * main() runs interact() and does not return.  If it ever does there
	 * is nothing to return to, so park in wfi: with no OS there is no fan
	 * control, and an idling core is the coolest way to sit.
	 */
3:	wfi
	b	3b

	/*
	 * In .data, not .bss, deliberately: a value the BSS-zeroing loop
	 * cannot reach even if someone later moves the store back before it.
	 */
	.section .data
	.align	3
	.globl	rpi_dtb_pa
rpi_dtb_pa:
	.quad	0
