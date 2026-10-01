/*-
 * rpi_mmu_asm.S -- the loader's EL2 exception vectors, and the handoff to
 * the kernel with the MMU and D-cache off.  See rpi_mmu.c.
 */

	.text

/*
 * VBAR_EL2: sixteen 128-byte entries, 2 KiB aligned.  Each passes its index
 * (kind in bits 1:0, origin in bits 3:2, the Arm ARM's order) to
 * rpi_exception(), which prints the syndrome and parks.  Nothing returns, so
 * nothing is saved; the report runs on the faulting stack.
 */
	.macro	vector idx
	.balign	0x80
	mov	x0, #\idx
	b	rpi_exception_common
	.endm

	.balign	0x800
	.globl	rpi_exception_vectors
rpi_exception_vectors:
	vector	0
	vector	1
	vector	2
	vector	3
	vector	4
	vector	5
	vector	6
	vector	7
	vector	8
	vector	9
	vector	10
	vector	11
	vector	12
	vector	13
	vector	14
	vector	15

rpi_exception_common:
	mrs	x1, esr_el2
	mrs	x2, elr_el2
	mrs	x3, far_el2
	mrs	x4, spsr_el2
	bl	rpi_exception
1:	wfi
	b	1b

/*
 * void rpi_mmu_handoff(void *entry, uint64_t arg, uintptr_t start,
 *     uintptr_t end) __dead2
 *
 * Enter the kernel the way sys/arm64/arm64/locore.S assumes -- MMU off,
 * D-cache off, I-cache either -- with no cache line anywhere in
 * [start, end) left to disagree with memory.
 *
 * The order is the point.  Turning the D-cache off first means that from
 * then on nothing this code does can allocate a line (with SCTLR_EL2.M and C
 * clear every data access is Non-cacheable), so the clean-and-invalidate
 * that follows leaves the range truly empty: dirty lines (the staged kernel,
 * its metadata and device tree, the loader's own data) are written back, and
 * clean ones, including any the CPU fetched speculatively after an earlier
 * clean, are dropped.  The kernel then writes its first page tables with the
 * MMU off and reads them back through the cache once it turns the MMU on;
 * a stale line there would hand it its own zeroed BSS.  Cleaning while the
 * cache was still on could not rule that out.
 *
 * Registers only from here on: memory reads would now bypass lines that may
 * still be dirty.  The caller passes [start, end) covering everything the
 * loader wrote or read through the cache.
 */
	.globl	rpi_mmu_handoff
rpi_mmu_handoff:
	mov	x19, x0			/* entry */
	mov	x20, x1			/* arg (modulep) */

	dsb	sy
	mrs	x4, sctlr_el2
	bic	x4, x4, #(1 << 0)	/* M */
	bic	x4, x4, #(1 << 2)	/* C */
	msr	sctlr_el2, x4
	isb

	/* D-cache line size: 4 << CTR_EL0.DminLine bytes. */
	mrs	x5, ctr_el0
	ubfx	x5, x5, #16, #4
	mov	x6, #4
	lsl	x6, x6, x5
	sub	x7, x6, #1
	bic	x2, x2, x7
1:	cmp	x2, x3
	b.hs	2f
	dc	civac, x2
	add	x2, x2, x6
	b	1b
2:	dsb	sy

	/* The loader's translations, and any instructions it wrote as data. */
	tlbi	alle2
	ic	iallu
	dsb	sy
	isb

	mov	x0, x20
	br	x19
