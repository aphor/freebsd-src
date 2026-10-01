/*-
 * rpi_mmu.c -- the MMU and caches for the Raspberry Pi 5 loader.
 *
 * WHY
 *
 * The firmware enters the loader at EL2 with the MMU and the D-cache off
 * (start.S), so every data access is Device-nGnRnE: uncached, unbuffered and
 * one at a time.  That was fine for a serial prompt, and is ruinous for the
 * HDMI console (rpi_fb.c): measured with "fbtest", one 1920-pixel row took
 * about 0.8 ms to write and as long to read, so a screen clear took 4.4 s and
 * scrolling one line about 1 s.  Device memory also faults on an unaligned
 * access, which is why rpiboot is built -mstrict-align and why a libsa object
 * built without it (lz4) once hung the console in setup_font().
 *
 * WHAT
 *
 * One identity map (VA == PA) for the EL2 translation regime, 4 KiB granule,
 * 39-bit addresses (T0SZ 25, so the walk starts at level 1: 512 GiB, which
 * covers DRAM and every BCM2712 peripheral window at 0x10_0000_0000 and up):
 *
 *	/memory ranges		Normal, write-back cacheable, executable
 *	the framebuffer		Normal, non-cacheable (rpi_mmu_set_nc())
 *	everything else		Device-nGnRnE, never executable
 *
 * "Everything else" keeps exactly the attribute the loader had before, so
 * the PL011, the mailbox, "peek", and anything not described as memory
 * behave as they did with the MMU off.  Only memory the firmware's device
 * tree calls RAM gets cached.  Blocks are 1 GiB where a whole gigabyte has
 * one attribute and 2 MiB otherwise; a 2 MiB block only partly inside a
 * /memory range stays Device (correct, just slow).  The first gigabyte, which
 * holds the loader, always gets a level-2 table, so a later rpi_mmu_set_nc()
 * never has to split the block the loader is running from.
 *
 * WHAT CHANGES FOR THE REST OF THE LOADER
 *
 * - Anything a non-coherent agent reads or writes in cached RAM needs cache
 *   maintenance.  There is one: the VPU, through the property mailbox, and
 *   rpi_mbox.c cleans and invalidates its buffer around each transaction.
 *   The framebuffer is scanned out by the HVS, and is mapped non-cacheable
 *   rather than cleaned after every draw.
 * - The kernel is entered with the MMU and the D-cache off again, which is
 *   what sys/arm64/arm64/locore.S assumes ("MMU on with an identity map, or
 *   off; D-Cache: off"); see rpi_mmu_handoff() in rpi_mmu_asm.S.
 *
 * An exception vector table is installed as well, so a fault -- the MMU
 * makes new ones possible -- prints a report instead of hanging silently.
 *
 * Register layouts: Arm ARM (DDI 0487) D8 (VMSAv8-64), D24 (SCTLR_EL2,
 * TCR_EL2 with HCR_EL2.E2H == 0, MAIR_EL2).
 */

#include <stand.h>
#include <sys/param.h>

#include <libfdt.h>

#include "bootstrap.h"
#include "librpiboot.h"

extern uint64_t	rpi_dtb_pa;		/* start.S: x0 at entry */
extern char	_start[], _end[];	/* ldscript.arm64 */
extern char	rpi_exception_vectors[];	/* rpi_mmu_asm.S */

/* MAIR_EL2 attribute indices. */
#define	ATTR_DEVICE	0	/* 0x00 Device-nGnRnE */
#define	ATTR_NC		1	/* 0x44 Normal, Inner/Outer Non-cacheable */
#define	ATTR_WB		2	/* 0xff Normal, Inner/Outer WB, RW-allocate */
#define	MMU_MAIR	(0x00UL | (0x44UL << 8) | (0xffUL << 16))

/* Stage 1 block and table descriptors (4 KiB granule). */
#define	PTE_BLOCK	0x1UL		/* level 1 or 2 block */
#define	PTE_TABLE	0x3UL		/* level 1 table */
#define	PTE_TYPE_MASK	0x3UL
#define	PTE_ATTRIDX(i)	((uint64_t)(i) << 2)
#define	PTE_ATTRIDX_MASK (0x7UL << 2)
#define	PTE_SH_IS	(0x3UL << 8)
#define	PTE_AF		(1UL << 10)
#define	PTE_XN		(1UL << 54)	/* EL2 regime: one XN bit */
#define	PTE_ADDR_MASK	0x0000fffffffff000UL

#define	L1_SHIFT	30
#define	L2_SHIFT	21
#define	L1_SIZE		(1UL << L1_SHIFT)
#define	L2_SIZE		(1UL << L2_SHIFT)
#define	NENTRIES	512

/* TCR_EL2 with HCR_EL2.E2H == 0. */
#define	TCR_EL2_RES1	((1UL << 31) | (1UL << 23))
#define	TCR_T0SZ	25		/* 39-bit VA: start at level 1 */
#define	TCR_IRGN0_WBWA	(1UL << 8)
#define	TCR_ORGN0_WBWA	(1UL << 10)
#define	TCR_SH0_IS	(3UL << 12)
#define	TCR_TG0_4K	(0UL << 14)
#define	TCR_PS_SHIFT	16

#define	SCTLR_M		(1UL << 0)
#define	SCTLR_A		(1UL << 1)
#define	SCTLR_C		(1UL << 2)
#define	SCTLR_I		(1UL << 12)
#define	SCTLR_WXN	(1UL << 19)
#define	SCTLR_EE	(1UL << 25)

#define	HCR_E2H		(1UL << 34)

/*
 * Level-2 tables.  dunn needs two at most: the first gigabyte (the loader,
 * and /memory's first bank ending at 0x3f400000 under the framebuffer) and
 * any gigabyte where a /memory range starts or ends off a gigabyte boundary.
 * In BSS, which start.S zeroes and image_size covers.
 */
#define	MMU_L2_TABLES	16

static uint64_t	l1_table[NENTRIES] __aligned(PAGE_SIZE);
static uint64_t	l2_tables[MMU_L2_TABLES][NENTRIES] __aligned(PAGE_SIZE);
static int	l2_used;

/* RAM, from the device tree, and the ranges to map non-cacheable. */
#define	MMU_RANGES	16
struct mmu_range {
	uint64_t	base;
	uint64_t	end;
};
static struct mmu_range	ram[MMU_RANGES];
static int		nram;
static struct mmu_range	nc[4];
static int		nnc;

static bool	mmu_on;
static const char *mmu_why = "not attempted";
static uint64_t	mmu_gib;		/* gigabytes mapped */
static uint64_t	mmu_enable_us;		/* time to build and enable */

#define	DSB(opt)	__asm __volatile("dsb " #opt ::: "memory")
#define	ISB()		__asm __volatile("isb" ::: "memory")
#define	READ_SYSREG(r) ({						\
	uint64_t _v;							\
	__asm __volatile("mrs %0, " #r : "=r" (_v));			\
	_v;								\
})
#define	WRITE_SYSREG(r, v) do {						\
	uint64_t _v = (v);						\
	__asm __volatile("msr " #r ", %0" :: "r" (_v));			\
} while (0)

static uint64_t
now_us(void)
{
	uint64_t frq = READ_SYSREG(cntfrq_el0);

	ISB();
	return (frq != 0 ? READ_SYSREG(cntpct_el0) / (frq / 1000000) : 0);
}

static size_t
dcache_line(void)
{
	return (4UL << ((READ_SYSREG(ctr_el0) >> 16) & 0xf));
}

/*
 * Clean and invalidate [p, p + len) to the point of coherency, so that a
 * non-coherent agent (the VPU) sees what the CPU wrote, and the CPU then
 * sees what the agent wrote.  Unconditional: unlike libarm64's
 * cpu_flush_dcache(), it does not skip the clean when CTR_EL0.IDC says the
 * I-cache needs none, because that bit says nothing about the PoC.
 * Harmless with the MMU off (VA == PA, and no line is allocated then).
 */
void
rpi_dcache_wbinv(const void *p, size_t len)
{
	uintptr_t a, end, line;

	line = dcache_line();
	end = (uintptr_t)p + len;
	DSB(sy);
	for (a = rounddown2((uintptr_t)p, line); a < end; a += line)
		__asm __volatile("dc civac, %0" :: "r" (a) : "memory");
	DSB(sy);
}

/* Invalidate only.  For memory nothing has written through the cache. */
static void
dcache_inv(uintptr_t start, uintptr_t end)
{
	uintptr_t a, line;

	line = dcache_line();
	DSB(sy);
	for (a = rounddown2(start, line); a < end; a += line)
		__asm __volatile("dc ivac, %0" :: "r" (a) : "memory");
	DSB(sy);
}

static void
tlb_flush(void)
{
	DSB(ishst);
	__asm __volatile("tlbi alle2" ::: "memory");
	DSB(ish);
	ISB();
}

static uint64_t
fdt_cells(const fdt32_t *p, int n)
{
	uint64_t v = 0;

	while (n-- > 0)
		v = (v << 32) | fdt32_to_cpu(*p++);
	return (v);
}

/* Every device_type = "memory" node's reg ranges. */
static int
mmu_read_memory(void)
{
	const void *dtb = (const void *)(uintptr_t)rpi_dtb_pa;
	const fdt32_t *p;
	const char *type;
	uint64_t a, s;
	int ac, sc, i, len, n, node;

	if (rpi_dtb_pa == 0 || fdt_check_header(dtb) != 0) {
		mmu_why = "no device tree";
		return (ENXIO);
	}
	ac = fdt_address_cells(dtb, 0);
	sc = fdt_size_cells(dtb, 0);
	if (ac < 1 || ac > 2 || sc < 1 || sc > 2) {
		mmu_why = "unexpected #address-cells/#size-cells at /";
		return (ENXIO);
	}
	nram = 0;
	for (node = fdt_first_subnode(dtb, 0); node >= 0;
	    node = fdt_next_subnode(dtb, node)) {
		type = fdt_getprop(dtb, node, "device_type", &len);
		if (type == NULL || strcmp(type, "memory") != 0)
			continue;
		p = fdt_getprop(dtb, node, "reg", &len);
		if (p == NULL)
			continue;
		n = len / (int)sizeof(*p) / (ac + sc);
		for (i = 0; i < n; i++, p += ac + sc) {
			a = fdt_cells(p, ac);
			s = fdt_cells(p + ac, sc);
			if (s == 0)
				continue;
			if (nram == MMU_RANGES) {
				mmu_why = "more /memory ranges than the "
				    "loader can map";
				return (E2BIG);
			}
			ram[nram].base = a;
			ram[nram].end = a + s;
			nram++;
		}
	}
	if (nram == 0) {
		mmu_why = "no /memory ranges in the device tree";
		return (ENXIO);
	}
	return (0);
}

static bool
in_ranges(const struct mmu_range *r, int n, uint64_t base, uint64_t size)
{
	int i;

	for (i = 0; i < n; i++)
		if (base >= r[i].base && base + size <= r[i].end)
			return (true);
	return (false);
}

static bool
touches_ranges(const struct mmu_range *r, int n, uint64_t base, uint64_t size)
{
	int i;

	for (i = 0; i < n; i++)
		if (base < r[i].end && r[i].base < base + size)
			return (true);
	return (false);
}

/* The attribute for [pa, pa + size): NC wins, then RAM, then Device. */
static int
attr_of(uint64_t pa, uint64_t size)
{
	if (touches_ranges(nc, nnc, pa, size))
		return (ATTR_NC);
	if (in_ranges(ram, nram, pa, size))
		return (ATTR_WB);
	return (ATTR_DEVICE);
}

static uint64_t
block_desc(uint64_t pa, int attr)
{
	uint64_t d;

	d = pa | PTE_BLOCK | PTE_AF | PTE_ATTRIDX(attr);
	if (attr == ATTR_DEVICE)
		d |= PTE_XN;
	else
		d |= PTE_SH_IS;
	if (attr == ATTR_NC)
		d |= PTE_XN;
	return (d);
}

static uint64_t *
l2_alloc(void)
{
	if (l2_used == MMU_L2_TABLES)
		return (NULL);
	return (l2_tables[l2_used++]);
}

/* Fill the level-1 table, and whatever level-2 tables it needs. */
static int
mmu_build(void)
{
	uint64_t *l2, gb;
	int attr, i, j;

	for (i = 0; i < (int)mmu_gib; i++) {
		gb = (uint64_t)i << L1_SHIFT;
		attr = attr_of(gb, L1_SIZE);
		/*
		 * One attribute for the whole gigabyte -- unless it touches
		 * RAM without being all RAM, or holds the loader.
		 */
		if (gb != rounddown2((uint64_t)(uintptr_t)_start, L1_SIZE) &&
		    (attr == ATTR_WB ||
		    (attr == ATTR_DEVICE &&
		    !touches_ranges(ram, nram, gb, L1_SIZE)))) {
			l1_table[i] = block_desc(gb, attr);
			continue;
		}
		if ((l2 = l2_alloc()) == NULL) {
			mmu_why = "out of level-2 tables";
			return (ENOMEM);
		}
		for (j = 0; j < NENTRIES; j++)
			l2[j] = block_desc(gb + ((uint64_t)j << L2_SHIFT),
			    attr_of(gb + ((uint64_t)j << L2_SHIFT), L2_SIZE));
		l1_table[i] = (uint64_t)(uintptr_t)l2 | PTE_TABLE;
	}
	return (0);
}

/*
 * Build the map and turn the MMU and caches on.  Called once from main()
 * with the console up, so that a failure here can at least be reported.
 * heap_start/heap_end are the loader's heap, which (like the image and the
 * device tree) has been written with the D-cache off.
 */
int
rpi_mmu_init(uintptr_t heap_start, uintptr_t heap_end)
{
	uint64_t el, hcr, mmfr0, parange, sctlr, tcr, t0;
	int pabits;
	static const int pa_bits[] = { 32, 36, 40, 42, 44, 48, 52 };

	t0 = now_us();

	/* First, so that anything below that faults says so. */
	WRITE_SYSREG(vbar_el2, (uint64_t)(uintptr_t)rpi_exception_vectors);
	ISB();

	el = (READ_SYSREG(CurrentEL) >> 2) & 3;
	if (el != 2) {
		mmu_why = "not at EL2";
		return (ENXIO);
	}
	hcr = READ_SYSREG(hcr_el2);
	if ((hcr & HCR_E2H) != 0) {
		/* TCR_EL2 has the EL1 layout then; not handled. */
		mmu_why = "HCR_EL2.E2H is set";
		return (ENXIO);
	}
	sctlr = READ_SYSREG(sctlr_el2);
	if ((sctlr & (SCTLR_M | SCTLR_C)) != 0) {
		mmu_why = "the MMU or D-cache was already on at entry";
		return (ENXIO);
	}
	if ((sctlr & SCTLR_EE) != 0) {
		mmu_why = "EL2 is big-endian";
		return (ENXIO);
	}

	/* Output address size: what the CPU implements, up to 48 bits. */
	mmfr0 = READ_SYSREG(id_aa64mmfr0_el1);
	parange = mmfr0 & 0xf;
	if (parange > 5)
		parange = 5;
	pabits = pa_bits[parange];
	if (pabits < 37) {
		mmu_why = "physical addresses narrower than the peripherals";
		return (ENXIO);
	}
	mmu_gib = 1UL << (MIN(pabits, 64 - TCR_T0SZ) - L1_SHIFT);
	if (mmu_gib > NENTRIES)
		mmu_gib = NENTRIES;

	if (mmu_read_memory() != 0)
		return (ENXIO);
	if (!in_ranges(ram, nram, (uintptr_t)_start,
	    (uintptr_t)_end - (uintptr_t)_start)) {
		mmu_why = "the loader image is not inside /memory";
		return (ENXIO);
	}
	if (mmu_build() != 0)
		return (ENOMEM);

	/*
	 * With the D-cache off nothing has been allocated in it, so memory
	 * holds what the firmware and this loader wrote.  Invalidate anyway
	 * whatever is about to be read through the cache -- the image (and
	 * the page tables inside it), the device tree, the heap -- so that a
	 * stale line from before entry, if a reset ever left one, cannot
	 * shadow it.
	 */
	dcache_inv((uintptr_t)_start, (uintptr_t)_end);
	if (rpi_dtb_pa != 0)
		dcache_inv(rpi_dtb_pa, rpi_dtb_pa +
		    fdt_totalsize((const void *)(uintptr_t)rpi_dtb_pa));
	dcache_inv(heap_start, heap_end);
	__asm __volatile("ic iallu" ::: "memory");
	DSB(sy);

	tcr = TCR_EL2_RES1 | TCR_T0SZ | TCR_IRGN0_WBWA | TCR_ORGN0_WBWA |
	    TCR_SH0_IS | TCR_TG0_4K | (parange << TCR_PS_SHIFT);
	WRITE_SYSREG(mair_el2, MMU_MAIR);
	WRITE_SYSREG(tcr_el2, tcr);
	WRITE_SYSREG(ttbr0_el2, (uint64_t)(uintptr_t)l1_table);
	ISB();
	tlb_flush();

	sctlr &= ~(SCTLR_A | SCTLR_WXN);
	sctlr |= SCTLR_M | SCTLR_C | SCTLR_I;
	WRITE_SYSREG(sctlr_el2, sctlr);
	ISB();

	mmu_on = true;
	mmu_why = NULL;
	mmu_enable_us = now_us() - t0;
	return (0);
}

/*
 * Map [pa, pa + size) Normal non-cacheable, for memory another agent reads
 * behind the CPU's back: the framebuffer.  Rounded out to 2 MiB.  Before
 * rpi_mmu_init() this only records the range; afterwards it rewrites the
 * level-2 entries with break-before-make.
 */
int
rpi_mmu_set_nc(uint64_t pa, uint64_t size)
{
	uint64_t a, end, gb, *l1e, *l2, *l2e;
	int attr, j;

	if (size == 0)
		return (0);
	if (nnc == nitems(nc))
		return (E2BIG);
	a = rounddown2(pa, L2_SIZE);
	end = roundup2(pa + size, L2_SIZE);
	nc[nnc].base = a;
	nc[nnc].end = end;
	nnc++;
	if (!mmu_on)
		return (0);

	for (; a < end; a += L2_SIZE) {
		if ((a >> L1_SHIFT) >= mmu_gib)
			return (EINVAL);
		l1e = &l1_table[a >> L1_SHIFT];
		gb = rounddown2(a, L1_SIZE);
		if ((*l1e & PTE_TYPE_MASK) == PTE_BLOCK) {
			/* Split the gigabyte; never the loader's (mmu_build). */
			if ((l2 = l2_alloc()) == NULL)
				return (ENOMEM);
			attr = (int)((*l1e & PTE_ATTRIDX_MASK) >> 2);
			for (j = 0; j < NENTRIES; j++)
				l2[j] = block_desc(gb +
				    ((uint64_t)j << L2_SHIFT), attr);
			DSB(ishst);
			*l1e = 0;
			tlb_flush();
			*l1e = (uint64_t)(uintptr_t)l2 | PTE_TABLE;
			DSB(ishst);
			ISB();
		}
		l2 = (uint64_t *)(uintptr_t)(*l1e & PTE_ADDR_MASK);
		l2e = &l2[(a >> L2_SHIFT) & (NENTRIES - 1)];
		attr = (int)((*l2e & PTE_ATTRIDX_MASK) >> 2);
		if (attr == ATTR_NC)
			continue;
		/* Cached lines written back before the alias goes away. */
		if (attr == ATTR_WB)
			rpi_dcache_wbinv((void *)(uintptr_t)a, L2_SIZE);
		*l2e = 0;
		tlb_flush();
		*l2e = block_desc(a, ATTR_NC);
		DSB(ishst);
		ISB();
	}
	return (0);
}

bool
rpi_mmu_enabled(void)
{
	return (mmu_on);
}

/*
 * The exception report.  Entered from rpi_mmu_asm.S on the faulting stack,
 * with whatever the MMU and caches were; never returns.
 */
void rpi_exception(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t)
    __dead2;

void
rpi_exception(uint64_t vec, uint64_t esr, uint64_t elr, uint64_t far,
    uint64_t spsr)
{
	static const char *kind[] = { "synchronous", "IRQ", "FIQ", "SError" };
	static const char *from[] = { "EL2, SP_EL0", "EL2, SP_EL2",
	    "a lower EL, AArch64", "a lower EL, AArch32" };
	static int nested;

	if (nested++ == 0) {
		printf("\n*** loader exception: %s, from %s\n",
		    kind[vec & 3], from[(vec >> 2) & 3]);
		printf("    ESR_EL2  0x%016jx (EC 0x%02jx)\n", (uintmax_t)esr,
		    (uintmax_t)((esr >> 26) & 0x3f));
		printf("    ELR_EL2  0x%016jx\n", (uintmax_t)elr);
		printf("    FAR_EL2  0x%016jx\n", (uintmax_t)far);
		printf("    SPSR_EL2 0x%016jx\n", (uintmax_t)spsr);
		printf("    MMU %s.  Power-cycle the board (or TryBoot "
		    "falls back to rpiboot.bin).\n", mmu_on ? "on" : "off");
	}
	for (;;)
		__asm __volatile("wfi");
}

static const char *
attr_name(int attr)
{
	switch (attr) {
	case ATTR_DEVICE:	return ("Device-nGnRnE");
	case ATTR_NC:		return ("Normal non-cacheable");
	case ATTR_WB:		return ("Normal write-back");
	default:		return ("?");
	}
}

/* Print the map as runs of one attribute. */
static void
mmu_print_map(void)
{
	uint64_t a, d, run, step, *l2;
	int attr, cur, i, j, nj;

	cur = -1;
	run = 0;
	for (i = 0; i < (int)mmu_gib; i++) {
		d = l1_table[i];
		l2 = NULL;
		nj = 1;
		step = L1_SIZE;
		if ((d & PTE_TYPE_MASK) == PTE_TABLE) {
			l2 = (uint64_t *)(uintptr_t)(d & PTE_ADDR_MASK);
			nj = NENTRIES;
			step = L2_SIZE;
		}
		for (j = 0; j < nj; j++) {
			a = ((uint64_t)i << L1_SHIFT) + (uint64_t)j * step;
			if (l2 != NULL)
				d = l2[j];
			attr = (int)((d & PTE_ATTRIDX_MASK) >> 2);
			if (attr != cur) {
				if (cur >= 0)
					printf("   0x%010jx-0x%010jx  %s\n",
					    (uintmax_t)run, (uintmax_t)a - 1,
					    attr_name(cur));
				cur = attr;
				run = a;
			}
		}
	}
	printf("   0x%010jx-0x%010jx  %s\n", (uintmax_t)run,
	    (uintmax_t)(mmu_gib << L1_SHIFT) - 1, attr_name(cur));
}

void
rpi_mmu_report(void)
{
	if (!mmu_on) {
		printf("   MMU:             off (%s)\n", mmu_why);
		return;
	}
	printf("   MMU:             on, caches on; identity map, %ju GiB, "
	    "%d level-2 tables, %ju us\n", (uintmax_t)mmu_gib, l2_used,
	    (uintmax_t)mmu_enable_us);
}

static int
command_mmu(int argc __unused, char *argv[] __unused)
{
	int i;

	rpi_mmu_report();
	printf("   SCTLR_EL2 0x%016jx  HCR_EL2 0x%016jx\n",
	    (uintmax_t)READ_SYSREG(sctlr_el2), (uintmax_t)READ_SYSREG(hcr_el2));
	printf("   TCR_EL2   0x%016jx  MAIR_EL2 0x%016jx\n",
	    (uintmax_t)READ_SYSREG(tcr_el2), (uintmax_t)READ_SYSREG(mair_el2));
	printf("   TTBR0_EL2 0x%016jx  VBAR_EL2 0x%016jx\n",
	    (uintmax_t)READ_SYSREG(ttbr0_el2), (uintmax_t)READ_SYSREG(vbar_el2));
	printf("   CTR_EL0   0x%016jx  (D-cache line %zu bytes)\n",
	    (uintmax_t)READ_SYSREG(ctr_el0), dcache_line());
	for (i = 0; i < nram; i++)
		printf("   /memory   0x%010jx-0x%010jx\n",
		    (uintmax_t)ram[i].base, (uintmax_t)ram[i].end - 1);
	for (i = 0; i < nnc; i++)
		printf("   non-cacheable 0x%010jx-0x%010jx\n",
		    (uintmax_t)nc[i].base, (uintmax_t)nc[i].end - 1);
	if (mmu_on)
		mmu_print_map();
	return (CMD_OK);
}
COMMAND_SET(mmu, "mmu", "show the loader's MMU, cache and memory map",
    command_mmu);
