/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Contributors
 * All rights reserved.
 *
 * Resolve BCM2712 / RP1 register windows through the device tree.
 *
 * Every driver in this set reaches its registers by physical address rather
 * than through a bus, because nothing enumerates the RP1 subtree: RP1 hangs
 * off a brcm,bcm2712-pcie root complex and FreeBSD has no host controller
 * driver for it.  The addresses themselves are in the device tree the
 * firmware publishes, and OFW is installed and initialised before
 * bus_probe() picks a bus method (sys/arm64/arm64/machdep.c), so they can be
 * read on an ACPI boot just as well as on an FDT one.
 *
 * There are two cases, because the tree has two shapes.
 *
 * Nodes hanging off the soc or axi simple-buses -- the AVS monitor, the PCIe
 * root complexes -- translate with plain ranges hops, and
 * bcm2712_fdt_reg() uses ofw_reg_to_paddr() for them.  ofw_subr.h calls that
 * the back end of OF_decode_addr() rather than a driver interface; it is used
 * directly here, as arm64's own gicv5_fdt.c and gicv5_its.c do, because these
 * drivers want a physical address for pmap_mapdev_attr(), not a bus_space
 * handle.
 *
 * RP1's windows do not, and bcm2712_fdt_rp1_reg() exists because of it.  The
 * chain is
 *
 *   ethernet@100000  reg  0xc0_40100000        (rp1, simple-bus)
 *     rp1 ranges     ->   PCI mem space, 0x0
 *     pcie ranges    ->   0x1f_00100000        (axi)
 *     axi ranges     ->   identity             (root)
 *
 * and ofw_reg_to_paddr() gets it wrong: it returns 0xc0_40100000 unchanged.
 * Measured on dunn, not deduced -- see the fdtprobe results in
 * doc/FDT_BOOT.md of rpi5_modules.git.
 *
 * The reason is that rp1 is a *simple-bus child of a PCI node*, which the
 * function does not expect.  For a node under rp1 it computes pci = 0 (rp1
 * has #address-cells = <2>, so the "3 address cells and 2 size cells plus
 * device_type = pci" test fails) and therefore pci_hi = OFW_PADDR_NOT_PCI,
 * giving spc = 0x03000000.  On the first hop it sees the parent is PCI and
 * takes pci_hi from cell[0] of the ranges it is walking -- but those are
 * rp1's ranges, whose child cells are plain 2-cell addresses, so cell[0] is
 * 0xc0, not a PCI space code, and spc becomes 0.  Every entry then fails the
 * rspc != spc test and no translation is applied.  The function assumes the
 * child side of the ranges is PCI-formatted, i.e. a PCI device under a PCI
 * bus; the RP1 is the other way round.
 *
 * So the RP1 hops are done here, reading the same two ranges properties
 * directly.  This is the derivation already written out by hand in the
 * comments in bcm2712_var.h and rp1_eth_var.h, done at run time from the
 * device tree instead of precomputed into a constant.
 *
 * Callers pass the fallback they already had and ignore a failure, so a board
 * whose device tree does not describe a block keeps working exactly as before.
 *
 * On the FDT lane (M2 phase 2), RP1 is a PCI device and BAR1 lands wherever
 * PCI puts it, not where rp1's ranges assume.  Once the rp1 PCI driver has
 * published BAR1 (bcm2712_rp1_bar()), hop 2 is replaced: the address is
 * BAR1's CPU address plus the offset into BAR1 that hop 1 gives.
 */

#ifndef _BCM2712_FDT_H_
#define _BCM2712_FDT_H_

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

/*
 * Find the first node present at one of `paths`.  Two spellings are in
 * circulation for the RP1 subtree -- the vendor DTB puts it under
 * /axi/pcie@1000120000/rp1 and an upstream-style tree under /soc/rp1 -- so
 * callers pass a NULL-terminated list and the unit address in the path is
 * what identifies the block.  If `compat` is non-NULL the node must also
 * carry it, which guards against a same-named node on a different board.
 */
/*
 * #address-cells / #size-cells of a node, with the device tree defaults.
 * (ofw_subr.c has an equivalent, but it is static to that file.)
 */
static inline void
get_addr_props_compat(phandle_t node, uint32_t *nap, uint32_t *nsp)
{
	uint32_t na, ns;

	if (OF_getencprop(node, "#address-cells", &na, sizeof(na)) == -1)
		na = 2;
	if (OF_getencprop(node, "#size-cells", &ns, sizeof(ns)) == -1)
		ns = 1;
	if (nap != NULL)
		*nap = na;
	if (nsp != NULL)
		*nsp = ns;
}

static inline phandle_t
bcm2712_fdt_find(const char * const *paths, const char *compat)
{
	phandle_t node;
	int i;

	for (i = 0; paths[i] != NULL; i++) {
		node = OF_finddevice(paths[i]);
		if (node == -1)
			continue;
		if (compat != NULL && !ofw_bus_node_is_compatible(node, compat))
			continue;
		return (node);
	}
	return (-1);
}

/*
 * Resolve register window `regno` of that node to a CPU physical address.
 * Returns true and overwrites *pa (and *size, if non-NULL) only on success,
 * so the idiom is:
 *
 *	bus_addr_t pa = SOME_BASE_PHYS;
 *	bool fdt = bcm2712_fdt_reg(paths, compat, 0, &pa, NULL);
 *
 * and *pa is usable either way.
 */
static inline bool
bcm2712_fdt_reg(const char * const *paths, const char *compat, int regno,
    bus_addr_t *pa, bus_size_t *size)
{
	phandle_t node;
	bus_addr_t addr;
	bus_size_t sz;

	node = bcm2712_fdt_find(paths, compat);
	if (node == -1)
		return (false);
	if (ofw_reg_to_paddr(node, regno, &addr, &sz, NULL) != 0)
		return (false);

	*pa = addr;
	if (size != NULL)
		*size = sz;
	return (true);
}

/*
 * Read one address/size pair out of a node's reg property, using the cell
 * counts its parent declares.
 */
static inline bool
bcm2712_fdt_reg_cells(phandle_t node, int regno, uint64_t *addr, uint64_t *size)
{
	pcell_t cells[32];
	uint32_t na, ns;
	int len, n, base, i;

	get_addr_props_compat(OF_parent(node), &na, &ns);
	if (na == 0 || na + ns == 0 || na + ns > nitems(cells))
		return (false);
	len = OF_getencprop(node, "reg", cells, sizeof(cells));
	if (len <= 0)
		return (false);
	n = len / (int)sizeof(cells[0]);
	base = regno * (int)(na + ns);
	if (base + (int)(na + ns) > n)
		return (false);

	*addr = 0;
	for (i = 0; i < (int)na; i++)
		*addr = (*addr << 32) | cells[base++];
	*size = 0;
	for (i = 0; i < (int)ns; i++)
		*size = (*size << 32) | cells[base++];
	return (true);
}

/*
 * Resolve reg[regno] of a node under the RP1 simple-bus to a CPU physical
 * address, by applying rp1's ranges (RP1-child -> PCI) and then the root
 * complex's ranges (PCI -> CPU).  See the file comment for why
 * ofw_reg_to_paddr() cannot be used for this.
 *
 * The final axi -> root hop is an identity map on this board, so it is not
 * applied.  ToDo: if a board ever appears whose /axi ranges are not identity
 * over the RP1 window, this needs a third hop.
 */
bool bcm2712_rp1_bar(bus_addr_t *pa, bus_size_t *size);	/* bcm2712.c */

static inline bool
bcm2712_fdt_rp1_reg(phandle_t node, int regno, bus_addr_t *pa,
    bus_size_t *size)
{
	phandle_t rp1, rc;
	pcell_t r[64];
	uint32_t na_rp1, ns_rp1, na_rc, ns_rc, na_par;
	uint64_t addr, sz, cbase, pbase, psize, pci_addr, pci_base0, cpu;
	bus_addr_t bar_pa;
	bus_size_t bar_size;
	uint32_t space;
	int len, n, i, e, c;
	bool hop1 = false;

	if (node == -1 || node == 0)
		return (false);
	rp1 = OF_parent(node);
	if (rp1 == 0)
		return (false);
	rc = OF_parent(rp1);
	if (rc == 0)
		return (false);

	if (!bcm2712_fdt_reg_cells(node, regno, &addr, &sz))
		return (false);

	/* Hop 1: rp1 ranges, child (rp1) -> parent (PCI, 3 cells). */
	get_addr_props_compat(rp1, &na_rp1, &ns_rp1);
	get_addr_props_compat(rc, &na_rc, &ns_rc);
	len = OF_getencprop(rp1, "ranges", r, sizeof(r));
	n = (len > 0) ? len / (int)sizeof(r[0]) : 0;
	e = (int)(na_rp1 + na_rc + ns_rp1);
	pci_base0 = 0;
	for (i = 0; e > 0 && i + e <= n; i += e) {
		c = i;
		cbase = 0;
		for (unsigned k = 0; k < na_rp1; k++)
			cbase = (cbase << 32) | r[c++];
		space = r[c] & 0x03000000;	/* PCI space code */
		pci_addr = 0;
		for (unsigned k = 1; k < na_rc; k++)
			pci_addr = (pci_addr << 32) | r[c + k];
		if (i == 0)
			pci_base0 = pci_addr;	/* where the DT puts BAR1 */
		c += na_rc;
		psize = 0;
		for (unsigned k = 0; k < ns_rp1; k++)
			psize = (psize << 32) | r[c++];
		if (addr < cbase || addr >= cbase + psize)
			continue;
		pci_addr += addr - cbase;
		hop1 = true;
		break;
	}
	if (!hop1)
		return (false);

	/* FDT lane: BAR1 is where PCI put it, as the rp1 driver found it. */
	if (bcm2712_rp1_bar(&bar_pa, &bar_size)) {
		if (pci_addr < pci_base0 || pci_addr - pci_base0 >= bar_size)
			return (false);
		*pa = bar_pa + (pci_addr - pci_base0);
		if (size != NULL)
			*size = sz;
		return (true);
	}

	/* Hop 2: root complex ranges, child (PCI, 3 cells) -> parent (axi). */
	get_addr_props_compat(OF_parent(rc), &na_par, NULL);
	len = OF_getencprop(rc, "ranges", r, sizeof(r));
	n = (len > 0) ? len / (int)sizeof(r[0]) : 0;
	e = (int)(na_rc + na_par + ns_rc);
	for (i = 0; e > 0 && i + e <= n; i += e) {
		c = i;
		if ((r[c] & 0x03000000) != space) {
			continue;
		}
		cbase = 0;
		for (unsigned k = 1; k < na_rc; k++)
			cbase = (cbase << 32) | r[c + k];
		c += na_rc;
		pbase = 0;
		for (unsigned k = 0; k < na_par; k++)
			pbase = (pbase << 32) | r[c++];
		psize = 0;
		for (unsigned k = 0; k < ns_rc; k++)
			psize = (psize << 32) | r[c++];
		if (pci_addr < cbase || pci_addr >= cbase + psize)
			continue;
		cpu = pbase + (pci_addr - cbase);
		*pa = cpu;
		if (size != NULL)
			*size = sz;
		return (true);
	}
	return (false);
}

/*
 * Path-based wrapper, for callers that do not already hold the node.
 */
static inline bool
bcm2712_fdt_rp1(const char * const *paths, const char *compat, int regno,
    bus_addr_t *pa, bus_size_t *size)
{
	phandle_t node;

	node = bcm2712_fdt_find(paths, compat);
	if (node == -1)
		return (false);
	return (bcm2712_fdt_rp1_reg(node, regno, pa, size));
}

/* The RP1 subtree, as the two device trees this board is known to publish. */
#define BCM2712_RP1_PATHS(name)						\
	{ "/axi/pcie@1000120000/rp1/" name, "/soc/rp1/" name, NULL }

#endif /* _BCM2712_FDT_H_ */
