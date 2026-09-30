/*-
 * kboot.h -- a shim, present only to satisfy stand/efi/loader/bootinfo.c.
 *
 * bootinfo.c is shared rather than forked, because the metadata it builds is a
 * contract with sys/arm64 and reimplementing it would be a worse trade than
 * this file.  It is already #ifdef EFI-guarded and stand/kboot reuses it the
 * same way, via .PATH.  But its non-EFI branch reads
 *
 *	#else
 *	#include "kboot.h"
 *	#endif
 *
 * and calls bi_loadsmap(kfp), so the file is coupled to *kboot* specifically
 * rather than to "not EFI" generally.  Hence this header, which declares the
 * one thing that branch needs.
 *
 * This is acknowledged debt, not a design.  The clean fix is a generic non-EFI
 * hook in bootinfo.c -- something like bi_load_platform_data() with a weak or
 * per-platform definition -- rather than a kboot-shaped one.  That is a change
 * to a file shared with kboot, so it wants doing deliberately and with kboot
 * retested, not smuggled in here.
 */

#ifndef	_RPIBOOT_KBOOT_SHIM_H_
#define	_RPIBOOT_KBOOT_SHIM_H_

struct preloaded_file;

void	bi_loadsmap(struct preloaded_file *kfp);

#endif	/* _RPIBOOT_KBOOT_SHIM_H_ */
