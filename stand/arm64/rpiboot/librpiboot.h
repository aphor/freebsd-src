/*-
 * librpiboot.h -- interfaces internal to the Raspberry Pi 5 loader.
 */

#ifndef	_LIBRPIBOOT_H_
#define	_LIBRPIBOOT_H_

struct env_var;

/* main.c */
int	rpi_autoload(void);
void	rpi_psci_reset(void);

/* rpi_mbox.c -- the VPU property mailbox and TryBoot. */
void	rpi_print_boot_config(void);

/* rpi_fdt.c */
int	rpi_fdt_tryboot(void);

/* copy.c -- and the staging translation; see that file for why one is needed. */
void	*rpi_translate(vm_offset_t va);
ssize_t	rpi_copyin(const void *src, vm_offset_t dest, const size_t len);
ssize_t	rpi_copyout(const vm_offset_t src, void *dest, const size_t len);
ssize_t	rpi_readin(readin_handle_t fd, vm_offset_t dest, const size_t len);

/* devicename.c -- setcurrdev comes from the MI gen_setcurrdev(). */
int	rpi_getdev(void **vdev, const char *devspec, const char **path);

/* pl011_console.c */
extern struct console pl011_console;

/*
 * bi_load() is shared from stand/efi/loader/bootinfo.c, but its only
 * declaration lives in stand/efi/loader/loader_efi.h -- a header full of EFI
 * types that cannot be included here.  stand/kboot has the same problem and
 * declares it locally too.  Repeated verbatim, so a signature change upstream
 * is a compile error rather than a silently wrong call.
 */
int	bi_load(char *args, vm_offset_t *modulep, vm_offset_t *kernendp,
	    bool exit_bs);

/* bootinfo_arch.c -- the platform half of bi_load(); see kboot.h. */
void	bi_loadsmap(struct preloaded_file *kfp);

#endif	/* _LIBRPIBOOT_H_ */
