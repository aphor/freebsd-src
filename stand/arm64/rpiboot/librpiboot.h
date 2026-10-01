/*-
 * librpiboot.h -- interfaces internal to the Raspberry Pi 5 loader.
 */

#ifndef	_LIBRPIBOOT_H_
#define	_LIBRPIBOOT_H_

struct env_var;

/* main.c */
#define	RPI_LOAD_ADDR	0x00200000UL	/* where the firmware loads us */
#define	RPI_HEAP_START	0x08000000UL	/* see main.c */
#define	RPI_HEAP_SIZE	(48UL * 1024 * 1024)
int	rpi_autoload(void);
void	rpi_psci_reset(void);

/* rpi_mmu.c, rpi_mmu_asm.S -- the MMU, the caches, and the kernel handoff. */
int	rpi_mmu_init(uintptr_t heap_start, uintptr_t heap_end);
int	rpi_mmu_set_nc(uint64_t pa, uint64_t size);
bool	rpi_mmu_enabled(void);
void	rpi_mmu_report(void);
void	rpi_dcache_wbinv(const void *p, size_t len);
void	rpi_mmu_handoff(void *entry, uint64_t arg, uintptr_t start,
	    uintptr_t end) __dead2;

/* rpi_mbox.c -- the VPU property mailbox and TryBoot. */
void	rpi_print_boot_config(void);
int	rpi_mbox_tag(uint32_t tag, uint32_t *val, uint32_t vallen,
	    uint32_t inlen);
int	rpi_mbox_property(uint32_t *buf);

/* rpi_fb.c -- the firmware's framebuffer: console and device-tree node. */
int	rpi_fb_probe(void);
int	rpi_fb_fdt_node(void *dtb);
extern struct console rpi_fb_console;

/* rpi_pcie.c -- PCIe2 and RP1, for the USB keyboard. */
int	rpi_pcie_rp1_init(const char **why);
uint64_t rpi_pcie_rp1_base(void);
void	rpi_pcie_shutdown(void);

/* rpi_usbkbd.c -- a USB keyboard on RP1's xHCI controllers. */
void	rpi_usbkbd_init(void);
bool	rpi_usbkbd_poll(void);
int	rpi_usbkbd_getchar(void);
void	rpi_usbkbd_shutdown(void);

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
