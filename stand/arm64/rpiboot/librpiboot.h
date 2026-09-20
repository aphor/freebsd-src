/*-
 * librpiboot.h -- interfaces internal to the Raspberry Pi 5 loader.
 */

#ifndef	_LIBRPIBOOT_H_
#define	_LIBRPIBOOT_H_

struct env_var;

/* main.c */
int	rpi_autoload(void);

/* copy.c -- trivial here because the MMU is off; see that file. */
ssize_t	rpi_copyin(const void *src, vm_offset_t dest, const size_t len);
ssize_t	rpi_copyout(const vm_offset_t src, void *dest, const size_t len);
ssize_t	rpi_readin(readin_handle_t fd, vm_offset_t dest, const size_t len);

/* devicename.c */
int	rpi_getdev(void **vdev, const char *devspec, const char **path);
int	rpi_setcurrdev(struct env_var *ev, int flags, const void *value);

/* pl011_console.c */
extern struct console pl011_console;

#endif	/* _LIBRPIBOOT_H_ */
