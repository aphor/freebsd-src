/*-
 * time.c -- timekeeping for the Raspberry Pi 5 loader.
 *
 * From the AArch64 generic timer, which needs no setup and is readable at EL2
 * with the MMU off.  CNTFRQ_EL0 reads 54,000,000 Hz on this board, agreeing
 * with the running kernel's "ARM MPCore Timecounter frequency 54000000 Hz".
 *
 * There is no real-time clock here.  Under EFI the loader gets wall-clock time
 * from the runtime services; booted straight from the VPU firmware there is
 * nothing to ask, so time() returns seconds since entry.  That is enough for
 * everything the loader does with it -- timeouts and the autoboot countdown --
 * and a plausible-looking wrong date would be worse than an obviously relative
 * one.
 */

#include <stand.h>

static inline uint64_t
cntfrq(void)
{
	uint64_t v;

	__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(v));
	return (v);
}

static inline uint64_t
cntpct(void)
{
	uint64_t v;

	/* isb: the counter read can otherwise be speculated ahead. */
	__asm__ __volatile__("isb; mrs %0, cntpct_el0" : "=r"(v));
	return (v);
}

time_t
time(time_t *tloc)
{
	uint64_t hz = cntfrq();
	time_t t;

	t = (hz == 0) ? 0 : (time_t)(cntpct() / hz);
	if (tloc != NULL)
		*tloc = t;
	return (t);
}

time_t
getsecs(void)
{
	return (time(NULL));
}

void
delay(int usec)
{
	uint64_t hz = cntfrq();
	uint64_t end;

	if (hz == 0 || usec <= 0)
		return;
	end = cntpct() + (hz / 1000000) * (uint64_t)usec;
	while (cntpct() < end)
		;
}
