/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 FreeBSD Contributors
 * All rights reserved.
 *
 * BCM2712 Common Hardware Support Module
 * Provides common functions for BCM2712-based systems
 * Including basic hardware abstraction and thermal sensor access
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/callout.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/bus.h>


#include "bcm2712_fdt.h"
#include "bcm2712_var.h"

MALLOC_DEFINE(M_BCM2712, "bcm2712", "BCM2712 driver memory");

/* Global BCM2712 state */
static struct bcm2712_softc *bcm2712_sc = NULL;

/* Debug flag for verbose logging */
static int bcm2712_debug = 0;

/*
 * BCM2712 AVS Ring Oscillator Thermal Sensor
 *
 * Hardware: AVS (Adaptive Voltage Scaling) monitor in BCM2712 SoC
 * The thermal sensor is implemented as a ring oscillator whose frequency
 * varies with temperature. The raw value is converted to actual temperature
 * using calibration slope and offset.
 *
 * The device tree defines the AVS monitor at physical address 0x7d542000
 * with size 0xf00. This driver attaches to the "brcm,bcm2711-thermal"
 * device node within the avs-monitor to access the temperature register.
 *
 * References:
 *   - Linux: drivers/thermal/broadcom/bcm2711_thermal.c
 *   - FreeBSD: sys/arm/mv/armada/thermal.c (driver pattern)
 *   - Device tree: bcm2712-d-rpi-5-b.dtb (AVS monitor @0x7d542000)
 */

/* AVS Monitor base physical address - after soc bus ranges translation
 * DTS address 0x7d542000 + soc base 0x1000000000 = physical 0x107d542000
 */
#define BCM2712_AVS_BASE_PHYS		0x107d542000UL

/*
 * Device tree nodes for the four windows this driver maps.  Each is resolved
 * at load time and the *_BASE_PHYS constants are only the fallback; see
 * bcm2712_fdt.h for why the addresses are resolved rather than allocated
 * through a bus.
 */
static const char * const avs_fdt_paths[] = {
	"/soc@107c000000/avs-monitor@7d542000",
	"/soc/avs-monitor@7d542000",
	NULL
};
static const char * const pwm1_fdt_paths[] = BCM2712_RP1_PATHS("pwm@9c000");
static const char * const rp1_clk_fdt_paths[] =
    BCM2712_RP1_PATHS("clocks@18000");
static const char * const rp1_gpio_fdt_paths[] =
    BCM2712_RP1_PATHS("gpio@d0000");

/* Temperature register offset within AVS monitor */
#define BCM2712_AVS_TEMP_OFFSET		0x200

/* Temperature register bit masks */
#define BCM2712_TEMP_DATA_MSK		0x03FF  /* Bits [9:0] - 10-bit code */
#define BCM2712_TEMP_VALID_MSK		0x10400 /* Bits [16] and [10] validity */

/* Temperature conversion calibration (empirically determined for RPi 5) */
#define BCM2712_TEMP_SLOPE		-550    /* millidegrees per code unit */
#define BCM2712_TEMP_OFFSET		450000  /* millidegrees Celsius base offset */

/* TZ_ZEROC in deciKelvin (FreeBSD thermal convention) */
#define TZ_ZEROC	2731

/*
 * Read BCM2712 AVS thermal sensor raw register value.
 * Must be called with thermal_mtx held.
 * Returns temperature in millidegrees Celsius.
 *
 * The AVS thermal sensor is a ring oscillator whose frequency varies with
 * temperature. The raw register value contains a 10-bit code that must be
 * converted to actual temperature using calibration formula.
 */
/*
 * Pretend the sensor has stopped answering, so the fan fail-safe downstream
 * can be exercised deliberately instead of only ever running for the first
 * time during a real sensor failure.  Safe to leave in: every effect of it
 * errs towards more cooling, never less.
 */
static int bcm2712_thermal_fail_inject = 0;

static bool
bcm2712_thermal_read_raw(struct bcm2712_softc *sc, uint32_t *temp_out)
{
	uint32_t raw_value;

	mtx_assert(&sc->thermal_mtx, MA_OWNED);

	if (bcm2712_thermal_fail_inject != 0)
		return (false);

	/*
	 * Returns true only when the hardware produced a reading this call.
	 * It deliberately does not fall back to the cached value: the caller
	 * has to know the difference between "the die is at N" and "the
	 * sensor did not answer", because the second one is a reason to run
	 * the fan, not a reason to reuse a number.
	 */
	if (!sc->avs_mapped || sc->avs_vaddr == NULL) {
		return (false);
	}

	/*
	 * Read raw sensor value from hardware register.
	 * Temperature register is at offset 0x200 within the AVS monitor.
	 * Use direct memory access via virtual address mapping.
	 */
	raw_value = *(uint32_t *)((uintptr_t)sc->avs_vaddr + BCM2712_AVS_TEMP_OFFSET);

	/*
	 * Validity checking: Bit 16 must be set to indicate the sensor has
	 * produced a valid reading. On this hardware, bit 10 is not reliably set
	 * even with valid readings. Accept reading if bit 16 is set.
	 */
	if ((raw_value & 0x10000) == 0) {
		return (false);
	}

	/* Extract 10-bit temperature code from lower bits */
	uint32_t raw_code = raw_value & BCM2712_TEMP_DATA_MSK;

	/*
	 * Convert raw code to temperature using calibration formula:
	 * Temperature = slope × code + offset
	 *
	 * The slope of -550 means higher temperatures have lower raw codes.
	 * This is characteristic of ring oscillator-based sensors.
	 */
	int32_t temp_mc = (BCM2712_TEMP_SLOPE * raw_code) + BCM2712_TEMP_OFFSET;

	/* Clamp to reasonable operating range */
	if (temp_mc < 0)
		temp_mc = 0;
	if (temp_mc > 120000)
		temp_mc = 120000;

	*temp_out = (uint32_t)temp_mc;
	return (true);
}

/*
 * This function is retained for architectural clarity but the actual
 * temperature conversion is now integrated into bcm2712_thermal_read_raw()
 * since we need to check validity and clamp ranges there anyway.
 *
 * Keeps the data/control flow clean: read_raw handles hardware register
 * reads and conversion, this layer is now a no-op.
 */
static uint32_t
bcm2712_thermal_raw_to_millic(uint32_t temp_mc)
{
	/* Conversion already done in bcm2712_thermal_read_raw() */
	return (temp_mc);
}

/*
 * sysctl handler for CPU temperature (deciKelvin format).
 * Converts cached milli-°C to deciKelvin (FreeBSD convention).
 */
static int
bcm2712_thermal_sysctl_temp(SYSCTL_HANDLER_ARGS)
{
	struct bcm2712_softc *sc = arg1;
	uint32_t temp_dk;

	mtx_lock(&sc->thermal_mtx);
	/* Convert cached milli-°C to deciKelvin: (mC / 100) + TZ_ZEROC */
	temp_dk = (sc->cached_temp_mc / 100) + TZ_ZEROC;
	mtx_unlock(&sc->thermal_mtx);

	return (sysctl_handle_int(oidp, &temp_dk, 0, req));
}

/*
 * Periodic thermal sensor update callback.
 * Called every second by callout timer to update cached temperature.
 */
static void
bcm2712_thermal_update(void *arg)
{
	struct bcm2712_softc *sc = arg;
	uint32_t temp_mc;

	mtx_assert(&sc->thermal_mtx, MA_OWNED);

	if (bcm2712_thermal_read_raw(sc, &temp_mc)) {
		sc->cached_temp_mc = bcm2712_thermal_raw_to_millic(temp_mc);
		sc->last_update = time_uptime;
		sc->thermal_invalid_run = 0;
		if (!sc->thermal_healthy) {
			sc->thermal_healthy = true;
			printf("bcm2712: thermal sensor is answering again "
			    "(%u.%u C)\n", sc->cached_temp_mc / 1000,
			    (sc->cached_temp_mc % 1000) / 100);
		}
	} else {
		sc->thermal_invalid_total++;
		if (sc->thermal_invalid_run < UINT32_MAX)
			sc->thermal_invalid_run++;
		/*
		 * Leave cached_temp_mc and last_update alone -- they are the
		 * last thing actually measured, and callers can tell how old
		 * that is from hw.bcm2712.thermal.invalid_run.
		 */
		if (sc->thermal_healthy &&
		    sc->thermal_invalid_run >= BCM2712_THERMAL_STALE_TICKS) {
			sc->thermal_healthy = false;
			printf("bcm2712: thermal sensor produced no valid "
			    "reading for %u ticks; temperature is untrusted "
			    "and the fan will be forced to full\n",
			    sc->thermal_invalid_run);
		}
	}

	/* Reschedule for next update (1 second interval) */
	callout_reset(&sc->thermal_callout, hz, bcm2712_thermal_update, sc);
}

/* Read BCM2712 CPU temperature */
int
bcm2712_read_cpu_temp(uint32_t *temp)
{
	struct bcm2712_softc *sc = bcm2712_sc;

	if (sc == NULL || temp == NULL)
		return (ENODEV);

	mtx_lock(&sc->thermal_mtx);
	if (!sc->thermal_healthy) {
		mtx_unlock(&sc->thermal_mtx);
		return (EIO);
	}
	*temp = sc->cached_temp_mc;
	mtx_unlock(&sc->thermal_mtx);

	return (0);
}

/* Get BCM2712 softc structure for other modules */
struct bcm2712_softc *
bcm2712_get_softc(void)
{
	return (bcm2712_sc);
}

/* Configure PWM channel period and duty cycle (values in nanoseconds). */
int
bcm2712_pwm_set_config(u_int channel, u_int period_ns, u_int duty_ns)
{
	struct bcm2712_softc *sc = bcm2712_sc;
	volatile uint32_t *base;
	uint32_t range, duty, chan_ctrl, gctrl;

	if (channel >= BCM2712_PWM_NCHANNELS)
		return (EINVAL);

	if (sc == NULL || !sc->pwm_mapped)
		return (ENODEV);

	/*
	 * Convert nanoseconds to RP1 PWM clock cycles.  The clock is
	 * 6.144 MHz, so RP1_PWM_CLK_PERIOD_NS is 163 ns per tick -- not the
	 * 50 MHz this comment used to claim.  For the fan's 41566 ns period
	 * that makes range exactly 255, so a 0-255 speed maps one-to-one
	 * onto duty ticks and 255 is precisely full scale.
	 */
	range = (period_ns + RP1_PWM_CLK_PERIOD_NS / 2) / RP1_PWM_CLK_PERIOD_NS;
	duty  = (duty_ns  + RP1_PWM_CLK_PERIOD_NS / 2) / RP1_PWM_CLK_PERIOD_NS;

	mtx_lock(&sc->mtx);

	base = (volatile uint32_t *)sc->pwm_vaddr;

	/* Initialize channel: trailing-edge M/S mode, inverted polarity (fan). */
	chan_ctrl = base[RP1_PWM_CHAN_CTRL(channel) / 4];
	chan_ctrl &= ~0x1ffu;
	chan_ctrl |= RP1_PWM_CHAN_DEFAULT | RP1_PWM_POLARITY;
	base[RP1_PWM_CHAN_CTRL(channel) / 4] = chan_ctrl;

	base[RP1_PWM_DUTY(channel) / 4]  = duty;
	base[RP1_PWM_RANGE(channel) / 4] = range;

	/* Commit changes via SET_UPDATE bit. */
	gctrl = base[RP1_PWM_GLOBAL_CTRL / 4];
	gctrl |= RP1_PWM_SET_UPDATE;
	base[RP1_PWM_GLOBAL_CTRL / 4] = gctrl;

	sc->channels[channel].period = period_ns;
	sc->channels[channel].duty_a = duty_ns;

	if (bcm2712_debug)
		printf("bcm2712: PWM ch%u range=%u duty=%u "
		    "(period=%uns duty=%uns GCTRL=0x%x)\n",
		    channel, range, duty, period_ns, duty_ns,
		    base[RP1_PWM_GLOBAL_CTRL / 4]);

	mtx_unlock(&sc->mtx);
	return (0);
}

/* Enable/disable PWM channel output. */
int
bcm2712_pwm_enable(u_int channel, bool enable)
{
	struct bcm2712_softc *sc = bcm2712_sc;
	volatile uint32_t *base;
	uint32_t gctrl;

	if (channel >= BCM2712_PWM_NCHANNELS)
		return (EINVAL);

	if (sc == NULL || !sc->pwm_mapped)
		return (ENODEV);

	mtx_lock(&sc->mtx);

	base = (volatile uint32_t *)sc->pwm_vaddr;
	sc->channels[channel].enabled = enable;

	gctrl = base[RP1_PWM_GLOBAL_CTRL / 4];
	if (enable)
		gctrl |= RP1_PWM_CHAN_ENABLE(channel);
	else
		gctrl &= ~RP1_PWM_CHAN_ENABLE(channel);
	gctrl |= RP1_PWM_SET_UPDATE;
	base[RP1_PWM_GLOBAL_CTRL / 4] = gctrl;

	if (bcm2712_debug)
		printf("bcm2712: PWM ch%u %s, GLOBAL_CTRL=0x%x\n",
		    channel, enable ? "enabled" : "disabled",
		    base[RP1_PWM_GLOBAL_CTRL / 4]);

	mtx_unlock(&sc->mtx);
	return (0);
}

/*
 * sysctl handler: read RP1 PWM channel-3 registers via already-mapped memory.
 * Returns a one-line string: GCTRL CHAN_CTRL3 RANGE3 DUTY3 — safe to call
 * at any time since the registers are in sc->pwm_vaddr (device-mapped).
 */
static int
bcm2712_sysctl_pwm_regs(SYSCTL_HANDLER_ARGS)
{
	struct bcm2712_softc *sc = arg1;
	volatile uint32_t *base;
	char buf[128];
	int len;

	if (sc == NULL || !sc->pwm_mapped)
		return (ENODEV);

	mtx_lock(&sc->mtx);
	base = (volatile uint32_t *)sc->pwm_vaddr;
	len = snprintf(buf, sizeof(buf),
	    "GCTRL=0x%08x CHAN_CTRL3=0x%08x RANGE3=%u DUTY3=%u",
	    base[RP1_PWM_GLOBAL_CTRL / 4],
	    base[RP1_PWM_CHAN_CTRL(3) / 4],
	    base[RP1_PWM_RANGE(3) / 4],
	    base[RP1_PWM_DUTY(3) / 4]);
	mtx_unlock(&sc->mtx);

	return (SYSCTL_OUT(req, buf, len + 1));
}

/*
 * Read RP1_PWM_TACH_RPM (offset 0x3C, CHAN2_PHASE per datasheet) from the
 * RP1 PWM1 register window.  Hardware testing confirmed this is a
 * firmware-preloaded static value (~10169); it does not track fan speed or
 * PWM state.  Exposed for diagnostic inspection only.
 * Returns 0 if the PWM window is not yet mapped.
 */
uint32_t
bcm2712_read_fan_rpm(void)
{
	struct bcm2712_softc *sc = bcm2712_sc;
	volatile uint32_t *base;
	uint32_t rpm;

	if (sc == NULL || !sc->pwm_mapped)
		return (0);

	mtx_lock(&sc->mtx);
	base = (volatile uint32_t *)sc->pwm_vaddr;
	rpm = base[RP1_PWM_TACH_RPM / 4];
	mtx_unlock(&sc->mtx);

	return (rpm);
}

/* Module event handler for bcm2712 initialization */
static int
bcm2712_modevent(module_t mod __unused, int event, void *arg __unused)
{
	struct bcm2712_softc *sc;
	struct sysctl_oid *tree, *thermal_tree;
	bus_addr_t avs_phys, pwm_phys;
	bool avs_from_fdt, pwm_from_fdt;
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		/* Allocate softc structure */
		sc = malloc(sizeof(struct bcm2712_softc), M_BCM2712, M_WAITOK | M_ZERO);
		if (sc == NULL) {
			printf("bcm2712: Cannot allocate softc\n");
			return (ENOMEM);
		}

		/* Initialize mutexes */
		mtx_init(&sc->mtx, "bcm2712", NULL, MTX_DEF);
		mtx_init(&sc->thermal_mtx, "bcm2712_thermal", NULL, MTX_DEF);

		/* Initialize thermal callout */
		callout_init_mtx(&sc->thermal_callout, &sc->thermal_mtx, 0);

		/* Initialize cached temperature */
		sc->cached_temp_mc = 50000;  /* 50°C */
		sc->last_update = 0;
		sc->thermal_invalid_total = 0;
		sc->thermal_invalid_run = 0;
		sc->thermal_healthy = true;

		/*
		 * Map the AVS monitor.  Its node is a plain child of the soc
		 * simple-bus, so the reg cells translate with a single
		 * identity ranges hop: reg = <0x7d542000 0xf00> becomes
		 * 0x10_7d542000.
		 */
		avs_phys = BCM2712_AVS_BASE_PHYS;
		avs_from_fdt = bcm2712_fdt_reg(avs_fdt_paths,
		    "brcm,bcm2711-avs-monitor", 0, &avs_phys, NULL);

		/* Map physical memory using pmap with cache-inhibited attributes */
		sc->avs_vaddr = pmap_mapdev_attr(avs_phys, 0x1000,
		    VM_MEMATTR_DEVICE);

		if (sc->avs_vaddr == NULL) {
			printf("bcm2712: Cannot map memory at 0x%lx\n",
			    (unsigned long)avs_phys);
			mtx_destroy(&sc->thermal_mtx);
			mtx_destroy(&sc->mtx);
			free(sc, M_BCM2712);
			return (ENXIO);
		}

		sc->avs_mapped = 1;
		printf("bcm2712: AVS thermal sensor mapped at 0x%lx (%s)\n",
		    (unsigned long)avs_phys,
		    avs_from_fdt ? "from FDT" : "hardcoded, no FDT node");

		/*
		 * Map RP1 PWM1, via the pcie2 outbound window.  RP1 has two
		 * PWM blocks and the unit address is what tells them apart:
		 * pwm@9c000 is PWM1, which drives the fan, and is the one the
		 * Pi 5 device tree marks status = "okay"; pwm@98000 (PWM0) is
		 * disabled.  Hence located by path, with compatible as a guard.
		 */
		pwm_phys = RP1_PWM1_BASE_PHYS;
		pwm_from_fdt = bcm2712_fdt_rp1(pwm1_fdt_paths,
		    "raspberrypi,rp1-pwm", 0, &pwm_phys, NULL);

		sc->pwm_vaddr = pmap_mapdev_attr(pwm_phys, RP1_PWM_MAP_SIZE,
		    VM_MEMATTR_DEVICE);

		if (sc->pwm_vaddr == NULL) {
			printf("bcm2712: Cannot map PWM memory at 0x%lx\n",
			    (unsigned long)pwm_phys);
			pmap_unmapdev(sc->avs_vaddr, 0x1000);
			sc->avs_vaddr = NULL;
			sc->avs_mapped = 0;
			mtx_destroy(&sc->thermal_mtx);
			mtx_destroy(&sc->mtx);
			free(sc, M_BCM2712);
			return (ENXIO);
		}

		sc->pwm_mapped = 1;
		printf("bcm2712: RP1 PWM1 controller mapped at 0x%lx (%s)\n",
		    (unsigned long)pwm_phys,
		    pwm_from_fdt ? "from FDT" : "hardcoded, no FDT node");

		/*
		 * Enable RP1 PWM1 clock (clock index 18).
		 *
		 * The RP1 PWM clocks are *disabled* at reset; without enabling
		 * them all register writes to the PWM peripheral appear to
		 * succeed but the channel output never changes — the fan
		 * free-runs at boot default (~100% speed from its pull-up).
		 *
		 * Configuration: 50 MHz xosc → ÷8.138 → 6.144 MHz PWM clock.
		 * The Linux DTB specifies assigned-clock-rates = <6144000> for
		 * this peripheral; it yields range=255 ticks at 24 kHz so the
		 * speed register (0-255) maps directly to duty ticks.
		 *
		 * Sequence per clk-rp1.c: set source first (no ENABLE), load
		 * divisors, then set ENABLE.
		 */
		{
			void *clk_map;
			volatile uint32_t *clk;
			bus_addr_t clk_phys = RP1_CLK_BASE_PHYS;
			bool clk_from_fdt;

			clk_from_fdt = bcm2712_fdt_rp1(rp1_clk_fdt_paths,
			    "raspberrypi,rp1-clocks", 0, &clk_phys, NULL);

			clk_map = pmap_mapdev_attr(clk_phys,
			    RP1_CLK_MAP_SIZE, VM_MEMATTR_DEVICE);
			if (clk_map != NULL) {
				clk = (volatile uint32_t *)clk_map;
				/* 1. Select source (aux/xosc), disable first */
				clk[RP1_CLK_PWM1_CTRL / 4] =
				    RP1_CLK_PWM1_CTRL_SRC;
				/* 2. Integer divisor */
				clk[RP1_CLK_PWM1_DIV_INT / 4] =
				    RP1_CLK_PWM1_DIV_INT_VAL;
				/* 3. Fractional divisor */
				clk[RP1_CLK_PWM1_DIV_FRAC / 4] =
				    RP1_CLK_PWM1_DIV_FRAC_VAL;
				/* 4. Enable */
				clk[RP1_CLK_PWM1_CTRL / 4] =
				    RP1_CLK_PWM1_CTRL_ENA;
				printf("bcm2712: RP1 PWM1 clock enabled "
				    "(CTRL=0x%08x DIV_INT=%u) at 0x%lx (%s)\n",
				    clk[RP1_CLK_PWM1_CTRL / 4],
				    clk[RP1_CLK_PWM1_DIV_INT / 4],
				    (unsigned long)clk_phys,
				    clk_from_fdt ? "from FDT" :
				    "hardcoded, no FDT node");
				pmap_unmapdev(clk_map, RP1_CLK_MAP_SIZE);
			} else {
				printf("bcm2712: WARNING: cannot map RP1 "
				    "clock controller at 0x%lx\n",
				    (unsigned long)clk_phys);
			}
		}

		/*
		 * Mux GPIO45 to PWM1 function (ALT0) so the PWM signal
		 * reaches the fan connector.  FreeBSD has no RP1 pinctrl
		 * driver, so we set FUNCSEL directly.  Map, write, unmap.
		 */
		{
			void *gpio_map;
			volatile uint32_t *ctrl_reg;
			uint32_t ctrl_val;
			bus_addr_t gpio_phys = RP1_GPIO_BASE_PHYS;
			bool gpio_from_fdt;

			/*
			 * reg[0] of gpio@d0000 is IO_BANK, which carries the
			 * CTRL registers this uses; reg[1] is SYS_RIO and
			 * reg[2] PADS_BANK (rp1_gpio maps all three).
			 */
			gpio_from_fdt = bcm2712_fdt_rp1(rp1_gpio_fdt_paths,
			    "raspberrypi,rp1-gpio", 0, &gpio_phys, NULL);

			gpio_map = pmap_mapdev_attr(gpio_phys,
			    RP1_GPIO_MAP_SIZE, VM_MEMATTR_DEVICE);
			if (gpio_map != NULL) {
				ctrl_reg = (volatile uint32_t *)
				    ((uintptr_t)gpio_map +
				    RP1_GPIO_CTRL_OFFSET(RP1_GPIO_FAN_PIN));
				ctrl_val = *ctrl_reg;
				printf("bcm2712: GPIO%d CTRL before=0x%x "
				    "(FUNCSEL=%u) via 0x%lx (%s)\n",
				    RP1_GPIO_FAN_PIN, ctrl_val,
				    ctrl_val & RP1_GPIO_FUNCSEL_MASK,
				    (unsigned long)gpio_phys,
				    gpio_from_fdt ? "from FDT" :
				    "hardcoded, no FDT node");
				ctrl_val &= ~RP1_GPIO_FUNCSEL_MASK;
				ctrl_val |= RP1_GPIO_FSEL_ALT0;
				*ctrl_reg = ctrl_val;
				printf("bcm2712: GPIO%d CTRL after=0x%x "
				    "(FUNCSEL=%u, pwm1)\n", RP1_GPIO_FAN_PIN,
				    ctrl_val, ctrl_val & RP1_GPIO_FUNCSEL_MASK);
				pmap_unmapdev(gpio_map, RP1_GPIO_MAP_SIZE);
			} else {
				printf("bcm2712: Cannot map GPIO registers\n");
			}
		}

		/* Initialize PWM channels */
		for (int i = 0; i < BCM2712_PWM_NCHANNELS; i++) {
			sc->channels[i].period = 41566;  /* ~24 kHz */
			sc->channels[i].duty_a = 0;
			sc->channels[i].duty_b = 0;
			sc->channels[i].flags = 0;
			sc->channels[i].enabled = false;
		}

		/* Initialize sysctl context */
		sysctl_ctx_init(&sc->sysctl_ctx);

		/* Create sysctl tree for BCM2712 */
		tree = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
		    SYSCTL_STATIC_CHILDREN(_hw),
		    OID_AUTO, "bcm2712", CTLFLAG_RD | CTLFLAG_MPSAFE,
		    0, "BCM2712 hardware support");
		sc->sysctl_tree = tree;

		if (tree != NULL) {
			/* Add debug flag */
			SYSCTL_ADD_INT(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(tree),
			    OID_AUTO, "debug", CTLFLAG_RW | CTLFLAG_MPSAFE,
			    &bcm2712_debug, 0,
			    "Enable debug messages (0=off, 1=on)");

			/* PWM channel-3 register dump (read-only diagnostic) */
			SYSCTL_ADD_PROC(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(tree),
			    OID_AUTO, "pwm_regs",
			    CTLFLAG_RD | CTLTYPE_STRING | CTLFLAG_MPSAFE,
			    sc, 0, bcm2712_sysctl_pwm_regs, "A",
			    "RP1 PWM ch3 registers: GCTRL CHAN_CTRL RANGE DUTY");

			/* Create thermal subtree */
			thermal_tree = SYSCTL_ADD_NODE(&sc->sysctl_ctx,
			    SYSCTL_CHILDREN(tree),
			    OID_AUTO, "thermal", CTLFLAG_RD | CTLFLAG_MPSAFE,
			    0, "Thermal sensor");

			if (thermal_tree != NULL) {
				/* Add temperature sysctl */
				SYSCTL_ADD_PROC(&sc->sysctl_ctx,
				    SYSCTL_CHILDREN(thermal_tree),
				    OID_AUTO, "cpu_temp", CTLFLAG_RD | CTLTYPE_INT | CTLFLAG_MPSAFE,
				    sc, 0, bcm2712_thermal_sysctl_temp, "IK",
				    "CPU temperature in deciKelvin");

				/*
				 * Sensor health.  A board that overheats while
				 * cpu_temp looks fine is the failure these
				 * exist to make visible.
				 */
				SYSCTL_ADD_BOOL(&sc->sysctl_ctx,
				    SYSCTL_CHILDREN(thermal_tree),
				    OID_AUTO, "healthy",
				    CTLFLAG_RD | CTLFLAG_MPSAFE,
				    &sc->thermal_healthy, 0,
				    "Temperature is trustworthy");
				SYSCTL_ADD_UINT(&sc->sysctl_ctx,
				    SYSCTL_CHILDREN(thermal_tree),
				    OID_AUTO, "invalid_total",
				    CTLFLAG_RD | CTLFLAG_MPSAFE,
				    &sc->thermal_invalid_total, 0,
				    "Ticks the sensor produced no valid reading");
				SYSCTL_ADD_UINT(&sc->sysctl_ctx,
				    SYSCTL_CHILDREN(thermal_tree),
				    OID_AUTO, "invalid_run",
				    CTLFLAG_RD | CTLFLAG_MPSAFE,
				    &sc->thermal_invalid_run, 0,
				    "Consecutive such ticks (age of cpu_temp)");
				SYSCTL_ADD_INT(&sc->sysctl_ctx,
				    SYSCTL_CHILDREN(thermal_tree),
				    OID_AUTO, "fail_inject",
				    CTLFLAG_RW | CTLFLAG_MPSAFE,
				    &bcm2712_thermal_fail_inject, 0,
				    "Simulate a silent sensor, to test the fan "
				    "fail-safe (errs towards cooling)");
			}
		}

		/* Start periodic temperature updates */
		mtx_lock(&sc->thermal_mtx);
		callout_reset(&sc->thermal_callout, hz, bcm2712_thermal_update, sc);
		mtx_unlock(&sc->thermal_mtx);

		/* Store global reference */
		bcm2712_sc = sc;

		printf("bcm2712: Thermal sensor initialized\n");
		break;

	case MOD_UNLOAD:
		sc = bcm2712_sc;
		if (sc == NULL)
			break;

		bcm2712_sc = NULL;

		/* Stop periodic updates */
		mtx_lock(&sc->thermal_mtx);
		callout_drain(&sc->thermal_callout);
		mtx_unlock(&sc->thermal_mtx);

		/* Clean up sysctl tree */
		sysctl_ctx_free(&sc->sysctl_ctx);

		/* Unmap memory */
		if (sc->pwm_vaddr != NULL) {
			pmap_unmapdev(sc->pwm_vaddr, RP1_PWM_MAP_SIZE);
			sc->pwm_vaddr = NULL;
			sc->pwm_mapped = 0;
		}

		if (sc->avs_vaddr != NULL) {
			pmap_unmapdev(sc->avs_vaddr, 0x1000);
			sc->avs_vaddr = NULL;
			sc->avs_mapped = 0;
		}

		/* Clean up mutexes */
		mtx_destroy(&sc->thermal_mtx);
		mtx_destroy(&sc->mtx);

		free(sc, M_BCM2712);
		printf("bcm2712: Thermal sensor unloaded\n");
		break;

	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t bcm2712_mdata = {
	"bcm2712",
	bcm2712_modevent,
	NULL
};

DECLARE_MODULE(bcm2712, bcm2712_mdata, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(bcm2712, 1);