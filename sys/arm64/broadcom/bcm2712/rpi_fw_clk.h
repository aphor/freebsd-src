/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * Written from a functional specification of the Raspberry Pi firmware
 * clock interface.
 */

#ifndef _ARM64_BROADCOM_BCM2712_RPI_FW_CLK_H_
#define	_ARM64_BROADCOM_BCM2712_RPI_FW_CLK_H_

#include <dev/clk/clk.h>

/*
 * The name of the clock this provider registers for the V3D GPU, firmware
 * clock 5, for a consumer whose device-tree node does not name its clock.
 */
#define	RPI_FW_CLK_NAME_V3D	"rpi_fw_v3d"

/* Bits of the firmware's clock state word. */
#define	RPI_FW_CLK_STATE_ON	0x00000001	/* the clock is running */
#define	RPI_FW_CLK_STATE_ABSENT	0x00000002	/* no such clock */

/*
 * Ask the firmware, now, for the state word and the rate in Hz of a clock
 * obtained from the "raspberrypi,firmware-clocks" provider.  Neither value
 * comes from a cache: clk_get_freq() may return a rate the clock framework
 * saved earlier, while the firmware changes its clocks on its own.
 *
 * Returns ENXIO if 'clk' is not one of this provider's clocks, or the
 * mailbox error.  Sleeps: call from thread context.
 */
int	rpi_fw_clk_query(clk_t clk, uint32_t *state, uint32_t *rate);

/*
 * Ask the firmware, now, for the highest rate in Hz it allows the clock.
 * Same errors as rpi_fw_clk_query().
 */
int	rpi_fw_clk_max_rate(clk_t clk, uint32_t *rate);

#endif /* _ARM64_BROADCOM_BCM2712_RPI_FW_CLK_H_ */
