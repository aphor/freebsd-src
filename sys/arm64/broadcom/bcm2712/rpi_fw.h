/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Jeremy McMillan
 */

/*
 * rpi_fw -- the Raspberry Pi 5 VPU property mailbox, for other drivers.
 */

#ifndef _ARM64_BROADCOM_BCM2712_RPI_FW_H_
#define	_ARM64_BROADCOM_BCM2712_RPI_FW_H_

/*
 * Send one property tag to the VPU firmware and wait for its reply.
 *
 * val is the tag's value buffer and vallen its size in bytes; the buffer
 * must hold vallen rounded up to a multiple of 4.  The first inlen bytes
 * of val are the request.  On success the firmware's reply replaces the
 * contents of val.
 *
 * Calls are serialised against every other user of the mailbox.  The call
 * holds a sleep mutex while it polls the mailbox, so it is made from thread
 * context, never from an interrupt filter or with a spin lock held.
 *
 * Returns 0 on success, or:
 *	ENXIO		rpi_fw has not attached
 *	EINVAL		inlen exceeds vallen, or the buffer is too large
 *	ETIMEDOUT	the firmware did not answer
 *	EIO		the firmware's reply code was not success
 *	EOPNOTSUPP	the firmware did not mark the tag as answered
 *
 * A caller declares MODULE_DEPEND(<module>, rpi_fw, 1, 1, 1).
 */
int	rpi_fw_property(uint32_t tag, uint32_t *val, uint32_t vallen,
	    uint32_t inlen);

#endif /* _ARM64_BROADCOM_BCM2712_RPI_FW_H_ */
