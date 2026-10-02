/*-
 * rpi_sdhci.h -- SD card block reads for the Raspberry Pi 5 loader.
 *
 * Scope is deliberately narrow: identify the card in the boot slot and read
 * 512-byte blocks from it.  No writes, no DMA, no UHS speeds, no eMMC, no
 * command queueing, no interrupts.  A loader needs to find and read a kernel;
 * everything else is the kernel's job.
 *
 * This is rpi5_modules.git loader/sdhci.[ch], which read blocks on hardware
 * as a bare-metal test image on 2026-09-20 (doc/LOADER_ZIMAGE.md, "SD block
 * reads work"), with multi-block reads added.  rpi_sd.c makes a loader disk
 * of it.
 *
 * WHICH CONTROLLER
 *
 * The SD slot is on the BCM2712's own SDHCI, in the firmware's device tree:
 *
 *	mmc@fff000 {
 *		compatible = "brcm,bcm2712-sdhci", "brcm,sdhci-brcmstb";
 *		reg = <0xfff000 0x260  0xfff400 0x200>;
 *		reg-names = "host", "cfg";
 *		clocks = <&clk_emmc2>;
 *		bus-width = <0x04>;
 *	};
 *
 * translated through the soc bus (child + 0x1000000000):
 *
 *	host = 0x1000fff000 size 0x260
 *	cfg  = 0x1000fff400 size 0x200
 *
 * The kernel's sdhci_bcm27120 reports the same host window, and sdda0
 * attaches to its slot.  The other controller, mmc@1100000, is the SDIO WiFi.
 *
 * BASE CLOCK
 *
 * clk-emmc2 is a fixed-clock of 200 MHz, and the firmware's boot log says the
 * same ("SD HOST: 200000000 ... BUS: 50000000 Hz ... div: 4 (2)"), which also
 * checks the divisor arithmetic in sdhci_set_clock().
 *
 * THE cfg WINDOW
 *
 * The vendor Linux driver's cfginit for 2712 (sdhci-brcmstb.c) is for tuned
 * UHS rates, forced card presence and command queueing, none of which this
 * driver uses.  One cfg register matters: SDIO_CFG_SD_PIN_SEL chooses SD
 * versus MMC pin behaviour, so the clock code sets it to SD.
 *
 * PIO, NOT DMA
 *
 * Reading through SDHCI_BUFFER needs no cache maintenance and no DMA
 * addresses, and costs nothing a loader cares about.
 */
#ifndef	_RPI_SDHCI_H_
#define	_RPI_SDHCI_H_

#include <stdint.h>

/* Translated physical addresses; see the header comment. */
#define	SDHCI_RPI5_HOST_BASE	0x1000fff000UL
#define	SDHCI_RPI5_CFG_BASE	0x1000fff400UL
#define	SDHCI_RPI5_BASE_CLOCK	200000000U	/* clk-emmc2 */

/* Register byte offsets, from sys/dev/sdhci/sdhci.h. */
#define	SDHCI_DMA_ADDRESS	0x00
#define	SDHCI_BLOCK_SIZE	0x04
#define	SDHCI_BLOCK_COUNT	0x06
#define	SDHCI_ARGUMENT		0x08
#define	SDHCI_TRANSFER_MODE	0x0c
#define	SDHCI_COMMAND_FLAGS	0x0e
#define	SDHCI_COMMAND		0x0f
#define	SDHCI_RESPONSE		0x10	/* 0x10..0x1f, four 32-bit words */
#define	SDHCI_BUFFER		0x20	/* PIO data port */
#define	SDHCI_PRESENT_STATE	0x24
#define	SDHCI_HOST_CONTROL	0x28
#define	SDHCI_POWER_CONTROL	0x29
#define	SDHCI_CLOCK_CONTROL	0x2c
#define	SDHCI_TIMEOUT_CONTROL	0x2e
#define	SDHCI_SOFTWARE_RESET	0x2f
#define	SDHCI_INT_STATUS	0x30
#define	SDHCI_INT_ENABLE	0x34
#define	SDHCI_SIGNAL_ENABLE	0x38
#define	SDHCI_ACMD12_ERR	0x3c
#define	SDHCI_HOST_CONTROL2	0x3e
#define	SDHCI_CAPABILITIES	0x40
#define	SDHCI_CAPABILITIES2	0x44
#define	SDHCI_HOST_VERSION	0xfe

/* TRANSFER_MODE */
#define	SDHCI_TRNS_DMA		0x01
#define	SDHCI_TRNS_BLK_CNT_EN	0x02
#define	SDHCI_TRNS_ACMD12	0x04
#define	SDHCI_TRNS_READ		0x10
#define	SDHCI_TRNS_MULTI	0x20

/* COMMAND_FLAGS */
#define	SDHCI_CMD_RESP_NONE	0x00
#define	SDHCI_CMD_RESP_LONG	0x01
#define	SDHCI_CMD_RESP_SHORT	0x02
#define	SDHCI_CMD_RESP_SHORT_BUSY 0x03
#define	SDHCI_CMD_CRC		0x08
#define	SDHCI_CMD_INDEX		0x10
#define	SDHCI_CMD_DATA		0x20
#define	SDHCI_CMD_TYPE_ABORT	0xc0

/* PRESENT_STATE */
#define	SDHCI_CMD_INHIBIT	0x00000001
#define	SDHCI_DAT_INHIBIT	0x00000002
#define	SDHCI_DAT_ACTIVE	0x00000004
#define	SDHCI_DATA_AVAILABLE	0x00000800
#define	SDHCI_CARD_PRESENT	0x00010000
#define	SDHCI_CARD_STABLE	0x00020000

/* HOST_CONTROL */
#define	SDHCI_CTRL_4BITBUS	0x02
#define	SDHCI_CTRL_HISPD	0x04

/* POWER_CONTROL */
#define	SDHCI_POWER_ON		0x01
#define	SDHCI_POWER_330		0x0e

/* CLOCK_CONTROL */
#define	SDHCI_DIVIDER_SHIFT	8
#define	SDHCI_DIVIDER_HI_SHIFT	6
#define	SDHCI_CLOCK_CARD_EN	0x0004
#define	SDHCI_CLOCK_INT_STABLE	0x0002
#define	SDHCI_CLOCK_INT_EN	0x0001

/* SOFTWARE_RESET */
#define	SDHCI_RESET_ALL		0x01
#define	SDHCI_RESET_CMD		0x02
#define	SDHCI_RESET_DATA	0x04

/* INT_STATUS */
#define	SDHCI_INT_RESPONSE	0x00000001
#define	SDHCI_INT_DATA_END	0x00000002
#define	SDHCI_INT_DATA_AVAIL	0x00000020
#define	SDHCI_INT_ERROR		0x00008000

/* The one cfg register this driver writes; see the header comment. */
#define	SDIO_CFG_SD_PIN_SEL		0x44
#define	SDIO_CFG_SD_PIN_SEL_MASK	0x3
#define	SDIO_CFG_SD_PIN_SEL_SD		0x2

/* SD commands used here. */
#define	SD_CMD_GO_IDLE			0
#define	SD_CMD_ALL_SEND_CID		2
#define	SD_CMD_SEND_REL_ADDR		3
#define	SD_CMD_SELECT_CARD		7
#define	SD_CMD_SEND_IF_COND		8
#define	SD_CMD_SEND_CSD			9
#define	SD_CMD_STOP_TRANSMISSION	12
#define	SD_CMD_SET_BLOCKLEN		16
#define	SD_CMD_READ_SINGLE_BLOCK	17
#define	SD_CMD_READ_MULTIPLE_BLOCK	18
#define	SD_CMD_APP_CMD			55
#define	SD_ACMD_SET_BUS_WIDTH		6
#define	SD_ACMD_SD_SEND_OP_COND		41

#define	SD_BLOCK_SIZE			512
#define	SD_MAX_MULTI			256	/* blocks per CMD18: 128 KiB */

/* OCR bits. */
#define	SD_OCR_BUSY			0x80000000U	/* 0 = still init */
#define	SD_OCR_CCS			0x40000000U	/* 1 = block addressed */

struct sdhci {
	uint64_t	host;		/* host register window	*/
	uint64_t	cfg;		/* brcmstb cfg window	*/
	uint32_t	base_clock;	/* Hz			*/
	uint32_t	ocr;		/* from ACMD41		*/
	uint16_t	rca;		/* from CMD3		*/
	uint8_t		block_addressed;/* OCR CCS		*/
	uint8_t		no_multi;	/* CMD18 given up on	*/
	uint32_t	cid[4];		/* raw CMD2 response	*/
	uint32_t	csd[4];		/* raw CMD9 response	*/
	uint32_t	last_int;	/* INT_STATUS at last error */

	/* What the reads cost, for "sdinfo" and the line before the kernel. */
	uint64_t	rd_blocks;
	uint64_t	rd_ticks;	/* of the generic timer	*/
	uint32_t	rd_cmds;
	uint32_t	multi_errors;	/* CMD18 transfers that failed	*/
	uint32_t	single_errors;	/* CMD17 retries		*/
};

void	sdhci_attach(struct sdhci *sc, uint64_t host, uint64_t cfg,
	    uint32_t base_clock);
uint32_t sdhci_read32(const struct sdhci *sc, uint32_t reg);
uint16_t sdhci_read16(const struct sdhci *sc, uint32_t reg);
uint8_t	 sdhci_read8(const struct sdhci *sc, uint32_t reg);

/*
 * Full card initialisation from reset: CMD0, CMD8, ACMD41 until ready, CMD2,
 * CMD3, CMD9, CMD7, 4-bit bus, 512-byte blocks, 25 MHz.  Returns 0 on
 * success.  Every wait is bounded; with no card it fails in about a second.
 */
int	sd_init(struct sdhci *sc);

/*
 * Read count 512-byte blocks.  CMD18 with auto CMD12, SD_MAX_MULTI blocks at
 * a time; a transfer that fails is recovered from and read again block by
 * block with CMD17.  buf needs no alignment.  Returns 0 on success, -1
 * otherwise (sc->last_int holds INT_STATUS).
 */
int	sd_read_blocks(struct sdhci *sc, uint64_t lba, uint32_t count,
	    void *buf);

/* The card's size in 512-byte blocks, from its CSD; 0 if unknown. */
uint64_t sd_capacity_blocks(const struct sdhci *sc);

/* Extract the 5-character product name from a CMD2 response. */
void	sd_cid_product(const struct sdhci *sc, char out[6]);

/* The generic timer the read statistics are in. */
uint64_t sd_timer_hz(void);

#endif	/* _RPI_SDHCI_H_ */
