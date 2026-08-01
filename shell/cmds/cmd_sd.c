/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_sd.c
 * @brief   `sd` shell command: microSD card over SDMMC1 (issue #6).
 *
 *   sd info        card type, capacity, block geometry, bus width, clock, CID/CSD
 *   sd read <lba>  hexdump one 512 B block (LBA addressing)
 *
 * Phase A of issue #6: the raw block layer only.  The FileX filesystem
 * subcommands (ls/cat/write/rm/mkdir/df/umount/format) land in Phase B on top of
 * the shared fs_cmd_core bodies, at which point `info`/`read` also pick up the
 * ownership gate that keeps them from re-probing a mounted card.
 *
 * Both commands probe on demand: this board has no card-detect line, so the only
 * way to learn whether a card is in the slot is to talk to it (see sd_card.h).
 *
 * Clean-room design; ported from the STM32F746 Discovery firmware's cmd_sd.c.
 */
#include "cli.h"
#include "sd_card.h"

#include <stdint.h>

static const char *sd_strerror(int rc)
{
	switch (rc) {
	case SD_ERR_PARAM:   return "bad argument";
	case SD_ERR_HAL:     return "SDMMC transfer failed";
	case SD_ERR_TIMEOUT: return "card not ready / transfer timed out";
	case SD_ERR_STATE:   return "driver not initialized / card not probed";
	case SD_ERR_NO_CARD: return "no card in slot";
	default:             return "unknown error";
	}
}

/*
 * Make sure a card is identified and answering.  Cheap when it already is
 * (sd_card_status is one CMD13); a full re-identification otherwise, which is
 * also how a freshly inserted or swapped card is picked up.
 */
static int sd_ensure_ready(struct cli_instance *sh)
{
	int rc;

	if (sd_card_status() == SD_OK)
		return 0;

	rc = sd_card_probe();
	if (rc != 0) {
		cli_error(sh, "sd: %s\r\n", sd_strerror(rc));
		return -1;
	}
	return 0;
}

static int cmd_sd_info(struct cli_instance *sh, int argc, char **argv)
{
	const struct sd_card_info *ci;

	(void)argc;
	(void)argv;

	if (sd_ensure_ready(sh) != 0)
		return 1;

	ci = sd_card_get_info();
	cli_print(sh, "type      : %s (v%s)\r\n",
	          (ci->type == 1u) ? "SDHC/SDXC" : "SDSC",
	          (ci->version == 1u) ? "2.x" : "1.x");
	cli_print(sh, "capacity  : %lu MiB (%lu blocks x %lu B)\r\n",
	          (unsigned long)(ci->capacity_bytes / (1024u * 1024u)),
	          (unsigned long)ci->block_count,
	          (unsigned long)ci->block_size);
	cli_print(sh, "bus width : %lu-bit\r\n", (unsigned long)ci->bus_width);
	cli_print(sh, "clock     : %lu kHz\r\n", (unsigned long)(ci->clock_hz / 1000u));
	cli_print(sh, "rca       : 0x%04lx\r\n", (unsigned long)ci->rca);
	cli_print(sh, "ccc       : 0x%03lx\r\n", (unsigned long)ci->card_class);
	cli_print(sh, "cid       : %08lx %08lx %08lx %08lx\r\n",
	          (unsigned long)ci->cid[0], (unsigned long)ci->cid[1],
	          (unsigned long)ci->cid[2], (unsigned long)ci->cid[3]);
	cli_print(sh, "csd       : %08lx %08lx %08lx %08lx\r\n",
	          (unsigned long)ci->csd[0], (unsigned long)ci->csd[1],
	          (unsigned long)ci->csd[2], (unsigned long)ci->csd[3]);
	return 0;
}

static int cmd_sd_read(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t buf[SD_BLOCK_SIZE];
	uint32_t lba;
	int rc;

	(void)argc;

	/* argv[0] = "read", argv[1] = LBA. */
	if (cli_parse_u32(argv[1], &lba) != 0) {
		cli_error(sh, "sd: bad LBA '%s'\r\n", argv[1]);
		return 1;
	}

	if (sd_ensure_ready(sh) != 0)
		return 1;

	rc = sd_card_read_blocks(lba, buf, 1);
	if (rc != 0) {
		cli_error(sh, "sd: read failed: %s\r\n", sd_strerror(rc));
		return 1;
	}

	return cli_hexdump_base(sh, buf, sizeof buf,
	                        (unsigned long long)lba * (unsigned long long)SD_BLOCK_SIZE)
	       == 0 ? 0 : 1;
}

CLI_SUBCMD_SET_CREATE(sd_subcmds,
	CLI_CMD_ARG(info, NULL, "card type/capacity/geometry/CID/CSD", cmd_sd_info, 1, 0),
	CLI_CMD_ARG(read, NULL, "hexdump one 512 B block <lba>",       cmd_sd_read, 2, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(sd, sd_subcmds,
                 "microSD card (SDMMC1) info/read", NULL, 1, 0);
