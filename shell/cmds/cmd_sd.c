/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_sd.c
 * @brief   `sd` shell command: microSD card + FileX filesystem (issue #6).
 *
 *   sd info            card type, capacity, block geometry, bus width, CID/CSD
 *   sd read <lba>      hexdump one 512 B block (LBA addressing)
 *   sd format yes      format the whole card as a FAT32 superfloppy (destructive)
 *   sd ls [path]       list a directory (default /)
 *   sd cat <path>      print a file
 *   sd write <p> <txt> create/overwrite a file
 *   sd rm <path>       delete a file or empty directory
 *   sd mkdir <path>    create a directory
 *   sd df              filesystem capacity / free / FAT type
 *   sd umount          flush + unmount
 *
 * The filesystem subcommands are thin binders over the media-independent bodies in
 * fs_cmd_core.c, parameterized by sd_dev.  The low-level `info`/`read` may re-probe
 * the card (HAL_SD_DeInit + re-identify), which would corrupt a mounted FX_MEDIA
 * underneath it, so they take the raw ownership slot -- refused while the
 * filesystem is mounted, with `sd umount` as the way out.
 *
 * The card is probed lazily throughout: this board has no card-detect line, so the
 * only way to learn whether a card is in the slot is to talk to it (see sd_card.h).
 *
 * Clean-room design; ported from the STM32F746 Discovery firmware's cmd_sd.c.
 */
#include "cli.h"
#include "sd_card.h"
#include "fs_cmd_core.h"
#include "sd_fs_glue.h"
#include "fx_sd_driver.h"

#include <stdint.h>
#include <string.h>

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

/* ---- low-level card commands (raw-gated) --------------------------------- */

/*
 * Take the raw slot for a card-disruptive op.  Refused while the filesystem is
 * mounted or busy, so a re-probe (HAL_SD_DeInit) can never run under a live
 * FX_MEDIA.
 */
static int sd_raw_gate(struct cli_instance *sh)
{
	if (sd_raw_begin() != FX_SUCCESS) {
		cli_error(sh, "sd: filesystem mounted or busy; run `sd umount` first\r\n");
		return -1;
	}
	return 0;
}

/* Identify the card if it is not already usable.  Call under the raw slot. */
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

	if (sd_raw_gate(sh) != 0)
		return 1;
	if (sd_ensure_ready(sh) != 0) {
		sd_raw_end();
		return 1;
	}

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
	sd_raw_end();
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

	if (sd_raw_gate(sh) != 0)
		return 1;
	if (sd_ensure_ready(sh) != 0) {
		sd_raw_end();
		return 1;
	}

	rc = sd_card_read_blocks(lba, buf, 1);
	sd_raw_end();        /* data is in buf; release before the hexdump */
	if (rc != 0) {
		cli_error(sh, "sd: read failed: %s\r\n", sd_strerror(rc));
		return 1;
	}

	return cli_hexdump_base(sh, buf, sizeof buf,
	                        (unsigned long long)lba * (unsigned long long)SD_BLOCK_SIZE)
	       == 0 ? 0 : 1;
}

/* ---- filesystem commands (shared core, bound to sd_dev) ------------------- */

static const struct fs_device sd_dev = {
	.name       = "sd",
	.mount_hint = "no FAT filesystem? insert a FAT-formatted card or run `sd format yes`",
	.acquire    = sd_media_acquire,
	.unmount    = sd_media_unmount,
	.is_mounted = sd_is_mounted,
	.op_begin   = sd_op_begin,        .op_end     = sd_op_end,
	.excl_begin = sd_exclusive_begin, .excl_end   = sd_exclusive_end,
	.dir_lock   = sd_dir_lock,        .dir_unlock = sd_dir_unlock,
	.info_extra = NULL,
};

/* Accessor for cross-command reuse (fs_core_read_file consumers). */
const struct fs_device *fs_sd_device(void)
{
	return &sd_dev;
}

static int cmd_sd_ls(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_ls(&sd_dev, sh, argc, argv);
}

static int cmd_sd_cat(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_cat(&sd_dev, sh, argc, argv);
}

static int cmd_sd_write(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_write(&sd_dev, sh, argc, argv);
}

static int cmd_sd_rm(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_rm(&sd_dev, sh, argc, argv);
}

static int cmd_sd_mkdir(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_mkdir(&sd_dev, sh, argc, argv);
}

static int cmd_sd_df(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_info(&sd_dev, sh, argc, argv);
}

static int cmd_sd_umount(struct cli_instance *sh, int argc, char **argv)
{
	return fs_core_umount(&sd_dev, sh, argc, argv);
}

/* ---- format --------------------------------------------------------------- */

/*
 * Pick sectors_per_cluster for a whole-card FAT32 format: a power of two that caps
 * the cluster count (which bounds the FAT size and the format time) while keeping
 * the count comfortably above the FAT32 minimum of 65525, so FileX's own post-FAT
 * recalculation still lands on FAT32.  Returns -1 if the card is too small.
 */
static int fat32_spc(uint32_t total, uint32_t *out_spc)
{
	uint32_t spc = 1;

	while ((total / spc) > 2097152u && spc < 64u)   /* cap ~2M clusters */
		spc <<= 1;
	if ((total / spc) < 80000u)                      /* FAT32 min + margin */
		return -1;
	*out_spc = spc;
	return 0;
}

static int cmd_sd_format(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t total, spc = 0;
	UINT status;
	FX_MEDIA *media = FX_NULL;
	int is_fat32 = 0;
	int rc;

	/* `sd format yes` -- the literal `yes` is the safety latch. */
	if (argc != 2 || strcmp(argv[1], "yes") != 0) {
		cli_error(sh, "sd: destructive; confirm with `sd format yes`\r\n");
		return 1;
	}

	/* Exclusive ownership for the whole format (refused while any op runs). */
	if (sd_exclusive_begin() != FX_SUCCESS) {
		cli_error(sh, "sd: %s\r\n", fs_strerror(FS_ERR_BUSY));
		return 1;
	}

	(void)sd_media_unmount();           /* flush + close any mounted FS */

	/* block_count is only valid after a probe -- get a real capacity before
	 * fx_media_format uses it as total_sectors. */
	rc = sd_card_probe();
	if (rc != SD_OK) {
		cli_error(sh, "sd: %s\r\n", sd_strerror(rc));
		sd_exclusive_end();
		return 1;
	}
	total = sd_card_get_info()->block_count;
	if (total == 0u || fat32_spc(total, &spc) != 0) {
		cli_error(sh, "sd: card too small for FAT32\r\n");
		sd_exclusive_end();
		return 1;
	}

	cli_print(sh, "formatting %lu MiB as FAT32 (%lu B clusters)...\r\n",
	          (unsigned long)(total / 2048u), (unsigned long)(spc * 512u));

	/* Format the whole card as a superfloppy: format mode makes the driver's INIT
	 * skip partition detection, which a blank card would fail.  num_fats=2,
	 * dir_entries=0 (a FAT32 root is a cluster chain, not a fixed table). */
	fx_sd_set_format_mode(1);
	status = fx_media_format(sd_glue_media(), fx_sd_driver, FX_NULL,
	                         sd_glue_cache(), sd_glue_cache_size(),
	                         "SD", 2, 0, 0, total, SD_BLOCK_SIZE, spc, 255, 63);
	fx_sd_set_format_mode(0);

	if (status != FX_SUCCESS) {
		sd_exclusive_end();
		cli_error(sh, "sd: format failed: %s (0x%02x)\r\n",
		          fs_strerror(status), status);
		return 1;
	}

	/*
	 * Remount WITHOUT dropping the exclusive slot first.  This firmware runs two
	 * shells (USB CDC and telnet) plus background jobs, so releasing here and then
	 * calling the normal mount path would leave a window in which another command
	 * takes ownership and the remount fails with FS_ERR_BUSY -- reporting failure
	 * for a format that actually succeeded.  sd_media_mount_exclusive() is the entry
	 * point that mounts while the slot is still held.
	 *
	 * The FAT32 check is made before releasing too, so the answer describes the
	 * media this command produced and not one a concurrent umount has since closed.
	 */
	status = sd_media_mount_exclusive(&media);
	if (status == FX_SUCCESS)
		is_fat32 = (media->fx_media_32_bit_FAT == FX_TRUE);
	sd_exclusive_end();

	if (status != FX_SUCCESS) {
		cli_error(sh, "sd: mount after format failed: %s (0x%02x)\r\n",
		          fs_strerror(status), status);
		return 1;
	}
	/* The command contract is "FAT32 superfloppy", so a FAT16 fallback is a
	 * failure, not a warning. */
	if (!is_fat32) {
		cli_error(sh, "sd: format did not produce FAT32 (card too small)\r\n");
		return 1;
	}
	cli_print(sh, "formatted and mounted (FAT32)\r\n");
	return 0;
}

CLI_SUBCMD_SET_CREATE(sd_subcmds,
	CLI_CMD_ARG(info,   NULL, "card type/capacity/geometry/CID/CSD",   cmd_sd_info,   1, 0),
	CLI_CMD_ARG(read,   NULL, "hexdump one 512 B block <lba>",         cmd_sd_read,   2, 0),
	CLI_CMD_ARG(format, NULL, "format as FAT32 -- confirm with 'yes'", cmd_sd_format, 1, 1),
	CLI_CMD_ARG(ls,     NULL, "list directory [path]",                 cmd_sd_ls,     1, 1),
	CLI_CMD_ARG(cat,    NULL, "print file <path>",                     cmd_sd_cat,    2, 0),
	CLI_CMD_ARG(write,  NULL, "write <path> <text>",                   cmd_sd_write,  3, 0),
	CLI_CMD_ARG(rm,     NULL, "delete file/empty dir <path>",          cmd_sd_rm,     2, 0),
	CLI_CMD_ARG(mkdir,  NULL, "create directory <path>",               cmd_sd_mkdir,  2, 0),
	CLI_CMD_ARG(df,     NULL, "filesystem capacity / free (FAT)",      cmd_sd_df,     1, 0),
	CLI_CMD_ARG(umount, NULL, "flush and unmount",                     cmd_sd_umount, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(sd, sd_subcmds,
                 "microSD card (SDMMC1) + FAT filesystem", NULL, 1, 0);
