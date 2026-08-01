/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 *
 * @file    fx_sd_driver.c
 * @brief   FileX media driver on the SDMMC1 block API (issue #6).
 *
 * See fx_sd_driver.h.  FileX's fx_media_open() always reads logical sector 0 for
 * the boot record and never calls _fx_partition_offset_calculate(), so an
 * MBR-partitioned card would otherwise fail to mount.  This driver detects the
 * layout at INIT (superfloppy VBR @ LBA 0, or MBR with partition 0) and adds the
 * partition start to every physical access, bounding all I/O to the partition.
 *
 * Clean-room rewrite; the FileX driver request set and the MBR/BPB field layout
 * were used as a reference only (no code reused).
 */
#include "fx_sd_driver.h"
#include "sd_card.h"

#include <stdint.h>

#define LOG_TAG "fxsd"
#include "log.h"

/* Partition window for the current mount (recomputed at INIT for hot-plug):
 * physical LBA = sd_part_lba + logical, valid for logical < sd_part_blocks. */
static uint32_t sd_part_lba;
static uint32_t sd_part_blocks;

/* When set, the next INIT skips partition detection and uses the whole card as
 * a superfloppy (for fx_media_format of a blank card).  Set/cleared by the `sd`
 * command under the exclusive ownership slot. */
static volatile int sd_format_mode;

/* LBA 0 scratch for partition detection (INIT only). sd_card_read_blocks()
 * bounces through its own aligned DMA buffer, so any alignment is fine. */
static uint8_t mbr_buf[SD_BLOCK_SIZE];

/* Result of the probe done by the last FX_DRIVER_INIT; see fx_sd_driver.h. */
static int last_probe_rc;

int fx_sd_last_probe_rc(void)
{
	return last_probe_rc;
}

void fx_sd_set_format_mode(int on)
{
	sd_format_mode = on;
}

static uint16_t rd_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* FAT partition types we accept in an MBR entry. */
static int is_fat_part_type(uint8_t t)
{
	return t == 0x01u || t == 0x04u || t == 0x06u ||
	       t == 0x0Bu || t == 0x0Cu || t == 0x0Eu;
}

/* Strong VBR sanity: a FAT boot sector starts with a jump and carries a
 * plausible BPB.  Avoids mistaking MBR boot code for a VBR. */
static int bpb_sane(const uint8_t *b)
{
	uint8_t  spc = b[13];                 /* sectors per cluster */

	if (rd_le16(b + 11) != SD_BLOCK_SIZE) /* bytes per sector */
		return 0;
	if (spc == 0u || (spc & (uint8_t)(spc - 1u)) != 0u)  /* power of two */
		return 0;
	if (rd_le16(b + 14) == 0u)            /* reserved sectors */
		return 0;
	if (b[16] != 1u && b[16] != 2u)       /* number of FATs */
		return 0;
	if (rd_le16(b + 22) == 0u && rd_le32(b + 36) == 0u)  /* FAT size 16/32 */
		return 0;
	if (rd_le16(b + 19) == 0u && rd_le32(b + 32) == 0u)  /* total sectors 16/32 */
		return 0;
	return 1;
}

/*
 * Read LBA 0 and decide the volume layout.  Returns 0 and fills
 * sd_part_lba/sd_part_blocks on success, nonzero (and zeroes them) when no
 * usable FAT layout is found.  All range checks use subtraction form so a
 * corrupt MBR cannot pass via 32-bit wrap.
 */
static int detect_partition(void)
{
	const struct sd_card_info *ci = sd_card_get_info();
	uint32_t block_count = ci->block_count;
	const uint8_t *b = mbr_buf;

	sd_part_lba = 0;
	sd_part_blocks = 0;

	if (block_count == 0u)
		return -1;
	if (sd_card_read_blocks(0, mbr_buf, 1) != SD_OK)
		return -1;
	if (b[510] != 0x55u || b[511] != 0xAAu)
		return -1;

	/* Superfloppy: VBR directly at LBA 0. */
	if (((b[0] == 0xEBu && b[2] == 0x90u) || b[0] == 0xE9u) && bpb_sane(b)) {
		sd_part_lba = 0;
		sd_part_blocks = block_count;
		LOG_INF("superfloppy (VBR@0), %lu blocks",
		        (unsigned long)block_count);
		return 0;
	}

	/* MBR: take partition entry 0 (offset 446, 16-byte entries). */
	{
		const uint8_t *e = b + 446;
		uint32_t first = rd_le32(e + 8);
		uint32_t size  = rd_le32(e + 12);

		if (is_fat_part_type(e[4]) && first != 0u && size != 0u &&
		    first < block_count && size <= block_count - first) {
			sd_part_lba = first;
			sd_part_blocks = size;
			LOG_INF("MBR part0 @%lu, %lu blocks",
			        (unsigned long)first, (unsigned long)size);
			return 0;
		}
	}

	LOG_WRN("no FAT VBR/MBR at LBA0");
	return -1;
}

/* Overflow-safe bound: logical..logical+count within the partition and the
 * physical addition does not wrap. */
static int bounds_ok(uint32_t logical, uint32_t count)
{
	if (count == 0u)
		return 0;
	if (logical >= sd_part_blocks)
		return 0;
	if (count > sd_part_blocks - logical)
		return 0;
	if (logical > 0xFFFFFFFFu - sd_part_lba)
		return 0;
	return 1;
}

VOID fx_sd_driver(FX_MEDIA *media_ptr)
{
	UCHAR *buffer = (UCHAR *)media_ptr->fx_media_driver_buffer;
	uint32_t sector = (uint32_t)media_ptr->fx_media_driver_logical_sector;
	uint32_t count  = (uint32_t)media_ptr->fx_media_driver_sectors;

	media_ptr->fx_media_driver_status = FX_IO_ERROR;

	switch (media_ptr->fx_media_driver_request) {
	case FX_DRIVER_INIT:
		/* SD has an internal FTL: no RELEASE_SECTORS forwarding needed. */
		media_ptr->fx_media_driver_free_sector_update = FX_FALSE;
		/* Overwritten unconditionally, before any early return: a stale
		 * SD_ERR_NO_CARD from an earlier mount attempt must never survive to
		 * make the glue misreport a later non-FAT card as an empty slot. */
		last_probe_rc = sd_card_probe();
		if (last_probe_rc != SD_OK)
			return;
		if (sd_format_mode) {
			/* Formatting: write the whole card as a superfloppy (a blank
			 * card has no VBR/MBR yet, so detection would fail). */
			sd_part_lba = 0;
			sd_part_blocks = sd_card_get_info()->block_count;
			if (sd_part_blocks == 0u)
				return;
		} else if (detect_partition() != 0) {
			return;     /* status stays FX_IO_ERROR -> mount fails cleanly */
		}
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_UNINIT:
		/* Leave the card probed so the next mount is cheap; the SDMMC
		 * teardown (hot-plug) is handled by the `sd` command, not here. */
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_BOOT_READ:
		if (!bounds_ok(0, 1))
			return;
		if (sd_card_read_blocks(sd_part_lba, buffer, 1) != SD_OK)
			return;
		/* FileX leaves the boot-signature check to the driver. */
		if (buffer[510] != 0x55u || buffer[511] != 0xAAu) {
			media_ptr->fx_media_driver_status = FX_MEDIA_INVALID;
			return;
		}
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_READ:
		if (!bounds_ok(sector, count))
			return;
		if (sd_card_read_blocks(sd_part_lba + sector, buffer, count) != SD_OK)
			return;
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_WRITE:
		if (!bounds_ok(sector, count))
			return;
		if (sd_card_write_blocks(sd_part_lba + sector, buffer, count) != SD_OK)
			return;
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_BOOT_WRITE:
		if (media_ptr->fx_media_bytes_per_sector != SD_BLOCK_SIZE)
			return;
		if (!bounds_ok(0, 1))
			return;
		if (sd_card_write_blocks(sd_part_lba, buffer, 1) != SD_OK)
			return;
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	case FX_DRIVER_RELEASE_SECTORS:
	case FX_DRIVER_FLUSH:
	case FX_DRIVER_ABORT:
		/* Block writes are synchronous to the card; nothing to flush. */
		media_ptr->fx_media_driver_status = FX_SUCCESS;
		break;

	default:
		LOG_WRN("unhandled driver request %u",
		        media_ptr->fx_media_driver_request);
		break;
	}
}
