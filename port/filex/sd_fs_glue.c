/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    sd_fs_glue.c
 * @brief   Lazy-mount singleton for the microSD filesystem (issue #6).
 *
 * Ported from the STM32F746 Discovery firmware, where an identical file sat beside
 * a second (LevelX/QSPI NOR) media and inherited fx_system_initialize() from it.
 * Here the SD media is the only one, so this file owns that call.
 *
 * The media cache is plain cacheable SRAM: fx_sd_driver hands every buffer to
 * sd_card_read/write_blocks(), which bounces through its own .axi_dma buffer and
 * does the D-cache maintenance, so nothing here needs special placement.
 */
#include "sd_fs_glue.h"     /* incl. the FS_ERR_* ownership sentinels */
#include "fx_sd_driver.h"
#include "sd_card.h"        /* probed/status/deinit for hot-plug */

#include "tx_api.h"

#define LOG_TAG "sdfs"
#include "log.h"

static FX_MEDIA sd_media;
static UCHAR    sd_media_cache[4096];
static TX_MUTEX sd_mount_lock;
static TX_MUTEX sd_dir_mutex;
static int      sd_mounted;
static int      sd_busy;        /* umount/format/raw owns the media         */
static int      sd_active_ops;  /* FS commands currently inside a body      */

void sd_fs_glue_init(void)
{
	tx_mutex_create(&sd_mount_lock, "sd_mount", TX_INHERIT);
	tx_mutex_create(&sd_dir_mutex, "sd_dir", TX_INHERIT);
	/* Sole FileX media in this firmware, so this file owns the one-time system
	   init (it sets up FileX's date/time and internal structures). */
	fx_system_initialize();
}

FX_MEDIA *sd_glue_media(void)
{
	return &sd_media;
}

UCHAR *sd_glue_cache(void)
{
	return sd_media_cache;
}

ULONG sd_glue_cache_size(void)
{
	return sizeof sd_media_cache;
}

int sd_is_mounted(void)
{
	return sd_mounted;
}

int sd_is_busy(void)
{
	return sd_busy;
}

UINT sd_op_begin(void)
{
	UINT status = FX_SUCCESS;

	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);
	if (sd_busy)
		status = FS_ERR_BUSY;
	else
		sd_active_ops++;
	tx_mutex_put(&sd_mount_lock);
	return status;
}

void sd_op_end(void)
{
	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);
	sd_active_ops--;
	tx_mutex_put(&sd_mount_lock);
}

/* Exclusive owner; @p require_unmounted additionally refuses while mounted
 * (a card re-probe or a format must never run under a live filesystem). */
static UINT sd_owner_begin(int require_unmounted)
{
	UINT status = FX_SUCCESS;

	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);
	if (sd_busy || sd_active_ops > 0 || (require_unmounted && sd_mounted))
		status = FS_ERR_BUSY;
	else
		sd_busy = 1;
	tx_mutex_put(&sd_mount_lock);
	return status;
}

UINT sd_exclusive_begin(void)
{
	return sd_owner_begin(0);
}

UINT sd_raw_begin(void)
{
	return sd_owner_begin(1);
}

void sd_exclusive_end(void)
{
	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);
	sd_busy = 0;
	tx_mutex_put(&sd_mount_lock);
}

void sd_raw_end(void)
{
	sd_exclusive_end();
}

void sd_dir_lock(void)
{
	tx_mutex_get(&sd_dir_mutex, TX_WAIT_FOREVER);
}

void sd_dir_unlock(void)
{
	tx_mutex_put(&sd_dir_mutex);
}

/* Shared mount body; the caller holds sd_mount_lock.  Maps an empty slot back out
 * of the flat FX_IO_ERROR that fx_media_open() reports for any INIT failure. */
static UINT mount_locked(void)
{
	UINT status;

	if (sd_mounted)
		return FX_SUCCESS;

	status = fx_media_open(&sd_media, "sd", fx_sd_driver,
	                       FX_NULL, sd_media_cache, sizeof sd_media_cache);
	if (status == FX_SUCCESS) {
		sd_mounted = 1;
		LOG_INF("media mounted");
	} else if (fx_sd_last_probe_rc() == SD_ERR_NO_CARD) {
		/* Empty slot, not a broken filesystem: report it as such so the shell
		 * prints "no card" instead of a mount hint about FAT. */
		status = FS_ERR_NO_CARD;
	}
	return status;
}

UINT sd_media_acquire(FX_MEDIA **out)
{
	UINT status = FX_SUCCESS;

	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);

	if (sd_busy) {
		/* umount/format/raw owns the media; do not mount behind its back. */
		tx_mutex_put(&sd_mount_lock);
		return FS_ERR_BUSY;
	}

	/* Hot-plug: if the mounted card was removed or swapped (the old RCA no longer
	 * answers CMD13), the open FX_MEDIA is now stale.  Evict it WITHOUT flushing --
	 * the card may be gone, so abort, not close.  Eviction is only safe when no
	 * other op is using the media (sd_active_ops counts this caller's own
	 * op_begin); if another op is in flight, refuse rather than hand back the stale
	 * media (that op fails with an I/O error and the next solo command evicts). */
	if (sd_mounted &&
	    (!sd_card_is_probed() || sd_card_status() != SD_OK)) {
		if (sd_active_ops > 1) {
			tx_mutex_put(&sd_mount_lock);
			return FS_ERR_BUSY;
		}
		(void)fx_media_abort(&sd_media);
		(void)sd_card_deinit();          /* next mount re-probes */
		sd_mounted = 0;
		LOG_INF("card removed/changed; media evicted");
	}

	/*
	 * NO "is a card present?" pre-check here, deliberately.
	 *
	 * The f746 original tested its card-detect GPIO at this point and returned
	 * FS_ERR_NO_CARD early.  This board has no such pin (see sd_card.h), so the
	 * nearest equivalent -- sd_card_is_probed() -- is a *cached* predicate that is
	 * false until something has already talked to the card.  Gating on it here
	 * would be a deadlock by construction: the probe happens inside FX_DRIVER_INIT,
	 * which is reached only by the fx_media_open() below, which the gate would have
	 * just refused.  A cold boot with a perfectly good card would report "no card"
	 * forever.
	 *
	 * So the probe decides, not a pre-check.  fx_media_open() reports any INIT
	 * failure as a flat FX_IO_ERROR, so the empty-slot case is recovered afterwards
	 * from the driver's recorded probe result.
	 */
	status = mount_locked();

	tx_mutex_put(&sd_mount_lock);

	if (status == FX_SUCCESS && out != FX_NULL)
		*out = &sd_media;
	return status;
}

UINT sd_media_mount_exclusive(FX_MEDIA **out)
{
	UINT status;

	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);
	status = mount_locked();
	tx_mutex_put(&sd_mount_lock);

	if (status == FX_SUCCESS && out != FX_NULL)
		*out = &sd_media;
	return status;
}

UINT sd_media_unmount(void)
{
	UINT status = FX_SUCCESS;

	tx_mutex_get(&sd_mount_lock, TX_WAIT_FOREVER);

	if (sd_mounted) {
		/* fx_media_close flushes and sends FX_DRIVER_UNINIT. */
		status = fx_media_close(&sd_media);
		sd_mounted = 0;
		if (status == FX_SUCCESS)
			LOG_INF("media unmounted");
		else
			LOG_ERR("fx_media_close failed (%u)", status);
	}

	tx_mutex_put(&sd_mount_lock);
	return status;
}
