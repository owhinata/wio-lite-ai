/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    sd_fs_glue.h
 * @brief   Lazy-mount singleton for the microSD filesystem (issue #6).
 *
 * One FX_MEDIA over fx_sd_driver, mounted on first use, plus the ownership model
 * the `sd` subcommands share.  After sd_media_acquire() succeeds, fx_* calls on
 * the returned media are serialized by FileX's own per-media mutex; this layer's
 * mutex only guards the mount/unmount transitions and the ownership counters.
 * Thread-context only.
 */
#ifndef SD_FS_GLUE_H
#define SD_FS_GLUE_H

#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Ownership sentinels returned by the gates below and by sd_media_acquire(),
 * deliberately outside the FileX status space so a status-to-string mapper cannot
 * confuse them with one.  Defined HERE, in the port layer, and re-declared under
 * the same guards by shell/cmds/fs_cmd_core.h -- the dependency runs
 * port/ <- shell/, never the other way (CLAUDE.md layering).
 */
#ifndef FS_ERR_BUSY
#define FS_ERR_BUSY 0xF0u      /* another owner (umount / format / raw) holds it */
#endif
#ifndef FS_ERR_NO_CARD
#define FS_ERR_NO_CARD 0xF1u   /* the slot is empty                              */
#endif

/** Create the glue mutexes and run fx_system_initialize().  Call once from
 *  tx_application_define(). */
void sd_fs_glue_init(void);

/** Return the mounted media in *out, mounting (fx_media_open) on first use.
 *  Returns FX_SUCCESS, FS_ERR_BUSY, FS_ERR_NO_CARD, or the failing fx status. */
UINT sd_media_acquire(FX_MEDIA **out);

/**
 * Mount while the caller ALREADY holds the exclusive slot.
 *
 * sd_media_acquire() refuses when sd_busy is set, which is exactly the state a
 * format is in -- so a format that released the slot before remounting would open
 * a window for the other console or a background job to take it and make the
 * remount fail with FS_ERR_BUSY even though the format succeeded.  This entry
 * point lets `sd format` keep the slot across both steps.  Skips the hot-plug
 * eviction check too: the card was just probed by the format itself.
 */
UINT sd_media_mount_exclusive(FX_MEDIA **out);

/** Flush + unmount (fx_media_close).  No-op when not mounted. */
UINT sd_media_unmount(void);

/** Nonzero while the SD media is mounted. */
int sd_is_mounted(void);

/** Nonzero while format/umount/raw owns the SD media. */
int sd_is_busy(void);

/*
 * Media ownership, reader/writer style: every FS subcommand holds a shared op slot
 * for its whole duration; `umount`/`format` take the exclusive slot (refused while
 * any op or another exclusive runs); the card-disruptive `sd info`/`sd read` (which
 * may re-probe, i.e. HAL_SD_DeInit) take the raw slot, which is the exclusive slot
 * plus a refusal while the media is mounted.  begin calls return FX_SUCCESS or
 * FS_ERR_BUSY and never block; pair every successful begin with its end on all
 * exit paths.
 */
UINT sd_op_begin(void);
void sd_op_end(void);
UINT sd_exclusive_begin(void);
void sd_exclusive_end(void);
UINT sd_raw_begin(void);
void sd_raw_end(void);

/** Serialize multi-call directory sequences (`sd ls` sets the media-global default
 *  directory, iterates, then restores it -- FileX's mutex only covers each call). */
void sd_dir_lock(void);
void sd_dir_unlock(void);

/** The media singleton + its sector cache, for `sd format` orchestration.
 *  (fx_media_format leaves the media closed, so the caller remounts through the
 *  normal sd_media_acquire() path afterwards and the mounted flag stays correct.) */
FX_MEDIA *sd_glue_media(void);
UCHAR    *sd_glue_cache(void);
ULONG     sd_glue_cache_size(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_FS_GLUE_H */
