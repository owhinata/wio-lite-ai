/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fs_cmd_core.h
 * @brief   Media-independent FileX command core (issue #6).
 *
 * The FileX command bodies (ls/cat/write/rm/mkdir/info/umount) are written once
 * and parameterized over a `struct fs_device` vtable, so everything media-specific
 * -- the lazy-mount singleton, the ownership gates, the trailing info line, the
 * mount-failure hint -- is reached through the device rather than hard-coded.
 * Each command file owns its `struct fs_device` instance and registers thin
 * wrappers that bind it (the CLI passes only the leaf subcommand name as argv[0],
 * so the device cannot be recovered from argv).
 *
 * Today microSD (cmd_sd.c) is the only device.  The indirection is kept from the
 * f746 original it was ported from -- where the same bodies also served a
 * LevelX/QSPI NOR filesystem -- because this board has an unused 16 MB W25Q128 on
 * OCTOSPI2 that issue #10 may bring back as exactly that second media.
 *
 * Thread-context only; the bodies use the cli_* output API and the device's
 * ownership gates, so they are safe from background jobs.
 */
#ifndef FS_CMD_CORE_H
#define FS_CMD_CORE_H

#include "fx_api.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cli_instance;

/* Sentinels returned by the ownership gates and by acquire(), deliberately outside
 * the FileX status space so fs_strerror() cannot confuse them with one.  The
 * authoritative copy lives in the port layer (port/filex/sd_fs_glue.h) since that
 * is what produces them; guarded so including both headers is fine, and repeated
 * here so this file stands alone for a media whose glue it does not include. */
#ifndef FS_ERR_BUSY
#define FS_ERR_BUSY 0xF0u      /* another owner (umount / format / raw) holds it */
#endif
#ifndef FS_ERR_NO_CARD
#define FS_ERR_NO_CARD 0xF1u   /* the slot is empty (removable media only)       */
#endif

/**
 * Media abstraction for the shared command bodies.  One instance per command file
 * (cmd_sd.c => microSD); the callbacks forward to that media's glue.
 */
struct fs_device {
	const char *name;        /* "sd": message prefix                           */
	const char *mount_hint;  /* second-line hint printed on a mount failure     */
	UINT (*acquire)(FX_MEDIA **out);   /* lazy mount, return the media          */
	UINT (*unmount)(void);
	int  (*is_mounted)(void);
	UINT (*op_begin)(void);  void (*op_end)(void);     /* shared op slot         */
	UINT (*excl_begin)(void);void (*excl_end)(void);   /* exclusive (umount)     */
	void (*dir_lock)(void);  void (*dir_unlock)(void); /* listing serialization  */
	/* Trailing info line(s) for `info`, or NULL.  Called with the mounted media. */
	void (*info_extra)(struct cli_instance *sh, FX_MEDIA *media);
};

/** FileX status -> human string (media-neutral; hints come from mount_hint). */
const char *fs_strerror(UINT status);

/** Mount-on-demand: returns the media or NULL after printing the failure + the
 *  device's mount_hint.  Exposed for device-specific handlers (the format remount). */
FX_MEDIA *fs_core_mount(const struct fs_device *dev, struct cli_instance *sh);

/* Shared command bodies; each binds a device.  Argument arity matches the
 * registered subcommands (argv[0] = leaf name). */
int fs_core_ls    (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_cat   (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_write (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_rm    (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_mkdir (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_info  (const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);
int fs_core_umount(const struct fs_device *dev, struct cli_instance *sh, int argc, char **argv);

/**
 * Read an entire file from @p dev into @p buf (capacity @p cap bytes) through the
 * device's shared op gate; fails if the file is larger than @p cap.  Returns 0 with
 * *out_len set to the bytes read, or 1 (a message is printed).  The cross-command
 * reuse point for future consumers that need a whole file in RAM (#7 camera, #9 AI).
 */
int fs_core_read_file(const struct fs_device *dev, struct cli_instance *sh,
                      const char *path, void *buf, uint32_t cap, uint32_t *out_len);

/** Device accessor for cross-command reuse.  Defined by the owning command file. */
const struct fs_device *fs_sd_device(void);   /* cmd_sd.c */

#ifdef __cplusplus
}
#endif

#endif /* FS_CMD_CORE_H */
