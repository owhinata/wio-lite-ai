/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fs_cmd_core.c
 * @brief   Media-independent FileX command bodies (issue #6).
 *
 * Parameterized over struct fs_device so the bodies carry no knowledge of which
 * media they are running on.  The FileX API is device-agnostic; only mount, the
 * ownership gates, the listing lock, the trailing info line and the mount hint go
 * through the device.  Ported unchanged from the STM32F746 Discovery firmware,
 * where the same file served both a microSD and a LevelX/QSPI NOR filesystem --
 * that it needed no edit at all is the evidence the abstraction is real.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "fs_cmd_core.h"

#include <stdint.h>
#include <string.h>

/* One read/copy unit for `cat`; small enough for the 4 KB shell stack. */
#define FS_CAT_CHUNK 256u

const char *fs_strerror(UINT status)
{
	switch (status) {
	case FS_ERR_BUSY:        return "busy (format in progress)";
	case FS_ERR_NO_CARD:     return "no card";
	case FX_BOOT_ERROR:      return "invalid boot record";
	case FX_MEDIA_INVALID:   return "media invalid";
	case FX_NOT_FOUND:       return "not found";
	case FX_NOT_A_FILE:      return "not a file";
	case FX_ACCESS_ERROR:    return "access error";
	case FX_FILE_CORRUPT:    return "file corrupt";
	case FX_NO_MORE_SPACE:   return "no space left";
	case FX_ALREADY_CREATED: return "already exists";
	case FX_INVALID_NAME:    return "invalid name";
	case FX_NOT_DIRECTORY:   return "not a directory";
	case FX_DIR_NOT_EMPTY:   return "directory not empty";
	case FX_MEDIA_NOT_OPEN:  return "media not open";
	case FX_WRITE_PROTECT:   return "write protected";
	case FX_IO_ERROR:        return "I/O error";
	default:                 return "filesystem error";
	}
}

/*
 * Run one normal subcommand body under a shared op slot, so format / umount /
 * raw operations can never yank the media away mid-command (they fail with
 * FS_ERR_BUSY instead while any body is inside).
 */
static int core_run_op(const struct fs_device *dev, struct cli_instance *sh,
                       int argc, char **argv,
                       int (*body)(const struct fs_device *,
                                   struct cli_instance *, int, char **))
{
	UINT status = dev->op_begin();
	int rc;

	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s\r\n", dev->name, fs_strerror(status));
		return 1;
	}
	rc = body(dev, sh, argc, argv);
	dev->op_end();
	return rc;
}

/* Mount-on-demand wrapper shared by every body that needs the media. */
FX_MEDIA *fs_core_mount(const struct fs_device *dev, struct cli_instance *sh)
{
	FX_MEDIA *media;
	UINT status;

	status = dev->acquire(&media);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: mount failed: %s (0x%02x)\r\n",
		          dev->name, fs_strerror(status), status);
		if (status != FS_ERR_BUSY && status != FS_ERR_NO_CARD)
			cli_error(sh, "%s: %s\r\n", dev->name, dev->mount_hint);
		return NULL;
	}
	return media;
}

/* ---- ls ------------------------------------------------------------------ */

static int do_ls(const struct fs_device *dev, struct cli_instance *sh,
                 int argc, char **argv)
{
	char name[FX_MAX_LONG_NAME_LEN];
	const char *path = (argc >= 2) ? argv[1] : "/";
	FX_MEDIA *media = fs_core_mount(dev, sh);
	UINT attr, status;
	ULONG size;
	int rc = 0;

	if (media == NULL)
		return 1;

	/* The default directory is media-global and the iteration spans many
	 * FileX calls; serialize whole listings against each other (bg jobs). */
	dev->dir_lock();

	status = fx_directory_default_set(media, (CHAR *)path);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s: %s (0x%02x)\r\n",
		          dev->name, path, fs_strerror(status), status);
		dev->dir_unlock();
		return 1;
	}

	status = fx_directory_first_full_entry_find(media, name, &attr, &size,
	                                            NULL, NULL, NULL,
	                                            NULL, NULL, NULL);
	while (status == FX_SUCCESS) {
		if (cli_cancel_requested(sh)) {
			rc = 1;
			break;
		}
		if (cli_print(sh, "%c %8lu  %s\r\n",
		              (attr & FX_DIRECTORY) ? 'd' : '-',
		              (unsigned long)size, name) < 0) {
			rc = 1;
			break;
		}
		status = fx_directory_next_full_entry_find(media, name, &attr, &size,
		                                           NULL, NULL, NULL,
		                                           NULL, NULL, NULL);
	}
	if (status != FX_SUCCESS && status != FX_NO_MORE_ENTRIES && rc == 0) {
		cli_error(sh, "%s: ls failed: %s (0x%02x)\r\n",
		          dev->name, fs_strerror(status), status);
		rc = 1;
	}

	(void)fx_directory_default_set(media, "/");
	dev->dir_unlock();
	return rc;
}

/* ---- cat ----------------------------------------------------------------- */

static int do_cat(const struct fs_device *dev, struct cli_instance *sh,
                  int argc, char **argv)
{
	char buf[FS_CAT_CHUNK];
	FX_MEDIA *media = fs_core_mount(dev, sh);
	FX_FILE file;
	ULONG got;
	UINT status;
	int rc = 0;

	(void)argc;

	if (media == NULL)
		return 1;

	status = fx_file_open(media, &file, argv[1], FX_OPEN_FOR_READ);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s: %s (0x%02x)\r\n",
		          dev->name, argv[1], fs_strerror(status), status);
		return 1;
	}

	for (;;) {
		if (cli_cancel_requested(sh)) {
			rc = 1;
			break;
		}
		status = fx_file_read(&file, buf, sizeof buf, &got);
		if (status == FX_END_OF_FILE)
			break;
		if (status != FX_SUCCESS) {
			cli_error(sh, "%s: read failed: %s (0x%02x)\r\n",
			          dev->name, fs_strerror(status), status);
			rc = 1;
			break;
		}
		if (cli_write(sh, buf, got) < 0) {
			rc = 1;
			break;
		}
	}
	(void)fx_file_close(&file);
	if (rc == 0)
		cli_print(sh, "\r\n");
	return rc;
}

/* ---- write --------------------------------------------------------------- */

static int do_write(const struct fs_device *dev, struct cli_instance *sh,
                    int argc, char **argv)
{
	FX_MEDIA *media = fs_core_mount(dev, sh);
	FX_FILE file;
	ULONG len = (ULONG)strlen(argv[2]);
	UINT status;

	(void)argc;

	if (media == NULL)
		return 1;

	status = fx_file_create(media, argv[1]);
	if (status != FX_SUCCESS && status != FX_ALREADY_CREATED) {
		cli_error(sh, "%s: create %s: %s (0x%02x)\r\n",
		          dev->name, argv[1], fs_strerror(status), status);
		return 1;
	}

	status = fx_file_open(media, &file, argv[1], FX_OPEN_FOR_WRITE);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: open %s: %s (0x%02x)\r\n",
		          dev->name, argv[1], fs_strerror(status), status);
		return 1;
	}

	/* Overwrite semantics: truncate, write, close, flush to media. */
	status = fx_file_truncate(&file, 0);
	if (status == FX_SUCCESS && len > 0)
		status = fx_file_write(&file, argv[2], len);
	(void)fx_file_close(&file);
	if (status == FX_SUCCESS)
		status = fx_media_flush(media);

	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: write failed: %s (0x%02x)\r\n",
		          dev->name, fs_strerror(status), status);
		return 1;
	}
	cli_print(sh, "wrote %lu bytes to %s\r\n", (unsigned long)len, argv[1]);
	return 0;
}

/* ---- rm / mkdir ---------------------------------------------------------- */

static int do_rm(const struct fs_device *dev, struct cli_instance *sh,
                 int argc, char **argv)
{
	FX_MEDIA *media = fs_core_mount(dev, sh);
	UINT status;

	(void)argc;

	if (media == NULL)
		return 1;

	status = fx_file_delete(media, argv[1]);
	if (status == FX_NOT_A_FILE)
		status = fx_directory_delete(media, argv[1]);
	if (status == FX_SUCCESS)
		status = fx_media_flush(media);

	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: rm %s: %s (0x%02x)\r\n",
		          dev->name, argv[1], fs_strerror(status), status);
		return 1;
	}
	cli_print(sh, "removed %s\r\n", argv[1]);
	return 0;
}

static int do_mkdir(const struct fs_device *dev, struct cli_instance *sh,
                    int argc, char **argv)
{
	FX_MEDIA *media = fs_core_mount(dev, sh);
	UINT status;

	(void)argc;

	if (media == NULL)
		return 1;

	status = fx_directory_create(media, argv[1]);
	if (status == FX_SUCCESS)
		status = fx_media_flush(media);

	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: mkdir %s: %s (0x%02x)\r\n",
		          dev->name, argv[1], fs_strerror(status), status);
		return 1;
	}
	cli_print(sh, "created %s\r\n", argv[1]);
	return 0;
}

/* ---- info ---------------------------------------------------------------- */

static int do_info(const struct fs_device *dev, struct cli_instance *sh,
                   int argc, char **argv)
{
	FX_MEDIA *media;
	ULONG64 avail = 0, total_bytes;
	ULONG clusters;
	UINT status;

	(void)argc; (void)argv;

	if (!dev->is_mounted())
		cli_print(sh, "state    : unmounted (mounting...)\r\n");
	media = fs_core_mount(dev, sh);
	if (media == NULL)
		return 1;

	clusters    = media->fx_media_total_clusters;
	/* 64-bit so multi-GB SD cards do not overflow (FileX's 32-bit
	 * fx_media_space_available saturates >4 GiB). */
	total_bytes = (ULONG64)clusters * media->fx_media_sectors_per_cluster *
	              media->fx_media_bytes_per_sector;
	status = fx_media_extended_space_available(media, &avail);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: space query failed: %s (0x%02x)\r\n",
		          dev->name, fs_strerror(status), status);
		return 1;
	}

	cli_print(sh, "state    : mounted\r\n");
	cli_print(sh, "fat      : FAT%s (%lu clusters)\r\n",
	          (clusters < 4085u) ? "12" : (clusters < 65525u) ? "16" : "32",
	          (unsigned long)clusters);
	cli_print(sh, "cluster  : %lu B\r\n",
	          (unsigned long)(media->fx_media_sectors_per_cluster *
	                          media->fx_media_bytes_per_sector));
	cli_print(sh, "total    : %llu KiB\r\n",
	          (unsigned long long)(total_bytes / 1024u));
	cli_print(sh, "free     : %llu KiB\r\n",
	          (unsigned long long)(avail / 1024u));

	if (dev->info_extra != NULL)
		dev->info_extra(sh, media);
	return 0;
}

/* ---- public entry points (bind the device + shared op slot) -------------- */

int fs_core_ls(const struct fs_device *dev, struct cli_instance *sh,
               int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_ls);
}

int fs_core_cat(const struct fs_device *dev, struct cli_instance *sh,
                int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_cat);
}

int fs_core_write(const struct fs_device *dev, struct cli_instance *sh,
                  int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_write);
}

int fs_core_rm(const struct fs_device *dev, struct cli_instance *sh,
               int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_rm);
}

int fs_core_mkdir(const struct fs_device *dev, struct cli_instance *sh,
                  int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_mkdir);
}

int fs_core_info(const struct fs_device *dev, struct cli_instance *sh,
                 int argc, char **argv)
{
	return core_run_op(dev, sh, argc, argv, do_info);
}

int fs_core_umount(const struct fs_device *dev, struct cli_instance *sh,
                   int argc, char **argv)
{
	UINT status;
	int rc = 0;

	(void)argc; (void)argv;

	/* Unmount closes the media (and any open files with it), so it needs the
	 * exclusive slot: refused while a command is inside. */
	status = dev->excl_begin();
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s\r\n", dev->name, fs_strerror(status));
		return 1;
	}

	if (!dev->is_mounted()) {
		cli_print(sh, "not mounted\r\n");
	} else {
		status = dev->unmount();
		if (status != FX_SUCCESS) {
			cli_error(sh, "%s: unmount failed: %s (0x%02x)\r\n",
			          dev->name, fs_strerror(status), status);
			rc = 1;
		} else {
			cli_print(sh, "unmounted\r\n");
		}
	}

	dev->excl_end();
	return rc;
}

/*
 * Read an entire file into a caller buffer through the device's shared op gate.
 * No in-tree caller yet (--gc-sections drops it until there is one); it exists as
 * the reuse point for the consumers that will want a whole file in RAM -- a camera
 * frame written back (#7), an AI model loaded (#9).  The
 * size is checked up front against @p cap (fx_file_current_file_size is filled by
 * fx_file_open), so the buffer is never partially filled on an oversize file.
 * Returns 0 and sets *out_len to the byte count, or 1 with a message printed.
 */
int fs_core_read_file(const struct fs_device *dev, struct cli_instance *sh,
                      const char *path, void *buf, uint32_t cap, uint32_t *out_len)
{
	FX_MEDIA *media;
	FX_FILE   file;
	ULONG64   fsize;
	uint32_t  off = 0;
	UINT      status;
	int       rc = 0;

	if (out_len)
		*out_len = 0;

	status = dev->op_begin();
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s\r\n", dev->name, fs_strerror(status));
		return 1;
	}
	media = fs_core_mount(dev, sh);      /* prints its own failure + hint */
	if (media == NULL) {
		dev->op_end();
		return 1;
	}

	status = fx_file_open(media, &file, (CHAR *)path, FX_OPEN_FOR_READ);
	if (status != FX_SUCCESS) {
		cli_error(sh, "%s: %s: %s (0x%02x)\r\n",
		          dev->name, path, fs_strerror(status), status);
		dev->op_end();
		return 1;
	}

	/* Reject oversize before reading a single byte (fx_file_open filled the size). */
	fsize = file.fx_file_current_file_size;
	if (fsize > (ULONG64)cap) {
		cli_error(sh, "%s: %s too large: %lu B > %lu B cap\r\n",
		          dev->name, path, (unsigned long)fsize, (unsigned long)cap);
		rc = 1;
	}

	while (rc == 0) {
		ULONG got;

		if (cli_cancel_requested(sh)) { rc = 1; break; }
		status = fx_file_read(&file, (UCHAR *)buf + off, cap - off, &got);
		if (status == FX_END_OF_FILE)
			break;
		if (status != FX_SUCCESS) {
			cli_error(sh, "%s: read failed: %s (0x%02x)\r\n",
			          dev->name, fs_strerror(status), status);
			rc = 1;
			break;
		}
		off += (uint32_t)got;
		if (got == 0 || off >= cap)      /* done or buffer full */
			break;
	}

	/* A short read (off != size) must not read as success -- otherwise a truncated
	 * transfer would hand a partial .tflite to the model loader (codex review). */
	if (rc == 0 && (ULONG64)off != fsize) {
		cli_error(sh, "%s: short read: %lu of %lu B\r\n",
		          dev->name, (unsigned long)off, (unsigned long)fsize);
		rc = 1;
	}

	(void)fx_file_close(&file);
	dev->op_end();
	if (rc == 0 && out_len)
		*out_len = off;
	return rc;
}
