/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fal_cfg.h
 * @brief   FAL (Flash Abstraction Layer) device and partition table (issue #37).
 *
 * FAL includes this by name from fal.h.  It declares which flash devices exist and
 * how they are carved up; the device itself is implemented in fal_nor.c.
 */
#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

/* Names are matched by strncmp at run time, so they must agree with the ones the
 * partition table and app/kv.c use -- hence the macros. */
#define NOR_FLASH_DEV_NAME             "w25q128"
#define KV_PART_NAME                   "kv"

/*
 * Partition layout of the 16 MB device.  Only the first megabyte is claimed:
 *
 *   0x000000  kv        1 MB   FlashDB KVDB, 4 KB sectors x 256
 *   0x100000  blob      3 MB   the read-only asset region (issue #10, widened by
 *                              issue #55): 6 slots of 512 KB, owned by app/blob.c.
 *                              It STILL has no partition entry, which is the point
 *                              -- blob.c addresses the device directly, so FAL (and
 *                              through it FlashDB) has no route out of the first
 *                              megabyte no matter what it is asked for.
 *   0x400000  (unused) 12 MB   unallocated, and likewise unreachable from here.
 *
 * The store sits at offset 0 so the asset region could grow upward without moving
 * -- and therefore without reformatting -- the configuration.
 *
 * 1 MB is far more than the key space needs (a few kilobytes).  The slack is the
 * point: FlashDB is log-structured, so spare sectors are what turn a rewrite into
 * an append instead of an erase, and at 100K erase cycles per sector this makes
 * the store effectively unwearable for configuration traffic.
 */
#define FAL_PART_HAS_TABLE_CFG

extern const struct fal_flash_dev nor_flash_dev;

#define FAL_FLASH_DEV_TABLE                                                    \
{                                                                              \
	&nor_flash_dev,                                                            \
}

/* FAL_PART_MAGIC_WORD is defined inside fal_partition.c, which is the only place
 * this macro is expanded -- expansion is lazy, so the reference resolves there. */
#define FAL_PART_TABLE                                                         \
{                                                                              \
	{FAL_PART_MAGIC_WORD, KV_PART_NAME, NOR_FLASH_DEV_NAME, 0, 1024 * 1024, 0},\
}

/* Route FAL's own logging into the same line-assembling shim FlashDB uses, so it
 * lands in `dmesg` rather than on a console someone may be typing into.  fal.h
 * includes this file before fal_def.h, whose defaults are #ifndef-guarded. */
void kv_fdb_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define FAL_PRINTF(...)                kv_fdb_print(__VA_ARGS__)

#endif /* _FAL_CFG_H_ */
