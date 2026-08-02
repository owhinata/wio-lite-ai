/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    kv.h
 * @brief   Persistent configuration key-value store (issue #37).
 *
 * A FlashDB KVDB in the first megabyte of the external NOR, wrapped so the rest of
 * the firmware never sees FlashDB or FAL.  This is the answer to "where does a
 * setting live": it must be readable at every boot, before any filesystem, on a
 * board whose microSD slot may well be empty and has no card-detect line to say so
 * -- which leaves the soldered-down NOR as the only candidate.
 *
 * WHAT THIS LAYER ADDS OVER FlashDB
 *
 *   - **One lock around whole operations.**  FlashDB takes its own lock inside
 *     fdb_kv_set_blob()/fdb_kv_get_blob(), but NOT inside fdb_kv_iterator_init(),
 *     fdb_kv_iterate() or fdb_blob_read() -- so an iteration that read values as it
 *     went would be unserialised against a concurrent write.  Every entry point
 *     here takes the NOR device mutex for the duration instead.  The mutex is
 *     recursive, so FlashDB re-taking it inside costs nothing.
 *   - **A refusal instead of a wrong answer.**  Until kv_init() has succeeded, and
 *     after anything has gone wrong, every call fails with a state that says which
 *     -- the shell must come up and be usable with a broken store, but it must not
 *     silently behave as if the store were empty.
 *   - **fdb_kv_get() is never called.**  It returns a pointer into a `static char`
 *     buffer, so two threads reading two keys would hand each other's values back.
 *     Only the blob API is used.
 *
 * WHAT IT DOES NOT DO: it does not format on its own.  A partition that is neither
 * a valid database nor entirely blank is reported as corrupt and left alone, for
 * the user to erase deliberately with `kv format yes`.  Quietly reformatting would
 * destroy the evidence of whatever caused it.
 *
 * THREAD CONTEXT ONLY, and only after the scheduler is running: erases sleep.
 */
#ifndef APP_KV_H
#define APP_KV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest key this store accepts, excluding the terminator (FDB_KV_NAME_MAX-1). */
#define KV_KEY_MAX      31

/* Longest value this store accepts.  Comfortably above the longest thing the key
 * space defines (a 63-character PSK) while keeping shell-side buffers on stacks. */
#define KV_VALUE_MAX    256

/* Error returns (negative); 0 is success. */
#define KV_OK            0
#define KV_ERR_PARAM    -1   /* bad key or value (length, characters, null)      */
#define KV_ERR_STATE    -2   /* store not ready -- see kv_state()                */
#define KV_ERR_BUSY     -3   /* another thread held the device too long          */
#define KV_ERR_IO       -4   /* FlashDB/flash reported a failure                 */
#define KV_ERR_NOTFOUND -5   /* no such key                                      */
#define KV_ERR_FULL     -6   /* partition is full                               */
#define KV_ERR_TRUNC    -7   /* value is longer than the buffer given            */

/* Store state (kv_state()). */
#define KV_STATE_INIT     0  /* kv_init() has not finished yet                   */
#define KV_STATE_READY    1  /* usable                                           */
#define KV_STATE_DOWN     2  /* the NOR device never came up                     */
#define KV_STATE_CORRUPT  3  /* partition holds something that is not a database */
#define KV_STATE_FAILED   4  /* initialisation failed for another reason         */

/** Snapshot for `kv info`. */
struct kv_info {
	uint32_t part_offset;    /* partition start, as a device offset             */
	uint32_t part_size;      /* partition length in bytes                       */
	uint32_t sec_size;       /* database sector size                            */
	uint32_t kv_count;       /* live keys                                       */
	uint32_t value_bytes;    /* sum of value lengths                            */
	uint32_t obj_bytes;      /* sum of on-flash record sizes (values + overhead)*/
};

/**
 * Open the database, formatting it ONLY if the partition is completely blank.
 *
 * Must run on a thread, after the scheduler has started: a first-time format
 * erases, and erases wait by sleeping.  It is called from the configuration thread
 * (app/kv_boot.c), not from tx_application_define().
 *
 * The two-stage approach is deliberate.  FlashDB will happily reformat a partition
 * whose sector headers it does not recognise, which would turn "something went
 * wrong" into "your settings are gone, and so is the evidence".  So the first
 * attempt runs with FDB_KVDB_CTRL_SET_NOT_FORMAT, which forbids that; only if that
 * fails AND a full scan shows the partition is erased does a second attempt run
 * with formatting allowed.  A normal boot never scans.
 *
 * Idempotent and fail-soft: on failure the store is latched into a state that
 * every other call reports, and the rest of the firmware is unaffected.
 */
int kv_init(void);

/** Current state (KV_STATE_*), and a one-line human explanation of it. */
int kv_state(void);
const char *kv_state_str(void);

/** Store @p len bytes under @p key, replacing any previous value. */
int kv_set(const char *key, const void *val, uint32_t len);

/**
 * Read @p key into @p buf.  @p out_len receives the STORED length even when it
 * exceeds @p buflen, in which case KV_ERR_TRUNC is returned and @p buf holds the
 * leading @p buflen bytes -- a caller can therefore size a second attempt.
 */
int kv_get(const char *key, void *buf, uint32_t buflen, uint32_t *out_len);

/** Delete @p key.  KV_ERR_NOTFOUND if it was not there. */
int kv_del(const char *key);

/** Erase the partition and lay down a fresh, empty database.  Destructive. */
int kv_format(void);

/** Fill @p info with the partition geometry and live usage. */
int kv_get_info(struct kv_info *info);

/**
 * Call @p cb once per key whose name starts with @p prefix (NULL or "" = all),
 * with the value already read into a caller-supplied buffer.
 *
 * The whole walk runs under one lock, which is what makes it consistent -- and
 * also means the callback MUST NOT call back into any kv_* function: the lock
 * would be granted (it is recursive) but FlashDB keeps its iteration state inside
 * the database object, so a nested operation would corrupt the walk in progress.
 * Returning non-zero from @p cb stops the walk early.
 *
 * @param buf     scratch the value is read into before each callback
 * @param buflen  its size; longer values are truncated and flagged to the callback
 *
 * @p truncated means "this is not the whole value" -- either it did not fit in
 * @p buf, or the flash read failed and returned nothing.  The two are folded
 * together on purpose: a listing must never present a short value as if it were
 * complete, and the distinction is of no use to the caller of a listing.
 */
typedef int (*kv_iter_cb)(const char *key, const void *val, uint32_t len,
                          int truncated, void *arg);
int kv_foreach(const char *prefix, void *buf, uint32_t buflen,
               kv_iter_cb cb, void *arg);

/**
 * Create the configuration thread (app/kv_boot.c), which opens the store once the
 * scheduler is running.  Pure ThreadX object creation -- call it from
 * tx_application_define().  Returns 0 on success.
 */
int kv_boot_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_KV_H */
