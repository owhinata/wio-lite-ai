/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    kv.c
 * @brief   Persistent configuration key-value store (issue #37).
 *
 * See kv.h for the contract.  This file is the only caller of FlashDB.
 */
#include "kv.h"

#include <flashdb.h>
#include <fal.h>

#include "nor_flash.h"
#include "tx_api.h"

#include <string.h>

#define LOG_TAG "kv"
#include "log.h"

/*
 * How long an outer entry point waits for the device before giving up.  It has to
 * outlast an ordinary sector erase (max 400 ms) by a wide margin without making a
 * stuck operation look like a hang.  It does NOT try to outlast `kv format`, which
 * erases a megabyte and can hold the device for tens of seconds in the worst case
 * -- a command that arrives during a format is told the store is busy, which is
 * both true and more useful than a console that stops answering.
 */
#define KV_LOCK_TIMEOUT_TICKS  5000u

/* Buffer for the blank-partition scan.  Only used on the failure path. */
#define KV_SCAN_CHUNK  256u

static struct fdb_kvdb kvdb;              /* static + zero-initialised */
static int kv_status = KV_STATE_INIT;
static struct kv_info kv_geom;            /* partition geometry, filled at init */

/* Set if the lock callback below ever failed.  FlashDB cannot be told about it at
 * the time (see kv_db_lock), so the next outer call refuses instead. */
static volatile int kv_lock_broken;

/* ------------------------------------------------------------------ *
 *  Locking
 * ------------------------------------------------------------------ */

/*
 * FlashDB's lock callback.  Its signature returns nothing and its caller does not
 * look at the result, so a failure here cannot be reported downward: FlashDB would
 * simply carry on and touch the device unlocked.  The only two defences are (a)
 * waiting forever rather than giving up, so the ordinary "someone else is erasing"
 * case can never turn into a failure at all, and (b) latching kv_lock_broken so
 * the next call through the public API refuses.
 */
static void kv_db_lock(fdb_db_t db)
{
	(void)db;
	if (nor_lock_blocking() != NOR_OK) {
		kv_lock_broken = 1;
		LOG_ERR("device lock failed inside FlashDB -- store latched offline");
	}
}

static void kv_db_unlock(fdb_db_t db)
{
	(void)db;
	nor_unlock();
}

/* Outer gate for every public entry point: refuse unless the store is usable, then
 * take the device with a finite wait.  Pairs with kv_op_end(). */
static int kv_op_begin(void)
{
	int rc;

	if (kv_status != KV_STATE_READY)
		return KV_ERR_STATE;
	if (kv_lock_broken || kv_fdb_assert_tripped())
		return KV_ERR_STATE;

	rc = nor_trylock(KV_LOCK_TIMEOUT_TICKS);
	if (rc == NOR_ERR_BUSY)
		return KV_ERR_BUSY;
	if (rc != NOR_OK)
		return KV_ERR_STATE;
	return KV_OK;
}

static void kv_op_end(void)
{
	nor_unlock();
}

/* ------------------------------------------------------------------ *
 *  Validation
 * ------------------------------------------------------------------ */

/*
 * Keys are dotted paths.  Restricting the alphabet is not decoration: keys are
 * typed on a shell line, printed in tables and compared by name, so a key with a
 * space or a control character in it would be storable but not addressable.
 */
static int kv_key_ok(const char *key)
{
	size_t i;

	if (key == NULL || key[0] == '\0')
		return 0;
	for (i = 0u; key[i] != '\0'; i++) {
		char c = key[i];

		if (i >= KV_KEY_MAX)
			return 0;
		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		      (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-'))
			return 0;
	}
	return 1;
}

/* Map FlashDB's error enum onto this layer's codes. */
static int kv_map_err(fdb_err_t err)
{
	switch (err) {
	case FDB_NO_ERR:        return KV_OK;
	case FDB_KV_NAME_ERR:   return KV_ERR_PARAM;
	case FDB_KV_NAME_EXIST: return KV_ERR_PARAM;
	case FDB_SAVED_FULL:    return KV_ERR_FULL;
	case FDB_READ_ERR:
	case FDB_WRITE_ERR:
	case FDB_ERASE_ERR:     return KV_ERR_IO;
	default:                return KV_ERR_IO;
	}
}

/* ------------------------------------------------------------------ *
 *  Initialisation
 * ------------------------------------------------------------------ */

/* Is the whole partition erased?  Only asked when an init attempt has already
 * failed, so the cost (a megabyte read, tens of milliseconds) is paid on the rare
 * path and a normal boot never scans. */
static int kv_partition_blank(const struct fal_partition *part)
{
	uint8_t buf[KV_SCAN_CHUNK];
	uint32_t off = 0u;

	while (off < part->len) {
		uint32_t chunk = part->len - off;
		uint32_t i;

		if (chunk > sizeof buf)
			chunk = sizeof buf;
		if (fal_partition_read(part, off, buf, chunk) < 0)
			return 0;
		for (i = 0u; i < chunk; i++) {
			if (buf[i] != 0xFFu)
				return 0;
		}
		off += chunk;
	}
	return 1;
}

/*
 * One initialisation attempt from a clean slate.
 *
 * The database object is re-zeroed and its three control settings re-applied every
 * time, so the second attempt cannot inherit anything the first one left behind.
 * That sidesteps the question of whether a half-initialised fdb_kvdb is reusable
 * entirely -- and it is safe to do because FlashDB keeps no registry of open
 * databases; the whole of its state lives in this struct.
 */
static fdb_err_t kv_try_init(int allow_format)
{
	bool not_format = allow_format ? false : true;

	memset(&kvdb, 0, sizeof kvdb);
	fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, (void *)(uintptr_t)kv_db_lock);
	fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)(uintptr_t)kv_db_unlock);
	fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_NOT_FORMAT, &not_format);
	/* The name and partition strings are stored by pointer, so they must be
	 * literals with static lifetime -- not stack buffers. */
	return fdb_kvdb_init(&kvdb, "cfg", KV_PART_NAME, NULL, NULL);
}

int kv_init(void)
{
	const struct fal_partition *part;
	fdb_err_t err;

	if (kv_status == KV_STATE_READY)
		return KV_OK;

	if (!nor_flash_ready()) {
		kv_status = KV_STATE_DOWN;
		LOG_ERR("NOR device down -- configuration store unavailable");
		return KV_ERR_STATE;
	}

	/* Hold the device across the whole of initialisation.  FlashDB drops its own
	 * lock between internal steps, and a first-time format erases a megabyte; an
	 * outer hold keeps a shell command from landing in the middle of that. */
	if (nor_lock_blocking() != NOR_OK) {
		kv_status = KV_STATE_FAILED;
		return KV_ERR_STATE;
	}

	if (fal_init() < 0) {
		kv_status = KV_STATE_FAILED;
		LOG_ERR("FAL init failed");
		goto out;
	}
	part = fal_partition_find(KV_PART_NAME);
	if (part == NULL) {
		kv_status = KV_STATE_FAILED;
		LOG_ERR("partition '%s' not found", KV_PART_NAME);
		goto out;
	}
	kv_geom.part_offset = (uint32_t)part->offset;
	kv_geom.part_size = (uint32_t)part->len;
	/* Take the sector size from the flash device rather than waiting to read it
	 * back out of the database.  `kv info` has to be useful precisely when the
	 * database did NOT open, and reporting "0 B sectors" there would look like a
	 * second fault on top of the first. */
	{
		const struct fal_flash_dev *dev = fal_flash_device_find(part->flash_name);

		if (dev != NULL)
			kv_geom.sec_size = (uint32_t)dev->blk_size;
	}

	/* First attempt: refuse to format.  A database that is merely unrecognised
	 * must not be erased on our way past it. */
	err = kv_try_init(0);
	if (err == FDB_NO_ERR) {
		kv_status = KV_STATE_READY;
		kv_geom.sec_size = kvdb.parent.sec_size;
		LOG_INF("store open: %lu KB partition, %lu B sectors",
		        (unsigned long)(kv_geom.part_size >> 10),
		        (unsigned long)kv_geom.sec_size);
		goto out;
	}

	/* It failed.  Only a partition that is entirely erased is a first boot;
	 * anything else is damage, and gets reported rather than overwritten. */
	if (!kv_partition_blank(part)) {
		kv_status = KV_STATE_CORRUPT;
		LOG_ERR("partition is neither a valid database nor blank (%d) -- "
		        "run `kv format yes` to discard it", (int)err);
		goto out;
	}

	LOG_INF("blank partition -- creating the store");
	err = kv_try_init(1);
	if (err == FDB_NO_ERR) {
		kv_status = KV_STATE_READY;
		kv_geom.sec_size = kvdb.parent.sec_size;
		LOG_INF("store created: %lu KB partition, %lu B sectors",
		        (unsigned long)(kv_geom.part_size >> 10),
		        (unsigned long)kv_geom.sec_size);
	} else {
		kv_status = KV_STATE_FAILED;
		LOG_ERR("format failed (%d)", (int)err);
	}

out:
	nor_unlock();
	return (kv_status == KV_STATE_READY) ? KV_OK : KV_ERR_STATE;
}

int kv_state(void)
{
	if (kv_status == KV_STATE_READY && (kv_lock_broken || kv_fdb_assert_tripped()))
		return KV_STATE_FAILED;
	return kv_status;
}

const char *kv_state_str(void)
{
	switch (kv_state()) {
	case KV_STATE_READY:   return "ready";
	case KV_STATE_INIT:    return "not initialized yet";
	case KV_STATE_DOWN:    return "NOR flash device is down";
	case KV_STATE_CORRUPT: return "partition is not a valid database "
	                              "(run `kv format yes` to discard it)";
	default:               return "initialization failed";
	}
}

/* ------------------------------------------------------------------ *
 *  Data path
 * ------------------------------------------------------------------ */
int kv_set(const char *key, const void *val, uint32_t len)
{
	struct fdb_blob blob;
	int rc;

	if (!kv_key_ok(key) || val == NULL || len == 0u || len > KV_VALUE_MAX)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_map_err(fdb_kv_set_blob(&kvdb, key,
	                                fdb_blob_make(&blob, val, len)));
	kv_op_end();
	return rc;
}

/*
 * Deliberately NOT fdb_kv_get_blob().  That call funnels into get_kv(), which
 * ignores what _fdb_flash_read() returned (fdb_kvdb.c:636-638) and reports the
 * requested length either way -- so a value the flash failed to hand over would
 * come back as a successful read of a stale buffer.  Looking the key up and then
 * reading the blob explicitly costs one extra call and gets an answer that is
 * either the stored bytes or an error: fdb_blob_read() returns 0 when the read
 * fails (fdb_utils.c:243-246).  This is also exactly what kv_walk() does, so both
 * read paths behave the same.
 */
int kv_get(const char *key, void *buf, uint32_t buflen, uint32_t *out_len)
{
	struct fdb_blob blob;
	struct fdb_kv kv;
	size_t got;
	int rc;

	if (!kv_key_ok(key) || buf == NULL || buflen == 0u)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;

	if (fdb_kv_get_obj(&kvdb, key, &kv) == NULL || kv.value_len == 0u) {
		/* A stored length of zero cannot belong to a live key -- kv_set()
		 * rejects empty values -- so treat it as absent rather than as an
		 * empty answer. */
		if (out_len != NULL)
			*out_len = 0u;
		rc = KV_ERR_NOTFOUND;
		goto out;
	}

	fdb_kv_to_blob(&kv, fdb_blob_make(&blob, buf, buflen));
	if (out_len != NULL)
		*out_len = (uint32_t)blob.saved.len;   /* the STORED length, not the read */
	got = fdb_blob_read(&kvdb.parent, &blob);
	if (got == 0u)
		rc = KV_ERR_IO;
	else if (got < blob.saved.len)
		rc = KV_ERR_TRUNC;   /* buf holds the leading `buflen` bytes */

out:
	kv_op_end();
	return rc;
}

int kv_del(const char *key)
{
	fdb_err_t err;
	int rc;

	if (!kv_key_ok(key))
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	err = fdb_kv_del(&kvdb, key);
	kv_op_end();
	/* FlashDB reports a missing key as a generic failure; distinguish it here so
	 * `kv del` can say "no such key" instead of "I/O error". */
	if (err == FDB_KV_NAME_ERR)
		return KV_ERR_NOTFOUND;
	return kv_map_err(err);
}

int kv_format(void)
{
	const struct fal_partition *part;
	fdb_err_t err;
	int rc = KV_OK;

	/* Formatting is the way OUT of a corrupt or failed store, so unlike every
	 * other entry point it must work when the store is not ready.  It still needs
	 * the device itself to be alive. */
	if (!nor_flash_ready())
		return KV_ERR_STATE;

	rc = nor_trylock(KV_LOCK_TIMEOUT_TICKS);
	if (rc == NOR_ERR_BUSY)
		return KV_ERR_BUSY;
	if (rc != NOR_OK)
		return KV_ERR_STATE;

	part = fal_partition_find(KV_PART_NAME);
	if (part == NULL) {
		nor_unlock();
		return KV_ERR_STATE;
	}

	/*
	 * Erase the partition ourselves before building, and do NOT mistake this for
	 * redundant work just because FlashDB's format_sector() erases every sector
	 * again on its way through (fdb_kvdb.c:776).  Without this step, a format
	 * asked of a *healthy* store would do nothing at all: kv_try_init() with
	 * formatting permitted still only formats a database it cannot open, so it
	 * would simply re-open the existing one and report success while every key
	 * survived.  Wiping first is what makes "kv format" mean what it says, and it
	 * is also what makes it work from a partition FlashDB refuses to open.
	 *
	 * It is slow, and unavoidably so: 256 sector erases inside FlashDB at up to
	 * 400 ms each dominate, so the whole command takes on the order of fifteen
	 * seconds (measured ~14 s on board #2).  Our own pass is the cheap part -- it
	 * uses 64 KB block erases -- and shortening it would not be noticed.
	 */
	if (fal_partition_erase_all(part) < 0) {
		nor_unlock();
		LOG_ERR("format: erase failed");
		kv_status = KV_STATE_FAILED;
		return KV_ERR_IO;
	}

	kv_lock_broken = 0;
	err = kv_try_init(1);
	if (err == FDB_NO_ERR) {
		kv_status = KV_STATE_READY;
		kv_geom.sec_size = kvdb.parent.sec_size;
		LOG_INF("store formatted");
	} else {
		kv_status = KV_STATE_FAILED;
		LOG_ERR("format: init failed (%d)", (int)err);
		rc = KV_ERR_IO;
	}
	nor_unlock();
	return rc;
}

/* ------------------------------------------------------------------ *
 *  Iteration + usage
 * ------------------------------------------------------------------ */

/*
 * Walk the database once, optionally invoking @p cb, and always accumulating the
 * usage totals.  Both `kv list` and `kv info` need exactly this walk, and doing it
 * in one place means the two can never disagree about what is in the store.
 * Called with the device already held.
 */
static int kv_walk(const char *prefix, void *buf, uint32_t buflen,
                   kv_iter_cb cb, void *arg, struct kv_info *info)
{
	struct fdb_kv_iterator itr;
	size_t prefix_len = (prefix != NULL) ? strlen(prefix) : 0u;

	if (info != NULL) {
		info->kv_count = 0u;
		info->value_bytes = 0u;
		info->obj_bytes = 0u;
	}

	fdb_kv_iterator_init(&kvdb, &itr);
	while (fdb_kv_iterate(&kvdb, &itr)) {
		struct fdb_kv *kv = &itr.curr_kv;
		struct fdb_blob blob;
		size_t got;
		int truncated;

		if (info != NULL) {
			info->kv_count++;
			info->value_bytes += (uint32_t)kv->value_len;
			info->obj_bytes += (uint32_t)kv->len;
		}
		if (cb == NULL)
			continue;
		if (prefix_len != 0u && strncmp(kv->name, prefix, prefix_len) != 0)
			continue;

		/* fdb_blob_read() takes no lock of its own -- it is safe here only
		 * because this whole walk runs under the caller's device lock.  It
		 * also returns 0 on a failed read, which lands in the same `truncated`
		 * flag as a too-small buffer: from the callback's point of view both
		 * mean "what you were handed is not the whole value", and a listing
		 * that silently showed a short value would be worse than either. */
		fdb_kv_to_blob(kv, fdb_blob_make(&blob, buf, buflen));
		got = fdb_blob_read(&kvdb.parent, &blob);
		truncated = (got < kv->value_len) ? 1 : 0;
		if (cb(kv->name, buf, (uint32_t)got, truncated, arg) != 0)
			break;
	}
	return KV_OK;
}

int kv_foreach(const char *prefix, void *buf, uint32_t buflen,
               kv_iter_cb cb, void *arg)
{
	int rc;

	if (buf == NULL || buflen == 0u || cb == NULL)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_walk(prefix, buf, buflen, cb, arg, NULL);
	kv_op_end();
	return rc;
}

int kv_get_info(struct kv_info *info)
{
	uint8_t scratch[1];
	int rc;

	if (info == NULL)
		return KV_ERR_PARAM;

	*info = kv_geom;
	if (kv_state() != KV_STATE_READY)
		return KV_ERR_STATE;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_walk(NULL, scratch, sizeof scratch, NULL, NULL, info);
	kv_op_end();
	info->part_offset = kv_geom.part_offset;
	info->part_size = kv_geom.part_size;
	info->sec_size = kv_geom.sec_size;
	return rc;
}
