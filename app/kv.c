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
 *  Record format
 * ------------------------------------------------------------------ *
 * What FlashDB stores under a key is not the bare value but a record carrying the
 * value's type and a human description alongside it.  That is the point of the
 * store: what a setting means lives next to the setting on the external flash, not
 * in a table compiled into the firmware, so the internal flash holds only this
 * pack/unpack and nothing that has to be reflashed when a setting is added.
 *
 *   offset 0   magic     u32   identifies our records and rejects everything else
 *          4   ver       u8    format version; an unknown one is refused, not guessed
 *          5   type      u8    KV_TYPE_*
 *          6   desc_len  u16
 *          8   total_len u32   whole record INCLUDING this 12-byte header
 *         12   desc[desc_len]  UTF-8, no terminator
 *              value[total_len - 12 - desc_len]
 *
 * Everything is packed and unpacked BYTE BY BYTE in little-endian order.  The
 * struct is never overlaid on the flash image: doing that would make the on-flash
 * layout depend on the compiler's padding and alignment rules, i.e. on a build
 * detail, for a format that has to survive firmware changes.
 */
#define KV_REC_MAGIC   0x31564B57u   /* bytes 'W','K','V','1' in memory order */
#define KV_REC_VER     1u
#define KV_REC_HDR     12u
#define KV_REC_MAX     (KV_REC_HDR + KV_DESC_MAX + KV_VALUE_MAX)

static void kv_put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void kv_put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t kv_get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t kv_get_u32_le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int kv_type_ok(uint8_t type)
{
	return type == KV_TYPE_STR || type == KV_TYPE_U32 ||
	       type == KV_TYPE_BOOL || type == KV_TYPE_BYTES;
}

const char *kv_type_name(uint8_t type)
{
	switch (type) {
	case KV_TYPE_STR:   return "str";
	case KV_TYPE_U32:   return "u32";
	case KV_TYPE_BOOL:  return "bool";
	case KV_TYPE_BYTES: return "bytes";
	default:            return "?";
	}
}

/* Types that fix their own length are checked here, so a u32 can never be stored
 * as three bytes and read back as garbage later. */
static int kv_value_len_ok(uint8_t type, uint32_t len)
{
	if (len == 0u || len > KV_VALUE_MAX)
		return 0;
	if (type == KV_TYPE_U32)
		return len == 4u;
	if (type == KV_TYPE_BOOL)
		return len == 1u;
	return 1;
}

/*
 * Content check for the types that constrain their own bytes, applied on the way
 * in AND on the way out.  Only KV_TYPE_BOOL does: kv.h promises callers a value of
 * 0 or 1, and kv_get_bool() hands back `data[0] != 0`, so a stored 2 would read as
 * a perfectly ordinary `true` -- a record this firmware could not have written,
 * accepted as if it had.  Checking the length alone would let that through.
 */
static int kv_value_bytes_ok(uint8_t type, const uint8_t *val, uint32_t len)
{
	if (!kv_value_len_ok(type, len))
		return 0;
	if (type == KV_TYPE_BOOL)
		return val[0] <= 1u;
	return 1;
}

/* Build a record.  Returns its length, or 0 if the inputs do not fit. */
static uint32_t kv_pack(uint8_t *out, uint8_t type, const void *val, uint32_t vlen,
                        const char *desc)
{
	uint32_t dlen = (desc != NULL) ? (uint32_t)strlen(desc) : 0u;
	uint32_t total;

	if (!kv_type_ok(type) || !kv_value_bytes_ok(type, val, vlen) ||
	    dlen > KV_DESC_MAX)
		return 0u;
	total = KV_REC_HDR + dlen + vlen;

	kv_put_u32(out + 0, KV_REC_MAGIC);
	out[4] = (uint8_t)KV_REC_VER;
	out[5] = type;
	kv_put_u16(out + 6, (uint16_t)dlen);
	kv_put_u32(out + 8, total);
	if (dlen != 0u)
		memcpy(out + KV_REC_HDR, desc, dlen);
	memcpy(out + KV_REC_HDR + dlen, val, vlen);
	return total;
}

/*
 * Decode a record into @p out.
 *
 * THE ORDER OF THESE CHECKS IS LOAD-BEARING.  Every length here is unsigned, so a
 * subtraction that goes negative wraps to something enormous instead of failing --
 * which is exactly how a corrupt record turns into an out-of-bounds read.  So each
 * step only subtracts quantities an earlier step has already proved large enough:
 * the header is present before any field is read, total_len is confirmed to equal
 * the real length before anything is subtracted from it, and desc_len is bounded
 * against the remainder before the value length is derived from it.
 */
static int kv_unpack(const uint8_t *raw, uint32_t rawlen, struct kv_value *out)
{
	uint32_t total, dlen, vlen;
	uint8_t type;

	out->type = KV_TYPE_INVALID;
	out->len = 0u;
	out->data[0] = '\0';
	out->desc[0] = '\0';

	if (rawlen < KV_REC_HDR)                        /* 1: header readable? */
		return KV_ERR_FORMAT;
	if (kv_get_u32_le(raw) != KV_REC_MAGIC)         /* 2: ours?            */
		return KV_ERR_FORMAT;
	if (raw[4] != (uint8_t)KV_REC_VER)
		return KV_ERR_FORMAT;
	type = raw[5];
	if (!kv_type_ok(type))
		return KV_ERR_FORMAT;
	total = kv_get_u32_le(raw + 8);
	if (total != rawlen)                            /* 3: self-consistent? */
		return KV_ERR_FORMAT;
	dlen = kv_get_u16(raw + 6);
	if (dlen > total - KV_REC_HDR)                  /* 4: safe to subtract */
		return KV_ERR_FORMAT;
	vlen = total - KV_REC_HDR - dlen;               /* 5: now derivable    */
	if (dlen > KV_DESC_MAX ||
	    !kv_value_bytes_ok(type, raw + KV_REC_HDR + dlen, vlen))
		return KV_ERR_FORMAT;

	out->type = type;
	out->len = vlen;
	memcpy(out->data, raw + KV_REC_HDR + dlen, vlen);
	out->data[vlen] = '\0';
	if (dlen != 0u)
		memcpy(out->desc, raw + KV_REC_HDR, dlen);
	out->desc[dlen] = '\0';
	return KV_OK;
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

/*
 * Raw record I/O.  Both assume the device lock is already held.
 *
 * Reading deliberately avoids fdb_kv_get_blob(): that call funnels into get_kv(),
 * which ignores what _fdb_flash_read() returned (fdb_kvdb.c:636-638) and reports
 * the requested length either way -- so a value the flash failed to hand over would
 * come back as a successful read of a stale buffer.  Looking the key up and reading
 * the blob explicitly costs one extra call and gets an answer that is either the
 * stored bytes or an error, because fdb_blob_read() returns 0 on a failed read
 * (fdb_utils.c:243-246).
 */
static int kv_raw_get(const char *key, uint8_t *raw, uint32_t rawsize,
                      uint32_t *rawlen)
{
	struct fdb_blob blob;
	struct fdb_kv kv;
	size_t got;

	*rawlen = 0u;
	if (fdb_kv_get_obj(&kvdb, key, &kv) == NULL || kv.value_len == 0u)
		return KV_ERR_NOTFOUND;
	if (kv.value_len > rawsize)
		return KV_ERR_FORMAT;   /* longer than any record we could have written */

	fdb_kv_to_blob(&kv, fdb_blob_make(&blob, raw, rawsize));
	got = fdb_blob_read(&kvdb.parent, &blob);
	if (got != blob.saved.len)
		return KV_ERR_IO;
	*rawlen = (uint32_t)got;
	return KV_OK;
}

static int kv_raw_set(const char *key, const uint8_t *raw, uint32_t rawlen)
{
	struct fdb_blob blob;

	return kv_map_err(fdb_kv_set_blob(&kvdb, key,
	                                  fdb_blob_make(&blob, raw, rawlen)));
}

/*
 * Write a value, optionally carrying the existing description forward.
 *
 * Read-modify-write, and the read half is why this holds one lock across both:
 * the description belongs to the key, not to the caller, so `kv set` on a
 * documented setting must not quietly drop the documentation, and a concurrent
 * `kv desc` must not be able to slip between the read and the write.
 */
int kv_set(const char *key, uint8_t type, const void *val, uint32_t len,
           const char *desc)
{
	uint8_t raw[KV_REC_MAX];
	char kept[KV_DESC_MAX + 1u];
	uint32_t rawlen;
	int rc;

	if (!kv_key_ok(key) || val == NULL || !kv_type_ok(type))
		return KV_ERR_PARAM;
	/* Reject before taking the device: kv_pack() would refuse it anyway, but
	 * only after a lock and a read-modify-write's worth of work. */
	if (!kv_value_bytes_ok(type, (const uint8_t *)val, len))
		return KV_ERR_PARAM;
	if (desc != NULL && strlen(desc) > KV_DESC_MAX)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;

	if (desc == NULL) {
		/* Inherit whatever description is already there.  A record we cannot
		 * decode simply contributes none -- overwriting it is what the caller
		 * asked for, and refusing would leave a bad record unfixable. */
		struct kv_value old;

		kept[0] = '\0';
		if (kv_raw_get(key, raw, sizeof raw, &rawlen) == KV_OK &&
		    kv_unpack(raw, rawlen, &old) == KV_OK)
			memcpy(kept, old.desc, strlen(old.desc) + 1u);
		desc = kept;
	}

	rawlen = kv_pack(raw, type, val, len, desc);
	rc = (rawlen == 0u) ? KV_ERR_PARAM : kv_raw_set(key, raw, rawlen);
	kv_op_end();
	return rc;
}

int kv_get(const char *key, struct kv_value *out)
{
	uint8_t raw[KV_REC_MAX];
	uint32_t rawlen;
	int rc;

	if (!kv_key_ok(key) || out == NULL)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_raw_get(key, raw, sizeof raw, &rawlen);
	if (rc == KV_OK)
		rc = kv_unpack(raw, rawlen, out);
	else
		out->type = KV_TYPE_INVALID;
	kv_op_end();
	return rc;
}

int kv_set_desc(const char *key, const char *desc)
{
	uint8_t raw[KV_REC_MAX];
	struct kv_value cur;
	uint32_t rawlen;
	int rc;

	if (!kv_key_ok(key) || desc == NULL || strlen(desc) > KV_DESC_MAX)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_raw_get(key, raw, sizeof raw, &rawlen);
	if (rc == KV_OK)
		rc = kv_unpack(raw, rawlen, &cur);
	if (rc == KV_OK) {
		/* Rebuild from the decoded value so the new record is as well-formed as
		 * a freshly written one -- patching the description in place would have
		 * to move the value and rewrite the lengths anyway. */
		rawlen = kv_pack(raw, cur.type, cur.data, cur.len, desc);
		rc = (rawlen == 0u) ? KV_ERR_FORMAT : kv_raw_set(key, raw, rawlen);
	}
	kv_op_end();
	return rc;
}

/* ---- typed readers ------------------------------------------------------- */

int kv_get_str(const char *key, char *buf, uint32_t buflen)
{
	struct kv_value v;
	int rc;

	if (buf == NULL || buflen == 0u)
		return KV_ERR_PARAM;
	rc = kv_get(key, &v);
	if (rc != KV_OK)
		return rc;
	if (v.type != KV_TYPE_STR)
		return KV_ERR_TYPE;
	if (v.len + 1u > buflen)
		return KV_ERR_TRUNC;
	memcpy(buf, v.data, v.len + 1u);
	return KV_OK;
}

int kv_get_u32(const char *key, uint32_t *out)
{
	struct kv_value v;
	int rc;

	if (out == NULL)
		return KV_ERR_PARAM;
	rc = kv_get(key, &v);
	if (rc != KV_OK)
		return rc;
	if (v.type != KV_TYPE_U32)
		return KV_ERR_TYPE;
	*out = kv_get_u32_le(v.data);
	return KV_OK;
}

int kv_get_bool(const char *key, int *out)
{
	struct kv_value v;
	int rc;

	if (out == NULL)
		return KV_ERR_PARAM;
	rc = kv_get(key, &v);
	if (rc != KV_OK)
		return rc;
	if (v.type != KV_TYPE_BOOL)
		return KV_ERR_TYPE;
	*out = (v.data[0] != 0u);
	return KV_OK;
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
/*
 * Scratch for the walk.  These are file-scope rather than automatic because a walk
 * needs a whole record plus its decoded form (~660 B together) and would otherwise
 * charge that to whichever thread happened to call `kv list`.  They are safe
 * unguarded for the same reason the log shim's buffer is: kv_walk() only ever runs
 * with the device lock held, so there is never a second thread inside it.
 */
static uint8_t kv_walk_raw[KV_REC_MAX];
static struct kv_value kv_walk_val;

static int kv_walk(const char *prefix, kv_iter_cb cb, void *arg,
                   struct kv_info *info)
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
		 * because this whole walk runs under the caller's device lock. */
		fdb_kv_to_blob(kv, fdb_blob_make(&blob, kv_walk_raw,
		                                 sizeof kv_walk_raw));
		got = fdb_blob_read(&kvdb.parent, &blob);
		/* A short or failed read, an oversized record and a record we cannot
		 * decode all end up as KV_TYPE_INVALID.  The callback still gets called:
		 * a listing that skipped what it could not read would make a store that
		 * is half-corrupt look healthy, which is the one impression it must
		 * never give. */
		if (got != blob.saved.len ||
		    kv_unpack(kv_walk_raw, (uint32_t)got, &kv_walk_val) != KV_OK) {
			kv_walk_val.type = KV_TYPE_INVALID;
			kv_walk_val.len = 0u;
			kv_walk_val.data[0] = '\0';
			kv_walk_val.desc[0] = '\0';
		}
		if (cb(kv->name, &kv_walk_val, arg) != 0)
			break;
	}
	return KV_OK;
}

int kv_foreach(const char *prefix, kv_iter_cb cb, void *arg)
{
	int rc;

	if (cb == NULL)
		return KV_ERR_PARAM;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_walk(prefix, cb, arg, NULL);
	kv_op_end();
	return rc;
}

int kv_get_info(struct kv_info *info)
{
	int rc;

	if (info == NULL)
		return KV_ERR_PARAM;

	*info = kv_geom;
	if (kv_state() != KV_STATE_READY)
		return KV_ERR_STATE;

	rc = kv_op_begin();
	if (rc != KV_OK)
		return rc;
	rc = kv_walk(NULL, NULL, NULL, info);
	kv_op_end();
	info->part_offset = kv_geom.part_offset;
	info->part_size = kv_geom.part_size;
	info->sec_size = kv_geom.sec_size;
	return rc;
}
