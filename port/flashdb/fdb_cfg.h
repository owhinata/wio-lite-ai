/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fdb_cfg.h
 * @brief   FlashDB build configuration for this firmware (issue #37).
 *
 * FlashDB includes this by name (`#include <fdb_cfg.h>`); it is the only place its
 * compile-time behaviour is set.  Everything here is a deliberate choice -- see
 * port/flashdb/fal_nor.c for the storage side and app/kv.c for how the database is
 * driven.
 */
#ifndef _FDB_CFG_H_
#define _FDB_CFG_H_

/* Key-value database only.  The time-series database (FDB_USING_TSDB) is a
 * separate feature with its own sector format and its own ~4 KB of code; nothing
 * here logs samples, and leaving it out keeps fdb_tsdb.c an empty object. */
#define FDB_USING_KVDB

/* Storage through FAL, i.e. through port/flashdb/fal_nor.c onto the external NOR.
 * The file-backed modes (LIBC/POSIX) need a filesystem this database must not
 * depend on -- the configuration has to be readable before anything mounts. */
#define FDB_USING_FAL_MODE

/*
 * Write granularity in BITS.  8 = one byte, and this is the single most
 * consequential line in the file.
 *
 * FlashDB marks a record's state by writing a status field, and _fdb_set_status()
 * behaves differently depending on this value (src/fdb_utils.c):
 *
 *   gran = 1  status advances 0xFF -> 0x7F -> 0x3F -> 0x1F, clearing one more bit
 *             of THE SAME byte each time -- i.e. it re-programs a byte that has
 *             already been programmed.
 *   gran = 8  each status gets its OWN byte: 0xFF FF FF -> 0x00 FF FF -> 0x00 00 FF.
 *             Every write targets a byte that still reads 0xFF.
 *
 * The W25Q128JV datasheet specifies page programming "at previously erased (FFh)
 * memory locations" (sec 8.2.13).  Read carefully, that constrains the *state* of
 * the target bytes, not the *history* of the page -- so gran = 8 stays entirely
 * inside what the datasheet promises, while gran = 1 depends on behaviour it does
 * not describe.  FlashDB's own comment agrees: "some flash (like stm32 onchip) NOT
 * supported repeated write before erase".
 *
 * The cost is 4 extra bytes per KV node and per sector header (the status table
 * formula differs between gran = 1 and gran != 1, inc/fdb_low_lvl.h: six statuses
 * take 1 byte at gran 1 and 5 bytes at gran 8).  Since 8 bits is byte alignment,
 * FDB_WG_ALIGN() does not pad names or values at all.
 *
 * `nor test` measured this device accepting a same-byte overwrite too, so gran = 1
 * would probably work here -- but "probably, on this chip, today" is not the
 * footing to put a configuration store on, and the price of certainty is 4 bytes.
 *
 * port/flashdb/fal_nor.c static-asserts that the FAL device agrees with this
 * value: fal_partition_write() passes its own write_gran field straight through
 * without checking it, so a mismatch would otherwise go undetected.
 */
#define FDB_WRITE_GRAN                 8

/* Longest key.  The keys this stores are dotted paths like "net.shell.autoarm";
 * 32 leaves room and shrinks fdb_kvdb (whose iterator and current-KV structs both
 * embed a name buffer) against the 64-byte default. */
#define FDB_KV_NAME_MAX                32

/* Lookup caches (both must be non-zero to enable caching at all).  16 KV slots
 * covers the whole key space defined for this store with room to spare, and 4
 * sector slots covers the handful of sectors a store this small ever writes.
 * These only save flash reads; correctness does not depend on them. */
#define FDB_KV_CACHE_TABLE_SIZE        16
#define FDB_SECTOR_CACHE_TABLE_SIZE    4

/* Little-endian: do NOT define FDB_BIG_ENDIAN.  The Cortex-M7 runs little-endian
 * here and app/kv.c packs its records the same way. */

/*
 * Diagnostics.  FlashDB emits its messages as several FDB_PRINT() fragments per
 * line, so this points at a line-assembling shim (port/flashdb/fal_nor.c) that
 * feeds whole lines into the reset-persistent log ring -- readable with `dmesg`,
 * and never written to a console that a user might be typing into.
 *
 * FDB_DEBUG_ENABLE stays off: it prefixes every message with __FILE__, which
 * would drag the full source paths into the image for messages nobody reads.
 */
void kv_fdb_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define FDB_PRINT(...)                 kv_fdb_print(__VA_ARGS__)

/*
 * Assertions.  The stock macro ends in `while (1)`, which on this board means the
 * IWDG resets it a couple of seconds later -- taking down the shell, the console
 * and any chance of asking what happened.  That trade is wrong here: every
 * FDB_ASSERT in the KVDB path guards a programming error (a null handle, an
 * initialisation-order mistake, our own sector alignment), never a data-dependent
 * invariant, so none of them can be tripped by a corrupt flash image.  Record it,
 * latch the database as wedged so `kv` refuses instead of pretending, and let the
 * rest of the firmware keep running.
 *
 * Be clear about what the latch does and does not buy: it stops the NEXT call, not
 * the current one.  Execution continues from the failed assertion into whatever the
 * function would have done anyway, exactly as it would with the C library's
 * NDEBUG-disabled assert.  That is an accepted trade -- app/kv.c validates every
 * argument it passes down, so the guarded conditions are unreachable from here --
 * and not an oversight to be relied on the other way round.
 *
 * Note the shape.  FlashDB writes FDB_ASSERT both with and without a trailing
 * semicolon (`FDB_ASSERT(db);` at fdb.c:33, `FDB_ASSERT(0)` and the
 * FDB_GC_EMPTY_SEC_THRESHOLD check at fdb_kvdb.c:1781 without one), so the
 * expansion has to be a complete statement on its own.  That rules out the usual
 * `do { ... } while (0)` wrapper: the trailing semicolon is part of do-while's
 * grammar, not an optional separator, so without one the next statement is a
 * syntax error.  The bare `if` below matches what the stock macro does and carries
 * the same (statement-level only) usage requirement.
 */
void kv_fdb_assert_failed(const char *expr, const char *func);
/** Nonzero once any FlashDB assertion has fired; app/kv.c latches on it. */
int kv_fdb_assert_tripped(void);
#define FDB_ASSERT(EXPR)                                                       \
	if (!(EXPR)) {                                                             \
		kv_fdb_assert_failed(#EXPR, __func__);                                 \
	}

#endif /* _FDB_CFG_H_ */
