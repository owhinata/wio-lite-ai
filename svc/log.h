/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    log.h
 * @brief   RAM log subsystem: levelled ring + dmesg + crash record.
 *
 * Messages are appended to a ring buffer that lives in the DTCM .log_noinit
 * section, so it survives every system reset (reboot / fault / IWDG / WWDG /
 * NRST / LPWR, RM0468 §8.4.4) and can be replayed after reboot with `dmesg`.  The
 * fault handler (app/fault.c) records the crash here as well, so a HardFault is
 * still visible after the next reset.  Clean-room: the *concept* is borrowed from
 * NuttX ramlog / Zephyr logging, but no code is reused.
 *
 * Usage -- define LOG_TAG before including this header, then call the macros:
 *     #define LOG_TAG "uart"
 *     #include "log.h"
 *     LOG_INF("baud=%u", baud);
 * A message is dropped at compile time when its level is above LOG_COMPILE_LEVEL,
 * and at run time when above the threshold set by log_set_level() (default INF).
 * Normal logs are NOT echoed to the console -- read them with `dmesg`.
 */
#ifndef LOG_H
#define LOG_H

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Severity levels: lower value == more severe (threshold compares with <=). */
#define LOG_LEVEL_ERR 0u
#define LOG_LEVEL_WRN 1u
#define LOG_LEVEL_INF 2u
#define LOG_LEVEL_DBG 3u

/* Compile-time floor: a level numerically above this is removed at build time.
 * Default keeps everything; override e.g. -DLOG_COMPILE_LEVEL=LOG_LEVEL_INF. */
#ifndef LOG_COMPILE_LEVEL
#define LOG_COMPILE_LEVEL LOG_LEVEL_DBG
#endif

/* Each translation unit may define its own module tag before #include "log.h". */
#ifndef LOG_TAG
#define LOG_TAG "??"
#endif

/* Longest message text and tag stored per record (each excluding the NUL). */
#define LOG_MSG_MAX 104
#define LOG_TAG_MAX 8

#define LOG_AT(lvl, ...) \
	do { \
		if ((lvl) <= LOG_COMPILE_LEVEL) \
			log_write((lvl), LOG_TAG, __VA_ARGS__); \
	} while (0)
#define LOG_ERR(...) LOG_AT(LOG_LEVEL_ERR, __VA_ARGS__)
#define LOG_WRN(...) LOG_AT(LOG_LEVEL_WRN, __VA_ARGS__)
#define LOG_INF(...) LOG_AT(LOG_LEVEL_INF, __VA_ARGS__)
#define LOG_DBG(...) LOG_AT(LOG_LEVEL_DBG, __VA_ARGS__)

/**
 * Validate (or re-initialise) the ring and record a boot marker with the reset
 * cause.  Call once from main() BEFORE fault_init() so the fault handler always
 * finds a valid ring.  log_write() is a no-op until this returns.
 */
void log_init(void);

/**
 * Append one formatted record at @p level tagged @p tag.  Safe from thread, ISR
 * and fault context (the ring update runs in a PRIMASK critical section and calls
 * no tx_* API).  No-op before log_init() or when @p level is above the run-time
 * threshold.  Console is never touched -- read records with `dmesg`.
 */
void log_write(unsigned level, const char *tag, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
void log_vwrite(unsigned level, const char *tag, const char *fmt, va_list ap);

/** Drop all stored records, keeping the sequence counter (dmesg -c). */
void log_clear(void);

/** Decoded cause of THIS boot's reset ("IWDG"/"SFT"/"POR"/...), captured by
 *  log_init() before the RCC->RSR flags are cleared; "?" if no flag was set. */
const char *log_reset_cause(void);

/** Run-time severity threshold: records above @p level are dropped (default
 *  INF).  log_get_level() returns the current threshold. */
void     log_set_level(unsigned level);
unsigned log_get_level(void);

/** One decoded record copied out for display. */
struct log_record {
	uint32_t ts_ms;                 /**< HAL_GetTick() at write time */
	uint32_t seq;                   /**< monotonic sequence number */
	uint8_t  level;                 /**< LOG_LEVEL_* */
	char     tag[LOG_TAG_MAX + 1];
	char     text[LOG_MSG_MAX + 1];
};

/** Snapshot cursor for one dmesg pass; fields are opaque to callers. */
struct log_iter {
	uint32_t pos;                   /**< next record offset (free-running) */
	uint32_t end;                   /**< head snapshot taken at iter_start */
};

/** Begin a pass over the records present now (snapshots head). */
void log_iter_start(struct log_iter *it);
/** Copy the next record into @p out; returns 1 on a record, 0 at end. */
int  log_iter_next(struct log_iter *it, struct log_record *out);

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
