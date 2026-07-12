/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    log.c
 * @brief   DTCM RAM log ring: reset-persistent, ISR/fault-safe.
 *
 * The ring lives in the .log_noinit section, mapped to the DTCM (0x20000000) by
 * the linker.  The DTCM sits on the Cortex-M7 TCM interface and bypasses the
 * D-cache, so a record written from fault context is committed to SRAM
 * immediately (no cache maintenance) and survives every *system* reset (reboot /
 * fault / IWDG / WWDG / NRST / LPWR, RM0468 §8.4.4).  The CMSIS Reset_Handler
 * initialises only .data/.bss, never .log_noinit, so the bytes carry over; a
 * *power* reset (POR/PDR/BOR) loses them, which log_init() detects via the magic
 * and re-initialises.
 *
 * Layout: a 32-byte header (magic/version/size/head/tail/seq/boot_count) then a
 * power-of-two data[] of variable-length records.  head/tail are free-running
 * 32-bit byte offsets (indexed with & (size-1)); empty == head==tail, full ==
 * head-tail==size.  A record never straddles the physical end of data[]: when it
 * would, a SKIP record fills the tail fragment and the record wraps to offset 0.
 * Oldest-overwrite evicts whole records from tail until the new one fits.
 *
 * Concurrency: the ring update runs under a PRIMASK critical section (no tx_*
 * call), so thread, ISR and fault context all share it safely.  Formatting into
 * a stack buffer happens before the section to keep interrupts disabled only for
 * the bounded copy + head commit.
 *
 * Clean-room: concept from NuttX ramlog / Zephyr logging; no code reused.
 */
#include "log.h"

#include <stddef.h>
#include <string.h>

#include "fmt.h"
#include "stm32h7xx_hal.h"

/* ---- on-DTCM ring ------------------------------------------------------ */

#define LOG_MAGIC       0x474F4C31u     /* "1LOG" */
#define LOG_VERSION     1u
#define LOG_REC_MAGIC   0xA5u           /* per-record sanity byte */
#define LOG_LEVEL_SKIP  0xFFu           /* fills the tail fragment on wrap */

#ifndef LOG_RING_DATA_SIZE
#define LOG_RING_DATA_SIZE 8192u        /* power of two: see below */
#endif

/* Fixed header preceding the text of every (non-SKIP) record.  Naturally packed
 * to exactly 20 bytes; built on the stack then memcpy'd into the ring. */
struct log_rec_hdr {
	uint16_t total_len;     /* whole record incl. header, multiple of 4 */
	uint8_t  level;
	uint8_t  rmagic;        /* LOG_REC_MAGIC; sanity for the boot walk */
	uint32_t ts_ms;
	uint32_t seq;
	char     tag[LOG_TAG_MAX];      /* NUL-padded, not necessarily terminated */
};

#define LOG_HDR_SIZE  ((uint32_t)sizeof(struct log_rec_hdr))     /* 20 */
#define LOG_REC_MIN   (LOG_HDR_SIZE + 4u)                        /* 24 */
/* Largest record: header + padded (text + NUL). */
#define LOG_REC_MAX   (LOG_HDR_SIZE + (((LOG_MSG_MAX + 1u) + 3u) & ~3u))

_Static_assert(LOG_HDR_SIZE == 20u, "log_rec_hdr must be 20 bytes");
_Static_assert((LOG_RING_DATA_SIZE & (LOG_RING_DATA_SIZE - 1u)) == 0u,
               "LOG_RING_DATA_SIZE must be a power of two");
_Static_assert(LOG_RING_DATA_SIZE >= 2u * LOG_REC_MAX,
               "ring must hold a SKIP fragment plus a max record");

struct log_ring {
	uint32_t magic;
	uint32_t version;
	uint32_t size;          /* == sizeof(data); detects a build-time resize */
	uint32_t head;          /* free-running write offset */
	uint32_t tail;          /* free-running oldest-record offset */
	uint32_t seq;           /* next record's sequence number */
	uint32_t boot_count;
	uint32_t reserved;      /* pad header to 32 bytes */
	uint8_t  data[LOG_RING_DATA_SIZE];
};

_Static_assert(offsetof(struct log_ring, data) == 32u,
               "ring header must be 32 bytes");

static struct log_ring g_log __attribute__((section(".log_noinit"), aligned(4)));

/* Writes are dropped until log_init() validates the ring; clears the early-fault
 * window (a fault before log_init() must not touch an unverified ring). */
static volatile uint8_t  log_ready;
static volatile uint8_t  log_level = LOG_LEVEL_INF;     /* run-time threshold */

/* ---- PRIMASK critical section (same shape as the UART backend) ---------- */

#define LOG_CRIT_ENTER()  do { uint32_t _pm = __get_PRIMASK(); __disable_irq()
#define LOG_CRIT_EXIT()   __set_PRIMASK(_pm); } while (0)

/* ---- straddle-safe ring copy (data[] is circular) ---------------------- */

static void ring_put(uint32_t off, const void *src, uint32_t len)
{
	uint32_t p = off & (LOG_RING_DATA_SIZE - 1u);
	uint32_t first = LOG_RING_DATA_SIZE - p;
	if (first >= len) {
		memcpy(&g_log.data[p], src, len);
	} else {
		memcpy(&g_log.data[p], src, first);
		memcpy(&g_log.data[0], (const uint8_t *)src + first, len - first);
	}
}

static void ring_get(uint32_t off, void *dst, uint32_t len)
{
	uint32_t p = off & (LOG_RING_DATA_SIZE - 1u);
	uint32_t first = LOG_RING_DATA_SIZE - p;
	if (first >= len) {
		memcpy(dst, &g_log.data[p], len);
	} else {
		memcpy(dst, &g_log.data[p], first);
		memcpy((uint8_t *)dst + first, &g_log.data[0], len - first);
	}
}

/* Read just the leading length word of the record at free-running @p off. */
static uint16_t rec_total_at(uint32_t off)
{
	uint16_t total;
	ring_get(off, &total, sizeof total);
	return total;
}

/* ---- init / reset cause ------------------------------------------------ */

/* RCC->RSR snapshot from log_init(), kept because log_init() clears the flags
 * (RMVF) right after reading them -- log_reset_cause() reads this back to show
 * what caused this boot (e.g. "IWDG").  H7 uses RCC->RSR (RM0468 §8.4.4); the
 * f7 sibling used RCC->CSR. */
static uint32_t g_reset_rsr;

static const char *reset_cause_str(uint32_t rsr)
{
	if (rsr & RCC_RSR_LPWRRSTF)  return "LPWR";
	if (rsr & RCC_RSR_WWDG1RSTF) return "WWDG";
	if (rsr & RCC_RSR_IWDG1RSTF) return "IWDG";
	if (rsr & RCC_RSR_SFTRSTF)   return "SFT";
	if (rsr & RCC_RSR_PORRSTF)   return "POR";
	if (rsr & RCC_RSR_BORRSTF)   return "BOR";
	if (rsr & RCC_RSR_PINRSTF)   return "PIN";
	return "?";
}

void log_init(void)
{
	uint32_t rsr = RCC->RSR;         /* capture before clearing the flags */
	RCC->RSR |= RCC_RSR_RMVF;        /* clear so the next boot reads its own cause */
	g_reset_rsr = rsr;               /* keep for log_reset_cause() */

	int valid = (g_log.magic == LOG_MAGIC) &&
	            (g_log.version == LOG_VERSION) &&
	            (g_log.size == LOG_RING_DATA_SIZE) &&
	            ((g_log.head & 3u) == 0u) && ((g_log.tail & 3u) == 0u) &&
	            ((uint32_t)(g_log.head - g_log.tail) <= g_log.size);

	if (valid) {
		/* Walk tail->head; truncate head at the first malformed record so a
		 * write interrupted by the reset costs only its own trailing record. */
		uint32_t pos = g_log.tail;
		while (pos != g_log.head) {
			if ((uint32_t)(g_log.head - pos) > g_log.size)
				break;                          /* safety: runaway */
			uint32_t w0;
			ring_get(pos, &w0, sizeof w0);
			uint16_t total = (uint16_t)(w0 & 0xFFFFu);
			uint8_t  level = (uint8_t)((w0 >> 16) & 0xFFu);
			uint8_t  rmag  = (uint8_t)((w0 >> 24) & 0xFFu);
			uint32_t p     = pos & (LOG_RING_DATA_SIZE - 1u);

			if (rmag != LOG_REC_MAGIC || (total & 3u))
				break;
			if (level == LOG_LEVEL_SKIP) {
				if (total < 4u || (p + total) != LOG_RING_DATA_SIZE)
					break;                      /* SKIP must reach the end */
			} else {
				if (total < LOG_REC_MIN || total > LOG_REC_MAX ||
				    (p + total) > LOG_RING_DATA_SIZE)
					break;
			}
			pos += total;
		}
		g_log.head = pos;
		g_log.boot_count += 1u;
	} else {
		memset(&g_log, 0, sizeof g_log);
		g_log.magic      = LOG_MAGIC;
		g_log.version    = LOG_VERSION;
		g_log.size       = LOG_RING_DATA_SIZE;
		g_log.boot_count = 1u;
	}

	log_level = LOG_LEVEL_INF;
	log_ready = 1u;                 /* enable writes BEFORE the boot marker */

	log_write(LOG_LEVEL_INF, "boot", "#%lu reset cause: %s",
	          (unsigned long)g_log.boot_count, reset_cause_str(rsr));
}

const char *log_reset_cause(void)
{
	return reset_cause_str(g_reset_rsr);
}

/* ---- append ------------------------------------------------------------ */

void log_vwrite(unsigned level, const char *tag, const char *fmt, va_list ap)
{
	if (level > LOG_LEVEL_DBG)
		level = LOG_LEVEL_DBG;
	if (!log_ready || level > log_level)
		return;

	char text[LOG_MSG_MAX + 1];
	int  n = fmt_vsnformat(text, sizeof text, fmt, ap);
	uint32_t stored  = (uint32_t)(n < 0 ? 0 : n) + 1u;      /* incl. NUL */
	uint32_t padded  = (stored + 3u) & ~3u;
	uint32_t rec_len = LOG_HDR_SIZE + padded;               /* 24..LOG_REC_MAX */

	struct log_rec_hdr h;
	h.total_len = (uint16_t)rec_len;
	h.level     = (uint8_t)level;
	h.rmagic    = LOG_REC_MAGIC;
	h.ts_ms     = HAL_GetTick();
	memset(h.tag, 0, sizeof h.tag);
	for (uint32_t i = 0; i < LOG_TAG_MAX && tag && tag[i]; i++)
		h.tag[i] = tag[i];

	LOG_CRIT_ENTER();
	{
		h.seq = g_log.seq;

		uint32_t o    = g_log.head & (LOG_RING_DATA_SIZE - 1u);
		uint32_t skip = (o + rec_len > LOG_RING_DATA_SIZE)
		                    ? (LOG_RING_DATA_SIZE - o) : 0u;
		uint32_t need = skip + rec_len;

		/* Evict whole records from tail until the new one (plus any SKIP) fits. */
		while ((uint32_t)(g_log.size - (g_log.head - g_log.tail)) < need &&
		       g_log.tail != g_log.head) {
			uint16_t tl = rec_total_at(g_log.tail);
			if (tl < 4u || (tl & 3u)) {     /* corrupt: drop everything */
				g_log.tail = g_log.head;
				break;
			}
			g_log.tail += tl;
		}

		if (skip) {
			uint32_t sw = (uint32_t)skip
			            | ((uint32_t)LOG_LEVEL_SKIP << 16)
			            | ((uint32_t)LOG_REC_MAGIC << 24);
			ring_put(g_log.head, &sw, sizeof sw);
			__DMB();
			g_log.head += skip;
		}

		ring_put(g_log.head, &h, LOG_HDR_SIZE);
		ring_put(g_log.head + LOG_HDR_SIZE, text, stored);
		if (padded > stored) {
			static const uint8_t zeros[4] = { 0, 0, 0, 0 };
			ring_put(g_log.head + LOG_HDR_SIZE + stored, zeros, padded - stored);
		}
		/* Bump seq before committing head: a reset between the two then leaves a
		 * gap in the sequence (the lost record is invisible), never a duplicate. */
		g_log.seq = h.seq + 1u;
		__DMB();                                /* body + seq visible before head */
		g_log.head += rec_len;
	}
	LOG_CRIT_EXIT();
}

void log_write(unsigned level, const char *tag, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	log_vwrite(level, tag, fmt, ap);
	va_end(ap);
}

/* ---- query / control --------------------------------------------------- */

void log_clear(void)
{
	LOG_CRIT_ENTER();
	g_log.tail = g_log.head;        /* seq keeps counting across a clear */
	LOG_CRIT_EXIT();
}

void log_set_level(unsigned level)
{
	if (level > LOG_LEVEL_DBG)
		level = LOG_LEVEL_DBG;
	log_level = (uint8_t)level;
}

unsigned log_get_level(void)
{
	return log_level;
}

/* ---- dmesg iterator ---------------------------------------------------- */

void log_iter_start(struct log_iter *it)
{
	LOG_CRIT_ENTER();
	it->pos = g_log.tail;
	it->end = g_log.head;           /* snapshot: do not chase a live tail */
	LOG_CRIT_EXIT();
}

int log_iter_next(struct log_iter *it, struct log_record *out)
{
	int got = 0;

	LOG_CRIT_ENTER();
	for (;;) {
		/* Resync if eviction overtook us since the last record. */
		if ((int32_t)(it->pos - g_log.tail) < 0)
			it->pos = g_log.tail;
		/* Stop once we reach the snapshot end (or run past it). */
		if ((int32_t)(it->pos - it->end) >= 0)
			break;

		uint32_t w0;
		ring_get(it->pos, &w0, sizeof w0);
		uint16_t total = (uint16_t)(w0 & 0xFFFFu);
		uint8_t  level = (uint8_t)((w0 >> 16) & 0xFFu);
		uint8_t  rmag  = (uint8_t)((w0 >> 24) & 0xFFu);
		uint32_t p     = it->pos & (LOG_RING_DATA_SIZE - 1u);

		/* Validate as strictly as the boot walk: a reset-persistent ring may be
		 * read after a corrupting event, so never trust the length blindly (a
		 * stray total < LOG_HDR_SIZE would otherwise underflow tlen below). */
		if (rmag != LOG_REC_MAGIC || total < 4u || (total & 3u))
			break;                              /* corrupt: end the pass */
		if (level == LOG_LEVEL_SKIP) {
			if ((p + total) != LOG_RING_DATA_SIZE)
				break;
			it->pos += total;
			continue;
		}
		if (total < LOG_REC_MIN || total > LOG_REC_MAX ||
		    (p + total) > LOG_RING_DATA_SIZE)
			break;

		struct log_rec_hdr h;
		ring_get(it->pos, &h, LOG_HDR_SIZE);
		out->ts_ms = h.ts_ms;
		out->seq   = h.seq;
		out->level = h.level;
		memcpy(out->tag, h.tag, LOG_TAG_MAX);
		out->tag[LOG_TAG_MAX] = '\0';

		uint32_t tlen = total - LOG_HDR_SIZE;
		if (tlen > sizeof out->text)
			tlen = sizeof out->text;
		ring_get(it->pos + LOG_HDR_SIZE, out->text, tlen);
		out->text[sizeof out->text - 1] = '\0';

		it->pos += total;
		got = 1;
		break;
	}
	LOG_CRIT_EXIT();
	return got;
}
