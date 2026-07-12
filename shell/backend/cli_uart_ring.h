/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_uart_ring.h
 * @brief   Minimal single-buffer byte ring for the UART backend (issue #7).
 *
 * A plain power-of-anything circular byte FIFO holding @p size-1 bytes (one slot
 * is reserved to distinguish full from empty).  The helpers are pure functions of
 * the ring state -- they take NO lock and call NO HAL/ThreadX API -- so they
 * compile and unit-test on the host (shell/test/test_uart_ring.c) without a
 * cross-toolchain.  The UART backend (cli_backend_uart.c) owns the concurrency:
 * it wraps the producer/consumer calls in a PRIMASK critical section because on
 * target the RX ring is filled from the USART1 ISR while drained by the shell
 * thread, and the TX ring is filled by two producers (the shell thread and the
 * printf retarget _write) while drained by the TxComplete ISR.
 *
 * Index discipline: cli_uart_ring_put / _put_buf advance @ref head (producer);
 * cli_uart_ring_get / _get_buf / _advance_tail advance @ref tail (consumer).
 * cli_uart_ring_contig returns the run from tail to either head or the buffer end
 * (whichever comes first) so the backend can hand a contiguous slice straight to
 * HAL_UART_Transmit_IT and advance the tail by the sent length on completion.
 */
#ifndef CLI_UART_RING_H
#define CLI_UART_RING_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Byte ring over a caller-owned buffer; stores @ref size - 1 bytes. */
struct cli_uart_ring {
	uint8_t        *buf;
	size_t          size;   /**< buffer capacity in bytes (usable depth is size-1) */
	volatile size_t head;   /**< next write index (producer) */
	volatile size_t tail;   /**< next read index (consumer) */
};

static inline void cli_uart_ring_init(struct cli_uart_ring *r, uint8_t *buf,
                                      size_t size)
{
	r->buf  = buf;
	r->size = size;
	r->head = 0;
	r->tail = 0;
}

/** Bytes currently stored (available to read).  Correct for any @ref size (the
 * explicit branch avoids relying on size_t wraparound when head < tail). */
static inline size_t cli_uart_ring_count(const struct cli_uart_ring *r)
{
	return (r->head >= r->tail) ? (r->head - r->tail)
	                            : (r->size - r->tail + r->head);
}

/** Free slots (bytes that can still be written before full). */
static inline size_t cli_uart_ring_free(const struct cli_uart_ring *r)
{
	return r->size - 1u - cli_uart_ring_count(r);
}

/** Store one byte; returns 1 if stored, 0 if the ring was full. */
static inline int cli_uart_ring_put(struct cli_uart_ring *r, uint8_t b)
{
	size_t next = (r->head + 1u) % r->size;
	if (next == r->tail)
		return 0;               /* full */
	r->buf[r->head] = b;
	r->head = next;
	return 1;
}

/** Pop one byte into @p b; returns 1 if a byte was read, 0 if empty. */
static inline int cli_uart_ring_get(struct cli_uart_ring *r, uint8_t *b)
{
	if (r->tail == r->head)
		return 0;               /* empty */
	*b = r->buf[r->tail];
	r->tail = (r->tail + 1u) % r->size;
	return 1;
}

/** Store up to @p n bytes; returns the count accepted (0..n). */
static inline size_t cli_uart_ring_put_buf(struct cli_uart_ring *r,
                                           const uint8_t *d, size_t n)
{
	size_t i = 0;
	while (i < n && cli_uart_ring_put(r, d[i]))
		i++;
	return i;
}

/** Drain up to @p cap bytes into @p d; returns the count copied (0..cap). */
static inline size_t cli_uart_ring_get_buf(struct cli_uart_ring *r,
                                           uint8_t *d, size_t cap)
{
	size_t i = 0;
	while (i < cap && cli_uart_ring_get(r, &d[i]))
		i++;
	return i;
}

/**
 * Largest contiguous readable run starting at @ref tail, capped at the buffer
 * end (so the slice never wraps).  On return *@p p points at &buf[tail] and the
 * run length is returned (0 when empty).  Hand the slice to HAL_UART_Transmit_IT
 * and call cli_uart_ring_advance_tail() by the transferred length on completion.
 */
static inline size_t cli_uart_ring_contig(const struct cli_uart_ring *r,
                                          const uint8_t **p)
{
	size_t head = r->head, tail = r->tail;
	size_t run  = (head >= tail) ? (head - tail) : (r->size - tail);
	*p = &r->buf[tail];
	return run;
}

/** Advance the read index by @p n bytes (consumer side of cli_uart_ring_contig). */
static inline void cli_uart_ring_advance_tail(struct cli_uart_ring *r, size_t n)
{
	r->tail = (r->tail + n) % r->size;
}

#ifdef __cplusplus
}
#endif

#endif /* CLI_UART_RING_H */
