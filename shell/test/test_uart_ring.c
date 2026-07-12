/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the UART backend byte ring (issue #7).  Exercises the pure,
 * lock-free ring helpers (cli_uart_ring.h) without any HAL/ThreadX dependency:
 *   1. empty/full bookkeeping (count/free, depth = size-1),
 *   2. single put/get FIFO order,
 *   3. overflow drops once full and never corrupts stored bytes,
 *   4. wrap-around across the buffer boundary,
 *   5. bulk put_buf/get_buf returning the accepted/copied counts,
 *   6. contiguous-run + advance_tail (the HAL_UART_Transmit_IT slice path).
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cli_uart_ring.h"

int main(void)
{
	uint8_t buf[8];
	struct cli_uart_ring r;

	/* 1. fresh ring: empty, depth = size-1 */
	cli_uart_ring_init(&r, buf, sizeof buf);
	assert(cli_uart_ring_count(&r) == 0);
	assert(cli_uart_ring_free(&r) == sizeof(buf) - 1);

	/* 2. single put/get keeps FIFO order */
	assert(cli_uart_ring_put(&r, 'A') == 1);
	assert(cli_uart_ring_put(&r, 'B') == 1);
	assert(cli_uart_ring_count(&r) == 2);
	uint8_t b = 0;
	assert(cli_uart_ring_get(&r, &b) == 1 && b == 'A');
	assert(cli_uart_ring_get(&r, &b) == 1 && b == 'B');
	assert(cli_uart_ring_get(&r, &b) == 0);     /* empty again */
	assert(cli_uart_ring_count(&r) == 0);

	/* 3. fill to capacity (size-1), then overflow is rejected */
	cli_uart_ring_init(&r, buf, sizeof buf);
	for (int i = 0; i < (int)sizeof(buf) - 1; i++)
		assert(cli_uart_ring_put(&r, (uint8_t)i) == 1);
	assert(cli_uart_ring_free(&r) == 0);
	assert(cli_uart_ring_put(&r, 0xFF) == 0);   /* full: dropped */
	assert(cli_uart_ring_count(&r) == sizeof(buf) - 1);
	/* stored bytes intact and in order */
	for (int i = 0; i < (int)sizeof(buf) - 1; i++) {
		assert(cli_uart_ring_get(&r, &b) == 1 && b == (uint8_t)i);
	}

	/* 4. wrap-around: advance head/tail past the end, FIFO still holds */
	cli_uart_ring_init(&r, buf, sizeof buf);
	for (int round = 0; round < 4; round++) {
		uint8_t seq[5];
		for (int i = 0; i < 5; i++)
			seq[i] = (uint8_t)(round * 5 + i);
		for (int i = 0; i < 5; i++)
			assert(cli_uart_ring_put(&r, seq[i]) == 1);
		for (int i = 0; i < 5; i++) {
			assert(cli_uart_ring_get(&r, &b) == 1);
			assert(b == seq[i]);
		}
	}
	assert(cli_uart_ring_count(&r) == 0);

	/* 5. bulk put_buf clamps to free space; get_buf clamps to available */
	cli_uart_ring_init(&r, buf, sizeof buf);
	const uint8_t src[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	size_t acc = cli_uart_ring_put_buf(&r, src, sizeof src);
	assert(acc == sizeof(buf) - 1);             /* only depth bytes accepted */
	uint8_t dst[16];
	size_t got = cli_uart_ring_get_buf(&r, dst, sizeof dst);
	assert(got == sizeof(buf) - 1);
	assert(memcmp(dst, src, got) == 0);
	assert(cli_uart_ring_count(&r) == 0);

	/* 6. contiguous run never wraps; advance_tail consumes it.  Force a wrap so
	 * the stored data straddles the buffer end and contig() returns the
	 * tail->end slice first, the wrapped remainder second. */
	cli_uart_ring_init(&r, buf, sizeof buf);
	/* push tail to index 6, leaving 2 contiguous slots before the end */
	for (int i = 0; i < 6; i++) {
		assert(cli_uart_ring_put(&r, (uint8_t)i) == 1);
		assert(cli_uart_ring_get(&r, &b) == 1);
	}
	assert(r.tail == 6 && r.head == 6);
	/* store 5 bytes: indices 6,7 then wrap to 0,1,2 */
	const uint8_t w[5] = { 10, 11, 12, 13, 14 };
	assert(cli_uart_ring_put_buf(&r, w, sizeof w) == 5);
	const uint8_t *p = NULL;
	size_t run = cli_uart_ring_contig(&r, &p);
	assert(run == 2);                           /* 6,7 to the buffer end */
	assert(p[0] == 10 && p[1] == 11);
	cli_uart_ring_advance_tail(&r, run);
	run = cli_uart_ring_contig(&r, &p);
	assert(run == 3);                           /* wrapped remainder 0,1,2 */
	assert(p[0] == 12 && p[1] == 13 && p[2] == 14);
	cli_uart_ring_advance_tail(&r, run);
	assert(cli_uart_ring_count(&r) == 0);

	printf("OK: uart ring count/free, FIFO, overflow, wrap, bulk, contig\n");
	return 0;
}
