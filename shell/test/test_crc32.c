/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 *
 * Host unit test for fdb_calc_crc32() AS app/blob.c USES IT (issue #10 / #9 P2b).
 *
 * The blob region stamps every stored asset with a CRC-32 computed from the YMODEM
 * stream as it arrives, chunk by chunk, and `blob verify` re-reads the flash and
 * compares.  That rests on two properties of a function this project did not write
 * and cannot change, neither of which is visible in its signature:
 *
 *   A. starting from 0 it produces standard CRC-32/ISO-HDLC -- the same number as
 *      `crc32` / Python's `zlib.crc32` on the PC, so a value printed by `blob list`
 *      can be checked against the file that was sent;
 *   B. feeding the previous result back in continues the same CRC, so a 189 KB file
 *      delivered as ~185 separate 1024-byte blocks ends up with the value it would
 *      have had in one call.
 *
 * (A) is the one that bit: FlashDB's implementation inverts at BOTH ends already
 * (fdb_utils.c: `crc = crc ^ ~0U` on entry, `return crc ^ ~0U` on exit), so the
 * usual "init 0xFFFFFFFF, complement the result" wrapper inverts twice and yields a
 * different number.  An earlier revision of the #10 plan called for exactly that
 * wrapper; it would have produced a board that disagreed with the host with no way
 * to tell whether the CRC, the transfer or the flash was at fault.  Case D pins the
 * trap so nobody re-adds the wrapper.
 *
 * This is also the ONLY part of the blob work that can be verified without the
 * board, which is why it exists at all.  Vectors cross-checked against zlib.crc32.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <flashdb.h>

/* ---- A: canonical CRC-32/ISO-HDLC vectors -------------------------------- */

static void test_vectors(void)
{
	static const struct {
		const char *s;
		uint32_t    crc;
	} v[] = {
		{ "",                                            0x00000000u },
		{ "a",                                           0xE8B7BE43u },
		{ "abc",                                         0x352441C2u },
		{ "123456789",                                   0xCBF43926u },
		{ "The quick brown fox jumps over the lazy dog", 0x414FA339u },
	};
	size_t i;

	for (i = 0u; i < sizeof v / sizeof v[0]; i++) {
		uint32_t got = fdb_calc_crc32(0u, v[i].s, strlen(v[i].s));

		if (got != v[i].crc) {
			printf("crc32(\"%s\") = %08X, expected %08X\n",
			       v[i].s, (unsigned)got, (unsigned)v[i].crc);
			assert(0);
		}
	}
	printf("  A. canonical vectors OK\n");
}

/* ---- B: chaining equals one call ----------------------------------------- */

/* Split "123456789" at every possible point; each split must reproduce the
 * one-shot value.  Includes the two degenerate splits (0 and 9 bytes), which is
 * also the zero-length-chunk case: an empty call must be the identity, or a
 * receiver handing the sink a 0-byte write would corrupt the running value. */
static void test_chain_all_splits(void)
{
	static const char msg[] = "123456789";
	size_t n = strlen(msg), cut;

	for (cut = 0u; cut <= n; cut++) {
		uint32_t crc = fdb_calc_crc32(0u, msg, cut);

		crc = fdb_calc_crc32(crc, msg + cut, n - cut);
		if (crc != 0xCBF43926u) {
			printf("split at %zu gave %08X\n", cut, (unsigned)crc);
			assert(0);
		}
	}
	printf("  B. chaining at every split point OK\n");
}

/* ---- C: chaining over a blob-sized stream in YMODEM-sized blocks ---------- */

/* The real shape: ~189 KB delivered as 1024-byte blocks with a short final one,
 * accumulated exactly the way blob_sink_write() does, versus one call over the
 * whole buffer. */
#define STREAM_LEN   193456u          /* 188 x 1024 + 944: a short final block */
#define BLOCK_LEN    1024u

static uint8_t stream[STREAM_LEN];

static void test_chain_stream(void)
{
	uint32_t whole, chained = 0u, off = 0u, blocks = 0u;
	uint32_t i, x = 0x12345678u;

	/* Deterministic pseudo-random content: a constant or a counter would let a
	 * byte-order or table mistake cancel out. */
	for (i = 0u; i < STREAM_LEN; i++) {
		x ^= x << 13; x ^= x >> 17; x ^= x << 5;
		stream[i] = (uint8_t)(x & 0xFFu);
	}

	whole = fdb_calc_crc32(0u, stream, STREAM_LEN);
	while (off < STREAM_LEN) {
		uint32_t take = STREAM_LEN - off;

		if (take > BLOCK_LEN)
			take = BLOCK_LEN;
		chained = fdb_calc_crc32(chained, stream + off, take);
		off += take;
		blocks++;
	}
	assert(blocks == 189u);
	if (chained != whole) {
		printf("chained %08X != whole %08X\n",
		       (unsigned)chained, (unsigned)whole);
		assert(0);
	}
	/* A single flipped bit must change the answer -- otherwise `blob verify`
	 * would be checking nothing. */
	stream[STREAM_LEN / 2u] ^= 0x01u;
	assert(fdb_calc_crc32(0u, stream, STREAM_LEN) != whole);
	printf("  C. %lu-byte stream in %lu blocks OK (and bit-sensitive)\n",
	       (unsigned long)STREAM_LEN, (unsigned long)blocks);
}

/* ---- D: the wrapper that must NOT be re-added ---------------------------- */

/* Documents the double-inversion trap by asserting that the "standard" idiom is
 * wrong HERE.  If this ever starts matching, fdb_calc_crc32() has changed its own
 * inversion and app/blob.c's comment (and this test) have to be revisited. */
static void test_no_double_inversion(void)
{
	uint32_t wrapped = ~fdb_calc_crc32(0xFFFFFFFFu, "123456789", 9u);

	assert(wrapped != 0xCBF43926u);
	printf("  D. init-FFFFFFFF + final-complement is NOT the canonical value "
	       "(%08X) -- do not wrap\n", (unsigned)wrapped);
}

int main(void)
{
	printf("test_crc32:\n");
	test_vectors();
	test_chain_all_splits();
	test_chain_stream();
	test_no_double_inversion();
	printf("test_crc32: all passed\n");
	return 0;
}
