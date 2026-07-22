/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    ymodem.c
 * @brief   Clean-room YMODEM-CRC batch sender (svc/ layer).
 *
 * Sender state machine: wait for the receiver's 'C' (CRC mode request) -> send
 * block 0 (filename + decimal size) -> send 1024-byte data blocks -> EOT
 * handshake -> a final null block 0 that closes the batch.  All IO and the data
 * source are injected (see ymodem.h), so this file is freestanding (no shell /
 * ThreadX / HAL) and host-testable.
 *
 * Ported from ../stm32f746g-disco for issue #19 M4 (RTL8720DN full-chip flash backup
 * over the USB CDC console).  The only deviation from the donor is a longer initial
 * handshake budget -- see YM_HANDSHAKE_RETRIES.
 */
#include "ymodem.h"

#include <string.h>

/* ---- protocol bytes ---------------------------------------------------- */
#define YM_SOH   0x01u   /* 128-byte block */
#define YM_STX   0x02u   /* 1024-byte block */
#define YM_EOT   0x04u   /* end of transmission */
#define YM_ACK   0x06u
#define YM_NAK   0x15u
#define YM_CAN   0x18u
#define YM_CRC_C 0x43u   /* 'C' -- receiver requests CRC mode */
#define YM_CPMEOF 0x1Au  /* short-block padding (CP/M EOF) */
#define YM_BS    0x08u   /* backspace, part of the cancel sequence */

/* ---- block geometry ---------------------------------------------------- */
#define YM_DATA_MAX   1024u
#define YM_BLOCK0_LEN 128u
#define YM_HDR        3u            /* kind, seq, ~seq */
#define YM_CRC        2u

/* ---- timeouts (ms == ThreadX tick) and retry caps ---------------------- */
#define YM_HANDSHAKE_TIMEOUT_MS 3000u
#define YM_ACK_TIMEOUT_MS       1000u
#define YM_EOT_TIMEOUT_MS       2000u
/*
 * Initial-handshake budget = TIMEOUT_MS * RETRIES.  Deviation from the f746 donor
 * (which used 10 = 30 s): here the receiver is started BY HAND, after the command has
 * already printed its prompt -- in picocom that means reading the message, pressing
 * Ctrl+A Ctrl+R and answering the file prompt.  30 s turned out to be genuinely tight,
 * and overrunning it fails confusingly: the send ends, the console goes back to the line
 * editor, and the receiver's 'C' is then ECHOED back to it, so lrzsz reports "Got 0103
 * sector header" (octal 103 = 'C') rather than anything about a timeout.  Two minutes
 * removes the race; Ctrl+C still aborts the wait at any point (io_getc maps it).
 */
#define YM_HANDSHAKE_RETRIES    40
#define YM_BLOCK_RETRIES        10

/*
 * Single block staging buffer: [kind | seq | ~seq | data[1024] | crc_hi | crc_lo].
 * File-scope (not on the 4 KB shell-thread stack).  Not reentrant -- the shell
 * serialises transfers by holding the console output lock for the whole send.
 */
static uint8_t s_block[YM_HDR + YM_DATA_MAX + YM_CRC];
#define S_DATA (s_block + YM_HDR)

uint16_t ymodem_crc16(const uint8_t *p, size_t n)
{
	uint16_t crc = 0;
	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)((uint16_t)p[i] << 8);
		for (int b = 0; b < 8; b++)
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
			                      : (uint16_t)(crc << 1);
	}
	return crc;
}

/* Render @p v as decimal ASCII into @p out (max 10 digits); return the length. */
static int u32_to_dec(uint32_t v, char *out)
{
	char tmp[10];
	int n = 0;
	if (v == 0) { out[0] = '0'; return 1; }
	while (v) { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; }
	for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
	return n;
}

/* Emit the lrzsz-style cancel: CAN x5 + BS x8, so the receiver bails cleanly. */
static void send_cancel(const struct ym_io *io)
{
	static const uint8_t seq[] = {
		YM_CAN, YM_CAN, YM_CAN, YM_CAN, YM_CAN,
		YM_BS,  YM_BS,  YM_BS,  YM_BS,  YM_BS, YM_BS, YM_BS, YM_BS,
	};
	(void)io->put(io->ctx, seq, sizeof seq);
}

/* A second CAN within a short window confirms a receiver-initiated abort. */
static int is_double_can(const struct ym_io *io)
{
	return io->getc(io->ctx, YM_ACK_TIMEOUT_MS) == (int)YM_CAN;
}

/*
 * Frame S_DATA[0..datalen) as block @p seq of @p kind, append the CRC, send it,
 * and wait for ACK -- retrying on NAK / 'C' / timeout up to YM_BLOCK_RETRIES.
 * The caller has already filled S_DATA (and padded to datalen).
 */
static enum ym_result send_block(const struct ym_io *io, uint8_t kind,
                                 uint8_t seq, uint16_t datalen)
{
	uint16_t crc;
	size_t   total;
	int      attempts = 0;

	s_block[0] = kind;
	s_block[1] = seq;
	s_block[2] = (uint8_t)~seq;
	crc = ymodem_crc16(S_DATA, datalen);
	s_block[YM_HDR + datalen]     = (uint8_t)(crc >> 8);
	s_block[YM_HDR + datalen + 1] = (uint8_t)(crc & 0xFFu);
	total = YM_HDR + datalen + YM_CRC;

	for (;;) {
		int b;
		if (io->put(io->ctx, s_block, total) < 0)
			return YM_ERR_IO;
		b = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
		if (b == (int)YM_ACK)
			return YM_OK;
		if (b == YM_IO_ABORT)
			return YM_ERR_CANCEL;
		if (b == (int)YM_CAN && is_double_can(io))
			return YM_ERR_CANCEL;
		/* NAK / 'C' (re-request) / single CAN / timeout / stray -> resend. */
		if (++attempts > YM_BLOCK_RETRIES)
			return YM_ERR_TIMEOUT;
	}
}

/* Wait for the receiver's initial 'C'.  CRC-only: 'C' starts; NAK (old checksum
 * mode request) is swallowed as compat and we keep waiting for 'C'. */
static enum ym_result wait_start(const struct ym_io *io)
{
	int timeouts = 0;
	for (;;) {
		int b = io->getc(io->ctx, YM_HANDSHAKE_TIMEOUT_MS);
		if (b == (int)YM_CRC_C)
			return YM_OK;
		if (b == YM_IO_ABORT)
			return YM_ERR_CANCEL;
		if (b == (int)YM_CAN && is_double_can(io))
			return YM_ERR_CANCEL;
		if (b == YM_IO_TIMEOUT && ++timeouts > YM_HANDSHAKE_RETRIES)
			return YM_ERR_TIMEOUT;
		/* NAK / single CAN / stray byte: keep waiting for 'C'. */
	}
}

/* Wait for a 'C' that requests the next block (after block 0 ACK / after EOT). */
static enum ym_result wait_c(const struct ym_io *io)
{
	int timeouts = 0;
	for (;;) {
		int b = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
		if (b == (int)YM_CRC_C)
			return YM_OK;
		if (b == YM_IO_ABORT)
			return YM_ERR_CANCEL;
		if (b == (int)YM_CAN && is_double_can(io))
			return YM_ERR_CANCEL;
		if (b == YM_IO_TIMEOUT && ++timeouts > YM_BLOCK_RETRIES)
			return YM_ERR_TIMEOUT;
	}
}

/* EOT handshake: some receivers NAK the first EOT (spec) and ACK the second. */
static enum ym_result send_eot(const struct ym_io *io)
{
	uint8_t eot = YM_EOT;
	int     attempts = 0;
	for (;;) {
		int b;
		if (io->put(io->ctx, &eot, 1) < 0)
			return YM_ERR_IO;
		b = io->getc(io->ctx, YM_EOT_TIMEOUT_MS);
		if (b == (int)YM_ACK)
			return YM_OK;
		if (b == YM_IO_ABORT)
			return YM_ERR_CANCEL;
		if (b == (int)YM_CAN && is_double_can(io))
			return YM_ERR_CANCEL;
		/* NAK / timeout -> resend EOT. */
		if (++attempts > YM_BLOCK_RETRIES)
			return YM_ERR_TIMEOUT;
	}
}

/* Build block 0 into S_DATA: "name\0" "size(decimal)\0", zero-padded to 128. */
static void build_block0(const struct ym_source *src)
{
	const char *n = (src->name && src->name[0]) ? src->name : "file.bin";
	size_t i = 0;
	char   num[10];
	int    nl;

	memset(S_DATA, 0, YM_BLOCK0_LEN);
	while (*n && i < 100)
		S_DATA[i++] = (uint8_t)*n++;
	S_DATA[i++] = 0;                        /* NUL after the name */
	nl = u32_to_dec(src->size, num);
	for (int k = 0; k < nl && i < YM_BLOCK0_LEN - 1; k++)
		S_DATA[i++] = (uint8_t)num[k];
	/* the NUL after the size and the rest are already zero from memset */
}

/* Run the batch; teardown on error is centralised in ymodem_send() below. */
static enum ym_result ym_run(const struct ym_io *io, const struct ym_source *src)
{
	enum ym_result rc;
	uint32_t       remaining = src->size;   /* exact size is authoritative */
	uint8_t        seq;

	/* 1. handshake: wait for the receiver's 'C'. */
	rc = wait_start(io);
	if (rc != YM_OK)
		return rc;

	/* 2. block 0: filename + exact size. */
	build_block0(src);
	rc = send_block(io, YM_SOH, 0, YM_BLOCK0_LEN);
	if (rc != YM_OK)
		return rc;

	/* receiver ACKs block 0, then sends 'C' to request the first data block. */
	rc = wait_c(io);
	if (rc != YM_OK)
		return rc;

	/* 3. data blocks 1..N, bounded by src->size (the block-0 size is the single
	 * source of truth -- a source that over- or under-reads cannot overrun). */
	seq = 1;
	while (remaining > 0) {
		uint32_t want = (remaining < YM_DATA_MAX) ? remaining : YM_DATA_MAX;
		uint32_t filled = 0, got;
		uint16_t datalen;

		while (filled < want) {
			int sr = src->read(src->ctx, S_DATA + filled, want - filled,
			                   &got);
			if (sr < 0)
				return YM_ERR_SOURCE;
			if (got == 0)
				break;                  /* source short of src->size: stop */
			filled += got;
		}
		if (filled == 0)
			break;

		datalen = (filled <= YM_BLOCK0_LEN) ? (uint16_t)YM_BLOCK0_LEN
		                                    : (uint16_t)YM_DATA_MAX;
		for (uint32_t i = filled; i < datalen; i++)
			S_DATA[i] = YM_CPMEOF;          /* pad the short final block */

		rc = send_block(io, datalen == YM_BLOCK0_LEN ? YM_SOH : YM_STX,
		                seq, datalen);
		if (rc != YM_OK)
			return rc;
		seq++;
		remaining -= filled;
		if (filled < want)
			break;                          /* source ended early */
	}

	/* 4. EOT. */
	rc = send_eot(io);
	if (rc != YM_OK)
		return rc;

	/* 5. null block 0 (empty name) closes the batch. */
	rc = wait_c(io);
	if (rc != YM_OK)
		return rc;
	memset(S_DATA, 0, YM_BLOCK0_LEN);
	return send_block(io, YM_SOH, 0, YM_BLOCK0_LEN);
}

enum ym_result ymodem_send(const struct ym_io *io, const struct ym_source *src)
{
	enum ym_result rc = ym_run(io, src);
	if (rc != YM_OK)
		send_cancel(io);                /* one teardown for every error exit */
	return rc;
}
