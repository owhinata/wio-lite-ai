/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    ymodem.c
 * @brief   Clean-room YMODEM-CRC batch sender + receiver (svc/ layer).
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
 *
 * Issue #19 M5 added the mirror-image RECEIVER at the bottom of this file (the board
 * has to take a firmware image FROM the PC before it may rewrite the RTL8720DN).  It
 * shares the framing constants and ymodem_crc16() but has its OWN static block buffer,
 * so the two directions never alias -- which is also what lets the host test drive
 * both concurrently over a pthread loopback.  Nothing above the "RECEIVER" banner was
 * changed by that work: the sender is the code already proven on board #2 in M4.
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

/* ======================================================================== *
 *  RECEIVER (issue #19 M5)
 * ======================================================================== *
 *
 * Receiver state machine, the mirror of ym_run() above:
 *   emit 'C' -> block 0 (name + size) -> sink->begin() -> ACK -> 'C' ->
 *   data blocks (ACK each, NAK a bad one) -> EOT -> ACK -> 'C' ->
 *   the sender's closing null block 0 -> ACK.
 *
 * Interoperates with ymodem_send() above and with lrzsz `sb` on the PC.
 */

/* Own block buffer -- deliberately NOT s_block (see the file banner). */
static uint8_t s_rdata[YM_DATA_MAX];

/* Diagnostics for the last run (see ymodem_recv_diag).  Single static because the
 * receiver is already documented as non-reentrant. */
static struct ym_recv_diag s_diag;

const struct ym_recv_diag *ymodem_recv_diag(void)
{
	return &s_diag;
}

/* Record the first rejected block; later rejections do not overwrite it. */
static void diag_first(int kind, int seq, int nseq, uint32_t got, uint32_t want,
                       uint16_t crc_want, uint16_t crc_got)
{
	if (s_diag.first_kind >= 0)
		return;
	s_diag.first_kind = kind;
	s_diag.first_seq = seq;
	s_diag.first_nseq = nseq;
	s_diag.first_got = got;
	s_diag.first_want = want;
	s_diag.first_crc_want = crc_want;
	s_diag.first_crc_got = crc_got;
}

/* wait_hdr() results that are not a protocol byte (both outside 0..255). */
#define YM_HDR_CANCEL (-3)      /* peer sent the double-CAN abort */

/* Bound on bytes discarded while hunting for a block header / draining a bad
 * block, so a jabbering line can never spin us forever. */
#define YM_RECV_DISCARD_MAX 8192

/* Swallow whatever is already buffered (used after a corrupt block, so the NAK'd
 * retransmission starts from a clean boundary).  timeout_ms == 0 polls once. */
static void recv_drain(const struct ym_io *io)
{
	for (int i = 0; i < YM_RECV_DISCARD_MAX; i++)
		if (io->getc(io->ctx, 0) < 0)
			return;
}

/*
 * Wait for a block-start byte, discarding line noise.  Returns YM_SOH / YM_STX /
 * YM_EOT, or YM_IO_TIMEOUT / YM_IO_ABORT / YM_HDR_CANCEL.
 */
static int wait_hdr(const struct ym_io *io, unsigned timeout_ms)
{
	for (int i = 0; i < YM_RECV_DISCARD_MAX; i++) {
		int b = io->getc(io->ctx, timeout_ms);

		if (b == (int)YM_SOH || b == (int)YM_STX || b == (int)YM_EOT)
			return b;
		if (b == YM_IO_TIMEOUT)
			s_diag.timeouts++;
		if (b == YM_IO_ABORT || b == YM_IO_TIMEOUT)
			return b;
		if (b == (int)YM_CAN && is_double_can(io))
			return YM_HDR_CANCEL;
		/* anything else: line noise / a stale ACK -- keep hunting */
	}
	return YM_IO_TIMEOUT;
}

/*
 * Read the body of a block whose header byte @p kind was already consumed:
 * seq, ~seq, data[128|1024], crc16.  Fills s_rdata + @p seq + @p datalen.
 * Returns 0 on a good block, -1 on a corrupt one (caller NAKs), -2 on abort.
 */
static int read_body(const struct ym_io *io, uint8_t kind, uint8_t *seq,
                     uint16_t *datalen)
{
	uint16_t n = (kind == YM_STX) ? (uint16_t)YM_DATA_MAX : (uint16_t)YM_BLOCK0_LEN;
	uint16_t crc, got;
	int      b, nseq;

	b = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
	if (b == YM_IO_ABORT)
		return -2;
	if (b < 0) {
		s_diag.short_read++;
		diag_first(kind, -1, -1, 0, n, 0, 0);
		return -1;
	}
	*seq = (uint8_t)b;

	nseq = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
	if (nseq == YM_IO_ABORT)
		return -2;
	if (nseq < 0) {
		s_diag.short_read++;
		diag_first(kind, *seq, -1, 0, n, 0, 0);
		return -1;
	}
	if ((uint8_t)nseq != (uint8_t)~*seq) {
		s_diag.bad_seq++;
		diag_first(kind, *seq, nseq, 0, n, 0, 0);
		return -1;
	}

	for (uint16_t i = 0; i < n; i++) {
		b = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
		if (b == YM_IO_ABORT)
			return -2;
		if (b < 0) {
			/* The distinguishing case: the sender is mid-block and bytes
			 * simply stopped arriving -> the transport lost them. */
			s_diag.short_read++;
			diag_first(kind, *seq, nseq, i, n, 0, 0);
			return -1;
		}
		s_rdata[i] = (uint8_t)b;
	}

	crc = 0;
	for (int i = 0; i < 2; i++) {
		b = io->getc(io->ctx, YM_ACK_TIMEOUT_MS);
		if (b == YM_IO_ABORT)
			return -2;
		if (b < 0) {
			s_diag.short_read++;
			diag_first(kind, *seq, nseq, n + (uint32_t)i, n, 0, 0);
			return -1;
		}
		crc = (uint16_t)((crc << 8) | (uint8_t)b);
	}

	got = ymodem_crc16(s_rdata, n);
	if (got != crc) {
		s_diag.bad_crc++;
		diag_first(kind, *seq, nseq, n, n, crc, got);
		return -1;
	}

	*datalen = n;
	s_diag.blocks++;
	return 0;
}

/*
 * Parse block 0's payload: "name\0" "size(decimal)\0".  @p name gets at most @p cap-1
 * chars (the field itself may be longer -- YMODEM allows up to ~100).  *@p size is 0 and
 * *@p have_size 0 when no size was declared, which is legal.
 *
 * Returns 0 on a well-formed block 0, -1 on a malformed one: an unterminated name field,
 * or a size that overflows 32 bits.  Rejecting rather than truncating matters because the
 * size is the value every downstream bound is derived from -- a wrapped size would
 * understate the file and slip past the sink's capacity check.
 */
static int parse_block0(char *name, size_t cap, uint32_t *size, int *have_size)
{
	size_t i = 0, n = 0;

	*size = 0;
	*have_size = 0;

	while (i < YM_BLOCK0_LEN && s_rdata[i] != 0) {
		if (n + 1 < cap)
			name[n++] = (char)s_rdata[i];
		i++;
	}
	name[n] = '\0';
	if (i >= YM_BLOCK0_LEN)
		return -1;                      /* name field ran off the block: malformed */
	i++;                                    /* past the NUL */

	while (i < YM_BLOCK0_LEN && s_rdata[i] >= '0' && s_rdata[i] <= '9') {
		uint32_t d = (uint32_t)(s_rdata[i] - '0');

		if (*size > (0xFFFFFFFFu - d) / 10u)
			return -1;              /* decimal size overflows 32 bits */
		*size = *size * 10u + d;
		*have_size = 1;
		i++;
	}
	return 0;
}

/* Emit a single protocol byte; <0 on transport failure. */
static int put1(const struct ym_io *io, uint8_t b)
{
	return io->put(io->ctx, &b, 1);
}

/* Run the receive batch; teardown on error is centralised in ymodem_recv(). */
static enum ym_result ym_recv_run(const struct ym_io *io, const struct ym_sink *sink)
{
	char     name[YM_BLOCK0_LEN];
	uint32_t size = 0, remaining = 0;
	uint16_t datalen = 0;
	uint8_t  seq = 0, expect = 1, prompt = YM_CRC_C;
	int      have_size = 0, attempts = 0, rc;

	/* 1. handshake: poke with 'C' until block 0 shows up.  The budget matches the
	 * sender's -- the human on the other end is starting `sb` by hand. */
	for (;;) {
		int h;

		if (put1(io, YM_CRC_C) < 0)
			return YM_ERR_IO;
		h = wait_hdr(io, YM_HANDSHAKE_TIMEOUT_MS);
		if (h == YM_IO_ABORT || h == YM_HDR_CANCEL)
			return YM_ERR_CANCEL;
		if (h == YM_IO_TIMEOUT) {
			if (++attempts > YM_HANDSHAKE_RETRIES)
				return YM_ERR_TIMEOUT;
			continue;
		}
		if (h == (int)YM_EOT) {         /* stray EOT before any file */
			(void)put1(io, YM_ACK);
			continue;
		}
		rc = read_body(io, (uint8_t)h, &seq, &datalen);
		if (rc == -2)
			return YM_ERR_CANCEL;
		if (rc != 0 || seq != 0) {      /* corrupt or not block 0: re-'C' */
			recv_drain(io);
			if (++attempts > YM_HANDSHAKE_RETRIES)
				return YM_ERR_TIMEOUT;
			continue;
		}
		break;
	}

	if (parse_block0(name, sizeof name, &size, &have_size) != 0)
		return YM_ERR_PROTO;
	if (name[0] == '\0') {
		/* An immediate null block 0 = the sender closed the batch without ever
		 * offering a file (e.g. `sb` on a missing path).  Nothing was received. */
		(void)put1(io, YM_ACK);
		return YM_ERR_CANCEL;
	}
	if (sink->begin(sink->ctx, name, size) < 0)
		return YM_ERR_SINK;

	if (put1(io, YM_ACK) < 0)               /* ACK block 0 */
		return YM_ERR_IO;
	if (put1(io, YM_CRC_C) < 0)             /* request data block 1 in CRC mode */
		return YM_ERR_IO;
	remaining = size;

	/* 2. data blocks.  A good block is ACKed and the ACK itself is the prompt for
	 * the next one; only a timeout re-prompts (with 'C' before the first data
	 * block, NAK afterwards), and a corrupt block is NAKed for a resend. */
	for (;;) {
		int h;

		attempts = 0;
		for (;;) {
			h = wait_hdr(io, YM_ACK_TIMEOUT_MS);
			if (h != YM_IO_TIMEOUT)
				break;
			if (++attempts > YM_BLOCK_RETRIES)
				return YM_ERR_TIMEOUT;
			if (put1(io, prompt) < 0)
				return YM_ERR_IO;
		}
		if (h == YM_IO_ABORT || h == YM_HDR_CANCEL)
			return YM_ERR_CANCEL;
		if (h == (int)YM_EOT) {
			/* The declared size is the contract.  A sender that stops early --
			 * a truncated file, a link that dropped blocks -- must NOT be able to
			 * reach the closing block and be reported as success: the caller would
			 * stage a short image and then program it.  (The batch is torn down by
			 * ymodem_recv()'s CAN, so the sender learns too.) */
			if (have_size && remaining != 0u)
				return YM_ERR_PROTO;
			if (put1(io, YM_ACK) < 0)
				return YM_ERR_IO;
			break;                          /* file complete */
		}

		rc = read_body(io, (uint8_t)h, &seq, &datalen);
		if (rc == -2)
			return YM_ERR_CANCEL;
		if (rc != 0) {
			recv_drain(io);
			prompt = YM_NAK;
			if (put1(io, YM_NAK) < 0)
				return YM_ERR_IO;
			continue;
		}

		if (seq == (uint8_t)(expect - 1u)) {
			/* Our previous ACK was lost and the sender resent: re-ACK, drop. */
			prompt = YM_NAK;
			if (put1(io, YM_ACK) < 0)
				return YM_ERR_IO;
			continue;
		}
		if (seq != expect)
			return YM_ERR_PROTO;            /* out of order: unrecoverable */

		/* Deliver, trimming the final block's CPMEOF padding via the declared
		 * size (authoritative, exactly as on the send side). */
		{
			uint32_t take = datalen;

			if (have_size) {
				if (take > remaining)
					take = remaining;
				remaining -= take;
			}
			if (take > 0u && sink->write(sink->ctx, s_rdata, take) < 0)
				return YM_ERR_SINK;
		}
		expect++;
		prompt = YM_NAK;
		if (put1(io, YM_ACK) < 0)
			return YM_ERR_IO;
	}

	/* 3. batch close.  Reaching the sender's closing null block 0 is REQUIRED for
	 * success -- see the ymodem_recv() contract in ymodem.h. */
	if (put1(io, YM_CRC_C) < 0)
		return YM_ERR_IO;
	attempts = 0;
	for (;;) {
		int h = wait_hdr(io, YM_ACK_TIMEOUT_MS);

		if (h == YM_IO_ABORT || h == YM_HDR_CANCEL)
			return YM_ERR_CANCEL;
		if (h == YM_IO_TIMEOUT) {
			if (++attempts > YM_BLOCK_RETRIES)
				return YM_ERR_TIMEOUT;
			if (put1(io, YM_CRC_C) < 0)
				return YM_ERR_IO;
			continue;
		}
		if (h == (int)YM_EOT) {                 /* sender repeated the EOT */
			if (put1(io, YM_ACK) < 0)
				return YM_ERR_IO;
			continue;
		}
		rc = read_body(io, (uint8_t)h, &seq, &datalen);
		if (rc == -2)
			return YM_ERR_CANCEL;
		if (rc != 0) {
			recv_drain(io);
			if (++attempts > YM_BLOCK_RETRIES)
				return YM_ERR_TIMEOUT;
			if (put1(io, YM_NAK) < 0)
				return YM_ERR_IO;
			continue;
		}
		if (seq != 0 || s_rdata[0] != 0)
			return YM_ERR_PROTO;            /* a second file: batches unsupported */
		(void)put1(io, YM_ACK);
		return YM_OK;
	}
}

enum ym_result ymodem_recv(const struct ym_io *io, const struct ym_sink *sink)
{
	enum ym_result rc;

	memset(&s_diag, 0, sizeof s_diag);
	s_diag.first_kind = -1;
	s_diag.first_seq = -1;
	s_diag.first_nseq = -1;

	rc = ym_recv_run(io, sink);
	if (rc != YM_OK)
		send_cancel(io);                /* one teardown for every error exit */
	return rc;
}
