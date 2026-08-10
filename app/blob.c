/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    blob.c
 * @brief   Read-only asset region on the external NOR (issue #10 / issue #9 P2b).
 *
 * See blob.h for the contract and for why the header is written last, in two steps,
 * and why the device lock is not held across a transfer.  This file is the only
 * place that knows the on-flash header format.
 */
#include "blob.h"

#include <flashdb.h>         /* fdb_calc_crc32 -- see the note below */
#include "nor_flash.h"
#include "stm32h7xx_hal.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */

#include <string.h>

#define LOG_TAG "blob"
#include "log.h"

/*
 * CRC-32 comes from FlashDB and is used RAW.
 *
 * fdb_calc_crc32() is already in this build (the KV store links it) and is already
 * the accumulating shape a streaming sink needs -- "crc must be 0 on first call",
 * feed the result back in for the next chunk.  It also already inverts at both ends
 * (fdb_utils.c: `crc ^ ~0U` on the way in and again on the way out), so starting
 * from 0 yields plain CRC-32/ISO-HDLC, byte-for-byte what `crc32` and Python's
 * `zlib.crc32` produce on the PC.  Wrapping it in the usual 0xFFFFFFFF-init /
 * final-complement idiom would invert twice and quietly produce a different number
 * on the board than on the host -- see shell/test/test_crc32.c, which pins both the
 * canonical vector and the chaining property this file depends on.
 *
 * Software, not the hardware CRC unit: that would mean enabling another peripheral
 * clock for something a table lookup already does fast enough.
 */

/* ------------------------------------------------------------------ *
 *  On-flash header
 * ------------------------------------------------------------------ *
 * Laid out at the base of the slot's 4 KB header sector, packed and unpacked BYTE
 * BY BYTE in little-endian order.  No struct is ever overlaid on the flash image:
 * that would make a format which has to outlive firmware changes depend on the
 * compiler's padding rules.
 *
 *   offset  0   magic     u32   'W','B','L','1' -- PROGRAMMED LAST, ON ITS OWN
 *           4   ver       u8    format version; an unknown one is refused
 *           5   name_len  u8    bytes of name[] in use
 *           6   pad       u16   0
 *           8   length    u32   payload byte count
 *          12   crc32     u32   CRC-32/ISO-HDLC over those payload bytes
 *          16   name[64]        printable ASCII, NUL-padded
 *
 * Magic sits at offset 0 and the body at 4..80, so both programs land in the SAME
 * 256-byte page.  That is deliberate and it is what makes the split commit legal on
 * this device: each program touches bytes that still read 0xFF, which is the
 * partial-page programming `nor test` accepts (W25Q128JV sec 8.2.13).
 */
#define BLOB_MAGIC       0x314C4257u   /* bytes 'W','B','L','1' in memory order */
/*
 * 🔴 VERSION 2 BECAUSE THE SLOT GEOMETRY CHANGED (issue #55).  Slots went from
 * 256 KB x 8 to 512 KB x 6, and the new slot bases (0x100000, 0x180000, 0x200000,
 * 0x280000, ...) LAND EXACTLY ON THE OLD EVEN SLOTS' HEADER SECTORS -- payload
 * included, since both start one 4 KB sector in.  A version-1 header written under
 * the old geometry therefore decodes perfectly under the new one, and the old
 * `length` still passes the (now larger) BLOB_PAYLOAD_MAX bound, so `blob list`
 * would show it as valid, `blob verify` would pass, and `ai model load` would load
 * it -- under a DIFFERENT SLOT NUMBER (old slot 2 appears as new slot 1, and the old
 * odd slots vanish into the middle of a new slot's payload).
 *
 * "Silently renumbered but working" is worse than "broken": nothing in the system
 * reports it, and every note that says which model is in which slot goes stale
 * without a word.  Bumping the version turns all of them into BLOB_INVALID, which is
 * exactly the state this field was added to express -- the header comment above
 * already promised that "an unknown one is refused".  Migration is
 * `blob erase <n>` + re-transfer.
 */
#define BLOB_FMT_VER     2u
#define BLOB_HDR_BODY    4u            /* first byte of the body                */
#define BLOB_HDR_BYTES   80u           /* magic + body + name[]                 */
#define BLOB_HDR_NAME    16u           /* offset of name[] within the header    */

_Static_assert(BLOB_HDR_BYTES <= NOR_PAGE_SIZE,
               "the header must fit in one page, so both commit steps program "
               "erased bytes of a single page");
_Static_assert(BLOB_HDR_NAME + BLOB_NAME_MAX + 1u <= BLOB_HDR_BYTES,
               "name[] must fit inside the header image");
_Static_assert(BLOB_REGION_END <= NOR_SIZE_BYTES,
               "the blob region must fit on the device");
_Static_assert(BLOB_REGION_BASE >= 1024u * 1024u,
               "the blob region must start above the kv partition");
_Static_assert((BLOB_SLOT_SIZE % NOR_BLOCK_SIZE) == 0u,
               "a slot must be a whole number of 64 KB erase blocks");

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * Names are reduced to printable ASCII, on the way IN and again on the way OUT.
 *
 * The name arrives from the PC in YMODEM block 0 and is printed by `blob list`, so
 * a control byte in a filename would be a remote party moving the terminal's cursor
 * or forging a line of output.  Filtering on the way in makes what is stored always
 * safe to print; filtering again on the way out covers a header this firmware did
 * not write (a foreign image, or damage).
 */
static char blob_safe_char(char c)
{
	unsigned char u = (unsigned char)c;

	return (u >= 0x20u && u < 0x7Fu) ? (char)u : '_';
}

/* ------------------------------------------------------------------ *
 *  Geometry
 * ------------------------------------------------------------------ */

static int slot_ok(unsigned slot)
{
	return slot < BLOB_SLOT_COUNT;
}

uint32_t blob_slot_addr(unsigned slot)
{
	return BLOB_REGION_BASE + slot * BLOB_SLOT_SIZE;
}

uint32_t blob_payload_addr(unsigned slot)
{
	return blob_slot_addr(slot) + BLOB_HDR_SIZE;
}

/* ------------------------------------------------------------------ *
 *  Read side
 * ------------------------------------------------------------------ */

/* Decode the 80-byte header image.  Returns the state; @p out is always filled in
 * (an unusable header leaves name/length/crc32 zeroed rather than half-decoded). */
static enum blob_state blob_decode(const uint8_t *hdr, struct blob_info *out)
{
	uint32_t i, name_len, length;
	int blank = 1;

	memset(out, 0, sizeof(*out));

	for (i = 0u; i < BLOB_HDR_BYTES; i++) {
		if (hdr[i] != 0xFFu) {
			blank = 0;
			break;
		}
	}
	if (blank)
		return BLOB_EMPTY;

	/* No magic yet, but bytes present: the body was programmed and the commit was
	 * cut short before the magic went down.  Reported apart from `empty` because
	 * it names a real, narrow window (power lost between the two programs) rather
	 * than leaving the operator to guess. */
	if (get_u32(hdr) == 0xFFFFFFFFu)
		return BLOB_INCOMPLETE;
	if (get_u32(hdr) != BLOB_MAGIC)
		return BLOB_INVALID;

	if (hdr[4] != (uint8_t)BLOB_FMT_VER)
		return BLOB_INVALID;
	name_len = hdr[5];
	if (name_len > BLOB_NAME_MAX)
		return BLOB_INVALID;
	if (get_u16(hdr + 6) != 0u)
		return BLOB_INVALID;
	length = get_u32(hdr + 8);
	if (length == 0u || length > BLOB_PAYLOAD_MAX)
		return BLOB_INVALID;

	for (i = 0u; i < name_len; i++)
		out->name[i] = blob_safe_char((char)hdr[BLOB_HDR_NAME + i]);
	out->name[name_len] = '\0';
	out->length = length;
	out->crc32 = get_u32(hdr + 12);
	return BLOB_VALID;
}

int blob_stat(unsigned slot, struct blob_info *out)
{
	uint8_t hdr[BLOB_HDR_BYTES];

	if (out == NULL)
		return BLOB_ERR_PARAM;
	/* Zero FIRST, so every failure path still leaves the caller with a defined
	 * struct rather than whatever was on its stack.  Callers routinely stat a slot
	 * and then print from the result on an error path. */
	memset(out, 0, sizeof(*out));
	if (!slot_ok(slot))
		return BLOB_ERR_PARAM;
	if (!nor_flash_ready())
		return BLOB_ERR_STATE;

	if (nor_read(blob_slot_addr(slot), hdr, sizeof hdr) != NOR_OK)
		return BLOB_ERR_IO;
	out->state = blob_decode(hdr, out);
	return BLOB_OK;
}

int blob_read(unsigned slot, uint32_t off, void *buf, uint32_t len)
{
	if (!slot_ok(slot) || buf == NULL || len == 0u ||
	    off >= BLOB_PAYLOAD_MAX || len > BLOB_PAYLOAD_MAX - off)
		return BLOB_ERR_PARAM;
	if (!nor_flash_ready())
		return BLOB_ERR_STATE;
	return (nor_read(blob_payload_addr(slot) + off, buf, len) == NOR_OK)
	               ? BLOB_OK : BLOB_ERR_IO;
}

int blob_verify(unsigned slot, uint32_t *crc_out)
{
	/* 256 B at a time, on the caller's stack.  The shell thread has 4 KB, so a
	 * kilobyte-scale buffer does not belong here -- cmd_nor.c keeps to the same
	 * page-sized chunk throughout for the same reason. */
	uint8_t buf[NOR_PAGE_SIZE];
	struct blob_info info;
	uint32_t done = 0u, crc = 0u;
	int rc;

	if (crc_out != NULL)
		*crc_out = 0u;

	rc = blob_stat(slot, &info);
	if (rc != BLOB_OK)
		return rc;
	if (info.state != BLOB_VALID)
		return BLOB_ERR_EMPTY;

	while (done < info.length) {
		uint32_t chunk = info.length - done;

		if (chunk > sizeof buf)
			chunk = sizeof buf;
		if (nor_read(blob_payload_addr(slot) + done, buf, chunk) != NOR_OK)
			return BLOB_ERR_IO;
		crc = fdb_calc_crc32(crc, buf, chunk);
		done += chunk;
	}

	if (crc_out != NULL)
		*crc_out = crc;
	return (crc == info.crc32) ? BLOB_OK : BLOB_ERR_CRC;
}

/* ------------------------------------------------------------------ *
 *  Mutating side
 * ------------------------------------------------------------------ */

/*
 * Serialises blob-mutating operations against each other.  A plain flag test-set
 * under a brief PRIMASK critical section: interrupt-safe, thread-agnostic (acquire
 * and release may run on different threads) and needing no one-time init, which
 * would itself race between the two consoles.  Same idiom as the NN session guard
 * in port/nn/nn.c, and PRIMASK is what this firmware's ThreadX port uses for its own
 * critical sections (TX_PORT_USE_BASEPRI is not defined), so it nests consistently.
 *
 * It guards the CALLERS, not the functions below: `blob write` has to hold it from
 * before the erase until its automatic verify has finished, which is a span only the
 * command knows.
 */
static volatile int blob_busy;

int blob_busy_acquire(void)
{
	uint32_t primask = __get_PRIMASK();
	int ok;

	__disable_irq();
	if (!blob_busy) { blob_busy = 1; ok = 1; } else { ok = 0; }
	__set_PRIMASK(primask);
	return ok ? BLOB_OK : BLOB_ERR_BUSY;
}

void blob_busy_release(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	blob_busy = 0;
	__set_PRIMASK(primask);
}

int blob_erase(unsigned slot)
{
	if (!slot_ok(slot))
		return BLOB_ERR_PARAM;
	if (!nor_flash_ready())
		return BLOB_ERR_STATE;
	return (nor_erase(blob_slot_addr(slot), BLOB_SLOT_SIZE) == NOR_OK)
	               ? BLOB_OK : BLOB_ERR_IO;
}

/* ---- YMODEM sink --------------------------------------------------------- */

static struct {
	int      armed;      /* blob_recv_arm() erased a slot and is waiting        */
	int      opened;     /* begin() accepted the file                          */
	int      failed;     /* sticky: a payload program failed                   */
	unsigned slot;
	uint32_t cap;        /* size block 0 declared (authoritative)              */
	uint32_t pos;        /* payload bytes programmed so far                    */
	uint32_t crc;        /* running CRC of what was RECEIVED, not read back    */
	uint8_t  name_len;
	char     name[BLOB_NAME_MAX + 1u];
} s_rx;

static void blob_rx_reset(void)
{
	memset(&s_rx, 0, sizeof s_rx);
}

int blob_recv_arm(unsigned slot)
{
	int rc;

	blob_rx_reset();
	if (!slot_ok(slot))
		return BLOB_ERR_PARAM;
	if (!nor_flash_ready())
		return BLOB_ERR_STATE;

	rc = blob_erase(slot);
	if (rc != BLOB_OK)
		return rc;

	s_rx.slot = slot;
	s_rx.armed = 1;
	return BLOB_OK;
}

void blob_recv_disarm(void)
{
	blob_rx_reset();
}

uint32_t blob_recv_pos(void)
{
	return s_rx.pos;
}

static int blob_sink_begin(void *ctx, const char *name, uint32_t size)
{
	uint32_t i;

	(void)ctx;
	if (!s_rx.armed)
		return -1;
	/* Re-entered begin() (a sender that restarts the batch) must not inherit the
	 * previous file's position or CRC. */
	s_rx.opened = 0;
	s_rx.failed = 0;
	s_rx.cap = 0u;
	s_rx.pos = 0u;
	s_rx.crc = 0u;

	/* A sender that declared no size is refused, not guessed at: without it the
	 * protocol cannot trim the final block's padding either, so there would be no
	 * exact length to record. */
	if (size == 0u || size > BLOB_PAYLOAD_MAX)
		return -1;

	for (i = 0u; i < BLOB_NAME_MAX && name[i] != '\0'; i++)
		s_rx.name[i] = blob_safe_char(name[i]);
	s_rx.name[i] = '\0';
	s_rx.name_len = (uint8_t)i;
	s_rx.cap = size;
	s_rx.opened = 1;
	return 0;
}

static int blob_sink_write(void *ctx, const uint8_t *data, uint32_t len)
{
	(void)ctx;
	if (!s_rx.armed || !s_rx.opened || s_rx.failed)
		return -1;
	/* Refuse more than was declared; do not clamp.  The first test also makes the
	 * subtraction safe -- without a successful begin() cap is 0, which no receiver
	 * bug can turn into a large bound. */
	if (s_rx.pos > s_rx.cap || len > s_rx.cap - s_rx.pos)
		return -1;

	if (nor_write(blob_payload_addr(s_rx.slot) + s_rx.pos, data, len) != NOR_OK) {
		s_rx.failed = 1;
		return -1;
	}
	/* CRC of the STREAM.  Deriving it from a read-back would make `blob verify`
	 * check the flash against itself (see blob.h). */
	s_rx.crc = fdb_calc_crc32(s_rx.crc, data, len);
	s_rx.pos += len;
	return 0;
}

/* No abort hook and nothing that polls cli_cancel_requested(): while a receive
 * runs, the ONLY permitted reader of the console RX ring is cmd_xfer.c's
 * io_getc_recv().  cli_cancel_poll() drains that ring and discards every non-0x03
 * byte, which here would eat the sender's block data (see cmd_xfer.h). */
static const struct ym_sink s_sink = { NULL, blob_sink_begin, blob_sink_write };

const struct ym_sink *blob_sink(void)
{
	return &s_sink;
}

int blob_recv_commit(struct blob_info *out)
{
	uint8_t hdr[BLOB_HDR_BYTES];
	uint8_t back[BLOB_HDR_BYTES];
	uint32_t base;
	uint32_t body = BLOB_HDR_BYTES - BLOB_HDR_BODY;

	if (!s_rx.armed || !s_rx.opened || s_rx.failed)
		return BLOB_ERR_STATE;
	/*
	 * The batch closed, but fewer bytes arrived than block 0 promised: the file was
	 * truncated on the way here.  Committing it with length = what arrived would
	 * store a short blob and stamp it valid, which is precisely what the whole
	 * header-last design exists to prevent -- valid has to mean complete.
	 */
	if (s_rx.pos != s_rx.cap)
		return BLOB_ERR_SHORT;

	base = blob_slot_addr(s_rx.slot);

	memset(hdr, 0, sizeof hdr);
	put_u32(hdr + 0, BLOB_MAGIC);
	hdr[4] = (uint8_t)BLOB_FMT_VER;
	hdr[5] = s_rx.name_len;
	put_u16(hdr + 6, 0u);
	put_u32(hdr + 8, s_rx.pos);
	put_u32(hdr + 12, s_rx.crc);
	memcpy(hdr + BLOB_HDR_NAME, s_rx.name, s_rx.name_len);

	/* Step 1: the body, everything except the magic. */
	if (nor_write(base + BLOB_HDR_BODY, hdr + BLOB_HDR_BODY, body) != NOR_OK) {
		LOG_ERR("slot %u: header body program failed", s_rx.slot);
		return BLOB_ERR_IO;
	}
	/* Step 2: prove it landed BEFORE the magic makes it authoritative. */
	if (nor_read(base + BLOB_HDR_BODY, back + BLOB_HDR_BODY, body) != NOR_OK ||
	    memcmp(hdr + BLOB_HDR_BODY, back + BLOB_HDR_BODY, body) != 0) {
		LOG_ERR("slot %u: header body did not read back", s_rx.slot);
		return BLOB_ERR_IO;
	}
	/* Step 3: the magic, alone, in its own program.  From here the slot is valid. */
	if (nor_write(base, hdr, BLOB_HDR_BODY) != NOR_OK) {
		LOG_ERR("slot %u: magic program failed", s_rx.slot);
		return BLOB_ERR_IO;
	}
	if (nor_read(base, back, BLOB_HDR_BODY) != NOR_OK ||
	    memcmp(hdr, back, BLOB_HDR_BODY) != 0) {
		/* Without this the slot would simply keep reading `incomplete` and the
		 * operator would be left to work out why a successful transfer vanished. */
		LOG_ERR("slot %u: magic did not read back", s_rx.slot);
		return BLOB_ERR_IO;
	}

	if (out != NULL) {
		memset(out, 0, sizeof(*out));
		out->state = BLOB_VALID;
		memcpy(out->name, s_rx.name, (size_t)s_rx.name_len + 1u);
		out->length = s_rx.pos;
		out->crc32 = s_rx.crc;
	}
	LOG_INF("slot %u: '%s', %lu B, crc32 %08lX", s_rx.slot, s_rx.name,
	        (unsigned long)s_rx.pos, (unsigned long)s_rx.crc);
	blob_rx_reset();
	return BLOB_OK;
}
