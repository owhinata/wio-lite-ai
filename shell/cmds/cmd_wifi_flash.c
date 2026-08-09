/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi_flash.c
 * @brief   The `wifi flash <sub>` subtree (issue #19): rewriting the RTL8720DN's own
 *          SPI flash over its mask-ROM UART download mode.
 *
 *   wifi flash info                capacity (address wrap) / status regs / checksum
 *   wifi flash read <off> [n]      survey sectors, erased-vs-data (read-only)
 *   wifi flash backup [off] [len]  back the flash up to the PC over YMODEM (read-only)
 *   wifi flash imgload             receive an image from the PC into PSRAM (read-only)
 *   wifi flash imginfo             show + re-verify the staged image (read-only)
 *   wifi flash write <off> confirm DESTRUCTIVE -- program the staged image
 *
 * They were seven flat `wifi flashXXX` / `wifi imgXXX` commands until issue #28: the
 * flat namespace could not keep them together in `help wifi` (prefix order and pipeline
 * order disagree -- `flashwrite` needs `imgload` to have run first), and the M3
 * single-sector self-test `wifi flashtest` went with the move, its erase/write/verify
 * path now proven by every real `wifi flash write` instead.
 *
 * None of this talks eRPC: every session drops the module into its mask-ROM UART
 * download mode (app/rtl8720_flash.c owns the protocol) and ALWAYS power-cycles it
 * back to the normal firmware on the way out -- flash_session_recover() below is that
 * epilogue, shared by every handler.  The table at the bottom is nested into the `wifi`
 * table (cmd_wifi.c) through cmd_wifi_priv.h, exactly like `wifi link`.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
/* cli_instance.h (ThreadX-aware) gives the full struct cli_instance, which
 * `wifi flash imgload` needs for sh->rx_dropped: the console backend silently drops --
 * and counts -- a byte when its RX ring overruns, and a bulk receive is the first
 * thing in this firmware that can provoke that, so the count has to be reported
 * rather than assumed to be zero. */
#include "cli_instance.h"
#include "cmd_wifi_priv.h"
#include "cmd_xfer.h"
#include "rtl8720.h"
#include "rtl8720_flash.h"
#include "rtl8720_img.h"     /* #19 M5: PSRAM staging for a host-supplied image */
#include "psram.h"           /* #19 M5: PSRAM_BASE_ADDR + the OCTOSPI1 guard */
#include "log.h"             /* #19 M5: transfer post-mortem into the dmesg ring */
#include "rtl_link.h"
#include "wifi_auto.h"       /* #32: a flash session disarms the host's re-association */
#include "nx_net.h"        /* the bridge has to be given back before we take the link */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
 * Take the link for a download session (issue #30 B2b).
 *
 * Since the bridge became permanent, the interface owner holds a UART reference for as
 * long as the host stack is up -- so the plain busy-reject claim these commands used
 * would refuse EVERY flash command, i.e. it would take away the issue-#19 recovery path
 * exactly when it is most needed.  Flashing is a recovery path and has to work when the
 * link is occupied, like `wifi on/off/reset`.
 *
 * The order matters and each step earns its place:
 *   1. ask the interface to unwind PROPERLY (the DATA consumer detaches and the telnet
 *      console releases its socket under their own contracts -- yanking the link instead
 *      would leave NetX holding a socket on an interface that no longer receives);
 *   2. claim with allow_busy, because after step 1 nothing should hold it and if
 *      something still does, this is the command that must win;
 *   3. re-check the state UNDER the claim: nx_net_down() only posts a request and does
 *      not hold the coarse mutex while it unwinds, so between step 1 finishing and step 2
 *      succeeding another console could have started `wifi connect` and armed it again.
 */
#define FLASH_STOP_POLL_MS   100u
#define FLASH_STOP_WAIT_MS   10000u

static int flash_claim(struct cli_instance *sh)
{
	unsigned waited = 0u;

	/*
	 * First line, before the teardown below (issue #32): an automatic re-association can
	 * be holding the coarse mutex for up to 22 s, and nx_net_down()'s unwind needs that
	 * same mutex -- so disarming here is what stops step 1 from waiting out an attempt.
	 * The credentials go with it, which is right: this session rewrites the module.
	 */
	wifi_auto_disarm("flash session");

	if (nx_net_state() != NX_NET_OFF) {
		cli_print(sh, "wifi: taking the host stack down first (flashing needs the "
		          "link)...\r\n");
		nx_net_down();
		while (nx_net_state() != NX_NET_OFF && waited < FLASH_STOP_WAIT_MS) {
			if (cli_sleep(sh, FLASH_STOP_POLL_MS))
				return -1;                  /* Ctrl+C */
			waited += FLASH_STOP_POLL_MS;
		}
		if (nx_net_state() != NX_NET_OFF) {
			cli_error(sh, "wifi: the host stack would not go down -- "
			          "`net shell stop` then retry, or `wifi reset`\r\n");
			return -1;
		}
	}
	if (rtl_link_hw_claim(sh, true) != 0)
		return -1;
	if (nx_net_state() != NX_NET_OFF) {
		rtl_link_hw_release(sh);
		cli_error(sh, "wifi: the host stack came back up just now -- retry\r\n");
		return -1;
	}
	return 0;
}

/*
 * Every download session ends here, success or failure: close UART9, power-cycle the
 * module back to its normal eRPC firmware, and invalidate the host-tracked module
 * state (rate, firmware generation, lwIP, address -- the module rebooted).  The
 * caller still owns the claim it took (rtl_link_hw_claim / psram_acquire) and
 * releases it in its own order; only the module-side epilogue is common.
 */
static void flash_session_recover(struct cli_instance *sh)
{
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	/* The session dropped the firmware proof along with everything else the host
	 * believed (rtl_link_force_quiesce -> rtl_link_forget_module), and `wifi connect`
	 * now needs it in order to bridge.  Say so here rather than letting the next
	 * connect discover it. */
	cli_print(sh, "  run `wifi ver` before `wifi connect` (this session dropped the "
	          "firmware proof)\r\n");
}

/* Open a download session: enter download mode + load the flashloader at 1.5 Mbaud.
 * Returns 0 on success (caller owns the session and must reach its `recover:` label). */
static int flash_session_open(struct cli_instance *sh)
{
	int rc = rtl_dl_enter(30000u, rtl_abort_cb, sh);

	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); return -1; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); return -1; }
	rc = rtl_dl_load_flashloader(1500000u, rtl_abort_cb, sh);
	if (rc != 0)  { cli_error(sh, "wifi: flashloader load failed (rc %d)\r\n", rc); return -1; }
	return 0;
}

/* Print the detected capacity, or explain why it could not be determined.  Returns the
 * size to use (0 = unknown). */
static uint32_t flash_report_size(struct cli_instance *sh, const struct rtl_dl_size *sz, int rc)
{
	if (rc == -3) {
		cli_warn(sh, "  capacity: UNKNOWN -- the first %lu KB reads all 0xFF, so there "
		          "is no data to detect the address wrap against\r\n",
		          (unsigned long)(RTL_DL_SIZE_PROBE_LEN / 1024u));
		return 0u;
	}
	if (rc != 0) {
		cli_error(sh, "  capacity: probe failed (rc %d)\r\n", rc);
		return 0u;
	}
	if (sz->size == 0u) {
		cli_warn(sh, "  capacity: UNKNOWN -- no address wrap up to 8 MB "
		          "(chip is >= 16 MB, or does not wrap)\r\n");
		return 0u;
	}
	/* The span comes from the constant, not a literal: it was 8 KB until issue #46
	   halved it, and a hardcoded number here would have gone on claiming 8 KB while
	   comparing 4 -- a report that lies is worse than no report. */
	cli_print(sh, "  capacity: %lu MB (0x%lX) -- address wrap at that offset, "
	          "%lu KB compared byte-for-byte\r\n",
	          (unsigned long)(sz->size >> 20), (unsigned long)sz->size,
	          (unsigned long)(RTL_DL_SIZE_PROBE_LEN / 1024u));
	return sz->size;
}
/* wifi flash read <offset> [nsectors] (issue #19, M3): NON-DESTRUCTIVE flash survey --
 * read sectors and show whether each looks erased.  It is how a range is checked before
 * and after `wifi flash write`, without needing a PC receiver like `backup` does. */
static int cmd_flash_read(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t offset, nsectors = 1u, s;
	uint8_t buf[128];
	int rc, ok = 0;

	if (cli_parse_u32(argv[1], &offset) != 0 || (offset & 0xFFFu) != 0u) {
		cli_error(sh, "wifi: bad offset (4KB-aligned hex, e.g. 0x180000)\r\n");
		return 1;
	}
	if (argc >= 3 && (cli_parse_u32(argv[2], &nsectors) != 0 || nsectors == 0u || nsectors > 64u)) {
		cli_error(sh, "wifi: bad nsectors (1..64)\r\n");
		return 1;
	}
	if (flash_claim(sh) != 0)
		return 1;

	cli_print(sh, "wifi: reading flash (NON-DESTRUCTIVE)...\r\n");
	rc = rtl_dl_enter(30000u, rtl_abort_cb, sh);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); goto recover; }
	rc = rtl_dl_load_flashloader(1500000u, rtl_abort_cb, sh);
	if (rc != 0) { cli_error(sh, "wifi: flashloader load failed (rc %d)\r\n", rc); goto recover; }

	ok = 1;
	for (s = 0u; s < nsectors; s++) {
		uint32_t off = offset + s * 4096u;
		int i, allff = 1;

		rc = rtl_dl_read_flash(off, 1u, buf, sizeof(buf), rtl_abort_cb, sh);
		if (rc < 0) {
			cli_error(sh, "wifi: read @0x%lX failed (rc %d)\r\n", (unsigned long)off, rc);
			ok = 0;
			break;
		}
		for (i = 0; i < rc; i++)
			if (buf[i] != 0xFFu) { allff = 0; break; }
		cli_print(sh, "0x%06lX: first %d B %s\r\n", (unsigned long)off, rc,
		          allff ? "all 0xFF (looks erased)" : "has data");
		cli_hexdump_base(sh, buf, (rc < 64) ? (size_t)rc : 64u, off);
	}

recover:
	flash_session_recover(sh);
	rtl_link_hw_release(sh);
	return ok ? 0 : 1;
}
/* wifi flash info (issue #19, M4): NON-DESTRUCTIVE flash identification.
 *
 * Runs in TWO download sessions on purpose.  Two operations each want to be last:
 * the 0x27 checksum (a timeout may still be answered later and would desynchronise
 * whatever follows) and the experimental RDID probe (its command shape is not
 * established by the reference tool, so a mis-framed reply may desynchronise too).
 * Session A therefore ends with the checksum, and the RDID probe gets a fresh session
 * of its own -- the same "power-cycle and re-enter" pattern the programmer uses between
 * its write and verify phases.
 * Only session A's wrap detection is authoritative; everything else is a diagnostic. */
static int cmd_flash_info(struct cli_instance *sh, int argc, char **argv)
{
	struct rtl_dl_size sz;
	struct rtl_dl_jedec jd;
	uint8_t sr[3];
	uint32_t size, sum = 0u;
	int rc, ok = 0;

	(void)argc; (void)argv;

	if (flash_claim(sh) != 0)
		return 1;

	cli_print(sh, "wifi: identifying RTL8720 flash (NON-DESTRUCTIVE)...\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;

	/* --- session A, step 1: capacity by address wrap (proven read path only) --- */
	rc = rtl_dl_detect_size(&sz, rtl_abort_cb, sh);
	size = flash_report_size(sh, &sz, rc);
	if (rc == -2)
		goto recover;                       /* the read path itself failed */
	ok = (size != 0u);                          /* success == the capacity is known */

	/* --- session A, step 2: status registers (reference-tool command shape only) --- */
	rc = rtl_dl_flash_status(sr, rtl_abort_cb, sh);
	if (rc != 0) {
		cli_warn(sh, "  status:   read failed (rc %d)\r\n", rc);
		goto recover;                       /* link is unhappy; do not push further */
	}
	cli_print(sh, "  status:   SR1 0x%02X  SR2 0x%02X  SR3 0x%02X\r\n",
	          sr[0], sr[1], sr[2]);

	/* --- session A, step 3 (LAST in this session): device-side checksum --- */
	rc = rtl_dl_flash_chksum(0u, 0x10000u, 5000u, &sum, rtl_abort_cb, sh);
	if (rc == 0)
		cli_print(sh, "  chksum:   0x%08lX over the first 64 KB (device-side, 0x27)\r\n",
		          (unsigned long)sum);
	else
		cli_warn(sh, "  chksum:   n/a (rc %d) -- ending the session, a late reply "
		          "would desynchronise it\r\n", rc);

	/* --- session B: the experimental RDID probe, alone in a fresh session --- */
	rtl8720_uart_close();
	cli_print(sh, "  (re-entering download mode for the experimental JEDEC probe)\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;
	rc = rtl_dl_flash_jedec(&jd, rtl_abort_cb, sh);
	if (rc == 0 && jd.ok) {
		cli_print(sh, "  jedec:    %02X %02X %02X", jd.id[0], jd.id[1], jd.id[2]);
		if (jd.size != 0u)
			cli_print(sh, " -> %lu MB", (unsigned long)(jd.size >> 20));
		cli_print(sh, "  (experimental; cross-check only)\r\n");
		if (jd.size != 0u && size != 0u && jd.size != size)
			cli_warn(sh, "  NOTE: JEDEC disagrees with the wrap probe -- trusting the "
			          "wrap probe (%lu MB)\r\n", (unsigned long)(size >> 20));
	} else {
		cli_print(sh, "  jedec:    not available (the 0x21 0x9F command shape is a "
		          "hypothesis, not confirmed by the reference tool)\r\n");
	}

recover:
	flash_session_recover(sh);
	rtl_link_hw_release(sh);
	return ok ? 0 : 1;
}
/*
 * YMODEM byte source over the RTL8720 flash (issue #19 M4).
 *
 * ymodem_send() pulls; rtl_dl_read_flash() streams whole sectors, so we refill a
 * chunk-sized staging buffer and serve slices out of it.
 *
 * 4 KB = one sector = the single-sector reads proven in M2.  It used to be 32 KB
 * (8 sectors), which cut the per-command overhead on the wire by 8x -- but that
 * bought throughput for a manual, non-destructive backup at the price of 28 KB of
 * AXI-SRAM held permanently, and AXI-SRAM is the only memory a bus master can
 * reach (issue #46).  `wifi flash backup` is now 8x more read commands over a
 * 6 Mbaud link and nobody will notice; the camera needed the bytes.
 *
 * NOTE the NULL abort hook in bak_src_read(): while ymodem_send() runs, its io_getc()
 * is the ONLY permitted reader of the console RX ring.  rtl_abort_cb() would call
 * cli_cancel_poll(), which drains that ring and discards every non-0x03 byte -- it
 * would eat the receiver's ACK/'C'/CAN and break the transfer.  Ctrl+C during the
 * transfer is handled by io_getc() instead (cmd_xfer.c).
 */
#define RTL_BACKUP_CHUNK  4096u

static uint8_t s_bak_chunk[RTL_BACKUP_CHUNK];   /* static: off the 4 KB shell stack */

struct rtl_bak_src {
	uint32_t base;        /* flash offset of stream byte 0 (4 KB-aligned) */
	uint32_t total;       /* bytes to send */
	uint32_t pos;         /* bytes served so far */
	uint32_t chunk_pos;   /* stream position of s_bak_chunk[0] */
	uint32_t chunk_len;   /* valid bytes in s_bak_chunk (0 = empty) */
	/* Running digest of everything served, in the module's own 0x27 algorithm, so the
	 * backup can be verified against the device without re-reading the flash.  The
	 * algorithm lives in one place (app/rtl8720_flash.c) and is shared with the M5
	 * staged-image checker -- see struct rtl_dl_digest. */
	struct rtl_dl_digest dg;
	int      failed;      /* sticky: a flash read failed */
};

static int bak_src_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct rtl_bak_src *s = (struct rtl_bak_src *)ctx;
	uint32_t avail, n;

	*got = 0u;
	if (s->pos >= s->total)
		return 0;                                   /* EOF */

	if (s->chunk_len == 0u || s->pos >= s->chunk_pos + s->chunk_len) {
		uint32_t remain = s->total - s->pos;
		uint32_t take   = (remain > RTL_BACKUP_CHUNK) ? RTL_BACKUP_CHUNK : remain;
		uint32_t secs   = (take + 4095u) / 4096u;   /* reads are whole sectors */
		int rc = rtl_dl_read_flash(s->base + s->pos, secs, s_bak_chunk,
		                           sizeof(s_bak_chunk), NULL, NULL);

		if (rc < (int)take) {                       /* short/failed read */
			s->failed = 1;
			return -1;
		}
		s->chunk_pos = s->pos;
		s->chunk_len = take;
	}

	avail = s->chunk_len - (s->pos - s->chunk_pos);
	n = (want < avail) ? want : avail;
	if (n > s->total - s->pos)
		n = s->total - s->pos;
	memcpy(dst, s_bak_chunk + (s->pos - s->chunk_pos), n);
	rtl_dl_digest_add(&s->dg, dst, n);
	s->pos += n;
	*got = n;
	return 0;
}

/* Append @v as @digits uppercase hex digits at @p; returns the new write position. */
static char *bak_put_hex(char *p, uint32_t v, int digits)
{
	static const char hex[] = "0123456789ABCDEF";

	for (int i = digits - 1; i >= 0; i--)
		*p++ = hex[(v >> (4 * i)) & 0xFu];
	return p;
}

/* Build the deterministic YMODEM block-0 filename "rtl8720_<off6>_<len6>.bin" into
 * @buf (needs >= 28 bytes).  No printf: the shell has no snprintf. */
static void bak_build_name(char *buf, uint32_t off, uint32_t len)
{
	const char *pre = "rtl8720_", *suf = ".bin";
	char *p = buf;

	while (*pre)
		*p++ = *pre++;
	p = bak_put_hex(p, off, 6);
	*p++ = '_';
	p = bak_put_hex(p, len, 6);
	while (*suf)
		*p++ = *suf++;
	*p = '\0';
}

/* wifi flash backup [offset] [len] (issue #19, M4): NON-DESTRUCTIVE full-chip backup.
 * Streams the flash to the PC over the console with YMODEM (receive with `rz`).
 * Defaults to the whole chip as detected by the address-wrap probe. */
static int cmd_flash_backup(struct cli_instance *sh, int argc, char **argv)
{
	struct rtl_dl_size sz;
	struct rtl_bak_src src_ctx;
	struct ym_source   src;
	char name[32];
	uint32_t offset = 0u, len = 0u, size, devsum = 0u;
	int rc, ok = 0;

	if (argc >= 2 && (cli_parse_u32(argv[1], &offset) != 0 || (offset & 0xFFFu) != 0u)) {
		cli_error(sh, "wifi: bad offset (4KB-aligned, e.g. 0x0)\r\n");
		return 1;
	}
	if (argc >= 3 && (cli_parse_u32(argv[2], &len) != 0 || len == 0u || (len & 0xFFFu) != 0u)) {
		cli_error(sh, "wifi: bad length (4KB-aligned, non-zero, e.g. 0x1000)\r\n");
		return 1;
	}
	/* Reject a range the protocol cannot express (24-bit offsets) BEFORE powering
	 * anything up: otherwise an explicit oversized range on a chip whose capacity we
	 * failed to detect would start the YMODEM transfer and then die part-way through,
	 * leaving the receiver holding a truncated file. */
	if (offset >= RTL_DL_FLASH_LIMIT ||
	    (len != 0u && len > RTL_DL_FLASH_LIMIT - offset)) {
		cli_error(sh, "wifi: range past the protocol's 16 MB (24-bit offset) limit\r\n");
		return 1;
	}
	if (flash_claim(sh) != 0)
		return 1;

	cli_print(sh, "wifi: flash backup (NON-DESTRUCTIVE)...\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;

	rc = rtl_dl_detect_size(&sz, rtl_abort_cb, sh);
	size = flash_report_size(sh, &sz, rc);
	if (rc == -2)
		goto recover;
	if (len == 0u) {
		if (size == 0u) {
			cli_error(sh, "wifi: capacity unknown -- pass an explicit length, "
			          "e.g. `wifi flash backup 0x0 0x200000`\r\n");
			goto recover;
		}
		len = size - offset;
	}
	if (size != 0u && (offset >= size || len > size - offset)) {
		cli_error(sh, "wifi: range 0x%lX+0x%lX is past the detected 0x%lX capacity\r\n",
		          (unsigned long)offset, (unsigned long)len, (unsigned long)size);
		goto recover;
	}

	memset(&src_ctx, 0, sizeof(src_ctx));
	src_ctx.base = offset;
	src_ctx.total = len;
	rtl_dl_digest_init(&src_ctx.dg);

	bak_build_name(name, offset, len);
	src.ctx = &src_ctx; src.name = name; src.size = len; src.read = bak_src_read;

	cli_print(sh, "wifi: sending '%s' (%lu bytes) -- start the receiver now "
	          "(`rz`, or Ctrl+A Ctrl+R in picocom)\r\n", name, (unsigned long)len);
	/* From here until the transfer ends, io_getc() owns the console RX (see above). */
	rc = xfer_send_source_locked(sh, &src);
	if (rc != 0) {
		if (src_ctx.failed)
			cli_error(sh, "wifi: flash read failed %lu bytes in\r\n",
			          (unsigned long)src_ctx.pos);
		goto recover;
	}
	cli_print(sh, "  host digest:   0x%08lX (u32-LE word sum of the bytes sent)\r\n",
	          (unsigned long)rtl_dl_digest_value(&src_ctx.dg));

	/* LAST operation of the session: the module's own digest over the same range, in
	 * the same algorithm -- so this is an END-TO-END VERIFY of the backup that does not
	 * depend on re-reading the flash.  A timeout poisons the session, so we only ever
	 * fall through to recover from here. */
	rc = rtl_dl_flash_chksum(offset, len, 30000u, &devsum, rtl_abort_cb, sh);
	if (rc != 0) {
		cli_warn(sh, "  device digest: n/a (rc %d) -- backup UNVERIFIED\r\n", rc);
	} else if (devsum == rtl_dl_digest_value(&src_ctx.dg)) {
		cli_print(sh, "  device digest: 0x%08lX -- VERIFIED (matches)\r\n",
		          (unsigned long)devsum);
		ok = 1;
	} else {
		cli_error(sh, "  device digest: 0x%08lX -- MISMATCH, the backup is NOT trustworthy\r\n",
		          (unsigned long)devsum);
	}

recover:
	flash_session_recover(sh);
	rtl_link_hw_release(sh);
	return ok ? 0 : 1;
}
/* ------------------------------------------------------------------ *
 *  issue #19 M5: image staging (host -> PSRAM) + programming the module
 * ------------------------------------------------------------------ */

/* Print the staged-image record, re-reading PSRAM to catch a clobber.  Returns 1 when a
 * valid image is present AND still intact, 0 otherwise (message already emitted). */
static int img_report(struct cli_instance *sh)
{
	const struct rtl_img *im = rtl_img_get();
	uint32_t              now;
	int                   i;

	if (!im->valid) {
		cli_warn(sh, "wifi: no image staged -- run `wifi flash imgload` first\r\n");
		return 0;
	}
	cli_print(sh, "  name:    '%s'\r\n", im->name);
	cli_print(sh, "  size:    %lu bytes (padded to %lu = %lu sectors with 0xFF)\r\n",
	          (unsigned long)im->len, (unsigned long)im->padded_len,
	          (unsigned long)(im->padded_len / 4096u));
	cli_print(sh, "  digest:  0x%08lX (device 0x27 algorithm, over the padded range)\r\n",
	          (unsigned long)im->digest);
	cli_print(sh, "  first16:");
	for (i = 0; i < 16; i++)
		cli_print(sh, " %02X", rtl_img_data()[i]);
	cli_print(sh, "%s\r\n",
	          memcmp(rtl_img_data(), rtl_dl_km0_magic, RTL_DL_KM0_MAGIC_LEN) == 0
	                  ? "   (AmebaD km0_boot magic)" : "");

	now = rtl_img_verify();
	if (now != im->digest) {
		cli_error(sh, "  RECHECK: 0x%08lX -- PSRAM was CLOBBERED since the load "
		          "(another command used it); re-run `wifi flash imgload`\r\n",
		          (unsigned long)now);
		return 0;
	}
	cli_print(sh, "  recheck: 0x%08lX -- intact\r\n", (unsigned long)now);
	return 1;
}

/*
 * wifi flash imgload (issue #19, M5): receive a firmware image from the PC over YMODEM into
 * the PSRAM staging buffer.  Touches NO RTL8720 hardware at all -- this is purely the
 * host-to-board transfer, and it is what makes the stock backup restorable.
 *
 * Console RX ownership: from cli_console_claim() until xfer_recv_sink_locked() returns,
 * the ONLY reader of the console RX ring is that helper's io_getc().  The sink must not
 * poll cli_cancel_requested() (see cmd_xfer.h) -- here it physically cannot, since
 * rtl_img_sink() has no abort hook to pass.
 */
static int cmd_flash_imgload(struct cli_instance *sh, int argc, char **argv)
{
	const struct rtl_img *im;
	uint32_t              drops0;
	int                   rc, ok = 0;

	(void)argc; (void)argv;

	if (!psram_ready()) {
		cli_error(sh, "wifi: PSRAM is not available -- nowhere to stage the image\r\n");
		return 1;
	}
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	/* Hold the OCTOSPI1 guard for the WHOLE transfer so a backgrounded psram/membench
	 * job cannot overwrite the staging area while it fills. */
	if (!psram_acquire()) {
		cli_console_release(sh);
		cli_error(sh, "wifi: PSRAM is busy (another command holds it, or the "
		              "LCD is scanning out of it -- run `lcd off`)\r\n");
		return 1;
	}

	rc = rtl_img_probe();               /* invalidates, then proves PSRAM stores data */
	if (rc != 0) {
		cli_error(sh, "wifi: PSRAM staging self-check failed (rc %d)\r\n", rc);
		goto out;
	}

	drops0 = sh->rx_dropped;
	cli_print(sh, "wifi: staging an RTL8720 image in PSRAM @0x%08lX (max %lu bytes). "
	          "NOTHING is written to the module.\r\n",
	          (unsigned long)PSRAM_BASE_ADDR, (unsigned long)RTL_IMG_MAX);
	/* xfer_recv_sink_locked() does not print this (its caller owns the console), but
	 * the handshake budget is only long enough if the operator starts now. */
	/* Not "Ctrl+C aborts": a receive has no local abort (cmd_xfer.h) -- 0x03 is
	 * ordinary file data and cannot be told apart from a keypress. */
	cli_print(sh, "wifi: start the sender now -- `sb <file>` (lrzsz YMODEM batch send; "
	          "`sz` will NOT work), or Ctrl+A Ctrl+S in picocom; cancel the sender on "
	          "the PC to abort\r\n");
	rc = xfer_recv_sink_locked(sh, rtl_img_sink());

	/* Print the transfer post-mortem BEFORE branching, so a failure is as
	 * informative as a success -- the counters say which layer broke (see
	 * struct ym_recv_diag). */
	{
		const struct ym_recv_diag *d = ymodem_recv_diag();

		cli_print(sh, "  ymodem: %lu blocks ok, %lu bad-crc, %lu bad-seq, "
		          "%lu short-read, %lu header-timeouts\r\n",
		          (unsigned long)d->blocks, (unsigned long)d->bad_crc,
		          (unsigned long)d->bad_seq, (unsigned long)d->short_read,
		          (unsigned long)d->timeouts);
		if (d->first_kind >= 0)
			cli_print(sh, "  first bad block: kind 0x%02X seq %d ~seq %d, "
			          "body %lu/%lu B, crc want %04X got %04X\r\n",
			          (unsigned)d->first_kind, d->first_seq, d->first_nseq,
			          (unsigned long)d->first_got, (unsigned long)d->first_want,
			          d->first_crc_want, d->first_crc_got);
		/* The console backend drops -- and counts -- a byte when its RX ring
		 * overruns.  A non-zero count here means the loss is below YMODEM. */
		cli_print(sh, "  rx drops during the transfer: %lu%s\r\n",
		          (unsigned long)(sh->rx_dropped - drops0),
		          (sh->rx_dropped - drops0) ? "  <-- NOT CLEAN" : "  (clean)");
		/* Also to the log ring: the PC's terminal is still attached to `sb` when
		 * these lines go out, so `dmesg` is where they can actually be read. */
		log_write((sh->rx_dropped - drops0) ? LOG_LEVEL_WRN : LOG_LEVEL_INF, "wifi",
		          "imgload rc=%d rx_drops=%lu", rc,
		          (unsigned long)(sh->rx_dropped - drops0));
	}

	if (rc != 0) {
		/* Includes "all the data arrived but the batch never closed" -- not a
		 * complete image, so it must not be left staged. */
		rtl_img_invalidate();
		goto out;
	}
	if (rtl_img_finish() != 0) {
		cli_error(sh, "wifi: empty transfer -- nothing staged\r\n");
		goto out;
	}

	im = rtl_img_get();
	cli_print(sh, "wifi: staged '%s', %lu bytes\r\n",
	          im->name, (unsigned long)im->len);
	cli_print(sh, "  padded: %lu bytes (%lu sectors, 0xFF filled)\r\n",
	          (unsigned long)im->padded_len, (unsigned long)(im->padded_len / 4096u));
	cli_print(sh, "  digest: 0x%08lX -- compare with the host-side u32-LE word sum\r\n",
	          (unsigned long)im->digest);
	ok = 1;

out:
	psram_release();
	cli_console_release(sh);
	return ok ? 0 : 1;
}

/* wifi flash imginfo (issue #19, M5): show the staged image and re-verify it against PSRAM. */
static int cmd_flash_imginfo(struct cli_instance *sh, int argc, char **argv)
{
	int ok;

	(void)argc; (void)argv;

	if (!psram_acquire()) {
		cli_error(sh, "wifi: PSRAM is busy (another command holds it, or the "
		              "LCD is scanning out of it -- run `lcd off`)\r\n");
		return 1;
	}
	ok = img_report(sh);
	psram_release();
	return ok ? 0 : 1;
}
/*
 * wifi flash write <offset> confirm (issue #19, M5): DESTRUCTIVE.  Erase and program the
 * staged image into the RTL8720DN's flash at <offset>, then verify it with the module's
 * own digest.  THIS IS THE COMMAND THAT CAN REWRITE THE MODULE'S BOOT SECTORS.
 *
 * The gates are layered on purpose (see rtl_dl_flash_program): here we require the
 * literal `confirm` token and re-verify the staged image against PSRAM, and the protocol
 * layer enforces alignment, the 2 MB destructive cap, the AmebaD boot magic at offset 0,
 * and the detected chip capacity.  Recovery if this ever goes wrong: re-enter download
 * mode (mask ROM -- always possible) and re-run with the full 2 MB stock backup staged.
 */
static int cmd_flash_write(struct cli_instance *sh, int argc, char **argv)
{
	const struct rtl_img *im = rtl_img_get();
	struct rtl_dl_program pr;
	uint32_t              offset, len;
	int                   rc, ok = 0;

	if (cli_parse_u32(argv[1], &offset) != 0) {
		cli_error(sh, "wifi: bad offset (hex, e.g. 0x0)\r\n");
		return 1;
	}
	if (!im->valid) {
		cli_error(sh, "wifi: no image staged -- run `wifi flash imgload` first\r\n");
		return 1;
	}
	len = im->padded_len;
	if (argc < 3 || strcmp(argv[2], "confirm") != 0) {
		cli_error(sh, "wifi: DESTRUCTIVE -- erases and rewrites %lu bytes of RTL8720 "
		          "flash at 0x%lX.\r\n", (unsigned long)len, (unsigned long)offset);
		cli_print(sh, "  re-run `wifi flash write 0x%lX confirm` to proceed\r\n",
		          (unsigned long)offset);
		return 1;
	}
	/* The factory WiFi-settings sector holds the SSID and a plaintext PSK; erasing it
	 * is legitimate for a full-chip restore but must never be a surprise. */
	if (offset <= 0x105000u && 0x105000u < offset + len)
		cli_warn(sh, "wifi: NOTE this range covers 0x105000, the factory WiFi settings "
		         "sector -- the stored SSID/password will be replaced\r\n");

	if (flash_claim(sh) != 0)
		return 1;
	if (!psram_acquire()) {                 /* the image is read straight out of PSRAM */
		rtl_link_hw_release(sh);
		cli_error(sh, "wifi: PSRAM is busy (another command holds it, or the "
		              "LCD is scanning out of it -- run `lcd off`)\r\n");
		return 1;
	}
	/* Last gate before any hardware moves: the staged bytes must still be the ones we
	 * digested at load time. */
	if (rtl_img_verify() != im->digest) {
		cli_error(sh, "wifi: staged image no longer matches its digest (PSRAM was "
		          "clobbered) -- re-run `wifi flash imgload`\r\n");
		goto out;
	}

	cli_print(sh, "wifi: programming '%s' -> flash 0x%lX..0x%lX (%lu sectors), "
	          "DESTRUCTIVE; power-cycles the module to verify...\r\n",
	          im->name, (unsigned long)offset, (unsigned long)(offset + len - 1u),
	          (unsigned long)(len / 4096u));
	/* erase + transfer + re-enter + digest; there is no progress output in between,
	 * so say so rather than let a long silence look like a hang. */
	cli_print(sh, "  (silent for roughly %lu s: erase, block transfer, power-cycle, "
	          "digest. Ctrl+C aborts.)\r\n",
	          (unsigned long)(10u + (len / 4096u) / 8u + (len >> 17)));
	rc = rtl_dl_flash_program(offset, rtl_img_data(), len, 30000u, &pr,
	                          rtl_abort_cb, sh);

	switch (rc) {
	case -1:
		cli_error(sh, "wifi: bad range -- 4KB-aligned and within 0x%lX required\r\n",
		          (unsigned long)0x200000u);
		goto recover;
	case -2:
		cli_error(sh, "wifi: download / flashloader setup failed\r\n");
		goto recover;
	case -3:
		cli_error(sh, "wifi: refusing -- writing offset 0 requires an AmebaD km0_boot "
		          "image (magic 99 99 96 96 3F CC 66 FC)\r\n");
		goto recover;
	case -4:
		cli_error(sh, "wifi: range past the detected 0x%lX capacity -- nothing "
		          "erased\r\n", (unsigned long)pr.cap);
		goto recover;
	case -10:
		cli_error(sh, "wifi: could not read the flash to size it -- nothing erased. "
		          "Retry; if it persists, check the link with `wifi flash info`\r\n");
		goto recover;
	default:
		break;
	}

	if (pr.cap_known)
		cli_print(sh, "  capacity: %lu MB (address wrap)\r\n",
		          (unsigned long)(pr.cap >> 20));
	cli_print(sh, "  erase:  %s (%lu/%lu sectors)\r\n", pr.erase_ok ? "OK" : "FAIL",
	          (unsigned long)pr.erased, (unsigned long)pr.sectors);
	cli_print(sh, "  write:  %s (%lu bytes)\r\n", pr.write_ok ? "OK" : "FAIL",
	          (unsigned long)pr.written);
	cli_print(sh, "  host digest:   0x%08lX\r\n", (unsigned long)pr.host_sum);
	if (rc == -8) {
		cli_error(sh, "  device digest: n/a -- UNVERIFIED\r\n");
	} else if (rc == -9) {
		cli_error(sh, "  device digest: 0x%08lX -- MISMATCH\r\n",
		          (unsigned long)pr.dev_sum);
	} else if (rc == 0) {
		cli_print(sh, "  device digest: 0x%08lX -- VERIFIED (matches)\r\n",
		          (unsigned long)pr.dev_sum);
	}

	if (rc == 0) {
		cli_print(sh, "wifi: PROGRAMMED and verified\r\n");
		ok = 1;
	} else {
		cli_error(sh, "wifi: FAILED (rc %d)\r\n", rc);
		cli_print(sh, "  the range is now INDETERMINATE. Re-run "
		          "`wifi flash write 0x%lX confirm` (erase+write is idempotent); if it "
		          "keeps failing, stage the full 2 MB backup and write it at 0x0.\r\n",
		          (unsigned long)offset);
	}

recover:
	flash_session_recover(sh);
out:
	psram_release();
	rtl_link_hw_release(sh);
	return ok ? 0 : 1;
}

/* ---- registration -------------------------------------------------------- */

/*
 * Registered UNDER `wifi` (cmd_wifi.c holds the root table; cmd_wifi_priv.h is the
 * seam), so this cannot be CLI_SUBCMD_SET_CREATE -- that macro makes the array static.
 *
 * Order is the order the commands are USED, which is what the flat names could not
 * express: identify the chip, survey it, save it, stage a replacement, check the
 * staging, then write.  `write` last is deliberate -- it is the only destructive one.
 */
const struct cli_cmd wifi_flash_subcmds[] = {
	CLI_CMD_ARG(info,    NULL, "identify the chip: capacity / status regs / checksum",
	            cmd_flash_info,    1, 0),
	CLI_CMD_ARG(read,    NULL, "survey sectors <offset> [nsectors] (non-destructive)",
	            cmd_flash_read,    2, 1),
	CLI_CMD_ARG(backup,  NULL, "back the flash up to the PC over YMODEM [offset] [len]",
	            cmd_flash_backup,  1, 2),
	CLI_CMD_ARG(imgload, NULL, "receive a firmware image from the PC into PSRAM (YMODEM `sb`)",
	            cmd_flash_imgload, 1, 0),
	CLI_CMD_ARG(imginfo, NULL, "show + re-verify the staged firmware image",
	            cmd_flash_imginfo, 1, 0),
	CLI_CMD_ARG(write,   NULL, "DESTRUCTIVE program staged image: write <offset> confirm",
	            cmd_flash_write,   2, 1),
	CLI_SUBCMD_SET_END
};
