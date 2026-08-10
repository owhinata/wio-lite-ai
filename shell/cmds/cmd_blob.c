/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_blob.c
 * @brief   `blob` shell command: the NOR asset region (issue #10 / issue #9 P2b).
 *
 *   blob list                    every slot: state, size, CRC32, name
 *   blob info <slot>             one slot in detail, with its device addresses
 *   blob write <slot>            erase the slot, receive a file over YMODEM, verify
 *   blob verify <slot>           re-read the payload and check it against the CRC32
 *   blob erase <slot>            erase the slot -- destructive
 *   blob read <slot> <off> [len] hexdump of the payload (debugging)
 *
 * A third sibling alongside `nor` (the raw device) and `kv` (the configuration
 * store), not a subcommand of either -- see app/blob.h.  The blob is named by the
 * SENDER: `sb model.tflite` stores it as "model.tflite", so there is no name to
 * type and no way for the name on the board to disagree with the file that was
 * sent.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
/* cli_instance.h (ThreadX-aware) gives the full struct cli_instance, which `blob
 * write` needs for sh->rx_dropped: the console backend silently drops -- and counts
 * -- a byte when its RX ring overruns, and that is the one loss YMODEM's own
 * counters cannot attribute, so it has to be reported rather than assumed zero. */
#include "cli_instance.h"
#include "blob.h"
#include "cmd_xfer.h"
#include "net_shell.h"   /* refuse a raw transfer on the telnet console */
#include "nor_flash.h"
#include "log.h"

#include <stdint.h>
#include <string.h>

static const char *blob_strerror(int rc)
{
	switch (rc) {
	case BLOB_ERR_PARAM: return "bad argument";
	case BLOB_ERR_IO:    return "NOR read/program/erase failed";
	case BLOB_ERR_STATE: return "NOR device down, or no transfer armed";
	case BLOB_ERR_BUSY:  return "another blob operation is running";
	case BLOB_ERR_EMPTY: return "no valid blob in that slot";
	case BLOB_ERR_CRC:   return "CRC32 mismatch";
	case BLOB_ERR_SHORT: return "fewer bytes arrived than the sender declared";
	default:             return "unknown error";
	}
}

static const char *blob_state_str(enum blob_state s)
{
	switch (s) {
	case BLOB_VALID:      return "valid";
	case BLOB_INCOMPLETE: return "incomplete";
	case BLOB_INVALID:    return "invalid";
	default:              return "empty";
	}
}

/* Refuse every subcommand while the device is down, with one message. */
static int blob_gate(struct cli_instance *sh)
{
	if (!nor_flash_ready()) {
		cli_error(sh, "blob: NOR device down (see `nor info`)\r\n");
		return -1;
	}
	return 0;
}

static int blob_parse_slot(struct cli_instance *sh, const char *s, unsigned *out)
{
	uint32_t n;

	if (cli_parse_u32(s, &n) != 0 || n >= BLOB_SLOT_COUNT) {
		cli_error(sh, "blob: bad slot (0 .. %lu)\r\n",
		          (unsigned long)(BLOB_SLOT_COUNT - 1u));
		return -1;
	}
	*out = (unsigned)n;
	return 0;
}

/* ---- subcommands --------------------------------------------------------- */

static int cmd_blob_list(struct cli_instance *sh, int argc, char **argv)
{
	unsigned slot;
	unsigned used = 0u;

	(void)argc; (void)argv;
	if (blob_gate(sh))
		return 1;

	cli_print(sh, "slot  addr      state        size      crc32     name\r\n");
	for (slot = 0u; slot < BLOB_SLOT_COUNT; slot++) {
		struct blob_info info;
		int rc = blob_stat(slot, &info);

		if (rc != BLOB_OK) {
			cli_error(sh, "%4u  %06lX    <%s>\r\n", slot,
			          (unsigned long)blob_slot_addr(slot), blob_strerror(rc));
			continue;
		}
		cli_print(sh, "%4u  %06lX    %-11s", slot,
		          (unsigned long)blob_slot_addr(slot),
		          blob_state_str(info.state));
		if (info.state == BLOB_VALID) {
			used++;
			cli_print(sh, "%8lu  %08lX  %s",
			          (unsigned long)info.length,
			          (unsigned long)info.crc32,
			          (info.name[0] != '\0') ? info.name : "-");
		}
		cli_print(sh, "\r\n");
	}
	cli_print(sh, "%u of %lu slot%s in use, %lu KB usable per slot\r\n",
	          used, (unsigned long)BLOB_SLOT_COUNT,
	          (used == 1u) ? "" : "s",
	          (unsigned long)(BLOB_PAYLOAD_MAX >> 10));
	return 0;
}

static int cmd_blob_info(struct cli_instance *sh, int argc, char **argv)
{
	struct blob_info info;
	unsigned slot;
	int rc;

	(void)argc;
	if (blob_gate(sh) || blob_parse_slot(sh, argv[1], &slot))
		return 1;
	rc = blob_stat(slot, &info);
	if (rc != BLOB_OK) {
		cli_error(sh, "blob: %s\r\n", blob_strerror(rc));
		return 1;
	}

	cli_print(sh, "blob slot %u:\r\n", slot);
	cli_print(sh, "  state    : %s\r\n", blob_state_str(info.state));
	cli_print(sh, "  header   : 0x%06lX (%lu B sector, reserved)\r\n",
	          (unsigned long)blob_slot_addr(slot), (unsigned long)BLOB_HDR_SIZE);
	cli_print(sh, "  payload  : 0x%06lX .. 0x%06lX (%lu B usable)\r\n",
	          (unsigned long)blob_payload_addr(slot),
	          (unsigned long)(blob_slot_addr(slot) + BLOB_SLOT_SIZE - 1u),
	          (unsigned long)BLOB_PAYLOAD_MAX);
	if (info.state != BLOB_VALID) {
		/* Say what the state means rather than leaving a bare word: `incomplete`
		 * in particular names a specific accident, not a spelling of `empty`. */
		if (info.state == BLOB_INCOMPLETE)
			cli_print(sh, "  (a header body was written but never given its "
			          "magic -- power was lost mid-commit; re-send)\r\n");
		else if (info.state == BLOB_INVALID)
			cli_print(sh, "  (a header is present but does not decode -- not "
			          "written by this firmware, or damaged; `blob erase %u`)\r\n",
			          slot);
		else
			cli_print(sh, "  (nothing stored -- `blob write %u`)\r\n", slot);
		return 1;
	}
	cli_print(sh, "  name     : %s\r\n",
	          (info.name[0] != '\0') ? info.name : "-");
	cli_print(sh, "  size     : %lu B (%lu KB), %lu B of the slot free\r\n",
	          (unsigned long)info.length, (unsigned long)(info.length >> 10),
	          (unsigned long)(BLOB_PAYLOAD_MAX - info.length));
	/* Be precise about what `valid` claims.  It means the HEADER decodes -- and,
	 * because of the two-step commit, that the header was read back when it was
	 * written.  It does not re-read 508 KB of payload on every listing.  `blob write`
	 * verifies once automatically; anything that could have disturbed the flash
	 * since (or a power loss between the commit and that verify) is what this line
	 * points at. */
	cli_print(sh, "  crc32    : %08lX (CRC-32/ISO-HDLC of the payload, as sent -- "
	          "`blob verify %u` checks the flash against it)\r\n",
	          (unsigned long)info.crc32, slot);
	return 0;
}

static int cmd_blob_verify(struct cli_instance *sh, int argc, char **argv)
{
	struct blob_info info;
	unsigned slot;
	uint32_t crc = 0u;
	int rc;

	(void)argc;
	if (blob_gate(sh) || blob_parse_slot(sh, argv[1], &slot))
		return 1;
	if (blob_stat(slot, &info) == BLOB_OK && info.state == BLOB_VALID)
		cli_print(sh, "blob: re-reading %lu B from slot %u...\r\n",
		          (unsigned long)info.length, slot);

	rc = blob_verify(slot, &crc);
	if (rc == BLOB_OK) {
		cli_print(sh, "blob: slot %u PASS (crc32 %08lX)\r\n", slot,
		          (unsigned long)crc);
		return 0;
	}
	if (rc == BLOB_ERR_CRC)
		cli_error(sh, "blob: slot %u FAIL -- read back %08lX, header says %08lX\r\n",
		          slot, (unsigned long)crc, (unsigned long)info.crc32);
	else
		cli_error(sh, "blob: %s\r\n", blob_strerror(rc));
	return 1;
}

static int cmd_blob_erase(struct cli_instance *sh, int argc, char **argv)
{
	unsigned slot;
	int rc;

	(void)argc;
	if (blob_gate(sh) || blob_parse_slot(sh, argv[1], &slot))
		return 1;
	/*
	 * No `confirm` token, unlike `kv format yes`.  The two are not comparable: a
	 * format destroys every setting on the board and some of them (a PSK) cannot be
	 * recovered from anywhere, while a blob is an asset that came from a file the
	 * PC still has, in a slot the operator named explicitly on the line.  `nor
	 * erase` is likewise unconfirmed.
	 */
	if (blob_busy_acquire() != BLOB_OK) {
		cli_error(sh, "blob: %s\r\n", blob_strerror(BLOB_ERR_BUSY));
		return 1;
	}
	cli_print(sh, "blob: erasing slot %u (%lu KB)...\r\n", slot,
	          (unsigned long)(BLOB_SLOT_SIZE >> 10));
	rc = blob_erase(slot);
	blob_busy_release();

	if (rc != BLOB_OK) {
		cli_error(sh, "blob: erase failed: %s\r\n", blob_strerror(rc));
		return 1;
	}
	cli_print(sh, "blob: slot %u erased\r\n", slot);
	return 0;
}

static int cmd_blob_read(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t buf[NOR_PAGE_SIZE];        /* 256 B: the shell thread has a 4 KB stack */
	unsigned slot;
	uint32_t off, len = 64u, done = 0u;

	if (blob_gate(sh) || blob_parse_slot(sh, argv[1], &slot))
		return 1;
	if (cli_parse_u32(argv[2], &off) != 0 || off >= BLOB_PAYLOAD_MAX) {
		cli_error(sh, "blob: bad offset (0 .. %lu)\r\n",
		          (unsigned long)(BLOB_PAYLOAD_MAX - 1u));
		return 1;
	}
	if (argc > 3 && (cli_parse_u32(argv[3], &len) != 0 || len == 0u || len > 1024u)) {
		cli_error(sh, "blob: bad length (1 .. 1024)\r\n");
		return 1;
	}
	if (len > BLOB_PAYLOAD_MAX - off)
		len = BLOB_PAYLOAD_MAX - off;

	while (done < len) {
		uint32_t chunk = len - done;
		int rc;

		if (chunk > sizeof buf)
			chunk = sizeof buf;
		rc = blob_read(slot, off + done, buf, chunk);
		if (rc != BLOB_OK) {
			cli_error(sh, "blob: read failed at +%lu: %s\r\n",
			          (unsigned long)(off + done), blob_strerror(rc));
			return 1;
		}
		/* Base the dump on the offset WITHIN the blob, which is what the command
		 * takes as an argument.  `blob info` prints the device address for anyone
		 * who wants to cross-check the same bytes with `nor read`. */
		cli_hexdump_base(sh, buf, chunk, off + done);
		done += chunk;
	}
	return 0;
}

/* ---- blob write ---------------------------------------------------------- */

/*
 * Receive a file into a slot.  The order of the gates is the whole design of this
 * handler, because the erase is DESTRUCTIVE AND UNCONDITIONAL: everything that can
 * refuse the command is asked before anything is erased.  Refusing after the erase
 * would destroy a stored blob to tell the operator they typed a trailing `&`.
 *
 * So: device up -> slot parses -> not the telnet console -> no other blob operation
 * -> the console can actually be claimed -> ONLY THEN erase.
 *
 * That is also why this uses xfer_recv_sink_locked() and claims the console itself,
 * rather than xfer_recv_sink() which claims it internally: the claim has to happen
 * before the erase, and holding it afterwards keeps the automatic verify's output
 * from interleaving with another session's.
 */
static int cmd_blob_write(struct cli_instance *sh, int argc, char **argv)
{
	struct blob_info info;
	unsigned slot;
	uint32_t drops0, crc = 0u;
	int rc, ok = 0;

	(void)argc;
	if (blob_gate(sh) || blob_parse_slot(sh, argv[1], &slot))
		return 1;
	/* Asked here as well as inside xfer_recv_sink_locked(): there it would fire
	 * after the erase.  Passing twice costs one predicate. */
	if (net_shell_guard(sh, "ymodem"))
		return 1;
	if (blob_busy_acquire() != BLOB_OK) {
		cli_error(sh, "blob: %s\r\n", blob_strerror(BLOB_ERR_BUSY));
		return 1;
	}
	switch (cli_console_claim(sh)) {
	case 0:
		break;
	case -2:
		blob_busy_release();
		cli_error(sh, "blob: cannot run in the background -- "
		          "drop the trailing '&'\r\n");
		return 1;
	default:
		blob_busy_release();
		cli_error(sh, "blob: console busy\r\n");
		return 1;
	}

	cli_print(sh, "blob: erasing slot %u (%lu KB, about 0.6 s) -- anything stored "
	          "there is gone even if the transfer fails\r\n", slot,
	          (unsigned long)(BLOB_SLOT_SIZE >> 10));
	rc = blob_recv_arm(slot);
	if (rc != BLOB_OK) {
		cli_error(sh, "blob: erase failed: %s\r\n", blob_strerror(rc));
		goto out;
	}

	cli_print(sh, "blob: start the sender now -- `sb <file>` (lrzsz YMODEM batch "
	          "send; `sz` will NOT work), or Ctrl+A Ctrl+S in picocom\r\n");
	/* The one piece of operating advice that is not obvious and cannot be guessed:
	 * see the io_getc_recv() note in cmd_xfer.c for why a receive has no local
	 * Ctrl+C at all. */
	cli_print(sh, "  (Ctrl+C does NOT abort a receive -- cancel the sender on the "
	          "PC, or wait for the handshake to time out; max %lu B)\r\n",
	          (unsigned long)BLOB_PAYLOAD_MAX);

	drops0 = sh->rx_dropped;
	rc = xfer_recv_sink_locked(sh, blob_sink());

	/* Post-mortem before branching, so a failure is as informative as a success:
	 * the YMODEM counters say which layer broke, and the console RX drop count is
	 * the one loss they cannot see (see struct ym_recv_diag).  Both also go to the
	 * log ring -- xfer_recv_sink_locked() puts the YMODEM half there already --
	 * because the PC's terminal still belongs to `sb` while these lines go out, so
	 * `dmesg` is where they can actually be read. */
	{
		const struct ym_recv_diag *d = ymodem_recv_diag();

		cli_print(sh, "  ymodem: %lu blocks ok, %lu bad-crc, %lu bad-seq, "
		          "%lu short-read, %lu header-timeouts\r\n",
		          (unsigned long)d->blocks, (unsigned long)d->bad_crc,
		          (unsigned long)d->bad_seq, (unsigned long)d->short_read,
		          (unsigned long)d->timeouts);
		cli_print(sh, "  rx drops during the transfer: %lu%s\r\n",
		          (unsigned long)(sh->rx_dropped - drops0),
		          (sh->rx_dropped - drops0) ? "  <-- NOT CLEAN" : "  (clean)");
		log_write((sh->rx_dropped - drops0) ? LOG_LEVEL_WRN : LOG_LEVEL_INF,
		          "blob", "write slot %u rc=%d pos=%lu rx_drops=%lu", slot, rc,
		          (unsigned long)blob_recv_pos(),
		          (unsigned long)(sh->rx_dropped - drops0));
	}

	if (rc != 0) {
		/* Includes "every byte arrived but the batch never closed", which
		 * ymodem.h is explicit is NOT a complete file.  No header is written, so
		 * the slot stays empty and cannot be mistaken for a stored blob. */
		cli_error(sh, "blob: transfer failed after %lu B -- slot %u left empty "
		          "(no header was written)\r\n",
		          (unsigned long)blob_recv_pos(), slot);
		blob_recv_disarm();
		goto out;
	}

	rc = blob_recv_commit(&info);
	if (rc != BLOB_OK) {
		cli_error(sh, "blob: commit failed: %s\r\n", blob_strerror(rc));
		if (rc == BLOB_ERR_SHORT)
			cli_print(sh, "  the payload is on the flash but has no header, so "
			          "the slot still reads empty -- re-send\r\n");
		blob_recv_disarm();
		goto out;
	}
	cli_print(sh, "blob: stored '%s' in slot %u, %lu B, crc32 %08lX\r\n",
	          info.name, slot, (unsigned long)info.length,
	          (unsigned long)info.crc32);

	/* Automatic read-back.  The stored CRC was computed from the bytes that came
	 * off the wire, so this is the first thing that can catch a program that did
	 * not take -- and doing it here means a `blob write` that returns success has
	 * already been checked, rather than leaving it to a step the operator may skip. */
	cli_print(sh, "  verifying...\r\n");
	rc = blob_verify(slot, &crc);
	if (rc == BLOB_OK) {
		cli_print(sh, "  verify: PASS (read back %08lX)\r\n", (unsigned long)crc);
		ok = 1;
	} else if (rc == BLOB_ERR_CRC) {
		cli_error(sh, "  verify: FAIL -- read back %08lX, header says %08lX. The "
		          "slot is NOT trustworthy; erase and re-send\r\n",
		          (unsigned long)crc, (unsigned long)info.crc32);
	} else {
		cli_error(sh, "  verify: %s\r\n", blob_strerror(rc));
	}

out:
	cli_console_release(sh);
	blob_busy_release();
	return ok ? 0 : 1;
}

/* ---- registration -------------------------------------------------------- */

CLI_SUBCMD_SET_CREATE(blob_subcmds,
	CLI_CMD_ARG(list,   NULL, "every slot: state, size, crc32, name",       cmd_blob_list,   1, 0),
	CLI_CMD_ARG(info,   NULL, "one slot in detail <slot>",                  cmd_blob_info,   2, 0),
	CLI_CMD_ARG(write,  NULL, "ERASE <slot> then receive a file (YMODEM `sb`)", cmd_blob_write, 2, 0),
	CLI_CMD_ARG(verify, NULL, "re-read and check against the crc32 <slot>", cmd_blob_verify, 2, 0),
	CLI_CMD_ARG(erase,  NULL, "erase a slot <slot> -- destructive",         cmd_blob_erase,  2, 0),
	CLI_CMD_ARG(read,   NULL, "hexdump payload <slot> <off> [len]",         cmd_blob_read,   3, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(blob, blob_subcmds,
                 "read-only asset region on the external NOR (models etc.)", NULL, 1, 0);
