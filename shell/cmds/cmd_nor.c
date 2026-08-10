/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_nor.c
 * @brief   `nor` shell command: external W25Q128 on OCTOSPI2 (issue #37).
 *
 *   nor info               bring-up state, JEDEC id, geometry, device clock
 *   nor read <addr> [len]  hexdump from a device offset (default 64 B)
 *   nor erase <addr> [len] erase 4 KB sector(s) -- destructive
 *   nor write <addr> <hex> program bytes given as a hex string
 *   nor test [addr]        acceptance test for the partial-page programming that
 *                          the KV store's FDB_WRITE_GRAN = 8 depends on
 *
 * `nor test` is the reason this command exists.  The whole KV design rests on one
 * datasheet reading -- that "program at previously erased (FFh) memory locations"
 * (W25Q128JV sec 8.2.13) constrains the *state* of the target bytes and not the
 * *history* of the page, so several partial programs into disjoint bytes of one
 * page are in specification while re-programming an already-programmed byte is
 * not.  FlashDB with FDB_WRITE_GRAN = 8 only ever does the former.  The test
 * proves that on this device before any of it is trusted, and separately
 * *observes* (without requiring) what a same-byte overwrite does, because that is
 * the thing a future FDB_WRITE_GRAN = 1 would depend on.
 *
 * Addresses are device offsets (0 .. 16 MB), not 0x70000000 pointers: the window
 * is never memory-mapped and stays fenced off in the MPU (see port/nor).
 *
 * EVERY SUBCOMMAND HERE REACHES THE BARE DEVICE, past both of its tenants.  Two
 * ranges hold live data and nothing in this file will stop you from erasing them:
 * [0, 1 MB) is the `kv` configuration partition (app/kv.c) and [1 MB, 4 MB) is the
 * blob region's six asset slots (app/blob.c).  Use `kv` and `blob` for those; this
 * is the tool for looking at the device itself, and it is deliberately unguarded.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "nor_flash.h"
/* For BLOB_REGION_END only: the `nor test` default address must stay clear of the
 * asset region, and the assert below is what keeps the two from drifting apart.
 * Both files are compiled only under BSP_ENABLE_KV (CMakeLists.txt), so the include
 * is always resolvable wherever this file is built. */
#include "blob.h"

#include <stdint.h>
#include <string.h>

/* Scratch for readbacks: one page, on the caller's stack. */
#define NOR_CHUNK  NOR_PAGE_SIZE

/* `nor read` bounds. */
#define NOR_READ_DEFAULT  64u
#define NOR_READ_MAX      1024u

/* `nor write` bound: one page per invocation keeps the hex argument sane. */
#define NOR_WRITE_MAX     NOR_PAGE_SIZE

/*
 * Default scratch sector for `nor test`.  At 4 MB -- the first sector ABOVE the
 * blob region (issue #10 put asset slots at 1 MB .. 3 MB; issue #55 widened them to
 * six 512 KB slots ending at 4 MB), in the 12 MB that is still unallocated -- so
 * running the acceptance test destroys neither a live configuration nor a stored model.
 *
 * 🔴 THIS ADDRESS IS TIED TO THE BLOB REGION'S END, so it moves whenever the region
 * does.  It was 0x00300000 until issue #55, which is exactly where the region now
 * ends -- had it stayed, a bare `nor test` would have programmed the header sector
 * of the new slot 4.  (Boards that ran `nor test` under the old default still have
 * that residue there: it decodes as `invalid` rather than as a blob, and one
 * `blob erase 4` clears it.)
 *
 * It used to be 0x00100000, chosen when everything past the KV partition was spare;
 * the blob region moved in on top of it, which would have made a bare `nor test`
 * erase slot 0's HEADER and silently retire the blob stored there.  The lesson is
 * in the default, not in a check: `nor` is the raw-device debugging tool, so an
 * EXPLICIT address inside the blob region is still honoured (as `nor erase` and
 * `nor write` would be) -- it is the argument-less form, the one that gets typed
 * without thinking, that has to land somewhere harmless.
 */
#define NOR_TEST_DEFAULT_HEX   0x400000
#define NOR_TEST_DEFAULT_ADDR  ((uint32_t)NOR_TEST_DEFAULT_HEX)

/*
 * The help text at the bottom of this file QUOTES this address, and a help string
 * naming a different address than the code uses is worse than one naming none -- it
 * is confidently wrong, and it is read precisely by the person who has not looked at
 * the code.  So the string is built from the same token, and the assert keeps the
 * token itself above the region it is supposed to avoid.  (Two macros because
 * stringifying the uint32_t-cast form would print the cast, not the address.)
 */
#define NOR_STR_(x)            #x
#define NOR_STR(x)             NOR_STR_(x)
#define NOR_TEST_DEFAULT_STR   NOR_STR(NOR_TEST_DEFAULT_HEX)

_Static_assert(NOR_TEST_DEFAULT_ADDR >= BLOB_REGION_END,
               "the `nor test` default must stay above the blob region -- issue #55 "
               "moved the region's end and this address has to follow it");

static const char *nor_strerror(int rc)
{
	switch (rc) {
	case NOR_ERR_PARAM:   return "bad argument";
	case NOR_ERR_IO:      return "OCTOSPI2 transaction did not complete";
	case NOR_ERR_TIMEOUT: return "device stayed busy";
	case NOR_ERR_STATE:   return "driver not initialized / device down";
	case NOR_ERR_BUSY:    return "device busy (another command holds it)";
	default:              return "unknown error";
	}
}

/* Refuse every hardware subcommand while the device is down, with one message. */
static int nor_gate(struct cli_instance *sh)
{
	if (!nor_flash_ready()) {
		cli_error(sh, "nor: device down (see `nor info`)\r\n");
		return -1;
	}
	return 0;
}

/* Parse a device offset and reject anything outside the flash. */
static int nor_parse_addr(struct cli_instance *sh, const char *s, uint32_t *out)
{
	if (cli_parse_u32(s, out) != 0 || *out >= NOR_SIZE_BYTES) {
		cli_error(sh, "nor: bad address (0 .. 0x%lX)\r\n",
		          (unsigned long)(NOR_SIZE_BYTES - 1u));
		return -1;
	}
	return 0;
}

static int hex_nibble(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

/* "DEADBEEF" -> bytes.  Returns the byte count, or -1 on a malformed string. */
static int hex_to_bytes(const char *s, uint8_t *out, uint32_t max)
{
	uint32_t n = 0u;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		s += 2;
	while (*s != '\0') {
		int hi = hex_nibble(s[0]);
		int lo = (hi >= 0) ? hex_nibble(s[1]) : -1;

		if (hi < 0 || lo < 0 || n >= max)
			return -1;
		out[n++] = (uint8_t)((hi << 4) | lo);
		s += 2;
	}
	return (n == 0u) ? -1 : (int)n;
}

/* ---- subcommands --------------------------------------------------------- */

static int cmd_nor_info(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t id[3];
	uint32_t regs[4];

	(void)argc; (void)argv;
	nor_flash_get_id(id);
	nor_flash_get_regs(regs);

	cli_print(sh, "NOR (OCTOSPI2 / W25Q128JV):\r\n");
	cli_print(sh, "  state   : %s\r\n",
	          nor_flash_ready() ? "ready (indirect)" : "NOT ready");
	cli_print(sh, "  jedec   : %02X %02X %02X\r\n", id[0], id[1], id[2]);
	cli_print(sh, "  size    : %lu MB (offsets 0 .. 0x%lX)\r\n",
	          (unsigned long)(NOR_SIZE_BYTES >> 20),
	          (unsigned long)(NOR_SIZE_BYTES - 1u));
	cli_print(sh, "  erase   : %lu B sector / %lu B block\r\n",
	          (unsigned long)NOR_SECTOR_SIZE, (unsigned long)NOR_BLOCK_SIZE);
	cli_print(sh, "  page    : %lu B\r\n", (unsigned long)NOR_PAGE_SIZE);
	cli_print(sh, "  clock   : %lu Hz (1-line SPI, Fast Read 0x0B)\r\n",
	          (unsigned long)nor_flash_clock_hz());
	cli_print(sh, "  mapping : none -- indirect only, window fenced off in the MPU\r\n");
	/* Register readback: a wrong controller configuration and a dead device look
	 * the same from the id alone (the first bring-up here read 00 00 00 with a
	 * clean CR.FSEL=0 -- transaction completed, wrong pins). */
	cli_print(sh, "  regs    : CR %08lX DCR1 %08lX DCR2 %08lX SR %08lX\r\n",
	          (unsigned long)regs[0], (unsigned long)regs[1],
	          (unsigned long)regs[2], (unsigned long)regs[3]);
	return 0;
}

static int cmd_nor_read(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t buf[NOR_CHUNK];
	uint32_t addr, len = NOR_READ_DEFAULT, done = 0u;

	if (nor_gate(sh) || nor_parse_addr(sh, argv[1], &addr))
		return 1;
	if (argc > 2 && (cli_parse_u32(argv[2], &len) != 0 || len == 0u ||
	                 len > NOR_READ_MAX)) {
		cli_error(sh, "nor: bad length (1 .. %lu)\r\n",
		          (unsigned long)NOR_READ_MAX);
		return 1;
	}
	if (len > NOR_SIZE_BYTES - addr)
		len = NOR_SIZE_BYTES - addr;

	while (done < len) {
		uint32_t chunk = len - done;
		int rc;

		if (chunk > sizeof buf)
			chunk = sizeof buf;
		rc = nor_read(addr + done, buf, chunk);
		if (rc != NOR_OK) {
			cli_error(sh, "nor: read failed at 0x%06lX: %s\r\n",
			          (unsigned long)(addr + done), nor_strerror(rc));
			return 1;
		}
		cli_hexdump_base(sh, buf, chunk, addr + done);
		done += chunk;
	}
	return 0;
}

static int cmd_nor_erase(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr, base, len = NOR_SECTOR_SIZE;
	int rc;

	if (nor_gate(sh) || nor_parse_addr(sh, argv[1], &addr))
		return 1;
	if (argc > 2 && (cli_parse_u32(argv[2], &len) != 0 || len == 0u)) {
		cli_error(sh, "nor: bad length\r\n");
		return 1;
	}
	/*
	 * Round the request out to whole sectors rather than refusing it: erase is a
	 * sector-granular operation and silently erasing less than asked would be
	 * worse than erasing the sector the caller pointed into.
	 *
	 * ORDER MATTERS.  The range check comes BEFORE any rounding, against the
	 * address the user actually gave: rounding first would let a huge length near
	 * the top of the device wrap the 32-bit sum, and the check would then pass a
	 * request it should have rejected -- turning an out-of-range argument into a
	 * short erase somewhere else.  After the check, addr + len <= NOR_SIZE_BYTES
	 * holds, and because base and NOR_SIZE_BYTES are both sector multiples the
	 * rounded span still fits inside the device with nothing to overflow.
	 */
	if (len > NOR_SIZE_BYTES - addr) {
		cli_error(sh, "nor: range runs past the end of the device\r\n");
		return 1;
	}
	base = addr - (addr % NOR_SECTOR_SIZE);
	len += addr - base;
	len = (len + NOR_SECTOR_SIZE - 1u) & ~(NOR_SECTOR_SIZE - 1u);
	addr = base;

	rc = nor_erase(addr, len);
	if (rc != NOR_OK) {
		cli_error(sh, "nor: erase failed: %s\r\n", nor_strerror(rc));
		return 1;
	}
	cli_print(sh, "erased 0x%06lX .. 0x%06lX (%lu KB)\r\n",
	          (unsigned long)addr, (unsigned long)(addr + len - 1u),
	          (unsigned long)(len >> 10));
	return 0;
}

static int cmd_nor_write(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t data[NOR_WRITE_MAX];
	uint32_t addr;
	int n, rc;

	(void)argc;
	if (nor_gate(sh) || nor_parse_addr(sh, argv[1], &addr))
		return 1;
	n = hex_to_bytes(argv[2], data, sizeof data);
	if (n < 0) {
		cli_error(sh, "nor: bad hex data (even number of hex digits, "
		          "max %lu bytes)\r\n", (unsigned long)sizeof data);
		return 1;
	}
	if ((uint32_t)n > NOR_SIZE_BYTES - addr) {
		cli_error(sh, "nor: range runs past the end of the device\r\n");
		return 1;
	}

	rc = nor_write(addr, data, (uint32_t)n);
	if (rc != NOR_OK) {
		cli_error(sh, "nor: write failed: %s\r\n", nor_strerror(rc));
		return 1;
	}
	cli_print(sh, "wrote %d B at 0x%06lX (NOR: bits only clear -- erase first)\r\n",
	          n, (unsigned long)addr);
	return 0;
}

/* ---- nor test ------------------------------------------------------------ */

/* Verify that [addr, addr+len) reads all 0xFF, chunk by chunk. */
static int nor_check_erased(struct cli_instance *sh, uint32_t addr, uint32_t len)
{
	uint8_t buf[NOR_CHUNK];
	uint32_t done = 0u;

	while (done < len) {
		uint32_t chunk = len - done;
		uint32_t i;
		int rc;

		if (chunk > sizeof buf)
			chunk = sizeof buf;
		rc = nor_read(addr + done, buf, chunk);
		if (rc != NOR_OK) {
			cli_error(sh, "  read failed at 0x%06lX: %s\r\n",
			          (unsigned long)(addr + done), nor_strerror(rc));
			return -1;
		}
		for (i = 0u; i < chunk; i++) {
			if (buf[i] != 0xFFu) {
				cli_error(sh, "  0x%06lX = 0x%02X, expected 0xFF\r\n",
				          (unsigned long)(addr + done + i), buf[i]);
				return -1;
			}
		}
		done += chunk;
	}
	return 0;
}

/*
 * The acceptance test for FDB_WRITE_GRAN = 8.  Three partial page programs land
 * in disjoint byte ranges of one page; after each, the whole page is read back
 * and checked so that (a) the new bytes took, (b) the previously written bytes
 * were not disturbed, and (c) everything else still reads 0xFF.  Point (c) is the
 * one that matters: it is what lets a later status byte be programmed into a page
 * that has already been programmed elsewhere.
 */
static int nor_test_partial(struct cli_instance *sh, uint32_t page)
{
	static const uint32_t off[3] = {0u, 16u, 32u};
	static const uint8_t  pat[3][4] = {
		{0xA5u, 0x5Au, 0x00u, 0xFFu},
		{0xDEu, 0xADu, 0xBEu, 0xEFu},
		{0x01u, 0x02u, 0x04u, 0x08u},
	};
	uint8_t back[NOR_CHUNK];
	uint32_t step;

	for (step = 0u; step < 3u; step++) {
		uint32_t i, w;
		int rc = nor_write(page + off[step], pat[step], sizeof pat[step]);

		if (rc != NOR_OK) {
			cli_error(sh, "  program %lu failed: %s\r\n",
			          (unsigned long)step, nor_strerror(rc));
			return -1;
		}
		rc = nor_read(page, back, sizeof back);
		if (rc != NOR_OK) {
			cli_error(sh, "  readback %lu failed: %s\r\n",
			          (unsigned long)step, nor_strerror(rc));
			return -1;
		}
		for (i = 0u; i < sizeof back; i++) {
			uint8_t want = 0xFFu;

			for (w = 0u; w <= step; w++) {
				if (i >= off[w] && i < off[w] + sizeof pat[w])
					want = pat[w][i - off[w]];
			}
			if (back[i] != want) {
				cli_error(sh, "  after program %lu: +%lu = 0x%02X, "
				          "expected 0x%02X\r\n", (unsigned long)step,
				          (unsigned long)i, back[i], want);
				return -1;
			}
		}
		cli_print(sh, "  program %lu (+%lu, %u B): page intact\r\n",
		          (unsigned long)step, (unsigned long)off[step],
		          (unsigned)sizeof pat[step]);
	}
	return 0;
}

/*
 * Observation only: re-program a byte that has already been programmed.  This is
 * what FDB_WRITE_GRAN = 1 would need and what the datasheet does not promise, so
 * the result is reported and never turned into a pass/fail -- the shipped
 * configuration does not depend on it.  Recording the answer here is what makes a
 * future decision about gran = 1 a measurement rather than a guess.
 */
static void nor_test_overwrite(struct cli_instance *sh, uint32_t addr)
{
	static const uint8_t seq[2] = {0xFEu, 0xFCu};
	uint8_t got[2] = {0u, 0u};
	uint32_t i;

	for (i = 0u; i < 2u; i++) {
		if (nor_write(addr, &seq[i], 1u) != NOR_OK ||
		    nor_read(addr, &got[i], 1u) != NOR_OK) {
			cli_print(sh, "  (overwrite probe aborted on an I/O error)\r\n");
			return;
		}
	}
	cli_print(sh, "  0xFF -> write 0xFE -> read 0x%02X -> write 0xFC -> read 0x%02X%s\r\n",
	          got[0], got[1],
	          (got[0] == 0xFEu && got[1] == 0xFCu) ? "  (accepted)"
	                                               : "  (NOT accepted)");
}

static int cmd_nor_test(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr = NOR_TEST_DEFAULT_ADDR;
	uint8_t id[3];
	int rc;

	if (nor_gate(sh))
		return 1;
	if (argc > 1 && nor_parse_addr(sh, argv[1], &addr))
		return 1;
	addr -= addr % NOR_SECTOR_SIZE;
	if (NOR_SIZE_BYTES - addr < NOR_SECTOR_SIZE) {
		cli_error(sh, "nor: no room for a sector at 0x%06lX\r\n",
		          (unsigned long)addr);
		return 1;
	}

	nor_flash_get_id(id);
	cli_print(sh, "nor test: sector 0x%06lX (4 KB will be ERASED)\r\n",
	          (unsigned long)addr);
	cli_print(sh, "jedec %02X %02X %02X @ %lu Hz\r\n", id[0], id[1], id[2],
	          (unsigned long)nor_flash_clock_hz());

	rc = nor_erase(addr, NOR_SECTOR_SIZE);
	if (rc != NOR_OK) {
		cli_error(sh, "nor: erase failed: %s\r\n", nor_strerror(rc));
		return 1;
	}
	cli_print(sh, "erase: OK\r\n");
	if (nor_check_erased(sh, addr, NOR_SECTOR_SIZE)) {
		cli_error(sh, "nor: FAIL -- sector is not blank after erase\r\n");
		return 1;
	}
	cli_print(sh, "blank check: OK (4096 B all 0xFF)\r\n");

	cli_print(sh, "partial page programs (what FDB_WRITE_GRAN=8 needs):\r\n");
	if (nor_test_partial(sh, addr)) {
		cli_error(sh, "nor: FAIL -- partial page programming disturbs the page\r\n");
		return 1;
	}

	cli_print(sh, "same-byte overwrite (observation only, gran=1 would need it):\r\n");
	nor_test_overwrite(sh, addr + 64u);

	cli_print(sh, "nor: PASS\r\n");
	return 0;
}

CLI_SUBCMD_SET_CREATE(nor_subcmds,
	CLI_CMD_ARG(info,  NULL, "bring-up state, JEDEC id, geometry, clock", cmd_nor_info,  1, 0),
	CLI_CMD_ARG(read,  NULL, "hexdump <addr> [len]",                      cmd_nor_read,  2, 1),
	CLI_CMD_ARG(erase, NULL, "erase 4 KB sector(s) <addr> [len]",         cmd_nor_erase, 2, 1),
	CLI_CMD_ARG(write, NULL, "program <addr> <hex> (erase first)",        cmd_nor_write, 3, 0),
	CLI_CMD_ARG(test,  NULL, "partial-page-program acceptance test [addr] (defaults to "
	                         NOR_TEST_DEFAULT_STR ", above kv+blob)", cmd_nor_test, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(nor, nor_subcmds,
                 "external NOR flash (OCTOSPI2 / W25Q128)", NULL, 1, 0);
