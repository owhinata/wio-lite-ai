/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_kv.c
 * @brief   `kv` shell command: the persistent configuration store (issue #37).
 *
 *   kv list [prefix]           key, type, value and description
 *   kv get <key>
 *   kv set <key> <value> [type]   type is str (default) / u32 / bool / bytes
 *   kv desc <key> <text...>    document what a setting means (may be several words)
 *   kv del <key>
 *   kv info                    state, partition geometry and live usage
 *   kv format yes              erase the partition and rebuild -- destructive
 *
 * Types are explicit rather than guessed from the text.  Inferring them would be
 * both surprising and lossy -- an SSID of "12345" is a string, not a number, and
 * nothing about the characters says which was meant.  `str` is the default because
 * it is what most settings are.
 *
 * The description is stored WITH the value on the external flash, so `kv list` on a
 * board explains itself without the reader having to find the source.  `kv set`
 * keeps whatever description is already there, so documenting a setting once is
 * enough.
 *
 * `kv format` is spelled `kv format yes`, matching `sd format yes`, because both
 * destroy everything on the medium and neither should be one keystroke away.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "kv.h"

#include <stdint.h>
#include <string.h>

/*
 * Keys whose value is a secret get masked in listings.  This hides the PSK from a
 * shoulder-surfer and from a pasted log; it is NOT storage protection, and must
 * not be mistaken for it -- the value is written to the external flash in the
 * clear, where SWD or a hot-air gun reads it back.  There is nowhere on this board
 * to keep a key that would make encryption meaningful, since the option bytes and
 * RDP are deliberately never touched (see CLAUDE.md).  `kv get` still prints the
 * value, because a user who names the key explicitly is asking for it.
 */
static int kv_is_secret(const char *key)
{
	return strcmp(key, "wifi.psk") == 0;
}

static const char *kv_strerror(int rc)
{
	switch (rc) {
	case KV_ERR_PARAM:    return "bad key or value";
	case KV_ERR_STATE:    return "store not ready";
	case KV_ERR_BUSY:     return "store busy";
	case KV_ERR_IO:       return "flash I/O failed";
	case KV_ERR_NOTFOUND: return "no such key";
	case KV_ERR_FULL:     return "partition full";
	case KV_ERR_TRUNC:    return "value longer than the buffer";
	case KV_ERR_FORMAT:   return "stored record is unreadable";
	case KV_ERR_TYPE:     return "value is not of the expected type";
	default:              return "unknown error";
	}
}

/* Refuse the data-path subcommands with one message that names the actual state,
 * so "not ready yet" and "corrupt, format it" never look the same. */
static int kv_gate(struct cli_instance *sh)
{
	if (kv_state() != KV_STATE_READY) {
		cli_error(sh, "kv: %s\r\n", kv_state_str());
		return -1;
	}
	return 0;
}

/* ---- value parsing ------------------------------------------------------- */

static int hex_nibble(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int kv_parse_type(const char *s, uint8_t *out)
{
	if (strcmp(s, "str") == 0)        *out = KV_TYPE_STR;
	else if (strcmp(s, "u32") == 0)   *out = KV_TYPE_U32;
	else if (strcmp(s, "bool") == 0)  *out = KV_TYPE_BOOL;
	else if (strcmp(s, "bytes") == 0) *out = KV_TYPE_BYTES;
	else return -1;
	return 0;
}

/*
 * Turn the typed text into the bytes that get stored.  Returns the length, or -1
 * with a message already printed -- the diagnostics differ per type, and a single
 * "bad value" would leave the user guessing which part was wrong.
 */
static int kv_parse_value(struct cli_instance *sh, uint8_t type, const char *s,
                          uint8_t *out, uint32_t outsize)
{
	size_t len = strlen(s);
	uint32_t n;
	uint32_t i;

	switch (type) {
	case KV_TYPE_STR:
		if (len == 0u || len > outsize) {
			cli_error(sh, "kv: string must be 1..%lu bytes\r\n",
			          (unsigned long)outsize);
			return -1;
		}
		memcpy(out, s, len);
		return (int)len;

	case KV_TYPE_U32:
		if (cli_parse_u32(s, &n) != 0) {
			cli_error(sh, "kv: not a number (decimal or 0x hex)\r\n");
			return -1;
		}
		out[0] = (uint8_t)(n & 0xFFu);
		out[1] = (uint8_t)((n >> 8) & 0xFFu);
		out[2] = (uint8_t)((n >> 16) & 0xFFu);
		out[3] = (uint8_t)((n >> 24) & 0xFFu);
		return 4;

	case KV_TYPE_BOOL:
		if (strcmp(s, "true") == 0 || strcmp(s, "1") == 0 ||
		    strcmp(s, "on") == 0 || strcmp(s, "yes") == 0)
			out[0] = 1u;
		else if (strcmp(s, "false") == 0 || strcmp(s, "0") == 0 ||
		         strcmp(s, "off") == 0 || strcmp(s, "no") == 0)
			out[0] = 0u;
		else {
			cli_error(sh, "kv: expected true/false (or 1/0, on/off, yes/no)\r\n");
			return -1;
		}
		return 1;

	default: /* KV_TYPE_BYTES */
		if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
			s += 2;
			len = strlen(s);
		}
		if (len == 0u || (len & 1u) != 0u || len / 2u > outsize) {
			cli_error(sh, "kv: expected an even number of hex digits "
			          "(max %lu bytes)\r\n", (unsigned long)outsize);
			return -1;
		}
		for (i = 0u; i < len / 2u; i++) {
			int hi = hex_nibble(s[i * 2u]);
			int lo = hex_nibble(s[i * 2u + 1u]);

			if (hi < 0 || lo < 0) {
				cli_error(sh, "kv: bad hex digit\r\n");
				return -1;
			}
			out[i] = (uint8_t)((hi << 4) | lo);
		}
		return (int)(len / 2u);
	}
}

/* ---- value rendering ----------------------------------------------------- */

/* Print a value the way its type says it should read.  Non-printable bytes in a
 * string become escapes so a stored control character cannot scramble the terminal
 * or forge a line of output. */
static void kv_print_value(struct cli_instance *sh, const struct kv_value *v)
{
	uint32_t i;

	switch (v->type) {
	case KV_TYPE_U32: {
		uint32_t n = (uint32_t)v->data[0] | ((uint32_t)v->data[1] << 8) |
		             ((uint32_t)v->data[2] << 16) | ((uint32_t)v->data[3] << 24);

		cli_print(sh, "%lu (0x%lX)", (unsigned long)n, (unsigned long)n);
		break;
	}
	case KV_TYPE_BOOL:
		cli_print(sh, "%s", v->data[0] ? "true" : "false");
		break;
	case KV_TYPE_BYTES:
		for (i = 0u; i < v->len; i++)
			cli_print(sh, "%02X", v->data[i]);
		break;
	case KV_TYPE_STR:
		for (i = 0u; i < v->len; i++) {
			if (v->data[i] >= 0x20u && v->data[i] < 0x7Fu)
				cli_print(sh, "%c", (char)v->data[i]);
			else
				cli_print(sh, "\\x%02X", v->data[i]);
		}
		break;
	default:
		/* Shown, not skipped: a listing that hid what it could not decode
		 * would make a half-corrupt store look healthy. */
		cli_print(sh, "<unreadable record>");
		break;
	}
}

/* ---- subcommands --------------------------------------------------------- */

struct kv_list_ctx {
	struct cli_instance *sh;
	uint32_t shown;
};

static int kv_list_one(const char *key, const struct kv_value *v, void *arg)
{
	struct kv_list_ctx *ctx = (struct kv_list_ctx *)arg;

	ctx->shown++;
	cli_print(ctx->sh, "  %-22s %-5s ", key, kv_type_name(v->type));
	if (kv_is_secret(key) && v->type != KV_TYPE_INVALID)
		cli_print(ctx->sh, "(hidden, %lu B)", (unsigned long)v->len);
	else
		kv_print_value(ctx->sh, v);
	if (v->desc[0] != '\0')
		cli_print(ctx->sh, "   -- %s", v->desc);
	cli_print(ctx->sh, "\r\n");
	return 0;
}

static int cmd_kv_list(struct cli_instance *sh, int argc, char **argv)
{
	struct kv_list_ctx ctx = { sh, 0u };
	const char *prefix = (argc > 1) ? argv[1] : NULL;
	int rc;

	if (kv_gate(sh))
		return 1;
	rc = kv_foreach(prefix, kv_list_one, &ctx);
	if (rc != KV_OK) {
		cli_error(sh, "kv: list failed: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "%lu key%s\r\n", (unsigned long)ctx.shown,
	          (ctx.shown == 1u) ? "" : "s");
	return 0;
}

static int cmd_kv_get(struct cli_instance *sh, int argc, char **argv)
{
	struct kv_value v;
	int rc;

	(void)argc;
	if (kv_gate(sh))
		return 1;
	rc = kv_get(argv[1], &v);
	if (rc != KV_OK) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "%s (%s) = ", argv[1], kv_type_name(v.type));
	kv_print_value(sh, &v);
	cli_print(sh, "\r\n");
	if (v.desc[0] != '\0')
		cli_print(sh, "  %s\r\n", v.desc);
	return 0;
}

static int cmd_kv_set(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t buf[KV_VALUE_MAX];
	uint8_t type = KV_TYPE_STR;
	int len, rc;

	if (kv_gate(sh))
		return 1;
	if (argc > 3 && kv_parse_type(argv[3], &type) != 0) {
		cli_error(sh, "kv: unknown type '%s' (str/u32/bool/bytes)\r\n", argv[3]);
		return 1;
	}
	len = kv_parse_value(sh, type, argv[2], buf, sizeof buf);
	if (len < 0)
		return 1;   /* kv_parse_value already explained which part was wrong */

	/* NULL description: keep whatever is already documented for this key. */
	rc = kv_set(argv[1], type, buf, (uint32_t)len, NULL);
	if (rc != KV_OK) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "%s (%s) = %d B\r\n", argv[1], kv_type_name(type), len);
	return 0;
}

/*
 * A description is a sentence, so accept it as several tokens and join them with
 * single spaces rather than demanding quotes: `kv desc net.mode dhcp / static /
 * manual` should work as typed.  (CLI_ARG_RAW would be the direct way to get the
 * untokenized tail, but it only captures one on a LEAF command -- see cli.h -- and
 * `desc` is a subcommand of `kv`, where it merely relaxes the argument count.  So
 * the tokens are rejoined here instead; runs of whitespace collapse to one space,
 * which costs a description nothing.)  A quoted argument still arrives as a single
 * token and passes through unchanged.
 */
static int cmd_kv_desc(struct cli_instance *sh, int argc, char **argv)
{
	char text[KV_DESC_MAX + 1u];
	uint32_t used = 0u;
	int i, rc;

	if (kv_gate(sh))
		return 1;

	for (i = 2; i < argc; i++) {
		uint32_t need = (uint32_t)strlen(argv[i]) + ((i > 2) ? 1u : 0u);

		if (used + need > KV_DESC_MAX) {
			cli_error(sh, "kv: description must be at most %lu characters\r\n",
			          (unsigned long)KV_DESC_MAX);
			return 1;
		}
		if (i > 2)
			text[used++] = ' ';
		memcpy(text + used, argv[i], strlen(argv[i]));
		used += (uint32_t)strlen(argv[i]);
	}
	text[used] = '\0';

	rc = kv_set_desc(argv[1], text);
	if (rc != KV_OK) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	/* Echo the JOINED text, not argv[2] -- the confirmation line has to show what
	 * was actually stored, and with several tokens argv[2] is only the first. */
	cli_print(sh, "%s: %s\r\n", argv[1], text);
	return 0;
}

static int cmd_kv_del(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	if (kv_gate(sh))
		return 1;
	rc = kv_del(argv[1]);
	if (rc != KV_OK) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "deleted %s\r\n", argv[1]);
	return 0;
}

static int cmd_kv_info(struct cli_instance *sh, int argc, char **argv)
{
	struct kv_info info;
	int rc;

	(void)argc; (void)argv;
	rc = kv_get_info(&info);

	cli_print(sh, "KV store (FlashDB on the external NOR):\r\n");
	cli_print(sh, "  state     : %s\r\n", kv_state_str());
	cli_print(sh, "  partition : offset 0x%06lX, %lu KB, %lu B sectors\r\n",
	          (unsigned long)info.part_offset,
	          (unsigned long)(info.part_size >> 10),
	          (unsigned long)info.sec_size);
	if (rc != KV_OK) {
		/* Geometry is known from initialisation even when the store cannot be
		 * walked, so print what is true and stop rather than showing zeros
		 * that would read as "the store is empty". */
		cli_print(sh, "  usage     : unavailable (%s)\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "  keys      : %lu\r\n", (unsigned long)info.kv_count);
	cli_print(sh, "  values    : %lu B\r\n", (unsigned long)info.value_bytes);
	cli_print(sh, "  on flash  : %lu B of %lu B (%lu%%)\r\n",
	          (unsigned long)info.obj_bytes, (unsigned long)info.part_size,
	          (unsigned long)(info.part_size ?
	                          (info.obj_bytes * 100u) / info.part_size : 0u));
	return 0;
}

static int cmd_kv_format(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	if (argc < 2 || strcmp(argv[1], "yes") != 0) {
		cli_error(sh, "kv: this ERASES every stored setting -- "
		          "confirm with `kv format yes`\r\n");
		return 1;
	}
	/* Say how long this takes before starting it.  A command that returns nothing
	 * for a quarter of a minute is indistinguishable from a hung console, and the
	 * time is inherent: FlashDB erases all 256 sectors individually while laying
	 * down its headers, at up to 400 ms each. */
	cli_print(sh, "erasing 1 MB and rebuilding -- about 15 s...\r\n");

	/* No kv_gate() here on purpose: formatting is the documented way out of a
	 * corrupt or failed store, so it has to work exactly when nothing else does. */
	rc = kv_format();
	if (rc != KV_OK) {
		cli_error(sh, "kv: format failed: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "formatted -- the store is empty\r\n");
	return 0;
}

CLI_SUBCMD_SET_CREATE(kv_subcmds,
	CLI_CMD_ARG(list,   NULL, "list keys and values [prefix]",        cmd_kv_list,   1, 1),
	CLI_CMD_ARG(get,    NULL, "print one entry <key>",                cmd_kv_get,    2, 0),
	CLI_CMD_ARG(set,    NULL, "store <key> <value> [str|u32|bool|bytes]", cmd_kv_set, 3, 1),
	CLI_CMD_ARG(desc,   NULL, "describe <key> <text...>",             cmd_kv_desc,   3, CLI_ARG_RAW),
	CLI_CMD_ARG(del,    NULL, "delete <key>",                         cmd_kv_del,    2, 0),
	CLI_CMD_ARG(info,   NULL, "state, partition geometry, usage",     cmd_kv_info,   1, 0),
	CLI_CMD_ARG(format, NULL, "erase every setting -- confirm 'yes'", cmd_kv_format, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(kv, kv_subcmds,
                 "persistent configuration store (external NOR)", NULL, 1, 0);
