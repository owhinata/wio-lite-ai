/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_kv.c
 * @brief   `kv` shell command: the persistent configuration store (issue #37).
 *
 *   kv list [prefix]     keys and values, optionally filtered by prefix
 *   kv get <key>
 *   kv set <key> <value>
 *   kv del <key>
 *   kv info              state, partition geometry and live usage
 *   kv format yes        erase the partition and rebuild -- destructive
 *
 * Values are handled as text here.  The store itself takes arbitrary bytes, and a
 * later increment wraps each value in a record carrying its type and a description;
 * this command deliberately stays as thin as it can until then, so what is on the
 * flash is exactly what was typed.
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

/* Scratch for one value, on the calling shell thread's stack. */
#define KV_CMD_VALUE_BUF  (KV_VALUE_MAX + 1u)

/*
 * Keys whose value is a secret get masked in listings.  This hides the PSK from a
 * shoulder-surfer and from a pasted log; it is NOT storage protection, and must
 * not be mistaken for it -- the value is written to the external flash in the
 * clear, where SWD or a hot-air gun reads it back. There is nowhere on this board
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

/* Print one value, rendering non-printable bytes as escapes so a binary value
 * cannot scramble the terminal or forge a line of output. */
static void kv_print_value(struct cli_instance *sh, const void *val, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)val;
	uint32_t i;

	for (i = 0u; i < len; i++) {
		if (p[i] >= 0x20u && p[i] < 0x7Fu)
			cli_print(sh, "%c", (char)p[i]);
		else
			cli_print(sh, "\\x%02X", p[i]);
	}
}

/* ---- subcommands --------------------------------------------------------- */

struct kv_list_ctx {
	struct cli_instance *sh;
	uint32_t shown;
};

static int kv_list_one(const char *key, const void *val, uint32_t len,
                       int truncated, void *arg)
{
	struct kv_list_ctx *ctx = (struct kv_list_ctx *)arg;

	ctx->shown++;
	cli_print(ctx->sh, "  %-24s ", key);
	if (kv_is_secret(key))
		cli_print(ctx->sh, "(hidden, %lu B)", (unsigned long)len);
	else
		kv_print_value(ctx->sh, val, len);
	if (truncated)
		cli_print(ctx->sh, " ...(truncated)");
	cli_print(ctx->sh, "\r\n");
	return 0;
}

static int cmd_kv_list(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t buf[KV_CMD_VALUE_BUF];
	struct kv_list_ctx ctx = { sh, 0u };
	const char *prefix = (argc > 1) ? argv[1] : NULL;
	int rc;

	if (kv_gate(sh))
		return 1;
	rc = kv_foreach(prefix, buf, sizeof buf, kv_list_one, &ctx);
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
	uint8_t buf[KV_CMD_VALUE_BUF];
	uint32_t len = 0u;
	int rc;

	(void)argc;
	if (kv_gate(sh))
		return 1;
	rc = kv_get(argv[1], buf, sizeof buf, &len);
	if (rc != KV_OK && rc != KV_ERR_TRUNC) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	kv_print_value(sh, buf, (len < sizeof buf) ? len : sizeof buf);
	if (rc == KV_ERR_TRUNC)
		cli_print(sh, " ...(%lu B stored)", (unsigned long)len);
	cli_print(sh, "\r\n");
	return 0;
}

static int cmd_kv_set(struct cli_instance *sh, int argc, char **argv)
{
	size_t len;
	int rc;

	(void)argc;
	if (kv_gate(sh))
		return 1;
	len = strlen(argv[2]);
	if (len == 0u || len > KV_VALUE_MAX) {
		cli_error(sh, "kv: value must be 1..%lu bytes\r\n",
		          (unsigned long)KV_VALUE_MAX);
		return 1;
	}
	rc = kv_set(argv[1], argv[2], (uint32_t)len);
	if (rc != KV_OK) {
		cli_error(sh, "kv: %s\r\n", kv_strerror(rc));
		return 1;
	}
	cli_print(sh, "%s = %lu B\r\n", argv[1], (unsigned long)len);
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
	CLI_CMD_ARG(get,    NULL, "print one value <key>",                cmd_kv_get,    2, 0),
	CLI_CMD_ARG(set,    NULL, "store <key> <value>",                  cmd_kv_set,    3, 0),
	CLI_CMD_ARG(del,    NULL, "delete <key>",                         cmd_kv_del,    2, 0),
	CLI_CMD_ARG(info,   NULL, "state, partition geometry, usage",     cmd_kv_info,   1, 0),
	CLI_CMD_ARG(format, NULL, "erase every setting -- confirm 'yes'", cmd_kv_format, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(kv, kv_subcmds,
                 "persistent configuration store (external NOR)", NULL, 1, 0);
