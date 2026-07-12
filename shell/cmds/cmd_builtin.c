/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_builtin.c
 * @brief   Minimal built-in shell commands for the #8 bring-up demo: help + echo.
 *
 * These are the only commands the `shell` application registers into
 * .shell_root_cmds; the richer builtins (version/uptime/reboot, thread, devmem)
 * arrive in #12-#14.  Both handlers touch only the shell instance passed to them
 * and write through the buffered output API, so they are reentrant when several
 * instances run the same command concurrently (req §10).
 *
 * This file is linked into the `shell` executable only -- never the host test
 * harness (shell/test) -- so the unit tests keep full control of their own
 * registered command set.
 *
 * Clean-room design; no third-party code reused.
 */
#include <stdio.h>   /* snprintf (command-path join) */
#include <string.h>  /* strcmp (command-tree walk) */

#include "cli.h"

/* Find a root command by exact name -- a public-API walk of .shell_root_cmds, so
 * this app-layer file needs no core-internal match_root(). */
static const struct cli_cmd *help_root(const char *name)
{
	CLI_ROOT_CMD_FOREACH(it)
		if (it->name != NULL && strcmp(it->name, name) == 0)
			return it;
	return NULL;
}

/* Find a subcommand by exact name in a CLI_SUBCMD_SET_END-terminated set. */
static const struct cli_cmd *help_sub(const struct cli_cmd *set, const char *name)
{
	for (; set->name != NULL; ++set)
		if (strcmp(set->name, name) == 0)
			return set;
	return NULL;
}

/* Join argv[from..to) into buf with single-space separators (for the command
 * path); always NUL-terminated, truncates safely if it would overflow. */
static const char *help_path(char *buf, size_t cap, char **argv, int from, int to)
{
	size_t n = 0;

	if (cap > 0)
		buf[0] = '\0';
	for (int i = from; i < to && n < cap; i++)
		n += (size_t)snprintf(buf + n, cap - n, i > from ? " %s" : "%s", argv[i]);
	return buf;
}

/* One listing line "  name        help [>]"; '>' flags a command group (has
 * subcommands).  Shared by the root list and any subcommand list. */
static void help_line(struct cli_instance *sh, const struct cli_cmd *c)
{
	cli_print(sh, "  %-10s %s%s\r\n", c->name, c->help ? c->help : "",
	          c->subcmds ? " >" : "");
}

/*
 * help: hierarchical command help (issue #37).
 *
 *   help              -> list every root command (the .shell_root_cmds walk is
 *                        itself the proof the section + boundary symbols linked,
 *                        acceptance §18.1); '>' marks command groups.
 *   help <cmd> [sub]  -> walk the command tree along the argv path; a parent
 *                        prints its subcommand list, a leaf prints its one-line
 *                        help as usage.  Reuses the existing .help strings -- no
 *                        new descriptor field (issue #37 scope).
 *
 * Registered with optional = CLI_MAX_SUBCMD_DEPTH + 1 so the deepest legal path
 * (help + root + CLI_MAX_SUBCMD_DEPTH subcommands) tokenizes; NOT CLI_ARG_RAW,
 * which would collapse the tail into a single argument.
 */
static int cmd_help(struct cli_instance *sh, int argc, char **argv)
{
	char path[CLI_CMD_BUFFER_SIZE];

	/* `help`: list every root command. */
	if (argc < 2) {
		cli_print(sh, "Commands:\r\n");
		CLI_ROOT_CMD_FOREACH(it)
			help_line(sh, it);
		cli_print(sh, "'>' marks a command group; type "
		              "'help <command> [subcommand]' for details.\r\n");
		return 0;
	}

	/* `help <path...>`: resolve argv[1] as a root, then descend argv[2..]. */
	const struct cli_cmd *cmd = help_root(argv[1]);
	if (cmd == NULL) {
		cli_error(sh, "help: no such command '%s'\r\n", argv[1]);
		return -1;
	}
	for (int i = 2; i < argc; i++) {
		const struct cli_cmd *sub =
			cmd->subcmds ? help_sub(cmd->subcmds, argv[i]) : NULL;
		if (sub == NULL) {
			cli_error(sh, "help: no such command '%s'\r\n",
			          help_path(path, sizeof path, argv, 1, i + 1));
			return -1;
		}
		cmd = sub;
	}

	if (cmd->subcmds != NULL) {
		/* Parent: own help line + its subcommand list. */
		cli_print(sh, "%s -- %s\r\n",
		          help_path(path, sizeof path, argv, 1, argc),
		          cmd->help ? cmd->help : "");
		cli_print(sh, "Subcommands:\r\n");
		for (const struct cli_cmd *s = cmd->subcmds; s->name != NULL; ++s)
			help_line(sh, s);
		cli_print(sh, "Type 'help %s <subcommand>' for details.\r\n",
		          help_path(path, sizeof path, argv, 1, argc));
	} else {
		/* Leaf: full path + its one-line help as usage. */
		cli_print(sh, "%s  %s\r\n",
		          help_path(path, sizeof path, argv, 1, argc),
		          cmd->help ? cmd->help : "");
	}
	return 0;
}

/*
 * echo: print the rest of the line verbatim.  Registered CLI_ARG_RAW, so after
 * the command name the untokenized tail (quotes/escapes untouched, leading space
 * trimmed) arrives as a single argv[1]; with no tail, argc is 1.
 */
static int cmd_echo(struct cli_instance *sh, int argc, char **argv)
{
	return cli_print(sh, "%s\r\n", argc > 1 ? argv[1] : "");
}

CLI_CMD_REGISTER(help, NULL, "list commands", cmd_help, 1, CLI_MAX_SUBCMD_DEPTH + 1);
CLI_CMD_REGISTER(echo, NULL, "echo the rest of the line", cmd_echo, 1, CLI_ARG_RAW);
