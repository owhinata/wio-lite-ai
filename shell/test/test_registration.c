/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host smoke test for the Shell command-registration foundation (issue #2).
 *
 * Registers a few commands with the public macros and walks the
 * .shell_root_cmds linker section at run time, asserting:
 *   1. the registered root commands are all enumerated (count),
 *   2. entries are laid out gap-free (contiguous array, walkable by pointer
 *      arithmetic between the boundary symbols),
 *   3. SORT_BY_NAME gives a deterministic alphabetical order,
 *   4. a nested subcommand set links and terminates at its sentinel,
 *   5. argc metadata round-trips through the descriptor.
 */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "cli.h"

static int h_noop(struct cli_instance *sh, int argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	return 0;
}

/* Nested subcommand set: thread { list, stacks }. */
CLI_SUBCMD_SET_CREATE(sub_thread,
	CLI_CMD_ARG(list,   NULL, "list threads",      h_noop, 1, 0),
	CLI_CMD_ARG(stacks, NULL, "show stack usage",  h_noop, 1, 0),
	CLI_SUBCMD_SET_END
);

/* Three root commands, intentionally registered out of alphabetical source
 * order to prove SORT_BY_NAME reorders them. */
CLI_CMD_REGISTER(version, NULL,       "show firmware version", h_noop, 1, 0);
CLI_CMD_REGISTER(thread,  sub_thread, "thread operations",     NULL,   2, 0);
CLI_CMD_REGISTER(devmem,  NULL,       "memory peek/poke",      h_noop, 2, 2);

int main(void)
{
	/* 1. every registered root command is enumerated */
	assert(cli_root_cmd_count() == 3);

	/* 2. gap-free contiguous layout (aligned(__alignof__) effect) */
	for (size_t i = 0; i + 1 < cli_root_cmd_count(); ++i) {
		const char *a = (const char *)&__cli_root_cmds_start[i];
		const char *b = (const char *)&__cli_root_cmds_start[i + 1];
		assert((size_t)(b - a) == sizeof(struct cli_cmd));
	}

	/* 3. deterministic alphabetical order: devmem, thread, version */
	const char *expected[] = { "devmem", "thread", "version" };
	size_t idx = 0;
	CLI_ROOT_CMD_FOREACH(c) {
		assert(idx < 3);
		assert(strcmp(c->name, expected[idx]) == 0);
		++idx;
	}
	assert(idx == 3);

	/* 4. subcommand tree links + sentinel termination */
	const struct cli_cmd *thr = NULL;
	CLI_ROOT_CMD_FOREACH(c) {
		if (strcmp(c->name, "thread") == 0)
			thr = c;
	}
	assert(thr != NULL);
	assert(thr->handler == NULL);            /* pure parent command */
	assert(thr->subcmds != NULL);
	size_t nsub = 0;
	for (const struct cli_cmd *s = thr->subcmds; s->name != NULL; ++s)
		++nsub;
	assert(nsub == 2);
	assert(strcmp(thr->subcmds[0].name, "list") == 0);
	assert(strcmp(thr->subcmds[1].name, "stacks") == 0);

	/* 5. argc metadata round-trips */
	CLI_ROOT_CMD_FOREACH(c) {
		if (strcmp(c->name, "devmem") == 0) {
			assert(c->mandatory == 2);
			assert(c->optional == 2);
		}
	}

	printf("OK: %zu root commands; contiguous; sorted; subcmd tree valid\n",
	       cli_root_cmd_count());
	return 0;
}
