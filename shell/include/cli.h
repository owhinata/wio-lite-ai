/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli.h
 * @brief   Shell (CLI) public API -- command registration foundation.
 *
 * Commands are registered declaratively from any translation unit with
 * CLI_CMD_REGISTER(); each root command lands as a `struct cli_cmd` in the
 * dedicated linker section `.shell_root_cmds`, between the boundary symbols
 * __cli_root_cmds_start / __cli_root_cmds_end.  Subcommands form a static tree
 * of `struct cli_cmd[]` arrays terminated by a CLI_SUBCMD_SET_END sentinel.
 * Root and subcommand entries share the same descriptor type and are read-only
 * data shared by every shell instance.
 *
 * This header is a clean-room design; it borrows the *concept* of an iterable
 * command section from Zephyr's shell but reuses none of its code.
 *
 * Scope of this file currently covers command registration only.  Output API
 * (cli_print/...), instance definition and the full `struct cli_instance` are
 * added by later issues; here cli_instance is only forward-declared so handler
 * pointers type-check.
 */
#ifndef CLI_H
#define CLI_H

#include <stdbool.h>  /* bool (cli_cancel_requested) */
#include <stddef.h>   /* size_t */
#include <stdint.h>   /* uint8_t */

#include "cli_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque shell instance; full definition comes with the core (issue #4). */
struct cli_instance;

/**
 * Command handler.  Receives the owning shell instance and the parsed argument
 * vector (argv[0] is the command name).  Returns 0 on success; non-zero marks
 * an error.  argc/argv validation (mandatory/optional) runs before the handler;
 * if it fails the handler is not called.
 *
 * A long-running or large-output handler should poll cli_cancel_requested()
 * (and use cli_sleep() for any delay) so Ctrl+C can interrupt it -- cancellation
 * is cooperative; the core never force-kills the handler thread (issue #16).
 */
typedef int (*cli_cmd_handler_t)(struct cli_instance *sh, int argc, char **argv);

/**
 * One command descriptor -- used for both root commands (placed in
 * .shell_root_cmds) and subcommand-set entries.
 */
struct cli_cmd {
	const char           *name;     /**< command name; NULL marks a sentinel */
	const char           *help;     /**< one-line help text */
	const struct cli_cmd *subcmds;  /**< sentinel-terminated subcommand array, or NULL */
	cli_cmd_handler_t     handler;  /**< handler, or NULL for a pure parent command */
	uint8_t               mandatory;/**< required argc (command name included) */
	uint8_t               optional; /**< number of optional args, or CLI_ARG_RAW */
};

/**
 * Sentinel for the `optional` field / the CLI_CMD_REGISTER / CLI_CMD_ARG
 * `optional` argument.  Marks a RAW command: after the `mandatory` tokens are
 * tokenized normally, the rest of the input line is passed verbatim (no quote
 * or escape processing, leading whitespace trimmed) as a single extra argument,
 * and the argument-count upper bound is not enforced.  Leaf commands only
 * (a command with no subcommands) -- on a command that has subcommands the raw
 * tail is not captured and CLI_ARG_RAW just acts as a large optional count.
 */
#define CLI_ARG_RAW 0xFFu

/*
 * Section attribute for a root command entry.  `used` + the linker's KEEP keep
 * the entry through -Wl,--gc-sections; `aligned(__alignof__(struct cli_cmd))`
 * pins the input-section alignment to the type's natural alignment so the
 * linker packs entries back-to-back with no padding, which is what lets the
 * section be walked as a plain array via pointer arithmetic.
 */
#define CLI_SECTION_ATTR(nm) \
	__attribute__((used, section(".shell_root_cmds." nm), \
	               aligned(__alignof__(struct cli_cmd))))

/**
 * Register a root command into the .shell_root_cmds section.
 *
 * @param name       command name as a bare C identifier (stringified; also used
 *                   to build a unique symbol/section name)
 * @param subcmds    pointer to a subcommand set (CLI_SUBCMD_SET_CREATE) or NULL
 * @param help       one-line help string
 * @param handler    cli_cmd_handler_t, or NULL for a pure parent command
 * @param mandatory  required argc including the command name
 * @param optional   number of optional arguments allowed
 *
 * Note: `name` must be a valid C identifier, so names with dashes (e.g.
 * "foo-bar") cannot be registered this way today.  Every standard-tier command
 * (version/uptime/reboot/thread/devmem/help/clear/history/backends) is a valid
 * identifier.  Duplicate root names are not detected: command lookup takes the
 * first match in SORT_BY_NAME order, so avoid registering the same root twice.
 */
/* Parameters are underscore-prefixed so a `name`/`help`/... parameter does not
 * collide with the identically named designated-initializer field below (the
 * preprocessor would otherwise rewrite `.name` into `.<arg>`). */
#define CLI_CMD_REGISTER(_name, _subcmds, _help, _handler, _mandatory, _optional) \
	static const struct cli_cmd __cli_cmd_##_name CLI_SECTION_ATTR(#_name) = { \
		.name      = #_name, \
		.help      = (_help), \
		.subcmds   = (_subcmds), \
		.handler   = (_handler), \
		.mandatory = (_mandatory), \
		.optional  = (_optional), \
	}

/**
 * Create a static subcommand set: a `struct cli_cmd[]` initialised with the
 * given CLI_CMD / CLI_CMD_ARG entries and terminated by CLI_SUBCMD_SET_END.
 */
#define CLI_SUBCMD_SET_CREATE(_set_name, ...) \
	static const struct cli_cmd _set_name[] = { __VA_ARGS__ }

/** A subcommand-set entry with explicit mandatory/optional argument counts. */
#define CLI_CMD_ARG(_name, _subcmds, _help, _handler, _mandatory, _optional) \
	{ .name = #_name, .help = (_help), .subcmds = (_subcmds), .handler = (_handler), \
	  .mandatory = (_mandatory), .optional = (_optional) }

/** A subcommand-set entry with the default argument counts (mandatory=1, optional=0). */
#define CLI_CMD(_name, _subcmds, _help, _handler) \
	CLI_CMD_ARG(_name, _subcmds, _help, _handler, 1, 0)

/** Sentinel that terminates a subcommand set. */
#define CLI_SUBCMD_SET_END { .name = NULL }

/* Boundary symbols provided by the linker script (PROVIDE_HIDDEN). */
extern const struct cli_cmd __cli_root_cmds_start[];
extern const struct cli_cmd __cli_root_cmds_end[];

/*
 * Output API (impl. #5).  Handlers print through these; each call formats into a
 * per-instance staging buffer, flushes to the instance's own transport under the
 * TX lock, and is flow-controlled (blocks on TX space, with a timeout).  Return
 * 0 on full success, <0 if output failed (a TX timeout dropped bytes, or the
 * output lock could not be acquired) -- a handler may propagate that as a
 * non-zero exit, and the core also forces a non-zero command result when output
 * failed (req §11).  Output API is thread-context only (it
 * waits on a ThreadX mutex / event flag); never call it from an ISR.  cli_error
 * is red, cli_warn yellow, cli_info green (req §2; disable with CLI_USE_COLOR=0).
 */
int cli_write(struct cli_instance *sh, const void *data, size_t len);
int cli_print(struct cli_instance *sh, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));
int cli_error(struct cli_instance *sh, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));
int cli_warn(struct cli_instance *sh, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));
int cli_info(struct cli_instance *sh, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));
int cli_hexdump(struct cli_instance *sh, const void *data, size_t len);
/* Like cli_hexdump, but the left-hand offset column counts from `base` instead
 * of 0 -- pass the data's absolute address to print absolute addresses (used by
 * the devmem dump command).  cli_hexdump() is the base == 0 case. */
int cli_hexdump_base(struct cli_instance *sh, const void *data, size_t len,
                     unsigned long long base);

/*
 * Cooperative cancellation (issue #16).  A running command is interrupted only
 * if it checks in -- the core never force-kills the handler thread (req §9).
 *
 * cli_cancel_requested(): true once the user pressed Ctrl+C (0x03) during THIS
 * command.  Long loops / large output should poll it and return promptly (any
 * non-zero result); the core then prints "^C" and restores the prompt.  Sticky
 * for the rest of the command.  Thread-context only (it drains the transport) --
 * never call it from an ISR.
 *
 * cli_sleep(): a cancellable delay of @p ticks ThreadX ticks (NOT milliseconds).
 * Returns 0 when the full delay elapsed, non-zero if it was cancelled (Ctrl+C)
 * or the instance is stopping; a handler should propagate the non-zero result.
 * ticks == 0 returns 0 immediately.  Building block for watch/sleep (#21).
 */
bool cli_cancel_requested(struct cli_instance *sh);
int  cli_sleep(struct cli_instance *sh, unsigned ticks);

/*
 * Raw binary console transfer (issue #50).  A command that streams binary data
 * over the same UART as the shell (e.g. a YMODEM send) takes over the console for
 * the duration of the transfer, bypassing the line editor / echo / Ctrl+C poll.
 *
 * cli_console_claim(): hold the output lock for the whole transfer (so background
 *   output cannot interleave into the byte stream) and enter raw mode (printf
 *   output is dropped; cli_tx_send_blocking stops draining RX).  Returns 0 on
 *   success, -2 if called from a background job (a transfer must run in the
 *   foreground -- it owns the shared RX), -1 if the lock could not be acquired.
 *   Pair with cli_console_release().  cli_write() is still the binary-raw,
 *   flow-controlled TX path to use for protocol bytes (the lock is reentrant).
 * cli_read_byte(): timed raw RX read -- returns 0..255 on a byte, -1 on timeout
 *   (timeout_ms; 0 = poll once), -2 if the instance is stopping.  Does NOT treat
 *   0x03 specially (the protocol decides what a Ctrl+C means).
 * cli_rx_flush(): discard buffered RX (call before and after a transfer so stray
 *   protocol bytes do not land on the prompt).
 * Thread-context only; never call from an ISR.
 */
int  cli_console_claim(struct cli_instance *sh);
void cli_console_release(struct cli_instance *sh);
int  cli_read_byte(struct cli_instance *sh, unsigned timeout_ms);
void cli_rx_flush(struct cli_instance *sh);

/** Iterate every registered root command in .shell_root_cmds order. */
#define CLI_ROOT_CMD_FOREACH(it) \
	for (const struct cli_cmd *it = __cli_root_cmds_start; \
	     it < __cli_root_cmds_end; ++it)

/** Number of registered root commands. */
static inline size_t cli_root_cmd_count(void)
{
	return (size_t)(__cli_root_cmds_end - __cli_root_cmds_start);
}

#ifdef __cplusplus
}
#endif

#endif /* CLI_H */
