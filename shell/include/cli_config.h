/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_config.h
 * @brief   Compile-time configuration knobs and resource limits for the Shell.
 *
 * Every knob has a default matching the requirements spec (§8) and may be
 * overridden at build time (e.g. -DCLI_CMD_BUFFER_SIZE=512) since each is wrapped
 * in #ifndef.  This task (#2) only *defines* the knobs; the over-limit runtime
 * behaviour noted next to each (BEL, FIFO discard, truncate, ...) is implemented
 * by the later issues referenced in the comments.
 *
 * All shell storage is statically allocated -- no heap is used at run time.
 */
#ifndef CLI_CONFIG_H
#define CLI_CONFIG_H

/* Command (line) input buffer length in bytes.
 * Over limit: ring the bell (BEL) and ignore further input chars (impl. #4). */
#ifndef CLI_CMD_BUFFER_SIZE
#define CLI_CMD_BUFFER_SIZE 256
#endif

/* Maximum argument count (argc), command name included.
 * Over limit: show an error and do not execute the command (impl. #3). */
#ifndef CLI_MAX_ARGC
#define CLI_MAX_ARGC 20
#endif

/* Command history ring size in bytes.
 * Over limit: discard the oldest entry first, FIFO (impl. #10). */
#ifndef CLI_HISTORY_BUFFER_SIZE
#define CLI_HISTORY_BUFFER_SIZE 512
#endif

/* printf/output staging buffer in bytes.
 * Over limit: flush when full (impl. #5, flow control §11). */
#ifndef CLI_PRINTF_BUFFER_SIZE
#define CLI_PRINTF_BUFFER_SIZE 32
#endif

/* Prompt buffer length in bytes.
 * Over limit: truncate (impl. #4/#9). */
#ifndef CLI_PROMPT_BUFFER_SIZE
#define CLI_PROMPT_BUFFER_SIZE 20
#endif

/* Per-instance ThreadX thread stack size in bytes (impl. #4). */
#ifndef CLI_INSTANCE_STACK_SIZE
#define CLI_INSTANCE_STACK_SIZE 2048
#endif

/* Maximum number of concurrent shell instances.
 * Fixed at compile time; exceeding it is a build-time error.  The actual count
 * check happens where instances are defined (#4/#6). */
#ifndef CLI_MAX_INSTANCES
#define CLI_MAX_INSTANCES 4
#endif

/* Maximum number of concurrent background jobs (issue #25): `cmd &` runs a
 * command in a worker thread drawn from a fixed static pool of this size.  Over
 * limit: the launch is rejected with an error (no dynamic allocation -- the pool
 * is statically sized).  Each slot costs one struct cli_instance + one worker
 * stack (CLI_BG_JOB_STACK_SIZE), so keep it small. */
#ifndef CLI_MAX_BG_JOBS
#define CLI_MAX_BG_JOBS 2
#endif

/* Per-job worker thread stack size in bytes (issue #25).  Defaults to the shell
 * instance stack; the threadx exe overrides both to 4096 so any command --
 * including coremark -- can run in the background.  A bg job runs an arbitrary
 * registered handler, so it needs the same headroom as the interactive thread. */
#ifndef CLI_BG_JOB_STACK_SIZE
#define CLI_BG_JOB_STACK_SIZE CLI_INSTANCE_STACK_SIZE
#endif

/* ThreadX priority of each background-job worker thread (issue #25).  One step
 * BELOW (numerically above) the interactive shell so a CPU-bound bg job (e.g.
 * `coremark &`, which never blocks) can never starve the interactive prompt --
 * any keystroke preempts it.  Plain integer (cli_config.h stays ThreadX-free). */
#ifndef CLI_BG_JOB_PRIORITY
#define CLI_BG_JOB_PRIORITY (CLI_INSTANCE_PRIORITY + 1)
#endif

/* TX-backpressure poll slice for a bg job in ThreadX ticks (issue #25).  When a
 * bg job's output finds the TX ring full it cannot wait on the foreground's
 * CLI_EVT_TX (a different event group), so it waits this long on its OWN events
 * for a kill, then retries the write -- while still honouring the overall
 * CLI_TX_TIMEOUT deadline so a wedged TX never pins the shared output lock. */
#ifndef CLI_BG_TX_POLL_TICKS
#define CLI_BG_TX_POLL_TICKS 2u
#endif

/* Overall no-progress deadline for a bg job's TX-full wait when CLI_TX_TIMEOUT is
 * 0 (== "never drop") (issue #25).  A bg job holds the SHARED fg->tx_lock while
 * sending, so -- unlike the interactive path -- it must NOT honour an infinite
 * timeout: a wedged TX would pin the lock and freeze the foreground.  So a bg job
 * always uses a finite deadline: CLI_TX_TIMEOUT when non-zero, else this value. */
#ifndef CLI_BG_TX_WEDGE_TICKS
#define CLI_BG_TX_WEDGE_TICKS 1000u
#endif

/* Size of the thread->instance registry that backs cli_current_instance()
 * (#18): printf/_write resolves the owning shell instance of the running
 * thread from this table.  Per-instance shell threads register one slot each;
 * each background-job worker (#25) also registers while it runs -> size for both
 * so a launch never fails to register (a full table makes cli_register_thread()
 * return -1, which the caller must treat as an error rather than misroute). */
#ifndef CLI_THREAD_MAP_MAX
#define CLI_THREAD_MAP_MAX (CLI_MAX_INSTANCES + CLI_MAX_BG_JOBS)
#endif

/* Maximum static subcommand tree nesting depth.
 * Over limit: show an error and do not execute the command (impl. #3). */
#ifndef CLI_MAX_SUBCMD_DEPTH
#define CLI_MAX_SUBCMD_DEPTH 8
#endif

/* ThreadX priority of each shell instance thread (impl. #4).  Plain integer
 * (no ThreadX symbols here, so cli_config.h stays ThreadX-independent and
 * host-includable).  Default 16: a development shell runs *below* the demo
 * application threads (priority 10 in src/app_threadx.c) so it never starves
 * real work; the <5 ms echo target (req §15) is paced by IRQ -> event flag ->
 * thread wake-up and is verified on target in issue #8. */
#ifndef CLI_INSTANCE_PRIORITY
#define CLI_INSTANCE_PRIORITY 16
#endif

/* Bytes drained from the transport per read() in the instance thread loop
 * (impl. #4).  Purely a batching size; does not bound input length. */
#ifndef CLI_RX_DRAIN_CHUNK
#define CLI_RX_DRAIN_CHUNK 32
#endif

/* TX flow-control timeout in ThreadX ticks (impl. #5, req §11): how long an
 * output blocks waiting for transport TX space before it gives up, drops the
 * rest, bumps the drop stat and fails the call.  0 == wait forever (never drop).
 * Default 1000 ~= 1 s on the usual 1 kHz ThreadX tick.  Plain integer (no
 * ThreadX symbol here); cli_core.c maps 0 -> TX_WAIT_FOREVER. */
#ifndef CLI_TX_TIMEOUT
#define CLI_TX_TIMEOUT 1000
#endif

/* TX/output mutex acquire timeout in ThreadX ticks (impl. #5).  0 == wait
 * forever; cli_core.c maps 0 -> TX_WAIT_FOREVER. */
#ifndef CLI_TX_MUTEX_WAIT
#define CLI_TX_MUTEX_WAIT 0
#endif

/* Colour output (impl. #5): 1 emits VT100 SGR for cli_error/warn/info, 0 emits
 * none (monochrome terminals / logs). */
#ifndef CLI_USE_COLOR
#define CLI_USE_COLOR 1
#endif

/* Default terminal width in columns (impl. #9).  Used for line-wrap redraw until
 * a CPR (cursor-position report) auto-detects the real width; also the fallback
 * for terminals that never answer the probe.  Plain integer, host-includable. */
#ifndef CLI_TERM_WIDTH
#define CLI_TERM_WIDTH 80
#endif

/* Backspace mode (impl. #9): which of BS (0x08) / DEL (0x7F) deletes forward.
 *   0 -- both 0x08 and 0x7F erase backward (Backspace).  Forward delete is on
 *        Ctrl+d and the Delete key (ESC[3~) only.  (Default.)
 *   1 -- 0x08 erases backward, 0x7F (DEL) deletes the char *under* the cursor
 *        (for terminals whose Backspace key sends 0x08 and Delete sends 0x7F).
 * Seeds the per-instance bs_swap field; flip at runtime with
 * cli_set_backspace_mode(). */
#ifndef CLI_BACKSPACE_MODE
#define CLI_BACKSPACE_MODE 0
#endif

/* Compile-time gate for dangerous commands (reboot, devmem), spec §12.  1 ==
 * built in (shell demo default), 0 == compiled out entirely so the descriptor
 * never reaches .shell_root_cmds (gone from help/completion too).  Production
 * integrations should build with -DCLI_ENABLE_DANGEROUS_CMDS=0.  The shell exe
 * forwards the CMake option of the same name to this define; this #ifndef
 * default is the fall-back for targets that do not (e.g. host tests). */
#ifndef CLI_ENABLE_DANGEROUS_CMDS
#define CLI_ENABLE_DANGEROUS_CMDS 1
#endif

/* Upper bound (bytes) on a single `devmem dump`.  Its hexdump holds the output
 * lock for the whole run, so an unbounded length would block other instances'
 * output for a long time (req §10); a request above this is rejected.  Override
 * via the CMake cache variable of the same name (forwarded to the define). */
#ifndef CLI_DEVMEM_DUMP_MAX_LEN
#define CLI_DEVMEM_DUMP_MAX_LEN 256
#endif

/* sleep / usleep / watch bounds (issue #21). */
/* `sleep N` upper bound in seconds.  cli_sleep() is cancellable, so a long sleep
 * is fine, but cap to keep N*1000 within a 32-bit ThreadX tick and reject typos. */
#ifndef CLI_SLEEP_MAX_SEC
#define CLI_SLEEP_MAX_SEC 86400u          /* 1 day */
#endif
/* `usleep N` upper bound in microseconds.  It busy-waits (no yield, not
 * cancellable), so keep it small; use `sleep` for long delays. */
#ifndef CLI_USLEEP_MAX_US
#define CLI_USLEEP_MAX_US 10000u          /* 10 ms */
#endif
/* `watch` default and maximum refresh interval in seconds. */
#ifndef CLI_WATCH_DEFAULT_SEC
#define CLI_WATCH_DEFAULT_SEC 2u
#endif
#ifndef CLI_WATCH_MAX_SEC
#define CLI_WATCH_MAX_SEC 3600u           /* 1 hour */
#endif

/*
 * The number of registered commands is bounded only by the linker section
 * capacity (effectively unlimited; the scan is linear).  Tab-completion does
 * not allocate -- it scans the command tree and prints inline, so there is no
 * "max completion candidates" knob.
 */

/* Sanity checks for the knobs that have meaning at this stage. */
_Static_assert(CLI_CMD_BUFFER_SIZE > 0,     "CLI_CMD_BUFFER_SIZE must be > 0");
_Static_assert(CLI_MAX_ARGC >= 1,           "CLI_MAX_ARGC must be >= 1");
_Static_assert(CLI_HISTORY_BUFFER_SIZE > 0, "CLI_HISTORY_BUFFER_SIZE must be > 0");
_Static_assert(CLI_HISTORY_BUFFER_SIZE <= 65535,
               "CLI_HISTORY_BUFFER_SIZE must fit uint16_t hist_used/hist_nav");
_Static_assert(CLI_PRINTF_BUFFER_SIZE > 0,  "CLI_PRINTF_BUFFER_SIZE must be > 0");
_Static_assert(CLI_PROMPT_BUFFER_SIZE > 0,  "CLI_PROMPT_BUFFER_SIZE must be > 0");
_Static_assert(CLI_INSTANCE_STACK_SIZE >= 512, "CLI_INSTANCE_STACK_SIZE too small");
_Static_assert(CLI_MAX_INSTANCES >= 1,      "CLI_MAX_INSTANCES must be >= 1");
_Static_assert(CLI_MAX_SUBCMD_DEPTH >= 1,   "CLI_MAX_SUBCMD_DEPTH must be >= 1");
_Static_assert(CLI_INSTANCE_PRIORITY <= 31, "CLI_INSTANCE_PRIORITY must be 0..31 (ThreadX)");
_Static_assert(CLI_MAX_BG_JOBS >= 1,        "CLI_MAX_BG_JOBS must be >= 1");
_Static_assert(CLI_BG_JOB_STACK_SIZE >= 512, "CLI_BG_JOB_STACK_SIZE too small");
_Static_assert(CLI_BG_JOB_PRIORITY <= 31,   "CLI_BG_JOB_PRIORITY must be 0..31 (ThreadX)");
_Static_assert(CLI_BG_TX_POLL_TICKS >= 1,   "CLI_BG_TX_POLL_TICKS must be >= 1");
_Static_assert(CLI_RX_DRAIN_CHUNK >= 1,     "CLI_RX_DRAIN_CHUNK must be >= 1");
_Static_assert(CLI_TERM_WIDTH >= 20 && CLI_TERM_WIDTH <= 255,
               "CLI_TERM_WIDTH must be 20..255 (fits uint8_t term_width)");
_Static_assert(CLI_BACKSPACE_MODE == 0 || CLI_BACKSPACE_MODE == 1,
               "CLI_BACKSPACE_MODE must be 0 or 1");
_Static_assert(CLI_ENABLE_DANGEROUS_CMDS == 0 || CLI_ENABLE_DANGEROUS_CMDS == 1,
               "CLI_ENABLE_DANGEROUS_CMDS must be 0 or 1");
_Static_assert(CLI_DEVMEM_DUMP_MAX_LEN > 0, "CLI_DEVMEM_DUMP_MAX_LEN must be > 0");
_Static_assert(CLI_SLEEP_MAX_SEC > 0 && CLI_SLEEP_MAX_SEC <= 0xFFFFFFFFu / 1000u,
               "CLI_SLEEP_MAX_SEC*1000 must fit a 32-bit ThreadX tick");
/* usleep busy-wait multiplies us by SystemCoreClock/1e6 = 550 (DWT cycles/us at
 * 550 MHz); keep the product in uint32. */
_Static_assert(CLI_USLEEP_MAX_US > 0 && CLI_USLEEP_MAX_US <= 0xFFFFFFFFu / 550u,
               "CLI_USLEEP_MAX_US*550 must fit uint32 (DWT cycles)");
_Static_assert(CLI_WATCH_DEFAULT_SEC <= CLI_WATCH_MAX_SEC,
               "CLI_WATCH_DEFAULT_SEC must be <= CLI_WATCH_MAX_SEC");
_Static_assert(CLI_WATCH_MAX_SEC <= 0xFFFFFFFFu / 1000u,
               "CLI_WATCH_MAX_SEC*1000 must fit a 32-bit ThreadX tick");

#endif /* CLI_CONFIG_H */
