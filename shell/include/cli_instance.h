/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_instance.h
 * @brief   Shell instance definition, transport abstraction and lifecycle API.
 *
 * This header carries the *ThreadX-aware* part of the shell that the command
 * registration header (cli.h) deliberately does not: it pulls in tx_api.h and
 * defines `struct cli_instance` (per-transport state: thread, event flags,
 * mutex, line buffer, parser scratch, prompt, RX state machine) plus the
 * transport interface (`struct cli_transport` / `struct cli_transport_api`) and
 * the lifecycle entry points (cli_init / cli_start).
 *
 * Only the core, the backends and the application include this header.  Command
 * source files and the host parser test include cli.h alone, which stays
 * ThreadX-independent, so they need no ThreadX headers to compile.  The line
 * editor / dispatch logic that operates on a `struct cli_instance` lives in
 * cli_session.c and never calls a tx_* function, which is what lets it be
 * compiled and unit-tested on the host against a small tx_api.h type shim.
 *
 * Clean-room design inspired by Zephyr shell's per-instance model; no code
 * reused.
 */
#ifndef CLI_INSTANCE_H
#define CLI_INSTANCE_H

#include <stddef.h>   /* size_t */
#include <stdint.h>   /* uintN_t */

#include "tx_api.h"        /* TX_THREAD / TX_EVENT_FLAGS_GROUP / TX_MUTEX / UCHAR / ULONG */
#include "cli_internal.h"  /* cli.h, CLI_ARGV_CAP, struct cli_parse_result */

#ifdef __cplusplus
extern "C" {
#endif

/* Event-flag bits signalled to an instance thread. */
#define CLI_EVT_RX    0x1u   /**< RX data available (set by the backend / ISR) */
#define CLI_EVT_TX    0x2u   /**< reserved for #5 flow control; unused in #4 */
#define CLI_EVT_KILL  0x4u   /**< request the thread to stop (full stop is future) */
#define CLI_EVT_CONN  0x8u   /**< a transport (re)connected: begin a fresh session
                              *   (issue #49 P4 -- the TCP backend posts this on
                              *   accept so the instance thread resets the editor
                              *   state and redraws the prompt; never set for a
                              *   persistent transport like the UART). */

/* Cooperative Ctrl+C cancel (issue #16) deliberately adds NO new event-flag bit:
 * the wake source for an in-flight command is the existing CLI_EVT_RX (the ISR
 * already sets it on every received byte).  The cancel STATE lives in
 * struct cli_instance::cancel_req; the 0x03 is detected in thread context by
 * draining the rx_ring (cli_cancel_poll), which keeps the backend a dumb byte
 * pipe (no 0x03 semantics in the ISR/backend). */

/* RX byte state machine (cli_edit.c, issue #9).  ESC begins an escape sequence;
 * '[' enters CSI (parameter-accumulating: arrows / Home / End / Del / Insert /
 * the CPR width report), 'O' enters SS3 (application-mode arrows), and a bare
 * 'b'/'f' is Alt+word-move.  Unknown / malformed sequences fall back to NORMAL
 * and are ignored (req §13). */
enum cli_rx_state {
	CLI_RX_NORMAL = 0,
	CLI_RX_ESC,            /**< saw ESC (0x1B); next byte selects CSI/SS3/meta */
	CLI_RX_CSI,            /**< inside ESC[ ... ; accumulating params to the final byte */
	CLI_RX_SS3,            /**< inside ESC O ... ; one final byte, no params */
};

/* Instance lifecycle state. */
enum cli_state {
	CLI_UNINIT = 0,
	CLI_INITED,
	CLI_STARTED,
};

struct cli_transport;

/**
 * Backend transport interface.  The core talks to hardware only through this.
 * init/enable/write/read are mandatory; uninit/update may be NULL.
 *
 * write (#5 contract): NON-blocking.  Enqueue up to @p len bytes into the
 * backend TX buffer and return the count actually accepted (0..len), or <0 on
 * error.  It must NOT block.  If it accepts fewer than @p len (TX full), the
 * backend is obliged to fire cli_transport_notify_tx() once space frees again --
 * the core suspends on CLI_EVT_TX waiting for exactly that, and realises the
 * "blocking until sent" behaviour of req §11 itself (with a timeout).
 * read: non-blocking drain returning 0..cap bytes copied.
 */
struct cli_transport_api {
	int  (*init)(struct cli_transport *tr);                          /**< prepare; sh is tr->sh */
	int  (*enable)(struct cli_transport *tr);                        /**< start RX (e.g. enable IRQ) */
	int  (*write)(struct cli_transport *tr, const uint8_t *data, size_t len); /**< non-blocking; ret accepted 0..len or <0 */
	int  (*read)(struct cli_transport *tr, uint8_t *data, size_t cap);        /**< non-blocking; ret 0..cap */
	void (*uninit)(struct cli_transport *tr);                        /**< optional (NULL ok) */
	void (*update)(struct cli_transport *tr);                        /**< optional periodic poll (NULL ok), req §4.1 */
	/* Optional (NULL ok): the instance thread calls this on CLI_EVT_CONN, AFTER
	 * resetting the editor state and BEFORE redrawing the prompt, to let a
	 * connection-oriented backend enable its output for the new session (the TCP
	 * backend sets `connected` here so the fresh prompt is the first thing the new
	 * client sees, and a previous command's output that drained after a reconnect
	 * is dropped -- see issue #49 P4).  Never invoked for a persistent transport. */
	void (*session_begin)(struct cli_transport *tr);
};

/** A backend instance: API table + owning shell + backend-private context. */
struct cli_transport {
	const struct cli_transport_api *api;
	struct cli_instance            *sh;   /**< set by cli_init() */
	void                           *ctx;  /**< backend-private (owns the HAL handle etc.) */
};

/**
 * One shell instance: all per-transport state, statically allocated (no heap).
 * The command tree is read-only data shared by every instance; the core keeps
 * no mutable globals, so several instances run concurrently without interfering
 * (requirements §10).
 */
struct cli_instance {
	/* ThreadX primitives (created by cli_init / cli_start). */
	TX_THREAD            thread;
	TX_EVENT_FLAGS_GROUP events;
	TX_MUTEX             tx_lock;     /**< created TX_INHERIT; reserved for #5 locked output */
	UCHAR               *stack;       /**< thread stack base (from CLI_INSTANCE_DEFINE) */
	ULONG                stack_size;

	struct cli_transport *tr;

	/* Background job link (issue #25).  NULL for an interactive instance; for a
	 * bg-job worker instance it points at the launching foreground shell.  When
	 * set, the output path (cli_lock / cli_tx_send_blocking TX-wait) targets the
	 * FOREGROUND's tx_lock + transport so bg output serialises against the fg
	 * line editor; cancel is kill-driven (the worker never drains the shared RX). */
	struct cli_instance *fg;

	/* Line input + cursor (issue #9: cur is split out from len for in-line edit). */
	char     line[CLI_CMD_BUFFER_SIZE];
	uint16_t len;                    /**< chars in line[]; line[len] kept NUL */
	uint16_t cur;                    /**< cursor index, 0..len (insert point) */
	uint8_t  overwrite;              /**< 0=insert (default), 1=overwrite; Insert key toggles */

	/* Command history: fixed byte ring (issue #10, req §8).  Entries are packed
	 * oldest->newest in hist[0..hist_used), each '\0'-terminated.  Adding a line
	 * evicts the oldest entries FIFO until the new one fits; no dynamic
	 * allocation.  All state is per-instance, so histories never cross (req §10). */
	char     hist[CLI_HISTORY_BUFFER_SIZE];
	uint16_t hist_used;              /**< bytes used incl. terminators (0..CLI_HISTORY_BUFFER_SIZE) */
	uint8_t  hist_nav_on;            /**< 1 = recalling a history entry; 0 = on the live line */
	uint16_t hist_nav;               /**< offset in hist[] of the entry currently recalled */

	/* Per-instance parser scratch (sized by #3's CLI_ARGV_CAP). */
	char                  *argv[CLI_ARGV_CAP];
	struct cli_parse_result pr;

	/* RX state machine + escape parsing (issue #9). */
	enum cli_rx_state rx;
	uint8_t           prev_cr;       /**< last byte was CR: swallow a following LF */
	uint16_t          esc_p[2];      /**< CSI numeric params (e.g. ESC[3~, ESC[r;cR) */
	uint8_t           esc_np;        /**< number of params seen (0..2) */
	uint8_t           esc_bad;       /**< current CSI had >2 params / extra ';' (reject CPR) */

	/* Terminal / line-editor render state (issue #9). */
	uint8_t  bs_swap;                /**< backspace mode: 1 makes DEL (0x7F) delete forward */
	uint8_t  term_width;             /**< columns; CLI_TERM_WIDTH until a CPR updates it */
	uint8_t  old_rows;               /**< physical rows the last render occupied (0 = none) */
	uint8_t  draw_row;               /**< row of the physical cursor within the render (0-based) */
	uint8_t  probing_cpr;            /**< a width probe (ESC[6n) is awaiting its CPR reply */
	uint8_t  tab_list_armed;         /**< Tab completion (issue #11): 1 = next Tab lists candidates;
	                                  *   set after an LCP-only extend or a no-extend first Tab,
	                                  *   reset by any non-Tab byte (bash-style two-stage). */
	volatile uint8_t render_dirty;   /**< issue #25: bg output broke this instance's input-line
	                                  *   render (old_rows/draw_row reset to 0 under tx_lock); the
	                                  *   fg thread repaints prompt+line in cli_input_byte() BEFORE
	                                  *   the next byte (incl. the fast-append path) and clears it. */

	char prompt[CLI_PROMPT_BUFFER_SIZE];

	/* Output staging + flow control (#5). */
	char     out_buf[CLI_PRINTF_BUFFER_SIZE]; /**< staging buffer; flushed when full */
	uint16_t out_len;
	uint8_t  tx_failed;              /**< output dropped this command (TX timeout); reset each dispatch */
	uint8_t  dispatching;            /**< 1 while a command handler runs (issue #16): gates the
	                                  *   cooperative Ctrl+C cancel paths (TX-blocked RX wake, fast-fail)
	                                  *   so the post-cancel ^C/prompt cleanup is never suppressed */
	volatile uint8_t cancel_req;     /**< sticky: a 0x03 (Ctrl+C) was seen during the running command
	                                  *   (issue #16); set by cli_cancel_poll, cleared each dispatch */

	enum cli_state state;
	int            last_result;      /**< return value of the most recent command */
	uint32_t       rx_dropped;       /**< RX-overflow drop count (incremented in #7) */
	uint32_t       tx_dropped;       /**< bytes dropped on TX timeout (#5) */
};

/**
 * Statically define a shell instance @p _name bound to transport @p _transport_ptr
 * with prompt string literal @p _prompt_str.  Allocates the thread stack (8-byte
 * aligned, ThreadX requires >=200 B) and seeds the config fields; all runtime
 * state is zero-initialised, i.e. CLI_UNINIT, until cli_init().
 *
 * Enforcing CLI_MAX_INSTANCES at build time happens where instances are
 * collected for startup (issues #6/#8), not here.
 */
#define CLI_INSTANCE_DEFINE(_name, _transport_ptr, _prompt_str)              \
	_Static_assert(sizeof(_prompt_str) <= CLI_PROMPT_BUFFER_SIZE,        \
		"CLI_INSTANCE_DEFINE: prompt exceeds CLI_PROMPT_BUFFER_SIZE"); \
	static UCHAR _name##_stack[CLI_INSTANCE_STACK_SIZE]                  \
		__attribute__((aligned(8)));                                \
	struct cli_instance _name = {                                       \
		.tr         = (_transport_ptr),                             \
		.stack      = _name##_stack,                                \
		.stack_size = CLI_INSTANCE_STACK_SIZE,                      \
		.prompt     = _prompt_str,                                  \
	}

/* Lifecycle (cli_core.c, ThreadX).  cli_init prepares the backend and creates
 * the ThreadX objects; cli_start spawns the auto-started instance thread.  Both
 * are one-shot per instance: a failed cli_init disables only that instance
 * (non-zero return, terminal -- do not retry) and leaves the rest of the system
 * running (requirements §9). */
int  cli_init(struct cli_instance *sh);
int  cli_start(struct cli_instance *sh);

/* Backend-to-core notifications (cli_core.c).  Called from the backend,
 * including from ISR context, to wake the instance thread.  They only set an
 * event flag -- never take a lock or touch the line state (requirements §10). */
void cli_transport_notify_rx(struct cli_instance *sh);
void cli_transport_notify_tx(struct cli_instance *sh);   /* reserved for #5 */
void cli_transport_notify_conn(struct cli_instance *sh); /* issue #49 P4: (re)connect -> fresh session */

/* Thread->instance registry (#18).  Lets a backend's printf retarget (_write)
 * resolve which shell instance owns the running thread, so printf follows the
 * calling terminal instead of a single global console.  cli_start() registers
 * its own instance thread before creating it; future background-job workers
 * (#25) register their worker thread against the owning instance the same way
 * (register BEFORE tx_thread_create so an auto-started thread is never seen
 * unregistered).  cli_register_thread returns 0 on success, -1 when the table
 * is full (caller must treat that as a failure, never silently continue --
 * see CLI_THREAD_MAP_MAX).  cli_current_instance returns NULL from ISR context,
 * before the scheduler starts, or for any unregistered thread. */
int                  cli_register_thread(TX_THREAD *t, struct cli_instance *sh);
void                 cli_unregister_thread(TX_THREAD *t);
struct cli_instance *cli_current_instance(void);

/* Background jobs (#25, cli_job.c).  `cmd &` runs the command in a worker thread
 * drawn from a fixed static pool.  cli_job_pool_init() creates the pool's event
 * groups once at boot (call after the interactive instances start).  The other
 * entry points run on the foreground thread: cli_job_launch() spawns a worker
 * for an already-`&`-stripped segment (returns 0 on success, <0 if the pool /
 * thread registry is full -- it prints its own error); cli_jobs_reap() deletes
 * any completed workers (TX_COMPLETED) and prints their done/killed notice;
 * cli_jobs_print() is the `jobs` listing; cli_job_kill() requests a cooperative
 * stop of the running job with id @p id (returns 0 if found+signalled, <0 if no
 * such running job).  cli_job_launch / cli_jobs_reap are also referenced by the
 * ThreadX-free cli_session.c, so the host test harness stubs them. */
void cli_job_pool_init(void);
int  cli_job_launch(struct cli_instance *fg, char *seg);
void cli_jobs_reap(struct cli_instance *fg);
void cli_jobs_print(struct cli_instance *sh);
int  cli_job_kill(struct cli_instance *sh, unsigned long id);

#ifdef __cplusplus
}
#endif

#endif /* CLI_INSTANCE_H */
