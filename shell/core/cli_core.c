/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_core.c
 * @brief   Shell ThreadX glue: instance lifecycle, thread loop, ISR notify.
 *
 * This is the only shell core file that calls ThreadX (tx_*) APIs; the line
 * editing (cli_edit.c) and dispatch (cli_session.c) logic it drives stays
 * ThreadX-free for host unit testing.  Per instance it owns one tx_thread, one
 * tx_event_flags group (RX / TX / KILL) and one tx_mutex.  The thread blocks on
 * the event flags, drains the transport on an RX signal and feeds each byte to
 * the state machine.  No mutable global state, so several instances run
 * concurrently and independently (requirements §10).
 *
 * Clean-room design inspired by Zephyr shell's thread model; no code reused.
 */
#include <stddef.h>

#include "cli_instance.h"
#include "cli_internal.h"

/* ---- thread -> instance registry (#18) ------------------------------------
 *
 * The backend's printf retarget (_write) asks cli_current_instance() which
 * shell instance owns the running thread, so printf output follows the calling
 * terminal instead of a single global console.  The table is tiny
 * (CLI_THREAD_MAP_MAX entries) and is mutated only inside a short
 * interrupt-disable critical section; the reader scans inside the same
 * critical section, so no volatile / memory barrier is needed today.  (If a
 * future caller ever scans without the lock -- e.g. lock-free fast path -- the
 * publish-.sh-last / retract-.sh-first ordering below must be paired with the
 * appropriate volatile/barriers.)
 */
static struct cli_thread_map {
	TX_THREAD           *thread;
	struct cli_instance *sh;
} cli_thread_reg[CLI_THREAD_MAP_MAX];

/*
 * True when running in exception / ISR context (IPSR != 0).  tx_thread_identify()
 * cannot tell us this: the cortex_m7/gnu port leaves _tx_thread_current_ptr
 * pointing at the *interrupted* thread across an ISR, so without this guard a
 * printf issued from an ISR that preempted a shell thread would be misattributed
 * to that thread's terminal.  Read IPSR with the same MRS the ThreadX port uses
 * so this lean (ThreadX-only, host-excluded) TU needs no CMSIS header.
 */
static inline unsigned int cli_in_isr(void)
{
	unsigned int ipsr;
	__asm__ volatile("MRS %0, IPSR" : "=r"(ipsr));
	return ipsr;
}

int cli_register_thread(TX_THREAD *t, struct cli_instance *sh)
{
	TX_INTERRUPT_SAVE_AREA
	int i, slot = -1;

	if (t == NULL || sh == NULL)
		return -1;

	TX_DISABLE
	for (i = 0; i < CLI_THREAD_MAP_MAX; i++) {
		if (cli_thread_reg[i].sh == NULL) {
			slot = i;
			break;
		}
	}
	if (slot >= 0) {
		cli_thread_reg[slot].thread = t;
		cli_thread_reg[slot].sh     = sh;   /* publish last */
	}
	TX_RESTORE

	return slot >= 0 ? 0 : -1;   /* -1: table full -- caller must not continue */
}

void cli_unregister_thread(TX_THREAD *t)
{
	TX_INTERRUPT_SAVE_AREA
	int i;

	if (t == NULL)
		return;

	TX_DISABLE
	for (i = 0; i < CLI_THREAD_MAP_MAX; i++) {
		if (cli_thread_reg[i].thread == t) {
			cli_thread_reg[i].sh     = NULL;   /* retract first */
			cli_thread_reg[i].thread = NULL;
			break;
		}
	}
	TX_RESTORE
}

struct cli_instance *cli_current_instance(void)
{
	TX_INTERRUPT_SAVE_AREA
	struct cli_instance *result = NULL;
	TX_THREAD *t;
	int i;

	if (cli_in_isr())            /* ISR/exception: no owning terminal */
		return NULL;

	t = tx_thread_identify();    /* NULL before the scheduler starts */
	if (t == NULL)
		return NULL;

	TX_DISABLE
	for (i = 0; i < CLI_THREAD_MAP_MAX; i++) {
		if (cli_thread_reg[i].thread == t && cli_thread_reg[i].sh != NULL) {
			result = cli_thread_reg[i].sh;
			break;
		}
	}
	TX_RESTORE

	return result;
}

/*
 * Instance thread.  Enable the backend, show the prompt, then loop: wait for an
 * RX (or KILL) signal, drain every available byte and run the state machine.
 * The mutex is created for #5's locked output path and is intentionally not
 * taken here -- in #4 all output comes from this one thread.
 */
static void cli_thread_entry(ULONG arg)
{
	struct cli_instance  *sh = (struct cli_instance *)arg;
	struct cli_transport *tr = sh->tr;
	ULONG flags;

	/* If the backend cannot start RX, do not spin pretending to be live: let
	 * the thread fall through and exit, leaving the rest of the system running
	 * (req §9).  Other instances are unaffected. */
	if (tr->api->enable(tr) != 0) {
		if (tr->api->uninit)
			tr->api->uninit(tr);
		cli_unregister_thread(&sh->thread);   /* #18: drop the thread->instance map */
		return;
	}
	cli_edit_session_start(sh);   /* probe terminal width + draw the first prompt */

	for (;;) {
		if (tx_event_flags_get(&sh->events,
		                       CLI_EVT_RX | CLI_EVT_KILL | CLI_EVT_CONN,
		                       TX_OR_CLEAR, &flags, TX_WAIT_FOREVER)
		    != TX_SUCCESS)
			continue;

		if (flags & CLI_EVT_KILL)
			break;                  /* full stop/uninit lifecycle is future (§14) */

		/* A transport (re)connected (issue #49 P4: TCP backend posts CLI_EVT_CONN
		 * on accept).  Start a fresh session ON THIS THREAD so every editor-state
		 * mutation stays single-threaded against the byte loop below.  Order is
		 * deliberate: reset state -> backend session_begin (the TCP backend sets
		 * `connected`, enabling output) -> draw the prompt.  Thus the fresh prompt
		 * is the first thing the new client sees, and a previous command's output
		 * that drained after a reconnect was dropped while connected was false. */
		if (flags & CLI_EVT_CONN) {
			cli_session_reset_state(sh);
			if (tr->api->session_begin)
				tr->api->session_begin(tr);
			cli_edit_session_start(sh);
		}

		/* Read ONE byte at a time and feed it immediately.  A command line ends
		 * with '\r', which dispatches the handler synchronously from inside
		 * cli_input_byte(); bulk-reading the ring first would pull any following
		 * type-ahead (e.g. a Ctrl+C) out of the ring into a local buffer, hiding
		 * it from cli_cancel_poll() while the handler runs (issue #16).  Feeding
		 * one byte at a time keeps every not-yet-consumed byte in the ring, so a
		 * running command can still see a 0x03 that arrived right after its line. */
		uint8_t b;
		while (tr->api->read(tr, &b, 1) > 0)
			cli_input_byte(sh, b);
	}

	if (tr->api->uninit)
		tr->api->uninit(tr);
	cli_unregister_thread(&sh->thread);   /* #18: drop the thread->instance map */
}

int cli_init(struct cli_instance *sh)
{
	struct cli_transport *tr = sh->tr;

	/* One-shot: cli_init runs once per instance (a failed init is terminal for
	 * that instance and must not be retried -- see the create-order note below). */
	if (sh->state != CLI_UNINIT)
		return -1;

	/* Mandatory transport ops must be present. */
	if (tr == NULL || tr->api == NULL ||
	    tr->api->init == NULL || tr->api->enable == NULL ||
	    tr->api->write == NULL || tr->api->read == NULL) {
		sh->state = CLI_UNINIT;
		return -1;
	}

	tr->sh          = sh;
	sh->len         = 0;
	sh->cur         = 0;
	sh->line[0]     = '\0';
	sh->rx          = CLI_RX_NORMAL;
	sh->prev_cr     = 0;
	sh->esc_np      = 0;
	sh->esc_bad     = 0;
	sh->esc_p[0]    = 0;
	sh->esc_p[1]    = 0;
	sh->overwrite   = 0;
	sh->hist_used   = 0;        /* command history ring empty (issue #10) */
	sh->hist_nav_on = 0;
	sh->hist_nav    = 0;
	sh->bs_swap     = CLI_BACKSPACE_MODE;
	sh->term_width  = CLI_TERM_WIDTH;
	sh->old_rows    = 0;
	sh->draw_row    = 0;
	sh->probing_cpr = 0;
	sh->tab_list_armed = 0;     /* Tab completion two-stage flag (issue #11) */
	sh->last_result = 0;
	sh->rx_dropped  = 0;
	sh->out_len     = 0;
	sh->tx_failed   = 0;
	sh->tx_dropped  = 0;

	/*
	 * Bring up the backend BEFORE creating any ThreadX object.  The common
	 * failure (backend init) then needs no ThreadX teardown -- which matters
	 * because cli_init runs inside tx_application_define(), where the public
	 * tx_*_delete services return TX_CALLER_ERROR (system state != 0).  On the
	 * rarer event-flags/mutex create failures we therefore do NOT delete; the
	 * half-created object is harmless because the instance is never started.
	 */
	if (tr->api->init(tr) != 0) {
		sh->state = CLI_UNINIT;
		return -1;
	}

	if (tx_event_flags_create(&sh->events, "cli_evt") != TX_SUCCESS) {
		if (tr->api->uninit)
			tr->api->uninit(tr);
		sh->state = CLI_UNINIT;
		return -1;
	}

	/* TX_INHERIT: priority inheritance so a low-priority shell holding the TX
	 * lock (added in #5) cannot be preempted indefinitely by a mid thread. */
	if (tx_mutex_create(&sh->tx_lock, "cli_tx", TX_INHERIT) != TX_SUCCESS) {
		if (tr->api->uninit)
			tr->api->uninit(tr);
		sh->state = CLI_UNINIT;
		return -1;
	}

	sh->state = CLI_INITED;
	return 0;
}

int cli_start(struct cli_instance *sh)
{
	if (sh->state != CLI_INITED)
		return -1;

	/* Register the thread->instance mapping BEFORE creating the auto-started
	 * thread (#18): &sh->thread is a stable member address valid before
	 * tx_thread_create(), so a thread that begins running immediately always
	 * finds itself registered (no register-after-start race).  A full registry
	 * is a start failure -- printf must never silently misroute. */
	if (cli_register_thread(&sh->thread, sh) != 0)
		return -1;

	if (tx_thread_create(&sh->thread, "cli", cli_thread_entry, (ULONG)sh,
	                     sh->stack, sh->stack_size,
	                     CLI_INSTANCE_PRIORITY, CLI_INSTANCE_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		cli_unregister_thread(&sh->thread);   /* roll back the registration */
		return -1;
	}

	sh->state = CLI_STARTED;
	return 0;
}

void cli_transport_notify_rx(struct cli_instance *sh)
{
	/* ISR-safe: only sets an event flag (no lock, no suspend). */
	tx_event_flags_set(&sh->events, CLI_EVT_RX, TX_OR);
}

void cli_transport_notify_conn(struct cli_instance *sh)
{
	tx_event_flags_set(&sh->events, CLI_EVT_CONN, TX_OR);
}

void cli_transport_notify_tx(struct cli_instance *sh)
{
	tx_event_flags_set(&sh->events, CLI_EVT_TX, TX_OR);
}

/* Map a cli_config tick value to a ThreadX wait option (0 == wait forever). */
static ULONG cli_wait(unsigned ticks)
{
	return ticks == 0u ? TX_WAIT_FOREVER : (ULONG)ticks;
}

/*
 * Output lock (issue #5): the per-instance TX mutex guards a whole output call
 * (format + stage + flush) so concurrent writers to one instance never corrupt
 * out_buf/out_len.  TX_INHERIT (set in cli_init) bounds priority inversion while
 * the lock is held across a TX-space wait.
 *
 * Background jobs (issue #25): a bg-job worker instance (sh->fg != NULL) has no
 * tx_lock of its own; it locks its FOREGROUND's tx_lock so its output serialises
 * against the fg line editor (which also outputs under that mutex).  The mutex
 * is owner-reentrant, so a fg call that nests another lock is harmless.
 */
static struct cli_instance *cli_out_target(struct cli_instance *sh)
{
	return sh->fg ? sh->fg : sh;
}

int cli_lock(struct cli_instance *sh)
{
	struct cli_instance *o = cli_out_target(sh);
	return tx_mutex_get(&o->tx_lock, cli_wait(CLI_TX_MUTEX_WAIT)) == TX_SUCCESS
	       ? 0 : -1;
}

void cli_unlock(struct cli_instance *sh)
{
	tx_mutex_put(&cli_out_target(sh)->tx_lock);
}

/* Set for the duration of a raw binary transfer (issue #50); see cli_internal.h. */
volatile uint8_t cli_xfer_active;

/*
 * Console hand-over for a binary transfer (issue #50).  Take the output lock for
 * the WHOLE transfer (so a background job's output cannot interleave into the
 * YMODEM byte stream -- it blocks and, per #25, drops on its wedge deadline) and
 * raise cli_xfer_active so cli_tx_send_blocking stops draining RX and _write drops
 * printf output.  Returns 0 on success, -2 if called from a background job, -1 if
 * the lock could not be acquired.
 *
 * A bg-job worker (sh->fg != NULL) is REFUSED: the RX ring is a strict SPSC pipe
 * owned by the foreground thread, and the USART RX ISR posts CLI_EVT_RX to the
 * FOREGROUND's event group (u->sh == fg), not the worker's -- so a worker's
 * cli_read_byte() would never wake on an ACK while the foreground line editor
 * drains the bytes.  A binary transfer must run in the foreground (#50/#25).
 */
int cli_console_claim(struct cli_instance *sh)
{
	if (sh->fg != NULL)
		return -2;
	if (cli_lock(sh) != 0)
		return -1;
	cli_xfer_active = 1;
	return 0;
}

void cli_console_release(struct cli_instance *sh)
{
	cli_xfer_active = 0;
	cli_unlock(sh);
}

/* Discard any bytes buffered in the transport RX ring (issue #50): used before a
 * transfer (drop the rest of the command line / type-ahead) and after (drop a
 * trailing 'O'/'C' or other protocol tail) so the shell prompt resumes clean. */
void cli_rx_flush(struct cli_instance *sh)
{
	struct cli_transport *tr = sh->tr;
	uint8_t b;
	while (tr->api->read(tr, &b, 1) > 0)
		;
}

/*
 * Timed raw RX read for a binary transfer (issue #50).  Returns 0..255 on a
 * received byte, -1 on timeout (timeout_ms elapsed with none), -2 on kill
 * (CLI_EVT_KILL).  Unlike cli_cancel_poll()/cli_sleep() it does NOT interpret
 * 0x03 -- Ctrl+C is returned as the byte 3 like any other, so YMODEM control
 * bytes are never consumed by a cancel check.  timeout_ms == 0 polls once
 * (non-blocking).  Deliberately does NOT route through cli_wait() (which maps 0
 * to TX_WAIT_FOREVER).  Modelled on cli_sleep()'s deadline wait, but returns the
 * byte.  Thread-context only.
 */
int cli_read_byte(struct cli_instance *sh, unsigned timeout_ms)
{
	struct cli_transport *tr    = sh->tr;
	ULONG                 start = tx_time_get();
	uint8_t               b;

	for (;;) {
		ULONG elapsed, remaining, flags;
		UINT  st;

		/* Drain the ring first: the main loop's TX_OR_CLEAR may already have
		 * consumed the RX flag for a byte still sitting in the ring (#16 inv 2). */
		if (tr->api->read(tr, &b, 1) > 0)
			return (int)b;
		if (timeout_ms == 0u)
			return -1;                          /* non-blocking poll: empty */

		elapsed = (ULONG)(tx_time_get() - start);   /* wrap-safe */
		if (elapsed >= (ULONG)timeout_ms)
			return -1;                          /* timed out */
		remaining = (ULONG)timeout_ms - elapsed;

		st = tx_event_flags_get(&sh->events, CLI_EVT_RX | CLI_EVT_KILL,
		                        TX_OR_CLEAR, &flags, remaining);
		if (st != TX_SUCCESS)
			return -1;                          /* TX_NO_EVENTS == timed out */
		if (flags & CLI_EVT_KILL) {
			tx_event_flags_set(&sh->events, CLI_EVT_KILL, TX_OR); /* re-post */
			return -2;                          /* instance stopping */
		}
		/* CLI_EVT_RX: loop and drain the ring (or re-wait the remainder). */
	}
}

/*
 * Push len bytes to the transport, realising req §11's "blocking until sent"
 * semantics.  MUST be called with the output lock held (see cli_lock).  write()
 * is non-blocking and returns the count accepted; when the TX buffer is full we
 * suspend on CLI_EVT_TX (set by the backend when space frees) with a timeout.
 * On timeout the rest is dropped (drop stat + failure).  CLI_EVT_KILL aborts the
 * wait even with an infinite timeout, and is re-posted for the main loop.
 * Returns 0 if all bytes were sent, <0 on drop/error.
 */
int cli_tx_send_blocking(struct cli_instance *sh, const uint8_t *data, size_t len)
{
	struct cli_transport *tr = sh->tr;
	size_t sent = 0;
	int    is_job = (sh->fg != NULL);
	/* bg job: base of the no-progress deadline that bounds a wedged TX so a job
	 * never pins the shared fg->tx_lock forever (issue #25); reset on progress. */
	ULONG  stall_start = is_job ? tx_time_get() : 0u;

	while (sent < len) {
		/* Cooperative Ctrl+C (issue #16): once cancel is latched, stop emitting
		 * at once so a handler that keeps printing finishes fast.  Gated by
		 * dispatching so the post-cancel "^C"/prompt cleanup (dispatching == 0)
		 * is never suppressed.  For a bg job cancel_req is set by `kill` (#25). */
		if (sh->dispatching && sh->cancel_req)
			return -1;

		int n = tr->api->write(tr, data + sent, len - sent);
		if (n < 0) {
			sh->tx_failed = 1;
			return -1;
		}
		if (n > 0) {
			if ((size_t)n > len - sent)     /* defensive clamp */
				n = (int)(len - sent);
			sent += (size_t)n;
			if (is_job)
				stall_start = tx_time_get();    /* progress: reset deadline */
			continue;
		}

		if (is_job) {
			/* bg job, TX full: NEVER drain the shared RX (it belongs to the
			 * interactive consumer; cancel is kill-driven).  TX-space is signalled
			 * on the FOREGROUND's event group (the ISR notifies u->sh == fg), so a
			 * job cannot wait for it on its own group -- instead wait a short slice
			 * on its OWN events for a kill, then retry the write.  Bound the
			 * no-progress time by CLI_TX_TIMEOUT so a wedged TX drops rather than
			 * pinning fg->tx_lock (issue #25). */
			ULONG flags;
			/* Always FINITE for a bg job (issue #25): it holds the shared
			 * fg->tx_lock while sending, so it must not honour CLI_TX_TIMEOUT==0
			 * (never-drop) -- a wedged TX would pin the lock and freeze the
			 * foreground.  Use the configured timeout, or CLI_BG_TX_WEDGE_TICKS. */
			ULONG budget = CLI_TX_TIMEOUT ? (ULONG)CLI_TX_TIMEOUT
			                              : (ULONG)CLI_BG_TX_WEDGE_TICKS;

			if ((ULONG)(tx_time_get() - stall_start) >= budget) {
				sh->tx_dropped += (uint32_t)(len - sent);   /* wedged: drop */
				return -1;
			}
			/* Wake promptly on `kill` (its own group); otherwise time-slice and
			 * retry.  Clearing CLI_EVT_KILL is harmless: cancel_req (sticky) is the
			 * source of truth, caught by the loop top on the next iteration. */
			tx_event_flags_get(&sh->events, CLI_EVT_KILL, TX_OR_CLEAR,
			                   &flags, CLI_BG_TX_POLL_TICKS);
			continue;
		}

		/* Interactive TX full: wait for space (or a kill request).  While a
		 * command runs we also wake on RX so a Ctrl+C arriving mid-output aborts
		 * the blocked send.  The main loop's TX_OR_CLEAR may already have consumed
		 * the RX flag for a 0x03 still sitting in the ring, so poll the ring
		 * BEFORE waiting (issue #16 invariant 2). */
		ULONG flags;
		/* During a raw binary transfer (issue #50) the RX ring belongs to the
		 * YMODEM protocol, so do NOT wake on / drain it for a Ctrl+C here -- that
		 * would eat an ACK/'C'/NAK.  Wait only on TX space / kill. */
		int   raw  = cli_xfer_active;
		ULONG mask = CLI_EVT_TX | CLI_EVT_KILL |
		             ((sh->dispatching && !raw) ? CLI_EVT_RX : 0u);

		if (sh->dispatching && !raw && cli_cancel_poll(sh))
			return -1;

		if (tx_event_flags_get(&sh->events, mask, TX_OR_CLEAR, &flags,
		                       cli_wait(CLI_TX_TIMEOUT)) != TX_SUCCESS) {
			sh->tx_dropped += (uint32_t)(len - sent);   /* timeout */
			return -1;
		}
		if (flags & CLI_EVT_KILL) {
			tx_event_flags_set(&sh->events, CLI_EVT_KILL, TX_OR); /* re-post */
			sh->tx_dropped += (uint32_t)(len - sent);
			return -1;
		}
		if (!raw && (flags & CLI_EVT_RX) && cli_cancel_poll(sh))
			return -1;
		/* else CLI_EVT_TX (space freed) or a non-cancel RX byte: retry write */
	}
	return 0;
}

/*
 * Cancellable delay (issue #16): wait up to @p ticks ThreadX ticks, returning
 * early (non-zero) if Ctrl+C is seen or a stop is requested; 0 when the full
 * delay elapsed.  Unlike tx_thread_sleep() the wait is on the instance event
 * flags, so an RX byte (the ISR sets CLI_EVT_RX) wakes it and we drain the ring
 * for a 0x03.  Deadline-based with a wrap-safe elapsed so a non-cancel RX wake
 * neither shortens nor extends the delay.  Building block for watch/sleep (#21).
 */
int cli_sleep(struct cli_instance *sh, unsigned ticks)
{
	if (ticks == 0)
		return 0;                           /* contract: ticks==0 elapses at once */

	/* bg job (issue #25): cancel is kill-driven, so wait ONLY on its own
	 * CLI_EVT_KILL (set by `kill`); never wake on / drain the shared RX, which
	 * belongs to the interactive consumer.  Interactive: the existing RX-wake
	 * path so a buffered 0x03 (Ctrl+C) cancels the delay. */
	int   is_job = (sh->fg != NULL);
	ULONG mask   = is_job ? CLI_EVT_KILL : (CLI_EVT_RX | CLI_EVT_KILL);
	ULONG start  = tx_time_get();

	for (;;) {
		ULONG elapsed, remaining, flags;
		UINT  st;

		if (cli_cancel_poll(sh))            /* job-aware: a job returns cancel_req */
			return 1;

		elapsed = (ULONG)(tx_time_get() - start);   /* wrap-safe */
		if (elapsed >= (ULONG)ticks)
			return 0;                       /* full delay elapsed */
		remaining = (ULONG)ticks - elapsed;

		st = tx_event_flags_get(&sh->events, mask, TX_OR_CLEAR, &flags, remaining);
		if (st != TX_SUCCESS)
			return 0;                       /* TX_NO_EVENTS (timed out) == elapsed */
		if (flags & CLI_EVT_KILL) {
			if (!is_job)
				tx_event_flags_set(&sh->events, CLI_EVT_KILL, TX_OR); /* re-post (thread stop) */
			return 1;                       /* job: `kill`; interactive: stop */
		}
		if ((flags & CLI_EVT_RX) && cli_cancel_poll(sh))
			return 1;
		/* non-cancel RX: type-ahead drained, loop and re-wait the remainder */
	}
}
