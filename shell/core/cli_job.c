/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_job.c
 * @brief   Background-job pool: `cmd &` worker threads + jobs/kill (issue #25).
 *
 * A trailing '&' on a command segment runs it in a background WORKER thread drawn
 * from a fixed static pool (CLI_MAX_BG_JOBS), so the interactive prompt returns
 * immediately.  Each worker is a full `struct cli_instance` (so every registered
 * command runs in it unchanged) wired as a "job" of its launching foreground
 * shell: inst.fg = fg and inst.tr = fg->tr.  That makes the worker's output --
 * cli_print AND printf -- serialise on the foreground's tx_lock and route to the
 * foreground's terminal (cli_out_begin / cli_current_instance, #18/#25), while
 * cancellation is kill-driven (the worker never drains the shared RX, which is a
 * single-consumer SPSC ring owned by the interactive thread).
 *
 * Lifecycle.  A worker's entry runs the one segment through the normal dispatch
 * path and returns, so ThreadX moves it to TX_COMPLETED -- a thread cannot delete
 * itself.  The FOREGROUND thread reaps lazily (cli_jobs_reap, called on every
 * line and by jobs/kill): it tx_thread_delete()s completed workers and prints
 * their "[id] Done" notice from foreground context, so the notice serialises with
 * the prompt with no cross-thread redraw.  Reaping keys strictly on the ThreadX
 * thread state (read under TX_DISABLE), never a self-set flag, so a worker still
 * executing its return epilogue is never deleted out from under itself.
 *
 * This is the only #25 file that calls ThreadX (tx_*); the parse seam
 * (cli_segment_is_background) and the cancel/dispatch seams it relies on are
 * ThreadX-free and host-tested.  Linked into the threadx executable only -- the
 * host harness stubs cli_job_launch / cli_jobs_reap (see host_glue.c).
 *
 * Clean-room design; no third-party code reused.
 */
#include <stddef.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "tx_api.h"   /* tx_thread_*, tx_event_flags_*, TX_COMPLETED/TX_TERMINATED, TX_DISABLE */

/* Captured command text shown by `jobs` / the done notice.  The worker's line[]
 * is rewritten in place by cli_parse() during dispatch, so the label is a
 * separate snapshot taken at launch (truncated; just for display). */
#ifndef CLI_JOB_LABEL_MAX
#define CLI_JOB_LABEL_MAX 40
#endif

/*
 * Slot state.  The CLAIM transitions out of FREE / RUNNING are made under
 * TX_DISABLE so the shared table is safe even when several reapers / launchers
 * run at once (multiple foreground instances, or a `jobs &` reaping from a worker
 * thread); the completion transitions are then done by that single claiming owner
 * outside the critical section:
 *   FREE      -> LAUNCHING : reserved by cli_job_launch        (under TX_DISABLE)
 *   LAUNCHING -> RUNNING   : worker thread created             (sole owner)
 *   LAUNCHING -> FREE      : launch failed before/at create    (sole owner)
 *   RUNNING   -> REAPING   : a reaper claimed a COMPLETED worker(under TX_DISABLE)
 *   REAPING   -> FREE      : claimer finished delete + notice  (sole owner)
 * LAUNCHING is needed because a reserved-but-not-yet-created slot's TX_THREAD
 * still holds a previous tenant's (possibly TX_COMPLETED) state; REAPING makes
 * the "I will delete this one" decision exclusive to a single reaper.
 */
enum cli_job_state {
	CLI_JOB_FREE = 0,
	CLI_JOB_LAUNCHING,
	CLI_JOB_RUNNING,
	CLI_JOB_REAPING,
};

struct cli_job {
	enum cli_job_state  state;
	unsigned long       id;                       /* monotonic; 0 when free */
	uint8_t             evt_ok;                    /* event group created (slot usable) */
	char                label[CLI_JOB_LABEL_MAX];
	struct cli_instance inst;                     /* the worker instance */
};

static struct cli_job  cli_jobs[CLI_MAX_BG_JOBS];
/* Worker stacks kept out of the job struct so each stays 8-byte aligned (ThreadX
 * requires it); CLI_BG_JOB_STACK_SIZE is a multiple of 8 so every row aligns. */
static UCHAR cli_job_stacks[CLI_MAX_BG_JOBS][CLI_BG_JOB_STACK_SIZE]
	__attribute__((aligned(8)));
static unsigned long cli_job_next_id = 1;          /* never reuses an id (#25) */

/* ---- worker instance reset (NOT cli_init: keep events, never touch tr->sh) -- *
 * cli_init() would set tr->sh = inst and re-init the backend, corrupting the
 * shared transport's ISR notify target (u->sh must stay the foreground).  So a
 * worker is reset field-by-field here: it shares fg->tx_lock (no own tx_lock) and
 * fg->tr (output alias), and reuses its own events group (created once at boot,
 * stale flags cleared on each launch). */
static void cli_job_inst_reset(struct cli_instance *inst, struct cli_instance *fg,
                               UCHAR *stack, const char *seg)
{
	inst->fg          = fg;
	inst->tr          = fg->tr;          /* output alias only; tr->sh stays = fg */
	inst->stack       = stack;
	inst->stack_size  = CLI_BG_JOB_STACK_SIZE;

	/* Line buffer holds the (already '&'-stripped) segment for cli_parse(). */
	strncpy(inst->line, seg, CLI_CMD_BUFFER_SIZE - 1);
	inst->line[CLI_CMD_BUFFER_SIZE - 1] = '\0';

	/* Runtime state a handler / the output path touches (mirrors cli_init). */
	inst->len = 0;
	inst->cur = 0;
	inst->overwrite = 0;
	inst->rx = CLI_RX_NORMAL;
	inst->prev_cr = 0;
	inst->esc_np = 0;
	inst->esc_bad = 0;
	inst->esc_p[0] = 0;
	inst->esc_p[1] = 0;
	inst->hist_used = 0;
	inst->hist_nav_on = 0;
	inst->hist_nav = 0;
	inst->bs_swap = 0;
	inst->term_width = CLI_TERM_WIDTH;
	inst->old_rows = 0;
	inst->draw_row = 0;
	inst->probing_cpr = 0;
	inst->tab_list_armed = 0;
	inst->render_dirty = 0;
	inst->prompt[0] = '\0';              /* a worker has no prompt */
	inst->out_len = 0;
	inst->tx_failed = 0;
	inst->dispatching = 0;
	inst->cancel_req = 0;
	inst->last_result = 0;
	inst->rx_dropped = 0;
	inst->tx_dropped = 0;
	inst->state = CLI_STARTED;

	/* Flush any event flag left set by a previous tenant of this slot. */
	tx_event_flags_set(&inst->events, 0, TX_AND);
}

/* ---- worker entry ------------------------------------------------------- */

/*
 * Run the single backgrounded segment through the normal parse/dispatch path,
 * then unregister (so a tail printf still routed correctly up to here) and
 * return -> the thread becomes TX_COMPLETED for the foreground reaper to delete.
 */
static void cli_job_entry(ULONG arg)
{
	struct cli_instance *inst = (struct cli_instance *)arg;

	cli_dispatch_segment(inst, inst->line);
	cli_unregister_thread(&inst->thread);
}

/* ---- pool init ---------------------------------------------------------- */

void cli_job_pool_init(void)
{
	int i;

	for (i = 0; i < CLI_MAX_BG_JOBS; i++) {
		cli_jobs[i].state = CLI_JOB_FREE;
		cli_jobs[i].id    = 0;
		/* One event-flags group per worker, created once and reused (kill wakeup
		 * + cancellable sleep / TX-full poll).  Mark the slot usable only if the
		 * group was created: without it cli_sleep()/kill on a worker would hit
		 * TX_GROUP_ERROR, so cli_job_launch() refuses to use a failed slot. */
		cli_jobs[i].evt_ok =
			(tx_event_flags_create(&cli_jobs[i].inst.events, "cli_job_evt")
			 == TX_SUCCESS) ? 1u : 0u;
	}
}

/* ---- reaping ------------------------------------------------------------ */

void cli_jobs_reap(struct cli_instance *fg)
{
	TX_INTERRUPT_SAVE_AREA
	int i;

	for (i = 0; i < CLI_MAX_BG_JOBS; i++) {
		struct cli_job *j = &cli_jobs[i];
		int claim = 0;

		/* Reap STRICTLY on the ThreadX thread state (read under TX_DISABLE), not a
		 * self-set flag: a worker still in its return epilogue is not yet
		 * COMPLETED.  Atomically claim a completed slot (RUNNING -> REAPING) so
		 * that with several reapers on the shared table (multiple fg instances, or
		 * `jobs &`) EXACTLY ONE deletes + frees it; the losers see REAPING and skip. */
		TX_DISABLE
		if (j->state == CLI_JOB_RUNNING) {
			UINT st = j->inst.thread.tx_thread_state;
			if (st == TX_COMPLETED || st == TX_TERMINATED) {
				j->state = CLI_JOB_REAPING;
				claim = 1;
			}
		}
		TX_RESTORE

		if (!claim)
			continue;

		cli_unregister_thread(&j->inst.thread);   /* defensive (worker already did) */
		tx_thread_delete(&j->inst.thread);        /* legal: COMPLETED/TERMINATED */

		cli_print(fg, "[%lu] Done       %s\r\n", j->id, j->label);
		j->id    = 0;
		j->state = CLI_JOB_FREE;                  /* single writer: the claimer */
	}
}

/* ---- launch ------------------------------------------------------------- */

/* True when @p seg is empty or only whitespace (a lone '&' strips to ""). */
static int cli_seg_blank(const char *seg)
{
	for (; *seg; seg++)
		if (*seg != ' ' && *seg != '\t')
			return 0;
	return 1;
}

int cli_job_launch(struct cli_instance *fg, char *seg)
{
	TX_INTERRUPT_SAVE_AREA
	int i, slot = -1;
	unsigned long id;
	struct cli_job *j;

	/* Free up any completed workers first so their slots can be reused. */
	cli_jobs_reap(fg);

	/* "&" with nothing before it: a dispatch no-op, do not spawn a thread. */
	if (cli_seg_blank(seg))
		return 0;

	/* Claim a free slot + assign a fresh monotonic id atomically (a bg `jobs &`
	 * could in principle race the foreground here). */
	TX_DISABLE
	for (i = 0; i < CLI_MAX_BG_JOBS; i++) {
		if (cli_jobs[i].state == CLI_JOB_FREE && cli_jobs[i].evt_ok) {
			slot = i;
			cli_jobs[i].state = CLI_JOB_LAUNCHING;  /* reserve; not yet reapable */
			break;
		}
	}
	id = cli_job_next_id;
	if (slot >= 0)
		cli_job_next_id++;
	TX_RESTORE

	if (slot < 0) {
		cli_error(fg, "too many background jobs (max %u)\r\n",
		          (unsigned)CLI_MAX_BG_JOBS);
		return -1;
	}
	j = &cli_jobs[slot];
	j->id = id;
	strncpy(j->label, seg, CLI_JOB_LABEL_MAX - 1);
	j->label[CLI_JOB_LABEL_MAX - 1] = '\0';

	cli_job_inst_reset(&j->inst, fg, cli_job_stacks[slot], seg);

	/* Register the worker thread BEFORE creating it (auto-start), so printf from a
	 * thread that begins running at once always resolves its instance (#18). */
	if (cli_register_thread(&j->inst.thread, &j->inst) != 0) {
		j->state = CLI_JOB_FREE;
		j->id    = 0;
		cli_error(fg, "background job table full\r\n");
		return -1;
	}

	if (tx_thread_create(&j->inst.thread, "cli-bg", cli_job_entry,
	                     (ULONG)&j->inst, j->inst.stack, j->inst.stack_size,
	                     CLI_BG_JOB_PRIORITY, CLI_BG_JOB_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		cli_unregister_thread(&j->inst.thread);
		j->state = CLI_JOB_FREE;
		j->id    = 0;
		cli_error(fg, "failed to start background job\r\n");
		return -1;
	}

	j->state = CLI_JOB_RUNNING;   /* now reapable: the worker thread exists */
	cli_print(fg, "[%lu] %s\r\n", j->id, j->label);
	return 0;
}

/* ---- jobs / kill -------------------------------------------------------- */

void cli_jobs_print(struct cli_instance *sh)
{
	int i, n = 0;

	cli_jobs_reap(sh);   /* announce + free anything that finished */

	for (i = 0; i < CLI_MAX_BG_JOBS; i++) {
		if (cli_jobs[i].state != CLI_JOB_RUNNING)
			continue;
		cli_print(sh, "[%lu] Running    %s\r\n", cli_jobs[i].id, cli_jobs[i].label);
		n++;
	}
	if (n == 0)
		cli_print(sh, "no background jobs\r\n");
}

int cli_job_kill(struct cli_instance *sh, unsigned long id)
{
	TX_INTERRUPT_SAVE_AREA
	int i, found = -1;

	(void)sh;
	for (i = 0; i < CLI_MAX_BG_JOBS; i++) {
		struct cli_job *j = &cli_jobs[i];

		/* Latch the cancel atomically vs reap's RUNNING->REAPING claim and a
		 * subsequent slot reuse, so a kill never sets a flag on a job being torn
		 * down or on a different job that reused the slot (the id check + latch are
		 * one critical section).  Cooperative (issue #16/#25): a worker polling
		 * cli_cancel_requested() / in cli_sleep() stops; a non-cooperative handler
		 * (e.g. coremark) ignores it and runs to the end.  tx_event_flags_set is
		 * ISR-safe, so calling it under TX_DISABLE only defers the wakeup switch. */
		TX_DISABLE
		if (j->state == CLI_JOB_RUNNING && j->id == id) {
			j->inst.cancel_req = 1;
			tx_event_flags_set(&j->inst.events, CLI_EVT_KILL, TX_OR);
			found = 0;
		}
		TX_RESTORE

		if (found == 0)
			break;
	}
	return found;   /* -1: no running job with that id */
}
