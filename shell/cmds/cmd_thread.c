/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_thread.c
 * @brief   `thread` built-in shell command (issue #13): one combined table of
 *          every ThreadX thread -- state / priority / run count + stack usage.
 *
 * Joins help/echo (cmd_builtin.c) and version/uptime/reboot (cmd_system.c) in the
 * `shell` executable only -- never linked into the host test harness.  It reads
 * board state through the standard buffered output API and touches only the shell
 * instance passed to it, so it stays reentrant across instances (req §10).
 *
 * Enumeration walks ThreadX's created-thread list (_tx_thread_created_ptr /
 * _tx_thread_created_count -- a circular doubly-linked list).  This firmware creates
 * every thread once in tx_application_define() and never deletes one, so the list is
 * static; we snapshot head+count under TX_DISABLE/TX_RESTORE, then walk it with
 * interrupts back on (cli_print waits on a mutex and must not run inside a critical
 * section).  Only those two internal globals are declared here; every per-thread
 * field we read lives in the public TX_THREAD typedef (tx_api.h), so the internal
 * tx_thread.h is not pulled in.
 *
 * Stack peak (high-water) usage is computed by scanning the 0xEF fill ThreadX lays
 * down at create time (tx_thread_create.c, TX_STACK_FILL == 0xEFEFEFEF -- present by
 * default unless TX_DISABLE_STACK_FILLING is defined, which this target does not).
 * This is the same best-effort method as ThreadX's own tx_thread_stack_analyze():
 * a used word can legitimately equal 0xEF, so peak may read a few bytes low.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"

#include "tx_api.h"          /* TX_THREAD, tx_thread_state defines, TX_DISABLE/RESTORE */
#ifdef TX_EXECUTION_PROFILE_ENABLE
#include "tx_execution_profile.h"  /* EXECUTION_TIME, _tx_execution_*_time_get (issue #19) */
#endif

#include <stdint.h>
#include <stdio.h>           /* snprintf for the cpu% field */

/* stack_peak_used() reads the 0xEF fill ThreadX applies by default.  If a build
 * ever turns that off, the stack columns would be meaningless -- fail loudly here
 * rather than silently print wrong numbers.  (Checked after tx_api.h so the port's
 * macro state is final; never fires for the shell target, which does not set it.) */
#ifdef TX_DISABLE_STACK_FILLING
# error "thread command needs the ThreadX stack fill; do not build the shell with TX_DISABLE_STACK_FILLING"
#endif

/* ThreadX created-thread list head + count (internal globals, declared in the
 * private tx_thread.h).  Declare just the two we need rather than including it. */
extern TX_THREAD *_tx_thread_created_ptr;
extern ULONG      _tx_thread_created_count;

/* tx_thread_state (tx_api.h §0..14) -> short label; index is the state value. */
static const char *state_name(UINT s)
{
	static const char *const names[] = {
		"ready",   /*  0 TX_READY          */
		"compl",   /*  1 TX_COMPLETED      */
		"term",    /*  2 TX_TERMINATED     */
		"susp",    /*  3 TX_SUSPENDED      */
		"sleep",   /*  4 TX_SLEEP          */
		"queue",   /*  5 TX_QUEUE_SUSP     */
		"sem",     /*  6 TX_SEMAPHORE_SUSP */
		"event",   /*  7 TX_EVENT_FLAG     */
		"block",   /*  8 TX_BLOCK_MEMORY   */
		"byte",    /*  9 TX_BYTE_MEMORY    */
		"io",      /* 10 TX_IO_DRIVER      */
		"file",    /* 11 TX_FILE           */
		"tcpip",   /* 12 TX_TCP_IP         */
		"mutex",   /* 13 TX_MUTEX_SUSP     */
		"pchg",    /* 14 TX_PRIORITY_CHANGE*/
	};
	return (s < (sizeof names / sizeof names[0])) ? names[s] : "?";
}

/*
 * Peak (high-water) stack usage in bytes.  The stack grows down from
 * tx_thread_stack_end (high) toward tx_thread_stack_start (low), so the untouched
 * tail keeps its 0xEF fill at the low end.  The leading 0xEF run from stack_start
 * is the free headroom; peak = size - free.  (Same scan as _tx_thread_stack_analyze.)
 */
static ULONG stack_peak_used(const TX_THREAD *t)
{
	const UCHAR *base = (const UCHAR *)t->tx_thread_stack_start;
	ULONG size  = t->tx_thread_stack_size;
	ULONG freeb = 0;

	while (freeb < size && base[freeb] == (UCHAR)0xEF)
		freeb++;
	return size - freeb;
}

#ifdef TX_EXECUTION_PROFILE_ENABLE
/*
 * "top"-style cpu% (issue #19).  The ThreadX Execution Profile Kit accumulates,
 * per thread and globally, busy time in TIM2 ticks (see tx_user.h / svc/timebase.c).  We
 * show each thread's share of the *window since the previous `thread` run*:
 * cpu% = delta_thread / window, where window = delta(all threads + isr + idle).
 * The previous snapshot lives here (threads are static and never deleted, so a
 * small pointer-keyed table suffices); the first run has no prior and prints "--".
 */
#define THREAD_CPU_PREV_MAX 16
struct cpu_snap { TX_THREAD *t; EXECUTION_TIME prev; };
static struct cpu_snap cpu_prev[THREAD_CPU_PREV_MAX];
static UINT           cpu_prev_count;
static EXECUTION_TIME cpu_prev_thread, cpu_prev_isr, cpu_prev_idle;
static int            cpu_have_prev;

static EXECUTION_TIME cpu_prev_lookup(const TX_THREAD *t)
{
	UINT i;
	for (i = 0; i < cpu_prev_count; i++)
		if (cpu_prev[i].t == t)
			return cpu_prev[i].prev;
	return 0;   /* unseen thread -> baseline 0 (its first window may read high once) */
}

/* Render delta/window as "NN.N%" clamped to 0..100.0, or "--" when there is no
 * prior window (have == 0) or the window is empty.  delta is pre-clamped <= window. */
static void cpu_fmt(char *buf, size_t n, int have, EXECUTION_TIME delta, EXECUTION_TIME window)
{
	ULONG tenths;

	if (!have || window == 0) {
		buf[0] = '-'; buf[1] = '-'; buf[2] = '\0';
		return;
	}
	tenths = (ULONG)((delta * 1000ULL) / window);
	if (tenths > 1000u)
		tenths = 1000u;   /* clamp (non-atomic 64-bit reads / quantization) */
	snprintf(buf, n, "%lu.%lu%%", tenths / 10u, tenths % 10u);
}
#endif /* TX_EXECUTION_PROFILE_ENABLE */

static int cmd_thread(struct cli_instance *sh, int argc, char **argv)
{
	TX_INTERRUPT_SAVE_AREA

	TX_THREAD *t;
	ULONG count, i;
#ifdef TX_EXECUTION_PROFILE_ENABLE
	EXECUTION_TIME tt = 0, it = 0, id = 0, window;
	struct cpu_snap next_prev[THREAD_CPU_PREV_MAX];
	UINT next_count = 0;
	int  have = cpu_have_prev;
	char cpubuf[8];
#endif

	(void)argc;
	(void)argv;

	/* Atomically snapshot the created-list head + count (and, for cpu%, the
	 * global busy totals -- 64-bit, so read inside the same critical section for
	 * a coherent window).  The nodes are static (nothing is ever deleted), so we
	 * walk the list afterwards with interrupts on -- cli_print waits on a mutex
	 * and must not run in a critical section. */
	TX_DISABLE
	t     = _tx_thread_created_ptr;
	count = _tx_thread_created_count;
#ifdef TX_EXECUTION_PROFILE_ENABLE
	_tx_execution_thread_total_time_get(&tt);
	_tx_execution_isr_time_get(&it);
	_tx_execution_idle_time_get(&id);
#endif
	TX_RESTORE

#ifdef TX_EXECUTION_PROFILE_ENABLE
	window = (tt - cpu_prev_thread) + (it - cpu_prev_isr) + (id - cpu_prev_idle);
	cli_print(sh, "%-20s %-6s %3s %6s %6s %5s %5s %6s\r\n",
	          "name", "state", "pri", "runs", "size", "peak", "use%", "cpu%");
#else
	cli_print(sh, "%-20s %-6s %3s %6s %6s %5s %5s %4s\r\n",
	          "name", "state", "pri", "runs", "size", "peak", "free", "use%");
#endif

	if (t == TX_NULL || count == 0) {
		cli_print(sh, "(no threads)\r\n");
		return 0;
	}

	for (i = 0; i < count; i++, t = t->tx_thread_created_next) {
		/* Ctrl+C between rows: stop before emitting the next one (issue #16).
		 * Outside the TX_DISABLE region below -- cli_cancel_requested() drains the
		 * transport and must not run with interrupts disabled.  The dispatcher
		 * detects cancel_req and prints "^C", so just return. */
		if (cli_cancel_requested(sh))
			return 0;

		ULONG size  = t->tx_thread_stack_size;
		ULONG peak  = stack_peak_used(t);
		ULONG pct   = size ? (peak * 100u) / size : 0u;
		const char *name = t->tx_thread_name ? t->tx_thread_name : "(unnamed)";
#ifdef TX_EXECUTION_PROFILE_ENABLE
		EXECUTION_TIME cur, prev, delta;

		/* Read the 64-bit per-thread total atomically (torn reads could spike a
		 * huge positive value that would also poison the snapshot). */
		TX_DISABLE
		cur = t->tx_thread_execution_time_total;
		TX_RESTORE

		prev  = cpu_prev_lookup(t);
		delta = (have && cur >= prev) ? (cur - prev) : 0;   /* clamp negative */
		cpu_fmt(cpubuf, sizeof cpubuf, have, delta, window);

		if (next_count < THREAD_CPU_PREV_MAX) {
			next_prev[next_count].t    = t;
			next_prev[next_count].prev = cur;
			next_count++;
		}

		cli_print(sh, "%-20s %-6s %3u %6lu %6lu %5lu %4lu%% %6s\r\n",
		          name, state_name(t->tx_thread_state),
		          t->tx_thread_priority,
		          (unsigned long)t->tx_thread_run_count,
		          (unsigned long)size, (unsigned long)peak,
		          (unsigned long)pct, cpubuf);
#else
		ULONG freeb = size - peak;

		cli_print(sh, "%-20s %-6s %3u %6lu %6lu %5lu %5lu %3lu%%\r\n",
		          name, state_name(t->tx_thread_state),
		          t->tx_thread_priority,
		          (unsigned long)t->tx_thread_run_count,
		          (unsigned long)size, (unsigned long)peak,
		          (unsigned long)freeb, (unsigned long)pct);
#endif
	}

#ifdef TX_EXECUTION_PROFILE_ENABLE
	/* Pseudo-rows so the cpu% column sums to ~100%: idle headroom and ISR load. */
	cpu_fmt(cpubuf, sizeof cpubuf, have, id - cpu_prev_idle, window);
	cli_print(sh, "%-20s %-6s %3s %6s %6s %5s %5s %6s\r\n",
	          "(idle)", "", "", "", "", "", "", cpubuf);
	cpu_fmt(cpubuf, sizeof cpubuf, have, it - cpu_prev_isr, window);
	cli_print(sh, "%-20s %-6s %3s %6s %6s %5s %5s %6s\r\n",
	          "(isr)", "", "", "", "", "", "", cpubuf);

	/* Commit this snapshot as the baseline for the next `thread`. */
	for (i = 0; i < next_count; i++)
		cpu_prev[i] = next_prev[i];
	cpu_prev_count  = next_count;
	cpu_prev_thread = tt;
	cpu_prev_isr    = it;
	cpu_prev_idle   = id;
	cpu_have_prev   = 1;
#endif

	return 0;
}

CLI_CMD_REGISTER(thread, NULL, "list threads + stack usage", cmd_thread, 1, 0);
