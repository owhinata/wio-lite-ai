/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn.c
 * @brief   Backend-agnostic nn API dispatcher + inference timing (issue #9 P1).
 *
 * A thin layer over the compiled-in backend (nn_backend_vt_selected).  It adds the
 * two cross-backend concerns the backends must not each reimplement:
 *   - a singleton public `struct nn_model` wrapping the backend's opaque handle,
 *     opened race-free even though two consoles can ask for it at once,
 *   - DWT CYCCNT timing around nn_run() for nn_last_cycles().
 *
 * WHY THIS DOES NOT ENABLE THE CYCLE COUNTER.  The donor firmware's copy of this
 * file enables DWT itself: TRCENA, then the CoreSight lock key 0xC5ACCE55 to the
 * Lock Access Register, then CYCCNTENA.  Here svc/timebase.c owns the counter --
 * app/main.c calls timebase_init() before the scheduler starts -- and udelay() is a
 * busy-wait whose only exit is CYCCNT advancing (svc/timebase.c).  If the counter
 * were frozen, `usleep` would hang forever rather than return early, so a working
 * `usleep` is standing proof that CYCCNT runs on this part without the lock-key
 * write.  Re-enabling it here would add a third copy of that sequence for nothing.
 * (cmd_membench.c keeps its own copy deliberately: it is a pinned instrument and is
 * meant to be self-sufficient.)
 *
 * WHAT THIS FILE MUST NEVER DO IS WRITE DWT->CYCCNT.  It is the shared free-running
 * timebase udelay() reads, so zeroing it would cut short a concurrent `usleep N &`.
 * All measurement is therefore wrap-safe unsigned subtraction from a sampled value.
 * nn_init() only *verifies* the counter advances, so a broken DWT degrades to
 * "reports 0 cycles" instead of "reports a wrong number".
 *
 * Clean-room design; no third-party code reused.
 */
#include "nn.h"
#include "nn_backend.h"

#include "tx_api.h"          /* tx_thread_sleep (open/init serialization) */
#include "stm32h7xx_hal.h"   /* DWT / CoreDebug / __DSB / __ISB / PRIMASK */

/* Public model handle: backend impl + last inference cycle count. */
struct nn_model {
	void    *impl;
	uint32_t last_cycles;
	uint8_t  open;
};

static struct nn_model g_model;     /* the one model this build can run */
static uint8_t         g_inited;
static uint8_t         g_dwt_ok;    /* CYCCNT was observed advancing at init */

/* ---- DWT cycle counter -------------------------------------------------------
 *
 * Verify only -- see the file header for why we neither enable nor zero it.  A
 * short spin is enough: at 550 MHz the counter moves every ~1.8 ns, so any
 * observable delay proves it. */
static int nn_dwt_verify(void)
{
	volatile uint32_t spin = 64u;
	uint32_t a;

	if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk)
		return -1;                  /* CYCCNT not implemented on this core */

	a = DWT->CYCCNT;
	while (spin--)
		__NOP();

	return ((uint32_t)(DWT->CYCCNT - a) != 0u) ? 0 : -1;
}

/* ---- public API ------------------------------------------------------------ */

int nn_init(void)
{
	int rc;

	if (g_inited)
		return 0;

	/* Timing is best-effort: with the counter stopped, inference still works and
	 * only nn_last_cycles() goes to 0, so this is not fatal to init. */
	g_dwt_ok = (nn_dwt_verify() == 0) ? 1u : 0u;

	rc = nn_backend_vt_selected.init();
	if (rc != 0)
		return rc;

	g_inited = 1;
	return 0;
}

const struct nn_backend_info *nn_backend(void)
{
	return nn_backend_vt_selected.info;
}

static volatile int g_opening;      /* one thread owns the singleton open/init */

int nn_model_open(struct nn_model **out)
{
	int rc;

	if (!out)
		return -1;

	/* Serialize the one-time singleton open/init across the two consoles: claim an
	 * init latch under a brief PRIMASK critical section so exactly one thread runs
	 * backend init/open; the other waits for g_model.open to be published.  A
	 * pre-created mutex would just move the race into its own creation, and this
	 * way the already-open path stays lock-free. */
	for (;;) {
		uint32_t primask = __get_PRIMASK();
		int owner = 0;

		__disable_irq();
		if (g_model.open) {
			__set_PRIMASK(primask);
			*out = &g_model;
			return 0;
		}
		if (!g_opening) { g_opening = 1; owner = 1; }
		__set_PRIMASK(primask);

		if (owner)
			break;
		tx_thread_sleep(1);         /* another thread is opening; wait + retry */
	}

	/* This thread owns the init. */
	rc = nn_init();
	if (rc == 0)
		rc = nn_backend_vt_selected.open(&g_model.impl);
	if (rc == 0) {
		g_model.last_cycles = 0;
		g_model.open = 1;           /* publish (waiters spin on this) */
	}
	g_opening = 0;

	if (rc != 0)
		return rc;
	*out = &g_model;
	return 0;
}

void nn_model_close(struct nn_model *m)
{
	if (!m || !m->open)
		return;
	nn_backend_vt_selected.close(m->impl);
	m->impl = NULL;
	m->open = 0;
	m->last_cycles = 0;
}

const char *nn_model_name(const struct nn_model *m)
{
	if (!m || !m->open)
		return "(none)";
	return nn_backend_vt_selected.model_name(m->impl);
}

int nn_input_count(const struct nn_model *m)
{
	return (m && m->open) ? nn_backend_vt_selected.in_count(m->impl) : 0;
}

int nn_output_count(const struct nn_model *m)
{
	return (m && m->open) ? nn_backend_vt_selected.out_count(m->impl) : 0;
}

struct nn_tensor *nn_input(struct nn_model *m, int idx)
{
	return (m && m->open) ? nn_backend_vt_selected.input(m->impl, idx) : NULL;
}

struct nn_tensor *nn_output(struct nn_model *m, int idx)
{
	return (m && m->open) ? nn_backend_vt_selected.output(m->impl, idx) : NULL;
}

uint32_t nn_activations_bytes(const struct nn_model *m)
{
	return (m && m->open) ? nn_backend_vt_selected.activations_bytes(m->impl) : 0u;
}

int nn_run(struct nn_model *m)
{
	uint32_t c0, c1;
	int rc;

	if (!m || !m->open)
		return -1;

	/* DSB drains outstanding accesses and ISB flushes the pipeline, so the sampled
	 * counter brackets the work and nothing either side leaks across (PM0253
	 * sec 3.12.4 / 3.12.5) -- the same fencing cmd_membench.c uses. */
	__DSB();
	__ISB();
	c0 = DWT->CYCCNT;

	rc = nn_backend_vt_selected.run(m->impl);

	__DSB();
	__ISB();
	c1 = DWT->CYCCNT;

	/* Wrap-safe for one wrap only; nn_run()'s contract in nn.h bounds a single run
	 * well below that. */
	if (rc == 0)
		m->last_cycles = g_dwt_ok ? (uint32_t)(c1 - c0) : 0u;
	return rc;
}

uint32_t nn_last_cycles(const struct nn_model *m)
{
	return (m && m->open) ? m->last_cycles : 0u;
}

/* ---- runtime model swap (issue #9 phase 2c) --------------------------------
 *
 * These three wrap the OPTIONAL vtable entries, and the reason they are written out
 * rather than left to the caller is that a NULL function pointer is a landmine.  The
 * `null` backend implements none of them; a caller that reached through the vtable
 * itself would branch to address 0, which on this part is ITCM -- so it would not even
 * fault cleanly, it would execute whatever interrupt code lives there.  Here it is a
 * return value instead. */

int nn_model_load_region(void **buf, uint32_t *cap)
{
	if (!buf || !cap)
		return NN_ERR_ARG;
	if (!nn_backend_vt_selected.load_region)
		return NN_ERR_NOSUP;
	return nn_backend_vt_selected.load_region(buf, cap);
}

int nn_model_reload(const void *data, uint32_t len, const char *name)
{
	void *impl = NULL;
	int rc;

	if (!nn_backend_vt_selected.reload)
		return NN_ERR_NOSUP;
	/* The singleton must already exist: reload REPLACES a model, and the open path
	 * is what runs backend init and settles the race between the two consoles.  It
	 * also means g_model.open below is a transition, never a first publication. */
	if (!g_model.open)
		return NN_ERR_STATE;

	rc = nn_backend_vt_selected.reload(data, len, name, &impl);

	/* Adopt whatever the backend ended up with, success or not: the vtable contract
	 * is that *impl_out is the ACTIVE handle afterwards -- the new model on success,
	 * the restored previous one on a rejection, NULL only if even that failed.  In
	 * that last case clearing `open` is what lets a later nn_model_open() retry a
	 * fresh build instead of handing out a handle to nothing. */
	g_model.impl = impl;
	g_model.open = impl ? 1u : 0u;
	g_model.last_cycles = 0;   /* the previous timing measured a different model */
	return rc;
}

const char *nn_model_strerror(int rc)
{
	switch (rc) {
	case 0:                    return "ok";
	case NN_MODEL_ERR_VERSION: return "schema version this runtime cannot read";
	case NN_MODEL_ERR_OPS:     return "operator resolver could not be built";
	case NN_MODEL_ERR_ARENA:   return "activations do not fit the arena, or the model "
	                                  "uses an operator this build did not register";
	case NN_MODEL_ERR_TENSOR:  return "a tensor has no buffer or too many dimensions";
	case NN_MODEL_ERR_SHAPE:   return "input/output tensor count out of range";
	case NN_MODEL_ERR_EMPTY:   return "empty or impossibly short model";
	case NN_MODEL_ERR_FORMAT:  return "not a valid model for this runtime";
	case NN_MODEL_ERR_SLOT:    return "buffer is not the staging region handed out";
	case NN_ERR_ARG:           return "bad argument";
	case NN_ERR_NOSUP:         return "this backend cannot load a model at run time";
	case NN_ERR_STATE:         return "no model session is open";
	default:                   return "unknown error";
	}
}

int nn_heap_allocs(uint32_t *out)
{
	if (!out)
		return NN_ERR_ARG;
	if (!nn_backend_vt_selected.heap_allocs)
		return NN_ERR_NOSUP;
	*out = nn_backend_vt_selected.heap_allocs();
	return 0;
}

/* ---- single-session guard -------------------------------------------------- */

/* A plain flag test-set under a brief PRIMASK critical section: interrupt-safe,
 * thread-agnostic (acquire and release may run on different threads), and needs no
 * one-time init, which would itself race between the two consoles.  PRIMASK is also
 * what this firmware's ThreadX port uses for its own critical sections
 * (TX_PORT_USE_BASEPRI is not defined), so this nests consistently with it. */
static volatile int nn_session_busy;

int nn_session_try_acquire(void)
{
	uint32_t primask = __get_PRIMASK();
	int ok;

	__disable_irq();
	if (!nn_session_busy) { nn_session_busy = 1; ok = 1; } else { ok = 0; }
	__set_PRIMASK(primask);
	return ok ? 0 : -1;
}

void nn_session_release(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	nn_session_busy = 0;
	__set_PRIMASK(primask);
}
