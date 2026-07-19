/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    malloc_lock.c
 * @brief   Thread-safe newlib heap: back __malloc_lock/__malloc_unlock with a
 *          ThreadX mutex (issue #14).
 *
 * The stock newlib-nano __malloc_lock/__malloc_unlock forward to the retargetable
 * lock hooks (__retarget_lock_acquire/release_recursive), which are no-op `bx lr`
 * stubs on this build -- so malloc/free/realloc/mallinfo (and printf's %f float
 * conversion, which allocates a scratch buffer) are NOT thread-safe.  Once more than
 * one heap user runs in different ThreadX threads (membench, coremark, background
 * jobs, per-thread printf), concurrent allocations race the arena and corrupt it.
 *
 * Provide strong __malloc_lock/__malloc_unlock so the linker resolves malloc's
 * reference here instead of pulling libc's mlock.o, and serialise every heap
 * operation on a single priority-inheriting mutex.
 *
 * POLICY: the newlib heap must NOT be used from ISR or ThreadX timer-callback
 * context.  The guard below only SKIPS the lock there (a mutex cannot be taken from
 * an ISR); it does NOT make an ISR malloc safe.  Today no ISR touches the heap
 * (OTG_HS -> tud_int_handler, SysTick -> HAL_IncTick + gated tx timer, the fault path
 * logs to the DTCM ring), so this holds.
 *
 * Clean-room glue.
 */
#include "tx_api.h"
#include "app.h"

/* newlib passes a struct _reent * we ignore; forward-declare to avoid <reent.h>. */
struct _reent;

static TX_MUTEX      g_heap_mutex;
static volatile UINT g_heap_lock_ready;

void malloc_lock_init(void)
{
	if (tx_mutex_create(&g_heap_mutex, "heap", TX_INHERIT) == TX_SUCCESS)
		g_heap_lock_ready = 1u;
}

/* True only in genuine post-scheduler THREAD context.  IPSR==0 excludes ISR/exception
 * context: the Cortex-M7/GNU port leaves _tx_thread_current_ptr pointing at the
 * interrupted thread during an ISR, so tx_thread_identify() alone cannot detect it.
 * tx_thread_identify()!=TX_NULL then excludes the pre-scheduler init context
 * (tx_application_define), where tx_mutex_get() would return TX_CALLER_ERROR.  Before
 * the scheduler runs everything is single-threaded, so skipping the lock is safe. */
static inline int heap_lock_usable(void)
{
	unsigned int ipsr;

	__asm__ volatile("MRS %0, IPSR" : "=r"(ipsr));
	return (g_heap_lock_ready != 0u) && (ipsr == 0u) &&
	       (tx_thread_identify() != TX_NULL);
}

void __malloc_lock(struct _reent *r)
{
	(void)r;
	if (heap_lock_usable())
		(void)tx_mutex_get(&g_heap_mutex, TX_WAIT_FOREVER);
}

void __malloc_unlock(struct _reent *r)
{
	(void)r;
	if (heap_lock_usable())
		(void)tx_mutex_put(&g_heap_mutex);
}
