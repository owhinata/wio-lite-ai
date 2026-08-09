/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_band.c
 * @brief   Refcounted fan-out for the camera's band stream (issue #9 phase 3).
 *          See cam_band.h for why this exists and what it promises.
 */
#include "cam_band.h"

#include "camera.h"
#include "stm32h7xx.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */
#include "tx_api.h"

#include <stddef.h>

#define LOG_TAG "camband"
#include "log.h"

/*
 * How long cam_band_release() waits for an in-flight callback to return, in ticks
 * (1 ms each, TX_TIMER_TICKS_PER_SECOND = 1000).  A callback is one band's work --
 * the preview's transpose measures ~1.3 ms and the NN's ingest well under 1 ms -- so
 * 200 ms is two orders of margin while still being bounded.  Deliberately far below
 * camera_stream_stop()'s own 1000 ms, because this wait happens FIRST and the two
 * are serial.
 */
#define CAM_BAND_DRAIN_TICKS 200

/*
 * How long claim/release wait for each other, in ticks.  A release can spend
 * CAM_BAND_DRAIN_TICKS draining plus camera_stream_stop()'s own 1000 ms, so this has
 * to exceed their sum with room to spare.
 */
#define CAM_BAND_LOCK_TICKS 2000

static volatile uint8_t band_claimed[CAM_BAND_CLIENTS];
/*
 * Callbacks currently INSIDE a client's function.  Only ever 0 or 1 in practice --
 * there is one producer thread -- but a counter rather than a flag because the pair
 * (test claim, enter callback) has to be atomic against release, and a counter makes
 * "has everyone left?" a single unambiguous test.
 */
static volatile uint8_t band_cb_active[CAM_BAND_CLIENTS];
static cam_band_client_fn band_fn[CAM_BAND_CLIENTS];

static volatile int band_lost;      /* the stream died under its claimants  */
static volatile int band_arming;    /* claim is inside camera_band_start()  */
static int          band_colorbar;

/*
 * Serializes claim against claim and against release -- NOT the fan-out, which stays
 * lock-free on PRIMASK because it runs on the producer thread and must never block.
 *
 * 🔴 WITHOUT THIS, TWO COLD STARTS RACE, and the consequence is the exact corruption
 * the drain exists to prevent.  Both consoles can issue a command at once (USB CDC
 * and telnet), so `camera preview on` and `ai stream start` can both observe
 * "no stream running", both publish their claim -- claims are published BEFORE the
 * stream starts, so that the very first band cannot be dropped -- and both call
 * camera_band_start().  The camera accepts one and returns CAM_ERR_BUSY to the other,
 * but the loser was in the fan-out set for the window in between, so a band can be
 * delivered to a client whose caller is already unwinding and releasing its guards.
 * Serializing here removes the second cold start entirely: the loser takes the lock
 * afterwards, re-evaluates, sees a stream running, and simply joins it.
 *
 * A PRIMASK test-and-set plus a bounded sleep rather than a TX_MUTEX, matching
 * blob_busy_acquire() and psram_acquire_shared(): it needs no object creation, so
 * there is no init-ordering question in a module that has no init call.
 * Thread context only -- it sleeps.
 */
static volatile uint8_t band_api_busy;

static int band_api_lock(void)
{
	int tries;

	for (tries = 0; tries < CAM_BAND_LOCK_TICKS; tries++) {
		uint32_t primask = __get_PRIMASK();
		int got;

		__disable_irq();
		got = !band_api_busy;
		if (got)
			band_api_busy = 1u;
		__set_PRIMASK(primask);

		if (got)
			return 1;
		tx_thread_sleep(1);
	}
	return 0;
}

static void band_api_unlock(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	band_api_busy = 0u;
	__set_PRIMASK(primask);
}

int cam_band_claimed(enum cam_band_client c)
{
	return (c < CAM_BAND_CLIENTS) ? band_claimed[c] != 0u : 0;
}

int cam_band_any_claimed(void)
{
	unsigned i;

	for (i = 0; i < CAM_BAND_CLIENTS; i++)
		if (band_claimed[i])
			return 1;
	return 0;
}

int cam_band_colorbar(void) { return band_colorbar; }

int cam_band_stream_lost(void)
{
	/*
	 * Lazy latch.  There is no observer thread and none is wanted: the camera stops
	 * calling us on teardown, so "the stream died" can only be noticed by someone
	 * asking, and everyone who cares (the stats commands, the NN worker's timeout)
	 * comes through here.
	 *
	 * band_arming keeps the window inside cam_band_claim() from latching a false
	 * positive: the claim is published BEFORE camera_band_start() is called (so the
	 * very first band cannot be dropped on the floor), which means there is a brief
	 * moment where a client is claimed and no stream is running yet.
	 */
	if (!band_lost && !band_arming && cam_band_any_claimed() &&
	    !camera_band_streaming())
		band_lost = 1;
	return band_lost;
}

/*
 * The one real camera_band_fn.  Runs on the camera's producer thread.
 *
 * 🔴 THE CLAIM TEST AND THE cb_active INCREMENT ARE ONE ATOMIC STEP.  Clearing a
 * claim and then waiting for a "busy" flag would miss the producer that has already
 * passed the claim test and not yet entered the callback: it would run afterwards,
 * writing a buffer whose owner believed the drain was complete.  Once band_claimed[c]
 * reads 0 inside the critical section, a producer pass has either already incremented
 * -- and release waits for it -- or will read 0 and skip.  There is no third case,
 * which is what a pair of separate flags could not guarantee.
 */
static void cam_band_fanout(void *ctx, unsigned band, const uint16_t *px,
                            unsigned rows)
{
	unsigned c;

	(void)ctx;

	if (band_lost)
		return;             /* latched dead: stop feeding clients */

	for (c = 0; c < CAM_BAND_CLIENTS; c++) {
		cam_band_client_fn fn;
		uint32_t primask;

		primask = __get_PRIMASK();
		__disable_irq();
		fn = band_claimed[c] ? band_fn[c] : NULL;
		if (fn != NULL)
			band_cb_active[c]++;
		__set_PRIMASK(primask);

		if (fn == NULL)
			continue;

		fn(band, px, rows);

		/* 🔴 Re-read PRIMASK here rather than reusing the value from entry: the
		 * callback may have changed the interrupt state, and restoring a stale
		 * value would leave interrupts in whatever state they were two sections
		 * ago.  The same reason nn_session_release() saves its own. */
		primask = __get_PRIMASK();
		__disable_irq();
		band_cb_active[c]--;
		__set_PRIMASK(primask);
	}
}

int cam_band_claim(enum cam_band_client c, int colorbar, cam_band_client_fn fn)
{
	uint32_t primask;
	int need_start, rc;

	if (c >= CAM_BAND_CLIENTS || fn == NULL)
		return CAM_BAND_ERR_ARG;
	if (!band_api_lock())
		return CAM_BAND_ERR_LOCK;

	/* An explicit claim is the documented recovery from an overrun, so it clears
	 * the latch.  Nothing else does -- see the header. */
	band_lost = 0;
	/* Evaluated INSIDE the lock, which is what turns a would-be second cold start
	 * into a join. */
	need_start = !camera_band_streaming();
	if (need_start)
		band_arming = 1;

	primask = __get_PRIMASK();
	__disable_irq();
	band_fn[c] = fn;
	band_claimed[c] = 1u;
	__set_PRIMASK(primask);

	if (!need_start) {
		band_api_unlock();
		return CAM_BAND_OK;     /* joining a stream that is already running */
	}

	/* Published before the stream starts, deliberately: the first band can arrive
	 * the moment camera_band_start() returns, and a claim set afterwards would
	 * drop it -- which for the NN means losing the band-0 latch that begins a
	 * frame, not just one band.  Safe to unwind without draining below, because
	 * on this path the stream never started, so no band was ever fanned out. */
	band_colorbar = colorbar;
	rc = camera_band_start(colorbar, cam_band_fanout, NULL);
	band_arming = 0;
	if (rc != CAM_OK) {
		primask = __get_PRIMASK();
		__disable_irq();
		band_claimed[c] = 0u;
		__set_PRIMASK(primask);
		band_api_unlock();
		LOG_ERR("band start failed (%d)", rc);
		return CAM_BAND_ERR_START;
	}
	band_api_unlock();
	return CAM_BAND_OK;
}

int cam_band_release(enum cam_band_client c)
{
	uint32_t primask;
	int tries;

	if (c >= CAM_BAND_CLIENTS)
		return CAM_BAND_ERR_ARG;
	if (!band_api_lock())
		return CAM_BAND_ERR_LOCK;

	primask = __get_PRIMASK();
	__disable_irq();
	band_claimed[c] = 0u;
	__set_PRIMASK(primask);

	/* Bounded drain.  The producer runs at priority 10 and every caller of this is
	 * below it, so a callback in flight preempts us and finishes promptly; the sleep
	 * is what lets it. */
	for (tries = CAM_BAND_DRAIN_TICKS; band_cb_active[c] != 0u && tries > 0; tries--)
		tx_thread_sleep(1);

	if (band_cb_active[c] != 0u) {
		/* Do NOT stop the stream and do NOT tell the caller it is safe to release
		 * whatever the callback writes into.  See cam_band.h. */
		LOG_ERR("band drain timeout, client %u still in callback", (unsigned)c);
		band_api_unlock();
		return CAM_BAND_ERR_BUSY;
	}

	if (cam_band_any_claimed()) {
		band_api_unlock();
		return CAM_BAND_OK;
	}

	/* Last one out. "Lost" only means something while somebody expects a stream. */
	band_lost = 0;
	if (camera_band_streaming() && camera_stream_stop() != CAM_OK) {
		LOG_ERR("band stream stop timeout");
		band_api_unlock();
		return CAM_BAND_ERR_STOP;
	}
	band_api_unlock();
	return CAM_BAND_OK;
}
