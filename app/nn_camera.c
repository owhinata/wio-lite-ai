/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_camera.c
 * @brief   Camera -> BlazeFace inference glue (issue #9 phase 3).  See nn_camera.h
 *          for the design and, in particular, for why there is no staging buffer.
 */
#include "nn_camera.h"

#include "cam_band.h"
#include "camera.h"
#include "nn.h"
#include "psram.h"

#include "stm32h7xx_hal.h"   /* HAL_GetTick, SystemCoreClock, DWT */
#include "tx_api.h"

#define LOG_TAG "nncam"
#include "log.h"
#include "mem_sections.h"    /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

#include <stddef.h>
#include <string.h>

/*
 * Priority 18, which nothing else in this firmware uses.
 *
 * BELOW the CLI (16) so `ai stream stop` always reaches the console -- inference is
 * a monolithic ~373 ms call with no yield in it, which is precisely why it must sit
 * under the thing that stops it.  BELOW the preview (12) and the camera producer
 * (10) so inference never delays the display or the band deadline.
 */
#define NNCAM_PRIO   18u
/*
 * 3,072 B in DTCM.  The same inference measured a 1,940 B peak on the CLI thread in
 * phase 2c, and blazeface_decode() adds ~250 B (a 64-byte NMS bitmap plus the
 * detection array), so this is ~1.4x the expected peak.
 *
 * 🔴 That margin is thinner than it looks from the phase 2c notes, which recorded
 * "DTCM free 15,808 B".  That figure counted the 8 KB main-stack reservation as
 * free; the number the linker ASSERT actually enforces (_dtcm_used_end <=
 * _smsp_stack) was 7,616 B, and this stack spends 3,072 of it.  `free` prints the
 * high-water mark for exactly this reason -- there is no MSPLIM on ARMv7-M and an
 * MPU guard page would lock the part up rather than report (see mem_sections.h).
 */
#define NNCAM_STACK  3072u

/*
 * How long the worker waits for a frame before re-checking the run flag.  Without a
 * bound it would wedge on a semaphore nothing will post: a DCMI overrun or a stream
 * that stops underneath ends the band flow with want_frame still set.
 */
#define NNCAM_FRAME_WAIT_TICKS 100u

/*
 * How long nn_camera_stop() waits for the worker to leave the run loop, in ticks
 * (1 ms).  One inference is ~373 ms and the worker is below the caller in priority,
 * so it normally clears in well under half of this; the bound exists so a wedged
 * worker reports rather than hangs the console.
 */
#define NNCAM_STOP_TICKS 1500

static TX_THREAD    nncam_thread;
static UCHAR        nn_worker_stack[NNCAM_STACK] DTCM_BSS __attribute__((aligned(8)));
static TX_SEMAPHORE nncam_start_sem;   /* worker idles here between streams   */
static TX_SEMAPHORE nncam_frame_sem;   /* producer -> worker: your frame is in */
static TX_MUTEX     nncam_det_lock;    /* guards the published detections      */
static int          nncam_created;

static struct nn_model *nncam_model;

/* Set by start/stop (thread context), read by the worker and the band callback. */
static volatile int nncam_run;
/* The worker is inside nn_run().  While set, the input tensor belongs to it -- the
 * arena reuses that space for intermediates, so a producer write here is corruption. */
static volatile int nncam_infer_active;
/* The worker is inside the run loop, i.e. it may touch the tensors at any moment. */
static volatile int nncam_worker_busy;
/* The worker wants a frame; while this is 0 the producer does not touch the input. */
static volatile int nncam_want_frame;
/* Frame-level latch: a fill starts at band 0 and never mid-frame. */
static volatile int nncam_filling;
/* Do we still hold the NN session + the OCTOSPI1 guard? */
static volatile int nncam_holds_guards;

static uint32_t nncam_infers, nncam_frames, nncam_skipped, nncam_errors;
/* Diagnostics for the ownership invariant (#54).  `raced` must stay 0: it counts bands
 * that wrote the input tensor while an inference owned it.  `stale` counts posts the
 * pre-arm drain threw away -- each one is a race that WOULD have started. */
static uint32_t nncam_raced, nncam_stale_posts;
static uint32_t nncam_ingest_last, nncam_ingest_max, nncam_infer_cyc;
static uint32_t nncam_start_tick;

static int nncam_norm_signed;   /* 0 = [0,1] (default), 1 = [-1,1] */
static int nncam_overlay;

static struct bf_det nncam_dets[BF_MAX_DET];
static int           nncam_ndet;

/* Input geometry, latched at start so the band callback does no shape work. */
static unsigned nncam_ow, nncam_oh, nncam_oc;

/* ------------------------------------------------------------------ guards ---- */

/*
 * Idempotent under a PRIMASK critical section, because two different threads can
 * legitimately try to be the last one out: nn_camera_stop() on the console, and the
 * worker on its way off the run loop after a stop that timed out.  Exactly one of
 * them performs the release.
 */
static void nncam_guards_give(void)
{
	uint32_t primask = __get_PRIMASK();
	int mine;

	__disable_irq();
	mine = nncam_holds_guards;
	nncam_holds_guards = 0;
	__set_PRIMASK(primask);

	if (mine) {
		psram_release();
		nn_session_release();
	}
}

/* ------------------------------------------------------------- preprocessing -- */

/*
 * First output row belonging to band @p band, for a model @p oh rows tall.
 *
 * Nearest-neighbour sampling puts output row oy on source row floor(oy*SRC_H/oh), so
 * oy belongs to band b exactly when that lands in [b*ROWS, (b+1)*ROWS) --
 * i.e. oy in [ceil(b*ROWS*oh / SRC_H), ceil((b+1)*ROWS*oh / SRC_H)).  For the 128
 * input this is the exact 32-rows-per-band split the design assumes (band b covers
 * output rows [32b, 32b+32), whose source rows [60b, 60b+58] lie wholly inside band
 * b), and there is no band-boundary sampling case at all.  Written in the general
 * form so a differently sized model input tiles correctly instead of silently
 * sampling across a band it was never handed; nn_camera_start() checks that the
 * tiling actually covers [0, oh).
 */
static unsigned nncam_oy_bound(unsigned band, unsigned oh)
{
	uint32_t n = (uint32_t)band * (uint32_t)CAMERA_BAND_ROWS * (uint32_t)oh;

	return (unsigned)((n + CAMERA_FRAME_HEIGHT - 1u) / CAMERA_FRAME_HEIGHT);
}

/*
 * Downsample output rows [oy0, oy_end) of the model input from one band.
 *
 * @p src is the band's first row (source row @p src_y0 of the full frame), tightly
 * packed RGB565, CAMERA_FRAME_WIDTH wide, in AXI-SRAM and already invalidated by the
 * camera driver.  It is 32-byte aligned, so unlike the donor -- which assembled each
 * pixel from two byte loads to stay alignment-agnostic on a pinned frame pointer --
 * this reads uint16_t directly.
 *
 * Layout is HWC (o = (oy*ow + ox) * oc), channel order RGB, matching a 1xHxWxC input.
 *
 * 🔴 For a quantized input the value is the NORMALIZED float put through the
 * tensor's OWN scale/zero_point, not the donor's hardcoded (rgb - 128).  This
 * board's struct nn_tensor carries the quantization params and the donor's did not,
 * which is the only reason it had to assume (1/128, 0); assuming it here would be
 * silently wrong for any model quantized differently, and `ai norm` would do nothing
 * on a quantized input.  The 1/scale reciprocal is computed once per band rather
 * than per channel: a vdiv.f32 is ~14 cycles and there are 3 per pixel.
 *
 * 🔴 scale > 0 IS GUARANTEED BY nn_camera_start(), which is why there is no fallback
 * here (issue #51).  There used to be one -- the donor's (rgb - 128) when the scale
 * read back as zero -- and it was not dead code: a PER-AXIS quantized tensor arrives
 * with TfLiteTensor::params.scale == 0, because its real parameters live in the
 * affine-quantization struct that port/nn/nn.h does not expose.  So the fallback's
 * only reachable case was the one where it silently fed the model wrong pixels, with
 * the same hardcoded assumption the paragraph above rejects.  Refusing the model at
 * start() says so once, out loud, instead of per pixel, never.
 */
static void nncam_rows(const uint16_t *src, unsigned src_y0,
                       unsigned oy0, unsigned oy_end, struct nn_tensor *in)
{
	const unsigned ow = nncam_ow, oh = nncam_oh, oc = nncam_oc;
	const int is_f32 = (in->dtype == NN_DTYPE_FLOAT32);
	const float bias = nncam_norm_signed ? -1.0f : 0.0f;
	const float gain = nncam_norm_signed ? (1.0f / 127.5f) : (1.0f / 255.0f);
	const float inv_scale = is_f32 ? 0.0f : (1.0f / in->scale);
	const int32_t zp = in->zero_point;
	unsigned oy, ox, c;

	for (oy = oy0; oy < oy_end; oy++) {
		unsigned sy = (unsigned)((uint32_t)oy * CAMERA_FRAME_HEIGHT / oh);
		const uint16_t *row = src + (size_t)(sy - src_y0) * CAMERA_FRAME_WIDTH;

		for (ox = 0; ox < ow; ox++) {
			unsigned sx = (unsigned)((uint32_t)ox * CAMERA_FRAME_WIDTH / ow);
			uint16_t px = row[sx];
			uint32_t o = ((uint32_t)oy * ow + ox) * oc;
			uint8_t rgb[3];

			/* Exact 5/6-bit -> 8-bit scaling, as the donor does.  The constant
			   divisors become a multiply-and-shift, so this costs no divide. */
			rgb[0] = (uint8_t)(((px >> 11) & 0x1Fu) * 255u / 31u);
			rgb[1] = (uint8_t)(((px >> 5) & 0x3Fu) * 255u / 63u);
			rgb[2] = (uint8_t)((px & 0x1Fu) * 255u / 31u);

			if (is_f32) {
				float *o32 = (float *)in->data + o;

				for (c = 0; c < oc && c < 3u; c++)
					o32[c] = (float)rgb[c] * gain + bias;
			} else {
				int8_t *o8 = (int8_t *)in->data + o;

				for (c = 0; c < oc && c < 3u; c++) {
					float f = ((float)rgb[c] * gain + bias) * inv_scale;
					/* Round half away from zero without libm: this
					   firmware links none, and lrintf() would pull it in
					   for three multiply-adds per pixel.  This is the
					   rounding TFLM's own QUANTIZE kernel does --
					   reference_ops::AffineQuantize() -> TfLiteRound()
					   -> std::round() -- so a model whose leading
					   QUANTIZE was stripped gets the identical tensor. */
					int q = (int)(f + (f >= 0.0f ? 0.5f : -0.5f)) + (int)zp;

					if (q < -128)
						q = -128;
					else if (q > 127)
						q = 127;
					o8[c] = (int8_t)q;
				}
			}
		}
	}
}

/* ----------------------------------------------------------- band ingest ------ */

/*
 * One band, on the camera's producer thread, fanned out by app/cam_band.c after the
 * preview has had it.  Must finish well inside a band period (~18.5 ms); the DWT
 * cycles below are what proves it does, and `ai stream stats` reports the worst one.
 */
static void nncam_band(unsigned band, const uint16_t *px, unsigned rows)
{
	const int last = (band + 1u >= CAMERA_BANDS_PER_FRAME);
	struct nn_tensor *in;
	uint32_t t0, cyc;
	unsigned oy0, oy_end;

	/* The band tiling below is derived from CAMERA_BAND_ROWS, and the driver's
	   contract is that every band is exactly that tall (port/camera/camera.h).  If
	   that ever stops being true the rows would be sampled from the wrong source
	   offsets and the image would simply be wrong -- so count it instead. */
	if (rows != CAMERA_BAND_ROWS) {
		nncam_filling = 0;
		nncam_errors++;
		return;
	}

	if (band == 0u && nncam_want_frame)
		nncam_filling = 1;

	if (!nncam_filling) {
		if (last)
			nncam_skipped++;   /* a whole frame went by while the worker ran */
		return;
	}

	/* 🔴 Re-read every frame, never cached across a session: `ai model load`
	   rebuilds the interpreter and re-plans the arena, so this pointer moves.
	   (That load cannot happen WHILE we stream -- it needs the NN session, which
	   this stream holds -- but the cost of re-reading is one load.) */
	in = nn_input(nncam_model, 0);
	if (in == NULL || in->data == NULL) {
		/* Abandon this frame WITHOUT posting: the worker must not run inference
		   over a half-filled tensor, and its bounded wait is exactly what makes
		   dropping the frame safe -- it re-arms want_frame on the next pass. */
		nncam_filling = 0;
		nncam_errors++;
		return;
	}

	/* The invariant this stream depends on: the worker and the producer never hold the
	 * input tensor at the same time.  Counted rather than assumed, because when it
	 * breaks the picture stays plausible -- part camera, part activations. */
	if (nncam_infer_active)
		nncam_raced++;

	oy0    = nncam_oy_bound(band, nncam_oh);
	oy_end = nncam_oy_bound(band + 1u, nncam_oh);

	t0 = DWT->CYCCNT;
	nncam_rows(px, band * rows, oy0, oy_end, in);
	cyc = DWT->CYCCNT - t0;
	nncam_ingest_last = cyc;
	if (cyc > nncam_ingest_max)
		nncam_ingest_max = cyc;

	if (last) {
		nncam_frames++;
		nncam_filling = 0;
		/* Order matters: clearing want_frame is what tells the next band 0 not to
		   start another fill, and it has to be clear before the worker -- which
		   will own the tensor from the moment it wakes -- can run. */
		nncam_want_frame = 0;
		(void)tx_semaphore_put(&nncam_frame_sem);
	}
}

/* ---------------------------------------------------------------- worker ------ */

static void nncam_publish(const struct bf_det *d, int n)
{
	if (tx_mutex_get(&nncam_det_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return;
	if (n > 0)
		memcpy(nncam_dets, d, (size_t)n * sizeof(*d));
	/*
	 * 🔴 A DECODER THAT DOES NOT RECOGNISE THE MODEL IS NOT "ZERO FACES" (issue #57).
	 * Collapsing blazeface_decode()'s -1 into 0 made every non-BlazeFace model report
	 * `dets: 0` -- which reads as a measurement -- next to a `maxscore` the decoder had
	 * returned too early to touch, so that number still belonged to whatever model ran
	 * before.  Two readings that both look like this model's answer, and neither is.
	 * The -1 is carried through to the shell instead, which prints the reason.
	 */
	nncam_ndet = (n < 0) ? -1 : n;
	(void)tx_mutex_put(&nncam_det_lock);
}

static void nncam_step(void)
{
	struct bf_det tmp[BF_MAX_DET];
	int n;

	/*
	 * 🔴 DISCARD ANY POST THAT PREDATES THIS ARM, AND DO IT BEFORE ARMING.
	 *
	 * want_frame is what licenses the producer to write the input tensor, so it must
	 * never still be set when nn_run() starts -- the tensor's arena space is reused by
	 * the intermediates of the very inference that is running (measured: an Invoke()
	 * rewrites 48,997 of the input's 49,152 bytes), so a producer writing into it
	 * concurrently and an inference reading it are the same memory.
	 *
	 * Without this drain, one stale post is enough to break that invariant FOREVER:
	 * the wait returns immediately, the inference starts with want_frame already set,
	 * the next band 0 begins filling underneath it, that fill completes during the
	 * inference and posts again -- and the next iteration repeats the whole thing.
	 * The state is self-sustaining, which is why the symptom is not an occasional bad
	 * frame but a stream that is wrong from some point onwards.
	 *
	 * The window that produces the stale post is real and routine: this wait times out
	 * after NNCAM_FRAME_WAIT_TICKS (100 ms) while a fill can legitimately take up to
	 * two frame periods (~148 ms) from arming, so the producer's post and the timeout
	 * can land together.  Discarding it costs one frame; not discarding it costs every
	 * frame after it.
	 *
	 * 🔴 What this does NOT establish is "the post I get back belongs to a fill that
	 * started after I armed".  A fill already in flight when the previous step timed
	 * out completes and posts after this arm, and that is fine -- the guarantee that
	 * matters comes from the PRODUCER, which clears want_frame BEFORE it posts (see
	 * nncam_band()).  So a post can only be observed after the producer has finished
	 * writing and given up its licence, which is exactly the invariant nn_run() needs.
	 * The drain's narrower job is to make sure the post being observed is not one from
	 * a frame whose licence was granted by an ARM THAT IS STILL IN FORCE.
	 */
	while (tx_semaphore_get(&nncam_frame_sem, TX_NO_WAIT) == TX_SUCCESS)
		nncam_stale_posts++;

	nncam_want_frame = 1;
	if (tx_semaphore_get(&nncam_frame_sem, NNCAM_FRAME_WAIT_TICKS) != TX_SUCCESS) {
		/* No frame within the bound.  The band flow may have ended without ever
		   posting -- a DCMI overrun, or the stream stopped underneath us -- in
		   which case there is nothing left to wait for.  cam_band_stream_lost()
		   is evaluated lazily by whoever asks, and this is one of the askers.
		   Drop the fill latch with it: the stream may have died part way through
		   a frame, and leaving it set would let a re-armed stream resume that
		   frame from whichever band arrives first.  No producer exists while the
		   stream is lost, so this is the one place it can safely be cleared. */
		if (cam_band_stream_lost()) {
			nncam_want_frame = 0;
			nncam_filling    = 0;
		}
		return;
	}
	if (!nncam_run)
		return;

	/* Guards the assertion above rather than any data: while this is set, NOTHING may
	 * write the input tensor, and nncam_band() counts it if anything does. */
	nncam_infer_active = 1;
	n = nn_run(nncam_model);
	nncam_infer_active = 0;
	if (n != 0) {
		nncam_errors++;
		return;
	}
	nncam_infer_cyc = nn_last_cycles(nncam_model);

	n = blazeface_decode(nncam_model, tmp, BF_MAX_DET);
	nncam_publish(tmp, n);
	/* Bumped LAST, after the boxes are published: `ai run` waits for this counter
	   to move and then reads the detections, so incrementing first would let it
	   read the PREVIOUS inference's boxes and report them as this one's. */
	nncam_infers++;
}

static void nncam_entry(ULONG arg)
{
	(void)arg;
	for (;;) {
		if (tx_semaphore_get(&nncam_start_sem, TX_WAIT_FOREVER) != TX_SUCCESS)
			continue;

		/* Set BEFORE the loop: while this is set the worker may be touching the
		   tensors, and nn_camera_stop() must not conclude otherwise.  A stop that
		   lands in the window between here and the test below simply finds the
		   flag clear, and the loop it is racing never executes a single step. */
		nncam_worker_busy = 1;
		while (nncam_run)
			nncam_step();
		nncam_worker_busy = 0;

		/* If a stop gave up waiting for us it left the guards held on purpose --
		   see nn_camera_stop().  We are now the last one out, so we release. */
		nncam_guards_give();
	}
}

/* ------------------------------------------------------------------- API ------ */

/*
 * Creation is serialized by the NN session, which nn_camera_start() takes before
 * calling this -- so no separate latch is needed even with two consoles.
 *
 * Unwound on partial failure rather than left half-created: these are static control
 * blocks, so a retry would re-create an object ThreadX already knows about, which is
 * a different (and much more confusing) failure than the one that got us here.
 */
static int nncam_create_objects(void)
{
	if (nncam_created)
		return NNCAM_OK;

	if (tx_semaphore_create(&nncam_start_sem, "nncam_st", 0) != TX_SUCCESS)
		return NNCAM_ERR_INIT;
	if (tx_semaphore_create(&nncam_frame_sem, "nncam_fr", 0) != TX_SUCCESS)
		goto del_start;
	if (tx_mutex_create(&nncam_det_lock, "nncam_dt", TX_INHERIT) != TX_SUCCESS)
		goto del_frame;
	if (tx_thread_create(&nncam_thread, "nn_work", nncam_entry, 0,
	                     nn_worker_stack, sizeof nn_worker_stack,
	                     NNCAM_PRIO, NNCAM_PRIO,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
		goto del_mutex;

	nncam_created = 1;
	return NNCAM_OK;

del_mutex:
	(void)tx_mutex_delete(&nncam_det_lock);
del_frame:
	(void)tx_semaphore_delete(&nncam_frame_sem);
del_start:
	(void)tx_semaphore_delete(&nncam_start_sem);
	LOG_ERR("worker objects could not be created");
	return NNCAM_ERR_INIT;
}

int nn_camera_running(void) { return nncam_run; }

int nn_camera_start(int colorbar)
{
	struct nn_model *m = NULL;
	struct nn_tensor *in;
	int rc;

	if (nncam_run) {
		/*
		 * Already running -- unless the stream died underneath us, in which case
		 * THIS IS THE DOCUMENTED RE-ARM.  Everything except the stream is still
		 * intact: we hold the session and the OCTOSPI1 guard, the worker is alive
		 * in its bounded wait, and the NN claim was never dropped.  So the re-arm
		 * is exactly one call, and it must NOT re-take the guards.
		 *
		 * Refusing here instead would make `ai stream stats`' own advice ("re-issue
		 * `ai stream start` to re-arm") wrong, which is worse than having no
		 * recovery hint at all.
		 */
		if (!cam_band_stream_lost())
			return NNCAM_ERR_RUNNING;

		/* Clear the fill latch before the stream comes back.  A stream that died
		   mid-frame leaves it set, and without clearing it the first re-armed band
		   would resume a frame that began before the outage -- the first inference
		   would then run over a tensor half of which is stale.  The worker's
		   timeout path clears it too; doing it here as well is what makes the
		   result independent of which of the two ran first (the re-arm can easily
		   arrive inside the worker's 100 ms window, or while it is mid-inference).
		   Safe to touch from here: the latch is only ever SET by the producer, and
		   cam_band_stream_lost() being true means there is no producer right now. */
		nncam_filling    = 0;
		nncam_want_frame = 0;

		if (cam_band_claim(CAM_BAND_NN, colorbar, nncam_band) != CAM_BAND_OK) {
			/* cam_band_claim() unwound our claim, but nncam_run and the guards
			   are still ours -- and with no claim left the lost latch cannot
			   re-arm itself, so a second `ai stream start` would just report
			   "already running".  Say what actually clears it instead of leaving
			   the user to discover a state with no obvious way out. */
			return NNCAM_ERR_REARM;
		}
		return NNCAM_OK;
	}
	if (nn_model_open(&m) != 0)
		return NNCAM_ERR_MODEL;

	/* Software claim first, hardware claim second -- taking the peripheral first
	   would mean holding it just to report that software was busy (the order
	   cmd_ai.c documents and phase 2a recorded). */
	if (nn_session_try_acquire() != 0)
		return NNCAM_ERR_SESSION;

	in = nn_input(m, 0);
	if (nn_input_count(m) < 1 || in == NULL || in->data == NULL ||
	    in->ndim != 4 || in->dims[0] != 1) {
		nn_session_release();
		return NNCAM_ERR_MODEL;
	}
	if (in->dtype != NN_DTYPE_FLOAT32 && in->dtype != NN_DTYPE_INT8) {
		nn_session_release();
		return NNCAM_ERR_GEOM;
	}
	/*
	 * 🔴 The quantization parameters have to be USABLE, not merely present (#51).
	 * nn_tensor carries ONE scale, and the backend fills it from
	 * TfLiteTensor::params.scale -- which a PER-AXIS quantized tensor leaves at zero,
	 * keeping its real parameters in the affine-quantization struct this API does not
	 * expose.  So a zero here does not mean "unit scale", it means "the number you
	 * need is somewhere you cannot see", and quantizing with any assumed constant
	 * would feed the model wrong pixels with nothing to show for it.  Refuse instead;
	 * nncam_rows() then needs no per-pixel fallback (see its comment).
	 */
	if (in->dtype == NN_DTYPE_INT8 && !(in->scale > 0.0f)) {
		nn_session_release();
		return NNCAM_ERR_QUANT;
	}
	nncam_oh = in->dims[1];
	nncam_ow = in->dims[2];
	nncam_oc = in->dims[3];
	/*
	 * 🔴 THE CHANNEL COUNT IS CHECKED BECAUSE nncam_rows() STOPS AT THREE (issue #57).
	 * It writes `c < oc && c < 3` channels from an RGB565 pixel, so a four-channel
	 * input would leave channel 3 of every pixel holding whatever the arena last had --
	 * no fault, no overrun, just one plane of stale activations fed to the model as if
	 * it were image data.  That is the failure mode this file already refuses a
	 * per-axis scale over: silently wrong pixels with nothing to show for it.  1 (the
	 * red channel alone) and 3 (RGB) are what the sampler can actually produce.
	 *
	 * The tiling then has to cover the input exactly, or some output rows would never
	 * be written (and would then be inferred from whatever the arena last held).
	 */
	if ((nncam_oc != 1u && nncam_oc != 3u) ||
	    nncam_oh == 0u || nncam_ow == 0u ||
	    nncam_oy_bound(0u, nncam_oh) != 0u ||
	    nncam_oy_bound(CAMERA_BANDS_PER_FRAME, nncam_oh) != nncam_oh ||
	    (uint32_t)nncam_ow * nncam_oh * nncam_oc *
	            (in->dtype == NN_DTYPE_FLOAT32 ? 4u : 1u) > in->bytes) {
		nn_session_release();
		return NNCAM_ERR_GEOM;
	}

	if (!psram_ready() || !psram_acquire_shared()) {
		nn_session_release();
		return NNCAM_ERR_PSRAM;
	}
	nncam_holds_guards = 1;

	rc = nncam_create_objects();
	if (rc != NNCAM_OK) {
		nncam_guards_give();
		return rc;
	}

	nncam_model       = m;
	nncam_infers      = 0u;
	nncam_frames      = 0u;
	nncam_skipped     = 0u;
	nncam_errors      = 0u;
	nncam_raced       = 0u;
	nncam_stale_posts = 0u;
	nncam_ingest_last = 0u;
	nncam_ingest_max  = 0u;
	nncam_infer_cyc   = 0u;
	nncam_start_tick  = HAL_GetTick();
	nncam_want_frame  = 0;
	nncam_filling     = 0;
	nncam_publish(NULL, 0);
	while (tx_semaphore_get(&nncam_frame_sem, TX_NO_WAIT) == TX_SUCCESS)
		;

	nncam_run = 1;
	rc = cam_band_claim(CAM_BAND_NN, colorbar, nncam_band);
	if (rc != CAM_BAND_OK) {
		nncam_run = 0;
		nncam_guards_give();
		return NNCAM_ERR_BAND;
	}
	(void)tx_semaphore_put(&nncam_start_sem);
	return NNCAM_OK;
}

int nn_camera_stop(void)
{
	int rc_band = CAM_BAND_OK;
	int i;

	if (!nncam_run && !nncam_holds_guards)
		return NNCAM_ERR_NOTRUN;

	nncam_run = 0;
	/* Poke the worker out of its bounded wait so it notices immediately rather
	   than after the remainder of a 100 ms timeout. */
	(void)tx_semaphore_put(&nncam_frame_sem);

	/*
	 * Producer side first.  cam_band_release() drops the claim and then drains any
	 * callback already in flight -- and the claim test and the in-flight count are
	 * updated together under one PRIMASK section, so a producer that passed the
	 * test but had not yet entered cannot slip through behind us.  Until this
	 * returns OK, something may still be writing the input tensor.
	 *
	 * 🔴 Called UNCONDITIONALLY, not just when the claim is still set.  A previous
	 * stop that timed out already cleared the claim but left a callback in flight;
	 * skipping the drain on the retry because "we are not claimed any more" would
	 * release the arena to `ai bench` with that callback still able to write the
	 * input tensor -- which is the precise thing the first stop refused to do.
	 * cam_band_release() is idempotent, so calling it again is free.
	 */
	rc_band = cam_band_release(CAM_BAND_NN);

	/* Consumer side second: the worker may be up to one inference (~373 ms) away
	   from noticing.  It is below us in priority, so sleeping is what lets it run. */
	for (i = 0; nncam_worker_busy && i < NNCAM_STOP_TICKS; i++)
		tx_thread_sleep(1);

	if (rc_band != CAM_BAND_OK || nncam_worker_busy) {
		/* 🔴 Deliberately do NOT release the session or the OCTOSPI1 guard.  See
		   nn_camera.h: handing the arena to `ai bench` while a dead-but-not-
		   returned callback can still write the input tensor is unrecoverable,
		   whereas holding a session nobody can use is not.  The worker releases
		   on its way out, or a second `ai stream stop` completes it. */
		LOG_WRN("stop: still tearing down (band %d, worker busy %d)",
		        rc_band, nncam_worker_busy);
		return NNCAM_ERR_TEARING;
	}

	nncam_guards_give();
	return NNCAM_OK;
}

void nn_camera_stats_get(struct nn_camera_stats *out)
{
	uint32_t now;

	if (out == NULL)
		return;
	memset(out, 0, sizeof(*out));

	out->running      = (uint8_t)(nncam_run != 0);
	out->holds_guards = (uint8_t)(nncam_holds_guards != 0);
	out->stream_lost  = (uint8_t)(cam_band_stream_lost() != 0);
	out->norm_signed  = (uint8_t)(nncam_norm_signed != 0);
	out->overlay      = (uint8_t)(nncam_overlay != 0);
	out->infers       = nncam_infers;
	out->frames       = nncam_frames;
	out->skipped      = nncam_skipped;
	out->errors       = nncam_errors;
	out->raced        = nncam_raced;
	out->stale_posts  = nncam_stale_posts;
	out->ingest_last_cyc = nncam_ingest_last;
	out->ingest_max_cyc  = nncam_ingest_max;
	out->infer_last_cyc  = nncam_infer_cyc;

	now = HAL_GetTick();
	out->elapsed_ms = nncam_start_tick ? (now - nncam_start_tick) : 0u;

	/* Read directly rather than under nncam_det_lock: a single int cannot tear on
	   this core, and taking the mutex here would make this function unusable before
	   the first start() has created it (`ai stream stats` on a cold boot). */
	out->ndet = nncam_ndet;
}

int nn_camera_dets_get(struct bf_det *out, int max)
{
	int n;

	if (out == NULL || max <= 0 || !nncam_created)
		return 0;
	if (tx_mutex_get(&nncam_det_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return 0;
	n = nncam_ndet;
	if (n > max)
		n = max;
	if (n > 0)
		memcpy(out, nncam_dets, (size_t)n * sizeof(*out));
	(void)tx_mutex_put(&nncam_det_lock);
	return n;
}

void nn_camera_set_norm(int signed_range) { nncam_norm_signed = signed_range ? 1 : 0; }
int  nn_camera_get_norm(void)             { return nncam_norm_signed; }
void nn_camera_set_overlay(int on)        { nncam_overlay = on ? 1 : 0; }
int  nn_camera_get_overlay(void)          { return nncam_overlay; }
