/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn.h
 * @brief   Backend-agnostic on-device neural-network inference API (issue #9 P1).
 *
 * A thin tensor-in / tensor-out abstraction over one selectable inference runtime.
 * Exactly one backend is compiled in per build (CONFIG_NN_BACKEND in CMakeLists.txt):
 * `null` -- no runtime, a synthetic stub that exercises the plumbing end to end and
 * always builds -- and later `tflm` (LiteRT / TensorFlow Lite Micro with CMSIS-NN),
 * which is what will actually run BlazeFace-128 int8 from the external NOR blob
 * region.  The abstraction is deliberately model-agnostic: model-specific pre- and
 * post-processing (BlazeFace anchor decode + NMS) belongs ABOVE this layer in
 * port/nn/models/, so the same API serves any model the backend can run.
 *
 * Layering (HAL/CMSIS/ThreadX <- port <- shell <- app): this is a port/ module.  It
 * depends on the compiled-in backend, on CMSIS for the cycle counter, and on
 * ThreadX for one tx_thread_sleep() in the open path -- nothing else.  It never
 * reaches up into shell/ or app/, and it does not log: there is no hardware here to
 * bring up, so there is nothing for `dmesg` to record.
 *
 * Threading: a single model instance is NOT reentrant, and this board has TWO
 * consoles (USB CDC and telnet) that can both issue `ai` commands.  Callers
 * serialize with nn_session_try_acquire() / nn_session_release() below.
 *
 * Timing: nn_run() captures the elapsed DWT cycle count for nn_last_cycles().
 * The counter belongs to svc/timebase.c, which app/main.c starts before the
 * scheduler; this module only READS it (see nn_run()).
 */
#ifndef NN_H
#define NN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Element type of a tensor buffer. */
enum nn_dtype {
	NN_DTYPE_NONE = 0,
	NN_DTYPE_INT8,
	NN_DTYPE_UINT8,
	NN_DTYPE_INT16,
	NN_DTYPE_INT32,
	NN_DTYPE_FLOAT32,
};

/** Maximum tensor rank and per-model I/O tensor count this API exposes. */
#define NN_MAX_DIMS 4
#define NN_MAX_IO   8

/*
 * Error codes this layer returns for its OWN failures, kept far from zero so they
 * cannot be confused with a backend's.  A backend reports why a particular model was
 * rejected with its own small negative numbers (the TFLM backend uses -1..-7); those
 * pass through unchanged, because only the backend knows what went wrong inside it.
 */
#define NN_ERR_ARG    (-90)   /**< a required argument was NULL or out of range     */
#define NN_ERR_NOSUP  (-91)   /**< the compiled-in backend does not implement this  */
#define NN_ERR_STATE  (-92)   /**< no model is open; call nn_model_open() first     */

/*
 * Why a backend refused a model (nn_model_reload).  These live here rather than inside
 * the backend so that the shell command reporting them stays backend-agnostic: the
 * reasons an interpreting runtime can reject a flatbuffer are generic, and a command
 * that had to know TFLM's private numbering would have to be rewritten for the next
 * backend.  nn_model_strerror() turns one into a sentence.
 */
#define NN_MODEL_ERR_VERSION  (-1)  /**< schema version the runtime cannot read      */
#define NN_MODEL_ERR_OPS      (-2)  /**< the operator resolver could not be built    */
#define NN_MODEL_ERR_ARENA    (-3)  /**< activations do not fit, or an op is missing */
#define NN_MODEL_ERR_TENSOR   (-4)  /**< a tensor has no buffer or too high a rank   */
#define NN_MODEL_ERR_SHAPE    (-5)  /**< I/O tensor count outside 1..NN_MAX_IO       */
#define NN_MODEL_ERR_EMPTY    (-6)  /**< zero-length or impossibly short model       */
#define NN_MODEL_ERR_FORMAT   (-7)  /**< not a valid flatbuffer for this runtime     */
#define NN_MODEL_ERR_SLOT     (-8)  /**< buffer is not the one load_region() gave    */

/**
 * A single input or output tensor.  @p data points at the backend/model-owned
 * buffer the caller fills (inputs) or reads (outputs); the caller does not own or
 * free it.  For quantized (int8/uint8) tensors @p scale and @p zero_point carry the
 * affine quantization params (real = scale * (q - zero_point)); @p scale is 0 for
 * non-quantized tensors.  Dims are most- to least-significant (e.g. NHWC:
 * dims[0]=N, dims[1]=H, dims[2]=W, dims[3]=C).
 */
struct nn_tensor {
	void    *data;                 /**< tensor buffer (backend-owned)         */
	uint32_t bytes;                /**< buffer size in bytes                  */
	uint16_t dims[NN_MAX_DIMS];    /**< shape, MSB..LSB; unused dims are 1    */
	uint8_t  ndim;                 /**< number of valid dims                  */
	uint8_t  dtype;                /**< enum nn_dtype                         */
	float    scale;                /**< quant scale (0 => not quantized)      */
	int32_t  zero_point;           /**< quant zero point                      */
};

/** Opaque model handle (defined in nn.c). */
struct nn_model;

/** Identity of the compiled-in backend, for `ai info`. */
struct nn_backend_info {
	const char *name;      /**< "null" | "tflm"                               */
	const char *version;   /**< backend/runtime version string, or ""         */
};

/**
 * One-time backend init.  Idempotent and cheap to call repeatedly (the shell calls
 * it lazily through nn_model_open() on first `ai` use, which is why app/main.c
 * needs no NN bring-up call).  Returns 0 on success, <0 on failure.
 */
int nn_init(void);

/** Identity of the compiled-in backend (never NULL, valid before any open). */
const struct nn_backend_info *nn_backend(void);

/**
 * Open the (single, compiled-in) model.  Returns 0 and sets @p *out on success, <0
 * otherwise.  The handle is a singleton; a second open returns the same instance.
 * Safe to call concurrently from both consoles -- the one-time init is serialized
 * internally.
 */
int  nn_model_open(struct nn_model **out);
void nn_model_close(struct nn_model *m);

/** Human-readable model name (never NULL). */
const char *nn_model_name(const struct nn_model *m);

int  nn_input_count(const struct nn_model *m);
int  nn_output_count(const struct nn_model *m);

/** Input/output tensor @p idx, or NULL if out of range. */
struct nn_tensor *nn_input(struct nn_model *m, int idx);
struct nn_tensor *nn_output(struct nn_model *m, int idx);

/** Size of the activation arena (bytes), for `ai info`; 0 if unknown. */
uint32_t nn_activations_bytes(const struct nn_model *m);

/**
 * Run one inference.  Inputs must be filled first; outputs are valid on return.
 * CPU-bound and blocking (it never sleeps).  Returns 0 on success, <0 on error.
 *
 * CONTRACT -- ONE RUN MUST BE SHORTER THAN THE CYCLE COUNTER WRAPS.  Timing uses
 * the 32-bit DWT CYCCNT, which wraps every 2^32 core cycles = 7.81 s at 550 MHz.
 * The unsigned delta is wrap-safe for exactly one wrap, so a run that took longer
 * does not report an error -- it silently aliases down to a small, plausible-looking
 * number.  Keep a single run below 2^31 cycles (~3.9 s); a backend that could exceed
 * that must move timing to a wider source (the WFI-safe TIM2 the ThreadX execution
 * profile kit uses, port/threadx/tx_glue.c) rather than trusting this field.
 */
int nn_run(struct nn_model *m);

/**
 * DWT core cycles spent inside the most recent successful nn_run(); 0 if none, or
 * if the cycle counter was found not to be running (see nn_init()).
 */
uint32_t nn_last_cycles(const struct nn_model *m);

/*
 * Runtime model swap (issue #9 phase 2c).  A backend that interprets a model at run
 * time -- `tflm` -- can be handed a different one without reflashing; the `null` stub
 * cannot, and returns NN_ERR_NOSUP from both calls below rather than pretending.
 *
 * Which model to load and where it comes from are NOT decided here.  This layer is
 * model-agnostic by design, and the answer on this board (a slot in the external NOR
 * blob region) belongs to the shell command that reads it -- see shell/cmds/cmd_ai.c.
 * All this API says is: here is a buffer, fill it, tell me you did.
 *
 * THE CALLER MUST HOLD THE NN SESSION (nn_session_try_acquire) ACROSS BOTH CALLS.
 * Between them the staging buffer is being filled, and after them the model handle and
 * every tensor pointer in it have been replaced.  A second console running `ai bench`
 * in that window would be reading tensors out of a model that no longer exists.
 */

/**
 * Ask the backend where a model should be staged.  On success @p *buf points at a
 * backend-owned buffer of @p *cap bytes that is NOT in use by the running model, so
 * filling it is safe even while inference is loaded.
 *
 * Returns 0, NN_ERR_ARG, or NN_ERR_NOSUP.
 */
int nn_model_load_region(void **buf, uint32_t *cap);

/**
 * Adopt the model now sitting in the staging buffer: @p len bytes at @p data, called
 * @p name for display.  @p data == NULL unloads, which leaves the backend open with no
 * model rather than closed (`ai info` still works, `ai bench` refuses).
 *
 * TRANSACTIONAL.  A model the backend cannot build is reported without disturbing the
 * one already loaded, so a truncated file costs nothing but the message.  Returns 0,
 * NN_ERR_NOSUP / NN_ERR_STATE, or the backend's own <0 reason for the rejection.
 */
int nn_model_reload(const void *data, uint32_t len, const char *name);

/**
 * One short sentence for a code returned by nn_model_reload(), never NULL.  Handles
 * both the NN_MODEL_ERR_* family and this layer's own NN_ERR_* codes.
 */
const char *nn_model_strerror(int rc);

/**
 * Heap allocations made by the backend's language runtime, for `ai info`.  Returns 0
 * and sets @p *out, or NN_ERR_NOSUP when the backend has no such runtime to count.
 *
 * The TFLM backend is built so this stays at 0 -- its interpreter allocates only from
 * the arena.  Printing it turns that from a design intention into something the board
 * confirms after every change.
 */
int nn_heap_allocs(uint32_t *out);

/*
 * Coarse single-session guard.  The singleton model and the backends are NOT
 * reentrant, and two shells (USB CDC + telnet) can issue `ai` commands at the same
 * priority, so only ONE inference activity may use the model at a time.
 * try_acquire() claims the session (returns 0) or reports it busy (<0); release()
 * frees it.  Interrupt-guarded and thread-agnostic -- the acquiring and releasing
 * threads may differ.  No init required.
 */
int  nn_session_try_acquire(void);
void nn_session_release(void);

#ifdef __cplusplus
}
#endif

#endif /* NN_H */
