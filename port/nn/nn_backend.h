/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_backend.h
 * @brief   Internal vtable a concrete nn backend implements (issue #9 P1).
 *
 * nn.c dispatches the public nn.h API to exactly one backend through this vtable.
 * Each backend (nn_null.c, later nn_tflm.cc) defines its own private model handle
 * type and exports a single `const struct nn_backend_vt nn_backend_vt_selected`.
 *
 * HOW "EXACTLY ONE" IS ENFORCED: purely at link time.  CONFIG_NN_BACKEND adds one
 * -- and only one -- backend translation unit to the build, and nn.c's extern
 * reference below resolves against it.  There is no #ifdef fan-out inside any
 * source file, and the `null` build needs no preprocessor define at all.
 *
 * The vtable operates on an opaque backend handle (`void *impl`); nn.c wraps it in
 * the public `struct nn_model` and adds the cross-backend concerns.  run() is the
 * pure inference call -- it must NOT time itself; nn.c owns timing.
 *
 * OPTIONAL ENTRIES.  The last three may be NULL, and the `null` backend leaves all
 * three that way.  nn.c checks for NULL and returns NN_ERR_NOSUP itself, so no caller
 * ever reads a vtable field: "this backend cannot do that" is a value the public API
 * returns, not a crash waiting for whoever forgets to check.
 */
#ifndef NN_BACKEND_H
#define NN_BACKEND_H

#include "nn.h"

#ifdef __cplusplus
extern "C" {
#endif

struct nn_backend_vt {
	const struct nn_backend_info *info;

	/** One-time init; idempotent. 0 on success, <0 on failure. */
	int  (*init)(void);

	/** Open the singleton model; set *impl_out. 0 on success, <0 otherwise.
	 *  MUST NOT touch tensor bodies -- only fill the struct nn_tensor descriptors.
	 *  `ai info` calls this without bringing the PSRAM up, so a backend whose
	 *  buffers live in external memory would fault or stall if open() dereferenced
	 *  them. */
	int  (*open)(void **impl_out);
	void (*close)(void *impl);

	const char *(*model_name)(void *impl);
	int  (*in_count)(void *impl);
	int  (*out_count)(void *impl);
	struct nn_tensor *(*input)(void *impl, int idx);
	struct nn_tensor *(*output)(void *impl, int idx);
	uint32_t (*activations_bytes)(void *impl);

	/** Pure inference (no timing). Inputs pre-filled; 0 on success, <0 on error. */
	int  (*run)(void *impl);

	/* ---- optional: runtime model swap (issue #9 phase 2c) ------------------
	 *
	 * The shape issue #9 phase 1 deliberately left unspecified, now that phase 2b
	 * has settled it: a model is a blob on the external NOR, read into a staging
	 * buffer the backend owns.  Only a backend that INTERPRETS a model at run time
	 * can implement these -- which is the whole reason TFLM was chosen over a code
	 * generator, whose model is compiled in and can only change by reflashing.
	 */

	/**
	 * Hand out the staging buffer a model should be read into, and its capacity.
	 * The caller fills [*buf, *buf + len) and passes the same pointer to reload().
	 *
	 * The buffer is BACKEND-OWNED and must be the one NOT currently in use, so that
	 * filling it cannot disturb a model that is still running.  0 on success.
	 */
	int  (*load_region)(void **buf, uint32_t *cap);

	/**
	 * Rebuild the model from @p data (@p len bytes), naming it @p name for display.
	 * @p data == NULL unloads, leaving the backend open with no model.
	 *
	 * TRANSACTIONAL: on failure the previously active model must still be usable and
	 * *@p impl_out must describe it, so a bad model costs the operator a message
	 * rather than the working state.  *@p impl_out is ALWAYS the resulting active
	 * handle, or NULL if even the restore failed.  Returns 0, or the <0 code for why
	 * the NEW model was rejected (even when the old one was restored successfully).
	 */
	int  (*reload)(const void *data, uint32_t len, const char *name, void **impl_out);

	/**
	 * How many times the backend's language runtime has allocated from the C heap.
	 *
	 * Exists for one claim: the TFLM backend is configured so that inference never
	 * touches the heap, and `ai info` should be able to show that rather than assert
	 * it.  A backend with no runtime of its own leaves this NULL.
	 */
	uint32_t (*heap_allocs)(void);
};

/** The one backend selected at build time (provided by exactly one backend TU). */
extern const struct nn_backend_vt nn_backend_vt_selected;

#ifdef __cplusplus
}
#endif

#endif /* NN_BACKEND_H */
