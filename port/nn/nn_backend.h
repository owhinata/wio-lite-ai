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
 * Not here yet, on purpose: the runtime model-swap slots (load_region/reload).  The
 * donor firmware grew those later, for loading a .tflite off an SD card; this board
 * will load from the external NOR blob region instead, so the slots are added when
 * that shape is known rather than guessed at now.
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
};

/** The one backend selected at build time (provided by exactly one backend TU). */
extern const struct nn_backend_vt nn_backend_vt_selected;

#ifdef __cplusplus
}
#endif

#endif /* NN_BACKEND_H */
