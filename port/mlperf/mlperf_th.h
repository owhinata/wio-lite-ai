/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    mlperf_th.h
 * @brief   The MLPerf Tiny / EEMBC harness bound to this board (issue #55).
 *
 * MLPerf Tiny splits its device firmware in two.  `lib/mlperf-tiny/benchmark/api/`
 * holds the parts every submitter shares -- the serial command parser, the `db` input
 * buffer, the timing brackets around a batch of inferences -- and calls out to a set
 * of `th_*` functions each submitter writes for its own hardware.  This directory is
 * that half.  The upstream half is a read-only submodule and is not edited.
 *
 * ONE FIRMWARE, ALL THE BENCHMARKS.  Upstream's reference submissions compile the
 * model into the image and answer the host's `profile` query with a compile-time
 * string, so each benchmark is a separate build.  This board already loads its model
 * at run time out of a NOR blob slot (issue #9), so the benchmark identity is a
 * function of WHICH MODEL IS OPEN, and mlperf_model_id() answers from the tensor
 * shapes.  That is what -DTH_MODEL_VERSION=mlperf_model_id() exploits: upstream
 * writes `th_printf("m-model-[%s]\r\n", TH_MODEL_VERSION)`, and a macro that expands
 * to a function call satisfies %s just as well as a literal does.  No upstream edit.
 *
 * Layering: this is a port/ module.  It sits on port/nn (the model), shell/ (the
 * console it writes protocol bytes to) and port/threadx (the microsecond clock).
 * The shell command that drives it is shell/cmds/cmd_mlperf.c; the guards, the raw
 * console takeover and the operator-facing messages all live there, because none of
 * them are the harness's business.
 *
 * NOT REENTRANT, and deliberately so: there is one model, one console can hold the
 * raw byte stream, and the EEMBC protocol is a strict request/response with a single
 * outstanding command.  cmd_mlperf.c holds the NN session for the whole monitor
 * session, which is what makes that true.
 */
#ifndef MLPERF_TH_H
#define MLPERF_TH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cli_instance;
struct nn_model;

/**
 * EEMBC model id of the bound model -- "ic01" | "kws01" | "vww01" | "ad01" -- or
 * "none" when nothing is bound.  Never NULL: it is expanded into a %s by upstream's
 * parser (see the header note), which has no way to check.
 *
 * ic02 (MLPerf's larger ResNet) reports "ic01".  Its input and output shapes are
 * identical to ic01's, so nothing on the device can tell them apart -- and upstream
 * does not either: benchmark/runner/tests_performance.yaml gives its ic02 entry
 * `model: ic01`.  Which of the two ran is a fact about the host's test selection and
 * the blob slot, not about the firmware.
 */
const char *mlperf_model_id(void);

/*
 * The hook that makes the paragraph above work.
 *
 * upstream's api/submitter_implemented.h opens with
 *     #ifndef TH_MODEL_VERSION
 *     #error "Please set TH_MODEL_VERSION to one of the EE_MODEL_VERSION_* defines"
 * because it expects a per-benchmark build to name its model.  We name a FUNCTION, and
 * upstream's only use of the macro is as a %s argument, so a `const char *`-valued
 * expression satisfies it exactly as a string literal would.
 *
 * It lives here, rather than on the compiler command line, for two reasons: CMake
 * refuses to pass function-style -D macros at all (it drops them with a warning), and
 * a macro whose expansion is a call belongs next to the declaration of what it calls.
 * cmake/mlperf-tiny.cmake force-includes this header into upstream's translation unit,
 * which is what gets both the macro and the prototype in front of it -- without
 * editing a read-only mirror.
 */
#ifndef TH_MODEL_VERSION
#define TH_MODEL_VERSION  mlperf_model_id()
#endif

/* Why mlperf_bind() refused. */
#define MLPERF_ERR_ARG     (-1)   /**< NULL argument                              */
#define MLPERF_ERR_SHAPE   (-2)   /**< I/O shape matches no MLPerf Tiny benchmark */
#define MLPERF_ERR_DTYPE   (-3)   /**< recognised shape, but not the int8 I/O the
                                       benchmark is defined in terms of           */
#define MLPERF_ERR_TODO    (-4)   /**< recognised, pre/post not implemented yet   */

/**
 * Bind the harness to @p sh (where protocol bytes go) and @p m (the open model).
 * Returns 0, or one of MLPERF_ERR_* -- in which case nothing is bound and the caller
 * must not enter the monitor.
 *
 * REFUSING BEFORE THE MONITOR STARTS IS THE POINT.  Once the host is driving, the
 * only channel back is the protocol, and "the DUT answered but the numbers are wrong"
 * is the failure mode this whole layer is built to avoid.  A model we cannot feed
 * correctly has to be rejected while there is still a human reading the console.
 */
int  mlperf_bind(struct cli_instance *sh, struct nn_model *m);
void mlperf_unbind(void);

/** One short sentence for an mlperf_bind() return code.  Never NULL. */
const char *mlperf_strerror(int rc);

/**
 * Emit upstream's start-up banner (`m-timestamp-mode-...`, a timestamp,
 * `m-init-done`, `m-ready`).  Call once after binding and after the console has gone
 * raw -- the host's first command expects a monitor that has already announced
 * itself.  Thin wrapper over ee_benchmark_initialize() so cmd_mlperf.c needs no
 * upstream header.
 */
void mlperf_monitor_start(void);

/**
 * Feed one received byte to upstream's command assembler.  A command is dispatched
 * synchronously, from this call, when the terminator arrives -- so this returns only
 * after a whole `infer` batch has run.  Thin wrapper over ee_serial_callback().
 */
void mlperf_feed(char c);

/**
 * Local, NON-PROTOCOL counters for the summary cmd_mlperf.c prints after the monitor
 * exits.  These are deliberately not on the wire: the host parses lines, and a line
 * it does not recognise is at best noise in its log.
 *
 * @c cycles is the DWT total across every th_infer(), which is a far finer
 * measurement than the microsecond timestamps the protocol carries -- but it is OURS,
 * not MLPerf's, and it is not what a published score would be.
 */
struct mlperf_stats {
	uint32_t inferences;    /**< th_infer() calls, warm-up included            */
	uint64_t cycles;        /**< DWT core cycles summed over those             */
	uint32_t infer_errors;  /**< nn_run() failures (each also reported on wire) */
	uint32_t short_loads;   /**< `db` buffers smaller than the input tensor     */
	uint32_t truncated;     /**< th_printf lines that did not fit the line buf  */
	uint32_t tx_errors;     /**< cli_write refusals (console backpressure)      */
};
const struct mlperf_stats *mlperf_get_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* MLPERF_TH_H */
