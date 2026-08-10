/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_tflm_ops.h
 * @brief   The operator set the tflm backend registers -- stated exactly once (#9 P2c).
 *
 * 🔴 THIS FILE EXISTS SO THE BOARD AND THE PC CANNOT DISAGREE.  Two things consume the
 * list: nn_tflm.cc, which registers the operators into the MicroMutableOpResolver, and
 * scripts/verify_tflite.cc, the host-side checker that tells you BEFORE a transfer
 * whether a model stays inside that set.  If those two lists were written out
 * separately they would drift, and a drifted checker is worse than no checker: it
 * reports "this model will run" about a firmware that never registered the operator.
 *
 * An X-macro rather than a table because the two users need different things from each
 * entry -- the firmware needs the resolver METHOD, the host tool needs the schema
 * ENUM -- and a macro is the only way to give each what it needs from one definition.
 *
 * Each entry is X(schema_enum_suffix, resolver_method):
 *   schema_enum_suffix  appended to tflite::BuiltinOperator_ (schema_generated.h)
 *   resolver_method     the MicroMutableOpResolver<N>::AddXxx() member
 *
 * Adding an operator here is not free: it is compiled kernel code in a 384 KB
 * partition.  The donor firmware measured the widening from the base set to the
 * extended one at +97,056 B, and on THIS firmware `extended` does not fit at all --
 * measured at issue #55: the link fails with FLASH overflowed by 29,200 B.  That is
 * the whole reason the `mlperf` profile below exists instead of just using `extended`.
 * Keep NN_TFLM_OPS=blazeface unless a model needs more.
 */
#ifndef NN_TFLM_OPS_H
#define NN_TFLM_OPS_H

/*
 * The eight operators BlazeFace-front 128 int8 actually uses.  Always registered.
 */
#define NN_TFLM_OPS_BASE(X)             \
	X(QUANTIZE,           AddQuantize)          \
	X(CONV_2D,            AddConv2D)            \
	X(DEPTHWISE_CONV_2D,  AddDepthwiseConv2D)   \
	X(ADD,                AddAdd)               \
	X(PAD,                AddPad)               \
	X(MAX_POOL_2D,        AddMaxPool2D)         \
	X(RESHAPE,            AddReshape)           \
	X(DEQUANTIZE,         AddDequantize)

/*
 * Exactly what MLPerf Tiny v1.4 needs on top of the base set, added by
 * -DNN_TFLM_OPS=mlperf (issue #55).
 *
 * 🔴 A MEASURED SET, NOT A GENEROUS ONE, AND THAT IS THE POINT.  All five v1.4
 * models -- ic01 (ResNet), ic02 (larger ResNet), kws01 (DS-CNN), vww01 (MobileNet)
 * and ad01 (deep autoencoder) -- were run through scripts/verify_tflite.cc against
 * the base set, and the union of everything they were missing is the first three
 * entries below.  `extended` would also cover them, and `extended` does not link
 * (see above), so the difference between measuring the requirement and assuming it
 * is the difference between a firmware that exists and one that does not.
 *
 * Measured cost of this profile over the base set: +23,624 B of text.  With the
 * MLPerf harness on top (CONFIG_MLPERF_TINY=ON) the image is 377,976 B of 393,216,
 * leaving 15,240 B.  If that runs out, the next thing to give up is PAD and
 * MAX_POOL_2D from the base set (worth 6,248 B) -- no MLPerf model uses either, and
 * only BlazeFace (issue #9) would stop running.
 *
 * 🔴 "ONE MORE OPERATOR" IS NOT A UNIT OF FLASH, so do not budget by counting them.
 * Adding MEAN and LOGISTIC to this profile was measured at ZERO bytes: their kernels
 * are already in the image because TFLM's shared operator code reaches them and
 * --gc-sections cannot drop them, so registering them buys only a resolver table
 * entry that vanishes into alignment.  What costs is a kernel nothing else already
 * pulled in.  Measure the profile you actually intend to ship.
 */
#define NN_TFLM_OPS_MLPERF(X)                   \
	X(FULLY_CONNECTED,    AddFullyConnected)    \
	X(AVERAGE_POOL_2D,    AddAveragePool2D)     \
	X(SOFTMAX,            AddSoftmax)

/*
 * The common-vision superset, added by -DNN_TFLM_OPS=extended so a different int8
 * model can be dropped into a blob slot without rebuilding the firmware.
 *
 * ⚠ This profile currently OVERFLOWS the app partition (issue #55).  It is kept
 * because it is the honest name for "everything a vision model might want" and
 * because the partition may not always be this full -- but it is not a working
 * configuration today, and `mlperf` is what you want instead.
 */
#define NN_TFLM_OPS_EXTRA(X)                            \
	X(LOGISTIC,               AddLogistic)               \
	X(RELU,                   AddRelu)                   \
	X(RELU6,                  AddRelu6)                  \
	X(MUL,                    AddMul)                    \
	X(SUB,                    AddSub)                    \
	X(MEAN,                   AddMean)                   \
	X(CONCATENATION,          AddConcatenation)          \
	X(RESIZE_BILINEAR,        AddResizeBilinear)         \
	X(RESIZE_NEAREST_NEIGHBOR, AddResizeNearestNeighbor) \
	X(STRIDED_SLICE,          AddStridedSlice)           \
	X(LEAKY_RELU,             AddLeakyRelu)              \
	X(TRANSPOSE,              AddTranspose)

/* Count the entries so MicroMutableOpResolver<N> is sized from the list itself and
 * cannot fall behind it.  (A resolver whose N is too small fails at registration, which
 * this backend treats as fatal -- so the failure would at least be loud, but sizing it
 * by hand is still one more thing to forget.) */
#define NN_TFLM_OPS_COUNT_ONE(e, m) +1
#define NN_TFLM_OPS_BASE_COUNT   (0 NN_TFLM_OPS_BASE(NN_TFLM_OPS_COUNT_ONE))
#define NN_TFLM_OPS_MLPERF_COUNT (0 NN_TFLM_OPS_MLPERF(NN_TFLM_OPS_COUNT_ONE))
#define NN_TFLM_OPS_EXTRA_COUNT  (0 NN_TFLM_OPS_EXTRA(NN_TFLM_OPS_COUNT_ONE))

/*
 * The three profiles are nested, not alternative: `extended` is a superset of
 * `mlperf`, which is a superset of `blazeface`.  NN_TFLM_OPS_EXTRA therefore does NOT
 * repeat the three MLPerf operators -- registering an operator twice is a resolver
 * error, and this is the only place that could introduce one.
 */
#if defined(NN_TFLM_OPS_EXTENDED)
#define NN_TFLM_OPS_ALL(X)  NN_TFLM_OPS_BASE(X) NN_TFLM_OPS_MLPERF(X) NN_TFLM_OPS_EXTRA(X)
#define NN_TFLM_OPS_TOTAL   (NN_TFLM_OPS_BASE_COUNT + NN_TFLM_OPS_MLPERF_COUNT + \
                             NN_TFLM_OPS_EXTRA_COUNT)
#elif defined(NN_TFLM_OPS_MLPERF_PROFILE)
#define NN_TFLM_OPS_ALL(X)  NN_TFLM_OPS_BASE(X) NN_TFLM_OPS_MLPERF(X)
#define NN_TFLM_OPS_TOTAL   (NN_TFLM_OPS_BASE_COUNT + NN_TFLM_OPS_MLPERF_COUNT)
#else
#define NN_TFLM_OPS_ALL(X)  NN_TFLM_OPS_BASE(X)
#define NN_TFLM_OPS_TOTAL   NN_TFLM_OPS_BASE_COUNT
#endif

#endif /* NN_TFLM_OPS_H */
