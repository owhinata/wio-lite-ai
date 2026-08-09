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
 * extended one at +97,056 B.  Keep NN_TFLM_OPS=blazeface unless a model needs more.
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
 * The common-vision superset, added by -DNN_TFLM_OPS=extended so a different int8
 * model can be dropped into a blob slot without rebuilding the firmware.
 */
#define NN_TFLM_OPS_EXTRA(X)                            \
	X(FULLY_CONNECTED,        AddFullyConnected)         \
	X(AVERAGE_POOL_2D,        AddAveragePool2D)          \
	X(SOFTMAX,                AddSoftmax)                \
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
#define NN_TFLM_OPS_BASE_COUNT  (0 NN_TFLM_OPS_BASE(NN_TFLM_OPS_COUNT_ONE))
#define NN_TFLM_OPS_EXTRA_COUNT (0 NN_TFLM_OPS_EXTRA(NN_TFLM_OPS_COUNT_ONE))

#if defined(NN_TFLM_OPS_EXTENDED)
#define NN_TFLM_OPS_ALL(X)  NN_TFLM_OPS_BASE(X) NN_TFLM_OPS_EXTRA(X)
#define NN_TFLM_OPS_TOTAL   (NN_TFLM_OPS_BASE_COUNT + NN_TFLM_OPS_EXTRA_COUNT)
#else
#define NN_TFLM_OPS_ALL(X)  NN_TFLM_OPS_BASE(X)
#define NN_TFLM_OPS_TOTAL   NN_TFLM_OPS_BASE_COUNT
#endif

#endif /* NN_TFLM_OPS_H */
