/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_tflm_bufs.c
 * @brief   The tflm backend's carve-out residents, defined in C (issue #9 P2c).
 *
 * THE POINT OF THIS FILE IS THAT IT IS C.  Both buffers are used only from C++
 * (nn_tflm.cc), and putting their definitions there would be the obvious thing to do.
 * It would also make them invisible to the post-link gate that guards the cacheable
 * PSRAM carve-out -- for two independent reasons, either of which is enough.  The
 * argument is written out once, in nn_tflm_priv.h; this file is what it produces.
 *
 * Both live in .psram_ai, the top 2 MB of the OCTOSPI1 window that app/mpu.c region 3
 * maps Normal write-back cacheable (issue #9 phase 2a).  That is the right home for
 * exactly the reason the rest of the firmware avoids it: this is bulk, CPU-ONLY
 * working set.  No bus master may ever touch either buffer -- not the DMA that fills a
 * camera frame, not the SDMMC IDMA -- because in this window a master's write is
 * coherent at the bus and stale in the D-cache, intermittently, only under cache
 * pressure.  The model bytes arrive here by CPU copy out of the NOR driver's indirect
 * reads, and the arena is never anything but CPU-touched.
 *
 * NOLOAD, so nothing zeroes them at reset: neither buffer may assume it starts as
 * zeros.  Neither does.  The arena is planned and written by TFLM's own allocator
 * before any kernel reads it, and a model slot is only ever read over the exact byte
 * range that was just written into it and CRC-checked.
 */
#include "nn_tflm_priv.h"

#include "mem_sections.h"   /* PSRAM_AI: the cacheable, CPU-only carve-out */

/* 32-byte aligned: one D-cache line on this part.  In a cacheable region that is not
 * cosmetic -- it keeps the arena and a model slot from sharing a line, so a write to
 * one cannot dirty a line the other half occupies. */
uint8_t nn_tflm_arena[NN_TFLM_ARENA_BYTES] PSRAM_AI __attribute__((aligned(32)));

/* Also 32, which subsumes the 16-byte alignment tflite::GetModel() needs to read the
 * flatbuffer's root table in place. */
uint8_t nn_tflm_model_buf[NN_TFLM_MODEL_SLOTS][NN_TFLM_MODEL_MAX]
	PSRAM_AI __attribute__((aligned(32)));
