/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_tflm_priv.h
 * @brief   Interfaces shared between the three files of the tflm backend (issue #9 P2c).
 *
 * Private to port/nn/tflm/.  Nothing outside this directory includes it: the rest of
 * the firmware sees the backend only through the nn_backend_vt in nn_backend.h.
 *
 * The two large buffers are declared here and DEFINED IN C (nn_tflm_bufs.c) even
 * though their only user is C++.  That is deliberate, and it took two review rounds
 * to arrive at:
 *
 *   - cmake/check_psram_ai_residency.py matches carve-out residents by exact name
 *     (after stripping GCC's clone suffixes).  A C++ file-scope `static` may be
 *     emitted as `_ZL13nn_tflm_arena`, which that gate would not recognise -- it would
 *     report "no such object in the image" and fail the build for the wrong reason, or
 *     worse, a rename could leave a real buffer unguarded.
 *
 *   - Adding `extern "C"` to a C++ definition does not fix it either.  A linkage
 *     specification is treated as containing `extern` ([dcl.link]), so
 *     `extern "C" uint8_t buf[N];` at namespace scope is a DECLARATION, not a
 *     definition, and no symbol is emitted.  The link still succeeds (the array is
 *     tentatively defined elsewhere or simply unreferenced in a way that resolves),
 *     and only the gate notices, with a message that points nowhere useful.
 *
 * Keeping the definitions in a C translation unit removes both questions: the symbol
 * names and the section attribute behave exactly like the carve-out's existing C
 * residents (nn_null.c's tensors, membench's row), and the gate needs no special case.
 *
 * The wider lesson from this phase: introducing C++ is not risky because of its
 * syntax.  It is risky because this firmware's safety net is a set of checks written
 * in C's terms -- linker-script ASSERTs that name symbols, nm-based gates that match
 * names -- and every one of them fails OPEN when the name it is looking for changes
 * shape.  The cheapest way to keep them working is to leave what they inspect in C.
 */
#ifndef NN_TFLM_PRIV_H
#define NN_TFLM_PRIV_H

#include <stdint.h>

/* Activation arena reservation.  Set from CMake (NN_TFLM_ARENA_KB); the fallback keeps
 * this header meaningful when read on its own.  The donor firmware measured
 * arena_used_bytes() = 470,304 B for BlazeFace-front 128 int8, so 512 KB is that plus
 * headroom -- and `ai info` reports the USED figure, which is the one that means
 * anything.  A reservation is not a measurement. */
#ifndef NN_TFLM_ARENA_KB
#define NN_TFLM_ARENA_KB 512
#endif
#define NN_TFLM_ARENA_BYTES  ((uint32_t)NN_TFLM_ARENA_KB * 1024u)

/* Two model slots, double-buffered.  Sized to swallow a WHOLE blob payload
 * (BLOB_PAYLOAD_MAX = 520,192 B, app/blob.h) so `ai model load` can never be refused
 * for a reason the operator cannot see from `blob list`: any blob that fits a slot on
 * the NOR fits a slot here.  512 KB = 524,288 B leaves 4,096 B of margin over that.
 *
 * Both numbers doubled in issue #55, and they had to move together: MLPerf Tiny's
 * IC02 is a 512,024 B flatbuffer, which is what set the blob slot size, and a staging
 * buffer smaller than a blob slot would reintroduce exactly the "refused for an
 * invisible reason" failure this sizing rule exists to prevent.
 *
 * TWO of them, not one, because a reload must not be able to destroy the model that is
 * currently running.  load_region() hands out the INACTIVE slot, so the flatbuffer the
 * live interpreter still holds pointers into is untouched while the new one is read
 * from the NOR -- and if the new model turns out to be unusable, the old interpreter
 * can be rebuilt from memory that was never overwritten.  With a single slot, a
 * corrupt .tflite would take the working model down with it. */
#define NN_TFLM_MODEL_SLOTS  2u
#define NN_TFLM_MODEL_MAX    (512u * 1024u)

/*
 * Carve-out budget (.psram_ai is 2 MB, 0x90600000..0x90800000):
 *     nn_tflm_arena        512 KB
 *     nn_tflm_model_buf  2x512 KB   (issue #55: was 2x256 KB)
 *     psram_ai_bench_buf    64 KB   (pre-existing, shell/cmds/cmd_membench.c)
 *     ----------------------------
 *                        1,600 KB of 2,048 KB   -> ~448 KB still free
 *
 * The linker script ASSERTs the total fits (_psram_ai_end <= 0x90800000);
 * cmake/check_psram_ai_residency.py checks that these are the objects actually in
 * there.  Nothing costs flash -- the section is NOLOAD -- so what the extra megabyte
 * spends is carve-out address space, and the ASSERT is what says whether there is any.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** TFLM's activation arena.  Defined in nn_tflm_bufs.c -- see the header note. */
extern uint8_t nn_tflm_arena[NN_TFLM_ARENA_BYTES];

/** The two .tflite staging slots.  Defined in nn_tflm_bufs.c. */
extern uint8_t nn_tflm_model_buf[NN_TFLM_MODEL_SLOTS][NN_TFLM_MODEL_MAX];

/**
 * Every call our global `operator new` has served (port/nn/tflm/cxx_runtime.cc).
 *
 * 🔴 BE PRECISE ABOUT WHAT THIS PROVES, because it is easy to overstate.  In the
 * current build `operator new` is not in the linked image at all: with
 * TF_LITE_STATIC_MEMORY nothing references it, so --gc-sections drops it, and
 * cmake/check_cxx_runtime.py reports that after every link.  The link-time absence is
 * the real evidence -- far stronger than any run could give -- and it means this
 * counter is 0 by construction rather than by observation.  `ai info` printing 0 is
 * therefore NOT a measurement of this build's behaviour.
 *
 * It is still worth having, as a tripwire that is currently inert.  The moment some
 * future path does allocate, `operator new` becomes reachable, gets linked, and the
 * counter starts meaning something -- and `ai info` shows it without anyone having to
 * remember to look.  On this firmware such an allocation would come out of the newlib
 * heap in AXI-SRAM, which is shared with everything else and was never sized for it.
 */
extern volatile uint32_t nn_tflm_cxx_new_calls;

#ifdef __cplusplus
}
#endif

#endif /* NN_TFLM_PRIV_H */
