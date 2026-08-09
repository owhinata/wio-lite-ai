# TensorFlow Lite Micro (tflite-micro) backend for CONFIG_NN_BACKEND=tflm (issue #9
# phase 2c).  Ported from the donor firmware ../stm32f746g-disco, which has been
# running this exact tree + CMSIS-NN pin on a Cortex-M7 since its issue #88.
#
# Included ONLY from the tflm branch of CMakeLists.txt, so C++ is enabled for that one
# build: the default (`null`) firmware never sees enable_language(CXX) and stays
# byte-identical.  All C++ is confined to the `tflm` STATIC library so the final link
# keeps the C driver (gcc) and does not auto-pull the full libstdc++ -- see
# nn_tflm_attach() at the bottom, where that is spelled out.
#
# The tflite-micro sources are NOT vendored and NOT a submodule.  At CONFIGURE time we
# fetch the pinned upstream commit and run its own create_tflm_tree.py, which drives
# tflite-micro's Makefile to list the library sources, download the header-only
# third-party dependencies (flatbuffers/gemmlowp/ruy) and -- with
# OPTIMIZED_KERNEL_DIR=cmsis_nn -- CMSIS_6 + CMSIS-NN at ITS OWN pinned SHAs.  The
# result is a self-contained, glob-able tree under the build directory, which we then
# compile with OUR flags, so the ABI (cortex-m7 / fpv5-d16 / hard-float /
# -fno-exceptions) is entirely ours.  This mirrors how this repository already
# downloads the ARM toolchain at configure time; only the tflm build pays the fetch
# cost, and a SHA-keyed stamp makes it a one-time cost.
#
# 🔴 lib/cmsis_core is NOT reused for the CMSIS-NN build.  The donor firmware recorded
# that a vendored CMSIS core/NN pair mismatched against the one tflite-micro expects
# produces legacy-API compile errors; letting upstream's own downloader pick the
# matched pair is the whole reason this works unattended.

enable_language(CXX)

# --- Pinned upstream ---------------------------------------------------------
set(TFLM_GIT_URL "https://github.com/tensorflow/tflite-micro.git")
set(TFLM_GIT_SHA "e142972d4f4382f77faa8212b7c70b37d28b9cb9")   # same pin as the donor
set(TFLM_SRC  "${CMAKE_BINARY_DIR}/tflm-src")     # shallow clone at the pinned SHA
set(TFLM_ROOT "${CMAKE_BINARY_DIR}/tflm-tree")    # generated self-contained tree
set(TFLM_GEN  "${TFLM_SRC}/tensorflow/lite/micro/tools/project_generation/create_tflm_tree.py")

# --- Knobs -------------------------------------------------------------------
# CMSIS-NN optimised kernels.  ON generates the tree with OPTIMIZED_KERNEL_DIR=cmsis_nn
# so the SIMD int8 kernels replace the reference ones; OFF keeps pure reference kernels
# for an `ai bench` A/B.  The choice is part of the tree stamp key, so flipping it
# regenerates.  The donor measured 622 ms vs 2,418 ms per BlazeFace inference at
# 216 MHz -- a 3.9x difference, which is why ON is the default.
set(NN_TFLM_CMSIS_NN ON CACHE BOOL "Use CMSIS-NN optimised kernels in the tflm backend")

# Which ops the resolver registers.  `blazeface` is the 8 ops BlazeFace-front 128 int8
# actually uses; `extended` adds the common-vision superset so another int8 model can
# be loaded into a blob slot without a rebuild.  This is a knob rather than a constant
# because the donor firmware measured the widening from 8 to 23 ops at +97,056 B of
# flash -- on a 384 KB partition that is a decision, not a detail.  Kept as a CONFIGURE
# option so widening the op set is a configure change, never a source edit.
set(NN_TFLM_OPS "blazeface" CACHE STRING "tflm op resolver set: blazeface | extended")
set_property(CACHE NN_TFLM_OPS PROPERTY STRINGS blazeface extended)
if(NOT NN_TFLM_OPS STREQUAL "blazeface" AND NOT NN_TFLM_OPS STREQUAL "extended")
    message(FATAL_ERROR "NN_TFLM_OPS must be 'blazeface' or 'extended'")
endif()

# Run the flatbuffers verifier over a model before handing it to the interpreter.
#
# 🔴 OFF by default, and it is worth being exact about what that gives up, because it
# is NOT "models are unchecked".  Three checks remain: the length, the "TFL3" file
# identifier, and the blob's CRC32 taken over the copy in PSRAM that is about to be
# interpreted.  What they cannot catch is a .tflite that was already truncated ON THE
# PC before `sb` read it -- an interrupted download, a converter still writing, a
# git-lfs pointer.  The blob region stores whatever arrived faithfully, so its CRC32
# matches a truncated file perfectly, and the "TFL3" identifier lives in the first
# eight bytes and survives any truncation.  Only a structural walk of the flatbuffer
# finds that, which is what this knob is.
#
# What it costs when it is missing is not a crash.  flatbuffers is zero-copy:
# GetModel() reinterprets the pointer and every field is reached by following offsets,
# so a truncated buffer sends those offsets past its end -- into the other model slot,
# the arena, or uninitialised PSRAM, all of which are mapped.  Sometimes that faults
# inside AllocateTensors() with no message (TF_LITE_STRIP_ERROR_STRINGS).  Sometimes it
# returns kTfLiteError and this backend reports NN_MODEL_ERR_ARENA -- "activations do
# not fit, or an unregistered operator" -- which is a LIE that sends the reader after
# the arena size instead of the file.  And sometimes it allocates, runs, and returns
# garbage detections with no error at all.  The last one is the real cost: not "it
# crashes", but "it runs and lies".
#
# 🔴 It is OFF because THE SAME CHECK IS AVAILABLE ON THE PC, FOR FREE, AND EARLIER.
# The `verify-model` target below builds scripts/verify_tflite.cc with the host
# compiler against this very tree, so it calls the identical VerifyModelBuffer() -- not
# an approximation of it -- and rejects the file before `blob write` has erased a slot
# or spent ~90 s on a YMODEM transfer.  It also names the missing OPERATOR when a model
# needs one this build did not register, which the board structurally cannot do:
# AllocateTensors() returns one kTfLiteError for that and for "the arena is too small",
# and TF_LITE_STRIP_ERROR_STRINGS leaves no message to separate them.
#
# So this knob is not a check that was dropped; it is a check that MOVED to where it is
# cheaper, earlier and more informative.  What it costs is that the check is no longer
# attached to the act of loading: a model reaching the NOR by some other route -- a
# different machine, a colleague, a script that skipped the step -- is not examined.
#
#   cmake --build build-tflm --target verify-model
#   ./build-tflm/verify_tflite model.tflite      # then blob write + sb
#
# 🔴 Turn it back ON for any build whose models will not pass through that command.
set(NN_TFLM_VERIFY OFF CACHE BOOL "Verify the flatbuffer before building an interpreter")

# Optimisation level for the vendored tree.
#
# -O2, because it was measured on the board and it is not free.  The rest of the
# firmware is -Os (issue #39); this library is the exception for the same reason
# coremark_obj keeps -O3 and cmd_membench.c keeps -O2 -- what it produces IS the
# deliverable.  Measured on board #2 with BlazeFace-front 128 int8, `lcd off`, 20 runs:
#
#     -O2   205.15 M cycles   373 ms   6.45 cyc/MACC
#     -Os   236.96 M cycles   431 ms   7.45 cyc/MACC     (+13.4 %)
#
# The plan predicted 5-15 % and hoped for "nearly free, because the arena is in
# external PSRAM and this is memory-bound".  Half right: the 470 KB arena does not fit
# the 16 KB D-cache, and against the donor firmware's 4.23 cyc/MACC on FMC SDRAM this
# part needs 1.53x the cycles even at -O2 -- so memory IS the dominant term.  But the
# remaining 1.16x is the optimiser, and 58 ms is not nothing.
#
# The flash it costs was found elsewhere instead: NN_TFLM_VERIFY moved to the host
# (above) and BSP_ENABLE_SD defaults OFF for this backend (CMakeLists.txt), which
# together give back far more than -Os did while taking nothing off the inference path.
#
# A STRING and not a boolean, because option() would silently turn it into ON/OFF --
# the same reasoning as BSP_OPT_LEVEL in CMakeLists.txt.
set(NN_TFLM_OPT_LEVEL "-O2" CACHE STRING "Optimisation level for the tflite-micro tree")

# Activation arena.  The donor firmware measured BlazeFace-front 128 int8 at
# arena_used_bytes() = 470,304 B; 512 KB is that plus headroom.  The number that
# matters at run time is the used figure, which `ai info` reports -- this is only the
# reservation.  It is a cache variable so a larger model can be tried without an edit:
# the carve-out has ~1.2 MB spare (see the layout note in nn_tflm_bufs.c).
set(NN_TFLM_ARENA_KB "512" CACHE STRING "tflm activation arena reservation, KB")

if(NN_TFLM_CMSIS_NN)
    set(TFLM_KERNEL_TAG "cmsisnn")
    set(TFLM_GEN_EXTRA "--makefile_options=OPTIMIZED_KERNEL_DIR=cmsis_nn")
else()
    set(TFLM_KERNEL_TAG "ref")
    set(TFLM_GEN_EXTRA "")
endif()
set(TFLM_STAMP "${TFLM_ROOT}/.tflm-generated-${TFLM_GIT_SHA}-${TFLM_KERNEL_TAG}")

# --- Python venv (numpy/Pillow are required by tflite-micro's Makefile) -------
# Created here if missing; the dependency INSTALL happens in the generation block below
# rather than being gated on venv creation, so a venv that predates this requirement,
# or one left half-installed by an interrupted run, is healed instead of skipped.
set(TFLM_VENV "${CMAKE_SOURCE_DIR}/.venv")
set(TFLM_PY   "${TFLM_VENV}/bin/python3")
if(NOT EXISTS "${TFLM_PY}")
    message(STATUS "tflm: creating ${TFLM_VENV}")
    execute_process(COMMAND "${Python3_EXECUTABLE}" -m venv "${TFLM_VENV}"
                    RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: python venv creation failed (need python3 + venv)")
    endif()
endif()

# --- Fetch + generate the tree (SHA-keyed stamp: once per pin) ----------------
# The stamp is written ONLY after a fully successful generation, and each attempt
# starts from a clean clone plus a fresh dependency install, so an interrupted run
# self-heals on the next configure.  That matters more than it sounds: upstream's
# third-party extractor SKIPS a download directory that merely exists, so a download
# interrupted half-way would never repair itself in place -- it would produce a tree
# that configures and then fails to compile, pointing at nothing in particular.
if(NOT EXISTS "${TFLM_STAMP}")
    message(STATUS "tflm: fetching tflite-micro @ ${TFLM_GIT_SHA} and generating the "
                   "source tree (one-time, a few minutes -- clones tflite-micro and "
                   "downloads flatbuffers/gemmlowp/ruy/CMSIS-NN)")

    execute_process(
        COMMAND "${TFLM_VENV}/bin/pip" install -q --disable-pip-version-check
                -r "${CMAKE_SOURCE_DIR}/requirements.txt"
        RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: pip install -r requirements.txt failed")
    endif()

    file(REMOVE_RECURSE "${TFLM_SRC}")
    file(MAKE_DIRECTORY "${TFLM_SRC}")
    execute_process(COMMAND git init -q WORKING_DIRECTORY "${TFLM_SRC}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: git init failed")
    endif()
    execute_process(COMMAND git remote add origin "${TFLM_GIT_URL}"
                    WORKING_DIRECTORY "${TFLM_SRC}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: git remote add failed")
    endif()
    execute_process(COMMAND git fetch --depth 1 -q origin "${TFLM_GIT_SHA}"
                    WORKING_DIRECTORY "${TFLM_SRC}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: git fetch ${TFLM_GIT_SHA} failed "
                            "(needs network and a reachable SHA)")
    endif()
    execute_process(COMMAND git checkout -q -f FETCH_HEAD
                    WORKING_DIRECTORY "${TFLM_SRC}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "tflm: git checkout FETCH_HEAD failed")
    endif()

    file(REMOVE_RECURSE "${TFLM_ROOT}")
    execute_process(
        # The venv goes on PATH, not just on the command line: make spawns its OWN
        # `python3` for generate_cc_arrays.py, so invoking create_tflm_tree.py with the
        # venv interpreter alone is not enough -- the subprocess would miss numpy.
        COMMAND "${CMAKE_COMMAND}" -E env "PATH=${TFLM_VENV}/bin:$ENV{PATH}"
                "${TFLM_PY}" "${TFLM_GEN}" -e hello_world ${TFLM_GEN_EXTRA} "${TFLM_ROOT}"
        WORKING_DIRECTORY "${TFLM_SRC}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0 OR NOT EXISTS "${TFLM_ROOT}/tensorflow/lite/micro/micro_interpreter.cc")
        message(FATAL_ERROR "tflm: create_tflm_tree.py failed "
                            "(needs numpy/Pillow in ${TFLM_VENV})")
    endif()
    file(WRITE "${TFLM_STAMP}" "${TFLM_GIT_SHA}\n")
endif()

# --- Sources -----------------------------------------------------------------
# create_tflm_tree emits only library sources under tensorflow/, but exclude the
# test-support cluster (helpers + mock/fake context) and any *_test.cc anyway.
# signal/*.cc are deliberately NOT compiled (no signal ops are registered); their
# headers stay on the include path because micro_mutable_op_resolver.h includes
# signal/micro/kernels/{irfft,rfft}.h.
file(GLOB_RECURSE TFLM_LIB_SOURCES CONFIGURE_DEPENDS
     "${TFLM_ROOT}/tensorflow/*.cc"
     "${TFLM_ROOT}/tensorflow/*.c")
list(FILTER TFLM_LIB_SOURCES EXCLUDE REGEX
     "(_test\\.cc|test_helpers\\.cc|test_helper_custom_ops\\.cc|mock_micro_graph\\.cc|fake_micro_context\\.cc)$")

# CMSIS-NN library sources (C), copied into the tree under third_party/cmsis_nn/Source.
# The optimised kernel WRAPPERS (tensorflow/lite/micro/kernels/cmsis_nn/*.cc) are
# already caught by the glob above -- create_tflm_tree lists them INSTEAD of the
# reference ones, so there is no duplicate registration.  The f16/f32 sources are
# dropped to match ARM_NN_ENABLE_F16=0 / F32=0 below.
if(NN_TFLM_CMSIS_NN)
    file(GLOB_RECURSE TFLM_CMSIS_NN_SOURCES CONFIGURE_DEPENDS
         "${TFLM_ROOT}/third_party/cmsis_nn/Source/*.c")
    list(FILTER TFLM_CMSIS_NN_SOURCES EXCLUDE REGEX "(_f16|_f32|arm_nntables_flt)\\.c$")
    list(APPEND TFLM_LIB_SOURCES ${TFLM_CMSIS_NN_SOURCES})
endif()

# 🔴 STATIC, not OBJECT, and that is a size decision rather than a style one.  Archive
# members are extracted ON DEMAND, so the ~15 kernels this build never registers --
# MicroMutableOpResolver<N> only instantiates the AddXxx() members that are actually
# called -- are never pulled into the link at all.  As an OBJECT library every object
# would be linked unconditionally, and worse, the linker script's
# KEEP(*(.init_array*)) would ANCHOR every kernel translation unit that has a static
# constructor, holding it against --gc-sections.  The donor firmware's +97,056 B for
# widening the op set is the scale of what that would cost.
add_library(tflm STATIC
    ${TFLM_LIB_SOURCES}
    "${CMAKE_SOURCE_DIR}/port/nn/tflm/cxx_runtime.cc"    # noexcept operator new/delete
    "${CMAKE_SOURCE_DIR}/port/nn/tflm/nn_tflm_bufs.c"    # arena + model slots (C, on purpose)
    "${CMAKE_SOURCE_DIR}/port/nn/tflm/nn_tflm.cc")       # extern "C" nn_backend_vt_selected

target_link_libraries(tflm PRIVATE bsp_iface)   # MCU_OPTS (fpv5-d16) + CMSIS/HAL includes

target_include_directories(tflm PRIVATE
    "${TFLM_ROOT}"                                   # "tensorflow/..." and "signal/..."
    "${TFLM_ROOT}/third_party/flatbuffers/include"   # "flatbuffers/..."
    "${TFLM_ROOT}/third_party/gemmlowp"              # "fixedpoint/..."
    "${TFLM_ROOT}/third_party/ruy"                   # "ruy/..."
    "${CMAKE_SOURCE_DIR}/port/nn")                   # nn.h / nn_backend.h for the bridge

# TF_LITE_STATIC_MEMORY changes TFLM's context/tensor struct layouts across the API
# boundary, so every translation unit that includes a TFLM header must agree -> PUBLIC.
# Stripping the error strings plus NDEBUG removes MicroPrintf and assert, which is what
# lets this build need no DebugLog implementation and no stdio at all.
target_compile_definitions(tflm
    PUBLIC  TF_LITE_STATIC_MEMORY
    PRIVATE TF_LITE_STRIP_ERROR_STRINGS NDEBUG)

# Our own knobs, consumed by nn_tflm.cc / nn_tflm_bufs.c.
target_compile_definitions(tflm PRIVATE
    NN_TFLM_ARENA_KB=${NN_TFLM_ARENA_KB}
    NN_TFLM_VERIFY=$<BOOL:${NN_TFLM_VERIFY}>
    $<$<STREQUAL:${NN_TFLM_OPS},extended>:NN_TFLM_OPS_EXTENDED=1>)

if(NN_TFLM_CMSIS_NN)
    # The CMSIS_6 core + CMSIS-NN include roots downloaded into the tree, mirroring the
    # tflm Makefile's own cmsis_nn.inc.  PRIVATE to this library, where no HAL or
    # repo-CMSIS translation unit lives, so the MATCHED pair of headers wins.
    target_include_directories(tflm PRIVATE
        "${TFLM_ROOT}/third_party/cmsis"
        "${TFLM_ROOT}/third_party/cmsis/CMSIS/Core/Include"
        "${TFLM_ROOT}/third_party/cmsis_nn"
        "${TFLM_ROOT}/third_party/cmsis_nn/Include")
    # 🔴 -DCMSIS_NN is not optional.  It is what OPTIMIZED_KERNEL_DIR=cmsis_nn adds in
    # the tflm Makefile (the uppercased directory name), and it switches the kernel
    # headers (conv.h, pooling.h, ...) from inline REFERENCE registrations to extern
    # declarations that the cmsis_nn *.cc then define.  Without it every optimised
    # kernel translation unit redefines the reference inline version and nothing links.
    target_compile_definitions(tflm PRIVATE
        CMSIS_NN ARM_NN_ENABLE_F16=0 ARM_NN_ENABLE_F32=0 NN_TFLM_CMSIS_NN=1)
endif()

# 🔴 C_STANDARD 11 is required, not cosmetic.  GCC 15 (this repository's pinned
# toolchain) defaults C to gnu23, and the CMSIS-NN .c sources in this tree are written
# against C11/C17.  The donor firmware never hit this: it was on GCC 13.3, where the
# default was still gnu17.
set_target_properties(tflm PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS OFF)

# Third-party code, isolated from the whole-program optimiser -- exactly the pattern
# coremark_obj (-O3 -funroll-loops) and cmd_membench.c (-O2 -fno-lto) already use in
# CMakeLists.txt.  -O2 rather than the firmware's -Os because this is the one part of
# the image whose speed IS the deliverable, and -fno-lto because C++ templates plus LTO
# plus a 384 KB partition is three risks interacting where two would do.  Mixing
# non-LTO objects into the -flto=auto link of `shell` is already proven here:
# coremark_obj is in the current image that way.
#
# -fno-strict-aliasing matches the firmware-wide flag for the same reason it exists
# there: flatbuffers reinterprets const uint8_t* as schema types and CMSIS-NN casts
# int8_t* to int32_t* to feed the SIMD path.  Both are the type punning whole-program
# alias analysis is entitled to miscompile.
#
# The C++-only flags are scoped with COMPILE_LANGUAGE:CXX because this target also
# compiles the CMSIS-NN C sources, which would warn "valid for C++ but not for C".
#
# NOT built with -Werror, deliberately: this is a vendored upstream tree, and a
# -Werror on it means a future toolchain bump breaks the build for reasons that have
# nothing to do with this firmware.  New warnings get silenced individually instead.
target_compile_options(tflm PRIVATE
    ${NN_TFLM_OPT_LEVEL} -fno-lto -fno-strict-aliasing -fno-delete-null-pointer-checks
    -fno-unwind-tables -fno-asynchronous-unwind-tables
    -Wno-unused-parameter -Wno-sign-compare -Wno-maybe-uninitialized
    # -fstack-usage writes a .su file next to every object.  The shell thread runs on a
    # 4,096 B stack and DTCM has under 8 KB free, so "the kernels fit" needs a
    # build-time answer and not just a `free` high-water reading after the fact.
    # cmake/report_tflm_stack.py sums the deepest frames after the build.
    -fstack-usage
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>)

# The nano C++ runtime archives plus libm, ordered AFTER tflm's own objects so that
# cxx_runtime.cc's operator new is resolved first and libstdc++_nano's THROWING one is
# never extracted (it would drag in __cxa_throw / _Unwind_* / std::bad_alloc type-info
# despite -fno-exceptions).  libm is explicit because the final link is forced onto the
# C driver, which -- unlike g++ -- does not link it automatically, and TFLM's
# QuantizeMultiplier calls round().
target_link_libraries(tflm PUBLIC stdc++_nano supc++_nano m)

# --- Host-side model checker (scripts/verify_tflite.cc) ----------------------
# Built with the HOST compiler, against the same fetched tree, so it runs the identical
# tflite::VerifyModelBuffer() the firmware would -- see the file's own header for why
# "identical" rather than "equivalent" is the whole point.
#
# It is a custom command driving the host compiler rather than an ordinary target
# because this project cross-compiles: every CMake target here is built for
# cortex-m7, and an executable that runs on the developer's machine cannot be one of
# them.  A one-file compile keeps that simple; there is nothing to link but libstdc++.
#
# `verify-model` is NOT part of ALL: it costs a few seconds to compile
# schema_generated.h, and most builds never need it.
find_program(HOST_CXX NAMES c++ g++ clang++)
if(HOST_CXX)
    # A plain variable, not a generator expression: an unset genex expands to an empty
    # ARGUMENT rather than to nothing, and the compiler then sees a literal "".
    set(_vt_ops_def "")
    if(NN_TFLM_OPS STREQUAL "extended")
        set(_vt_ops_def "-DNN_TFLM_OPS_EXTENDED=1")
    endif()
    add_custom_command(
        OUTPUT "${CMAKE_BINARY_DIR}/verify_tflite"
        COMMAND "${HOST_CXX}" -std=c++17 -O1 -w
                -I "${TFLM_ROOT}"
                -I "${TFLM_ROOT}/third_party/flatbuffers/include"
                -I "${TFLM_ROOT}/third_party/gemmlowp"
                -I "${TFLM_ROOT}/third_party/ruy"
                -I "${CMAKE_SOURCE_DIR}/port/nn/tflm"
                ${_vt_ops_def}
                "${CMAKE_SOURCE_DIR}/scripts/verify_tflite.cc"
                # GetBuiltinCode() is declared in a header and defined here; it is the
                # accessor that reads an operator code correctly across the schema's
                # deprecated/current field pair, so it is worth linking rather than
                # reimplementing.
                "${TFLM_ROOT}/tensorflow/compiler/mlir/lite/schema/schema_utils.cc"
                -o "${CMAKE_BINARY_DIR}/verify_tflite"
        DEPENDS "${CMAKE_SOURCE_DIR}/scripts/verify_tflite.cc"
                "${CMAKE_SOURCE_DIR}/port/nn/tflm/nn_tflm_ops.h"
        COMMENT "host c++ -> verify_tflite (checks a .tflite before you send it)"
        VERBATIM)
    add_custom_target(verify-model DEPENDS "${CMAKE_BINARY_DIR}/verify_tflite")

    # --- Host-side model rewriter (scripts/tflite_int8_input.cc) --------------
    # Issue #51.  Strips a model's own leading QUANTIZE so the graph input becomes the
    # quantized tensor -- which is the only way to exercise app/nn_camera.c's int8
    # ingest path, because "an int8 model" from the model zoo means int8 WEIGHTS and
    # float I/O.  See the file header.
    #
    # Same shape as verify-model above (host compiler, custom command, not in ALL) and
    # for the same reason: this project cross-compiles, so a developer-machine
    # executable cannot be an ordinary target here.  It does NOT include
    # nn_tflm_ops.h -- deciding whether a model's operators are registered belongs to
    # verify_tflite, and one question should have one answer in one place.
    add_custom_command(
        OUTPUT "${CMAKE_BINARY_DIR}/tflite_int8_input"
        COMMAND "${HOST_CXX}" -std=c++17 -O1 -w
                -I "${TFLM_ROOT}"
                -I "${TFLM_ROOT}/third_party/flatbuffers/include"
                # gemmlowp/ruy are reachable from micro_interpreter.h, which this tool
                # includes for TFLITE_SCHEMA_VERSION alone.
                -I "${TFLM_ROOT}/third_party/gemmlowp"
                -I "${TFLM_ROOT}/third_party/ruy"
                "${CMAKE_SOURCE_DIR}/scripts/tflite_int8_input.cc"
                # GetBuiltinCode() again: the accessor that reads an operator code
                # across the schema's deprecated/current field pair.  Linked rather
                # than reimplemented against the unpacked object, so both host tools
                # identify operators the same way.
                "${TFLM_ROOT}/tensorflow/compiler/mlir/lite/schema/schema_utils.cc"
                -o "${CMAKE_BINARY_DIR}/tflite_int8_input"
        DEPENDS "${CMAKE_SOURCE_DIR}/scripts/tflite_int8_input.cc"
        COMMENT "host c++ -> tflite_int8_input (rewrites a model to an int8 input)"
        VERBATIM)
    add_custom_target(int8-input-model DEPENDS "${CMAKE_BINARY_DIR}/tflite_int8_input")
else()
    message(STATUS "tflm: no host C++ compiler found -- the `verify-model` and "
                   "`int8-input-model` targets are unavailable (the firmware itself "
                   "is unaffected)")
endif()

# --- Attach to a firmware target ---------------------------------------------
# Called from CMakeLists.txt once the `shell` executable exists.  Everything the final
# link needs to know about C++ is in here, so CMakeLists.txt keeps one line.
function(nn_tflm_attach tgt)
    target_link_libraries(${tgt} PRIVATE tflm)

    # 🔴 LINKER_LANGUAGE C is required, not defensive.  CMake picks the link driver by
    # the highest CMAKE_<LANG>_LINKER_PREFERENCE among the target's sources AND the
    # languages of the libraries it links -- CXX is 30, C is 10, and it PROPAGATES out
    # of a linked static library.  Left alone, `shell` would be linked with g++, which
    # implicitly links the full libstdc++ (exceptions, unwinder and all) on top of the
    # nano archives we asked for.  The image would grow by tens of KB for code that
    # -fno-exceptions guarantees can never run.
    set_target_properties(${tgt} PROPERTIES LINKER_LANGUAGE C)

    # nn.c's extern reference already demands nn_backend_vt_selected, which is what
    # extracts nn_tflm.cc.o from the archive.  Naming it as an undefined symbol makes
    # that independent of how the LTO plugin orders its symbol resolution against
    # archive member extraction, which is the one part of this link that is not plain
    # ELF on both sides.
    target_link_options(${tgt} PRIVATE -Wl,-u,nn_backend_vt_selected)
endfunction()
