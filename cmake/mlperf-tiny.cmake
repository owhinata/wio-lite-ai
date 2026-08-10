# ============================================================================
#  MLPerf Tiny v1.4 benchmark harness (issue #55)
#
#  Included from CMakeLists.txt when CONFIG_MLPERF_TINY is ON.  Builds upstream's
#  shared harness half and ours into one object library, which the `shell` target
#  links; the `mlperf` shell command lives in SHELL_SOURCES like any other.
# ============================================================================

# --- The upstream mirror ------------------------------------------------------
# NOT in the top-level submodule bootstrap, and that is deliberate: the mirror is
# ~340 MB (it carries every reference model and the training scripts the accuracy
# datasets are generated from), and no default build has any use for it.  Fetched
# here, shallow, only once someone has asked for the benchmark.
set(MLPERF_DIR "${CMAKE_SOURCE_DIR}/lib/mlperf-tiny")
set(MLPERF_API "${MLPERF_DIR}/benchmark/api")

if(NOT EXISTS "${MLPERF_API}/internally_implemented.cpp")
    message(STATUS "Fetching the MLPerf Tiny submodule (~340 MB, shallow) ...")
    execute_process(
        COMMAND git submodule update --init --depth 1 -- lib/mlperf-tiny
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _mlperf_sm)
    if(NOT _mlperf_sm EQUAL 0 OR NOT EXISTS "${MLPERF_API}/internally_implemented.cpp")
        message(FATAL_ERROR
            "Could not fetch lib/mlperf-tiny.  Run it by hand:\n"
            "  git submodule update --init --depth 1 -- lib/mlperf-tiny")
    endif()
endif()

# --- The harness object library ----------------------------------------------
# 🔴 A LIBRARY OF ITS OWN, NOT TWO MORE ENTRIES IN SHELL_SOURCES.  Both sources are
# C++, and `shell` is compiled without any of the flags that keep C++ from dragging a
# runtime into a bare-metal image -- it has never needed them, because until now every
# C++ translation unit in this firmware lived inside the `tflm` library.  Adding these
# to `shell` directly would produce the one C++ object in the build with exceptions,
# RTTI and thread-safe statics all still enabled.  Same flags as
# cmake/tflite-micro.cmake, for the same reasons, and cmake/check_cxx_runtime.py keeps
# the result honest after every link.
add_library(mlperf_obj OBJECT
    "${MLPERF_API}/internally_implemented.cpp"
    "${CMAKE_SOURCE_DIR}/port/mlperf/mlperf_th.cc")

target_link_libraries(mlperf_obj PUBLIC bsp_iface)

target_include_directories(mlperf_obj PRIVATE
    "${MLPERF_DIR}/benchmark"            # upstream includes itself as "api/..."
    "${CMAKE_SOURCE_DIR}/port/mlperf"
    "${CMAKE_SOURCE_DIR}/port/nn"        # nn.h: the model this harness runs
    "${CMAKE_SOURCE_DIR}/port/threadx"   # tx_glue.h: the microsecond clock
    "${CMAKE_SOURCE_DIR}/shell/include"  # cli.h: where protocol bytes go
    "${CMAKE_SOURCE_DIR}/svc"            # fmt.h: the printf that has no %f
    "${TX_DIR}/common/inc"               # cli.h -> cli_config.h -> ThreadX types
    "${TX_PORT}/inc")

# No TH_MODEL_VERSION here: it is defined in port/mlperf/mlperf_th.h, next to the
# function it calls, and reaches upstream through the -include below.  CMake cannot
# pass it as a -D at all -- it drops function-style definitions with a warning.
#
# TH_VENDOR_NAME_STRING is deliberately left at upstream's "unspecified".  It is
# defined unconditionally in submitter_implemented.h, so a -D would only produce a
# redefinition warning and lose -- and it is a display string in the host's log.

target_compile_options(mlperf_obj PRIVATE
    ${BSP_OPT_LEVEL} -fno-lto
    -fno-unwind-tables -fno-asynchronous-unwind-tables
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>)

# Inject our declarations into UPSTREAM'S translation unit only.  It has to see a
# prototype for mlperf_model_id() before the macro above is used, and the file is a
# read-only mirror -- so the declaration is pushed in from the command line rather
# than by editing it.  mlperf_th.cc includes the same header itself.
set_source_files_properties("${MLPERF_API}/internally_implemented.cpp" PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_SOURCE_DIR}/port/mlperf/mlperf_th.h")
