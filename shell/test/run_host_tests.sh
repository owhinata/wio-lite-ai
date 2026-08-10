#!/usr/bin/env sh
#
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 ThreadX Shell Project
#
# Host smoke/unit tests for the Shell core, built and run with the host gcc.
# The Shell core (shell/core, shell/include, shell/backend) and svc/fmt are a
# verbatim port from stm32f746g-disco, so these HW-independent tests run
# unchanged.  Each test links shell/test/host_sections.ld, which supplies the
# .shell_root_cmds section + boundary symbols that the target ldscript
# (ldscript/STM32H725AEIx_IROM.ld) provides on hardware.  No firmware build is
# involved -- this runs on the build host, not the board.
#
# svc/ymodem.c came over with issue #19 M4 (the RTL8720DN flash backup streams
# over the console with YMODEM), so its test is ported too.  (The donor's
# frame_pipeline test covers a camera module that has no counterpart here.)
set -eu

here=$(cd "$(dirname "$0")" && pwd)
inc="$here/../include"
core="$here/../core"
svc="$here/../../svc"       # freestanding service layer (fmt.c / fmt.h)
backend="$here/../backend"
fdb="$here/../../lib/flashdb"   # third-party FlashDB (its CRC-32 is used by app/blob.c)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

# Flags mirror the target link so the tests exercise the real retention path:
#   -ffunction-sections -fdata-sections + -Wl,--gc-sections : same GC as the
#       firmware; proves `used` + linker KEEP keep the (otherwise unreferenced)
#       command entries from being garbage-collected.
#   -no-pie : resolve the const command/pointer table absolutely at link time so
#       it stays read-only without runtime text relocations (target firmware is
#       linked absolute/static, so this only matters on the host).
CFLAGS="-std=c11 -Wall -Wextra -ffunction-sections -fdata-sections -no-pie"
LDFLAGS="-Wl,--gc-sections -Wl,-T,$here/host_sections.ld"

# command registration foundation.
gcc $CFLAGS -I "$inc" \
    "$here/test_registration.c" \
    $LDFLAGS -o "$out/test_registration"
"$out/test_registration"

# command-line parser.  cli_parse.c and the test share one compile so the small
# CLI_MAX_ARGC / CLI_MAX_SUBCMD_DEPTH overrides (used to exercise the token-limit
# and nesting-limit paths with a compact tree) apply consistently.
gcc $CFLAGS -DCLI_MAX_ARGC=8 -DCLI_MAX_SUBCMD_DEPTH=2 \
    -I "$inc" -I "$core" \
    "$here/test_parse.c" "$core/cli_parse.c" \
    $LDFLAGS -o "$out/test_parse"
"$out/test_parse"

# Shared host pieces for the core / output / integration tests: the dummy
# (loopback) backend and the ThreadX-free glue (no-op lock/notify, a faithful
# cli_tx_send_blocking over tr->api->write, and the RX pump).  Found via the test
# dir (host_glue.h) and the backend dir (cli_backend_dummy.h).
glue="$backend/cli_backend_dummy.c $here/host_glue.c"
glue_inc="-I $here -I $backend"

# shell core: ASCII filter, RX state machine, dispatch, fail-safe.  cli_session.c /
# cli_edit.c are ThreadX-free (the tx_* glue lives in cli_core.c, firmware only),
# so they build on the host against the tx_api.h shim in test/shim, placed first on
# the include path.  Output + tx_* glue route through the shared dummy backend.
gcc $CFLAGS -DCLI_CMD_BUFFER_SIZE=16 -DCLI_MAX_ARGC=4 -DCLI_MAX_SUBCMD_DEPTH=2 \
    -DCLI_USE_COLOR=0 \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_core.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_core"
"$out/test_core"

# output API: minimal formatter, 32 B staging + autoflush, VT100 colour, hexdump,
# TX-failure drop/return.  cli_printf.c is ThreadX-free; colour ON (default) and
# the real 32 B CLI_PRINTF_BUFFER_SIZE so the SGR escapes and autoflush are
# exercised.  cli_session.c is linked for the cancel helpers.
gcc $CFLAGS \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_output.c" "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_session.c" \
    $glue \
    $LDFLAGS -o "$out/test_output"
"$out/test_output"

# dummy backend end-to-end: input -> execute -> output driven THROUGH the transport
# (cli_dummy_inject -> read() -> state machine -> write() -> capture), flow control
# (backpressure completes / timeout drops / immediate fail), abnormal cases and
# multi-instance isolation.  Small CLI_* limits + colour OFF as for the core test.
gcc $CFLAGS -DCLI_CMD_BUFFER_SIZE=16 -DCLI_MAX_ARGC=4 -DCLI_MAX_SUBCMD_DEPTH=2 \
    -DCLI_USE_COLOR=0 \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_integration.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_integration"
"$out/test_integration"

# line editor: cursor model (cur split from len), in-line insert/overwrite/delete,
# meta keys (Ctrl+a/b/d/e/f/k/u/w, Alt+b/f, Ctrl+l), VT100 escapes (arrows / Home /
# End / Del / Insert / SS3), invalid-escape ignore, the CPR width probe + guarded
# reply, and wrap redraw at a forced small term_width.  Drives cli_input_byte
# directly (model assertions) so it needs no backend.  Colour OFF.
gcc $CFLAGS -DCLI_USE_COLOR=0 \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_edit.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_edit"
"$out/test_edit"

# command history fixed ring: add + recall (arrows / Ctrl+p,n), consecutive-
# duplicate suppression, non-consecutive duplicates kept, FIFO eviction at the byte
# cap, empty lines skipped, navigation-state reset on submit / Ctrl+C / blank
# re-submit, no-draft-restore, and per-instance isolation.  A small 32 B
# CLI_HISTORY_BUFFER_SIZE forces eviction with a few short entries; colour OFF.
gcc $CFLAGS -DCLI_USE_COLOR=0 -DCLI_HISTORY_BUFFER_SIZE=32 \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_history.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_history"
"$out/test_history"

# UART backend byte ring: the pure, lock-free FIFO helpers (cli_uart_ring.h) that
# the ring-buffered backends layer RX/TX on.  HAL/ThreadX-free, so it builds with
# the host gcc and needs no shim -- only the backend include dir for the header.
gcc $CFLAGS -I "$backend" \
    "$here/test_uart_ring.c" \
    $LDFLAGS -o "$out/test_uart_ring"
"$out/test_uart_ring"

# Tab completion: word boundary + read-only token walk (command-set resolution),
# prefix scan with longest-common-prefix tracking, single-candidate complete +
# trailing space, bash-style two-stage candidate list, BEL on no match / argument
# territory, and the buffer-full guard.  Drives cli_input_byte (Tab=0x09) +
# cli_tab_complete directly and asserts the model + captured output.  Colour OFF.
gcc $CFLAGS -DCLI_USE_COLOR=0 \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_complete.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_complete"
"$out/test_complete"

# Tab completion (buffer-full): same as above but a tiny CLI_CMD_BUFFER_SIZE so
# completion that would overflow the line rings BEL and leaves the line unchanged,
# and an LCP-extend that cannot fit still reaches the two-stage list on the next Tab.
gcc $CFLAGS -DCLI_USE_COLOR=0 -DCLI_CMD_BUFFER_SIZE=8 -DTEST_COMPLETE_SMALL_BUF \
    $glue_inc -I "$here/shim" -I "$inc" -I "$core" -I "$svc" \
    "$here/test_complete.c" "$core/cli_session.c" "$core/cli_edit.c" "$core/cli_history.c" \
    "$core/cli_printf.c" "$svc/fmt.c" "$core/cli_parse.c" "$core/cli_complete.c" \
    $glue \
    $LDFLAGS -o "$out/test_complete_smallbuf"
"$out/test_complete_smallbuf"

# #19 M4 -- clean-room YMODEM-CRC sender (svc/ymodem.c): CRC-16/CCITT vectors, block
# framing (block 0 name+size, STX/SOH, 0x1A short-block padding, seq/~seq, CRC),
# NAK resend, CAN abort + teardown, seq wrap mod 256, and a 1-byte-at-a-time
# source filling full blocks.  Pure svc layer -- HAL/ThreadX/shell-free, so it
# builds with the host gcc and needs only the svc include dir for the header.
gcc $CFLAGS -I "$svc" \
    "$here/test_ymodem.c" "$svc/ymodem.c" \
    $LDFLAGS -o "$out/test_ymodem"
"$out/test_ymodem"

# #19 M5 -- the YMODEM RECEIVER (ymodem_recv), which is what lets the board take a
# firmware image FROM the PC (and therefore what makes the stock backup restorable).
# Two harnesses: a pthread duplex loopback that runs ymodem_send() and ymodem_recv()
# against each other through blocking FIFOs -- the only way to exercise the real
# handshake, since the sender only advances on the receiver's 'C'/ACKs -- and scripted
# transcripts for the error paths (CRC damage, duplicate/out-of-order blocks, a sink
# that refuses the file, a batch that never closes, short-block trimming).
gcc $CFLAGS -I "$svc" \
    "$here/test_ymodem_recv.c" "$svc/ymodem.c" \
    $LDFLAGS -pthread -o "$out/test_ymodem_recv"
"$out/test_ymodem_recv"

# issue #10 (#9 P2b) -- the CRC-32 the blob region stamps assets with.  app/blob.c
# does not implement one: it reuses FlashDB's fdb_calc_crc32(), which is already in
# the build and already accumulating.  Two properties it relies on are invisible in
# that function's signature -- that starting from 0 gives standard CRC-32/ISO-HDLC
# (so the board and the PC agree) and that feeding the result back in continues the
# same CRC (so a file arriving as ~185 YMODEM blocks lands on the one-call value).
# Pinned here because it is the only part of the blob work verifiable off the board.
# Built against port/flashdb (fdb_cfg.h) exactly as the firmware is; the FlashDB
# sources are third-party, hence the one relaxed warning.
gcc $CFLAGS -Wno-unused-parameter \
    -I "$here/../../port/flashdb" -I "$fdb/inc" -I "$fdb/port/fal/inc" \
    "$here/test_crc32.c" "$fdb/src/fdb_utils.c" \
    $LDFLAGS -o "$out/test_crc32"
"$out/test_crc32"

# issue #8 phase 3a -- camera frame pipeline core (svc/frame_pipeline.c): ring slot
# acquire/publish, refcount pin/put, DROP/LATEST policy + pending transfer, detach
# in-flight count, read_latest generation, and an N=4 ring cycling under a counting
# sink.  Pure svc layer -- HAL/ThreadX/shell-free, and the mutual exclusion it needs
# is injected (struct frame_os), so the whole engine runs on the host with a no-op
# lock.  This is the only part of the camera work that can be verified without the
# board, which is exactly why it is ported byte-identical from the f746 firmware.
gcc $CFLAGS -I "$svc" \
    "$here/test_frame_pipeline.c" "$svc/frame_pipeline.c" \
    $LDFLAGS -o "$out/test_frame_pipeline"
"$out/test_frame_pipeline"

# issue #9 phase 3 -- the BlazeFace decoder (port/nn/models/blazeface.c): SSD anchor
# decode + NMS + the score-threshold knob.  This is the one piece of new arithmetic
# whose failure is silent -- a wrong anchor scale or an off-by-512 into the second
# anchor group draws a plausible rectangle in the wrong place, and on the board that
# is indistinguishable from bad exposure or a wrong normalization.  Here the expected
# box is computed by hand.  The decoder depends on nn.h alone (no HAL, no ThreadX, no
# libm), and struct nn_model is opaque, so the test supplies its own nn_output_count()
# / nn_output() and runs the real decoder unmodified.  Built against the REAL
# include/mem_sections.h so the PSRAM_AI attribute on the host is the same one the
# firmware uses -- a shimmed copy could drift from it without anything noticing.
gcc $CFLAGS -I "$here/../../include" -I "$here/../../port/nn" \
    -I "$here/../../port/nn/models" \
    "$here/test_blazeface.c" "$here/../../port/nn/models/blazeface.c" \
    $LDFLAGS -lm -o "$out/test_blazeface"
"$out/test_blazeface"

# issue #55 -- the MLPerf Tiny harness (port/mlperf/mlperf_th.cc), driven through
# UPSTREAM'S OWN PARSER (lib/mlperf-tiny/benchmark/api/internally_implemented.cpp,
# unmodified).  So what is under test is the protocol itself, fed the way the host's
# runner feeds it -- `db load N`, 31-byte hex chunks, `infer N W` -- and what is
# asserted is the bytes the board would put on the wire.
#
# It earns its place because all three things this layer can get wrong are SILENT on
# hardware: the per-benchmark input transform (shift by 128 / pass through / quantize
# from float) still produces confident scores when it is wrong, the three-decimal
# formatting is assembled from integers because svc/fmt.c has no %f, and the benchmark
# identification is what decides which test the host runs at all.  None of them
# announce themselves; they come back as accuracy that is quietly a few points low.
#
# Skipped rather than failed when the submodule is absent: it is ~340 MB and only
# fetched for CONFIG_MLPERF_TINY builds, so a plain checkout must still run the suite.
# g++ links it -- upstream's half is C++ and declares no linkage, which is the whole
# reason mlperf_th is C++ too (see its header).
mlperf="$here/../../lib/mlperf-tiny/benchmark"
if [ -f "$mlperf/api/internally_implemented.cpp" ]; then
    gcc $CFLAGS -c -I "$svc" "$svc/fmt.c" -o "$out/fmt.o"
    gcc $CFLAGS -I "$here/../../port/nn" -I "$here/../../port/mlperf" \
        -c "$here/test_mlperf.c" -o "$out/test_mlperf.o"
    # -std=gnu++17 to match the firmware, which passes no -std and so gets the GNU
    # dialect by default: the test should compile the shared headers the same way the
    # board does.  (This is also where cli_config.h's C11 _Static_assert was caught --
    # GCC only accepts that spelling in C++ from version 14, the ARM toolchain is 15
    # and this host's g++ is 13.  The header now uses CLI_STATIC_ASSERT and works in
    # both, but the dialect match is what made the difference visible.)
    g++ -std=gnu++17 -Wall -Wextra -ffunction-sections -fdata-sections -no-pie \
        -I "$mlperf" -I "$here/../../port/mlperf" -I "$here/../../port/nn" \
        -I "$inc" -I "$svc" -I "$here/../../port/threadx" \
        -fno-exceptions -fno-rtti \
        -include "$here/../../port/mlperf/mlperf_th.h" \
        "$mlperf/api/internally_implemented.cpp" \
        "$here/../../port/mlperf/mlperf_th.cc" \
        "$out/test_mlperf.o" "$out/fmt.o" \
        -Wl,--gc-sections -lm -o "$out/test_mlperf"
    "$out/test_mlperf"
else
    echo "test_mlperf: SKIP (lib/mlperf-tiny not checked out)"
fi

echo "host tests passed"
