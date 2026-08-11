# ============================================================================
#  Wio Lite AI (STM32H725AEI6) firmware.
#
#  Firmwares:
#    boot  -- standalone USB DFU bootloader (see boot/README.md).  Runs from
#             internal flash 0x08000000 (replaces TinyUF2); flashed via ST-Link
#             (mode=UR).  Produces build/boot.{elf,bin,hex} (no UF2).
#    blink -- LED blink reference app.  Runs from the internal flash app
#             partition 0x08020000; load it with
#             `dfu-util -d 0483:df11 -a 0 -D build/blink.bin` while `boot` is in
#             DFU mode (convenience target: `dfu-blink`).
#
#  Configure (downloads the ARM toolchain into ./tools on first run):
#    cmake -B build -G Ninja \
#          -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
#  Build:  cmake --build build    # -> build/boot.{elf,bin,hex}, build/blink.{elf,bin,hex}
#  Flash boot to 0x08000000 (see boot/README.md):
#    STM32_Programmer_CLI -c port=SWD mode=UR -w build/boot.bin 0x08000000 -v
# ============================================================================
cmake_minimum_required(VERSION 3.20)

# Fetch HAL/CMSIS/TinyUSB/CoreMark submodules if any is missing.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32h7xx_hal_driver/Src/stm32h7xx_hal.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_core/Include/core_cm7.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_h7/Include/stm32h725xx.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/src/tusb.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/coremark/core_main.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/netxduo/common/src/nx_ip_create.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/flashdb/src/fdb_kvdb.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/filex/common/src/fx_media_open.c")
    message(STATUS "Initializing git submodules ...")
    execute_process(
        COMMAND git submodule update --init
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        RESULT_VARIABLE _sm)
    if(NOT _sm EQUAL 0)
        message(FATAL_ERROR "git submodule update failed")
    endif()
endif()

project(wio-lite-ai C ASM)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# REQUIRED, not optional: cmake/check_itcm_residency.py is a correctness gate (issue
# #39), and a gate that quietly skips itself when its interpreter is missing is worse
# than no gate at all -- it reports the same "nothing to see here" as a passing run.
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ============================================================================
#  Common build setup (shared by the bootloader and the apps)
# ============================================================================
# --- Target / common build options -----------------------------------------
# Cortex-M7 with the double-precision FPU (fpv5-d16) on the STM32H725.
set(MCU_OPTS
    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

set(HAL_DIR    "${CMAKE_SOURCE_DIR}/lib/stm32h7xx_hal_driver")
set(CMSIS_DEV  "${CMAKE_SOURCE_DIR}/lib/cmsis_device_h7")
set(CMSIS_CORE "${CMAKE_SOURCE_DIR}/lib/cmsis_core")
set(LDSCRIPT_APP "${CMAKE_SOURCE_DIR}/ldscript/STM32H725AEIx_IROM.ld") # app (internal 0x08020000)
set(LDSCRIPT_ROM "${CMAKE_SOURCE_DIR}/ldscript/STM32H725AEIx_ROM.ld")  # standalone (0x08000000)

# stm32h7xx_hal_conf.h from the upstream template (its default HSE_VALUE of
# 25 MHz already matches the board crystal).
set(GEN_DIR "${CMAKE_BINARY_DIR}/gen")
configure_file("${HAL_DIR}/Inc/stm32h7xx_hal_conf_template.h"
               "${GEN_DIR}/stm32h7xx_hal_conf.h" COPYONLY)

# Usage requirements shared by the target (includes, defs, MCU flags, link).
add_library(bsp_iface INTERFACE)
target_include_directories(bsp_iface INTERFACE
    "${CMAKE_SOURCE_DIR}/include"
    "${GEN_DIR}"
    "${HAL_DIR}/Inc"
    "${CMSIS_DEV}/Include"
    "${CMSIS_CORE}/Include")
target_compile_definitions(bsp_iface INTERFACE USE_HAL_DRIVER STM32H725xx)
target_compile_options(bsp_iface INTERFACE
    ${MCU_OPTS} -Wall -fdata-sections -ffunction-sections -g -gdwarf-2)
target_link_options(bsp_iface INTERFACE
    ${MCU_OPTS} -specs=nano.specs -specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage)   # ldscript (-T) is added per firmware

# Compile every HAL driver source; each file self-guards on its
# HAL_<module>_MODULE_ENABLED define, so unused ones become empty objects.
file(GLOB HAL_SOURCES "${HAL_DIR}/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX "_template\\.c$")

# ============================================================================
#  Bootloader: boot  (internal flash 0x08000000, flashed via ST-Link mode=UR)
# ============================================================================
# Standalone bootloader (boot): linked at 0x08000000, flashed via ST-Link (no UF2).
function(rom_finalize tgt)
    target_link_options(${tgt} PRIVATE "-T${LDSCRIPT_ROM}" -Wl,-Map=${tgt}.map,--cref)
    set_target_properties(${tgt} PROPERTIES LINK_DEPENDS "${LDSCRIPT_ROM}")
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary -S $<TARGET_FILE:${tgt}> ${tgt}.bin
        COMMAND "${CMAKE_OBJCOPY}" -O ihex   $<TARGET_FILE:${tgt}> ${tgt}.hex
        COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:${tgt}>
        BYPRODUCTS ${tgt}.bin ${tgt}.hex
        COMMENT "objcopy -> ${tgt}.bin/.hex (flash to 0x08000000 with ST-Link mode=UR)")
endfunction()

# Replaces the stock TinyUF2 at internal flash 0x08000000 (LDSCRIPT_ROM): it
# configures the clock tree itself (stock CMSIS system file resets to HSI, then
# SystemClock_Config), enumerates as a composite DFU + CDC device, and programs
# downloads into the internal app partition at 0x08020000 (sectors 1-3, issue
# #25).  Flashed with the ST-Link (mode=UR), not via UF2.  TinyUSB and its
# config/descriptors are compiled into ONLY this target (blink stays a pure
# bare-metal app).
set(TUSB_DIR "${CMAKE_SOURCE_DIR}/lib/tinyusb/src")

# Device-mode subset of TinyUSB: core + FIFO + device stack + DFU + CDC classes +
# the Synopsys dwc2 device controller (the H725's USB1_OTG_HS is a dwc2 core).
set(TINYUSB_SOURCES
    "${TUSB_DIR}/tusb.c"
    "${TUSB_DIR}/common/tusb_fifo.c"
    "${TUSB_DIR}/device/usbd.c"
    "${TUSB_DIR}/class/dfu/dfu_device.c"
    "${TUSB_DIR}/class/cdc/cdc_device.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dcd_dwc2.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dwc2_common.c")

add_executable(boot
    "${CMAKE_SOURCE_DIR}/boot/main.c"
    "${CMAKE_SOURCE_DIR}/boot/clock.c"
    "${CMAKE_SOURCE_DIR}/boot/iflash.c"
    "${CMAKE_SOURCE_DIR}/boot/usb_descriptors.c"
    "${CMAKE_SOURCE_DIR}/boot/dfu_callbacks.c"
    "${CMSIS_DEV}/Source/Templates/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${TINYUSB_SOURCES}
    ${HAL_SOURCES})
target_link_libraries(boot PRIVATE bsp_iface)
# boot/ first so TinyUSB picks up our tusb_config.h; then the TinyUSB headers.
target_include_directories(boot PRIVATE
    "${CMAKE_SOURCE_DIR}/boot"
    "${TUSB_DIR}")
# Selects the H7 dwc2 port inside TinyUSB (OTG_HS core, internal FS PHY on H725).
target_compile_definitions(boot PRIVATE CFG_TUSB_MCU=OPT_MCU_STM32H7)
target_compile_options(boot PRIVATE -O2)

rom_finalize(boot)

# ============================================================================
#  Apps: blink and the ThreadX shell
#  Internal flash app partition 0x08020000 (sectors 1-3, issue #25); flashed over
#  DFU (dfu-util / dfu-<tgt>)
# ============================================================================
# --- Post-build artifacts + DFU flash target -------------------------------
# App firmwares (blink, shell): linked at 0x08020000, emitted as raw .bin (what the
# DFU bootloader programs over dfu-util) + .hex.  Overflowing the 384 KB partition
# is a link error -- it cannot silently grow into the bootloader's sector.
function(firmware_finalize tgt)
    target_link_options(${tgt} PRIVATE "-T${LDSCRIPT_APP}" -Wl,-Map=${tgt}.map,--cref)
    set_target_properties(${tgt} PROPERTIES LINK_DEPENDS "${LDSCRIPT_APP}")
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary -S $<TARGET_FILE:${tgt}> ${tgt}.bin
        COMMAND "${CMAKE_OBJCOPY}" -O ihex   $<TARGET_FILE:${tgt}> ${tgt}.hex
        COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:${tgt}>
        BYPRODUCTS ${tgt}.bin ${tgt}.hex
        COMMENT "objcopy -> ${tgt}.bin/.hex")

    # `dfu-<tgt>`: program the app to the internal partition over the custom DFU
    # bootloader.  Put the board in DFU mode first (hold USER/PF1 at reset -- the
    # red LED stays on).  Named per firmware so app targets never collide on one
    # `dfu` target.  (TinyUF2 and its .uf2 drive are gone: the bootloader that
    # replaced it takes a raw .bin over dfu-util, so no UF2 is produced.)
    add_custom_target(dfu-${tgt}
        COMMAND dfu-util -d 0483:df11 -a 0 -D ${tgt}.bin
        DEPENDS ${tgt}
        USES_TERMINAL
        COMMENT "dfu-util -> ${tgt}.bin to internal flash 0x08020000 (hold PF1 at reset for DFU mode)")
endfunction()

add_executable(blink
    "${CMAKE_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${HAL_SOURCES})
target_link_libraries(blink PRIVATE bsp_iface)
target_compile_options(blink PRIVATE -O2)

firmware_finalize(blink)

# --- shell: interactive Eclipse ThreadX CLI console -------------------------
# App at 0x08020000 (firmware_finalize -> dfu-shell): ThreadX + an interactive
# CLI over USB CDC.  Reuses src/system_stm32h7xx.c (clock-free SystemInit) so the
# app inherits the bootloader's 550 MHz clock tree.  ThreadX core
# (common/src) + Cortex-M7/GNU port asm; the app supplies its own
# _tx_initialize_low_level + shared SysTick handler in port/threadx/tx_glue.c.
# ONE executable compiled uniformly (TX_INCLUDE_USER_DEFINE_FILE + port/threadx on
# the include path) so the ThreadX core, the shell core and the app agree on the
# TX_THREAD layout (ABI).
set(TX_DIR  "${CMAKE_SOURCE_DIR}/lib/threadx")
set(TX_PORT "${TX_DIR}/ports/cortex_m7/gnu")
file(GLOB TX_CORE "${TX_DIR}/common/src/*.c")
list(FILTER TX_CORE EXCLUDE REGEX "tx_misra\\.c$")
file(GLOB TX_ASM  "${TX_PORT}/src/*.S")
list(FILTER TX_ASM  EXCLUDE REGEX "tx_misra\\.S$")
# Execution Profile Kit (issue #2: `thread` cpu%).  It lives under utility/, not
# common/src, so the TX_CORE glob above does not pick it up -- add it explicitly.
set(TX_EPK "${TX_DIR}/utility/execution_profile_kit/tx_execution_profile.c")

# ---- NetX Duo (issue #23 U3) ------------------------------------------------
# The host's own TCP/IP stack.  Its "Ethernet MAC" is app/nx_link_driver.c, which puts
# raw frames on the RTL8720 link's DATA channel -- there is no MAC and no DMA here.
# Compiled into the same single executable as everything else, so every translation
# unit that sees NX_PACKET agrees on its layout (NX_INCLUDE_USER_DEFINE_FILE +
# port/netxduo on the include path), the same ABI argument as ThreadX above.
# The whole common/src glob is taken: the IPv6 sources compile to empty objects
# because nx_user.h defines NX_DISABLE_IPV6, and --gc-sections drops the rest.
set(NX_DIR "${CMAKE_SOURCE_DIR}/lib/netxduo")
file(GLOB NX_CORE "${NX_DIR}/common/src/*.c")
list(FILTER NX_CORE EXCLUDE REGEX "nx_ram_network_driver")
# DHCP is the only addon: `net dhcp` needs a client once the module's is out of the way.
set(NX_ADDONS "${NX_DIR}/addons/dhcp/nxd_dhcp_client.c")

# CDC-only TinyUSB device subset (the boot set minus the DFU class).
set(APP_TINYUSB_SOURCES
    "${TUSB_DIR}/tusb.c"
    "${TUSB_DIR}/common/tusb_fifo.c"
    "${TUSB_DIR}/device/usbd.c"
    "${TUSB_DIR}/class/cdc/cdc_device.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dcd_dwc2.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dwc2_common.c")

# Shell core (ThreadX-aware; cli_core.c is the only tx_* caller) + the USB CDC and
# dummy backends + the ports of the HW-independent commands + the clean-room printf.
set(SHELL_SOURCES
    "${CMAKE_SOURCE_DIR}/shell/core/cli_core.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_complete.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_edit.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_history.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_job.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_parse.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_printf.c"
    "${CMAKE_SOURCE_DIR}/shell/core/cli_session.c"
    "${CMAKE_SOURCE_DIR}/shell/backend/cli_backend_usbcdc.c"
    "${CMAKE_SOURCE_DIR}/shell/backend/cli_backend_dummy.c"
    "${CMAKE_SOURCE_DIR}/shell/backend/cli_backend_log.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_builtin.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_system.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_thread.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_free.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_sleep.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_watch.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_jobs.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_devmem.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_psram.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_dmesg.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_crash.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_wdt.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_coremark.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_membench.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_wifi.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_wifi_flash.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_net.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_wifi_link.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_xfer.c"
    "${CMAKE_SOURCE_DIR}/svc/fmt.c"
    "${CMAKE_SOURCE_DIR}/svc/timebase.c"
    "${CMAKE_SOURCE_DIR}/svc/ymodem.c"
    # issue #8 phase 3a: the camera frame ring + sink dispatch.  Nothing calls it
    # yet (phase 3b wires the DCMI producer to it); it is here so the engine and
    # its host tests land before the hardware work that depends on them.
    # --gc-sections drops it from the image until it has a caller.
    "${CMAKE_SOURCE_DIR}/svc/frame_pipeline.c"
    "${CMAKE_SOURCE_DIR}/svc/gfx_rot.c"
    "${CMAKE_SOURCE_DIR}/svc/log.c")

# --- CoreMark object library (run as the shell `coremark` command) ----------
# Built once at -O3 -funroll-loops and linked into the shell firmware below;
# cmd_coremark.c calls coremark_main().  core_main.c is compiled with
# -Dmain=coremark_main so its main() does not clash with the app main() in
# app/main.c.  MEM_METHOD=MEM_MALLOC (issue #14) malloc's the TOTAL_DATA_SIZE (2 KB)
# working set from the newlib heap at each run's start and frees it at the end (off
# both the shell thread stack and permanent .bss); cmd_coremark.c pre-flights the
# alloc.  ITERATIONS=0 -> CoreMark auto-calibrates the run time.
set(CMK_DIR "${CMAKE_SOURCE_DIR}/lib/coremark")
add_library(coremark_obj OBJECT
    "${CMK_DIR}/core_list_join.c"
    "${CMK_DIR}/core_main.c"
    "${CMK_DIR}/core_matrix.c"
    "${CMK_DIR}/core_state.c"
    "${CMK_DIR}/core_util.c"
    "${CMAKE_SOURCE_DIR}/port/coremark/core_portme.c")
target_link_libraries(coremark_obj PUBLIC bsp_iface)   # HAL_GetTick for timing
target_include_directories(coremark_obj PRIVATE
    "${CMK_DIR}" "${CMAKE_SOURCE_DIR}/port/coremark")
target_compile_definitions(coremark_obj PRIVATE
    ITERATIONS=0 MEM_METHOD=MEM_MALLOC MEM_LOCATION="HEAP")
target_compile_options(coremark_obj PRIVATE -O3 -funroll-loops)
# Rename the benchmark entry so it does not collide with the app main().
set_source_files_properties("${CMK_DIR}/core_main.c" PROPERTIES
    COMPILE_DEFINITIONS "main=coremark_main")

# Generate cli_version.h (CLI_FW_NAME / _VERSION / _GIT_DESC) into GEN_DIR (already
# on the bsp_iface include path) so cmd_system.c can #include "cli_version.h".
execute_process(
    COMMAND git -C "${CMAKE_SOURCE_DIR}" describe --always --dirty --tags
    OUTPUT_VARIABLE CLI_GIT_DESC OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _gitrc)
if(NOT _gitrc EQUAL 0)
    set(CLI_GIT_DESC "unknown")
endif()
configure_file("${CMAKE_SOURCE_DIR}/cmake/cli_version.h.in"
               "${GEN_DIR}/cli_version.h" @ONLY)

# RTL8720DN download flashloader stub (issue #19 M2): the on-device flasher
# (app/rtl8720_flash.c) uploads this Realtek AmebaD ROM flashloader into the module's
# SRAM.  It is a Realtek-proprietary blob, so it is NOT committed -- fetch it at
# configure time into the build tree and embed it into GEN_DIR from there.
#
# issue #58: this used to read _ref/ambd/, which made a plain `git clone` fail to
# configure -- _ref/ is a git-ignored dump of local reference material (datasheets,
# vendor SDKs, flash backups), not a build input.  Nothing outside build/ is written
# now, so `rm -rf build` really does return the tree to a clean checkout.
#
# The pin is a commit, not a branch: a branch would let the blob change underneath us,
# and this one is uploaded into the module's SRAM and then executed.  EXPECTED_HASH
# both verifies the download and makes CMake skip it when the cached copy already
# matches, so re-configuring an existing build tree does not touch the network.
set(RTL_FLASHLOADER_URL
    "https://raw.githubusercontent.com/Seeed-Studio/ambd_flash_tool/de56db9716f9cbf5641fe6e440b1c96033c41962/imgtool_flashloader_amebad.bin")
set(RTL_FLASHLOADER_SHA256
    "9307121385cb390dfd2da64da2c6c515f17b5a9556b3d04021487c9b9f220b55")
# Offline / air-gapped escape hatch: point this at any local copy of the 4688 B blob.
# It is hash-checked exactly like the download -- a wrong file here would be uploaded
# to the module and run, so "the user supplied it" is not a reason to trust it.
set(RTL_FLASHLOADER_BIN "" CACHE FILEPATH
    "Local imgtool_flashloader_amebad.bin (offline builds); empty = fetch into build/")

if(RTL_FLASHLOADER_BIN)
    if(NOT EXISTS "${RTL_FLASHLOADER_BIN}")
        message(FATAL_ERROR "RTL_FLASHLOADER_BIN=${RTL_FLASHLOADER_BIN} does not exist")
    endif()
    file(SHA256 "${RTL_FLASHLOADER_BIN}" _RTL_FL_GOT)
    if(NOT _RTL_FL_GOT STREQUAL RTL_FLASHLOADER_SHA256)
        message(FATAL_ERROR
            "RTL_FLASHLOADER_BIN=${RTL_FLASHLOADER_BIN} is not the AmebaD flashloader\n"
            "  expected SHA256 ${RTL_FLASHLOADER_SHA256}\n"
            "  got      SHA256 ${_RTL_FL_GOT}")
    endif()
    set(_RTL_FL_FILE "${RTL_FLASHLOADER_BIN}")
else()
    set(_RTL_FL_FILE "${CMAKE_BINARY_DIR}/ambd/imgtool_flashloader_amebad.bin")
    file(DOWNLOAD "${RTL_FLASHLOADER_URL}" "${_RTL_FL_FILE}"
         EXPECTED_HASH "SHA256=${RTL_FLASHLOADER_SHA256}"
         TLS_VERIFY ON
         STATUS _RTL_FL_DL)
    list(GET _RTL_FL_DL 0 _RTL_FL_DL_CODE)
    if(NOT _RTL_FL_DL_CODE EQUAL 0)
        file(REMOVE "${_RTL_FL_FILE}")
        message(FATAL_ERROR
            "Could not fetch the AmebaD flashloader stub: ${_RTL_FL_DL}\n"
            "  ${RTL_FLASHLOADER_URL}\n"
            "  For an offline build, pass -DRTL_FLASHLOADER_BIN=/path/to/"
            "imgtool_flashloader_amebad.bin\n"
            "  (4688 B, load 0x00082000; it also ships inside ameba_d_tools-1.0.4 at\n"
            "  bsp/image/, and in github.com/openshwprojects/SharpRTL872xTool -- issue #19).")
    endif()
endif()

file(READ "${_RTL_FL_FILE}" _RTL_FL_HEX HEX)
string(LENGTH "${_RTL_FL_HEX}" _RTL_FL_HEXLEN)
math(EXPR _RTL_FL_LEN "${_RTL_FL_HEXLEN} / 2")
string(REGEX REPLACE "(..)" "0x\\1," _RTL_FL_ARRAY "${_RTL_FL_HEX}")
file(WRITE "${GEN_DIR}/rtl8720_flashloader_data.c"
"/* AUTO-GENERATED at configure time from ${_RTL_FL_FILE}.\n"
" * Do NOT edit and do NOT commit -- it is a Realtek-proprietary blob (issue #19 M2). */\n"
"#include <stdint.h>\n"
"const uint8_t rtl8720_flashloader[] = { ${_RTL_FL_ARRAY} };\n"
"const uint32_t rtl8720_flashloader_len = ${_RTL_FL_LEN};\n")

# IWDG watchdog (issue #4): ON builds in the IWDG1 arming + priority-5 petter thread
# + the `wdt` starve test; OFF compiles them out entirely (`wdt info` then reports
# "disabled (build)").  Turn OFF for SWD sessions that must hold a breakpoint longer
# than the ~3 s IWDG timeout (the app does not touch DBGMCU to freeze it under debug).
option(BSP_ENABLE_IWDG "Enable the IWDG watchdog and wdt shell command" ON)
# issue #39: the shell firmware is built for SIZE, not speed.  The app partition is
# 384 KB and the image had reached 82.5% of it with issue #9 (on-device AI) not even
# started; -Os plus link-time optimisation gives back ~54 KB without touching the
# memory map, the MPU or the DFU update path -- the three things every other option in
# issue #39 (strings to the external NOR, XIP, feature toggles) would have had to.
# The hot paths are protected elsewhere and stay untouched: the interrupt paths live
# in ITCM (issue #24/#29, and cmake/check_itcm_residency.py below keeps them there),
# and CoreMark keeps its own -O3 -funroll-loops because the score IS the deliverable.
#
# Both knobs exist because -Os -flto makes single-stepping over SWD much worse.  For a
# debugging session build with -DBSP_OPT_LEVEL=-O2 -DBSP_ENABLE_LTO=OFF; the residency
# check runs in that configuration too.  BSP_OPT_LEVEL is a STRING, not a boolean --
# option() would silently turn it into ON/OFF.
set(BSP_OPT_LEVEL "-Os" CACHE STRING "Optimisation level for the shell firmware")
option(BSP_ENABLE_LTO "Build the shell firmware with link-time optimisation" ON)
# issue #2: ThreadX idle WFI sleep (TX_ENABLE_WFI in tx_user.h).  Default ON; build
# with -DBSP_ENABLE_WFI=OFF for a busy-idle (no-sleep) variant that is easier to
# attach over SWD (a WFI-sleeping core needs connect-under-reset).
option(BSP_ENABLE_WFI "Enable ThreadX idle WFI power saving" ON)
# issue #3: OCTOSPI1 APS6408 8 MB PSRAM at 0x90000000.  BSP_ENABLE_PSRAM builds the
# MPU non-cacheable window + `psram` command; BSP_PSRAM_INIT_IN_APP does the OCTOSPI1
# bring-up app-side (app-first validation).  Once the register values are validated
# the bring-up moves into the bootloader and BSP_PSRAM_INIT_IN_APP is turned OFF.
option(BSP_ENABLE_PSRAM "Enable OCTOSPI1 APS6408 PSRAM (MPU window + psram command)" ON)
option(BSP_PSRAM_INIT_IN_APP "Bring up OCTOSPI1 PSRAM app-side (app-first validation)" ON)
# issue #9: which NN backend this build links.  Declared HERE, above the BSP_ENABLE_*
# options, because BSP_ENABLE_SD's default depends on it -- the rest of the NN wiring
# stays further down with the other source lists.
set(CONFIG_NN_BACKEND "null" CACHE STRING "NN inference backend: null | tflm")
set_property(CACHE CONFIG_NN_BACKEND PROPERTY STRINGS null tflm)

# issue #6: on-board microSD over SDMMC1 + IDMA.  ON builds the block driver
# (port/sd), its .axi_dma bounce buffer and the `sd` command.  Turn OFF to build a
# firmware with no bus master at all -- useful when bisecting a D-cache coherency
# suspicion, since SDMMC1's IDMA is the only DMA engine this app enables.
#
# 🔴 The DEFAULT flips to OFF for the tflm backend (issue #9 phase 2c).  That build adds
# ~103 KB of interpreter and kernels to a 384 KB partition, and the microSD costs
# 34,432 B of flash plus 18,624 B of AXI-SRAM -- the latter being the scarce memory,
# since it is the only RAM a bus master can address (issue #46).  Nothing on the AI path
# uses it: models come from the external NOR's blob region, which is the whole reason
# issue #9 chose that route.  Turning SD off gives back a third of what TFLM costs and
# takes nothing off the inference path.
#
# It is a DEFAULT and not a forced value: `-DBSP_ENABLE_SD=ON` still wins, because
# option() leaves an already-defined cache entry alone.  The message below makes the
# coupling visible -- a peripheral disappearing as a side effect of an unrelated switch
# is the kind of thing that should never be discovered by wondering where `sd` went.
if(CONFIG_NN_BACKEND STREQUAL "tflm" AND NOT DEFINED BSP_ENABLE_SD)
    message(STATUS "tflm backend: defaulting BSP_ENABLE_SD=OFF "
                   "(saves 34,432 B flash + 18,624 B AXI-SRAM; "
                   "models come from the NOR blob region, not the SD card). "
                   "Pass -DBSP_ENABLE_SD=ON to keep the microSD and FileX.")
    set(BSP_ENABLE_SD OFF CACHE BOOL
        "Enable the microSD card (SDMMC1) and the sd command")
endif()
option(BSP_ENABLE_SD "Enable the microSD card (SDMMC1) and the sd command" ON)

# The block driver, the FileX media stack and the command are one unit -- cmd_sd.c
# is the only caller of port/sd and port/filex, so they are added or omitted
# together.  This append must stay BELOW the option() above: SHELL_SOURCES is set
# earlier in the file, when BSP_ENABLE_SD is still undefined, so appending there
# silently drops the driver and the link fails on sd_card_init().
#
# FileX (eclipse-threadx/filex, MIT, pinned to v6.5.1.202602_rel) is GLOB'd whole
# and configured through port/filex/fx_user.h, the same pattern as ThreadX and NetX
# Duo -- one executable compiled uniformly so every translation unit agrees on the
# FX_MEDIA / FX_FILE layout.  --gc-sections drops the feature paths nothing calls.
if(BSP_ENABLE_SD)
    set(FX_DIR "${CMAKE_SOURCE_DIR}/lib/filex")
    file(GLOB FX_CORE "${FX_DIR}/common/src/*.c")
    list(FILTER FX_CORE EXCLUDE REGEX "simulat")     # RAM-disk demo drivers
    list(APPEND SHELL_SOURCES
        "${CMAKE_SOURCE_DIR}/port/sd/sd_card.c"
        "${CMAKE_SOURCE_DIR}/port/filex/fx_sd_driver.c"
        "${CMAKE_SOURCE_DIR}/port/filex/sd_fs_glue.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/fs_cmd_core.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_sd.c"
        ${FX_CORE})
endif()

# issue #7: the FPC-40 RGB LCD over LTDC + DMA2D.  ON builds the display driver
# and the `lcd` command; the two frame buffers land in the OCTOSPI1 PSRAM, so this
# only does anything useful with BSP_ENABLE_PSRAM.  Turn OFF to build a firmware
# whose LTDC never touches the clock tree (ltdc_clock_init briefly stops PLL3,
# which is also the USB console's clock) or never reads OCTOSPI1 -- useful when
# bisecting a PLL3/USB or a PSRAM-bandwidth suspicion.  Also drops the LTDC
# interlock inside psram_acquire().
option(BSP_ENABLE_LCD "Enable the FPC-40 RGB LCD (LTDC) and the lcd command" ON)


# Driver and command move together -- cmd_lcd.c is the only caller of port/ltdc --
# and, like the SD block above, this append must stay BELOW the option().
if(BSP_ENABLE_LCD)
    list(APPEND SHELL_SOURCES
        "${CMAKE_SOURCE_DIR}/port/ltdc/ltdc_display.c"
        "${CMAKE_SOURCE_DIR}/port/ltdc/st7789_rgb.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_lcd.c")
endif()

# issue #37 (from the #10 RFC): the external W25Q128 NOR on OCTOSPI2 and the
# configuration key-value store that lives on it.  ON builds the NOR driver
# (port/nor) and the `nor` command; the FlashDB key-value layer is added to this
# same block as it lands.  Turn OFF for a firmware that never configures the
# OCTOSPI2 pins or registers -- useful when bisecting anything that suspects the
# second OCTOSPI of disturbing the PSRAM on the first.
#
# NOTE: this does NOT open the memory-mapped window at 0x70000000.  The driver is
# indirect-only and app/mpu.c keeps its no-access + XN region over that quarter
# either way -- see port/nor/nor_flash.h for why that fence stays.
option(BSP_ENABLE_KV "Enable the external NOR flash (OCTOSPI2) and the config KV store" ON)

# Driver, database and commands move together, and -- like the SD/LCD/camera
# blocks -- this append must stay BELOW the option().
#
# FlashDB (armink/FlashDB, Apache-2.0, pinned to 2.2.0) is taken as three core
# sources plus the FAL it ships in its own port/ directory.  Unlike ThreadX/FileX/
# NetX Duo above, this one is NOT GLOB'd whole: fdb_tsdb.c (time-series database)
# and fdb_file.c (filesystem-backed storage) are features port/flashdb/fdb_cfg.h
# does not enable, and fal_rtt.c is the RT-Thread integration and does not build
# outside it.  Configuration comes from port/flashdb (fdb_cfg.h + fal_cfg.h), which
# is why that directory is on the include path before FlashDB's own.
if(BSP_ENABLE_KV)
    set(FDB_DIR "${CMAKE_SOURCE_DIR}/lib/flashdb")
    list(APPEND SHELL_SOURCES
        "${CMAKE_SOURCE_DIR}/port/nor/nor_flash.c"
        "${CMAKE_SOURCE_DIR}/port/flashdb/fal_nor.c"
        "${CMAKE_SOURCE_DIR}/app/kv.c"
        "${CMAKE_SOURCE_DIR}/app/kv_boot.c"
        "${CMAKE_SOURCE_DIR}/app/blob.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_nor.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_kv.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_blob.c"
        "${FDB_DIR}/src/fdb.c"
        "${FDB_DIR}/src/fdb_kvdb.c"
        "${FDB_DIR}/src/fdb_utils.c"
        "${FDB_DIR}/port/fal/src/fal.c"
        "${FDB_DIR}/port/fal/src/fal_flash.c"
        "${FDB_DIR}/port/fal/src/fal_partition.c")
endif()

# issue #8: the FPC-24 DVP camera.  Phase 1 builds only the sensor-side bring-up
# (XCLK on TIM5_CH3, SCCB on I2C4, PWDN/RESETB) and the `camera` command; DCMI and
# DMA arrive in phase 2.  Turn OFF for a firmware that leaves TIM5, I2C4 and the
# camera pins untouched -- useful when bisecting a timer or D3-domain suspicion.
option(BSP_ENABLE_CAMERA "Enable the FPC-24 DVP camera (DCMI) and the camera command" ON)

# Driver and command move together -- cmd_camera.c is the only caller of
# port/camera -- and, like the two blocks above, this append must stay BELOW the
# option().
if(BSP_ENABLE_CAMERA)
    list(APPEND SHELL_SOURCES
        "${CMAKE_SOURCE_DIR}/port/camera/camera.c"
        "${CMAKE_SOURCE_DIR}/port/camera/ov2640_regs.c"
        # issue #9 phase 3: the band stream now has two consumers (the preview and
        # the NN ingest), and port/camera holds exactly one callback.  The mux is
        # gated on the camera ALONE -- the NN client needs no display, and running
        # inference with the preview off is the fastest configuration this board has.
        "${CMAKE_SOURCE_DIR}/app/cam_band.c"
        # issue #9 phase 3: the camera -> inference glue.  Camera-only for the same
        # reason as the mux; the overlay it feeds is drawn by cam_preview.c, which
        # is the file already gated on the pair.
        "${CMAKE_SOURCE_DIR}/app/nn_camera.c"
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_camera.c")
    # The LCD preview (issue #8 phase 3c) needs BOTH drivers, so it is the one
    # source gated on the pair.  With either option off there is nothing to glue
    # and the file is simply not built.
    if(BSP_ENABLE_LCD)
        list(APPEND SHELL_SOURCES "${CMAKE_SOURCE_DIR}/app/cam_preview.c")
    endif()
endif()

# issue #9: on-device NN inference.  Unlike the blocks above there is no
# BSP_ENABLE_AI: the BSP_ENABLE_* switches all name a piece of HARDWARE, and their
# point is to build a firmware that leaves those pins and peripherals alone while
# bisecting a suspicion.  This is a software layer with no hardware of its own, so
# what varies is not whether it exists but WHICH RUNTIME it dispatches to.
#
# Exactly one backend translation unit is compiled, and that is the whole selection
# mechanism: nn.c has one `extern const struct nn_backend_vt nn_backend_vt_selected`
# and the linker resolves it against whichever backend was added here.  No source
# file has an #ifdef for this, and the default `null` build needs no -D at all.
#
# If the app partition ever gets tight again (issue #39), a `none` value that drops
# the whole layer is the natural place to add the toggle.
#
# CONFIG_NN_BACKEND itself is declared much earlier in this file, above the BSP_ENABLE_*
# block, because BSP_ENABLE_SD's default depends on which backend was selected.
set(NN_SOURCES "${CMAKE_SOURCE_DIR}/port/nn/nn.c")
# Which buffers the cacheable-carve-out gate must find in the image depends on which
# backend was selected, and CMake is the only place that knows.  Before issue #9 phase
# 2c the list was hard-coded in the script, which meant the script itself refused every
# tflm build -- the `null` stub's buffers are not in that image.
set(NN_PSRAM_AI_REQUIRED "")
if(CONFIG_NN_BACKEND STREQUAL "null")
    list(APPEND NN_SOURCES "${CMAKE_SOURCE_DIR}/port/nn/nn_null.c")
    list(APPEND NN_PSRAM_AI_REQUIRED
         --require null_in_buf --require null_box_buf --require null_scr_buf)
elseif(CONFIG_NN_BACKEND STREQUAL "tflm")
    # TensorFlow Lite Micro (issue #9 phase 2c).  The backend translation unit is NOT
    # added to SHELL_SOURCES: it is C++ and lives in the `tflm` static library that
    # this file builds, together with the fetched tflite-micro + CMSIS-NN tree.  Only
    # nn.c stays here, and it finds nn_backend_vt_selected in that archive at link
    # time -- the same one-definition mechanism the `null` build uses.
    include("${CMAKE_SOURCE_DIR}/cmake/tflite-micro.cmake")
    list(APPEND NN_PSRAM_AI_REQUIRED
         --require nn_tflm_arena --require nn_tflm_model_buf)
else()
    message(FATAL_ERROR "CONFIG_NN_BACKEND must be 'null' or 'tflm'")
endif()
# membench's cacheable row is in the carve-out whichever backend is built.
list(APPEND NN_PSRAM_AI_REQUIRED --require psram_ai_bench_buf)
# issue #9 phase 3: model-specific post-processing, above the model-agnostic nn API.
# Built UNCONDITIONALLY -- including in the `null` build, whose stub tensors it simply
# does not recognise (blazeface_decode() returns -1 without touching anything).  That
# is what lets `ai dets` be registered unconditionally, and keeping the reference in
# every build is also what keeps the three --require lines below honest: a symbol
# --gc-sections dropped would be reported as "no such object in the image", which
# reads like a placement regression and is not one.
list(APPEND NN_SOURCES "${CMAKE_SOURCE_DIR}/port/nn/models/blazeface.c")
list(APPEND NN_PSRAM_AI_REQUIRED
     --require bf_cx --require bf_cy --require bf_cand)
# The other half of that gate: PSRAM buffers a bus master owns, which must stay OUT
# of the cacheable carve-out.  Named here rather than in the script for the same
# reason as the DTCM list above -- each belongs to a BSP_ENABLE_* option, and a name
# demanded from a build that never compiled it is reported as "no such object in the
# image", which reads like a placement regression and is not one.  That is issue #50:
# check_dtcm_residency.py was fixed this way in issue #9 phase 2c while this gate kept
# its hard-coded list, so BSP_ENABLE_CAMERA=OFF still could not be built.
if(BSP_ENABLE_LCD)
    list(APPEND NN_PSRAM_AI_REQUIRED --noncacheable ltdc_fb)   # LTDC scan-out + DMA2D
endif()
if(BSP_ENABLE_CAMERA)
    list(APPEND NN_PSRAM_AI_REQUIRED --noncacheable cam_ring)  # DCMI via DMA2_Stream1
    list(APPEND NN_PSRAM_AI_REQUIRED --noncacheable cam_frame) # DCMI single capture
endif()
list(APPEND SHELL_SOURCES ${NN_SOURCES} "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_ai.c")

# issue #55: the MLPerf Tiny v1.4 benchmark harness.  Like CONFIG_NN_BACKEND above and
# for the same reason, this is NOT a BSP_ENABLE_* switch -- those all name a piece of
# hardware, and the point of each is a firmware that leaves those pins alone while
# bisecting a suspicion.  There is no MLPerf peripheral.
#
# OFF by default because it is not free: it needs the ~340 MB upstream mirror, it adds
# ~28 KB of harness and gp_buff to a partition that is already 93% full, and the
# operator ops it needs (NN_TFLM_OPS=mlperf) cost more flash again.  A benchmark run is
# a deliberate act, so turning it on is one too.
option(CONFIG_MLPERF_TINY "Build the MLPerf Tiny v1.4 benchmark harness and `mlperf`" OFF)
if(CONFIG_MLPERF_TINY)
    # Required, not merely recommended.  The harness runs whatever model `ai model
    # load` put in the interpreter, and the `null` backend has no interpreter --
    # nn_model_reload() returns NN_ERR_NOSUP.  The build would succeed and the command
    # would refuse every model, which is the least useful place to discover this.
    if(NOT CONFIG_NN_BACKEND STREQUAL "tflm")
        message(FATAL_ERROR
            "CONFIG_MLPERF_TINY=ON needs CONFIG_NN_BACKEND=tflm (it benchmarks a model "
            "the interpreter loads at run time; the `null` backend has no interpreter). "
            "Configure with -DCONFIG_NN_BACKEND=tflm -DNN_TFLM_OPS=mlperf.")
    endif()
    # Not fatal: the operator set is a separate decision, and someone may deliberately
    # be measuring a narrower one.  But `blazeface` cannot run a single MLPerf model,
    # so saying nothing would let the failure surface as an unexplained refusal from
    # `ai model load` instead.
    if(NOT NN_TFLM_OPS STREQUAL "mlperf" AND NOT NN_TFLM_OPS STREQUAL "extended")
        message(WARNING
            "CONFIG_MLPERF_TINY=ON with NN_TFLM_OPS=${NN_TFLM_OPS}: none of the MLPerf "
            "Tiny models will load (they need FULLY_CONNECTED, AVERAGE_POOL_2D and "
            "SOFTMAX).  Add -DNN_TFLM_OPS=mlperf.")
    endif()
    include("${CMAKE_SOURCE_DIR}/cmake/mlperf-tiny.cmake")
    list(APPEND SHELL_SOURCES "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_mlperf.c")
endif()

add_executable(shell
    "${CMAKE_SOURCE_DIR}/app/main.c"
    "${CMAKE_SOURCE_DIR}/app/usb_cdc.c"
    "${CMAKE_SOURCE_DIR}/app/usb_descriptors.c"
    "${CMAKE_SOURCE_DIR}/app/retarget.c"
    "${CMAKE_SOURCE_DIR}/app/fault.c"
    "${CMAKE_SOURCE_DIR}/app/iwdg.c"
    "${CMAKE_SOURCE_DIR}/app/malloc_lock.c"
    "${CMAKE_SOURCE_DIR}/app/mpu.c"
    "${CMAKE_SOURCE_DIR}/app/psram.c"
    "${CMAKE_SOURCE_DIR}/app/rtl8720.c"
    "${CMAKE_SOURCE_DIR}/app/rtl8720_flash.c"
    "${CMAKE_SOURCE_DIR}/app/rtl8720_img.c"
    "${GEN_DIR}/rtl8720_flashloader_data.c"
    "${CMAKE_SOURCE_DIR}/app/erpc.c"
    "${CMAKE_SOURCE_DIR}/app/link_data.c"
    "${CMAKE_SOURCE_DIR}/app/wifi_rpc.c"
    "${CMAKE_SOURCE_DIR}/app/wifi_auto.c"
    "${CMAKE_SOURCE_DIR}/app/wifi_connect.c"
    "${CMAKE_SOURCE_DIR}/app/rtl_link.c"
    "${CMAKE_SOURCE_DIR}/app/net_shell.c"
    "${CMAKE_SOURCE_DIR}/app/nx_echo.c"
    "${CMAKE_SOURCE_DIR}/app/nx_link_driver.c"
    "${CMAKE_SOURCE_DIR}/app/nx_net.c"
    "${CMAKE_SOURCE_DIR}/port/threadx/tx_glue.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${SHELL_SOURCES}
    ${APP_TINYUSB_SOURCES}
    ${TX_CORE} ${TX_ASM} ${TX_EPK}
    ${NX_CORE} ${NX_ADDONS}
    ${HAL_SOURCES})
target_link_libraries(shell PRIVATE bsp_iface coremark_obj)
# CoreMark's canonical report prints its score with %f; pull in newlib's float
# printf (newlib-nano omits it by default).
target_link_options(shell PRIVATE -u _printf_float)
target_include_directories(shell PRIVATE
    "${CMAKE_SOURCE_DIR}/app"
    "${CMAKE_SOURCE_DIR}/port/threadx"
    "${CMAKE_SOURCE_DIR}/port/sd"                # issue #6: sd_card.h
    "${CMAKE_SOURCE_DIR}/port/filex"             # issue #6: fx_user.h + the SD glue
    "${CMAKE_SOURCE_DIR}/port/ltdc"              # issue #7: ltdc_display.h
    "${CMAKE_SOURCE_DIR}/port/camera"            # issue #8: camera.h
    "${CMAKE_SOURCE_DIR}/port/nor"               # issue #37: nor_flash.h
    "${CMAKE_SOURCE_DIR}/port/nn"                # issue #9: nn.h / nn_backend.h
    "${CMAKE_SOURCE_DIR}/port/nn/models"         # issue #9 P3: blazeface.h
    # issue #37: our fdb_cfg.h / fal_cfg.h must be found BEFORE FlashDB's own inc/,
    # which ships fdb_cfg_template.h only -- FlashDB includes <fdb_cfg.h> by name.
    "${CMAKE_SOURCE_DIR}/port/flashdb"
    "${CMAKE_SOURCE_DIR}/lib/flashdb/inc"
    "${CMAKE_SOURCE_DIR}/lib/flashdb/port/fal/inc"
    "${CMAKE_SOURCE_DIR}/lib/filex/common/inc"
    "${CMAKE_SOURCE_DIR}/lib/filex/ports/cortex_m7/gnu/inc"
    "${CMAKE_SOURCE_DIR}/shell/include"
    "${CMAKE_SOURCE_DIR}/shell/core"
    "${CMAKE_SOURCE_DIR}/shell/backend"
    "${CMAKE_SOURCE_DIR}/svc"
    "${TUSB_DIR}"
    "${TX_DIR}/common/inc"
    "${TX_DIR}/utility/execution_profile_kit"   # issue #2: tx_execution_profile.h
    "${TX_PORT}/inc"
    "${NX_DIR}/common/inc"                      # issue #23 U3: NetX Duo
    "${NX_DIR}/ports/cortex_m7/gnu/inc"
    "${NX_DIR}/addons/dhcp"
    "${CMAKE_SOURCE_DIR}/port/netxduo")         # nx_user.h
target_compile_definitions(shell PRIVATE
    CFG_TUSB_MCU=OPT_MCU_STM32H7
    TX_INCLUDE_USER_DEFINE_FILE
    NX_INCLUDE_USER_DEFINE_FILE        # issue #23 U3: port/netxduo/nx_user.h
    # The DHCP client shares the one packet pool instead of creating its own, and its
    # thread sits just below the NetX IP thread (11) so IP processing always wins.
    NX_DHCP_CLIENT_USER_CREATE_PACKET_POOL
    NX_DHCP_THREAD_PRIORITY=13
    CLI_ENABLE_DANGEROUS_CMDS=1        # reboot / devmem / crash
    CLI_INSTANCE_STACK_SIZE=4096       # headroom for cli_print + coremark
    CLI_BG_JOB_STACK_SIZE=4096
    CLI_USBCDC_TX_BUFFER_SIZE=4096     # holds a full CoreMark report burst
    # issue #19 M5: the console now RECEIVES bulk data (`wifi imgload` takes a
    # firmware image over YMODEM).  The backend drops -- and counts -- a byte
    # whenever this ring is full, so the default 256 B is too tight for a 1029 B
    # YMODEM block; 4 KB makes an overrun require the shell thread to be starved
    # for four whole blocks.  `wifi imgload` reports the drop count either way.
    CLI_USBCDC_RX_BUFFER_SIZE=4096
    BSP_ENABLE_IWDG=$<BOOL:${BSP_ENABLE_IWDG}>   # issue #4: IWDG1 arm + petter + wdt
    BSP_ENABLE_WFI=$<BOOL:${BSP_ENABLE_WFI}>      # issue #2: gates TX_ENABLE_WFI (tx_user.h)
    BSP_ENABLE_PSRAM=$<BOOL:${BSP_ENABLE_PSRAM}>          # issue #3: PSRAM window + psram cmd
    BSP_PSRAM_INIT_IN_APP=$<BOOL:${BSP_PSRAM_INIT_IN_APP}>  # issue #3: app-side OCTOSPI1 bring-up
    FX_INCLUDE_USER_DEFINE_FILE             # issue #6: port/filex/fx_user.h
    BSP_ENABLE_SD=$<BOOL:${BSP_ENABLE_SD}>                  # issue #6: SDMMC1 microSD + sd cmd
    BSP_ENABLE_LCD=$<BOOL:${BSP_ENABLE_LCD}>                # issue #7: LTDC LCD + lcd cmd
    BSP_ENABLE_CAMERA=$<BOOL:${BSP_ENABLE_CAMERA}>          # issue #8: DVP camera + camera cmd
    BSP_ENABLE_KV=$<BOOL:${BSP_ENABLE_KV}>)                 # issue #37: OCTOSPI2 NOR + KV store
# issue #39: -Os + LTO, plus two flags that disable optimisations which are legal C
# but wrong for THIS firmware.  Both are cheap (measured: 1,008 B and 224 B) and both
# only start to matter under LTO, which propagates these inferences ACROSS
# translation units for the first time:
#
#  -fno-strict-aliasing            NetX Duo and FileX cast packet buffers to protocol
#                                  headers constantly.  Whole-program alias analysis is
#                                  exactly the condition under which those type-punning
#                                  violations stop being harmless.
#  -fno-delete-null-pointer-checks ITCM lives at 0x00000000 on this part, so address 0
#                                  is a real, deliberately accessed location -- our
#                                  SystemInit writes and reads back the whole 64 KB
#                                  (src/system_stm32h7xx.c).  Without this, "code
#                                  dereferenced this pointer, so it cannot be NULL" is
#                                  free to travel between translation units.
set(SHELL_OPT_FLAGS ${BSP_OPT_LEVEL} -fno-strict-aliasing -fno-delete-null-pointer-checks)
if(BSP_ENABLE_LTO)
    list(APPEND SHELL_OPT_FLAGS -flto=auto)
endif()
target_compile_options(shell PRIVATE ${SHELL_OPT_FLAGS})
# ...except membench, which is an INSTRUMENT and must keep its calibration (issue #39).
# Its bandwidth loops are the yardstick issue #3 tuned the PSRAM against and issue #25
# used to justify moving execution to the internal flash (54 vs 905 MB/s), so a reading
# has to mean the same thing across builds.  -Os disables loop unrolling, which turns
# the read loop from 73 instructions into 47 and makes it LOOP-bound rather than
# memory-bound: measured that way, ITCM, DTCM and cached SRAM all report an identical
# 1453 MB/s and the internal flash reads "859 MB/s" instead of 905 -- numbers about the
# benchmark, not about the memory.  Same reasoning as coremark_obj above; -fno-lto
# keeps the whole-program optimiser from reintroducing the dependency at link time.
set_source_files_properties("${CMAKE_SOURCE_DIR}/shell/cmds/cmd_membench.c"
    PROPERTIES COMPILE_OPTIONS "-O2;-fno-lto")
# The SAME flags again at link time: with LTO the final code generation happens in the
# linker, so compile-only flags are advisory at best.  -ffunction-sections /
# -fdata-sections in particular MUST be repeated here -- without them LTO emits one
# monolithic .text, which defeats --gc-sections AND makes every .itcm input-section
# pattern in the linker script stop matching.
target_link_options(shell PRIVATE ${SHELL_OPT_FLAGS} -ffunction-sections -fdata-sections)

# issue #9 phase 2c: link the C++ TFLM backend in, and force the final link back onto
# the C driver.  Everything that decision involves is in cmake/tflite-micro.cmake --
# it has to run here rather than in that file because the `shell` target does not exist
# yet where the backend selection above includes it.
if(CONFIG_NN_BACKEND STREQUAL "tflm")
    nn_tflm_attach(shell)
endif()

# issue #55: the benchmark harness objects.  An OBJECT library, so these land in the
# image exactly as if they had been listed in add_executable -- but compiled with the
# C++ containment flags `shell` itself does not carry (cmake/mlperf-tiny.cmake).  The
# link driver stays C: nn_tflm_attach() above already forces LINKER_LANGUAGE, which is
# required anyway and covers this too.
if(CONFIG_MLPERF_TINY)
    target_link_libraries(shell PRIVATE mlperf_obj)
    target_include_directories(shell PRIVATE "${CMAKE_SOURCE_DIR}/port/mlperf")
    target_compile_definitions(shell PRIVATE CONFIG_MLPERF_TINY=1)
endif()

firmware_finalize(shell)

# issue #39: the last line of defence for the ITCM residency of the interrupt paths
# (issue #24/#29).  The linker-script ASSERTs cannot do this job under LTO -- see the
# comment on them in ldscript/STM32H725AEIx_IROM.ld -- because LTO renames the very
# symbols they are written against.  This checks the linked image instead, and is
# deliberately attached to `shell` only: blink shares the linker script but has no
# ThreadX and no RTL8720 UART, so the required-symbol list does not apply to it.
add_custom_command(TARGET shell POST_BUILD
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/cmake/check_itcm_residency.py"
            --nm "${CMAKE_NM}" --objdump "${CMAKE_OBJDUMP}"
            $<TARGET_FILE:shell>
    COMMENT "check .itcm residency of the interrupt paths (issue #39)"
    VERBATIM)

# The same kind of gate for the AXI-SRAM / DTCM split (issue #46), and it guards both
# directions: a stack that quietly returned to AXI-SRAM (budget regression, no error)
# and a DMA buffer that quietly moved to DTCM (the transfer moves nothing, no error).
# `shell` only, like the check above: blink has none of these objects.
#
# The objects that only exist in SOME configurations are named here rather than in the
# script, for the reason spelled out at the top of it: a required symbol the build never
# compiled is reported as "no such object in the image", which reads like a placement
# regression and is not one.  Before this, `BSP_ENABLE_SD=OFF` could not be built at
# all -- the gate failed on sd_bounce -- which defeated the option's entire purpose,
# since it exists to produce a firmware with no bus master while bisecting a D-cache
# suspicion.  The condition on each line below is the same one that adds its source.
set(DTCM_REQUIRED "")
if(BSP_ENABLE_KV)
    list(APPEND DTCM_REQUIRED --require-dtcm kv_boot_stack)      # app/kv_boot.c
endif()
if(BSP_ENABLE_CAMERA)
    list(APPEND DTCM_REQUIRED --require-dtcm cam_producer_stack) # port/camera/camera.c
    list(APPEND DTCM_REQUIRED --require-dtcm nn_worker_stack)    # app/nn_camera.c
    list(APPEND DTCM_REQUIRED --require-axi  cam_band)           # DCMI via DMA2_Stream1
    if(BSP_ENABLE_LCD)
        list(APPEND DTCM_REQUIRED --require-dtcm preview_stack)  # app/cam_preview.c
    endif()
endif()
if(BSP_ENABLE_SD)
    list(APPEND DTCM_REQUIRED --require-axi sd_bounce)           # SDMMC1 IDMA
endif()

add_custom_command(TARGET shell POST_BUILD
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/cmake/check_dtcm_residency.py"
            --nm "${CMAKE_NM}" ${DTCM_REQUIRED}
            $<TARGET_FILE:shell>
    COMMENT "check .dtcm_bss residency and DMA reachability (issue #46)"
    VERBATIM)

# And the same kind of gate for the cacheable PSRAM carve-out (issue #9 phase 2a).
# It guards the rule that replaced "everything in PSRAM is safe for DMA" once the top
# 2 MB stopped being non-cacheable, and that rule has no runtime enforcement at all: a
# DMA buffer in there transfers correctly and is then read back stale from the data
# cache, intermittently, only under cache pressure.  `shell` only, like the two above.
add_custom_command(TARGET shell POST_BUILD
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/cmake/check_psram_ai_residency.py"
            --nm "${CMAKE_NM}" ${NN_PSRAM_AI_REQUIRED}
            $<TARGET_FILE:shell>
    COMMENT "check .psram_ai residency and cache safety (issue #9)"
    VERBATIM)

# issue #9 phase 2c: the C++ containment gate, for the tflm build only.  This firmware
# had no C++ at all until that phase, and the two things that go wrong when C++ arrives
# in a bare-metal image are both invisible at link time.
#
#   A global constructor runs from __libc_init_array, which the startup code calls
#   immediately BEFORE main -- so before the MPU is programmed, before the caches are
#   on, before the PSRAM window is usable, before log_init() and long before ThreadX.
#   A constructor that touches the carve-out hangs the board with nothing in `dmesg`.
#
#   The exception machinery re-enters through the back door: one undefined reference to
#   the throwing operator new is enough for the linker to extract __cxa_throw and the
#   whole unwinder, in a build compiled -fno-exceptions from end to end.
#
# Both are checked on the linked image, because a linker-script ASSERT cannot do it:
# C++ name mangling defeats DEFINED(sym) selectors the same way LTO's renaming does --
# the condition goes false, the ternary collapses to 1, and the ASSERT passes.
if(CONFIG_NN_BACKEND STREQUAL "tflm")
    add_custom_command(TARGET shell POST_BUILD
        COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/cmake/check_cxx_runtime.py"
                --nm "${CMAKE_NM}" --objdump "${CMAKE_OBJDUMP}"
                $<TARGET_FILE:shell>
        COMMENT "check C++ runtime containment: no static ctors, no unwinder (issue #9)"
        VERBATIM)

    # Not a gate -- a report.  The shell thread's stack is 4,096 B and DTCM has under
    # 8 KB free, so there is very little room to react if a CMSIS-NN kernel turns out to
    # want a deep frame.  -fstack-usage answers that at build time instead of waiting
    # for a `free` high-water reading to come back from the board.
    add_custom_command(TARGET shell POST_BUILD
        COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/cmake/report_tflm_stack.py"
                "${CMAKE_BINARY_DIR}/CMakeFiles/tflm.dir"
        COMMENT "report the deepest tflm stack frames (issue #9)"
        VERBATIM)
endif()
