# ============================================================================
#  Wio Lite AI (STM32H725AEI6) firmware.
#
#  Firmwares:
#    boot  -- standalone USB DFU bootloader (see boot/README.md).  Runs from
#             internal flash 0x08000000 (replaces TinyUF2); flashed via ST-Link
#             (mode=UR).  Produces build/boot.{elf,bin,hex} (no UF2).
#    blink -- LED blink reference app.  XIP from external OCTOSPI2 0x70000000;
#             load it with `dfu-util -d 0483:df11 -a 0 -D build/blink.bin` while
#             `boot` is in DFU mode (convenience target: `dfu-blink`).
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
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/coremark/core_main.c")
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
set(LDSCRIPT_XIP "${CMAKE_SOURCE_DIR}/ldscript/STM32H725AEIx_XIP.ld")  # app-first (0x70000000)
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
# configures the clock tree / OCTOSPI2 itself (stock CMSIS system file resets to
# HSI, then SystemClock_Config), enumerates as a composite DFU + CDC device, and
# programs downloads to the app base 0x70000000.  Flashed with the ST-Link
# (mode=UR), not via UF2.  TinyUSB and its config/descriptors are compiled into
# ONLY this target (blink stays a pure bare-metal app).
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
    "${CMAKE_SOURCE_DIR}/boot/octospi.c"
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
#  Apps: blink (and the future ThreadX shell)
#  XIP from external OCTOSPI2 0x70000000; flashed over DFU (dfu-util / dfu-<tgt>)
# ============================================================================
# --- Post-build artifacts + DFU flash target -------------------------------
# App XIP firmwares (blink, and the future ThreadX shell): linked at 0x70000000,
# emitted as raw .bin (what the DFU bootloader programs over dfu-util) + .hex.
function(firmware_finalize tgt)
    target_link_options(${tgt} PRIVATE "-T${LDSCRIPT_XIP}" -Wl,-Map=${tgt}.map,--cref)
    set_target_properties(${tgt} PROPERTIES LINK_DEPENDS "${LDSCRIPT_XIP}")
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary -S $<TARGET_FILE:${tgt}> ${tgt}.bin
        COMMAND "${CMAKE_OBJCOPY}" -O ihex   $<TARGET_FILE:${tgt}> ${tgt}.hex
        COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:${tgt}>
        BYPRODUCTS ${tgt}.bin ${tgt}.hex
        COMMENT "objcopy -> ${tgt}.bin/.hex")

    # `dfu-<tgt>`: program the app to OCTOSPI2 0x70000000 over the custom DFU
    # bootloader.  Put the board in DFU mode first (hold USER/PF1 at reset -- the
    # red LED stays on).  Named per firmware so app targets never collide on one
    # `dfu` target.  (TinyUF2 and its .uf2 drive are gone: the bootloader that
    # replaced it takes a raw .bin over dfu-util, so no UF2 is produced.)
    add_custom_target(dfu-${tgt}
        COMMAND dfu-util -d 0483:df11 -a 0 -D ${tgt}.bin
        DEPENDS ${tgt}
        USES_TERMINAL
        COMMENT "dfu-util -> ${tgt}.bin to OCTOSPI2 0x70000000 (hold PF1 at reset for DFU mode)")
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
# XIP app at 0x70000000 (firmware_finalize -> dfu-shell): ThreadX + an interactive
# CLI over USB CDC.  Reuses src/system_stm32h7xx.c (clock-free SystemInit) so the
# app inherits the bootloader's 550 MHz clock + OCTOSPI2 XIP.  ThreadX core
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
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_builtin.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_system.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_thread.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_free.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_sleep.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_watch.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_jobs.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_devmem.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_dmesg.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_crash.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_wdt.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_coremark.c"
    "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_membench.c"
    "${CMAKE_SOURCE_DIR}/svc/fmt.c"
    "${CMAKE_SOURCE_DIR}/svc/timebase.c"
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

# IWDG watchdog (issue #4): ON builds in the IWDG1 arming + priority-5 petter thread
# + the `wdt` starve test; OFF compiles them out entirely (`wdt info` then reports
# "disabled (build)").  Turn OFF for SWD sessions that must hold a breakpoint longer
# than the ~3 s IWDG timeout (the app does not touch DBGMCU to freeze it under debug).
option(BSP_ENABLE_IWDG "Enable the IWDG watchdog and wdt shell command" ON)

add_executable(shell
    "${CMAKE_SOURCE_DIR}/app/main.c"
    "${CMAKE_SOURCE_DIR}/app/usb_cdc.c"
    "${CMAKE_SOURCE_DIR}/app/usb_descriptors.c"
    "${CMAKE_SOURCE_DIR}/app/retarget.c"
    "${CMAKE_SOURCE_DIR}/app/fault.c"
    "${CMAKE_SOURCE_DIR}/app/iwdg.c"
    "${CMAKE_SOURCE_DIR}/app/malloc_lock.c"
    "${CMAKE_SOURCE_DIR}/port/threadx/tx_glue.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${SHELL_SOURCES}
    ${APP_TINYUSB_SOURCES}
    ${TX_CORE} ${TX_ASM}
    ${HAL_SOURCES})
target_link_libraries(shell PRIVATE bsp_iface coremark_obj)
# CoreMark's canonical report prints its score with %f; pull in newlib's float
# printf (newlib-nano omits it by default).
target_link_options(shell PRIVATE -u _printf_float)
target_include_directories(shell PRIVATE
    "${CMAKE_SOURCE_DIR}/app"
    "${CMAKE_SOURCE_DIR}/port/threadx"
    "${CMAKE_SOURCE_DIR}/shell/include"
    "${CMAKE_SOURCE_DIR}/shell/core"
    "${CMAKE_SOURCE_DIR}/shell/backend"
    "${CMAKE_SOURCE_DIR}/svc"
    "${TUSB_DIR}"
    "${TX_DIR}/common/inc"
    "${TX_PORT}/inc")
target_compile_definitions(shell PRIVATE
    CFG_TUSB_MCU=OPT_MCU_STM32H7
    TX_INCLUDE_USER_DEFINE_FILE
    CLI_ENABLE_DANGEROUS_CMDS=1        # reboot / devmem / crash
    CLI_INSTANCE_STACK_SIZE=4096       # headroom for cli_print + coremark
    CLI_BG_JOB_STACK_SIZE=4096
    CLI_USBCDC_TX_BUFFER_SIZE=4096     # holds a full CoreMark report burst
    BSP_ENABLE_IWDG=$<BOOL:${BSP_ENABLE_IWDG}>)  # issue #4: IWDG1 arm + petter + wdt
target_compile_options(shell PRIVATE -O2)

firmware_finalize(shell)
