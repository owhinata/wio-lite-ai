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
# SRAM.  It is a Realtek-proprietary blob, so it is NOT committed -- embed it at
# configure time from _ref/ambd/ (git-ignored) into GEN_DIR.  Fail loudly if absent
# rather than building a non-functional flasher.
set(RTL_FLASHLOADER_BIN "${CMAKE_SOURCE_DIR}/_ref/ambd/imgtool_flashloader_amebad.bin")
if(NOT EXISTS "${RTL_FLASHLOADER_BIN}")
    message(FATAL_ERROR
        "Missing ${RTL_FLASHLOADER_BIN}\n"
        "  Place imgtool_flashloader_amebad.bin (4688 B AmebaD flashloader, load 0x00082000)\n"
        "  there, e.g. from github.com/Seeed-Studio/ambd_flash_tool or\n"
        "  github.com/openshwprojects/SharpRTL872xTool (issue #19).")
endif()
file(READ "${RTL_FLASHLOADER_BIN}" _RTL_FL_HEX HEX)
string(LENGTH "${_RTL_FL_HEX}" _RTL_FL_HEXLEN)
math(EXPR _RTL_FL_LEN "${_RTL_FL_HEXLEN} / 2")
string(REGEX REPLACE "(..)" "0x\\1," _RTL_FL_ARRAY "${_RTL_FL_HEX}")
file(WRITE "${GEN_DIR}/rtl8720_flashloader_data.c"
"/* AUTO-GENERATED at configure time from _ref/ambd/imgtool_flashloader_amebad.bin.\n"
" * Do NOT edit and do NOT commit -- it is a Realtek-proprietary blob (issue #19 M2). */\n"
"#include <stdint.h>\n"
"const uint8_t rtl8720_flashloader[] = { ${_RTL_FL_ARRAY} };\n"
"const uint32_t rtl8720_flashloader_len = ${_RTL_FL_LEN};\n")

# IWDG watchdog (issue #4): ON builds in the IWDG1 arming + priority-5 petter thread
# + the `wdt` starve test; OFF compiles them out entirely (`wdt info` then reports
# "disabled (build)").  Turn OFF for SWD sessions that must hold a breakpoint longer
# than the ~3 s IWDG timeout (the app does not touch DBGMCU to freeze it under debug).
option(BSP_ENABLE_IWDG "Enable the IWDG watchdog and wdt shell command" ON)
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
# issue #6: on-board microSD over SDMMC1 + IDMA.  ON builds the block driver
# (port/sd), its .axi_dma bounce buffer and the `sd` command.  Turn OFF to build a
# firmware with no bus master at all -- useful when bisecting a D-cache coherency
# suspicion, since SDMMC1's IDMA is the only DMA engine this app enables.
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
        "${CMAKE_SOURCE_DIR}/shell/cmds/cmd_camera.c")
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
    BSP_ENABLE_CAMERA=$<BOOL:${BSP_ENABLE_CAMERA}>)         # issue #8: DVP camera + camera cmd
target_compile_options(shell PRIVATE -O2)

firmware_finalize(shell)
