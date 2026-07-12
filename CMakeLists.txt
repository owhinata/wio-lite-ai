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

# Fetch HAL/CMSIS/TinyUSB submodules if any is missing.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32h7xx_hal_driver/Src/stm32h7xx_hal.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_core/Include/core_cm7.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_h7/Include/stm32h725xx.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/src/tusb.h")
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

add_executable(blink
    "${CMAKE_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${HAL_SOURCES})
target_link_libraries(blink PRIVATE bsp_iface)
target_compile_options(blink PRIVATE -O2)

firmware_finalize(blink)

# --- boot: standalone USB DFU bootloader ------------------------------------
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
