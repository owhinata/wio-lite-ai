# ============================================================================
#  Wio Lite AI (STM32H725AEI6) firmware, all XIP from external OCTOSPI2 flash.
#
#  Firmwares (each packaged as <name>.uf2 for the TinyUF2 bootloader):
#    blink -- LED blink reference app.
#    boot  -- custom DFU bootloader, built app-first (linked at 0x70000000 and run
#             under the intact TinyUF2 so board #2 stays recoverable; see HANDOFF.md).
#
#  Configure (downloads the ARM toolchain into ./tools on first run):
#    cmake -B build -G Ninja \
#          -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
#  Build:                 cmake --build build          # -> build/{blink,boot}.elf/.bin/.hex/.uf2
#  Flash (UF2 dropzone):  cmake --build build --target flash-blink   (or flash-boot)
# ============================================================================
cmake_minimum_required(VERSION 3.20)

# Fetch HAL/CMSIS submodules if any is missing.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32h7xx_hal_driver/Src/stm32h7xx_hal.c"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_core/Include/core_cm7.h"
   OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_h7/Include/stm32h725xx.h")
    message(STATUS "Initializing git submodules ...")
    execute_process(
        COMMAND git submodule update --init --recursive
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
set(LDSCRIPT   "${CMAKE_SOURCE_DIR}/ldscript/STM32H725AEIx_XIP.ld")

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
    ${MCU_OPTS} -specs=nano.specs -specs=nosys.specs "-T${LDSCRIPT}"
    -Wl,--gc-sections -Wl,--print-memory-usage)

# Compile every HAL driver source; each file self-guards on its
# HAL_<module>_MODULE_ENABLED define, so unused ones become empty objects.
file(GLOB HAL_SOURCES "${HAL_DIR}/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX "_template\\.c$")

# --- Post-build artifacts + UF2 + flash target -----------------------------
function(firmware_finalize tgt)
    target_link_options(${tgt} PRIVATE -Wl,-Map=${tgt}.map,--cref)
    set_target_properties(${tgt} PROPERTIES LINK_DEPENDS "${LDSCRIPT}")
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary -S $<TARGET_FILE:${tgt}> ${tgt}.bin
        COMMAND "${CMAKE_OBJCOPY}" -O ihex   $<TARGET_FILE:${tgt}> ${tgt}.hex
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/scripts/bin2uf2.py"
                ${tgt}.bin ${tgt}.uf2 0x70000000 0x6db66082
        COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:${tgt}>
        BYPRODUCTS ${tgt}.bin ${tgt}.hex ${tgt}.uf2
        COMMENT "objcopy -> ${tgt}.bin/.hex ; bin2uf2 -> ${tgt}.uf2")

    # `flash-<tgt>`: drop the .uf2 onto the mounted TinyUF2 mass-storage drive.
    # Enter the bootloader first (double-tap RST or hold USER button while pressing
    # RST) so the "Arduino"-labelled drive appears at /media/$USER/Arduino.  Named
    # per firmware so `blink` and `boot` don't collide on a single `flash` target.
    add_custom_target(flash-${tgt}
        COMMAND "${CMAKE_COMMAND}" -E copy ${tgt}.uf2 "/media/$ENV{USER}/Arduino/${tgt}.uf2"
        DEPENDS ${tgt}
        USES_TERMINAL
        COMMENT "Copying ${tgt}.uf2 to the TinyUF2 UF2 drive")
endfunction()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_executable(blink
    "${CMAKE_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
    "${CMSIS_DEV}/Source/Templates/gcc/startup_stm32h725xx.s"
    ${HAL_SOURCES})
target_link_libraries(blink PRIVATE bsp_iface)
target_compile_options(blink PRIVATE -O2)

firmware_finalize(blink)

# --- boot: custom DFU bootloader (Phase 1.x app-first) ----------------------
# Same XIP-at-0x70000000 build pipeline as blink, sharing the hands-off-the-clocks
# system layer and the CMSIS startup, plus the TinyUSB device stack + standard DFU
# class (Phase 1.2).  TinyUSB and its config/descriptors are compiled into ONLY this
# target (blink stays a pure bare-metal app).
set(TUSB_DIR "${CMAKE_SOURCE_DIR}/lib/tinyuf2/lib/tinyusb/src")

# Device-mode subset of TinyUSB: core + FIFO + device stack + DFU class + the
# Synopsys dwc2 device controller driver (the H725's USB1_OTG_HS is a dwc2 core).
set(TINYUSB_SOURCES
    "${TUSB_DIR}/tusb.c"
    "${TUSB_DIR}/common/tusb_fifo.c"
    "${TUSB_DIR}/device/usbd.c"
    "${TUSB_DIR}/device/usbd_control.c"
    "${TUSB_DIR}/class/dfu/dfu_device.c"
    "${TUSB_DIR}/class/cdc/cdc_device.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dcd_dwc2.c"
    "${TUSB_DIR}/portable/synopsys/dwc2/dwc2_common.c")

add_executable(boot
    "${CMAKE_SOURCE_DIR}/boot/main.c"
    "${CMAKE_SOURCE_DIR}/boot/usb_descriptors.c"
    "${CMAKE_SOURCE_DIR}/boot/dfu_callbacks.c"
    "${CMAKE_SOURCE_DIR}/boot/ospi_ram.c"
    "${CMAKE_SOURCE_DIR}/src/system_stm32h7xx.c"
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

firmware_finalize(boot)
