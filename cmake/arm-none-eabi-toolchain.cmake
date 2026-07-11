# Toolchain file for the ARM GNU bare-metal toolchain.
#
# The toolchain is downloaded automatically into ./tools on first configure.
# Toolchain files are processed before the compiler check, so the download must
# happen here.  Version tracks the latest ARM GNU release (arm-none-eabi).

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# --- Auto-download the ARM GNU toolchain -----------------------------------
set(TC_VERSION  "15.2.rel1")
set(TC_NAME     "arm-gnu-toolchain-${TC_VERSION}-x86_64-arm-none-eabi")
set(TC_URL      "https://developer.arm.com/-/media/Files/downloads/gnu/${TC_VERSION}/binrel/${TC_NAME}.tar.xz")
set(TOOLS_DIR   "${CMAKE_CURRENT_LIST_DIR}/../tools")
set(GCC_DIR     "${TOOLS_DIR}/${TC_NAME}")
set(GCC_BIN     "${GCC_DIR}/bin")

if(NOT EXISTS "${GCC_BIN}/arm-none-eabi-gcc")
    message(STATUS "Downloading ARM GNU toolchain ${TC_VERSION} (~155 MB) ...")
    file(MAKE_DIRECTORY "${TOOLS_DIR}")
    set(_tarball "${TOOLS_DIR}/${TC_NAME}.tar.xz")
    if(NOT EXISTS "${_tarball}")
        file(DOWNLOAD "${TC_URL}" "${_tarball}" SHOW_PROGRESS STATUS _dl)
        list(GET _dl 0 _dl_code)
        if(NOT _dl_code EQUAL 0)
            file(REMOVE "${_tarball}")
            message(FATAL_ERROR "Toolchain download failed: ${_dl}")
        endif()
    endif()
    message(STATUS "Extracting toolchain ...")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xJf "${_tarball}"
        WORKING_DIRECTORY "${TOOLS_DIR}"
        RESULT_VARIABLE _untar)
    file(REMOVE "${_tarball}")
    if(NOT _untar EQUAL 0)
        message(FATAL_ERROR "Toolchain extraction failed")
    endif()
endif()

# --- Compilers / tools ------------------------------------------------------
set(CMAKE_C_COMPILER   "${GCC_BIN}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${GCC_BIN}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY      "${GCC_BIN}/arm-none-eabi-objcopy" CACHE FILEPATH "")
set(CMAKE_SIZE         "${GCC_BIN}/arm-none-eabi-size"    CACHE FILEPATH "")

# Don't try to link a full executable during the compiler check (no _start yet).
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
