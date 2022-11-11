# Redistribution and use is allowed under the MIT license.
# Copyright (c) 2022 Dmitry Bolshakov. All rights reserved.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX "arm-none-eabi-")

if(MINGW OR CYGWIN OR WIN32)
    set(BINUTILS_SEARCH_CMD "where")
elseif(UNIX OR APPLE)
    set(BINUTILS_SEARCH_CMD "which")
endif()

execute_process(
  COMMAND ${BINUTILS_SEARCH_CMD} "${TOOLCHAIN_PREFIX}gcc"
  OUTPUT_VARIABLE GCC_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

get_filename_component(ARM_BINUTILS_DIR ${GCC_PATH} DIRECTORY)
find_program(CMAKE_ASM_COMPILER NAMES "${CMAKE_C_COMPILER}")
find_program(CMAKE_C_COMPILER NAMES "${TOOLCHAIN_PREFIX}gcc" PATHS ${ARM_BINUTILS_DIR})
find_program(CMAKE_CXX_COMPILER NAMES "${TOOLCHAIN_PREFIX}c++" PATHS ${ARM_BINUTILS_DIR})
find_program(CMAKE_SIZE_UTIL NAMES "${TOOLCHAIN_PREFIX}size" PATHS ${ARM_BINUTILS_DIR})
find_program(CMAKE_OBJCOPY NAMES "${TOOLCHAIN_PREFIX}objcopy" PATHS ${ARM_BINUTILS_DIR})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

function(add_baremetal_executable _target)
    cmake_parse_arguments(BM "" "" "" ${ARGN})

    add_executable(${_target} ${BM_UNPARSED_ARGUMENTS})
    set_target_properties(${_target} PROPERTIES SUFFIX ".elf")

    set(BM_FIRMWARE_PATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_target}")
    add_custom_command(
       TARGET ${_target} POST_BUILD
       COMMAND "${CMAKE_OBJCOPY}" -O ihex "${BM_FIRMWARE_PATH}.elf" "${BM_FIRMWARE_PATH}.hex"
       COMMAND "${CMAKE_OBJCOPY}" -O binary "${BM_FIRMWARE_PATH}.elf" "${BM_FIRMWARE_PATH}.bin"
       VERBATIM USES_TERMINAL)
endfunction()

