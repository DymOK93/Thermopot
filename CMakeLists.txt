cmake_minimum_required(VERSION 3.15)
project(Thermopot)
enable_language(C ASM)

set(TP_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_TOOLCHAIN_FILE "${TP_DIR}/tools/cmake/ArmNoneEabiGcc.cmake")
set(LINKER_SCRIPT "${TP_DIR}/tools/link/STM32F051R8_FLASH.lds")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${TP_DIR}/build")

set(ARCHITECTURE_SPECIFIC_OPTIONS
        -mcpu=cortex-m0
        -mthumb
        -mfpu=auto
        -mfloat-abi=soft)

set(BASIC_COMPILE_OPTIONS
        ${ARCHITECTURE_SPECIFIC_OPTIONS}
        -fdata-sections -ffunction-sections
        -Wall -Wextra
        -Wduplicated-branches -Wduplicated-cond 
        -Wlogical-op
        -Wcast-qual)

set(ISO_COMPILE_OPTIONS 
        -Werror 
        -Wpedantic)

set(BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS
        $<$<CONFIG:Debug>:-Og -g3>
        $<$<CONFIG:Release>:-O3 -flto>)

set(BASIC_LINK_OPTIONS
        ${ARCHITECTURE_SPECIFIC_OPTIONS}
        -specs=nano.specs
        -lc -lm -lnosys)

add_subdirectory(external)
add_subdirectory(src)