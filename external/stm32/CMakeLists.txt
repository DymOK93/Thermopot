set(STM32_RUNTIME_INCLUDES
	    "include/core_cm0.h"
	    "include/core_cmInstr.h"
	    "include/core_cmFunc.h"
        "include/stm32f0xx.h"
        "include/system_stm32f0xx.h")

set(STM32_RUNTIME_SOURCES
        "src/init.c"
        "src/startup_stm32f051x8.s"
        "src/system_stm32f0xx.c")

add_library(stm32_f051r8 STATIC ${STM32_RUNTIME_SOURCES})
        
target_sources(stm32_f051r8 PUBLIC
        ${STM32_RUNTIME_INCLUDES}
        ${STM32_RUNTIME_SOURCES})

add_library(stm32::f051r8 ALIAS stm32_f051r8)

target_compile_definitions(stm32_f051r8 PUBLIC 
        STM32F051
        STM32F051x8)

target_compile_features(stm32_f051r8 PUBLIC c_std_99)

target_compile_options(stm32_f051r8 PUBLIC
        ${BASIC_COMPILE_OPTIONS}
        ${OPT_COMPILE_OPTIONS})

target_link_options(stm32_f051r8 PUBLIC
        ${BASIC_LINK_OPTIONS})

target_include_directories(stm32_f051r8 PUBLIC
		"${CMAKE_CURRENT_SOURCE_DIR}/include")

