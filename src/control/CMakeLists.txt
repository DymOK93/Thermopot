add_library(controller)

target_sources(controller
        PUBLIC
            "control.h"
        PRIVATE
            "control.c")

target_compile_features(controller PUBLIC c_std_99)

target_compile_options(controller PRIVATE
        ${BASIC_COMPILE_OPTIONS}
        ${ISO_COMPILE_OPTIONS}
        ${BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS})

target_link_options(controller PUBLIC ${BASIC_LINK_OPTIONS})

target_include_directories(controller 
        PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE 
            ${THERMOPOT_SOURCE_DIR})

target_link_libraries(controller PUBLIC stm32::f051r8)

