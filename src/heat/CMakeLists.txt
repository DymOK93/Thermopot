add_library(heat_manager)

target_sources(heat_manager
        PUBLIC
            "heat.h"
        PRIVATE
            "heat.c")

target_compile_features(heat_manager PUBLIC c_std_99)

target_compile_options(heat_manager PRIVATE
        ${BASIC_COMPILE_OPTIONS}
        ${ISO_COMPILE_OPTIONS}
        ${BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS})

target_link_options(heat_manager PUBLIC ${BASIC_LINK_OPTIONS})

target_include_directories(heat_manager 
        PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE 
            ${PHOTO_VIEWER_SOURCE_DIR})

target_link_libraries(heat_manager PUBLIC stm32::f051r8)

