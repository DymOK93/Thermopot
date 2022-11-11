add_library(thermal_manager)

target_sources(thermal_manager
        PUBLIC
            "thermal.h"
        PRIVATE
            "thermal.c")

target_compile_features(thermal_manager PUBLIC c_std_99)

target_compile_options(thermal_manager PRIVATE
        ${BASIC_COMPILE_OPTIONS}
        ${ISO_COMPILE_OPTIONS}
        ${BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS})

target_link_options(thermal_manager PUBLIC ${BASIC_LINK_OPTIONS})

target_include_directories(thermal_manager 
        PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE 
            ${PHOTO_VIEWER_SOURCE_DIR})

target_link_libraries(thermal_manager PUBLIC stm32::f051r8)

