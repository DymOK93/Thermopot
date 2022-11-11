add_library(ssi_manager)

target_sources(ssi_manager
        PUBLIC
            "ssi.h"
        PRIVATE
            "ssi.c")

target_compile_features(ssi_manager PUBLIC c_std_99)

target_compile_options(ssi_manager PRIVATE
        ${BASIC_COMPILE_OPTIONS}
        ${ISO_COMPILE_OPTIONS}
        ${BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS})

target_link_options(ssi_manager PUBLIC ${BASIC_LINK_OPTIONS})

target_include_directories(ssi_manager 
        PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE 
            ${PHOTO_VIEWER_SOURCE_DIR})

target_link_libraries(ssi_manager PUBLIC stm32::f051r8)

