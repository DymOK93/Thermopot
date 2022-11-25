set(THERMOPOT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(control)
add_subdirectory(heat)
add_subdirectory(ssi)
add_subdirectory(thermal)

add_baremetal_executable(thermopot "thermopot.c")

target_compile_features(thermopot PRIVATE c_std_99)

target_compile_options(thermopot PRIVATE
        ${BASIC_COMPILE_OPTIONS}
        ${ISO_COMPILE_OPTIONS}
        ${BUILD_TYPE_SPECIFIC_COMPILE_OPTIONS})

target_link_options(thermopot PRIVATE
        -T ${LINKER_SCRIPT}
        ${BASIC_LINK_OPTIONS}
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Wl,--print-memory-usage)
        
target_include_directories(thermopot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(thermopot PUBLIC 
        controller
        heat_manager
        ssi_manager
        thermal_manager
        stm32::f051r8)


