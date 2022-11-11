add_library(tools INTERFACE)

target_compile_features(tools INTERFACE c_std_99)

target_include_directories(tools INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

