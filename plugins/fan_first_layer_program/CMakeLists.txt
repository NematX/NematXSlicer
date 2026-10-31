# First-layer fan program of the legacy slicer (enable_fan_first_layers,
# fan_speed_layer_gradient): every file of this plugin lives in this directory
# (sources, tests, package metadata, documentation).

add_library(nematx_fan_first_layer_program SHARED src/FanFirstLayerProgram.cpp)
target_compile_definitions(nematx_fan_first_layer_program PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_fan_first_layer_program PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_fan_first_layer_program PRIVATE nematx_host_sdk)
set_target_properties(nematx_fan_first_layer_program PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/fan_first_layer_program_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_fan_first_layer_program PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_fan_first_layer_program
    PLUGIN_ID nematx_fan_first_layer_program
    PACKAGE_ID nematx-fan-first-layer-program
    PACKAGE_VERSION 0.1.0
    NAME "NematX first-layer fan program"
    DESCRIPTION "First-layer fan program of the legacy slicer: enabled, disabled and ramped layers."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/fan_first_layer_program"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-fan-first-layer-program
)

# The program kernel is plain C++: its tests need no host DLL.
add_executable(fan_first_layer_program_tests tests/kernel_tests.cpp)
target_include_directories(fan_first_layer_program_tests PRIVATE src)
add_test(NAME fan_first_layer_program_kernel COMMAND fan_first_layer_program_tests)