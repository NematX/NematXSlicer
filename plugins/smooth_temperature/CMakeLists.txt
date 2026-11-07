# Smoothed temperature changes of the legacy slicer (print_temperature_smooth_change):
# every file of this plugin lives in this directory (sources, tests, package
# metadata, documentation).

# The plugin derives from the host's default layer temperature planner. The host
# library exports only the C API, so the package also compiles the planner's two
# host sources; they use only the plugin SDK headers.
add_library(nematx_smooth_temperature SHARED
    src/SmoothTemperature.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Plugins/LayerExtrusionEdit/DefaultTemperature.cpp"
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Plugins/LayerExtrusionEdit/ExtrusionProcessParameterHelpers.cpp"
)
target_compile_definitions(nematx_smooth_temperature PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_smooth_temperature PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_smooth_temperature PRIVATE nematx_host_sdk)
set_target_properties(nematx_smooth_temperature PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/smooth_temperature_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_smooth_temperature PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_smooth_temperature
    PLUGIN_ID nematx_smooth_temperature
    PACKAGE_ID nematx-smooth-temperature
    PACKAGE_VERSION 0.1.0
    NAME "NematX temperature smoothing"
    DESCRIPTION "Spreads general nozzle temperature changes of each object over several layers."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/smooth_temperature"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-smooth-temperature
)

# The ramp kernel is plain C++: its tests need no host DLL.
add_executable(smooth_temperature_tests tests/kernel_tests.cpp)
target_include_directories(smooth_temperature_tests PRIVATE src)
add_test(NAME smooth_temperature_kernel COMMAND smooth_temperature_tests)
