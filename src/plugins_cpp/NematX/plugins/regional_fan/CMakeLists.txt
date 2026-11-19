# Regional fan speed overrides (print_fan_speed and its per-role variants):
# every file of this plugin lives in this directory (sources, tests, package
# metadata, documentation).

add_library(nematx_regional_fan SHARED src/RegionalFan.cpp)
target_compile_definitions(nematx_regional_fan PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_regional_fan PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_regional_fan PRIVATE nematx_host_sdk)
set_target_properties(nematx_regional_fan PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/regional_fan_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_regional_fan PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_regional_fan
    PLUGIN_ID nematx_regional_fan
    PACKAGE_ID nematx-regional-fan
    PACKAGE_VERSION 0.1.0
    NAME "NematX regional fan overrides"
    DESCRIPTION "Forces the fan speed of the paths of a region, for all of them or per role."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/regional_fan"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-regional-fan
)

# The tree pass runs on stored entities through the public SDK, linked to the
# existing host library (generic launcher from the grid infill package).
add_executable(regional_fan_sdk_tests tests/sdk_tests.cpp)
target_include_directories(regional_fan_sdk_tests PRIVATE src)
target_link_libraries(regional_fan_sdk_tests PRIVATE nematx_host_sdk)
if(WIN32)
    target_compile_definitions(regional_fan_sdk_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
get_filename_component(_regional_fan_host_lib_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
add_test(NAME regional_fan_sdk
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/plugins/infill_gridvarspeed/tests/run_sdk_tests.py"
            --library-dir "${_regional_fan_host_lib_dir}" --exe "$<TARGET_FILE:regional_fan_sdk_tests>")
