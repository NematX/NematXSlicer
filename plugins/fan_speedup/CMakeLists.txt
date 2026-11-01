# Fan anticipation and kickstart of the legacy slicer (fan_speedup_time,
# fan_speedup_overhangs, fan_kickstart): the sources, tests, package metadata and
# documentation of this plugin live in this directory; only the packaging helpers
# (cmake/) and the SDK test launcher (infill_gridvarspeed/tests) are shared.

add_library(nematx_fan_speedup SHARED src/FanSpeedup.cpp)
target_compile_definitions(nematx_fan_speedup PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_fan_speedup PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_fan_speedup PRIVATE nematx_host_sdk)
set_target_properties(nematx_fan_speedup PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/fan_speedup_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_fan_speedup PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_fan_speedup
    PLUGIN_ID nematx_fan_speedup
    PACKAGE_ID nematx-fan-speedup
    PACKAGE_VERSION 0.1.0
    NAME "NematX fan anticipation and kickstart"
    DESCRIPTION "Moves the fan speed rises earlier and kick-starts the fan (fan_speedup_time, fan_kickstart)."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/fan_speedup"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-fan-speedup
)

# The planning kernel is plain C++: its tests need no host DLL.
add_executable(fan_speedup_kernel_tests tests/kernel_tests.cpp)
target_include_directories(fan_speedup_kernel_tests PRIVATE src)
add_test(NAME fan_speedup_kernel COMMAND fan_speedup_kernel_tests)

# The tree adapter runs on stored entities through the public SDK, linked to the
# existing host library (generic launcher from the grid infill package).
add_executable(fan_speedup_sdk_tests tests/sdk_tests.cpp)
target_include_directories(fan_speedup_sdk_tests PRIVATE src)
target_link_libraries(fan_speedup_sdk_tests PRIVATE nematx_host_sdk)
if(WIN32)
    target_compile_definitions(fan_speedup_sdk_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
get_filename_component(_fan_speedup_host_lib_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
add_test(NAME fan_speedup_sdk
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/plugins/infill_gridvarspeed/tests/run_sdk_tests.py"
            --library-dir "${_fan_speedup_host_lib_dir}" --exe "$<TARGET_FILE:fan_speedup_sdk_tests>")
