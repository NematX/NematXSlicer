# Standalone support helpers use only the C ABI. The embedded module reuses
# the host's compiled helpers instead of linking a second copy.
set(_support "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Plugins/Support")
set(_gap_sources
    src/InterfaceGapFill.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/PrintHelpers.cpp"
    "${_support}/Common/ClipperResults.cpp"
    "${_support}/Common/PerimeterFill.cpp"
    "${_support}/Common/SupportCounters.cpp"
    "${_support}/Common/SupportParams.cpp"
    "${_support}/Common/SupportPaths.cpp"
    "${_support}/Common/SupportSettings.cpp"
    "${_support}/Toolpath/LegacySupportPaths.cpp")
add_library(nematx_support_interface_gap_fill SHARED src/SupportInterfaceGapFill.cpp ${_gap_sources})
target_compile_definitions(nematx_support_interface_gap_fill PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_support_interface_gap_fill PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES)
endif()
target_link_libraries(nematx_support_interface_gap_fill PRIVATE nematx_host_sdk)
set_target_properties(nematx_support_interface_gap_fill PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/support_gap_fill_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_support_interface_gap_fill PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_support_interface_gap_fill
    PLUGIN_ID nematx.support.interface_gap_fill
    PACKAGE_ID nematx-support-interface-gap-fill PACKAGE_VERSION 0.1.0
    NAME "NematX support interface gap fill"
    DESCRIPTION "Completes narrow uncovered areas of dense support interfaces."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/support_interface_gap_fill"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-support-interface-gap-fill)

add_executable(support_gap_fill_geometry_tests tests/geometry_tests.cpp ${_gap_sources})
target_include_directories(support_gap_fill_geometry_tests PRIVATE src)
target_link_libraries(support_gap_fill_geometry_tests PRIVATE nematx_host_sdk)
if(WIN32)
    target_compile_definitions(support_gap_fill_geometry_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES)
endif()
get_filename_component(_gap_runtime "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
set(SUPPORT_GAP_FILL_RUNTIME_PATH "" CACHE STRING "Additional runtime DLL directories for support gap-fill checks")
set(_gap_runtime_args --runtime-path "${_gap_runtime}")
foreach(_runtime IN LISTS SUPPORT_GAP_FILL_RUNTIME_PATH)
    list(APPEND _gap_runtime_args --runtime-path "${_runtime}")
endforeach()
add_test(NAME support_gap_fill_geometry
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/plugins/support_interface_perimeters/tests/run_geometry.py"
        --exe "$<TARGET_FILE:support_gap_fill_geometry_tests>" ${_gap_runtime_args})
set_tests_properties(support_gap_fill_geometry PROPERTIES LABELS "support-interface-gap-fill" TIMEOUT 60)
add_test(NAME support_gap_fill_e2e
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_gap_fill.py"
        --host "${SUPERSLICER_HOST_EXECUTABLE}" --host-source "${SUPERSLICER_SOURCE_DIR}"
        --prepared "${CMAKE_BINARY_DIR}/test-data/support_interface_gap_fill"
        --output "${CMAKE_BINARY_DIR}/temp/support-gap-fill-e2e" ${_gap_runtime_args})
set_tests_properties(support_gap_fill_e2e PROPERTIES LABELS "support-interface-gap-fill;plugin-e2e" TIMEOUT 540)
