# The policy uses only public SDK views; routing remains a host service.
add_library(nematx_top_surface_retraction SHARED src/TopSurfaceRetraction.cpp)
target_compile_definitions(nematx_top_surface_retraction PRIVATE SLIC3R_PLUGIN_EXPORTS)
target_link_libraries(nematx_top_surface_retraction PRIVATE nematx_host_sdk)
set_target_properties(nematx_top_surface_retraction PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/top_surface_retraction_import/$<CONFIG>")
if(WIN32)
    target_compile_definitions(nematx_top_surface_retraction PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_top_surface_retraction PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_top_surface_retraction
    PLUGIN_ID nematx_top_surface_retraction
    PACKAGE_ID nematx-top-surface-retraction
    PACKAGE_VERSION 0.1.0
    NAME "NematX top-surface retraction"
    DESCRIPTION "Retains travel retractions over exposed top surfaces with the common distance and modifier rules."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/top_surface_retraction"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-top-surface-retraction)

set(TOP_SURFACE_RETRACTION_RUNTIME_PATH "" CACHE STRING "Runtime directories for top-surface retraction exports")
get_filename_component(_top_host_library_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
set(_top_runtime_args --runtime-path "${_top_host_library_dir}")
foreach(_runtime IN LISTS TOP_SURFACE_RETRACTION_RUNTIME_PATH)
    list(APPEND _top_runtime_args --runtime-path "${_runtime}")
endforeach()
add_test(NAME top_surface_retraction_package
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_exports.py"
        --host "${SUPERSLICER_HOST_EXECUTABLE}"
        --defaults "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
        --package-dir "$<TARGET_FILE_DIR:nematx_top_surface_retraction>"
        --output "${CMAKE_BINARY_DIR}/temp/f21/top-surface-exports" ${_top_runtime_args})
set_tests_properties(top_surface_retraction_package PROPERTIES LABELS "retraction;plugin-e2e" TIMEOUT 900)
