# The package shares the host's analytic constraint helper, but obtains the Cost
# fallback as a C service. It never compiles a second copy of the Cost engine.
add_library(nematx_seam SHARED
    src/InternalSeam.cpp
    src/InternalSeamPlugin.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Plugins/SeamPlacer/SeamPlacementConstraints.cpp")
target_compile_definitions(nematx_seam PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_seam PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_seam PRIVATE nematx_host_sdk)
set_target_properties(nematx_seam PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/seam_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_seam PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_seam PLUGIN_ID nematx_seam
    PACKAGE_ID nematx-seam PACKAGE_VERSION 0.1.0
    NAME "NematX internal seam attractor"
    DESCRIPTION "Attracts internal perimeter seams through the volume height, preserving external cost-based placement."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/seam"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-seam)

set(SEAM_RUNTIME_PATH "" CACHE STRING "Additional runtime library directories for seam checks")
get_filename_component(_seam_host_library_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
set(_seam_runtime_args --runtime-path "${_seam_host_library_dir}")
foreach(_runtime IN LISTS SEAM_RUNTIME_PATH)
    list(APPEND _seam_runtime_args --runtime-path "${_runtime}")
endforeach()
add_test(NAME seam_reader
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_reader.py")
set_tests_properties(seam_reader PROPERTIES LABELS "seam" TIMEOUT 3)
add_test(NAME seam_exports
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_exports.py"
        --host-executable "${SUPERSLICER_HOST_EXECUTABLE}" --build-dir "${CMAKE_BINARY_DIR}"
        ${_seam_runtime_args})
set_tests_properties(seam_exports PROPERTIES LABELS "seam;plugin-e2e" TIMEOUT 100)
