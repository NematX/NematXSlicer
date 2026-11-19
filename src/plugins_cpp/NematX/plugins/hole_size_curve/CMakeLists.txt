# Hole compensation by diameter (hole_size_compensations_curve): every file of
# this plugin lives in this directory (sources, tests, package metadata,
# documentation).

# The regional hole compensation comes from the SDK library
# slic3r::plugin_sdk_cpp (compensate_convex_holes).
if(NOT TARGET slic3r::plugin_sdk_cpp)
    add_subdirectory("${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin"
                     "${CMAKE_BINARY_DIR}/slic3r_plugin_sdk")
endif()

add_library(nematx_hole_size_curve SHARED src/HoleSizeCurve.cpp)
target_compile_definitions(nematx_hole_size_curve PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_hole_size_curve PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_hole_size_curve PRIVATE slic3r::plugin_sdk_cpp nematx_host_sdk)
set_target_properties(nematx_hole_size_curve PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/hole_size_curve_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_hole_size_curve PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_hole_size_curve
    PLUGIN_ID nematx_hole_size_curve
    PACKAGE_ID nematx-hole-size-curve
    PACKAGE_VERSION 0.1.0
    NAME "NematX hole compensation by diameter"
    DESCRIPTION "Adds an XY compensation read from a curve of the hole diameter to convex holes."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/hole_size_curve"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-hole-size-curve
)
