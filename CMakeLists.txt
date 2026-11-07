cmake_minimum_required(VERSION 3.20)

project(nematxslicer_plugins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(SUPERSLICER_SOURCE_DIR "" CACHE PATH "SuperSlicer source checkout")
set(SUPERSLICER_HOST_LIBRARY "" CACHE FILEPATH "Linkable host library (Windows import library or Unix host archive)")
set(SUPERSLICER_HOST_LINK_LIBRARIES "" CACHE STRING "Explicit additional link libraries required by the host library")
set(SUPERSLICER_HOST_EXECUTABLE "" CACHE FILEPATH "Host executable used by debugger launches and integration checks")
set(SUPERSLICER_PLUGIN_REPOSITORY_DIR "" CACHE PATH "Optional host plugin repository receiving generated ZIPs")
set(SUPERSLICER_DEFAULT_ACTIVATED_FILE "" CACHE FILEPATH "Default plugin activation settings from the host build")
set(SUPERSLICER_VERSION_FULL "" CACHE STRING "Full SuperSlicer version")

set(default_second_layer_flow_host_datadir "${CMAKE_BINARY_DIR}/test-data/host")
set(default_second_layer_flow_datadir "${CMAKE_BINARY_DIR}/test-data/second_layer_flow")
set(SECOND_LAYER_FLOW_HOST_DATADIR "${default_second_layer_flow_host_datadir}" CACHE PATH "Baseline datadir for second-layer flow integration checks")
set(SECOND_LAYER_FLOW_DATADIR "${default_second_layer_flow_datadir}" CACHE PATH "Second-layer flow plugin datadir")

include(cmake/NematXHostSdk.cmake)

if(NOT SUPERSLICER_HOST_EXECUTABLE OR NOT EXISTS "${SUPERSLICER_HOST_EXECUTABLE}")
    message(FATAL_ERROR "Set SUPERSLICER_HOST_EXECUTABLE to an existing host executable.")
endif()
if(NOT IS_ABSOLUTE "${SUPERSLICER_HOST_EXECUTABLE}")
    message(FATAL_ERROR "SUPERSLICER_HOST_EXECUTABLE must be an absolute path: ${SUPERSLICER_HOST_EXECUTABLE}")
endif()
if(NOT EXISTS "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}")
    message(FATAL_ERROR "Default plugin activation file not found: ${SUPERSLICER_DEFAULT_ACTIVATED_FILE}")
endif()
if(NOT IS_ABSOLUTE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}")
    message(FATAL_ERROR "SUPERSLICER_DEFAULT_ACTIVATED_FILE must be an absolute path: ${SUPERSLICER_DEFAULT_ACTIVATED_FILE}")
endif()
if(SUPERSLICER_PLUGIN_REPOSITORY_DIR AND NOT IS_ABSOLUTE "${SUPERSLICER_PLUGIN_REPOSITORY_DIR}")
    message(FATAL_ERROR "SUPERSLICER_PLUGIN_REPOSITORY_DIR must be an absolute path: ${SUPERSLICER_PLUGIN_REPOSITORY_DIR}")
endif()
if(NOT SUPERSLICER_VERSION_FULL)
    message(FATAL_ERROR "Set SUPERSLICER_VERSION_FULL to the version from the configured host build")
endif()

include(cmake/NematXPluginPackage.cmake)

file(MAKE_DIRECTORY "${SECOND_LAYER_FLOW_HOST_DATADIR}/plugins")
configure_file(
    "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    "${SECOND_LAYER_FLOW_HOST_DATADIR}/plugins/activated.ini"
    COPYONLY
)

add_library(nematx_second_layer_flow SHARED
    src/SecondLayerFlow.cpp
)
target_compile_definitions(nematx_second_layer_flow PRIVATE NEMATX_SECOND_LAYER_FLOW_DLL SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_second_layer_flow PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_second_layer_flow PRIVATE nematx_host_sdk)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_second_layer_flow PRIVATE "LINKER:--no-undefined")
endif()

get_filename_component(SUPERSLICER_HOST_WORKING_DIRECTORY "${SUPERSLICER_HOST_EXECUTABLE}" DIRECTORY)

nematx_package_plugin(
    TARGET nematx_second_layer_flow
    PLUGIN_ID nematx_second_layer_flow
    PACKAGE_ID nematx-extrusion
    PACKAGE_VERSION 0.1.0
    NAME "NematX extrusion"
    DESCRIPTION "Adjusts deposited volume on the second layer using region settings."
    DATADIR "${SECOND_LAYER_FLOW_DATADIR}"
    ACTIVATION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/nematx-extrusion/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/nematx-extrusion/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    PLUGIN_REPOSITORY_DIR "${SUPERSLICER_PLUGIN_REPOSITORY_DIR}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    MODEL "${CMAKE_CURRENT_SOURCE_DIR}/tests/second_layer_flow/fixtures/cube-5mm.stl"
    CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/tests/second_layer_flow/fixtures/second-layer-flow.ini"
    DEBUGGER_COMMAND "${SUPERSLICER_HOST_EXECUTABLE}"
    DEBUGGER_WORKING_DIRECTORY "${SUPERSLICER_HOST_WORKING_DIRECTORY}"
    OUTPUT_SUBDIR nematx-extrusion
)

if(MSVC)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT nematx_second_layer_flow)
endif()

add_library(nematx_corner_stretch SHARED src/CornerStretch.cpp)
target_compile_definitions(nematx_corner_stretch PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_corner_stretch PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_corner_stretch PRIVATE nematx_host_sdk)
set_target_properties(nematx_corner_stretch PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/corner_stretch_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_corner_stretch PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_corner_stretch PLUGIN_ID nematx_corner_stretch
    PACKAGE_ID nematx-corner-stretch PACKAGE_VERSION 0.1.0
    NAME "NematX corner stretch" DESCRIPTION "Volume-preserving perimeter corner stretching."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/corner_stretch"
    ACTIVATION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/nematx-extrusion/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/nematx-extrusion/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-corner-stretch
)
enable_testing()
add_subdirectory(plugins/fan_first_layer_program)
add_subdirectory(plugins/fan_speedup)
add_subdirectory(plugins/hole_size_curve)
add_subdirectory(plugins/infill_aroundholes)
add_subdirectory(plugins/infill_gridvarspeed)
add_subdirectory(plugins/nematx_firmware)
add_subdirectory(plugins/regional_fan)
add_subdirectory(plugins/wipetower)
add_executable(corner_stretch_tests tests/corner_stretch/geometry_tests.cpp)
target_include_directories(corner_stretch_tests PRIVATE src)
add_test(NAME corner_stretch_geometry COMMAND corner_stretch_tests)

# Compile and link the public fan kernels as an external SDK consumer. The
# generic launcher supplies only the existing host C ABI library's runtime path.
add_executable(fan_policy_sdk_tests tests/fan_policy/sdk_tests.cpp)
target_link_libraries(fan_policy_sdk_tests PRIVATE slic3r::plugin_sdk_cpp nematx_host_sdk)
get_filename_component(_fan_policy_host_lib_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
add_test(NAME fan_policy_sdk
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/plugins/infill_gridvarspeed/tests/run_sdk_tests.py"
            --library-dir "${_fan_policy_host_lib_dir}" --exe "$<TARGET_FILE:fan_policy_sdk_tests>")
