add_library(nematx_infill_gridvarspeed SHARED
    src/GridVarSpeedPattern.cpp src/GridVarSpeedProcess.cpp
    src/rectilinear/RectilinearFills.cpp)
target_compile_definitions(nematx_infill_gridvarspeed PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_infill_gridvarspeed PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_infill_gridvarspeed PRIVATE nematx_host_sdk)
set_target_properties(nematx_infill_gridvarspeed PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/infill_gridvarspeed_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_infill_gridvarspeed PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_infill_gridvarspeed
    PLUGIN_ID gridvarspeed
    EXTRA_PLUGIN_IDS nematx_gridvarspeed_speed nematx_gridvarspeed_recipe
    PACKAGE_ID nematx-infill-gridvarspeed
    PACKAGE_VERSION 0.1.0
    NAME "NematX grid with varying speed and Z"
    DESCRIPTION "Connected grid infill with crossing flow, speed and Z modulation."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/infill_gridvarspeed"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-infill-gridvarspeed)

add_executable(infill_gridvarspeed_tests tests/kernel_tests.cpp)
target_include_directories(infill_gridvarspeed_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME infill_gridvarspeed_kernel COMMAND infill_gridvarspeed_tests)
add_test(NAME infill_gridvarspeed_export_parser
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_smoke_parser.py")

add_executable(infill_gridvarspeed_sdk_tests tests/sdk_tests.cpp src/rectilinear/RectilinearFills.cpp)
target_include_directories(infill_gridvarspeed_sdk_tests PRIVATE src)
target_link_libraries(infill_gridvarspeed_sdk_tests PRIVATE nematx_host_sdk)
if(WIN32)
    target_compile_definitions(infill_gridvarspeed_sdk_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
get_filename_component(_gridvarspeed_host_lib_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
add_test(NAME infill_gridvarspeed_sdk
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_sdk_tests.py"
            --library-dir "${_gridvarspeed_host_lib_dir}" --exe "$<TARGET_FILE:infill_gridvarspeed_sdk_tests>")
add_test(NAME infill_gridvarspeed_import
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_import.py"
            "${SUPERSLICER_SOURCE_DIR}/src/libslic3r")
