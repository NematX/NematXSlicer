# The standalone package compiles the shared support pipeline against the C ABI.
# Built-in registration uses the same sources, with the pipeline already in libslic3r.
set(_support_common "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Plugins/Support")
set(_support_interface_engine_sources
    src/InterfacePerimeters.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/PrintHelpers.cpp"
    "${_support_common}/Common/ClipperResults.cpp"
    "${_support_common}/Common/PerimeterFill.cpp"
    "${_support_common}/Common/SupportCounters.cpp"
    "${_support_common}/Common/SupportParams.cpp"
    "${_support_common}/Common/SupportPaths.cpp"
    "${_support_common}/Common/SupportSettings.cpp"
    "${_support_common}/Toolpath/LegacySupportPaths.cpp")
add_library(nematx_support_interface_perimeters SHARED src/SupportInterfacePerimeters.cpp ${_support_interface_engine_sources})
target_compile_definitions(nematx_support_interface_perimeters PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_support_interface_perimeters PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES)
endif()
target_link_libraries(nematx_support_interface_perimeters PRIVATE nematx_host_sdk)
set_target_properties(nematx_support_interface_perimeters PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/support_interface_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_support_interface_perimeters PRIVATE "LINKER:--no-undefined")
endif()
nematx_package_plugin(
    TARGET nematx_support_interface_perimeters
    PLUGIN_ID nematx.support.bottom_contact
    EXTRA_PLUGIN_IDS nematx.support.bottom_interface nematx.support.top_contact nematx.support.top_interface
    PACKAGE_ID nematx-support-interface-perimeters PACKAGE_VERSION 0.1.0
    NAME "NematX support interface perimeters"
    DESCRIPTION "Wraps support contacts and interfaces in configurable contour loops."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/support_interface_perimeters"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-support-interface-perimeters)

add_executable(support_interface_geometry_tests tests/geometry_tests.cpp ${_support_interface_engine_sources})
target_include_directories(support_interface_geometry_tests PRIVATE src)
target_link_libraries(support_interface_geometry_tests PRIVATE nematx_host_sdk)
if(WIN32)
    target_compile_definitions(support_interface_geometry_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES)
endif()
get_filename_component(_interface_host_library_dir "${SUPERSLICER_HOST_LIBRARY}" DIRECTORY)
set(SUPPORT_INTERFACE_RUNTIME_PATH "" CACHE STRING "Additional runtime library directories for support interface checks")
set(_interface_runtime_args --runtime-path "${_interface_host_library_dir}")
foreach(_runtime IN LISTS SUPPORT_INTERFACE_RUNTIME_PATH)
    list(APPEND _interface_runtime_args --runtime-path "${_runtime}")
endforeach()
add_test(NAME support_interface_geometry
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_geometry.py"
        --exe "$<TARGET_FILE:support_interface_geometry_tests>" ${_interface_runtime_args})
set_tests_properties(support_interface_geometry PROPERTIES LABELS "support-interface" TIMEOUT 30)
add_test(NAME support_interface_reader
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_reader.py")
set_tests_properties(support_interface_reader PROPERTIES LABELS "support-interface" TIMEOUT 5)

add_test(NAME support_interface_e2e
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_support_interfaces.py"
        --host-executable "${SUPERSLICER_HOST_EXECUTABLE}"
        --host-source "${SUPERSLICER_SOURCE_DIR}"
        --build-dir "${CMAKE_BINARY_DIR}"
        --default-activated "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
        ${_interface_runtime_args}
        --output "${CMAKE_BINARY_DIR}/temp/support-interface-e2e")
set_tests_properties(support_interface_e2e PROPERTIES LABELS "support-interface;plugin-e2e" TIMEOUT 660)
