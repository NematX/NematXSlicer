# "Rectilinear around holes" infill pattern: every file of this plugin lives
# in this directory (sources, tests, package metadata, documentation).

add_library(nematx_infill_aroundholes SHARED src/AroundHolesPattern.cpp)
target_compile_definitions(nematx_infill_aroundholes PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_infill_aroundholes PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_infill_aroundholes PRIVATE nematx_host_sdk)
set_target_properties(nematx_infill_aroundholes PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/infill_aroundholes_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_infill_aroundholes PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_infill_aroundholes
    PLUGIN_ID rectilineararoundholes
    PACKAGE_ID nematx-infill-aroundholes
    PACKAGE_VERSION 0.1.0
    NAME "NematX rectilinear around holes"
    DESCRIPTION "Rectilinear infill that goes around the holes with a low-flow path."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/infill_aroundholes"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-infill-aroundholes
)

# The kernel is plain C++: its tests need no host DLL.
add_executable(infill_aroundholes_tests tests/kernel_tests.cpp)
target_include_directories(infill_aroundholes_tests PRIVATE src)
add_test(NAME infill_aroundholes_kernel COMMAND infill_aroundholes_tests)
