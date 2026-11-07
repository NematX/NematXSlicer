# NematX wipe tower manager (step_wipetower_plugin = nematx_wipetower): the
# sources, tests, package metadata and documentation of this plugin live in this
# directory; only the packaging helpers (cmake/) are shared.

add_library(nematx_wipetower SHARED
    src/WipeTowerManager.cpp src/WipeTowerLayout.cpp src/TowerFootprints.cpp
    src/rectilinear/RectilinearFills.cpp src/rectilinear/RectilinearEdgeGrid.cpp)
target_include_directories(nematx_wipetower PRIVATE src src/rectilinear)
target_compile_definitions(nematx_wipetower PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    # The copied host fills use M_PI, which MSVC defines only on request.
    target_compile_definitions(nematx_wipetower PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES)
endif()
target_link_libraries(nematx_wipetower PRIVATE nematx_host_sdk)
set_target_properties(nematx_wipetower PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wipetower_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_wipetower PRIVATE "LINKER:--no-undefined")
endif()

nematx_package_plugin(
    TARGET nematx_wipetower
    PLUGIN_ID nematx_wipetower
    PACKAGE_ID nematx-wipetower
    PACKAGE_VERSION 0.1.0
    NAME "NematX wipe tower manager"
    DESCRIPTION "Wipe tower manager of the NematX printers."
    DATADIR "${CMAKE_BINARY_DIR}/test-data/wipetower"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-wipetower
)

add_test(NAME wipetower_import
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_import.py"
            "${SUPERSLICER_SOURCE_DIR}/src/libslic3r")
