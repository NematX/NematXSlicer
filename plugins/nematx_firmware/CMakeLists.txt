# NematX firmware package: G-code firmware gcode.firmware.nematx and reader
# gcode.reader.nematx. Every file of this package lives in this directory.

# The firmware session and reader derive from SDK classes compiled into the
# static slic3r::plugin_sdk_cpp library (Boost and fast_float headers needed,
# for example with CMAKE_PREFIX_PATH set to the slicer dependencies).
if(NOT TARGET slic3r::plugin_sdk_cpp)
    add_subdirectory("${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin"
                     "${CMAKE_BINARY_DIR}/slic3r_plugin_sdk")
endif()

add_library(nematx_firmware SHARED
    src/NematXFirmwarePlugin.cpp
    src/NematXFirmwareSession.cpp
    src/NematXGCodeReader.cpp
)
target_compile_definitions(nematx_firmware PRIVATE SLIC3R_PLUGIN_EXPORTS)
if(WIN32)
    target_compile_definitions(nematx_firmware PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()
target_link_libraries(nematx_firmware PRIVATE slic3r::plugin_sdk_cpp nematx_host_sdk)
set_target_properties(nematx_firmware PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/nematx_firmware_import/$<CONFIG>")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(nematx_firmware PRIVATE "LINKER:--no-undefined")
endif()

set(NEMATX_FIRMWARE_DATADIR "${CMAKE_BINARY_DIR}/test-data/nematx_firmware")
nematx_package_plugin(
    TARGET nematx_firmware
    PLUGIN_ID gcode.firmware.nematx
    EXTRA_PLUGIN_IDS gcode.reader.nematx
    PACKAGE_ID nematx-firmware
    PACKAGE_VERSION 0.1.0
    NAME "NematX firmware"
    DESCRIPTION "G-code firmware and reader for NematX printers."
    DATADIR "${NEMATX_FIRMWARE_DATADIR}"
    ACTIVATION_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/prepare_activated_ini.py"
    ABI_SCRIPT "${SUPERSLICER_SOURCE_DIR}/src/plugins_cpp/write_plugin_abi_manifest.py"
    DESCRIPTION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/description.ini.in"
    VERSION_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/package/version-base.ini.in"
    SLICER_VERSION "${SUPERSLICER_VERSION_FULL}"
    DEFAULT_ACTIVATED_FILE "${SUPERSLICER_DEFAULT_ACTIVATED_FILE}"
    OUTPUT_SUBDIR nematx-firmware
)

# The syntax rules are plain C++: their tests need no host DLL.
add_executable(nematx_firmware_syntax_tests tests/syntax_tests.cpp)
target_include_directories(nematx_firmware_syntax_tests PRIVATE src)
add_test(NAME nematx_firmware_syntax COMMAND nematx_firmware_syntax_tests)
