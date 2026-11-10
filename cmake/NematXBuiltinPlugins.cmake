# Compile the independently maintained NematX modules into the host without
# editing their sources or their standalone shared-library/package build.
set(NEMATX_PLUGINS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../NematXSlicer_plugins"
    CACHE PATH "Source checkout of the independently packaged NematX plugins")
get_filename_component(NEMATX_PLUGINS_SOURCE_DIR "${NEMATX_PLUGINS_SOURCE_DIR}" ABSOLUTE)
set(_nematx_builtin_default OFF)
if(SLIC3R_APP_KEY STREQUAL "NematXSlicer" AND EXISTS "${NEMATX_PLUGINS_SOURCE_DIR}/CMakeLists.txt")
    set(_nematx_builtin_default ON)
endif()
option(SLIC3R_NEMATX_BUILTIN_PLUGINS "Embed the NematX plugins into the host" ${_nematx_builtin_default})

set_property(GLOBAL PROPERTY SLIC3R_NEMATX_DEFAULT_ACTIVATIONS "")
set_property(GLOBAL PROPERTY SLIC3R_NEMATX_BUILTIN_DECLARATIONS "")
set_property(GLOBAL PROPERTY SLIC3R_NEMATX_BUILTIN_REGISTRATIONS "")

# Each module normally exports the same three C symbols. Unique names let the
# original translation units coexist in the host archive without DLL exports.
function(nematx_add_builtin_module name)
    cmake_parse_arguments(MODULE "" "" "SOURCES;PLUGIN_IDS;DEFINITIONS" ${ARGN})
    set(_sources)
    foreach(_source IN LISTS MODULE_SOURCES)
        list(APPEND _sources "${NEMATX_PLUGINS_SOURCE_DIR}/${_source}")
    endforeach()
    add_library(nematx_builtin_${name} OBJECT ${_sources})
    target_compile_features(nematx_builtin_${name} PRIVATE cxx_std_17)
    target_include_directories(nematx_builtin_${name} PRIVATE "${PROJECT_SOURCE_DIR}/..")
    target_link_libraries(nematx_builtin_${name} PRIVATE boost_headeronly)
    target_compile_definitions(nematx_builtin_${name} PRIVATE
        register_plugin=nematx_builtin_register_${name}
        slic3r_plugin_abi_version=nematx_builtin_abi_${name}
        slic3r_plugin_abi_manifest=nematx_builtin_manifest_${name}
        ${MODULE_DEFINITIONS})
    set_target_properties(nematx_builtin_${name} PROPERTIES
        POSITION_INDEPENDENT_CODE ON FOLDER "plugins/NematX built-in")
    if(MSVC)
        target_compile_definitions(nematx_builtin_${name} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
        target_compile_options(nematx_builtin_${name} PRIVATE /EHsc /utf-8 /bigobj)
    endif()
    target_sources(libslic3r PRIVATE $<TARGET_OBJECTS:nematx_builtin_${name}>)
    set_property(GLOBAL APPEND_STRING PROPERTY SLIC3R_NEMATX_BUILTIN_DECLARATIONS
        "extern \"C\" void nematx_builtin_register_${name}(orchestrator_handle *);\n")

    # An installed standalone package takes precedence. Test the registered
    # set, not the active set, because activation happens after all registration.
    set(_condition "true")
    foreach(_id IN LISTS MODULE_PLUGIN_IDS)
        string(APPEND _condition " && host.get_plugin(\"${_id}\") == nullptr")
        set_property(GLOBAL APPEND_STRING PROPERTY SLIC3R_NEMATX_DEFAULT_ACTIVATIONS "${_id} = 1\n")
    endforeach()
    set_property(GLOBAL APPEND_STRING PROPERTY SLIC3R_NEMATX_BUILTIN_REGISTRATIONS
        "    if (${_condition})\n        nematx_builtin_register_${name}(orchestrator);\n")
endfunction()

if(SLIC3R_NEMATX_BUILTIN_PLUGINS)
    if(NOT EXISTS "${NEMATX_PLUGINS_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "NematX plugin sources not found: ${NEMATX_PLUGINS_SOURCE_DIR}")
    endif()
    nematx_add_builtin_module(second_layer_flow
        SOURCES src/SecondLayerFlow.cpp
        PLUGIN_IDS nematx_second_layer_flow
        DEFINITIONS NEMATX_SECOND_LAYER_FLOW_DLL)
    nematx_add_builtin_module(corner_stretch
        SOURCES src/CornerStretch.cpp PLUGIN_IDS nematx_corner_stretch)
    nematx_add_builtin_module(fan_first_layer_program
        SOURCES plugins/fan_first_layer_program/src/FanFirstLayerProgram.cpp
        PLUGIN_IDS nematx_fan_first_layer_program)
    nematx_add_builtin_module(fan_speedup
        SOURCES plugins/fan_speedup/src/FanSpeedup.cpp PLUGIN_IDS nematx_fan_speedup)
    nematx_add_builtin_module(hole_size_curve
        SOURCES plugins/hole_size_curve/src/HoleSizeCurve.cpp PLUGIN_IDS nematx_hole_size_curve)
    nematx_add_builtin_module(infill_aroundholes
        SOURCES plugins/infill_aroundholes/src/AroundHolesPattern.cpp PLUGIN_IDS rectilineararoundholes)
    nematx_add_builtin_module(infill_gridvarspeed
        SOURCES plugins/infill_gridvarspeed/src/GridVarSpeedPattern.cpp
            plugins/infill_gridvarspeed/src/GridVarSpeedProcess.cpp
            plugins/infill_gridvarspeed/src/rectilinear/RectilinearFills.cpp
        PLUGIN_IDS gridvarspeed nematx_gridvarspeed_speed nematx_gridvarspeed_recipe)
    nematx_add_builtin_module(firmware
        SOURCES plugins/nematx_firmware/src/NematXFirmwarePlugin.cpp
            plugins/nematx_firmware/src/NematXFirmwareSession.cpp
            plugins/nematx_firmware/src/NematXGCodeReader.cpp
        PLUGIN_IDS gcode.firmware.nematx gcode.reader.nematx)
    nematx_add_builtin_module(regional_fan
        SOURCES plugins/regional_fan/src/RegionalFan.cpp PLUGIN_IDS nematx_regional_fan)
    nematx_add_builtin_module(wipetower
        SOURCES plugins/wipetower/src/WipeTowerManager.cpp
            plugins/wipetower/src/WipeTowerLayout.cpp
            plugins/wipetower/src/TowerFootprints.cpp
            plugins/wipetower/src/FilamentSections.cpp
            plugins/wipetower/src/rectilinear/RectilinearFills.cpp
            plugins/wipetower/src/rectilinear/RectilinearEdgeGrid.cpp
        PLUGIN_IDS nematx_wipetower)
    # The module derives from the host's LayerTemperature, already in libslic3r.
    nematx_add_builtin_module(smooth_temperature
        SOURCES plugins/smooth_temperature/src/SmoothTemperature.cpp PLUGIN_IDS nematx_smooth_temperature)
    nematx_add_builtin_module(support_interface_perimeters
        SOURCES plugins/support_interface_perimeters/src/SupportInterfacePerimeters.cpp
            plugins/support_interface_perimeters/src/InterfacePerimeters.cpp
        PLUGIN_IDS nematx.support.bottom_contact nematx.support.bottom_interface
            nematx.support.top_contact nematx.support.top_interface)
    message(STATUS "NematX plugins embedded from ${NEMATX_PLUGINS_SOURCE_DIR}")
elseif(SLIC3R_APP_KEY STREQUAL "NematXSlicer")
    message(STATUS "NematX built-in plugins disabled; standalone packages remain supported")
endif()

get_property(_nematx_declarations GLOBAL PROPERTY SLIC3R_NEMATX_BUILTIN_DECLARATIONS)
get_property(_nematx_registrations GLOBAL PROPERTY SLIC3R_NEMATX_BUILTIN_REGISTRATIONS)
set(_nematx_registry "${CMAKE_CURRENT_BINARY_DIR}/NematXBuiltinPlugins.cpp")
file(WRITE "${_nematx_registry}.in"
    "// Generated registration adapter; standalone plugin sources remain unchanged.\n#include \"libslic3r/Plugins/NematXBuiltinPlugins.hpp\"\n#include \"libslic3r/Api/host/Orchestrator.hpp\"\n${_nematx_declarations}\nnamespace Slic3r {\nvoid register_nematx_builtin_plugins(orchestrator_handle *orchestrator)\n{\n    Orchestrator &host = *reinterpret_cast<Orchestrator *>(orchestrator);\n    (void)host;\n${_nematx_registrations}}\n}\n")
configure_file("${_nematx_registry}.in" "${_nematx_registry}" COPYONLY)
target_sources(libslic3r PRIVATE "${_nematx_registry}")
