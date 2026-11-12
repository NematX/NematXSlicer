#/|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

add_library(nozzle_cleaning_plugin SHARED
    NozzleCleaning.cpp
    NozzleCleaning.hpp
)

target_compile_definitions(nozzle_cleaning_plugin PRIVATE NOZZLE_CLEANING_PLUGIN_DLL SLIC3R_PLUGIN_EXPORTS)
target_include_directories(nozzle_cleaning_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
if (WIN32)
    target_link_libraries(nozzle_cleaning_plugin PRIVATE Slic3r)
else()
    target_link_libraries(nozzle_cleaning_plugin PRIVATE libslic3r)
endif()

set_target_properties(nozzle_cleaning_plugin PROPERTIES
    FOLDER "plugins"
)

slic3r_package_plugin(
    nozzle_cleaning_plugin
    nozzle_cleaning
    USER_PACKAGE
    VERSION 2.7.63.0
    CHANGELOG changelog.json
    NAME "Nozzle cleaning"
    FULL_NAME "Nozzle cleaning"
    DESCRIPTION "Insert nozzle-cleaning scripts at configured print transitions."
)
