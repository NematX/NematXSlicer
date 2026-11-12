#/|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

if (NOT DEFINED PLUGIN_RESOURCE_DIRECTORY OR NOT DEFINED PLUGIN_PACKAGE_NAME)
    message(FATAL_ERROR "PLUGIN_RESOURCE_DIRECTORY and PLUGIN_PACKAGE_NAME are required")
endif()

file(GLOB _user_plugin_archives
    "${PLUGIN_RESOURCE_DIRECTORY}/${PLUGIN_PACKAGE_NAME}_*.zip")
if (_user_plugin_archives)
    file(REMOVE ${_user_plugin_archives})
endif()
