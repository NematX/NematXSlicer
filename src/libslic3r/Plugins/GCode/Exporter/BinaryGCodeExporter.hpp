///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Exporter_BinaryGCodeExporter_hpp_
#define slic3r_Plugins_GCode_Exporter_BinaryGCodeExporter_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
Built-in binary G-code exporter
===============================

The registered provider advertises `.bgcode` and is compatible only when the
printer profile declares binary G-code support. It writes the file and printer
metadata blocks, any encoded raw-image thumbnails, then print and slicer
metadata, notes and G-code body into a private file before publishing those bytes.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {

inline constexpr const char *k_binary_gcode_exporter_extension = ".bgcode";

// Register the process-lived binary G-code exporter with the borrowed orchestrator handle.
void register_binary_gcode_exporter_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Exporter

#endif // slic3r_Plugins_GCode_Exporter_BinaryGCodeExporter_hpp_
