///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_
#define slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
Built-in text G-code exporter registration
==========================================

The registration function installs one process-lived provider that advertises
the `.gcode` suffix and copies the generated G-code reference into the host's
temporary output file.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {

/* Pass the borrowed host handle to register the process-lived text exporter. */
void register_default_gcode_exporter_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Exporter

#endif // slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_
