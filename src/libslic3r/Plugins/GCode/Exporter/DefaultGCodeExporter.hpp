///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_
#define slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_

#include <array>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
Built-in text G-code exporter
=============================

The registration function installs one process-lived provider that advertises
the `.gcode` suffix. Its output is the text envelope (thumbnails, producer
line, notes), then the G-code reference copied unchanged, then the trailing
thumbnails. The file part of that recipe is write_gcode_text_file() in GCodeTextEnvelope.hpp.

The complete text recipe also includes the host's available statistics and the
filtered configuration after the reference, before any trailing thumbnails.
The registration entry point installs the provider; the shared text-envelope
helper implements this layout rather than duplicating it in each file exporter.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {

/* Main suffix of the built-in exporter, and the usual text aliases offered by its file filter. A destination may use any suffix. */
inline constexpr const char *k_default_gcode_exporter_extension = ".gcode";
inline constexpr std::array<const char *, 2> k_default_gcode_exporter_aliases = {".g", ".gco"};

/* Pass the borrowed host handle to register the process-lived text exporter. */
void register_default_gcode_exporter_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Exporter

#endif // slic3r_Plugins_GCode_Exporter_DefaultGCodeExporter_hpp_
