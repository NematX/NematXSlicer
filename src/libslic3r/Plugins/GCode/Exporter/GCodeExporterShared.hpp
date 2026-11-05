///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Exporter_GCodeExporterShared_hpp_
#define slic3r_Plugins_GCode_Exporter_GCodeExporterShared_hpp_

#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFilePublisher.hpp"
#include "libslic3r/Config/ConfigOption.hpp"

/*
Shared input for built-in file exporters
========================================

The text and binary exporters use the same `output_path` setting and suggestion.
This helper creates the save-path input from each exporter's suffixes, so the
request dialog describes the file format that the selected exporter publishes.

From an exporter's input_definitions(), append the shared destination request:

    append_gcode_output_path_input_definition(format, config, definitions);

No field is appended when output_path already contains a destination. Otherwise
the definition borrows thread-local label/filter/default text until the host
consumes the synchronous request. Do not retain those pointers as a saved form
or rebuild another definition on the same thread before consuming the first.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {

// Save-dialog text and optional explicit wildcard patterns; empty patterns use the format's existing suffixes.
struct GCodeOutputPathDescription
{
    std::string label = "G-code";
    std::string tooltip = "Where the G-code file is written.";
    std::string patterns;
};

// Append a save-path request using the borrowed format and description when config has no destination; the synchronous consumer borrows thread-local text and allocation failures propagate.
inline void append_gcode_output_path_input_definition(const GCodeFileFormat &format,
                                                       const Config &config,
                                                       std::vector<raw_config_option_def> &definitions,
                                                       const GCodeOutputPathDescription &description = {})
{
    if (!gcode_export_output_path(config).empty())
        return;

    static thread_local std::string default_value;
    static thread_local std::string file_filter;
    static thread_local std::string tooltip;
    // The host parses this as serialized config text, so escape Windows path backslashes before storing the default.
    default_value = config.has("output_path_suggestion") ?
        Slic3r::escape_string_cstyle(config.get("output_path_suggestion").get_string()) : std::string();
    std::string patterns = description.patterns;
    if (patterns.empty()) {
        patterns = std::string("*") + format.extension;
        for (const std::string &alias : format.aliases)
            patterns += std::string(";*") + alias;
    }
    file_filter = description.label + " (" + patterns + ")|" + patterns;
    tooltip = description.tooltip;

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "output_path";
    definition.type = RAW_CO_STRING;
    definition.container_type = RAW_CONTAINER_TYPE_PLATER;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    definition.printer_technology = RAW_PT_FFF;
    definition.category = RAW_OPTION_CATEGORY_OUTPUT;
    definition.gui_type = RAW_GUI_TYPE_SAVE_PATH;
    definition.label = "Output path";
    definition.tooltip = tooltip.c_str();
    // The definition borrows the three text buffers; thread-local strings keep them alive while the host synchronously consumes this request on this thread.
    definition.sidetext = file_filter.c_str();
    definition.default_serialized_value = default_value.c_str();
    definitions.push_back(definition);
}

}}} // namespace slic3r_api::GCodeGeneration::Exporter

#endif // slic3r_Plugins_GCode_Exporter_GCodeExporterShared_hpp_
