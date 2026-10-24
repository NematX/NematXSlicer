///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepExport.hpp"

#include <optional>
#include <string>
#include <utility>

#include <boost/filesystem.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils/ASCIIFolding.hpp"

/*
STEP_EXPORT runner implementation
=================================

The runner is the only reader of the export request. It moves the request out of
the Print first, so that every attempt consumes it whatever its outcome, checks
the reference, queries the exporter with the request settings and delegates
the run to Orchestrator::export_print. For file exporters it sets the request's
output_path_suggestion from the optional directory and filename base; a directory
that is not present is ignored, leaving the model-derived directory in use. When
requested, only the suggested filename is folded to ASCII. The exporter reads its
destination from request configuration and publishes its own output. The reference
path comes from the Print (the source STEP_GCODE published), never from the request.
The configuration and thumbnail renderer come from the request alone. What the
exporter reports is kept in the Print as soon as it is reported.

Representative direct calls (branches are not a timeline):

    run_step()
        |-- print.take_export_request()
        |-- print.ready_gcode_reference_path()
        |-- orchestrator.select_gcode_exporter()
        `-- orchestrator.export_print()
*/

namespace Slic3r::Steps::StepExport {

// Copy export-stage selection rules without running an export or changing print data.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    traits.invalidates_step = STEP_NONE;
    traits.show_with_one_provider = true;
    traits.read_with_one_provider = true;
    traits.strict_selection = true;
    traits.value_when_absent = "gcode.exporter.simple";
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_EXPORT, traits);
    assert(registered == 1);
    (void)registered;
    Orchestrator::ExclusiveGroupDefinition definition;
    definition.traits = std::move(traits);
    definition.traits.exclusive_group = "gcode_exporter_plugin";
    definition.traits.option_key = "gcode_exporter_plugin";
    definition.label = "G-code exporter";
    definition.tooltip = "Selects the plugin that exports G-code.";
    definition.translation_domain = "Slic3r";
    const int32_t group_registered = orchestrator.register_builtin_exclusive_group(STEP_EXPORT, std::move(definition));
    assert(group_registered == 1);
    (void)group_registered;
}

// Consume one request, validate its ready reference, form a statistics-based suggestion from its path fields, optionally fold only its filename, and export with the request-owned renderer; store reported results and propagate request, reference, formatting, exporter, and cancellation failures.
void run_step(Orchestrator &orchestrator, Print &print)
{
    std::optional<Print::ExportRequest> request = print.take_export_request();
    if (!request.has_value())
        throw RuntimeError("No export was requested: deposit an export request before running the export.");

    // The reference must be the one STEP_GCODE finished: an invalidation between the request and now makes the
    // export fail instead of replaying the writer.
    const std::string reference = print.ready_gcode_reference_path();
    boost::system::error_code filesystem_error;
    if (reference.empty() ||
        !boost::filesystem::is_regular_file(boost::filesystem::path(reference), filesystem_error) || filesystem_error)
        throw RuntimeError("The G-code reference is not ready: slice again before exporting.");

    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(print, request->config);

    // STEP_GCODE is done, so output_filepath formats the final statistics. Use the proposed directory only while it exists;
    // otherwise output_filepath falls back to its current model-derived directory.
    if (!selection.extension.empty()) {
        std::string suggestion_directory;
        boost::system::error_code suggestion_error;
        if (!request->output_path_suggestion_directory.empty() &&
            boost::filesystem::is_directory(boost::filesystem::path(request->output_path_suggestion_directory), suggestion_error) &&
            !suggestion_error)
            suggestion_directory = request->output_path_suggestion_directory;
        std::string suggestion = print.output_filepath(suggestion_directory,
                                                       request->output_path_suggestion_filename_base,
                                                       selection.extension);
        if (request->output_path_suggestion_ascii_fallback) {
            const boost::filesystem::path suggestion_path(suggestion);
            suggestion = (suggestion_path.parent_path() /
                          Slic3r::fold_utf8_to_ascii(suggestion_path.filename().string())).make_preferred().string();
        }
        request->config.set_key_value("output_path_suggestion", new ConfigOptionString(suggestion));
    }
    orchestrator.export_print(print, request->config, reference, selection, request->thumbnails, request->inputs);
}

} // namespace Slic3r::Steps::StepExport
