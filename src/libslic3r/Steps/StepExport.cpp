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

/*
STEP_EXPORT runner implementation
=================================

The runner is the only reader of the export request. It moves the request out of
the Print first, so that every attempt consumes it whatever its outcome, then
checks the reference and delegates the publication to Orchestrator::export_print.
Two inputs have different owners. The reference path comes from the Print (the
source STEP_GCODE published), never from the request: the request cannot
redirect the export to another file. The destination (output_path), the
configuration and the thumbnail renderer come from the request alone.

Representative direct calls (branches are not a timeline):

    run_step()
        |-- print.take_export_request()
        |-- print.ready_gcode_reference_path()
        |-- orchestrator.select_gcode_exporter()
        |-- orchestrator.export_print()
        `-- print.set_export_result_path()
*/

namespace Slic3r::Steps::StepExport {

// Copy export-stage selection rules without running an export or changing print data.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_OUTPUT;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    traits.invalidates_step = STEP_EXPORT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_EXPORT, std::move(traits));
    assert(registered == 1);
    (void)registered;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    std::optional<Print::ExportRequest> request = print.take_export_request();
    if (!request.has_value())
        throw RuntimeError("No export was requested: deposit an export request before running the export.");

    const std::string destination = request->output_path();
    if (destination.empty())
        throw RuntimeError("The export request has no destination (output_path).");

    // The reference must be the one STEP_GCODE finished: an invalidation between the request and now makes the
    // export fail instead of replaying the writer.
    const std::string reference = print.ready_gcode_reference_path();
    boost::system::error_code filesystem_error;
    if (reference.empty() ||
        !boost::filesystem::is_regular_file(boost::filesystem::path(reference), filesystem_error) || filesystem_error)
        throw RuntimeError("The G-code reference is not ready: slice again before exporting.");

    const Orchestrator::GCodeExporterSelection selection = orchestrator.select_gcode_exporter(request->config);
    const std::string published = orchestrator.export_print(
        print, request->config, reference, destination, selection, request->thumbnails);
    // Record the file before the step returns: the caller must know it even if a later check cancels the step.
    print.set_export_result_path(published);
}

} // namespace Slic3r::Steps::StepExport
