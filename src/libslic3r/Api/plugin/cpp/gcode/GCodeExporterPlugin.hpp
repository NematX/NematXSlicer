///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_

#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"

namespace slic3r_api {

class Config;
class Print;

struct GCodeExportQuery
{
    raw_gcode_export_query_status status = RAW_GCODE_EXPORT_QUERY_UNSET;
    std::string extension;
    std::string message;
};

/*
G-code exporter C++ provider skeleton
======================================

GCodeExporterPluginBase adapts the synchronous GCODE_EXPORT payload to two
provider methods. query_impl() checks the borrowed full print configuration
(printer, print and filament settings, including the exporter selector) and the
firmware identity without writing a file. It runs on the calling thread, often
the GUI thread, so it must be fast and free of side effects. export_impl() reads
the generated reference and creates the host's temporary output. Settings are read from the
borrowed configuration copy given to both methods, never from the Print, whose own
configuration may already differ. The paths, Print, configuration, renderer
and opaque metadata are borrowed only for their call; implementations do not
retain them. PluginBase contains exceptions at the C boundary.
*/
class GCodeExporterPluginBase : public PluginBase
{
protected:
    // Construct a provider bound to the borrowed host handle used for registration and run callbacks.
    explicit GCodeExporterPluginBase(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    // Inspect the borrowed full print configuration and firmware ID, returning provider-owned status/text or an incompatible diagnostic.
    virtual GCodeExportQuery query_impl(const Config &config,
                                        std::string_view firmware_plugin_id) const = 0;

    // Read the borrowed Print, configuration copy, reference and views for one run and write only the supplied output path; check cancellation and throw on failure.
    virtual void export_impl(const plugin_run_context &run_context,
                             const Print &print,
                             const Config &config,
                             std::string_view reference_path_utf8,
                             std::string_view temporary_output_path_utf8,
                             GCodeThumbnailRendererView thumbnail_renderer,
                             const raw_gcode_export_metadata *metadata) const = 0;

private:
    // Report GCODE_EXPORT as this provider's service step.
    slicing_step_t step_impl() const noexcept final { return GCODE_EXPORT; }

    // Place the provider ID in the shared printer-profile selector.
    const char *exclusive_group_impl() const noexcept final { return "gcode_exporter_plugin"; }

    // Validate borrowed operation fields, dispatch QUERY/EXPORT, and convert invalid fields or provider failures into PluginBase error handling.
    void run_impl(const plugin_run_context *run_context) const final;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_
