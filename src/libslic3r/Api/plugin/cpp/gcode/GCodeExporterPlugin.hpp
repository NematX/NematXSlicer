///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_

// The provider adapter lends Print/configuration views for synchronous QUERY and
// EXPORT calls. Query values own their text and default to removable-media suitability.

#include <string>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_export.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"

namespace slic3r_api {

class Config;
class Print;

struct GCodeExportQuery
{
    raw_gcode_export_query_status status = RAW_GCODE_EXPORT_QUERY_UNSET;
    // Empty for an exporter that writes no local file.
    std::string extension;
    std::string message;
    // Optional untranslated UTF-8 button text without ellipsis, in this provider's translation domain.
    std::string action_label;
    bool supports_removable_media = true;
};

// Return the destination that the requester set for this export (the `output_path` setting of the export
// configuration), or an empty string when the request has none.
std::string gcode_export_output_path(const Config &config);

// Reports the outcome of one export to the host. The host accepts one report per export.
class GCodeExportResultReporter
{
public:
    // Borrow the EXPORT context; the reporter is valid for the duration of the run.
    explicit GCodeExportResultReporter(const run_ctx_gcode_export &context) : m_context(context) {}

    // Tell the host which local file the export wrote (empty when it wrote none) and a label to display (may be
    // empty). The host copies both strings during the call. Throws when the host refuses the report, which happens
    // for a second call.
    void report(std::string_view local_path_utf8, std::string_view label) const;

private:
    const run_ctx_gcode_export &m_context;
};

/*
G-code exporter C++ provider skeleton
======================================

GCodeExporterPluginBase adapts the synchronous STEP_EXPORT payload to two
provider methods. Both read their settings from the borrowed config argument,
never from the Print, whose own configuration may differ. query_impl() checks
compatibility using config and the firmware identity. It may inspect only Print
data stable during computation: applied configuration and selected services,
never geometry or layers. Compatibility must not depend on request-only settings
such as output_path and output_path_suggestion. QUERY may acquire a service to
inspect its capabilities without rendering or writing. It runs on the calling
thread, often the GUI thread, so it must be fast and free of side effects.

export_impl() reads the generated reference and publishes the output itself.
It reads its destination from config's output_path (gcode_export_output_path())
and reports what it wrote through GCodeExportResultReporter. A file exporter
calls publish_gcode_file() (GCodeFilePublisher.hpp), which writes next to the
destination and replaces it only when the write succeeds. input_definitions()
lets the exporter ask the requester for values before exporting; the answers
are then in config. The reference path, Print, configuration, renderer and
metadata are borrowed only for their call; implementations do not retain them.
The metadata view checks its size prefix and exposes read-only entries.
PluginBase contains exceptions at the C boundary.
*/
class GCodeExporterPluginBase : public PluginBase
{
protected:
    // Construct a provider bound to the borrowed host handle used for registration and run callbacks.
    explicit GCodeExporterPluginBase(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    // Inspect stable borrowed Print data, config and firmware ID, returning owned compatibility and action text with removable suitability without rendering or writing.
    virtual GCodeExportQuery query_impl(const Print &print, const Config &config,
                                        std::string_view firmware_plugin_id) const = 0;

    // Read borrowed run data and metadata, publish the output at the destination named by the configuration, report
    // what was written, and throw on invalid metadata, cancellation, or I/O failure.
    virtual void export_impl(const plugin_run_context &run_context,
                             const Print &print,
                             const Config &config,
                             std::string_view reference_path_utf8,
                             GCodeThumbnailRendererView thumbnail_renderer,
                             const raw_gcode_export_metadata *metadata,
                             const GCodeExportResultReporter &result) const = 0;

    // Declare the settings that this export asks the requester for, by appending their definitions (default: none). It runs at the
    // start of an EXPORT, before run, with the borrowed configuration copy, so a key that the configuration already holds is
    // proposed with its value when that value is not empty, else with the default value of the definition. The pointed strings must stay valid until the call returns. When the requester answers, the answers
    // are in the configuration that export_impl() receives; when it does not (cancellation, refusal), the export stops here and
    // export_impl() is not called.
    virtual void input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const;

private:
    // Ask the requester for the inputs declared by input_definitions() and throw when the export has to stop.
    void setup_run_impl(const plugin_run_context *run_context) const final;

    // Report STEP_EXPORT as this provider's step.
    slicing_step_t step_impl() const noexcept final { return STEP_EXPORT; }

    // Place the provider ID in the shared printer-profile selector.
    const char *exclusive_group_impl() const noexcept final { return "gcode_exporter_plugin"; }

    // Validate borrowed operation fields, dispatch QUERY/EXPORT, and convert invalid fields or provider failures into PluginBase error handling.
    void run_impl(const plugin_run_context *run_context) const final;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeExporterPlugin_hpp_
