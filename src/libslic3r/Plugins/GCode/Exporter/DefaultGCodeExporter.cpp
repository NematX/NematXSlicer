///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultGCodeExporter.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeTextEnvelope.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
Built-in G-code text exporter
=============================

The provider advertises the text extension and writes the host-supplied
temporary path: thumbnails, generated producer line, notes and extrusion widths
before the reference; the statistics (when the host measured them), the filtered configuration and end thumbnails follow it.
Thumbnails are encoded before the file is opened, so a render failure creates
nothing. It does not invoke scripts or publish the destination; the host handles
publication after close.

Representative direct calls (branches are not a timeline):

    DefaultGCodeExporter::export_impl()
        |-- build_gcode_text_envelope()
        `-- write_gcode_text_file()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {
namespace {

const char *const k_no_dependencies[] = { nullptr };

// Settings of the envelope: the notes and every setting of the default thumbnail requests.
const raw_used_config_key k_used_keys[] = {
    {"notes", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_color_mode", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_end_file", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_format", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_tag_format", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_bed", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_support", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

class DefaultGCodeExporter final : public GCodeExporterPluginBase
{
public:
    // Bind this process-lived exporter to the borrowed host handle supplied at registration.
    explicit DefaultGCodeExporter(orchestrator_handle *orchestrator) : GCodeExporterPluginBase(orchestrator) {}

private:
    // Return the stable built-in exporter identity.
    const char *id_impl() const noexcept override { return "gcode.exporter.simple"; }

    // Describe the text exporter in user-facing lists.
    const char *name_impl() const noexcept override { return "Simple G-code exporter"; }

    // Return the stable user-facing description without reading run state or allocating; the static text remains valid for the process lifetime.
    const char *description_impl() const noexcept override
    { return "Writes a versioned header, notes, and extrusion-width comments around the reference, with statistics (when available) and filtered configuration before any end thumbnail."; }

    // Declare the settings read by the envelope; the host checks that they exist.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_keys), std::end(k_used_keys), keys);
        return int32_t(std::size(k_used_keys));
    }

    // Supply no plugin dependencies for this self-contained exporter.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }

    // Run the built-in provider at the default priority in the exporter group.
    int32_t priority_impl() const noexcept override { return 0; }

    // Accept every firmware configuration and advertise the text-file suffix.
    GCodeExportQuery query_impl(const Config &, std::string_view) const override;

    // Format the host-provided ABI 2.2 metadata with the configuration copy, then write it around the reference; borrowed inputs are retained only for the call and render, I/O, or cancellation failures throw.
    void export_impl(const plugin_run_context &run_context,
                     const Print &print,
                     const Config &config,
                     std::string_view reference_path_utf8,
                     std::string_view temporary_output_path_utf8,
                     GCodeThumbnailRendererView thumbnail_renderer,
                     const raw_gcode_export_metadata *metadata) const override;
};

} // namespace

// Ignore borrowed settings and firmware ID, returning the owned always-compatible `.gcode` result without changing Print state.
GCodeExportQuery DefaultGCodeExporter::query_impl(const Config &, std::string_view) const
{
    GCodeExportQuery query;
    query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
    query.extension = ".gcode";
    return query;
}

// Format thumbnails, notes, widths and ABI 2.2 metadata before opening output, then copy the reference and ordered trailer; borrowed inputs are not retained and cancellation, encoding, or I/O failures throw.
void DefaultGCodeExporter::export_impl(const plugin_run_context &run_context,
                                       const Print &print,
                                       const Config &config,
                                       std::string_view reference_path_utf8,
                                       std::string_view temporary_output_path_utf8,
                                       GCodeThumbnailRendererView thumbnail_renderer,
                                       const raw_gcode_export_metadata *metadata) const
{
    throw_if_cancelled(&run_context);

    const GCodeTextEnvelope envelope = build_gcode_text_envelope(
        OrchestratorView(m_orchestrator), print, config, thumbnail_renderer.handle(),
        default_gcode_thumbnail_requests(config), metadata);
    throw_if_cancelled(&run_context);
    write_gcode_text_file(run_context, reference_path_utf8, temporary_output_path_utf8, envelope);
}

}}} // namespace slic3r_api::GCodeGeneration::Exporter

// Pass the process-lived provider and borrowed host handle to registration; retain no handle here, and propagate any C++ registration exception.
void slic3r_api::GCodeGeneration::Exporter::register_default_gcode_exporter_plugin(
    orchestrator_handle *orchestrator)
{
    static slic3r_api::GCodeGeneration::Exporter::DefaultGCodeExporter plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
