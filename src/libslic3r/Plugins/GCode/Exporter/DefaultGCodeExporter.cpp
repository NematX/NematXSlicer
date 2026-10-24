///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultGCodeExporter.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFilePublisher.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeTextEnvelope.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Config/ConfigOption.hpp"

/*
Built-in G-code text exporter
=============================

The provider advertises the text extension and writes the file named by the
output_path setting of the export: thumbnails, generated producer line, notes and extrusion widths
before the reference; the statistics (when the host measured them), the filtered configuration and end thumbnails follow it.
Thumbnails are encoded before envelope bytes are written. The publisher creates
the sibling first and removes it if rendering or encoding fails; it replaces the destination only when complete
(publish_gcode_file()). The provider then reports the published path and a label to the host. It does not invoke scripts.

When the export has no destination (the output_path setting is empty), the provider asks the requester for one while the export sets up: a save-path
input whose proposal is the output_path_suggestion setting that the host computed. The answer is then the output_path setting that
export_impl() reads. With a destination, nothing is asked.

Representative direct calls (branches are not a timeline):

    DefaultGCodeExporter::export_impl()
        |-- publish_gcode_file()
        `-- result.report()

    publish_gcode_file writer callback [not a direct call of the enclosing function]
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

    // Ask for the destination when the export has none: a save-path input proposed with the host's suggested path.
    void input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const override;

    // Accept every firmware configuration and advertise the text-file suffix.
    GCodeExportQuery query_impl(const Print &, const Config &, std::string_view) const override;

    // Format the host metadata with the configuration copy, write it around the reference into a sibling of the destination, replace the destination and report the published file; borrowed inputs are retained only for the call and render, I/O, or cancellation failures throw. A failure or cancellation before the replacement leaves the destination as it was; a failed replacement only tries to restore it, and a cancellation after the replacement leaves the new file published (and reported).
    void export_impl(const plugin_run_context &run_context,
                     const Print &print,
                     const Config &config,
                     std::string_view reference_path_utf8,
                     GCodeThumbnailRendererView thumbnail_renderer,
                     const raw_gcode_export_metadata *metadata,
                     const GCodeExportResultReporter &result) const override;
};

} // namespace

// When output_path is empty, append one save-path definition whose default is the output_path_suggestion setting and whose file filter
// lists the text suffix and its aliases. The strings are thread-local so they stay valid while the host reads the definition on this thread.
void DefaultGCodeExporter::input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const
{
    if (!gcode_export_output_path(config).empty())
        return;

    static thread_local std::string default_value;
    static thread_local std::string file_filter;
    // The default is a serialized value: backslashes of a Windows path are escaped.
    default_value = config.has("output_path_suggestion") ?
        Slic3r::escape_string_cstyle(config.get("output_path_suggestion").get_string()) : std::string();
    std::string patterns = std::string("*") + k_default_gcode_exporter_extension;
    for (const char *alias : k_default_gcode_exporter_aliases)
        patterns += std::string(";*") + alias;
    file_filter = "G-code (" + patterns + ")|" + patterns;

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "output_path";
    definition.type = RAW_CO_STRING;
    definition.container_type = RAW_CONTAINER_TYPE_PLATER;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    definition.printer_technology = RAW_PT_FFF;
    definition.category = RAW_OPTION_CATEGORY_OUTPUT;
    definition.gui_type = RAW_GUI_TYPE_SAVE_PATH;
    definition.label = "Output path";
    definition.tooltip = "Where the G-code file is written.";
    definition.sidetext = file_filter.c_str();
    definition.default_serialized_value = default_value.c_str();
    definitions.push_back(definition);
}
// Ignore the borrowed Print, settings and firmware ID, returning the owned always-compatible `.gcode` result without changing Print state or failing.
GCodeExportQuery DefaultGCodeExporter::query_impl(const Print &, const Config &, std::string_view) const
{
    GCodeExportQuery query;
    query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
    query.extension = k_default_gcode_exporter_extension;
    return query;
}

// Check the destination first (publish_gcode_file), then format thumbnails, notes, widths and metadata and write the envelope around the reference into the helper-owned stream. The helper verifies and publishes the sibling; a refusal renders no thumbnail, borrowed inputs are not retained, and cancellation, encoding or I/O failures throw.
void DefaultGCodeExporter::export_impl(const plugin_run_context &run_context,
                                       const Print &print,
                                       const Config &config,
                                       std::string_view reference_path_utf8,
                                       GCodeThumbnailRendererView thumbnail_renderer,
                                       const raw_gcode_export_metadata *metadata,
                                       const GCodeExportResultReporter &result) const
{
    throw_if_cancelled(&run_context);

    GCodeFileFormat format;
    format.exporter_id = id_impl();
    format.extension = k_default_gcode_exporter_extension;
    format.aliases.assign(k_default_gcode_exporter_aliases.begin(), k_default_gcode_exporter_aliases.end());
    const std::string published = publish_gcode_file(
        run_context, format, gcode_export_output_path(config), reference_path_utf8,
        [&](std::ostream &output) {
            const GCodeTextEnvelope envelope = build_gcode_text_envelope(
                OrchestratorView(m_orchestrator), print, config, thumbnail_renderer.handle(),
                default_gcode_thumbnail_requests(config), metadata);
            throw_if_cancelled(&run_context);
            write_gcode_text_file(run_context, reference_path_utf8, output, envelope);
            return true;
        });
    result.report(published, "G-code file");
}

}}} // namespace slic3r_api::GCodeGeneration::Exporter

// Pass the process-lived provider and borrowed host handle to registration; retain no handle here, and propagate any C++ registration exception.
void slic3r_api::GCodeGeneration::Exporter::register_default_gcode_exporter_plugin(
    orchestrator_handle *orchestrator)
{
    static slic3r_api::GCodeGeneration::Exporter::DefaultGCodeExporter plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
