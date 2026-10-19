///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultGCodeExporter.hpp"

#include <array>
#include <fstream>
#include <stdexcept>
#include <string>

#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
Built-in G-code text exporter
=============================

The provider advertises the text extension and copies the generated reference
to the host-supplied temporary path. It does not add metadata, invoke scripts,
or publish the user's destination; the host handles publication after close.

Representative direct calls (branches are not a timeline):

    DefaultGCodeExporter::export_impl()
        |-- input.read()
        |-- output.write()
        |-- output.flush()
        `-- output.close()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {
namespace {

const char *const k_no_dependencies[] = { nullptr };

class DefaultGCodeExporter final : public GCodeExporterPluginBase
{
public:
    // Bind this process-lived exporter to the borrowed host handle supplied at registration.
    explicit DefaultGCodeExporter(orchestrator_handle *orchestrator) : GCodeExporterPluginBase(orchestrator) {}

private:
    // Return the stable built-in exporter identity.
    const char *id_impl() const noexcept override { return "gcode.exporter.simple"; }

    // Describe the byte-preserving text exporter in user-facing lists.
    const char *name_impl() const noexcept override { return "Simple G-code exporter"; }

    // State that this provider copies the generated G-code reference unchanged.
    const char *description_impl() const noexcept override { return "Copies the generated G-code reference unchanged."; }

    // Supply no plugin dependencies for this self-contained exporter.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }

    // Run the built-in provider at the default priority in the exporter group.
    int32_t priority_impl() const noexcept override { return 0; }

    // Accept every firmware configuration and advertise the text-file suffix.
    GCodeExportQuery query_impl(const Config &, std::string_view) const override;

    // Copy the reference in bounded blocks and stop on read, write, flush, close, or cancellation failure.
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

// Read the borrowed UTF-8 reference in bounded blocks and close the borrowed temporary output; retain neither path and throw on open, read, write, flush, close, or cancellation failure.
void DefaultGCodeExporter::export_impl(const plugin_run_context &run_context,
                                       const Print &print,
                                       const Config &config,
                                       std::string_view reference_path_utf8,
                                       std::string_view temporary_output_path_utf8,
                                       GCodeThumbnailRendererView thumbnail_renderer,
                                       const raw_gcode_export_metadata *metadata) const
{
    (void)print;
    (void)config;
    (void)thumbnail_renderer;
    (void)metadata;
    throw_if_cancelled(&run_context);

    const std::string reference_path(reference_path_utf8);
    const std::string output_path(temporary_output_path_utf8);
    boost::nowide::ifstream input(reference_path.c_str(), std::ios::in | std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open G-code reference file '" + reference_path + "'.");
    boost::nowide::ofstream output(output_path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("Cannot open temporary G-code output file '" + output_path + "'.");

    std::array<char, 64 * 1024> buffer;
    while (true) {
        throw_if_cancelled(&run_context);
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        if (count > 0) {
            output.write(buffer.data(), count);
            if (!output)
                throw std::runtime_error("Failed while writing the temporary G-code output file.");
        }
        if (input.bad() || (input.fail() && !input.eof()))
            throw std::runtime_error("Failed while reading the G-code reference file.");
        if (input.eof())
            break;
    }

    output.flush();
    if (!output)
        throw std::runtime_error("Failed while flushing the temporary G-code output file.");
    output.close();
    if (!output)
        throw std::runtime_error("Failed while closing the temporary G-code output file.");
}

}}} // namespace slic3r_api::GCodeGeneration::Exporter

// Pass the process-lived provider and borrowed host handle to registration; retain no handle here, and propagate any C++ registration exception.
void slic3r_api::GCodeGeneration::Exporter::register_default_gcode_exporter_plugin(
    orchestrator_handle *orchestrator)
{
    static slic3r_api::GCodeGeneration::Exporter::DefaultGCodeExporter plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
