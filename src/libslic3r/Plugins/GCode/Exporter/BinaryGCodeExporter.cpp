///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Provide the built-in binary G-code file format to the export service.
QUERY checks printer capability. EXPORT gathers configuration, measured metadata
and optional raw images, then uses libbgcode to construct the binary blocks.
Because that library writes through a seekable FILE, it uses a private file
before copying the completed bytes into the publisher's verified stream.

Printer metadata can be consumed outside the slicer. Its presence here does not
establish compatibility with every printer or print-host implementation that
reads similarly named historical fields.

Binary G-code exporter
======================

The provider checks the printer's binary-G-code capability during QUERY. During
EXPORT it renders raw-image thumbnails and builds the metadata blocks inside the
publisher's writer, then asks libbgcode to write a private file and copies that
file into the publisher's verified stream.

The estimated normal-mode time and the time of the first layer are written as
get_time_dhms() text, like the historical binary G-code, and only when the host
measured them (the statistics list is not empty); max_layer_z is the highest layer
Z of the printing plan with two decimals. No silent-mode time exists, so no
silent-mode line is written.

The historical printer and print metadata keys do not establish compatibility
with every external consumer, including PrusaLink and printer firmware.

Binary G-code output
====================

libbgcode's writer accepts a FILE reference, while the file publisher provides
an ostream that fingerprints the completed output. This exporter writes a
unique sibling of the read-only G-code reference, closes and rereads it, and
copies it to the publisher only after all binary blocks have finalized. The
private-file guard closes and attempts removal on success, failure, or
cancellation.

Representative direct calls (branches are not a timeline):

    BinaryGCodeExporter::export_impl()
        |-- publish_gcode_file()
        `-- result.report()

    publish_gcode_file writer callback [not a direct call of the enclosing function]
        `-- write_binary_gcode_file()
*/

#include "BinaryGCodeExporter.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <iomanip>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/fstream.hpp>

#include "GCodeExporterShared.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExportMetadataViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFilePublisher.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodePrivateFile.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeTextEnvelope.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Utils.hpp"



namespace slic3r_api { namespace GCodeGeneration { namespace Exporter {
namespace {

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_keys[] = {
    {"notes", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_color_mode", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails", RAW_CO_VECTOR_POINT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_format", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_bed", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_support", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

class BinaryGCodeExporter final : public GCodeExporterPluginBase
{
public:
    // Keep the borrowed orchestrator in the base for process-wide registration; the provider owns no host state and base construction failures propagate.
    explicit BinaryGCodeExporter(orchestrator_handle *orchestrator) : GCodeExporterPluginBase(orchestrator) {}

private:
    // Return the static provider ID as a borrowed string without changing state or failing.
    const char *id_impl() const noexcept override { return "gcode.exporter.bgcode"; }
    // Return the static display name borrowed by exporter lists without changing state or failing.
    const char *name_impl() const noexcept override { return "Binary G-code"; }
    // Return the static description borrowed by the host without changing state or failing.
    const char *description_impl() const noexcept override
    { return "Writes binary G-code metadata and raw-image thumbnails around the generated G-code body."; }
    // Copy static key descriptors when storage is supplied and always return their count; no provider state changes.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_keys), std::end(k_used_keys), keys);
        return int32_t(std::size(k_used_keys));
    }
    // Return the static null-terminated empty dependency list without transferring storage or changing state.
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Return priority zero without allocating or changing state.
    int32_t priority_impl() const noexcept override { return 0; }
    // Append the shared save-path request using the `.bgcode` suffix.
    void input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const override;
    // Check printer support and requested thumbnail image capability without rendering, and advertise `.bgcode` when compatible.
    GCodeExportQuery query_impl(const Print &print, const Config &config, std::string_view firmware_plugin_id) const override;
    // Publish a binary file from borrowed export inputs; validation, rendering, binarizer and I/O failures throw before a result is reported.
    void export_impl(const plugin_run_context &run_context,
                     const Print &print,
                     const Config &config,
                     std::string_view reference_path_utf8,
                     GCodeThumbnailRendererView thumbnail_renderer,
                     const raw_gcode_export_metadata *metadata,
                     const GCodeExportResultReporter &result) const override;
};

// Populate metadata and thumbnail blocks, append notes and reference bytes without line-ending conversion, then copy the closed private file to the publisher stream.
void write_binary_gcode_file(const plugin_run_context &run_context,
                             const OrchestratorView &orchestrator,
                             const Print &print,
                             const Config &config,
                             std::string_view reference_path_utf8,
                             GCodeThumbnailRendererView thumbnail_renderer,
                             const raw_gcode_export_metadata *metadata,
                             std::ostream &output);

// Copy only the named borrowed metadata entries, in requested order, into owned binarizer storage.
void append_metadata_keys(const GCodeExportMetadataListView &source,
                          std::initializer_list<std::string_view> keys,
                          std::vector<std::pair<std::string, std::string>> &destination);

// Convert owned encoded images to libbgcode thumbnail blocks with dimensions and codec identifiers.
void append_binary_thumbnails(bgcode::binarize::Binarizer &binarizer,
                              const std::vector<GCodeThumbnailEncodedImage> &images);

// Populate the four metadata blocks from the version API, filtered configuration, text statistics and numeric statistics (times only when measured, plan Z always).
void populate_binary_metadata(bgcode::binarize::Binarizer &binarizer,
                              const raw_gcode_export_metadata *metadata);

// Format a number with two decimals and a decimal point independent of the process locale.
std::string format_two_decimals(double value);

// Throw a translated runtime error unless one libbgcode operation returned Success.
void check_binarizer_result(bgcode::core::EResult result, const char *operation);

// Append notes and the reference's original line endings to libbgcode, preserving whether the final line had a newline.
void append_reference_gcode(const plugin_run_context &run_context,
                            bgcode::binarize::Binarizer &binarizer,
                            std::string_view reference_path_utf8,
                            const Config &config);

// Use borrowed export inputs to build blocks in a unique-named file beside the reference, append notes and reference bytes, finalize, then copy to the publisher stream; the guard owns cleanup, and cancellation, binarizer, reference-I/O or output errors abort the writer.
void write_binary_gcode_file(const plugin_run_context &run_context,
                             const OrchestratorView &orchestrator,
                             const Print &print,
                             const Config &config,
                             std::string_view reference_path_utf8,
                             GCodeThumbnailRendererView thumbnail_renderer,
                             const raw_gcode_export_metadata *metadata,
                             std::ostream &output)
{
    GCodePrivateFile private_file(reference_path_utf8);
    bgcode::binarize::Binarizer binarizer;
    binarizer.set_enabled(true);

    const std::vector<GCodeThumbnailRequest> requests = default_gcode_thumbnail_requests(config);
    const std::vector<GCodeThumbnailEncodedImage> images = requests.empty() ?
        std::vector<GCodeThumbnailEncodedImage>() :
        encode_gcode_thumbnail_images(orchestrator, print, config, thumbnail_renderer.handle(), requests);
    populate_binary_metadata(binarizer, metadata);
    append_binary_thumbnails(binarizer, images);

    bgcode::binarize::BinarizerConfig binarizer_config = Slic3r::GCodeProcessor::get_binarizer_config();
    check_binarizer_result(binarizer.initialize(*private_file.handle(), binarizer_config),
                           "initialization");
    append_reference_gcode(run_context, binarizer, reference_path_utf8, config);
    check_binarizer_result(binarizer.finalize(), "finalization");

    private_file.close();
    private_file.open_read();
    copy_private_gcode_file(run_context, private_file.handle(), output);
    private_file.close();
}

// Append requested matching entries in order from borrowed metadata into owned strings; missing keys are skipped and allocation failures propagate.
void append_metadata_keys(const GCodeExportMetadataListView &source,
                          std::initializer_list<std::string_view> keys,
                          std::vector<std::pair<std::string, std::string>> &destination)
{
    for (const std::string_view key : keys) {
        for (std::size_t index = 0; index < source.size(); ++index) {
            const GCodeExportMetadataEntryView entry = source.at(index);
            if (entry.key == key) {
                destination.emplace_back(std::string(entry.key), std::string(entry.value));
                break;
            }
        }
    }
}

// Format a number with two decimals through the classic locale (the legacy "%.2lf"); allocation failures propagate.
std::string format_two_decimals(double value)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(2) << value;
    return output.str();
}

// Populate libbgcode's owned metadata from host version and borrowed export views; invalid version data or config/allocation failures propagate, and no borrowed view is retained.
void populate_binary_metadata(bgcode::binarize::Binarizer &binarizer,
                              const raw_gcode_export_metadata *metadata)
{
    raw_slic3r_version_info version_info = {};
    version_info.struct_size = uint32_t(sizeof(version_info));
    if (orchestrator_get_version_info(&version_info) == 0 || version_info.application_name == nullptr ||
        version_info.version == nullptr)
        throw std::runtime_error("The host could not provide binary G-code producer information.");

    bgcode::binarize::BinaryData &binary_data = binarizer.get_binary_data();
    binary_data.file_metadata.raw_data.emplace_back(
        "Producer", std::string(version_info.application_name) + " " + std::string(version_info.version));

    const GCodeExportMetadataView metadata_view(metadata);
    const GCodeExportMetadataListView configuration = metadata_view.configuration();
    for (std::size_t index = 0; index < configuration.size(); ++index) {
        const GCodeExportMetadataEntryView entry = configuration.at(index);
        binary_data.slicer_metadata.raw_data.emplace_back(std::string(entry.key), std::string(entry.value));
    }

    append_metadata_keys(configuration,
        {"printer_model", "filament_type", "nozzle_diameter", "bed_temperature", "brim_width",
         "fill_density", "layer_height", "temperature", "ironing", "support_material", "extruder_colour"},
        binary_data.printer_metadata.raw_data);

    const GCodeExportMetadataListView statistics = metadata_view.statistics();
    append_metadata_keys(statistics,
        {"filament used [mm]", "filament used [g]", "filament cost", "filament used [cm3]",
         "total filament used for wipe tower [g]"},
        binary_data.printer_metadata.raw_data);
    // The times are written only when the host measured them; an unmeasured time would be a false zero. No silent-mode
    // time exists, so no silent-mode line is ever written.
    const GCodeExportNumericStatisticsView numeric = metadata_view.numeric_statistics();
    const bool measured = metadata_view.has_measured_statistics();
    const std::string normal_time = Slic3r::get_time_dhms(float(numeric.estimated_time_seconds));
    if (measured)
        binary_data.printer_metadata.raw_data.emplace_back("estimated printing time (normal mode)", normal_time);
    binary_data.printer_metadata.raw_data.emplace_back("max_layer_z", format_two_decimals(numeric.max_print_z));

    append_metadata_keys(statistics,
        {"total toolchanges", "filament used [mm]", "filament used [cm3]", "filament used [g]",
         "filament cost", "total filament used [g]", "total filament cost",
         "total filament used for wipe tower [g]"},
        binary_data.print_metadata.raw_data);
    if (measured) {
        binary_data.print_metadata.raw_data.emplace_back("estimated printing time (normal mode)", normal_time);
        binary_data.print_metadata.raw_data.emplace_back(
            "estimated first layer printing time (normal mode)",
            Slic3r::get_time_dhms(float(numeric.first_layer_time_seconds)));
    }
    // libbgcode refuses an empty print metadata block, and an export without measured statistics has no other entry:
    // the plan's highest Z then stands alone.
    if (binary_data.print_metadata.raw_data.empty())
        binary_data.print_metadata.raw_data.emplace_back("max_layer_z", format_two_decimals(numeric.max_print_z));
}

// Append codec, dimensions and copied bytes for each encoded image; source buffers remain borrowed, while unsupported formats and allocation failures throw.
void append_binary_thumbnails(bgcode::binarize::Binarizer &binarizer,
                              const std::vector<GCodeThumbnailEncodedImage> &images)
{
    std::vector<bgcode::binarize::ThumbnailBlock> &thumbnails = binarizer.get_binary_data().thumbnails;
    for (const GCodeThumbnailEncodedImage &image : images) {
        bgcode::binarize::ThumbnailBlock &thumbnail = thumbnails.emplace_back();
        thumbnail.params.width = uint16_t(image.width);
        thumbnail.params.height = uint16_t(image.height);
        switch (image.format) {
        case RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_PNG:
            thumbnail.params.format = uint16_t(bgcode::core::EThumbnailFormat::PNG);
            break;
        case RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_JPG:
            thumbnail.params.format = uint16_t(bgcode::core::EThumbnailFormat::JPG);
            break;
        case RAW_GCODE_THUMBNAIL_ENCODED_FORMAT_QOI:
            thumbnail.params.format = uint16_t(bgcode::core::EThumbnailFormat::QOI);
            break;
        default:
            throw std::runtime_error("The thumbnail encoder returned a format unsupported by binary G-code.");
        }
        thumbnail.data.resize(image.bytes.size());
        std::memcpy(thumbnail.data.data(), image.bytes.data(), image.bytes.size());
    }
}

// Leave successful operations unchanged and translate every other libbgcode result into an operation-specific runtime_error.
void check_binarizer_result(bgcode::core::EResult result, const char *operation)
{
    if (result != bgcode::core::EResult::Success)
        throw std::runtime_error(std::string("Binary G-code ") + operation + " failed: " +
                                 std::string(bgcode::core::translate_result(result)));
}

// Read the borrowed UTF-8 reference in binary mode, preserve its line endings, prefix notes to the first chunk and append chunks; file, cancellation or binarizer failures throw, and no reference bytes are retained.
void append_reference_gcode(const plugin_run_context &run_context,
                            bgcode::binarize::Binarizer &binarizer,
                            std::string_view reference_path_utf8,
                            const Config &config)
{
    const std::string notes = format_gcode_notes(config);
    const std::string reference_path(reference_path_utf8);
    boost::nowide::ifstream input(reference_path.c_str(), std::ios::in | std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open G-code reference file '" + reference_path + "'.");

    bool notes_appended = false;
    std::string line;
    while (std::getline(input, line)) {
        throw_if_cancelled(&run_context);
        if (!input.eof())
            line.push_back('\n');
        if (!notes_appended && !notes.empty()) {
            line.insert(0, notes);
            notes_appended = true;
        }
        check_binarizer_result(binarizer.append_gcode(line), "G-code append");
    }
    if (input.bad() || (input.fail() && !input.eof()))
        throw std::runtime_error("Failed while reading the G-code reference file.");
    if (!notes_appended && !notes.empty()) {
        throw_if_cancelled(&run_context);
        check_binarizer_result(binarizer.append_gcode(notes), "notes append");
    }
}

// Read the borrowed config and append the shared save-path request only when output_path is empty; thread-local strings back serialized pointers until host consumption, and config/allocation failures propagate without changing Print state.
void BinaryGCodeExporter::input_definitions(const Config &config,
                                           std::vector<raw_config_option_def> &definitions) const
{
    GCodeFileFormat format;
    format.exporter_id = id_impl();
    format.extension = k_binary_gcode_exporter_extension;
    append_gcode_output_path_input_definition(format, config, definitions);
}

// Read settings from config and acquire the selected thumbnail session through the borrowed Print only when images are requested; return owned binary-file action information suitable for removable media on success, or an incompatibility diagnostic without rendering or writing, while acquisition failures propagate.
GCodeExportQuery BinaryGCodeExporter::query_impl(const Print &print, const Config &config, std::string_view) const
{
    GCodeExportQuery query;
    if (!config.bool_or_default("binary_gcode", false)) {
        query.status = RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE;
        query.message = "The printer does not declare binary G-code support (binary_gcode).";
        return query;
    }
    if (!default_gcode_thumbnail_requests(config).empty()) {
        const std::unique_ptr<GCodeThumbnailEncoder> encoder = acquire_gcode_thumbnail_image_encoder(
            OrchestratorView(m_orchestrator), print, config, nullptr, std::string(), query.message);
        if (!query.message.empty()) {
            query.status = RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE;
            return query;
        }
    }
    query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
    query.extension = k_binary_gcode_exporter_extension;
    query.action_label = L("Export binary G-code");
    query.supports_removable_media = true;
    return query;
}

// Let the publisher validate the destination before its writer creates a private reference-side file; render and binarize in that writer, then report the committed path, while any validation, cancellation or I/O error throws before a report.
void BinaryGCodeExporter::export_impl(const plugin_run_context &run_context,
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
    format.extension = k_binary_gcode_exporter_extension;
    const std::string published = publish_gcode_file(
        run_context, format, gcode_export_output_path(config), reference_path_utf8,
        [&](std::ostream &output) {
            write_binary_gcode_file(run_context, OrchestratorView(m_orchestrator), print, config,
                                    reference_path_utf8, thumbnail_renderer, metadata, output);
            return true;
        });
    result.report(published, "Binary G-code file");
}

} // namespace
}}} // namespace slic3r_api::GCodeGeneration::Exporter

// Register the statically owned provider through the borrowed host handle; registration failures propagate to plugin initialization.
void slic3r_api::GCodeGeneration::Exporter::register_binary_gcode_exporter_plugin(
    orchestrator_handle *orchestrator)
{
    static slic3r_api::GCodeGeneration::Exporter::BinaryGCodeExporter plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
