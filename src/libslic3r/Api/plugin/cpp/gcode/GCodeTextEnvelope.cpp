///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeTextEnvelope.hpp"

#include <array>
#include <fstream>
#include <stdexcept>

#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/libslic3r.h"

/*
Build and write the descriptive text surrounding a G-code reference.
Preparation gathers the host metadata and encoded thumbnails into an envelope;
writing emits its prefix, copies the reference without changing it, then emits
the suffix. File destination checks and replacement belong to the publisher,
not to the envelope formatter. The layout and sources of each block are listed
in GCodeTextEnvelope.hpp.

Text envelope of an exported G-code file
========================================

See GCodeTextEnvelope.hpp for the layout. Thumbnail blocks are indexed by
placement; width comments precede the reference, the statistics (when the host has
any) and then the filtered configuration follow it, and any end thumbnail comes last.

Representative direct calls (branches are not a timeline):

    build_gcode_text_envelope()
        |-- encode_gcode_thumbnails()
        |-- format_gcode_generated_by()
        |-- format_gcode_notes()
        |-- format_gcode_extrusion_widths()
        |-- format_gcode_statistics()
        `-- format_gcode_export_configuration()

    write_gcode_text_file()
        |-- output.write()
        |-- input.read()
        |-- output.flush()
        `-- output.close()
*/

namespace slic3r_api {
namespace {

// Format borrowed width entries as comment lines with blank region separators and return owned text.
std::string format_gcode_extrusion_widths(const GCodeExportMetadataView &metadata);

// Format borrowed statistics entries as "; label = value" comment lines and return owned text, empty without statistics.
std::string format_gcode_statistics(const GCodeExportMetadataView &metadata);

// Format borrowed configuration entries between application-named delimiters and return owned text.
std::string format_gcode_export_configuration(const GCodeExportMetadataView &metadata);

// Format borrowed width entries as comment lines and render an empty pair as a blank region separator; return owned text and propagate invalid-entry or allocation failures.
std::string format_gcode_extrusion_widths(const GCodeExportMetadataView &metadata)
{
    const GCodeExportMetadataListView entries = metadata.extrusion_widths();
    std::string output;
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const GCodeExportMetadataEntryView entry = entries.at(index);
        if (entry.key.empty() && entry.value.empty()) {
            output += "\n";
        } else {
            output += "; ";
            output += entry.key;
            output += " = ";
            output += entry.value;
            output += "\n";
        }
    }
    return output;
}

// Format each borrowed statistics entry on its own comment line, in host order; return owned text, empty when the list is empty, and propagate invalid-entry or allocation failures.
std::string format_gcode_statistics(const GCodeExportMetadataView &metadata)
{
    const GCodeExportMetadataListView entries = metadata.statistics();
    std::string output;
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const GCodeExportMetadataEntryView entry = entries.at(index);
        output += "; ";
        output += entry.key;
        output += " = ";
        output += entry.value;
        output += "\n";
    }
    return output;
}

// Format borrowed configuration entries between application-named delimiters; return owned text and propagate invalid-entry or allocation failures.
std::string format_gcode_export_configuration(const GCodeExportMetadataView &metadata)
{
    const GCodeExportMetadataListView entries = metadata.configuration();
    std::string output = "\n; " SLIC3R_APP_NAME "_config = begin\n";
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const GCodeExportMetadataEntryView entry = entries.at(index);
        output += "; ";
        output += entry.key;
        output += " = ";
        output += entry.value;
        output += "\n";
    }
    output += "; " SLIC3R_APP_NAME "_config = end\n";
    return output;
}

} // namespace

// Copy the C version API's borrowed generated header into owned comment text without changing host state; missing API text throws and allocation failures propagate.
std::string format_gcode_generated_by()
{
    raw_slic3r_version_info version_info = {};
    version_info.struct_size = uint32_t(sizeof(version_info));
    if (orchestrator_get_version_info(&version_info) == 0 || version_info.generated_header == nullptr)
        throw std::runtime_error("The host could not provide the generated G-code header.");
    return "; " + std::string(version_info.generated_header) + "\n\n";
}

// Split the notes on '\n', strip the '\r' of Windows line ends, comment every line and end with an empty line.
// Borrow config, return owned text (empty for empty notes), preserve a final empty note as a "; " line, and propagate allocation failures.
std::string format_gcode_notes(const Config &config)
{
    const std::string notes = config.string_or_default("notes", "");
    if (notes.empty())
        return {};

    std::string output;
    size_t line_begin = 0;
    while (true) {
        const size_t line_end = notes.find('\n', line_begin);
        std::string line = notes.substr(line_begin, line_end == std::string::npos ? std::string::npos : line_end - line_begin);
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        output += "; " + line + "\n";
        if (line_end == std::string::npos)
            break;
        line_begin = line_end + 1;
    }
    return output + "\n";
}

// Borrow render inputs and the required ABI 2.1 metadata (plus the optional ABI 2.2 statistics list), encode requests, and return owned ordered text; invalid metadata or encoder failures throw before file I/O.
GCodeTextEnvelope build_gcode_text_envelope(const OrchestratorView &orchestrator,
                                             const Print &print,
                                             const Config &config,
                                             const raw_gcode_thumbnail_renderer *renderer,
                                             const std::vector<GCodeThumbnailRequest> &requests,
                                             const raw_gcode_export_metadata *metadata)
{
    const GCodeExportMetadataView metadata_view(metadata);
    std::array<std::string, 3> blocks;
    if (!requests.empty())
        blocks = encode_gcode_thumbnails(orchestrator, print, config, renderer, requests);

    GCodeTextEnvelope envelope;
    envelope.header = blocks[RAW_GCODE_THUMBNAIL_FILE_PREFIX] + format_gcode_generated_by() +
                      blocks[RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE] + format_gcode_notes(config) +
                      format_gcode_extrusion_widths(metadata_view);
    envelope.trailer = format_gcode_statistics(metadata_view) + format_gcode_export_configuration(metadata_view) +
                       blocks[RAW_GCODE_THUMBNAIL_FILE_SUFFIX];
    return envelope;
}


// Open borrowed UTF-8 paths, write the borrowed envelope around bounded reference blocks, then flush and close; I/O or cancellation throws and the host removes partial output.
void write_gcode_text_file(const plugin_run_context &run_context,
                           std::string_view reference_path_utf8,
                           std::string_view output_path_utf8,
                           const GCodeTextEnvelope &envelope)
{
    const std::string reference_path(reference_path_utf8);
    const std::string output_path(output_path_utf8);
    boost::nowide::ifstream input(reference_path.c_str(), std::ios::in | std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open G-code reference file '" + reference_path + "'.");
    boost::nowide::ofstream output(output_path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("Cannot open temporary G-code output file '" + output_path + "'.");

    output.write(envelope.header.data(), static_cast<std::streamsize>(envelope.header.size()));
    if (!output)
        throw std::runtime_error("Failed while writing the temporary G-code output file.");

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

    output.write(envelope.trailer.data(), static_cast<std::streamsize>(envelope.trailer.size()));
    output.flush();
    if (!output)
        throw std::runtime_error("Failed while flushing the temporary G-code output file.");
    output.close();
    if (!output)
        throw std::runtime_error("Failed while closing the temporary G-code output file.");
}

} // namespace slic3r_api
