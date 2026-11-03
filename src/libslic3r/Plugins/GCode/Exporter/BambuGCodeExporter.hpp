#ifndef slic3r_BambuGCodeExporter_hpp_
#define slic3r_BambuGCodeExporter_hpp_
#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExporterPlugin.hpp"

/*
Export one sliced plate as a Bambu-compatible ZIP container without model meshes.
The archive carries executable G-code, translated settings, optional thumbnails
and plate metadata. It is a print-ready container, not a project from which the
original model can be recovered or a Bambu Connect authorization package.

The G-code reference supplies already-generated commands. A copied plan catalogue
associates real object instances and generated features with their measurements;
SDK views supply source slices while the Print stays alive. An owned plate
description separates collecting those facts from serializing JSON, so JSON can
also be inspected without creating an archive.

The private ZIP is completed before its bytes enter transactional publication.
That separation allows seekable archive writes without seeking in the publisher's
verified stream or changing the source reference.

Integrated Bambu sliced-file exporter. The archive contains one executable
plateau, translated settings, optional PNGs and first-layer bounding JSON, but no model meshes.
Fill an owned PlateJsonDescription and call serialize_plate_json to inspect JSON
without writing an archive. The writer collects catalog footprints and SDK slices
during EXPORT; all host inputs are borrowed only for that synchronous call.
Boxes are [minx,miny,maxx,maxy] in plateau mm, areas are mm2 and time is seconds.
Filament and initial-tool indices are zero-based. A description with no object
entries or no visited tools produces no JSON text.
*/
namespace boost::uuids::detail { class md5; }
namespace slic3r_api::GCodeGeneration::Exporter {
struct PlateJsonObject
{
    uint32_t id = 0;
    std::string name;
    std::array<double, 4> bbox{};
    double area = 0.;
    double layer_height = 0.;
};
struct PlateJsonDescription
{
    std::array<double, 4> bbox_all{};
    std::vector<PlateJsonObject> objects;
    std::vector<uint32_t> filament_ids;
    std::vector<std::string> filament_colors;
    uint32_t first_extruder = 0;
    double nozzle_diameter = .4;
    bool is_seq_print = false;
    std::string bed_type = "unknown";
    double first_layer_time = 0.;
};
// Serialize finite owned values as compact sorted-key JSON; no objects or visited tools returns empty text.
std::string serialize_plate_json(const PlateJsonDescription &description);
// Register the process-lived Bambu sliced-file provider.
void register_bambu_gcode_exporter_plugin(orchestrator_handle *orchestrator);
// Finish a progressive MD5 as 32 uppercase hexadecimal characters without a newline.
std::string bambu_md5_hex(boost::uuids::detail::md5 &digest);
// Write a private finalized archive to the borrowed sequential stream, cleaning private files on every exit.
void write_bambu_gcode_archive(const plugin_run_context &run_context, const OrchestratorView &orchestrator,
    const Print &print, const Config &config, std::string_view reference,
    const raw_gcode_thumbnail_renderer *renderer, const raw_gcode_export_metadata *metadata, std::ostream &output);
}
#endif
