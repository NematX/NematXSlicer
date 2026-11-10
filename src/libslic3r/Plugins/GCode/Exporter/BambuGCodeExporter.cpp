#include "BambuGCodeExporter.hpp"
#include "GCodeExporterShared.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iterator>
#include <locale>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <boost/algorithm/hex.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/uuid/detail/md5.hpp>
#include "miniz/miniz.h"
#include "nlohmann/json.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeExportObjects.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeExportMetadataViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodePrivateFile.hpp"
#include "libslic3r/Format/BambuConfigExport.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Utils.hpp"

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

Bambu plate sliced archive (one plate, SkipModel)
==============================================
The publisher validates the destination, then this writer owns a seekable private
ZIP, finalized before bounded copying to the publisher. XML relationships point
only to entries actually written. The core 3MF model has empty resources/build.
G-code is HEADER + translated CONFIG + EXECUTABLE(reference) + text statistics.
The reference is never edited or loaded wholly into memory. Its complete wrapped
bytes feed both a staged ZIP entry and progressive MD5; the staged compressor is
released before the archive on cancellation or I/O failure. Entries and the
complete archive are limited to less than 4 GiB. This is an SD/LAN container,
not Bambu Connect authorization.
The host's numeric export metadata supplies nominal time/material and plan
layer/Z facts. Measured values are omitted when its statistics list is empty.
The copied printing catalog supplies ids, object/instance indices, visited tools
and deposited-width footprints. collect_plate_json reads object layer 0 through
SDK views; serialize_plate_json owns no views and writes sorted compact JSON.
Object ids are the first printed instance's label, not Orca's PrintObject id.
Object boxes union all printed copies; object area is slices minus holes once.
Object-owned brim admits first-layer support, whereas global brim affects only
bbox_all. Skirt is excluded. Generated towers form one id-1000 entry with union
coverage area and their minimum first-layer thickness. Coordinates are plateau
deposition positions, without bed-origin or tool-offset corrections.
translate_config_to_bambu translates the export configuration once for JSON
and G-code. JSON format version stays fixed; software identity changes only
producer/client metadata, following the host producer line's date policy.
PNGs use explicit 512/128 requests; skipped rendering adds no dangling references.

Operation notes:
- publish_gcode_file() :  validate the destination and publish on success
- write_bambu_gcode_archive() :  own and finalize the private ZIP
- copy_gcode_export_objects() :  copy final-plan object facts
- collect_plate_json() :  read slices and prepare plate metadata
- encode_gcode_thumbnail_images() :  request the two PNG sizes
- archive_metadata() :  form headers and statistics
- add_gcode() :  stream the reference and compute its MD5
- copy_private_gcode_file() :  copy the closed archive to output

Representative direct calls (branches are not a timeline):

    BambuGCodeExporter::export_impl()
        |-- publish_gcode_file()
        `-- result.report()

    write_bambu_gcode_archive()
        |-- Slic3r::ApiHost::copy_gcode_export_objects()
        |-- serialize_plate_json()
        |-- collect_plate_json()
        |-- Slic3r::translate_config_to_bambu()
        |-- mz_zip_writer_finalize_archive()
        `-- copy_private_gcode_file()

    publish_gcode_file writer callback [not a direct call of the enclosing function]
        `-- write_bambu_gcode_archive()
*/
namespace slic3r_api::GCodeGeneration::Exporter {
namespace {
using Slic3r::ApiHost::GCodeExportObject;
using Slic3r::ApiHost::GCodeExportDescription;
const char *const k_dependencies[] = {nullptr};
// Orca revision 42608687, version.inc: SLIC3R_VERSION and SoftFever_VERSION.
constexpr const char *k_bambu_version = "02.08.01.55";
constexpr const char *k_orca_version = "2.5.0-dev";
constexpr const char *k_gcode = "Metadata/plate_1.gcode";
constexpr const char *k_plate_json = "Metadata/plate_1.json";
const raw_used_config_key k_used_keys[] = {
    {"notes", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_background_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnail_color_mode", RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_color", RAW_CO_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_bed", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"thumbnails_with_support", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};
struct Identity { std::string producer; std::string application; std::string client; std::string orca; };
// Compose owned identity strings using the borrowed export configuration and host version API.
Identity archive_identity(const Config &config);
// Return a shared PNG availability/capability diagnostic for QUERY and EXPORT without rendering.
std::string png_incompatibility(const OrchestratorView &orchestrator, const Print &print, const Config &config);
// Format finite host numbers with classic-locale decimal precision.
std::string decimal(double value, int precision = 2);
// Format one escaped XML key/value element from borrowed text.
std::string xml_item(const std::string &key, const std::string &value, const char *tag = "metadata");
// Compose the HEADER text, executable-end/statistics trailer and slice-info XML from borrowed export data.
void archive_metadata(const Config &config, const GCodeExportMetadataView &metadata, const Identity &identity,
    const std::vector<GCodeExportObject> &objects, std::string &header, std::string &trailer, std::string &slice);
// Write header/reference/trailer progressively to the borrowed ZIP and return their MD5.
std::string add_gcode(mz_zip_archive &zip, const plugin_run_context &context, std::string_view reference,
    const std::string &header, const std::string &trailer);
// Collect owned plate values from SDK slices and copied final-plan footprints, retaining no view after return.
PlateJsonDescription collect_plate_json(const Print &print, const Config &config,
    const GCodeExportMetadataView &metadata, const GCodeExportDescription &facts);
// Extend an optional mm bounding box by another defined box.
void merge_plate_box(std::optional<std::array<double, 4>> &box, const std::array<double, 4> &part);
// Translate the configured bed's exact name to the plate JSON token, or unknown.
std::string plate_bed_type(const Config &config);

class BambuGCodeExporter final : public GCodeExporterPluginBase
{
public:
    // Borrow the process-lived host through the base; base initialization failures propagate.
    explicit BambuGCodeExporter(orchestrator_handle *host) : GCodeExporterPluginBase(host) {}
private:
    // Return the static provider id without effects or ownership transfer.
    const char *id_impl() const noexcept override { return "gcode.exporter.bambu3mf"; }
    // Return the static display name without effects or ownership transfer.
    const char *name_impl() const noexcept override { return "Bambu Lab sliced file (.gcode.3mf)"; }
    // Return the static description without effects or ownership transfer.
    const char *description_impl() const noexcept override { return "Writes a model-free Bambu sliced archive for SD or LAN."; }
    // Return borrowed empty dependencies without effects or failure.
    const char *const *dependencies_impl() const noexcept override { return k_dependencies; }
    // Share the execution priority required by exporter alternatives.
    int32_t priority_impl() const noexcept override { return 0; }
    // Rank this alternative after exporters that keep the default group preference of 100.
    int32_t exclusive_group_priority_impl() const noexcept override { return 110; }
    // Copy static visual setting descriptors when supplied and return their count without retaining output storage.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override {
        if (keys != nullptr)
            std::copy(std::begin(k_used_keys), std::end(k_used_keys), keys);
        return int32_t(std::size(k_used_keys));
    }
    // Append the Bambu save-path description, with both complete and short suffix patterns.
    void input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const override;
    // Require Bambu firmware and PNG image capability without geometry access or rendering.
    GCodeExportQuery query_impl(const Print &print, const Config &config, std::string_view firmware) const override;
    // Publish the complete archive from borrowed synchronous inputs, reporting only after commit.
    void export_impl(const plugin_run_context &context, const Print &print, const Config &config,
        std::string_view reference, GCodeThumbnailRendererView renderer, const raw_gcode_export_metadata *metadata,
        const GCodeExportResultReporter &result) const override;
};

// Own the miniz writer state, but borrow the private FILE; init failure throws and destruction releases archive allocations before the FILE owner.
struct ZipWriter {
    mz_zip_archive zip = {};
    explicit ZipWriter(FILE *file) {
        if (!mz_zip_writer_init_cfile(&zip, file, 0))
            throw std::runtime_error("Cannot initialize Bambu archive.");
    }
    // End an initialized archive without closing its borrowed FILE or throwing.
    ~ZipWriter() { mz_zip_writer_end(&zip); }
};
// Own only a successfully opened staged entry's compressor; data/finish failures may already release it, so destruction frees only a remaining pointer, never finishes a failed entry.
struct StagedEntry {
    mz_zip_writer_staged_context &stage;
    explicit StagedEntry(mz_zip_writer_staged_context &value) : stage(value) {}
    // Release outstanding compressor memory through the still-live archive allocator without throwing.
    ~StagedEntry() {
        if (stage.pCompressor != nullptr) {
            stage.pZip->m_pFree(stage.pZip->m_pAlloc_opaque, stage.pCompressor);
            stage.pCompressor = nullptr;
        }
    }
};

// Copy the host version and date suffix into owned identity strings; undated producer lines omit ' on', missing host data throws, allocation failures propagate and no Print or date policy is changed.
Identity archive_identity(const Config &config)
{
    raw_slic3r_version_info version = {};
    version.struct_size = sizeof(version);
    if (!orchestrator_get_version_info(&version) || !version.version || !version.application_name || !version.generated_header)
        throw std::runtime_error("Cannot obtain Bambu archive producer information.");
    const std::string selected = config.has("bambu_slicer_identity") ? config.get("bambu_slicer_identity").serialize() : "superslicer";
    std::string producer = version.generated_header;
    const size_t dated = producer.rfind(" on ");
    const std::string date_suffix = dated == std::string::npos ? std::string() : producer.substr(dated);
    if (dated == std::string::npos && producer.size() >= 3 && producer.compare(producer.size() - 3, 3, " on") == 0)
        producer.resize(producer.size() - 3);
    if (selected == "bambustudio")
        return {std::string("BambuStudio ") + k_bambu_version, std::string("BambuStudio-") + k_bambu_version, k_bambu_version, {}};
    if (selected == "orcaslicer")
        return {std::string("generated by OrcaSlicer ") + k_orca_version + date_suffix, std::string("BambuStudio-") + k_bambu_version, k_bambu_version, k_orca_version};
    return {producer, std::string(version.application_name) + "-" + version.version, version.version, {}};
}
// Format a borrowed numeric value into owned fixed-decimal text using the classic locale; stream/allocation failures propagate and no host data is changed.
std::string decimal(double value, int precision)
{
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}
// Escape borrowed key/value text into an owned XML attribute element of the supplied static tag; allocation failures propagate without changing the input.
std::string xml_item(const std::string &key, const std::string &value, const char *tag)
{
    return std::string("<") + tag + " key=\"" + Slic3r::xml_escape(key) + "\" value=\"" + Slic3r::xml_escape(value) + "\"/>\n";
}
// Fill owned HEADER text, executable-end/statistics trailer and slice-info XML from borrowed catalog, configuration and host metadata; only measured time/material are emitted, labels follow the firmware's real-object limit and invalid inputs or allocation failures propagate.
void archive_metadata(const Config &config, const GCodeExportMetadataView &metadata, const Identity &identity,
    const std::vector<GCodeExportObject> &objects, std::string &header, std::string &trailer, std::string &slice)
{
    const GCodeExportNumericStatisticsView numeric = metadata.numeric_statistics();
    const bool measured = metadata.has_measured_statistics();
    size_t real_count = 0;
    for (const GCodeExportObject &object : objects)
        real_count += object.real ? 1 : 0;
    const std::string style = config.has("gcode_label_objects") ? config.get("gcode_label_objects").serialize() : "disabled";
    const bool labels = real_count > 0 && real_count <= 64 && (style == "firmware" || style == "both");
    header = "; HEADER_BLOCK_START\n; " + identity.producer + "\n";
    if (measured)
        header += "; total estimated time: " + Slic3r::get_time_dhms(float(numeric.estimated_time_seconds)) + "\n";
    header += "; total layer number: " + std::to_string(numeric.layer_count) + "\n";
    std::string ids;
    for (const GCodeExportObject &object : objects) {
        if (object.real && labels)
            ids += (ids.empty() ? "" : ",") + std::to_string(object.id);
    }
    if (labels)
        header += "; model label id: " + ids + "\n";
    header += "; filament_density: " + config.get("filament_density").serialize() + "\n";
    header += "; filament_diameter: " + config.get("filament_diameter").serialize() + "\n";
    header += "; max_z_height: " + decimal(numeric.max_print_z) + "\n";
    std::string filaments;
    slice = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<config><header>\n" +
        xml_item("X-BBL-Client-Type", "slicer", "header_item") + xml_item("X-BBL-Client-Version", identity.client, "header_item");
    if (!identity.orca.empty())
        slice += xml_item("OrcaSlicer-Version", identity.orca, "header_item");
    slice += "</header><plate>\n" + xml_item("index", "1");
    const std::string model = config.string_or_default("printer_model_id", "");
    if (!model.empty())
        slice += xml_item("printer_model_id", model);
    slice += xml_item("nozzle_diameters", config.get("nozzle_diameter").serialize());
    if (measured) {
        slice += xml_item("prediction", std::to_string(int64_t(numeric.estimated_time_seconds)));
        slice += xml_item("weight", decimal(numeric.total_weight_g));
        slice += xml_item("first_layer_time", decimal(numeric.first_layer_time_seconds, 6));
    }
    bool support_used = false;
    const GCodeExportMetadataListView statistics = metadata.statistics();
    for (size_t index = 0; index < statistics.size(); ++index) {
        const GCodeExportMetadataEntryView entry = statistics.at(index);
        if (entry.key == "has_support")
            support_used = entry.value == "1";
    }
    slice += xml_item("support_used", support_used ? "true" : "false");
    slice += xml_item("label_object_enabled", labels ? "true" : "false");
    for (const GCodeExportObject &object : objects) {
        if (object.real)
            slice += "<object identify_id=\"" + std::to_string(object.id) + "\" name=\"" +
                Slic3r::xml_escape(object.name) + "\" skipped=\"false\"/>\n";
    }
    const GCodeExportFilamentListView usage = metadata.filaments();
    for (size_t index = 0; measured && index < usage.size(); ++index) {
        const GCodeExportFilamentUsageView material = usage.at(index);
        if (material.length_mm <= 0. && material.volume_mm3 <= 0.)
            continue;
        const std::string id = std::to_string(material.tool + 1);
        filaments += (filaments.empty() ? "" : ",") + id;
        slice += "<filament id=\"" + id + "\" type=\"" + Slic3r::xml_escape(config.get("filament_type").get_string(material.tool)) +
            "\" color=\"" + Slic3r::xml_escape(config.get("filament_colour").get_string(material.tool)) +
            "\" used_m=\"" + decimal(material.length_mm / 1000.) + "\" used_g=\"" + decimal(material.weight_g) + "\"/>\n";
    }
    slice += "</plate></config>\n";
    header += "; filament: " + filaments + "\n; HEADER_BLOCK_END\n";
    trailer = "; EXECUTABLE_BLOCK_END\n";
    for (size_t index = 0; index < statistics.size(); ++index) {
        const GCodeExportMetadataEntryView entry = statistics.at(index);
        trailer += "; " + std::string(entry.key) + " = " + std::string(entry.value) + "\n";
    }
}
// Feed the exact borrowed reference with its envelope and a separating LF only if needed to ZIP and MD5; the bound reserves that optional byte, local compressor ownership precedes archive destruction and cancellation/read/compression failures throw without finishing a failed entry.
std::string add_gcode(mz_zip_archive &zip, const plugin_run_context &context, std::string_view reference,
    const std::string &header, const std::string &trailer)
{
    const std::string path(reference);
    const uint64_t size = boost::filesystem::file_size(path) + header.size() + trailer.size() + 1;
    if (size >= UINT32_MAX)
        throw std::runtime_error("Bambu G-code entries must be smaller than 4 GiB.");
    boost::nowide::ifstream input(path.c_str(), std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot open Bambu G-code reference.");
    mz_zip_writer_staged_context stage = {};
    if (!mz_zip_writer_add_staged_open(&zip, &stage, k_gcode, size, nullptr, nullptr, 0,
        MZ_DEFAULT_COMPRESSION, nullptr, 0, nullptr, 0))
        throw std::runtime_error("Cannot open Bambu G-code ZIP entry.");
    StagedEntry guard(stage);
    boost::uuids::detail::md5 digest;
    // The same bytes enter both consumers, including header and trailer, with cancellation inside the active staged entry.
    const std::function<void(const char *, size_t)> append = [&](const char *bytes, size_t count) {
        throw_if_cancelled(&context);
        if (count != 0 && !mz_zip_writer_add_staged_data(&stage, bytes, count))
            throw std::runtime_error("Cannot write Bambu G-code ZIP entry.");
        digest.process_bytes(bytes, count);
    };
    append(header.data(), header.size());
    std::array<char, 64 * 1024> buffer;
    bool reference_ends_with_lf = true;
    while (input) {
        input.read(buffer.data(), buffer.size());
        const size_t count = size_t(input.gcount());
        append(buffer.data(), count);
        if (count != 0)
            reference_ends_with_lf = buffer[count - 1] == '\n';
    }
    if (!input.eof())
        throw std::runtime_error("Cannot read Bambu G-code reference.");
    if (!reference_ends_with_lf)
        append("\n", 1);
    append(trailer.data(), trailer.size());
    if (!mz_zip_writer_add_staged_finish(&stage))
        throw std::runtime_error("Cannot finalize Bambu G-code ZIP entry.");
    return bambu_md5_hex(digest);
}
// Append borrowed save-dialog data using the shared helper; description text is copied to thread-local buffers and allocation/configuration failures propagate.
void BambuGCodeExporter::input_definitions(const Config &config, std::vector<raw_config_option_def> &definitions) const
{
    const GCodeFileFormat format = {id_impl(), ".gcode.3mf", {}};
    const GCodeOutputPathDescription description = {"Bambu sliced file", "Where the sliced archive is written.", "*.gcode.3mf;*.3mf"};
    append_gcode_output_path_input_definition(format, config, definitions, description);
}
// Inspect the borrowed firmware identity, then PNG presence and image capability; return owned incompatibility text for an unavailable encoder without rendering or writes, while other service failures propagate.
GCodeExportQuery BambuGCodeExporter::query_impl(const Print &print, const Config &config, std::string_view firmware) const
{
    GCodeExportQuery query;
    query.status = RAW_GCODE_EXPORT_QUERY_INCOMPATIBLE;
    if (firmware != "bambu") {
        query.message = "Bambu sliced files require the bambu firmware.";
        return query;
    }
    query.message = png_incompatibility(OrchestratorView(m_orchestrator), print, config);
    if (!query.message.empty())
        return query;
    query.status = RAW_GCODE_EXPORT_QUERY_COMPATIBLE;
    query.extension = ".gcode.3mf";
    query.action_label = L("Export Bambu sliced file");
    query.supports_removable_media = true;
    return query;
}
// Check the active provider before acquiring its session, then inspect raw-image support without rendering; borrow all inputs, return owned diagnostic text and propagate other service or allocation failures.
std::string png_incompatibility(const OrchestratorView &orchestrator, const Print &print, const Config &config)
{
    const PluginView provider = orchestrator.find_plugin("PNG");
    if (!provider.valid() || provider.step() != GCODE_THUMBNAIL)
        return "The PNG thumbnail encoder is unavailable.";
    std::string message;
    const std::unique_ptr<GCodeThumbnailEncoder> encoder = acquire_gcode_thumbnail_image_encoder(
        orchestrator, print, config, nullptr, "PNG", message);
    return message;
}
// Validate destination before private-file creation, write through the borrowed publisher stream and report only the committed path; inputs stay borrowed and cancellation/rendering/translation/I/O failures propagate.
void BambuGCodeExporter::export_impl(const plugin_run_context &context, const Print &print, const Config &config,
    std::string_view reference, GCodeThumbnailRendererView renderer, const raw_gcode_export_metadata *metadata,
    const GCodeExportResultReporter &result) const
{
    const GCodeFileFormat format = {id_impl(), ".gcode.3mf", {}};
    const std::string path = publish_gcode_file(context, format, gcode_export_output_path(config), reference,
        [&](std::ostream &output) {
            write_bambu_gcode_archive(context, OrchestratorView(m_orchestrator), print, config, reference, renderer.handle(), metadata, output);
            return true;
        });
    result.report(path, "Bambu sliced file");
}

void merge_plate_box(std::optional<std::array<double, 4>> &box, const std::array<double, 4> &part)
{
    if (!box) { box = part; return; }
    (*box)[0] = std::min((*box)[0], part[0]);
    (*box)[1] = std::min((*box)[1], part[1]);
    (*box)[2] = std::max((*box)[2], part[2]);
    (*box)[3] = std::max((*box)[3], part[3]);
}

std::string plate_bed_type(const Config &config)
{
    const std::string name = config.has("curr_bed_type") ? config.get("curr_bed_type").serialize() : "";
    const std::map<std::string, std::string> types = {{"Cool Plate", "cool_plate"}, {"Engineering Plate", "eng_plate"},
        {"High Temp Plate", "hot_plate"}, {"Textured PEI Plate", "textured_plate"},
        {"Textured Cool Plate", "textured_cool_plate"}, {"Supertack Plate", "supertack_plate"}};
    const std::map<std::string, std::string>::const_iterator found = types.find(name);
    return found == types.end() ? "unknown" : found->second;
}

PlateJsonDescription collect_plate_json(const Print &print, const Config &config,
    const GCodeExportMetadataView &metadata, const GCodeExportDescription &facts)
{
    PlateJsonDescription out;
    if (facts.used_tools.empty()) return out;
    std::map<uint32_t, std::vector<const GCodeExportObject *>> groups;
    for (const GCodeExportObject &entry : facts.objects)
        if (entry.real) groups[entry.object_index].push_back(&entry);
    std::optional<std::array<double, 4>> all;
    for (const auto &[index, instances] : groups) {
        const Object object = print.object(index);
        if (object.layer_count() == 0) continue;
        const Layer first_layer = object.layer(0);
        std::optional<std::array<double, 4>> local, placed;
        double area = 0.;
        for (ExPolygon slice : first_layer.slices()) {
            area += slice.area() * unscaled(1.) * unscaled(1.);
            for (c_point point : slice.contour().points()) {
                const double x = unscaled(point.x), y = unscaled(point.y);
                merge_plate_box(local, {x, y, x, y});
            }
        }
        if (!local) continue;
        const GCodeExportObject *first = instances.front();
        bool has_brim = false;
        for (const GCodeExportObject *entry : instances) {
            if (entry->instance_index < first->instance_index ||
                (entry->instance_index == first->instance_index && entry->id < first->id)) first = entry;
            has_brim = has_brim || entry->brim.present;
            const c_point shift = object.instance_shift(entry->instance_index);
            merge_plate_box(placed, {(*local)[0] + unscaled(shift.x), (*local)[1] + unscaled(shift.y),
                                    (*local)[2] + unscaled(shift.x), (*local)[3] + unscaled(shift.y)});
        }
        // Only attributable generated brim admits support into an object's box; global brim cannot do so.
        if (has_brim)
            for (const GCodeExportObject *entry : instances) {
                if (entry->brim.present) merge_plate_box(placed, entry->brim.bbox);
                if (entry->support.present) merge_plate_box(placed, entry->support.bbox);
            }
        out.objects.push_back({first->id, first->name, *placed, area, unscaled(first_layer.height())});
        merge_plate_box(all, *placed);
    }
    if (out.objects.empty()) return out;
    if (facts.wipe_tower.present) {
        out.objects.push_back({1000, "wipe_tower", facts.wipe_tower.bbox, facts.wipe_tower.area, facts.wipe_tower.layer_height});
        merge_plate_box(all, facts.wipe_tower.bbox);
    }
    if (facts.global_brim.present) merge_plate_box(all, facts.global_brim.bbox);
    out.bbox_all = *all;
    out.filament_ids = facts.used_tools;
    out.first_extruder = facts.initial_tool;
    for (uint32_t tool : facts.used_tools) out.filament_colors.push_back(config.get("filament_colour").get_string(tool));
    out.nozzle_diameter = config.get("nozzle_diameter").get_float(facts.initial_tool);
    out.is_seq_print = config.get("complete_objects").get_bool();
    out.bed_type = plate_bed_type(config);
    if (metadata.has_measured_statistics()) out.first_layer_time = metadata.numeric_statistics().first_layer_time_seconds;
    return out;
}
}

std::string serialize_plate_json(const PlateJsonDescription &description)
{
    if (description.objects.empty() || description.filament_ids.empty()) return {};
    // JSON would silently turn non-finite doubles into null, which is not a numeric plate field.
    const std::function<void(double)> finite = [](double value) {
        if (!std::isfinite(value)) throw std::runtime_error("Bambu plate JSON requires finite numbers.");
    };
    for (double value : description.bbox_all) finite(value);
    finite(description.nozzle_diameter);
    finite(description.first_layer_time);
    nlohmann::json objects = nlohmann::json::array();
    for (const PlateJsonObject &object : description.objects) {
        for (double value : object.bbox) finite(value);
        finite(object.area);
        finite(object.layer_height);
        objects.push_back({{"id", object.id}, {"name", object.name}, {"bbox", object.bbox},
                           {"area", object.area}, {"layer_height", object.layer_height}});
    }
    return nlohmann::json{{"bbox_all", description.bbox_all}, {"bbox_objects", objects},
        {"filament_ids", description.filament_ids}, {"filament_colors", description.filament_colors},
        {"first_extruder", description.first_extruder}, {"nozzle_diameter", description.nozzle_diameter},
        {"is_seq_print", description.is_seq_print}, {"bed_type", description.bed_type},
        {"first_layer_time", description.first_layer_time}, {"version", 2}}.dump();
}

// Finish the borrowed progressive digest and encode its words using Boost's uppercase hex formatter; return owned text, propagating allocation failures and retaining no host data.
std::string bambu_md5_hex(boost::uuids::detail::md5 &digest)
{
    boost::uuids::detail::md5::digest_type words = {};
    digest.get_digest(words);
    std::string hex;
    boost::algorithm::hex(std::begin(words), std::end(words), std::back_inserter(hex));
    return hex;
}
// Copy owned plan facts, then write ZIP entries for G-code, settings, PNGs and first-layer JSON.
// Collection, encoding and I/O failures propagate to the caller.
void write_bambu_gcode_archive(const plugin_run_context &context, const OrchestratorView &orchestrator,
    const Print &print, const Config &config, std::string_view reference,
    const raw_gcode_thumbnail_renderer *renderer, const raw_gcode_export_metadata *metadata, std::ostream &output)
{
    const std::string unavailable = png_incompatibility(orchestrator, print, config);
    if (!unavailable.empty())
        throw std::runtime_error(unavailable);
    const GCodeExportDescription facts = Slic3r::ApiHost::copy_gcode_export_objects(print.handle());
    const std::vector<GCodeExportObject> &objects = facts.objects;
    const std::string plate_json = serialize_plate_json(collect_plate_json(print, config, GCodeExportMetadataView(metadata), facts));
    const Identity identity = archive_identity(config);
    const Slic3r::BambuConfigTranslation translation = Slic3r::translate_config_to_bambu(*Slic3r::ApiHost::to_config(config.handle()));
    if (!translation.warnings.empty()) {
        std::string warnings;
        for (const std::string &warning : translation.warnings)
            warnings += warning + "\n";
        report_warning(&context, warnings.c_str());
    }
    GCodePrivateFile file(reference, "bambu3mf");
    {
        ZipWriter writer(file.handle());
        mz_zip_archive &zip = writer.zip;
        // Every small entry is copied synchronously by miniz; image data is already compressed and stored verbatim.
        const std::function<void(const char *, const std::string &, mz_uint)> add =
            [&](const char *name, const std::string &bytes, mz_uint compression) {
                throw_if_cancelled(&context);
                if (!mz_zip_writer_add_mem(&zip, name, bytes.data(), bytes.size(), compression))
                    throw std::runtime_error(std::string("Cannot write Bambu archive entry: ") + name);
            };
        std::vector<GCodeThumbnailRequest> requests;
        for (uint32_t size : {512u, 128u})
            requests.push_back({gcode_thumbnail_request_for_size(config, size, size), "PNG", RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE, false});
        const std::vector<GCodeThumbnailEncodedImage> images = encode_gcode_thumbnail_images(orchestrator, print, config, renderer, requests);
        bool large = false, small = false;
        add("[Content_Types].xml", "<?xml version=\"1.0\"?><Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
            "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
            "<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>"
            "<Default Extension=\"png\" ContentType=\"image/png\"/><Default Extension=\"gcode\" ContentType=\"text/x.gcode\"/>"
            "<Default Extension=\"config\" ContentType=\"application/xml\"/><Default Extension=\"md5\" ContentType=\"text/plain\"/>"
            "<Override PartName=\"/Metadata/project_settings.config\" ContentType=\"application/json\"/>" +
            (plate_json.empty() ? std::string() : "<Default Extension=\"json\" ContentType=\"application/json\"/>") + "</Types>", MZ_DEFAULT_COMPRESSION);
        for (const GCodeThumbnailEncodedImage &image : images) {
            const bool is_large = image.width == 512;
            const char *name = is_large ? "Metadata/plate_1.png" : "Metadata/plate_1_small.png";
            add(name, std::string(reinterpret_cast<const char *>(image.bytes.data()), image.bytes.size()), MZ_NO_COMPRESSION);
            large |= is_large;
            small |= !is_large;
        }
        std::string relationships = "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
            "<Relationship Target=\"/3D/3dmodel.model\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>";
        if (large)
            relationships += "<Relationship Target=\"/Metadata/plate_1.png\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>"
                "<Relationship Target=\"/Metadata/plate_1.png\" Id=\"rel-4\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-middle\"/>";
        if (small)
            relationships += "<Relationship Target=\"/Metadata/plate_1_small.png\" Id=\"rel-5\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-small\"/>";
        add("_rels/.rels", relationships + "</Relationships>", MZ_DEFAULT_COMPRESSION);
        std::string model = "<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" xmlns:BambuStudio=\"http://schemas.bambulab.com/package/2021\">"
            "<metadata name=\"Application\">" + Slic3r::xml_escape(identity.application) + "</metadata><metadata name=\"BambuStudio:3mfVersion\">1</metadata>";
        if (!identity.orca.empty())
            model += "<metadata name=\"OrcaSlicer\">" + identity.orca + "</metadata>";
        add("3D/3dmodel.model", model + "<resources/><build/></model>", MZ_DEFAULT_COMPRESSION);
        add("Metadata/project_settings.config", Slic3r::bambu_project_settings_json(translation, k_bambu_version), MZ_DEFAULT_COMPRESSION);
        std::string header, trailer, slice;
        archive_metadata(config, GCodeExportMetadataView(metadata), identity, objects, header, trailer, slice);
        header += "; CONFIG_BLOCK_START\n";
        for (const std::pair<std::string, std::string> &entry : Slic3r::bambu_config_block(translation))
            header += "; " + entry.first + " = " + entry.second + "\n";
        header += "; CONFIG_BLOCK_END\n; EXECUTABLE_BLOCK_START\n";
        const std::string md5 = add_gcode(zip, context, reference, header, trailer);
        add("Metadata/plate_1.gcode.md5", md5, MZ_DEFAULT_COMPRESSION);
        std::string settings = "<config><plate>\n" + xml_item("plater_id", "1") + xml_item("plater_name", "") +
            xml_item("locked", "false") + xml_item("gcode_file", k_gcode);
        if (large)
            settings += xml_item("thumbnail_file", "Metadata/plate_1.png");
        if (!plate_json.empty()) {
            add(k_plate_json, plate_json, MZ_DEFAULT_COMPRESSION);
            settings += xml_item("pattern_bbox_file", k_plate_json);
        }
        add("Metadata/model_settings.config", settings + "</plate></config>", MZ_DEFAULT_COMPRESSION);
        add("Metadata/_rels/model_settings.config.rels", "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
            "<Relationship Target=\"/Metadata/plate_1.gcode\" Id=\"rel-1\" Type=\"http://schemas.bambulab.com/package/2021/gcode\"/></Relationships>", MZ_DEFAULT_COMPRESSION);
        add("Metadata/slice_info.config", slice, MZ_DEFAULT_COMPRESSION);
        if (!mz_zip_writer_finalize_archive(&zip))
            throw std::runtime_error("Cannot finalize Bambu archive.");
        if (zip.m_archive_size >= UINT32_MAX)
            throw std::runtime_error("Bambu archives must be smaller than 4 GiB.");
    }
    file.close();
    file.open_read();
    copy_private_gcode_file(context, file.handle(), output);
    file.close();
}
// Register one statically owned provider through the borrowed orchestrator; host registration failures propagate.
void register_bambu_gcode_exporter_plugin(orchestrator_handle *orchestrator)
{
    static BambuGCodeExporter plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}
}
