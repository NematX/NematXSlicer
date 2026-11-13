///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher.
#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <boost/filesystem.hpp>
#include <tbb/global_control.h>

#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/TriangleMesh.hpp"

// Qualifies pressure exports from a two-layer STL through the normal pipeline.
// The external firmware and Reader are mandatory. Outputs remain in the run
// directory supplied by the runner, including files from failed assertions.
//     configuration -> STL reload -> Print::apply -> process -> host export
//     same Print -> changed configuration -> apply/process -> fresh comparison
//     real Reader -> signed extrusion and final coordinates
//     limit/precision checks -> Reader positions and SDK lexical words -> numeric bounds
//     wipe checks -> relative withdrawal blocks and E-only restoration -> unchanged commands
using namespace Slic3r;
using namespace Slic3r::Test::GCode;

namespace {
// Return a complete one-tool profile with explicitly established startup axes.
DynamicPrintConfig qualification_config();
// Reload a small ASCII STL into an owned model and apply the complete profile.
// Report mesh loading and Print initialization times separately.
void initialize(Print &print, Model &model, const DynamicPrintConfig &config);
// Process/export through the host and return the retained file's bytes.
// Provider failures propagate; later assertion failures do not remove outputs.
// Report processing and host export times separately for the retained file.
std::string generate(Print &print, const std::string &name);
// Remove comments and blank rows, retaining command order and numeric spelling.
std::string useful(const std::string &text);
// Compare ordered polylines, signed E at cycle/layer boundaries and writer
// accounting. Save Reader measurements after structural checks, then report
// separate Reader and assertion timings. A fatal assertion can prevent saving.
void compare(Print &print, const std::string &nominal, const std::string &actual, const std::string &name);
// Return the largest absolute annotated correction, requiring each phase's K.
double correction(const std::string &text, double k);
// Copy the numeric annotation fields from one generated phase; reject missing
// or duplicate words without interpreting any G-code command.
std::map<std::string, std::string> phase_fields(const std::string &line);
// Check only annotated movement words with the SDK lexer, allowing equivalent
// decimal/scientific spelling while requiring the configured output quantum.
void check_precision(const std::string &text, bool extrusion, int digits);
// Collect actual wipe blocks and relative E-only restorations in source order.
// The two-tool qualification profile is relative; each restoration must repay
// an earlier withdrawal, but a tool may retain debt at the end of the file.
std::vector<std::string> wipe_commands(const std::string &text);

// A typed key with a representative value or exemption, and its coverage rationale.
struct Domain {
    const char *key;
    raw_config_option_type type;
    const char *value;
    const char *reason;
};

// These are pressure inputs, not a catalogue of every firmware feature. Empty
// values identify explicit exclusions whose existing proof is named here.
const Domain pressure_domains[] = {
    {"extruder_pressure_factor", RAW_CO_VECTOR_FLOAT, "0.02", "four coupled K/F09 modes in both E modes"},
    {"split_extrusion_acceleration", RAW_CO_VECTOR_BOOL, "1", "four coupled K/F09 modes in both E modes"},
    {"filament_compressibility_factor", RAW_CO_VECTOR_FLOAT, "3.5", "inert edit and return to neutral on the same Print"},
    {"use_relative_e_distances", RAW_CO_BOOL, "0", "absolute and relative exports"},
    {"machine_limits_usage", RAW_CO_ENUM, "limits", "ignore, time_estimate_only and limits"},
    {"machine_max_acceleration_x", RAW_CO_VECTOR_FLOAT, "30", "low projected X acceleration"},
    {"machine_max_acceleration_y", RAW_CO_VECTOR_FLOAT, "40", "low projected Y acceleration"},
    {"machine_max_acceleration_z", RAW_CO_VECTOR_FLOAT, "2", "Z changes in this STL are travels and therefore close printing cycles"},
    {"machine_max_acceleration_e", RAW_CO_VECTOR_FLOAT, "1", "low projected filament acceleration"},
    {"machine_max_acceleration_extruding", RAW_CO_VECTOR_FLOAT, "20", "low overall printing acceleration"},
    {"machine_max_feedrate_x", RAW_CO_VECTOR_FLOAT, "4", "low projected X speed"},
    {"machine_max_feedrate_y", RAW_CO_VECTOR_FLOAT, "5", "low projected Y speed"},
    {"machine_max_feedrate_z", RAW_CO_VECTOR_FLOAT, "1", "Z travels remain barriers"},
    {"machine_max_feedrate_e", RAW_CO_VECTOR_FLOAT, "0.1", "low projected filament speed"},
    {"gcode_precision_xyz", RAW_CO_INT, "2", "phase merging at coarse XYZ precision"},
    {"gcode_precision_e", RAW_CO_INT, "3", "signed balance at coarse E precision"},
    {"extruder_axis", RAW_CO_VECTOR_STRING, "", "the two-tool STL below writes A/B; generated two-tool pressure export also covers SEMM"},
    {"use_volumetric_e", RAW_CO_BOOL, "", "pressure failures never replace a published file proves active volumetric refusal"},
    {"single_extruder_multi_material", RAW_CO_BOOL, "", "generated two tool pressure export handles queued fans and SEMM closes shared physical cycles"}
};

// Provider inputs not used by pressure retain their published types. Only the
// explicit entries below are exempt from pressure variation domains.
const Domain legacy_exemptions[] = {
    {"deretract_speed", RAW_CO_VECTOR_FLOAT, "", "semantic retraction remains unchanged"},
    {"extruder_fan_offset", RAW_CO_VECTOR_PERCENT, "", "fan encoding, not pressure physics"},
    {"fan_printer_min_speed", RAW_CO_INT, "", "fan encoding"},
    {"extruder_offset", RAW_CO_VECTOR_POINT, "", "nominal writer geometry"},
    {"extruder_temperature_offset", RAW_CO_VECTOR_FLOAT, "", "heater encoding"},
    {"extrusion_multiplier", RAW_CO_VECTOR_FLOAT, "", "nominal writer flow"},
    {"fan_percentage", RAW_CO_BOOL, "", "fan encoding"},
    {"filament_diameter", RAW_CO_VECTOR_FLOAT, "", "nominal writer units; volumetric pressure is refused"},
    {"gcode_label_objects", RAW_CO_ENUM, "", "pressure boundaries preserve same tool junctions and semantic scripts covers neutral markers"},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, "", "nominal writer geometry"},
    {"retract_speed", RAW_CO_VECTOR_FLOAT, "", "semantic retraction remains unchanged"},
    {"thumbnail_background", RAW_CO_ENUM, "", "thumbnail rendering"},
    {"thumbnail_background_color", RAW_CO_STRING, "", "thumbnail rendering"},
    {"thumbnail_color_mode", RAW_CO_ENUM, "", "thumbnail rendering"},
    {"thumbnails", RAW_CO_VECTOR_POINT, "", "thumbnail rendering"},
    {"thumbnails_color", RAW_CO_STRING, "", "thumbnail rendering"},
    {"thumbnails_end_file", RAW_CO_BOOL, "", "thumbnail rendering"},
    {"thumbnails_format", RAW_CO_ENUM, "", "thumbnail rendering"},
    {"thumbnails_tag_format", RAW_CO_BOOL, "", "thumbnail rendering"},
    {"thumbnails_with_bed", RAW_CO_BOOL, "", "thumbnail rendering"},
    {"thumbnails_with_support", RAW_CO_BOOL, "", "thumbnail rendering"},
    {"travel_speed", RAW_CO_FLOAT, "", "travels close cycles without pressure correction"},
    {"use_firmware_retraction", RAW_CO_BOOL, "", "generated paths retain nominal accounting and semantic commands covers ordinary retraction; firmware retraction is refused"},
    {"z_offset", RAW_CO_FLOAT, "", "nominal writer geometry"},
    {"disable_fan_from_unused_extruder", RAW_CO_BOOL, "", "fan encoding"},
    {"gcode_line_number", RAW_CO_BOOL, "", "pressure output filtering numbers only final lines proves contiguous numbering"},
    {"gcode_no_comment", RAW_CO_BOOL, "", "pressure output filtering numbers only final lines proves comment removal after pressure"}
};

// Use Reader endpoints to select genuine axis-aligned phases, then bound the
// annotated speed or acceleration by the exercised X/Y/E/printing limit.
// Z is excluded because this STL changes Z only in non-printing moves.
void check_limit(Print &print, const std::string &text, const Domain &domain);

// Return uncovered or mistyped runtime keys; callers explicitly test rejection.
std::vector<std::string> uncovered(const std::vector<Plugin::DefinedConfigKey> &defined,
                                  const std::vector<Plugin::UsedConfigKey> &used);

DynamicPrintConfig qualification_config()
{
    REQUIRE(std::getenv("SLIC3R_TEST_EXTERNAL_PLUGIN_REPOSITORY") != nullptr);
    REQUIRE(std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT") != nullptr);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    if (std::getenv("SLIC3R_PRESSURE_FEATURE_FIXTURES") != nullptr) {
        // Definitions are initialized before selector freezing; execution is
        // enabled only inside the native feature-pair case below.
        for (const char *id : {"gridvarspeed", "nematx_gridvarspeed_speed", "nematx_gridvarspeed_recipe", "nematx_corner_stretch"})
            REQUIRE(Orchestrator::instance().set_plugin_active(id, false));
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_flavor", "nematx"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"nozzle_diameter", "0.4"}, {"perimeters", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"fill_density", "20%"},
        {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"top_solid_min_thickness", "0"}, {"bottom_solid_min_thickness", "0"},
        {"solid_infill_below_area", "0"},
        {"cooling", "0"}, {"disable_fan_first_layers", "0"},
        {"slowdown_below_layer_time", "0"}, {"fan_below_layer_time", "0"},
        {"use_relative_e_distances", "1"}, {"use_volumetric_e", "0"},
        {"machine_limits_usage", "ignore"},
        {"extruder_pressure_factor", "0.02"}, {"split_extrusion_acceleration", "1"},
        {"filament_compressibility_factor", "0"},
        {"gcode_precision_xyz", "3"}, {"gcode_precision_e", "5"},
        {"gcode_no_comment", "0"}, {"gcode_line_number", "0"},
        {"retract_length", "0.8"}, {"wipe", "1"},
        {"wipe_only_crossing", "0"}, {"wipe_speed", "40"},
        {"retract_before_travel", "0.1"}, {"retract_layer_change", "1"},
        {"seam_position", "rear"}, {"seam_visibility", "0"}
    });
    config.set_key_value("start_gcode", new ConfigOptionString(
        "G71\nG90\nG1 G90 X0 Y0 Z0 G91 A0 F6000\n{\"A[SET_POSITION POS=0]\"}\n"));
    config.set_key_value("end_gcode", new ConfigOptionString(
        ";Q_VOLUME={extruded_volume[0]}\n;Q_RETRACT={e_retracted[0]}\n"));
    config.set_num_extruders(1);
    return config;
}

void initialize(Print &print, Model &model, const DynamicPrintConfig &config)
{
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    const boost::filesystem::path directory(std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT"));
    boost::filesystem::create_directories(directory);
    const std::string path = (directory / "box-4x4x0.4.stl").string();
    TriangleMesh mesh = make_cube(4., 4., 0.4);
    REQUIRE(store_stl(path.c_str(), &mesh, false));
    REQUIRE(load_stl(path.c_str(), &model));
    REQUIRE(model.objects().size() == 1);
    model.objects()[0].add_instance();
    model.objects()[0].ensure_on_bed();
    const std::chrono::steady_clock::time_point loaded = std::chrono::steady_clock::now();
    print.apply(model, config);
    print.set_status_silent();
    std::cout << "PRESSURE_TIMING initialize mesh_s=" << std::chrono::duration<double>(loaded - started).count()
              << " apply_s=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - loaded).count() << '\n';
}

std::string generate(Print &print, const std::string &name)
{
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    print.process();
    const std::chrono::steady_clock::time_point processed = std::chrono::steady_clock::now();
    REQUIRE(print.objects()[0].layer_count() == 2);
    const boost::filesystem::path path = boost::filesystem::path(
        std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT")) / (name + ".gcode");
    Orchestrator::instance().export_gcode(print, path.string(), nullptr);
    const std::chrono::steady_clock::time_point exported = std::chrono::steady_clock::now();
    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(print.gcode_reader_plugin_id() == "gcode.reader.nematx");
    const std::string output = read_text_file(path);
    std::cout << "PRESSURE_TIMING generate " << name << " process_s=" << std::chrono::duration<double>(processed - started).count()
              << " export_s=" << std::chrono::duration<double>(exported - processed).count()
              << " read_s=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - exported).count() << '\n';
    return output;
}

std::string useful(const std::string &text)
{
    std::istringstream rows(text);
    std::string row, result;
    while (std::getline(rows, row)) {
        row.erase(row.find(';') == std::string::npos ? row.size() : row.find(';'));
        const size_t end = row.find_last_not_of(" \t\r");
        if (end != std::string::npos) result += row.substr(0, end + 1) + '\n';
    }
    return result;
}

void compare(Print &print, const std::string &nominal, const std::string &actual, const std::string &name)
{
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    GCodeReaderService reader(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult before = reader.read_for_viewer("gcode.reader.nematx", nominal);
    const GCodeProcessorResult after = reader.read_for_viewer("gcode.reader.nematx", actual);
    const std::chrono::steady_clock::time_point parsed = std::chrono::steady_clock::now();
    const double xyz_quantum = std::pow(10., -print.config().gcode_precision_xyz.value);
    const double e_quantum = std::pow(10., -print.config().gcode_precision_e.value);
    double max_geometry_error = 0, max_cycle_error = 0;
    size_t cycles = 0;
    REQUIRE(before.moves.size() > 1);
    REQUIRE(after.moves.size() >= before.moves.size());
    size_t segment = 1;
    Vec3f prior = after.moves.front().position;
    std::map<std::pair<int, int>, double> signed_before, signed_after;
    std::map<int, double> nominal_total, actual_total;
    std::vector<double> nominal_prefix;
    std::vector<std::string> nominal_rows;
    std::istringstream source(nominal);
    std::string source_row;
    while (std::getline(source, source_row)) nominal_rows.push_back(source_row);
    for (const GCodeProcessorResult::MoveVertex &move : before.moves) {
        nominal_total[int(move.extruder_id)] += move.delta_extruder;
        nominal_prefix.push_back(nominal_total.at(int(move.extruder_id)));
    }
    signed_after[{int(after.moves.front().extruder_id), int(std::lround(after.moves.front().position.z() * 1000))}] += after.moves.front().delta_extruder;
    actual_total[int(after.moves.front().extruder_id)] += after.moves.front().delta_extruder;
    for (const GCodeProcessorResult::MoveVertex &move : before.moves)
        signed_before[{int(move.extruder_id), int(std::lround(move.position.z() * 1000))}] += move.delta_extruder;
    for (size_t index = 1; index < after.moves.size(); ++index) {
        const GCodeProcessorResult::MoveVertex &move = after.moves[index];
        signed_after[{int(move.extruder_id), int(std::lround(move.position.z() * 1000))}] += move.delta_extruder;
        actual_total[int(move.extruder_id)] += move.delta_extruder;
        if ((move.position - prior).norm() <= 0.00001f) continue;
        while (segment < before.moves.size() &&
               (before.moves[segment].position - before.moves[segment - 1].position).norm() <= 0.00001f)
            ++segment;
        REQUIRE(segment < before.moves.size());
        const Vec3f start = before.moves[segment - 1].position;
        const Vec3f end = before.moves[segment].position;
        const Vec3f delta = end - start;
        const double t = (move.position - start).dot(delta) / delta.squaredNorm();
        const double prior_t = (prior - start).dot(delta) / delta.squaredNorm();
        CHECK(move.extruder_id == before.moves[segment].extruder_id);
        CHECK(t >= -xyz_quantum / delta.norm());
        CHECK(t >= prior_t - xyz_quantum / delta.norm());
        CHECK(t <= 1 + xyz_quantum / delta.norm());
        CHECK((move.position - (start + float(t) * delta)).norm() <= 2 * xyz_quantum);
        max_geometry_error = std::max(max_geometry_error, double((move.position - (start + float(t) * delta)).norm()));
        if ((move.position - end).norm() <= 0.00001f) {
            // Rounded zero E in GridVarSpeed is still printing. Only a nominal
            // retract or a row without an extrusion word proves a travel barrier.
            bool barrier = segment + 1 == before.moves.size();
            if (!barrier) {
                const GCodeProcessorResult::MoveVertex &next = before.moves[segment + 1];
                REQUIRE(next.gcode_id > 0);
                REQUIRE(next.gcode_id <= nominal_rows.size());
                const std::string &row = nominal_rows[next.gcode_id - 1];
                barrier = next.extruder_id != move.extruder_id || next.delta_extruder < 0 ||
                    (next.delta_extruder == 0 && row.find(" A") == std::string::npos && row.find(" B") == std::string::npos);
            }
            if (before.moves[segment].type == EMoveType::Extrude && barrier) {
                ++cycles;
                max_cycle_error = std::max(max_cycle_error, std::abs(actual_total.at(int(move.extruder_id)) - nominal_prefix[segment]));
                CHECK(actual_total.at(int(move.extruder_id)) == Approx(nominal_prefix[segment]).margin(e_quantum + 0.000001));
            }
            ++segment;
        }
        prior = move.position;
    }
    while (segment < before.moves.size() &&
           (before.moves[segment].position - before.moves[segment - 1].position).norm() <= 0.00001f)
        ++segment;
    CHECK(segment == before.moves.size());
    REQUIRE(signed_before.size() == signed_after.size());
    for (const std::pair<const std::pair<int, int>, double> &layer : signed_before) {
        CAPTURE(layer.first);
        CHECK(signed_after.at(layer.first) == Approx(layer.second).margin(e_quantum + 0.000001));
    }
    for (const char *key : {";Q_VOLUME=", ";Q_RETRACT="}) {
        const size_t a = nominal.find(key), b = actual.find(key);
        REQUIRE(a != std::string::npos);
        REQUIRE(b != std::string::npos);
        CHECK(std::stod(nominal.substr(a + std::strlen(key))) ==
              Approx(std::stod(actual.substr(b + std::strlen(key)))).margin(0.00001));
    }
    const double writer_volume = std::stod(actual.substr(actual.find(";Q_VOLUME=") + 10));
    const double writer_retracted = std::stod(actual.substr(actual.find(";Q_RETRACT=") + 11));
    const double area = PI * 1.75 * 1.75 / 4;
    CHECK((actual_total.at(0) + writer_retracted) * area == Approx(writer_volume).margin(e_quantum * area + 0.0001));
    CHECK(actual.find("NX_INTERNAL") == std::string::npos);
    REQUIRE(cycles > 0);
    std::ofstream metrics((boost::filesystem::path(std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT")) /
                           ("reader-" + name + ".json")).string());
    REQUIRE(metrics.is_open());
    metrics.precision(17);
    metrics << "{\"name\":\"" << name << "\",\"nominal_vertices\":" << before.moves.size()
            << ",\"actual_vertices\":" << after.moves.size() << ",\"xyz_quantum\":" << xyz_quantum
            << ",\"max_geometry_error_mm\":" << max_geometry_error << ",\"cycles_checked\":" << cycles
            << ",\"max_cycle_error_mm\":" << max_cycle_error << ",\"e_quantum\":" << e_quantum
            << ",\"signed_nominal_e0_mm\":" << nominal_total.at(0)
            << ",\"signed_actual_e0_mm\":" << actual_total.at(0)
            << ",\"writer_volume0_mm3\":" << writer_volume
            << ",\"writer_retracted0_mm\":" << writer_retracted << "}\n";
    metrics.close();
    REQUIRE(metrics.good());
    std::cout << "PRESSURE_TIMING compare " << name << " reader_s=" << std::chrono::duration<double>(parsed - started).count()
              << " assertions_s=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - parsed).count() << '\n';
}

double correction(const std::string &text, double k)
{
    std::istringstream lines(text);
    std::string line;
    double largest = 0;
    size_t count = 0;
    while (std::getline(lines, line)) {
        const size_t at = line.find(";NX_PRESSURE ");
        if (at == std::string::npos) continue;
        ++count;
        std::map<std::string, std::string> fields;
        std::istringstream words(line.substr(at + 13));
        std::string word;
        while (words >> word) {
            const size_t equal = word.find('=');
            REQUIRE(equal != std::string::npos);
            fields.emplace(word.substr(0, equal), word.substr(equal + 1));
        }
        CHECK(std::stod(fields.at("k_s")) == Approx(k));
        CHECK(std::stod(fields.at("dt_s")) > 0);
        largest = std::max(largest, std::abs(std::stod(fields.at("de_pressure_mm"))));
    }
    REQUIRE(count > 0);
    return largest;
}

std::vector<std::string> uncovered(const std::vector<Plugin::DefinedConfigKey> &defined,
                                  const std::vector<Plugin::UsedConfigKey> &used)
{
    std::map<std::string, raw_config_option_type> covered;
    for (const Domain &domain : pressure_domains) covered.emplace(domain.key, domain.type);
    for (const Domain &domain : legacy_exemptions) covered.emplace(domain.key, domain.type);
    std::vector<std::string> missing;
    for (const Plugin::DefinedConfigKey &key : defined) {
        const std::map<std::string, raw_config_option_type>::const_iterator found = covered.find(key.key);
        if (found == covered.end() || found->second != key.type) missing.push_back(key.key);
    }
    for (const Plugin::UsedConfigKey &key : used) {
        const std::map<std::string, raw_config_option_type>::const_iterator found = covered.find(key.key);
        if (found == covered.end() || found->second != key.type) missing.push_back(key.key);
    }
    return missing;
}
} // namespace

// Check runtime metadata and vary machine-limit and precision inputs; the
// remaining pressure domains are exercised by the replay and focused cases.
TEST_CASE("NematX pressure domains cover the loaded provider and linear STL inputs",
          "[plugins][nematx-pressure-qualification][plugin-e2e][.]")
{
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 1);
    DynamicPrintConfig config = qualification_config();
    const Plugin *provider = Orchestrator::instance().get_plugin("nematx");
    REQUIRE(provider != nullptr);
    REQUIRE_FALSE(provider->get_package_root().empty());
    const std::vector<std::string> missing = uncovered(provider->get_defined_config_keys(), provider->get_used_config_keys());
    CAPTURE(missing);
    REQUIRE(missing.empty());
    for (const Domain &domain : pressure_domains) {
        CAPTURE(domain.key);
        const std::vector<Plugin::UsedConfigKey> &used = provider->get_used_config_keys();
        REQUIRE(std::any_of(used.begin(), used.end(), [&domain](const Plugin::UsedConfigKey &key) {
            return key.key == domain.key && key.type == domain.type;
        }));
        if (std::string(domain.key) == "extruder_pressure_factor" ||
            std::string(domain.key) == "split_extrusion_acceleration" ||
            std::string(domain.key) == "filament_compressibility_factor") {
            const std::vector<Plugin::DefinedConfigKey> &defined = provider->get_defined_config_keys();
            REQUIRE(std::any_of(defined.begin(), defined.end(), [&domain](const Plugin::DefinedConfigKey &key) {
                return key.key == domain.key && key.type == domain.type;
            }));
        }
    }
    std::vector<Plugin::DefinedConfigKey> added = provider->get_defined_config_keys();
    added.push_back({"pressure_test_uncovered_key", RAW_CO_VECTOR_FLOAT});
    CHECK(uncovered(added, provider->get_used_config_keys()) == std::vector<std::string>{"pressure_test_uncovered_key"});
    DynamicPrintConfig neutral_config = config;
    neutral_config.set_deserialize_strict("extruder_pressure_factor", "0");
    neutral_config.set_deserialize_strict("split_extrusion_acceleration", "0");
    Print print;
    Model model;
    initialize(print, model, neutral_config);
    const std::string baseline = generate(print, "domain-neutral-baseline");
    for (const Domain &domain : pressure_domains) {
        if (std::string(domain.key).find("machine_") != 0 &&
            std::string(domain.key).find("gcode_precision_") != 0) continue;
        CAPTURE(domain.key, domain.reason);
        DynamicPrintConfig variant = config;
        variant.set_deserialize_strict("machine_limits_usage", "time_estimate_only");
        variant.set_deserialize_strict(domain.key, domain.value);
        print.apply(model, variant);
        CHECK_FALSE(print.is_step_done(STEP_GCODE));
        CHECK(print.full_print_config().opt_serialize(domain.key) == domain.value);
        const std::string active = generate(print, std::string("domain-") + domain.key);
        REQUIRE(correction(active, 0.02) > 0);
        const std::string key(domain.key);
        if (key.find("machine_max_") == 0 && key != "machine_max_acceleration_z" && key != "machine_max_feedrate_z")
            check_limit(print, active, domain);
        if (key == "gcode_precision_xyz" || key == "gcode_precision_e")
            check_precision(active, key == "gcode_precision_e", std::stoi(domain.value));
        std::string neutral = baseline;
        // Machine limits do not change nominal coordinates or filament. Only
        // output precision needs its own quantized baseline within this run.
        if (std::string(domain.key).find("gcode_precision_") == 0) {
            variant.set_deserialize_strict("extruder_pressure_factor", "0");
            variant.set_deserialize_strict("split_extrusion_acceleration", "0");
            print.apply(model, variant);
            CHECK_FALSE(print.is_step_done(STEP_GCODE));
            neutral = generate(print, std::string("domain-neutral-") + domain.key);
        }
        compare(print, neutral, active, domain.key);
    }
}

// Exercise a genuine STL and retain command identity across an inert profile edit.
TEST_CASE("NematX small STL pressure export and real Print apply replay",
          "[plugins][nematx-pressure-qualification][plugin-e2e][.]")
{
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 1);
    DynamicPrintConfig config = qualification_config();
    Print print;
    Model model;
    initialize(print, model, config);
    for (bool relative : {true, false}) {
        const std::string mode = relative ? "rel" : "abs";
        config.set_deserialize_strict("use_relative_e_distances", relative ? "1" : "0");
        config.set_key_value("start_gcode", new ConfigOptionString(
            "G71\nG90\nG1 G90 X0 Y0 Z0 " + std::string(relative ? "G91" : "G90") +
            " A0 F6000\n{\"A[SET_POSITION POS=0]\"}\n"));
        config.set_deserialize_strict("extruder_pressure_factor", "0");
        config.set_deserialize_strict("split_extrusion_acceleration", "0");
        print.apply(model, config);
        const std::string nominal = generate(print, mode + "-neutral");
        REQUIRE(nominal.find("NX_PRESSURE") == std::string::npos);
        const PrintStateBase::TimeStamp sliced = print.step_state_with_warnings(STEP_POST_SLICING).timestamp;
        std::string split_only, correction_only;
        for (int setting = 0; setting < 5; ++setting) {
            CAPTURE(mode, setting);
            config.set_deserialize_strict("extruder_pressure_factor", (setting == 1 || setting == 2) ? "0.02" : "0");
            config.set_deserialize_strict("split_extrusion_acceleration", (setting == 0 || setting == 2) ? "1" : "0");
            config.set_deserialize_strict("filament_compressibility_factor", setting == 3 ? "3.5" : "0");
            print.apply(model, config);
            CHECK(print.is_step_done(STEP_POST_SLICING));
            CHECK(print.step_state_with_warnings(STEP_POST_SLICING).timestamp == sliced);
            CHECK_FALSE(print.is_step_done(STEP_GCODE));
            const std::string output = generate(print, mode + "-edit-" + std::to_string(setting));
            if (setting == 0 || setting == 2) CHECK(output.find("NX_PRESSURE") != std::string::npos);
            else CHECK(output.find("NX_PRESSURE") == std::string::npos);
            if (setting >= 3) CHECK(useful(output) == useful(nominal));
            else {
                compare(print, nominal, output, mode + "-" + std::to_string(setting));
                CHECK(useful(output) != useful(nominal));
            }
            if (setting == 0) {
                CHECK(correction(output, 0) == 0);
                split_only = useful(output);
            }
            if (setting == 1) correction_only = useful(output);
            if (setting == 2) {
                CHECK(correction(output, 0.02) > 0.05);
                CHECK(useful(output) != split_only);
                CHECK(useful(output) == correction_only);
            }
            if (setting < 4) {
                Print fresh;
                Model fresh_model;
                initialize(fresh, fresh_model, config);
                CHECK(useful(generate(fresh, mode + "-fresh-" + std::to_string(setting))) == useful(output));
                if (setting == 2) {
                    CHECK(useful(generate(print, mode + "-repeat-active")) == useful(output));
                    DynamicPrintConfig compressible = config;
                    compressible.set_deserialize_strict("filament_compressibility_factor", "3.5");
                    print.apply(model, compressible);
                    CHECK_FALSE(print.is_step_done(STEP_GCODE));
                    CHECK(useful(generate(print, mode + "-active-compressibility")) == useful(output));
                    print.apply(model, config);
                    CHECK_FALSE(print.is_step_done(STEP_GCODE));
                }
            } else {
                CHECK(useful(generate(print, mode + "-repeat-neutral")) == useful(output));
            }
        }
    }
}

// Two STL objects select distinct tools while only A has pressure enabled.
TEST_CASE("NematX two tool STL keeps the neutral tool and wipe commands",
          "[plugins][nematx-pressure-qualification][plugin-e2e][.]")
{
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 1);
    DynamicPrintConfig config = qualification_config();
    Orchestrator &orchestrator = Orchestrator::instance();
    const bool wipe_active = orchestrator.is_plugin_active("layer_extrusion_edit.wipe.default");
    REQUIRE(orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", true));
    try {
    config.set_num_extruders(2);
    config.set_deserialize_strict("filament_diameter", "1.75,1.75");
    config.set_deserialize_strict("extrusion_multiplier", "1,1");
    config.set_deserialize_strict("extruder_axis", "\"A\";\"B\"");
    config.set_deserialize_strict("extruder_pressure_factor", "0,0");
    config.set_deserialize_strict("split_extrusion_acceleration", "0,0");
    config.set_key_value("start_gcode", new ConfigOptionString(
        "G71\nG90\nG1 G90 X0 Y0 Z0 G91 A0 F6000\n{\"B[SET_POSITION POS=0]\"}\n{\"A[SET_POSITION POS=0]\"}\n"));
    Print print;
    Model model;
    initialize(print, model, config);
    const std::string path = (boost::filesystem::path(std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT")) /
                              "box-4x4x0.4.stl").string();
    REQUIRE(load_stl(path.c_str(), &model));
    model.objects()[1].add_instance();
    model.objects()[1].instances[0]->set_offset(Vec3d(6., 0., 0.));
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    const std::string neutral = generate(print, "two-tools-neutral");
    config.set_deserialize_strict("extruder_pressure_factor", "0.02,0");
    config.set_deserialize_strict("split_extrusion_acceleration", "1,0");
    print.apply(model, config);
    const std::string active = generate(print, "two-tools-active");
    compare(print, neutral, active, "two-tools");
    REQUIRE(correction(active, 0.02) > 0.05);
    CHECK(active.find("tool=1 phase=") == std::string::npos);
    CHECK(active.find(";WIPE_START") != std::string::npos);
    CHECK(active.find(";WIPE_END") != std::string::npos);
    CHECK(wipe_commands(active) == wipe_commands(neutral));
    std::istringstream rows(active), before(neutral);
    std::string row, b_active, b_neutral;
    while (std::getline(rows, row))
        if (row.find("G1 ") == 0 && row.find(" B") != std::string::npos) b_active += row + '\n';
    while (std::getline(before, row))
        if (row.find("G1 ") == 0 && row.find(" B") != std::string::npos) b_neutral += row + '\n';
    REQUIRE_FALSE(b_neutral.empty());
    CHECK(b_active == b_neutral);
    } catch (...) {
        orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", wipe_active);
        throw;
    }
    REQUIRE(orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", wipe_active));
}

// CLI exports use the normal DLL and embedded firmware. Feature pairs use
// native-host DLLs and the same supplied profile, including generated wipes.
// The scoped activations restore the original host state on assertion failure.
TEST_CASE("NematX CLI standalone embedded and feature pairs pass the real Reader",
          "[plugins][nematx-pressure-qualification][plugin-e2e][.]")
{
    tbb::global_control threads(tbb::global_control::max_allowed_parallelism, 1);
    DynamicPrintConfig config = qualification_config();
    REQUIRE(std::getenv("SLIC3R_PRESSURE_FEATURE_FIXTURES") != nullptr);
    config.set_deserialize_strict({
        {"fill_pattern", "rectilinear"}, {"infill_speed", "30"}, {"perimeter_speed", "20"},
        {"external_perimeter_speed", "20"}, {"first_layer_speed", "20"},
        {"first_layer_infill_speed", "20"}, {"first_layer_min_speed", "0"}
    });
    const char *const feature_ids[] = {"gridvarspeed", "nematx_gridvarspeed_speed", "nematx_gridvarspeed_recipe", "nematx_corner_stretch"};
    Orchestrator &orchestrator = Orchestrator::instance();
    for (const char *id : feature_ids) {
        const Plugin *provider = orchestrator.get_plugin(id);
        REQUIRE(provider != nullptr);
        REQUIRE_FALSE(provider->get_package_root().empty());
        CHECK_FALSE(orchestrator.is_plugin_active(id));
        std::cout << "PRESSURE_FEATURE provider=" << id << " package_root=" << provider->get_package_root() << '\n';
    }
    REQUIRE(config.option<ConfigOptionPercent>("fill_gridvarspeed_width_diff") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatOrPercent>("fill_gridvarspeed_zhop") != nullptr);
    REQUIRE(config.option<ConfigOptionBool>("stretch_corners") != nullptr);
    REQUIRE(config.option<ConfigOptionBool>("stretch_corners_arc") != nullptr);
    const bool wipe_active = orchestrator.is_plugin_active("layer_extrusion_edit.wipe.default");
    REQUIRE(orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", true));
    const boost::filesystem::path directory(std::getenv("SLIC3R_PRESSURE_QUALIFICATION_OUTPUT"));
    try {
        for (const std::string pair : {"grid", "corner", "cooling"}) {
            for (const char *id : feature_ids) {
                const bool enabled = pair == "grid" ? std::string(id) != "nematx_corner_stretch" :
                                     pair == "corner" && std::string(id) == "nematx_corner_stretch";
                REQUIRE(orchestrator.set_plugin_active(id, enabled));
                CHECK(orchestrator.is_plugin_active(id) == enabled);
            }
            DynamicPrintConfig variant = config;
            variant.set_deserialize_strict("extruder_pressure_factor", "0");
            variant.set_deserialize_strict("split_extrusion_acceleration", "0");
            if (pair == "grid") variant.set_deserialize_strict({
                {"fill_pattern", "gridvarspeed"}, {"top_fill_pattern", "gridvarspeed"}, {"top_solid_layers", "1"},
                {"fill_gridvarspeed_width_diff", "50%"}, {"fill_gridvarspeed_zhop", "0"}, {"top_solid_infill_speed", "30"}
            });
            if (pair == "corner") variant.set_deserialize_strict({{"stretch_corners", "1"}, {"stretch_corners_arc", "0"}});
            if (pair == "cooling") variant.set_deserialize_strict({
                {"cooling", "1"}, {"slowdown_below_layer_time", "20"}, {"min_print_speed", "3"}
            });
            Print feature_print;
            Model feature_model;
            initialize(feature_print, feature_model, variant);
            // Match the CLI's centering of the 4 mm box around (100, 100).
            feature_model.objects()[0].instances[0]->set_offset(Vec3d(98., 98., 0.));
            feature_print.apply(feature_model, variant);
            generate(feature_print, "native-" + pair + "-neutral");
            feature_print.full_print_config().save((directory / ("native-" + pair + "-neutral-effective.ini")).string(), ConfigWriteTarget::GCode);
            variant.set_deserialize_strict("extruder_pressure_factor", "0.02");
            variant.set_deserialize_strict("split_extrusion_acceleration", "1");
            feature_print.apply(feature_model, variant);
            CHECK_FALSE(feature_print.is_step_done(STEP_GCODE));
            generate(feature_print, "native-" + pair + "-active");
            feature_print.full_print_config().save((directory / ("native-" + pair + "-active-effective.ini")).string(), ConfigWriteTarget::GCode);
        }
    } catch (...) {
        for (const char *id : feature_ids) orchestrator.set_plugin_active(id, false);
        orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", wipe_active);
        throw;
    }
    for (const char *id : feature_ids) {
        REQUIRE(orchestrator.set_plugin_active(id, false));
        CHECK_FALSE(orchestrator.is_plugin_active(id));
    }
    REQUIRE(orchestrator.set_plugin_active("layer_extrusion_edit.wipe.default", wipe_active));
    Print print;
    Model model;
    initialize(print, model, config);
    for (const char *pair : {"standalone", "embedded", "grid", "corner", "cooling"}) {
        CAPTURE(pair);
        const std::string prefix = std::string(pair) == "standalone" || std::string(pair) == "embedded" ? "cli-" : "native-";
        const std::string neutral = read_text_file(directory / (prefix + pair + "-neutral.gcode"));
        const std::string active = read_text_file(directory / (prefix + pair + "-active.gcode"));
        REQUIRE_FALSE(neutral.empty());
        REQUIRE_FALSE(active.empty());
        compare(print, neutral, active, pair);
        CHECK(correction(active, 0.02) > 0);
        CHECK(neutral.find("NX_PRESSURE") == std::string::npos);
        CHECK(useful(active) != useful(neutral));
    }
    for (const char *state : {"neutral", "active"}) {
        const std::string standalone = read_text_file(directory / (std::string("cli-standalone-") + state + ".gcode"));
        const std::string embedded = read_text_file(directory / (std::string("cli-embedded-") + state + ".gcode"));
        CHECK(useful(standalone) == useful(embedded));
    }
    const std::string base = useful(read_text_file(directory / "cli-embedded-neutral.gcode"));
    CHECK(useful(read_text_file(directory / "native-grid-neutral.gcode")) != base);
    CHECK(useful(read_text_file(directory / "native-corner-neutral.gcode")) != base);
    CHECK(useful(read_text_file(directory / "native-cooling-neutral.gcode")) != base);
    GCodeReaderService reader(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult grid = reader.read_for_viewer("gcode.reader.nematx",
        read_text_file(directory / "native-grid-neutral.gcode"));
    bool full_speed = false, reduced_flow_speed = false;
    for (const GCodeProcessorResult::MoveVertex &move : grid.moves) {
        if (move.extrusion_role != GCodeExtrusionRole::TopSolidInfill || move.delta_extruder <= 0) continue;
        full_speed = full_speed || std::abs(move.feedrate - 30.f) < 0.01f;
        reduced_flow_speed = reduced_flow_speed || std::abs(move.feedrate - 60.f) < 0.01f;
    }
    CHECK(full_speed);
    CHECK(reduced_flow_speed);
    const GCodeProcessorResult base_moves = reader.read_for_viewer("gcode.reader.nematx",
        read_text_file(directory / "cli-embedded-neutral.gcode"));
    const GCodeProcessorResult cooled_moves = reader.read_for_viewer("gcode.reader.nematx",
        read_text_file(directory / "native-cooling-neutral.gcode"));
    std::vector<const GCodeProcessorResult::MoveVertex *> base_printing, cooled_printing;
    for (const GCodeProcessorResult::MoveVertex &move : base_moves.moves)
        if (move.type == EMoveType::Extrude) base_printing.push_back(&move);
    for (const GCodeProcessorResult::MoveVertex &move : cooled_moves.moves)
        if (move.type == EMoveType::Extrude) cooled_printing.push_back(&move);
    REQUIRE_FALSE(base_printing.empty());
    REQUIRE(base_printing.size() == cooled_printing.size());
    bool slowed = false;
    double largest_reduction = 0, minimum_cooled = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < base_printing.size(); ++index) {
        const GCodeProcessorResult::MoveVertex &before = *base_printing[index], &after = *cooled_printing[index];
        CHECK(before.extruder_id == after.extruder_id);
        CHECK(before.extrusion_role == after.extrusion_role);
        CHECK((before.position - after.position).norm() <= 0.00001f);
        CHECK(after.feedrate >= 3.f - 0.01f);
        slowed = slowed || after.feedrate < before.feedrate - 0.01f;
        largest_reduction = std::max(largest_reduction, double(before.feedrate - after.feedrate));
        minimum_cooled = std::min(minimum_cooled, double(after.feedrate));
    }
    CHECK(slowed);
    std::cout << "PRESSURE_SLOWDOWN samples=" << base_printing.size() << " maximum_reduction_mm_s=" << largest_reduction
              << " minimum_feedrate_mm_s=" << minimum_cooled << '\n';
}

namespace {
void check_limit(Print &print, const std::string &text, const Domain &domain)
{
    GCodeReaderService reader(Orchestrator::instance(), nullptr);
    const GCodeProcessorResult actual = reader.read_for_viewer("gcode.reader.nematx", text);
    std::vector<std::string> rows;
    std::istringstream source(text);
    std::string row;
    while (std::getline(source, row)) rows.push_back(row);
    const std::string key(domain.key);
    const bool acceleration = key.find("acceleration") != std::string::npos;
    const bool filament = key.back() == 'e';
    const int axis = key.back() == 'x' ? 0 : key.back() == 'y' ? 1 : -1;
    const double limit = std::stod(domain.value);
    // Annotations retain double precision, independently of rounded coordinates.
    const double tolerance = 1e-6 * std::max(1., limit);
    double largest = 0;
    size_t samples = 0;
    for (size_t index = 1; index < actual.moves.size(); ++index) {
        const GCodeProcessorResult::MoveVertex &move = actual.moves[index];
        REQUIRE(move.gcode_id > 0);
        REQUIRE(move.gcode_id <= rows.size());
        const std::string &line = rows[move.gcode_id - 1];
        if (line.find(";NX_PRESSURE ") == std::string::npos) continue;
        if (axis >= 0) {
            const Vec3f delta = move.position - actual.moves[index - 1].position;
            // Long pure-axis pieces avoid mistaking a tiny rounded diagonal
            // for an axis-aligned source movement.
            if (std::abs(delta[axis]) < 0.01f || std::abs(delta[1 - axis]) > 0.00001f ||
                std::abs(delta.z()) > 0.00001f) continue;
        }
        const std::map<std::string, std::string> fields = phase_fields(line);
        if (acceleration && fields.at("phase") != "accel" && fields.at("phase") != "decel") continue;
        const double begin = std::stod(fields.at(filament ? "e0_mm_s" : "v0_mm_s"));
        const double end = std::stod(fields.at(filament ? "e1_mm_s" : "v1_mm_s"));
        const double value = acceleration ? std::abs(end - begin) / std::stod(fields.at("dt_s")) : std::max(begin, end);
        CAPTURE(domain.key, move.gcode_id, value, limit);
        CHECK(std::isfinite(value));
        CHECK(value <= limit + tolerance);
        largest = std::max(largest, value);
        ++samples;
    }
    REQUIRE(samples > 0);
    CHECK(largest >= 0.9 * limit);
    // These low caps are reachable on the box's long pure-axis phases.
    CHECK(largest == Approx(limit).margin(tolerance));
    std::cout << "PRESSURE_LIMIT " << domain.key << " samples=" << samples << " maximum=" << largest << " cap=" << limit << '\n';
}

void check_precision(const std::string &text, bool extrusion, int digits)
{
    slic3r_api::GCodeGeneration::GCodeLineParser parser;
    const double quantum = std::pow(10., -digits);
    size_t words = 0;
    bool fractional = false;
    parser.parse_buffer(text, [&](slic3r_api::GCodeGeneration::GCodeLineParser &,
                                  const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line) {
        if (line.raw().find(";NX_PRESSURE ") == std::string::npos) return;
        for (const char axis : std::string(extrusion ? "AB" : "XYZ")) {
            std::string_view token = line.parameter_position(axis);
            if (token.empty()) continue;
            token = token.substr(1, token.find_first_of(" \t;", 1) - 1);
            const std::string number(token);
            const double value = std::stod(number);
            CHECK(std::abs(value / quantum - std::round(value / quantum)) <= 1e-7);
            const size_t exponent_at = number.find_first_of("eE");
            const size_t end = exponent_at == std::string::npos ? number.size() : exponent_at;
            const size_t dot = number.find('.');
            const int decimals = dot == std::string::npos || dot >= end ? 0 : int(end - dot - 1);
            const int exponent = exponent_at == std::string::npos ? 0 : std::stoi(number.substr(exponent_at + 1));
            CHECK(std::max(0, decimals - exponent) <= digits);
            fractional = fractional || std::abs(value - std::round(value)) > quantum * 0.25;
            ++words;
        }
    });
    REQUIRE(words > 0);
    CHECK(fractional);
    std::cout << "PRESSURE_PRECISION axis=" << (extrusion ? "E" : "XYZ") << " digits=" << digits << " words=" << words << '\n';
}

std::vector<std::string> wipe_commands(const std::string &text)
{
    slic3r_api::GCodeGeneration::GCodeLineParser parser;
    std::vector<std::string> commands;
    std::map<char, double> pending, withdrawn, restored;
    bool inside = false;
    std::string block;
    parser.parse_buffer(text, [&](slic3r_api::GCodeGeneration::GCodeLineParser &,
                                  const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line) {
        if (line.raw() == ";WIPE_START") { REQUIRE_FALSE(inside); inside = true; }
        if (inside) {
            CHECK(line.raw().find("NX_PRESSURE") == std::string::npos);
            block += line.raw() + '\n';
        }
        for (const char axis : {'A', 'B'}) {
            const std::optional<float> value = line.parameter_value(axis);
            if (!value) continue;
            if (inside && *value < 0) {
                pending[axis] -= *value;
                withdrawn[axis] -= *value;
            } else if (!inside && line.command() == "G1" && *value > 0 &&
                       !line.has_parameter('X') && !line.has_parameter('Y') && !line.has_parameter('Z')) {
                CHECK(line.raw().find("NX_PRESSURE") == std::string::npos);
                CHECK(pending[axis] > 0);
                CHECK(*value <= pending[axis] + 0.0001);
                pending[axis] -= *value;
                restored[axis] += *value;
                commands.push_back(line.raw());
            }
        }
        if (line.raw() == ";WIPE_END") {
            REQUIRE(inside);
            commands.push_back(block);
            block.clear();
            inside = false;
        }
    });
    REQUIRE_FALSE(inside);
    for (const char axis : {'A', 'B'}) {
        CHECK(withdrawn[axis] > 0);
        CHECK(restored[axis] > 0);
        std::cout << "PRESSURE_RETRACTION axis=" << axis << " withdrawn_mm=" << withdrawn[axis]
                  << " restored_mm=" << restored[axis] << " remaining_mm=" << pending[axis] << '\n';
    }
    return commands;
}

std::map<std::string, std::string> phase_fields(const std::string &line)
{
    const size_t at = line.find(";NX_PRESSURE ");
    REQUIRE(at != std::string::npos);
    std::istringstream words(line.substr(at + 13));
    std::map<std::string, std::string> fields;
    std::string word;
    while (words >> word) {
        const size_t equal = word.find('=');
        REQUIRE(equal != std::string::npos);
        REQUIRE(fields.emplace(word.substr(0, equal), word.substr(equal + 1)).second);
    }
    for (const char *key : {"phase", "v0_mm_s", "v1_mm_s", "e0_mm_s", "e1_mm_s", "dt_s"})
        REQUIRE(fields.count(key) == 1);
    REQUIRE(std::stod(fields.at("dt_s")) > 0);
    return fields;
}
} // namespace
