#include <catch2/catch.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"

/*
This production gate slices two small square islands and publishes their G-code.
It checks that the wipe plugin divides retraction between stationary and moving
motions and applies nozzle-relative inside-end depths. Each Print owns its plan;
only copied segments and parsed machine moves escape the processing phase.
Both wipe passes are active; the crossing-only variation also requires a real
generated crossing against the sliced target mask and a non-empty returning wipe.
The setting catalogue requires a variation domain for every used and defined
key of the first pass and every used key of the crossing pass.
Catalogue variations change one setting at a time, except the dedicated depth
and inside-end variants, which also enable inside-end and disable return.
Only those variants measure inside-end depth.
Additional lift cases use explicit groups of lift, retraction,
crossing-only and inside-end/return settings.

An ordered scope groups printable content and its surrounding transition phases.
collect_wipes copies geometric leaves marked Wipe from each scope's after phase.
check_output locates those segments in emitted order by both XYZ endpoints, then
checks their joins and measures E deltas, including stationary commands at the
source endpoint. These checks use output rounding tolerances, not G-code comments.
Only complete two-millimetre retractions immediately following deposited material
are measured; initial/final machine events are not mistaken for source wipes.
The scene has straight paths, one tool and no custom scripts. Lift variations
check that the whole wipe rises continuously and joins the following travel.

    TEST_CASE: validate catalogue and compare linear setting variations
    `-- run_case: slice the islands and publish using the production exporter
        |-- collect_wipes: copy per-scope wipe segments and layer Z
        `-- check_output: parse E and XYZ, match wipes, check joins and inside offsets
*/
namespace {
using namespace Slic3r;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;
constexpr const char *wipe_id = "layer_extrusion_edit.wipe.default";

struct Move { Vec3d a; Vec3d b; double e; };
struct Wipe { std::vector<c_extrusion_segment> segments; Vec3d source; double z = 0.; std::optional<Move> travel; };

// Save and restore plugin activation after the case-local Prints are destroyed.
struct Activation {
    Orchestrator &host;
    std::vector<Plugin *> previous;
    bool success = true;
    // Retain unrelated providers and enable wipe, straight travel, lift and text export.
    explicit Activation(Orchestrator &value) : host(value)
    {
        for (Plugin *plugin : host.active_plugins()) previous.push_back(plugin);
        success &= host.set_plugin_active(wipe_id, true);
        success &= host.set_plugin_active("layer_extrusion_edit.wipe.crossing", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.default", true);
        success &= host.set_plugin_active("layer_extrusion_edit.lift.default", true);
        success &= host.set_plugin_active(k_default_gcode_exporter_id, true);
    }
    // Restore the captured active set without retaining any case-local plan views.
    ~Activation()
    {
        host.clear_active_plugins();
        for (Plugin *plugin : previous) host.set_plugin_active(plugin, true);
    }
};

// Copy wipe leaves in child order, inheriting role flags; the borrowed tree remains unchanged.
void append_wipe(const slic3r_api::ExtrusionEntity &entity, raw_extrusion_role role, Wipe &wipe);
// Copy outgoing wipe geometry from marked scopes, without retaining handles into the Print.
void collect_wipes(slic3r_api::MutableExtrusionEntity entity, double z, std::vector<Wipe> &wipes);
// Read the last geometric endpoint of printable content in child order; return false for empty content.
bool content_end(const slic3r_api::ExtrusionEntity &entity, double z, Vec3d &point);
// Copy the first moving connector from a borrowed travel phase, skipping points that emit no machine move.
bool first_travel(const slic3r_api::ExtrusionEntity &entity, double z, Move &move);
// Compare XYZ within the text exporter's millimetre rounding tolerance.
bool near(const Vec3d &a, const Vec3d &b);
// Parse published moves, check each copied wipe in output order, and return deposited volume in mm3.
double check_output(const std::string &gcode, const DynamicPrintConfig &config, const std::vector<Wipe> &wipes,
                    bool check_retraction, double stationary, double inside_depth);
// Slice two islands, export with the production publication helper, check copied geometry and remove successful outputs.
double run_case(const DynamicPrintConfig &config, const std::string &name, bool check_retraction,
                 double stationary, double inside_depth);

// Count generated scope travels crossing the real target mask, in target-instance coordinates.
size_t crossing_travels(slic3r_api::MutableExtrusionEntity entity, const Printing::PrintingExtrusion &extrusion,
    const slic3r_api::ConstPrintingPlan &plan, extruder_index_t tool, double nozzle);

// Exercise enabled defaults and one setting at a time, coupling inside-end with depth only to make it observable.
TEST_CASE("Wipe percentages reach production G-code", "[plugins][plugin-e2e][layer-extrusion-edit][wipe]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    Activation active(host);
    REQUIRE(active.success);
    const Plugin *plugin = host.get_plugin(wipe_id);
    REQUIRE(plugin != nullptr);
    const std::map<std::string, std::vector<std::string>> domains{
        {"retract_before_wipe", {"0%", "100%"}}, {"wipe_inside_depth", {"0%", "100%"}},
        {"wipe_inside_end", {"1"}}, {"wipe_inside_start", {"1"}}, {"wipe", {"0"}},
        {"wipe_speed", {"30"}}, {"wipe_min", {"5"}}, {"wipe_return", {"1"}},
        {"wipe_only_crossing", {"1"}}, {"retract_speed", {"50"}}, {"travel_speed", {"100"}},
        {"nozzle_diameter", {"0.6"}}, {"wipe_extra_perimeter", {"1"}}};
    for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) {
        INFO("used setting: " << key.key);
        REQUIRE(domains.count(key.key) == 1);
    }
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys()) {
        INFO("defined setting: " << key.key);
        REQUIRE(domains.count(key.key) == 1);
    }
    const Plugin *crossing = host.get_plugin("layer_extrusion_edit.wipe.crossing");
    REQUIRE(crossing != nullptr);
    for (const Plugin::UsedConfigKey &key : crossing->get_used_config_keys()) {
        INFO("crossing used setting: " << key.key);
        REQUIRE(domains.count(key.key) == 1);
    }
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    const std::string selector = Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
                                                                        "layer_extrusion_edit.travel");
    baseline.set_deserialize_strict(selector, "layer_extrusion_edit.travel.default");
    baseline.set_deserialize_strict({{"avoid_crossing_perimeters", "0"}, {"wipe", "1"}, {"wipe_only_crossing", "0"},
        {"wipe_min", "4"}, {"wipe_speed", "20"}, {"wipe_return", "0"},
        {"wipe_inside_start", "0"}, {"wipe_inside_end", "0"}, {"wipe_inside_depth", "50%"},
        {"wipe_extra_perimeter", "0"}, {"retract_before_wipe", "20%"}, {"retract_length", "2"},
        {"retract_before_travel", "0"}, {"retract_layer_change", "0"}, {"retract_lift", "0"},
        {"wipe_lift", "0"}, {"wipe_lift_length", "0"},
        {"retract_restart_extra", "0"}, {"retract_speed", "40"}, {"use_firmware_retraction", "0"},
        {"nozzle_diameter", "0.4"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"skirts", "0"}, {"brim_width", "0"}, {"use_relative_e_distances", "0"}, {"binary_gcode", "0"},
        {"filament_diameter", "1.75"}, {"start_gcode", ""}, {"end_gcode", ""},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const double volume = run_case(baseline, "baseline", true, 0.4, -1.);
    for (const std::pair<const std::string, std::vector<std::string>> &domain : domains)
        for (const std::string &value : domain.second) {
            INFO(domain.first << "=" << value);
            DynamicPrintConfig config = baseline;
            config.set_deserialize_strict(domain.first, value);
            double depth = -1.;
            if (domain.first == "wipe_inside_depth" || domain.first == "wipe_inside_end") {
                config.set_deserialize_strict("wipe_inside_end", "1");
                // Zero depth uses the plugin's half-nozzle fallback.
                depth = value == "100%" ? 0.4 : 0.2;
            }
            const double stationary = domain.first == "retract_before_wipe" ? (value == "100%" ? 2. : 0.) : 0.4;
            const double actual = run_case(config, domain.first + "-" + value, domain.first != "wipe", stationary, depth);
            if (domain.first != "nozzle_diameter") CHECK(actual == Approx(volume).margin(0.01));
         }
    for (const char *before : {"20%", "100%"}) {
        DynamicPrintConfig config = baseline;
        config.set_deserialize_strict({{"wipe_lift", "1"}, {"wipe_lift_length", "100%"},
            {"retract_before_wipe", before}, {"wipe_only_crossing", "1"}});
        CHECK(run_case(config, std::string("lift-") + before, true,
                       std::string(before) == "100%" ? 2. : 0.4, -1.) == Approx(volume).margin(0.01));
    }
    DynamicPrintConfig inside_lift = baseline;
    inside_lift.set_deserialize_strict({{"wipe_lift", "1"}, {"wipe_lift_length", "100%"},
        {"wipe_inside_end", "1"}, {"wipe_return", "0"}});
    CHECK(run_case(inside_lift, "lift-inside-end", true, 0.4, 0.2) == Approx(volume).margin(0.01));
}

void append_wipe(const slic3r_api::ExtrusionEntity &entity, raw_extrusion_role role, Wipe &wipe)
{
    if (!entity.valid()) return;
    if (const slic3r_api::EPropertyAttributes *attributes = entity.get(slic3r_api::EPropertyAttributes::key))
        role = attributes->extrusion_role();
    if (RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE))
        for (uint32_t idx = 0; idx < entity.segment_count(); ++idx) wipe.segments.push_back(entity.segment(idx));
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx) append_wipe(entity.child(idx), role, wipe);
}

void collect_wipes(slic3r_api::MutableExtrusionEntity entity, double z, std::vector<Wipe> &wipes)
{
    const auto key = slic3r_api::printing_extrusion_scope_property_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    if (entity.get(key) != nullptr) {
        OrderedExtrusionScope scope(entity, key);
        if (!wipes.empty() && !wipes.back().travel && scope.travel().valid()) {
            Move travel;
            if (first_travel(scope.travel(), z, travel)) wipes.back().travel = travel;
        }
        Wipe wipe;
        wipe.z = z;
        if (scope.after().valid()) append_wipe(scope.after(), RAW_EXTRUSION_ROLE_NONE, wipe);
        if (!wipe.segments.empty()) {
            REQUIRE(content_end(scope.content(), z, wipe.source));
            wipes.push_back(std::move(wipe));
        }
        return;
    }
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx) collect_wipes(entity.child_mutable(idx), z, wipes);
}

bool content_end(const slic3r_api::ExtrusionEntity &entity, double z, Vec3d &point)
{
    for (uint32_t idx = entity.child_count(); idx > 0; --idx)
        if (content_end(entity.child(idx - 1), z, point)) return true;
    if (entity.segment_count() == 0) return false;
    const c_extrusion_segment segment = entity.segment(entity.segment_count() - 1);
    point = Vec3d(unscaled(segment.point_b.x), unscaled(segment.point_b.y), z + unscaled(segment.z_offset_b));
    return true;
}

bool first_travel(const slic3r_api::ExtrusionEntity &entity, double z, Move &move)
{
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        if (first_travel(entity.child(idx), z, move)) return true;
    for (const c_extrusion_segment &segment : entity.segments()) {
        if (segment.point_a.x == segment.point_b.x && segment.point_a.y == segment.point_b.y &&
            segment.z_offset_a == segment.z_offset_b)
            continue;
        move = {{unscaled(segment.point_a.x), unscaled(segment.point_a.y), z + unscaled(segment.z_offset_a)},
                {unscaled(segment.point_b.x), unscaled(segment.point_b.y), z + unscaled(segment.z_offset_b)}, 0.};
        return true;
    }
    return false;
}

bool near(const Vec3d &a, const Vec3d &b) { return (a - b).cwiseAbs().maxCoeff() <= 0.0011; }

double check_output(const std::string &gcode, const DynamicPrintConfig &config, const std::vector<Wipe> &wipes,
                    bool check_retraction, double stationary, double inside_depth)
{
    std::vector<Move> moves;
    double volume = 0., retracted = 0.;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("G0") && !line.cmd_is("G1")) return;
        moves.push_back({{state.x(), state.y(), state.z()},
            {line.new_X(state), line.new_Y(state), line.new_Z(state)}, line.dist_E(state)});
        // Returning filament during an inside-start approach is not deposited volume.
        const double delta = line.dist_E(state);
        if (delta < 0.) retracted -= delta;
        else {
            const double recovered = std::min(retracted, delta);
            retracted -= recovered;
            if (line.dist_XY(state) > 0.) volume += (delta - recovered) * PI * 1.75 * 1.75 / 4.;
        }
    });
    CHECK(volume > 0.);
    size_t cursor = 0, measured = 0, inside = 0;
    for (const Wipe &wipe : wipes) {
        size_t first = 0, last = 0;
        double moving = 0.;
        for (size_t idx = 0; idx < wipe.segments.size(); ++idx) {
            const c_extrusion_segment &segment = wipe.segments[idx];
            const Vec3d a(unscaled(segment.point_a.x), unscaled(segment.point_a.y), wipe.z + unscaled(segment.z_offset_a));
            const Vec3d b(unscaled(segment.point_b.x), unscaled(segment.point_b.y), wipe.z + unscaled(segment.z_offset_b));
            if (config.option("wipe_lift")->serialize() == "1") CHECK(b.z() >= a.z());
            if (idx == 0) CHECK(near(wipe.source, a));
            while (cursor < moves.size() && !(near(moves[cursor].a, a) && near(moves[cursor].b, b))) ++cursor;
            REQUIRE(cursor < moves.size());
            if (idx == 0) first = cursor;
            else {
                CHECK(near(moves[last].b, moves[cursor].a));
                // No unaccounted geometric moves may occur between consecutive wipe segments.
                for (size_t between = last + 1; between < cursor; ++between) CHECK(near(moves[between].a, moves[between].b));
            }
            moving -= std::min(0., moves[cursor].e);
            if (stationary == 2.) CHECK(moves[cursor].e >= -0.0001);
            last = cursor++;
        }
        if (config.option("wipe_lift")->serialize() == "1")
            CHECK(unscaled(wipe.segments.back().z_offset_b) == Approx(1.).margin(0.000002));
        if (first > 0) CHECK(near(moves[first - 1].b, moves[first].a));
        if (last + 1 < moves.size()) CHECK(near(moves[last].b, moves[last + 1].a));
        if (wipe.travel) {
            size_t next = last + 1;
            while (next < moves.size() && near(moves[next].a, moves[next].b)) ++next;
            REQUIRE(next < moves.size());
            CHECK(near(moves[last].b, wipe.travel->a));
            CHECK(near(moves[next].a, wipe.travel->a));
            CHECK(near(moves[next].b, wipe.travel->b));
        }
        size_t previous = first;
        double initial = 0.;
        while (previous > 0 && near(moves[previous - 1].a, moves[previous - 1].b)) {
            initial -= std::min(0., moves[previous - 1].e);
            --previous;
        }
        if (check_retraction && previous > 0 && moves[previous - 1].e > 0.) {
            CHECK(initial == Approx(stationary).margin(0.0002));
            CHECK(initial + moving == Approx(2.).margin(0.0002));
            CHECK(moving == Approx(2. - stationary).margin(0.0002));
            ++measured;
        }
        if (inside_depth >= 0. && wipe.segments.size() >= 2) {
            const c_extrusion_segment &a = wipe.segments[wipe.segments.size() - 2];
            const c_extrusion_segment &b = wipe.segments.back();
            const Vec2d tangent(double(a.point_b.x - a.point_a.x), double(a.point_b.y - a.point_a.y));
            const Vec2d inward(double(b.point_b.x - b.point_a.x), double(b.point_b.y - b.point_a.y));
            if (std::abs(tangent.normalized().dot(inward.normalized())) < 0.001) {
                CHECK(unscaled(inward.norm()) == Approx(inside_depth).margin(0.000002));
                ++inside;
            }
        }
    }
    if (check_retraction) REQUIRE(measured > 0);
    if (inside_depth >= 0.) REQUIRE(inside > 0);
    return volume;
}

size_t crossing_travels(slic3r_api::MutableExtrusionEntity entity, const Printing::PrintingExtrusion &extrusion,
    const slic3r_api::ConstPrintingPlan &plan, const extruder_index_t tool, const double nozzle)
{
    const auto key = slic3r_api::printing_extrusion_scope_property_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    if (entity.get(key) == nullptr) {
        size_t count = 0;
        for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
            count += crossing_travels(entity.child_mutable(idx), extrusion, plan, tool, nozzle);
        return count;
    }
    const OrderedExtrusionScope scope(entity, key);
    if (!scope.has_incoming_transition() || scope.travel().segment_count() == 0 || extrusion.region_island == nullptr)
        return 0;
    const slic3r_api::LayerRegionIsland region(reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
    if (region.region_count() == 0)
        return 0;
    const Layer *layer = reinterpret_cast<const Layer *>(region.region(0).layer().handle());
    AvoidCrossingPerimeters oracle;
    oracle.init_layer(*layer);
    const std::vector<const Layer *> printed{layer};
    oracle.prepare_crossing_test({*layer, printed, extrusion.printing_catalog_id, tool,
        std::max<coord_t>(1, scale_i(nozzle * 0.5)), []() {}});
    const c_point shift = plan.object_instance_shift(slic3r_api::ConstPrintingExtrusion(
        reinterpret_cast<const printing_extrusion_handle *>(&extrusion)));
    Polyline travel;
    const c_point start = scope.travel().segment(0).point_a;
    travel.points.emplace_back(start.x - shift.x, start.y - shift.y);
    for (const c_extrusion_segment &segment : scope.travel().segments())
        travel.points.emplace_back(segment.point_b.x - shift.x, segment.point_b.y - shift.y);
    return oracle.can_cross_perimeter(travel, true) ? 1 : 0;
}

double run_case(const DynamicPrintConfig &config, const std::string &name, bool check_retraction,
                double stationary, double inside_depth)
{
    TriangleMesh mesh = make_cube(10., 10., 0.6);
    TriangleMesh other = make_cube(10., 10., 0.6);
    other.translate(Vec3f(20.f, 0.f, 0.f));
    mesh.merge(other);
    Model model;
    Print print;
    Test::init_print({mesh}, print, model, config);
    print.set_status_silent();
    print.process();
    REQUIRE_FALSE(Orchestrator::instance().is_plugin_cancelled());
    Test::GCode::select_printing_plan_firmware(print, "marlin2");
    const boost::filesystem::path temporary = Test::GCode::temporary_gcode_path();
    const boost::filesystem::path path = temporary.parent_path() / (name + "-" + temporary.filename().string());
    const boost::filesystem::path exported = path.parent_path() / ("export-" + path.filename().string());
    Orchestrator &host = Orchestrator::instance();
    host.export_gcode(print, path.string(), nullptr);
    const Orchestrator::GCodeExporterSelection selection = host.select_gcode_exporter(print, print.full_print_config());
    REQUIRE(selection.plugin_id == k_default_gcode_exporter_id);
    REQUIRE(Test::export_print_to(host, print, print.full_print_config(), path.string(), exported.string(), selection) == exported.string());
    REQUIRE(print.printing_plan() != nullptr);
    std::vector<Wipe> wipes;
    size_t real_crossings = 0;
    const slic3r_api::ConstPrintingPlan plan(reinterpret_cast<const printing_plan_handle *>(print.printing_plan()));
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                     collect_wipes(slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get())),
                                   unscaled(layer.print_z), wipes);
                    if (config.opt_bool("wipe_only_crossing", 0))
                        real_crossings += crossing_travels(slic3r_api::MutableExtrusionEntity(
                            reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get())), extrusion, plan,
                            tool.extruder_id, config.opt_float("nozzle_diameter", tool.extruder_id));
                }
    if (config.opt_bool("wipe_only_crossing", 0)) {
        REQUIRE(real_crossings > 0);
        REQUIRE_FALSE(wipes.empty());
    }
    if (check_retraction) REQUIRE_FALSE(wipes.empty());
    else CHECK(wipes.empty());
    const double volume = check_output(Test::GCode::read_text_file(exported), config, wipes, check_retraction, stationary, inside_depth);
    Test::GCode::remove_output_pair(path);
    Test::GCode::remove_output_pair(exported);
    return volume;
}
} // namespace
