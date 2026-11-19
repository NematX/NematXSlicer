#include <catch2/catch.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "avoid_crossing_test_helpers.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/AvoidCrossingInput.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/ZonePlanner.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/DefaultZoneContributors.hpp"

/*
This end-to-end gate checks that advanced travel settings
affect a processed printing plan and the text G-code published by the production
exporter. The scene is a 0.6 mm-high square ring with a separate central island,
so travels can stay on one island or cross between the two.

The settings table varies one setting at a time from an enabled baseline,
except that ironing is enabled together with top avoidance. Every setting read
by the plugin must have either a variation or an explicit exclusion; every
setting defined by it must have a variation. This makes missing table entries
fail rather than silently leaving a newly declared setting unchecked.

Each case round-trips the scene through the E2E STL helper, builds and processes
its own Print, verifies the selected provider, then publishes and reads text
G-code. An ordered extrusion scope groups the motions around one extrusion into
named phases; its reserved travel phase holds the connector travel. A leaf here
is an extrusion entity with a polyline and no children. collect_travels walks the
entity tree children-first, opens every ordered scope it finds and copies the
points of its valid travel phase; append_points appends that phase's polyline
leaves in child order and skips consecutive duplicate X, Y, Z-offset triples.
Routes with fewer than two vertices are ignored. The Print owns the plan; the
routes copied into Result outlive it. A route is counted as deviated when it has more
than two vertices; this is not a geometric test of departure from a straight line.

The settings matrix compares disabled avoidance with the straight-travel provider
and varies the selector toward it. One additional run omits the travel selector and
verifies that the implicit advanced provider preserves the explicit baseline routes.
The gate checks that the first-layer exemption and a 5 mm detour limit reduce
deviation counts, and that island weights change the routes while keeping their
endpoints. A separate dumbbell scene checks real travels through its thin bridge, segment
containment in sliced material, and publication of those detours in G-code.
The one-perimeter advanced variation also replays published XY gaps and checks
source-layer, instance and raw-slice provenance, accepted domain routes and zero
planner fallbacks. This replay is restricted to the ring scene with top
avoidance and support disabled; no forbidden-top contributor can apply.
Those bridge assertions apply only to gaps with a source extrusion, identified
by the provider's ordered-scope traversal. Layer-entry travels have no source
extrusion and stay straight by design; they are exempt, regardless of Z or point count.
It compares deposited volume except for settings that change extrusion geometry.
For each case with deviations, at least one intermediate plan XY must occur in
the published G0/G1 XY moves without an E word within 0.001 mm. This does not match every
route to G-code or compare with the core router: it checks one scene's production
integration, not general routing correctness.

The activation guard saves and restores the active plugin set, including the
exporter, after the case-local Prints are destroyed. Output removal is attempted
only on the normal return path, and removal errors are ignored.
Contributor activation cases select advanced travel and round-trip a
reference with both members and each member disabled independently.
The printed-top case activates its opt-in member separately, checks its plugin
declaration, and compares both static-top activation states through STL export.
The support travel case explicitly selects an overhanging roof with a separate
post and two instances. It identifies real support travel phases whose endpoint
chords cross an object slice component containing neither endpoint. Slice copies
are shifted using the plan catalog. With the zone contributors disabled,
the advanced planner uses platter blockers and publishes support detours in G-code.
The support-crossing matrix selects advanced travel and independently
varies crossing-only wipe and avoidance settings. It
counts support wipe leaves, checks straight versus deviated routes and preserves
deposited volume for settings that do not change extrusion geometry. It validates
the declarations of advanced travel and crossing wipe against independent
variation domains, so a newly read setting requires an effective variation.
Within a support extrusion, successive scopes identify support-to-support gaps.
previous_support also pairs the last scope of the preceding support extrusion
with the first scope of the next one when layer, tool and instance are unchanged.
With avoidance disabled, intersection with expanded printed-object areas witnesses
a wiped gap across the part. Two separate overhangs supply distinct support islands;
an unwiped gap over void must join different islands and exceed wipe_min.

The tower scene uses the same two separated overhangs plus a second object printed
with tool two. WipeTowerManager generates its own tower and scopes; no paths or
plan entries are inserted by the test. Consecutive off-object scopes in one layer
are inspected when at least one belongs to the synthetic tower entry. Copied,
translated normal object slices independently classify their straight travel chords.
A support-to-tower chord with positive lengths both inside and outside the
expanded printed areas crosses a contour and must wipe, including when its source
is inside. This check does not classify crossings from endpoint containment.
A long tower-to-tower chord over
void must not. The corresponding scope in the unconditional-wipe run must wipe,
so the void witness is not explained by an ineligible source. A third run places
the tower to the left at (-25,4), making a support-to-tower chord cross original
part material rather than only the crossing detector's expanded margin. It enables
advanced routing and requires outside-part tower gaps crossing a non-containing
part to detour, with every travel segment clear of its original slices. All three
runs preserve deposited volume. Diagnostics name both endpoint kinds and their XY.

The NoTravel scene prints two islands with a NoTravel box between them. Straight
travel, advanced travel with avoidance disabled and advanced travel with avoidance
enabled must keep every detour out of the box, replay model edits like a fresh
Print, and publish the detours as consecutive G-code travel moves.

    TEST_CASE: validate setting coverage and compare the resulting cases
    |-- TravelActivation: enable travel providers and export, then restore the saved set
    `-- run_case: process the ring and central island, publish text, copy observations
        |-- PrintingEntityPropertyTraversal: identify scopes with a source extrusion
        |-- collect_travels: copy reserved travel phases and their source provenance
        |   `-- append_points: join geometric leaves without consecutive duplicate vertices
        |-- count_wipes: count support wipe leaves with inherited roles
        |-- collect_scope_handles: gather scopes for inspecting outgoing wipes and incoming travels
        |   `-- previous_support: join the last and first scopes across support extrusions in the same layer, tool and instance
        `-- GCodeReader::parse_buffer: measure deposited volume and collect emitted travel XY
*/
namespace {
using namespace Slic3r;
enum class Scene { Ring, Support, SupportCrossing, Dumbbell, Tower };
struct Result {
    // Travel provider selected by the host before processing.
    std::string provider;
    // Each route contains consecutive scaled X, Y, Z-offset triples in execution order.
    std::vector<std::vector<coord_t>> travels;
    // For each route, whether the provider's ordered-scope traversal supplies a source extrusion.
    std::vector<bool> travel_has_source;
    // Routes whose endpoints lie on different islands.
    std::vector<std::vector<coord_t>> inter_island;
    // Number of copied routes with more than two vertices, regardless of collinearity.
    size_t deviated = 0;
    // Reserved travel phases found in actual support printing extrusions.
    size_t support_travels = 0;
    // Wipe leaves belonging to support printing extrusions.
    size_t support_wipes = 0;
    // Same-extrusion support gaps with a returning wipe over object material, or no wipe over void.
    size_t support_part_wipes = 0;
    size_t support_void_without_wipe = 0;
    // Support routes whose endpoint chord intersects an object component containing neither endpoint.
    std::vector<std::vector<coord_t>> support_crossings;
    // Routes with an intermediate vertex found among published travel moves.
    std::vector<std::vector<coord_t>> published_detours;
    // Real routes joining the dumbbell's two lobes through its printed bridge.
    std::vector<std::vector<coord_t>> bridge_travels;
    // Bridge routes whose complete sequence after the departure appears in consecutive published XY travel moves.
    size_t published_bridges = 0;
    // Extruded volume read back from the published G-code, in mm³.
    double volume = 0.;
    // Observations for the real two-tool tower scene; scope ordinals identify the same gaps in both wipe modes.
    size_t tower_scopes = 0;
    size_t tower_part_wipes = 0;
    size_t tower_routed_crossings = 0;
    size_t tower_void_without_wipe = 0;
    size_t tower_void_with_wipe = 0;
    uint32_t tower_object_tools = 0;
    std::vector<size_t> tower_void_scope_ids;
    std::vector<size_t> tower_wiped_scope_ids;
};
// Restore the saved active plugin set, including the exporter, after the case-local Prints are destroyed.
struct TravelActivation {
    Orchestrator &host;
    std::vector<Plugin *> previous;
    bool success = true;
    // Capture the active set and enable travel and text export while retaining unrelated providers.
    explicit TravelActivation(Orchestrator &orchestrator) : host(orchestrator)
    {
        for (Plugin *plugin : host.active_plugins())
            previous.push_back(plugin);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.default", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.island_attractors", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", true);
        success &= host.set_plugin_active(k_default_gcode_exporter_id, true);
    }
    // Clear the test activation and reactivate the saved plugins after all case-local Prints are destroyed.
    ~TravelActivation()
    {
        host.clear_active_plugins();
        for (Plugin *plugin : previous)
            host.set_plugin_active(plugin, true);
    }
};
// Slice and process the selected scene, checking the stored provider or implicit advanced default; publish text G-code under file names built from name,
// and return copied routes, support/tower crossing-wipe witnesses, published detour matches and deposited volume.
// Pipeline or export errors remain failures; remove both output pairs only on the normal return path, ignoring removal errors.
Result run_case(const DynamicPrintConfig &config, const std::string &name, Scene scene = Scene::Ring);
// Require the one-perimeter ring scene with top avoidance and support disabled; replay without writing travels and compare XY paths, source provenance and fallback counts.
void check_one_perimeter_routes(Print &print, const Result &result, const DynamicPrintConfig &config, Scene scene);
// Copy reserved travel phases and their source provenance without changing the plan; skip routes below two vertices and count those above two.
void collect_travels(const ExtrusionEntity &entity, const std::set<const extrusion_entity_handle *> &scopes_with_source, Result &result);
// Append geometric leaves in child order, skipping consecutive duplicate XY and Z-offset triples.
void append_points(const ExtrusionEntity &entity, std::vector<coord_t> &points);
// Count marked wipe leaves in a borrowed extrusion tree without retaining its views.
size_t count_wipes(const slic3r_api::ExtrusionEntity &entity, raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE);
// Copy ordered scope handles for immediate inspection while their owning Print remains alive.
void collect_scope_handles(slic3r_api::MutableExtrusionEntity entity,
                           std::vector<slic3r_api::MutableExtrusionEntity> &scopes);
// Publish a real two-island print and require every deviated plan route to occur
// as consecutive, non-extruding G-code XY moves, not just as isolated vertices.
Result no_travel_export(Print &print, const DynamicPrintConfig &config);

Result no_travel_export(Print &print, const DynamicPrintConfig &config)
{
    print.set_status_silent();
    print.process();
    Test::GCode::select_printing_plan_firmware(print, "marlin2");
    const auto reference = Test::GCode::temporary_gcode_path();
    const auto output = Test::GCode::temporary_gcode_path();
    auto &host = Orchestrator::instance();
    host.export_gcode(print, reference.string(), nullptr);
    const auto selection = host.select_gcode_exporter(print, print.full_print_config());
    REQUIRE(selection.plugin_id == k_default_gcode_exporter_id);
    REQUIRE(Test::export_print_to(host, print, print.full_print_config(), reference.string(), output.string(), selection) == output.string());
    Result result;
    for (const auto &group : print.printing_plan()->groups)
        for (const auto &layer : group.layers)
            for (const auto &tool : layer.tool_groups)
                for (const auto &extrusion : tool.extrusions)
                    collect_travels(*extrusion.root, {}, result);
    std::vector<Point> emitted;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(Test::GCode::read_text_file(output), [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.extruding(state) && line.dist_XY(state) > 0.)
            result.volume += line.dist_E(state) * PI * 1.75 * 1.75 / 4.;
        // Extrusions break the sequence; they cannot complete a detour match.
        if (line.extruding(state)) emitted.emplace_back(std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max());
        else if ((line.cmd_is("G0") || line.cmd_is("G1")) && !line.has_e() && (line.has_x() || line.has_y()))
            emitted.push_back(line.new_XY_scaled(state));
    });
    REQUIRE(result.volume > 0.);
    for (const auto &route : result.travels) {
        if (route.size() <= 6) continue;
        // A point within the 0.001 mm output resolution of the previous one publishes no move.
        std::vector<Point> moves;
        Point last(route[0], route[1]);
        for (size_t p = 3; p < route.size(); p += 3) {
            const Point point(route[p], route[p+1]);
            if (std::abs(point.x() - last.x()) <= scale_i(0.001) && std::abs(point.y() - last.y()) <= scale_i(0.001))
                continue;
            moves.push_back(point);
            last = point;
        }
        bool found = false;
        for (size_t begin = 0; begin + moves.size() <= emitted.size() && !found; ++begin) {
            bool matches = true;
            for (size_t p = 0; p < moves.size(); ++p)
                if (std::abs(double(emitted[begin+p].x()) - moves[p].x()) > scale_i(0.001) ||
                    std::abs(double(emitted[begin+p].y()) - moves[p].y()) > scale_i(0.001)) {
                    matches = false; break;
                }
            found = matches;
        }
        CHECK(found);
    }
    Test::GCode::remove_output_pair(reference);
    Test::GCode::remove_output_pair(output);
    return result;
}

TEST_CASE("NoTravel routing reaches G-code and replays model edits", "[plugins][plugin-e2e][no-travel-routing]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    auto &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    // Straight travel, and advanced travel outside and inside perimeter avoidance.
    const std::string mode = GENERATE(std::string("straight"), std::string("advanced"), std::string("advanced-avoiding"));
    const bool avoiding = mode == "advanced-avoiding";
    CAPTURE(mode);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), mode == "straight" ? "layer_extrusion_edit.travel.default" :
                                                            "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"fill_density", "0"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"avoid_crossing_perimeters", avoiding ? "1" : "0"},
        {"avoid_crossing_not_first_layer", "0"}, {"retract_lift", "0"}});
    TriangleMesh mesh = make_cube(4., 4., 0.6);
    TriangleMesh right = make_cube(4., 4., 0.6);
    right.translate(20., 0., 0.); mesh.merge(right);
    Model model;
    auto *object = model.add_object("two islands", "islands.stl", mesh);
    object->add_instance()->set_offset(Vec3d(40., 40., 0.));
    Print replay;
    replay.apply(model, config);
    const auto baseline = no_travel_export(replay, config);
    if (!avoiding) CHECK(baseline.deviated == 0);
    auto *helper = object->add_volume(make_cube(4., 8., 1.), ModelVolumeType::NO_TRAVEL, false);
    helper->set_offset(Vec3d(10., -2., 0.));
    for (int edit = 0; edit < 5; ++edit) {
        CAPTURE(edit);
        if (edit == 1) helper->set_offset(Vec3d(12., -2., 0.));
        if (edit == 2) helper->set_type(ModelVolumeType::SEAM_POSITION_CENTER_Z);
        if (edit == 3) helper->set_type(ModelVolumeType::NO_TRAVEL);
        if (edit == 4) object->delete_volume(1);
        replay.apply(model, config);
        const auto changed = no_travel_export(replay, config);
        Print fresh;
        fresh.apply(model, config);
        const auto expected = no_travel_export(fresh, config);
        CHECK(changed.travels == expected.travels);
        CHECK(changed.volume == Approx(expected.volume));
        CHECK(changed.volume == Approx(baseline.volume));
        if (edit == 0 || edit == 1 || edit == 3) {
            REQUIRE(changed.deviated > 0);
            const double x = edit == 0 ? 50. : 52.;
            const ExPolygons core{ExPolygon(Polygon(Points{Point::new_scale(x+0.002, 38.002),
                Point::new_scale(x+3.998, 38.002), Point::new_scale(x+3.998, 45.998), Point::new_scale(x+0.002, 45.998)}))};
            for (const auto &route : changed.travels) {
                if (route.size() <= 6) continue; // Ordinary detours, not out-of-scope layer entries.
                Polyline path;
                for (size_t p = 0; p < route.size(); p += 3) path.points.emplace_back(route[p], route[p+1]);
                CHECK(intersection_pl(Polylines{path}, core).empty());
            }
        } else if (!avoiding) CHECK(changed.deviated == 0);
        if (edit == 4) CHECK(changed.travels == baseline.travels);
    }
}

// Vary support crossing settings independently through slicing, routing, wiping and text publication.
TEST_CASE("Support crossing settings reach production G-code", "[plugins][plugin-e2e][support-crossing-e2e]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.wipe.default", true));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.wipe.crossing", true));
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    baseline.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_top", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"}, {"fill_density", "20%"},
        {"skirts", "0"}, {"brim_width", "0"}, {"support_material", "1"}, {"support_material_style", "grid"},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
        {"support_material_contact_distance", "0.2"}, {"dont_support_bridges", "0"},
        {"wipe", "1"}, {"wipe_only_crossing", "1"}, {"wipe_min", "2"}, {"wipe_inside_end", "0"},
        {"wipe_return", "0"}, {"retract_before_travel", "0"}, {"retract_length", "2"},
        {"retract_before_wipe", "20%"}, {"retract_lift", "0"}, {"wipe_lift", "0"},
        {"nozzle_diameter", "0.4,0.4"}, {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"avoid_travel_island", "0"}, {"avoid_travel_island_weight", "1"}, {"ironing", "0"},
        {"wipe_speed", "20"}, {"travel_speed", "130"}, {"retract_speed", "40"}, {"wipe_extra_perimeter", "0"},
        {"binary_gcode", "0"}, {"use_relative_e_distances", "0"}, {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const std::vector<Test::PluginE2E::Domain> domains{
        {{"wipe_only_crossing"}, {{"all-wipes", {{"wipe_only_crossing", "0"}}}}},
        {{"avoid_crossing_perimeters"}, {{"straight", {{"avoid_crossing_perimeters", "0"}}}}},
        {{"avoid_crossing_not_first_layer"}, {{"skip-first", {{"avoid_crossing_not_first_layer", "1"}}}}},
        {{"avoid_crossing_perimeters_max_detour"}, {{"detour-limit", {{"avoid_crossing_perimeters_max_detour", "5"}}}}},
        {{"avoid_crossing_perimeters_inset"}, {{"inset", {{"avoid_crossing_perimeters_inset", "50%"}}}}},
        {{"nozzle_diameter"}, {{"nozzle", {{"nozzle_diameter", "0.6,0.4"}}}}},
        {{"bed_shape"}, {{"bed-shape", {{"bed_shape", "-40x-40,140x-40,140x140,-40x140"}}}}},
        {{"avoid_crossing_top"}, {{"tops", {{"avoid_crossing_top", "1"}}}}},
        {{"avoid_travel_island"}, {{"island", {{"avoid_travel_island", "1"}}}}},
        {{"avoid_travel_island_weight"}, {{"island-weight", {{"avoid_travel_island_weight", "2"}}}}},
        // Separate role assignments exercise sparse-only, perimeter-only and solid-only tools.
        {{"infill_extruder"}, {{"infill-tool", {{"infill_extruder", "2"}}}}},
        {{"perimeter_extruder"}, {{"perimeter-tool", {{"perimeter_extruder", "2"}}}}},
        {{"solid_infill_extruder"}, {{"solid-tool", {{"solid_infill_extruder", "2"}}}}},
        {{"ironing"}, {{"ironing", {{"ironing", "1"}}}}},
        {{"perimeters"}, {{"perimeters", {{"perimeters", "3"}}}}},
        {{"wipe"}, {{"no-wipe", {{"wipe", "0"}}}}},
        {{"wipe_speed"}, {{"wipe-speed", {{"wipe_speed", "30"}}}}},
        {{"travel_speed"}, {{"travel-speed", {{"travel_speed", "100"}}}}},
        {{"wipe_min"}, {{"wipe-length", {{"wipe_min", "3"}}}}},
        {{"wipe_extra_perimeter"}, {{"extra-wipe", {{"wipe_extra_perimeter", "1"}}}}},
        {{"retract_speed"}, {{"retract-speed", {{"retract_speed", "50"}}}}},
        {{"retract_before_wipe"}, {{"before-wipe", {{"retract_before_wipe", "100%"}}}}}
    };
    const std::vector<Test::PluginE2E::Case> cases = Test::PluginE2E::linear_cases(host,
        {"layer_extrusion_edit.travel.advanced", "layer_extrusion_edit.wipe.crossing"}, baseline, domains);
    double volume = 0.;
    size_t baseline_wipes = 0;
    for (const Test::PluginE2E::Case &test : cases) {
        INFO(test.name);
        const Result result = run_case(Test::PluginE2E::apply_case(baseline, test), "support-crossing-" + test.name, Scene::SupportCrossing);
        REQUIRE(result.support_travels > 0);
        if (test.name == "no-wipe") CHECK(result.support_wipes == 0);
        else REQUIRE(result.support_wipes > 0);
        if (test.settings.empty()) {
            volume = result.volume;
            baseline_wipes = result.support_wipes;
            CHECK(result.deviated > 0);
            CHECK(std::any_of(result.support_crossings.begin(), result.support_crossings.end(),
                [](const std::vector<coord_t> &route) { return route.size() > 6; }));
        } else {
            if (test.name != "nozzle" && test.name != "ironing" && test.name != "perimeters")
                CHECK(result.volume == Approx(volume).margin(0.01));
            if (test.name == "straight") {
                CHECK(result.deviated == 0);
                CHECK(result.support_part_wipes > 0);
                CHECK(result.support_void_without_wipe > 0);
            }
            if (test.name == "all-wipes")
                CHECK(result.support_wipes > baseline_wipes);
        }
    }
}

// Compare crossing-only/unconditional wipes and advanced platter avoidance in one real two-tool tower scene, preserving volume.
TEST_CASE("Off-object tower gaps reach production crossing wipes", "[plugins][plugin-e2e][off-object-tower-e2e]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.wipe.default", true));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.wipe.crossing", true));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.default");
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", "1"}, {"brim_width", "0"}, {"perimeters", "2"}, {"fill_density", "20%"},
        {"support_material", "1"}, {"support_material_style", "grid"},
        {"support_material_extruder", "1"}, {"support_material_interface_extruder", "1"},
        {"support_material_contact_distance", "0.2"}, {"dont_support_bridges", "0"},
        {"wipe_tower", "1"}, {"wipe_tower_x", "20"}, {"wipe_tower_y", "0"},
        {"wipe_tower_width", "20"}, {"single_extruder_multi_material", "0"},
        {"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"min_layer_height", "0.05,0.05"}, {"max_layer_height", "0.3,0.3"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"},
        {"wipe", "1,1"}, {"wipe_only_crossing", "1,1"}, {"wipe_min", "2,2"},
        {"wipe_inside_end", "0,0"}, {"wipe_return", "0,0"}, {"wipe_lift", "0,0"},
        {"retract_before_travel", "0,0"}, {"retract_length", "2,2"},
        {"retract_length_toolchange", "2,2"}, {"retract_lift", "0,0"},
        {"retract_before_wipe", "20%,20%"}, {"binary_gcode", "0"},
        {"use_relative_e_distances", "0"}, {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result crossing = run_case(config, "off-object-tower-crossing", Scene::Tower);
    REQUIRE(crossing.tower_scopes > 0);
    CHECK(crossing.tower_object_tools == 3);
    CHECK(crossing.tower_part_wipes > 0);
    CHECK(crossing.tower_void_without_wipe > 0);
    config.set_deserialize_strict("wipe_only_crossing", "0,0");
    const Result all = run_case(config, "off-object-tower-all", Scene::Tower);
    CHECK(all.tower_scopes == crossing.tower_scopes);
    CHECK(all.tower_void_with_wipe > 0);
    CHECK(std::any_of(crossing.tower_void_scope_ids.begin(), crossing.tower_void_scope_ids.end(),
        [&all](size_t id) { return std::find(all.tower_wiped_scope_ids.begin(), all.tower_wiped_scope_ids.end(), id) != all.tower_wiped_scope_ids.end(); }));
    CHECK(all.volume == Approx(crossing.volume).margin(0.01));
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"wipe_tower_x", "-25"}, {"wipe_tower_y", "4"},
        {"bed_shape", "-40x-40,140x-40,140x140,-40x140"}});
    const Result routed = run_case(config, "off-object-tower-routed", Scene::Tower);
    CHECK(routed.tower_scopes == crossing.tower_scopes);
    CHECK(routed.tower_routed_crossings > 0);
    CHECK(routed.volume == Approx(crossing.volume).margin(0.01));
    std::cout << "advanced tower: scopes=" << routed.tower_scopes << " routed crossings=" << routed.tower_routed_crossings << '\n';
}

// Check non-exclusive declarations and dependency blocking when the advanced provider is inactive.
TEST_CASE("Advanced zone contributors expose independent activation", "[plugins][travel-zones]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced", false));
    for (const char *id : {"layer_extrusion_edit.travel.advanced.zones.island_attractors",
                           "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"}) {
        const Plugin *plugin = host.get_plugin(id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_id() == id);
        CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
        CHECK(plugin->get_priority() == -50);
        CHECK(plugin->get_exclusive_group().empty());
        CHECK(plugin->get_used_config_keys().empty());
        CHECK(plugin->get_defined_config_keys().empty());
        CHECK(plugin->get_dependencies() == std::vector<std::string>{"layer_extrusion_edit.travel.advanced"});
        CHECK(host.is_plugin_active(id));
    }
    host.block_unsatisfied_plugin_dependencies();
    for (const char *id : {"layer_extrusion_edit.travel.advanced.zones.island_attractors",
                           "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"}) {
        CHECK_FALSE(host.is_plugin_active(id));
        REQUIRE(host.blocked_plugin_activations().count(id) == 1);
        CHECK(host.blocked_plugin_activations().at(id).find("layer_extrusion_edit.travel.advanced") != std::string::npos);
    }
}

// Process the reference and one contributor toggle at a time through STL and text export without changing deposited volume.
TEST_CASE("Advanced contributor activation reaches production G-code", "[plugins][plugin-e2e][travel-zones]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"}, {"ironing", "0"},
        {"binary_gcode", "0"}, {"use_relative_e_distances", "0"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result reference = run_case(config, "contributors-reference");
    REQUIRE(reference.deviated > 0);
    for (const char *id : {"layer_extrusion_edit.travel.advanced.zones.island_attractors",
                           "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"}) {
        INFO(id);
        REQUIRE(host.set_plugin_active(id, false));
        const Result result = run_case(config, "contributor-disabled");
        CHECK(result.provider == "layer_extrusion_edit.travel.advanced");
        CHECK(result.volume == Approx(reference.volume).margin(0.01));
        CHECK_FALSE(result.travels.empty());
        REQUIRE(host.set_plugin_active(id, true));
    }
}

// Check the opt-in declaration and dated-top replacement through STL, slicing and text export.
TEST_CASE("Printed tops are opt-in and replace static tops through G-code", "[plugins][plugin-e2e][travel-zones]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    const char *id = "layer_extrusion_edit.travel.advanced.zones.printed_tops";
    const Plugin *plugin = host.get_plugin(id);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_id() == id);
    CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(plugin->get_priority() == -50);
    CHECK(plugin->get_exclusive_group().empty());
    CHECK(plugin->get_used_config_keys().empty());
    CHECK(plugin->get_defined_config_keys().empty());
    CHECK(plugin->get_dependencies() == std::vector<std::string>{"layer_extrusion_edit.travel.advanced"});
    CHECK_FALSE(host.is_plugin_active(id));
    REQUIRE(host.set_plugin_active(id, true));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"}, {"ironing", "0"},
        {"binary_gcode", "0"}, {"use_relative_e_distances", "0"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result both = run_case(config, "printed-tops-both");
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", false));
    const Result dated = run_case(config, "printed-tops-only");
    CHECK(dated.provider == "layer_extrusion_edit.travel.advanced");
    CHECK(dated.travels == both.travels);
    CHECK(dated.volume == Approx(both.volume).margin(0.01));
    CHECK(dated.volume > 0.);
    REQUIRE_FALSE(dated.travels.empty());
    REQUIRE(host.set_plugin_active(id, false));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", true));
    const Result static_only = run_case(config, "printed-tops-static-reference");
    CHECK(dated.travels != static_only.travels);
    CHECK(dated.volume == Approx(static_only.volume).margin(0.01));
    REQUIRE(host.set_plugin_active(id, true));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced", false));
    host.block_unsatisfied_plugin_dependencies();
    CHECK_FALSE(host.is_plugin_active(id));
    CHECK(host.blocked_plugin_activations().count(id) == 1);
}

// Check that advanced platter routing publishes support detours and preserves the straight-travel extrusion volume.
TEST_CASE("Advanced platter avoidance publishes support detours", "[plugins][plugin-e2e][travel-zones]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const std::string selector = Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel");
    config.set_deserialize_strict(selector, "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"},
        {"support_material", "1"}, {"support_material_style", "grid"}, {"support_material_extruder", "1"},
        {"support_material_interface_extruder", "1"}, {"support_material_contact_distance", "0.2"},
        {"dont_support_bridges", "0"}, {"binary_gcode", "0"}, {"use_relative_e_distances", "0"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.island_attractors", false));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", false));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.printed_tops", false));
    const Result advanced = run_case(config, "support-platter-advanced", Scene::Support);
    REQUIRE(advanced.support_travels > 0);
    CHECK(advanced.provider == "layer_extrusion_edit.travel.advanced");
    size_t observed_detours = 0;
    for (const std::vector<coord_t> &route : advanced.support_crossings) {
        if (route.size() <= 6) continue;
        REQUIRE(std::find(advanced.published_detours.begin(), advanced.published_detours.end(), route) != advanced.published_detours.end());
        ++observed_detours;
    }
    REQUIRE(observed_detours > 0);
    config.set_deserialize_strict(selector, "layer_extrusion_edit.travel.default");
    const Result straight = run_case(config, "support-platter-straight", Scene::Support);
    CHECK(straight.support_travels > 0);
    CHECK(straight.volume == Approx(advanced.volume).margin(0.01));
}

// Validate setting coverage and default route parity through text export; ironing is coupled with top avoidance.
TEST_CASE("Avoid-crossing settings reach production G-code", "[plugins][plugin-e2e][avoid-crossing-perimeters]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    const char *provider = "layer_extrusion_edit.travel.advanced";
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        host, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    std::map<std::string, std::vector<std::string>> domains{
        {"avoid_crossing_perimeters", {"0"}},
        {"avoid_crossing_not_first_layer", {"1"}},
        {"avoid_crossing_perimeters_max_detour", {"5", "200%"}},
        {"avoid_crossing_perimeters_inset", {"50%", "0.2"}},
        {"avoid_crossing_top", {"1"}},
        {"ironing", {"1"}},
        {"perimeters", {"1", "3"}},
        {"avoid_travel_island", {"0"}},
        {"avoid_travel_island_weight", {"0", "2"}},
        {"nozzle_diameter", {"0.6"}},
        {"bed_shape", {"-40x-40,140x-40,140x140,-40x140"}},
        {selector, {"layer_extrusion_edit.travel.default"}}
    };
    // Tool-dependent boundary partitioning is covered by the crossing and boundary tests.
    const std::map<std::string, std::string> exclusions{
        {"perimeter_extruder", "Partitions boundaries by perimeter tool; covered by crossing/boundary tests."},
        {"infill_extruder", "Partitions boundaries by infill tool; covered by crossing/boundary tests."},
        {"solid_infill_extruder", "Partitions boundaries by solid tool; covered by crossing/boundary tests."}
    };
    const Plugin *plugin = host.get_plugin(provider);
    REQUIRE(plugin != nullptr);
    for (const Plugin::UsedConfigKey &key : plugin->get_used_config_keys()) {
        INFO("used setting: " << key.key);
        REQUIRE((domains.count(key.key) + exclusions.count(key.key)) == 1);
    }
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys()) {
        INFO("defined setting: " << key.key);
        REQUIRE(domains.count(key.key) == 1);
    }
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict(selector, provider);
    baseline.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "1"},
        {"nozzle_diameter", "0.4"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"},
        {"ironing", "0"}, {"use_relative_e_distances", "0"}, {"binary_gcode", "0"}, {"filament_diameter", "1.75"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    baseline.set_deserialize_strict("bed_shape", "-50x-50,150x-50,150x150,-50x150");
    const Result enabled = run_case(baseline, "baseline");
    std::cout << "advanced baseline: travels=" << enabled.travels.size() << " deviated=" << enabled.deviated << '\n';
    DynamicPrintConfig implicit = baseline;
    implicit.erase(selector);
    REQUIRE(implicit.option(selector) == nullptr);
    const Plugin *selected = Steps::selected_active_plugin_from_group(host,
        STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel", &implicit);
    REQUIRE(selected != nullptr);
    REQUIRE(selected->get_id() == "layer_extrusion_edit.travel.advanced");
    const Result default_routes = run_case(implicit, "implicit-default");
    CHECK(default_routes.provider == "layer_extrusion_edit.travel.advanced");
    CHECK(default_routes.travels == enabled.travels);
    REQUIRE(enabled.provider == "layer_extrusion_edit.travel.advanced");
    REQUIRE(enabled.deviated > 0);
    Result disabled, straight, weight_zero, weight_two;
    for (const auto &domain : domains)
        for (const std::string &value : domain.second) {
            const std::string domain_name = domain.first == "ironing" ? "avoid_crossing_top + ironing" : domain.first;
            INFO(domain_name << "=" << value);
            DynamicPrintConfig config = baseline;
            config.set_deserialize_strict(domain.first, value);
            // Ironing and top avoidance jointly control the retained top perimeter band.
            if (domain.first == "ironing") {
                config.set_deserialize_strict("avoid_crossing_top", "1");
            }
            const Result result = run_case(config, domain.first + "-" + value);
            const bool is_disabled = domain.first == "avoid_crossing_perimeters" && value == "0";
            const bool is_straight = domain.first == selector && value == "layer_extrusion_edit.travel.default";
            if (is_disabled) disabled = result;
            if (is_straight) straight = result;
            if (domain.first == "avoid_travel_island_weight" && value == "0") weight_zero = result;
            if (domain.first == "avoid_travel_island_weight" && value == "2") weight_two = result;
            // These three settings can change extrusion geometry, so only the other variations must preserve volume.
            if (domain.first != "perimeters" && domain.first != "ironing" && domain.first != "nozzle_diameter")
                CHECK(result.volume == Approx(enabled.volume).margin(0.01));
            if (is_disabled || is_straight)
                CHECK(result.deviated == 0);
            else
                CHECK(result.deviated > 0);
            // A different inset moves the attractor rings, so routes inside the islands change.
            if (domain.first == "avoid_crossing_perimeters_inset")
                CHECK(result.travels != enabled.travels);
            if ((domain.first == "avoid_crossing_perimeters_max_detour" && value == "5") ||
                (domain.first == "avoid_crossing_not_first_layer" && value == "1"))
                CHECK(result.deviated < enabled.deviated);
            std::cout << domain_name << "=" << value << ": travels=" << result.travels.size()
                      << " deviated=" << result.deviated << " volume=" << result.volume << '\n';
        }
    // Without avoidance, advanced travel keeps the direct gaps of straight travel.
    CHECK(disabled.travels == straight.travels);
    REQUIRE_FALSE(weight_zero.inter_island.empty());
    REQUIRE(weight_zero.inter_island.size() == weight_two.inter_island.size());
    // Fixed endpoints distinguish a changed detour from a change in which extrusions the travel connects.
    for (size_t idx = 0; idx < weight_zero.inter_island.size(); ++idx) {
        const std::vector<coord_t> &zero = weight_zero.inter_island[idx];
        const std::vector<coord_t> &two = weight_two.inter_island[idx];
        CHECK(std::vector<coord_t>(zero.begin(), zero.begin() + 3) == std::vector<coord_t>(two.begin(), two.begin() + 3));
        CHECK(std::vector<coord_t>(zero.end() - 3, zero.end()) == std::vector<coord_t>(two.end() - 3, two.end()));
    }
    CHECK(weight_zero.inter_island != weight_two.inter_island);
}

// Observe actual sliced-plan travels through a narrow connected bridge and find their detours in published text.
TEST_CASE("Advanced domain bridge travels reach production G-code", "[plugins][plugin-e2e][advanced-router-e2e]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"thin_walls", "1"}, {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"}, {"ironing", "0"},
        {"binary_gcode", "0"}, {"use_relative_e_distances", "0"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result routed = run_case(config, "domain-dumbbell", Scene::Dumbbell);
    REQUIRE_FALSE(routed.bridge_travels.empty());
    CHECK(routed.published_bridges > 0);
    config.set_deserialize_strict("avoid_crossing_perimeters", "0");
    const Result straight = run_case(config, "domain-dumbbell-straight", Scene::Dumbbell);
    CHECK(straight.volume == Approx(routed.volume).margin(0.01));
}

Result run_case(const DynamicPrintConfig &config, const std::string &name, const Scene scene)
{
    TriangleMesh mesh = make_cube(20., 4., 0.6);
    const std::pair<Vec3d, Vec3f> bars[] = {
        {{20., 4., 0.6}, {0.f, 16.f, 0.f}},
        {{4., 12., 0.6}, {0.f, 4.f, 0.f}},
        {{4., 12., 0.6}, {16.f, 4.f, 0.f}},
        {{4., 4., 0.6}, {8.f, 8.f, 0.f}}
    };
    for (const auto &spec : bars) {
        TriangleMesh bar = make_cube(spec.first.x(), spec.first.y(), spec.first.z());
        bar.translate(spec.second);
        mesh.merge(bar);
    }
    CHECK(mesh.volume() == Approx(163.2).margin(0.001));
    if (scene == Scene::Support || scene == Scene::SupportCrossing || scene == Scene::Tower) {
        mesh = make_cube(4., 8., 2.);
        TriangleMesh post = make_cube(4., 8., 2.);
        post.translate(12., 0., 0.); mesh.merge(post);
        TriangleMesh roof = make_cube(12., 8., 0.6);
        roof.translate(0., 0., 2.);
        mesh.merge(roof);
        CHECK(mesh.volume() == Approx(185.6).margin(0.001));
        if (scene == Scene::SupportCrossing || scene == Scene::Tower) {
            TriangleMesh second = mesh;
            second.translate(0., 20., 0.);
            mesh.merge(second);
        }
    }
    if (scene == Scene::Dumbbell) mesh = Test::Plugins::dumbbell(0.4);
    mesh = Test::PluginE2E::stl_fixture(mesh);
    Model model;
    Print print;
    Test::init_print({mesh}, print, model, config);
    if (scene == Scene::Tower) {
        model.objects().front().instances.front()->set_offset(Vec3d(0., 0., 0.));
        ModelObject *second = model.add_object();
        second->add_volume(Test::PluginE2E::stl_fixture(make_cube(4., 4., 2.6)));
        second->add_instance()->set_offset(Vec3d(60., 0., 0.));
        DynamicPrintConfig regional;
        regional.set_deserialize_strict({{"perimeter_extruder", "2"}, {"infill_extruder", "2"}, {"solid_infill_extruder", "2"}});
        second->config.assign_config(regional);
        print.apply(model, config);
    }
    if (scene == Scene::Support || scene == Scene::SupportCrossing) {
        ModelObject &object = model.objects().front();
        object.add_instance()->set_offset(object.instances.front()->get_offset() + Vec3d(35., 0., 0.));
        print.apply(model, config);
    }
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    Plugin *selected = Steps::selected_active_plugin_from_group(orchestrator,
        STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel", &print.config());
    REQUIRE(selected != nullptr);
    REQUIRE(selected->get_id() == (config.option(selector) == nullptr ?
        "layer_extrusion_edit.travel.advanced" : config.opt_serialize(selector)));
    print.set_status_silent();
    print.process();
    REQUIRE_FALSE(Orchestrator::instance().is_plugin_cancelled());
    Test::GCode::select_printing_plan_firmware(print, "marlin2");
    const boost::filesystem::path temporary = Test::GCode::temporary_gcode_path();
    const boost::filesystem::path path = temporary.parent_path() / (name + "-" + temporary.filename().string());
    Orchestrator::instance().export_gcode(print, path.string(), nullptr);
    const boost::filesystem::path exported = path.parent_path() / ("export-" + path.filename().string());
    Orchestrator &host = Orchestrator::instance();
    const Orchestrator::GCodeExporterSelection selection = host.select_gcode_exporter(print, print.full_print_config());
    REQUIRE(selection.plugin_id == k_default_gcode_exporter_id);
    const std::string published_path = Test::export_print_to(
        host, print, print.full_print_config(), path.string(), exported.string(), selection);
    INFO("exported path: " << print.export_result().local_path);
    REQUIRE(published_path == exported.string());
    REQUIRE(print.printing_plan() != nullptr);
    Result result;
    // Use the provider's scope traversal to distinguish planned gaps from straight layer-entry travels.
    std::set<const extrusion_entity_handle *> scopes_with_source;
    const auto scope_key = slic3r_api::printing_extrusion_scope_property_key(reinterpret_cast<orchestrator_handle *>(&host));
    using ScopeEntity = slic3r_api::PrintingEntity<slic3r_api::PrintingExtrusionScopeProperty>;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers) {
            slic3r_api::PrintingEntityPropertyTraversal<slic3r_api::PrintingExtrusionScopeProperty> traversal(
                scope_key, [&scopes_with_source](ScopeEntity *previous, ScopeEntity *next) {
                    if (previous != nullptr && next != nullptr) scopes_with_source.insert(next->entity.handle());
                }, slic3r_api::MatchingEntityDescendants::Skip);
            slic3r_api::PrintingLayerGroup layer_view(reinterpret_cast<printing_layer_group_handle *>(
                const_cast<Printing::PrintingLayerGroup *>(&layer)));
            traversal.process(layer_view);
        }
    slic3r_api::MutableExtrusionEntity previous_off_scope;
    const Printing::PrintingLayerGroup *previous_off_layer = nullptr;
    std::string previous_off_kind;
    size_t tower_scope_id = 0;
    slic3r_api::MutableExtrusionEntity previous_support;
    const Printing::PrintingLayerGroup *previous_support_layer = nullptr;
    uint32_t previous_support_catalog = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    extruder_index_t previous_support_tool = EXTRUDER_ID_UNKNOWN;
    result.provider = selected->get_id();
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions) {
                    if (scene == Scene::Tower) {
                        const bool tower = extrusion.printing_catalog_id < print.printing_plan()->object_instances.size() &&
                            print.printing_plan()->object_instances[extrusion.printing_catalog_id].object == nullptr;
                        const slic3r_api::LayerRegionIsland region = extrusion.region_island == nullptr ?
                            slic3r_api::LayerRegionIsland() : slic3r_api::LayerRegionIsland(
                                reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                        const bool off_object = !region.valid() || region.region_count() == 0 ||
                            slic3r_api::LayerExtrusionEdit::AvoidCrossing::is_off_object_layer(region.region(0).layer());
                        const std::string kind = tower ? "tower" : extrusion.sregion_island_role.is_support() ? "support" : "skirt";
                        if (!off_object && tool.extruder_id < 2) result.tower_object_tools |= 1u << tool.extruder_id;
                        std::vector<slic3r_api::MutableExtrusionEntity> scopes;
                        collect_scope_handles(slic3r_api::MutableExtrusionEntity(
                            reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get())), scopes);
                        const auto key = slic3r_api::printing_extrusion_scope_property_key(reinterpret_cast<orchestrator_handle *>(&host));
                        for (slic3r_api::MutableExtrusionEntity entity : scopes) {
                            ++tower_scope_id;
                            const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target(entity, key);
                            if (tower) ++result.tower_scopes;
                            if (off_object && previous_off_scope.valid() && previous_off_layer == &layer &&
                                (tower || previous_off_kind == "tower") && target.travel().valid() && target.travel().segment_count() > 0) {
                                const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope source(previous_off_scope, key);
                                const c_extrusion_segment first = target.travel().segment(0);
                                const c_extrusion_segment last = target.travel().segment(target.travel().segment_count() - 1);
                                const Polyline path(Points{Point(first.point_a.x, first.point_a.y), Point(last.point_b.x, last.point_b.y)});
                                ExPolygons printed;
                                for (const Printing::PrintingObjectInstance &instance : print.printing_plan()->object_instances) {
                                    if (instance.object == nullptr) continue;
                                    const Layer *below = nullptr;
                                    for (const Layer &candidate : instance.object->layers()) {
                                        if (candidate.scaled_print_z() >= layer.print_z + SCALED_EPSILON) break;
                                        below = &candidate;
                                    }
                                    if (below == nullptr) continue;
                                    for (ExPolygon area : below->lslices()) {
                                        area.translate(instance.object->instances()[instance.object_instance_idx].shift);
                                        printed.push_back(std::move(area));
                                    }
                                }
                                if (result.provider == "layer_extrusion_edit.travel.advanced") {
                                    // Outside-to-outside gaps retain no domain; only components containing an endpoint may be crossed.
                                    ExPolygons blockers = union_ex(printed);
                                    blockers.erase(std::remove_if(blockers.begin(), blockers.end(), [&path](const ExPolygon &area) {
                                        return area.contains(path.front()) || area.contains(path.back());
                                    }), blockers.end());
                                    if (!blockers.empty() && !intersection_pl(Polylines{path}, blockers).empty()) {
                                        for (size_t segment = 0; segment < target.travel().segment_count(); ++segment) {
                                            const c_extrusion_segment move = target.travel().segment(segment);
                                            const Polyline actual(Points{Point(move.point_a.x, move.point_a.y), Point(move.point_b.x, move.point_b.y)});
                                            CHECK(intersection_pl(Polylines{actual}, blockers).empty());
                                        }
                                        CHECK(target.travel().point_count() > 2);
                                        ++result.tower_routed_crossings;
                                    }
                                }
                                printed = offset_ex(printed, scale_i(config.opt_float("nozzle_diameter", tool.extruder_id) * 0.75));
                                const Polylines interior = intersection_pl(Polylines{path}, printed);
                                const bool touches = !interior.empty();
                                // Positive portions on both sides witness a contour crossing even with an interior source.
                                double inside_length = 0., outside_length = 0.;
                                for (const Polyline &part : interior) inside_length += part.length();
                                for (const Polyline &part : diff_pl(Polylines{path}, printed)) outside_length += part.length();
                                const bool crosses = inside_length > SCALED_EPSILON && outside_length > SCALED_EPSILON;
                                bool inside = false;
                                for (const ExPolygon &area : printed) inside |= area.contains(path.front()) || area.contains(path.back());
                                const bool wiped = source.after().valid() && count_wipes(source.after().readonly()) > 0;
                                if (wiped) result.tower_wiped_scope_ids.push_back(tower_scope_id);
                                if (tower && previous_off_kind == "support" && crosses && wiped)
                                    ++result.tower_part_wipes;
                                if (tower && previous_off_kind == "tower" && !touches && path.length() > scale_i(2.)) {
                                    if (wiped) ++result.tower_void_with_wipe;
                                    else {
                                        ++result.tower_void_without_wipe;
                                        result.tower_void_scope_ids.push_back(tower_scope_id);
                                    }
                                }
                                std::cout << "tower gap " << previous_off_kind << " -> " << kind << " z=" << unscaled(layer.print_z)
                                    << " touches=" << touches << " inside=" << inside << " crosses=" << crosses << " wipe=" << wiped
                                    << " from=" << unscaled(path.front().x()) << ',' << unscaled(path.front().y())
                                    << " to=" << unscaled(path.back().x()) << ',' << unscaled(path.back().y()) << '\n';
                            }
                            previous_off_scope = off_object ? entity : slic3r_api::MutableExtrusionEntity();
                            previous_off_kind = kind;
                            previous_off_layer = &layer;
                        }
                    }
                    const size_t before = result.travels.size();
                    collect_travels(*extrusion.root, scopes_with_source, result);
                    if (extrusion.sregion_island_role.is_support()) {
                        result.support_wipes += count_wipes(slic3r_api::ExtrusionEntity(
                            reinterpret_cast<const extrusion_entity_handle *>(extrusion.root.get())));
                        // Consecutive support scopes in one layer, tool and instance cannot wipe because of a material change.
                        ExPolygons printed;
                        for (const Printing::PrintingObjectInstance &instance : print.printing_plan()->object_instances) {
                            if (instance.object == nullptr) continue;
                            const Layer *below = nullptr;
                            for (const Layer &candidate : instance.object->layers()) {
                                if (candidate.scaled_print_z() >= layer.print_z + SCALED_EPSILON) break;
                                below = &candidate;
                            }
                            if (below == nullptr) continue;
                            const Point shift = instance.object->instances()[instance.object_instance_idx].shift;
                            for (ExPolygon area : below->lslices()) {
                                area.translate(shift);
                                printed.push_back(std::move(area));
                            }
                        }
                        printed = offset_ex(printed, scale_i(config.opt_float("nozzle_diameter", 0) * 0.75));
                        std::vector<slic3r_api::MutableExtrusionEntity> scopes;
                        collect_scope_handles(slic3r_api::MutableExtrusionEntity(
                            reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get())), scopes);
                        if (previous_support.valid() && previous_support_layer == &layer &&
                            previous_support_catalog == extrusion.printing_catalog_id && previous_support_tool == tool.extruder_id)
                            scopes.insert(scopes.begin(), previous_support);
                        if (!scopes.empty()) previous_support = scopes.back();
                        previous_support_layer = &layer;
                        previous_support_catalog = extrusion.printing_catalog_id;
                        previous_support_tool = tool.extruder_id;
                        const slic3r_api::LayerRegionIsland support_region(
                            reinterpret_cast<const layer_region_island_handle *>(extrusion.region_island));
                        REQUIRE(support_region.valid());
                        REQUIRE(support_region.region_count() > 0);
                        REQUIRE(extrusion.printing_catalog_id < print.printing_plan()->object_instances.size());
                        REQUIRE(print.printing_plan()->object_instances[extrusion.printing_catalog_id].object != nullptr);
                        const Layer &support_layer = *reinterpret_cast<const Layer *>(support_region.region(0).layer().handle());
                        const Point support_shift = print.printing_plan()->object_instances[extrusion.printing_catalog_id].object->
                            instances()[print.printing_plan()->object_instances[extrusion.printing_catalog_id].object_instance_idx].shift;
                        const auto scope_key = slic3r_api::printing_extrusion_scope_property_key(
                            reinterpret_cast<orchestrator_handle *>(&host));
                        for (size_t i = 1; i < scopes.size(); ++i) {
                            const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope source(scopes[i - 1], scope_key);
                            const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target(scopes[i], scope_key);
                            if (!target.travel().valid()) continue;
                            std::vector<coord_t> points;
                            append_points(*reinterpret_cast<const ExtrusionEntity *>(target.travel().handle()), points);
                            if (points.size() < 6) continue;
                            Points path;
                            for (size_t p = 0; p < points.size(); p += 3)
                                path.emplace_back(points[p], points[p + 1]);
                            const bool wiped = source.after().valid() && count_wipes(source.after().readonly()) > 0;
                            const bool touches = !intersection_pl(Polylines{Polyline(path)}, printed).empty();
                            bool endpoints_inside = false;
                            for (const ExPolygon &area : printed)
                                endpoints_inside |= area.contains(path.front()) || area.contains(path.back());
                            if (touches && !endpoints_inside && wiped) ++result.support_part_wipes;
                            // Only a retractable jump between distinct support islands witnesses the void rule.
                            size_t from_island = SIZE_MAX, to_island = SIZE_MAX;
                            for (size_t s = 0; s < support_layer.lslices().size(); ++s) {
                                if (support_layer.lslices()[s].contains(path.front() - support_shift)) from_island = s;
                                if (support_layer.lslices()[s].contains(path.back() - support_shift)) to_island = s;
                            }
                            if (!touches && !wiped && from_island != SIZE_MAX && to_island != SIZE_MAX &&
                                from_island != to_island && Polyline(path).length() > scale_i(print.config().wipe_min.get_at(tool.extruder_id).value))
                                ++result.support_void_without_wipe;
                        }
                        result.support_travels += result.travels.size() - before;
                        for (size_t r = before; r < result.travels.size(); ++r) {
                            const std::vector<coord_t> &route = result.travels[r];
                            const Point from(route[0], route[1]);
                            const Point to(route[route.size() - 3], route[route.size() - 2]);
                            bool crosses = false;
                            for (const Printing::PrintingObjectInstance &instance : print.printing_plan()->object_instances) {
                                if (instance.object == nullptr) continue;
                                const Layer *below = nullptr;
                                for (const Layer &candidate : instance.object->layers())
                                    if (candidate.scaled_print_z() <= layer.print_z + SCALED_EPSILON) below = &candidate;
                                if (below == nullptr) continue;
                                const Point shift = instance.object->instances()[instance.object_instance_idx].shift;
                                for (ExPolygon area : below->lslices()) {
                                    area.translate(shift);
                                    if (!area.contains(from) && !area.contains(to) &&
                                        !intersection_pl(Polylines{Polyline(Points{from, to})}, ExPolygons{area}).empty())
                                        crosses = true;
                                }
                            }
                            if (crosses) result.support_crossings.push_back(route);
                        }
                    } else
                        previous_support = {};
                }
    REQUIRE(result.travels.size() > 10);
    REQUIRE(result.travels.size() < 10000);
    const std::string gcode = Test::GCode::read_text_file(exported);
    REQUIRE_FALSE(gcode.empty());
    CHECK(Test::GCode::count_occurrences(gcode, "\nG1 ") > result.travels.size());
    std::vector<Point> emitted_travels;
    GCodeReader reader;
    reader.apply_config(config);
    // The parser tracks absolute E and G92 resets; only material deposited during XY moves contributes.
    reader.parse_buffer(gcode, [&result, &emitted_travels](GCodeReader &state, const GCodeReader::GCodeLine &line) {
        if (line.extruding(state) && line.dist_XY(state) > 0.)
            result.volume += line.dist_E(state) * PI * 1.75 * 1.75 / 4.;
        if ((line.cmd_is("G0") || line.cmd_is("G1")) && !line.has_e() && (line.has_x() || line.has_y()))
            emitted_travels.push_back(line.new_XY_scaled(state));
    });
    CHECK(result.volume > 0.);
    // Finding an intermediate plan XY in the published moves checks that a detour reaches text output.
    bool emitted_detour = false;
    for (size_t travel_idx = 0; travel_idx < result.travels.size(); ++travel_idx) {
        const std::vector<coord_t> &route = result.travels[travel_idx];
        bool route_emitted = false;
        // Both islands are centred at (100,100); a 6 mm box around the centre identifies endpoints on the central island.
        const bool source_central = route[0] > scale_i(97.) && route[0] < scale_i(103.) &&
            route[1] > scale_i(97.) && route[1] < scale_i(103.);
        const size_t last = route.size() - 3;
        const bool target_central = route[last] > scale_i(97.) && route[last] < scale_i(103.) &&
            route[last + 1] > scale_i(97.) && route[last + 1] < scale_i(103.);
        if (source_central != target_central)
            result.inter_island.push_back(route);
        for (size_t idx = 3; idx + 3 < route.size(); idx += 3)
            for (const Point &point : emitted_travels)
                if (std::abs(point.x() - route[idx]) <= scale_i(0.001) &&
                    std::abs(point.y() - route[idx + 1]) <= scale_i(0.001))
                    emitted_detour = route_emitted = true;
        if (route_emitted) result.published_detours.push_back(route);
        if (scene == Scene::Dumbbell && config.opt_bool("avoid_crossing_perimeters") && result.travel_has_source[travel_idx]) {
            const Layer &layer = print.object(0).layers().back();
            const Point shift = print.object(0).instances().front().shift;
            const Point minimum = get_extents(layer.lslices()).min + shift;
            const Point from(route[0], route[1]), to(route[last], route[last + 1]);
            const coord_t left = minimum.x() + scale_i(10.), right = minimum.x() + scale_i(15.);
            if ((from.x() < left && to.x() > right) || (to.x() < left && from.x() > right)) {
                ExPolygons material = layer.lslices();
                for (ExPolygon &area : material) area.translate(shift);
                const ExPolygons bridge{ExPolygon(Polygon(Points{
                    minimum + Point(scale_i(10.), scale_i(4.8)), minimum + Point(scale_i(15.), scale_i(4.8)),
                    minimum + Point(scale_i(15.), scale_i(5.2)), minimum + Point(scale_i(10.), scale_i(5.2))}))};
                bool through = false;
                for (size_t p = 3; p < route.size(); p += 3) {
                    const Polyline segment(Points{Point(route[p - 3], route[p - 2]), Point(route[p], route[p + 1])});
                    CHECK(diff_pl(Polylines{segment}, material).empty());
                    through |= !intersection_pl(Polylines{segment}, bridge).empty();
                }
                CHECK(through);
                result.bridge_travels.push_back(route);
                bool published = false;
                for (size_t first = 0; first + route.size() / 3 - 1 <= emitted_travels.size(); ++first) {
                    bool matches = true;
                    for (size_t p = 3; p < route.size(); p += 3) {
                        const Point &point = emitted_travels[first + p / 3 - 1];
                        if (std::abs(point.x() - route[p]) > scale_i(0.001) ||
                            std::abs(point.y() - route[p + 1]) > scale_i(0.001)) { matches = false; break; }
                    }
                    if (matches) { published = true; break; }
                }
                if (published && route.size() > 6) ++result.published_bridges;
            }
        }
    }
    if (result.deviated > 0)
        CHECK(emitted_detour);
    REQUIRE_FALSE(Orchestrator::instance().is_plugin_cancelled());
    Test::GCode::remove_output_pair(path);
    Test::GCode::remove_output_pair(exported);
    if (scene == Scene::Ring && config.opt_int("perimeters") == 1 &&
        result.provider == "layer_extrusion_edit.travel.advanced")
        check_one_perimeter_routes(print, result, config, scene);
    return result;
}

void check_one_perimeter_routes(Print &print, const Result &result, const DynamicPrintConfig &config, const Scene scene)
{
    REQUIRE(scene == Scene::Ring);
    REQUIRE(config.opt_int("perimeters") == 1);
    REQUIRE_FALSE(config.opt_bool("avoid_crossing_top"));
    REQUIRE_FALSE(config.opt_bool("support_material"));
    namespace travel = slic3r_api::LayerExtrusionEdit::TravelConnection;
    namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;
    namespace ac = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
    namespace work = slic3r_api::PluginCommon::polygon_work;
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    const auto scope_key = slic3r_api::printing_extrusion_scope_property_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    using ScopeEntity = slic3r_api::PrintingEntity<slic3r_api::PrintingExtrusionScopeProperty>;
    PluginStorage storage;
    plugin_run_context context{};
    context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
    size_t checked = 0, fallbacks = 0, domains = 0, provenance_matches = 0, provenance_mismatches = 0,
        chords_outside_material = 0;
    for (uint32_t group = 0; group < plan.groups.size(); ++group)
        for (uint32_t layer = 0; layer < plan.groups[group].layers.size(); ++layer) {
            slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&plan.groups[group].layers[layer]));
            travel::LayerTravelChronology chronology;
            // Published travel endpoints retain the seam-snapped XY positions; copy their real scope owners for eligibility and dates.
            slic3r_api::PrintingEntityPropertyTraversal<slic3r_api::PrintingExtrusionScopeProperty> traversal(
                scope_key, [&chronology, &scope_key, &view](ScopeEntity *previous, ScopeEntity *next) {
                    if (previous == nullptr || next == nullptr) return;
                    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target(next->entity, scope_key);
                    if (!target.travel().valid() || target.travel().segment_count() == 0) return;
                    const c_extrusion_segment first = target.travel().segment(0);
                    const c_extrusion_segment last = target.travel().segment(target.travel().segment_count() - 1);
                    travel::PendingTravel gap;
                    gap.source.position = {first.point_a.x, first.point_a.y, view.print_z(), true};
                    gap.target.position = {last.point_b.x, last.point_b.y, view.print_z(), true};
                    gap.source.region_island = previous->printing_extrusion.region_island();
                    gap.target.region_island = next->printing_extrusion.region_island();
                    gap.source.printing_catalog_id = previous->printing_extrusion.printing_catalog_id();
                    gap.target.printing_catalog_id = next->printing_extrusion.printing_catalog_id();
                    gap.source_extrusion = previous->printing_extrusion;
                    gap.target_extrusion = next->printing_extrusion;
                    gap.extruder_id = next->tool_group.extruder_id();
                    gap.pair_index = chronology.pair_extrusions.size();
                    chronology.pair_extrusions.push_back({gap.target_extrusion, gap.extruder_id});
                    chronology.travels.push_back(gap);
                }, slic3r_api::MatchingEntityDescendants::Skip);
            traversal.process(view);
            ac::CompatibilityPlanner compatibility(print_view, plan_view, &context, group, view.print_z());
            ac::LayerInputCache cache;
            const zones::ZoneContributorInput input{print_view, plan_view, context, cache, chronology.travels};
            zones::DefaultAttractorContributor attractors;
            zones::DefaultForbiddenTopsContributor tops;
            zones::PrintedTopsContributor printed_tops;
            zones::TravelZoneView zone_view({std::cref(attractors), std::cref(tops), std::cref(printed_tops)}, input);
            zones::ZonePlanner planner(compatibility, zone_view, print_view.config(), true, &context, print_view, cache,
                plan_view, group, view.print_z());
            for (size_t t = 0; t < chronology.travels.size(); ++t) {
                const travel::PendingTravel &gap = chronology.travels[t];
                if (gap.source_extrusion.handle() == nullptr) continue;
                const std::vector<c_point> route = planner.path(t, gap);
                REQUIRE(route.size() >= 2);
                // The published route with these endpoints must match the replay, not merely its point count.
                bool matched = false;
                for (const std::vector<coord_t> &published : result.travels) {
                    if (published.size() != 3 * route.size()) continue;
                    bool equal = true;
                    for (size_t point = 0; point < route.size(); ++point)
                        equal = equal && published[3 * point] == route[point].x && published[3 * point + 1] == route[point].y;
                    matched = matched || equal;
                }
                CHECK(matched);
                ++checked;
                const zones::ActiveZones &active = zone_view.query(t, gap);
                CHECK(active.forbidden_indexes.empty());
                const slic3r_api::Layer endpoint_layer = gap.target.region_island.region(0).layer();
                const ac::LayerInput &layer_input = *cache.get(print_view, endpoint_layer, &context).target;
                REQUIRE_FALSE(layer_input.regions.empty());
                CHECK(layer_input.regions.front().avoid_travel_island);
                CHECK(layer_input.regions.front().avoid_travel_island_weight == Approx(1.));
                for (const travel::TravelEndpoint *endpoint : {&gap.source, &gap.target}) {
                    const work::Point point(endpoint->position.x, endpoint->position.y);
                    bool provenance_match = false;
                    for (const size_t index : active.attractor_indexes) {
                        const zones::TravelZone &zone = zone_view.zone(index);
                        if (zone.source_layer == endpoint->region_island.region(0).layer().handle() &&
                            zone.scope.printing_catalog_id == endpoint->printing_catalog_id &&
                            zone_view.zone_area(index).front().contains(point, true)) {
                            provenance_match = true;
                            break;
                        }
                    }
                    CHECK(provenance_match);
                    if (provenance_match) ++provenance_matches;
                    else ++provenance_mismatches;
                }
                const Layer &material_layer = *reinterpret_cast<const Layer *>(endpoint_layer.handle());
                ExPolygons material = material_layer.lslices();
                const Point shift = print.object(0).instances().front().shift;
                for (ExPolygon &area : material) area.translate(shift);
                if (!diff_pl(Polylines{Polyline(Points{Point(route.front().x, route.front().y), Point(route.back().x, route.back().y)})}, material).empty()) {
                    ++chords_outside_material;
                }
            }
            fallbacks += planner.fallbacks();
            domains += planner.domain_routes();
        }
    CHECK(checked > 0);
    CHECK(fallbacks == 0);
    CHECK(domains > 0);
    CHECK(provenance_mismatches == 0);
    CHECK(provenance_matches == 2 * checked);
    CHECK(chords_outside_material > 0);
    std::cout << "one-perimeter replay: checked=" << checked << " fallbacks=" << fallbacks << " domain_routes=" << domains
        << " provenance_matches=" << provenance_matches << " provenance_mismatches=" << provenance_mismatches
        << " chords_outside_material=" << chords_outside_material << '\n';
}

size_t count_wipes(const slic3r_api::ExtrusionEntity &entity, raw_extrusion_role role)
{
    size_t count = 0;
    const slic3r_api::EPropertyAttributes *attributes = entity.get(slic3r_api::EPropertyAttributes::key);
    if (attributes != nullptr)
        role = attributes->extrusion_role();
    if (entity.child_count() == 0 && entity.segment_count() > 0 &&
        RAW_EXTRUSION_ROLE_HAS(role, RAW_EXTRUSION_ROLE_WIPE))
        ++count;
    for (uint32_t i = 0; i < entity.child_count(); ++i)
        count += count_wipes(entity.child(i), role);
    return count;
}

void collect_scope_handles(slic3r_api::MutableExtrusionEntity entity,
                          std::vector<slic3r_api::MutableExtrusionEntity> &scopes)
{
    const auto key = slic3r_api::printing_extrusion_scope_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    if (entity.get(key) != nullptr) {
        scopes.push_back(entity);
        return;
    }
    for (uint32_t i = 0; i < entity.child_count(); ++i)
        collect_scope_handles(entity.child_mutable(i), scopes);
}

void collect_travels(const ExtrusionEntity &entity, const std::set<const extrusion_entity_handle *> &scopes_with_source, Result &result)
{
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        collect_travels(entity.child(idx), scopes_with_source, result);
    const auto key = slic3r_api::printing_extrusion_scope_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    const slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(
        const_cast<ExtrusionEntity *>(&entity)));
    if (view.get(key) == nullptr)
        return;
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope scope(view, key);
    if (!scope.travel().valid())
        return;
    const ExtrusionEntity &travel = *reinterpret_cast<const ExtrusionEntity *>(scope.travel().handle());
    std::vector<coord_t> points;
    append_points(travel, points);
    if (points.size() < 6)
        return;
    result.deviated += points.size() > 6;
    result.travel_has_source.push_back(scopes_with_source.count(view.handle()) != 0);
    result.travels.push_back(std::move(points));
}

void append_points(const ExtrusionEntity &entity, std::vector<coord_t> &points)
{
    if (entity.child_count() != 0) {
        for (size_t idx = 0; idx < entity.child_count(); ++idx)
            append_points(entity.child(idx), points);
    } else if (entity.has_polyline()) {
        const ArcPolyline &line = entity.polyline_ref();
        for (size_t idx = 0; idx < line.size(); ++idx) {
            const Point point = line.get_point(idx);
            if (points.size() >= 3 && points[points.size() - 3] == point.x() &&
                points[points.size() - 2] == point.y() && points.back() == line.z_offset(idx))
                continue;
            points.push_back(point.x());
            points.push_back(point.y());
            points.push_back(line.z_offset(idx));
        }
    }
}
} // namespace
