#include <catch2/catch.hpp>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "gcode_test_helpers.hpp"
#include "gcode_export_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"

/*
This end-to-end gate checks that compatible and zone-based travel providers' settings
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

For the avoid-crossing baseline, the gate compares disabled avoidance with the
straight-travel provider; the advanced baseline instead varies the selector toward
the avoid-crossing provider. The advanced matrix runs with both compatibility
and zones selected. For every baseline, the gate checks
that the first-layer exemption and a 5 mm detour limit reduce deviation counts,
and checks that island weights change compatibility routes while keeping their
endpoints; the zones planner's inter-island routes are independent of those weights.
It compares deposited volume except for settings that change extrusion geometry.
For each case with deviations, at least one intermediate plan XY must occur in
the published G0/G1 XY moves without an E word within 0.001 mm. This does not match every
route to G-code or compare with the core router: it checks one scene's production
integration, not general routing correctness.

The activation guard saves and restores the active plugin set, including the
exporter, after the case-local Prints are destroyed. Output removal is attempted
only on the normal return path, and removal errors are ignored.
Contributor activation cases keep the zones planner selected and round-trip a
reference with both members and each member disabled independently.

    TEST_CASE: validate setting coverage and compare the resulting cases
    |-- TravelActivation: enable travel providers and export, then restore the saved set
    `-- run_case: process the ring and central island, publish text, copy observations
        |-- collect_travels: visit ordered scopes and copy their reserved travel phases
        |   `-- append_points: join geometric leaves without consecutive duplicate vertices
        `-- GCodeReader::parse_buffer: measure deposited volume and collect emitted travel XY
*/
namespace {
using namespace Slic3r;
struct Result {
    // Planner selected by the host before processing; empty when no member is active.
    std::string selected_planner;
    // Each route contains consecutive scaled X, Y, Z-offset triples in execution order.
    std::vector<std::vector<coord_t>> travels;
    // Routes whose endpoints lie on different islands.
    std::vector<std::vector<coord_t>> inter_island;
    // Number of copied routes with more than two vertices, regardless of collinearity.
    size_t deviated = 0;
    // Extruded volume read back from the published G-code, in mm³.
    double volume = 0.;
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
        success &= host.set_plugin_active("layer_extrusion_edit.travel.avoid_crossing_perimeters", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced.planner.compatibility", true);
        success &= host.set_plugin_active("layer_extrusion_edit.travel.advanced.planner.zones", true);
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
// Slice and process the two-island mesh for config, publish text G-code under file names built from name,
// and return the copied plan routes with the volume read from the published file.
// Pipeline or export errors remain failures; remove both output pairs only on the normal return path, ignoring removal errors.
Result run_case(const DynamicPrintConfig &config, const std::string &name);
// Copy reserved travel phases without changing the plan; skip routes below two vertices and count those above two.
void collect_travels(const ExtrusionEntity &entity, Result &result);
// Append geometric leaves in child order, skipping consecutive duplicate XY and Z-offset triples.
void append_points(const ExtrusionEntity &entity, std::vector<coord_t> &points);

// Compare explicit planner selection and activation fallbacks through STL, slicing and exported G-code.
TEST_CASE("Advanced travel honours planner selection and inactive fallbacks", "[plugins][plugin-e2e][travel][travel-zones]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    for (const char *id : {"layer_extrusion_edit.travel.advanced.planner.compatibility",
                           "layer_extrusion_edit.travel.advanced.planner.zones"}) {
        const Plugin *planner = host.get_plugin(id);
        REQUIRE(planner != nullptr);
        CHECK(planner->get_used_config_keys().empty());
        CHECK(planner->get_defined_config_keys().empty());
    }
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        host, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    const std::string planner_selector = Steps::plugin_exclusive_group_option_key(
        host, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel.advanced.planner");
    const char *compatibility = "layer_extrusion_edit.travel.advanced.planner.compatibility";
    const char *zones = "layer_extrusion_edit.travel.advanced.planner.zones";
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(selector, "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"fill_density", "20%"}, {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"}, {"ironing", "0"},
        {"binary_gcode", "0"}, {"use_relative_e_distances", "0"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result baseline = run_case(config, "planner-default");
    CHECK(baseline.selected_planner == compatibility);
    REQUIRE(baseline.deviated > 0);
    bool expect_zones = false;
    std::string expected = compatibility;
    SECTION("Explicit compatibility") { config.set_deserialize_strict(planner_selector, compatibility); }
    SECTION("Explicit zones") {
        config.set_deserialize_strict(planner_selector, zones);
        expected = zones;
        expect_zones = true;
    }
    SECTION("Both inactive") {
        REQUIRE(host.set_plugin_active(compatibility, false));
        REQUIRE(host.set_plugin_active(zones, false));
        expected.clear();
    }
    SECTION("Only compatibility active") { REQUIRE(host.set_plugin_active(zones, false)); }
    SECTION("Only zones active") {
        REQUIRE(host.set_plugin_active(compatibility, false));
        expected = zones;
        expect_zones = true;
    }
    const Result result = run_case(config, "planner-selection");
    CHECK(result.selected_planner == expected);
    CHECK(result.volume == Approx(baseline.volume).margin(0.01));
    CHECK(result.deviated > 0);
    if (expect_zones)
        CHECK(result.travels != baseline.travels);
    else
        CHECK(result.travels == baseline.travels);
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
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel.advanced.planner"), "layer_extrusion_edit.travel.advanced.planner.zones");
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
        CHECK(result.selected_planner == "layer_extrusion_edit.travel.advanced.planner.zones");
        CHECK(result.volume == Approx(reference.volume).margin(0.01));
        CHECK_FALSE(result.travels.empty());
        REQUIRE(host.set_plugin_active(id, true));
    }
}

// Validate setting coverage and compare an enabled baseline with variations; ironing is coupled with top avoidance.
TEST_CASE("Avoid-crossing settings reach production G-code", "[plugins][plugin-e2e][avoid-crossing-perimeters]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    TravelActivation active(host);
    REQUIRE(active.success);
    const std::string mode = GENERATE(std::string("avoid-crossing"), std::string("compatibility"), std::string("zones"));
    const bool use_zones = mode == "zones";
    const char *provider = mode == "avoid-crossing" ? "layer_extrusion_edit.travel.avoid_crossing_perimeters" :
                                                    "layer_extrusion_edit.travel.advanced";
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        host, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    const std::map<std::string, std::vector<std::string>> domains{
        {"avoid_crossing_perimeters", {"0"}},
        {"avoid_crossing_not_first_layer", {"1"}},
        {"avoid_crossing_perimeters_max_detour", {"5", "200%"}},
        {"avoid_crossing_top", {"1"}},
        {"ironing", {"1"}},
        {"perimeters", {"1", "3"}},
        {"avoid_travel_island", {"0"}},
        {"avoid_travel_island_weight", {"0", "2"}},
        {"nozzle_diameter", {"0.6"}},
        {selector, {std::string(provider) == "layer_extrusion_edit.travel.advanced" ?
            "layer_extrusion_edit.travel.avoid_crossing_perimeters" : "layer_extrusion_edit.travel.default"}}
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
    if (use_zones)
        baseline.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
            "layer_extrusion_edit.travel.advanced.planner"), "layer_extrusion_edit.travel.advanced.planner.zones");
    baseline.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "0"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"avoid_travel_island", "1"}, {"avoid_travel_island_weight", "1"},
        {"nozzle_diameter", "0.4"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"perimeters", "2"}, {"fill_density", "20%"}, {"top_solid_layers", "1"},
        {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe", "0"},
        {"ironing", "0"}, {"use_relative_e_distances", "0"}, {"binary_gcode", "0"}, {"filament_diameter", "1.75"},
        {"gcode_exporter_plugin", k_default_gcode_exporter_id}});
    const Result enabled = run_case(baseline, "baseline");
    if (std::string(provider) == "layer_extrusion_edit.travel.advanced")
        REQUIRE(enabled.selected_planner == (use_zones ? "layer_extrusion_edit.travel.advanced.planner.zones" :
                                                        "layer_extrusion_edit.travel.advanced.planner.compatibility"));
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
            if ((domain.first == "avoid_crossing_perimeters_max_detour" && value == "5") ||
                (domain.first == "avoid_crossing_not_first_layer" && value == "1"))
                CHECK(result.deviated < enabled.deviated);
            std::cout << domain_name << "=" << value << ": travels=" << result.travels.size()
                      << " deviated=" << result.deviated << " volume=" << result.volume << '\n';
        }
    if (std::string(provider) != "layer_extrusion_edit.travel.advanced")
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
    if (use_zones)
        CHECK(weight_zero.inter_island == weight_two.inter_island);
    else
        CHECK(weight_zero.inter_island != weight_two.inter_island);
}

Result run_case(const DynamicPrintConfig &config, const std::string &name)
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
    mesh = Test::PluginE2E::stl_fixture(mesh);
    Model model;
    Print print;
    Test::init_print({mesh}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    Plugin *selected = Steps::selected_active_plugin_from_group(orchestrator,
        STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel", &print.config());
    REQUIRE(selected != nullptr);
    REQUIRE(selected->get_id() == config.opt_serialize(selector));
    Plugin *selected_planner = Steps::selected_active_plugin_from_group(orchestrator,
        STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel.advanced.planner", &print.config());
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
    if (selected_planner != nullptr) result.selected_planner = selected_planner->get_id();
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &tool : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : tool.extrusions)
                    collect_travels(*extrusion.root, result);
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
    for (const std::vector<coord_t> &route : result.travels) {
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
                    emitted_detour = true;
    }
    if (result.deviated > 0)
        CHECK(emitted_detour);
    REQUIRE_FALSE(Orchestrator::instance().is_plugin_cancelled());
    Test::GCode::remove_output_pair(path);
    Test::GCode::remove_output_pair(exported);
    return result;
}

void collect_travels(const ExtrusionEntity &entity, Result &result)
{
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        collect_travels(entity.child(idx), result);
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
