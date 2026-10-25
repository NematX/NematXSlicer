/* wipe_tower.toolchange_unretract.default rewrites the semantic tool-change unretraction that
   CreateRetraction placed in the before phase of a visit that starts on the wipe tower. These
   tests build compact plans through the real transition, retraction and unretraction-wipe
   plugins: the tower content is a hand-built tower path, the tool history is exercised with
   repeated tool visits, and the speed ceiling is resolved from the print defaults or from a
   sliced object region. One export test checks the Marlin 2 sequence around a tool change. */
#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/scope_exit.hpp>
#include <tbb/global_control.h>

#include "gcode_test_helpers.hpp"
#include "layer_extrusion_edit_transition_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingExtrusionScopeProperty.h"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;
using slic3r_api::EPropertyAttributes;
using slic3r_api::EPropertyExtrusionAxis;
using slic3r_api::EPropertySpeed;
using slic3r_api::MutableExtrusionEntity;
using slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope;

constexpr const char *UNRETRACT_PLUGIN = "wipe_tower.toolchange_unretract.default";

/* Prepare a two-tool print with the retraction settings used by every case. */
void prepare_print(Print &print, Model &model,
                   std::initializer_list<ConfigBase::SetDeserializeItem> overrides)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({
        {"retract_before_travel", "0"}, {"retract_layer_change", "0"},
        {"retract_length", "2,2"}, {"retract_length_toolchange", "10,10"},
        {"retract_restart_extra", "0.1,0.1"}, {"retract_restart_extra_toolchange", "0.2,0.2"},
        {"retract_restart_wipe_toolchange", "20,20"},
        {"retract_restart_toolchange_on_perimeter", "1,1"},
        {"retract_speed", "40,40"}, {"deretract_speed", "30,30"},
        {"filament_diameter", "1.75,1.75"}, {"use_firmware_retraction", "0"},
        {"single_extruder_multi_material", "0"}, {"wipe_tower", "1"}, {"wipe_tower_speed", "80"},
        {"infill_speed", "0"}, {"solid_infill_speed", "0"}, {"support_material_speed", "0"},
        {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"},
        {"layer_height", "0.3"}, {"fill_density", "20%"}});
    config.set_deserialize_strict(overrides);
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model, config);
}

/* Create one auxiliary region island so the speed ceiling can read a real region config. */
const LayerRegionIsland &make_region_island(Print &print, PluginStorage &storage)
{
    const ExPolygons subject{ExPolygon(Polygon({
        Point::new_scale(10., 10.), Point::new_scale(30., 10.),
        Point::new_scale(30., 30.), Point::new_scale(10., 30.)}))};
    const slic3r_api::ExPolygonCollection subject_view(
        reinterpret_cast<const expolygon_collection_handle *>(&subject));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view(
        reinterpret_cast<const object_handle *>(&print.object(0)));
    const slic3r_api::AuxiliaryLayerBuildResult result =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view, object_view, subject_view,
            scale_i(0.2), scale_i(0.2), scale_i(0.1));
    REQUIRE(result.created);
    REQUIRE(result.layer.island_count() == 1);
    layer_region_island_handle *region_island = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(result.layer.island(0).handle()), nullptr, 0, 0);
    REQUIRE(region_island != nullptr);
    return *reinterpret_cast<const LayerRegionIsland *>(region_island);
}

/* Append one straight path leaf to a root. */
void append_path(ExtrusionEntityCollection &root, const double start_x, const double end_x,
                 const ExtrusionRole role, const double mm3_per_mm)
{
    ArcPolyline polyline;
    polyline.append(Point::new_scale(start_x, 0.));
    polyline.append(Point::new_scale(end_x, 0.));
    root.append(std::make_unique<ExtrusionPath>(
        polyline, ExtrusionAttributes(role, ExtrusionFlow(mm3_per_mm, 0.4f, 0.2f)), nullptr, true));
}

/* Append one closed square loop whose perimeter is tower_length_mm. */
void append_loop(ExtrusionEntityCollection &root, const double start_x, const double perimeter_mm,
                 const ExtrusionRole role, const double mm3_per_mm)
{
    const double side = perimeter_mm / 4.;
    ArcPolyline polyline;
    polyline.append(Point::new_scale(start_x, 0.));
    polyline.append(Point::new_scale(start_x + side, 0.));
    polyline.append(Point::new_scale(start_x + side, side));
    polyline.append(Point::new_scale(start_x, side));
    polyline.append(Point::new_scale(start_x, 0.));
    root.append(std::make_unique<ExtrusionPath>(
        polyline, ExtrusionAttributes(role, ExtrusionFlow(mm3_per_mm, 0.4f, 0.2f)), nullptr, true));
}

/* Replace the plan with one layer of tool visits. A visit whose tool is listed in tower_tools
   starts with a tower path, then every visit owns one perimeter path. The tower extrusion uses
   the synthetic instance tower_name, or no catalog entry when it is null. A closed tower leaf
   is a square loop of the requested perimeter instead of an open line. */
void build_plan(Print &print, const std::vector<extruder_index_t> &tools,
                const std::vector<extruder_index_t> &tower_tools, const double tower_length_mm,
                const char *tower_name, const bool closed_tower = false)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.back();
    group.layers.clear();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.print_z = scale_i(0.2);

    uint32_t catalog_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    if (tower_name != nullptr) {
        const coord_t side = scale_i(1.0);
        const Polygon outline({Point(0, 0), Point(side, 0), Point(0, side)});
        catalog_id = plan.add_synthetic_object_instance(tower_name, outline);
        REQUIRE(catalog_id != SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE);
    }

    double x = 0.;
    for (const extruder_index_t tool : tools) {
        layer.tool_groups.emplace_back();
        PrintingToolGroup &visit = layer.tool_groups.back();
        visit.extruder_id = tool;
        const bool tower = std::find(tower_tools.begin(), tower_tools.end(), tool) !=
                           tower_tools.end();
        if (tower) {
            PrintingExtrusion extrusion;
            std::unique_ptr<ExtrusionEntityCollection> root(
                new ExtrusionEntityCollection(false, false));
            if (closed_tower)
                append_loop(*root, x, tower_length_mm, ExtrusionRole::WipeTower, 0.1);
            else
                append_path(*root, x, x + tower_length_mm, ExtrusionRole::WipeTower, 0.1);
            extrusion.root = std::move(root);
            extrusion.sregion_island_role = ExtrusionRole::WipeTower;
            extrusion.printing_catalog_id = catalog_id;
            visit.extrusions.push_back(std::move(extrusion));
            x += tower_length_mm + 1.;
        }
        PrintingExtrusion perimeter;
        perimeter.root = std::make_unique<ExtrusionPath>(
            ArcPolyline(Points{Point::new_scale(x, 0.), Point::new_scale(x + 5., 0.)}),
            ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)),
            nullptr, true);
        perimeter.sregion_island_role = ExtrusionRole::Perimeter;
        visit.extrusions.push_back(std::move(perimeter));
        x += 6.;
    }
}

/* Point the last extrusion of one visit at a real region island, after the retraction run. */
void attach_island(Print &print, const uint32_t visit_index, const LayerRegionIsland &island)
{
    PrintingToolGroup &visit =
        print.mutable_printing_plan().groups.front().layers.front().tool_groups[visit_index];
    REQUIRE_FALSE(visit.extrusions.empty());
    visit.extrusions.back().region_island = &island;
}

/* Return the marked scope root of one visit's first extrusion. */
MutableExtrusionEntity visit_scope(Print &print, const uint32_t visit_index)
{
    PrintingToolGroup &visit =
        print.mutable_printing_plan().groups.front().layers.front().tool_groups[visit_index];
    REQUIRE_FALSE(visit.extrusions.empty());
    REQUIRE(visit.extrusions.front().root != nullptr);
    return find_scope(entity_view(*visit.extrusions.front().root));
}

/* Return the direct ordered events of one phase; a leaf phase is its own single event. */
std::vector<MutableExtrusionEntity> phase_events(MutableExtrusionEntity phase)
{
    std::vector<MutableExtrusionEntity> events;
    if (!phase.valid())
        return events;
    if (phase.child_count() == 0) {
        if (phase.segment_count() > 0 || phase.property_count() > 0)
            events.push_back(phase);
        return events;
    }
    for (uint32_t idx = 0; idx < phase.child_count(); ++idx)
        events.push_back(phase.child_mutable(idx));
    return events;
}

/* Find a point-less E-axis event with the given request, or an invalid view. */
MutableExtrusionEntity find_axis_target(MutableExtrusionEntity entity,
                                        const c_extrusion_axis_operation operation,
                                        const double value, const bool toolchange)
{
    const EPropertyExtrusionAxis *axis = entity.get(EPropertyExtrusionAxis::key);
    if (axis != nullptr && entity.segment_count() == 0 &&
        axis->operation == operation && axis->value == Approx(value) &&
        (axis->toolchange != 0) == toolchange)
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const MutableExtrusionEntity found =
            find_axis_target(entity.child_mutable(child_idx), operation, value, toolchange);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

/* Find a leaf carrying the given role, or an invalid view. */
MutableExtrusionEntity find_role(MutableExtrusionEntity entity, const raw_extrusion_role role)
{
    const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key);
    if (attributes != nullptr && RAW_EXTRUSION_ROLE_HAS(attributes->extrusion_role(), role))
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        const MutableExtrusionEntity found = find_role(entity.child_mutable(child_idx), role);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

/* Collect the segments of the first leaf of a borrowed tree, in print order. */
bool first_leaf_segments(MutableExtrusionEntity entity, std::vector<c_extrusion_segment> &segments)
{
    if (entity.segment_count() > 0) {
        segments = entity.segments();
        return true;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (first_leaf_segments(entity.child_mutable(child_idx), segments))
            return true;
    return false;
}

/* Return the point at the given arc distance along a segment path. */
c_point point_at_distance(const std::vector<c_extrusion_segment> &segments, double distance)
{
    for (const c_extrusion_segment &segment : segments) {
        const double length = c_point_distance_to(segment.point_a, segment.point_b);
        if (length <= 0.)
            continue;
        if (distance <= length) {
            const double ratio = distance / length;
            return c_point{
                coord_t(std::llround(double(segment.point_a.x) +
                                     (double(segment.point_b.x) - double(segment.point_a.x)) * ratio)),
                coord_t(std::llround(double(segment.point_a.y) +
                                     (double(segment.point_b.y) - double(segment.point_a.y)) * ratio))};
        }
        distance -= length;
    }
    return segments.empty() ? c_point{} : segments.back().point_b;
}

/* The expected wiped filament and path length of one nominal rewrite. */
struct ExpectedWipe
{
    double filament = 0.;
    double length = 0.;
    double trip_length = 0.;
};

ExpectedWipe expected_wipe(const double pct, const double retract_length, const double extra,
                           const double filament_diameter, const double mm3_per_mm,
                           const double tower_length_mm)
{
    ExpectedWipe expected;
    expected.filament = pct * retract_length + std::max(0.0, extra);
    const double area = PI * filament_diameter * filament_diameter * 0.25;
    expected.length = expected.filament * area / mm3_per_mm;
    expected.trip_length = 2. * tower_length_mm;
    return expected;
}

/* Run the retraction chain and then the unretraction wipe plugin. */
void run_pipeline(Print &print)
{
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN, UNRETRACT_PLUGIN});
}

/* Invoke one layer-edit plugin directly, without the step progress accounting. */
void run_single_plugin(Print &print, const char *plugin_id)
{
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    host.reset_plugin_cancel();
    plugin_host_context context =
        host.prepare_plugin_host_context(STEP_LAYER_EXTRUSION_EDIT, plugin, &print);
    std::atomic_bool error{false};
    context.execution_error = &error;
    plugin_run_context run =
        host.prepare_plugin_run_context(STEP_LAYER_EXTRUSION_EDIT, plugin, &context);
    run_ctx_layer_extrusion_edition payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
    payload.group = reinterpret_cast<printing_group_handle *>(
        &print.mutable_printing_plan().groups.front());
    payload.layer_group = reinterpret_cast<printing_layer_group_handle *>(
        &print.mutable_printing_plan().groups.front().layers.front());
    run.data = &payload;
    plugin->setup(run, 1);
    plugin->setup_run(run);
    plugin->run(run);
    REQUIRE_FALSE(error.load());
    REQUIRE_FALSE(host.is_plugin_cancelled());
}

/* Return the E word of one G1 movement line, or nullopt. */
std::optional<double> line_e(const std::string &line)
{
    if (line.rfind("G1 ", 0) != 0)
        return std::nullopt;
    const size_t pos = line.find('E');
    if (pos == std::string::npos)
        return std::nullopt;
    try {
        return std::stod(line.substr(pos + 1));
    } catch (...) {
        return std::nullopt;
    }
}

/* Return the lines of one G-code text. */
std::vector<std::string> split_lines(const std::string &text)
{
    std::vector<std::string> lines;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

} // namespace

// Check the registration contract of the new non-exclusive layer-edit plugin.
TEST_CASE("Tool-change unretraction wipe registers after retraction",
          "[plugins][wipe-tower][retraction][registration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(UNRETRACT_PLUGIN);
    REQUIRE(plugin != nullptr);
    CHECK(Orchestrator::instance().is_plugin_active(UNRETRACT_PLUGIN));
    CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(plugin->get_priority() == -70);
    CHECK(plugin->get_exclusive_group().empty());
    REQUIRE(plugin->get_dependencies().size() == 1);
    CHECK(plugin->get_dependencies().front() == TRANSITION_SCOPE_PLUGIN);
    bool declares_percentage = false;
    for (const Plugin::UsedConfigKey &used : plugin->get_used_config_keys())
        declares_percentage = declares_percentage ||
            used.key == "retract_restart_wipe_toolchange";
    CHECK(declares_percentage);
}

// A change back to a tool that already left the tower rewrites its unretraction: the in-place
// part first, then a round trip over the tower start that ends where the tower content begins.
// The first use of that tool is not a return and keeps the semantic unretraction.
TEST_CASE("Tool-change unretraction wipes the tower start after a return change",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    prepare_print(print, model, {});
    build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
    run_pipeline(print);

    // First use of tool 1: not a return, the semantic unretraction stays in place.
    {
        const OrderedExtrusionScope scope(visit_scope(print, 1), scope_property_key());
        const std::vector<MutableExtrusionEntity> events = phase_events(scope.before());
        REQUIRE(events.size() == 1);
        const EPropertyExtrusionAxis *axis = events.front().get(EPropertyExtrusionAxis::key);
        REQUIRE(axis != nullptr);
        CHECK(axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT);
        CHECK(axis->value == Approx(0.0));
        CHECK(axis->toolchange == 1);
        CHECK(events.front().segment_count() == 0);
        CHECK_FALSE(find_role(scope.before(), RAW_EXTRUSION_ROLE_WIPE).valid());
    }

    // Second visit of tool 1 is a return: the partial in place, then the wipe round trip.
    const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
    const std::vector<MutableExtrusionEntity> events = phase_events(scope.before());
    REQUIRE(events.size() == 2);
    const EPropertyExtrusionAxis *partial = events[0].get(EPropertyExtrusionAxis::key);
    REQUIRE(partial != nullptr);
    CHECK(partial->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT);
    CHECK(partial->value == Approx(8.0));
    CHECK(partial->restart_extra == Approx(0.0));
    CHECK(partial->toolchange == 1);
    CHECK(events[0].segment_count() == 0);
    const EPropertyAttributes *wipe_attributes = events[1].get(EPropertyAttributes::key);
    REQUIRE(wipe_attributes != nullptr);
    CHECK(RAW_EXTRUSION_ROLE_IS_WIPE(wipe_attributes->extrusion_role()));
    CHECK(RAW_EXTRUSION_ROLE_IS_UNRETRACT(wipe_attributes->extrusion_role()));
    const EPropertyExtrusionAxis *wipe_axis = events[1].get(EPropertyExtrusionAxis::key);
    REQUIRE(wipe_axis != nullptr);
    CHECK(wipe_axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT);
    CHECK(wipe_axis->value == Approx(0.0));
    CHECK(wipe_axis->restart_extra == Approx(0.2));
    CHECK(wipe_axis->toolchange == 1);
    CHECK_FALSE(find_axis_target(scope.before(), C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0, true)
                    .valid());

    const ExpectedWipe expected = expected_wipe(0.2, 10.0, 0.2, 1.75, 0.1, 5.0);
    const double length = unscaled(events[1].local_length());
    // The path is out and back over the short leaf until exactly the requested length is covered.
    CHECK(length == Approx(expected.length).margin(1e-6));
    const std::vector<c_extrusion_segment> segments = events[1].segments();
    REQUIRE_FALSE(segments.empty());
    CHECK(segments.front().point_a.x == segments.back().point_b.x);
    CHECK(segments.front().point_a.y == segments.back().point_b.y);
    const EPropertySpeed *speed = events[1].get(EPropertySpeed::key);
    REQUIRE(speed != nullptr);
    // infill_speed and support_material_speed are zero, so the cap is wipe_tower_speed.
    CHECK(speed->speed_mm_per_s == Approx(80.f));
}

// A leaf longer than half the requested length is followed out for half the length and back on
// the same path, for an open line and for a closed perimeter loop.
TEST_CASE("Tool-change unretraction wipe turns at half its length",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    const auto check_turn = [](Print &print, const double leaf_length_mm) {
        const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
        MutableExtrusionEntity wipe = find_role(scope.before(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        const ExpectedWipe expected = expected_wipe(0.2, 10.0, 0.2, 1.75, 0.1, leaf_length_mm);
        REQUIRE(expected.length / 2. < leaf_length_mm);
        const std::vector<c_extrusion_segment> segments = wipe.segments();
        REQUIRE_FALSE(segments.empty());
        CHECK(unscaled(wipe.local_length()) == Approx(expected.length).margin(1e-6));
        // The far point is at half the requested length along the followed leaf.
        std::vector<c_extrusion_segment> forward;
        REQUIRE(first_leaf_segments(scope.content(), forward));
        const c_point far = point_at_distance(forward, expected.length / 2.);
        const c_point wipe_far = point_at_distance(segments, expected.length / 2.);
        CHECK(wipe_far.x == far.x);
        CHECK(wipe_far.y == far.y);
        CHECK((wipe_far.x != segments.front().point_a.x ||
               wipe_far.y != segments.front().point_a.y));
        CHECK(segments.front().point_a.x == segments.back().point_b.x);
        CHECK(segments.front().point_a.y == segments.back().point_b.y);
    };

    // An open purge-like line of 100 mm.
    {
        Print print;
        Model model;
        prepare_print(print, model, {});
        build_plan(print, {0, 1, 0, 1}, {1}, 100.0, "Wipe tower");
        run_pipeline(print);
        check_turn(print, 100.0);
    }

    // A closed perimeter loop of 100 mm.
    {
        Print print;
        Model model;
        prepare_print(print, model, {});
        build_plan(print, {0, 1, 0, 1}, {1}, 100.0, "Wipe tower", true);
        run_pipeline(print);
        check_turn(print, 100.0);
    }
}

// Every guard leaves the original semantic unretraction untouched.
TEST_CASE("Tool-change unretraction wipe keeps its no-op cases",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    const auto require_untouched = [](Print &print, const uint32_t visit_index,
                                      const bool toolchange) {
        const OrderedExtrusionScope scope(visit_scope(print, visit_index), scope_property_key());
        CHECK_FALSE(find_role(scope.before(), RAW_EXTRUSION_ROLE_WIPE).valid());
        CHECK(find_axis_target(scope.before(), C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0, toolchange)
                  .valid());
    };

    // A zero percentage.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"retract_restart_wipe_toolchange", "0,0"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        require_untouched(print, 3, true);
    }

    // The first content of the incoming visit is not the tower.
    {
        Print print;
        Model model;
        prepare_print(print, model, {});
        build_plan(print, {0, 1, 0, 1}, {}, 5.0, "Wipe tower");
        run_pipeline(print);
        require_untouched(print, 3, true);
    }

    // Single-extruder multi-material.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"single_extruder_multi_material", "1"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        require_untouched(print, 3, true);
    }

    // Firmware retraction keeps the in-place unretraction for G11.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"use_firmware_retraction", "1"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        require_untouched(print, 3, true);
    }

    // A non-toolchange unretraction at a tool-change boundary.
    {
        Print print;
        Model model;
        prepare_print(print, model, {});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});
        const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
        MutableExtrusionEntity unretract = find_axis_target(
            scope.before(), C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0, true);
        REQUIRE(unretract.valid());
        EPropertyExtrusionAxis *axis = unretract.get_mutable(EPropertyExtrusionAxis::key);
        REQUIRE(axis != nullptr);
        axis->toolchange = 0;
        run_single_plugin(print, UNRETRACT_PLUGIN);
        require_untouched(print, 3, false);
    }
}

// A full percentage leaves no in-place event: the original event itself becomes the wipe leaf.
TEST_CASE("Tool-change unretraction wipe at one hundred percent is only the wipe",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    prepare_print(print, model, {{"retract_restart_wipe_toolchange", "100,100"}});
    build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
    run_pipeline(print);

    const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
    const std::vector<MutableExtrusionEntity> events = phase_events(scope.before());
    REQUIRE(events.size() == 1);
    const EPropertyAttributes *attributes = events.front().get(EPropertyAttributes::key);
    REQUIRE(attributes != nullptr);
    CHECK(RAW_EXTRUSION_ROLE_IS_WIPE(attributes->extrusion_role()));
    CHECK(RAW_EXTRUSION_ROLE_IS_UNRETRACT(attributes->extrusion_role()));
    const EPropertyExtrusionAxis *axis = events.front().get(EPropertyExtrusionAxis::key);
    REQUIRE(axis != nullptr);
    CHECK(axis->restart_extra == Approx(0.2));
    CHECK(axis->toolchange == 1);
    CHECK(events.front().segment_count() > 0);
    const ExpectedWipe expected = expected_wipe(1.0, 10.0, 0.2, 1.75, 0.1, 5.0);
    CHECK(unscaled(events.front().local_length()) >= expected.length - 1e-6);
}

// A first leaf shorter than half the requested length is traversed by several round trips, and
// the path still ends on the tower content's first point.
TEST_CASE("Tool-change unretraction wipe repeats a short tower leaf",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    prepare_print(print, model, {{"retract_restart_wipe_toolchange", "100,100"}});
    build_plan(print, {0, 1, 0, 1}, {1}, 0.5, "Wipe tower");
    run_pipeline(print);

    const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
    const std::vector<MutableExtrusionEntity> events = phase_events(scope.before());
    REQUIRE(events.size() == 1);
    const ExpectedWipe expected = expected_wipe(1.0, 10.0, 0.2, 1.75, 0.1, 0.5);
    // 245 whole out-and-back trips plus a partial one still total the requested length.
    CHECK(expected.length / (2. * 0.5) > 100.);
    CHECK(unscaled(events.front().local_length()) == Approx(expected.length).margin(1e-6));
    const std::vector<c_extrusion_segment> segments = events.front().segments();
    REQUIRE(segments.size() > 100);
    CHECK(segments.front().point_a.x == segments.back().point_b.x);
    CHECK(segments.front().point_a.y == segments.back().point_b.y);
}

// The XY wipe speed is the deretraction speed scaled by the path length, capped by the fastest
// sparse-infill or support speed; a zero deretraction speed uses the retraction speed.
TEST_CASE("Tool-change unretraction wipe resolves its speed ceiling",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    const auto wipe_speed = [](Print &print) {
        const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
        MutableExtrusionEntity wipe = find_role(scope.before(), RAW_EXTRUSION_ROLE_WIPE);
        REQUIRE(wipe.valid());
        const EPropertySpeed *speed = wipe.get(EPropertySpeed::key);
        REQUIRE(speed != nullptr);
        return double(speed->speed_mm_per_s);
    };

    // The ceiling is reached: 10 mm/s is slower than the necessary deretraction speed.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"infill_speed", "10"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        CHECK(wipe_speed(print) == Approx(10.0));
    }

    // The ceiling is not reached: the deretraction speed gives 24.05 mm/s.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"infill_speed", "1000"},
                                     {"deretract_speed", "0,0"}, {"retract_speed", "1,1"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        const ExpectedWipe expected = expected_wipe(0.2, 10.0, 0.2, 1.75, 0.1, 5.0);
        CHECK(wipe_speed(print) == Approx(expected.length * 1.0 / expected.filament));
    }

    // A zero deretraction speed falls back to the retraction speed.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"infill_speed", "1000"},
                                     {"deretract_speed", "0,0"}, {"retract_speed", "40,40"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        const ExpectedWipe expected = expected_wipe(0.2, 10.0, 0.2, 1.75, 0.1, 5.0);
        CHECK(wipe_speed(print) == Approx(expected.length * 40.0 / expected.filament));
    }

    // A missing infill speed falls back to wipe_tower_speed.
    {
        Print print;
        Model model;
        prepare_print(print, model, {{"infill_speed", "0"}});
        build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower");
        run_pipeline(print);
        CHECK(wipe_speed(print) == Approx(80.0));
    }
}

// An instance tower resolves the ceiling from the object printed in the same visit.
TEST_CASE("Tool-change unretraction wipe resolves an instance tower ceiling",
          "[plugins][wipe-tower][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    PluginStorage storage;
    prepare_print(print, model, {{"infill_speed", "10"}});
    const LayerRegionIsland &island = make_region_island(print, storage);
    build_plan(print, {0, 1, 0, 1}, {1}, 5.0, "Wipe tower cube #1");
    run_layer_plugins(print, {TRANSITION_SCOPE_PLUGIN, RETRACTION_PLUGIN});
    attach_island(print, 3, island);
    run_single_plugin(print, UNRETRACT_PLUGIN);

    const OrderedExtrusionScope scope(visit_scope(print, 3), scope_property_key());
    MutableExtrusionEntity wipe = find_role(scope.before(), RAW_EXTRUSION_ROLE_WIPE);
    REQUIRE(wipe.valid());
    const EPropertySpeed *speed = wipe.get(EPropertySpeed::key);
    REQUIRE(speed != nullptr);
    CHECK(double(speed->speed_mm_per_s) == Approx(10.0));
}

// Export two cubes, each printed by its own tool: between the T of a return change and the tower
// content, the filament is restored in place and then wiped back and forth to the first tower
// point. Firmware retraction keeps its in-place G11 instead.
TEST_CASE("Tool-change unretraction exports the partial then the tower wipe",
          "[plugins][wipe-tower][retraction][export]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    // The production runtime activates the travel provider and the test runtime does not.
    // Activate it here so the export exercises the real travel phase, and restore the previous
    // activation state for the other tests.
    Orchestrator &host = Orchestrator::instance();
    Plugin *travel = host.get_plugin("layer_extrusion_edit.travel.default");
    REQUIRE(travel != nullptr);
    const bool travel_was_active = host.active_plugins().count(travel) != 0;
    BOOST_SCOPE_EXIT((&host)(travel)(travel_was_active)) {
        host.set_plugin_active(travel, travel_was_active);
    } BOOST_SCOPE_EXIT_END
    REQUIRE(host.set_plugin_active(travel, true));
    const bool relative_e = GENERATE(false, true);
    const bool firmware = GENERATE(false, true);
    CAPTURE(relative_e, firmware);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({
        {"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"},
        {"wipe_tower_brim_width", "0"}, {"wiping_volumes_matrix", "0,140,140,0"},
        {"filament_minimal_purge_on_wipe_tower", "15,15"}, {"filament_diameter", "1.75,1.75"},
        {"extrusion_multiplier", "1,1"}, {"perimeter_extruder", "1"}, {"infill_extruder", "1"},
        {"solid_infill_extruder", "1"}, {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"},
        {"layer_height", "0.3"}, {"fill_density", "20%"},
        {"retract_before_travel", "0"}, {"retract_layer_change", "0"},
        {"retract_length", "2,2"}, {"retract_length_toolchange", "10,10"},
        {"retract_restart_extra", "0.1,0.1"}, {"retract_restart_extra_toolchange", "0.2,0.2"},
        {"retract_restart_wipe_toolchange", "20,20"},
        {"retract_speed", "40,40"}, {"deretract_speed", "30,30"},
        // Only the rewritten tool-change unretraction may emit a wipe block.
        {"wipe", "0,0"},
        {"use_firmware_retraction", firmware ? "1" : "0"},
        {"use_relative_e_distances", relative_e ? "1" : "0"},
        {"single_extruder_multi_material", "0"}});
    config.option<ConfigOptionStrings>("start_filament_gcode")
        ->set(std::vector<std::string>{";START_FILAMENT", ";START_FILAMENT"});

    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20},
                     print, model, config);
    for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
        model.objects()[1].config.set_deserialize_strict(key, "2");
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    const boost::filesystem::path checkout =
        boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::string path = (checkout / "build" / "temp" /
        (std::string("toolchange_unretract_") + (firmware ? "firmware" : "explicit") +
         (relative_e ? "_relative" : "_absolute") + ".gcode")).string();
    GCodeProcessorResult result;
    REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

    const std::vector<std::string> lines = split_lines(Test::GCode::read_text_file(path));
    const size_t not_found = lines.size();
    const auto find_line = [&lines](const size_t from, const char *text) {
        for (size_t idx = from; idx < lines.size(); ++idx)
            if (lines[idx].find(text) != std::string::npos)
                return idx;
        return lines.size();
    };
    if (firmware) {
        // The semantic unretraction stays in place and the firmware writes G11.
        CHECK(find_line(0, "G11") != not_found);
        CHECK(find_line(0, ";WIPE_START") == not_found);
        return;
    }

    const size_t wipe_start = find_line(0, ";WIPE_START");
    REQUIRE(wipe_start != not_found);
    const size_t wipe_end = find_line(wipe_start + 1, ";WIPE_END");
    REQUIRE(wipe_end != not_found);
    // The wipe keeps the capped speed instead of a DefaultSpeed value.
    CHECK(lines[wipe_start + 1].find("F4800") != std::string::npos);

    // The in-place part is the only E-only move before the wipe, and the start script is before it.
    size_t partial = not_found;
    for (size_t idx = wipe_start; idx > 0; --idx) {
        const std::optional<double> e = line_e(lines[idx - 1]);
        if (e && lines[idx - 1].find('X') == std::string::npos &&
            lines[idx - 1].find('Y') == std::string::npos) {
            partial = idx - 1;
            break;
        }
    }
    REQUIRE(partial != not_found);
    CHECK(lines[partial].find("F1800") != std::string::npos);

    // The travel provider moves to the first tower point before the start script and the
    // in-place part, and no other unretraction runs between the tool change and the content.
    const auto line_axis = [&lines](const size_t idx, const char axis) {
        const size_t pos = lines[idx].find(axis);
        if (pos == std::string::npos)
            return std::optional<double>();
        try {
            return std::optional<double>(std::stod(lines[idx].substr(pos + 1)));
        } catch (...) {
            return std::optional<double>();
        }
    };
    size_t travel_line = not_found;
    for (size_t idx = partial; idx > 0; --idx)
        if (lines[idx - 1].rfind("G0 ", 0) == 0) {
            travel_line = idx - 1;
            break;
        }
    REQUIRE(travel_line != not_found);
    REQUIRE(line_axis(travel_line, 'X').has_value());
    REQUIRE(line_axis(travel_line, 'Y').has_value());
    CHECK(*line_axis(travel_line, 'X') == Approx(40.0));
    CHECK(*line_axis(travel_line, 'Y') == Approx(60.0));
    size_t change_line = not_found;
    for (size_t idx = travel_line; idx > 0; --idx)
        if (lines[idx - 1].rfind("T0", 0) == 0 || lines[idx - 1].rfind("T1", 0) == 0) {
            change_line = idx - 1;
            break;
        }
    REQUIRE(change_line != not_found);
    const size_t script_line = find_line(travel_line, ";START_FILAMENT");
    CHECK(change_line < travel_line);
    CHECK(travel_line < script_line);
    CHECK(script_line < partial);
    for (size_t idx = travel_line + 1; idx < partial; ++idx)
        CHECK(!(line_e(lines[idx]).has_value() &&
                lines[idx].find('X') == std::string::npos &&
                lines[idx].find('Y') == std::string::npos));

    // E restored in place and during the wipe; relative mode already reports the deltas.
    double last_e = 0.;
    for (size_t idx = 0; idx < partial; ++idx)
        if (lines[idx].find("G92 E0") != std::string::npos)
            last_e = 0.;
        else if (const std::optional<double> e = line_e(lines[idx]))
            last_e = *e;
    const std::optional<double> partial_e = line_e(lines[partial]);
    REQUIRE(partial_e.has_value());
    CHECK((relative_e ? *partial_e : *partial_e - last_e) == Approx(8.0).margin(0.01));
    last_e = *partial_e;

    double wiped_e = 0.;
    size_t wipe_moves = 0;
    for (size_t idx = wipe_start + 1; idx < wipe_end; ++idx) {
        const std::optional<double> e = line_e(lines[idx]);
        if (!e)
            continue;
        wiped_e += relative_e ? *e : *e - last_e;
        last_e = *e;
        ++wipe_moves;
    }
    CHECK(wipe_moves > 0);
    CHECK(wiped_e == Approx(2.2).margin(0.05));

    // The wipe returns to the first tower point, so no travel separates it from the tower
    // content: the first move after the wipe block already carries E.
    size_t first_content = not_found;
    for (size_t idx = wipe_end + 1; idx < lines.size(); ++idx)
        if (line_e(lines[idx])) {
            first_content = idx;
            break;
        }
    REQUIRE(first_content != not_found);
    for (size_t idx = wipe_end + 1; idx < first_content; ++idx) {
        CHECK(lines[idx].rfind("G0 ", 0) != 0);
        CHECK(!(lines[idx].rfind("G1 ", 0) == 0 && !line_e(lines[idx]).has_value()));
    }
}
