#include <catch2/catch.hpp>
#include "libslic3r/Steps/StepPipeline.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "avoid_crossing_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/GCode/AvoidCrossingPerimeters.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/TravelConnectionHelpers.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossing/CompatibilityPlanner.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/TravelZones.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AdvancedTravel/DefaultZoneContributors.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingExtrusionScopeProperty.h"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepLayerExtrusionEdition.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
Straight-travel plugin tests
============================

The fixtures build already ordered PrintingPlan trees directly. The entry-state
producer first publishes each layer boundary; the travel producer then runs in
the real parallel host and must connect the leaves without consulting another
worker's layer or changing their original printable properties.
Fixture trees acquire canonical storage on entry to the plan, before validation.
Direct connector checks stop after scope and entry publication, then inspect
prepare's ordered gaps and seam edits before apply writes paths and flags.
All borrowed views remain within the lifetime of their unchanged plan structure.
Eligibility checks slice a cube and attach controlled paths to its real region
island, then evaluate prepared gaps with the compatibility planner's predicate.
NoTravel checks attach controlled gaps in platter coordinates without regional
context and run them through straight travel and through advanced travel, whose
gaps outside perimeter avoidance take the same best-effort NoTravel path.
The ring scene routes a gap across a hole with advanced travel and its island
attractors, and checks that the published route stays out of the hole.
The controlled three-island scene toggles contributor activation with either
advanced or straight travel selected. Advanced travel may detour its inter-island
gap because of attractors or forbidden tops; layer entry stays straight in every case.
The same-attractor case keeps its direct travel; only inter-component gaps or a
forbidden-top contributor can add a detour.
The dated-top scene crosses a third island between two distinct attractors
before and after that island's last arrival, in two shifted instances. Every
segment of the later detour is checked against the third island's core top.
The narrow-bridge scene checks its sliced and printed connection separately
from expected routing. A chain of deposited perimeter paths must span the neck,
joining only at endpoints within the scaled epsilon. A controlled gap has a chord outside the material;
active attractors and every routed segment are checked in platter coordinates.

    controlled scene: slice region masks, attach ordered paths, select active members
        `-- StepLayerExtrusionEdition::run_step: insert travels for point-count checks
    narrow_bridge_scene: verify a continuous printed chain, prepare eligible gap, query default zones
        `-- StepLayerExtrusionEdition::run_step: route the controlled gap with advanced travel
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;

constexpr const char *ENTRY_STATE_PLUGIN = "layer_extrusion_edit.entry_state.default";
constexpr const char *TRANSITION_SCOPE_PLUGIN =
    "layer_extrusion_edit.transition_scope.default";
constexpr const char *TRAVEL_PLUGIN = "layer_extrusion_edit.travel.default";
constexpr const char *ADVANCED_TRAVEL_PLUGIN = "layer_extrusion_edit.travel.advanced";

/* Publish all connector summaries before direct preparation; views borrow the unchanged plan and invalid setup throws. */
void setup_connector(Print &print,
    slic3r_api::LayerExtrusionEdit::TravelConnection::ScopeTravelConnector &connector,
    slic3r_api::PluginProgress &progress);

struct NarrowBridgeResult
{
    ExPolygons slices;
    ExPolygons bridge;
    Point source;
    Point target;
    slic3r_api::PluginCommon::polygon_work::ExPolygons attractors;
    Polyline route;
};

// Slice a one-line bridge, require an endpoint-connected deposited chain and eligible gap, then copy active zones and the production route.
NarrowBridgeResult narrow_bridge_scene();

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

std::unique_ptr<ExtrusionPath> make_path(const Point &start,
                                        const Point &end,
                                        coord_t start_z_offset = 0,
                                        coord_t end_z_offset = 0,
                                        ExtrusionRole role = ExtrusionRole::Perimeter)
{
    ArcPolyline polyline;
    polyline.append(start);
    polyline.append(end);
    polyline.set_z_offset(0, start_z_offset);
    polyline.set_z_offset(1, end_z_offset);
    return std::make_unique<ExtrusionPath>(
        polyline, ExtrusionAttributes(role, ExtrusionFlow(
            role == ExtrusionRole::Travel ? 0.0 : 0.2, 0.4f, 0.2f)), nullptr, true);
}

std::unique_ptr<ExtrusionPath> make_closed_arc_path(const Point &seam)
{
    ArcPolyline polyline;
    polyline.append(seam);
    polyline.append(Geometry::ArcWelder::Segment(
        Point(seam.x() + scale_i(1.), seam.y() + scale_i(1.)),
        float(scale_i(1.)),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.append(seam);
    polyline.set_z_offset(0, scale_i(0.03));
    polyline.set_z_offset(1, scale_i(0.04));
    polyline.set_z_offset(2, scale_i(0.03));
    return std::make_unique<ExtrusionPath>(
        polyline, ExtrusionAttributes(
            ExtrusionRole::Perimeter, ExtrusionFlow(0.2, 0.4f, 0.2f)), nullptr, true);
}

// Acquire canonical fixture content before publishing it to the plan.
void append_extrusion(PrintingToolGroup &tool,
                      std::unique_ptr<ExtrusionEntity> root,
                      const LayerRegionIsland *region_island = nullptr,
                      uint32_t printing_catalog_id = PRINTING_OBJECT_INSTANCE_ID_NONE)
{
    PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.printing_catalog_id = printing_catalog_id;
    extrusion.root = CanonicalExtrusion::acquire(*root);
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    tool.extrusions.push_back(std::move(extrusion));
}

void collect_geometric_leaves(const ExtrusionEntity &entity,
                              std::vector<const ExtrusionEntity *> &leaves)
{
    if (entity.child_count() > 0) {
        for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
            collect_geometric_leaves(entity.child(child_idx), leaves);
        return;
    }
    if (entity.has_polyline())
        leaves.push_back(&entity);
}

std::vector<const ExtrusionEntity *> plan_leaves(const Print &print)
{
    std::vector<const ExtrusionEntity *> leaves;
    REQUIRE(print.printing_plan() != nullptr);
    for (const PrintingGroup &group : print.printing_plan()->groups)
        for (const PrintingLayerGroup &layer : group.layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions)
                    collect_geometric_leaves(*extrusion.root, leaves);
    return leaves;
}

size_t travel_count(const Print &print)
{
    size_t count = 0;
    for (const ExtrusionEntity *leaf : plan_leaves(print)) {
        const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
        if (attributes != nullptr && attributes->extrusion_role() == ExtrusionRole::Travel)
            ++count;
    }
    return count;
}

/* Return the private scope key shared by the transition and travel providers. */
slic3r_api::PluginPropertyKey<slic3r_api::PrintingExtrusionScopeProperty>
scope_property_key()
{
    return slic3r_api::printing_extrusion_scope_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
}

/* Open one core root through the named compact-scope API. */
slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope
ordered_scope(ExtrusionEntity &root)
{
    return slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope(
        slic3r_api::MutableExtrusionEntity(
            reinterpret_cast<extrusion_entity_handle *>(&root)),
        scope_property_key());
}

/* Run one explicit layer-edit chain through the production step host. */
void run_layer_edit_plugins(Print &print,
                            std::initializer_list<const char *> plugin_ids)
{
    ScopedActivePlugins active(plugin_ids);
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Steps::StepLayerExtrusionEdition::run_step(orchestrator, print);
    REQUIRE_FALSE(orchestrator.is_plugin_cancelled());
}

void run_travel_plugins(Print &print, const char *travel_plugin = TRAVEL_PLUGIN)
{
    run_layer_edit_plugins(
        print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN, travel_plugin});
}

void setup_connector(Print &print,
    slic3r_api::LayerExtrusionEdit::TravelConnection::ScopeTravelConnector &connector,
    slic3r_api::PluginProgress &progress)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    uint32_t count = 0;
    for (const PrintingGroup &group : plan.groups)
        count += uint32_t(group.layers.size());
    progress.reset(nullptr, count, nullptr);
    connector.setup(slic3r_api::ConstPrintingPlan(
        reinterpret_cast<const printing_plan_handle *>(&plan)), count);
    for (uint32_t group_idx = 0; group_idx < plan.groups.size(); ++group_idx) {
        for (uint32_t layer_idx = 0; layer_idx < plan.groups[group_idx].layers.size(); ++layer_idx) {
            slic3r_api::PrintingLayerGroup layer(reinterpret_cast<printing_layer_group_handle *>(
                &plan.groups[group_idx].layers[layer_idx]));
            connector.setup_run(layer, group_idx, layer_idx, progress);
        }
    }
}

TEST_CASE("Advanced travel keeps the straight fallback without regional context",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(
        Point(scale_i(0.), 0), Point(scale_i(1.), 0)));
    append_extrusion(tool, make_path(
        Point(scale_i(3.), 0), Point(scale_i(4.), 0)));

    run_travel_plugins(print, ADVANCED_TRAVEL_PLUGIN);

    REQUIRE(travel_count(print) == 1);
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope scope(
        slic3r_api::MutableExtrusionEntity(
            reinterpret_cast<extrusion_entity_handle *>(tool.extrusions[1].root.get())),
        scope_property_key());
    REQUIRE(scope.travel().segment_count() == 1);
    CHECK(scope.travel().point(0).x == scale_i(1.));
    CHECK(scope.travel().point(1).x == scale_i(3.));
}

} // namespace

namespace {
// Copied travel routes and host warnings from one controlled run.
struct NoTravelResult { std::vector<Polyline> routes; std::vector<std::string> warnings; };

// Run the real layer-edit host on controlled platter-space gaps. The model is
// applied normally, so NoTravel reads the same transformed helper volumes as a
// sliced print. Multiple layers exercise the worker-local sections and summary.
NoTravelResult controlled_no_travel(Print &print, const Points &endpoints,
                                   const char *provider = TRAVEL_PLUGIN);
// Add one NoTravel box of the given XY size and offset to an object.
ModelVolume *no_travel_box(ModelObject &object, double x, double y, double width, double depth, double z = 0.);
// Return the axis-aligned rectangle (x, y, width, depth) in scaled coordinates.
Polygon rectangle(double x, double y, double width, double depth);
// Closed square frame [0, size]^2 whose walls are width thick, with a free hole inside.
TriangleMesh no_travel_frame(double size, double width, double height);

NoTravelResult controlled_no_travel(Print &print, const Points &endpoints,
                                   const char *provider)
{
    REQUIRE(endpoints.size() % 2 == 0);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan = PrintingPlan{};
    for (size_t o = 0; o < print.objects().size(); ++o)
        for (size_t i = 0; i < print.object(o).instances().size(); ++i)
            plan.find_or_add_object_instance(print.object(o), i);
    plan.groups.resize(1);
    plan.groups[0].layers.resize(endpoints.size() / 2);
    for (size_t i = 0; i < endpoints.size() / 2; ++i) {
        auto &layer = plan.groups[0].layers[i];
        layer.print_z = scale_i(0.2 + 0.2 * i);
        layer.tool_groups.resize(1);
        auto &tool = layer.tool_groups[0];
        tool.extruder_id = 0;
        append_extrusion(tool, make_path(endpoints[2*i] - Point(scale_i(1.), 0), endpoints[2*i]));
        append_extrusion(tool, make_path(endpoints[2*i+1], endpoints[2*i+1] + Point(scale_i(1.), 0)));
    }
    Orchestrator::instance().consume_plugin_messages();
    print.pipeline_progress().reset();
    run_travel_plugins(print, provider);
    NoTravelResult result;
    for (auto &layer : plan.groups[0].layers) {
        const auto path = ordered_scope(*layer.tool_groups[0].extrusions[1].root).travel();
        Polyline route;
        for (uint32_t i = 0; i < path.point_count(); ++i)
            route.points.emplace_back(path.point(i).x, path.point(i).y);
        result.routes.push_back(std::move(route));
    }
    for (const auto &message : Orchestrator::instance().consume_plugin_messages())
        if (message.plugin_id == provider) result.warnings.push_back(message.message);
    return result;
}

ModelVolume *no_travel_box(ModelObject &object, double x, double y, double width, double depth, double z)
{
    auto *volume = object.add_volume(make_cube(width, depth, 1.), ModelVolumeType::NO_TRAVEL, false);
    volume->set_offset(Vec3d(x, y, z));
    return volume;
}

Polygon rectangle(double x, double y, double width, double depth)
{
    return Polygon(Points{Point::new_scale(x, y), Point::new_scale(x+width, y),
        Point::new_scale(x+width, y+depth), Point::new_scale(x, y+depth)});
}

TriangleMesh no_travel_frame(double size, double width, double height)
{
    TriangleMesh frame = make_cube(size, width, height);
    TriangleMesh top = make_cube(size, width, height);
    top.translate(0.f, float(size - width), 0.f);
    TriangleMesh left = make_cube(width, size - 2*width, height);
    left.translate(0.f, float(width), 0.f);
    TriangleMesh right = make_cube(width, size - 2*width, height);
    right.translate(float(size - width), float(width), 0.f);
    frame.merge(top);
    frame.merge(left);
    frame.merge(right);
    return frame;
}
} // namespace

TEST_CASE("Travel avoids NoTravel sections without perimeter settings", "[plugins][travel][no-travel-routing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool avoid_perimeters = GENERATE(false, true);
    const char *provider = GENERATE(TRAVEL_PLUGIN, ADVANCED_TRAVEL_PLUGIN);
    Model model;
    auto *object = model.add_object("part", "part.stl", make_cube(24., 20., 1.));
    object->add_instance();
    no_travel_box(*object, 10., 4., 4., 12.);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("avoid_crossing_perimeters", avoid_perimeters ? "1" : "0");
    Print print;
    print.apply(model, config);
    const Points endpoints{Point::new_scale(2., 10.), Point::new_scale(22., 10.)};
    const auto result = controlled_no_travel(print, endpoints, provider);
    REQUIRE(result.routes.size() == 1);
    const auto &path = result.routes.front();
    REQUIRE(path.size() > 2);
    CHECK(path.first_point() == endpoints.front());
    CHECK(path.last_point() == endpoints.back());
    CHECK(intersection_pl(path, ExPolygon(rectangle(10.001, 4.001, 3.998, 11.998))).empty());
    CHECK(result.warnings.empty());
}

TEST_CASE("Travel keeps direct paths without an applicable NoTravel section", "[plugins][travel][no-travel-routing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *provider = GENERATE(TRAVEL_PLUGIN, ADVANCED_TRAVEL_PLUGIN);
    Model model;
    auto *object = model.add_object("part", "part.stl", make_cube(24., 20., 1.));
    object->add_instance();
    SECTION("No helper") {}
    SECTION("Helper above the layer") { no_travel_box(*object, 10., 4., 4., 12., 2.); }
    SECTION("Helper beside the path") { no_travel_box(*object, 10., 14., 4., 2.); }
    SECTION("Path in a hole") {
        object->add_volume(no_travel_frame(20., 2., 1.), ModelVolumeType::NO_TRAVEL, false);
    }
    Print print;
    print.apply(model, DynamicPrintConfig::full_print_config());
    const Points endpoints{Point::new_scale(3., 10.), Point::new_scale(17., 10.)};
    const auto result = controlled_no_travel(print, endpoints, provider);
    CHECK(result.routes.front().points == endpoints);
    CHECK(result.warnings.empty());
}

TEST_CASE("Travel exempts endpoint NoTravel components but still avoids other components", "[plugins][travel][no-travel-routing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *provider = GENERATE(TRAVEL_PLUGIN, ADVANCED_TRAVEL_PLUGIN);
    Model model;
    auto *object = model.add_object("part", "part.stl", make_cube(24., 20., 1.));
    object->add_instance();
    no_travel_box(*object, 0., 8., 4., 4.);
    no_travel_box(*object, 3., 8., 3., 4.); // Union makes the exempt component extend to X=6.
    no_travel_box(*object, 10., 4., 4., 12.);
    Print print;
    print.apply(model, DynamicPrintConfig::full_print_config());
    const Points endpoints{Point::new_scale(2., 10.), Point::new_scale(22., 10.),
                          Point::new_scale(2., 10.), Point::new_scale(22., 10.)};
    const auto result = controlled_no_travel(print, endpoints, provider);
    REQUIRE(result.warnings.size() == 1); // One summary for both parallel layer runs.
    CHECK(result.warnings.front().find("2 travels exempted") != std::string::npos);
    for (const auto &path : result.routes) {
        REQUIRE(path.size() > 2);
        CHECK(intersection_pl(path, ExPolygon(rectangle(10.001, 4.001, 3.998, 11.998))).empty());
    }
}

TEST_CASE("Travel uses transformed NoTravel volumes and every real instance", "[plugins][travel][no-travel-routing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *provider = GENERATE(TRAVEL_PLUGIN, ADVANCED_TRAVEL_PLUGIN);
    Model model;
    auto *object = model.add_object("part", "part.stl", make_cube(24., 20., 1.));
    // Volume rotation makes its XY section [10,14] x [4,16].
    auto *helper = no_travel_box(*object, 14., 4., 12., 4.);
    helper->set_rotation(Vec3d(0., 0., PI/2));
    object->add_instance()->set_rotation(Vec3d(0., 0., PI/2));
    object->instances.front()->set_offset(Vec3d(40., 30., 0.));
    auto *copy = object->add_instance(*object->instances.front());
    copy->set_offset(Vec3d(90., 60., 0.));
    Print print;
    print.apply(model, DynamicPrintConfig::full_print_config());
    const auto result = controlled_no_travel(print, {Point::new_scale(30., 32.), Point::new_scale(30., 52.),
                                                    Point::new_scale(80., 62.), Point::new_scale(80., 82.)}, provider);
    REQUIRE(result.routes.size() == 2);
    for (size_t i = 0; i < 2; ++i) {
        const auto &path = result.routes[i];
        REQUIRE(path.size() > 2);
        const double dx = i == 0 ? 0. : 50., dy = i == 0 ? 0. : 30.;
        CHECK(intersection_pl(path, ExPolygon(rectangle(24.001+dx, 40.001+dy, 11.998, 3.998))).empty());
    }
    CHECK(result.warnings.empty());
}

TEST_CASE("Travel reports an inadmissible NoTravel detour and retains its direct path", "[plugins][travel][no-travel-routing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const char *provider = GENERATE(TRAVEL_PLUGIN, ADVANCED_TRAVEL_PLUGIN);
    Model model;
    auto *object = model.add_object("part", "part.stl", make_cube(24., 20., 1.));
    object->add_instance();
    // A closed frame: following the inner hole cannot get outside its outer
    // contour. Both endpoints are free, so no component is exempt.
    object->add_volume(no_travel_frame(20., 2., 1.), ModelVolumeType::NO_TRAVEL, false);
    Print print;
    print.apply(model, DynamicPrintConfig::full_print_config());
    const Points endpoints{Point::new_scale(10., 10.), Point::new_scale(22., 10.)};
    const auto result = controlled_no_travel(print, endpoints, provider);
    CHECK(result.routes.front().points == endpoints);
    REQUIRE(result.warnings.size() == 1);
    CHECK(result.warnings.front().find("1 inadmissible detours") != std::string::npos);
}

TEST_CASE("Straight travel plugin exposes its ordered-layer contract",
          "[plugins][layer-extrusion-edit][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(TRAVEL_PLUGIN);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(plugin->get_priority() == -50);
    CHECK(plugin->get_exclusive_group() == "layer_extrusion_edit.travel");
    REQUIRE(plugin->get_dependencies().size() == 2);
    CHECK(plugin->get_dependencies()[0] == TRANSITION_SCOPE_PLUGIN);
    CHECK(plugin->get_dependencies()[1] == ENTRY_STATE_PLUGIN);
    CHECK(plugin->get_used_config_keys().empty());
}

// The advanced provider reads the compatibility planner's routing settings, its attractor inset and the fixed bed shape.
TEST_CASE("Advanced travel exposes routing settings and bed shape",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *advanced = Orchestrator::instance().get_plugin(ADVANCED_TRAVEL_PLUGIN);
    REQUIRE(advanced != nullptr);
    CHECK(advanced->get_id() == ADVANCED_TRAVEL_PLUGIN);
    CHECK(advanced->get_step() == STEP_LAYER_EXTRUSION_EDIT);
    CHECK(advanced->get_priority() == -50);
    CHECK(advanced->get_exclusive_group_priority() == -2);
    CHECK(advanced->get_exclusive_group() == "layer_extrusion_edit.travel");
    CHECK(advanced->get_dependencies() == std::vector<std::string>{TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});
    const std::vector<Plugin::UsedConfigKey> actual = advanced->get_used_config_keys();
    namespace avoidance = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
    std::vector<raw_used_config_key> expected(size_t(avoidance::CompatibilityPlanner::used_config_keys(nullptr)));
    avoidance::CompatibilityPlanner::used_config_keys(expected.data());
    REQUIRE(actual.size() == expected.size() + 2);
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        CHECK(actual[idx].key == expected[idx].key);
        CHECK(actual[idx].type == expected[idx].type);
        CHECK(actual[idx].container_type == expected[idx].container_type);
        CHECK(actual[idx].option_preset_type == expected[idx].option_preset_type);
    }
    CHECK(std::any_of(actual.begin(), actual.end(), [](const Plugin::UsedConfigKey &key) {
        return key.key == "avoid_crossing_perimeters" && key.type == RAW_CO_BOOL;
    }));
    CHECK(actual[expected.size()].key == "avoid_crossing_perimeters_inset");
    CHECK(actual[expected.size()].type == RAW_CO_FLOAT_OR_PERCENT);
    CHECK(actual.back().key == "bed_shape");
    CHECK(actual.back().type == RAW_CO_VECTOR_POINT);
    CHECK(actual.back().container_type == RAW_CONTAINER_TYPE_NONE);
    CHECK(actual.back().option_preset_type == RAW_PRESET_TYPE_NONE);
    CHECK(PrintConfigDef::instance().option_keys(RAW_PRESET_TYPE_FFF_PRINTER).count("bed_shape") == 1);
}

TEST_CASE("AvoidCrossingPerimeters crossing test rejects contained travels",
          "[AvoidCrossingPerimeters][plugins][layer-extrusion-edit][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"avoid_crossing_perimeters", "1"},
        {"nozzle_diameter", "0.4"}
    });
    Print print;
    Model model;
    Slic3r::Test::init_print(
        {Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);

    REQUIRE(print.object(0).layer_count() > 0);
    const Layer &layer = print.object(0).layer(0);
    REQUIRE_FALSE(layer.lslices().empty());
    const BoundingBox bounds = get_extents(layer.lslices());
    const Point center = bounds.center();

    AvoidCrossingPerimeters detector;
    const std::vector<const Layer *> printed_layers{&layer};
    const AvoidCrossingPerimeters::PerimeterCrossingContext context{
        layer,
        printed_layers,
        0,
        0,
        scale_i(0.2),
        []() {}
    };
    detector.prepare_crossing_test(context);

    // A short segment wholly inside the printable island is the hot-path
    // rejection used before the travel router builds a detour.
    Polyline inside;
    inside.points = {
        Point(center.x() - scale_i(1.), center.y()),
        Point(center.x() + scale_i(1.), center.y())
    };
    CHECK_FALSE(detector.can_cross_perimeter(inside, true));

    // Leaving the object crosses its offset contour and must request routing.
    Polyline leaving;
    leaving.points = {
        center,
        Point(bounds.max.x() + scale_i(2.), center.y())
    };
    CHECK(detector.can_cross_perimeter(leaving, true));
}

TEST_CASE("Advanced travel routes around a hole between enabled endpoints",
          "[plugins][layer-extrusion-edit][travel][avoid-crossing-perimeters]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool snap_closed_source = GENERATE(false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"avoid_crossing_perimeters", "1"},
        {"avoid_crossing_not_first_layer", "0"},
        {"avoid_crossing_perimeters_max_detour", "0"},
        {"nozzle_diameter", "0.4"}
    });
    Print print;
    Model model;
    TriangleMesh mesh = make_cube(30., 10., 0.6);
    TriangleMesh upper = make_cube(30., 10., 0.6);
    upper.translate(0., 20., 0.);
    mesh.merge(upper);
    TriangleMesh left = make_cube(10., 10., 0.6);
    left.translate(0., 10., 0.);
    mesh.merge(left);
    TriangleMesh right = make_cube(10., 10., 0.6);
    right.translate(20., 10., 0.);
    mesh.merge(right);
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", "0"}, {"brim_width", "0"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"}});
    Slic3r::Test::init_print({mesh}, print, model, config);
    print.set_status_silent();
    print.process();
    print.pipeline_progress().reset();
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Object object_view(
        reinterpret_cast<const object_handle *>(&print.object(0)));
    Layer *built_layer = &print.object(0).layers().front();
    const slic3r_api::LayerRegionIsland region_view =
        object_view.layer(0).island(0).region_island(0);
    REQUIRE(region_view.valid());
    const LayerRegionIsland *region_island =
        reinterpret_cast<const LayerRegionIsland *>(region_view.handle());

    const c_point instance_shift = object_view.instance_shift(0);
    const Point shift(instance_shift.x, instance_shift.y);
    const BoundingBox box = get_extents(built_layer->lslices());
    const Point source = box.min + Point(scale_i(5.), scale_i(15.)) + shift;
    const Point target = box.min + Point(scale_i(25.), scale_i(15.)) + shift;

    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    const PrintingObjectInstanceId catalog_id =
        plan.find_or_add_object_instance(print.object(0), 0);
    plan.groups.emplace_back();
    plan.groups.back().printing_catalog_ids.push_back(catalog_id);
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &plan_layer = plan.groups.back().layers.back();
    plan_layer.print_z = scale_i(0.2);
    plan_layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = plan_layer.tool_groups.back();
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(
        Point(source.x() - scale_i(1.), source.y()), source,
        0, snap_closed_source ? scale_i(0.03) : 0), region_island, catalog_id);
    if (snap_closed_source)
        append_extrusion(tool, make_closed_arc_path(
            Point(source.x() + SCALED_EPSILON - 1, source.y())), region_island, catalog_id);
    append_extrusion(tool, make_path(
        target, Point(target.x() + scale_i(1.), target.y())), region_island, catalog_id);

    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN, ADVANCED_TRAVEL_PLUGIN,
        "layer_extrusion_edit.travel.advanced.zones.island_attractors"});

    REQUIRE(travel_count(print) == 1);
    const ExtrusionEntity &scope = *tool.extrusions[snap_closed_source ? 2 : 1].root;
    REQUIRE(scope.child_count() == 3);
    const ExtrusionEntity &travel = scope.child(0);
    REQUIRE(travel.has_polyline());
    CHECK(travel.polyline_ref().size() > 2);
    CHECK(travel.first_point() == source);
    CHECK(travel.last_point() == target);
    // The hole spans [10, 20] in both axes of the object; the route must stay out of its interior.
    const ExPolygon hole(Polygon(Points{box.min + Point(scale_i(10.1), scale_i(10.1)) + shift,
        box.min + Point(scale_i(19.9), scale_i(10.1)) + shift, box.min + Point(scale_i(19.9), scale_i(19.9)) + shift,
        box.min + Point(scale_i(10.1), scale_i(19.9)) + shift}));
    Polyline route;
    for (size_t idx = 0; idx < travel.polyline_ref().size(); ++idx)
        route.points.push_back(travel.polyline_ref().get_point(idx));
    CHECK(intersection_pl(route, hole).empty());
    if (snap_closed_source) {
        const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope snapped =
            ordered_scope(*tool.extrusions[1].root);
        CHECK(snapped.content().point(0).x == source.x());
        CHECK(snapped.content().point(snapped.content().point_count() - 1).x == source.x());
        CHECK(travel.polyline_ref().z_offset(0) == scale_i(0.03));
        CHECK(travel.polyline_ref().z_offset(travel.polyline_ref().size() - 1) == 0);
    }
}

TEST_CASE("Straight travels connect trees tools layers and printing groups",
          "[plugins][layer-extrusion-edit][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &first_group = plan.groups.back();
    first_group.layers.emplace_back();
    PrintingLayerGroup &first_layer = first_group.layers.back();
    first_layer.print_z = scale_i(0.2);
    first_layer.tool_groups.emplace_back();
    first_layer.tool_groups.back().extruder_id = 0;
    append_extrusion(first_layer.tool_groups.back(), make_path(
        Point(scale_i(0.), scale_i(0.)), Point(scale_i(1.), scale_i(0.))));

    // A new tool section starts away from the preceding path and receives one
    // connector before its original extrusion content.
    first_layer.tool_groups.emplace_back();
    first_layer.tool_groups.back().extruder_id = 1;
    append_extrusion(first_layer.tool_groups.back(), make_path(
        Point(scale_i(2.), scale_i(0.)), Point(scale_i(3.), scale_i(0.))));

    // Empty scopes do not break the entry-state prefix. The next non-empty
    // layer still starts from the last point observed above.
    first_group.layers.emplace_back();
    first_group.layers.back().print_z = scale_i(0.4);
    first_group.layers.emplace_back();
    PrintingLayerGroup &third_layer = first_group.layers.back();
    third_layer.print_z = scale_i(0.6);
    third_layer.tool_groups.emplace_back();
    third_layer.tool_groups.back().extruder_id = 1;
    append_extrusion(third_layer.tool_groups.back(), make_path(
        Point(scale_i(4.), scale_i(0.)), Point(scale_i(5.), scale_i(0.))));

    // PrintingGroup boundaries preserve machine continuity as well.
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &second_group_layer = plan.groups.back().layers.back();
    second_group_layer.print_z = scale_i(0.2);
    second_group_layer.tool_groups.emplace_back();
    second_group_layer.tool_groups.back().extruder_id = 0;
    append_extrusion(second_group_layer.tool_groups.back(), make_path(
        Point(scale_i(6.), scale_i(0.)), Point(scale_i(7.), scale_i(0.))));

    run_travel_plugins(print);
    CHECK(travel_count(print) == 3);

    // The pair facts are final and symmetric even when empty layers separate
    // two scopes and each worker may only mutate its own layer.
    const slic3r_api::PrintingExtrusionScopeProperty *first_source =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            plan.groups[0].layers[0].tool_groups[0].extrusions[0].root.get()))
            .get(scope_property_key());
    const slic3r_api::PrintingExtrusionScopeProperty *same_layer_target =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            plan.groups[0].layers[0].tool_groups[1].extrusions[0].root.get()))
            .get(scope_property_key());
    const slic3r_api::PrintingExtrusionScopeProperty *later_layer_target =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            plan.groups[0].layers[2].tool_groups[0].extrusions[0].root.get()))
            .get(scope_property_key());
    const slic3r_api::PrintingExtrusionScopeProperty *next_group_target =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            plan.groups[1].layers[0].tool_groups[0].extrusions[0].root.get()))
            .get(scope_property_key());
    REQUIRE(first_source != nullptr);
    REQUIRE(same_layer_target != nullptr);
    REQUIRE(later_layer_target != nullptr);
    REQUIRE(next_group_target != nullptr);
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *first_source, slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *same_layer_target, slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *same_layer_target, slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *later_layer_target, slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *later_layer_target, slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *next_group_target, slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));

    const ExtrusionEntity &wrapped =
        *plan.groups[0].layers[0].tool_groups[1].extrusions[0].root;
    REQUIRE(wrapped.child_count() == 4);
    CHECK_FALSE(wrapped.can_sort());
    CHECK_FALSE(wrapped.can_reverse());
    const ExtrusionAttributes *travel_attributes =
        wrapped.child(0).get_property<ExtrusionAttributes>();
    const ExtrusionAttributes *print_attributes =
        wrapped.child(2).get_property<ExtrusionAttributes>();
    REQUIRE(travel_attributes != nullptr);
    REQUIRE(print_attributes != nullptr);
    CHECK(travel_attributes->extrusion_role() == ExtrusionRole::Travel);
    CHECK(travel_attributes->mm3_per_mm == 0.0);
    CHECK(print_attributes->extrusion_role() == ExtrusionRole::Perimeter);

    const std::vector<const ExtrusionEntity *> leaves = plan_leaves(print);
    REQUIRE(leaves.size() == 7);
    for (size_t leaf_idx = 1; leaf_idx < leaves.size(); ++leaf_idx)
        CHECK(leaves[leaf_idx - 1]->last_point() == leaves[leaf_idx]->first_point());

    // A second run sees the explicit connectors as part of the continuous
    // stream and therefore inserts nothing else.
    run_travel_plugins(print);
    CHECK(travel_count(print) == 3);
    CHECK(plan_leaves(print).size() == 7);
}

TEST_CASE("Travel connects distinct scopes in one PrintingExtrusion",
          "[plugins][layer-extrusion-edit][travel][inherited-z]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;

    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    root->add_property(ExtrusionPropertyZOffset(scale_i(0.04)));
    root->append(make_path(Point(scale_i(0.), 0), Point(scale_i(1.), 0),
                           scale_i(0.01), scale_i(0.01)));
    root->append(make_path(Point(scale_i(3.), 0), Point(scale_i(4.), 0),
                           scale_i(0.03), scale_i(0.03)));
    append_extrusion(tool, std::move(root));
    ExtrusionEntity *const owner_root = tool.extrusions.back().root.get();

    run_travel_plugins(print);

    // The PrintingExtrusion root remains the common owner. Its two children
    // become sibling compact scopes, and only the target owns the connector.
    REQUIRE(tool.extrusions[0].root.get() == owner_root);
    REQUIRE(owner_root->child_count() == 2);
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope source =
        ordered_scope(owner_root->child(0));
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target =
        ordered_scope(owner_root->child(1));
    REQUIRE(source.has_outgoing_transition());
    REQUIRE(target.has_incoming_transition());
    REQUIRE(target.travel().point_count() == 2);
    CHECK(target.travel().point(0).x == scale_i(1.));
    CHECK(target.travel().point(1).x == scale_i(3.));
    CHECK(target.travel().z_offset(0) == scale_i(0.01));
    CHECK(target.travel().z_offset(1) == scale_i(0.03));
}

TEST_CASE("Straight travel snapping uses XYZ epsilon and preserves closed seams",
          "[plugins][layer-extrusion-edit][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;

    const Point origin(scale_i(0.), scale_i(0.));
    const Point first_end(scale_i(1.), scale_i(0.));
    append_extrusion(tool, make_path(origin, first_end, 0, scale_i(0.03)));

    // This closed arc begins less than epsilon away in XY and at the same Z.
    // Both seam copies must snap while its arc metadata survives.
    const Point near_seam(first_end.x() + SCALED_EPSILON - 1, first_end.y());
    append_extrusion(tool, make_closed_arc_path(near_seam));

    // An exact-epsilon XY gap and a later pure-Z gap both require explicit
    // connector leaves because only strictly smaller distances are snapped.
    const Point arc_end = near_seam;
    append_extrusion(tool, make_path(
        Point(arc_end.x() + SCALED_EPSILON, arc_end.y()),
        Point(scale_i(3.), scale_i(0.)), scale_i(0.03), scale_i(0.03)));
    append_extrusion(tool, make_path(
        Point(scale_i(3.), scale_i(0.)),
        Point(scale_i(4.), scale_i(0.)),
        scale_i(0.03) + SCALED_EPSILON,
        scale_i(0.03) + SCALED_EPSILON));

    ExtrusionEntity *closed_scope = tool.extrusions[1].root.get();
    REQUIRE(closed_scope->polyline_ref().has_arc());
    run_travel_plugins(print);

    CHECK(travel_count(print) == 2);
    REQUIRE(closed_scope->child_count() == 2);
    const ExtrusionEntity &closed_source = closed_scope->child(0);
    REQUIRE(closed_source.has_polyline());
    CHECK(closed_source.first_point() == first_end);
    CHECK(closed_source.last_point() == first_end);
    CHECK(closed_source.polyline_ref().z_offset(0) == scale_i(0.03));
    CHECK(closed_source.polyline_ref().z_offset(closed_source.polyline_ref().size() - 1) ==
          scale_i(0.03));
    CHECK(closed_source.polyline_ref().has_arc());

    const ExtrusionEntity &xy_wrapper = *tool.extrusions[2].root;
    REQUIRE(xy_wrapper.child_count() == 4);
    REQUIRE(xy_wrapper.child(0).has_polyline());
    CHECK(xy_wrapper.child(0).polyline_ref().z_offset(0) == scale_i(0.03));
    CHECK(xy_wrapper.child(0).polyline_ref().z_offset(1) == scale_i(0.03));

    const ExtrusionEntity &z_wrapper = *tool.extrusions[3].root;
    REQUIRE(z_wrapper.child_count() == 3);
    REQUIRE(z_wrapper.child(0).has_polyline());
    CHECK(z_wrapper.child(0).first_point() == z_wrapper.child(0).last_point());
    CHECK(z_wrapper.child(0).polyline_ref().z_offset(0) == scale_i(0.03));
    CHECK(z_wrapper.child(0).polyline_ref().z_offset(1) ==
          scale_i(0.03) + SCALED_EPSILON);

    const std::vector<const ExtrusionEntity *> leaves = plan_leaves(print);
    for (size_t leaf_idx = 1; leaf_idx < leaves.size(); ++leaf_idx) {
        CHECK(leaves[leaf_idx - 1]->last_point() == leaves[leaf_idx]->first_point());
    }
}

TEST_CASE("Existing travels remain ordinary connected leaves",
          "[plugins][layer-extrusion-edit][travel]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(
        Point(scale_i(0.), 0), Point(scale_i(1.), 0)));
    append_extrusion(tool, make_path(
        Point(scale_i(1.), 0), Point(scale_i(2.), 0), 0, 0, ExtrusionRole::Travel));
    append_extrusion(tool, make_path(
        Point(scale_i(2.), 0), Point(scale_i(3.), 0)));

    ExtrusionEntity *existing_travel = tool.extrusions[1].root.get();
    run_travel_plugins(print);

    CHECK(travel_count(print) == 1);
    CHECK(tool.extrusions[1].root.get() == existing_travel);
    CHECK(existing_travel->first_point() == Point(scale_i(1.), 0));
    CHECK(existing_travel->last_point() == Point(scale_i(2.), 0));

    const slic3r_api::PrintingExtrusionScopeProperty *source =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            tool.extrusions[0].root.get())).get(scope_property_key());
    const slic3r_api::PrintingExtrusionScopeProperty *target =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            tool.extrusions[2].root.get())).get(scope_property_key());
    REQUIRE(source != nullptr);
    REQUIRE(target != nullptr);
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *source, slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(
        *target, slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
    CHECK(ordered_scope(*tool.extrusions[2].root).travel().segment_count() == 0);
}

TEST_CASE("Travel connector uses source after and target before phases",
          "[plugins][layer-extrusion-edit][travel][scope-phases]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(Point(scale_i(0.), 0), Point(scale_i(1.), 0)));
    append_extrusion(tool, make_path(Point(scale_i(3.), 0), Point(scale_i(4.), 0)));

    // Prepare the compact shapes first, then emulate process plugins which
    // append geometry without changing the marked roots or their child lists.
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN});
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope source =
        ordered_scope(*tool.extrusions[0].root);
    slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target =
        ordered_scope(*tool.extrusions[1].root);
    REQUIRE(source.after().valid());
    REQUIRE(target.before().valid());
    REQUIRE(source.after().set_points(std::vector<c_point>{
        c_point{scale_i(1.), 0}, c_point{scale_i(1.5), 0}}));
    REQUIRE(target.before().set_points(std::vector<c_point>{
        c_point{scale_i(2.5), 0}, c_point{scale_i(3.), 0}}));
    source.after().get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE)
        .mm3_per_mm(0.).width(0.f).height(0.f);
    target.before().get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE)
        .mm3_per_mm(0.).width(0.f).height(0.f);

    run_layer_edit_plugins(print, {ENTRY_STATE_PLUGIN, TRAVEL_PLUGIN});

    target = ordered_scope(*tool.extrusions[1].root);
    REQUIRE(target.travel().point_count() == 2);
    CHECK(target.travel().point(0).x == scale_i(1.5));
    CHECK(target.travel().point(1).x == scale_i(2.5));
}

TEST_CASE("Semantic continuous transitions keep the travel slot empty",
          "[plugins][layer-extrusion-edit][travel][semantic-transition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = 0;
    std::unique_ptr<ExtrusionPath> source = make_path(
        Point(scale_i(0.), 0), Point(scale_i(1.), 0));
    ExtrusionPropertyModifier modifier;
    modifier.set_enforce_retraction(true);
    source->add_property(modifier);
    append_extrusion(tool, std::move(source));
    append_extrusion(tool, make_path(
        Point(scale_i(1.), 0), Point(scale_i(2.), 0)));

    run_travel_plugins(print);

    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target =
        ordered_scope(*tool.extrusions[1].root);
    REQUIRE(target.has_incoming_transition());
    CHECK(target.travel().segment_count() == 0);
    const slic3r_api::PrintingExtrusionScopeProperty *source_property =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            tool.extrusions[0].root.get())).get(scope_property_key());
    const slic3r_api::PrintingExtrusionScopeProperty *target_property =
        slic3r_api::MutableExtrusionEntity(reinterpret_cast<extrusion_entity_handle *>(
            tool.extrusions[1].root.get())).get(scope_property_key());
    REQUIRE(source_property != nullptr);
    REQUIRE(target_property != nullptr);
    CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(
        *source_property,
        slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(
        *target_property,
        slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
}

namespace {
NarrowBridgeResult narrow_bridge_scene()
{
    namespace travel = slic3r_api::LayerExtrusionEdit::TravelConnection;
    namespace zones = slic3r_api::LayerExtrusionEdit::AdvancedTravel;
    namespace avoidance = slic3r_api::LayerExtrusionEdit::AvoidCrossing;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "1"},
        {"external_perimeter_extrusion_width", "0.4"}, {"perimeter_extrusion_width", "0.4"},
        {"thin_walls", "1"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
        {"fill_density", "0%"}, {"skirts", "0"}, {"brim_width", "0"}});
    Print print;
    Model model;
    Slic3r::Test::init_print({Test::Plugins::dumbbell(0.4)}, print, model, config);
    print.set_status_silent();
    print.process();
    Layer &layer = print.object(0).layers()[1];
    REQUIRE(layer.lslices().size() == 1);
    const Point minimum = get_extents(layer.lslices()).min;
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const c_point shift = print_view.object(0).instance_shift(0);
    const Point origin = minimum + Point(shift.x, shift.y);
    NarrowBridgeResult result;
    result.slices = layer.lslices();
    for (ExPolygon &slice : result.slices) slice.translate(shift.x, shift.y);
    result.bridge = {ExPolygon(Polygon(Points{
        origin + Point(scale_i(10.), scale_i(4.8)), origin + Point(scale_i(15.), scale_i(4.8)),
        origin + Point(scale_i(15.), scale_i(5.2)), origin + Point(scale_i(10.), scale_i(5.2))}))};
    result.source = origin + Point(scale_i(5.), scale_i(2.));
    result.target = origin + Point(scale_i(20.), scale_i(2.));
    REQUIRE(result.slices.front().contains(result.source));
    REQUIRE(result.slices.front().contains(result.target));
    REQUIRE(diff_ex(result.bridge, result.slices).empty());
    REQUIRE_FALSE(diff_pl(Polylines{Polyline(Points{result.source, result.target})}, result.slices).empty());

    // Require a continuous deposited perimeter chain across the neck; attribute changes may split it into leaves.
    bool printed_bridge = false;
    Polylines bridge_parts;
    std::vector<const ExtrusionEntity *> deposited;
    for (const LayerSliceIsland &island : layer.islands())
        for (const LayerRegionIsland &region : island.regions_islands())
            for (const auto &[role, bucket] : region.extrusion_buckets())
                collect_geometric_leaves(bucket, deposited);
    for (const ExtrusionEntity *leaf : deposited) {
        const ExtrusionAttributes *attributes = leaf->get_property<ExtrusionAttributes>();
        if (attributes == nullptr || !attributes->extrusion_role().is_perimeter()) continue;
        Polyline path = leaf->polyline_ref().to_polyline();
        path.translate(shift.x, shift.y);
        append(bridge_parts, intersection_pl(Polylines{path}, result.bridge));
    }
    std::vector<size_t> pending_parts;
    std::vector<bool> reached(bridge_parts.size(), false);
    for (size_t i = 0; i < bridge_parts.size(); ++i)
        if (get_extents(bridge_parts[i]).min.x() <= origin.x() + scale_i(10.1)) pending_parts.push_back(i);
    while (!pending_parts.empty()) {
        const size_t current = pending_parts.back();
        pending_parts.pop_back();
        if (reached[current]) continue;
        reached[current] = true;
        const Polyline &part = bridge_parts[current];
        if (get_extents(part).max.x() >= origin.x() + scale_i(14.9)) printed_bridge = true;
        // Only shared endpoints within the scaled epsilon connect deposited pieces.
        for (size_t i = 0; i < bridge_parts.size(); ++i)
            if (!reached[i] && (part.front().coincides_with_epsilon(bridge_parts[i].front()) ||
                part.front().coincides_with_epsilon(bridge_parts[i].back()) ||
                part.back().coincides_with_epsilon(bridge_parts[i].front()) ||
                part.back().coincides_with_epsilon(bridge_parts[i].back()))) pending_parts.push_back(i);
    }
    REQUIRE(printed_bridge);
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    const LayerRegionIsland *region = reinterpret_cast<const LayerRegionIsland *>(
        layer_view.island(0).get_or_create_full_region_island(0).handle());
    PrintingPlan &plan = print.mutable_printing_plan();
    const uint32_t catalog = plan.find_or_add_object_instance(print.object(0), 0);
    plan.groups.clear(); plan.groups.resize(1); plan.groups[0].layers.resize(1);
    PrintingLayerGroup &current = plan.groups[0].layers[0];
    current.print_z = layer.scaled_print_z(); current.tool_groups.resize(1);
    PrintingToolGroup &tool = current.tool_groups[0]; tool.extruder_id = 0;
    append_extrusion(tool, make_path(result.source - Point(scale_i(0.1), 0), result.source), region, catalog);
    append_extrusion(tool, make_path(result.target, result.target + Point(scale_i(0.1), 0)), region, catalog);
    print.pipeline_progress().reset();
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});
    travel::ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&host));
    slic3r_api::PluginProgress progress;
    setup_connector(print, connector, progress);
    slic3r_api::PrintingLayerGroup current_view(reinterpret_cast<printing_layer_group_handle *>(&current));
    travel::LayerTravelChronology chronology = connector.prepare(current_view, 0, 0);
    REQUIRE(chronology.travels.size() == 1);
    PluginStorage storage;
    plugin_run_context context{};
    context.plugin_storage = reinterpret_cast<storage_handle *>(&storage);
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    avoidance::CompatibilityPlanner eligibility(print_view, plan_view, &context, 0, 0);
    const travel::PendingTravel &pending = chronology.travels.front();
    REQUIRE(eligibility.avoidance_enabled(pending.source, pending.target));
    REQUIRE(pending.source.position.x == result.source.x());
    REQUIRE(pending.source.position.y == result.source.y());
    REQUIRE(pending.target.position.x == result.target.x());
    REQUIRE(pending.target.position.y == result.target.y());
    avoidance::LayerInputCache cache;
    const zones::ZoneContributorInput input{print_view, plan_view, context, cache, chronology.travels};
    zones::DefaultAttractorContributor attractors;
    zones::DefaultForbiddenTopsContributor tops;
    zones::TravelZoneView zone_view({std::cref(attractors), std::cref(tops)}, input);
    result.attractors = zone_view.query(0, pending).attractors;

    print.pipeline_progress().reset();
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN,
        "layer_extrusion_edit.travel.advanced",
        "layer_extrusion_edit.travel.advanced.zones.island_attractors",
        "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"});
    const slic3r_api::ExtrusionEntity route = ordered_scope(*tool.extrusions[1].root).travel();
    REQUIRE(route.point_count() >= 2);
    for (size_t p = 0; p < route.point_count(); ++p) {
        const c_point point = route.point(p);
        result.route.points.emplace_back(point.x, point.y);
    }
    REQUIRE(result.route.points.front() == result.source);
    REQUIRE(result.route.points.back() == result.target);
    return result;
}
} // namespace

// Keep scene validity visible independently of expected routing failures.
TEST_CASE("Advanced narrow bridge scene has connected printed material and an eligible travel", "[plugins][advanced-router]")
{
    narrow_bridge_scene();
}

// A route between the squares must use their printed neck, not the empty chord below it.
TEST_CASE("Advanced travel follows the connected attractor through a one-perimeter bridge", "[plugins][advanced-router]")
{
    const NarrowBridgeResult result = narrow_bridge_scene();
    size_t connected_attractors = 0;
    for (const slic3r_api::PluginCommon::polygon_work::ExPolygon &area : result.attractors) {
        if (area.contains({result.source.x(), result.source.y()}) &&
            area.contains({result.target.x(), result.target.y()}) &&
            area.contains({(result.source.x() + result.target.x()) / 2, result.source.y() + scale_i(3.)}))
            ++connected_attractors;
    }
    CAPTURE(result.attractors.size(), connected_attractors, result.route.points.size());
    CHECK(connected_attractors == 1);
    bool through_bridge = false;
    for (size_t p = 1; p < result.route.points.size(); ++p) {
        const Polyline segment(Points{result.route.points[p - 1], result.route.points[p]});
        through_bridge |= !intersection_pl(Polylines{segment}, result.bridge).empty();
        CAPTURE(p, segment.points.front(), segment.points.back());
        CHECK(diff_pl(Polylines{segment}, result.slices).empty());
    }
    CHECK(through_bridge);
}

// Keep layer entry straight; for same-attractor endpoints the option without a domain is admitted in M and cheaper than weighted links, while a later cross-component or top route detours.
TEST_CASE("Advanced zones keep layer entry straight while later travels avoid tops", "[plugins][travel][travel-zones]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool following_gap = GENERATE(false, true);
    const int contributors = GENERATE(0, 1, 2, 3);
    const bool advanced = GENERATE(false, true);
    const bool same_attractor = GENERATE(false, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), advanced ? "layer_extrusion_edit.travel.advanced" : TRAVEL_PLUGIN);
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}});
    TriangleMesh mesh = make_cube(4., 4., 0.6);
    TriangleMesh middle = make_cube(4., 4., 0.6);
    middle.translate(8., 0., 0.); mesh.merge(middle);
    TriangleMesh tower = make_cube(4., 4., 1.);
    tower.translate(16., 0., 0.); mesh.merge(tower);
    Print print;
    Model model;
    Slic3r::Test::init_print({mesh}, print, model, config);
    print.set_status_silent();
    print.process();
    Layer &layer = print.object(0).layers()[2];
    const Point minimum = get_extents(layer.lslices()).min;
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const c_point shift = print_view.object(0).instance_shift(0);
    const Point left = minimum + Point(scale_i(2.), scale_i(2.));
    const Point right = minimum + Point(scale_i(18.), scale_i(2.));
    const LayerRegionIsland *left_region = nullptr, *right_region = nullptr;
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    for (size_t i = 0; i < layer.lslices().size(); ++i) {
        const slic3r_api::LayerRegionIsland region = layer_view.island(i).get_or_create_full_region_island(0);
        if (layer.lslices()[i].contains(left)) left_region = reinterpret_cast<const LayerRegionIsland *>(region.handle());
        if (layer.lslices()[i].contains(right)) right_region = reinterpret_cast<const LayerRegionIsland *>(region.handle());
    }
    REQUIRE(left_region != nullptr);
    REQUIRE(right_region != nullptr);
    PrintingPlan &plan = print.mutable_printing_plan();
    const uint32_t catalog = plan.find_or_add_object_instance(print.object(0), 0);
    plan.groups.clear();
    plan.groups.resize(1);
    plan.groups[0].layers.resize(2);
    PrintingLayerGroup &previous = plan.groups[0].layers[0];
    previous.print_z = scale_i(0.4);
    previous.tool_groups.resize(1);
    previous.tool_groups[0].extruder_id = 0;
    const Point entry = minimum + Point(scale_i(-2.) + shift.x, scale_i(2.) + shift.y);
    append_extrusion(previous.tool_groups[0], make_path(entry - Point(scale_i(1.), 0), entry));
    PrintingLayerGroup &current = plan.groups[0].layers[1];
    current.print_z = layer.scaled_print_z();
    current.tool_groups.resize(1);
    PrintingToolGroup &tool = current.tool_groups[0];
    tool.extruder_id = 0;
    const Point target = same_attractor ? left + Point(shift.x + scale_i(1.8), shift.y) : right + Point(shift.x, shift.y);
    append_extrusion(tool, make_path(target, target + (same_attractor ? Point(0, scale_i(0.1)) : Point(scale_i(1.), 0))),
        same_attractor ? left_region : right_region, catalog);
    if (following_gap)
        append_extrusion(tool, make_path(left + Point(shift.x - (same_attractor ? scale_i(1.8) : 0), shift.y),
            left + Point(shift.x + scale_i(1.), shift.y)), left_region, catalog);
    // This hand-built plan is a separate layer-edit run, not another unit of the completed slicing run.
    print.pipeline_progress().reset();
    ScopedActivePlugins active({TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN,
        TRAVEL_PLUGIN,
        "layer_extrusion_edit.travel.advanced",
        "layer_extrusion_edit.travel.advanced.zones.island_attractors",
        "layer_extrusion_edit.travel.advanced.zones.forbidden_tops"});
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.island_attractors", (contributors & 1) != 0));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", !same_attractor && (contributors & 2) != 0));
    host.reset_plugin_cancel();
    Steps::StepLayerExtrusionEdition::run_step(host, print);
    REQUIRE_FALSE(host.is_plugin_cancelled());
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope first = ordered_scope(*tool.extrusions[0].root);
    REQUIRE(first.travel().point_count() == 2);
    CHECK(first.travel().point(0).x == entry.x());
    CHECK(first.travel().point(1).x == target.x());
    if (following_gap) {
        const bool expected_detour = advanced && !same_attractor &&
            ((contributors & 1) != 0 || (contributors & 2) != 0);
        CHECK((ordered_scope(*tool.extrusions[1].root).travel().point_count() > 2) ==
            expected_detour);
    }
}

// Compare inter-island crossings before and after the middle island's arrival, with both static-top activation states.
TEST_CASE("Printed tops start avoiding a third island only after its last arrival", "[plugins][travel][travel-zones]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool static_tops = GENERATE(false, true);
    const bool printed_tops = GENERATE(false, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(Steps::plugin_exclusive_group_option_key(host, STEP_LAYER_EXTRUSION_EDIT,
        "layer_extrusion_edit.travel"), "layer_extrusion_edit.travel.advanced");
    config.set_deserialize_strict({{"avoid_crossing_perimeters", "1"}, {"avoid_crossing_top", "1"},
        {"avoid_crossing_not_first_layer", "0"}, {"avoid_crossing_perimeters_max_detour", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"perimeters", "2"},
        {"top_solid_layers", "1"}, {"bottom_solid_layers", "1"}, {"skirts", "0"}, {"brim_width", "0"}});
    TriangleMesh mesh = make_cube(4., 4., 0.6);
    for (double x : {8., 16.}) {
        TriangleMesh cube = make_cube(4., 4., 0.6);
        cube.translate(x, 0., 0.); mesh.merge(cube);
    }
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(mesh, ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->add_instance()->set_offset(Vec3d(60., 30., 0.));
    object->ensure_on_bed();
    Print print;
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    Layer &layer = print.object(0).layers().back();
    const Point minimum = get_extents(layer.lslices()).min;
    const slic3r_api::Layer layer_view(reinterpret_cast<const layer_handle *>(&layer));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    Point centers[3];
    const LayerRegionIsland *regions[3] = {nullptr, nullptr, nullptr};
    for (size_t i = 0; i < 3; ++i) {
        centers[i] = minimum + Point(scale_i(2. + 8. * i), scale_i(2.));
        for (size_t j = 0; j < layer.lslices().size(); ++j)
            if (layer.lslices()[j].contains(centers[i]))
                regions[i] = reinterpret_cast<const LayerRegionIsland *>(
                    layer_view.island(j).get_or_create_full_region_island(0).handle());
        REQUIRE(regions[i] != nullptr);
    }
    AvoidCrossingPerimeters oracle;
    oracle.init_layer(layer);
    oracle.travel_to({layer, centers[0], Point::Zero(), extruder_index_t(0), 0., false}, centers[2]);
    ExPolygons middle_tops;
    for (const ExPolygon &top : oracle.internal_boundary().to_avoid)
        if (top.contains(centers[1])) middle_tops.push_back(top);
    REQUIRE_FALSE(middle_tops.empty());
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear(); plan.groups.resize(1); plan.groups[0].layers.resize(1);
    PrintingLayerGroup &current = plan.groups[0].layers[0];
    current.print_z = layer.scaled_print_z(); current.tool_groups.resize(1);
    PrintingToolGroup &tool = current.tool_groups[0]; tool.extruder_id = 0;
    for (size_t instance = 0; instance < 2; ++instance) {
        const uint32_t id = plan.find_or_add_object_instance(print.object(0), instance);
        const c_point shift = print_view.object(0).instance_shift(instance);
        for (size_t island : {size_t(0), size_t(2), size_t(1), size_t(0), size_t(2)}) {
            const Point start = centers[island] + Point(shift.x, shift.y);
            append_extrusion(tool, make_path(start, start + Point(scale_i(0.1), 0)), regions[island], id);
        }
    }
    print.pipeline_progress().reset();
    ScopedActivePlugins active({TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN,
        "layer_extrusion_edit.travel.advanced",
        "layer_extrusion_edit.travel.advanced.zones.island_attractors"});
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.forbidden_tops", static_tops));
    REQUIRE(host.set_plugin_active("layer_extrusion_edit.travel.advanced.zones.printed_tops", printed_tops));
    host.reset_plugin_cancel();
    Steps::StepLayerExtrusionEdition::run_step(host, print);
    REQUIRE_FALSE(host.is_plugin_cancelled());
    for (size_t instance = 0; instance < 2; ++instance) {
        const c_point shift = print_view.object(0).instance_shift(instance);
        for (size_t index : {size_t(1), size_t(4)}) {
            const slic3r_api::ExtrusionEntity route = ordered_scope(*tool.extrusions[instance * 5 + index].root).travel();
            const bool detour = printed_tops ? index == 4 : static_tops;
            double length = 0.;
            bool crosses = false;
            for (size_t p = 1; p < route.point_count(); ++p) {
                const c_point a = route.point(p - 1), b = route.point(p);
                const Polyline segment(Points{Point(a.x - shift.x, a.y - shift.y), Point(b.x - shift.x, b.y - shift.y)});
                length += segment.length();
                crosses |= !intersection_pl(Polylines{segment}, middle_tops).empty();
            }
            CHECK(crosses == !detour);
            const c_point first = route.point(0), last = route.point(route.point_count() - 1);
            const double direct = Point(first.x, first.y).distance_to(Point(last.x, last.y));
            CHECK((length > direct + SCALED_EPSILON) == detour);
        }
    }
}

TEST_CASE("Prepared travel chronology records snapped sources and layer entry provenance",
          "[plugins][layer-extrusion-edit][travel][chronology]")
{
    using namespace slic3r_api::LayerExtrusionEdit::TravelConnection;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].layers.resize(4);
    PrintingLayerGroup &first = plan.groups[0].layers[0];
    first.print_z = scale_i(0.2);
    first.tool_groups.resize(2);
    first.tool_groups[0].extruder_id = 0;
    first.tool_groups[1].extruder_id = 1;
    const Point seam(scale_i(1.), 0);
    append_extrusion(first.tool_groups[0], make_path(Point(0, 0), seam, 0, scale_i(0.03)));
    append_extrusion(first.tool_groups[0], make_closed_arc_path(
        Point(seam.x() + SCALED_EPSILON - 1, 0)));
    append_extrusion(first.tool_groups[1], make_path(
        Point(scale_i(3.), 0), Point(scale_i(4.), 0), scale_i(0.03), scale_i(0.03)));
    append_extrusion(first.tool_groups[1], make_path(
        Point(scale_i(4.), 0), Point(scale_i(5.), 0), scale_i(0.05), scale_i(0.05)));
    plan.groups[0].layers[1].print_z = scale_i(0.4);
    PrintingLayerGroup &later = plan.groups[0].layers[2];
    later.print_z = scale_i(0.6);
    later.tool_groups.resize(1);
    later.tool_groups[0].extruder_id = 1;
    append_extrusion(later.tool_groups[0], make_path(
        Point(scale_i(7.), 0), Point(scale_i(8.), 0)));
    plan.groups[0].layers[3].print_z = scale_i(0.8);
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});

    ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    slic3r_api::PluginProgress progress;
    setup_connector(print, connector, progress);
    slic3r_api::PrintingLayerGroup first_view(reinterpret_cast<printing_layer_group_handle *>(&first));
    LayerTravelChronology chronology = connector.prepare(first_view, 0, 0);
    REQUIRE(chronology.travels.size() == 2);
    CHECK(progress.done() == 0);
    const PendingTravel &xy = chronology.travels[0];
    CHECK(xy.pair_index == 2);
    CHECK(xy.extruder_id == 1);
    CHECK(xy.source.position.x == seam.x());
    CHECK(xy.source.position.y == 0);
    CHECK(xy.source.position.z == scale_i(0.23));
    CHECK(xy.target.position.x == scale_i(3.));
    CHECK(xy.target.position.z == scale_i(0.23));
    CHECK(xy.source_extrusion.handle() == reinterpret_cast<const printing_extrusion_handle *>(
        &first.tool_groups[0].extrusions[1]));
    CHECK(xy.target_extrusion.handle() == reinterpret_cast<const printing_extrusion_handle *>(
        &first.tool_groups[1].extrusions[0]));
    const PendingTravel &z = chronology.travels[1];
    CHECK(z.pair_index == 3);
    CHECK(z.source.position.x == scale_i(4.));
    CHECK(z.target.position.x == scale_i(4.));
    CHECK(z.source.position.z == scale_i(0.23));
    CHECK(z.target.position.z == scale_i(0.25));
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope closed =
        ordered_scope(*first.tool_groups[0].extrusions[1].root);
    CHECK(closed.content().point(0).x == seam.x());
    CHECK(closed.content().point(closed.content().point_count() - 1).x == seam.x());
    CHECK(closed.content().segment(0).radius != 0.f);
    CHECK(ordered_scope(*first.tool_groups[1].extrusions[0].root).travel().point_count() == 0);
    size_t calls = 0;
    connector.apply(chronology, [&calls, &chronology](const TravelEndpoint &source,
        const TravelEndpoint &target, extruder_index_t tool) {
        REQUIRE(calls < chronology.travels.size());
        CHECK(source.position.x == chronology.travels[calls].source.position.x);
        CHECK(target.position.x == chronology.travels[calls].target.position.x);
        CHECK(tool == chronology.travels[calls].extruder_id);
        ++calls;
        return straight_path(source, target, tool);
    }, progress);
    CHECK(calls == 2);
    CHECK(calls == chronology.travels.size());
    CHECK(progress.done() == 4);
    size_t index = 0;
    for (PrintingToolGroup &tool : first.tool_groups) {
        for (PrintingExtrusion &extrusion : tool.extrusions) {
            const slic3r_api::PrintingExtrusionScopeProperty *property =
                ordered_scope(*extrusion.root).root().get(scope_property_key());
            REQUIRE(property != nullptr);
            CHECK(slic3r_api::printing_extrusion_scope_has_flag(*property,
                slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED) == (index >= 2));
            CHECK(slic3r_api::printing_extrusion_scope_has_flag(*property,
                slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED) == (index >= 1));
            ++index;
        }
    }
    slic3r_api::PrintingLayerGroup later_view(reinterpret_cast<printing_layer_group_handle *>(&later));
    LayerTravelChronology entry = connector.prepare(later_view, 0, 2);
    REQUIRE(entry.travels.size() == 1);
    CHECK_FALSE(entry.travels[0].source_extrusion.valid());
    CHECK(entry.travels[0].target_extrusion.valid());
    CHECK(entry.travels[0].pair_index == 0);
    CHECK(entry.travels[0].source.position.x == scale_i(5.));
    CHECK(entry.travels[0].source.position.z == scale_i(0.25));
    CHECK(entry.travels[0].target.position.x == scale_i(7.));
    CHECK(entry.travels[0].target.position.z == scale_i(0.6));
    connector.apply(entry, straight_path, progress);
    CHECK(progress.done() == 5);
    const slic3r_api::PrintingExtrusionScopeProperty *final_property =
        ordered_scope(*later.tool_groups[0].extrusions[0].root).root().get(scope_property_key());
    REQUIRE(final_property != nullptr);
    CHECK(slic3r_api::printing_extrusion_scope_has_flag(*final_property,
        slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
    CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(*final_property,
        slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
    for (uint32_t empty_index : {1u, 3u}) {
        slic3r_api::PrintingLayerGroup empty(reinterpret_cast<printing_layer_group_handle *>(
            &plan.groups[0].layers[empty_index]));
        LayerTravelChronology none = connector.prepare(empty, 0, empty_index);
        CHECK(none.travels.empty());
        connector.apply(none, straight_path, progress);
        CHECK(progress.done() == 5);
    }
}

// Check activation and summary flags on controlled gaps with real sliced regional provenance.
TEST_CASE("Compatibility eligibility supplies dated travel activation",
          "[plugins][layer-extrusion-edit][travel][travel-zones][avoid-crossing-perimeters]")
{
    using namespace slic3r_api::LayerExtrusionEdit;
    using namespace TravelConnection;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool enabled = GENERATE(false, true);
    const bool exclude_first = GENERATE(false, true);
    const size_t layer_index = GENERATE(size_t(0), size_t(1));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"avoid_crossing_perimeters", enabled ? "1" : "0"},
        {"avoid_crossing_not_first_layer", exclude_first ? "1" : "0"}});
    Print print;
    Model model;
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
    slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    const slic3r_api::Layer layer = print_view.object(0).layer(layer_index);
    REQUIRE(layer.island_count() > 0);
    const slic3r_api::LayerRegionIsland region = layer.island(0).get_or_create_full_region_island(0);
    REQUIRE(region.region_count() > 0);
    const Point center = get_extents(print.object(0).layer(layer_index).lslices()).center();
    const c_point shift = print_view.object(0).instance_shift(0);
    const Point start = center + Point(shift.x, shift.y);
    PrintingPlan &plan = print.mutable_printing_plan();
    const uint32_t catalog = plan.find_or_add_object_instance(print.object(0), 0);
    plan.groups.resize(1);
    plan.groups[0].layers.resize(1);
    PrintingLayerGroup &group = plan.groups[0].layers[0];
    group.print_z = print.object(0).layer(layer_index).scaled_print_z();
    group.tool_groups.resize(1);
    PrintingToolGroup &tool = group.tool_groups[0];
    tool.extruder_id = 0;
    const LayerRegionIsland *core_region = reinterpret_cast<const LayerRegionIsland *>(region.handle());
    append_extrusion(tool, make_path(start, start + Point(scale_i(1.), 0)), core_region, catalog);
    append_extrusion(tool, make_path(start + Point(scale_i(2.), 0), start + Point(scale_i(3.), 0)), core_region, catalog);
    append_extrusion(tool, make_path(start + Point(scale_i(4.), 0), start + Point(scale_i(5.), 0)));
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});
    ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&orchestrator));
    slic3r_api::PluginProgress progress;
    setup_connector(print, connector, progress);
    slic3r_api::PrintingLayerGroup group_view(reinterpret_cast<printing_layer_group_handle *>(&group));
    const LayerTravelChronology chronology = connector.prepare(group_view, 0, 0);
    REQUIRE(chronology.travels.size() == 2);
    const slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&plan));
    plugin_run_context context{};
    AvoidCrossing::CompatibilityPlanner planner(print_view, plan_view, &context, 0, 0);
    const bool expected = enabled && !(exclude_first && layer_index == 0);
    CHECK(planner.avoidance_enabled(chronology.travels[0].source, chronology.travels[0].target) == expected);
    CHECK_FALSE(planner.avoidance_enabled(chronology.travels[1].source, chronology.travels[1].target));
    CHECK_FALSE(planner.avoidance_enabled(chronology.travels[1].target, chronology.travels[1].source));
    TravelEndpoint invalid_instance = chronology.travels[0].target;
    invalid_instance.printing_catalog_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    CHECK_FALSE(planner.avoidance_enabled(chronology.travels[0].source, invalid_instance));
    bool any_eligible = false;
    for (const PendingTravel &travel : chronology.travels)
        any_eligible = any_eligible || (travel.source_extrusion.handle() != nullptr &&
            planner.avoidance_enabled(travel.source, travel.target));
    CHECK(any_eligible == expected);
}

TEST_CASE("Connector preparation snaps a stationary deposit",
          "[plugins][layer-extrusion-edit][travel][deposit][chronology]")
{
    using namespace slic3r_api::LayerExtrusionEdit::TravelConnection;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].layers.resize(1);
    PrintingLayerGroup &layer = plan.groups[0].layers[0];
    layer.print_z = scale_i(0.2);
    layer.tool_groups.resize(1);
    PrintingToolGroup &tool = layer.tool_groups[0];
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(Point(0, 0), Point(scale_i(1.), 0)));
    std::unique_ptr<ExtrusionPath> deposit = make_path(Point(0, 0), Point(1, 0));
    deposit->polyline() = ArcPolyline();
    deposit->polyline().append(Point(scale_i(1.) + SCALED_EPSILON - 1, 0));
    deposit->add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.5));
    append_extrusion(tool, std::move(deposit));
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});
    ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    slic3r_api::PluginProgress progress;
    setup_connector(print, connector, progress);
    slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
    LayerTravelChronology chronology = connector.prepare(view, 0, 0);
    CHECK(chronology.travels.empty());
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target =
        ordered_scope(*tool.extrusions[1].root);
    REQUIRE(target.content().is_deposit());
    CHECK(target.content().point(0).x == scale_i(1.));
    CHECK(target.content().z_offset(0) == 0);
    CHECK(target.content().get(slic3r_api::EPropertySpecialCommand::key)->extra_data == 0.5);
    connector.apply(chronology, straight_path, progress);
    CHECK(progress.done() == 2);
    CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(
        *target.root().get(scope_property_key()),
        slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
    CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(
        *target.root().get(scope_property_key()),
        slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
}

TEST_CASE("Prepared connector validates stored travels and planner endpoints",
          "[plugins][layer-extrusion-edit][travel][chronology]")
{
    using namespace slic3r_api::LayerExtrusionEdit::TravelConnection;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.resize(1);
    plan.groups[0].layers.resize(1);
    PrintingLayerGroup &layer = plan.groups[0].layers[0];
    layer.print_z = scale_i(0.2);
    layer.tool_groups.resize(1);
    PrintingToolGroup &tool = layer.tool_groups[0];
    tool.extruder_id = 0;
    append_extrusion(tool, make_path(Point(0, 0), Point(scale_i(1.), 0)));
    append_extrusion(tool, make_path(Point(scale_i(3.), 0), Point(scale_i(4.), 0)));
    run_layer_edit_plugins(print, {TRANSITION_SCOPE_PLUGIN, ENTRY_STATE_PLUGIN});
    ScopeTravelConnector connector(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
    slic3r_api::PluginProgress progress;
    setup_connector(print, connector, progress);
    slic3r_api::PrintingLayerGroup view(reinterpret_cast<printing_layer_group_handle *>(&layer));
    const slic3r_api::LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope target =
        ordered_scope(*tool.extrusions[1].root);
    SECTION("Wrong planner endpoints fail during application") {
        LayerTravelChronology chronology = connector.prepare(view, 0, 0);
        REQUIRE(chronology.travels.size() == 1);
        CHECK_THROWS_WITH(connector.apply(chronology,
            [](const TravelEndpoint &, const TravelEndpoint &, extruder_index_t) {
                return std::vector<c_point>{{0, 0}, {1, 0}};
            }, progress), "A travel planner must preserve both endpoints.");
        CHECK(target.travel().point_count() == 0);
        CHECK(progress.done() == 1);
    }
    SECTION("Malformed stored phase is checked even when marked") {
        REQUIRE(target.travel().set_points(std::vector<c_point>{{scale_i(1.), 0}, {scale_i(3.), 0}}));
        slic3r_api::printing_extrusion_scope_set_flag(*target.root().get_mutable(scope_property_key()),
            slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED, true);
        CHECK_THROWS_WITH(connector.prepare(view, 0, 0),
            "A non-empty travel phase must carry the Travel role.");
        CHECK(progress.done() == 0);
    }
    SECTION("Stored unmarked travel is retained and flags are published") {
        REQUIRE(target.travel().set_points(std::vector<c_point>{{scale_i(1.), 0}, {scale_i(3.), 0}}));
        target.travel().get_or_add(slic3r_api::EPropertyAttributes::key)
            .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL);
        LayerTravelChronology chronology = connector.prepare(view, 0, 0);
        CHECK(chronology.travels.empty());
        connector.apply(chronology, straight_path, progress);
        CHECK(progress.done() == 2);
        CHECK(slic3r_api::printing_extrusion_scope_has_flag(
            *target.root().get(scope_property_key()),
            slic3r_api::PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED));
        CHECK_FALSE(slic3r_api::printing_extrusion_scope_has_flag(
            *target.root().get(scope_property_key()),
            slic3r_api::PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED));
        LayerTravelChronology marked = connector.prepare(view, 0, 0);
        CHECK(marked.travels.empty());
    }
}
