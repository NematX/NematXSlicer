#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
#include <tuple>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/ExtrusionPropertyAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_printing_plan.h"
#include "libslic3r/Api/plugin/c/slic3r_volume.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_extrusion_edit.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/AuxiliaryLayerHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/GCode/PrintExtents.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Plugins/Ordering/ObjectInstanceCatalog.hpp"
#include "libslic3r/Plugins/Ordering/PlanBuilderCommon.hpp"
#include "libslic3r/Plugins/Ordering/IslandStacks.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Printing/PrintingPlanValidation.hpp"
#include "libslic3r/ExtrusionEntityCanonical.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "test_data.hpp"

/*
PrintingPlan tests build small ordered trees and source layers to verify cloning,
scope events and plugin ordering. A deposit's one stored position follows the
same instance translation as segmented leaves while the source stays unchanged.
Finalization probes run each tree orderer, inspect every event scope and restore
checkpoints. A late provider observes live permissions, appends work and can fail;
only a successful complete chain freezes the resulting plan. Reversal probes
verify the full authorized unit, including fixed descendants and command phases.
Native-reader probes preserve warmed child-pointer views and stable identities
through ordering, and compare extents using inherited widths across siblings.
SEMM probes distinguish repeated visits from distinct tools, including empty
visits introduced for height-marker events.
Common-builder probes select source islands and half-open Z intervals, then check
the cloned roots and source context for the selected object instance.
Island-stack probes construct ordered source islands directly to exercise bridge
merges, tool-independent grouping, lower links and empty extrusion selections.
Plate scenes compare source role coverage, translations and parent order against
the resulting plan. For the rectangular scenes, bounding-box distance is the exact
slice distance; a forward graph evaluation independently bounds every prior stack's
height when lower work is printed. No stack algorithm state is used by these checks.
History scenes replace sliced layers with rectangular source graphs after releasing
the original plan, then invoke the same registered builder and independent checks.

    Island-stack tests -> build synthetic layers and clearance graphs
        add_stack_test_extrusion() -> give selected islands printable tool buckets
        IslandStacks::separate_islands() -> inspect borrowed stack membership
        island_stack_test_plan() -> inspect emitted tool groups in cloned selections
    Plate and owned-auxiliary tests -> inspect owner-only cloned paths
        run_island_plan_builder() -> invoke By object without later orderers
        check_island_plan_oracles() -> check coverage, parents and collision lead
        append_stack_scene_layer() -> construct owned rectangular source graphs
*/
namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using slic3r_api::PluginPropertyKey;

// Borrow the complete function table with its byte count; optional cleanup/preference functions are null, and no changes or failures occur.
const plugin_vtable *plan_event_editor_vtable();
// Ignore borrowed initialization arguments and return success without state changes or failure.
int32_t plan_event_editor_initialize(void *, storage_handle *);

static_assert(sizeof(extruder_index_t) == sizeof(int16_t));
static_assert(std::is_signed<extruder_index_t>::value);
static_assert(EXTRUDER_ID_UNKNOWN == extruder_index_t(-1));

/* Small typed marker used to verify every PrintingPlan scope property view. */
struct TestPrintingScopePayload
{
    uint32_t value = 0;
};

PluginPropertyKey<TestPrintingScopePayload> printing_scope_property_key(
    orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<TestPrintingScopePayload>::register_dynamic(
        orchestrator, "tests.printing-plan.scope-properties");
}

/* Set one registered payload on a plan scope without transferring its layout. */
void set_test_scope_value(PluginPropertyContainer &properties,
                          const PluginPropertyKey<TestPrintingScopePayload> &key,
                          const uint32_t value)
{
    void *data = properties.get_or_add_property_data_mutable(
        key.type(), sizeof(TestPrintingScopePayload), alignof(TestPrintingScopePayload));
    REQUIRE(data != nullptr);
    reinterpret_cast<TestPrintingScopePayload *>(data)->value = value;
}

/* Read the fixture payload from one plan scope after a clone or restore. */
uint32_t test_scope_value(const PluginPropertyContainer &properties,
                          const PluginPropertyKey<TestPrintingScopePayload> &key)
{
    const void *data = properties.property_data(key.type());
    REQUIRE(data != nullptr);
    return data == nullptr ? 0 : reinterpret_cast<const TestPrintingScopePayload *>(data)->value;
}

PrintingToolGroup test_tool_group(const extruder_index_t extruder_id, const uint16_t marker)
{
    PrintingToolGroup group;
    group.extruder_id = extruder_id;

    /*
    PrintingToolGroup has no standalone debug id. Store a default
    PrintingExtrusion with a marker in printing_catalog_id so the tests can
    verify that groups with the same extruder keep their relative order after
    being moved.
    */
    PrintingExtrusion extrusion;
    extrusion.printing_catalog_id = marker;
    group.extrusions.push_back(std::move(extrusion));
    return group;
}

PrintingLayerGroup test_layer(std::initializer_list<extruder_index_t> extruders)
{
    PrintingLayerGroup layer;
    uint16_t marker = 1;
    for (const extruder_index_t extruder_id : extruders)
        layer.tool_groups.push_back(test_tool_group(extruder_id, marker++));
    return layer;
}

std::vector<extruder_index_t> extruder_order(const PrintingLayerGroup &layer)
{
    std::vector<extruder_index_t> out;
    out.reserve(layer.tool_groups.size());
    for (const PrintingToolGroup &tool_group : layer.tool_groups)
        out.push_back(tool_group.extruder_id);
    return out;
}

std::vector<uint16_t> marker_order(const PrintingLayerGroup &layer)
{
    std::vector<uint16_t> out;
    out.reserve(layer.tool_groups.size());
    for (const PrintingToolGroup &tool_group : layer.tool_groups) {
        REQUIRE_FALSE(tool_group.extrusions.empty());
        out.push_back(tool_group.extrusions.front().printing_catalog_id);
    }
    return out;
}

ExtrusionEntityUPtr test_path(std::initializer_list<Point> points, const bool can_reverse = true)
{
    return std::make_unique<ExtrusionEntity>(can_reverse, ArcPolyline(Points(points)));
}

ExPolygon rectangle_expolygon(const double min_x, const double min_y, const double max_x, const double max_y)
{
    return ExPolygon(Polygon({
        Point(scale_i(min_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(max_y)),
        Point(scale_i(min_x), scale_i(max_y))
    }));
}

slic3r_api::Print print_view(Print &print)
{
    return slic3r_api::Print(reinterpret_cast<const print_handle *>(&print));
}

slic3r_api::ExPolygonCollection expolygons_view(const ExPolygons &expolygons)
{
    return slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&expolygons));
}

std::unique_ptr<ExtrusionEntityCollection> test_root(const bool can_sort = true, const bool can_reverse = true)
{
    return std::make_unique<ExtrusionEntityCollection>(can_sort, can_reverse);
}

class TestLoopEntryAnalysis final : public LoopEntryAnalysis
{
public:
    TestLoopEntryAnalysis(const ExtrusionEntity &loop_root,
                          const Point preferred_point,
                          const bool has_preferred_point,
                          const Point omitted_point,
                          const bool has_omitted_point)
        : m_preferred_point(preferred_point)
        , m_has_preferred_point(has_preferred_point)
    {
        loop_root.collect_points(m_points);
        if (has_omitted_point)
            m_points.erase(std::remove(m_points.begin(), m_points.end(), omitted_point), m_points.end());
    }

    size_t candidate_count() const override { return m_points.size(); }

    Point candidate_point(const size_t candidate_idx) const override
    {
        assert(candidate_idx < m_points.size());
        return candidate_idx < m_points.size() ? m_points[candidate_idx] : Point();
    }

    double score_candidate(const size_t candidate_idx, const Point &start_near) const override
    {
        assert(candidate_idx < m_points.size());
        if (candidate_idx >= m_points.size())
            return (std::numeric_limits<double>::max)();
        if (m_has_preferred_point && m_points[candidate_idx] == m_preferred_point)
            return -1.0;
        return start_near.distance_to_square(m_points[candidate_idx]);
    }

    void rotate_loop_to_candidate(ExtrusionEntity &loop_root, const size_t candidate_idx) const override
    {
        REQUIRE(candidate_idx < m_points.size());
        ArcPolyline *polyline = loop_root.polyline_or_null();
        REQUIRE(polyline != nullptr);

        size_t point_idx = size_t(-1);
        for (size_t idx = 0; idx < polyline->size(); ++idx)
            if (polyline->get_point(idx) == m_points[candidate_idx]) {
                point_idx = idx;
                break;
            }

        REQUIRE(point_idx != size_t(-1));
        if (point_idx == 0 || (point_idx == polyline->size() - 1 && polyline->front() == polyline->back()))
            return;

        ArcPolyline before_entry;
        ArcPolyline after_entry;
        REQUIRE(polyline->split_at_index(point_idx, before_entry, after_entry));
        after_entry.append(std::move(before_entry));
        polyline->swap(after_entry);
    }

private:
    Points m_points;
    Point m_preferred_point;
    bool m_has_preferred_point = false;
};

class TestLoopEntryPolicy final : public LoopEntryPolicy
{
public:
    Point preferred_point;
    bool has_preferred_point = false;
    Point omitted_point;
    bool has_omitted_point = false;

    mutable const ExtrusionEntity *seen_loop_root = nullptr;
    mutable const ExtrusionEntity *seen_context_root = nullptr;
    mutable const LayerRegionIsland *seen_region_island = nullptr;
    mutable ExtrusionRole seen_role = ExtrusionRole::None;

    std::unique_ptr<LoopEntryAnalysis> analyze_loop(const ExtrusionEntity &loop_root,
                                                    const LoopEntryContext &context) const override
    {
        seen_loop_root = &loop_root;
        seen_context_root = context.root;
        seen_region_island = context.region_island;
        seen_role = context.role;
        return std::make_unique<TestLoopEntryAnalysis>(loop_root,
                                                       preferred_point,
                                                       has_preferred_point,
                                                       omitted_point,
                                                       has_omitted_point);
    }
};

// Preserve the process-wide active plugin list while a runner test selects
// only its local probes.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids)
        : m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        m_orchestrator.reset_plugin_cancel();
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
        m_orchestrator.reset_plugin_cancel();
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

struct PlanEventEditorState
{
    const char *id = nullptr;
    int32_t priority = 0;
    uint32_t expected_event_count = 0;
    std::atomic_uint32_t setup_count{0};
    std::atomic_uint32_t setup_run_count{0};
    std::atomic_uint32_t run_count{0};
    std::atomic_bool payload_valid{true};
};

PlanEventEditorState g_first_plan_event_editor{"test.extrusion_edit.events.first", -10, 0};
PlanEventEditorState g_second_plan_event_editor{"test.extrusion_edit.events.second", 10, 1};
PlanEventEditorState g_ordering_boundary_probe{"test.ordering.boundary.probe", 20000, 0};
bool g_ordering_boundary_fail = false;

const char *plan_event_editor_id(void *ctx) { return static_cast<PlanEventEditorState *>(ctx)->id; }
const char *plan_event_editor_name(void *ctx) { return static_cast<PlanEventEditorState *>(ctx)->id; }
const char *plan_event_editor_description(void *) { return "Records global plan event scheduling."; }
const char *plan_event_editor_group(void *) { return ""; }
slicing_step_t plan_event_editor_step(void *) { return STEP_EXTRUSION_EDIT; }
int32_t plan_event_editor_priority(void *ctx) { return static_cast<PlanEventEditorState *>(ctx)->priority; }
const_strings_t plan_event_editor_dependencies(void *) { return {}; }
int32_t plan_event_editor_used_keys(void *, raw_used_config_key *) { return 0; }
int32_t plan_event_editor_defined_keys(void *, const char **) { return 0; }
int32_t plan_event_editor_initialize(void *, storage_handle *) { return PLUGIN_ERROR_OK; }

void plan_event_editor_setup(void *ctx, const plugin_run_context *run_ctx, uint32_t run_count)
{
    PlanEventEditorState &state = *static_cast<PlanEventEditorState *>(ctx);
    const run_ctx_extrusion_edition *payload = plugin_ctx_as_extrusion_edition(run_ctx);
    state.payload_valid = payload != nullptr && payload->print != nullptr && payload->plan != nullptr &&
                          run_count == 1;
    ++state.setup_count;
}

void plan_event_editor_setup_run(void *ctx, const plugin_run_context *run_ctx)
{
    PlanEventEditorState &state = *static_cast<PlanEventEditorState *>(ctx);
    const run_ctx_extrusion_edition *payload = plugin_ctx_as_extrusion_edition(run_ctx);
    if (payload == nullptr || payload->plan == nullptr)
        state.payload_valid = false;
    ++state.setup_run_count;
}

void plan_event_editor_run(void *ctx, const plugin_run_context *run_ctx)
{
    PlanEventEditorState &state = *static_cast<PlanEventEditorState *>(ctx);
    const run_ctx_extrusion_edition *payload = plugin_ctx_as_extrusion_edition(run_ctx);
    if (payload == nullptr || payload->plan == nullptr) {
        state.payload_valid = false;
        return;
    }

    printing_scope_events_handle *events = printing_plan_get_events_mutable(payload->plan);
    const extrusion_entity_handle *before = printing_scope_events_get_before(events);
    if (before == nullptr || extrusion_child_count(before) != state.expected_event_count)
        state.payload_valid = false;

    // Each probe appends one event. The second probe's expected count proves
    // that lower-priority plugins complete before the next plugin starts.
    ExtrusionEntity event(true);
    if (printing_scope_events_append_before_clone(
            events, reinterpret_cast<const extrusion_entity_handle *>(&event)) == nullptr)
        state.payload_valid = false;
    ++state.run_count;
}

const plugin_vtable *plan_event_editor_vtable()
{
    static const plugin_vtable table = {
        sizeof(plugin_vtable),
        &plan_event_editor_id,
        &plan_event_editor_name,
        &plan_event_editor_description,
        &plan_event_editor_group,
        &plan_event_editor_step,
        &plan_event_editor_dependencies,
        &plan_event_editor_priority,
        &plan_event_editor_used_keys,
        &plan_event_editor_defined_keys,
        &plan_event_editor_initialize,
        &plan_event_editor_setup,
        &plan_event_editor_setup_run,
        &plan_event_editor_run
    };
    return &table;
}

void register_plan_event_editors()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    PlanEventEditorState *states[] = {&g_first_plan_event_editor, &g_second_plan_event_editor};
    for (PlanEventEditorState *state : states) {
        if (orchestrator.get_plugin(state->id) != nullptr)
            continue;
        plugin_instance instance = {state, plan_event_editor_vtable()};
        REQUIRE(orchestrator.register_plugin(instance));
    }
}

void reset_plan_event_editor(PlanEventEditorState &state)
{
    state.setup_count = 0;
    state.setup_run_count = 0;
    state.run_count = 0;
    state.payload_valid = true;
}

void append_test_layer(slic3r_api::PrintingGroup group,
                       const std::vector<extruder_index_t> &extruder_ids,
                       const coord_t print_z)
{
    slic3r_api::PrintingLayerGroup layer = group.append_layer_group(print_z);
    for (const extruder_index_t extruder_id : extruder_ids)
        layer.append_tool_group(extruder_id);
}

void run_default_tool_group_ordering_plugin(Print &print)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("ordering.tool_groups.default");
    REQUIRE(plugin != nullptr);

    PrintingPlan &native_plan = print.mutable_printing_plan();
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&native_plan);

    plugin_host_context host_context =
        orchestrator.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    plugin_run_context run_context =
        orchestrator.prepare_plugin_run_context(STEP_ORDERING, plugin, &host_context);
    run_context.data = &payload;

    plugin->setup(run_context, 1);
    plugin->setup_run(run_context);
    plugin->run(run_context);
}

std::vector<std::vector<extruder_index_t>> default_plugin_extruder_order(
    const std::vector<std::vector<extruder_index_t>> &layers)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &native_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&native_plan));
    slic3r_api::PrintingGroup group = plan_view.append_group();

    coord_t print_z = 0;
    for (const std::vector<extruder_index_t> &layer : layers) {
        append_test_layer(group, layer, print_z);
        ++print_z;
    }

    run_default_tool_group_ordering_plugin(print);

    REQUIRE(native_plan.groups.size() == 1);
    std::vector<std::vector<extruder_index_t>> out;
    out.reserve(native_plan.groups.front().layers.size());
    for (const PrintingLayerGroup &layer : native_plan.groups.front().layers)
        out.push_back(extruder_order(layer));
    return out;
}

} // namespace

TEST_CASE("PrintingPlan scopes carry mutable plugin properties",
          "[printing][plan][properties]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *orchestrator_handle_value =
        reinterpret_cast<orchestrator_handle *>(&orchestrator);
    const PluginPropertyKey<TestPrintingScopePayload> property_key =
        printing_scope_property_key(orchestrator_handle_value);
    PrintingPlan native_plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&native_plan);
    slic3r_api::PrintingPlan plan(plan_handle);

    slic3r_api::PrintingGroup group = plan.append_group();
    slic3r_api::PrintingLayerGroup layer = group.append_layer_group(42);
    slic3r_api::PrintingToolGroup tool = layer.append_tool_group(3);

    // Each view writes only its own auxiliary container. The C getters expose
    // those same borrowed containers, so C and C++ plugins observe one state.
    property_key.get_or_add(plan.properties()).value = 10;
    property_key.get_or_add(group.properties()).value = 20;
    property_key.get_or_add(layer.properties()).value = 30;
    property_key.get_or_add(tool.properties()).value = 40;

    REQUIRE(printing_plan_get_properties(plan_handle) != nullptr);
    REQUIRE(printing_group_get_properties(group.handle()) != nullptr);
    REQUIRE(printing_layer_group_get_properties(layer.handle()) != nullptr);
    REQUIRE(printing_tool_group_get_properties(tool.handle()) != nullptr);
    REQUIRE(property_key.get(plan.properties()) != nullptr);
    REQUIRE(property_key.get(group.properties()) != nullptr);
    REQUIRE(property_key.get(layer.properties()) != nullptr);
    REQUIRE(property_key.get(tool.properties()) != nullptr);
    CHECK(property_key.get(plan.properties())->value == 10);
    CHECK(property_key.get(group.properties())->value == 20);
    CHECK(property_key.get(layer.properties())->value == 30);
    CHECK(property_key.get(tool.properties())->value == 40);

    // Reordering moves complete scope values. Reacquire every child view after
    // moving its parent because vector operations invalidate borrowed handles.
    plan.append_group();
    REQUIRE(plan.move_group(0, 1));
    group = plan.group(1);
    REQUIRE(property_key.get(group.properties()) != nullptr);
    CHECK(property_key.get(group.properties())->value == 20);

    group.append_layer_group(84);
    REQUIRE(group.move_layer_group(0, 1));
    layer = group.layer_group(1);
    REQUIRE(property_key.get(layer.properties()) != nullptr);
    CHECK(property_key.get(layer.properties())->value == 30);

    layer.append_tool_group(7);
    REQUIRE(layer.move_tool_group(0, 1));
    tool = layer.tool_group(1);
    REQUIRE(property_key.get(tool.properties()) != nullptr);
    CHECK(property_key.get(tool.properties())->value == 40);

    // Metadata is meaningful content: cleanup may remove the other empty tool
    // group, but it must not silently discard the annotated one.
    CHECK_FALSE(layer.remove_empty_tool_group(1));
    CHECK(layer.remove_empty_tool_group(0));
    CHECK(layer.tool_group_count() == 1);

    // Clearing a group resets its own metadata and all descendants. Clearing
    // the plan then resets plan-wide metadata as well.
    group.clear();
    CHECK(group.properties().count() == 0);
    CHECK(group.layer_group_count() == 0);
    CHECK(plan.properties().count() == 1);

    plan.clear();
    CHECK(plan.properties().count() == 0);
    CHECK(plan.group_count() == 0);
}

TEST_CASE("PrintingPlan tool ordering keeps one-extruder layers stable", "[printing][plan]")
{
    // A layer using only one extruder cannot reduce tool changes by moving its
    // groups. The function may still rebuild the vector internally, but it must
    // keep duplicate tool groups in their original relative order.
    PrintingGroup group;
    group.layers.push_back(test_layer({2, 2}));

    order_printing_tool_groups(group);

    CHECK(extruder_order(group.layers.front()) == std::vector<extruder_index_t>({2, 2}));
    CHECK(marker_order(group.layers.front()) == std::vector<uint16_t>({1, 2}));
}

TEST_CASE("PrintingPlan tool ordering starts with requested first extruder", "[printing][plan]")
{
    // When the caller knows the active tool before this PrintingGroup, the first
    // printable layer should start with that tool if it is available. The rest
    // of the layer stays as close as possible to the original order.
    PrintingGroup group;
    group.layers.push_back(test_layer({2, 1, 3}));

    order_printing_tool_groups(group, 1);

    CHECK(extruder_order(group.layers.front()) == std::vector<extruder_index_t>({1, 2, 3}));
}

TEST_CASE("PrintingPlan tool ordering uses smallest extruder as default start", "[printing][plan]")
{
    // If no first tool is known, the smallest extruder id is only a deterministic
    // tie-break. This prevents the first layer from depending on incidental
    // source traversal order when several orders have the same tool-change cost.
    PrintingGroup group;
    group.layers.push_back(test_layer({2, 3, 1}));

    order_printing_tool_groups(group, EXTRUDER_ID_UNKNOWN);

    CHECK(extruder_order(group.layers.front()) == std::vector<extruder_index_t>({1, 2, 3}));
}

TEST_CASE("PrintingPlan tool ordering connects consecutive layers through shared tools", "[printing][plan]")
{
    // The last tool of one layer is chosen with the next layer in mind. Here the
    // first layer can end with extruder 2, allowing the second layer to start
    // with extruder 2 and avoid one tool change.
    PrintingGroup group;
    group.layers.push_back(test_layer({1, 2}));
    group.layers.push_back(test_layer({2, 3}));

    order_printing_tool_groups(group, EXTRUDER_ID_UNKNOWN);

    CHECK(extruder_order(group.layers[0]) == std::vector<extruder_index_t>({1, 2}));
    CHECK(extruder_order(group.layers[1]) == std::vector<extruder_index_t>({2, 3}));
}

TEST_CASE("PrintingPlan tool ordering minimizes transitions over several layers", "[printing][plan]")
{
    // This case needs a global choice, not a local per-layer shuffle. The best
    // sequence is 1->2, 2->3, then 3->1, which gives zero inter-layer tool
    // changes across all three printable layers.
    PrintingGroup group;
    group.layers.push_back(test_layer({1, 2}));
    group.layers.push_back(test_layer({2, 3}));
    group.layers.push_back(test_layer({1, 3}));

    order_printing_tool_groups(group, EXTRUDER_ID_UNKNOWN);

    CHECK(extruder_order(group.layers[0]) == std::vector<extruder_index_t>({1, 2}));
    CHECK(extruder_order(group.layers[1]) == std::vector<extruder_index_t>({2, 3}));
    CHECK(extruder_order(group.layers[2]) == std::vector<extruder_index_t>({3, 1}));
}

TEST_CASE("PrintingPlan tool ordering preserves duplicate tool group order", "[printing][plan]")
{
    // A layer may contain several groups with the same extruder. Ordering may
    // move all groups of an extruder together, but it must not reverse or sort
    // the groups inside that extruder bucket because they may already encode a
    // meaningful upstream sequence.
    PrintingGroup group;
    group.layers.push_back(test_layer({2, 1, 2, 3, 1}));

    order_printing_tool_groups(group, 1);

    CHECK(extruder_order(group.layers.front()) == std::vector<extruder_index_t>({1, 1, 2, 2, 3}));
    CHECK(marker_order(group.layers.front()) == std::vector<uint16_t>({2, 5, 1, 3, 4}));
}

TEST_CASE("PrintingPlan tool ordering skips empty layers in transition search", "[printing][plan]")
{
    // Empty layer groups are placeholders, not tool-change events. They should
    // remain empty and should not prevent the next printable layer from using
    // the requested first extruder.
    PrintingGroup group;
    group.layers.emplace_back();
    group.layers.push_back(test_layer({2, 1}));

    order_printing_tool_groups(group, 1);

    CHECK(group.layers.front().tool_groups.empty());
    CHECK(extruder_order(group.layers[1]) == std::vector<extruder_index_t>({1, 2}));
}

TEST_CASE("Default tool group ordering resolves unassigned tool groups",
          "[printing][plan][step-ordering]")
{
    CHECK(default_plugin_extruder_order({{EXTRUDER_ID_UNKNOWN, 2}}) ==
          std::vector<std::vector<extruder_index_t>>({{2, 2}}));

    CHECK(default_plugin_extruder_order({{1, EXTRUDER_ID_UNKNOWN, 2}}) ==
          std::vector<std::vector<extruder_index_t>>({{1, 1, 2}}));

    CHECK(default_plugin_extruder_order({{EXTRUDER_ID_UNKNOWN, EXTRUDER_ID_UNKNOWN}}) ==
          std::vector<std::vector<extruder_index_t>>({{0, 0}}));

    CHECK(default_plugin_extruder_order({{3}, {EXTRUDER_ID_UNKNOWN, 2}}) ==
          std::vector<std::vector<extruder_index_t>>({{3}, {3, 2}}));
}

TEST_CASE("STEP_ORDERING post validation rejects unresolved tool groups",
          "[printing][plan][step-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &native_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&native_plan));
    slic3r_api::PrintingGroup group = plan_view.append_group();
    append_test_layer(group, std::vector<extruder_index_t>({EXTRUDER_ID_UNKNOWN}), 0);

    std::string error;
    CHECK_FALSE(Steps::StepExtrusionOrdering::validate_post(print, &error));
    CHECK(error.find("group[0].layer[0].tool[0]") != std::string::npos);
    CHECK(error.find("no concrete extruder") != std::string::npos);
}

TEST_CASE("STEP_ORDERING post validation rejects a source whose regions are on different layers",
          "[printing][plan][step-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);

    // Two auxiliary layers of the same object, each with its own region.
    PluginStorage storage;
    const ExPolygons subject{ rectangle_expolygon(10., 10., 30., 30.) };
    std::vector<const LayerRegion *> regions;
    for (const double print_z : { 0.2, 0.4 }) {
        const slic3r_api::AuxiliaryLayerBuildResult result =
            slic3r_api::build_auxiliary_layer_regions_from_subject(
                reinterpret_cast<storage_handle *>(&storage),
                slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
                slic3r_api::Object(reinterpret_cast<const object_handle *>(&print.object(0))),
                slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&subject)),
                scale_i(0.2), scale_i(print_z), scale_i(print_z - 0.1));
        REQUIRE(result.created);
        REQUIRE(result.layer.region_count() > 0);
        regions.push_back(reinterpret_cast<const LayerRegion *>(result.layer.region(0).handle()));
    }
    REQUIRE(regions[0]->layer() != regions[1]->layer());

    // The plugin API cannot build such a source; build it natively.
    auto validation_error = [&print](const LayerRegionIsland &source) {
        PrintingPlan &native_plan = print.mutable_printing_plan();
        native_plan.groups.clear();
        slic3r_api::PrintingPlan plan_view(reinterpret_cast<printing_plan_handle *>(&native_plan));
        append_test_layer(plan_view.append_group(), std::vector<extruder_index_t>({0}), scale_i(0.2));
        PrintingExtrusion &extrusion =
            native_plan.groups.front().layers.front().tool_groups.front().extrusions.emplace_back();
        extrusion.region_island = &source;
        extrusion.root = test_path({Point(0, 0), Point(scale_i(1.), 0)});
        std::string error;
        CHECK_FALSE(Steps::StepExtrusionOrdering::validate_post(print, &error));
        return error;
    };

    const LayerRegionIsland mixed({ regions[0], regions[1] }, EXTRUDER_ID_UNKNOWN);
    const std::string mixed_error = validation_error(mixed);
    INFO(mixed_error);
    CHECK(mixed_error.find("extrusion source region island is invalid") != std::string::npos);

    // Control: a single-layer source passes this check. The plan then fails on
    // the missing object instance, which is unrelated to the source layers.
    const LayerRegionIsland single({ regions[0] }, EXTRUDER_ID_UNKNOWN);
    const std::string single_error = validation_error(single);
    INFO(single_error);
    CHECK(single_error.find("region island is invalid") == std::string::npos);
    print.mutable_printing_plan().groups.clear();
}

TEST_CASE("Native ordering reorders owned children without relying on a collection adapter",
          "[plugins][printing][plan][native-retirement]")
{
    for (const bool collection : {false, true}) {
        CAPTURE(collection);
        ExtrusionEntityUPtr root = collection ? ExtrusionEntityUPtr(new ExtrusionEntityCollection(true, false)) :
            std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), true, false, false);
        root->add_property(ExtrusionPropertyCustomGcodeText("M117 retained owner"));
        const uint64_t root_id = root->get_id();
        std::vector<const ExtrusionEntity *> original;
        std::vector<uint64_t> ids;
        for (const coord_t x : {100, 20, 60}) {
            ExtrusionEntityUPtr path = test_path({Point(x, 0), Point(x + 10, 0)}, false);
            path->set_can_rotate(false);
            path->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
            original.push_back(path.get());
            ids.push_back(path->get_id());
            root->append_child(std::move(path));
        }
        if (collection) {
            const ExtrusionEntitiesPtr &view = static_cast<const ExtrusionEntityCollection &>(*root).entities();
            REQUIRE(view.size() == 3);
            for (size_t i = 0; i < view.size(); ++i) CHECK(view[i] == original[i]);
        }
        order_extrusion_tree(*root, Point(0, 0));
        REQUIRE(root->child_count() == 3);
        const std::vector<size_t> expected{1, 2, 0};
        for (size_t i = 0; i < expected.size(); ++i) {
            const ExtrusionEntity &child = root->child(i);
            CHECK(&child == original[expected[i]]);
            CHECK(child.get_id() == ids[expected[i]]);
            CHECK_FALSE(child.can_reverse());
            CHECK_FALSE(child.can_rotate());
            REQUIRE(child.get_property<ExtrusionAttributes>() != nullptr);
            CHECK(child.get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::Perimeter);
        }
        CHECK(root->get_id() == root_id);
        CHECK_FALSE(root->can_sort());
        CHECK_FALSE(root->can_reverse());
        CHECK_FALSE(root->can_rotate());
        REQUIRE(root->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
        CHECK(root->custom_gcode_string(*root->get_property<ExtrusionPropertyCustomGcode>()) == "M117 retained owner");
        if (collection) {
            const ExtrusionEntitiesPtr &view = static_cast<const ExtrusionEntityCollection &>(*root).entities();
            REQUIRE(view.size() == 3);
            for (size_t i = 0; i < view.size(); ++i) CHECK(view[i] == original[expected[i]]);
        }
    }
}

TEST_CASE("Native loop rotation preserves warmed compatibility views and child identities",
          "[plugins][printing][plan][native-retirement]")
{
    for (const bool collection : {false, true}) {
        CAPTURE(collection);
        ExtrusionEntityUPtr loop = collection ? ExtrusionEntityUPtr(new ExtrusionEntityCollection(false, false)) :
            std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        loop->add_property(ExtrusionAttributes{ExtrusionRole::Perimeter});
        const Points square{{0, 0}, {100, 0}, {100, 100}, {0, 100}, {0, 0}};
        std::vector<const ExtrusionEntity *> children;
        std::vector<uint64_t> ids;
        for (size_t i = 1; i < square.size(); ++i) {
            ExtrusionEntityUPtr edge = test_path({square[i - 1], square[i]}, false);
            edge->set_can_rotate(false);
            children.push_back(edge.get());
            ids.push_back(edge->get_id());
            loop->append_child(std::move(edge));
        }
        if (collection) {
            const ExtrusionEntitiesPtr &cached = static_cast<const ExtrusionEntityCollection &>(*loop).entities();
            REQUIRE(cached.size() == 4);
            CHECK(cached[0] == children[0]);
        }
        const ExtrusionEntity *loop_handle = loop.get();
        const uint64_t loop_id = loop->get_id();
        ExtrusionEntityCollection outer(true, false);
        outer.append(std::move(loop));
        REQUIRE(outer.entities().size() == 1);
        order_extrusion_tree(outer, Point(100, 0));
        REQUIRE(outer.child_count() == 1);
        const ExtrusionEntity &rotated = outer.child(0);
        CHECK(&rotated == loop_handle);
        CHECK(rotated.get_id() == loop_id);
        REQUIRE(rotated.child_count() == 4);
        CHECK(rotated.ExtrusionEntity::first_point() == Point(100, 0));
        CHECK(rotated.ExtrusionEntity::last_point() == Point(100, 0));
        for (size_t i = 0; i < 4; ++i) {
            const size_t original = (i + 1) % 4;
            CHECK(&rotated.child(i) == children[original]);
            CHECK(rotated.child(i).get_id() == ids[original]);
            CHECK(rotated.child(i).polyline_ref().front() == square[original]);
            CHECK(rotated.child(i).polyline_ref().back() == square[original + 1]);
            CHECK_FALSE(rotated.child(i).can_reverse());
            CHECK_FALSE(rotated.child(i).can_rotate());
        }
        REQUIRE(rotated.get_property<ExtrusionAttributes>() != nullptr);
        CHECK(rotated.get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::Perimeter);
        CHECK_FALSE(rotated.can_rotate());
        REQUIRE(outer.entities().size() == 1);
        CHECK(outer.entities()[0] == loop_handle);
        if (collection) {
            const ExtrusionEntitiesPtr &cached = static_cast<const ExtrusionEntityCollection &>(rotated).entities();
            REQUIRE(cached.size() == 4);
            for (size_t i = 0; i < cached.size(); ++i) CHECK(cached[i] == children[(i + 1) % 4]);
        }
    }
}

TEST_CASE("Native extrusion extents use stored geometry and restore inherited widths between siblings",
          "[plugins][printing][plan][native-retirement]")
{
    const ExtrusionAttributes normal{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const ExtrusionAttributes wide{ExtrusionRole::ExternalPerimeter, ExtrusionFlow(0.2, 1.f, 0.2f)};
    const coord_t normal_radius = scale_i(0.5 * normal.width);
    const coord_t wide_radius = scale_i(0.5 * wide.width);
    for (const std::string form : {"base", "collection", "loop", "path"}) {
        CAPTURE(form);
        ExtrusionEntityUPtr root;
        if (form == "collection") root = std::make_unique<ExtrusionEntityCollection>();
        else if (form == "loop") root = std::make_unique<ExtrusionLoop>();
        else if (form == "path") root = std::make_unique<ExtrusionPath>(normal);
        else root = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), true, true, false);
        root->add_property(normal);
        root->append_child(test_path({Point::new_scale(-2., 0.), Point::new_scale(0., 0.)}));
        ExtrusionEntityUPtr nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
        nested->add_property(wide);
        nested->append_child(test_path({Point::new_scale(10., 0.), Point::new_scale(10., 2.)}));
        root->append_child(std::move(nested));
        ExtrusionEntityUPtr command = std::make_unique<ExtrusionEntity>(false);
        command->add_property(ExtrusionAttributes{ExtrusionRole::Travel, ExtrusionFlow(0., 100.f, 0.2f)});
        command->add_property(ExtrusionPropertyCustomGcodeText("M117 no local position"));
        root->append_child(std::move(command));
        root->append_child(test_path({Point::new_scale(20., 0.), Point::new_scale(20., 1.)}));
        const ExtrusionEntityUPtr canonical = root->clone_canonical();
        for (const ExtrusionEntity *view : {root.get(), canonical.get()}) {
            const BoundingBoxf bounds = get_extrusion_extents(*view);
            REQUIRE(bounds.defined);
            CHECK(bounds.min == unscale_p(Point(scale_i(-2.) - normal_radius, -wide_radius)));
            CHECK(bounds.max == unscale_p(Point(scale_i(20.) + normal_radius, scale_i(2.) + wide_radius)));
        }
    }
    ExtrusionEntity command(false);
    command.add_property(normal);
    CHECK_FALSE(get_extrusion_extents(command).defined);
    ExtrusionEntity no_attributes(false, ArcPolyline(Points{{0, 0}, {100, 0}}));
    CHECK_FALSE(get_extrusion_extents(no_attributes).defined);
}

TEST_CASE("Native extrusion extents ignore degenerate local boxes before merging",
          "[plugins][printing][plan][native-retirement]")
{
    const ExtrusionAttributes normal{ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)};
    const coord_t radius = scale_i(0.5 * normal.width);
    for (const ExtrusionRole role : {ExtrusionRole::Perimeter, ExtrusionRole::Travel}) {
        CAPTURE(role);
        ExtrusionEntityCollection source;
        source.append(ExtrusionPath(ArcPolyline(Points{Point::new_scale(-1., -1.), Point::new_scale(1., 1.)}),
                                    normal, nullptr));
        source.append(ExtrusionPath(ArcPolyline(Points{Point::new_scale(0., 100.), Point::new_scale(10., 100.)}),
                                    ExtrusionAttributes{role, ExtrusionFlow(0., 0.f, 0.2f)}, nullptr));
        REQUIRE(source.child_count() == 2);
        CHECK_FALSE(get_extrusion_extents(source.child(1)).defined);
        const ExtrusionEntityUPtr canonical = source.clone_canonical();
        for (const ExtrusionEntity *view : {static_cast<const ExtrusionEntity *>(&source),
                                            static_cast<const ExtrusionEntity *>(canonical.get())}) {
            const BoundingBoxf bounds = get_extrusion_extents(*view);
            REQUIRE(bounds.defined);
            CHECK(bounds.min == unscale_p(Point(scale_i(-1.) - radius, scale_i(-1.) - radius)));
            CHECK(bounds.max == unscale_p(Point(scale_i(1.) + radius, scale_i(1.) + radius)));
        }
    }
}

TEST_CASE("PrintingPlan extrusion ordering leaves non-sortable nodes unchanged", "[printing][plan]")
{
    // A non-sortable collection is an upstream contract: its child order may
    // encode a continuous multipath or a required process order. Ordering must
    // therefore treat it as one atomic sequence and leave its children alone.
    std::unique_ptr<ExtrusionEntityCollection> root = test_root(false, false);
    root->append(test_path({Point(100, 0), Point(110, 0)}));
    root->append(test_path({Point(0, 0), Point(10, 0)}));

    order_extrusion_tree(*root, Point(0, 0));

    CHECK(root->child(0).first_point() == Point(100, 0));
    CHECK(root->child(1).first_point() == Point(0, 0));
    CHECK_FALSE(root->can_sort());
}

TEST_CASE("PrintingPlan extrusion ordering chooses nearest sortable child", "[printing][plan]")
{
    // Sortable children are selected by their closest legal entry point. Once a
    // concrete order is chosen, the parent becomes non-sortable so later code
    // sees a fixed print sequence instead of an optimization request.
    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(test_path({Point(100, 0), Point(110, 0)}));
    root->append(test_path({Point(5, 0), Point(15, 0)}));

    order_extrusion_tree(*root, Point(0, 0));

    CHECK(root->child(0).first_point() == Point(5, 0));
    CHECK(root->child(1).first_point() == Point(100, 0));
    CHECK_FALSE(root->can_sort());
    CHECK_FALSE(root->can_reverse());
}

TEST_CASE("PrintingPlan extrusion ordering reverses only reversible open paths", "[printing][plan]")
{
    // A reversible child may be flipped when its end is closer to the current
    // position. A non-reversible child must keep its original direction even if
    // that forces a longer travel.
    std::unique_ptr<ExtrusionEntityCollection> reversible_root = test_root();
    reversible_root->append(test_path({Point(100, 0), Point(10, 0)}, true));

    order_extrusion_tree(*reversible_root, Point(0, 0));

    CHECK(reversible_root->child(0).first_point() == Point(10, 0));
    CHECK(reversible_root->child(0).last_point() == Point(100, 0));

    std::unique_ptr<ExtrusionEntityCollection> fixed_root = test_root();
    fixed_root->append(test_path({Point(100, 0), Point(10, 0)}, false));

    order_extrusion_tree(*fixed_root, Point(0, 0));

    CHECK(fixed_root->child(0).first_point() == Point(100, 0));
    CHECK(fixed_root->child(0).last_point() == Point(10, 0));
}

TEST_CASE("PrintingPlan extrusion ordering rotates leaf loops to nearest vertex", "[printing][plan]")
{
    // Loops are not reversed, but they may be rotated. The selected vertex
    // becomes both the first and last point, preserving the closed geometry
    // while avoiding a needless travel to the old arbitrary start point.
    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(test_path({Point(0, 0), Point(100, 0), Point(100, 100), Point(0, 0)}));

    order_extrusion_tree(*root, Point(100, 95));

    CHECK(root->child(0).is_loop());
    CHECK(root->child(0).first_point() == Point(100, 100));
    CHECK(root->child(0).last_point() == Point(100, 100));
}

TEST_CASE("PrintingPlan extrusion ordering lets a policy prefer a farther loop point", "[printing][plan]")
{
    // The loop policy owns the loop-entry decision. This policy gives one
    // distant vertex the best score, so ordering must use it even though the
    // default distance-only policy would start at Point(0, 0).
    TestLoopEntryPolicy policy;
    policy.preferred_point = Point(100, 100);
    policy.has_preferred_point = true;

    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(test_path({Point(0, 0), Point(100, 0), Point(100, 100), Point(0, 0)}));

    LoopEntryContext context;
    order_extrusion_tree(*root, Point(0, 0), policy, context);

    CHECK(root->child(0).is_loop());
    CHECK(root->child(0).first_point() == Point(100, 100));
    CHECK(policy.seen_context_root == root.get());
}

TEST_CASE("PrintingPlan extrusion ordering lets a policy remove a loop candidate", "[printing][plan]")
{
    // A future plugin may forbid a seam candidate because of painting, angle or
    // extrusion properties. This test models that by omitting the normally best
    // vertex; the next best available vertex must be selected instead.
    TestLoopEntryPolicy policy;
    policy.omitted_point = Point(0, 0);
    policy.has_omitted_point = true;

    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(test_path({Point(0, 0), Point(100, 0), Point(100, 100), Point(0, 0)}));

    LoopEntryContext context;
    order_extrusion_tree(*root, Point(0, 0), policy, context);

    CHECK(root->child(0).is_loop());
    CHECK(root->child(0).first_point() == Point(100, 0));
}

TEST_CASE("PrintingPlan extrusion ordering rotates composed loops without flattening", "[printing][plan]")
{
    // A loop may be represented as a non-sortable collection of several open
    // leaf paths. When the best entry point lies inside one leaf, ordering
    // splits only that leaf and rotates the child list; it must not flatten the
    // collection or lose the loop continuity.
    std::unique_ptr<ExtrusionEntityCollection> loop = test_root(false, false);
    loop->append(test_path({Point(0, 0), Point(100, 0)}));
    loop->append(test_path({Point(100, 0), Point(100, 100), Point(0, 0)}));
    REQUIRE(loop->is_loop());

    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(std::move(loop));

    order_extrusion_tree(*root, Point(100, 95));

    REQUIRE(root->child_count() == 1);
    const ExtrusionEntity &rotated_loop = root->child(0);
    CHECK(rotated_loop.is_loop());
    CHECK(rotated_loop.first_point() == Point(100, 100));
    CHECK(rotated_loop.last_point() == Point(100, 100));
    CHECK(rotated_loop.child_count() == 3);
}

TEST_CASE("PrintingPlan extrusion ordering preserves intermediate node properties", "[printing][plan]")
{
    // Ordering moves owned subtrees; it must not clone children into a flatter
    // shape or drop properties carried by intermediate collection nodes. Those
    // properties are how later processing stages inherit print state.
    std::unique_ptr<ExtrusionEntityCollection> nested = test_root(false, false);
    nested->add_property(ExtrusionPropertySpeed(42.f));
    nested->append(test_path({Point(20, 0), Point(30, 0)}));

    std::unique_ptr<ExtrusionEntityCollection> root = test_root();
    root->append(test_path({Point(100, 0), Point(110, 0)}));
    root->append(std::move(nested));

    order_extrusion_tree(*root, Point(0, 0));

    REQUIRE(root->child_count() == 2);
    CHECK(root->child(0).get_property<ExtrusionPropertySpeed>() != nullptr);
    CHECK(root->child(0).first_point() == Point(20, 0));
}

TEST_CASE("PrintingPlan group extrusion ordering propagates current position", "[printing][plan]")
{
    // The group overload keeps the higher-level print order unchanged, but the
    // end of one root becomes the start hint for the next root. This lets the
    // second root prefer work near the first root's final point.
    PrintingGroup group;
    group.layers.emplace_back();
    group.layers.front().tool_groups.push_back(test_tool_group(1, 1));
    PrintingToolGroup &tool_group = group.layers.front().tool_groups.front();
    tool_group.extrusions.clear();

    PrintingExtrusion first_extrusion;
    first_extrusion.root = test_root();
    static_cast<ExtrusionEntityCollection *>(first_extrusion.root.get())->append(
        test_path({Point(0, 0), Point(100, 0)}, false));
    tool_group.extrusions.push_back(std::move(first_extrusion));

    PrintingExtrusion second_extrusion;
    second_extrusion.root = test_root();
    static_cast<ExtrusionEntityCollection *>(second_extrusion.root.get())->append(
        test_path({Point(1, 0), Point(2, 0)}, false));
    static_cast<ExtrusionEntityCollection *>(second_extrusion.root.get())->append(
        test_path({Point(101, 0), Point(102, 0)}, false));
    tool_group.extrusions.push_back(std::move(second_extrusion));

    order_extrusion_tree(group, Point(0, 0));

    REQUIRE(tool_group.extrusions.size() == 2);
    REQUIRE(tool_group.extrusions[1].root != nullptr);
    CHECK(tool_group.extrusions[1].root->child(0).first_point() == Point(101, 0));
}

TEST_CASE("PrintingPlan group extrusion ordering passes source context to loop policy", "[printing][plan]")
{
    // The group overload is the bridge between a PrintingPlan and loop-entry
    // scoring. It must pass the source LayerRegionIsland and role bucket so a
    // later plugin-backed policy can read region settings or source properties.
    const LayerRegionIsland *region_island =
        reinterpret_cast<const LayerRegionIsland *>(uintptr_t(0x1234));
    TestLoopEntryPolicy policy;

    PrintingGroup group;
    group.layers.emplace_back();
    group.layers.front().tool_groups.push_back(test_tool_group(1, 1));
    PrintingToolGroup &tool_group = group.layers.front().tool_groups.front();
    tool_group.extrusions.clear();

    PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    extrusion.root = test_root();
    static_cast<ExtrusionEntityCollection *>(extrusion.root.get())->append(
        test_path({Point(0, 0), Point(100, 0), Point(100, 100), Point(0, 0)}));
    tool_group.extrusions.push_back(std::move(extrusion));

    order_extrusion_tree(group, Point(10, 0), policy);

    CHECK(policy.seen_region_island == region_island);
    CHECK(policy.seen_role == ExtrusionRole::Perimeter);
    REQUIRE_FALSE(tool_group.extrusions.empty());
    CHECK(policy.seen_context_root == tool_group.extrusions.front().root.get());
    CHECK(policy.seen_loop_root == &tool_group.extrusions.front().root->child(0));
}

TEST_CASE("PrintingPlan C API builds and mutates plan objects", "[printing][plan][api]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    // Ordering plugins see only opaque C handles. This test exercises that ABI
    // directly on a local PrintingPlan so a future external plugin can rely on
    // the same append/read/move primitives without including native plan types.
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model,
                     DynamicPrintConfig::full_print_config(), false, 3);
    PluginStorage storage;
    storage_handle *storage_handle_value = reinterpret_cast<storage_handle *>(&storage);
    PrintingPlan plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&plan);
    printing_group_handle *group_handle = printing_plan_append_group(plan_handle);
    REQUIRE(group_handle != nullptr);

    const uint32_t printing_catalog_id = printing_plan_register_object_instance(
        plan_handle, reinterpret_cast<const object_handle *>(&print.object(0)), 2, nullptr, nullptr);
    REQUIRE(printing_catalog_id == 0);
    CHECK(printing_plan_register_object_instance(
              plan_handle, reinterpret_cast<const object_handle *>(&print.object(0)), 2, nullptr, nullptr) == printing_catalog_id);
    REQUIRE(printing_group_append_printing_catalog_id(plan_handle, group_handle, printing_catalog_id) == 1);
    CHECK(printing_group_count_object_instance(group_handle) == 1);
    CHECK(printing_group_get_printing_catalog_id(group_handle, 0) == printing_catalog_id);
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::
        populate_object_instance_catalog(
            reinterpret_cast<const print_handle *>(&print), plan_handle, storage_handle_value);
    CHECK(printing_group_get_printing_catalog_id(group_handle, 0) == printing_catalog_id);
    CHECK(printing_plan_register_object_instance(
              plan_handle, reinterpret_cast<const object_handle *>(&print.object(0)), 2, nullptr, nullptr) == printing_catalog_id);
    CHECK(printing_plan_register_object_instance(
              plan_handle, reinterpret_cast<const object_handle *>(&print.object(0)), 1, nullptr, nullptr) == 1);

    printing_layer_group_handle *layer_handle = printing_group_append_layer_group(group_handle, 42);
    REQUIRE(layer_handle != nullptr);
    CHECK(printing_layer_group_get_print_z(layer_handle) == 42);

    printing_tool_group_handle *tool_handle = printing_layer_group_append_tool_group(layer_handle, 3);
    REQUIRE(tool_handle != nullptr);
    CHECK(printing_tool_group_get_extruder_id(tool_handle) == 3);

    const layer_region_island_handle *fake_region_island =
        reinterpret_cast<const layer_region_island_handle *>(uintptr_t(0x2345));
    printing_tool_group_append_region_island(tool_handle, fake_region_island);
    CHECK(printing_tool_group_get_region_island(tool_handle, 0) == fake_region_island);

    ExtrusionEntity source(true);
    source.append_child(test_path({Point(0, 0), Point(10, 0)}));
    const extrusion_entity_handle *source_handle = reinterpret_cast<const extrusion_entity_handle *>(&source);
    printing_extrusion_handle *extrusion_handle = printing_tool_group_append_extrusion_clone(
        plan_handle, tool_handle,
        nullptr,
        RAW_EXTRUSION_ROLE_PERIMETER,
        source_handle,
        PRINTING_OBJECT_INSTANCE_ID_NONE);
    REQUIRE(extrusion_handle != nullptr);

    CHECK(printing_tool_group_count_extrusion(tool_handle) == 1);
    CHECK(printing_extrusion_get_region_island(extrusion_handle) == nullptr);
    CHECK(printing_extrusion_get_role(extrusion_handle) == RAW_EXTRUSION_ROLE_PERIMETER);
    CHECK(printing_extrusion_get_printing_catalog_id(extrusion_handle) ==
          PRINTING_OBJECT_INSTANCE_ID_NONE);

    extrusion_entity_handle *clone_root = printing_extrusion_get_root_mutable(extrusion_handle);
    REQUIRE(clone_root != nullptr);
    CHECK(clone_root != source_handle);
    CHECK(extrusion_child_count(clone_root) == 1);
    CHECK(source.child_count() == 1);
}

TEST_CASE("Extruder index boundaries stay explicit across PrintingPlan APIs",
          "[printing][plan][api][extruder-index]")
{
    PrintingPlan native_plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&native_plan);
    printing_group_handle *group_handle = printing_plan_append_group(plan_handle);
    REQUIRE(group_handle != nullptr);
    printing_layer_group_handle *layer_handle = printing_group_append_layer_group(group_handle, 0);
    REQUIRE(layer_handle != nullptr);

    const extruder_index_t maximum = (std::numeric_limits<extruder_index_t>::max)();
    for (const extruder_index_t extruder_id : {
             EXTRUDER_ID_UNKNOWN, extruder_index_t(0), maximum}) {
        printing_tool_group_handle *tool_handle =
            printing_layer_group_append_tool_group(layer_handle, extruder_id);
        REQUIRE(tool_handle != nullptr);
        CHECK(printing_tool_group_get_extruder_id(tool_handle) == extruder_id);
    }
    CHECK(printing_layer_group_append_tool_group(layer_handle, extruder_index_t(-2)) == nullptr);

    printing_tool_group_handle *setter_handle =
        printing_layer_group_append_tool_group(layer_handle, maximum);
    REQUIRE(setter_handle != nullptr);
    printing_tool_group_set_extruder_id(setter_handle, extruder_index_t(-2));
    CHECK(printing_tool_group_get_extruder_id(setter_handle) == maximum);
    printing_tool_group_set_extruder_id(setter_handle, EXTRUDER_ID_UNKNOWN);
    CHECK(printing_tool_group_get_extruder_id(setter_handle) == EXTRUDER_ID_UNKNOWN);

    slic3r_api::PrintingLayerGroup layer(layer_handle);
    slic3r_api::PrintingToolGroup tool = layer.append_tool_group(0);
    REQUIRE(tool.valid());
    tool.set_extruder_id(EXTRUDER_ID_UNKNOWN);
    CHECK(tool.extruder_id() == EXTRUDER_ID_UNKNOWN);
    tool.set_extruder_id(maximum);
    CHECK(tool.extruder_id() == maximum);
    CHECK_THROWS_AS(tool.set_extruder_id(-2), std::out_of_range);
    CHECK_THROWS_AS(tool.set_extruder_id(int64_t(maximum) + 1), std::out_of_range);
    CHECK(tool.extruder_id() == maximum);
}

TEST_CASE("Plugin object names and mesh projections distinguish empty results from errors",
          "[printing][plan][api][object-instance]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);

    // The print-level auxiliary Object is a valid API view but has no
    // ModelObject from which a user-facing name could be copied.
    PrintObject &auxiliary = print.mutable_auxiliary_object();
    const object_handle *auxiliary_handle =
        reinterpret_cast<const object_handle *>(&auxiliary);
    CHECK(object_get_name(auxiliary_handle, nullptr, 0) == 0);
    CHECK(slic3r_api::Object(auxiliary_handle).name().empty());

    PluginStorage storage;
    storage_handle *storage_handle_value = reinterpret_cast<storage_handle *>(&storage);
    const Slic3r::TriangleMesh &native_mesh =
        print.object(0).model_object()->volumes.front()->mesh();
    const triangle_mesh_handle *mesh_handle =
        reinterpret_cast<const triangle_mesh_handle *>(&native_mesh);
    c_matrix4d invalid_transform = {};
    invalid_transform.value[0] = std::numeric_limits<double>::quiet_NaN();

    CHECK(triangle_mesh_project_to_polygons(
              storage_handle_value, mesh_handle, invalid_transform) == nullptr);
    const slic3r_api::TriangleMesh mesh(mesh_handle);
    CHECK_THROWS_AS(
        mesh.project_to_polygons(storage_handle_value, invalid_transform),
        std::invalid_argument);
}

TEST_CASE("Object instance catalog falls back to positive geometry when subtraction is empty",
          "[printing][plan][object-instance][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);

    // A negative volume with the same XY projection may remove the complete
    // projected material even when printable geometry exists at other Zs. The
    // catalog therefore uses the positive model-part hull as its stable label
    // outline, matching the existing LabelObjects policy.
    ModelObject *model_object = const_cast<ModelObject *>(print.object(0).model_object());
    REQUIRE(model_object != nullptr);
    REQUIRE_FALSE(model_object->volumes.empty());
    model_object->add_volume(
        model_object->volumes.front()->mesh(), ModelVolumeType::NEGATIVE_VOLUME, false);

    PrintingPlan plan;
    REQUIRE(plan.find_or_add_object_instance(print.objects().front(), 0) == 0);
    PluginStorage storage;
    CHECK_NOTHROW(
        slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
            reinterpret_cast<const print_handle *>(&print),
            reinterpret_cast<printing_plan_handle *>(&plan),
            reinterpret_cast<storage_handle *>(&storage)));
    REQUIRE(plan.object_instances.size() == 1);
    CHECK_FALSE(plan.object_instances.front().projected_outline.empty());
}

TEST_CASE("PrintingPlan object instance ids remain stable after plan reordering",
          "[printing][plan][object-instance]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20,
                      Test::TestMesh::cube_20x20x20},
                     print, model, DynamicPrintConfig::full_print_config(), false, 2);

    PrintingPlan plan = build_printing_plan_by_object(print);
    REQUIRE(plan.object_instances.size() == 4);
    for (const PrintingObjectInstance &instance : plan.object_instances) {
        CHECK(instance.name.empty());
        CHECK(instance.projected_outline.empty());
    }
    PluginStorage storage;
    slic3r_api::Ordering::ObjectInstanceCatalogPlugin::populate_object_instance_catalog(
        reinterpret_cast<const print_handle *>(&print),
        reinterpret_cast<printing_plan_handle *>(&plan),
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(plan.object_instances.size() == 4);
    REQUIRE(plan.groups.size() == 4);
    for (size_t index = 0; index < plan.object_instances.size(); ++index) {
        const PrintingObjectInstance &instance = plan.object_instances[index];
        CHECK_FALSE(instance.name.empty());
        CHECK_FALSE(instance.projected_outline.empty());
        REQUIRE(plan.groups[index].printing_catalog_ids.size() == 1);
        CHECK(plan.groups[index].printing_catalog_ids.front() == index);
    }

    std::reverse(plan.groups.begin(), plan.groups.end());
    CHECK(plan.groups[0].printing_catalog_ids.front() == 3);
    CHECK(plan.groups[1].printing_catalog_ids.front() == 2);
    CHECK(plan.groups[2].printing_catalog_ids.front() == 1);
    CHECK(plan.groups[3].printing_catalog_ids.front() == 0);

    const printing_plan_handle *handle =
        reinterpret_cast<const printing_plan_handle *>(&plan);
    CHECK(printing_plan_count_object_instance(handle) == 4);
    const c_printing_object_instance second =
        printing_plan_get_object_instance_by_printing_catalog_id(handle, 2);
    CHECK(second.id == 2);
    CHECK(second.name != nullptr);
    CHECK(second.projected_outline != nullptr);

    printing_plan_clear(const_cast<printing_plan_handle *>(handle));
    CHECK(printing_plan_count_group(handle) == 0);
    CHECK(printing_plan_count_object_instance(handle) == 0);

    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(print.printing_plan()->object_instances.size() == 4);
}

TEST_CASE("PrintingPlan C API moves extrusion content into the plan", "[printing][plan][api]")
{
    // The move API transfers the content of a plugin-owned extrusion handle
    // into the plan clone. The handle itself is still owned by plugin storage,
    // so the host leaves it valid but empty after the move.
    PrintingPlan plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&plan);
    printing_group_handle *group_handle = printing_plan_append_group(plan_handle);
    printing_layer_group_handle *layer_handle = printing_group_append_layer_group(group_handle, 0);
    printing_tool_group_handle *tool_handle = printing_layer_group_append_tool_group(layer_handle, 0);

    ExtrusionEntity source(true);
    source.append_child(test_path({Point(0, 0), Point(10, 0)}));
    extrusion_entity_handle *source_handle = reinterpret_cast<extrusion_entity_handle *>(&source);

    printing_extrusion_handle *extrusion_handle = printing_tool_group_append_extrusion_move(
        plan_handle, tool_handle,
        nullptr,
        RAW_EXTRUSION_ROLE_INFILL,
        source_handle,
        PRINTING_OBJECT_INSTANCE_ID_NONE);
    REQUIRE(extrusion_handle != nullptr);

    CHECK(source.empty());
    REQUIRE(printing_extrusion_get_root(extrusion_handle) != nullptr);
    CHECK(extrusion_child_count(printing_extrusion_get_root(extrusion_handle)) == 1);
}

TEST_CASE("PrintingPlan C API transfers extrusions between tool groups",
          "[printing][plan][api][plugins][ordering]")
{
    /* A transfer updates both cached region-island lists before the source
       group is removed. This is the mutation sequence used by ordering plugins
       when a marker changes the tool assigned to an object extrusion. */
    PrintingPlan plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&plan);
    printing_group_handle *group_handle = printing_plan_append_group(plan_handle);
    printing_layer_group_handle *layer_handle = printing_group_append_layer_group(group_handle, 0);
    REQUIRE(printing_layer_group_append_tool_group(layer_handle, 0) != nullptr);
    REQUIRE(printing_layer_group_append_tool_group(layer_handle, 1) != nullptr);
    // Appending to the vector may move its elements, so acquire stable handles
    // only after the layer contains both tool groups used by the transfer.
    printing_tool_group_handle *source = printing_layer_group_get_tool_group_mutable(layer_handle, 0);
    printing_tool_group_handle *destination = printing_layer_group_get_tool_group_mutable(layer_handle, 1);
    const layer_region_island_handle *region_island =
        reinterpret_cast<const layer_region_island_handle *>(uintptr_t(0x3456));

    PrintingToolGroup *native_source =
        reinterpret_cast<PrintingToolGroup *>(source);
    PrintingExtrusion extrusion;
    extrusion.region_island =
        reinterpret_cast<const LayerRegionIsland *>(region_island);
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    extrusion.root = test_root();
    extrusion.root->append_child(test_path({Point(0, 0), Point(10, 0)}));
    native_source->region_islands.push_back(extrusion.region_island);
    native_source->extrusions.push_back(std::move(extrusion));
    REQUIRE(printing_tool_group_transfer_extrusion(source, 0, destination) == 1);

    CHECK(printing_tool_group_count_extrusion(source) == 0);
    CHECK(printing_tool_group_count_region_island(source) == 0);
    CHECK(printing_tool_group_count_extrusion(destination) == 1);
    CHECK(printing_tool_group_count_region_island(destination) == 1);
    CHECK(printing_tool_group_get_region_island(destination, 0) == region_island);
    CHECK(printing_layer_group_remove_empty_tool_group(layer_handle, 0) == 1);
    CHECK(printing_layer_group_count_tool_group(layer_handle) == 1);
    CHECK(printing_tool_group_get_extruder_id(
              printing_layer_group_get_tool_group(layer_handle, 0)) == 1);
}

TEST_CASE("PrintingPlan scopes expose fixed ordered event roots", "[printing][plan][events][api]")
{
    PrintingPlan plan;
    printing_plan_handle *plan_handle = reinterpret_cast<printing_plan_handle *>(&plan);
    printing_group_handle *group_handle = printing_plan_append_group(plan_handle);
    printing_layer_group_handle *layer_handle = printing_group_append_layer_group(group_handle, 42);
    printing_tool_group_handle *tool_handle = printing_layer_group_append_tool_group(layer_handle, 3);

    const printing_scope_events_handle *scope_events[] = {
        printing_plan_get_events(plan_handle),
        printing_group_get_events(group_handle),
        printing_layer_group_get_events(layer_handle),
        printing_tool_group_get_events(tool_handle)
    };
    for (const printing_scope_events_handle *events : scope_events) {
        REQUIRE(events != nullptr);
        CHECK_FALSE(printing_scope_events_has_before(events));
        CHECK_FALSE(printing_scope_events_has_after(events));

        const extrusion_entity_handle *before = printing_scope_events_get_before(events);
        const extrusion_entity_handle *after = printing_scope_events_get_after(events);
        REQUIRE(before != nullptr);
        REQUIRE(after != nullptr);
        CHECK(extrusion_has_children(before));
        CHECK(extrusion_has_children(after));
        CHECK(extrusion_child_count(before) == 0);
        CHECK(extrusion_child_count(after) == 0);
        CHECK(extrusion_flags(before) == 0);
        CHECK(extrusion_flags(after) == 0);
    }

    ExtrusionEntity cloned_event(true);
    cloned_event.append_child(test_path({Point(0, 0), Point(10, 0)}));
    const extrusion_entity_handle *cloned_event_handle =
        reinterpret_cast<const extrusion_entity_handle *>(&cloned_event);
    printing_scope_events_handle *plan_events = printing_plan_get_events_mutable(plan_handle);
    extrusion_entity_handle *appended_event =
        printing_scope_events_append_before_clone(plan_events, cloned_event_handle);
    REQUIRE(appended_event != nullptr);
    CHECK(printing_scope_events_has_before(plan_events));
    CHECK(extrusion_child_count(printing_scope_events_get_before(plan_events)) == 1);
    CHECK(extrusion_child_count(appended_event) == 1);
    CHECK(cloned_event.child_count() == 1);

    ExtrusionEntity moved_event(true);
    moved_event.append_child(test_path({Point(20, 0), Point(30, 0)}));
    extrusion_entity_handle *moved_event_handle = reinterpret_cast<extrusion_entity_handle *>(&moved_event);
    slic3r_api::PrintingPlan plan_view(plan_handle);
    slic3r_api::PrintingScopeEvents group_events = plan_view.group(0).events();
    group_events.append_after_move(slic3r_api::MutableExtrusionEntity(moved_event_handle));
    CHECK(group_events.has_after());
    CHECK(group_events.after().child_count() == 1);
    CHECK(moved_event.empty());

    // Event sequences belong to their scope value, so vector reordering moves
    // the sequence together with the group instead of leaving it at an index.
    printing_group_handle *second_group = printing_plan_append_group(plan_handle);
    ExtrusionEntity second_group_event(true);
    REQUIRE(printing_scope_events_append_before_clone(
                printing_group_get_events_mutable(second_group),
                reinterpret_cast<const extrusion_entity_handle *>(&second_group_event)) != nullptr);
    REQUIRE(printing_plan_move_group(plan_handle, 1, 0));
    CHECK(printing_scope_events_has_before(
        printing_group_get_events(printing_plan_get_group(plan_handle, 0))));

    printing_plan_clear(plan_handle);
    CHECK(printing_plan_count_group(plan_handle) == 0);
    CHECK_FALSE(printing_scope_events_has_before(printing_plan_get_events(plan_handle)));
    CHECK(extrusion_flags(printing_scope_events_get_before(printing_plan_get_events(plan_handle))) == 0);
}

TEST_CASE("Event roots remain fixed across repeated clear append and cloned scope cycles",
          "[plugins][printing][plan][events][scope-cycle]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();
    PrintingScopeEvents *scopes[] = {
        &plan.events, &plan.groups.front().events,
        &plan.groups.front().layers.front().events,
        &plan.groups.front().layers.front().tool_groups.front().events
    };
    std::vector<const ExtrusionEntity *> roots;
    std::vector<size_t> ids;
    for (PrintingScopeEvents *scope : scopes) {
        for (const ExtrusionEntity *root : {&scope->before(), &scope->after()}) {
            roots.push_back(root);
            ids.push_back(root->get_id());
            // These owned roots are exposed as const; seed existing direct
            // state only to prove clear does not erase unrelated properties.
            const_cast<ExtrusionEntity *>(root)->add_property(ExtrusionPropertySpeed(17.f));
        }
    }
    const auto fixed = [](const ExtrusionEntity &root, size_t count) {
        CHECK_FALSE(root.is_leaf());
        CHECK(root.child_count() == count);
        CHECK_FALSE(root.can_sort());
        CHECK_FALSE(root.can_reverse());
        CHECK_FALSE(root.can_rotate());
    };
    const auto check_event = [](const ExtrusionEntity &event) {
        const ExtrusionPropertyCustomGcode *property = event.get_property<ExtrusionPropertyCustomGcode>();
        REQUIRE(property != nullptr);
        CHECK(event.custom_gcode_string(*property) == "M117 scope cycle");
    };
    for (int cycle = 0; cycle < 3; ++cycle) {
        CAPTURE(cycle);
        Steps::StepExtrusionEdition::clear(print);
        Steps::StepExtrusionEdition::clear(print);
        size_t root_index = 0;
        for (PrintingScopeEvents *scope : scopes) {
            for (const ExtrusionEntity *root : {&scope->before(), &scope->after()}) {
                CHECK(root == roots.at(root_index));
                CHECK(root->get_id() == ids.at(root_index++));
                fixed(*root, 0);
                REQUIRE(root->get_property<ExtrusionPropertySpeed>() != nullptr);
                CHECK(root->get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 17.f);
            }
            ExtrusionEntity event(false);
            event.add_property(ExtrusionPropertyCustomGcodeText("M117 scope cycle"));
            scope->append_before(static_cast<const ExtrusionEntity &>(event));
            ExtrusionEntity moved_before(event);
            scope->append_before(std::move(moved_before));
            CHECK_FALSE(moved_before.has_properties());
            printing_scope_events_handle *handle = reinterpret_cast<printing_scope_events_handle *>(scope);
            REQUIRE(printing_scope_events_append_after_clone(handle,
                reinterpret_cast<const extrusion_entity_handle *>(&event)) != nullptr);
            ExtrusionEntity moved_after(event);
            REQUIRE(printing_scope_events_append_after_move(handle,
                reinterpret_cast<extrusion_entity_handle *>(&moved_after)) != nullptr);
            CHECK_FALSE(moved_after.has_properties());
            check_event(event); // Clone append did not consume its source.
            for (const ExtrusionEntity *root : {&scope->before(), &scope->after()}) {
                fixed(*root, 2);
                check_event(root->child(0));
                check_event(root->child(1));
            }
        }
        std::unique_ptr<PrintingPlan> copy = plan.clone();
        REQUIRE(copy->groups.size() == 1);
        REQUIRE(copy->groups.front().layers.size() == 1);
        REQUIRE(copy->groups.front().layers.front().tool_groups.size() == 1);
        PrintingScopeEvents *copied[] = {
            &copy->events, &copy->groups.front().events,
            &copy->groups.front().layers.front().events,
            &copy->groups.front().layers.front().tool_groups.front().events
        };
        for (size_t index = 0; index < 4; ++index) {
            PrintingScopeEvents &scope = *copied[index];
            const ExtrusionEntity *before = &scope.before();
            const ExtrusionEntity *after = &scope.after();
            fixed(*before, 2);
            fixed(*after, 2);
            CHECK(&before->child(0) != &scopes[index]->before().child(0));
            check_event(before->child(0));
            check_event(after->child(1));
            scope.clear();
            scope.clear();
            fixed(scope.before(), 0);
            fixed(scope.after(), 0);
            ExtrusionEntity next(false);
            next.add_property(ExtrusionPropertyCustomGcodeText("M117 scope cycle"));
            scope.append_before(static_cast<const ExtrusionEntity &>(next));
            scope.append_after(std::move(next));
            CHECK(&scope.before() == before);
            CHECK(&scope.after() == after);
            fixed(scope.before(), 1);
            fixed(scope.after(), 1);
            fixed(scopes[index]->before(), 2); // Clearing the clone is independent.
            fixed(scopes[index]->after(), 2);
        }
    }
}

// The last plugin observes live permissions and may append more event content.
// A reported error must escape without consuming those permissions.
void ordering_boundary_probe_run(void *ctx, const plugin_run_context *run_ctx);
// Register the late ordering probe once, preserving its state for controlled runs.
void register_ordering_boundary_probe();

void ordering_boundary_probe_run(void *ctx, const plugin_run_context *run_ctx)
{
    PlanEventEditorState &state = *static_cast<PlanEventEditorState *>(ctx);
    const run_ctx_extrusion_ordering *payload = plugin_ctx_as_extrusion_ordering(run_ctx);
    if (payload == nullptr || payload->plan == nullptr) {
        state.payload_valid = false;
        return;
    }
    PrintingPlan &plan = *reinterpret_cast<PrintingPlan *>(payload->plan);
    state.payload_valid = plan.events.before().child_count() == 1 &&
                          plan.events.before().child(0).can_reverse() &&
                          plan.events.before().child(0).can_rotate();
    ExtrusionEntity appended(true);
    plan.events.append_after(std::move(appended));
    ++state.run_count;
    if (g_ordering_boundary_fail)
        run_ctx->report_error(run_ctx->host_context, "Intentional ordering boundary failure");
}

void register_ordering_boundary_probe()
{
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(g_ordering_boundary_probe.id) != nullptr)
        return;
    static const plugin_vtable table = [] {
        plugin_vtable value = *plan_event_editor_vtable();
        value.get_step = [](void *) { return STEP_ORDERING; };
        value.setup = [](void *, const plugin_run_context *, uint32_t) {};
        value.setup_run = [](void *, const plugin_run_context *) {};
        value.run = &ordering_boundary_probe_run;
        return value;
    }();
    REQUIRE(orchestrator.register_plugin({&g_ordering_boundary_probe, &table}));
}

TEST_CASE("PrintingPlan deep clone owns independent trees, events and properties",
          "[plugins][printing][plan][clone]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const PluginPropertyKey<TestPrintingScopePayload> property_key =
        printing_scope_property_key(reinterpret_cast<orchestrator_handle *>(&orchestrator));

    PrintingPlan source;
    source.object_instances.emplace_back();
    PrintingObjectInstance &instance = source.object_instances.back();
    instance.object_instance_idx = size_t(-1);
    instance.name = "snapshot fixture";
    instance.projected_outline = Polygon({
        Point(0, 0), Point(scale_i(10.0), 0),
        Point(scale_i(10.0), scale_i(10.0)), Point(0, scale_i(10.0))
    });
    instance.projected_outline_center = instance.projected_outline.centroid();
    set_test_scope_value(source.properties, property_key, 10);

    source.groups.emplace_back();
    PrintingGroup &group = source.groups.back();
    group.plan_identity = source.identity_token();
    group.printing_catalog_ids.push_back(0);
    set_test_scope_value(group.properties, property_key, 20);
    ExtrusionEntityUPtr group_event = test_path({Point(0, 0), Point(10, 0)});
    group.events.append_before(*group_event);

    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.plan_identity = source.identity_token();
    layer.print_z = scale_i(0.2);
    set_test_scope_value(layer.properties, property_key, 30);
    ExtrusionEntityUPtr layer_event = test_path({Point(0, 10), Point(10, 10)});
    layer.events.append_after(*layer_event);

    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.plan_identity = source.identity_token();
    tool.extruder_id = 0;
    set_test_scope_value(tool.properties, property_key, 40);
    ExtrusionEntityUPtr tool_event = test_path({Point(0, 20), Point(10, 20)});
    tool.events.append_before(*tool_event);

    tool.extrusions.emplace_back();
    PrintingExtrusion &extrusion = tool.extrusions.back();
    extrusion.printing_catalog_id = 0;
    extrusion.root = test_path({Point(0, 30), Point(10, 30)});
    extrusion.root->add_property(ExtrusionPropertyCustomGcodeText(
        ExtrusionPropertyCustomGcode::Code::SCRIPT,
        "; cloned script data",
        GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM));

    std::unique_ptr<PrintingPlan> copy = source.clone();

    REQUIRE(copy != nullptr);
    CHECK(copy->identity_token() != source.identity_token());
    REQUIRE(copy->groups.size() == 1);
    CHECK(copy->groups[0].plan_identity == copy->identity_token());
    CHECK(copy->groups[0].layers[0].plan_identity == copy->identity_token());
    CHECK(copy->groups[0].layers[0].tool_groups[0].plan_identity == copy->identity_token());
    CHECK(copy->object_instances[0].name == "snapshot fixture");
    CHECK(test_scope_value(copy->properties, property_key) == 10);
    CHECK(test_scope_value(copy->groups[0].properties, property_key) == 20);
    CHECK(test_scope_value(copy->groups[0].layers[0].properties, property_key) == 30);
    CHECK(test_scope_value(copy->groups[0].layers[0].tool_groups[0].properties,
                           property_key) == 40);
    CHECK(copy->events.before().child_count() == 0);
    CHECK(copy->groups[0].events.before().child_count() == 1);
    CHECK(copy->groups[0].layers[0].events.after().child_count() == 1);
    CHECK(copy->groups[0].layers[0].tool_groups[0].events.before().child_count() == 1);

    PrintingExtrusion &copied_extrusion =
        copy->groups[0].layers[0].tool_groups[0].extrusions[0];
    REQUIRE(copied_extrusion.root != nullptr);
    CHECK(copied_extrusion.root.get() != extrusion.root.get());
    const ExtrusionPropertyCustomGcode *copied_script =
        copied_extrusion.root->get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(copied_script != nullptr);
    CHECK(copied_extrusion.root->custom_gcode_string(*copied_script) ==
          "; cloned script data");

    // Mutating the original after capture cannot alter the independent copy.
    set_test_scope_value(source.properties, property_key, 99);
    source.events.clear();
    source.groups[0].events.clear();
    source.groups[0].layers[0].events.clear();
    source.groups[0].layers[0].tool_groups[0].events.clear();
    extrusion.root->clear_content();
    extrusion.root->remove_property<ExtrusionPropertyCustomGcode>();
    CHECK(test_scope_value(copy->properties, property_key) == 10);
    CHECK(copy->groups[0].events.before().child_count() == 1);
    CHECK(copy->groups[0].layers[0].events.after().child_count() == 1);
    CHECK(copied_extrusion.root->length() > 0);
    CHECK(copied_extrusion.root->custom_gcode_string(*copied_script) ==
          "; cloned script data");
}

TEST_CASE("Print ordering checkpoint survives only valid downstream edits",
           "[plugins][printing][plan][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &working_plan = print.mutable_printing_plan();
    working_plan.groups.emplace_back();
    PrintingGroup &group = working_plan.groups.back();
    group.plan_identity = working_plan.identity_token();
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.plan_identity = working_plan.identity_token();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.plan_identity = working_plan.identity_token();
    tool.extruder_id = 0;
    tool.extrusions.emplace_back();
    tool.extrusions.back().root = test_path({Point(0, 0), Point(10, 0)});

    const coordf_t original_length = tool.extrusions[0].root->length();
    REQUIRE(print.save_ordering_checkpoint());
    REQUIRE(print.has_ordering_checkpoint());

    // A downstream editor mutates the active tree. The checkpoint remains the
    // ordered source, and every restoration creates a fresh mutable work copy.
    tool.extrusions[0].root->clear_content();
    REQUIRE(print.restore_ordering_checkpoint());
    REQUIRE(print.printing_plan() != nullptr);
    CHECK(print.printing_plan()->groups[0].plan_identity ==
          print.printing_plan()->identity_token());
    CHECK(print.printing_plan()->groups[0].layers[0].tool_groups[0]
              .extrusions[0].root->length() == original_length);

    // G-code and plan-editor replay both preserve their ordered source.
    print.request_replay_from_worker(Steps::Checkpoint::Plan);
    CHECK(print.has_ordering_checkpoint());
    print.request_replay_from_worker(Steps::Checkpoint::Ordering);
    CHECK(print.has_ordering_checkpoint());
    REQUIRE(print.restore_ordering_checkpoint());

    // An upstream change makes the saved source stale and must release it.
    print.request_replay_from_worker(Steps::Checkpoint::Slicing);
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK_FALSE(print.restore_ordering_checkpoint());

    print.reset_printing_plan();
    CHECK(print.printing_plan() == nullptr);
}

TEST_CASE("Print checkpoint preference controls ordered snapshot retention",
           "[plugins][printing][plan][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    print.mutable_printing_plan().groups.emplace_back();

    print.set_keep_ordering_checkpoint(false);
    CHECK_FALSE(print.save_ordering_checkpoint());
    CHECK_FALSE(print.has_ordering_checkpoint());

    print.set_keep_ordering_checkpoint(true);
    REQUIRE(print.save_ordering_checkpoint());
    CHECK(print.has_ordering_checkpoint());
    print.set_keep_ordering_checkpoint(false);
    CHECK_FALSE(print.has_ordering_checkpoint());
}

TEST_CASE("STEP_EXTRUSION_EDIT rebuilds scope events in plugin priority order",
          "[plugins][extrusion-edit][printing][plan][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_plan_event_editors();
    reset_plan_event_editor(g_first_plan_event_editor);
    reset_plan_event_editor(g_second_plan_event_editor);

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    plan.groups.front().layers.front().tool_groups.emplace_back();

    // Seed every scope with stale data. clear() must remove these
    // events without changing the surrounding ordered plan hierarchy.
    ExtrusionEntity stale_event(true);
    plan.events.append_before(stale_event);
    plan.groups.front().events.append_after(stale_event);
    plan.groups.front().layers.front().events.append_before(stale_event);
    plan.groups.front().layers.front().tool_groups.front().events.append_after(stale_event);

    Steps::StepExtrusionEdition::clear(print);
    CHECK_FALSE(plan.events.has_before());
    CHECK_FALSE(plan.groups.front().events.has_after());
    CHECK_FALSE(plan.groups.front().layers.front().events.has_before());
    CHECK_FALSE(plan.groups.front().layers.front().tool_groups.front().events.has_after());

    ScopedActivePlugins active({g_second_plan_event_editor.id, g_first_plan_event_editor.id});
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);

    CHECK(g_first_plan_event_editor.payload_valid.load());
    CHECK(g_second_plan_event_editor.payload_valid.load());
    CHECK(g_first_plan_event_editor.setup_count.load() == 1);
    CHECK(g_second_plan_event_editor.setup_count.load() == 1);
    CHECK(g_first_plan_event_editor.setup_run_count.load() == 1);
    CHECK(g_second_plan_event_editor.setup_run_count.load() == 1);
    CHECK(g_first_plan_event_editor.run_count.load() == 1);
    CHECK(g_second_plan_event_editor.run_count.load() == 1);
    CHECK(plan.events.before().child_count() == 2);

    // A repeated pipeline execution rebuilds the same ordered sequences. It
    // must not retain the two events produced by the previous execution.
    Steps::StepExtrusionEdition::clear(print);
    Steps::StepExtrusionEdition::run_step(Orchestrator::instance(), print);
    CHECK(g_first_plan_event_editor.run_count.load() == 2);
    CHECK(g_second_plan_event_editor.run_count.load() == 2);
    CHECK(plan.events.before().child_count() == 2);
    CHECK_FALSE(plan.events.before().is_leaf());
    CHECK_FALSE(plan.events.after().is_leaf());
    CHECK_FALSE(plan.events.before().can_sort());
    CHECK_FALSE(plan.events.before().can_reverse());
    CHECK_FALSE(plan.events.before().can_rotate());
    CHECK_FALSE(plan.events.after().can_sort());
    CHECK_FALSE(plan.events.after().can_reverse());
    CHECK_FALSE(plan.events.after().can_rotate());
}

// The real default builder clones the source support bucket and translates its
// single-point leaf by the object's instance shift before later ordering runs.
TEST_CASE("Default plan builder clones and instance-shifts stationary deposits",
          "[printing][plan][step-ordering][deposit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    PluginStorage storage;
    const ExPolygons subject{rectangle_expolygon(0., 0., 20., 20.)};
    const slic3r_api::Object object(reinterpret_cast<const object_handle *>(&print.object(0)));
    const c_point shift = object.instance_shift(0);
    REQUIRE((shift.x != 0 || shift.y != 0));
    const slic3r_api::AuxiliaryLayerBuildResult result = slic3r_api::build_auxiliary_layer_regions_from_subject(
        reinterpret_cast<storage_handle *>(&storage), print_view(print), object, expolygons_view(subject),
        scale_i(0.2), scale_i(0.4), scale_i(0.3));
    REQUIRE(result.created);
    layer_region_island_handle *region_island = layer_island_get_or_create_region_island(
        const_cast<layer_island_handle *>(result.layer.island(0).handle()), nullptr, 0, 0);
    REQUIRE(region_island != nullptr);
    extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
        region_island, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL);
    REQUIRE(root_handle != nullptr);
    ExtrusionEntityCollection &root = *reinterpret_cast<ExtrusionEntityCollection *>(root_handle);
    const Point position = Point::new_scale(1.25, 2.5);
    std::unique_ptr<ExtrusionEntity> deposit = test_path({position}, false);
    deposit->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)));
    deposit->add_property(ExtrusionPropertySpecialCommand(ExtrusionPropertySpecialCommand::Code::DEPOSIT, 0.4));
    root.append(std::move(deposit));
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(print.printing_plan()->groups.size() == 1);
    const PrintingGroup &group = print.printing_plan()->groups.front();
    REQUIRE(group.layers.size() == 1);
    REQUIRE(group.layers.front().tool_groups.size() == 1);
    const PrintingToolGroup &tool = group.layers.front().tool_groups.front();
    REQUIRE(tool.extrusions.size() == 1);
    const ExtrusionEntity &clone = tool.extrusions.front().root->child(0);
    CHECK(&clone != &root.child(0));
    REQUIRE(clone.polyline_or_null() != nullptr);
    CHECK(clone.polyline_or_null()->size() == 1);
    CHECK(clone.polyline_or_null()->front() == position + Point(shift.x, shift.y));
    CHECK(root.child(0).polyline_or_null()->front() == position);
    REQUIRE(clone.get_property<ExtrusionPropertySpecialCommand>() != nullptr);
    CHECK(clone.get_property<ExtrusionPropertySpecialCommand>()->command_code() == ExtrusionPropertySpecialCommand::Code::DEPOSIT);
    CHECK(clone.get_property<ExtrusionPropertySpecialCommand>()->extra_data == Approx(0.4));
    CHECK(clone.get_property<ExtrusionAttributes>()->mm3_per_mm == Approx(0.08));
}

TEST_CASE("Source buckets retain stored hybrid command geometry independently of virtual emptiness",
          "[plugins][printing][plan][source-presence][tree-transform]")
{
    for (const bool local : {false, true}) {
        for (const bool canonical : {false, true}) {
            CAPTURE(local, canonical);
            LayerRegionIsland source({}, 0);
            std::unique_ptr<ExtrusionNop> command = std::make_unique<ExtrusionNop>();
            command->position = Point(1200, 3400);
            command->add_property(ExtrusionPropertyCustomGcodeText("M117 retained source command"));
            const ArcPolyline path(Points{{0, 0}, {10000, 0}});
            if (local)
                command->set_polyline(path);
            else
                command->append_child(std::make_unique<ExtrusionEntity>(false, path));
            REQUIRE(command->empty()); // Legacy Nop behavior itself is not changed.
            REQUIRE_FALSE(command->ExtrusionEntity::empty());
            ExtrusionEntityUPtr stored = canonical ? command->clone_canonical() : std::move(command);
            const ExtrusionEntity *identity = stored.get();
            ExtrusionEntityCollection &bucket = source.mutable_extrusion(LayerRegionIsland::SUPPORT);
            bucket.append(std::move(stored));
            const ExtrusionEntityCollection *bucket_identity = &bucket;
            const auto *handle = reinterpret_cast<const layer_region_island_handle *>(&source);
            CHECK(source.has_extrusions());
            CHECK(source.has_extrusion(LayerRegionIsland::SUPPORT));
            CHECK(layer_region_island_has_extrusions(handle) != 0);
            CHECK(layer_region_island_has_extrusion(handle, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL) != 0);
            CHECK(layer_region_island_get_extrusion(handle, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL) ==
                  reinterpret_cast<const extrusion_entity_handle *>(bucket_identity));
            // A genuinely empty bucket must not keep an empty tool visit alive.
            source.mutable_extrusion(LayerRegionIsland::INFILLS);
            CHECK_FALSE(source.has_extrusion(LayerRegionIsland::INFILLS));
            source.remove_empty_extrusions();
            REQUIRE(source.has_extrusion(LayerRegionIsland::SUPPORT));
            CHECK(&source.extrusion(LayerRegionIsland::SUPPORT) == bucket_identity);
            CHECK(&source.extrusion(LayerRegionIsland::SUPPORT).child(0) == identity);
            REQUIRE(identity->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
            CHECK(identity->custom_gcode_string(*identity->get_property<ExtrusionPropertyCustomGcode>()) ==
                  "M117 retained source command");
            auto *nop = dynamic_cast<const ExtrusionNop *>(identity);
            if (canonical) {
                CHECK(nop == nullptr);
                CHECK(typeid(*identity) == typeid(ExtrusionEntity));
            } else {
                REQUIRE(nop != nullptr);
                CHECK(nop->position == Point(1200, 3400));
            }
        }
    }
    LayerRegionIsland commands_only({}, 0);
    commands_only.mutable_extrusion(LayerRegionIsland::SUPPORT).append(std::make_unique<ExtrusionNop>());
    CHECK_FALSE(commands_only.has_extrusion(LayerRegionIsland::SUPPORT));
    CHECK_FALSE(commands_only.has_extrusions());
}

TEST_CASE("Default plan builder publishes generic command properties and shifted stored geometry",
          "[plugins][printing][plan][step-ordering][tree-transform]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const bool positioned : {false, true}) {
        CAPTURE(positioned);
        Model model;
        Print print;
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
        PluginStorage storage;
        const ExPolygons subject{rectangle_expolygon(0., 0., 20., 20.)};
        const slic3r_api::Object object(reinterpret_cast<const object_handle *>(&print.object(0)));
        const c_point raw_shift = object.instance_shift(0);
        const Point shift(raw_shift.x, raw_shift.y);
        REQUIRE(shift != Point(0, 0));
        const auto result = slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view(print), object, expolygons_view(subject),
            scale_i(0.2), scale_i(0.4), scale_i(0.3));
        REQUIRE(result.created);
        layer_region_island_handle *island = layer_island_get_or_create_region_island(
            const_cast<layer_island_handle *>(result.layer.island(0).handle()), nullptr, 0, 0);
        REQUIRE(island != nullptr);
        ExtrusionEntityCollection *root = reinterpret_cast<ExtrusionEntityCollection *>(
            layer_region_island_get_mutable_extrusion(island, RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL));
        REQUIRE(root != nullptr);
        std::unique_ptr<ExtrusionNop> command = std::make_unique<ExtrusionNop>();
        command->position = positioned ? Point::new_scale(1.25, 2.5) : ExtrusionNop::NOT_A_POINT;
        command->set_role(ExtrusionRole::SupportMaterial);
        command->add_property(ExtrusionAttributes{ExtrusionRole::SupportMaterial});
        command->set_can_rotate(false);
        command->add_property(ExtrusionPropertyCustomGcodeText("M117 instance-local command"));
        ArcPolyline arc;
        arc.append(Point::new_scale(2, 0));
        arc.append(Geometry::ArcWelder::Segment(Point::new_scale(0, 2), float(scale_i(2)), Geometry::ArcWelder::Orientation::CCW));
        arc.set_z_offset(0, 100);
        arc.set_z_offset(1, 900);
        std::unique_ptr<ExtrusionEntity> geometry = std::make_unique<ExtrusionEntity>(false, arc);
        geometry->add_property(ExtrusionAttributes(ExtrusionRole::SupportMaterial, ExtrusionFlow(0.08, 0.4f, 0.2f)));
        geometry->add_property(ExtrusionPropertySpeed(20.f));
        command->append_child(std::move(geometry));
        command->set_can_sort_reverse(false, false);
        const ExtrusionNop *source = command.get();
        const Point original_position = source->position;
        root->append(std::move(command));
        Steps::StepExtrusionOrdering::clear(print);
        Steps::StepExtrusionOrdering::prepare(print);
        Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
        REQUIRE(print.printing_plan() != nullptr);
        REQUIRE(print.printing_plan()->groups.size() == 1);
        const PrintingGroup &group = print.printing_plan()->groups.front();
        REQUIRE(group.layers.size() == 1);
        REQUIRE(group.layers.front().tool_groups.size() == 1);
        const std::vector<PrintingExtrusion> &extrusions = group.layers.front().tool_groups.front().extrusions;
        REQUIRE(extrusions.size() == 1);
        REQUIRE(extrusions.front().root->child_count() == 1);
        const ExtrusionEntity *copy = &extrusions.front().root->child(0);
        REQUIRE(typeid(*copy) == typeid(ExtrusionEntity));
        CHECK(copy != source);
        CHECK(dynamic_cast<const ExtrusionNop *>(copy) == nullptr);
        REQUIRE(copy->get_property<ExtrusionAttributes>() != nullptr);
        CHECK(copy->get_property<ExtrusionAttributes>()->extrusion_role() == ExtrusionRole::SupportMaterial);
        REQUIRE(copy->child_count() == 1);
        const ArcPolyline &translated = copy->child(0).polyline_ref();
        CHECK(translated.front() == arc.front() + shift);
        CHECK(translated.back() == arc.back() + shift);
        CHECK(translated.get_arc(1).radius == arc.get_arc(1).radius);
        CHECK(translated.z_offset(0) == 100);
        CHECK(translated.z_offset(1) == 900);
        REQUIRE(copy->get_property<ExtrusionPropertyCustomGcode>() != nullptr);
        CHECK(copy->custom_gcode_string(*copy->get_property<ExtrusionPropertyCustomGcode>()) == "M117 instance-local command");
        CHECK(source->position == original_position);
        CHECK(source->child(0).polyline_ref().front() == arc.front());
        CHECK(source->child(0).polyline_ref().back() == arc.back());
    }
}

TEST_CASE("STEP_ORDERING locks unvisited trees and all event scopes only at its final boundary",
          "[plugins][printing][plan][step-ordering][ordering-finalization]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    using Access = ApiInternal::ExtrusionPropertyAccess;
    for (const char *engine : {"ordering.extrusion_tree.default", "ordering.extrusion_tree.advanced",
                               "ordering.extrusion_tree.xy_serpentine"}) {
        CAPTURE(engine);
        ScopedActivePlugins active{engine};
        Print print;
        PrintingPlan &plan = print.mutable_printing_plan();
        slic3r_api::PrintingPlan view(reinterpret_cast<printing_plan_handle *>(&plan));
        slic3r_api::PrintingToolGroup tool_view = view.append_group().append_layer_group(1000).append_tool_group(0);
        (void)tool_view;
        PrintingGroup &group = plan.groups[0];
        PrintingLayerGroup &layer = group.layers[0];
        PrintingToolGroup &tool = layer.tool_groups[0];
        // A fixed root is deliberately not a request for a new ordering or seam.
        std::unique_ptr<ExtrusionEntityCollection> source = test_root(false, false);
        source->set_can_rotate(false);
        source->append(test_path({Point(200, 0), Point(210, 0)}));
        std::unique_ptr<ExtrusionEntityCollection> empty = test_root(true, true);
        empty->append_child(std::make_unique<ExtrusionEntity>(true));
        source->append(std::move(empty));
        // Keep both reversible paths already nearest-entry oriented, so the
        // exact geometry oracle isolates finalization from legal optimization.
        source->append(test_path({Point(300, 0), Point(310, 0)}));
        const ExtrusionEntityUPtr original = source->clone_canonical();
        tool.extrusions.emplace_back();
        tool.extrusions.back().root = source->clone_canonical();
        auto key = slic3r_api::entry_point_property_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
        slic3r_api::EntryPointProperty &entry = key.get_or_add(slic3r_api::MutableExtrusionEntity(
            reinterpret_cast<extrusion_entity_handle *>(tool.extrusions.back().root.get())));
        entry.entry = {200, 0};
        entry.exit = {310, 0};
        tool.extrusions.emplace_back();
        tool.extrusions.back().root = std::make_unique<ExtrusionEntity>(true); // empty root too
        std::vector<const ExtrusionEntity *> roots{tool.extrusions[0].root.get(), tool.extrusions[1].root.get()};
        std::vector<ExtrusionEntity *> event_nodes;
        for (PrintingScopeEvents *events : {&plan.events, &group.events, &layer.events, &tool.events}) {
            ExtrusionEntity event(ExtrusionEntity::Children(), true, true, false);
            event.append_child(test_path({Point(1, 2), Point(3, 4)}));
            event.append_child(test_path({Point(30, 40), Point(50, 60)}));
            event.append_child(std::make_unique<ExtrusionNop>(ExtrusionPropertyCustomGcodeText("M117 final scope")));
            event_nodes.push_back(&events->append_before(event));
            event_nodes.push_back(&events->append_after(event));
            roots.push_back(&events->before());
            roots.push_back(&events->after());
        }
        std::vector<ExtrusionEntityUPtr> snapshots;
        std::vector<const ExtrusionEntity *> handles;
        const auto collect = [&](const auto &self, const ExtrusionEntity &node) -> void {
            handles.push_back(&node);
            for (const ExtrusionEntityUPtr &child : node.children()) if (child) self(self, *child);
        };
        for (const ExtrusionEntity *root : roots) {
            snapshots.push_back(root->clone_canonical());
            collect(collect, *root);
        }
        REQUIRE(source->child(0).can_reverse());
        REQUIRE(event_nodes.front()->can_sort());
        Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
        const auto check = [&](const auto &self, const ExtrusionEntity &node, const ExtrusionEntity &before,
                               const bool same_handles, const bool new_scope_root) -> void {
            if (same_handles) CHECK(std::find(handles.begin(), handles.end(), &node) != handles.end());
            CHECK_FALSE(node.can_sort());
            CHECK_FALSE(node.can_reverse());
            CHECK_FALSE(node.can_rotate());
            if (new_scope_root) CHECK(node.get_id() != before.get_id());
            else CHECK(node.get_id() == before.get_id());
            REQUIRE(node.has_polyline() == before.has_polyline());
            if (node.has_polyline()) {
                const ArcPolyline &poly = node.polyline_ref();
                REQUIRE(poly.size() == before.polyline_ref().size());
                for (size_t i = 0; i < poly.size(); ++i)
                    CHECK(poly.get_point(i) == before.polyline_ref().get_point(i));
            }
            // Ordering may add its entry-point diagnostic property. Existing
            // payloads must remain exact; finalization must not remove them.
            REQUIRE(Access::property_count(node) >= Access::property_count(before));
            for (size_t i = 0; i < Access::property_count(before); ++i)
                CHECK(Access::same_property_payload(node, Access::property_type_at(before, i), before));
            REQUIRE(node.child_count() == before.child_count());
            for (size_t i = 0; i < node.child_count(); ++i) self(self, node.child(i), before.child(i), same_handles, false);
        };
        for (size_t i = 0; i < roots.size(); ++i) check(check, *roots[i], *snapshots[i], true, false);
        CHECK(source->child(0).can_reverse());
        CHECK(source->child(1).can_reverse());
        CHECK(source->child(0).polyline_ref().front() == original->child(0).polyline_ref().front());
        std::string error;
        CHECK(Steps::StepExtrusionOrdering::validate_post(print, &error));
        REQUIRE(print.save_ordering_checkpoint());
        tool.extrusions[0].root->set_can_rotate(true);
        REQUIRE(print.restore_ordering_checkpoint());
        const PrintingPlan &restored = *print.printing_plan();
        const PrintingGroup &restored_group = restored.groups[0];
        const PrintingLayerGroup &restored_layer = restored_group.layers[0];
        const PrintingToolGroup &restored_tool = restored_layer.tool_groups[0];
        std::vector<const ExtrusionEntity *> restored_roots{restored_tool.extrusions[0].root.get(), restored_tool.extrusions[1].root.get()};
        for (const PrintingScopeEvents *events : {&restored.events, &restored_group.events, &restored_layer.events, &restored_tool.events}) {
            restored_roots.push_back(&events->before());
            restored_roots.push_back(&events->after());
        }
        // Checkpoint scope roots are newly owned nodes; their event descendants
        // retain identity, payloads and selected geometry across that clone.
        for (size_t i = 0; i < restored_roots.size(); ++i) check(check, *restored_roots[i], *snapshots[i], false, i >= 2);
        CHECK(Steps::StepExtrusionOrdering::validate_post(print, &error));
    }
}

TEST_CASE("Ordering providers reverse authorized composed units with fixed paths and commands",
          "[plugins][printing][plan][tree-reverse][ordering-finalization]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    for (const char *engine : {"native", "ordering.extrusion_tree.default", "ordering.extrusion_tree.advanced",
                               "ordering.extrusion_tree.xy_serpentine"}) {
        CAPTURE(engine);
        ExtrusionEntityUPtr unit = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, true, false);
        unit->set_can_rotate(false);
        ExtrusionEntityUPtr before = std::make_unique<ExtrusionEntity>(false);
        before->add_property(ExtrusionPropertyCustomGcodeText("M117 unit before"));
        const ExtrusionEntity *before_handle = before.get();
        unit->append_child(std::move(before));
        unit->append_child(test_path({Point(100, 0), Point(50, 0)}, false)->clone_canonical());
        unit->append_child(test_path({Point(50, 0), Point(10, 0)}, false)->clone_canonical());
        ExtrusionEntityUPtr after = std::make_unique<ExtrusionEntity>(false);
        after->add_property(ExtrusionPropertyCustomGcodeText("M117 unit after"));
        const ExtrusionEntity *after_handle = after.get();
        unit->append_child(std::move(after));
        ExtrusionEntityUPtr outer = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), true, true, false);
        const ExtrusionEntity *unit_handle = unit.get();
        outer->append_child(std::move(unit));
        if (std::string(engine) == "native") {
            order_extrusion_tree(*outer, Point(0, 0));
        } else {
            ScopedActivePlugins active{engine};
            Print print;
            PrintingPlan &plan = print.mutable_printing_plan();
            slic3r_api::PrintingPlan view(reinterpret_cast<printing_plan_handle *>(&plan));
            (void)view.append_group().append_layer_group(1000).append_tool_group(0);
            PrintingToolGroup &tool = plan.groups[0].layers[0].tool_groups[0];
            tool.extrusions.emplace_back();
            tool.extrusions[0].root = std::move(outer);
            auto key = slic3r_api::entry_point_property_key(reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
            slic3r_api::EntryPointProperty &entry = key.get_or_add(slic3r_api::MutableExtrusionEntity(
                reinterpret_cast<extrusion_entity_handle *>(tool.extrusions[0].root.get())));
            entry.entry = {0, 0};
            entry.exit = {10, 0};
            Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
            outer = std::move(tool.extrusions[0].root);
        }
        REQUIRE(outer->child_count() == 1);
        const ExtrusionEntity &actual = outer->child(0);
        CHECK(&actual == unit_handle);
        REQUIRE(actual.child_count() == 4);
        CHECK(&actual.child(0) == after_handle);
        CHECK(&actual.child(3) == before_handle);
        CHECK(actual.child(1).polyline_ref().front() == Point(10, 0));
        CHECK(actual.child(1).polyline_ref().back() == Point(50, 0));
        CHECK(actual.child(2).polyline_ref().front() == Point(50, 0));
        CHECK(actual.child(2).polyline_ref().back() == Point(100, 0));
        CHECK(actual.child(0).custom_gcode_string(*actual.child(0).get_property<ExtrusionPropertyCustomGcode>()) == "M117 unit after");
        CHECK(actual.child(3).custom_gcode_string(*actual.child(3).get_property<ExtrusionPropertyCustomGcode>()) == "M117 unit before");
    }
}

TEST_CASE("Ordering finalization waits for the last plugin and never hides a plugin failure",
          "[plugins][printing][plan][ordering-finalization]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    register_ordering_boundary_probe();
    for (const bool fail : {false, true}) {
        CAPTURE(fail);
        ScopedActivePlugins active{g_ordering_boundary_probe.id};
        g_ordering_boundary_fail = fail;
        reset_plan_event_editor(g_ordering_boundary_probe);
        Orchestrator::instance().reset_plugin_cancel();
        Orchestrator::instance().consume_plugin_messages();
        Print print;
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.events.append_before(ExtrusionEntity(true));
        std::string error;
        try { Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print); }
        catch (const std::exception &exception) { error = exception.what(); }
        const auto messages = Orchestrator::instance().consume_plugin_messages();
        CHECK(g_ordering_boundary_probe.run_count == 1);
        CHECK(g_ordering_boundary_probe.payload_valid);
        REQUIRE(plan.events.after().child_count() == 1);
        if (fail) {
            CHECK_FALSE(error.empty());
            CHECK(error.find(g_ordering_boundary_probe.id) != std::string::npos);
            CHECK(error.find("during run") != std::string::npos);
            REQUIRE(messages.size() == 1);
            CHECK(messages[0].message == "Intentional ordering boundary failure");
            CHECK(messages[0].level == Orchestrator::PluginMessageLevel::Error);
            CHECK(messages[0].step == STEP_ORDERING);
            CHECK(messages[0].plugin_id == g_ordering_boundary_probe.id);
            CHECK(plan.events.before().child(0).can_reverse());
            CHECK(plan.events.after().child(0).can_rotate());
        } else {
            CHECK(error.empty());
            CHECK(messages.empty());
            CHECK_FALSE(plan.events.before().child(0).can_reverse());
            CHECK_FALSE(plan.events.before().child(0).can_rotate());
            CHECK_FALSE(plan.events.after().child(0).can_reverse());
            CHECK_FALSE(plan.events.after().child(0).can_rotate());
        }
        Orchestrator::instance().reset_plugin_cancel();
    }
    g_ordering_boundary_fail = false;
}

TEST_CASE("STEP_ORDERING post validation diagnoses remaining permissions at exact tree paths",
          "[plugins][printing][plan][ordering-finalization]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active{};
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan view(reinterpret_cast<printing_plan_handle *>(&plan));
    (void)view.append_group().append_layer_group(1000).append_tool_group(0);
    PrintingGroup &group = plan.groups[0];
    PrintingLayerGroup &layer = group.layers[0];
    PrintingToolGroup &tool = layer.tool_groups[0];
    ExtrusionEntity event(ExtrusionEntity::Children(), false, false, false);
    event.set_can_rotate(false);
    event.append_child(test_path({Point(0, 0), Point(10, 0)}, false));
    event.append_child(test_path({Point(20, 0), Point(30, 0)}, false));
    std::vector<std::pair<ExtrusionEntity *, std::string>> nodes;
    tool.extrusions.emplace_back();
    tool.extrusions[0].root = event.clone_canonical();
    nodes.emplace_back(tool.extrusions[0].root.get(), "group[0].layer[0].tool[0].extrusion[0].root");
    for (const auto scope : std::vector<std::pair<PrintingScopeEvents *, std::string>>{
             {&plan.events, "plan"}, {&group.events, "group[0]"},
             {&layer.events, "group[0].layer[0]"}, {&tool.events, "group[0].layer[0].tool[0]"}}) {
        nodes.emplace_back(&scope.first->append_before(event), scope.second + ".events.before.child[0]");
        nodes.emplace_back(&scope.first->append_after(event), scope.second + ".events.after.child[0]");
    }
    Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), print);
    for (const auto &node : nodes) {
        for (const std::string permission : {"can_sort", "can_reverse", "can_rotate"}) {
            CAPTURE(node.second, permission);
            node.first->set_can_sort_reverse(permission == "can_sort", permission == "can_reverse");
            node.first->set_can_rotate(permission == "can_rotate");
            std::string error;
            CHECK_FALSE(Steps::StepExtrusionOrdering::validate_post(print, &error));
            CHECK(error.find(node.second) != std::string::npos);
            CHECK(error.find(permission) != std::string::npos);
            node.first->set_can_sort_reverse(false, false);
            node.first->set_can_rotate(false);
            CHECK(Steps::StepExtrusionOrdering::validate_post(print, &error));
        }
    }
}

TEST_CASE("Plan ingress and scope clones publish canonical nodes with owned command data",
          "[plugins][printing][plan][canonical-ingress]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto make_source = [] {
        std::unique_ptr<ExtrusionEntityCollection> source = std::make_unique<ExtrusionEntityCollection>(false, true);
        source->add_property(ExtrusionPropertyCustomGcodeText("M117 canonical root"));
        std::unique_ptr<ExtrusionPath> path = std::make_unique<ExtrusionPath>(ExtrusionAttributes{ExtrusionRole::Perimeter}, false);
        path->set_polyline(ArcPolyline(Points{{100, 0}, {200, 0}}));
        source->append(std::move(path));
        return source;
    };
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan view(reinterpret_cast<printing_plan_handle *>(&plan));
    (void)view.append_group().append_layer_group(1000).append_tool_group(0);
    PrintingGroup &group = plan.groups[0];
    PrintingLayerGroup &layer = group.layers[0];
    PrintingToolGroup &tool = layer.tool_groups[0];
    for (PrintingScopeEvents *events : {&plan.events, &group.events, &layer.events, &tool.events}) {
        const_cast<ExtrusionEntity &>(events->before()).add_property(ExtrusionPropertyCustomGcodeText("M117 root scope data"));
        std::unique_ptr<ExtrusionEntityCollection> source = make_source();
        const ExtrusionEntity *source_child = &source->child(0);
        const extrusion_entity_handle *source_handle = reinterpret_cast<const extrusion_entity_handle *>(source.get());
        printing_scope_events_handle *scope = reinterpret_cast<printing_scope_events_handle *>(events);
        const extrusion_entity_handle *cloned = printing_scope_events_append_before_clone(scope, source_handle);
        REQUIRE(cloned != nullptr);
        CHECK(CanonicalExtrusion::is_tree(*reinterpret_cast<const ExtrusionEntity *>(cloned)));
        CHECK(&source->child(0) == source_child);
        CHECK(typeid(source->child(0)) == typeid(ExtrusionPath));
        const extrusion_entity_handle *moved = printing_scope_events_append_after_move(
            scope, reinterpret_cast<extrusion_entity_handle *>(source.get()));
        REQUIRE(moved != nullptr);
        CHECK(CanonicalExtrusion::is_tree(*reinterpret_cast<const ExtrusionEntity *>(moved)));
        CHECK(source->ExtrusionEntity::empty());
        CHECK_FALSE(source->has_properties());
        CHECK_FALSE(events->before().can_sort());
        CHECK_FALSE(events->before().can_reverse());
        CHECK_FALSE(events->before().can_rotate());
    }
    std::unique_ptr<ExtrusionEntityCollection> source = make_source();
    printing_plan_handle *raw_plan = reinterpret_cast<printing_plan_handle *>(&plan);
    printing_tool_group_handle *raw_tool = reinterpret_cast<printing_tool_group_handle *>(&tool);
    REQUIRE(printing_tool_group_append_extrusion_clone(raw_plan, raw_tool, nullptr, RAW_EXTRUSION_ROLE_PERIMETER,
        reinterpret_cast<const extrusion_entity_handle *>(source.get()), UINT32_MAX) != nullptr);
    CHECK(CanonicalExtrusion::is_tree(*tool.extrusions[0].root));
    ExtrusionEntityUPtr conforming = source->clone_canonical();
    const ExtrusionEntity *child = &conforming->child(0);
    CHECK(printing_tool_group_append_extrusion_move(raw_plan, raw_tool, nullptr, RAW_EXTRUSION_ROLE_PERIMETER,
        reinterpret_cast<extrusion_entity_handle *>(conforming.get()), 0) == nullptr);
    CHECK(&conforming->child(0) == child);
    CHECK(conforming->has_properties());
    REQUIRE(printing_tool_group_append_extrusion_move(raw_plan, raw_tool, nullptr, RAW_EXTRUSION_ROLE_PERIMETER,
        reinterpret_cast<extrusion_entity_handle *>(conforming.get()), UINT32_MAX) != nullptr);
    CHECK(&tool.extrusions[1].root->child(0) == child);
    CHECK(conforming->ExtrusionEntity::empty());
    CHECK_FALSE(conforming->has_properties());
    std::string error;
    CHECK(validate_canonical_plan(plan, &error));
    std::unique_ptr<PrintingPlan> copy = plan.clone();
    CHECK(validate_canonical_plan(*copy, &error));
    const PrintingScopeEvents *copy_scopes[] = {&copy->events, &copy->groups[0].events,
        &copy->groups[0].layers[0].events, &copy->groups[0].layers[0].tool_groups[0].events};
    for (const PrintingScopeEvents *events : copy_scopes) {
        REQUIRE(events->before().get_property<ExtrusionPropertyCustomGcode>() != nullptr);
        CHECK(events->before().custom_gcode_string(*events->before().get_property<ExtrusionPropertyCustomGcode>()) == "M117 root scope data");
        REQUIRE(events->after().child_count() == 1);
        CHECK(events->after().child(0).custom_gcode_string(*events->after().child(0).get_property<ExtrusionPropertyCustomGcode>()) == "M117 canonical root");
    }
    for (const bool move : {false, true}) {
        for (const bool descendant : {false, true}) {
            CAPTURE(move, descendant);
            tool.extrusions[0].root = source->clone_canonical();
            ExtrusionEntity *input = descendant ? &tool.extrusions[0].root->child(0) : tool.extrusions[0].root.get();
            const Point first = input->ExtrusionEntity::first_point();
            const Point last = input->ExtrusionEntity::last_point();
            const uint64_t id = input->get_id();
            printing_extrusion_handle *target = reinterpret_cast<printing_extrusion_handle *>(&tool.extrusions[0]);
            const int32_t ok = move ? printing_extrusion_set_root_move(target, reinterpret_cast<extrusion_entity_handle *>(input)) :
                printing_extrusion_set_root_clone(target, reinterpret_cast<const extrusion_entity_handle *>(input));
            REQUIRE(ok != 0);
            CHECK(CanonicalExtrusion::is_tree(*tool.extrusions[0].root));
            CHECK(tool.extrusions[0].root->get_id() == id);
            CHECK(tool.extrusions[0].root->first_point() == first);
            CHECK(tool.extrusions[0].root->last_point() == last);
        }
    }
    // A source edit cannot change any copied root or its separately owned text.
    plan.events.clear();
    const_cast<ExtrusionEntity &>(plan.events.before()).remove_property<ExtrusionPropertyCustomGcode>();
    CHECK(copy->events.before().custom_gcode_string(*copy->events.before().get_property<ExtrusionPropertyCustomGcode>()) == "M117 root scope data");
}

TEST_CASE("Canonical boundary validation diagnoses legacy and null nodes without repairing them",
          "[plugins][printing][plan][canonical-ingress]")
{
    PrintingPlan plan;
    plan.groups.emplace_back();
    plan.groups[0].layers.emplace_back();
    plan.groups[0].layers[0].tool_groups.emplace_back();
    PrintingToolGroup &tool = plan.groups[0].layers[0].tool_groups[0];
    tool.extrusions.emplace_back();
    tool.extrusions[0].root = std::make_unique<ExtrusionEntityCollection>();
    const ExtrusionEntity *identity = tool.extrusions[0].root.get();
    std::string error;
    CHECK_FALSE(validate_canonical_plan(plan, &error));
    CHECK(error.find("group[0].layer[0].tool[0].extrusion[0].root") != std::string::npos);
    CHECK(error.find("legacy extrusion type") != std::string::npos);
    CHECK(tool.extrusions[0].root.get() == identity);
    tool.extrusions[0].root = tool.extrusions[0].root->clone_canonical();
    CHECK(validate_canonical_plan(plan, &error));
    tool.extrusions[0].root->children().emplace_back(nullptr);
    CHECK_FALSE(validate_canonical_plan(plan, &error));
    CHECK(error.find(".root.child[0]") != std::string::npos);
    CHECK(error.find("null owned extrusion child") != std::string::npos);
    tool.extrusions[0].root->children().clear();
    ExtrusionEntity &event = plan.events.append_before(ExtrusionEntity(false));
    event.append_child(std::make_unique<ExtrusionLoop>());
    CHECK_FALSE(validate_canonical_plan(plan, &error));
    CHECK(error.find("plan.events.before.child[0].child[0]") != std::string::npos);
    CHECK(typeid(event.child(0)) == typeid(ExtrusionLoop));
}

TEST_CASE("STEP_ORDERING runs default plugin chain on a shared PrintingPlan", "[printing][plan][step-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    Print print;
    Steps::StepExtrusionOrdering::clear(print);
    REQUIRE(print.printing_plan() == nullptr);

    const std::vector<Plugin *> plugins =
        Steps::selected_or_active_plugins_for_step(orchestrator, STEP_ORDERING, &print.full_print_config());
    REQUIRE(plugins.size() >= 7);
    CHECK(plugins[0]->get_id() == "ordering.plan_builder.by_layer");
    CHECK(plugins[1]->get_id() == "ordering.object_instance_catalog");
    CHECK(plugins[2]->get_id() == "ordering.custom_gcode_tool_overrides");
    CHECK(plugins[3]->get_id() == "ordering.tool_groups.default");
    CHECK(plugins[4]->get_id() == "ordering.custom_gcode_event_tools");
    CHECK(plugins[5]->get_id() == "ordering.printing_extrusion.presort.default");
    CHECK(plugins[6]->get_id() == "ordering.extrusion_tree.default");
    CHECK(Steps::get_exclusive_steps().find(STEP_ORDERING) == Steps::get_exclusive_steps().end());

    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);

    REQUIRE(print.printing_plan() != nullptr);
    CHECK(print.printing_plan()->groups.size() == 1);
}

TEST_CASE("STEP_ORDERING includes print-level auxiliary layers", "[printing][plan][step-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    CHECK(print_count_object(reinterpret_cast<const print_handle *>(&print)) == 0);

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const ExPolygons subject{ rectangle_expolygon(20., 20., 40., 40.) };

    /*
    Build a print-level auxiliary layer without adding any real PrintObject.
    The default plan builder must still see this layer through the explicit
    print auxiliary object API.
    */
    slic3r_api::AuxiliaryLayerBuildResult result =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            storage,
            print_view(print),
            print_view(print).auxiliary_object(),
            expolygons_view(subject),
            scale_i(1.),
            scale_i(1.),
            scale_i(0.5));
    REQUIRE(result.created);

    slic3r_api::Layer layer = result.layer;
    REQUIRE(layer.island_count() == 1);
    layer_region_island_handle *region_island_handle =
        layer_island_get_or_create_region_island(
            const_cast<layer_island_handle *>(layer.island(0).handle()),
            nullptr,
            0,
            0);
    REQUIRE(region_island_handle != nullptr);

    extrusion_entity_handle *extrusion_handle =
        layer_region_island_get_mutable_extrusion(region_island_handle, RAW_EXTRUSION_ROLE_PERIMETER);
    REQUIRE(extrusion_handle != nullptr);
    ExtrusionEntityCollection *extrusion_root =
        reinterpret_cast<ExtrusionEntityCollection *>(extrusion_handle);
    extrusion_root->append(test_path({Point(scale_i(20.), scale_i(20.)),
                                      Point(scale_i(40.), scale_i(20.))}));

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepExtrusionOrdering::clear(print);
    Steps::StepExtrusionOrdering::prepare(print);
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);

    REQUIRE(print.printing_plan() != nullptr);
    REQUIRE(print.printing_plan()->groups.size() == 1);
    const PrintingGroup &group = print.printing_plan()->groups.front();
    CHECK(group.printing_catalog_ids.empty());
    REQUIRE(group.layers.size() == 1);
    REQUIRE(group.layers.front().tool_groups.size() == 1);
    REQUIRE(group.layers.front().tool_groups.front().extrusions.size() == 1);
    CHECK(group.layers.front().tool_groups.front().extrusions.front().printing_catalog_id ==
          PRINTING_OBJECT_INSTANCE_ID_NONE);
}

// Build two printable source islands for one instance, apply each selection,
// and compare the plan-owned clones with their unchanged borrowed sources.
TEST_CASE("Common plan builder filters layer extrusions for one instance",
          "[plugins][printing-plan][plan-builder-common]")
{
    using namespace slic3r_api::Ordering::PlanBuilderCommon;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    const slic3r_api::Object object = print_view(print).object(0);
    PluginStorage storage;
    const ExPolygons subject{
        rectangle_expolygon(0., 0., 5., 5.),
        rectangle_expolygon(20., 0., 25., 5.)};
    const slic3r_api::AuxiliaryLayerBuildResult built =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view(print),
            object, expolygons_view(subject), scale_i(1.), scale_i(1.), scale_i(0.5));
    REQUIRE(built.created);
    const slic3r_api::Layer source = built.layer;
    REQUIRE(source.island_count() == 2);
    std::vector<const LayerRegionIsland *> source_regions;
    for (uint32_t index = 0; index < source.island_count(); ++index) {
        const slic3r_api::LayerRegionIsland region =
            source.island(index).get_or_create_full_region_island(0);
        REQUIRE(region.valid());
        extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
            const_cast<layer_region_island_handle *>(region.handle()), RAW_EXTRUSION_ROLE_PERIMETER);
        REQUIRE(root_handle != nullptr);
        ExtrusionEntityCollection &root = *reinterpret_cast<ExtrusionEntityCollection *>(root_handle);
        root.append(test_path({Point(scale_i(index * 20.), 0), Point(scale_i(index * 20. + 5.), 0)}));
        source_regions.push_back(reinterpret_cast<const LayerRegionIsland *>(region.handle()));
    }

    LayerExtrusionFilter filter;
    std::vector<uint32_t> selected;
    std::vector<uint32_t> expected{0, 1};
    bool use_default = false;
    SECTION("Absent filter copies every island") { use_default = true; }
    SECTION("Z interval includes its start") {
        filter.print_z_interval = std::make_pair(source.print_z(), source.print_z() + 1);
    }
    SECTION("Z interval excludes its end") {
        filter.print_z_interval = std::make_pair(source.print_z() - 1, source.print_z());
        expected.clear();
    }
    SECTION("Island list selects only one island") {
        selected = {1};
        filter.island_indices = &selected;
        expected = selected;
    }
    SECTION("Empty island list selects nothing") {
        filter.island_indices = &selected;
        expected.clear();
    }

    PrintingPlan native_plan;
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native_plan));
    const uint32_t catalog_id = plan.find_or_add_object_instance(object, 0);
    slic3r_api::PrintingGroup group = plan.append_group();
    slic3r_api::PrintingLayerGroup layer_group = group.append_layer_group(source.print_z());
    if (use_default)
        append_layer_instance_extrusions(plan, layer_group, source, object.instance_shift(0), catalog_id, nullptr);
    else
        append_layer_instance_extrusions(plan, layer_group, source, object.instance_shift(0), catalog_id, nullptr, filter);

    const PrintingLayerGroup &result = native_plan.groups.front().layers.front();
    if (expected.empty()) {
        CHECK(result.tool_groups.empty());
    } else {
        REQUIRE(result.tool_groups.size() == 1);
        const PrintingToolGroup &tool = result.tool_groups.front();
        REQUIRE(tool.extrusions.size() == expected.size());
        REQUIRE(tool.region_islands.size() == expected.size());
        for (size_t index = 0; index < expected.size(); ++index) {
            const LayerRegionIsland *region = source_regions[expected[index]];
            const PrintingExtrusion &extrusion = tool.extrusions[index];
            CHECK(tool.region_islands[index] == region);
            CHECK(extrusion.region_island == region);
            CHECK(extrusion.printing_catalog_id == catalog_id);
            REQUIRE(extrusion.root != nullptr);
            CHECK_FALSE(extrusion.root->empty());
            CHECK(extrusion.root.get() != &region->extrusion(LayerRegionIsland::PERIMETERS));
        }
    }
    for (const LayerRegionIsland *region : source_regions)
        CHECK_FALSE(region->extrusion(LayerRegionIsland::PERIMETERS).empty());
}

// Own a synthetic layer whose islands retain the supplied order for stack tests.
struct IslandStackTestLayer : Layer
{
    // Own polygons at Z with the supplied thickness; the optional source object remains borrowed.
    IslandStackTestLayer(double z, const ExPolygons &slices, PrintObject *object = nullptr, double height = 0.2)
        : Layer(1, object, scale_to_layer_coord(height), scale_to_layer_coord(z), z - height / 2., true)
    {
        for (const ExPolygon &slice : slices)
            m_islands.emplace_back(new LayerSliceIsland(slice));
    }
};

// Add one printable tool bucket to a synthetic island; its layer owns the path.
void add_stack_test_extrusion(LayerSliceIsland &island, extruder_index_t tool);

// Replace the owned plan using the registered builder on borrowed print sources,
// without subsequent orderers changing cloned paths. Host failures propagate.
void run_island_plan_builder(Print &print);

// Check every source role exactly once at its source Z and translated position,
// then check parents and the lead of every previously printed rectangular unit
// from another group against a forward evaluation of the augmented graphs.
// At and above tool_change_z, selected_tool also limits the allowed height lead.
// The print owns all source pointers; Catch reports missing or repeated work.
void check_island_plan_oracles(const Print &print, const DynamicPrintConfig &config,
    coord_t tool_change_z = std::numeric_limits<coord_t>::max(), extruder_index_t selected_tool = EXTRUDER_ID_UNKNOWN);

// Clone synthetic stack selections into an owned plan and apply the supplied
// shared tool orders. Sources must outlive the result; allocation failures propagate.
PrintingPlan island_stack_test_plan(const std::vector<slic3r_api::Ordering::IslandStacks::ObjectsLayerToPrint> &stacks);

// Append an owned rectangular layer with one printable role per island to the
// chosen source list. The returned reference borrows the object's layer lifetime.
Layer &append_stack_scene_layer(PrintObject &object, double z, const ExPolygons &slices, bool auxiliary = false, double height = 0.2);

// A single-Z footprint introduces no higher obstacle, regardless of tool count.
TEST_CASE("Island stacks keep equal-Z footprints separate regardless of tools",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer layer(1., {rectangle_expolygon(0., 0., 1., 1.),
        rectangle_expolygon(8., 0., 9., 1.), rectangle_expolygon(4., 0., 5., 1.)});
    IslandStackTestLayer support(1., {rectangle_expolygon(4., 0., 5., 1.)});
    bool different_tools = false, multi_bridge = false, empty_bridge = false, support_bridge = false;
    SECTION("A third island at the same Z stays separate") {}
    SECTION("Different tools stay separate") { different_tools = true; }
    SECTION("A two-tool footprint stays separate from single-tool islands") { different_tools = multi_bridge = true; }
    SECTION("An island without extrusions is ignored") { empty_bridge = true; }
    SECTION("Support footprints at the same Z stay separate") { support_bridge = true; }
    add_stack_test_extrusion(layer.island(0), 0);
    add_stack_test_extrusion(layer.island(1), different_tools ? 1 : 0);
    if (!empty_bridge && !support_bridge)
        add_stack_test_extrusion(layer.island(2), 0);
    if (multi_bridge)
        add_stack_test_extrusion(layer.island(2), 1);
    if (support_bridge)
        add_stack_test_extrusion(support.island(0), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::GraphType::LINEAR,
        {{0., 0.}, {3.5, 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{
        view.get("extruder_clearance").graph(0).value(), view.get("extruder_clearance").graph(1).value()};
    PluginStorage storage;
    ObjectLayerToPrint input;
    input.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&layer));
    if (support_bridge)
        input.auxiliary_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&support));
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({input},
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == (empty_bridge ? 2 : 3));
    size_t selected = 0;
    for (const ObjectsLayerToPrint &stack : stacks) {
        REQUIRE(stack.size() == 1);
        selected += stack.front().islands.size();
    }
    CHECK(selected == (empty_bridge ? 2 : 3));
    if (support_bridge)
        CHECK(std::count_if(stacks.begin(), stacks.end(), [&input](const ObjectsLayerToPrint &stack) {
            return stack.front().islands.count(input.auxiliary_layer.island(0).handle()) != 0;
        }) == 1);
}

// Lower links join stacks across layers independently of their tools.
// One upper island spans both lower islands, merging their existing stacks.
TEST_CASE("Island stacks follow lower islands including multi-tool stacks",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer lower(1., {rectangle_expolygon(0., 0., 1., 1.), rectangle_expolygon(8., 0., 9., 1.)});
    IslandStackTestLayer upper(1.2, {rectangle_expolygon(0., 0., 9., 1.)});
    bool multiple = false;
    SECTION("Bridge island follows both lower stacks") {}
    SECTION("Multi-tool lower island remains extendible") { multiple = true; }
    add_stack_test_extrusion(lower.island(0), 0);
    add_stack_test_extrusion(lower.island(1), 0);
    add_stack_test_extrusion(upper.island(0), 0);
    if (multiple)
        add_stack_test_extrusion(lower.island(0), 1);
    upper.island(0).overlaps_below = {{&lower.island(0), 1.f}, {&lower.island(1), 1.f}};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::GraphType::LINEAR, {{0., 0.}, {1., 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{
        view.get("extruder_clearance").graph(0).value(), view.get("extruder_clearance").graph(1).value()};
    PluginStorage storage;
    ObjectLayerToPrint low, high;
    low.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&lower));
    high.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&upper));
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high},
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    CHECK(stacks.front().size() == 1);
    CHECK(stacks.back().size() == 2);
    CHECK(stacks.back().back().islands.count(reinterpret_cast<const layer_island_handle *>(&upper.island(0))) == 1);
}

// A current tool above all island tools must not dereference the end of a tool set.
TEST_CASE("Island stacks accept multi-tool islands below the current tool",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer lower(1., {rectangle_expolygon(0., 0., 1., 1.)});
    IslandStackTestLayer upper(1.2, {rectangle_expolygon(0., 0., 1., 1.)});
    add_stack_test_extrusion(lower.island(0), 2);
    add_stack_test_extrusion(upper.island(0), 0);
    add_stack_test_extrusion(upper.island(0), 1);
    upper.island(0).overlaps_below = {{&lower.island(0), 1.f}};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::GraphType::LINEAR, {{0., 0.}, {1., 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value(),
        view.get("extruder_clearance").graph(1).value(), view.get("extruder_clearance").graph(2).value()};
    PluginStorage storage;
    ObjectLayerToPrint low, high;
    low.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&lower));
    high.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&upper));
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high},
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 1);
    REQUIRE(stacks[0].size() == 2);
    CHECK(stacks[0][0].islands == std::set<const layer_island_handle *>{
        reinterpret_cast<const layer_island_handle *>(&lower.island(0))});
    CHECK(stacks[0][1].islands == std::set<const layer_island_handle *>{
        reinterpret_cast<const layer_island_handle *>(&upper.island(0))});
    const PrintingPlan plan = island_stack_test_plan(stacks);
    REQUIRE(plan.groups.size() == 1);
    REQUIRE(plan.groups[0].layers.size() == 2);
    REQUIRE(plan.groups[0].layers[0].tool_groups.size() == 1);
    CHECK(plan.groups[0].layers[0].tool_groups[0].extruder_id == 2);
    REQUIRE(plan.groups[0].layers[1].tool_groups.size() == 2);
    CHECK(plan.groups[0].layers[1].tool_groups[0].extruder_id == 0);
    CHECK(plan.groups[0].layers[1].tool_groups[1].extruder_id == 1);
}

// Separated multi-tool islands keep independent parent identities and exact coverage.
TEST_CASE("Island stacks retain later multi-tool islands as parents",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer lower(1., {rectangle_expolygon(0., 0., 2., 2.), rectangle_expolygon(20., 0., 21., 1.)});
    IslandStackTestLayer upper(1.2, {rectangle_expolygon(20., 0., 21., 1.)});
    for (size_t idx = 0; idx < 2; ++idx) {
        add_stack_test_extrusion(lower.island(idx), 0);
        add_stack_test_extrusion(lower.island(idx), 1);
    }
    add_stack_test_extrusion(upper.island(0), 0);
    upper.island(0).overlaps_below = {{&lower.island(1), 1.f}};
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::GraphType::LINEAR, {{0., 0.}, {1., 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{
        view.get("extruder_clearance").graph(0).value(), view.get("extruder_clearance").graph(1).value()};
    PluginStorage storage;
    ObjectLayerToPrint low, high;
    low.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&lower));
    high.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&upper));
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high},
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    REQUIRE(stacks.front().size() == 1);
    REQUIRE(stacks.back().size() == 2);
    std::map<const layer_island_handle *, size_t> visits;
    for (const ObjectsLayerToPrint &stack : stacks)
        for (const ObjectLayerToPrint &selection : stack)
            for (const layer_island_handle *island : selection.islands)
                ++visits[island];
    CHECK(visits.size() == 3);
    CHECK(visits[reinterpret_cast<const layer_island_handle *>(&lower.island(0))] == 1);
    CHECK(visits[reinterpret_cast<const layer_island_handle *>(&lower.island(1))] == 1);
    CHECK(visits[reinterpret_cast<const layer_island_handle *>(&upper.island(0))] == 1);
}

// An earlier lower stack does not obstruct a later higher single-layer copy.
TEST_CASE("Island stacks do not group a later copy above all earlier geometry",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    bool far = false, above_clearance = false;
    SECTION("A gap below clearance at max_height is safe in increasing Z") {}
    SECTION("Far copies stay separate across different Z") { far = true; }
    SECTION("A gap above clearance at max_height stays separate") { above_clearance = true; }
    IslandStackTestLayer lower(1., {rectangle_expolygon(0., 0., 1., 1.)});
    IslandStackTestLayer upper(1.1, {rectangle_expolygon(0., 0., 1., 1.)});
    add_stack_test_extrusion(lower.island(0), 0);
    add_stack_test_extrusion(upper.island(0), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {2., 1.}})}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value()};
    ObjectLayerToPrint low, high;
    low.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&lower));
    high.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&upper));
    high.instance_id = 1;
    high.shift.x = scale_i(far ? 10. : above_clearance ? 1.9 : 1.7);
    PluginStorage storage;
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({high, low},
        scale_i(0.4), graphs, EXTRUDER_ID_UNKNOWN, reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    REQUIRE(stacks.front().size() == 1);
    REQUIRE(stacks.back().size() == 1);
    CHECK(stacks.front()[0].instance_id == 0);
    CHECK(stacks.back()[0].instance_id == 1);
}

// Tool preferences reorder independent stacks only after their clearance groups exist.
TEST_CASE("Island stacks continue the previous end tool instead of alternating",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    bool mixed_copy = false;
    SECTION("Independent single-tool stacks avoid alternating") {}
    SECTION("A single-tool copy stays separate from an equal-Z two-tool copy") { mixed_copy = true; }
    IslandStackTestLayer layer(1., {rectangle_expolygon(0., 0., 1., 1.),
        rectangle_expolygon(20., 0., 21., 1.), rectangle_expolygon(40., 0., 41., 1.),
        rectangle_expolygon(60., 0., 61., 1.)});
    IslandStackTestLayer other(1., {rectangle_expolygon(1.5, 0., 2.5, 1.)});
    for (size_t idx = 0; idx < 4; ++idx)
        add_stack_test_extrusion(layer.island(idx), extruder_index_t(idx % 2));
    if (mixed_copy) {
        add_stack_test_extrusion(layer.island(0), 1);
        add_stack_test_extrusion(other.island(0), 0);
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::LINEAR, {{0., 0.}, {1., 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value(),
        view.get("extruder_clearance").graph(1).value()};
    ObjectLayerToPrint input;
    input.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&layer));
    ObjectsLayerToPrint inputs{input};
    if (mixed_copy) {
        ObjectLayerToPrint copy;
        copy.instance_id = 1;
        copy.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&other));
        inputs.push_back(copy);
    }
    PluginStorage storage;
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands(inputs,
        scale_i(0.4), graphs, EXTRUDER_ID_UNKNOWN, reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == (mixed_copy ? 5 : 4));
    const PrintingPlan plan = island_stack_test_plan(stacks);
    REQUIRE(plan.groups.size() == (mixed_copy ? 5 : 4));
    std::vector<extruder_index_t> tools;
    for (size_t idx = 0; idx < stacks.size(); ++idx) {
        const ObjectsLayerToPrint &stack = stacks[idx];
        REQUIRE(stack.size() == 1);
        REQUIRE(plan.groups[idx].layers.size() == 1);
        const std::vector<PrintingToolGroup> &tool_groups = plan.groups[idx].layers.front().tool_groups;
        REQUIRE(tool_groups.size() == ((mixed_copy && idx == 0) ? 2 : 1));
        tools.push_back(tool_groups.back().extruder_id);
    }
    if (mixed_copy)
        CHECK(tools == std::vector<extruder_index_t>{1, 1, 1, 0, 0});
    else
        CHECK(tools == std::vector<extruder_index_t>{0, 0, 1, 1});
}

// Continuing a tool must not reorder closed stacks beyond the configured height step.
TEST_CASE("Island stack tool preferences preserve the height step",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    std::vector<std::unique_ptr<IslandStackTestLayer>> layers;
    ObjectsLayerToPrint inputs;
    for (size_t idx = 0; idx < 8; ++idx) {
        layers.emplace_back(new IslandStackTestLayer(1. + 0.2 * idx,
            {rectangle_expolygon(0., 0., 1., 1.), rectangle_expolygon(20., 0., 21., 1.)}));
        for (size_t island = 0; island < 2; ++island) {
            add_stack_test_extrusion(layers.back()->island(island), extruder_index_t(island));
            if (idx != 0)
                layers.back()->island(island).overlaps_below = {{&layers[idx - 1]->island(island), 1.f}};
        }
        ObjectLayerToPrint input;
        input.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(layers.back().get()));
        inputs.push_back(input);
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::LINEAR, {{0., 0.}, {1., 10.}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value(),
        view.get("extruder_clearance").graph(1).value()};
    PluginStorage storage;
    const coord_t step = scale_i(0.4);
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands(inputs,
        step, graphs, EXTRUDER_ID_UNKNOWN, reinterpret_cast<storage_handle *>(&storage));
    coord_t highest[2] = {0, 0};
    std::set<const layer_island_handle *> visited;
    for (const ObjectsLayerToPrint &stack : stacks)
        for (const ObjectLayerToPrint &selection : stack)
            for (uint32_t idx = 0; idx < selection.layer().island_count(); ++idx) {
                const slic3r_api::LayerIsland island = selection.layer().island(idx);
                if (selection.islands.count(island.handle()) == 0)
                    continue;
                const extruder_index_t tool = island.region_island(0).extruder_id();
                REQUIRE(tool >= 0);
                REQUIRE(tool < 2);
                CHECK(visited.insert(island.handle()).second);
                CHECK(selection.layer().print_z() > highest[tool]);
                CHECK(highest[1 - tool] - selection.layer().print_z() <= step);
                highest[tool] = selection.layer().print_z();
            }
    CHECK(visited.size() == 16);
}

PrintingPlan island_stack_test_plan(const std::vector<slic3r_api::Ordering::IslandStacks::ObjectsLayerToPrint> &stacks)
{
    using namespace slic3r_api::Ordering::PlanBuilderCommon;
    PrintingPlan native;
    for (const slic3r_api::Ordering::IslandStacks::ObjectsLayerToPrint &stack : stacks)
        for (const slic3r_api::Ordering::IslandStacks::ObjectLayerToPrint &selection : stack)
            native.object_instances.resize(std::max(native.object_instances.size(), size_t(selection.instance_id) + 1));
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native));
    for (const slic3r_api::Ordering::IslandStacks::ObjectsLayerToPrint &stack : stacks) {
        slic3r_api::PrintingGroup group = plan.append_group();
        std::map<coord_t, uint32_t> indices;
        std::set<uint32_t> instances;
        for (const slic3r_api::Ordering::IslandStacks::ObjectLayerToPrint &selection : stack) {
            if (instances.insert(selection.instance_id).second)
                group.append_printing_catalog_id(plan, selection.instance_id);
            for (const slic3r_api::Layer &source : {selection.object_layer, selection.auxiliary_layer}) {
                if (!source.valid())
                    continue;
                std::vector<uint32_t> islands;
                for (uint32_t idx = 0; idx < source.island_count(); ++idx)
                    if (selection.islands.count(source.island(idx).handle()))
                        islands.push_back(idx);
                LayerExtrusionFilter filter;
                filter.island_indices = &islands;
                slic3r_api::PrintingLayerGroup layer = layer_group_for_print_z(group, indices, source.print_z(), nullptr);
                append_layer_once(layer, source, nullptr);
                append_layer_instance_extrusions(plan, layer, source, selection.shift, selection.instance_id, nullptr, filter);
            }
        }
        std::set<coord_t> ordered_z;
        for (const slic3r_api::Ordering::IslandStacks::ObjectLayerToPrint &selection : stack) {
            const coord_t z = selection.layer().print_z();
            if (!ordered_z.insert(z).second)
                continue;
            slic3r_api::PrintingLayerGroup layer = group.layer_group(indices.at(z));
            for (uint32_t target = 0; target < selection.extruders_order.size(); ++target)
                for (uint32_t tool = target; tool < layer.tool_group_count(); ++tool)
                    if (layer.tool_group(tool).extruder_id() == selection.extruders_order[target]) {
                        layer.move_tool_group(tool, target);
                        break;
                    }
        }
        sort_layer_groups_by_print_z(group, nullptr);
    }
    return native;
}

void add_stack_test_extrusion(LayerSliceIsland &island, extruder_index_t tool)
{
    LayerRegionIsland &region = island.add_region_island({}, tool);
    region.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(
        test_path({Point(0, 0), Point(scale_i(1.), 0)}));
}

// Owned auxiliaries contribute their own island units without leaking to another copy.
TEST_CASE("Island stacks include owned auxiliary units for their instance only",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::pair<double, double> cases[] = {{0.3, 0.}, {0.8, 0.}, {1.6, 0.}, {0.25, 0.3}, {0.8, 0.7}};
    for (const std::pair<double, double> &test : cases) {
        const double z = test.first, limit = test.second;
        CAPTURE(z, limit);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", "0.5"}, {"complete_islands", "1"}, {"complete_objects_one_first_layer", "1"}, {"layer_height", "0.2"},
            {"complete_objects_step_max_z", std::to_string(limit)},
            {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"}});
        Model model;
        Print print;
        TriangleMesh mesh = make_cube(3., 3., 1.2);
        TriangleMesh second = make_cube(3., 3., 1.2);
        second.translate(Vec3f(20.f, 0.f, 0.f));
        mesh.merge(second);
        Test::init_print({mesh}, print, model, config);
        model.objects().front().instances.front()->set_offset(Vec3d(30., 30., 0.));
        model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(Vec3d(100., 30., 0.));
        print.apply(model, config);
        print.process();
        const coord_t owned_z = scale_to_layer_coord(z);
        PluginStorage storage;
        const ExPolygons subject{rectangle_expolygon(300., 0., 301., 1.)};
        const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view(print), print_view(print).object(0),
            expolygons_view(subject), scale_i(0.2), owned_z, scale_i(z - 0.1));
        REQUIRE(built.created);
        built.layer.mutable_properties().get_or_add(slic3r_api::LayerInstanceProperty::key).object_instance_idx = 1;
        const slic3r_api::LayerRegionIsland region = built.layer.island(0).get_or_create_full_region_island(0);
        extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
            const_cast<layer_region_island_handle *>(region.handle()), RAW_EXTRUSION_ROLE_PERIMETER);
        REQUIRE(root_handle != nullptr);
        reinterpret_cast<ExtrusionEntityCollection *>(root_handle)->append(test_path({Point(scale_i(300.), 0), Point(scale_i(301.), 0)}));
        run_island_plan_builder(print);
        check_island_plan_oracles(print, config);
        const PrintingPlan &actual = *print.printing_plan();
        size_t visits = 0;
        for (size_t idx = 0; idx < actual.groups.size(); ++idx)
            for (const PrintingLayerGroup &layer : actual.groups[idx].layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.region_island == reinterpret_cast<const LayerRegionIsland *>(region.handle())) {
                            ++visits;
                            // An unlinked auxiliary starts a stack of its own,
                            // except above max_z where it belongs to the shared tail.
                            if (limit == 0. || z < limit) {
                                CHECK(actual.groups[idx].layers.front().print_z == owned_z);
                                CHECK(actual.groups[idx].layers.back().print_z == owned_z);
                            } else {
                                CHECK(idx + 1 == actual.groups.size());
                            }
                            CHECK(layer.print_z == owned_z);
                            const PrintingObjectInstance &instance = actual.object_instances[extrusion.printing_catalog_id];
                            CHECK(instance.object_instance_idx == 1);
                            REQUIRE(extrusion.root != nullptr);
                            CHECK(extrusion.root->first_point() == print.object(0).instances()[1].shift + Point(scale_i(300.), 0));
                        }
        CHECK(visits == 1);
    }
}

// Shared handles on copies must remain distinct, and different source layer grids
// must meet the same physical clearance rule as equal-Z copies.
TEST_CASE("Plate-wide island stacks group close copies and different layer grids",
          "[plugins][printing-plan][parallel-islands][island-stacks][plate-island-units]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    bool far = false, grids = false, multi_tools = false;
    double limit = 0.;
    double step = 0.6;
    SECTION("Close copies share island stacks") {}
    SECTION("Far copies keep independent stacks") { far = true; }
    SECTION("Far copies remain independent when the step reaches the global height") { far = true; step = 1.2; }
    SECTION("Close objects have different layer grids") { grids = true; }
    SECTION("A shared stack retains both tools at every layer") { multi_tools = true; }
    SECTION("Close copies have a shared by-layer tail") { limit = 0.9; }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", std::to_string(step)}, {"complete_islands", "1"}, {"complete_objects_one_first_layer", "1"}, {"layer_height", "0.2"},
        {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"complete_objects_step_max_z", std::to_string(limit)},
        {"extruder_clearance_radius", "10"}, {"extruder_clearance_height", "1"}});
    const GraphData graph(0, 2, GraphData::LINEAR, {{0., 0.}, {4., 0.8}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    Model model;
    Print print;
    const TriangleMesh mesh = make_cube(2., 2., 2.4);
    Test::init_print(grids ? std::vector<TriangleMesh>{mesh, mesh} : std::vector<TriangleMesh>{mesh},
        print, model, config);
    model.objects().front().instances.front()->set_offset(Vec3d(30., 30., 0.));
    if (grids) {
        model.objects()[1].instances.front()->set_offset(Vec3d(33., 30., 0.));
        model.objects()[1].config.set_key_value("layer_height", new ConfigOptionFloat(0.3));
    } else {
        model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(
            Vec3d(far ? 100. : 33., 30., 0.));
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    if (multi_tools)
        for (Layer &layer : print.object(0).layers())
            add_stack_test_extrusion(layer.island(0), 1);
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() > 1);
    bool shared = false;
    for (size_t idx = 1; idx < plan.groups.size(); ++idx) {
        if (limit > 0. && idx + 1 == plan.groups.size())
            break;
        shared |= plan.groups[idx].printing_catalog_ids.size() == 2;
        if (far)
            CHECK(plan.groups[idx].printing_catalog_ids.size() == 1);
    }
    CHECK(shared == !far);
    if (multi_tools) {
        const extruder_index_t first_pass_end = plan.groups[0].layers.back().tool_groups.back().extruder_id;
        extruder_index_t previous = first_pass_end;
        for (size_t idx = 1; idx < plan.groups.size(); ++idx)
            for (const PrintingLayerGroup &layer : plan.groups[idx].layers) {
                REQUIRE(layer.tool_groups.size() == 2);
                if (previous != EXTRUDER_ID_UNKNOWN)
                    CHECK(layer.tool_groups.front().extruder_id == previous);
                previous = layer.tool_groups.back().extruder_id;
            }
    }
    if (grids) {
        REQUIRE(print.objects().size() == 2);
        CHECK(print.object(0).layers()[1].scaled_print_z() != print.object(1).layers()[1].scaled_print_z());
    }
    if (limit > 0.) {
        REQUIRE(plan.groups.back().printing_catalog_ids.size() == 2);
        for (const PrintingLayerGroup &layer : plan.groups.back().layers)
            CHECK(unscaled(layer.print_z) >= limit - 0.00001);
    }
}

// Literal plate layouts distinguish complete copies, height-limited stacks,
// shared first layers and the final by-layer tail, while source oracles check all work.
TEST_CASE("Whole-copy units use the plate-wide stack process in every mode",
          "[plugins][printing-plan][ordering][print-sequence][object-step][plate-copy-units]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    struct Case { double step; double limit; bool first; bool close; size_t groups; };
    const Case cases[] = {{0., 0., false, false, 2}, {0., 0., false, true, 1},
        {0.4, 0., false, false, 3}, {0., 0., true, false, 3}, {0., 0., true, true, 2},
        {0., 0.5, false, false, 3}, {0.4, 0.5, false, false, 3},
        {0., 0.1, false, false, 1}, {0., 0.1, true, false, 2}};
    for (const Case &test : cases) {
        CAPTURE(test.step, test.limit, test.first, test.close);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_sort", "object"}, {"complete_islands", "0"},
            {"complete_objects_step", std::to_string(test.step)},
            {"complete_objects_step_max_z", std::to_string(test.limit)},
            {"complete_objects_one_first_layer", test.first ? "1" : "0"},
            {"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "0"},
            {"brim_width", "0"}, {"wipe_tower", "0"}, {"perimeters", "1"},
            {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
            {"extruder_clearance_radius", "10"}, {"extruder_clearance_height", "1"}});
        config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
            GraphData::LINEAR, {{0., 0.}, {4., 0.8}})}));
        Model model;
        Print print;
        TriangleMesh mesh = make_cube(2., 2., 0.8);
        TriangleMesh island = make_cube(2., 2., 0.8);
        island.translate(Vec3f(20.f, 0.f, 0.f));
        mesh.merge(island);
        Test::init_print({mesh}, print, model, config);
        model.objects().front().instances.front()->set_offset(Vec3d(30., 30., 0.));
        model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(
            Vec3d(30., test.close ? 33. : 100., 0.));
        print.apply(model, config);
        print.set_status_silent();
        print.process();
        run_island_plan_builder(print);
        check_island_plan_oracles(print, config);
        const PrintingPlan &plan = *print.printing_plan();
        REQUIRE(plan.groups.size() == test.groups);
        if (test.first) {
            REQUIRE(plan.groups.front().printing_catalog_ids.size() == 2);
            REQUIRE(plan.groups.front().layers.size() == 1);
            CHECK(plan.groups.front().layers.front().print_z == scale_to_layer_coord(0.2));
        }
        if (test.limit > 0.) {
            REQUIRE(plan.groups.back().printing_catalog_ids.size() == 2);
            for (const PrintingLayerGroup &layer : plan.groups.back().layers)
                CHECK(unscaled(layer.print_z) >= test.limit - 0.00001);
            for (size_t idx = test.first ? 1 : 0; idx + 1 < plan.groups.size(); ++idx)
                for (const PrintingLayerGroup &layer : plan.groups[idx].layers)
                    CHECK(unscaled(layer.print_z) < test.limit);
        } else if (!test.close) {
            for (size_t idx = test.first ? 1 : 0; idx < plan.groups.size(); ++idx)
                REQUIRE(plan.groups[idx].printing_catalog_ids.size() == 1);
            if (test.step == 0.)
                for (size_t copy = 0; copy < 2; ++copy) {
                    const PrintingGroup &group = plan.groups[copy + (test.first ? 1 : 0)];
                    CHECK(plan.object_instances[group.printing_catalog_ids.front()].object_instance_idx == copy);
                    CHECK(group.layers.size() == (test.first ? 3 : 4));
                    for (const PrintingLayerGroup &layer : group.layers)
                        CHECK(layer.tool_groups.front().extrusions.size() == 4);
                }
        }
    }
}

// Height rows split every copy and global skirt at the same scaled Z, even when
// later consumers are not invoked. Independent source checks retain every role,
// parent and geometric clearance constraint across the resulting groups.
TEST_CASE("Height G-code barriers partition the complete By object plan",
          "[plugins][printing-plan][ordering][object-step][custom-gcode][height-bands]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool islands = GENERATE(false, true);
    const CustomGCode::Type barrier_type = GENERATE(CustomGCode::PausePrint, CustomGCode::ColorChange,
        CustomGCode::ToolChange, CustomGCode::Template, CustomGCode::Custom);
    bool prefix = false, tail = false, grids = false, skirt = false, tool_change = false, source_large = false;
    bool production_skirt = false, multi_extruder = false, incompatible = false, stale = false, sorted_prefix = false;
    bool draft_shield = false, single_record = false;
    double barrier = 1.;
    SECTION("All event kinds and repeated heights") {}
    SECTION("Barrier intersects the shared prefix") { prefix = true; grids = true; barrier = 0.3; }
    SECTION("Split prefix preserves copy sorting") { prefix = true; grids = true; barrier = 0.3; sorted_prefix = true; }
    SECTION("Barrier intersects the by-layer tail") { tail = true; }
    SECTION("Copies use different layer grids") { grids = true; }
    SECTION("Global skirt spans the barrier") { skirt = true; }
    SECTION("Production skirt keeps its published source Z") { skirt = true; production_skirt = true; }
    SECTION("Production draft shield keeps its published source Z") { skirt = true; production_skirt = true; draft_shield = true; }
    SECTION("Selected tool enlarges clearance") { tool_change = true; }
    SECTION("Source tool retains its larger clearance") { tool_change = true; source_large = true; }
    SECTION("MultiExtruder plan does not widen clearance") { tool_change = true; multi_extruder = true; grids = true; }
    SECTION("Incompatible record mode does not widen clearance") { tool_change = true; incompatible = true; }
    SECTION("SingleExtruder record does not widen clearance") { tool_change = true; incompatible = true; single_record = true; }
    SECTION("Disabled consumers accept a stale tool number") { tool_change = true; stale = true; }
    CAPTURE(islands, prefix, tail, grids, skirt, tool_change);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_islands", islands ? "1" : "0"}, {"complete_objects_step", "0.6"},
        {"complete_objects_one_first_layer", prefix ? "1" : "0"},
        {"complete_objects_step_max_z", tail ? "0.7" : "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", skirt ? "1" : "0"}, {"skirt_height", "8"}, {"skirt_distance", "2"},
        {"skirt_per_object", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"extruder_clearance_radius", skirt ? "0" : "10"},
        {"extruder_clearance_height", "10"}});
    const GraphData small(0, 2, GraphData::LINEAR, {{0., 0.}, {1., 10.}});
    const GraphData large(0, 2, GraphData::LINEAR, {{0., 0.}, {9., 0.1}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs(source_large ?
        std::vector<GraphData>{large, small, large} : std::vector<GraphData>{small, multi_extruder ? small : large, large}));
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.4,0.4");
    config.set_deserialize_strict("min_layer_height", "0.05,0.05,0.05");
    config.set_deserialize_strict("max_layer_height", "0.3,0.3,0.3");
    if (sorted_prefix)
        config.set_deserialize_strict("complete_objects_sort", "lowy");
    if (draft_shield)
        config.set_deserialize_strict("draft_shield", "enabled");
    Model model;
    Print print;
    const TriangleMesh mesh = make_cube(2., 2., 1.6);
    Test::init_print(grids ? std::vector<TriangleMesh>{mesh, mesh} : std::vector<TriangleMesh>{mesh}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., sorted_prefix ? 100. : 30., 0.));
    if (grids) {
        model.objects()[1].instances.front()->set_offset(Vec3d(multi_extruder ? 35. : 100., 30., 0.));
        model.objects()[1].config.set_deserialize_strict("layer_height", "0.3");
        if (multi_extruder)
            model.objects()[1].config.set_deserialize_strict("perimeter_extruder", "2");
        if (prefix)
            model.objects()[1].config.set_deserialize_strict("first_layer_height", "0.4");
    } else {
        model.objects()[0].add_instance(*model.objects()[0].instances.front())->set_offset(
            Vec3d(tool_change ? 35. : 100., 30., 0.));
    }
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    // Publish after slicing so this probe exercises the builder, not event consumers.
    // This synthetic regression materializes the reusable skirt outline at later
    // Z values. The separate production case leaves the generated sources intact.
    if (skirt && !production_skirt) {
        PrintObject &auxiliary = print.mutable_auxiliary_object();
        REQUIRE_FALSE(auxiliary.auxiliary_layers().empty());
        const Layer &source = auxiliary.auxiliary_layers().front();
        const ExPolygons slices = source.lslices();
        for (double z : {0.6, 1., 1.4}) {
            std::unique_ptr<IslandStackTestLayer> layer(new IslandStackTestLayer(z, slices, &auxiliary, 0.2));
            for (size_t idx = 0; idx < source.islands().size(); ++idx)
                for (const LayerRegionIsland &source_region : source.island(idx).regions_islands()) {
                    LayerRegionIsland &region = layer->island(idx).add_region_island({}, 0);
                    for (const auto &[role, root] : source_region.extrusion_buckets())
                        region.mutable_extrusion(role).append(root);
                }
            auxiliary.mutable_auxiliary_layers().push_back(std::move(layer));
        }
    }
    CustomGCode::Info events;
    events.mode = incompatible ? (single_record ? CustomGCode::SingleExtruder : CustomGCode::MultiExtruder) : CustomGCode::MultiAsSingle;
    const CustomGCode::Type type = tool_change ? CustomGCode::ToolChange : barrier_type;
    const int selected = stale ? 99 : multi_extruder ? 3 : tool_change ? 2 : 1;
    for (size_t duplicate = 0; duplicate < 2; ++duplicate)
        events.gcodes.push_back({scale_to_layer_coord(barrier), type, duplicate == 1 ? selected : 1, "#FF0000", "M117 barrier"});
    events.gcodes.push_back({scale_to_layer_coord(0.01), CustomGCode::Custom, 1, "", "M117 before"});
    events.gcodes.push_back({scale_to_layer_coord(5.), CustomGCode::Custom, 1, "", "M117 above"});
    std::stable_sort(events.gcodes.begin(), events.gcodes.end());
    DynamicConfig record = CustomGCode::make_print_record(events);
    print.records().get_or_add(CustomGCode::PrintRecordChannel).swap(record);
    Orchestrator &host = Orchestrator::instance();
    struct ConsumerActivationGuard {
        Orchestrator &host;
        std::array<std::pair<Plugin *, bool>, 3> states{};
        // Snapshot all three borrowed plugin handles before disabling any of them.
        // The caller keeps the host alive until restoration during scope exit.
        ConsumerActivationGuard(Orchestrator &host, bool disable) : host(host)
        {
            if (!disable)
                return;
            size_t idx = 0;
            for (const char *id : {"ordering.custom_gcode_tool_overrides", "ordering.custom_gcode_event_tools", "gcode.custom_gcode_per_print_z"}) {
                Plugin *plugin = host.get_plugin(id);
                states[idx++] = {plugin, host.active_plugins().count(plugin) != 0};
            }
            for (const std::pair<Plugin *, bool> &state : states)
                host.set_plugin_active(state.first, false);
        }
        // Restore each saved activation state on normal exit or assertion/exception
        // unwinding; null entries mean this guard made no change.
        ~ConsumerActivationGuard()
        {
            for (const std::pair<Plugin *, bool> &state : states)
                if (state.first != nullptr)
                    host.set_plugin_active(state.first, state.second);
        }
    };
    {
        ConsumerActivationGuard guard(host, stale);
        run_island_plan_builder(print);
    }
    const extruder_index_t clearance_tool = incompatible || multi_extruder ? EXTRUDER_ID_UNKNOWN : stale ? 2 : tool_change ? 1 : 0;
    check_island_plan_oracles(print, config, scale_to_layer_coord(barrier), clearance_tool);
    const PrintingPlan &plan = *print.printing_plan();
    if (multi_extruder) {
        std::set<extruder_index_t> tools;
        for (const PrintingGroup &group : plan.groups)
            for (const PrintingLayerGroup &layer : group.layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    if (!tool.extrusions.empty())
                        tools.insert(tool.extruder_id);
        CHECK(tools == (std::set<extruder_index_t>{0, 1}));
    }
    if (sorted_prefix) {
        REQUIRE(plan.groups.size() >= 2);
        const PrintingGroup &prefix_group = plan.groups[1];
        REQUIRE(prefix_group.printing_catalog_ids.size() == 2);
        CHECK(plan.object_instances[prefix_group.printing_catalog_ids[0]].object == &print.object(1));
        CHECK(plan.object_instances[prefix_group.printing_catalog_ids[1]].object == &print.object(0));
        REQUIRE_FALSE(prefix_group.layers.empty());
        REQUIRE_FALSE(prefix_group.layers.front().tool_groups.empty());
        const PrintingToolGroup &tool = prefix_group.layers.front().tool_groups.front();
        REQUIRE_FALSE(tool.extrusions.empty());
        CHECK(tool.extrusions.front().printing_catalog_id == prefix_group.printing_catalog_ids[0]);
    }
    bool reached = false;
    size_t below = 0, above = 0, skirt_below = 0, skirt_above = 0;
    for (const PrintingGroup &group : plan.groups) {
        REQUIRE_FALSE(group.layers.empty());
        const bool upper_group = group.layers.front().print_z >= scale_to_layer_coord(barrier);
        if (upper_group && (incompatible || multi_extruder))
            CHECK(group.printing_catalog_ids.size() == 1);
        for (const PrintingLayerGroup &layer : group.layers) {
            const bool upper = layer.print_z >= scale_to_layer_coord(barrier);
            CHECK(upper == upper_group);
            if (upper) { reached = true; ++above; }
            else { CHECK_FALSE(reached); ++below; }
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions)
                    if (extrusion.printing_catalog_id == PRINTING_OBJECT_INSTANCE_ID_NONE) {
                        if (upper) ++skirt_above;
                        else ++skirt_below;
                    }
        }
    }
    CHECK(below > 0);
    CHECK(above > 0);
    if (skirt && !production_skirt) {
        CHECK(skirt_below > 0);
        CHECK(skirt_above > 0);
    }
    if (production_skirt) {
        REQUIRE(print.auxiliary_object() != nullptr);
        REQUIRE_FALSE(print.auxiliary_object()->auxiliary_layers().empty());
        for (const Layer &layer : print.auxiliary_object()->auxiliary_layers())
            CHECK(layer.scaled_print_z() == scale_to_layer_coord(0.2));
        CHECK(skirt_below > 0);
        CHECK(skirt_above == 0);
    }
    // Same-Z rows do not create repeated groups; empty outer bands create none.
    const size_t groups = plan.groups.size();
    events.gcodes = {{scale_to_layer_coord(barrier), type, selected, "", ""}};
    record = CustomGCode::make_print_record(events);
    print.records().get_or_add(CustomGCode::PrintRecordChannel).swap(record);
    run_island_plan_builder(print);
    CHECK(print.printing_plan()->groups.size() == groups);
}

// Different first-layer heights require one shared prefix across close copies;
// the whole-plan oracles verify coverage, parents and clearance after that prefix.
TEST_CASE("Shared first layers reach the highest first layer across close copies",
          "[plugins][printing-plan][ordering][object-step][plate-copy-units]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool global_brim = GENERATE(false, true);
    CAPTURE(global_brim);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_one_first_layer", "1"}, {"complete_islands", "0"},
        {"complete_objects_step", "0"}, {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "10"}, {"extruder_clearance_height", "1"}});
    if (global_brim)
        config.set_deserialize_strict({{"brim_width", "1"}, {"brim_per_object", "0"},
            {"complete_objects_step_max_z", "0.1"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {4., 0.8}})}));
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 0.8), make_cube(2., 2., 0.8)}, print, model, config);
    model.objects()[0].instances.front()->set_offset(Vec3d(30., 30., 0.));
    model.objects()[1].instances.front()->set_offset(Vec3d(30., 33., 0.));
    model.objects()[1].config.set_deserialize_strict("first_layer_height", "0.4");
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE(print.object(0).layer(0).scaled_print_z() != print.object(1).layer(0).scaled_print_z());
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.front().layers.size() == 2);
    CHECK(plan.groups.front().layers[0].print_z == scale_to_layer_coord(0.2));
    CHECK(plan.groups.front().layers[1].print_z == scale_to_layer_coord(0.4));
    REQUIRE(plan.groups.front().printing_catalog_ids.size() == 2);
    if (global_brim) {
        size_t global_extrusions = 0;
        for (size_t idx = 0; idx < plan.groups.size(); ++idx)
            for (const PrintingLayerGroup &layer : plan.groups[idx].layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.printing_catalog_id == PRINTING_OBJECT_INSTANCE_ID_NONE) {
                            ++global_extrusions;
                            CHECK(idx == 0);
                            CHECK(layer.print_z == scale_to_layer_coord(0.2));
                        }
        REQUIRE(global_extrusions > 0);
    }
    for (size_t idx = 1; idx < plan.groups.size(); ++idx)
        for (const PrintingLayerGroup &layer : plan.groups[idx].layers)
            CHECK(layer.print_z > scale_to_layer_coord(0.4));
}

// Ready stacks prefer the current tool even when its copy has a later sort rank.
TEST_CASE("Ready stacks continue the tool before copy rank",
          "[plugins][printing-plan][ordering][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer first(0.2, {rectangle_expolygon(0., 0., 1., 1.)});
    IslandStackTestLayer second(0.2, {rectangle_expolygon(20., 0., 21., 1.)});
    add_stack_test_extrusion(first.island(0), 0);
    add_stack_test_extrusion(second.island(0), 1);
    ObjectLayerToPrint a, b;
    a.instance_id = 0;
    a.copy_rank = 0;
    a.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&first));
    b.instance_id = 1;
    b.copy_rank = 1;
    b.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&second));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::LINEAR, {{0., 0.}, {1., 1.}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value(),
        view.get("extruder_clearance").graph(1).value()};
    for (const bool islands : {false, true}) {
        PluginStorage storage;
        const std::vector<ObjectsLayerToPrint> stacks = separate_islands({a, b}, scale_i(1.), graphs, 1,
            reinterpret_cast<storage_handle *>(&storage), islands);
        REQUIRE(stacks.size() == 2);
        CHECK(stacks[0].front().instance_id == 1);
        CHECK(stacks[1].front().instance_id == 0);
    }
}

// Independent source grids and widening layers separate child exhaustion,
// actual top admission and historical clearance. All scenes check the whole plan.
TEST_CASE("Stack construction follows children and actual layer heights",
          "[plugins][printing-plan][ordering][island-stacks][stack-rules]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool islands = GENERATE(false, true);
    CAPTURE(islands);
    enum class Scene { Offset, Thick, Jump, Pyramid };
    Scene scene = Scene::Offset;
    SECTION("Offset 0.2 and 0.3 grids do not fragment far copies") {}
    SECTION("A thick continuation fits above a newer stack") { scene = Scene::Thick; }
    SECTION("A thin layer after a Z jump starts at the end") { scene = Scene::Jump; }
    SECTION("An inverted pyramid is checked against lower history") { scene = Scene::Pyramid; }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_one_first_layer", "0"}, {"complete_objects_sort", "object"},
        {"complete_islands", islands ? "1" : "0"},
        {"complete_objects_step", scene == Scene::Thick ? "0.4" : "1"},
        {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "10"}, {"extruder_clearance_height", "2"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {10., 2.}})}));
    Model model;
    Print print;
    Test::init_print({make_cube(2., 2., 0.8), make_cube(2., 2., 0.8)}, print, model, config);
    for (ModelObject &object : model.objects())
        object.instances.front()->set_offset(Vec3d(30., 30., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan())).clear();
    for (PrintObject &object : print.objects()) {
        object.mutable_layers().clear();
        object.mutable_auxiliary_layers().clear();
    }
    Layer *previous[2] = {nullptr, nullptr};
    const auto add = [&](size_t copy, double z, double height, double width) {
        const double x = copy == 0 ? 0. : scene == Scene::Pyramid ? 21. : 100.;
        Layer &layer = append_stack_scene_layer(print.object(copy), z,
            {rectangle_expolygon(x, 0., x + width, 1.)}, false, height);
        if (previous[copy] != nullptr)
            layer.island(0).overlaps_below = {{&previous[copy]->island(0), 1.f}};
        previous[copy] = &layer;
    };
    if (scene == Scene::Offset) {
        for (size_t idx = 1; idx <= 6; ++idx)
            add(0, 0.2 * idx, 0.2, 1.);
        for (size_t idx = 1; idx <= 4; ++idx)
            add(1, 0.3 * idx, 0.3, 1.);
    } else if (scene == Scene::Thick) {
        add(0, 0.4, 0.4, 1.);
        add(1, 0.5, 0.2, 1.);
        add(0, 0.8, 0.4, 1.);
    } else if (scene == Scene::Jump) {
        add(0, 0.4, 0.2, 1.);
        add(0, 1., 0.2, 1.);
        add(1, 0.8, 0.2, 1.);
        add(0, 2., 0.1, 1.);
    } else {
        for (size_t idx = 1; idx <= 3; ++idx) {
            add(0, 0.2 * idx, 0.2, idx == 1 ? 1. : idx == 2 ? 10. : 20.);
            if (idx == 1)
                add(1, 0.2, 0.2, 1.);
        }
    }
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() == (scene == Scene::Jump ? 3 : scene == Scene::Pyramid ? 1 : 2));
    if (scene == Scene::Offset || scene == Scene::Thick) {
        CHECK(plan.groups[0].layers.size() == (scene == Scene::Offset ? 6 : 2));
        CHECK(plan.groups[1].layers.size() == (scene == Scene::Offset ? 4 : 1));
    } else if (scene == Scene::Jump) {
        CHECK(plan.groups[0].layers.size() == 2);
        CHECK(plan.groups[1].layers.front().print_z == scale_to_layer_coord(0.8));
        CHECK(plan.groups[2].layers.front().print_z == scale_to_layer_coord(2.));
    } else {
        REQUIRE(plan.groups[0].printing_catalog_ids.size() == 2);
        CHECK(plan.groups[0].layers.size() == 3);
    }
}

// Same-Z local skirts can overlap the radius envelope without obstructing the
// other copy's higher body layers. Coverage includes the real auxiliary sources.
TEST_CASE("Local first-layer skirts do not merge otherwise clear whole copies",
          "[plugins][printing-plan][ordering][skirt-brim][stack-rules]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_one_first_layer", "0"}, {"complete_objects_sort", "object"},
        {"complete_islands", "0"}, {"complete_objects_step", "0"},
        {"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"skirts", "1"}, {"skirt_height", "1"}, {"skirt_distance", "6"}, {"min_skirt_length", "0"},
        {"complete_objects_one_skirt", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "20"}, {"extruder_clearance_height", "1"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {20., 1.}})}));
    Model model;
    Print print;
    Test::init_print({make_cube(20., 20., 0.8)}, print, model, config);
    model.objects().front().instances.front()->set_offset(Vec3d(40., 40., 0.));
    model.objects().front().add_instance(*model.objects().front().instances.front())->set_offset(Vec3d(86.5, 40., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    size_t skirts = 0;
    for (const Layer &layer : print.object(0).auxiliary_layers())
        if (LayerAdhesionProperty::layer_is_normal_skirt(layer))
            for (const LayerSliceIsland &slice : layer.islands())
                for (const LayerRegionIsland &region : slice.regions_islands())
                    for (const auto &[role, root] : region.extrusion_buckets())
                        if (!root.empty())
                            ++skirts;
    REQUIRE(skirts > 0);
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() == 2);
    for (size_t copy = 0; copy < 2; ++copy) {
        REQUIRE(plan.groups[copy].printing_catalog_ids.size() == 1);
        CHECK(plan.object_instances[plan.groups[copy].printing_catalog_ids.front()].object_instance_idx == copy);
        CHECK(plan.groups[copy].layers.size() == 4);
    }
}

// A bounded Z window keeps long, independent and merging copies inexpensive. Only construction
// is timed; exact source coverage and parent order are checked afterward.
// Far copies retain 100 layers to catch cubic work. The eight-island merge variant
// uses ten thicker layers to retain full merging without flaky parallel-suite timings.
TEST_CASE("Stack construction stays bounded for ten hundred-layer copies",
          "[plugins][printing-plan][ordering][island-stacks][stack-rules][stack-timing]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool islands = GENERATE(false, true);
    const bool close = GENERATE(false, true);
    const size_t island_count = close ? 8 : 1;
    const size_t layer_count = close ? 10 : 100;
    const double layer_height = close ? 1. : 0.2;
    ExPolygons slices;
    for (size_t idx = 0; idx < island_count; ++idx)
        slices.push_back(rectangle_expolygon(0., 3. * idx, 1., 3. * idx + 1.));
    std::vector<std::unique_ptr<IslandStackTestLayer>> layers;
    for (size_t idx = 0; idx < layer_count; ++idx) {
        layers.emplace_back(new IslandStackTestLayer(layer_height * (idx + 1), slices, nullptr, layer_height));
        for (size_t island = 0; island < island_count; ++island) {
            add_stack_test_extrusion(layers.back()->island(island), 0);
            if (idx != 0)
                layers.back()->island(island).overlaps_below = {{&layers[idx - 1]->island(island), 1.f}};
        }
    }
    ObjectsLayerToPrint inputs;
    for (uint32_t copy = 0; copy < 10; ++copy)
        for (const std::unique_ptr<IslandStackTestLayer> &layer : layers) {
            ObjectLayerToPrint input;
            input.instance_id = input.copy_rank = copy;
            input.shift.x = scale_i((close ? 1.5 : 100.) * copy);
            input.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(layer.get()));
            inputs.push_back(input);
        }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {10., 20.}})}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{view.get("extruder_clearance").graph(0).value()};
    PluginStorage storage;
    const auto begin = std::chrono::steady_clock::now();
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands(inputs, scale_i(20.), graphs,
        EXTRUDER_ID_UNKNOWN, reinterpret_cast<storage_handle *>(&storage), islands);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    CAPTURE(islands, close, seconds);
    CHECK(seconds < 30.);
    REQUIRE(stacks.size() == (close ? 1 : 10));
    std::set<std::pair<uint32_t, const layer_island_handle *>> visited;
    for (const ObjectsLayerToPrint &stack : stacks) {
        if (!close)
            REQUIRE(stack.size() == 100);
        for (const ObjectLayerToPrint &selection : stack) {
            REQUIRE_FALSE(selection.islands.empty());
            for (uint32_t idx = 0; idx < selection.object_layer.island_count(); ++idx) {
                const slic3r_api::LayerIsland island = selection.object_layer.island(idx);
                if (!selection.islands.count(island.handle()))
                    continue;
                CHECK(visited.emplace(selection.instance_id, island.handle()).second);
                for (const slic3r_api::LayerIsland &parent : island.lower_islands())
                    CHECK(visited.count({selection.instance_id, parent.handle()}) == 1);
            }
        }
    }
    CHECK(visited.size() == 10 * layer_count * island_count);
}

// Joining parents can move a tall history across a short intervening stack.
// Admission relocates that merge instead of querying a profile above max_height.
TEST_CASE("A tall merged history respects short intervening stacks",
          "[plugins][printing-plan][ordering][island-stacks][stack-rules]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool far_copy = GENERATE(false, true);
    CAPTURE(far_copy);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_one_first_layer", "0"}, {"complete_islands", "1"},
        {"complete_objects_step", "1"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "10"}, {"extruder_clearance_height", "1"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {10., 1.}})}));
    Model model;
    Print print;
    Test::init_print(std::vector<TriangleMesh>(far_copy ? 2 : 1, make_cube(2., 2., 0.8)), print, model, config);
    print.set_status_silent();
    print.process();
    slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan())).clear();
    PrintObject &object = print.object(0);
    object.mutable_layers().clear();
    object.mutable_auxiliary_layers().clear();
    Layer &a = append_stack_scene_layer(object, 0.2, {rectangle_expolygon(0., 0., 1., 1.)});
    append_stack_scene_layer(object, 0.3, {rectangle_expolygon(200., 0., 201., 1.)});
    Layer &b = append_stack_scene_layer(object, 0.4, {rectangle_expolygon(100., 0., 101., 1.)});
    append_stack_scene_layer(object, 0.5, {rectangle_expolygon(110., 0., 111., 1.)});
    Layer &b2 = append_stack_scene_layer(object, 0.8, {rectangle_expolygon(100., 0., 101., 1.)});
    b2.island(0).overlaps_below = {{&b.island(0), 1.f}};
    Layer &b3 = append_stack_scene_layer(object, 1.2, {rectangle_expolygon(100., 0., 101., 1.)});
    b3.island(0).overlaps_below = {{&b2.island(0), 1.f}};
    Layer &join = append_stack_scene_layer(object, 1.4, {rectangle_expolygon(0., 0., 111., 1.)});
    join.island(0).overlaps_below = {{&a.island(0), 1.f}, {&b3.island(0), 1.f}};
    if (far_copy) {
        PrintObject &far = print.object(1);
        far.mutable_layers().clear();
        far.mutable_auxiliary_layers().clear();
        Layer &lower = append_stack_scene_layer(far, 0.6, {rectangle_expolygon(400., 0., 401., 1.)});
        Layer &upper = append_stack_scene_layer(far, 1.3, {rectangle_expolygon(400., 0., 401., 1.)});
        upper.island(0).overlaps_below = {{&lower.island(0), 1.f}};
    }
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() == (far_copy ? 3 : 2));
    REQUIRE(plan.groups[0].layers.size() == 1);
    CHECK(plan.groups[0].layers[0].print_z == scale_to_layer_coord(0.3));
    CHECK(plan.groups[1].layers.size() == 6);
    if (far_copy) {
        REQUIRE(plan.groups[2].printing_catalog_ids.size() == 1);
        CHECK(plan.object_instances[plan.groups[2].printing_catalog_ids.front()].object == &print.object(1));
        REQUIRE(plan.groups[2].layers.size() == 2);
        CHECK(plan.groups[2].layers[0].print_z == scale_to_layer_coord(0.6));
        CHECK(plan.groups[2].layers[1].print_z == scale_to_layer_coord(1.3));
    }
}

// Split and join paths bring a refused stack into an admitted covering merge.
// Both copies' following units must continue in the group containing that join.
TEST_CASE("An admitted covering merge reopens a refused destination",
          "[plugins][printing-plan][ordering][island-stacks][stack-rules][stack-reopen]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_one_first_layer", "0"}, {"complete_islands", "1"},
        {"complete_objects_step", "0.4"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"extruder_clearance_radius", "5"}, {"extruder_clearance_height", "0.4"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {5., 0.4}})}));
    Model model;
    Print print;
    Test::init_print(std::vector<TriangleMesh>(3, make_cube(2., 2., 0.8)), print, model, config);
    for (ModelObject &object : model.objects())
        object.instances.front()->set_offset(Vec3d(30., 30., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan())).clear();
    for (PrintObject &object : print.objects()) {
        object.mutable_layers().clear();
        object.mutable_auxiliary_layers().clear();
    }
    struct Row { size_t copy; double z; double x; double width; int parent; int second; };
    const Row rows[] = {
        {0, 0.8, 76., 5., -1, -1}, {0, 0.8, 44., 1., -1, -1},
        {0, 1.0, 72., 8., 0, -1}, {0, 1.2, 60., 7., 2, -1},
        {0, 1.2, 68., 6., -1, -1}, {0, 1.4, 16., 7., 4, 3},
        {1, 0.83, 16., 10., -1, -1}, {1, 0.83, 48., 8., -1, -1},
        {1, 1.23, 16., 7., -1, -1}, {1, 1.63, 28., 7., -1, -1},
        {2, 0.66, 40., 1., -1, -1}, {2, 1.26, 8., 1., -1, -1},
        {2, 1.26, 40., 10., -1, -1}, {2, 1.46, 4., 6., 11, -1},
        {2, 1.46, 28., 8., 12, 11}, {2, 1.66, 28., 8., 13, 14}};
    std::vector<LayerSliceIsland *> units;
    for (size_t start = 0; start < std::size(rows); ) {
        size_t end = start;
        ExPolygons slices;
        while (end < std::size(rows) && rows[end].copy == rows[start].copy && rows[end].z == rows[start].z) {
            slices.push_back(rectangle_expolygon(rows[end].x, 0., rows[end].x + rows[end].width, 1.));
            ++end;
        }
        Layer &layer = append_stack_scene_layer(print.object(rows[start].copy), rows[start].z, slices);
        for (size_t idx = start; idx < end; ++idx)
            units.push_back(&layer.island(idx - start));
        start = end;
    }
    for (size_t idx = 0; idx < std::size(rows); ++idx)
        for (int parent : {rows[idx].parent, rows[idx].second})
            if (parent >= 0)
                units[idx]->overlaps_below.push_back({units[parent], 1.f});
    std::vector<const LayerRegionIsland *> tail;
    for (size_t copy : {size_t(0), size_t(2)}) {
        LayerSliceIsland *previous = units[copy == 0 ? 5 : 15];
        for (double z : {1.76, 1.86}) {
            const double x = copy == 0 ? 28. : 37.;
            Layer &layer = append_stack_scene_layer(print.object(copy), z, {rectangle_expolygon(x, 0., x + 8., 1.)});
            layer.island(0).overlaps_below = {{previous, 1.f}};
            previous = &layer.island(0);
            tail.push_back(&previous->regions_islands().front());
        }
    }
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    std::map<const LayerRegionIsland *, size_t> group_by_source;
    for (size_t idx = 0; idx < plan.groups.size(); ++idx)
        for (const PrintingLayerGroup &layer : plan.groups[idx].layers)
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions)
                    group_by_source.emplace(extrusion.region_island, idx);
    const size_t joined_group = group_by_source.at(&units[15]->regions_islands().front());
    CAPTURE(plan.groups.size(), joined_group);
    CHECK(plan.groups.size() == 4);
    for (const LayerRegionIsland *source : tail)
        CHECK(group_by_source.at(source) == joined_group);
}

// History-dependent rectangles exercise full-history merging and safe emission
// when a later merge crosses parent paths. Explicit sizes describe each result.
TEST_CASE("Island stack histories always produce a safe complete plan",
          "[plugins][printing-plan][parallel-islands][island-stacks][plate-island-units]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    enum class Scene { Rotating, BridgeSplitCycle, SupportGrid, MergeAfterSplit };
    Scene scene = Scene::Rotating;
    SECTION("Rotating rectangles merge their full histories") {}
    SECTION("A bridge then split joined by another instance keeps parent paths") { scene = Scene::BridgeSplitCycle; }
    SECTION("Support of the same copy uses a different Z grid") { scene = Scene::SupportGrid; }
    SECTION("A merge after a split keeps its parents first") { scene = Scene::MergeAfterSplit; }
    const size_t object_count = scene == Scene::SupportGrid ? 1 : 2;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"complete_objects_step", "0.6"}, {"complete_islands", "1"}, {"complete_objects_one_first_layer", "1"}, {"layer_height", "0.2"},
        {"first_layer_height", "0.2"}, {"skirts", "0"}, {"brim_width", "0"}, {"wipe_tower", "0"},
        {"perimeters", "1"}, {"fill_density", "0%"}, {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"}, {"extruder_clearance_radius", "20"}, {"extruder_clearance_height", "10"}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({GraphData(0, 2,
        GraphData::LINEAR, {{0., 0.}, {20., 10.}})}));
    Model model;
    Print print;
    Test::init_print(std::vector<TriangleMesh>(object_count, make_cube(3., 3., 1.)), print, model, config);
    for (ModelObject &object : model.objects())
        object.instances.front()->set_offset(Vec3d(30., 30., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();
    REQUIRE(print.objects().size() == object_count);
    if (object_count == 2)
        REQUIRE(print.object(0).instances()[0].shift == print.object(1).instances()[0].shift);
    // Release the original plan before replacing its borrowed source layers.
    slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan())).clear();
    for (PrintObject &object : print.objects()) {
        object.mutable_layers().clear();
        object.mutable_auxiliary_layers().clear();
        append_stack_scene_layer(object, 0.2, {});
    }
    if (scene == Scene::Rotating) {
        Layer *previous[2] = {nullptr, nullptr};
        for (size_t level = 0; level < 6; ++level) {
            const double angle = double(level) * std::acos(-1.) / 5.;
            for (size_t copy = 0; copy < 2; ++copy) {
                const double sign = copy == 0 ? 1. : -1.;
                const double x = sign * 10. * std::cos(angle), y = sign * 10. * std::sin(angle);
                Layer &layer = append_stack_scene_layer(print.object(copy), 0.3 + 0.1 * level,
                    {rectangle_expolygon(x - 4., y - 4., x + 4., y + 4.)});
                if (previous[copy] != nullptr)
                    layer.island(0).overlaps_below = {{&previous[copy]->island(0), 1.f}};
                previous[copy] = &layer;
            }
        }
    } else if (scene == Scene::BridgeSplitCycle) {
        Layer &base = append_stack_scene_layer(print.object(0), 1.,
            {rectangle_expolygon(0., 0., 101., 1.), rectangle_expolygon(300., 0., 303., 1.)});
        Layer &bridge = append_stack_scene_layer(print.object(0), 1.2,
            {rectangle_expolygon(100., 0., 303., 1.), rectangle_expolygon(0., 0., 1., 1.)});
        bridge.island(0).overlaps_below = {{&base.island(0), 1.f}, {&base.island(1), 1.f}};
        bridge.island(1).overlaps_below = {{&base.island(0), 1.f}};
        Layer &split = append_stack_scene_layer(print.object(0), 1.4,
            {rectangle_expolygon(300., 0., 303., 1.), rectangle_expolygon(100., 0., 101., 1.),
             rectangle_expolygon(0., 0., 1., 1.)});
        split.island(0).overlaps_below = {{&bridge.island(0), 1.f}};
        split.island(1).overlaps_below = {{&bridge.island(0), 1.f}};
        split.island(2).overlaps_below = {{&bridge.island(1), 1.f}};
        append_stack_scene_layer(print.object(1), 1.5, {rectangle_expolygon(0., 2., 101., 3.)});
    } else if (scene == Scene::SupportGrid) {
        Layer &lower = append_stack_scene_layer(print.object(0), 0.3, {rectangle_expolygon(0., 0., 1., 1.)});
        Layer &upper = append_stack_scene_layer(print.object(0), 0.8, {rectangle_expolygon(0., 0., 1., 1.)});
        upper.island(0).overlaps_below = {{&lower.island(0), 1.f}};
        append_stack_scene_layer(print.object(0), 0.5, {rectangle_expolygon(1.5, 0., 2.5, 1.)}, true);
    } else {
        Layer &base = append_stack_scene_layer(print.object(0), 1., {rectangle_expolygon(0., 0., 101., 1.)});
        Layer &split = append_stack_scene_layer(print.object(0), 1.2,
            {rectangle_expolygon(0., 0., 1., 1.), rectangle_expolygon(100., 0., 101., 1.)});
        split.island(0).overlaps_below = {{&base.island(0), 1.f}};
        split.island(1).overlaps_below = {{&base.island(0), 1.f}};
        append_stack_scene_layer(print.object(1), 1.3, {rectangle_expolygon(0., 2., 101., 3.)});
    }
    run_island_plan_builder(print);
    check_island_plan_oracles(print, config);
    const PrintingPlan &plan = *print.printing_plan();
    REQUIRE(plan.groups.size() > 1);
    const PrintingGroup &first = plan.groups[1];
    std::vector<size_t> group_sizes;
    for (const PrintingGroup &group : plan.groups)
        group_sizes.push_back(group.layers.size());
    CAPTURE(int(scene), group_sizes);
    if (scene == Scene::Rotating || scene == Scene::BridgeSplitCycle) {
        // Real-height history comparisons combine rotating layers and preserve
        // the bridge's separate parent paths without a mandatory initial fallback.
        CHECK(group_sizes == (scene == Scene::Rotating ? std::vector<size_t>{1, 6} : std::vector<size_t>{1, 3, 3, 1, 1}));
        CHECK(first.layers.front().print_z == scale_to_layer_coord(scene == Scene::Rotating ? 0.3 : 1.));
        REQUIRE(first.printing_catalog_ids.size() == (scene == Scene::Rotating ? 2 : 1));
        REQUIRE(first.layers.front().tool_groups.size() == 1);
        CHECK(first.layers.front().tool_groups.front().extrusions.size() == (scene == Scene::Rotating ? 2 : 1));
    } else {
        // Support below a later body is a real obstacle; a later single-layer
        // copy above a split does not introduce a backwards height obstacle.
        CHECK(group_sizes == (scene == Scene::SupportGrid ? std::vector<size_t>{1, 3} : std::vector<size_t>{1, 2, 1, 1}));
    }
}

Layer &append_stack_scene_layer(PrintObject &object, double z, const ExPolygons &slices, bool auxiliary, double height)
{
    std::unique_ptr<IslandStackTestLayer> layer(new IslandStackTestLayer(z, slices, &object, height));
    for (size_t idx = 0; idx < slices.size(); ++idx) {
        LayerRegionIsland &region = layer->island(idx).add_region_island({}, 0);
        const Point start = slices[idx].contour.points.front();
        region.mutable_extrusion(auxiliary ? LayerRegionIsland::SUPPORT : LayerRegionIsland::PERIMETERS).append(
            test_path({start, start + Point(scale_i(0.5), 0)}));
    }
    Layer &result = *layer;
    if (auxiliary)
        object.mutable_auxiliary_layers().push_back(std::move(layer));
    else
        object.mutable_layers().push_back(std::move(layer));
    return result;
}

void run_island_plan_builder(Print &print)
{
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin("ordering.plan_builder.by_object");
    REQUIRE(plugin != nullptr);
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
    plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, plugin, &context);
    run.data = &payload;
    plugin->setup(run, 1);
    plugin->setup_run(run);
    plugin->run(run);
}

void check_island_plan_oracles(const Print &print, const DynamicPrintConfig &config,
    coord_t tool_change_z, extruder_index_t selected_tool)
{
    REQUIRE(print.printing_plan() != nullptr);
    const PrintingPlan &plan = *print.printing_plan();
    using RoleKey = std::tuple<const LayerRegionIsland *, uint32_t, ExtrusionRole>;
    using UnitKey = std::pair<const LayerSliceIsland *, uint32_t>;
    struct Source {
        const Layer *layer;
        const LayerSliceIsland *island;
        Point shift;
    };
    std::map<RoleKey, Source> expected;
    // Enumerate the source independently, including every physical copy and owner.
    for (const PrintObject &object : print.objects()) {
        for (uint32_t instance = 0; instance < object.instances().size(); ++instance) {
            uint32_t catalog_id = uint32_t(-1);
            for (uint32_t idx = 0; idx < plan.object_instances.size(); ++idx)
                if (plan.object_instances[idx].object == &object &&
                    plan.object_instances[idx].object_instance_idx == instance)
                    catalog_id = idx;
            REQUIRE(catalog_id != uint32_t(-1));
            std::vector<const Layer *> layers;
            for (const Layer &layer : object.layers())
                layers.push_back(&layer);
            for (const Layer &layer : object.auxiliary_layers()) {
                const std::optional<uint32_t> owner = slic3r_api::layer_instance_owner(
                    slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&layer)));
                if (!owner || *owner == instance)
                    layers.push_back(&layer);
            }
            for (const Layer *layer : layers)
                for (const LayerSliceIsland &island : layer->islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        for (const auto &[role, root] : region.extrusion_buckets())
                            if (!root.empty())
                                expected.emplace(RoleKey{&region, catalog_id, role},
                                    Source{layer, &island, object.instances()[instance].shift});
        }
    }
    // Global auxiliaries have no catalog instance and retain plate coordinates.
    if (const PrintObject *auxiliary = print.auxiliary_object())
        for (const Layer &layer : auxiliary->auxiliary_layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region : island.regions_islands())
                    for (const auto &[role, root] : region.extrusion_buckets())
                        if (!root.empty())
                            expected.emplace(RoleKey{&region, PRINTING_OBJECT_INSTANCE_ID_NONE, role},
                                Source{&layer, &island, Point(0, 0)});
    const double radius = config.opt_float("extruder_clearance_radius");
    const double height = config.opt_float("extruder_clearance_height");
    std::vector<GraphData> graphs;
    for (const GraphData &source : config.option<ConfigOptionGraphs>("extruder_clearance")->get_values()) {
        Pointfs points;
        for (const Vec2d &point : source.data())
            if (point.x() < radius && point.y() < height)
                points.push_back(point);
        points.emplace_back(radius, height);
        graphs.emplace_back(0, points.size(), source.type, points);
    }
    struct Visit {
        UnitKey key;
        BoundingBox box;
        coord_t z;
        size_t group;
        extruder_index_t tool;
    };
    std::vector<Visit> printed;
    std::set<RoleKey> actual;
    std::set<UnitKey> visited_units;
    for (size_t group_idx = 0; group_idx < plan.groups.size(); ++group_idx) {
        const PrintingGroup &group = plan.groups[group_idx];
        for (const PrintingLayerGroup &layer : group.layers) {
            std::set<UnitKey> layer_units;
            for (const PrintingToolGroup &tool : layer.tool_groups)
                for (const PrintingExtrusion &extrusion : tool.extrusions) {
                    const RoleKey role{extrusion.region_island, extrusion.printing_catalog_id, extrusion.sregion_island_role};
                    REQUIRE(expected.count(role) == 1);
                    CHECK(actual.insert(role).second);
                    const Source &source = expected.at(role);
                    CHECK(layer.print_z == source.layer->scaled_print_z());
                    REQUIRE(extrusion.root != nullptr);
                    Points source_points, cloned_points;
                    extrusion.region_island->extrusion(extrusion.sregion_island_role).collect_points(source_points);
                    extrusion.root->collect_points(cloned_points);
                    for (Point &point : source_points)
                        point += source.shift;
                    CHECK(cloned_points == source_points);
                    const UnitKey unit{source.island, extrusion.printing_catalog_id};
                    if (layer_units.insert(unit).second) {
                        CHECK(visited_units.insert(unit).second);
                        for (const auto &[parent, overlap] : source.island->overlaps_below)
                            if (parent->has_extrusions())
                                CHECK(visited_units.count(UnitKey{parent, extrusion.printing_catalog_id}) == 1);
                    }
                    BoundingBox box = source.island->get_bounding_box();
                    box.translate(source.shift);
                    for (const Visit &previous : printed) {
                        if (previous.group == group_idx || previous.z <= layer.print_z)
                            continue;
                        const double dx = unscaled(std::max(coord_t(0), std::max(
                            box.min.x() - previous.box.max.x(), previous.box.min.x() - box.max.x())));
                        const double dy = unscaled(std::max(coord_t(0), std::max(
                            box.min.y() - previous.box.max.y(), previous.box.min.y() - box.max.y())));
                        const double distance = std::hypot(dx, dy);
                        double allowed = height;
                        std::vector<extruder_index_t> clearance_tools{tool.extruder_id, previous.tool};
                        if (selected_tool != EXTRUDER_ID_UNKNOWN && layer.print_z >= tool_change_z)
                            clearance_tools.push_back(selected_tool);
                        for (extruder_index_t id : clearance_tools) {
                            if (id == EXTRUDER_ID_UNKNOWN) {
                                for (const GraphData &graph : graphs)
                                    allowed = std::min(allowed, graph.interpolate(distance));
                            } else {
                                REQUIRE(size_t(id) < graphs.size());
                                allowed = std::min(allowed, graphs[id].interpolate(distance));
                            }
                        }
                        CAPTURE(group_idx, distance, allowed, previous.z, layer.print_z);
                        CHECK(unscaled(previous.z - layer.print_z) <= allowed + 0.00001);
                    }
                    printed.push_back(Visit{unit, box, layer.print_z, group_idx, tool.extruder_id});
                }
        }
    }
    CHECK(actual.size() == expected.size());
}

TEST_CASE("DataTree extruder index boundaries validate before narrowing",
          "[printing][datatree][extruder-index]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PluginStorage plugin_storage;
    const ExPolygons subject{ rectangle_expolygon(0., 0., 20., 20.) };
    const slic3r_api::AuxiliaryLayerBuildResult built =
        slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&plugin_storage),
            print_view(print),
            print_view(print).auxiliary_object(),
            expolygons_view(subject),
            scale_i(1.),
            scale_i(1.),
            scale_i(0.5));
    REQUIRE(built.created);
    REQUIRE(built.layer.island_count() == 1);
    const slic3r_api::Layer layer = built.layer;
    const slic3r_api::LayerIsland island = layer.island(0);
    const extruder_index_t maximum = (std::numeric_limits<extruder_index_t>::max)();

    for (const int64_t extruder_id : {
             int64_t(EXTRUDER_ID_UNKNOWN), int64_t(0), int64_t(maximum)}) {
        const slic3r_api::LayerRegionIsland region_island =
            island.get_or_create_full_region_island(extruder_id);
        REQUIRE(region_island.valid());
        CHECK(region_island.extruder_id() == extruder_index_t(extruder_id));
    }
    CHECK_THROWS_AS(island.get_or_create_full_region_island(-2), std::out_of_range);
    CHECK_THROWS_AS(
        island.get_or_create_full_region_island(int64_t(maximum) + 1),
        std::out_of_range);

    layer_region_island_handle *c_region_island =
        layer_island_get_or_create_region_island(
            const_cast<layer_island_handle *>(island.handle()), nullptr, 0, maximum);
    REQUIRE(c_region_island != nullptr);
    CHECK(layer_island_get_or_create_region_island(
              const_cast<layer_island_handle *>(island.handle()),
              nullptr, 0, extruder_index_t(-2)) == nullptr);
}

// SEMM follows actual tool identities, not the number of visits or their extrusion counts.
TEST_CASE("PrintingPlan enables SEMM only for distinct concrete tools",
          "[printing][plan][semm]")
{
    PrintingPlan native_plan;
    const slic3r_api::ConstPrintingPlan plan(
        reinterpret_cast<const printing_plan_handle *>(&native_plan));
    CHECK_FALSE(plan.uses_single_extruder_multi_material(true));
    native_plan.groups.emplace_back();
    native_plan.groups.back().layers.emplace_back();
    native_plan.groups.back().layers.back().tool_groups.emplace_back();
    native_plan.groups.back().layers.back().tool_groups.back().extruder_id = EXTRUDER_ID_UNKNOWN;
    CHECK_FALSE(plan.uses_single_extruder_multi_material(true));
    native_plan.groups.back().layers.back().tool_groups.back().extruder_id = 1;
    CHECK_FALSE(plan.uses_single_extruder_multi_material(true));
    native_plan.groups.emplace_back();
    native_plan.groups.back().layers.emplace_back();
    native_plan.groups.back().layers.back().tool_groups.emplace_back();
    native_plan.groups.back().layers.back().tool_groups.back().extruder_id = 1;
    CHECK_FALSE(plan.uses_single_extruder_multi_material(true));
    native_plan.groups.back().layers.back().tool_groups.emplace_back();
    native_plan.groups.back().layers.back().tool_groups.back().extruder_id = 0;
    CHECK(plan.uses_single_extruder_multi_material(true));
    CHECK_FALSE(plan.uses_single_extruder_multi_material(false));
}
