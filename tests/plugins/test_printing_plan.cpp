#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
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
Common-builder probes select source islands and half-open Z intervals, then check
the cloned roots and source context for the selected object instance.
Island-stack probes construct ordered source islands directly to exercise bridge
merges, tool compatibility, lower links and empty extrusion selections.

    Island-stack tests -> build synthetic layers and clearance graphs
        add_stack_test_extrusion() -> give selected islands printable tool buckets
        IslandStacks::separate_islands() -> inspect borrowed stack membership
    Owned-auxiliary test -> compare baseline stacks and owner-only cloned paths
        run_builder() -> invoke the By object provider without later orderers
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
    // Own the supplied polygons as islands at Z, without slicing or region settings.
    IslandStackTestLayer(double z, const ExPolygons &slices)
        : Layer(1, nullptr, scale_to_layer_coord(0.2), scale_to_layer_coord(z), z - 0.1, true)
    {
        for (const ExPolygon &slice : slices)
            m_islands.emplace_back(new LayerSliceIsland(slice));
    }
};

// Add one printable tool bucket to a synthetic island; its layer owns the path.
void add_stack_test_extrusion(LayerSliceIsland &island, extruder_index_t tool);

// Compare stack membership rather than extrusion count: one island may have several roles.
TEST_CASE("Island stacks group clearance bridges and preserve tool compatibility",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer layer(1., {rectangle_expolygon(0., 0., 1., 1.),
        rectangle_expolygon(8., 0., 9., 1.), rectangle_expolygon(4., 0., 5., 1.)});
    IslandStackTestLayer support(1., {rectangle_expolygon(4., 0., 5., 1.)});
    bool different_tools = false, low_graph = false, empty_bridge = false, support_bridge = false;
    SECTION("A third island merges two clearance groups") {}
    SECTION("Different tools cannot merge") { different_tools = true; }
    SECTION("A graph below the step keeps all islands grouped") { low_graph = true; }
    SECTION("An island without extrusions is ignored") { empty_bridge = true; }
    SECTION("Support islands take part in the same clearance grouping") { support_bridge = true; }
    add_stack_test_extrusion(layer.island(0), 0);
    add_stack_test_extrusion(layer.island(1), different_tools ? 1 : 0);
    if (!empty_bridge && !support_bridge)
        add_stack_test_extrusion(layer.island(2), 0);
    if (support_bridge)
        add_stack_test_extrusion(support.island(0), 0);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const GraphData graph(0, 2, GraphData::GraphType::LINEAR,
        {{0., 0.}, {3.5, low_graph ? 0.1 : 0.4}});
    config.set_key_value("extruder_clearance", new ConfigOptionGraphs({graph, graph}));
    const slic3r_api::Config view(reinterpret_cast<const config_handle *>(static_cast<const ConfigBase *>(&config)));
    const std::vector<slic3r_api::GraphData> graphs{
        view.get("extruder_clearance").graph(0).value(), view.get("extruder_clearance").graph(1).value()};
    PluginStorage storage;
    ObjectLayerToPrint input;
    input.object_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&layer));
    if (support_bridge)
        input.auxiliary_layer = slic3r_api::Layer(reinterpret_cast<const layer_handle *>(&support));
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({input}, 0,
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == ((different_tools || empty_bridge) ? 2 : 1));
    size_t selected = 0;
    for (const ObjectsLayerToPrint &stack : stacks) {
        REQUIRE(stack.size() == 1);
        selected += stack.front().islands.size();
    }
    CHECK(selected == (empty_bridge ? 2 : 3));
    if (support_bridge)
        CHECK(stacks.front().front().auxiliary_layer.handle() == input.auxiliary_layer.handle());
}

// Lower links join stacks across layers; a multi-tool stack cannot be extended.
TEST_CASE("Island stacks follow lower islands and close multi-tool stacks",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    using namespace slic3r_api::Ordering::IslandStacks;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    IslandStackTestLayer lower(1., {rectangle_expolygon(0., 0., 1., 1.), rectangle_expolygon(8., 0., 9., 1.)});
    IslandStackTestLayer upper(1.2, {rectangle_expolygon(0., 0., 9., 1.)});
    bool multiple = false;
    SECTION("Bridge island follows both lower stacks") {}
    SECTION("Multi-tool lower island is closed") { multiple = true; }
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
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high}, 0,
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    if (multiple) {
        CHECK(stacks.front().size() == 1);
        CHECK(stacks.front().front().allow_wipe_tower);
    }
    CHECK(stacks.back().size() == 2);
    CHECK(stacks.back().back().islands.count(reinterpret_cast<const layer_island_handle *>(&upper.island(0))) == 1);
}

// The MMU exit tool must precede the current tool when all its tools are lower.
TEST_CASE("Island stacks choose the previous MMU exit tool below the current tool",
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
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high}, 0,
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    REQUIRE(stacks[0].size() == 1);
    REQUIRE(stacks[1].size() == 1);
    CHECK(stacks[0][0].islands == std::set<const layer_island_handle *>{
        reinterpret_cast<const layer_island_handle *>(&lower.island(0))});
    CHECK(stacks[1][0].islands == std::set<const layer_island_handle *>{
        reinterpret_cast<const layer_island_handle *>(&upper.island(0))});
    REQUIRE_FALSE(stacks[0][0].extruders_order.empty());
    CHECK(stacks[0][0].extruders_order.back() == 2);
    CHECK((stacks[1][0].extruders_order == std::vector<extruder_index_t>{2, 0, 1}));
}

// Every island appended to an already-closed MMU stack is finished before parent lookup.
TEST_CASE("Island stacks remove later MMU islands from pending parents",
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
    const std::vector<ObjectsLayerToPrint> stacks = separate_islands({low, high}, 0,
        scale_to_layer_coord(0.4), graphs, EXTRUDER_ID_UNKNOWN,
        reinterpret_cast<storage_handle *>(&storage));
    REQUIRE(stacks.size() == 2);
    REQUIRE(stacks.front().size() == 2);
    REQUIRE(stacks.back().size() == 1);
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

void add_stack_test_extrusion(LayerSliceIsland &island, extruder_index_t tool)
{
    LayerRegionIsland &region = island.add_region_island({}, tool);
    region.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(
        test_path({Point(0, 0), Point(scale_i(1.), 0)}));
}

// Owned auxiliaries must not change shared stacks or leak to the other copy.
// Baseline ranges supply an independent expected destination in print order.
// A limit between the first two object layers leaves only first-pass destinations.
TEST_CASE("Island stacks place owned auxiliaries by range for their instance only",
          "[plugins][printing-plan][parallel-islands][island-stacks]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const std::pair<double, double> cases[] = {{0.3, 0.}, {0.8, 0.}, {1.6, 0.}, {0.25, 0.3}};
    for (const std::pair<double, double> &test : cases) {
        const double z = test.first, limit = test.second;
        CAPTURE(z, limit);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
            {"complete_objects_step", "0.5"}, {"complete_islands", "1"}, {"layer_height", "0.2"},
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
        Orchestrator &host = Orchestrator::instance();
        Plugin *plugin = host.get_plugin("ordering.plan_builder.by_object");
        REQUIRE(plugin != nullptr);
        // Invoke just the builder so later ordering stages cannot hide its placement.
        const auto run_builder = [&]() {
            run_ctx_extrusion_ordering payload = {};
            payload.print = reinterpret_cast<const print_handle *>(&print);
            payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
            plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
            plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, plugin, &context);
            run.data = &payload;
            plugin->setup(run, 1);
            plugin->setup_run(run);
            plugin->run(run);
        };
        run_builder();
        const PrintingPlan &baseline = *print.printing_plan();
        const size_t group_count = baseline.groups.size();
        if (limit == 0.)
            REQUIRE(group_count > 4);
        else
            REQUIRE(group_count == 3);
        std::vector<size_t> owner_groups;
        for (size_t idx = 2; idx < group_count; ++idx)
            if (baseline.object_instances[baseline.groups[idx].printing_catalog_ids.front()].object_instance_idx == 1)
                owner_groups.push_back(idx);
        if (limit == 0.)
            REQUIRE(owner_groups.size() > 1);
        else
            REQUIRE(owner_groups.empty());
        size_t expected_group = owner_groups.empty() ? 1 : owner_groups.back();
        const coord_t owned_z = scale_to_layer_coord(z);
        bool contains = false;
        for (const size_t idx : owner_groups) {
            const PrintingGroup &group = baseline.groups[idx];
            if (owned_z >= group.layers.front().print_z &&
                owned_z <= group.layers.back().print_z) {
                expected_group = idx;
                contains = true;
                break;
            }
        }
        if (!contains)
            for (const size_t idx : owner_groups)
                if (baseline.groups[idx].layers.front().print_z > owned_z) {
                    expected_group = idx;
                    break;
                }
        PluginStorage storage;
        const ExPolygons subject{rectangle_expolygon(0., 0., 1., 1.)};
        const slic3r_api::AuxiliaryLayerBuildResult built = slic3r_api::build_auxiliary_layer_regions_from_subject(
            reinterpret_cast<storage_handle *>(&storage), print_view(print), print_view(print).object(0),
            expolygons_view(subject), scale_i(0.2), owned_z, scale_i(z - 0.1));
        REQUIRE(built.created);
        built.layer.mutable_properties().get_or_add(slic3r_api::LayerInstanceProperty::key).object_instance_idx = 1;
        const slic3r_api::LayerRegionIsland region = built.layer.island(0).get_or_create_full_region_island(0);
        extrusion_entity_handle *root_handle = layer_region_island_get_mutable_extrusion(
            const_cast<layer_region_island_handle *>(region.handle()), RAW_EXTRUSION_ROLE_PERIMETER);
        REQUIRE(root_handle != nullptr);
        reinterpret_cast<ExtrusionEntityCollection *>(root_handle)->append(test_path({Point(0, 0), Point(scale_i(1.), 0)}));
        run_builder();
        const PrintingPlan &actual = *print.printing_plan();
        REQUIRE(actual.groups.size() == group_count);
        size_t visits = 0;
        for (size_t idx = 0; idx < actual.groups.size(); ++idx)
            for (const PrintingLayerGroup &layer : actual.groups[idx].layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.region_island == reinterpret_cast<const LayerRegionIsland *>(region.handle())) {
                            ++visits;
                            CHECK(idx == expected_group);
                            CHECK(layer.print_z == owned_z);
                            const PrintingObjectInstance &instance = actual.object_instances[extrusion.printing_catalog_id];
                            CHECK(instance.object_instance_idx == 1);
                            REQUIRE(extrusion.root != nullptr);
                            CHECK(extrusion.root->first_point() == print.object(0).instances()[1].shift);
                        }
        CHECK(visits == 1);
    }
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
