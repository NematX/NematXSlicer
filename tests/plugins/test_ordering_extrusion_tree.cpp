#include <catch2/catch.hpp>

#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"

/*
Fallback extrusion-tree ordering tests
======================================

The provider runs after the coarse PrintingExtrusion pre-sort. These tests
prebuild the relevant PrintingPlan fragments so they can verify how the first
estimate starts the sequential pass and how exact exits then propagate through
the remaining plan.
*/

namespace {
// Execute only the fallback tree-ordering provider through the real host on the borrowed print-owned plan; execution failure fails REQUIRE.
void run_tree_ordering(Slic3r::Print &print);
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::PerimeterPluginTests;
using slic3r_api::EntryPointProperty;
using slic3r_api::PluginPropertyKey;

constexpr const char *TREE_ORDERING_PLUGIN =
    "ordering.extrusion_tree.default";

/* Keep one real LayerRegionIsland owner around while each test changes its small source tree. */
class PerimeterSourceFixture
{
public:
    PerimeterSourceFixture()
    {
        prepare_cube_print(m_prepared, perimeter_config({}));
        PrintObject &object = m_prepared.print.object(0);
        REQUIRE(object.layer_count() > 0);
        Layer &layer = object.layer(0);
        REQUIRE_FALSE(layer.islands().empty());
        LayerSliceIsland &island = layer.island(0);
        REQUIRE_FALSE(island.regions().empty());
        m_region_island = &island.get_or_add_region_island(island.regions(), 0);
    }

    Print &print() { return m_prepared.print; }
    LayerRegionIsland &region_island() { return *m_region_island; }

    LayerRegionIsland &add_object_auxiliary_region_island()
    {
        return add_auxiliary_region_island(m_prepared.print.object(0));
    }

    LayerRegionIsland &add_print_auxiliary_region_island()
    {
        return add_auxiliary_region_island(m_prepared.print.mutable_auxiliary_object());
    }

private:
    LayerRegionIsland &add_auxiliary_region_island(PrintObject &owner)
    {
        REQUIRE(owner.num_printing_regions() > 0);
        const ExPolygon subject = m_prepared.print.object(0).layer(0).island(0).get_slice();
        Layer &layer = owner.add_auxiliary_layer(0, scale_i(0.2), scale_i(0.2));
        ApiInternal::LayerAccess::add_region(layer, owner.printing_region(0));
        ApiInternal::LayerRegionAccess::slices_mutable(layer.region(0)) = ExPolygons{subject};
        ApiInternal::LayerAccess::set_islands(layer, ExPolygons{subject});
        layer.island(0).fill_regions(layer);
        REQUIRE_FALSE(layer.islands().empty());
        REQUIRE_FALSE(layer.island(0).regions().empty());
        return layer.island(0).get_or_add_region_island(layer.island(0).regions(), 0);
    }

    PreparedPerimeterPrint m_prepared;
    LayerRegionIsland *m_region_island = nullptr;
};

ExtrusionEntityUPtr make_polyline_entity(const Points &points)
{
    return std::make_unique<ExtrusionEntity>(true, ArcPolyline(points));
}

void add_loop_property(ExtrusionEntity &entity, const bool loop_flag)
{
    entity.get_or_add_property<ExtrusionPropertyLoopRole>().set_perimeter_role(
        loop_flag ? elrDefault : elrNotALoop);
}

ExtrusionEntityUPtr make_closed_terminal_loop(const bool with_property,
                                             const bool loop_flag)
{
    ExtrusionEntityUPtr loop = make_polyline_entity(
        Points{Point(0, 0), Point(100, 0), Point(0, 0)});
    loop->get_or_add_property<ExtrusionAttributes>().set_role(
        ExtrusionRole::ExternalPerimeter);
    if (with_property)
        add_loop_property(*loop, loop_flag);
    return loop;
}

ExtrusionEntityUPtr make_closed_loop_container()
{
    ExtrusionEntity::Children children;
    children.emplace_back(make_closed_terminal_loop(true, true));
    return std::make_unique<ExtrusionEntity>(std::move(children), false, true, true);
}

ExtrusionEntityUPtr make_loop_with_open_fragments()
{
    ExtrusionEntity::Children children;
    children.emplace_back(make_polyline_entity(Points{Point(0, 0), Point(100, 0)}));
    children.emplace_back(make_polyline_entity(Points{Point(100, 0), Point(0, 0)}));
    ExtrusionEntityUPtr owner =
        std::make_unique<ExtrusionEntity>(std::move(children), false, true, true);
    add_loop_property(*owner, true);
    return owner;
}

ExtrusionEntityUPtr make_loop_with_flagged_open_fragment()
{
    ExtrusionEntity::Children children;
    ExtrusionEntityUPtr flagged_fragment =
        make_polyline_entity(Points{Point(0, 0), Point(100, 0)});
    add_loop_property(*flagged_fragment, true);
    children.emplace_back(std::move(flagged_fragment));
    children.emplace_back(make_polyline_entity(Points{Point(100, 0), Point(0, 0)}));
    ExtrusionEntityUPtr owner =
        std::make_unique<ExtrusionEntity>(std::move(children), false, true, true);
    add_loop_property(*owner, true);
    return owner;
}

ExtrusionEntityUPtr make_open_wrapper_with_invalid_loop()
{
    ExtrusionEntity::Children children;
    children.emplace_back(make_closed_terminal_loop(false, false));
    return std::make_unique<ExtrusionEntity>(std::move(children), true, true, true);
}

void publish_perimeter_tree(LayerRegionIsland &region_island, ExtrusionEntityUPtr root)
{
    ExtrusionEntityCollection &perimeters =
        region_island.mutable_extrusion(LayerRegionIsland::PERIMETERS);
    perimeters.clear();
    perimeters.append(std::move(root));
}

void publish_perimeter_tree(PerimeterSourceFixture &fixture, ExtrusionEntityUPtr root)
{
    publish_perimeter_tree(fixture.region_island(), std::move(root));
}

void require_precondition_failure(const Print &print,
                                  const char *cause,
                                  const char *path_fragment)
{
    std::string error;
    CHECK_FALSE(Steps::StepExtrusionOrdering::validate_pre(print, &error));
    CHECK(error.find(cause) != std::string::npos);
    CHECK(error.find(path_fragment) != std::string::npos);
}

/* Restore the process-wide active providers after one focused step run. */
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

/* Build one open leaf whose endpoints make ordering decisions easy to read. */
ExtrusionEntityUPtr make_path(const Point &start,
                              const Point &end,
                              const bool reversible = false)
{
    return std::make_unique<ExtrusionEntity>(
        reversible, ArcPolyline(Points{start, end}));
}

/* Build a sortable root containing the supplied open paths in source order. */
std::unique_ptr<ExtrusionEntityCollection> make_sortable_root(
    std::initializer_list<std::pair<Point, Point>> paths)
{
    std::unique_ptr<ExtrusionEntityCollection> root =
        std::make_unique<ExtrusionEntityCollection>(true, true);
    for (const std::pair<Point, Point> &path : paths)
        root->append(make_path(path.first, path.second));
    return root;
}

/* Append one owned root to a tool visit without requiring regional context. */
PrintingExtrusion &append_extrusion(PrintingToolGroup &tool,
                                    ExtrusionEntityUPtr root)
{
    PrintingExtrusion extrusion;
    extrusion.root = std::move(root);
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    tool.extrusions.push_back(std::move(extrusion));
    return tool.extrusions.back();
}

/* Add one layer with one tool visit to a native PrintingGroup. */
PrintingLayerGroup &append_layer(PrintingGroup &group)
{
    group.layers.emplace_back();
    PrintingLayerGroup &layer = group.layers.back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    return layer;
}

/* Resolve the private key from the same orchestrator as the production plugin. */
PluginPropertyKey<EntryPointProperty> entry_point_key()
{
    return slic3r_api::entry_point_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
}

/* Return a mutable API view over a native extrusion root. */
slic3r_api::MutableExtrusionEntity mutable_root(PrintingExtrusion &extrusion)
{
    return slic3r_api::MutableExtrusionEntity(
        reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get()));
}

/* Give hand-built groups the same ownership identity as the plan under test. */
void attach_plan_identity(PrintingPlan &plan)
{
    const PrintingPlanIdentity *identity = plan.identity_token();
    for (PrintingGroup &group : plan.groups) {
        group.plan_identity = identity;
        for (PrintingLayerGroup &layer : group.layers) {
            layer.plan_identity = identity;
            for (PrintingToolGroup &tool : layer.tool_groups)
                tool.plan_identity = identity;
        }
    }
}

void run_tree_ordering(Print &print)
{
    attach_plan_identity(print.mutable_printing_plan());
    ScopedActivePlugins active{TREE_ORDERING_PLUGIN};
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Steps::StepExtrusionOrdering::run_step(orchestrator, print);
    REQUIRE_FALSE(orchestrator.is_plugin_cancelled());
}

} // namespace

// Read registered ordering descriptions and check compatible properties without changing plugin ownership.
TEST_CASE("EntryPointProperty has one compatible private dynamic contract",
          "[plugins][ordering][extrusion-tree-ordering][properties]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator_handle *handle = reinterpret_cast<orchestrator_handle *>(&orchestrator);
    Plugin *plugin = orchestrator.get_plugin(TREE_ORDERING_PLUGIN);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == STEP_ORDERING);
    CHECK(plugin->get_priority() == 1050);
    CHECK(plugin->get_exclusive_group_priority() == 20);
    CHECK(plugin->get_exclusive_group() == "ordering.extrusion_tree");
    CHECK(plugin->get_dependencies().empty());

    const PluginPropertyKey<EntryPointProperty> first =
        slic3r_api::entry_point_property_key(handle);
    const PluginPropertyKey<EntryPointProperty> second =
        slic3r_api::entry_point_property_key(handle);
    CHECK(first.type() == second.type());

    struct IncompatibleEntryPointProperty
    {
        c_point entry;
    };
    CHECK_THROWS_AS(
        PluginPropertyKey<IncompatibleEntryPointProperty>::register_dynamic(
            handle, PRINTING_EXTRUSION_ENTRY_POINT_PROPERTY_NAME),
        std::runtime_error);
}

TEST_CASE("Extrusion-tree ordering publishes exact endpoints and propagates local exits",
          "[plugins][ordering][extrusion-tree-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingLayerGroup &layer = append_layer(plan.groups.back());
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extrusions.reserve(2);

    PrintingExtrusion &first = append_extrusion(
        tool, make_path(Point(0, 0), Point(100, 0)));
    PrintingExtrusion &second = append_extrusion(
        tool, make_sortable_root({
            {Point(1, 0), Point(2, 0)},
            {Point(101, 0), Point(102, 0)}}));

    run_tree_ordering(print);

    REQUIRE(second.root->child_count() == 2);
    CHECK(second.root->child(0).first_point() == Point(101, 0));
    CHECK_FALSE(second.root->can_sort());

    const PluginPropertyKey<EntryPointProperty> key = entry_point_key();
    const EntryPointProperty *first_points = key.get(mutable_root(first));
    const EntryPointProperty *second_points = key.get(mutable_root(second));
    REQUIRE(first_points != nullptr);
    REQUIRE(second_points != nullptr);
    CHECK(first_points->entry.x == 0);
    CHECK(first_points->exit.x == 100);
    CHECK(second_points->entry.x == 101);
    CHECK(second_points->exit.x == 2);
}

TEST_CASE("Extrusion-tree ordering propagates exact exits across layers",
          "[plugins][ordering][extrusion-tree-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.back();
    group.layers.reserve(2);

    PrintingLayerGroup &first_layer = append_layer(group);
    append_extrusion(first_layer.tool_groups.back(), make_path(Point(0, 0), Point(10, 0)));

    PrintingLayerGroup &second_layer = append_layer(group);
    append_extrusion(
        second_layer.tool_groups.back(),
        make_sortable_root({
            {Point(1, 0), Point(2, 0)},
            {Point(101, 0), Point(102, 0)}}));

    PrintingExtrusion &source = group.layers[0].tool_groups[0].extrusions[0];
    PrintingExtrusion &target = group.layers[1].tool_groups[0].extrusions[0];

    // The pre-sort estimate is deliberately stale. Sequential tree ordering
    // replaces it before selecting the first child of the following layer.
    EntryPointProperty &estimate = entry_point_key().get_or_add(mutable_root(source));
    estimate.entry = c_point{0, 0};
    estimate.exit = c_point{100, 0};

    run_tree_ordering(print);

    CHECK(target.root->child(0).first_point() == Point(1, 0));
    const EntryPointProperty *source_points = entry_point_key().get(mutable_root(source));
    REQUIRE(source_points != nullptr);
    CHECK(source_points->exit.x == 10);
}

TEST_CASE("Extrusion-tree ordering falls back to origin and removes empty metadata",
          "[plugins][ordering][extrusion-tree-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingGroup &group = plan.groups.back();
    group.layers.reserve(2);
    append_layer(group);

    PrintingLayerGroup &second_layer = append_layer(group);
    PrintingToolGroup &tool = second_layer.tool_groups.back();
    tool.extrusions.reserve(2);
    append_extrusion(
        tool, make_sortable_root({
            {Point(100, 0), Point(110, 0)},
            {Point(1, 0), Point(2, 0)}}));
    append_extrusion(
        tool, std::make_unique<ExtrusionEntityCollection>(true, true));

    PrintingExtrusion &ordered = group.layers[1].tool_groups[0].extrusions[0];
    PrintingExtrusion &empty = group.layers[1].tool_groups[0].extrusions[1];

    const PluginPropertyKey<EntryPointProperty> key = entry_point_key();
    EntryPointProperty &stale = key.get_or_add(mutable_root(empty));
    stale.entry = c_point{7, 8};
    stale.exit = c_point{9, 10};

    run_tree_ordering(print);

    // No earlier geometric root or pre-sort estimate exists, so the first
    // sortable root starts from the documented neutral origin.
    CHECK(ordered.root->child(0).first_point() == Point(1, 0));
    CHECK(key.get(mutable_root(empty)) == nullptr);
}

TEST_CASE("Extrusion-tree ordering uses the first pre-sort estimate",
          "[plugins][ordering][extrusion-tree-ordering]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    PrintingLayerGroup &layer = append_layer(plan.groups.back());
    PrintingExtrusion &extrusion = append_extrusion(
        layer.tool_groups.back(),
        make_sortable_root({
            {Point(1, 0), Point(2, 0)},
            {Point(101, 0), Point(102, 0)}}));

    EntryPointProperty &estimate =
        entry_point_key().get_or_add(mutable_root(extrusion));
    estimate.entry = c_point{100, 0};
    estimate.exit = c_point{100, 0};

    run_tree_ordering(print);

    CHECK(extrusion.root->child(0).first_point() == Point(101, 0));
    const EntryPointProperty *exact =
        entry_point_key().get(mutable_root(extrusion));
    REQUIRE(exact != nullptr);
    CHECK(exact->entry.x == 101);
    CHECK(exact->exit.x == 2);
}

TEST_CASE("STEP_ORDERING precondition validates perimeter loop metadata",
          "[plugins][ordering][precondition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("accepts a valid terminal loop")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_closed_terminal_loop(true, true));

        std::string error = "stale diagnostic";
        CHECK(Steps::StepExtrusionOrdering::validate_pre(fixture.print(), &error));
        CHECK(error.empty());
    }

    SECTION("reports a terminal loop without its direct property")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_closed_terminal_loop(false, false));

        require_precondition_failure(
            fixture.print(),
            "missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter",
            "object[0].normal_layer[0].island[0].region_island[0].perimeters.root.child[0]");
    }

    SECTION("reports a terminal loop whose property lacks the LOOP bit")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_closed_terminal_loop(true, false));

        require_precondition_failure(
            fixture.print(),
            "C_EXTRUSION_PERIMETER_FLAG_LOOP",
            "object[0].normal_layer[0].island[0].region_island[0].perimeters.root.child[0]");
    }

    SECTION("validates the direct loop child of a closed container")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_closed_loop_container());

        std::string error;
        CHECK(Steps::StepExtrusionOrdering::validate_pre(fixture.print(), &error));
        CHECK(error.empty());
    }

    SECTION("does not require properties on open fragments inside a loop")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_loop_with_open_fragments());

        std::string error;
        CHECK(Steps::StepExtrusionOrdering::validate_pre(fixture.print(), &error));
        CHECK(error.empty());
    }

    SECTION("rejects the LOOP flag on an open fragment")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_loop_with_flagged_open_fragment());

        require_precondition_failure(
            fixture.print(),
            "C_EXTRUSION_PERIMETER_FLAG_LOOP but is_loop() is false",
            "object[0].normal_layer[0].island[0].region_island[0].perimeters.root.child[0].child[0]");
    }

    SECTION("recurses through an open wrapper to its terminal loop")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_open_wrapper_with_invalid_loop());

        require_precondition_failure(
            fixture.print(),
            "missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter",
            "object[0].normal_layer[0].island[0].region_island[0].perimeters.root.child[0].child[0]");
    }

    SECTION("checks object auxiliary layers")
    {
        PerimeterSourceFixture fixture;
        LayerRegionIsland &region_island = fixture.add_object_auxiliary_region_island();
        publish_perimeter_tree(region_island, make_closed_terminal_loop(false, false));

        require_precondition_failure(
            fixture.print(),
            "missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter",
            "object[0].auxiliary_layer[0].island[0].region_island[0].perimeters.root.child[0]");
    }

    SECTION("checks print auxiliary layers")
    {
        PerimeterSourceFixture fixture;
        LayerRegionIsland &region_island = fixture.add_print_auxiliary_region_island();
        publish_perimeter_tree(region_island, make_closed_terminal_loop(false, false));

        require_precondition_failure(
            fixture.print(),
            "missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter",
            "print.auxiliary_layer[0].island[0].region_island[0].perimeters.root.child[0]");
    }

    SECTION("run_step rejects invalid source trees before creating the plan")
    {
        PerimeterSourceFixture fixture;
        publish_perimeter_tree(fixture, make_closed_terminal_loop(false, false));

        std::string thrown_message;
        try {
            Steps::StepExtrusionOrdering::run_step(Orchestrator::instance(), fixture.print());
        } catch (const RuntimeError &error) {
            thrown_message = error.what();
        }

        CHECK(thrown_message.find("STEP_ORDERING pre-validation failed") != std::string::npos);
        CHECK(thrown_message.find("missing direct ExtrusionPropertyLoopRole/EPropertyPerimeter") !=
              std::string::npos);
        CHECK(fixture.print().printing_plan() == nullptr);
    }
}
