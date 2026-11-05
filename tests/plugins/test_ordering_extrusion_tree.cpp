#include <catch2/catch.hpp>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionSeamPlanning.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
Fallback extrusion-tree ordering tests
======================================

The provider runs after the coarse PrintingExtrusion pre-sort. These tests
prebuild the relevant PrintingPlan fragments so they can verify how the first
estimate starts the sequential pass and how exact exits then propagate through
the remaining plan.
The linear STL export catalogue also exercises real ordering providers and random
seam settings through the complete pipeline, checking closed geometry and emitted
extrusions rather than treating the availability of a seam API as activation.
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
    // Capture activation, optionally retain the full pipeline, then enable the requested alternatives.
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids, bool keep_existing = false) :
        m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        if (!keep_existing)
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

TEST_CASE("Rotating a composed loop to a seam next to a fragment end keeps its fragments valid",
          "[plugins][ordering][seam]")
{
    // ArcPolyline::split_at() returns a one-point piece when the seam is within
    // SCALED_EPSILON / 2 of an end of the path. The rotation must move such a seam
    // onto the fragment boundary instead of keeping that piece as a closing leaf.
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    const auto point = [](const double x, const double y) {
        return c_point{static_cast<decltype(c_point::x)>(std::llround(x * 1e6)),
                       static_cast<decltype(c_point::y)>(std::llround(y * 1e6))};
    };
    const auto rotate_square_loop = [&](const c_point seam, const c_point boundary) {
        slic3r_api::StoredExtrusionEntity first(handle, {point(0, 0), point(10, 0), point(10, 10)});
        slic3r_api::StoredExtrusionEntity second(handle, {point(10, 10), point(0, 10), point(0, 0)});
        slic3r_api::StoredExtrusionEntity loop(handle);
        loop.append_child_move(first.mutable_view());
        loop.append_child_move(second.mutable_view());
        loop.disable_sort().disable_reverse();
        REQUIRE(loop.readonly().is_loop());
        const distf_t length = loop.readonly().length();

        slic3r_api::Ordering::TreeOrderingGeometry::rotate_loop_to_seam(handle, loop.mutable_view(), seam, nullptr);

        CHECK(loop.readonly().is_loop());
        CHECK(loop.readonly().length() == length);
        CHECK(slic3r_api::points_equal(loop.readonly().front(), boundary));
        CHECK(slic3r_api::points_equal(loop.readonly().back(), boundary));
        REQUIRE(loop.child_count() == 2);
        for (uint32_t child_idx = 0; child_idx < loop.child_count(); ++child_idx)
            CHECK(loop.child(child_idx).point_count() >= 2);
    };

    SECTION("near the start of a fragment")
    {
        c_point seam = point(0, 0);
        seam.x += SCALED_EPSILON / 4;
        rotate_square_loop(seam, point(0, 0));
    }

    SECTION("near the end of a fragment")
    {
        c_point seam = point(10, 10);
        seam.y -= SCALED_EPSILON / 4;
        rotate_square_loop(seam, point(10, 10));
    }
}

// Exercise the declared random-seam settings and all tree-ordering alternatives
// independently. Every run owns a fresh STL-derived plan and final output file.
TEST_CASE("Seam consumers pass a declared linear STL export catalogue",
          "[plugins][plugin-e2e][seam-consumer-e2e]")
{
    namespace E2E = Slic3r::Test::PluginE2E;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedActivePlugins active({"ordering.extrusion_tree.default",
        "ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine", "seam_placer.random"}, true);
    Orchestrator &host = Orchestrator::instance();
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    const std::string seam_key = Steps::plugin_exclusive_group_option_key(host, SEAM_PLACER, "seam_placer_plugin");
    REQUIRE_FALSE(seam_key.empty());
    baseline.set_deserialize_strict(seam_key, "seam_placer.random");
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"skirts", "1"},
        {"seam_random_seed", "42"}, {"seam_random_vertices_only", "1"}, {"seam_random_align_inner_seams", "1"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
    const std::vector<E2E::Domain> domains = {
        {{"seam_random_vertices_only"}, {{"projected-random", {{"seam_random_vertices_only", "0"}}}}},
        {{"seam_random_align_inner_seams"}, {{"independent-inner", {{"seam_random_align_inner_seams", "0"}}}}},
        {{"seam_random_seed"}, {{"fresh-entropy", {{"seam_random_seed", "0"}}},
            {"seed-one", {{"seam_random_seed", "1"}}}, {"seed-large", {{"seam_random_seed", "2147483647"}}}}}
    };
    const TriangleMesh mesh = E2E::stl_fixture(false);
    std::vector<E2E::Case> cases = E2E::linear_cases(host, {"ordering.extrusion_tree.default",
        "ordering.extrusion_tree.advanced", "ordering.extrusion_tree.xy_serpentine", "seam_placer.random"}, baseline, domains);
    // Startup exposes no tree selector when only the fallback was active while
    // definitions were finalized. Vary activation without changing that schema.
    cases.push_back({"fallback", {}});
    cases.push_back({"serpentine", {}});
    for (const E2E::Case &test : cases) {
        INFO(test.name);
        ScopedActivePlugins restore_activation({}, true);
        const std::string selected = test.name == "fallback" ? "ordering.extrusion_tree.default" :
            test.name == "serpentine" ? "ordering.extrusion_tree.xy_serpentine" : "ordering.extrusion_tree.advanced";
        for (const char *id : {"ordering.extrusion_tree.default", "ordering.extrusion_tree.advanced",
                              "ordering.extrusion_tree.xy_serpentine"})
            REQUIRE(host.set_plugin_active(id, selected == id));
        REQUIRE(host.is_plugin_active(host.get_plugin(selected)));
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
        print.process();
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            code = Test::GCode::read_text_file(host.export_gcode_cli(print, path.string()));
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        REQUIRE(print.printing_plan() != nullptr);
        size_t closed = 0;
        // Inspect all owned descendants without calling legacy path views.
        const auto inspect = [&closed](auto &&self, const ExtrusionEntity &entity) -> void {
            if (entity.ExtrusionEntity::is_loop()) ++closed;
            if (entity.has_polyline()) CHECK(entity.polyline_ref().is_valid());
            for (const ExtrusionEntityUPtr &child : entity.children()) self(self, *child);
        };
        for (const PrintingGroup &group : print.printing_plan()->groups)
            for (const PrintingLayerGroup &layer : group.layers)
                for (const PrintingToolGroup &tool : layer.tool_groups)
                    for (const PrintingExtrusion &extrusion : tool.extrusions)
                        if (extrusion.root) inspect(inspect, *extrusion.root);
        CHECK(closed > 0);
        CHECK(code.find(";SEAM:") != std::string::npos);
        CHECK(code.find(";TYPE:Skirt") != std::string::npos);
        size_t deposited_moves = 0;
        GCodeReader reader;
        reader.parse_buffer(code, [&deposited_moves](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (line.extruding(state) && line.dist_XY(state) > 0.) ++deposited_moves;
        });
        CHECK(deposited_moves > 0);
    }
}

// Delegated rotations reuse the owned contour while property-only siblings stay fixed.
TEST_CASE("Prepared seam materialization delegates successive seams without moving empty events",
          "[plugins][ordering][seam-materialization-preparation]")
{
    namespace Geometry = slic3r_api::Ordering::TreeOrderingGeometry;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(std::make_unique<ExtrusionEntity>(false));
    root.child(0).add_property(ExtrusionPropertySpeed(25.f));
    std::unique_ptr<ExtrusionEntity> nested = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    nested->append_child(std::make_unique<ExtrusionEntity>(false,
        ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 10000}, {0, 0}})));
    root.append_child(std::move(nested));
    const ExtrusionEntity *event = &root.child(0);
    const ExtrusionEntity *contour = &root.child(1).child(0);
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&root));
    for (const c_point requested : {c_point{5000, 0}, c_point{10000, 5000}}) {
        const c_point legal = Geometry::legal_seam_point(view.readonly(), requested, true);
        CHECK(legal.x == requested.x);
        CHECK(legal.y == requested.y);
        Geometry::rotate_loop_to_legal_seam(handle, view, requested, true);
        CHECK(root.ExtrusionEntity::first_point() == Point(requested.x, requested.y));
        CHECK(root.ExtrusionEntity::last_point() == Point(requested.x, requested.y));
        REQUIRE(root.child_count() == 2);
        REQUIRE(root.child(1).child_count() == 1);
        CHECK(&root.child(0) == event);
        CHECK(&root.child(1).child(0) == contour);
        CHECK(root.child(0).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 25.f);
        CHECK(root.can_rotate());
        CHECK(contour->can_rotate());
    }
}

// A composed open child keeps its entry and handles; only direct leaf splits add a closing fragment.
TEST_CASE("Prepared seam materialization keeps open composites atomic and splits only a direct leaf",
          "[plugins][ordering][seam-materialization-preparation]")
{
    namespace Geometry = slic3r_api::Ordering::TreeOrderingGeometry;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(make_polyline_entity(Points{{0, 0}, {10000, 0}}));
    std::unique_ptr<ExtrusionEntity> open = std::make_unique<ExtrusionEntity>(ExtrusionEntity::Children(), false, false, true);
    open->set_can_rotate(false);
    open->append_child(make_polyline_entity(Points{{10000, 0}, {10000, 10000}}));
    open->append_child(make_polyline_entity(Points{{10000, 10000}, {0, 10000}}));
    root.append_child(std::move(open));
    root.append_child(make_polyline_entity(Points{{0, 10000}, {0, 0}}));
    root.child(2).add_property(ExtrusionPropertySpeed(37.f));
    const ExtrusionEntity *atomic = &root.child(1);
    const ExtrusionEntity *suffix = &root.child(2);
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&root));
    Geometry::rotate_loop_to_legal_seam(handle, view, c_point{10000, 5000}, true);
    REQUIRE(root.child_count() == 3);
    CHECK(&root.child(0) == atomic);
    CHECK(root.ExtrusionEntity::first_point() == Point(10000, 0));
    CHECK(atomic->child(0).polyline_ref().front() == Point(10000, 0));
    CHECK(atomic->child(1).polyline_ref().front() == Point(10000, 10000));
    Geometry::rotate_loop_to_legal_seam(handle, view, c_point{0, 5000}, true);
    REQUIRE(root.child_count() == 4);
    CHECK(&root.child(0) == suffix);
    CHECK(root.ExtrusionEntity::first_point() == Point(0, 5000));
    CHECK(root.ExtrusionEntity::last_point() == Point(0, 5000));
    CHECK(root.is_continuous());
    CHECK(root.child(0).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 37.f);
    CHECK(root.child(3).get_property<ExtrusionPropertySpeed>()->speed_mm_per_s == 37.f);
    CHECK(root.can_rotate());
    CHECK_FALSE(atomic->can_rotate());
}

// Invalid requests reject before edits; a valid blocked request retains the existing start.
TEST_CASE("Prepared seam materialization rejects forbidden requests before mutation",
          "[plugins][ordering][seam-materialization-preparation]")
{
    namespace Geometry = slic3r_api::Ordering::TreeOrderingGeometry;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
    root.append_child(make_polyline_entity(Points{{0, 0}, {10000, 0}}));
    root.append_child(make_polyline_entity(Points{{10000, 0}, {0, 0}}));
    root.append_child(std::make_unique<ExtrusionEntity>(false));
    root.child(2).add_property(ExtrusionPropertySpeed(25.f));
    const ExtrusionEntity *first = &root.child(0);
    const ExtrusionEntity *event = &root.child(2);
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&root));
    CHECK_THROWS(Geometry::rotate_loop_to_legal_seam(handle, view, c_point{5000, 0}, false));
    CHECK_THROWS(Geometry::rotate_loop_to_legal_seam(handle, view, c_point{5000, 5000}, true));
    Geometry::rotate_loop_to_legal_seam(handle, view, c_point{5000, 0}, true);
    REQUIRE(root.child_count() == 3);
    CHECK(&root.child(0) == first);
    CHECK(&root.child(2) == event);
    CHECK(root.ExtrusionEntity::first_point() == Point(0, 0));
    CHECK(root.child(0).polyline_ref().size() == 2);
    root.set_can_rotate(false);
    CHECK_THROWS(Geometry::rotate_loop_to_legal_seam(handle, view, c_point{0, 0}, true));
    CHECK(root.ExtrusionEntity::first_point() == Point(0, 0));
    CHECK(&root.child(2) == event);
}

// Compare the announced entry and arc/Z metadata against the native split operation itself.
TEST_CASE("Prepared seam materialization agrees exactly with oblique and curved native splits",
          "[plugins][ordering][seam-materialization-preparation]")
{
    namespace Geometry = slic3r_api::Ordering::TreeOrderingGeometry;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    ArcPolyline path;
    Point requested;
    SECTION("oblique rounding regression") {
        path = ArcPolyline(Points{{0, 0}, {10000, 20100}, {0, 20100}, {0, 0}});
        requested = Point(141, 64);
    }
    SECTION("arc rounding and endpoint Z") {
        path.append(Point(10000, 0));
        path.append(Slic3r::Geometry::ArcWelder::Segment(Point(0, 10000), 10000.f, Slic3r::Geometry::ArcWelder::Orientation::CCW));
        path.append(Point(0, 0));
        path.append(Point(10000, 0));
        requested = Point(6001, 8000);
    }
    for (size_t index = 0; index < path.size(); ++index)
        path.set_z_offset(index, coord_t(100 + index * 300));
    // A cyclic path has the same stored Z at its closing endpoint.
    path.set_z_offset(path.size() - 1, path.z_offset(0));
    for (const bool composed : {false, true}) {
        CAPTURE(composed);
        ExtrusionEntity root(false, path);
        if (composed) {
            ArcPolyline first;
            ArcPolyline second;
            REQUIRE(path.split_at_index(1, first, second));
            root = ExtrusionEntity(ExtrusionEntity::Children(), false, false, true);
            root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(first)));
            root.append_child(std::make_unique<ExtrusionEntity>(false, std::move(second)));
        }
        const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(root, requested, true);
        REQUIRE(plan);
        const ArcPolyline &source = composed ? root.child(0).polyline_ref() : root.polyline_ref();
        ArcPolyline before;
        ArcPolyline after;
        Point input = plan->split_point;
        source.split_at(input, before, after);
        REQUIRE(before.size() >= 2);
        REQUIRE(after.size() >= 2);
        CHECK(before.back() == plan->point);
        CHECK(after.front() == plan->point);
        slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&root));
        const c_point desired{requested.x(), requested.y()};
        const c_point announced = Geometry::legal_seam_point(view.readonly(), desired, true);
        CHECK(Point(announced.x, announced.y) == plan->point);
        Geometry::rotate_loop_to_legal_seam(handle, view, desired, true);
        CHECK(root.ExtrusionEntity::first_point() == plan->point);
        CHECK(root.ExtrusionEntity::last_point() == plan->point);
        CHECK(root.is_continuous());
        const ArcPolyline &suffix = composed ? root.child(0).polyline_ref() : root.polyline_ref();
        CHECK(suffix.z_offset(0) == after.z_offset(0));
        CHECK(suffix.get_arc(1).radius == after.get_arc(1).radius);
        CHECK(suffix.get_arc(1).orientation == after.get_arc(1).orientation);
        CHECK(root.can_rotate());
    }
}

// A rotatable parent cannot split a fixed closed child, with or without empty siblings.
TEST_CASE("Prepared seam materialization does not bypass a closed local child's permission",
          "[plugins][ordering][seam-materialization-preparation]")
{
    namespace Geometry = slic3r_api::Ordering::TreeOrderingGeometry;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    for (const bool empty_sibling : {false, true}) {
        CAPTURE(empty_sibling);
        ExtrusionEntity root(ExtrusionEntity::Children(), false, false, true);
        root.append_child(std::make_unique<ExtrusionEntity>(false,
            ArcPolyline(Points{{0, 0}, {10000, 0}, {10000, 10000}, {0, 0}})));
        root.child(0).set_can_rotate(false);
        if (empty_sibling)
            root.append_child(std::make_unique<ExtrusionEntity>(false));
        const ExtrusionEntity *contour = &root.child(0);
        slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&root));
        Geometry::rotate_loop_to_legal_seam(handle, view, c_point{5000, 0}, true);
        CHECK(root.ExtrusionEntity::first_point() == Point(0, 0));
        CHECK(root.child_count() == (empty_sibling ? 2 : 1));
        CHECK(&root.child(0) == contour);
        CHECK(root.child(0).polyline_ref().size() == 4);
        CHECK_FALSE(contour->can_rotate());
        CHECK(root.can_rotate());
    }
}
