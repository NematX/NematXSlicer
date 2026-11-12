#include <catch2/catch.hpp>

#include <atomic>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"

/* This file tests XY-serpentine extrusion ordering, seam positions and failures.
   Extrusions are material-depositing movements. Serpentine ordering visits
   geometry in alternating spatial directions; a seam is a closed loop's start.

   SerpentineSeamPlugin supplies a controlled seam service, a directly callable
   plugin job. SerpentineSeamSession returns a chosen position for one use.
   Orchestrator, the application's plugin manager, temporarily enables these
   alternatives. Group preference ranks them separately from execution order.

   TEST_CASE declarations, executed by the test runner, build source extrusion
   trees, call run_serpentine(), then examine PrintingPlan, the output movement
   plan, and reported errors. Cases cover geometry ordering and seam-service
   failures without permanently changing application plugin selection.

   Call graph:
     TEST_CASE
       run_serpentine
         OrchestratorView::execute
           Orchestrator::execute_plugin */

/*
XY-serpentine ordering tests
============================

The fixtures expose each subtree centre through small paths and identify moved
PrintingExtrusion values through a side table keyed by stable root handles. A
deterministic seam service keeps spatial-order assertions independent from seam
policy while test-only identity remains beside the stable extrusion root.
*/

namespace {
// Execute the serpentine provider on the borrowed print's mutable plan using prepared source regions; return the execution status.
// Missing providers fail REQUIRE, and the plan remains print-owned.
raw_plugin_execution_status run_serpentine(Slic3r::Print &print);

using namespace Slic3r;
using namespace Slic3r::Printing;
using slic3r_api::EntryPointProperty;
using slic3r_api::PluginPropertyKey;

const char *k_serpentine_plugin = "ordering.extrusion_tree.xy_serpentine";
const char *k_seam_plugin = "slic3r.test.xy_serpentine.seam_placer";
const char *k_seam_group = "seam_placer_plugin";

enum class SeamMode { Front, Fixed, Empty };

struct SeamState
{
    std::atomic<uint32_t> initialize_count { 0 };
    std::atomic<uint32_t> destroy_count { 0 };
    std::atomic<uint32_t> place_count { 0 };
    SeamMode mode = SeamMode::Front;
    c_point fixed = {};

    void reset()
    {
        initialize_count.store(0, std::memory_order_relaxed);
        destroy_count.store(0, std::memory_order_relaxed);
        place_count.store(0, std::memory_order_relaxed);
        mode = SeamMode::Front;
        fixed = {};
    }
};

SeamState g_seam;
std::vector<std::unique_ptr<LayerRegionIsland>> g_source_islands;
std::map<const ExtrusionEntity *, uint16_t> g_extrusion_identities;

/*
Keep one real broad LayerRegion alive for synthetic plan source buckets.

Ordering now resolves regional seam settings from the source island. The broad
slice associates every synthetic extrusion with one real PrintRegion while the
static PreparedPerimeterPrint preserves all borrowed lifetimes for the suite.
*/
Print &broad_source_print()
{
    using namespace Slic3r::Test::PerimeterPluginTests;
    static std::unique_ptr<PreparedPerimeterPrint> prepared = []() {
        std::unique_ptr<PreparedPerimeterPrint> value(new PreparedPerimeterPrint());
        prepare_cube_print(*value, perimeter_config({}));
        LayerRegion &region = value->print.object(0).layer(0).region(0);
        ExPolygons &raw_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
        raw_slices = ExPolygons{
            rectangle_expolygon(-1000.0, -1000.0, 1000.0, 1000.0)};
        ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(
            raw_slices, stPosInternal | stDensSolid);
        return value;
    }();
    return prepared->print;
}

const LayerRegion &broad_source_region()
{
    return broad_source_print().object(0).layer(0).region(0);
}

class SerpentineSeamSession final : public slic3r_api::SeamPlacerSession
{
public:
    ~SerpentineSeamSession() override
    {
        g_seam.destroy_count.fetch_add(1, std::memory_order_relaxed);
    }

    void initialize(
        const slic3r_api::PrintingPlan &,
        const std::vector<slic3r_api::PrintRegion> &) override
    {
        g_seam.initialize_count.fetch_add(1, std::memory_order_relaxed);
    }

    float get_priority(const slic3r_api::ExtrusionEntity &,
                       const slic3r_api::LayerRegion &) const override
    {
        return 0.f;
    }

    c_point place_seam(const slic3r_api::ExtrusionEntity &loop,
                       const slic3r_api::LayerRegion &,
                       c_point) const override
    {
        g_seam.place_count.fetch_add(1, std::memory_order_relaxed);
        return g_seam.mode == SeamMode::Fixed ? g_seam.fixed : loop.front();
    }
};

class SerpentineSeamPlugin final : public slic3r_api::PluginBase
{
public:
    explicit SerpentineSeamPlugin(orchestrator_handle *orchestrator) :
        PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_seam_plugin; }
    const char *exclusive_group_impl() const noexcept override { return k_seam_group; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    const char *const *dependencies_impl() const noexcept override
    {
        static const char *dependencies[] = { nullptr };
        return dependencies;
    }
    // Return the priority shared by seam-placement jobs. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 600; }
    // Return -1 to prefer this test service over application alternatives. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return -1; }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_ctx);
        if (context == nullptr || context->plan == nullptr)
            throw std::invalid_argument("The seam fixture needs a PrintingPlan.");
        if (g_seam.mode == SeamMode::Empty)
            return;
        std::vector<slic3r_api::PrintRegion> regions;
        regions.reserve(context->region_count);
        for (uint32_t region_idx = 0; region_idx < context->region_count; ++region_idx)
            regions.push_back(slic3r_api::PrintRegion(context->regions[region_idx]));
        context->instance = slic3r_api::make_seam_placer_instance(
            std::unique_ptr<slic3r_api::SeamPlacerSession>(
                new SerpentineSeamSession()),
            slic3r_api::PrintingPlan(context->plan), regions);
    }
};

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> ids) :
        m_orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous.push_back(plugin);
        m_orchestrator.clear_active_plugins();
        for (const char *id : ids)
            REQUIRE(m_orchestrator.set_plugin_active(id, true));
        m_orchestrator.reset_plugin_cancel();
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous)
            m_orchestrator.set_plugin_active(plugin, true);
        m_orchestrator.reset_plugin_cancel();
        g_seam.reset();
        g_source_islands.clear();
        g_extrusion_identities.clear();
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous;
};

/* Register the test service once; the production provider is registered by the runtime. */
void ensure_plugins_registered()
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    (void)broad_source_region();
    Orchestrator &orchestrator = Orchestrator::instance();
    if (orchestrator.get_plugin(k_seam_plugin) != nullptr)
        return;
    static std::unique_ptr<SerpentineSeamPlugin> seam_plugin;
    seam_plugin.reset(new SerpentineSeamPlugin(
        reinterpret_cast<orchestrator_handle *>(&orchestrator)));
    REQUIRE(orchestrator.register_plugin(seam_plugin->c_instance()));
}

c_point mm_point(const double x, const double y)
{
    return c_point{scale_i(x), scale_i(y)};
}

bool same_point(const c_point lhs, const c_point rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

ExtrusionEntityUPtr make_open_path(const double x0, const double y0,
                                   const double x1, const double y1,
                                   const bool reversible = false)
{
    return std::make_unique<ExtrusionEntity>(
        reversible, ArcPolyline(Points{Point::new_scale(x0, y0),
                                       Point::new_scale(x1, y1)}));
}

ExtrusionEntityUPtr make_path(const Points &points)
{
    return std::make_unique<ExtrusionEntity>(false, ArcPolyline(points));
}

ExtrusionEntityUPtr make_empty()
{
    return std::make_unique<ExtrusionEntity>(false);
}

ExtrusionEntityUPtr make_collection(ExtrusionEntity::Children children,
                                    const bool sortable,
                                    const bool reversible)
{
    return std::make_unique<ExtrusionEntity>(
        std::move(children), sortable, reversible, false);
}

ExtrusionEntityUPtr make_rectangular_loop()
{
    return std::make_unique<ExtrusionEntity>(
        false, ArcPolyline(Points{Point::new_scale(0.0, 0.0),
                                  Point::new_scale(10.0, 0.0),
                                  Point::new_scale(10.0, 10.0),
                                  Point::new_scale(0.0, 10.0),
                                  Point::new_scale(0.0, 0.0)}));
}

ExtrusionEntityUPtr make_composed_loop()
{
    ExtrusionEntity::Children children;
    children.push_back(make_open_path(0.0, 0.0, 10.0, 0.0));
    children.push_back(make_open_path(10.0, 0.0, 10.0, 10.0));
    children.push_back(make_open_path(10.0, 10.0, 0.0, 0.0));
    return make_collection(std::move(children), false, false);
}

ExtrusionEntityUPtr make_arc_loop()
{
    const coord_t radius = scale_i(5.0);
    const Point center = Point::new_scale(10.0, 10.0);
    ArcPolyline polyline;
    polyline.append(center + Point(radius, 0));
    polyline.append(Geometry::ArcWelder::Segment(
        center + Point(0, radius), float(radius),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.append(Geometry::ArcWelder::Segment(
        center + Point(-radius, 0), float(radius),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.append(Geometry::ArcWelder::Segment(
        center + Point(0, -radius), float(radius),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.append(Geometry::ArcWelder::Segment(
        center + Point(radius, 0), float(radius),
        Geometry::ArcWelder::Orientation::CCW));
    polyline.set_z_offset(0, 0);
    polyline.set_z_offset(1, scale_i(0.2));
    polyline.set_z_offset(2, scale_i(0.4));
    polyline.set_z_offset(3, scale_i(0.2));
    polyline.set_z_offset(4, 0);
    return std::make_unique<ExtrusionEntity>(false, std::move(polyline));
}

PrintingToolGroup &make_plan_with_tool(Print &print, const extruder_index_t extruder = 0)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    plan.groups.back().layers.back().tool_groups.emplace_back();
    PrintingToolGroup &tool = plan.groups.back().layers.back().tool_groups.back();
    tool.extruder_id = extruder;
    return tool;
}

/* Attach the perimeter marker to a root; it applies to every loop below it. */
void mark_perimeter(ExtrusionEntity &entity)
{
    slic3r_api::MutableExtrusionEntity view(reinterpret_cast<extrusion_entity_handle *>(&entity));
    view.get_or_add(slic3r_api::EPropertyPerimeter::key).perimeter_flags(C_EXTRUSION_PERIMETER_FLAG_LOOP);
}

PrintingExtrusion &append_extrusion(PrintingToolGroup &tool,
                                    ExtrusionEntityUPtr root,
                                    const uint16_t identity)
{
    // Synthetic roots stand for perimeter output; seam providers bind only marked loops.
    if (root != nullptr && !root->empty())
        mark_perimeter(*root);
    // Mirror production data: retain source geometry in the role bucket, then
    // give PrintingExtrusion an independently owned clone to reorder.
    std::unique_ptr<LayerRegionIsland> source(new LayerRegionIsland(
        LayerRegionSetCPtrs{&broad_source_region()}, tool.extruder_id));
    if (root != nullptr && !root->empty())
        source->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(*root);

    PrintingExtrusion extrusion;
    extrusion.region_island = source.get();
    extrusion.root = std::move(root);
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    if (extrusion.root != nullptr)
        g_extrusion_identities.emplace(extrusion.root.get(), identity);
    tool.region_islands.push_back(source.get());
    g_source_islands.push_back(std::move(source));
    tool.extrusions.push_back(std::move(extrusion));
    return tool.extrusions.back();
}

uint16_t extrusion_identity(const PrintingExtrusion &extrusion)
{
    if (extrusion.root == nullptr)
        throw std::runtime_error("An empty extrusion has no test identity.");
    return g_extrusion_identities.at(extrusion.root.get());
}

slic3r_api::MutableExtrusionEntity mutable_root(PrintingExtrusion &extrusion)
{
    return slic3r_api::MutableExtrusionEntity(
        reinterpret_cast<extrusion_entity_handle *>(extrusion.root.get()));
}

PluginPropertyKey<EntryPointProperty> entry_point_key()
{
    return slic3r_api::entry_point_property_key(
        reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()));
}

const EntryPointProperty &entry_points(PrintingExtrusion &extrusion)
{
    const EntryPointProperty *property = entry_point_key().get(mutable_root(extrusion));
    REQUIRE(property != nullptr);
    return *property;
}

void check_fixed(const ExtrusionEntity &entity)
{
    CHECK_FALSE(entity.can_sort());
    CHECK_FALSE(entity.can_reverse());
    for (const ExtrusionEntityUPtr &child : entity.children())
        check_fixed(*child);
}

raw_plugin_execution_status run_serpentine(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    slic3r_api::OrchestratorView view(
        reinterpret_cast<orchestrator_handle *>(&orchestrator));
    const slic3r_api::PluginView provider = view.find_plugin(k_serpentine_plugin);
    REQUIRE(provider.valid());
    PrintingPlan &plan = print.mutable_printing_plan();
    run_ctx_extrusion_ordering context = {};
    // Regional routing reads PrintRegions from the real prepared Print while
    // the ordering provider mutates the deliberately synthetic plan below.
    context.print = reinterpret_cast<const print_handle *>(&broad_source_print());
    context.plan = reinterpret_cast<printing_plan_handle *>(&plan);
    return view.execute(provider, context.print, context);
}

} // namespace

// Read registered ordering descriptions without mutation and check execution priority separately from the XY algorithm's preference.
TEST_CASE("XY-serpentine ordering is enabled without becoming the default provider",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *provider = orchestrator.get_plugin(k_serpentine_plugin);
    REQUIRE(provider != nullptr);
    CHECK(provider->get_step() == STEP_ORDERING);
    CHECK(provider->get_priority() == 1050);
    CHECK(provider->get_exclusive_group_priority() == 10);
    CHECK(provider->get_exclusive_group() == "ordering.extrusion_tree");

    std::ifstream stream(
        std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("ordering.extrusion_tree.advanced = 1") !=
          std::string::npos);
    CHECK(contents.str().find("ordering.extrusion_tree.xy_serpentine = 1") !=
          std::string::npos);
}

TEST_CASE("XY-serpentine ordering follows its directed weighted sweep",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    Print print;
    PrintingToolGroup &tool = make_plan_with_tool(print);

    append_extrusion(tool, make_open_path(99.5, 0.0, 100.5, 0.0), 100);
    append_extrusion(tool, make_open_path(-0.5, 12.0, 0.5, 12.0), 12);
    append_extrusion(tool, make_open_path(9.5, 2.0, 10.5, 2.0), 10);
    append_extrusion(tool, make_open_path(-0.5, 0.0, 0.5, 0.0), 0);

    REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(tool.extrusions.size() == 4);
    CHECK(extrusion_identity(tool.extrusions[0]) == 0);
    CHECK(extrusion_identity(tool.extrusions[1]) == 10);
    CHECK(extrusion_identity(tool.extrusions[2]) == 100);
    CHECK(extrusion_identity(tool.extrusions[3]) == 12);
    CHECK(g_seam.initialize_count.load(std::memory_order_relaxed) == 1);
    CHECK(g_seam.destroy_count.load(std::memory_order_relaxed) == 1);
}

TEST_CASE("XY-serpentine ordering uses bounding-box centres",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    Print print;
    PrintingToolGroup &tool = make_plan_with_tool(print);

    append_extrusion(tool, make_open_path(-0.5, 0.0, 0.5, 0.0), 0);
    append_extrusion(
        tool,
        make_path(Points{Point::new_scale(0.0, 1.0),
                         Point::new_scale(90.0, 1.0),
                         Point::new_scale(95.0, 1.0),
                         Point::new_scale(100.0, 1.0)}),
        50);
    append_extrusion(tool, make_open_path(69.5, 0.0, 70.5, 0.0), 70);

    REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(extrusion_identity(tool.extrusions[0]) == 0);
    CHECK(extrusion_identity(tool.extrusions[1]) == 50);
    CHECK(extrusion_identity(tool.extrusions[2]) == 70);
}

TEST_CASE("XY-serpentine ordering respects ownership and empty slots",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    Print print;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.back().layers.back();
    layer.tool_groups.emplace_back();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &first = layer.tool_groups[0];
    PrintingToolGroup &second = layer.tool_groups[1];
    first.extruder_id = 0;
    second.extruder_id = 1;

    append_extrusion(first, make_open_path(49.5, 0.0, 50.5, 0.0), 50);
    append_extrusion(first, make_empty(), 99);
    append_extrusion(first, make_open_path(-0.5, 0.0, 0.5, 0.0), 0);
    append_extrusion(second, make_open_path(19.5, 0.0, 20.5, 0.0), 20);
    append_extrusion(second, make_open_path(9.5, 0.0, 10.5, 0.0), 10);

    REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(extrusion_identity(first.extrusions[0]) == 0);
    CHECK(extrusion_identity(first.extrusions[1]) == 99);
    CHECK(extrusion_identity(first.extrusions[2]) == 50);
    CHECK(extrusion_identity(second.extrusions[0]) == 10);
    CHECK(extrusion_identity(second.extrusions[1]) == 20);
    CHECK(entry_point_key().get(mutable_root(first.extrusions[1])) == nullptr);
}

TEST_CASE("XY-serpentine ordering recursively fixes sortable trees",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    Print print;
    PrintingToolGroup &tool = make_plan_with_tool(print);

    ExtrusionEntity::Children nested_children;
    nested_children.push_back(make_open_path(60.0, 0.0, 61.0, 0.0, true));
    nested_children.push_back(make_open_path(20.0, 0.0, 21.0, 0.0, true));
    ExtrusionEntityUPtr nested =
        make_collection(std::move(nested_children), true, true);
    ExtrusionEntity *nested_first = nested->children()[0].get();
    ExtrusionEntity *nested_second = nested->children()[1].get();

    ExtrusionEntity::Children children;
    children.push_back(make_open_path(100.0, 0.0, 101.0, 0.0));
    children.push_back(make_empty());
    children.push_back(make_open_path(0.0, 0.0, 1.0, 0.0));
    children.push_back(std::move(nested));
    append_extrusion(tool, make_collection(std::move(children), true, false), 1);

    REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    const ExtrusionEntity &root = *tool.extrusions[0].root;
    REQUIRE(root.child_count() == 4);
    CHECK(root.child(0).first_point() == Point::new_scale(0.0, 0.0));
    CHECK(root.child(1).empty());
    CHECK(root.child(2).first_point() == Point::new_scale(20.0, 0.0));
    CHECK(root.child(3).first_point() == Point::new_scale(100.0, 0.0));
    CHECK(&root.child(2).child(0) == nested_second);
    CHECK(&root.child(2).child(1) == nested_first);
    check_fixed(root);
    CHECK(same_point(entry_points(tool.extrusions[0]).entry, mm_point(0.0, 0.0)));
    CHECK(same_point(entry_points(tool.extrusions[0]).exit, mm_point(101.0, 0.0)));
}

TEST_CASE("XY-serpentine ordering chooses strict reversible orientations",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};

    SECTION("a nearer end reverses an open path") {
        Print print;
        PrintingToolGroup &tool = make_plan_with_tool(print);
        append_extrusion(tool, make_open_path(10.0, 0.0, 1.0, 0.0, true), 1);
        REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
        CHECK(tool.extrusions[0].root->first_point() == Point::new_scale(1.0, 0.0));
        CHECK(tool.extrusions[0].root->last_point() == Point::new_scale(10.0, 0.0));
    }

    SECTION("equal distances preserve direction") {
        Print print;
        PrintingToolGroup &tool = make_plan_with_tool(print);
        append_extrusion(tool, make_open_path(-1.0, 0.0, 1.0, 0.0, true), 2);
        REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
        CHECK(tool.extrusions[0].root->first_point() == Point::new_scale(-1.0, 0.0));
        CHECK(tool.extrusions[0].root->last_point() == Point::new_scale(1.0, 0.0));
    }

    SECTION("a fixed collection reverses as one unit") {
        Print print;
        PrintingToolGroup &tool = make_plan_with_tool(print);
        append_extrusion(tool, make_open_path(30.5, -10.0, 31.0, -10.0), 1);
        ExtrusionEntity::Children children;
        children.push_back(make_open_path(10.0, 0.0, 20.0, 0.0, true));
        children.push_back(make_open_path(20.0, 0.0, 30.0, 0.0, true));
        ExtrusionEntityUPtr collection =
            make_collection(std::move(children), false, true);
        ExtrusionEntity *first = collection->children()[0].get();
        ExtrusionEntity *second = collection->children()[1].get();
        append_extrusion(tool, std::move(collection), 2);

        REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
        REQUIRE(extrusion_identity(tool.extrusions[1]) == 2);
        CHECK(tool.extrusions[1].root->children()[0].get() == second);
        CHECK(tool.extrusions[1].root->children()[1].get() == first);
        CHECK(tool.extrusions[1].root->first_point() == Point::new_scale(30.0, 0.0));
        CHECK(tool.extrusions[1].root->last_point() == Point::new_scale(10.0, 0.0));
    }
}

TEST_CASE("XY-serpentine ordering materializes line and arc seams",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    g_seam.mode = SeamMode::Fixed;

    SECTION("straight loop") {
        g_seam.fixed = mm_point(5.0, 0.0);
        Print print;
        PrintingToolGroup &tool = make_plan_with_tool(print);
        append_extrusion(tool, make_rectangular_loop(), 1);
        REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
        CHECK(tool.extrusions[0].root->first_point() == Point::new_scale(5.0, 0.0));
        CHECK(tool.extrusions[0].root->last_point() == Point::new_scale(5.0, 0.0));
        CHECK(same_point(entry_points(tool.extrusions[0]).entry, mm_point(5.0, 0.0)));
    }

    SECTION("arc loop") {
        g_seam.fixed = mm_point(13.0, 6.0);
        Print print;
        PrintingToolGroup &tool = make_plan_with_tool(print);
        append_extrusion(tool, make_arc_loop(), 2);
        slic3r_api::MutableExtrusionEntity root = mutable_root(tool.extrusions[0]);
        root.get_or_add(slic3r_api::EPropertyZOffset::key).set(scale_i(0.3));
        REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
        CHECK(tool.extrusions[0].root->first_point() == Point::new_scale(13.0, 6.0));
        CHECK(tool.extrusions[0].root->polyline_ref().has_arc());
        CHECK(tool.extrusions[0].root->polyline_ref().has_z_offset());
        REQUIRE(slic3r_api::EPropertyZOffset::key.get(root) != nullptr);
        CHECK(slic3r_api::EPropertyZOffset::key.get(root)->get() == scale_i(0.3));
    }
}

TEST_CASE("XY-serpentine ordering rotates a composed loop in place",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();
    ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
    g_seam.mode = SeamMode::Fixed;
    g_seam.fixed = mm_point(5.0, 0.0);
    Print print;
    PrintingToolGroup &tool = make_plan_with_tool(print);
    ExtrusionEntityUPtr loop = make_composed_loop();
    ExtrusionEntity *split = loop->children()[0].get();
    ExtrusionEntity *vertical = loop->children()[1].get();
    ExtrusionEntity *diagonal = loop->children()[2].get();
    append_extrusion(tool, std::move(loop), 1);
    slic3r_api::MutableExtrusionEntity split_view(
        reinterpret_cast<extrusion_entity_handle *>(split));
    split_view.get_or_add(slic3r_api::EPropertyZOffset::key).set(scale_i(0.4));

    REQUIRE(run_serpentine(print) == RAW_PLUGIN_EXECUTION_SUCCESS);
    const ExtrusionEntity &result = *tool.extrusions[0].root;
    REQUIRE(result.child_count() == 4);
    CHECK(result.children()[0].get() == split);
    CHECK(result.children()[1].get() == vertical);
    CHECK(result.children()[2].get() == diagonal);
    CHECK(result.first_point() == Point::new_scale(5.0, 0.0));
    CHECK(result.last_point() == result.first_point());
    REQUIRE(slic3r_api::EPropertyZOffset::key.get(split_view) != nullptr);
    CHECK(slic3r_api::EPropertyZOffset::key.get(split_view)->get() == scale_i(0.4));
    check_fixed(result);
}

TEST_CASE("XY-serpentine ordering reports an unavailable seam service",
          "[plugins][ordering][xy-serpentine]")
{
    ensure_plugins_registered();

    SECTION("no active provider") {
        ScopedActivePlugins active{k_serpentine_plugin};
        Print print;
        make_plan_with_tool(print);
        CHECK(run_serpentine(print) == RAW_PLUGIN_EXECUTION_PLUGIN_ERROR);
    }

    SECTION("provider publishes no instance") {
        ScopedActivePlugins active{k_serpentine_plugin, k_seam_plugin};
        g_seam.mode = SeamMode::Empty;
        Print print;
        make_plan_with_tool(print);
        CHECK(run_serpentine(print) == RAW_PLUGIN_EXECUTION_PLUGIN_ERROR);
    }
}
