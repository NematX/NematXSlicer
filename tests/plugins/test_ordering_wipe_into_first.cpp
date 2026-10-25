/* The wipe-into-first ordering plugin moves the geometry a wipe tower can purge to the
   front of the visit that follows a tool change. These tests exercise the plugin alone on
   compact hand-built plans (visit and bucket order) and through the production export
   pipeline (real sliced geometry, tool changes, purge volume and reader statistics).
   The hand-built plans borrow region islands from one sliced print so the object, region
   and perimeter-tool options read by the plugin are real configuration, and a real sliced
   pedestal checks that sparse infill stays separable at the infill bucket's direct
   children, which the plugin relies on to move sparse surfaces first. */
#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/scope_exit.hpp>
#include <tbb/global_control.h>

#include "gcode_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Steps/StepExtrusionOrdering.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

namespace {
using namespace Slic3r;

const char *const plugin_id = "ordering.wipe_into_first.default";

// One printable leaf of a hand-built plan extrusion: its role, its length in mm and a unique
// identity used as the start x of the leaf path.
struct LeafSpec
{
    ExtrusionRole role;
    double length;
    double identity;
};

// One hand-built PrintingExtrusion: its source bucket role, the borrowed region island and the
// printable leaves of its root.
struct ExtrusionSpec
{
    uint32_t id;
    ExtrusionRole bucket_role;
    const LayerRegionIsland *island = nullptr;
    std::vector<LeafSpec> leaves;
};

// One hand-built tool visit with its extrusions in print order.
struct VisitSpec
{
    extruder_index_t tool;
    std::vector<ExtrusionSpec> extrusions;
};

// Collect the raw roles of the printable leaves of a borrowed tree, in depth-first order.
void collect_printable_leaf_roles(const ExtrusionEntity &entity, const ExtrusionAttributes *inherited,
                                  std::vector<uint16_t> &roles)
{
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        inherited = attributes;
    if (entity.child_count() == 0) {
        if (inherited != nullptr && inherited->mm3_per_mm > 0. && entity.has_polyline())
            roles.push_back(inherited->role);
        return;
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        collect_printable_leaf_roles(entity.child(idx), inherited, roles);
}

// Sum the volume of the leaves of a tree whose stored attribute role equals raw_role.
double raw_role_volume(const ExtrusionEntity &entity, const uint16_t raw_role)
{
    if (entity.child_count() == 0) {
        const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>();
        return attributes != nullptr && attributes->role == raw_role ?
            unscaled(entity.length()) * attributes->mm3_per_mm : 0.;
    }
    double volume = 0.;
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        volume += raw_role_volume(entity.child(idx), raw_role);
    return volume;
}

// Return the printable roles of the direct children of one root, in order.
std::vector<std::vector<uint16_t>> bucket_child_roles(const ExtrusionEntity &root)
{
    std::vector<std::vector<uint16_t>> children;
    for (size_t idx = 0; idx < root.child_count(); ++idx) {
        std::vector<uint16_t> roles;
        collect_printable_leaf_roles(root.child(idx), nullptr, roles);
        children.push_back(std::move(roles));
    }
    return children;
}

// Classify the direct children of one infill bucket: children whose printable leaves are all
// sparse infill, children with no sparse leaf at all, and children mixing both.
void classify_infill_children(const ExtrusionEntity &root, size_t &sparse, size_t &non_sparse, size_t &mixed)
{
    sparse = non_sparse = mixed = 0;
    for (size_t idx = 0; idx < root.child_count(); ++idx) {
        std::vector<uint16_t> roles;
        collect_printable_leaf_roles(root.child(idx), nullptr, roles);
        const bool any_sparse =
            std::find(roles.begin(), roles.end(), uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)) != roles.end();
        const bool any_other = std::any_of(roles.begin(), roles.end(), [](const uint16_t role) {
            return role != uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
        });
        if (any_sparse && any_other)
            ++mixed;
        else if (any_sparse)
            ++sparse;
        else
            ++non_sparse;
    }
}

// Invoke only the registered wipe-into-first ordering plugin on the plan owned by print, with
// invocation-local error tracking; callback failures and cancellation fail the test.
void run_wipe_into_first(Print &print)
{
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    host.reset_plugin_cancel();
    plugin_host_context context = host.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
    std::atomic_bool error{false};
    context.execution_error = &error;
    plugin_run_context run = host.prepare_plugin_run_context(STEP_ORDERING, plugin, &context);
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&print.mutable_printing_plan());
    run.data = &payload;
    plugin->setup(run, 1);
    plugin->setup_run(run);
    plugin->run(run);
    REQUIRE_FALSE(error.load());
    REQUIRE_FALSE(host.is_plugin_cancelled());
}

// Slice two cubes once with a two-tool tower configuration, so the hand-built plans below can
// borrow two real region islands; configuration and slicing failures fail the test.
void slice_two_cubes(Print &print, Model &model, DynamicPrintConfig &config)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"},
        {"layer_height", "0.3"}, {"fill_density", "20%"}});
    Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
}

// Set the wipe-into options of one borrowed island; the first region owns the perimeter tool.
void set_island_options(const LayerRegionIsland &island, const bool into_infill, const bool infill_first,
                        const int perimeter_extruder)
{
    PrintRegionConfig &region =
        const_cast<PrintRegionConfig &>((*island.regions().begin())->region().config());
    region.wipe_into_infill.value = into_infill;
    region.infill_first.value = infill_first;
    region.perimeter_extruder.value = perimeter_extruder;
}

// Set the wipe-into-objects option of the object owning one borrowed island.
void set_object_option(const LayerRegionIsland &island, const bool into_objects)
{
    const_cast<PrintObjectConfig &>(
        (*island.regions().begin())->layer()->object()->config()).wipe_into_objects.value = into_objects;
}

// Replace the plan owned by print with one group, one layer and the given visits; every
// extrusion root is a collection of firmware paths carrying its leaf roles.
void build_plan(Print &print, const std::vector<VisitSpec> &visits)
{
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.clear();
    plan.groups.emplace_back();
    Printing::PrintingGroup &group = plan.groups.front();
    group.layers.clear();
    group.layers.emplace_back();
    Printing::PrintingLayerGroup &layer = group.layers.front();
    layer.print_z = scale_i(0.3);
    layer.tool_groups.clear();
    for (const VisitSpec &visit_spec : visits) {
        layer.tool_groups.emplace_back();
        Printing::PrintingToolGroup &visit = layer.tool_groups.back();
        visit.extruder_id = visit_spec.tool;
        visit.extrusions.clear();
        for (const ExtrusionSpec &extrusion_spec : visit_spec.extrusions) {
            Printing::PrintingExtrusion extrusion;
            std::unique_ptr<ExtrusionEntityCollection> root(new ExtrusionEntityCollection(false, false));
            for (const LeafSpec &leaf : extrusion_spec.leaves)
                root->append(Test::GCode::make_firmware_path(
                    ArcPolyline(Points{Point::new_scale(leaf.identity, 0.),
                                       Point::new_scale(leaf.identity + leaf.length, 0.)}),
                    20.f, 500.f, -1.f, -1.f, -1.f, leaf.role));
            extrusion.root = std::move(root);
            extrusion.sregion_island_role = extrusion_spec.bucket_role;
            extrusion.region_island = extrusion_spec.island;
            extrusion.printing_catalog_id = extrusion_spec.id;
            visit.extrusions.push_back(std::move(extrusion));
        }
    }
}

// Return the identities of the extrusions of one visit of the hand-built plan, in order.
std::vector<uint32_t> visit_ids(const Print &print, const uint32_t visit_index)
{
    const Printing::PrintingToolGroup &visit =
        print.printing_plan()->groups[0].layers[0].tool_groups[visit_index];
    std::vector<uint32_t> ids;
    for (const Printing::PrintingExtrusion &extrusion : visit.extrusions)
        ids.push_back(extrusion.printing_catalog_id);
    return ids;
}

// Find one extrusion of the hand-built plan by its identity, or null when it is absent.
const Printing::PrintingExtrusion *find_extrusion(const Print &print, const uint32_t id)
{
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &visit : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : visit.extrusions)
                    if (extrusion.printing_catalog_id == id)
                        return &extrusion;
    return nullptr;
}

// Return one extrusion of the hand-built plan by its identity; a missing identity fails the test.
const Printing::PrintingExtrusion &require_extrusion(const Print &print, const uint32_t id)
{
    const Printing::PrintingExtrusion *found = find_extrusion(print, id);
    REQUIRE(found != nullptr);
    return *found;
}

// The sparse-first infill bucket used by the hand-built tests: solid, sparse, solid, sparse.
ExtrusionSpec infill_bucket_spec(const uint32_t id, const LayerRegionIsland &island)
{
    return ExtrusionSpec{id, ExtrusionRole::InternalInfill, &island,
        {{ExtrusionRole::SolidInfill, 10., 100.},
         {ExtrusionRole::InternalInfill, 20., 200.},
         {ExtrusionRole::SolidInfill, 30., 300.},
         {ExtrusionRole::InternalInfill, 40., 400.}}};
}

// The unchanged child order of the sparse-first infill bucket used by the hand-built tests.
std::vector<std::vector<uint16_t>> unchanged_infill_child_roles()
{
    return {{uint16_t(RAW_EXTRUSION_ROLE_SOLID_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_SOLID_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)}};
}

// The sparse-first child order the plugin produces for the same bucket.
std::vector<std::vector<uint16_t>> sparse_first_infill_child_roles()
{
    return {{uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_INTERNAL_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_SOLID_INFILL)},
            {uint16_t(RAW_EXTRUSION_ROLE_SOLID_INFILL)}};
}
} // namespace

// Check the registration contract: an active, non-exclusive ordering plugin after the tree
// ordering plugins.
TEST_CASE("Wipe-into-first ordering registers after the tree ordering",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Plugin *plugin = Orchestrator::instance().get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    CHECK(Orchestrator::instance().is_plugin_active(plugin_id));
    CHECK(plugin->get_step() == STEP_ORDERING);
    CHECK(plugin->get_priority() == 1200);
    CHECK(plugin->get_exclusive_group().empty());
}

// A change to a tool whose island prints perimeters with the incoming tool: the island's infill
// bucket moves to the front of the visit and its sparse-only children move to the front of the
// bucket, keeping their relative order.
TEST_CASE("Wipe-into-first moves the infill bucket of the first island after a change",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    const LayerRegionIsland &island1 = print.object(1).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    set_object_option(island1, false);
    // Identical objects share one PrintRegion, so island1 keeps the default region options.
    set_island_options(island0, true, false, 2);

    build_plan(print, {
        VisitSpec{0, {ExtrusionSpec{1, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {ExtrusionSpec{2, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 20.}}},
                     infill_bucket_spec(3, island0),
                     ExtrusionSpec{4, ExtrusionRole::Ironing, &island0,
                                   {{ExtrusionRole::Ironing, 10., 30.}}}}}});
    run_wipe_into_first(print);

    CHECK(visit_ids(print, 0) == std::vector<uint32_t>{1});
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{3, 2, 4});
    const Printing::PrintingExtrusion *infill = find_extrusion(print, 3);
    REQUIRE(infill != nullptr);
    CHECK(bucket_child_roles(*infill->root) == sparse_first_infill_child_roles());
}

// A visit without a change is never touched, and a soluble outgoing or incoming filament keeps
// the plan untouched too.
TEST_CASE("Wipe-into-first leaves a visit without a change or a soluble filament untouched",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    set_island_options(island0, true, false, 2);
    PrintConfig &print_config = const_cast<PrintConfig &>(print.config());

    // One visit only: the plugin has no previous concrete tool, so nothing can follow a change.
    build_plan(print, {VisitSpec{0, {infill_bucket_spec(10, island0)}}});
    run_wipe_into_first(print);
    CHECK(visit_ids(print, 0) == std::vector<uint32_t>{10});
    CHECK(bucket_child_roles(*require_extrusion(print, 10).root) == unchanged_infill_child_roles());

    // A real change 0 -> 1, but one of the two filaments is soluble.
    const std::vector<VisitSpec> change_plan{
        VisitSpec{0, {ExtrusionSpec{11, ExtrusionRole::Perimeter, &island0,
                                    {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {infill_bucket_spec(12, island0)}}};
    build_plan(print, change_plan);
    print_config.filament_soluble.set(std::vector<unsigned char>{1, 0});
    run_wipe_into_first(print);
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{12});
    CHECK(bucket_child_roles(*require_extrusion(print, 12).root) == unchanged_infill_child_roles());
    print_config.filament_soluble.set(std::vector<unsigned char>{0, 1});
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 12).root) == unchanged_infill_child_roles());
    print_config.filament_soluble.set(std::vector<unsigned char>{0, 0});
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 12).root) == sparse_first_infill_child_roles());
}

// The perimeters-first rule: with wipe_into_infill alone, the island is only touched when its
// perimeter tool is the incoming tool, prints earlier in the layer, or infill_first is set.
TEST_CASE("Wipe-into-first requires the perimeters to be printed before the incoming visit",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    const std::vector<VisitSpec> plan{
        VisitSpec{0, {ExtrusionSpec{20, ExtrusionRole::Perimeter, &island0,
                                    {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {infill_bucket_spec(21, island0)}},
        VisitSpec{2, {ExtrusionSpec{22, ExtrusionRole::Perimeter, &island0,
                                    {{ExtrusionRole::Perimeter, 10., 20.}}}}}};

    // Perimeter tool 3 prints after the incoming tool 1: the visit stays untouched.
    set_island_options(island0, true, false, 3);
    build_plan(print, plan);
    run_wipe_into_first(print);
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{21});
    CHECK(bucket_child_roles(*require_extrusion(print, 21).root) == unchanged_infill_child_roles());

    // Perimeter tool 1 printed in the visit before: the bucket moves and sparse comes first.
    set_island_options(island0, true, false, 1);
    build_plan(print, plan);
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 21).root) == sparse_first_infill_child_roles());

    // infill_first skips the perimeter rule.
    set_island_options(island0, true, true, 3);
    build_plan(print, plan);
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 21).root) == sparse_first_infill_child_roles());
}

// wipe_into_objects moves every extrusion whose island belongs to that object to the front of
// the visit, keeping their relative order; support has no island and stays where it was.
TEST_CASE("Wipe-into-first moves wipe-into-object extrusions first and leaves support alone",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    const LayerRegionIsland &island1 = print.object(1).layer(0).islands().front().regions_islands().front();
    set_island_options(island0, false, false, 1);
    set_island_options(island1, false, false, 1);
    set_object_option(island0, true);
    set_object_option(island1, false);

    build_plan(print, {
        VisitSpec{0, {ExtrusionSpec{30, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {ExtrusionSpec{31, ExtrusionRole::SupportMaterial, nullptr,
                                   {{ExtrusionRole::SupportMaterial, 10., 10.}}},
                     ExtrusionSpec{32, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 20.}}},
                     ExtrusionSpec{33, ExtrusionRole::InternalInfill, &island0,
                                   {{ExtrusionRole::InternalInfill, 10., 30.}}},
                     ExtrusionSpec{34, ExtrusionRole::Perimeter, &island1,
                                   {{ExtrusionRole::Perimeter, 10., 40.}}},
                     ExtrusionSpec{35, ExtrusionRole::Ironing, &island0,
                                   {{ExtrusionRole::Ironing, 10., 50.}}}}}});
    run_wipe_into_first(print);

    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{32, 33, 35, 31, 34});
}

// Only the first island after the wipe-into-objects block can receive the infill-first move;
// later islands of the same visit keep their buckets and children.
TEST_CASE("Wipe-into-first touches only the first island after a change",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    const LayerRegionIsland &island1 = print.object(1).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    set_object_option(island1, false);
    set_island_options(island0, true, false, 2);
    set_island_options(island1, true, false, 2);

    build_plan(print, {
        VisitSpec{0, {ExtrusionSpec{40, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {ExtrusionSpec{41, ExtrusionRole::Perimeter, &island0,
                                   {{ExtrusionRole::Perimeter, 10., 20.}}},
                     infill_bucket_spec(42, island0),
                     ExtrusionSpec{43, ExtrusionRole::Perimeter, &island1,
                                   {{ExtrusionRole::Perimeter, 10., 30.}}},
                     infill_bucket_spec(44, island1)}}});
    run_wipe_into_first(print);

    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{42, 41, 43, 44});
    CHECK(bucket_child_roles(*require_extrusion(print, 42).root) == sparse_first_infill_child_roles());
    CHECK(bucket_child_roles(*require_extrusion(print, 44).root) == unchanged_infill_child_roles());
}

// With wipe_into_infill off or without a wipe tower, the plan is the same as before the plugin.
TEST_CASE("Wipe-into-first leaves the plan unchanged when the options are off or there is no tower",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    set_island_options(island0, true, false, 2);
    const std::vector<VisitSpec> plan{
        VisitSpec{0, {ExtrusionSpec{50, ExtrusionRole::Perimeter, &island0,
                                    {{ExtrusionRole::Perimeter, 10., 10.}}}}},
        VisitSpec{1, {ExtrusionSpec{51, ExtrusionRole::Perimeter, &island0,
                                    {{ExtrusionRole::Perimeter, 10., 20.}}},
                     infill_bucket_spec(52, island0)}}};

    // The option is off.
    set_island_options(island0, false, false, 2);
    build_plan(print, plan);
    run_wipe_into_first(print);
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{51, 52});
    CHECK(bucket_child_roles(*require_extrusion(print, 52).root) == unchanged_infill_child_roles());

    // The option is on but the print has no wipe tower.
    set_island_options(island0, true, false, 2);
    build_plan(print, plan);
    PrintObjectConfig &object_config = const_cast<PrintObjectConfig &>(print.default_object_config());
    const bool tower_was_enabled = object_config.wipe_tower.value;
    object_config.wipe_tower.value = false;
    run_wipe_into_first(print);
    object_config.wipe_tower.value = tower_was_enabled;
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{51, 52});
    CHECK(bucket_child_roles(*require_extrusion(print, 52).root) == unchanged_infill_child_roles());

    // Control: with the option on and the tower present, the same plan changes.
    build_plan(print, plan);
    run_wipe_into_first(print);
    CHECK(visit_ids(print, 1) == std::vector<uint32_t>{52, 51});
    CHECK(bucket_child_roles(*require_extrusion(print, 52).root) == sparse_first_infill_child_roles());
}

// An unresolved or unconfigured visit cannot be purged, so it is skipped without resetting the
// previous concrete tool: the next real change still moves the infill bucket first.
TEST_CASE("Wipe-into-first keeps the previous tool across unresolved visits",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Print print;
    Model model;
    DynamicPrintConfig config;
    slice_two_cubes(print, model, config);
    const LayerRegionIsland &island0 = print.object(0).layer(0).islands().front().regions_islands().front();
    set_object_option(island0, false);
    set_island_options(island0, true, false, 2);

    const VisitSpec previous{0, {ExtrusionSpec{60, ExtrusionRole::Perimeter, &island0,
                                               {{ExtrusionRole::Perimeter, 10., 10.}}}}};
    const VisitSpec incoming{1, {infill_bucket_spec(61, island0)}};

    // An unresolved tool visit between the two concrete tools.
    build_plan(print, {previous, VisitSpec{EXTRUDER_ID_UNKNOWN, {}}, incoming});
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 61).root) == sparse_first_infill_child_roles());

    // An unconfigured tool index is skipped the same way.
    build_plan(print, {previous, VisitSpec{5, {}}, incoming});
    run_wipe_into_first(print);
    CHECK(bucket_child_roles(*require_extrusion(print, 61).root) == sparse_first_infill_child_roles());
}

// Export two cubes, each printed by its own tool, with wipe_into_infill: after every tool change
// the object prints sparse infill first, and the tower purge and the reader's tower volume are
// smaller than with the option off.
TEST_CASE("Wipe-into-first orders sparse infill first in the exported two-cube print",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const boost::filesystem::path checkout =
        boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    double purge[2] = {0., 0.};
    double tower_volume[2] = {0., 0.};
    size_t changes[2] = {0, 0};
    size_t checked_types[2] = {0, 0};
    std::vector<std::pair<size_t, std::string>> first_object_types[2];
    for (int into = 0; into < 2; ++into) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_num_extruders(2);
        config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
            {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
            {"wiping_volumes_matrix", "0,140,140,0"}, {"filament_minimal_purge_on_wipe_tower", "15,15"},
            {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
            {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
            {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
            {"step_perimeter_plugin", "perimeter.generator.classic"},
            {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
            {"step_wipetower_plugin", "wipe_tower.manager.default"},
            {"layer_height", "0.3"}, {"fill_density", "20%"},
            {"wipe_into_infill", into != 0 ? "1" : "0"}});
        Print print;
        Model model;
        Test::init_print({Test::TestMesh::cube_20x20x20, Test::TestMesh::cube_20x20x20}, print, model, config);
        for (const char *key : {"perimeter_extruder", "infill_extruder", "solid_infill_extruder"})
            model.objects()[1].config.set_deserialize_strict(key, "2");
        print.apply(model, config);
        REQUIRE_NOTHROW(print.process());
        const std::string path = (checkout / "build" / "temp" /
            (into != 0 ? "wipe_into_first_on.gcode" : "wipe_into_first_off.gcode")).string();
        GCodeProcessorResult result;
        REQUIRE_NOTHROW(Orchestrator::instance().export_gcode(print, path, &result));

        extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
        for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
            for (const Printing::PrintingLayerGroup &layer : group.layers)
                for (const Printing::PrintingToolGroup &visit : layer.tool_groups) {
                    if (previous != EXTRUDER_ID_UNKNOWN && previous != visit.extruder_id)
                        ++changes[into];
                    previous = visit.extruder_id;
                    for (const Printing::PrintingExtrusion &extrusion : visit.extrusions)
                        if (extrusion.root && extrusion.sregion_island_role.is_wipetower())
                            purge[into] += raw_role_volume(*extrusion.root,
                                                           uint16_t(RAW_EXTRUSION_ROLE_WIPE_TOWER_WIPE));
                }
        const std::map<GCodeExtrusionRole, std::map<extruder_index_t, double>>::const_iterator wipe =
            result.print_statistics.volumes_per_role_per_extruder.find(GCodeExtrusionRole::WipeTower);
        if (wipe != result.print_statistics.volumes_per_role_per_extruder.end())
            for (const std::pair<const extruder_index_t, double> &entry : wipe->second)
                tower_volume[into] += entry.second;

        // After each tool change, the first object extrusion is infill, and on the sparse layers
        // it is sparse infill; the tower purge printed before it is skipped. The forced solid
        // first and last layers are recorded but not required to be sparse.
        const std::string gcode = Test::GCode::read_text_file(path);
        std::istringstream lines(gcode);
        std::string line;
        size_t layer_changes = 0;
        size_t pending_layer = 0;
        std::string pending_tool;
        while (std::getline(lines, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.find(";LAYER_CHANGE") != std::string::npos)
                ++layer_changes;
            if (layer_changes >= 2 && (line.rfind("T0", 0) == 0 || line.rfind("T1", 0) == 0)) {
                pending_tool = line.substr(0, 2);
                pending_layer = layer_changes - 1;
            }
            if (!pending_tool.empty() && line.rfind(";TYPE:", 0) == 0) {
                const std::string role = line.substr(6);
                if (role == "Wipe tower" || role == "Travel" || role == "Skirt/Brim")
                    continue;
                first_object_types[into].emplace_back(pending_layer, role);
                ++checked_types[into];
                pending_tool.clear();
            }
        }
    }

    REQUIRE(changes[0] > 0);
    CHECK(changes[1] == changes[0]);
    CHECK(purge[1] < purge[0] - 1.);
    CHECK(tower_volume[1] < tower_volume[0] - 1.);
    REQUIRE(checked_types[1] > 0);
    size_t last_layer = 0;
    for (const std::pair<const size_t, std::string> &entry : first_object_types[1])
        last_layer = std::max(last_layer, entry.first);
    size_t sparse_checked = 0;
    for (const std::pair<const size_t, std::string> &entry : first_object_types[1]) {
        // Every tool change starts with infill, never with perimeters or support.
        CHECK((entry.second == "Internal infill" || entry.second == "Solid infill" ||
               entry.second == "Top solid infill" || entry.second == "Bridge infill"));
        if (entry.first >= 1 && entry.first + 1 < last_layer) {
            CHECK(entry.second == "Internal infill");
            ++sparse_checked;
        }
    }
    CHECK(sparse_checked > 0);
}

// Real geometry check for the assumption the plugin relies on: at STEP_ORDERING after the tree
// ordering, the direct children of an infill bucket are single-role subtrees, and a layer that
// has both sparse and solid infill separates them at that level.
TEST_CASE("STEP_ORDERING separates sparse and solid infill at the infill bucket's direct children",
          "[plugins][ordering][wipe-tower][wipe-into]")
{
    const tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism, 1);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_deserialize_strict({{"wipe_tower", "1"}, {"wipe_tower_x", "40"}, {"wipe_tower_y", "60"},
        {"wipe_tower_width", "20"}, {"wipe_tower_rotation_angle", "0"}, {"wipe_tower_brim_width", "0"},
        {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
        {"perimeter_extruder", "1"}, {"infill_extruder", "1"}, {"solid_infill_extruder", "1"},
        {"step_perimeter_plugin", "perimeter.generator.classic"},
        {"step_surface_generation_plugin", "surface.initial_typed_surface_builder"},
        {"step_wipetower_plugin", "wipe_tower.manager.default"},
        {"layer_height", "0.3"}, {"fill_density", "20%"}});
    Print print;
    Model model;
    // A 20x20x5 base with a 12x12 column on top, merged into one shell: the layer that spans
    // the base top has an exposed top ring (solid infill) next to the sparse column interior.
    ModelObject *object = model.add_object();
    object->name = "pedestal";
    TriangleMesh pedestal = make_cube(20., 20., 5.);
    TriangleMesh column = make_cube(12., 12., 12.);
    column.translate(Vec3f(4.f, 4.f, 5.f));
    pedestal.merge(column);
    object->add_volume(std::move(pedestal), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());

    // Observe the plan as the tree ordering leaves it, without the wipe-into plugin.
    Orchestrator &host = Orchestrator::instance();
    Plugin *plugin = host.get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);
    const bool plugin_was_active = host.is_plugin_active(plugin);
    REQUIRE(host.set_plugin_active(plugin, false));
    BOOST_SCOPE_EXIT((&host)(plugin)(plugin_was_active)) {
        host.set_plugin_active(plugin, plugin_was_active);
    } BOOST_SCOPE_EXIT_END
    Steps::StepExtrusionOrdering::run_step(host, print);

    size_t infill_buckets = 0;
    size_t mixed_layers = 0;
    size_t mixed_children = 0;
    for (const Printing::PrintingGroup &group : print.printing_plan()->groups)
        for (const Printing::PrintingLayerGroup &layer : group.layers)
            for (const Printing::PrintingToolGroup &visit : layer.tool_groups)
                for (const Printing::PrintingExtrusion &extrusion : visit.extrusions) {
                    if (!extrusion.root || extrusion.sregion_island_role != ExtrusionRole::InternalInfill)
                        continue;
                    ++infill_buckets;
                    size_t sparse = 0, non_sparse = 0, mixed = 0;
                    classify_infill_children(*extrusion.root, sparse, non_sparse, mixed);
                    mixed_children += mixed;
                    if (sparse > 0 && non_sparse > 0)
                        ++mixed_layers;
                }
    CHECK(infill_buckets > 0);
    CHECK(mixed_layers > 0);
    CHECK(mixed_children == 0);
}
