#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/steps/SupportDemandStep.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/Steps/StepSupportDemand.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace Slic3r;

TriangleMesh make_sloped_cube(const double top_x_expansion)
{
    // Build a simple 20 x 20 x 10 mm closed prism.
    //
    // The bottom footprint is a square from X=-10 to X=10. The top footprint
    // keeps the left side vertical and expands only the right side in +X by
    // top_x_expansion. With the 1 mm layer height used below, the unsupported
    // horizontal growth between two adjacent layers is approximately
    // top_x_expansion / 10.
    //
    // This gives the tests a predictable one-sided overhang without involving
    // bridges, holes, modifiers, or multiple islands.
    const float height = 10.f;
    const float half_size = 10.f;
    const float top_right_x = half_size + float(top_x_expansion);
    std::vector<Vec3f> vertices = {
        {-half_size, -half_size, 0.f},
        { half_size, -half_size, 0.f},
        { half_size,  half_size, 0.f},
        {-half_size,  half_size, 0.f},
        {-half_size, -half_size, height},
        { top_right_x, -half_size, height},
        { top_right_x,  half_size, height},
        {-half_size,  half_size, height}
    };
    std::vector<Vec3i32> faces = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6},
        {3, 0, 4}, {3, 4, 7}
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

TriangleMesh make_sloped_cube_with_surface_angle(const double degrees)
{
    // support_material_threshold is expressed as the maximum printable surface
    // angle used by the support detector. For our one-sided prism, the matching
    // horizontal top expansion is height / tan(angle).
    const double height = 10.;
    const double radians = degrees * PI / 180.;
    return make_sloped_cube(height / std::tan(radians));
}

DynamicPrintConfig support_demand_config(const bool support_material,
                                         const bool support_material_auto,
                                         const int support_material_threshold,
                                         const int support_material_enforce_layers)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "1"},
        {"first_layer_height", "1"},
        {"nozzle_diameter", "0.4"},
        {"support_material", support_material ? "1" : "0"},
        {"support_material_auto", support_material_auto ? "1" : "0"},
        {"support_material_threshold", std::to_string(support_material_threshold)},
        {"support_material_enforce_layers", std::to_string(support_material_enforce_layers)}
    });
    return config;
}

struct DemandSummary
{
    uint32_t entry_count = 0;
    size_t polygon_count = 0;
    double area_mm2 = 0.;
};

// Sum the support demand stored in the object's islands; entry_count counts the
// islands that ask for support.
DemandSummary summarize_support_demand(const PrintObject &object)
{
    DemandSummary summary;
    for (const Layer &layer : object.layers())
        for (const LayerSliceIsland &island : layer.islands()) {
            const ExPolygons &polygons = island.support_demand();
            if (polygons.empty())
                continue;
            ++summary.entry_count;
            summary.polygon_count += polygons.size();
            summary.area_mm2 += unscaled(unscaled(area(polygons)));
        }
    return summary;
}

void run_until_support_demand_input(Orchestrator &orchestrator, Print &print)
{
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
}

DemandSummary run_support_demand(const TriangleMesh &mesh, const DynamicPrintConfig &config)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    Slic3r::Test::init_print({mesh}, print, model, config);

    Orchestrator &orchestrator = Orchestrator::instance();
    run_until_support_demand_input(orchestrator, print);

    Steps::StepSupportDemand::run_step(orchestrator, print);
    return summarize_support_demand(print.object(0));
}

bool has_meaningful_demand(const DemandSummary &summary)
{
    return summary.entry_count > 0 && summary.polygon_count > 0 && summary.area_mm2 > 0.1;
}

} // namespace

TEST_CASE("SupportDemandOverhangs is gated by support_material", "[plugins][support-demand]")
{
    // A 5 mm top expansion creates about 0.5 mm of new horizontal footprint
    // per layer. With threshold=0, SupportDemandOverhangs falls back to half
    // the external perimeter width, so this geometry should produce support
    // demand if and only if support material itself is enabled.
    const TriangleMesh steep_slope = make_sloped_cube(5.);

    const DemandSummary disabled = run_support_demand(
        steep_slope, support_demand_config(false, true, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(disabled));

    const DemandSummary enabled = run_support_demand(
        steep_slope, support_demand_config(true, true, 0, 0));
    REQUIRE(has_meaningful_demand(enabled));
}

TEST_CASE("SupportDemandOverhangs is gated by support_material_auto", "[plugins][support-demand]")
{
    // Same clearly unsupported slope as above, but this time support material
    // is enabled. The plugin should still stay silent when automatic support
    // generation is disabled and no enforced support layers are requested.
    const TriangleMesh steep_slope = make_sloped_cube(5.);

    const DemandSummary automatic_disabled = run_support_demand(
        steep_slope, support_demand_config(true, false, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(automatic_disabled));

    const DemandSummary automatic_enabled = run_support_demand(
        steep_slope, support_demand_config(true, true, 0, 0));
    REQUIRE(has_meaningful_demand(automatic_enabled));
}

TEST_CASE("SupportDemandOverhangs uses support_material_threshold as an angle limit", "[plugins][support-demand]")
{
    // This shape grows by about 0.5 mm per 1 mm layer. A 45 degree threshold
    // accepts roughly 0.97 mm of horizontal growth per layer, so no demand is
    // expected. An 80 degree threshold accepts only about 0.16 mm, so the same
    // geometry must now be detected as unsupported.
    const TriangleMesh medium_slope = make_sloped_cube(5.);

    const DemandSummary permissive_angle = run_support_demand(
        medium_slope, support_demand_config(true, true, 45, 0));
    REQUIRE_FALSE(has_meaningful_demand(permissive_angle));

    const DemandSummary strict_angle = run_support_demand(
        medium_slope, support_demand_config(true, true, 80, 0));
    REQUIRE(has_meaningful_demand(strict_angle));
}

TEST_CASE("SupportDemandOverhangs switches around the slope angle", "[plugins][support-demand]")
{
    // This prism has a 60 degree sloped side. The implementation adds 1 degree
    // to support_material_threshold to make the UI value inclusive, so config
    // values 58 and 60 exercise effective thresholds of 59 and 61 degrees.
    // Just below the shape angle, the layer below is expanded enough and no
    // support demand should appear. Just above it, the same slope is considered
    // unsupported.
    const TriangleMesh sixty_degree_slope = make_sloped_cube_with_surface_angle(60.);
    
    const DemandSummary just_below_slope = run_support_demand(
        sixty_degree_slope, support_demand_config(true, true, 58, 0));
    REQUIRE_FALSE(has_meaningful_demand(just_below_slope));

    const DemandSummary just_at_slope = run_support_demand(
        sixty_degree_slope, support_demand_config(true, true, 59, 0));
    REQUIRE_FALSE(has_meaningful_demand(just_at_slope));

    const DemandSummary just_above_slope = run_support_demand(
        sixty_degree_slope, support_demand_config(true, true, 60, 0));
    REQUIRE(has_meaningful_demand(just_above_slope));
}

TEST_CASE("SupportDemandOverhangs uses support_material_enforce_layers for early layers", "[plugins][support-demand]")
{
    // A 1 mm top expansion grows by about 0.1 mm per layer, intentionally below
    // the default half-width fallback, so automatic overhang detection should
    // not create demand. Enforcing the first two support layers should still
    // mark the first non-bottom layer as support demand, even with auto support
    // disabled.
    const TriangleMesh shallow_slope = make_sloped_cube(1.);

    const DemandSummary no_enforced_layers = run_support_demand(
        shallow_slope, support_demand_config(true, true, 0, 0));
    REQUIRE_FALSE(has_meaningful_demand(no_enforced_layers));

    const DemandSummary enforced_layer = run_support_demand(
        shallow_slope, support_demand_config(true, false, 0, 2));
    REQUIRE(has_meaningful_demand(enforced_layer));
}

TEST_CASE("Support demand is island data, readable later and emptied by the step cleanup", "[plugins][support-demand]")
{
    // STEP_SUPPORT_DEMAND now runs right after slicing and stores its result in
    // the layer islands, so any later step can read it through the data tree.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Slic3r::Test::init_print({make_sloped_cube(5.)}, print, model, support_demand_config(true, true, 0, 0));
    Orchestrator &orchestrator = Orchestrator::instance();
    run_until_support_demand_input(orchestrator, print);
    REQUIRE_FALSE(has_meaningful_demand(summarize_support_demand(print.object(0))));

    Steps::StepSupportDemand::run_step(orchestrator, print);
    REQUIRE(has_meaningful_demand(summarize_support_demand(print.object(0))));

    // The public getter borrows the same island-owned collection.
    for (const Layer &layer : print.object(0).layers())
        for (const LayerSliceIsland &island : layer.islands()) {
            const expolygon_collection_handle *demand =
                layer_island_get_support_demand(reinterpret_cast<const layer_island_handle *>(&island));
            CHECK(reinterpret_cast<const ExPolygons *>(demand) == &island.support_demand());
        }

    // A replay from Checkpoint::Slicing clears the step outputs before rerunning it.
    Steps::StepSupportDemand::clear(print);
    CHECK(summarize_support_demand(print.object(0)).entry_count == 0);
}

TEST_CASE("Support demand edits are allowed from STEP_SUPPORT_DEMAND through STEP_SUPPORT", "[plugins][support-demand]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Model model;
    Print print;
    Slic3r::Test::init_print({make_sloped_cube(5.)}, print, model, support_demand_config(true, true, 0, 0));
    run_until_support_demand_input(Orchestrator::instance(), print);
    const LayerSliceIsland &first_island = *print.object(0).layer(1).islands().begin();
    const layer_island_handle *island = reinterpret_cast<const layer_island_handle *>(&first_island);

    // A minimal run context: the host only needs the Print (for its running
    // step) and the error callback.
    struct Errors { int count = 0; } errors;
    plugin_host_context host_context;
    host_context.print = &print;
    plugin_run_context run_ctx = {};
    run_ctx.host_context = &host_context;
    static Errors *s_errors = nullptr;
    s_errors = &errors;
    run_ctx.report_error = [](plugin_host_context *, const char *) { ++s_errors->count; };

    // Outside a pipeline run there is no restriction, like other data-tree edits.
    CHECK(support_demand_get_island_mutable(&run_ctx, island) != nullptr);
    // Before the demand is built, or after support generation, it is refused.
    print.set_running_step(STEP_POST_SLICING);
    CHECK(support_demand_get_island_mutable(&run_ctx, island) == nullptr);
    print.set_running_step(STEP_SKIRT_BRIM);
    CHECK(support_demand_get_island_mutable(&run_ctx, island) == nullptr);
    CHECK(errors.count == 2);

    // Every step of the window may edit it: the demand step, a step after the
    // perimeters, and support generation itself.
    for (const slicing_step_t step : {STEP_SUPPORT_DEMAND, STEP_POST_PERIMETER, STEP_INFILL, STEP_SUPPORT}) {
        print.set_running_step(step);
        expolygon_collection_handle *demand = support_demand_get_island_mutable(&run_ctx, island);
        CHECK(reinterpret_cast<const ExPolygons *>(demand) == &first_island.support_demand());
    }
    print.set_running_step(STEP_NONE);
    CHECK(errors.count == 2);
}
