#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/steps/LayerHeightStep.hpp"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Api/plugin/cpp/LayerHeightLimits.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

/* Exercise standard and flat-area layer-height generation on small owned models.
   Prepared meshes distinguish retained flat-surface anchors from filtered ones;
   descriptor and sliced-layer checks retain first-layer and raft placement.
   Small-nozzle cases compare host limits, plugin validation and explicit profiles
   at the 0.01 mm floor. Helper tests separately cover resolved percentages, disabled
   maxima and caller-owned z-step rounding.

       TEST_CASE -> configuration and owned Model/Print
           |-- run_layer_height_plugin(): publish explicit descriptors
           |-- require_valid_layer_descriptors(): check shape and continuity
           `-- slicing cases: compare descriptors with final object layers
*/

namespace {

using namespace Slic3r;

TriangleMesh make_translated_cube(const double side_mm, const double z_min_mm, const double z_max_mm, const double x_mm)
{
    TriangleMesh cube = make_cube(side_mm, side_mm, z_max_mm - z_min_mm);
    cube.translate(float(x_mm), 0.f, float(z_min_mm));
    return cube;
}

TriangleMesh make_flat_area_test_mesh()
{
    // One object made from several disconnected cubes at different Z ranges.
    //
    // The tiny base cube keeps the object on the bed and is ignored by the
    // plugin because it is below the fixed first layer. The two large cubes
    // have horizontal face areas above 50 mm2; the two small cubes are below
    // that threshold. This lets the test verify both "all flat areas are
    // considered" and "the min flat area setting removes only the small ones".
    TriangleMesh mesh = make_translated_cube(1., 0., 0.1, -30.);
    mesh.merge(make_translated_cube(10., 2.3, 2.6, -15.));
    mesh.merge(make_translated_cube(3., 4.15, 4.45, 0.));
    mesh.merge(make_translated_cube(8., 6.1, 6.4, 15.));
    mesh.merge(make_translated_cube(2., 8.05, 8.35, 30.));
    return mesh;
}

TriangleMesh make_no_internal_flat_area_test_mesh()
{
    // Same total object height as make_flat_area_test_mesh(), but as one
    // continuous cube. Its only horizontal polygons are the bed face and the
    // top face; both are intentionally ignored by the plugin. Therefore none
    // of the intermediate Z anchors from make_flat_area_test_mesh() should be
    // present in the generated explicit layer Z list.
    return make_cube(10., 10., 8.35);
}

DynamicPrintConfig flat_area_config(const double min_flat_area_mm2)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"first_layer_height", "0.5"},
        {"layer_height", "2"},
        {"layer_height_min_flat_area", std::to_string(min_flat_area_mm2)},
        {"max_layer_height", "2"},
        {"min_layer_height", "0.05"},
        {"nozzle_diameter", "2"},
        {"perimeters", "1"},
        {"z_step", "0"}
    });
    return config;
}

DynamicPrintConfig flat_area_slicing_config(const int raft_layers)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"first_layer_height", "0.5"},
        {"layer_height", "1"},
        {"layer_height_min_flat_area", "9999"},
        {"max_layer_height", "1"},
        {"min_layer_height", "0.05"},
        {"nozzle_diameter", "1.5"},
        {"perimeters", "1"},
        {"raft_layers", std::to_string(raft_layers)},
        {"step_layer_height_plugin", "flat_area_layer_height"},
        {"z_step", "0"}
    });
    return config;
}

struct LayerPlanResult
{
    std::vector<coord_t> descriptors;
    std::vector<double> bottom_zs;
    std::vector<double> print_zs;
    std::vector<double> heights;
    std::vector<size_t> layer_ids;
    bool has_raft = false;
};

void require_valid_layer_descriptors(const std::vector<coord_t> &profile);
// A 0.05 mm nozzle printing 0.025 mm layers above a 0.04 mm first layer, with a 0.04 mm maximum,
// the given min_layer_height and the given layer-height plugin.
DynamicPrintConfig small_nozzle_config(const std::string &min_layer_height, const char *plugin_id);
// Heights in mm of the layers above the first one, read from explicit [layer_top_z, layer_height] pairs.
std::vector<double> heights_above_first_layer(const std::vector<coord_t> &profile);

void run_layer_height_plugin(Print &print, const char *plugin_id)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(plugin_id);
    REQUIRE(plugin != nullptr);

    plugin_host_context host_context = orchestrator.prepare_plugin_host_context(STEP_LAYER_HEIGHT, plugin, &print);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_LAYER_HEIGHT, plugin, &host_context);
    plugin->setup(run_context, 1);

    std::unique_ptr<ApiHost::Steps::LayerHeightRunContext> layer_context =
        ApiHost::Steps::make_layer_height_run_context(print, 0);
    run_context.data = &layer_context->context_step;
    plugin->setup_run(run_context);
    plugin->run(run_context);
}

void run_flat_area_layer_height_plugin(Print &print)
{
    run_layer_height_plugin(print, "flat_area_layer_height");
}

std::vector<coord_t> run_flat_area_layer_profile(const TriangleMesh &mesh, const double min_flat_area_mm2)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    Slic3r::Test::init_print({mesh}, print, model, flat_area_config(min_flat_area_mm2));
    run_flat_area_layer_height_plugin(print);
    return print.object(0).layer_profile();
}

LayerPlanResult run_layer_height_and_slicing(const char *plugin_id, const int raft_layers)
{
    // Exercise the real step boundary: a layer-height plugin writes explicit
    // descriptors through the host callback, then STEP_SLICING turns those
    // descriptors into Layer objects and slices the mesh at their midpoints.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    Slic3r::Test::init_print({make_cube(10., 10., 3.)},
                             print,
                             model,
                             flat_area_slicing_config(raft_layers));

    Orchestrator &orchestrator = Orchestrator::instance();
    run_layer_height_plugin(print, plugin_id);
    require_valid_layer_descriptors(print.object(0).layer_profile());
    Steps::StepSlicing::run_step(orchestrator, print);

    PrintObject &object = print.object(0);
    LayerPlanResult out;
    out.descriptors = object.layer_profile();
    out.has_raft = object.has_raft();
    out.bottom_zs.reserve(object.layer_count());
    out.print_zs.reserve(object.layer_count());
    out.heights.reserve(object.layer_count());
    out.layer_ids.reserve(object.layer_count());
    for (Layer &layer : object.layers()) {
        out.bottom_zs.push_back(unscaled(layer.scaled_bottom_z()));
        out.print_zs.push_back(layer.unscaled_print_z());
        out.heights.push_back(layer.unscaled_height());
        out.layer_ids.push_back(layer.id());
    }
    return out;
}

LayerPlanResult run_flat_area_layer_height_and_slicing(const int raft_layers)
{
    return run_layer_height_and_slicing("flat_area_layer_height", raft_layers);
}

void require_valid_layer_descriptors(const std::vector<coord_t> &profile)
{
    // STEP_LAYER_HEIGHT stores explicit pairs: [layer_top_z, layer_height].
    // This validator documents the contract consumed by STEP_SLICING: positive
    // heights, sorted top Zs, and no overlap between consecutive layer
    // intervals. Gaps are allowed and are tested separately below.
    REQUIRE_FALSE(profile.empty());
    REQUIRE((profile.size() % 2) == 0);

    coord_t previous_hi = 0;
    for (size_t idx = 0; idx + 1 < profile.size(); idx += 2) {
        const coord_t hi = profile[idx];
        const coord_t height = profile[idx + 1];
        const coord_t lo = hi - height;
        REQUIRE(height > 0);
        REQUIRE(lo >= previous_hi);
        REQUIRE(hi > lo);
        previous_hi = hi;
    }
}

std::vector<coord_t> layer_top_zs(const std::vector<coord_t> &profile)
{
    std::vector<coord_t> zs;
    zs.reserve(profile.size() / 2);
    for (size_t idx = 0; idx + 1 < profile.size(); idx += 2)
        zs.push_back(profile[idx]);
    return zs;
}

bool contains_z(const std::vector<coord_t> &zs, const double z_mm)
{
    const coord_t expected = scale_i(z_mm);
    const coord_t tolerance = scale_i(0.005);
    for (coord_t z : zs)
        if (std::abs(z - expected) <= tolerance)
            return true;
    return false;
}

DynamicPrintConfig small_nozzle_config(const std::string &min_layer_height, const char *plugin_id)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"first_layer_height", "0.04"},
        {"layer_height", "0.025"},
        {"layer_height_min_flat_area", "9999"},
        {"max_layer_height", "0.04"},
        {"min_layer_height", min_layer_height},
        {"nozzle_diameter", "0.05"},
        {"perimeters", "1"},
        {"step_layer_height_plugin", plugin_id},
        {"z_step", "0"}
    });
    return config;
}

std::vector<double> heights_above_first_layer(const std::vector<coord_t> &profile)
{
    std::vector<double> heights;
    for (size_t idx = 3; idx < profile.size(); idx += 2)
        heights.push_back(unscaled(profile[idx]));
    return heights;
}

} // namespace

TEST_CASE("FlatAreaLayerHeight filters small flat cube surfaces", "[plugins][flat-area-layer-height]")
{
    // With a zero threshold, the plugin should expose layer-height anchors on
    // the horizontal faces of every cube above the first layer. Raising the
    // threshold to 50 mm2 should keep the 10x10 and 8x8 cube faces, while
    // removing the 3x3 and 2x2 cube faces.
    const TriangleMesh mesh = make_flat_area_test_mesh();

    const std::vector<coord_t> all_flat_area_profile = run_flat_area_layer_profile(mesh, 0.);
    require_valid_layer_descriptors(all_flat_area_profile);
    const std::vector<coord_t> all_flat_area_zs = layer_top_zs(all_flat_area_profile);
    REQUIRE(contains_z(all_flat_area_zs, 2.3));
    REQUIRE(contains_z(all_flat_area_zs, 2.6));
    REQUIRE(contains_z(all_flat_area_zs, 4.15));
    REQUIRE(contains_z(all_flat_area_zs, 4.45));
    REQUIRE(contains_z(all_flat_area_zs, 6.1));
    REQUIRE(contains_z(all_flat_area_zs, 6.4));
    REQUIRE(contains_z(all_flat_area_zs, 8.05));

    const std::vector<coord_t> filtered_flat_area_profile = run_flat_area_layer_profile(mesh, 50.);
    require_valid_layer_descriptors(filtered_flat_area_profile);
    const std::vector<coord_t> filtered_flat_area_zs = layer_top_zs(filtered_flat_area_profile);
    REQUIRE(contains_z(filtered_flat_area_zs, 2.3));
    REQUIRE(contains_z(filtered_flat_area_zs, 2.6));
    REQUIRE_FALSE(contains_z(filtered_flat_area_zs, 4.15));
    REQUIRE_FALSE(contains_z(filtered_flat_area_zs, 4.45));
    REQUIRE(contains_z(filtered_flat_area_zs, 6.1));
    REQUIRE(contains_z(filtered_flat_area_zs, 6.4));
    REQUIRE_FALSE(contains_z(filtered_flat_area_zs, 8.05));
}

TEST_CASE("FlatAreaLayerHeight observes cancellation before external provider work",
          "[plugins][flat-area-layer-height][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    Slic3r::Test::init_print(
        {make_flat_area_test_mesh()}, print, model, flat_area_config(0.));

    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("flat_area_layer_height");
    REQUIRE(plugin != nullptr);

    plugin_host_context host_context = orchestrator.prepare_plugin_host_context(
        STEP_LAYER_HEIGHT, plugin, &print);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(
        STEP_LAYER_HEIGHT, plugin, &host_context);
    plugin->setup(run_context, 1);

    std::unique_ptr<ApiHost::Steps::LayerHeightRunContext> layer_context =
        ApiHost::Steps::make_layer_height_run_context(print, 0);
    run_context.data = &layer_context->context_step;
    plugin->setup_run(run_context);
    const std::vector<coord_t> profile_before = print.object(0).layer_profile();

    orchestrator.request_plugin_cancel();
    plugin->run(run_context);
    orchestrator.reset_plugin_cancel();

    CHECK(print.object(0).layer_profile() == profile_before);
}

TEST_CASE("Plugin option definitions default invalidation to plugin step", "[plugins][flat-area-layer-height][config]")
{
    // FlatAreaLayerHeight intentionally leaves raw_config_option_def::invalidates_step
    // at the raw initializer default. The orchestrator resolves that missing
    // value while the plugin initializes, so the stored ConfigOptionDef should
    // invalidate at STEP_LAYER_HEIGHT without every plugin setting having to
    // repeat its own step.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *def = PrintConfigDef::instance().get("layer_height_min_flat_area");
    REQUIRE(def != nullptr);
    CHECK(def->invalidates_step == STEP_LAYER_HEIGHT);
}

TEST_CASE("FlatAreaLayerHeight does not add flat-area anchors without internal flat polygons", "[plugins][flat-area-layer-height]")
{
    // This contrasts the multi-cube mesh with a single continuous cube. The
    // regular layer-height generator may still create intermediate layers, but
    // it should not create the exact Z anchors produced by horizontal cube
    // faces when no such internal flat polygons exist.
    const TriangleMesh no_flat_polygons_mesh = make_no_internal_flat_area_test_mesh();
    const std::vector<coord_t> profile_without_flat_polygons =
        run_flat_area_layer_profile(no_flat_polygons_mesh, 0.);
    require_valid_layer_descriptors(profile_without_flat_polygons);
    const std::vector<coord_t> z_without_flat_polygons = layer_top_zs(profile_without_flat_polygons);

    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 2.3));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 2.6));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 4.15));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 4.45));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 6.1));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 6.4));
    REQUIRE_FALSE(contains_z(z_without_flat_polygons, 8.05));
}

TEST_CASE("StepLayerHeightGeneration validates explicit layer descriptors", "[plugins][layer-height]")
{
    // This is the contract checked immediately after a layer-height plugin
    // runs, before STEP_SLICING consumes the result. The validator accepts
    // sorted non-overlapping intervals, including a deliberate gap, and rejects
    // malformed payloads while the owning plugin is still the likely culprit.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    DynamicPrintConfig config = flat_area_config(0.);
    Slic3r::Test::init_print({make_cube(10., 10., 3.)}, print, model, config);

    std::vector<coord_t> valid_descriptors;
    valid_descriptors.push_back(scale_i(0.4));
    valid_descriptors.push_back(scale_i(0.4));
    valid_descriptors.push_back(scale_i(2.0));
    valid_descriptors.push_back(scale_i(0.4));
    ApiInternal::PrintObjectAccess::set_layer_profile(print.object(0), std::move(valid_descriptors));

    std::string error;
    CHECK(Steps::StepLayerHeightGeneration::validate_post(print, error));

    std::vector<coord_t> odd_descriptors;
    odd_descriptors.push_back(scale_i(0.4));
    odd_descriptors.push_back(scale_i(0.4));
    odd_descriptors.push_back(scale_i(2.0));
    ApiInternal::PrintObjectAccess::set_layer_profile(print.object(0), std::move(odd_descriptors));

    error.clear();
    CHECK_FALSE(Steps::StepLayerHeightGeneration::validate_post(print, error));
    CHECK(error.find("odd descriptor count") != std::string::npos);
}

TEST_CASE("StepSlicing consumes explicit layer descriptors with vertical gaps", "[plugins][layer-height][slicing]")
{
    // The layer-height step no longer sends the historical compact
    // [z, height_at_z] profile to slicing. It sends explicit intervals encoded
    // as [top_z, height]. This test creates two valid layers with an empty
    // vertical interval between them. If STEP_SLICING accidentally converted
    // the payload back as a compact profile, the second layer would start at
    // the previous layer top instead of preserving the deliberate gap.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    DynamicPrintConfig config = flat_area_config(0.);
    Slic3r::Test::init_print({make_cube(10., 10., 3.)}, print, model, config);

    std::vector<coord_t> layer_descriptors;
    layer_descriptors.push_back(scale_i(0.4));
    layer_descriptors.push_back(scale_i(0.4));
    layer_descriptors.push_back(scale_i(2.0));
    layer_descriptors.push_back(scale_i(0.4));
    ApiInternal::PrintObjectAccess::set_layer_profile(print.object(0), std::move(layer_descriptors));

    std::string error;
    REQUIRE(Steps::StepSlicing::validate_pre(print, error));
    Steps::StepSlicing::clear(print);
    Steps::StepSlicing::prepare(print);

    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() == 2);
    CHECK(object.layer(0).unscaled_print_z() == Approx(0.4));
    CHECK(object.layer(0).unscaled_height() == Approx(0.4));
    CHECK(unscaled(object.layer(0).scaled_bottom_z()) == Approx(0.0));
    CHECK(object.layer(1).unscaled_print_z() == Approx(2.0));
    CHECK(object.layer(1).unscaled_height() == Approx(0.4));
    CHECK(unscaled(object.layer(1).scaled_bottom_z()) == Approx(1.6));
}

TEST_CASE("StepSlicing rejects overlapping explicit layer descriptors", "[plugins][layer-height][slicing]")
{
    // Gaps are intentional and valid, but overlapping layer intervals would
    // make slicing ambiguous: two layers would claim the same vertical span of
    // the object. The pre-step validator catches this before new layers are
    // constructed.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    DynamicPrintConfig config = flat_area_config(0.);
    Slic3r::Test::init_print({make_cube(10., 10., 3.)}, print, model, config);

    std::vector<coord_t> overlapping_descriptors;
    overlapping_descriptors.push_back(scale_i(0.6));
    overlapping_descriptors.push_back(scale_i(0.6));
    overlapping_descriptors.push_back(scale_i(1.0));
    overlapping_descriptors.push_back(scale_i(0.6));
    ApiInternal::PrintObjectAccess::set_layer_profile(print.object(0), std::move(overlapping_descriptors));

    std::string error;
    CHECK_FALSE(Steps::StepSlicing::validate_pre(print, error));
    CHECK(error.find("overlaps the previous layer") != std::string::npos);
}

TEST_CASE("Layer height and slicing keep object layers independent from raft", "[plugins][layer-height][slicing]")
{
    // Run the same cube through STEP_LAYER_HEIGHT and STEP_SLICING with and
    // without raft. The raft setting may affect later raft/support generation,
    // but it must not affect the object's own layer descriptors, layer ids, or
    // object-local Zs.
    const char *plugin_ids[] = { "flat_area_layer_height", "standard_layer_height_generator" };
    for (const char *plugin_id : plugin_ids) {
        INFO(plugin_id);
        const LayerPlanResult no_raft = run_layer_height_and_slicing(plugin_id, 0);
        const LayerPlanResult with_raft = run_layer_height_and_slicing(plugin_id, 2);

        REQUIRE_FALSE(no_raft.has_raft);
        REQUIRE(with_raft.has_raft);

        REQUIRE(no_raft.descriptors == with_raft.descriptors);
        REQUIRE(no_raft.layer_ids == with_raft.layer_ids);
        REQUIRE(no_raft.bottom_zs.size() == with_raft.bottom_zs.size());
        REQUIRE(no_raft.print_zs.size() == with_raft.print_zs.size());
        REQUIRE(no_raft.heights.size() == with_raft.heights.size());

        REQUIRE(no_raft.print_zs.size() >= 2);
        CHECK(no_raft.layer_ids.front() == 0);
        CHECK(unscaled(no_raft.descriptors[0]) == Approx(0.5));
        CHECK(unscaled(no_raft.descriptors[1]) == Approx(0.5));

        for (size_t idx = 0; idx < no_raft.print_zs.size(); ++idx) {
            CHECK(with_raft.bottom_zs[idx] == Approx(no_raft.bottom_zs[idx]));
            CHECK(with_raft.print_zs[idx] == Approx(no_raft.print_zs[idx]));
            CHECK(with_raft.heights[idx] == Approx(no_raft.heights[idx]));
        }

        CHECK(no_raft.bottom_zs[0] == Approx(0.0));
        CHECK(no_raft.print_zs[0] == Approx(0.5));
        CHECK(no_raft.heights[0] == Approx(0.5));
        CHECK(no_raft.bottom_zs[1] == Approx(0.5));
        CHECK(no_raft.bottom_zs.back() < no_raft.print_zs.back());
    }
}

TEST_CASE("Layer height limits apply the floor and the unset default", "[plugins][layer-height]")
{
    // The SDK helpers alone: 0 takes an eighth of the nozzle, any value below the floor is raised, other values are kept;
    // a zero or disabled maximum falls back to 75% of the nozzle.
    CHECK(slic3r_api::min_layer_height_mm(0., 0.4) == Approx(0.05));
    CHECK(slic3r_api::min_layer_height_mm(0., 0.05) == slic3r_api::k_min_layer_height_floor_mm);
    CHECK(slic3r_api::min_layer_height_mm(0.005, 0.4) == slic3r_api::k_min_layer_height_floor_mm);
    CHECK(slic3r_api::min_layer_height_mm(0.03, 0.05) == 0.03);
    CHECK(slic3r_api::max_layer_height_mm(0., true, 0.05) == Approx(0.0375));
    CHECK(slic3r_api::max_layer_height_mm(0.04, false, 0.05) == Approx(0.0375));
    CHECK(slic3r_api::max_layer_height_mm(0.04, true, 0.05) == 0.04);
}

TEST_CASE("Layer height generators export a linear small-nozzle STL matrix", "[plugins][layer-height][plugin-e2e][layer-height-e2e]")
{
    namespace E2E = Slic3r::Test::PluginE2E;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    const std::vector<E2E::Domain> domains = {
        {{"min_layer_height"}, {{"floor", {{"min_layer_height", "0.0005"}}},
            {"percent", {{"min_layer_height", "10%"}}}}},
        {{"max_layer_height"}, {{"automatic-max", {{"max_layer_height", "0"}}},
            {"disabled-max", {{"max_layer_height", "!0.04"}}}}},
        {{"z_step"}, {{"rounded-step", {{"z_step", "0.005"}}}}},
        {{"layer_height_min_flat_area"}, {{"all-flat-areas", {{"layer_height_min_flat_area", "0"}}}}}
    };
    TriangleMesh mesh = E2E::stl_fixture(false);
    mesh.scale(Vec3f(0.25f, 0.25f, 0.5f));
    for (const char *plugin_id : {"standard_layer_height_generator", "flat_area_layer_height"}) {
        DynamicPrintConfig baseline = small_nozzle_config("0", plugin_id);
        baseline.set_deserialize_strict({{"skirts", "0"}, {"brim_width", "0"},
            {"fill_density", "0%"}, {"top_solid_layers", "0"}, {"bottom_solid_layers", "0"},
            {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
            {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
        const std::vector<E2E::Case> cases = E2E::linear_cases(host, {plugin_id}, baseline, domains);
        for (const E2E::Case &test : cases) {
            CAPTURE(plugin_id, test.name);
            Model model;
            Print print;
            Slic3r::Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
            print.process();
            const boost::filesystem::path path = Slic3r::Test::GCode::temporary_gcode_path();
            std::string code;
            try {
                code = Slic3r::Test::GCode::read_text_file(host.export_gcode_cli(print, path.string()));
                Slic3r::Test::GCode::remove_output_pair(path);
            } catch (...) {
                Slic3r::Test::GCode::remove_output_pair(path);
                throw;
            }
            REQUIRE(print.objects().size() == 1);
            REQUIRE(print.object(0).layer_count() > 2);
            for (size_t index = 1; index < print.object(0).layer_count(); ++index) {
                CHECK(print.object(0).layer(index).scaled_height() > 0);
                CHECK(unscaled(print.object(0).layer(index).scaled_height()) <= 0.04 + 1e-5);
            }
            size_t extruding = 0;
            GCodeReader reader;
            reader.parse_buffer(code, [&extruding](GCodeReader &state, const GCodeReader::GCodeLine &line) {
                if (line.extruding(state) && line.dist_XY(state) > 0.)
                    ++extruding;
            });
            CHECK(extruding > 0);
        }
    }
}

TEST_CASE("Resolved layer height limits retain percentages disabled maxima and z steps", "[plugins][layer-height]")
{
    struct LimitsCase {
        const char *minimum;
        const char *maximum;
        const char *step;
        double expected_minimum;
        double expected_maximum;
    };
    const LimitsCase cases[] = {
        {"0", "0", "0", 0.01, 0.0375},
        {"10%", "75%", "0", 0.01, 0.0375},
        {"0.0005", "!0.04", "0", 0.01, 0.0375},
        {"10%", "0", "0.001", 0.01, 0.038},
        {"0.03", "0.02", "0", 0.03, 0.03}
    };
    for (const LimitsCase &test : cases) {
        CAPTURE(test.minimum, test.maximum, test.step);
        DynamicPrintConfig config = small_nozzle_config(test.minimum, "standard_layer_height_generator");
        config.set_deserialize_strict({{"max_layer_height", test.maximum}, {"z_step", test.step}});
        CHECK(Slicing::min_layer_height_from_nozzle(config, 0) == Approx(test.expected_minimum));
        CHECK(Slicing::max_layer_height_from_nozzle(config, 0) == Approx(test.expected_maximum));
    }
}

TEST_CASE("Small nozzles get the same layer height limits on the host and plugin paths", "[plugins][layer-height]")
{
    // A 0.05 mm nozzle with an unset, small or too small min_layer_height: the host slicing parameters and both layer-height generators keep the 0.025 mm layer height, and the layer-height validation, which compares them in Debug builds, accepts the print.
    const std::pair<const char *, double> minimums[] = {{"0", 0.01}, {"0.02", 0.02}, {"0.0005", 0.01}};
    const char *plugin_ids[] = {"standard_layer_height_generator", "flat_area_layer_height"};
    for (const std::pair<const char *, double> &minimum : minimums) {
        for (const char *plugin_id : plugin_ids) {
            INFO("min_layer_height " << minimum.first << ", " << plugin_id);
            Model model;
            Print print;
            Slic3r::Test::init_print({make_cube(2., 2., 0.29)}, print, model, small_nozzle_config(minimum.first, plugin_id));

            const SlicingParameters &native = print.object(0).slicing_parameters();
            CHECK(native.min_layer_height == Approx(minimum.second));
            CHECK(native.layer_height == Approx(0.025));

            std::string error;
            const bool valid = Steps::StepLayerHeightGeneration::validate_pre(print, error);
            INFO(error);
            CHECK(valid);

            run_layer_height_plugin(print, plugin_id);
            const std::vector<coord_t> &profile = print.object(0).layer_profile();
            require_valid_layer_descriptors(profile);
            CHECK(unscaled(profile[1]) == Approx(0.04));
            const std::vector<double> heights = heights_above_first_layer(profile);
            CHECK(heights.size() == 10);
            // Layer tops sit on the integer coordinate grid, so a height may differ from 0.025 by a few nanometres.
            for (const double height : heights)
                CHECK(height == Approx(0.025).margin(1e-5));
        }
    }
}

TEST_CASE("A variable layer height profile at the floor reaches the layer descriptors", "[plugins][layer-height]")
{
    // A variable layer height profile of 0.01 mm layers, at the floor, reaches the layer descriptors unchanged.
    Model model;
    Print print;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(2., 2., 0.29));
    object->layer_height_profile.set(std::vector<coordf_t>{0., 0.01, 0.29, 0.01});
    object->add_instance();
    model.center_instances_around_point({100, 100});
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, small_nozzle_config("0", "standard_layer_height_generator"));

    run_layer_height_plugin(print, "standard_layer_height_generator");
    const std::vector<coord_t> &profile = print.object(0).layer_profile();
    require_valid_layer_descriptors(profile);
    CHECK(unscaled(profile[1]) == Approx(0.04));
    const std::vector<double> heights = heights_above_first_layer(profile);
    CHECK(heights.size() == 25);
    // Layer tops sit on the integer coordinate grid, so a height may differ from 0.01 by a few nanometres.
    for (const double height : heights)
        CHECK(height == Approx(0.01).margin(1e-5));
}
