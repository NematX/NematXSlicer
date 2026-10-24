#include <catch2/catch.hpp>

#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "test_data.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"

/*
These tests drive XY compensation through the host's slicing and post-slicing
steps. Raw slices distinguish one application from duplicate compensation;
regional and multiobject fixtures check publication and independent settings.
The STL export matrix also runs the normal full pipeline on a perforated frame.

    TEST_CASE
    |-- run_raw_slicing(): generate layers without dimensional offsets
    |-- StepPostSlicing::run_step(): publish compensated regional geometry
    `-- Print::process() / export_gcode_cli(): slice an STL and write G-code

ScopedXYPlugins restores activation after focused step tests. The export matrix
uses the normal initialized plugin set and requires every declared setting to
have an independent variation domain.
*/

namespace {
using namespace Slic3r;

// Restrict these tests to the slicing/XY chain, and restore the shared test
// runtime afterwards so their activation choices cannot affect other tests.
class ScopedXYPlugins
{
public:
    // Keep the initialized pipeline for exports, or isolate raw slicing for focused cases.
    explicit ScopedXYPlugins(bool full_pipeline = false) : orchestrator(Orchestrator::instance())
    {
        for (Plugin *plugin : orchestrator.active_plugins())
            previous.push_back(plugin);
        if (!full_pipeline)
            orchestrator.clear_active_plugins();
        for (const char *id : { "standard_layer_height_generator", "slice_volume",
                               "slicing.xy_size_compensation" })
            REQUIRE(orchestrator.set_plugin_active(id, true));
        if (full_pipeline)
            REQUIRE(orchestrator.set_plugin_active("ordering.extrusion_tree.advanced", true));
    }
    ~ScopedXYPlugins()
    {
        orchestrator.clear_active_plugins();
        for (Plugin *plugin : previous)
            orchestrator.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &orchestrator;
    std::vector<Plugin *> previous;
};

// Build neutral settings for focused slicing without requiring a full export.
DynamicPrintConfig xy_config();
// Return the union's X extent in millimeters.
double layer_width(const Layer &layer);
// Run layer generation and slicing, leaving post-slicing to the caller.
void run_raw_slicing(Print &print);

DynamicPrintConfig xy_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "1" }, { "first_layer_height", "1" },
        { "xy_size_compensation", "0" }, { "xy_inner_size_compensation", "0" },
        { "first_layer_size_compensation", "0" },
        { "first_layer_size_compensation_no_collapse", "0" },
        { "slice_closing_radius", "0" }, { "raft_layers", "0" }
    });
    return config;
}

double layer_width(const Layer &layer)
{
    return unscaled(get_extents(layer.lslices()).size().x());
}

void run_raw_slicing(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
}
} // namespace

TEST_CASE("XY declared settings pass a linear STL slicing and export matrix",
          "[plugins][plugin-e2e][xy-size-compensation][xy-e2e]")
{
    namespace E2E = Slic3r::Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active(true);
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.set_plugin_active("slicing.xy_size_compensation", true));
    DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
    baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
        {"xy_size_compensation", "0.1"}, {"xy_inner_size_compensation", "0.05"},
        {"first_layer_size_compensation", "-0.05"},
        {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
    const std::string ordering_key = Steps::plugin_exclusive_group_option_key(
        host, STEP_ORDERING, "ordering.extrusion_tree");
    REQUIRE_FALSE(ordering_key.empty());
    baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
    std::vector<E2E::Domain> domains;
    const auto add = [&domains](const std::string &key, const std::vector<std::string> &values) {
        E2E::Domain domain{{key}, {}};
        for (const std::string &value : values)
            domain.cases.push_back({key + "=" + value, {{key, value}}});
        domains.push_back(std::move(domain));
    };
    add("xy_size_compensation", {"-0.1", "0", "0.2"});
    add("xy_inner_size_compensation", {"-0.1", "0", "0.2"});
    add("first_layer_size_compensation", {"-0.1", "0", "0.1"});
    add("first_layer_size_compensation_layers", {"1", "3", "30"});
    add("first_layer_size_compensation_no_collapse", {"0", "1"});
    add("external_perimeter_extrusion_width", {"0", "0.5"});
    add("external_perimeter_extrusion_spacing", {"0", "100%"});
    add("external_perimeter_extrusion_change_odd_layers", {"0", "80%", "120%"});
    add("external_perimeter_overlap", {"50%", "100%"});
    domains.push_back({{"perimeter_extruder", "nozzle_diameter", "min_layer_height", "max_layer_height",
                        "filament_diameter", "extrusion_multiplier"}, {
        {"second nozzle", {{"perimeter_extruder", "2"}, {"nozzle_diameter", "0.4,0.6"},
                            {"min_layer_height", "5%,5%"}, {"max_layer_height", "75%,75%"},
                            {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"}}}}});
    add("first_layer_extrusion_width", {"0", "120%"});
    add("first_layer_extrusion_spacing", {"0", "100%"});
    add("extrusion_width", {"0", "0.5"});
    add("extrusion_spacing", {"0", "100%"});
    add("filament_max_overlap", {"50%", "100%"});
    add("raft_layers", {"0", "2"});
    const std::vector<std::string> ids{"slicing.xy_size_compensation"};
    const std::vector<E2E::Case> cases = E2E::linear_cases(host, ids, baseline, domains);
    const TriangleMesh mesh = E2E::stl_fixture(true);
    for (const E2E::Case &test : cases) {
        INFO(test.name);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
        print.process();
        REQUIRE(print.object(0).layer_count() > 0);
        for (const Layer &layer : print.object(0).layers()) {
            REQUIRE_FALSE(layer.lslices().empty());
            CHECK(area(layer.lslices()) > 0.);
        }
        // The public export pipeline creates the ordering plan before its writer runs.
        const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
        std::string code;
        try {
            const std::string exported = host.export_gcode_cli(print, path.string());
            code = Test::GCode::read_text_file(exported);
            Test::GCode::remove_output_pair(path);
        } catch (...) {
            Test::GCode::remove_output_pair(path);
            throw;
        }
        CHECK(code.find("G1 ") != std::string::npos);
        CHECK(code.find(" E") != std::string::npos);
    }

    // This percentage replaces spacing rather than adding to it. At 20% the
    // native flow is too narrow for this layer height, with or without XY edits.
    DynamicPrintConfig invalid_spacing = baseline;
    invalid_spacing.set_deserialize_strict("external_perimeter_extrusion_change_odd_layers", "20%");
    for (bool xy_enabled : {false, true}) {
        CAPTURE(xy_enabled);
        host.reset_plugin_cancel();
        REQUIRE(host.set_plugin_active("slicing.xy_size_compensation", xy_enabled));
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, invalid_spacing);
        CHECK_THROWS_WITH(print.process(), Catch::Contains("STEP_INFILL stopped"));
    }
    host.reset_plugin_cancel();
}

TEST_CASE("XY compensation is applied once after volume slicing",
          "[plugins][xy-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    DynamicPrintConfig config = xy_config();
    // Equal positive outer and inner compensation must be applied once, by the
    // post-slicing plugin, and not also as an offset of the mesh before slicing.
    config.set_deserialize_strict("xy_size_compensation", "1");
    config.set_deserialize_strict("xy_inner_size_compensation", "1");
    Print print;
    Model model;
    Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, config);
    run_raw_slicing(print);
    REQUIRE(!print.objects().empty());
    const PrintObject &object = print.objects().front();
    REQUIRE(object.layer_count() > 0);
    CHECK(layer_width(object.layer(0)) == Approx(20.).margin(0.00001));

    Steps::StepPostSlicing::run_step(Orchestrator::instance(), print);
    CHECK(layer_width(object.layer(0)) == Approx(22.).margin(0.00001));
    std::string error;
    const bool valid = Steps::StepPostSlicing::validate_post(print, error);
    INFO(error);
    CHECK(valid);
}

TEST_CASE("XY compensation runs objects in parallel without a plugin lock",
          "[plugins][xy-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    DynamicPrintConfig config = xy_config();
    // The host runs one post-slicing job per object in parallel, all sharing
    // the plugin storage. Alternate the signs so each object has its own result.
    constexpr int k_object_count = 8;
    Model model;
    for (int index = 0; index < k_object_count; ++index) {
        ModelObject *model_object = model.add_object();
        model_object->add_volume(make_cube(10., 10., 2.));
        model_object->config.set_deserialize_strict("xy_size_compensation", index % 2 == 0 ? "-0.5" : "0.5");
        model_object->add_instance()->set_offset(Vec3d(20. * index, 100., 0.));
    }
    Print print;
    for (ModelObject &model_object : model.objects())
        print.auto_assign_extruders(&model_object);
    print.apply(model, config);
    run_raw_slicing(print);
    REQUIRE(print.objects().size() == std::size_t(k_object_count));

    Steps::StepPostSlicing::run_step(Orchestrator::instance(), print);
    for (int index = 0; index < k_object_count; ++index) {
        INFO("object " << index);
        const PrintObject &object = print.object(index);
        REQUIRE(object.layer_count() > 0);
        CHECK(layer_width(object.layer(0)) == Approx(index % 2 == 0 ? 9. : 11.).margin(0.00001));
    }
    std::string error;
    const bool valid = Steps::StepPostSlicing::validate_post(print, error);
    INFO(error);
    CHECK(valid);
}

TEST_CASE("First-layer XY compensation fades over the configured object layers",
          "[plugins][xy-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    DynamicPrintConfig config = xy_config();
    config.set_deserialize_strict("first_layer_size_compensation", "-0.2");
    config.set_deserialize_strict("first_layer_size_compensation_layers", "2");
    double first_width = 19.6;
    SECTION("No raft: full compensation on object layer zero") {}
    SECTION("No-collapse obtains its external-perimeter flow from the host") {
        config.set_deserialize_strict("first_layer_size_compensation_no_collapse", "1");
    }
    SECTION("Raft: preserve the legacy object-index eligibility condition") {
        config.set_deserialize_strict("raft_layers", "1");
        first_width = 20.;
    }
    Print print;
    Model model;
    Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, config);
    run_raw_slicing(print);
    Steps::StepPostSlicing::run_step(Orchestrator::instance(), print);
    const PrintObject &object = print.objects().front();
    REQUIRE(object.layer_count() >= 3);
    CHECK(layer_width(object.layer(0)) == Approx(first_width).margin(0.00001));
    CHECK(layer_width(object.layer(1)) == Approx(19.8).margin(0.00001));
    CHECK(layer_width(object.layer(2)) == Approx(20.).margin(0.00001));
}

TEST_CASE("XY compensation uses raw region geometry rather than a simplified island cache",
          "[plugins][xy-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    DynamicPrintConfig config = xy_config();
    config.set_deserialize_strict("xy_size_compensation", "1");
    Print print;
    Model model;
    Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, config);
    run_raw_slicing(print);
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 0);

    // StepSlicing may simplify/safety-union its island cache separately from
    // the exact raw regions. Model that discrepancy without changing the raw
    // input: the cached contour must not become an uncovered source mask.
    Layer &layer = object.layer(0);
    ApiInternal::LayerAccess::set_islands(layer,
        offset_ex(layer.lslices(), double(SCALED_EPSILON)));
    REQUIRE(layer_width(layer) > 20.);
    REQUIRE_NOTHROW(Steps::StepPostSlicing::run_step(Orchestrator::instance(), print));
    CHECK(layer_width(layer) == Approx(22.).margin(0.00001));
}

TEST_CASE("XY compensation reads distinct volume regions on the same island",
          "[plugins][xy-size-compensation][integration]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    DynamicPrintConfig config = xy_config();
    Model model;
    ModelObject *model_object = model.add_object();
    // The touching boxes form one island, but their volume overrides must
    // remain distinct regions all the way from profile resolution to XY.
    for (int index = 0; index < 2; ++index) {
        TriangleMesh mesh = make_cube(10., 10., 2.);
        mesh.translate(10. * index, 0., 0.);
        ModelVolume *volume = model_object->add_volume(std::move(mesh));
        volume->config.set_deserialize_strict("xy_size_compensation", index == 0 ? "-1" : "-2");
    }
    model_object->add_instance();
    model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(model_object);
    print.apply(model, config);
    run_raw_slicing(print);
    const PrintObject &object = print.objects().front();
    REQUIRE(object.layer_count() > 0);
    // The host may also retain an empty default region alongside the two overrides.
    std::vector<std::size_t> active_regions;
    for (std::size_t index = 0; index < object.layer(0).region_count(); ++index)
        if (!object.layer(0).region(index).get_raw_slices().empty())
            active_regions.push_back(index);
    REQUIRE(active_regions.size() == 2);
    REQUIRE(object.layer(0).lslices().size() == 1);
    CHECK(layer_width(object.layer(0)) == Approx(20.).margin(0.00001));

    Steps::StepPostSlicing::run_step(Orchestrator::instance(), print);
    // The left edge withdraws by 1 mm and the right edge by 2 mm.
    CHECK(layer_width(object.layer(0)) == Approx(17.).margin(0.00001));
    const Layer &layer = object.layer(0);
    CHECK(intersection_ex(layer.region(active_regions[0]).get_raw_slices(),
                          layer.region(active_regions[1]).get_raw_slices()).empty());
    // Multi-region host reconstruction asserts that every published ring has
    // no nearby consecutive vertices. Validate the resulting layer as well.
    // Check explicitly first: assert_valid() stops the process instead of
    // reporting a test failure.
    const auto has_nearby_vertices = [](const Polygon &ring) {
        for (std::size_t idx = 0; idx < ring.size(); ++idx)
            if (ring.points[idx].coincides_with_epsilon(ring.points[(idx + 1) % ring.size()]))
                return true;
        return false;
    };
    for (const LayerRegion &region : layer.regions())
        for (const ExPolygon &area : region.get_raw_slices()) {
            REQUIRE(area.contour.size() >= 3);
            REQUIRE_FALSE(has_nearby_vertices(area.contour));
            for (const Polygon &hole : area.holes) {
                REQUIRE(hole.size() >= 3);
                REQUIRE_FALSE(has_nearby_vertices(hole));
            }
            area.assert_valid();
        }
    std::string error;
    const bool valid = Steps::StepPostSlicing::validate_post(print, error);
    INFO(error);
    CHECK(valid);
}

TEST_CASE("XY regional fallback reports boundary artifacts and preserves the host layer",
          "[plugins][xy-size-compensation][integration][regional-fallback]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    ScopedXYPlugins active;
    Model model;
    ModelObject *model_object = model.add_object();
    for (int index = 0; index < 2; ++index) {
        TriangleMesh mesh = make_cube(10., 10., 2.);
        mesh.translate(10. * index, 0., 0.);
        ModelVolume *volume = model_object->add_volume(std::move(mesh));
        volume->config.set_deserialize_strict("xy_size_compensation", index == 0 ? "1" : "-0.5");
    }
    model_object->add_instance();
    model.center_instances_around_point({100., 100.});
    model_object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(model_object);
    print.apply(model, xy_config());
    run_raw_slicing(print);
    PrintObject &object = print.object(0);
    REQUIRE(object.layer_count() > 0);
    Layer &layer = object.layer(0);
    std::vector<std::size_t> active_regions;
    for (std::size_t idx = 0; idx < layer.region_count(); ++idx)
        if (!layer.region(idx).get_raw_slices().empty())
            active_regions.push_back(idx);
    REQUIRE(active_regions.size() == 2);

    // Replace the raw box slices with the acute triangle from the geometry
    // regression. Keep the real volume settings and post-slicing host path.
    const ExPolygon source(Polygon{ Point(scale_i(6.), scale_i(6.)),
        Point(scale_i(15.), scale_i(1.)), Point(scale_i(24.), scale_i(6.)) });
    const ExPolygon left_half(Polygon{ Point(scale_i(-30.), scale_i(-30.)),
        Point(scale_i(15.), scale_i(-30.)), Point(scale_i(15.), scale_i(60.)),
        Point(scale_i(-30.), scale_i(60.)) });
    const ExPolygons left = intersection_ex(ExPolygons{ source }, ExPolygons{ left_half });
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(active_regions[0])) = left;
    ApiInternal::LayerRegionAccess::slices_mutable(layer.region(active_regions[1])) =
        diff_ex(ExPolygons{ source }, left);
    ApiInternal::LayerAccess::recompute_slices_from_layer_regions(layer);

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.consume_plugin_messages();
    Steps::StepPostSlicing::run_step(orchestrator, print);
    const auto messages = orchestrator.consume_plugin_messages();
    std::size_t boundary_warnings = 0;
    for (const Orchestrator::PluginMessage &message : messages) {
        if (message.plugin_id != "slicing.xy_size_compensation")
            continue;
        INFO(message.message);
        CHECK(message.level != Orchestrator::PluginMessageLevel::Error);
        if (message.message.find(" on layer 0 ") == std::string::npos)
            continue;
        ++boundary_warnings;
        CHECK(message.level == Orchestrator::PluginMessageLevel::Warning);
        CHECK(message.step == STEP_POST_SLICING);
        const std::string prefix = "XY size compensation: ";
        REQUIRE(message.message.find(prefix) == 0);
        CHECK(std::stod(message.message.substr(prefix.size())) == Approx(5. / 9.).margin(0.00001));
        CHECK(message.message.find(" mm2 of new material on layer 0 had no owning region and was "
                                   "assigned using the regional fallback rule.") != std::string::npos);
    }
    CHECK(boundary_warnings == 1);
    const Point extra_material(scale_i(15.8), scale_i(-0.3));
    CHECK(expolygons_contain(layer.lslices(), extra_material));
    CHECK(expolygons_contain(layer.region(active_regions[0]).get_raw_slices(), extra_material));
    CHECK_FALSE(expolygons_contain(layer.region(active_regions[1]).get_raw_slices(), extra_material));
    CHECK(intersection_ex(layer.region(active_regions[0]).get_raw_slices(),
                          layer.region(active_regions[1]).get_raw_slices()).empty());
    std::string error;
    const bool valid = Steps::StepPostSlicing::validate_post(print, error);
    INFO(error);
    CHECK(valid);
}
