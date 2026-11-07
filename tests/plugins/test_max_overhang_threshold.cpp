#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

/*
Tests for the max_overhang_threshold post-slicing plugin.

The fixture is a "T": a 4 mm wide stem carrying a 20 mm wide plate, with open
cantilevers on both sides of the stem. The cantilevers are anchored on one
side only, so the bridge detector finds no bridge over them and the plugin
must cut them back to the maximum slope.

The tall stem separates the overhang from first-layer behavior. A 0.3 mm first
layer followed by 0.2 mm layers keeps the plate bottom between slice planes.
The three look-ahead variants are separate scheduling units. The area oracle
reuses an evaluated result when both adjacent slice lists are exactly equal
to a previously checked pair, while retaining a CHECK for every layer pair.
SLIC3R_TEST_PROFILE_MAX_OVERHANG prints step times and the oracle evaluation
count without changing the test's settings or geometry.

    regression case
    `-- check_max_slope(): run one look-ahead value
        |-- max_slope_config(): keep layer sampling and slope explicit
        |-- run_until_post_slicing(): slice the T and execute the real plugins
        `-- over_slope_area_mm2(): check the bound between every pair of layers
*/

namespace {

using namespace Slic3r;

const char *const STANDARD_LAYER_HEIGHT_GENERATOR = "standard_layer_height_generator";
const char *const SLICE_VOLUME = "slice_volume";
const char *const BRIDGE_DETECTOR = "bridge_detector.default";
const char *const MAX_OVERHANG_THRESHOLD = "max_overhang_threshold";

constexpr double plate_bottom_z = 8.1;
constexpr double model_top_z = 10.;

// Build one axis-aligned owned box for the stem or plate.
TriangleMesh make_box(double min_x, double min_y, double min_z,
                      double max_x, double max_y, double max_z);
// Put the wide plate on a tall stem, away from the first-layer boundary.
TriangleMesh make_t_shape();
// Construct the slope and look-ahead settings with 0.2 mm layers and a 0.3 mm first layer.
DynamicPrintConfig max_slope_config(const std::string &max_slope, const std::string &upper_layers);
// Apply the model and run the production layer-height, slicing and post-slicing steps.
void run_until_post_slicing(Model &model, Print &print, const DynamicPrintConfig &config);
// Check slope bounds for every layer pair and the first/last plate geometry.
// Reuse an oracle value only when both input slice lists are unchanged.
void check_max_slope(const std::string &upper_layers);

// Restrict the active plugins to the ones a test needs, and restore the
// previous activation set at scope exit.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids) {
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

TriangleMesh make_box(const double min_x, const double min_y, const double min_z,
                      const double max_x, const double max_y, const double max_z)
{
    TriangleMesh box(its_make_cube(max_x - min_x, max_y - min_y, max_z - min_z));
    box.translate(float(min_x), float(min_y), float(min_z));
    return box;
}

TriangleMesh make_t_shape()
{
    TriangleMesh mesh = make_box(-2., -10., 0., 2., 10., model_top_z);
    mesh.merge(make_box(-10., -10., plate_bottom_z, 10., 10., model_top_z));
    return mesh;
}

DynamicPrintConfig max_slope_config(const std::string &max_slope, const std::string &upper_layers)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"nozzle_diameter", "0.4"},
        {"layer_height", "0.2"},
        {"first_layer_height", "0.3"},
        {"overhangs_max_slope", max_slope},
        {"overhangs_bridge_upper_layers", upper_layers}
    });
    return config;
}

void run_until_post_slicing(Model &model, Print &print, const DynamicPrintConfig &config)
{
    using Clock = std::chrono::steady_clock;
    const Clock::time_point started = Clock::now();
    ModelObject *object = model.add_object();
    object->name = "max_overhang_threshold_test.stl";
    object->add_volume(make_t_shape(), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    model.center_instances_around_point({100, 100});
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();

    ScopedActivePlugins active({STANDARD_LAYER_HEIGHT_GENERATOR, SLICE_VOLUME, BRIDGE_DETECTOR, MAX_OVERHANG_THRESHOLD});
    Orchestrator &orchestrator = Orchestrator::instance();
    const Clock::time_point prepared = Clock::now();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    const Clock::time_point heights = Clock::now();
    Steps::StepSlicing::run_step(orchestrator, print);
    const Clock::time_point sliced = Clock::now();
    Steps::StepPostSlicing::run_step(orchestrator, print);
    const Clock::time_point processed = Clock::now();
    if (std::getenv("SLIC3R_TEST_PROFILE_MAX_OVERHANG") != nullptr) {
        std::fprintf(stderr, "MAX_OVERHANG_PROFILE prepare=%.3f height=%.3f slicing=%.3f post_slicing=%.3f ms\n",
            std::chrono::duration<double, std::milli>(prepared - started).count(),
            std::chrono::duration<double, std::milli>(heights - prepared).count(),
            std::chrono::duration<double, std::milli>(sliced - heights).count(),
            std::chrono::duration<double, std::milli>(processed - sliced).count());
    }
}

double area_mm2(const ExPolygons &areas)
{
    double out = 0.;
    for (const ExPolygon &area : areas)
        out += std::abs(area.area());
    return unscaled(unscaled(out));
}

// Area of the layer above that hangs more than slope_mm over `layer`.
double over_slope_area_mm2(const Layer &layer, const Layer &upper, const double slope_mm)
{
    return area_mm2(diff_ex(upper.lslices(), offset_ex(layer.lslices(), float(scale_d(slope_mm + 0.01)))));
}

void check_max_slope(const std::string &upper_layers)
{
    INFO("overhangs_bridge_upper_layers = " << upper_layers);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    run_until_post_slicing(model, print, max_slope_config("1", upper_layers));
    const PrintObject &object = print.object(0);

    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    size_t evaluated_pair = 0, evaluations = 0;
    double excess = 0.;
    // The plate is cut so that each layer hangs at most 1 mm over the one below.
    for (size_t layer_idx = 0; layer_idx + 1 < object.layer_count(); ++layer_idx) {
        INFO("layer " << layer_idx);
        if (evaluations == 0 || object.layer(layer_idx).lslices() != object.layer(evaluated_pair).lslices() ||
            object.layer(layer_idx + 1).lslices() != object.layer(evaluated_pair + 1).lslices()) {
            excess = over_slope_area_mm2(object.layer(layer_idx), object.layer(layer_idx + 1), 1.);
            evaluated_pair = layer_idx;
            ++evaluations;
        }
        CHECK(excess < 0.05);
    }
    // The first plate layer keeps the 4 mm stem plus 1 mm on each side, over the
    // 20 mm depth, instead of the full 20 x 20 mm plate.
    size_t plate_idx = 0;
    while (plate_idx < object.layer_count() && object.layer(plate_idx).slice_z < plate_bottom_z)
        ++plate_idx;
    REQUIRE(plate_idx < object.layer_count());
    REQUIRE(plate_idx >= 2);
    REQUIRE(plate_idx + 2 < object.layer_count());
    CHECK(area_mm2(object.layer(plate_idx).lslices()) == Approx(120.).margin(1.));
    CHECK(area_mm2(object.layer(plate_idx - 1).lslices()) == Approx(80.).margin(1.));
    CHECK(area_mm2(object.layer(object.layer_count() - 1).lslices()) == Approx(400.).margin(1.));
    if (std::getenv("SLIC3R_TEST_PROFILE_MAX_OVERHANG") != nullptr) {
        std::fprintf(stderr, "MAX_OVERHANG_PROFILE upper=%s oracle=%.3f ms evaluations=%zu layer_pairs=%zu\n",
            upper_layers.c_str(), std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count(),
            evaluations, object.layer_count() - 1);
    }
}
} // namespace

// Regression: enabled look-ahead must handle cantilevers with no bridge operand.
TEST_CASE("Max overhang threshold cuts unbridged cantilevers (look-ahead 2)",
          "[plugins][post-slicing][max_overhang_threshold]")
{
    check_max_slope("2");
}

// A zero look-ahead retains the same adjacent-layer slope contract.
TEST_CASE("Max overhang threshold cuts unbridged cantilevers (look-ahead 0)",
          "[plugins][post-slicing][max_overhang_threshold]")
{
    check_max_slope("0");
}

// A stored but disabled look-ahead value must retain that contract too.
TEST_CASE("Max overhang threshold cuts unbridged cantilevers (look-ahead disabled)",
          "[plugins][post-slicing][max_overhang_threshold]")
{
    check_max_slope("!2");
}
