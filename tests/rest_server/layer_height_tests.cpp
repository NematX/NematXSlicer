// Direct tests cover profile validation, the GUI's adaptive/smoothing engines, and effective layers.
#include <catch2/catch.hpp>

#include "layer_height.hpp"
#include "rest_models_test_helpers.hpp"

#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Return an owned scene with one grounded 20 mm cube; callers may change its profile without shared state.
Model make_cube_scene()
{
    Model scene;
    ModelObject *object = scene.add_object("cube", "cube.stl", make_cube(20., 20., 20.));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Return an owned sphere scene whose sloped surface supplies adaptive samples.
Model make_sphere_scene()
{
    Model scene;
    ModelObject *object = scene.add_object("sphere", "sphere.stl", make_sphere(10., 2. * PI / 36.));
    object->add_instance();
    object->ensure_on_bed();
    return scene;
}

// Return an owning FFF configuration with matching 0.2 mm first and regular layer heights.
DynamicPrintConfig make_session()
{
    DynamicPrintConfig session = DynamicPrintConfig::full_print_config();
    session.set_deserialize_strict("layer_height", "0.2");
    session.set_deserialize_strict("first_layer_height", "0.2");
    return session;
}

// Validate points and return the caught zero-based point/reason; a missing point error fails the test.
std::pair<std::size_t, std::string> point_error(const ModelObject &object, const DynamicPrintConfig &session,
    const std::vector<std::pair<double, double>> &points)
{
    LayerHeightRequest request;
    request.profile = points;
    try {
        compute_layer_height_profile(object, session, request);
    } catch (const LayerHeightPointError &error) {
        CHECK(error.status() == 422);
        CHECK(error.code() == "invalid_layer_height_profile");
        return {error.point(), error.reason()};
    }
    FAIL("compute_layer_height_profile did not throw a point error.");
    return {};
}

// Run a request and assert its 422 code/message; acceptance or a different error fails the test.
void require_profile_error(const ModelObject &object, const DynamicPrintConfig &session,
    const LayerHeightRequest &request, const std::string &reason)
{
    try {
        compute_layer_height_profile(object, session, request);
        FAIL("compute_layer_height_profile did not reject the request.");
    } catch (const SceneEditError &error) {
        CHECK(error.status() == 422);
        CHECK(error.code() == "invalid_layer_height_profile");
        CHECK(std::string(error.what()).find(reason) != std::string::npos);
    }
}

// Return the maximum absolute change between adjacent height values without modifying the profile.
double maximum_adjacent_height_change(const std::vector<double> &profile)
{
    double maximum = 0.;
    for (std::size_t index = 3; index < profile.size(); index += 2)
        maximum = std::max(maximum, std::abs(profile[index] - profile[index - 2]));
    return maximum;
}

// Return whether relative layer tops contain the requested positive height; the JSON array is read only.
bool has_layer_height(const nlohmann::json &layers, double layer_height)
{
    double previous = 0.;
    for (const nlohmann::json &layer : layers) {
        const double top = layer.get<double>();
        if (std::abs(top - previous - layer_height) < 1e-6)
            return true;
        previous = top;
    }
    return false;
}

} // namespace

// Validate normalized explicit points and engine profiles on an owned cube; invalid requests must report their offending points.
TEST_CASE("Explicit layer height profiles are validated", "[layer_height]")
{
    ensure_engine_initialized();
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    const DynamicPrintConfig session = make_session();
    const std::shared_ptr<SlicingParameters> parameters = PrintObject::slicing_parameters(
        session, *object, static_cast<float>(object->max_z()));

    LayerHeightRequest valid;
    valid.profile = std::vector<std::pair<double, double>>{{0., 0.2}, {5., 0.2}, {5., 0.1}, {20.0005, 0.1}};
    const std::vector<double> flat = compute_layer_height_profile(*object, session, valid);
    REQUIRE(flat.size() == 8);
    CHECK(flat[6] == Approx(parameters->object_print_z_height()).margin(1e-9));
    object->layer_height_profile.set(flat);

    const nlohmann::json json = layer_height_json(*object, session);
    CHECK(json.at("applied") == true);
    CHECK(json.at("object_height") == Approx(parameters->object_print_z_height()).margin(1e-9));
    REQUIRE(json.at("profile").size() == 4);
    CHECK(json.at("profile").at(3).at(0) == Approx(parameters->object_print_z_height()).margin(1e-9));
    REQUIRE(json.at("layers").size() > 10);
    const std::vector<double> tops = json.at("layers").get<std::vector<double>>();
    double previous = 0.;
    for (std::size_t index = 0; index < tops.size(); ++index) {
        const double layer_height = tops[index] - previous;
        if (index == 0)
            CHECK(layer_height == Approx(parameters->first_object_layer_height).margin(1e-6));
        else
            CHECK(layer_height == Approx(tops[index] <= 5. + 1e-6 ? 0.2 : 0.1).margin(1e-6));
        previous = tops[index];
    }

    LayerHeightRequest ramp;
    ramp.profile = std::vector<std::pair<double, double>>{{0., 0.2}, {20., 0.1}};
    object->layer_height_profile.set(compute_layer_height_profile(*object, session, ramp));
    const std::vector<double> ramp_tops = layer_height_json(*object, session).at("layers").get<std::vector<double>>();
    REQUIRE(ramp_tops.size() > 3);
    std::vector<double> ramp_heights;
    double ramp_previous = 0.;
    for (double top : ramp_tops) {
        ramp_heights.push_back(top - ramp_previous);
        ramp_previous = top;
    }
    CHECK(ramp_heights.front() == Approx(parameters->first_object_layer_height).margin(1e-6));
    CHECK(ramp_heights[1] > ramp_heights.back());
    for (std::size_t index = 2; index < ramp_heights.size(); ++index)
        CHECK(ramp_heights[index] <= ramp_heights[index - 1] + 1e-6);

    LayerHeightRequest too_few;
    too_few.profile = std::vector<std::pair<double, double>>{{0., 0.2}};
    try {
        compute_layer_height_profile(*object, session, too_few);
        FAIL("A one-point profile was accepted.");
    } catch (const SceneEditError &error) {
        CHECK(error.status() == 422);
        CHECK(error.code() == "invalid_layer_height_profile");
        CHECK(std::string(error.what()).find("too_few_points") != std::string::npos);
    }
    Model no_solid_scene;
    ModelObject *no_solid_object = no_solid_scene.add_object();
    no_solid_object->add_instance();
    LayerHeightRequest reset_without_solid;
    reset_without_solid.reset = true;
    require_profile_error(*no_solid_object, session, reset_without_solid, "no solid part");
    CHECK(point_error(*object, session, {{0.1, 0.2}, {20., 0.1}}) ==
        std::make_pair(std::size_t(0), std::string("first_z")));
    CHECK(point_error(*object, session, {{0., 0.2}, {5., 0.2}, {4., 0.1}, {20., 0.1}}) ==
        std::make_pair(std::size_t(2), std::string("z_order")));
    CHECK(point_error(*object, session, {{0., 0.2}, {5., 0.2}, {5., 0.15}, {5., 0.1}, {20., 0.1}}) ==
        std::make_pair(std::size_t(3), std::string("z_order")));
    CHECK(point_error(*object, session, {{0., 0.2}, {5., 1.}, {20., 0.1}}) ==
        std::make_pair(std::size_t(1), std::string("height_range")));
    CHECK(point_error(*object, session, {{0., 0.2}, {19.5, 0.1}}) ==
        std::make_pair(std::size_t(1), std::string("last_z")));
    CHECK(point_error(*object, session, {{0., 0.2}, {20.0002, 0.1}, {20.0005, 0.1}}) ==
        std::make_pair(std::size_t(2), std::string("z_order")));
    CHECK(point_error(*object, session, {{0., 0.2}, {20., 0.2}, {20., 0.1}, {20.0005, 0.1}}) ==
        std::make_pair(std::size_t(3), std::string("z_order")));

    DynamicPrintConfig narrow_nozzle_session = session;
    narrow_nozzle_session.set_deserialize_strict("nozzle_diameter", "0.25");
    const std::shared_ptr<SlicingParameters> narrow_parameters = PrintObject::slicing_parameters(
        narrow_nozzle_session, *object, static_cast<float>(object->max_z()));
    REQUIRE(narrow_parameters->first_object_layer_height > narrow_parameters->max_layer_height);
    CHECK(point_error(*object, narrow_nozzle_session, {{0., 0.2}, {0.3, 0.1}, {20., 0.1}}) ==
        std::make_pair(std::size_t(0), std::string("height_range")));
}

// Check adaptive bounds and sphere-slope response, smooth a valid profile, and confirm reset exposes a local range override.
TEST_CASE("Adaptive and smoothed profiles follow the GUI tools", "[layer_height]")
{
    ensure_engine_initialized();
    Model scene = make_sphere_scene();
    ModelObject *object = scene.object_ptrs().front();
    const DynamicPrintConfig session = make_session();
    const std::shared_ptr<SlicingParameters> parameters = PrintObject::slicing_parameters(
        session, *object, static_cast<float>(object->max_z()));

    LayerHeightRequest adaptive;
    adaptive.adaptive = LayerHeightAdaptiveRequest{0.5, 0.08, 0.28};
    const std::vector<double> generated = compute_layer_height_profile(*object, session, adaptive);
    REQUIRE(generated.size() > 8);
    CHECK(generated[1] == Approx(parameters->first_object_layer_height).margin(1e-6));
    CHECK(generated[3] == Approx(parameters->first_object_layer_height).margin(1e-6));
    CHECK(generated[generated.size() - 2] == Approx(parameters->object_print_z_height()).margin(1e-6));
    const std::size_t point_count = generated.size() / 2;
    CHECK(generated.back() >= parameters->min_layer_height - 1e-6);
    CHECK(generated.back() <= parameters->max_layer_height + 1e-6);
    double minimum = parameters->max_layer_height;
    double maximum = parameters->min_layer_height;
    double bottom_sum = 0.;
    double equator_sum = 0.;
    double top_sum = 0.;
    std::size_t bottom_count = 0;
    std::size_t equator_count = 0;
    std::size_t top_count = 0;
    for (std::size_t point = 2; point + 1 < point_count; ++point) {
        const double z = generated[point * 2];
        const double layer_height = generated[point * 2 + 1];
        CHECK(layer_height >= 0.08 - 1e-6);
        CHECK(layer_height <= 0.28 + 1e-6);
        minimum = std::min(minimum, layer_height);
        maximum = std::max(maximum, layer_height);
        if (z > 0.2 && z < 3.) {
            bottom_sum += layer_height;
            ++bottom_count;
        } else if (z >= 8. && z <= 12.) {
            equator_sum += layer_height;
            ++equator_count;
        } else if (z > 17. && z < parameters->object_print_z_height()) {
            top_sum += layer_height;
            ++top_count;
        }
    }
    CHECK(maximum > minimum);
    REQUIRE(bottom_count > 0);
    REQUIRE(equator_count > 0);
    REQUIRE(top_count > 0);
    CHECK(bottom_sum / bottom_count < equator_sum / equator_count);
    CHECK(top_sum / top_count < equator_sum / equator_count);

    LayerHeightRequest invalid_adaptive;
    invalid_adaptive.adaptive = LayerHeightAdaptiveRequest{0.5, 0.28, 0.08};
    require_profile_error(*object, session, invalid_adaptive, "Adaptive heights");
    invalid_adaptive.adaptive = LayerHeightAdaptiveRequest{1.1, 0.08, 0.28};
    require_profile_error(*object, session, invalid_adaptive, "Adaptive quality");
    invalid_adaptive.adaptive = LayerHeightAdaptiveRequest{0.5,
        parameters->min_layer_height - 0.01, parameters->max_layer_height};
    require_profile_error(*object, session, invalid_adaptive, "Adaptive heights");

    LayerHeightRequest staircase;
    std::vector<std::pair<double, double>> staircase_points;
    staircase_points.reserve(41);
    for (std::size_t index = 0; index <= 40; ++index) {
        const double height = index == 0 ? 0.2 : index < 20 ? 0.25 : 0.15;
        staircase_points.emplace_back(index * 0.5, height);
    }
    staircase.profile = std::move(staircase_points);
    const std::vector<double> unsmoothed = compute_layer_height_profile(*object, session, staircase);
    object->layer_height_profile.set(unsmoothed);
    LayerHeightRequest smooth;
    smooth.smooth = HeightProfileSmoothingParams(5, false);
    const std::vector<double> smoothed = compute_layer_height_profile(*object, session, smooth);
    CHECK(maximum_adjacent_height_change(smoothed) < maximum_adjacent_height_change(unsmoothed));
    smooth.smooth = HeightProfileSmoothingParams(5, true);
    const std::vector<double> kept_minimum = compute_layer_height_profile(*object, session, smooth);
    REQUIRE(kept_minimum.size() == unsmoothed.size());
    for (std::size_t point = 0; point * 2 < unsmoothed.size(); ++point) {
        CHECK(kept_minimum[point * 2] == Approx(unsmoothed[point * 2]).margin(1e-9));
        CHECK(kept_minimum[point * 2 + 1] <= unsmoothed[point * 2 + 1] + 1e-6);
    }
    smooth.smooth = HeightProfileSmoothingParams(11, false);
    require_profile_error(*object, session, smooth, "Smoothing radius");

    object->layer_config_ranges[{0., 20.}].set_key_value("layer_height", new ConfigOptionFloat(0.1));
    LayerHeightRequest reset;
    reset.reset = true;
    CHECK(compute_layer_height_profile(*object, session, reset).empty());
    object->layer_height_profile.clear();
    const nlohmann::json reset_json = layer_height_json(*object, session);
    CHECK(reset_json.at("profile").empty());
    CHECK(reset_json.at("applied") == false);
    CHECK(has_layer_height(reset_json.at("layers"), 0.1));
}

// Scale an object to stale its stored top, then verify GET and smoothing reject stale-top and odd-length data without generating layers.
TEST_CASE("Stale layer height profiles are reported", "[layer_height]")
{
    ensure_engine_initialized();
    Model scene = make_cube_scene();
    ModelObject *object = scene.object_ptrs().front();
    const DynamicPrintConfig session = make_session();
    LayerHeightRequest profile;
    profile.profile = std::vector<std::pair<double, double>>{{0., 0.2}, {5., 0.1}, {20., 0.1}};
    object->layer_height_profile.set(compute_layer_height_profile(*object, session, profile));
    const std::vector<double> stored = object->layer_height_profile.get();
    object->instances.front()->set_scaling_factor(Vec3d(1., 1., 2.));
    object->invalidate_bounding_box();

    const nlohmann::json stale = layer_height_json(*object, session);
    CHECK(stale.at("applied") == false);
    CHECK(stale.at("valid") == false);
    CHECK_FALSE(stale.contains("layers"));
    REQUIRE(stale.at("errors").size() == 1);
    CHECK(stale.at("errors").at(0) == nlohmann::json{{"point", 2}, {"reason", "last_z"}});
    CHECK(stale.at("object_height") == Approx(40.).margin(1e-6));
    REQUIRE(stale.at("profile").size() == 3);

    LayerHeightRequest smooth_stale;
    smooth_stale.smooth = HeightProfileSmoothingParams(3, false);
    require_profile_error(*object, session, smooth_stale, "last_z");

    object->layer_height_profile.clear();
    const nlohmann::json fallback = layer_height_json(*object, session);
    CHECK(fallback.at("valid") == true);
    CHECK_FALSE(fallback.at("layers").empty());
    object->layer_height_profile.set(stored);
    const nlohmann::json stale_again = layer_height_json(*object, session);
    CHECK(stale_again.at("valid") == false);
    CHECK_FALSE(stale_again.contains("layers"));

    object->layer_height_profile.set({0., 0.2, 5.});
    const nlohmann::json invalid_length = layer_height_json(*object, session);
    CHECK(invalid_length.at("valid") == false);
    CHECK(invalid_length.at("applied") == false);
    CHECK_FALSE(invalid_length.contains("layers"));
    REQUIRE(invalid_length.at("errors").size() == 1);
    CHECK(invalid_length.at("errors").at(0) == nlohmann::json{{"reason", "invalid_length"}});
    require_profile_error(*object, session, smooth_stale, "invalid_length");
}
