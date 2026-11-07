// HTTP tests cover layer-height request shapes, scene edits, project persistence and sliced output.
#include <catch2/catch.hpp>

#include "layer_height.hpp"
#include "rest_models_test_helpers.hpp"

#include "libslic3r/PrintObject.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// POST an owning JSON serialization and return the HTTP response; the server may commit the requested mutation.
HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target,
    const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

// Create a session, upload one grounded cube, and return its route; failed HTTP assertions stop the test.
std::string session_with_cube(RestServerFixture &fixture)
{
    const std::string route = "/session/" + create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return route;
}

// GET and parse an owning scene listing; a non-200 response fails the test.
nlohmann::json list_scene(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// GET and parse the owning profile response; a non-200 response fails the test.
nlohmann::json get_layer_height_profile(const RestServerFixture &fixture, const std::string &route,
    const std::string &object_id)
{
    const HttpResponseMessage response = fixture.request(http::verb::get,
        route + "/get_layer_height_profile/" + object_id);
    INFO("Response: " << response.body());
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// POST one profile operation and parse its owning response; the scene may change and non-200 fails the test.
nlohmann::json set_layer_height_profile(const RestServerFixture &fixture, const std::string &route,
    const std::string &object_id, const nlohmann::json &body)
{
    const HttpResponseMessage response = post_json(fixture,
        route + "/set_layer_height_profile/" + object_id, body);
    INFO("Response: " << response.body());
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// POST one invalid operation and return its parsed error; unexpected status/code or malformed output fails the test.
nlohmann::json set_error(const RestServerFixture &fixture, const std::string &route,
    const std::string &object_id, const nlohmann::json &body, http::status status, const std::string &code)
{
    const HttpResponseMessage response = post_json(fixture,
        route + "/set_layer_height_profile/" + object_id, body);
    INFO("Response: " << response.body());
    require_error(response, status, code);
    return nlohmann::json::parse(response.body()).at("error");
}

// Copy and commit session configuration values; the session changes and failed mutation assertions stop the test.
void set_options(const std::shared_ptr<SessionContext> &session,
    const std::vector<std::pair<std::string, std::string>> &options)
{
    SessionData data = session->data_snapshot().value();
    for (const std::pair<std::string, std::string> &option : options)
        data.configuration.set_deserialize_strict(option.first, option.second);
    const SessionContext::MutationToken token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(token, std::move(data), true, SessionContext::Clock::now()));
}

// Poll read-only session status until the requested state or 180-second deadline; return false on failure or timeout.
bool wait_for_status(const std::shared_ptr<SessionContext> &session, SessionStatus status)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
    while (std::chrono::steady_clock::now() < deadline) {
        const SessionStatus current = session->snapshot().status;
        if (current == status)
            return true;
        if (current == SessionStatus::failed)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return session->snapshot().status == status;
}

// Start slicing, wait for success, and return owning G-code text; rejected requests or failed/late slices fail the test.
std::string slice_gcode(const RestServerFixture &fixture, const std::string &route,
    const std::shared_ptr<SessionContext> &session)
{
    REQUIRE(fixture.request(http::verb::post, route + "/slice").result() == http::status::accepted);
    INFO("Slice error: " << session->snapshot().error_message);
    REQUIRE(wait_for_status(session, SessionStatus::sliced));
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_gcode");
    REQUIRE(response.result() == http::status::ok);
    return response.body();
}

// Return owning numeric values parsed from every absolute Z marker in the supplied G-code.
std::vector<double> gcode_z_markers(const std::string &gcode)
{
    const std::regex marker(R"(;Z:([0-9]+(?:\.[0-9]+)?))");
    const std::sregex_iterator end;
    std::vector<double> result;
    for (std::sregex_iterator match(gcode.begin(), gcode.end(), marker); match != end; ++match)
        result.push_back(std::stod((*match)[1].str()));
    return result;
}

// Return owning group heights paired with Z markers, not later per-extrusion heights.
std::vector<double> gcode_height_markers(const std::string &gcode)
{
    const std::regex marker(R"(;Z:[0-9]+(?:\.[0-9]+)?\n;MAX_LAYER_HEIGHT:([0-9]+(?:\.[0-9]+)?))");
    const std::sregex_iterator end;
    std::vector<double> result;
    for (std::sregex_iterator match(gcode.begin(), gcode.end(), marker); match != end; ++match)
        result.push_back(std::stod((*match)[1].str()));
    return result;
}

} // namespace

// Send invalid operations to one local session and assert request, entity, and target error responses.
TEST_CASE("REST layer height profiles validate requests", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const nlohmann::json listed = list_scene(fixture, route);
    const nlohmann::json object = listed.at("objects").at(0);
    const std::string object_id = object.at("id").get<std::string>();
    const std::string part_id = object.at("parts").at(0).at("id").get<std::string>();
    const std::string instance_id = object.at("instances").at(0).at("id").get<std::string>();

    for (const nlohmann::json &body : std::vector<nlohmann::json>{
            nlohmann::json::object(),
            {{"profile", {{0., 0.2}, {20., 0.1}}}, {"reset", true}},
            {{"reset", false}},
            {{"profile", "invalid"}},
            {{"adaptive", {{"quality", "0.5"}}}},
            {{"adaptive", {{"quality", 0.5}, {"unexpected", 1}}}},
            {{"smooth", {{"radius", 1.5}}}},
            {{"smooth", {{"radius", 3}, {"keep_min", 1}}}},
            {{"extra", true}}}) {
        INFO("Body: " << body.dump());
        set_error(fixture, route, object_id, body, http::status::bad_request, "invalid_request");
    }
    require_error(fixture.request(http::verb::post, route + "/set_layer_height_profile/" + object_id,
        "{", "application/json"), http::status::bad_request, "invalid_json");
    require_error(fixture.request(http::verb::post, route + "/set_layer_height_profile/" + object_id),
        http::status::bad_request, "invalid_request");

    const nlohmann::json too_few = set_error(fixture, route, object_id,
        {{"profile", {{0., 0.2}}}}, http::status::unprocessable_entity, "invalid_layer_height_profile");
    CHECK_FALSE(too_few.contains("details"));
    CHECK(too_few.at("message").get<std::string>().find("too_few_points") != std::string::npos);
    const std::vector<std::pair<nlohmann::json, nlohmann::json>> point_refusals = {
        {{{"profile", {{0.1, 0.2}, {20., 0.1}}}}, {{"point", 0}, {"reason", "first_z"}}},
        {{{"profile", {{0., 0.2}, {5., 0.2}, {4., 0.1}, {20., 0.1}}}}, {{"point", 2}, {"reason", "z_order"}}},
        {{{"profile", {{0., 0.2}, {5., 1.}, {20., 0.1}}}}, {{"point", 1}, {"reason", "height_range"}}},
        {{{"profile", {{0., 0.2}, {19.5, 0.1}}}}, {{"point", 1}, {"reason", "last_z"}}}};
    for (const std::pair<nlohmann::json, nlohmann::json> &refusal : point_refusals) {
        const nlohmann::json error = set_error(fixture, route, object_id, refusal.first,
            http::status::unprocessable_entity, "invalid_layer_height_profile");
        REQUIRE(error.at("details").size() == 1);
        CHECK(error.at("details").at(0) == refusal.second);
    }
    set_error(fixture, route, object_id, {{"adaptive", {{"quality", 1.5}}}},
        http::status::unprocessable_entity, "invalid_layer_height_profile");
    const nlohmann::json bounds = get_layer_height_profile(fixture, route, object_id);
    set_error(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}, {"min_height", 0.01}, {"max_height", bounds.at("max_layer_height")}}}},
        http::status::unprocessable_entity, "invalid_layer_height_profile");
    set_error(fixture, route, object_id, {{"smooth", {{"radius", 11}}}},
        http::status::unprocessable_entity, "invalid_layer_height_profile");

    set_error(fixture, route, part_id, {{"reset", true}}, http::status::unprocessable_entity,
        "invalid_layer_height_profile");
    set_error(fixture, route, instance_id, {{"reset", true}}, http::status::unprocessable_entity,
        "invalid_layer_height_profile");
    require_error(post_json(fixture, "/session/unknown/set_layer_height_profile/object", {{"reset", true}}),
        http::status::not_found, "session_not_found");
    require_error(post_json(fixture, route + "/set_layer_height_profile/unknown", {{"reset", true}}),
        http::status::not_found, "entity_not_found");
    require_error(fixture.request(http::verb::get, route + "/get_layer_height_profile/unknown"),
        http::status::not_found, "entity_not_found");
    require_error(fixture.request(http::verb::get, route + "/get_layer_height_profile/" + part_id),
        http::status::unprocessable_entity, "invalid_layer_height_profile");
    require_error(fixture.request(http::verb::get,
        route + "/get_layer_height_profile/" + object_id + "?x=1"), http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::get,
        route + "/get_layer_height_profile/" + object_id, "{}", "application/json"),
        http::status::bad_request, "invalid_request");
}

// Store a ten-micron profile through HTTP using the shared small-nozzle limits, retaining it after an invalid replacement.
TEST_CASE("REST small-nozzle profiles round-trip ten-micron layers", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    set_options(session, {{"nozzle_diameter", "0.05"}, {"min_layer_height", "0"},
        {"max_layer_height", "0"}, {"first_layer_height", "0.04"}, {"layer_height", "0.025"}, {"z_step", "0"}});
    const nlohmann::json limits = get_layer_height_profile(fixture, route, object_id);
    CHECK(limits.at("min_layer_height").get<double>() == Approx(0.01));
    CHECK(limits.at("max_layer_height").get<double>() == Approx(0.0375));
    const nlohmann::json points = nlohmann::json::array({{0., 0.01}, {20., 0.01}});
    const nlohmann::json saved = set_layer_height_profile(fixture, route, object_id, {{"profile", points}});
    CHECK(saved.at("valid") == true);
    CHECK(saved.at("applied") == true);
    CHECK(saved.at("profile") == points);
    CHECK(get_layer_height_profile(fixture, route, object_id) == saved);
    CHECK(list_scene(fixture, route).at("objects").at(0).at("variable_layer_height") == true);
    set_error(fixture, route, object_id, {{"profile", {{0., 0.0005}, {20., 0.0005}}}},
        http::status::unprocessable_entity, "invalid_layer_height_profile");
    CHECK(get_layer_height_profile(fixture, route, object_id) == saved);
}

// Mutate one local object through the HTTP routes and verify response data, no-op revisions, and its list flag.
TEST_CASE("REST layer height profiles replace and reset", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json initial = get_layer_height_profile(fixture, route, object_id);
    CHECK(initial.at("profile").empty());
    CHECK(initial.at("valid") == true);
    CHECK(initial.at("applied") == false);
    CHECK(list_scene(fixture, route).at("objects").at(0).at("variable_layer_height") == false);

    const nlohmann::json points = nlohmann::json::array({{0., 0.2}, {5., 0.1}, {20., 0.1}});
    const nlohmann::json explicit_result = set_layer_height_profile(fixture, route, object_id, {{"profile", points}});
    CHECK(explicit_result.at("applied") == true);
    CHECK(explicit_result.at("valid") == true);
    REQUIRE(explicit_result.at("profile").size() == 3);
    CHECK(get_layer_height_profile(fixture, route, object_id) == explicit_result);
    const std::uint64_t revision = session->snapshot().revision;
    const nlohmann::json normalized_points = nlohmann::json::array({{0., 0.2}, {5., 0.1}, {20.0005, 0.1}});
    CHECK(set_layer_height_profile(fixture, route, object_id, {{"profile", normalized_points}}) == explicit_result);
    CHECK(session->snapshot().revision == revision);
    CHECK(list_scene(fixture, route).at("objects").at(0).at("variable_layer_height") == true);

    const nlohmann::json adaptive = set_layer_height_profile(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}, {"min_height", 0.08}, {"max_height", 0.28}}}});
    CHECK_FALSE(adaptive.at("profile").empty());
    const nlohmann::json adaptive_get = get_layer_height_profile(fixture, route, object_id);
    CHECK(adaptive_get == adaptive);
    const std::uint64_t adaptive_revision = session->snapshot().revision;
    CHECK(set_layer_height_profile(fixture, route, object_id,
        {{"profile", adaptive_get.at("profile")}}) == adaptive_get);
    CHECK(session->snapshot().revision == adaptive_revision);
    const nlohmann::json smooth = set_layer_height_profile(fixture, route, object_id,
        {{"smooth", {{"radius", 3}, {"keep_min", true}}}});
    CHECK_FALSE(smooth.at("profile").empty());
    CHECK(smooth.at("valid") == true);
    const std::uint64_t smooth_revision = session->snapshot().revision;
    CHECK(set_layer_height_profile(fixture, route, object_id,
        {{"profile", smooth.at("profile")}}) == smooth);
    CHECK(session->snapshot().revision == smooth_revision);
    const nlohmann::json reset = set_layer_height_profile(fixture, route, object_id, {{"reset", true}});
    CHECK(reset.at("profile").empty());
    CHECK(reset.at("valid") == true);
    CHECK(reset.at("applied") == false);
    CHECK(list_scene(fixture, route).at("objects").at(0).at("variable_layer_height") == false);
}

// Exercise omitted/echoed printer bounds and confirm the adaptive engine's rounded minimum in REST output.
TEST_CASE("REST adaptive bounds constrain profiles within printer limits", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const nlohmann::json limits = get_layer_height_profile(fixture, route, object_id);

    const nlohmann::json defaults = set_layer_height_profile(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}}}});
    CHECK(defaults.at("valid") == true);
    CHECK_FALSE(defaults.at("profile").empty());
    const nlohmann::json echoed = set_layer_height_profile(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}, {"min_height", limits.at("min_layer_height")},
            {"max_height", limits.at("max_layer_height")}}}});
    CHECK(echoed.at("valid") == true);
    CHECK_FALSE(echoed.at("profile").empty());
    set_error(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}, {"max_height", limits.at("max_layer_height").get<double>() + 0.01}}}},
        http::status::unprocessable_entity, "invalid_layer_height_profile");

    set_options(session, {{"z_step", "0.005"}});
    const nlohmann::json step_limits = get_layer_height_profile(fixture, route, object_id);
    const double engine_minimum = std::clamp(check_z_step(0.081, 0.005),
        step_limits.at("min_layer_height").get<double>(), step_limits.at("max_layer_height").get<double>());
    const nlohmann::json stepped = set_layer_height_profile(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}, {"min_height", 0.081}, {"max_height", 0.081}}}});
    CHECK(stepped.at("valid") == true);
    const nlohmann::json &profile = stepped.at("profile");
    REQUIRE(profile.size() > 3);
    for (std::size_t point = 2; point + 1 < profile.size(); ++point) {
        CHECK(profile.at(point).at(1).get<double>() >= engine_minimum - 1e-6);
        CHECK(profile.at(point).at(1).get<double>() <= check_z_step(0.081, 0.005) + 1e-6);
    }
}

// Confirm fixed first-layer anchors above the regular maximum can be read and posted back without a revision change.
TEST_CASE("REST adaptive profiles round-trip a fixed first layer above the maximum", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    set_options(session, {{"nozzle_diameter", "0.25"}, {"first_layer_height", "0.2"}});
    const SessionData data = session->data_snapshot().value();
    const ModelObject *object = data.scene->object_ptrs().front();
    const std::shared_ptr<SlicingParameters> parameters = PrintObject::slicing_parameters(
        data.configuration, *object, static_cast<float>(object->instance_bounding_box(0).max.z()));
    REQUIRE(parameters->first_object_layer_height > parameters->max_layer_height);

    const nlohmann::json adaptive = set_layer_height_profile(fixture, route, object_id,
        {{"adaptive", {{"quality", 0.5}}}});
    CHECK(adaptive.at("valid") == true);
    const nlohmann::json retrieved = get_layer_height_profile(fixture, route, object_id);
    CHECK(retrieved == adaptive);
    const std::uint64_t revision = session->snapshot().revision;
    CHECK(set_layer_height_profile(fixture, route, object_id,
        {{"profile", retrieved.at("profile")}}) == retrieved);
    CHECK(session->snapshot().revision == revision);
}

// Store a bounds-invalid profile through a private scene copy and verify GET reports retention while smoothing and reset avoid layer generation.
TEST_CASE("REST invalid stored layer height profiles are reported safely", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();

    SessionData invalid_data = session->data_snapshot().value();
    invalid_data.scene = std::make_shared<Model>(*invalid_data.scene);
    invalid_data.scene->object_ptrs().front()->layer_height_profile.set({0., 0.2, 5., 1., 20., 0.1});
    const SessionContext::MutationToken invalid_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(invalid_token, std::move(invalid_data), true, SessionContext::Clock::now()));

    const nlohmann::json invalid = get_layer_height_profile(fixture, route, object_id);
    CHECK(invalid.at("valid") == false);
    CHECK(invalid.at("applied") == true);
    CHECK_FALSE(invalid.contains("layers"));
    REQUIRE(invalid.at("errors").size() == 1);
    CHECK(invalid.at("errors").at(0) == nlohmann::json{{"point", 1}, {"reason", "height_range"}});
    const nlohmann::json smooth_error = set_error(fixture, route, object_id,
        {{"smooth", {{"radius", 3}}}}, http::status::unprocessable_entity, "invalid_layer_height_profile");
    REQUIRE(smooth_error.at("details").size() == 1);
    CHECK(smooth_error.at("details").at(0) == nlohmann::json{{"point", 1}, {"reason", "height_range"}});

    const nlohmann::json reset = set_layer_height_profile(fixture, route, object_id, {{"reset", true}});
    CHECK(reset.at("valid") == true);
    CHECK_FALSE(reset.contains("errors"));
    CHECK(reset.contains("layers"));

    SessionData first_z_data = session->data_snapshot().value();
    first_z_data.scene = std::make_shared<Model>(*first_z_data.scene);
    first_z_data.scene->object_ptrs().front()->layer_height_profile.set({1., 0.2, 20., 0.1});
    const SessionContext::MutationToken first_z_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(first_z_token, std::move(first_z_data), true, SessionContext::Clock::now()));
    const nlohmann::json invalid_first_z = get_layer_height_profile(fixture, route, object_id);
    CHECK(invalid_first_z.at("valid") == false);
    CHECK_FALSE(invalid_first_z.contains("layers"));
    REQUIRE(invalid_first_z.at("errors").size() == 1);
    CHECK(invalid_first_z.at("errors").at(0) == nlohmann::json{{"point", 0}, {"reason", "first_z"}});

    set_options(session, {{"nozzle_diameter", "0.25"}, {"first_layer_height", "0.2"}});
    SessionData unsafe_data = session->data_snapshot().value();
    unsafe_data.scene = std::make_shared<Model>(*unsafe_data.scene);
    unsafe_data.scene->object_ptrs().front()->layer_height_profile.set({0., 0.2, 20., 0.1});
    const SessionContext::MutationToken unsafe_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(unsafe_token, std::move(unsafe_data), true, SessionContext::Clock::now()));
    const nlohmann::json unsafe = get_layer_height_profile(fixture, route, object_id);
    CHECK(unsafe.at("valid") == false);
    CHECK(unsafe.at("applied") == true);
    CHECK_FALSE(unsafe.contains("layers"));
    REQUIRE(unsafe.at("errors").size() == 1);
    CHECK(unsafe.at("errors").at(0) == nlohmann::json{{"point", 0}, {"reason", "height_range"}});
}

// Save and reload a local 3MF project to compare profile responses, including a two-point profile round trip.
TEST_CASE("REST layer height profiles persist in projects", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json before = set_layer_height_profile(fixture, route, object_id,
        {{"profile", nlohmann::json::array({{0., 0.2}, {5., 0.2}, {5., 0.1}, {20., 0.1}})}});

    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    const std::string restored_session_id = create_session(fixture);
    const std::string restored_route = "/session/" + restored_session_id;
    REQUIRE(object_ids(upload_model(fixture, restored_route + "/load_project", "profile.3mf", saved.body())).size() == 1);
    const std::string restored_object = list_scene(fixture, restored_route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json restored = get_layer_height_profile(fixture, restored_route, restored_object);
    CHECK(restored == before);
    const std::shared_ptr<SessionContext> restored_session = find_session(fixture, restored_session_id);
    const std::uint64_t revision = restored_session->snapshot().revision;
    CHECK(set_layer_height_profile(fixture, restored_route, restored_object,
        {{"profile", restored.at("profile")}}) == restored);
    CHECK(restored_session->snapshot().revision == revision);

    const double object_height = get_layer_height_profile(fixture, route, object_id).at("object_height").get<double>();
    const nlohmann::json two_point = set_layer_height_profile(fixture, route, object_id,
        {{"profile", nlohmann::json::array({{0., 0.2}, {object_height, 0.1}})}});
    REQUIRE(two_point.at("valid") == true);
    REQUIRE(two_point.at("applied") == true);
    const HttpResponseMessage two_point_saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(two_point_saved.result() == http::status::ok);
    const std::string two_point_route = "/session/" + create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, two_point_route + "/load_project", "two-point-profile.3mf", two_point_saved.body())).size() == 1);
    const std::string two_point_object = list_scene(fixture, two_point_route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json two_point_restored = get_layer_height_profile(fixture, two_point_route, two_point_object);
    CHECK(two_point_restored == two_point);
    CHECK(two_point_restored.at("valid") == true);
    CHECK(two_point_restored.at("applied") == true);
}

// Persist a three-point profile on a scaled cube and compare valid engine-retained responses and layer tops before and after 3MF round-trip.
TEST_CASE("REST layer height profiles persist non-round object heights", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();

    SessionData scaled_data = session->data_snapshot().value();
    scaled_data.scene = std::make_shared<Model>(*scaled_data.scene);
    ModelObject *scaled_object = scaled_data.scene->object_ptrs().front();
    scaled_object->instances.front()->set_scaling_factor(Vec3d(1., 1., 20.1234567 / 20.));
    scaled_object->invalidate_bounding_box();
    const SessionContext::MutationToken scaled_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(scaled_token, std::move(scaled_data), true, SessionContext::Clock::now()));

    const double object_height = get_layer_height_profile(fixture, route, object_id).at("object_height").get<double>();
    CHECK(object_height == Approx(20.1234567).margin(1e-5));
    const nlohmann::json before = set_layer_height_profile(fixture, route, object_id,
        {{"profile", nlohmann::json::array({{0., 0.2}, {5.123456, 0.15}, {object_height, 0.1}})}});
    CHECK(before.at("valid") == true);
    CHECK(before.at("applied") == true);

    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    const std::string restored_session_id = create_session(fixture);
    const std::string restored_route = "/session/" + restored_session_id;
    REQUIRE(object_ids(upload_model(fixture, restored_route + "/load_project", "scaled-profile.3mf", saved.body())).size() == 1);
    const std::string restored_object = list_scene(fixture, restored_route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json restored = get_layer_height_profile(fixture, restored_route, restored_object);
    CHECK(restored.at("valid") == true);
    CHECK(restored.at("applied") == true);
    const std::vector<double> before_layers = before.at("layers").get<std::vector<double>>();
    const std::vector<double> restored_layers = restored.at("layers").get<std::vector<double>>();
    REQUIRE(restored_layers.size() == before_layers.size());
    for (std::size_t index = 0; index < before_layers.size(); ++index)
        CHECK(restored_layers[index] == Approx(before_layers[index]).margin(1e-5));
}

// Validate a range-derived profile ending within EPSILON of the object top and keep failures profile-wide when storage is empty.
TEST_CASE("REST range profiles accept near-top endpoints and omit point indices", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const double object_height = get_layer_height_profile(fixture, route, object_id).at("object_height").get<double>();
    set_options(session, {{"z_step", "0.00001"}});

    SessionData near_top_data = session->data_snapshot().value();
    near_top_data.scene = std::make_shared<Model>(*near_top_data.scene);
    ModelObject *near_top_object = near_top_data.scene->object_ptrs().front();
    near_top_object->layer_config_ranges[{0., object_height - EPSILON * 0.5}]
        .set_key_value("layer_height", new ConfigOptionFloat(0.1));
    const SessionContext::MutationToken near_top_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(near_top_token, std::move(near_top_data), true, SessionContext::Clock::now()));

    const nlohmann::json near_top = get_layer_height_profile(fixture, route, object_id);
    CHECK(near_top.at("valid") == true);
    CHECK(near_top.at("profile").empty());
    CHECK(near_top.at("applied") == false);
    CHECK_FALSE(near_top.at("layers").empty());

    SessionData invalid_range_data = session->data_snapshot().value();
    invalid_range_data.scene = std::make_shared<Model>(*invalid_range_data.scene);
    ModelObject *invalid_range_object = invalid_range_data.scene->object_ptrs().front();
    invalid_range_object->layer_config_ranges.clear();
    invalid_range_object->layer_config_ranges[{0., object_height}]
        .set_key_value("layer_height", new ConfigOptionFloat(1.));
    const SessionContext::MutationToken invalid_range_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(invalid_range_token, std::move(invalid_range_data), true, SessionContext::Clock::now()));

    const nlohmann::json invalid_range = get_layer_height_profile(fixture, route, object_id);
    CHECK(invalid_range.at("valid") == false);
    CHECK(invalid_range.at("applied") == false);
    CHECK_FALSE(invalid_range.contains("layers"));
    REQUIRE(invalid_range.at("errors").size() == 1);
    CHECK(invalid_range.at("errors").at(0) == nlohmann::json{{"reason", "height_range"}});
    const nlohmann::json smooth_error = set_error(fixture, route, object_id,
        {{"smooth", {{"radius", 3}}}}, http::status::unprocessable_entity, "invalid_layer_height_profile");
    CHECK(smooth_error.at("details") == invalid_range.at("errors"));
}

// Store stale-top and odd-length profiles ignored by the engine; GET and smoothing must report identical failures without producing layers.
TEST_CASE("REST smoothing rejects stored profiles ignored by the engine", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();

    SessionData stale_data = session->data_snapshot().value();
    stale_data.scene = std::make_shared<Model>(*stale_data.scene);
    ModelObject *stale_object = stale_data.scene->object_ptrs().front();
    stale_object->instances.front()->set_scaling_factor(Vec3d(1., 1., 2.));
    stale_object->invalidate_bounding_box();
    stale_object->layer_height_profile.set({0., 0.2, 5., 0.1, 20., 0.1});
    const SessionContext::MutationToken stale_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(stale_token, std::move(stale_data), true, SessionContext::Clock::now()));

    const nlohmann::json stale = get_layer_height_profile(fixture, route, object_id);
    CHECK(stale.at("valid") == false);
    CHECK(stale.at("applied") == false);
    CHECK_FALSE(stale.contains("layers"));
    REQUIRE(stale.at("errors").size() == 1);
    CHECK(stale.at("errors").at(0) == nlohmann::json{{"point", 2}, {"reason", "last_z"}});
    const nlohmann::json stale_smooth_error = set_error(fixture, route, object_id,
        {{"smooth", {{"radius", 3}}}}, http::status::unprocessable_entity, "invalid_layer_height_profile");
    CHECK(stale_smooth_error.at("details") == stale.at("errors"));

    SessionData odd_data = session->data_snapshot().value();
    odd_data.scene = std::make_shared<Model>(*odd_data.scene);
    odd_data.scene->object_ptrs().front()->layer_height_profile.set({0., 0.2, 5.});
    const SessionContext::MutationToken odd_token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(odd_token, std::move(odd_data), true, SessionContext::Clock::now()));

    const nlohmann::json odd = get_layer_height_profile(fixture, route, object_id);
    CHECK(odd.at("valid") == false);
    CHECK(odd.at("applied") == false);
    CHECK_FALSE(odd.contains("layers"));
    REQUIRE(odd.at("errors").size() == 1);
    CHECK(odd.at("errors").at(0) == nlohmann::json{{"reason", "invalid_length"}});
    const nlohmann::json odd_smooth_error = set_error(fixture, route, object_id,
        {{"smooth", {{"radius", 3}}}}, http::status::unprocessable_entity, "invalid_layer_height_profile");
    CHECK(odd_smooth_error.at("details") == odd.at("errors"));
}

// Exercise read and mutation routes around active work and an observer-driven competing session revision.
TEST_CASE("REST layer height profiles are refused while slicing and on concurrent changes", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json body = {{"profile", nlohmann::json::array({{0., 0.2}, {20., 0.1}})}};

    const SessionContext::WorkToken work = session->begin_work(SessionContext::Clock::now()).value();
    require_error(post_json(fixture, route + "/set_layer_height_profile/" + object_id, body),
        http::status::conflict, "session_busy");
    CHECK(get_layer_height_profile(fixture, route, object_id).at("profile").empty());
    REQUIRE(session->cancel_work(work, SessionContext::Clock::now()));

    fixture.stop();
    const std::shared_ptr<std::shared_ptr<SessionContext>> observed =
        std::make_shared<std::shared_ptr<SessionContext>>();
    // Commit a preference change between profile preparation and publication.
    fixture.set_scene_edit_observer([observed]() {
        const std::shared_ptr<SessionContext> current = *observed;
        if (!current)
            return;
        const std::optional<SessionContext::MutationToken> token = current->begin_mutation();
        if (!token)
            return;
        SessionData data = current->data_snapshot().value();
        data.preferences["layer_height_observer"] = true;
        current->commit_mutation(*token, std::move(data), false, SessionContext::Clock::now());
    });
    fixture.start();
    const std::string observed_route = session_with_cube(fixture);
    const std::string observed_id = observed_route.substr(9);
    const std::string observed_object = list_scene(fixture, observed_route).at("objects").at(0).at("id").get<std::string>();
    *observed = find_session(fixture, observed_id);
    require_error(post_json(fixture, observed_route + "/set_layer_height_profile/" + observed_object, body),
        http::status::conflict, "session_changed");
    CHECK(get_layer_height_profile(fixture, observed_route, observed_object).at("profile").empty());
}

// Slice one local object and compare emitted layer markers with REST layers and explicit profile height expectations.
TEST_CASE("REST layer height profiles reach the G-code", "[rest_layer_height]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::string session_id = route.substr(9);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = list_scene(fixture, route).at("objects").at(0).at("id").get<std::string>();
    set_options(session, {{"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"seam_visibility", "0"}});
    set_layer_height_profile(fixture, route, object_id,
        {{"profile", nlohmann::json::array({{0., 0.2}, {5., 0.2}, {5., 0.1}, {20., 0.1}})}});
    const std::vector<double> expected_layers =
        get_layer_height_profile(fixture, route, object_id).at("layers").get<std::vector<double>>();
    const std::string gcode = slice_gcode(fixture, route, session);
    const std::vector<double> z_markers = gcode_z_markers(gcode);
    REQUIRE(z_markers.size() == expected_layers.size());
    for (std::size_t index = 0; index < z_markers.size(); ++index)
        CHECK(z_markers[index] == Approx(expected_layers[index]).margin(1e-3));

    const std::vector<double> heights = gcode_height_markers(gcode);
    REQUIRE(heights.size() == z_markers.size());
    for (std::size_t index = 0; index < heights.size(); ++index)
        CHECK(heights[index] == Approx(index == 0 || z_markers[index] <= 5. + 1e-3 ? 0.2 : 0.1).margin(1e-3));
}
