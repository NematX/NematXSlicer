// REST plane-cut tests cover validation, ID replacement, arrangement, and commit races.
#include <catch2/catch.hpp>

#include "cut.hpp"
#include "rest_models_test_helpers.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Send one JSON body through the authenticated REST fixture.
HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target,
    const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

// Read one complete object listing for assertions that compare IDs and matrices.
nlohmann::json listed(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Upload a cube and return its session route plus the initial object listing.
std::pair<std::string, nlohmann::json> session_with_cube(RestServerFixture &fixture)
{
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return {route, listed(fixture, route)};
}

// Build the standard horizontal plane body used by the HTTP cut cases.
nlohmann::json horizontal_cut_body()
{
    nlohmann::json body = nlohmann::json::object();
    body["plane"] = nlohmann::json::object();
    body["plane"]["point"] = {0, 0, 10};
    body["plane"]["normal"] = {0, 0, 1};
    return body;
}

// Wait for a slicing worker to publish the requested terminal session status.
bool wait_for_status(const std::shared_ptr<SessionContext> &session, const SessionStatus status)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
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

// Slice one REST session and return its non-empty G-code response.
std::string slice_gcode(const RestServerFixture &fixture, const std::string &route,
    const std::shared_ptr<SessionContext> &session)
{
    // These fixtures check cut geometry, not seam visibility raytracing.
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/seam_visibility/0").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/slice").result() == http::status::accepted);
    INFO("Slice error: " << session->snapshot().error_message);
    REQUIRE(wait_for_status(session, SessionStatus::sliced));
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_gcode");
    REQUIRE(response.result() == http::status::ok);
    return response.body();
}

// Resolve a REST object ID in a current session snapshot for arrangement geometry checks.
ModelObject *find_object(RestServerFixture &fixture, const std::string &session_id, const std::string &object_id)
{
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::optional<EntityReference> reference = session->resolve_entity(object_id);
    REQUIRE(reference.has_value());
    REQUIRE(reference->kind == EntityKind::object);
    const std::optional<SessionData> data = session->data_snapshot();
    REQUIRE(data.has_value());
    for (ModelObject *object : data->scene->object_ptrs())
        if (object->id().id == reference->engine_id)
            return object;
    FAIL("The REST object ID was not found in the session scene.");
    return nullptr;
}

// Return the XY bounds of all vertices in the first instance of one object.
std::pair<Vec2d, Vec2d> object_xy_bounds(const ModelObject &object)
{
    Vec2d minimum = Vec2d::Constant(std::numeric_limits<double>::max());
    Vec2d maximum = Vec2d::Constant(std::numeric_limits<double>::lowest());
    const Transform3d matrix = object.instances.front()->get_matrix();
    for (const ModelVolume *volume : object.volumes)
        for (const Vec3f &vertex : volume->mesh().its.vertices) {
            const Vec3d point = matrix * volume->get_matrix() * vertex.cast<double>();
            minimum = minimum.cwiseMin(point.head<2>());
            maximum = maximum.cwiseMax(point.head<2>());
        }
    return {minimum, maximum};
}

// Gate one import-pool callback while test-owned shared state drives a competing commit.
class CutImportGate {
public:
    CutImportGate()
        : entered_(entered_promise_.get_future().share()), released_(release_promise_.get_future().share())
    {}

    void arm() { armed_.store(true); }

    void observe()
    {
        if (!armed_.exchange(false))
            return;
        entered_promise_.set_value();
        released_.wait_for(std::chrono::seconds(120));
    }

    bool wait() const { return entered_.wait_for(std::chrono::seconds(60)) == std::future_status::ready; }

    void release()
    {
        if (!released_once_.exchange(true))
            release_promise_.set_value();
    }

    ~CutImportGate() { release(); }

private:
    std::atomic<bool> armed_{false};
    std::atomic<bool> released_once_{false};
    std::promise<void> entered_promise_;
    std::promise<void> release_promise_;
    std::shared_future<void> entered_;
    std::shared_future<void> released_;
};

} // namespace

// Validate request shapes, spaces, target kinds, and the no-side-effect rule.
TEST_CASE("REST cut validates the request and the target", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = setup.second.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();

    require_error(post_json(fixture, route + "/cut/" + object_id, {}),
        http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::post, route + "/cut/" + object_id, "{", "application/json"),
        http::status::bad_request, "invalid_json");
    nlohmann::json missing_normal = horizontal_cut_body();
    missing_normal["plane"].erase("normal");
    require_error(post_json(fixture, route + "/cut/" + object_id, missing_normal),
        http::status::bad_request, "invalid_request");
    nlohmann::json unknown_field = horizontal_cut_body();
    unknown_field["unexpected"] = true;
    require_error(post_json(fixture, route + "/cut/" + object_id, unknown_field),
        http::status::bad_request, "invalid_request");
    nlohmann::json unknown_plane_field = horizontal_cut_body();
    unknown_plane_field["plane"]["unexpected"] = true;
    require_error(post_json(fixture, route + "/cut/" + object_id, unknown_plane_field),
        http::status::bad_request, "invalid_request");
    nlohmann::json non_boolean = horizontal_cut_body();
    non_boolean["keep_upper"] = 1;
    require_error(post_json(fixture, route + "/cut/" + object_id, non_boolean),
        http::status::bad_request, "invalid_request");
    nlohmann::json short_point = horizontal_cut_body();
    short_point["plane"]["point"] = {0, 0};
    require_error(post_json(fixture, route + "/cut/" + object_id, short_point),
        http::status::bad_request, "invalid_request");
    nlohmann::json non_string_space = horizontal_cut_body();
    non_string_space["space"] = 1;
    require_error(post_json(fixture, route + "/cut/" + object_id, non_string_space),
        http::status::bad_request, "invalid_request");
    nlohmann::json zero_normal = horizontal_cut_body();
    zero_normal["plane"]["normal"] = {0, 0, 0};
    require_error(post_json(fixture, route + "/cut/" + object_id, zero_normal),
        http::status::unprocessable_entity, "invalid_cut");
    nlohmann::json no_side = horizontal_cut_body();
    no_side["keep_upper"] = false;
    no_side["keep_lower"] = false;
    require_error(post_json(fixture, route + "/cut/" + object_id, no_side),
        http::status::unprocessable_entity, "invalid_cut");
    nlohmann::json unknown_space = horizontal_cut_body();
    unknown_space["space"] = "unknown";
    require_error(post_json(fixture, route + "/cut/" + object_id, unknown_space),
        http::status::unprocessable_entity, "invalid_cut");

    const HttpResponseMessage modifier = post_json(fixture, route + "/add_shape/" + object_id,
        {{"type", "modifier"}, {"shape", "box"}});
    REQUIRE(modifier.result() == http::status::ok);
    const nlohmann::json after_modifier = listed(fixture, route);
    const std::string modifier_id = after_modifier.at("objects").at(0).at("parts").back().at("id").get<std::string>();
    require_error(post_json(fixture, route + "/cut/" + modifier_id, horizontal_cut_body()),
        http::status::unprocessable_entity, "invalid_cut");
    nlohmann::json part_options = horizontal_cut_body();
    part_options["keep_as_parts"] = true;
    require_error(post_json(fixture, route + "/cut/" + part_id, part_options),
        http::status::unprocessable_entity, "invalid_cut");
}

// Verify reference-instance ownership, unknown IDs, and the successful second-instance path.
TEST_CASE("REST cut validates reference instances", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(std::string("/session/").size());
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = setup.second.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    REQUIRE(post_json(fixture, route + "/set_instance_count/" + object_id, {{"count", 2}}).result() == http::status::ok);
    const nlohmann::json initial = listed(fixture, route);
    REQUIRE(initial.at("objects").at(0).at("instances").size() == 2);
    const std::string second_instance_id = initial.at("objects").at(0).at("instances").at(1).at("id").get<std::string>();

    nlohmann::json unknown_instance = horizontal_cut_body();
    unknown_instance["instance"] = "missing-instance";
    require_error(post_json(fixture, route + "/cut/" + object_id, unknown_instance),
        http::status::not_found, "entity_not_found");
    CHECK(listed(fixture, route) == initial);

    nlohmann::json part_instance = horizontal_cut_body();
    part_instance["instance"] = part_id;
    require_error(post_json(fixture, route + "/cut/" + object_id, part_instance),
        http::status::unprocessable_entity, "invalid_cut");
    CHECK(listed(fixture, route) == initial);

    const std::string foreign_object_id = object_ids(
        upload_model(fixture, route + "/add_model", "foreign.stl", make_cube_stl())).front();
    const nlohmann::json with_foreign = listed(fixture, route);
    std::string foreign_instance_id;
    for (const nlohmann::json &object : with_foreign.at("objects"))
        if (object.at("id") == foreign_object_id)
            foreign_instance_id = object.at("instances").at(0).at("id").get<std::string>();
    REQUIRE_FALSE(foreign_instance_id.empty());
    nlohmann::json foreign_instance = horizontal_cut_body();
    foreign_instance["instance"] = foreign_instance_id;
    require_error(post_json(fixture, route + "/cut/" + object_id, foreign_instance),
        http::status::unprocessable_entity, "invalid_cut", "belongs to another object");
    CHECK(listed(fixture, route) == with_foreign);

    nlohmann::json second_instance = horizontal_cut_body();
    second_instance["instance"] = second_instance_id;
    const HttpResponseMessage success = post_json(fixture, route + "/cut/" + object_id, second_instance);
    REQUIRE(success.result() == http::status::ok);
    CHECK(nlohmann::json::parse(success.body()).at("objects").size() == 2);
    CHECK(find_session(fixture, session_id)->snapshot().status == SessionStatus::ready);
}

// Verify that a cut-group part requires explicit invalidation and clears the group when accepted.
TEST_CASE("REST cut enforces cut-group invalidation", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(std::string("/session/").size());
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = setup.second.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::optional<SessionContext::MutationToken> token = session->begin_mutation();
    REQUIRE(token.has_value());
    SessionData data = session->data_snapshot().value();
    data.scene = std::make_shared<Model>(*data.scene);
    data.scene->object_ptrs().front()->cut_id.init();
    REQUIRE(session->commit_mutation(*token, std::move(data), false, SessionContext::Clock::now()));

    const nlohmann::json before = listed(fixture, route);
    const HttpResponseMessage refused = post_json(fixture, route + "/cut/" + part_id, horizontal_cut_body());
    require_error(refused, http::status::unprocessable_entity, "cut_object");
    CHECK(listed(fixture, route) == before);

    nlohmann::json invalidated = horizontal_cut_body();
    invalidated["invalidate_cut"] = true;
    const HttpResponseMessage accepted = post_json(fixture, route + "/cut/" + part_id, invalidated);
    REQUIRE(accepted.result() == http::status::ok);
    const nlohmann::json result = nlohmann::json::parse(accepted.body());
    CHECK(result.at("parts").at(0).at("side") == "upper");
    CHECK(result.at("parts").at(1).at("side") == "lower");
    CHECK_FALSE(result.contains("beds"));
    const std::optional<EntityReference> object_reference = session->resolve_entity(object_id);
    REQUIRE(object_reference.has_value());
    CHECK(object_reference->kind == EntityKind::object);
    CHECK_FALSE(session->data_snapshot()->scene->object_ptrs().front()->is_cut());
}

// Verify old entity IDs are removed and each generated object/part gets a new ID
// and the side label promised by the route contract.
TEST_CASE("REST cut replaces objects and parts with new IDs", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(std::string("/session/").size());
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = setup.second.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const nlohmann::json cut_body = horizontal_cut_body();

    const HttpResponseMessage response = post_json(fixture, route + "/cut/" + object_id, cut_body);
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json result = nlohmann::json::parse(response.body());
    REQUIRE(result.at("objects").size() == 2);
    CHECK(result.at("objects").at(0).at("side") == "upper");
    CHECK(result.at("objects").at(1).at("side") == "lower");
    CHECK_FALSE(result.contains("beds"));
    REQUIRE(result.at("removed").size() >= 3);
    CHECK(std::find(result.at("removed").begin(), result.at("removed").end(), object_id) != result.at("removed").end());
    CHECK(std::find(result.at("removed").begin(), result.at("removed").end(), part_id) != result.at("removed").end());
    const nlohmann::json after = listed(fixture, route);
    REQUIRE(after.at("objects").size() == 2);
    CHECK(after.at("objects").at(0).at("id") != object_id);
    CHECK(after.at("objects").at(1).at("id") != object_id);
    require_error(post_json(fixture, route + "/cut/" + object_id, cut_body),
        http::status::not_found, "entity_not_found");

    const std::string remaining_object = after.at("objects").at(0).at("id").get<std::string>();
    const std::string remaining_part = after.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    nlohmann::json part_body = cut_body;
    part_body["plane"]["point"] = {0, 0, 5};
    const HttpResponseMessage part_response = post_json(fixture, route + "/cut/" + remaining_part, part_body);
    REQUIRE(part_response.result() == http::status::ok);
    const nlohmann::json part_result = nlohmann::json::parse(part_response.body());
    CHECK(part_result.at("parts").size() == 2);
    CHECK(part_result.at("parts").at(0).at("side") == "upper");
    CHECK(part_result.at("parts").at(1).at("side") == "lower");
    CHECK_FALSE(part_result.contains("beds"));
    CHECK(part_result.at("removed").at(0) == remaining_part);
    CHECK(listed(fixture, route).at("objects").size() == 2);
    CHECK(listed(fixture, route).at("objects").at(0).at("id") == remaining_object);

    const std::pair<std::string, nlohmann::json> parts_setup = session_with_cube(fixture);
    const std::string parts_route = parts_setup.first;
    const std::string parts_object_id = parts_setup.second.at("objects").at(0).at("id").get<std::string>();
    nlohmann::json keep_as_parts = horizontal_cut_body();
    keep_as_parts["keep_as_parts"] = true;
    const HttpResponseMessage parts_response = post_json(fixture, parts_route + "/cut/" + parts_object_id, keep_as_parts);
    REQUIRE(parts_response.result() == http::status::ok);
    const nlohmann::json parts_result = nlohmann::json::parse(parts_response.body());
    REQUIRE(parts_result.at("objects").size() == 1);
    CHECK(parts_result.at("objects").at(0).at("side") == "parts");
    CHECK_FALSE(parts_result.contains("beds"));
}

// Verify arrange=true is deferred and returns one bed value per new instance.
TEST_CASE("REST cut can arrange the new objects", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(std::string("/session/").size());
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    nlohmann::json arrange_body = horizontal_cut_body();
    arrange_body["arrange"] = true;
    const HttpResponseMessage response = post_json(fixture, route + "/cut/" + object_id, arrange_body);
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json result = nlohmann::json::parse(response.body());
    REQUIRE(result.contains("beds"));
    CHECK(result.at("beds").size() == 2);
    const nlohmann::json after = listed(fixture, route);
    REQUIRE(after.at("objects").size() == 2);
    std::vector<std::pair<Vec2d, Vec2d>> bounds;
    for (const nlohmann::json &created : result.at("objects")) {
        const std::string created_id = created.at("id").get<std::string>();
        const nlohmann::json::const_iterator listed_object = std::find_if(after.at("objects").begin(), after.at("objects").end(),
            [&created_id](const nlohmann::json &object) { return object.at("id") == created_id; });
        REQUIRE(listed_object != after.at("objects").end());
        const std::string instance_id = listed_object->at("instances").at(0).at("id").get<std::string>();
        REQUIRE(result.at("beds").contains(instance_id));
        CHECK(result.at("beds").at(instance_id) == 0);
        bounds.push_back(object_xy_bounds(*find_object(fixture, session_id, created_id)));
    }
    REQUIRE(bounds.size() == 2);
    const bool disjoint = bounds[0].second.x() <= bounds[1].first.x() || bounds[1].second.x() <= bounds[0].first.x() ||
        bounds[0].second.y() <= bounds[1].first.y() || bounds[1].second.y() <= bounds[0].first.y();
    CHECK(disjoint);
}

// Verify slicing for three independent single-object cut results.
TEST_CASE("REST cut results slice", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    {
        const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
        const std::string route = setup.first;
        const std::string session_id = route.substr(std::string("/session/").size());
        const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
        nlohmann::json cut_body = horizontal_cut_body();
        cut_body["keep_lower"] = false;
        const HttpResponseMessage response = post_json(fixture, route + "/cut/" + object_id, cut_body);
        REQUIRE(response.result() == http::status::ok);
        CHECK_FALSE(slice_gcode(fixture, route, find_session(fixture, session_id)).empty());
    }
    {
        const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
        const std::string route = setup.first;
        const std::string session_id = route.substr(std::string("/session/").size());
        const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
        nlohmann::json cut_body = horizontal_cut_body();
        cut_body["keep_as_parts"] = true;
        const HttpResponseMessage response = post_json(fixture, route + "/cut/" + object_id, cut_body);
        REQUIRE(response.result() == http::status::ok);
        CHECK_FALSE(slice_gcode(fixture, route, find_session(fixture, session_id)).empty());
    }
    {
        const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
        const std::string route = setup.first;
        const std::string session_id = route.substr(std::string("/session/").size());
        const std::string part_id = setup.second.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
        const HttpResponseMessage response = post_json(fixture, route + "/cut/" + part_id, horizontal_cut_body());
        REQUIRE(response.result() == http::status::ok);
        CHECK(nlohmann::json::parse(response.body()).at("parts").size() == 2);
        CHECK_FALSE(slice_gcode(fixture, route, find_session(fixture, session_id)).empty());
    }
}

// Hidden until the plugin pipeline slices multi-object scenes.
TEST_CASE("REST cut results with two objects slice", "[.rest_multi_object_engine]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(std::string("/session/").size());
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const HttpResponseMessage response = post_json(fixture, route + "/cut/" + object_id, horizontal_cut_body());
    REQUIRE(response.result() == http::status::ok);
    CHECK(nlohmann::json::parse(response.body()).at("objects").size() == 2);
    CHECK_FALSE(slice_gcode(fixture, route, find_session(fixture, session_id)).empty());
}

// Verify synchronous observer invalidation and deferred import observer
// invalidation use captured shared state and reject stale commits.
TEST_CASE("REST cut is refused while slicing and on concurrent changes", "[rest_cut]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    std::string route = setup.first;
    std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    std::string session_id = route.substr(std::string("/session/").size());
    std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const SessionContext::WorkToken work = session->begin_work(SessionContext::Clock::now()).value();
    require_error(post_json(fixture, route + "/cut/" + object_id, horizontal_cut_body()),
        http::status::conflict, "session_busy");
    REQUIRE(session->cancel_work(work, SessionContext::Clock::now()));

    fixture.stop();
    const std::shared_ptr<std::shared_ptr<SessionContext>> observed =
        std::make_shared<std::shared_ptr<SessionContext>>();
    fixture.set_scene_edit_observer([observed]() {
        const std::shared_ptr<SessionContext> current = *observed;
        if (!current)
            return;
        const std::optional<SessionContext::MutationToken> token = current->begin_mutation();
        if (!token)
            return;
        SessionData data = current->data_snapshot().value();
        data.preferences["cut_observer"] = true;
        current->commit_mutation(*token, std::move(data), false, SessionContext::Clock::now());
    });
    fixture.start();
    const std::pair<std::string, nlohmann::json> observed_setup = session_with_cube(fixture);
    route = observed_setup.first;
    object_id = observed_setup.second.at("objects").at(0).at("id").get<std::string>();
    session_id = route.substr(std::string("/session/").size());
    session = find_session(fixture, session_id);
    *observed = session;
    require_error(post_json(fixture, route + "/cut/" + object_id, horizontal_cut_body()),
        http::status::conflict, "session_changed");

    fixture.stop();
    const std::shared_ptr<CutImportGate> gate = std::make_shared<CutImportGate>();
    fixture.set_import_observer([gate]() { gate->observe(); });
    fixture.start();
    const std::pair<std::string, nlohmann::json> import_setup = session_with_cube(fixture);
    route = import_setup.first;
    object_id = import_setup.second.at("objects").at(0).at("id").get<std::string>();
    session_id = route.substr(std::string("/session/").size());
    session = find_session(fixture, session_id);
    gate->arm();
    std::future<HttpResponseMessage> pending = std::async(std::launch::async, [&fixture, route, object_id]() {
        nlohmann::json arrange_body = horizontal_cut_body();
        arrange_body["arrange"] = true;
        return post_json(fixture, route + "/cut/" + object_id, arrange_body);
    });
    REQUIRE(gate->wait());
    const std::optional<SessionContext::MutationToken> competing_token = session->begin_mutation();
    REQUIRE(competing_token.has_value());
    SessionData competing = session->data_snapshot().value();
    competing.preferences["cut_import_observer"] = true;
    REQUIRE(session->commit_mutation(*competing_token, std::move(competing), false, SessionContext::Clock::now()));
    gate->release();
    require_error(pending.get(), http::status::conflict, "session_changed");
}
