/*
 * HTTP tests for REST arrangement and automatic orientation.
 *
 * Each test uses the live session pipeline so it covers target ID resolution,
 * deferred worker publication, revision conflicts, and the additive response
 * contract as well as the placement algorithms.
 */
#include <catch2/catch.hpp>

#include "placement.hpp"
#include "rest_models_test_helpers.hpp"

#include "libslic3r/Model.hpp"

#include <atomic>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <utility>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Keep JSON request construction consistent with other REST model tests.
HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target,
    const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

// Return the current object projection after requiring a successful response.
nlohmann::json listed(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Create a session containing one imported cube and return its route and ID.
std::pair<std::string, std::string> session_with_cube(RestServerFixture &fixture)
{
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return {session_id, route};
}

// A one-shot barrier lets a test publish a competing session revision while a placement worker is queued.
class OneShotGate {
public:
    OneShotGate()
        : entered_(entered_promise_.get_future().share()),
          release_(release_promise_.get_future().share())
    {}

    void arm() { armed_.store(true); }

    void observe()
    {
        if (!armed_.exchange(false))
            return;
        entered_promise_.set_value();
        release_.wait_for(std::chrono::seconds(120));
    }

    bool wait() const { return entered_.wait_for(std::chrono::seconds(60)) == std::future_status::ready; }

    void release()
    {
        if (!released_.exchange(true))
            release_promise_.set_value();
    }

    ~OneShotGate() { release(); }

private:
    std::atomic<bool> armed_{false};
    std::atomic<bool> released_{false};
    std::promise<void> entered_promise_;
    std::promise<void> release_promise_;
    std::shared_future<void> entered_;
    std::shared_future<void> release_;
};

// Build an off-axis rectangular STL so orientation must change its instance transform.
std::string make_rectangular_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "rectangular.stl";
    TriangleMesh mesh = make_cube(10., 20., 40.);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

} // namespace

TEST_CASE("REST arrange validates parameters and targets", "[rest_placement]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;
    const std::pair<std::string, std::string> created = session_with_cube(fixture);
    const std::string &session_id = created.first;
    const std::string &route = created.second;
    const nlohmann::json before = listed(fixture, route);
    const std::string object_id = before.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = before.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const HttpResponseMessage range_response = post_json(fixture, route + "/add_layer_range/" + object_id,
        {{"min", 5.}, {"max", 10.}, {"layer_height", 0.3}});
    REQUIRE(range_response.result() == http::status::ok);
    const std::string range_id = nlohmann::json::parse(range_response.body()).at("id").get<std::string>();

    require_error(post_json(fixture, route + "/arrange", {{"unknown", true}}),
        http::status::bad_request, "invalid_request");
    require_error(post_json(fixture, route + "/arrange", {{"distance", -1.}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"distance", 1e20}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"distance_from_bed", 1e20}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"alignment", "sideways"}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"geometry", "unknown"}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"strategy", "unknown"}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"ids", {"missing"}}}),
        http::status::not_found, "entity_not_found");
    require_error(post_json(fixture, route + "/arrange", {{"ids", {part_id}}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/arrange", {{"ids", {range_id}}}),
        http::status::unprocessable_entity, "invalid_arrange");
    require_error(post_json(fixture, route + "/orient", {{"overhang_angle", 44.9}}),
        http::status::unprocessable_entity, "invalid_orient");
    require_error(post_json(fixture, route + "/orient", {{"ids", {part_id}}}),
        http::status::unprocessable_entity, "invalid_orient");
    require_error(post_json(fixture, route + "/orient", {{"ids", {range_id}}}),
        http::status::unprocessable_entity, "invalid_orient");

    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);
    SessionData sequential = session->data_snapshot().value();
    sequential.configuration.set_deserialize_strict({{"print_plan_builder", "ordering.plan_builder.by_object"},
        {"extruder_clearance_radius", "50"}, {"skirts", "0"}, {"brim_width", "0"}});
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), std::move(sequential), false,
        SessionRegistry::Clock::now()));
    require_error(post_json(fixture, route + "/arrange", {{"ids", {object_id}}, {"distance", 1.}}),
        http::status::unprocessable_entity, "invalid_arrange");
}

TEST_CASE("REST arrange moves instances onto the bed and reports their beds", "[rest_placement]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;
    const std::pair<std::string, std::string> created = session_with_cube(fixture);
    const std::string &session_id = created.first;
    const std::string &route = created.second;
    const std::string object_id = listed(fixture, route).at("objects").at(0).at("id").get<std::string>();
    REQUIRE(post_json(fixture, route + "/set_instance_count/" + object_id, {{"count", 2}}).result() == http::status::ok);
    const nlohmann::json before = listed(fixture, route);
    REQUIRE(before.at("objects").at(0).at("instances").size() == 2);

    const HttpResponseMessage response = post_json(fixture, route + "/arrange", {{"distance", 6.}});
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json result = nlohmann::json::parse(response.body());
    REQUIRE(result.at("instances").size() == 2);
    const nlohmann::json after = listed(fixture, route);
    bool any_moved = false;
    for (std::size_t index = 0; index < 2; ++index) {
        const nlohmann::json &placed = result.at("instances").at(index);
        CHECK(placed.at("id") == before.at("objects").at(0).at("instances").at(index).at("id"));
        REQUIRE(placed.at("bed").is_number_integer());
        CHECK(placed.at("bed") == 0);
        const nlohmann::json &old_matrix = before.at("objects").at(0).at("instances").at(index).at("matrix");
        const nlohmann::json &new_matrix = after.at("objects").at(0).at("instances").at(index).at("matrix");
        CHECK(placed.at("id") == after.at("objects").at(0).at("instances").at(index).at("id"));
        any_moved = any_moved || old_matrix != new_matrix;
    }
    CHECK(any_moved);

    const std::string empty_session_id = create_session(fixture);
    const std::shared_ptr<SessionContext> empty_session = find_session(fixture, empty_session_id);
    REQUIRE(empty_session != nullptr);
    const std::uint64_t empty_revision = empty_session->snapshot().revision;
    const HttpResponseMessage empty_result = fixture.request(
        http::verb::post, "/session/" + empty_session_id + "/arrange");
    REQUIRE(empty_result.result() == http::status::ok);
    CHECK(nlohmann::json::parse(empty_result.body()).at("instances").empty());
    CHECK(empty_session->snapshot().revision == empty_revision);
}

TEST_CASE("REST orient validates parameters and rotates instances onto the bed", "[rest_placement]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::vector<std::string> imported_ids = object_ids(
        upload_model(fixture, route + "/add_model", "rectangular.stl", make_rectangular_stl()));
    REQUIRE(imported_ids.size() == 1);
    REQUIRE(post_json(fixture, route + "/set_instance_count/" + imported_ids.front(), {{"count", 2}}).result() == http::status::ok);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);
    SessionData candidate = session->data_snapshot().value();
    candidate.scene = std::make_shared<Model>(*candidate.scene);
    ModelObject *object = candidate.scene->object_ptrs().front();
    object->instances.front()->set_rotation(Vec3d(std::acos(-1.) / 2., 0., 0.));
    object->ensure_on_bed();
    ModelInstance *sibling = object->instances[1];
    sibling->set_offset(sibling->get_offset() + Vec3d(60., 0., 0.));
    object->invalidate_bounding_box();
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), std::move(candidate), true,
        SessionRegistry::Clock::now()));
    const nlohmann::json before = listed(fixture, route);
    REQUIRE(before.at("objects").at(0).at("instances").size() == 2);
    const std::string instance_id = before.at("objects").at(0).at("instances").at(0).at("id").get<std::string>();

    const HttpResponseMessage response = post_json(fixture, route + "/orient",
        {{"ids", {instance_id}}, {"min_area", true}, {"overhang_angle", 60.}});
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json result = nlohmann::json::parse(response.body());
    REQUIRE(result.at("instances").size() == 1);
    CHECK(result.at("instances").at(0).at("id") == instance_id);
    const nlohmann::json after = listed(fixture, route);
    CHECK(result.at("instances").at(0).at("matrix") == after.at("objects").at(0).at("instances").at(0).at("matrix"));
    CHECK(result.at("instances").at(0).at("matrix") != before.at("objects").at(0).at("instances").at(0).at("matrix"));
    CHECK(after.at("objects").at(0).at("instances").at(1).at("matrix") ==
        before.at("objects").at(0).at("instances").at(1).at("matrix"));
    CHECK(session->data_snapshot()->scene->object_ptrs().front()->get_instance_min_z(0) == Approx(0.).margin(1e-5));
}

TEST_CASE("REST arrange and orient are refused while slicing and on concurrent changes", "[rest_placement]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;
    fixture.stop();
    const std::shared_ptr<OneShotGate> gate = std::make_shared<OneShotGate>();
    fixture.set_import_observer([gate]() { gate->observe(); });
    fixture.start();
    const std::pair<std::string, std::string> created = session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, created.first);
    REQUIRE(session != nullptr);
    const SessionContext::WorkToken work = session->begin_work(SessionContext::Clock::now()).value();
    require_error(post_json(fixture, created.second + "/arrange", {{"unknown", true}}),
        http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::post, created.second + "/arrange", "{", "application/json"),
        http::status::bad_request, "invalid_json");
    require_error(post_json(fixture, created.second + "/arrange", nlohmann::json::object()),
        http::status::conflict, "session_busy");
    require_error(post_json(fixture, created.second + "/orient", nlohmann::json::object()),
        http::status::conflict, "session_busy");
    REQUIRE(session->cancel_work(work, SessionContext::Clock::now()));

    gate->arm();
    std::future<HttpResponseMessage> response = std::async(std::launch::async, [&fixture, route = created.second]() {
        return post_json(fixture, route + "/arrange", nlohmann::json::object());
    });
    REQUIRE(gate->wait());
    SessionData competing = session->data_snapshot().value();
    competing.preferences["placement_test_race"] = true;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), std::move(competing), false,
        SessionContext::Clock::now()));
    gate->release();
    REQUIRE(response.wait_for(std::chrono::seconds(300)) == std::future_status::ready);
    require_error(response.get(), http::status::conflict, "session_changed");
}
