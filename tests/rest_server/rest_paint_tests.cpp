// REST painting tests cover channel discovery, HTTP validation, persistence, and session races.
#include <catch2/catch.hpp>

#include "paint.hpp"
#include "rest_models_test_helpers.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Send one authenticated JSON mutation through the loopback REST fixture.
HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target,
    const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

// Read one complete object listing for stable object, part, and instance IDs.
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

// Return the first object's part ID from a standard cube listing.
std::string first_part_id(const nlohmann::json &scene)
{
    return scene.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
}

// Return the first object's instance ID from a standard cube listing.
std::string first_instance_id(const nlohmann::json &scene)
{
    return scene.at("objects").at(0).at("instances").at(0).at("id").get<std::string>();
}

// Return a solid part's platter-space bounds from the session's immutable snapshot.
std::pair<Vec3d, Vec3d> part_bounds(const std::shared_ptr<SessionContext> &session,
    const std::string &part_id)
{
    const std::optional<EntityReference> entity = session->resolve_entity(part_id);
    REQUIRE(entity.has_value());
    REQUIRE(entity->kind == EntityKind::volume);
    const std::optional<SessionData> data = session->data_snapshot();
    REQUIRE(data.has_value());
    for (ModelObject *object : data->scene->object_ptrs())
        for (ModelVolume *volume : object->volumes)
            if (volume->id().id == entity->engine_id) {
                const Transform3d matrix = object->instances.front()->get_matrix() * volume->get_matrix();
                Vec3d minimum = Vec3d::Constant(std::numeric_limits<double>::max());
                Vec3d maximum = Vec3d::Constant(std::numeric_limits<double>::lowest());
                for (const Vec3f &vertex : volume->mesh().its.vertices) {
                    const Vec3d point = matrix * vertex.cast<double>();
                    minimum = minimum.cwiseMin(point);
                    maximum = maximum.cwiseMax(point);
                }
                return {minimum, maximum};
            }
    FAIL("The requested part was not found in the scene.");
    return {};
}

// Create a circle request at one platter-space surface point.
nlohmann::json circle_body(const std::string &kind, const nlohmann::json &state, const Vec3d &point)
{
    return {
        {"kind", kind}, {"state", state}, {"space", "platter"}, {"tool", "circle"}, {"radius", 2.},
        {"strokes", {{{point.x(), point.y(), point.z()}}}}
    };
}

// Serialize an affine matrix in the row-major form accepted by the transform route.
nlohmann::json transform_body(const Transform3d &matrix, const std::string &space,
    const std::string &action = "replace", bool ensure_on_bed = false)
{
    nlohmann::json values = nlohmann::json::array();
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            values.push_back(matrix.matrix()(row, column));
    return { {"matrix", values}, {"space", space}, {"action", action}, {"ensure_on_bed", ensure_on_bed} };
}

// Build a two-level STL whose upper block is unsupported until painted support is consumed.
std::string make_overhang_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "overhang.stl";
    TriangleMesh mesh = make_cube(2., 2., 15.);
    TriangleMesh floating = make_cube(10., 10., 5.);
    floating.translate(Vec3f(10.f, 0.f, 10.f));
    mesh.merge(floating);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

// Return the JSON body from a successful get_paint response and assert its HTTP status.
nlohmann::json get_paint(const RestServerFixture &fixture, const std::string &target)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, target);
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Wait for a session to publish the requested slicing terminal state.
bool wait_for_status(const std::shared_ptr<SessionContext> &session, SessionStatus status)
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

// Slice one prepared session and return its current G-code text.
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

// Remove comment-only G-code lines so geometry and toolpath changes can be compared.
std::string non_comment_gcode(const std::string &gcode)
{
    std::string result;
    std::size_t start = 0;
    while (start < gcode.size()) {
        const std::size_t end = gcode.find('\n', start);
        const std::string line = gcode.substr(start, end == std::string::npos ? end : end - start);
        if (line.empty() || line.front() != ';')
            result += line + '\n';
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return result;
}

// Gate one import callback so a competing change precedes publication.
class PaintImportGate {
public:
    // Initialize the one-shot callback synchronization channels.
    PaintImportGate()
        : entered_(entered_promise_.get_future().share()), released_(release_promise_.get_future().share())
    {}

    // Arm the next observer invocation.
    void arm() { armed_.store(true); }

    // Signal entry once and wait until the test releases the deferred worker.
    void observe()
    {
        if (!armed_.exchange(false))
            return;
        entered_promise_.set_value();
        released_.wait_for(std::chrono::seconds(120));
    }

    // Wait until the deferred worker has entered the gate.
    bool wait() const { return entered_.wait_for(std::chrono::seconds(60)) == std::future_status::ready; }

    // Release the deferred worker, including during fixture destruction.
    void release()
    {
        if (!released_once_.exchange(true))
            release_promise_.set_value();
    }

    // Release any worker still waiting at destruction.
    ~PaintImportGate() { release(); }

private:
    std::atomic<bool> armed_{false};
    std::atomic<bool> released_once_{false};
    std::promise<void> entered_promise_;
    std::promise<void> release_promise_;
    std::shared_future<void> entered_;
    std::shared_future<void> released_;
};

} // namespace

// Verify fixed and active generic paint channels, including the fuzzy-skin plugin annotation.
TEST_CASE("REST paint kinds are listed", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const HttpResponseMessage response = fixture.request(http::verb::get, "/list_paint_kinds");
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json kinds = nlohmann::json::parse(response.body());
    REQUIRE(kinds.is_array());
    std::vector<std::string> ids;
    for (const nlohmann::json &kind : kinds) {
        ids.push_back(kind.at("id").get<std::string>());
        CHECK(kind.at("states").is_array());
    }
    REQUIRE(ids.size() >= 4);
    CHECK(std::find(ids.begin(), ids.end(), "support") != ids.end());
    CHECK(std::find(ids.begin(), ids.end(), "mmu") != ids.end());
    CHECK(std::find(ids.begin(), ids.end(), "builtin:seam") != ids.end());
    CHECK(std::find(ids.begin(), ids.end(), "perimeter.post_process.fuzzy_skin.painting") != ids.end());
    const nlohmann::json mmu = *std::find_if(kinds.begin(), kinds.end(),
        [](const nlohmann::json &kind) { return kind.at("id") == "mmu"; });
    CHECK(mmu.at("states") == nlohmann::json::array({"extruder"}));
    CHECK(mmu.at("max_extruder") == 15);
}

// Verify JSON validation, point details, target kinds, and instances.
TEST_CASE("REST paint validates the request and the target", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string part_id = first_part_id(setup.second);
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    const std::pair<Vec3d, Vec3d> bounds = part_bounds(session, part_id);
    const Vec3d top((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, bounds.second.z());

    require_error(post_json(fixture, route + "/paint/" + part_id, {}),
        http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::post, route + "/paint/" + part_id, "{", "application/json"),
        http::status::bad_request, "invalid_json");
    const HttpResponseMessage off_surface = post_json(fixture, route + "/paint/" + part_id,
        circle_body("support", "enforce", top + Vec3d(0., 0., 1.)));
    REQUIRE(off_surface.result() == http::status::unprocessable_entity);
    const nlohmann::json off_surface_error = nlohmann::json::parse(off_surface.body()).at("error");
    CHECK(off_surface_error.at("code") == "invalid_paint");
    REQUIRE(off_surface_error.at("details").size() == 1);
    CHECK(off_surface_error.at("details").at(0) == nlohmann::json{
        {"stroke", 0}, {"point", 0}, {"reason", "off_surface"}});
    nlohmann::json invalid = circle_body("support", "enforce", top);
    invalid["unknown"] = true;
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::bad_request, "invalid_request");
    invalid = circle_body("support", "enforce", top);
    invalid["radius"] = "2";
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::bad_request, "invalid_request");
    invalid = circle_body("support", "enforce", top);
    invalid["strokes"] = "points";
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::bad_request, "invalid_request");
    invalid = circle_body("support", "enforce", top);
    invalid["strokes"] = {{{top.x(), top.y()}}};
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::bad_request, "invalid_request");
    invalid = circle_body("support", "enforce", top);
    invalid["tool"] = "unknown";
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::unprocessable_entity, "invalid_paint");
    invalid = circle_body("support", "enforce", top);
    invalid["space"] = "unknown";
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::unprocessable_entity, "invalid_paint");
    invalid = circle_body("support", "unknown", top);
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::unprocessable_entity, "invalid_paint");
    require_error(post_json(fixture, route + "/paint/" + part_id,
        circle_body("mmu", nlohmann::json{{"extruder", 1}}, top)),
        http::status::unprocessable_entity, "invalid_paint");
    require_error(fixture.request(http::verb::get,
        route + "/get_paint/" + object_id + "?kind=support"),
        http::status::unprocessable_entity, "invalid_paint");
    invalid = circle_body("support", "enforce", top);
    invalid["instance"] = "missing";
    require_error(post_json(fixture, route + "/paint/" + part_id, invalid),
        http::status::not_found, "entity_not_found");
    require_error(post_json(fixture, route + "/clear_paint/" + part_id, { {"kind", "unknown"} }),
        http::status::unprocessable_entity, "invalid_paint");
    require_error(post_json(fixture, route + "/paint/missing", circle_body("support", "enforce", top)),
        http::status::not_found, "entity_not_found");

    const std::string second_object = object_ids(upload_model(fixture, route + "/add_model", "other.stl", make_cube_stl())).back();
    const nlohmann::json two_objects = listed(fixture, route);
    const std::string foreign_instance = two_objects.at("objects").at(1).at("instances").at(0).at("id").get<std::string>();
    nlohmann::json foreign = circle_body("support", "enforce", top);
    foreign["instance"] = foreign_instance;
    require_error(post_json(fixture, route + "/paint/" + object_id, foreign),
        http::status::unprocessable_entity, "invalid_paint");
    const std::string modifier = nlohmann::json::parse(post_json(fixture, route + "/add_shape/" + second_object,
        {{"type", "modifier"}, {"shape", "box"}}).body()).at("part_id").get<std::string>();
    require_error(post_json(fixture, route + "/paint/" + modifier, circle_body("support", "enforce", top)),
        http::status::unprocessable_entity, "invalid_paint");
}

// Verify changed IDs, unchanged revisions, generic/MMU storage, and channel/all clearing.
TEST_CASE("REST paint updates parts and reports changes", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(9);
    const std::string part_id = first_part_id(setup.second);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::pair<Vec3d, Vec3d> bounds = part_bounds(session, part_id);
    const Vec3d top((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, bounds.second.z());
    const nlohmann::json body = circle_body("support", "enforce", top);
    const nlohmann::json changed = nlohmann::json::parse(post_json(fixture, route + "/paint/" + part_id, body).body());
    REQUIRE(changed.at("changed") == nlohmann::json::array({part_id}));
    CHECK(listed(fixture, route).at("objects").at(0).at("parts").at(0).at("painted") ==
        nlohmann::json::array({"support"}));
    const std::uint64_t unchanged_revision = session->snapshot().revision;
    const nlohmann::json unchanged = nlohmann::json::parse(post_json(fixture, route + "/paint/" + part_id, body).body());
    CHECK(unchanged.at("changed").empty());
    CHECK(session->snapshot().revision == unchanged_revision);

    const HttpResponseMessage clear = post_json(fixture, route + "/clear_paint/" + part_id, { {"kind", "support"} });
    REQUIRE(clear.result() == http::status::ok);
    CHECK(nlohmann::json::parse(clear.body()).at("changed") == nlohmann::json::array({part_id}));
    const std::uint64_t clear_revision = session->snapshot().revision;
    const HttpResponseMessage clear_again = post_json(fixture, route + "/clear_paint/" + part_id, { {"kind", "support"} });
    REQUIRE(clear_again.result() == http::status::ok);
    CHECK(nlohmann::json::parse(clear_again.body()).at("changed").empty());
    CHECK(session->snapshot().revision == clear_revision);

    SessionData dual = session->data_snapshot().value();
    dual.configuration.set_deserialize_strict("nozzle_diameter", "0.4,0.4,0.4");
    const SessionContext::MutationToken token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(token, std::move(dual), false, SessionContext::Clock::now()));
    const HttpResponseMessage mmu = post_json(fixture, route + "/paint/" + part_id,
        circle_body("mmu", nlohmann::json{{"extruder", 2}}, top));
    REQUIRE(mmu.result() == http::status::ok);
    CHECK(listed(fixture, route).at("objects").at(0).at("parts").at(0).at("painted") ==
        nlohmann::json::array({"mmu"}));
    require_error(post_json(fixture, route + "/paint/" + part_id,
        circle_body("mmu", nlohmann::json{{"extruder", 4}}, top)),
        http::status::unprocessable_entity, "invalid_paint");
    REQUIRE(post_json(fixture, route + "/paint/" + part_id,
        circle_body("builtin:seam", "block", top)).result() == http::status::ok);
    const HttpResponseMessage clear_all = post_json(fixture, route + "/clear_paint/" + part_id, {{"kind", "all"}});
    REQUIRE(clear_all.result() == http::status::ok);
    CHECK(nlohmann::json::parse(clear_all.body()).at("changed") == nlohmann::json::array({part_id}));
    CHECK(listed(fixture, route).at("objects").at(0).at("parts").at(0).at("painted").empty());
}

// Verify painted triangle output, output spaces, empty channels, and strict query keys.
TEST_CASE("REST get_paint returns painted triangles", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string part_id = first_part_id(setup.second);
    const std::string object_id = setup.second.at("objects").at(0).at("id").get<std::string>();
    const std::string first_instance = first_instance_id(setup.second);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));

    Transform3d first_transform = Transform3d::Identity();
    Matrix3d first_scale = Matrix3d::Identity();
    first_scale.diagonal() << 2., 1., 1.;
    first_transform.linear() = Eigen::AngleAxisd(.4, Vec3d::UnitZ()).toRotationMatrix() * first_scale;
    first_transform.translation() = Vec3d(30., 20., 0.);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + first_instance,
        transform_body(first_transform, "platter", "replace", true).dump(), "application/json").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_instance_count/" + object_id,
        R"({"count":2})", "application/json").result() == http::status::ok);
    const nlohmann::json transformed = listed(fixture, route);
    const std::string second_instance_id = transformed.at("objects").at(0).at("instances").at(1).at("id").get<std::string>();
    Transform3d second_transform = Transform3d::Identity();
    second_transform.translation() = Vec3d(60., 20., 0.);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + second_instance_id,
        transform_body(second_transform, "platter", "replace", true).dump(), "application/json").result() == http::status::ok);
    Transform3d volume_transform = Transform3d::Identity();
    volume_transform.translation() = Vec3d(3., -2., 1.);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + part_id,
        transform_body(volume_transform, "object", "replace", true).dump(), "application/json").result() == http::status::ok);
    const std::pair<Vec3d, Vec3d> bounds = part_bounds(session, part_id);
    const Vec3d top((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, bounds.second.z());
    const nlohmann::json empty = get_paint(fixture, route + "/get_paint/" + part_id + "?kind=support");
    CHECK(empty.at("states").empty());
    REQUIRE(post_json(fixture, route + "/paint/" + part_id,
        circle_body("support", "enforce", top)).result() == http::status::ok);
    const nlohmann::json platter = get_paint(fixture, route + "/get_paint/" + part_id + "?kind=support");
    REQUIRE(platter.at("states").size() == 1);
    CHECK(platter.at("states").at(0).at("state") == "enforce");
    REQUIRE(platter.at("states").at(0).at("triangles").size() > 0);
    for (const nlohmann::json &triangle : platter.at("states").at(0).at("triangles"))
        for (const nlohmann::json &index : triangle)
            CHECK(platter.at("states").at(0).at("vertices").at(index.get<std::size_t>()).at(2).get<double>() ==
                Approx(top.z()).margin(1e-4));
    const nlohmann::json object = get_paint(fixture, route + "/get_paint/" + part_id + "?kind=support&space=object");
    const nlohmann::json volume = get_paint(fixture, route + "/get_paint/" + part_id + "?kind=support&space=volume");
    const nlohmann::json &platter_state = platter.at("states").at(0);
    const nlohmann::json &object_state = object.at("states").at(0);
    const nlohmann::json &volume_state = volume.at("states").at(0);
    REQUIRE(object_state.at("vertices").size() == platter_state.at("vertices").size());
    REQUIRE(volume_state.at("vertices").size() == platter_state.at("vertices").size());
    CHECK(object_state.at("triangles") == platter_state.at("triangles"));
    CHECK(volume_state.at("triangles") == platter_state.at("triangles"));
    const SessionData transformed_data = session->data_snapshot().value();
    const ModelObject *transformed_object = transformed_data.scene->object_ptrs().front();
    const ModelVolume *transformed_volume = transformed_object->volumes.front();
    const Transform3d first_matrix = transformed_object->instances.front()->get_matrix();
    const Transform3d volume_matrix = transformed_volume->get_matrix();
    for (std::size_t index = 0; index < platter_state.at("vertices").size(); ++index) {
        const nlohmann::json &platter_vertex = platter_state.at("vertices").at(index);
        const nlohmann::json &object_vertex = object_state.at("vertices").at(index);
        const nlohmann::json &volume_vertex = volume_state.at("vertices").at(index);
        const Vec3d platter_point(platter_vertex.at(0).get<double>(), platter_vertex.at(1).get<double>(),
            platter_vertex.at(2).get<double>());
        const Vec3d object_point = first_matrix.inverse() * platter_point;
        const Vec3d volume_point = (first_matrix * volume_matrix).inverse() * platter_point;
        for (int coordinate = 0; coordinate < 3; ++coordinate) {
            CHECK(object_vertex.at(coordinate).get<double>() == Approx(object_point[coordinate]).margin(1e-3));
            CHECK(volume_vertex.at(coordinate).get<double>() == Approx(volume_point[coordinate]).margin(1e-3));
        }
    }
    const nlohmann::json second_platter = get_paint(fixture,
        route + "/get_paint/" + part_id + "?kind=support&instance=" + second_instance_id);
    REQUIRE(second_platter.at("states").size() == 1);
    REQUIRE(second_platter.at("states").at(0).at("vertices").size() == platter_state.at("vertices").size());
    CHECK(second_platter.at("states").at(0).at("vertices") != platter_state.at("vertices"));
    const Transform3d second_matrix = transformed_object->instances.at(1)->get_matrix();
    for (std::size_t index = 0; index < platter_state.at("vertices").size(); ++index) {
        const nlohmann::json &platter_vertex = platter_state.at("vertices").at(index);
        const nlohmann::json &second_vertex = second_platter.at("states").at(0).at("vertices").at(index);
        const Vec3d local_point = (first_matrix * volume_matrix).inverse() * Vec3d(
            platter_vertex.at(0).get<double>(), platter_vertex.at(1).get<double>(), platter_vertex.at(2).get<double>());
        const Vec3d second_point = second_matrix * volume_matrix * local_point;
        for (int coordinate = 0; coordinate < 3; ++coordinate)
            CHECK(second_vertex.at(coordinate).get<double>() == Approx(second_point[coordinate]).margin(1e-3));
    }
    require_error(fixture.request(http::verb::get, route + "/get_paint/" + part_id + "?kind=support&other=x"),
        http::status::bad_request, "invalid_request");
}

// Verify save_project and load_project preserve all painted channel data.
TEST_CASE("REST paint persists in projects", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string part_id = first_part_id(setup.second);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    const std::pair<Vec3d, Vec3d> bounds = part_bounds(session, part_id);
    const Vec3d top((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, bounds.second.z());
    REQUIRE(post_json(fixture, route + "/paint/" + part_id,
        circle_body("builtin:seam", "enforce", top)).result() == http::status::ok);
    const nlohmann::json before = get_paint(fixture, route + "/get_paint/" + part_id + "?kind=builtin%3Aseam");
    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    const std::string restored_id = create_session(fixture);
    const std::string restored_route = "/session/" + restored_id;
    REQUIRE(object_ids(upload_model(fixture, restored_route + "/load_project", "painted.3mf", saved.body())).size() == 1);
    const nlohmann::json restored_scene = listed(fixture, restored_route);
    const nlohmann::json after = get_paint(fixture, restored_route + "/get_paint/" +
        first_part_id(restored_scene) + "?kind=builtin%3Aseam");
    CHECK(after == before);
}

// Verify slicing refusal, synchronous clear races, and deferred paint revision races.
TEST_CASE("REST paint is refused while slicing and on concurrent changes", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> setup = session_with_cube(fixture);
    const std::string route = setup.first;
    const std::string session_id = route.substr(9);
    const std::string part_id = first_part_id(setup.second);
    const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
    const std::pair<Vec3d, Vec3d> bounds = part_bounds(session, part_id);
    const Vec3d top((bounds.first.x() + bounds.second.x()) * .5,
        (bounds.first.y() + bounds.second.y()) * .5, bounds.second.z());
    const SessionContext::WorkToken work = session->begin_work(SessionContext::Clock::now()).value();
    require_error(post_json(fixture, route + "/paint/" + part_id,
        circle_body("support", "enforce", top)), http::status::conflict, "session_busy");
    require_error(post_json(fixture, route + "/clear_paint/" + part_id, {{"kind", "support"}}),
        http::status::conflict, "session_busy");
    REQUIRE(session->cancel_work(work, SessionContext::Clock::now()));

    REQUIRE(post_json(fixture, route + "/paint/" + part_id,
        circle_body("support", "enforce", top)).result() == http::status::ok);
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
        data.preferences["paint_clear_observer"] = true;
        current->commit_mutation(*token, std::move(data), false, SessionContext::Clock::now());
    });
    fixture.start();
    const std::pair<std::string, nlohmann::json> observed_setup = session_with_cube(fixture);
    const std::string observed_route = observed_setup.first;
    const std::string observed_part = first_part_id(observed_setup.second);
    const std::shared_ptr<SessionContext> observed_session = find_session(fixture, observed_route.substr(9));
    const std::pair<Vec3d, Vec3d> observed_bounds = part_bounds(observed_session, observed_part);
    const Vec3d observed_top((observed_bounds.first.x() + observed_bounds.second.x()) * .5,
        (observed_bounds.first.y() + observed_bounds.second.y()) * .5, observed_bounds.second.z());
    REQUIRE(post_json(fixture, observed_route + "/paint/" + observed_part,
        circle_body("support", "enforce", observed_top)).result() == http::status::ok);
    *observed = observed_session;
    require_error(post_json(fixture, observed_route + "/clear_paint/" + observed_part, {{"kind", "support"}}),
        http::status::conflict, "session_changed");

    fixture.stop();
    const std::shared_ptr<PaintImportGate> gate = std::make_shared<PaintImportGate>();
    fixture.set_import_observer([gate]() { gate->observe(); });
    fixture.start();
    const std::pair<std::string, nlohmann::json> import_setup = session_with_cube(fixture);
    const std::string import_route = import_setup.first;
    const std::string import_part = first_part_id(import_setup.second);
    const std::shared_ptr<SessionContext> import_session = find_session(fixture, import_route.substr(9));
    const std::pair<Vec3d, Vec3d> import_bounds = part_bounds(import_session, import_part);
    const Vec3d import_top((import_bounds.first.x() + import_bounds.second.x()) * .5,
        (import_bounds.first.y() + import_bounds.second.y()) * .5, import_bounds.second.z());
    gate->arm();
    std::future<HttpResponseMessage> pending = std::async(std::launch::async, [&fixture, import_route, import_part, import_top]() {
        return post_json(fixture, import_route + "/paint/" + import_part,
            circle_body("support", "enforce", import_top));
    });
    REQUIRE(gate->wait());
    const SessionContext::MutationToken competing_token = import_session->begin_mutation().value();
    SessionData competing = import_session->data_snapshot().value();
    competing.preferences["paint_import_observer"] = true;
    REQUIRE(import_session->commit_mutation(competing_token, std::move(competing), false, SessionContext::Clock::now()));
    gate->release();
    require_error(pending.get(), http::status::conflict, "session_changed");
}

// Verify seam and fuzzy-skin painting reaches their existing slicing plugins.
TEST_CASE("REST painted parts slice", "[rest_paint]")
{
    ensure_engine_initialized();
    RestServerFixture baseline_fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> baseline_setup = session_with_cube(baseline_fixture);
    const std::string baseline_route = baseline_setup.first;
    const std::shared_ptr<SessionContext> baseline_session = find_session(baseline_fixture, baseline_route.substr(9));
    const std::string baseline_gcode = slice_gcode(baseline_fixture, baseline_route, baseline_session);

    RestServerFixture seam_fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> seam_setup = session_with_cube(seam_fixture);
    const std::string seam_route = seam_setup.first;
    const std::string seam_part = first_part_id(seam_setup.second);
    const std::shared_ptr<SessionContext> seam_session = find_session(seam_fixture, seam_route.substr(9));
    const std::pair<Vec3d, Vec3d> seam_bounds = part_bounds(seam_session, seam_part);
    const Vec3d side(seam_bounds.second.x(), (seam_bounds.first.y() + seam_bounds.second.y()) * .5,
        (seam_bounds.first.z() + seam_bounds.second.z()) * .5);
    REQUIRE(post_json(seam_fixture, seam_route + "/paint/" + seam_part,
        circle_body("builtin:seam", "enforce", side)).result() == http::status::ok);
    const std::string seam_gcode = slice_gcode(seam_fixture, seam_route, seam_session);
    CHECK(non_comment_gcode(seam_gcode) != non_comment_gcode(baseline_gcode));

    RestServerFixture fuzzy_fixture({}, true, true);
    const std::pair<std::string, nlohmann::json> fuzzy_setup = session_with_cube(fuzzy_fixture);
    const std::string fuzzy_route = fuzzy_setup.first;
    const std::string fuzzy_part = first_part_id(fuzzy_setup.second);
    const std::shared_ptr<SessionContext> fuzzy_session = find_session(fuzzy_fixture, fuzzy_route.substr(9));
    const std::pair<Vec3d, Vec3d> fuzzy_bounds = part_bounds(fuzzy_session, fuzzy_part);
    const Vec3d fuzzy_side(fuzzy_bounds.second.x(), (fuzzy_bounds.first.y() + fuzzy_bounds.second.y()) * .5,
        (fuzzy_bounds.first.z() + fuzzy_bounds.second.z()) * .5);
    REQUIRE(post_json(fuzzy_fixture, fuzzy_route + "/paint/" + fuzzy_part,
        circle_body("perimeter.post_process.fuzzy_skin.painting", "enforce", fuzzy_side)).result() == http::status::ok);
    const std::string fuzzy_gcode = slice_gcode(fuzzy_fixture, fuzzy_route, fuzzy_session);
    CHECK(non_comment_gcode(fuzzy_gcode) != non_comment_gcode(baseline_gcode));
}

// Hidden until support generation reads painted facets.
TEST_CASE("REST support painting reaches the support engine", "[.rest_support_engine]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const auto tune = [&fixture](const std::string &route) {
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.3").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeters/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/top_solid_layers/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/bottom_solid_layers/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/fill_density/10%25").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeter_generator/classic").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/support_material/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/support_material_auto/0").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/dont_support_bridges/0").result() == http::status::ok);
    };
    const auto support_slice = [&fixture, &tune](bool painted) {
        const std::string session_id = create_session(fixture);
        const std::string route = "/session/" + session_id;
        REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "overhang.stl", make_overhang_stl())).size() == 1);
        tune(route);
        const std::shared_ptr<SessionContext> session = find_session(fixture, session_id);
        const nlohmann::json scene = listed(fixture, route);
        const std::string part_id = first_part_id(scene);
        if (painted) {
            const SessionData data = session->data_snapshot().value();
            const BoundingBoxf3 bounds = data.scene->get_object(0).instance_bounding_box(0);
            const Vec3d center(bounds.max.x() - 5., bounds.center().y(), bounds.max.z() - 5.);
            const nlohmann::json body = {
                {"kind", "support"}, {"state", "enforce"}, {"tool", "smart_fill"},
                {"strokes", {{{center.x(), center.y(), center.z()}}}}
            };
            REQUIRE(post_json(fixture, route + "/paint/" + part_id, body).result() == http::status::ok);
        }
        return slice_gcode(fixture, route, session);
    };
    const std::string support_marker = ";TYPE:Support material";
    CHECK(support_slice(false).find(support_marker) == std::string::npos);
    CHECK(support_slice(true).find(support_marker) != std::string::npos);
}
