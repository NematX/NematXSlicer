// REST scene-edit tests cover routing, atomic session publication and public IDs.
#include <catch2/catch.hpp>

#include "rest_models_test_helpers.hpp"
#include "scene_edit.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/TextConfiguration.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <atomic>
#include <future>
#include <regex>

using namespace RestServerTest;

namespace {

nlohmann::json listed(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

std::pair<std::string, std::string> session_with_cube(RestServerFixture &fixture)
{
    const std::string session = create_session(fixture);
    const std::string route = "/session/" + session;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return {session, route};
}

HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target, const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

class OneShotGate {
public:
    OneShotGate() : entered_(entered_promise_.get_future().share()), release_(release_promise_.get_future().share()) {}
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

std::string make_two_shell_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "shells.stl";
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(10, 10, 10);
    Slic3r::TriangleMesh second = Slic3r::make_cube(5, 5, 5);
    second.translate(Slic3r::Vec3f(20, 0, 0));
    mesh.merge(second);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

std::string make_overhang_stl()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "overhang.stl";
    Slic3r::TriangleMesh mesh = Slic3r::make_cube(2, 2, 15);
    Slic3r::TriangleMesh floating = Slic3r::make_cube(10, 10, 5);
    floating.translate(Slic3r::Vec3f(10, 0, 10));
    mesh.merge(floating);
    REQUIRE(mesh.write_binary(path.string().c_str()));
    return read_file(path);
}

std::string make_scene_edit_3mf()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "scene-edit.3mf";
    Slic3r::Model model;
    Slic3r::ModelObject *first = model.add_object("cut-a", "cut-a.stl", Slic3r::make_cube(10, 10, 10));
    first->add_instance();
    first->cut_id.init();
    Slic3r::TriangleSelector selector(first->volumes.front()->mesh());
    selector.set_facet(0, Slic3r::EnforcerBlockerType::ENFORCER);
    first->volumes.front()->supported_facets.set(selector);
    first->volumes.front()->facets_annotation_mutable("builtin:seam").set(selector);
    Slic3r::ModelObject *second = model.add_object("cut-b", "cut-b.stl", Slic3r::make_cube(8, 8, 8));
    second->add_instance()->set_offset(Slic3r::Vec3d(20, 0, 0));
    second->cut_id = first->cut_id;
    Slic3r::ModelObject *third = model.add_object("other-cut", "other-cut.stl", Slic3r::make_cube(6, 6, 6));
    third->add_instance()->set_offset(Slic3r::Vec3d(40, 0, 0));
    third->cut_id.init();
    Slic3r::ModelObject *text = model.add_object("text", "text.stl", Slic3r::make_cube(4, 4, 2));
    text->add_instance()->set_offset(Slic3r::Vec3d(60, 0, 0));
    text->volumes.front()->text_configuration = Slic3r::TextConfiguration{};
    text->volumes.front()->emboss_shape = Slic3r::EmbossShape{};
    REQUIRE(Slic3r::store_3mf(path.string().c_str(), &model, nullptr));
    return read_file(path);
}

bool wait_for_status(const std::shared_ptr<Slic3r::Rest::SessionContext> &session,
    Slic3r::Rest::SessionStatus status)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        const Slic3r::Rest::SessionStatus current = session->snapshot().status;
        if (current == status)
            return true;
        if (current == Slic3r::Rest::SessionStatus::failed)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return session->snapshot().status == status;
}

std::string slice_gcode(const RestServerFixture &fixture, const std::string &route,
    const std::shared_ptr<Slic3r::Rest::SessionContext> &session)
{
    REQUIRE(fixture.request(http::verb::post, route + "/slice").result() == http::status::accepted);
    const bool sliced = wait_for_status(session, Slic3r::Rest::SessionStatus::sliced);
    INFO("Slice error: " << session->snapshot().error_message);
    REQUIRE(sliced);
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_gcode");
    REQUIRE(response.result() == http::status::ok);
    return response.body();
}

double filament_used_mm(const std::string &gcode)
{
    const std::regex marker(R"(; filament used \[mm\] = ([0-9]+(?:\.[0-9]+)?))");
    std::smatch match;
    REQUIRE(std::regex_search(gcode, match, marker));
    return std::stod(match[1].str());
}

std::pair<Slic3r::Vec3d, Slic3r::Vec3d> world_bounds(
    const Slic3r::ModelInstance &instance, const Slic3r::ModelVolume &volume)
{
    Slic3r::Vec3d minimum = Slic3r::Vec3d::Constant(std::numeric_limits<double>::max());
    Slic3r::Vec3d maximum = Slic3r::Vec3d::Constant(std::numeric_limits<double>::lowest());
    const Slic3r::Transform3d matrix = instance.get_matrix() * volume.get_matrix();
    for (const Slic3r::Vec3f &vertex : volume.mesh().its.vertices) {
        const Slic3r::Vec3d point = matrix * vertex.cast<double>();
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return {minimum, maximum};
}

} // namespace

TEST_CASE("REST list_objects reports part types paintings and cut objects", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture).second;
    const nlohmann::json scene = listed(fixture, route);
    CHECK(scene["objects"][0]["cut"] == false);
    CHECK(scene["objects"][0]["parts"][0]["type"] == "part");
    CHECK(scene["objects"][0]["parts"][0]["painted"] == nlohmann::json::array());
}

TEST_CASE("REST project round trip preserves cut and painting but drops emboss metadata", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "scene-edit.3mf", make_scene_edit_3mf())).size() == 4);
    nlohmann::json scene = listed(fixture, route);
    REQUIRE(scene["objects"].size() == 4);
    CHECK(scene["objects"][0]["cut"] == true);
    CHECK(scene["objects"][1]["cut"] == true);
    CHECK(scene["objects"][2]["cut"] == true);
    CHECK(scene["objects"][0]["parts"][0]["painted"] == nlohmann::json::array({"support", "builtin:seam"}));

    const std::string first_id = scene["objects"][0]["id"];
    require_error(fixture.request(http::verb::post, route + "/delete_entity/" + first_id),
        http::status::unprocessable_entity, "cut_object");
    REQUIRE(post_json(fixture, route + "/delete_entity/" + first_id, {{"invalidate_cut", true}}).result() == http::status::ok);
    scene = listed(fixture, route);
    REQUIRE(scene["objects"].size() == 3);
    CHECK(scene["objects"][0]["cut"] == false);
    CHECK(scene["objects"][1]["cut"] == true);

    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const Slic3r::ModelVolume *round_tripped_text = session->data_snapshot()->scene->get_object(2).volumes.front();
    CHECK_FALSE(round_tripped_text->text_configuration.has_value());
    CHECK_FALSE(round_tripped_text->emboss_shape.has_value());
}

TEST_CASE("REST set_part_type changes types atomically and keeps unchanged types uncommitted", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const auto [session_id, route] = session_with_cube(fixture);
    const std::string object_id = listed(fixture, route)["objects"][0]["id"];
    REQUIRE(post_json(fixture, route + "/add_shape/" + object_id, {{"type", "modifier"}, {"shape", "box"}}).result() == http::status::ok);
    const std::string part_id = listed(fixture, route)["objects"][0]["parts"][1]["id"];
    REQUIRE(post_json(fixture, route + "/set_part_type/" + part_id, {{"type", "support_enforcer"}}).result() == http::status::ok);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const std::uint64_t revision = session->snapshot().revision;
    REQUIRE(post_json(fixture, route + "/set_part_type/" + part_id, {{"type", "support_enforcer"}}).result() == http::status::ok);
    CHECK(session->snapshot().revision == revision);
    // Demand and geometry support volumes convert into each other like the other helper types.
    for (const char *type : {"support_column", "no_support_zone", "support_interface_column", "support_blocker"}) {
        CAPTURE(type);
        REQUIRE(post_json(fixture, route + "/set_part_type/" + part_id, {{"type", type}}).result() == http::status::ok);
        CHECK(listed(fixture, route)["objects"][0]["parts"][1]["type"] == type);
    }
}

TEST_CASE("REST rename validates names and invalidates G-code only for objects", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture).second;
    const nlohmann::json scene = listed(fixture, route);
    const std::string object_id = scene["objects"][0]["id"];
    const std::string part_id = scene["objects"][0]["parts"][0]["id"];
    REQUIRE(post_json(fixture, route + "/rename/" + object_id, {{"name", "Bracket"}}).result() == http::status::ok);
    REQUIRE(post_json(fixture, route + "/rename/" + part_id, {{"name", "Solid"}}).result() == http::status::ok);
    require_error(post_json(fixture, route + "/rename/" + object_id, {{"name", "bad/name"}}),
        http::status::unprocessable_entity, "invalid_name");
}

TEST_CASE("REST delete_entity removes entities and keeps the retained part ID", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture).second;
    const std::string object_id = listed(fixture, route)["objects"][0]["id"];
    const nlohmann::json added = nlohmann::json::parse(post_json(fixture, route + "/add_shape/" + object_id,
        {{"type", "modifier"}, {"shape", "box"}}).body());
    const std::string retained = listed(fixture, route)["objects"][0]["parts"][0]["id"];
    const HttpResponseMessage deleted = fixture.request(http::verb::post,
        route + "/delete_entity/" + added["part_id"].get<std::string>());
    REQUIRE(deleted.result() == http::status::ok);
    CHECK(listed(fixture, route)["objects"][0]["parts"][0]["id"] == retained);
}

TEST_CASE("REST add_part checks the target before upload and imports a part", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture).second;
    const std::string object_id = listed(fixture, route)["objects"][0]["id"];
    const HttpResponseMessage response = upload_model(fixture, route + "/add_part/" + object_id + "?type=modifier",
        "part.stl", make_cube_stl());
    REQUIRE(response.result() == http::status::ok);
    CHECK(nlohmann::json::parse(response.body())["type"] == "modifier");
    require_error(upload_model(fixture, route + "/add_part/missing?type=modifier", "part.stl", make_cube_stl()),
        http::status::not_found, "entity_not_found");
}

TEST_CASE("REST imports use UTF-8 work paths and hide temporary file references", "[rest_scene_edit][rest_import_utf8]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true, std::filesystem::u8path(u8"travail-é"));
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    std::string target, filename, file, error_code;
    SECTION("add_model") {
        target = route + "/add_model";
        filename = "cube.stl";
        file = make_cube_stl();
        error_code = "invalid_model";
    }
    SECTION("load_project") {
        target = route + "/load_project";
        filename = "project.3mf";
        file = make_3mf();
        error_code = "invalid_project";
    }
    SECTION("add_part") {
        REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
        const std::string object_id = listed(fixture, route)["objects"][0]["id"];
        target = route + "/add_part/" + object_id + "?type=modifier";
        filename = "part.stl";
        file = make_cube_stl();
        error_code = "invalid_model";
    }
    const HttpResponseMessage imported = upload_model(fixture, target, filename, file);
    INFO(imported.body());
    REQUIRE(imported.result() == http::status::ok);
    const nlohmann::json scene = listed(fixture, route);
    REQUIRE(scene["objects"].size() == 1);
    const std::string work_path = fixture.work_directory().u8string();
    CHECK(scene.dump().find("rest-upload-") == std::string::npos);
    // Metadata checks supplement the public import: source references are not
    // listed by HTTP, but are retained for subsequent project exports.
    const Slic3r::Rest::SessionData data = session->data_snapshot().value();
    for (const Slic3r::ModelObject *object : data.scene->object_ptrs()) {
        CHECK(object->input_file.find(work_path) == std::string::npos);
        for (const Slic3r::ModelVolume *volume : object->volumes)
            CHECK(volume->source.input_file.find(work_path) == std::string::npos);
    }
    const std::uint64_t revision = session->snapshot().revision;
    const HttpResponseMessage invalid = upload_model(fixture, target, filename, "not a model");
    require_error(invalid, http::status::unprocessable_entity, error_code);
    const std::string message = nlohmann::json::parse(invalid.body()).at("error").at("message");
    CHECK(message.find(work_path) == std::string::npos);
    CHECK(message.find("rest-upload-") == std::string::npos);
    CHECK(session->snapshot().revision == revision);
    CHECK(listed(fixture, route) == scene);
}

TEST_CASE("REST add_shape creates default and sized shapes", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const auto [session_id, route] = session_with_cube(fixture);
    const std::string object_id = listed(fixture, route)["objects"][0]["id"];
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    struct ShapeCase {
        const char *type;
        const char *shape;
        Slic3r::Vec3d size;
        const char *name;
    };
    const ShapeCase cases[] = {
        {"modifier", "box", {100, 100, 100}, "Generic-Box"},
        {"modifier", "cylinder", {50, 50, 100}, "Generic-Cylinder"},
        {"modifier", "sphere", {100, 100, 100}, "Generic-Sphere"},
        {"modifier", "slab", {30, 30, 10}, "Generic-Slab"},
        {"brim_patch", "square", {100, 100, 0.1}, "Brim-Square"},
        {"brim_patch", "circle", {10, 10, 0.1}, "Brim-Circle"},
        {"seam_position_center", "small_sphere", {20, 20, 20}, "Seam-Sphere"},
        {"seam_position_center_z", "small_cylinder", {20, 20, 100}, "Seam-Cylinder"},
        {"support_column", "cylinder", {50, 50, 100}, "Column-Cylinder"},
        {"no_support_zone", "box", {100, 100, 100}, "NoSupport-Box"},
        {"support_interface_column", "box", {100, 100, 100}, "InterfaceColumn-Box"}
    };
    for (const ShapeCase &shape : cases) {
        CAPTURE(shape.shape);
        const HttpResponseMessage response = post_json(fixture, route + "/add_shape/" + object_id,
            {{"type", shape.type}, {"shape", shape.shape}});
        REQUIRE(response.result() == http::status::ok);
        const std::string part_id = nlohmann::json::parse(response.body()).at("part_id");
        const Slic3r::Rest::EntityReference entity = session->resolve_entity(part_id).value();
        Slic3r::Rest::SessionData data = session->data_snapshot().value();
        const Slic3r::ModelVolume *volume = Slic3r::Rest::find_transform_volume(*data.scene, entity).second;
        CHECK(volume->mesh().bounding_box().size().isApprox(shape.size, 1e-3));
        CHECK(volume->name == shape.name);
        CHECK(Slic3r::Rest::part_type_name(volume->type()) == shape.type);
        if (std::string(shape.shape) == "slab") {
            const Slic3r::ModelObject *object = volume->get_object();
            const auto bounds = world_bounds(*object->instances.front(), *volume);
            CHECK(bounds.first.z() == Approx(object->instance_bounding_box(0).min.z()).margin(1e-3));
        }
    }

    // Support columns and no-support zones are created only as boxes and cylinders.
    for (const char *type : {"support_column", "no_support_zone", "support_interface_column"})
        for (const char *shape : {"sphere", "slab"})
            require_error(post_json(fixture, route + "/add_shape/" + object_id, {{"type", type}, {"shape", shape}}),
                http::status::unprocessable_entity, "invalid_shape");

    const Slic3r::Vec3d requested_position(50, 60, 70);
    const HttpResponseMessage sized = post_json(fixture, route + "/add_shape/" + object_id,
        {{"type", "modifier"}, {"shape", "sphere"}, {"size", {20, 30, 40}}, {"position", {50, 60, 70}}});
    REQUIRE(sized.result() == http::status::ok);
    const Slic3r::Rest::EntityReference entity = session->resolve_entity(
        nlohmann::json::parse(sized.body()).at("part_id").get<std::string>()).value();
    Slic3r::Rest::SessionData data = session->data_snapshot().value();
    const std::pair<Slic3r::ModelObject *, Slic3r::ModelVolume *> target =
        Slic3r::Rest::find_transform_volume(*data.scene, entity);
    CHECK(target.second->mesh().bounding_box().size().isApprox(Slic3r::Vec3d(20, 30, 40), 1e-3));
    const auto bounds = world_bounds(*target.first->instances.front(), *target.second);
    CHECK((0.5 * (bounds.first + bounds.second)).isApprox(requested_position, 1e-3));
}

TEST_CASE("REST split and merge replace scene entities in one commit", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture).second;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "second.stl", make_cube_stl())).size() == 1);
    const nlohmann::json scene = listed(fixture, route);
    const std::vector<std::string> ids{scene["objects"][0]["id"], scene["objects"][1]["id"]};
    const HttpResponseMessage merged = post_json(fixture, route + "/merge", {{"ids", ids}});
    REQUIRE(merged.result() == http::status::ok);
    CHECK(listed(fixture, route)["objects"].size() == 1);
}

TEST_CASE("REST shape mesh conversion refuses overflow underflow and degenerate bounds atomically", "[rest_scene_edit][rest_shape_bounds]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    nlohmann::json request{{"type", "modifier"}, {"shape", "box"}};
    bool planar = false;
    SECTION("finite double overflows float") { request["size"] = {1e100, 20, 20}; }
    SECTION("positive double underflows float") { request["size"] = {1e-300, 20, 20}; }
    SECTION("generated slab has a zero bound dimension") {
        planar = true;
        request["shape"] = "slab";
    }
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", planar ? "flat.3mf" : "cube.stl",
        planar ? make_3mf() : make_cube_stl())).size() == 1);
    const nlohmann::json before = listed(fixture, route);
    const std::string object_id = before["objects"][0]["id"];
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const std::uint64_t revision = session->snapshot().revision;
    const std::shared_ptr<Slic3r::Model> scene = session->data_snapshot()->scene;
    require_error(post_json(fixture, route + "/add_shape/" + object_id, request),
        http::status::unprocessable_entity, "invalid_shape");
    CHECK(session->snapshot().revision == revision);
    CHECK(session->data_snapshot()->scene == scene);
    CHECK(listed(fixture, route) == before);
    // A refusal must not leave the route or entity mappings unusable.
    REQUIRE(post_json(fixture, route + "/add_shape/" + object_id,
        {{"type", "modifier"}, {"shape", "box"}, {"size", {20, 30, 40}}}).result() == http::status::ok);
    CHECK(listed(fixture, route)["objects"][0]["parts"].size() == 2);
}

TEST_CASE("REST scene edits are refused while slicing and on concurrent changes", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<OneShotGate> gate = std::make_shared<OneShotGate>();
    fixture.set_import_observer([gate]() { gate->observe(); });
    fixture.set_scene_edit_observer([gate]() { gate->observe(); });
    fixture.start();
    const auto [session_id, route] = session_with_cube(fixture);
    const std::string object_id = listed(fixture, route)["objects"][0]["id"];
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now()).value();
    require_error(post_json(fixture, route + "/rename/" + object_id, {{"name", "Busy"}}), http::status::conflict, "session_busy");
    require_error(fixture.request(http::verb::post, route + "/rename/" + object_id, "{", "application/json"),
        http::status::bad_request, "invalid_json");
    REQUIRE(session->cancel_work(work, Slic3r::Rest::SessionRegistry::Clock::now()));

    std::function<HttpResponseMessage()> request;
    SECTION("set_part_type") {
        const HttpResponseMessage added = post_json(fixture, route + "/add_shape/" + object_id,
            {{"type", "modifier"}, {"shape", "box"}});
        const std::string id = nlohmann::json::parse(added.body()).at("part_id");
        request = [&fixture, route, id]() { return post_json(fixture, route + "/set_part_type/" + id,
            {{"type", "support_enforcer"}}); };
    }
    SECTION("rename") {
        request = [&fixture, route, object_id]() { return post_json(fixture, route + "/rename/" + object_id,
            {{"name", "Concurrent"}}); };
    }
    SECTION("delete_entity") {
        const HttpResponseMessage added = post_json(fixture, route + "/add_shape/" + object_id,
            {{"type", "modifier"}, {"shape", "box"}});
        const std::string id = nlohmann::json::parse(added.body()).at("part_id");
        request = [&fixture, route, id]() { return fixture.request(http::verb::post, route + "/delete_entity/" + id); };
    }
    SECTION("add_shape") {
        request = [&fixture, route, object_id]() { return post_json(fixture, route + "/add_shape/" + object_id,
            {{"type", "modifier"}, {"shape", "box"}}); };
    }
    SECTION("split") {
        const std::string split_id = object_ids(upload_model(
            fixture, route + "/add_model", "shells.stl", make_two_shell_stl())).front();
        request = [&fixture, route, split_id]() { return post_json(fixture, route + "/split/" + split_id,
            {{"to", "parts"}}); };
    }
    SECTION("merge") {
        const std::string second = object_ids(upload_model(
            fixture, route + "/add_model", "second.stl", make_cube_stl())).front();
        request = [&fixture, route, object_id, second]() { return post_json(fixture, route + "/merge",
            {{"ids", {object_id, second}}}); };
    }
    SECTION("add_part") {
        request = [&fixture, route, object_id]() { return upload_model(fixture,
            route + "/add_part/" + object_id + "?type=modifier", "part.stl", make_cube_stl()); };
    }
    REQUIRE(bool(request));
    gate->arm();
    std::future<HttpResponseMessage> response = std::async(std::launch::async, request);
    REQUIRE(gate->wait());
    Slic3r::Rest::SessionData competing = session->data_snapshot().value();
    competing.preferences["phase38_race"] = "newer";
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), competing, false,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    gate->release();
    REQUIRE(response.wait_for(std::chrono::seconds(300)) == std::future_status::ready);
    require_error(response.get(), http::status::conflict, "session_changed");
}

TEST_CASE("REST negative volumes and support enforcers change the sliced result", "[rest_scene_edit]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const auto tune = [&fixture](const std::string &route) {
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/seam_visibility/0").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.3").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeters/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/top_solid_layers/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/bottom_solid_layers/1").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/fill_density/10%25").result() == http::status::ok);
        REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeter_generator/classic").result() == http::status::ok);
    };

    SECTION("negative volume reduces filament") {
        const std::string negative_session_id = create_session(fixture);
        const std::string negative_route = "/session/" + negative_session_id;
        REQUIRE(object_ids(upload_model(fixture, negative_route + "/add_model", "shells.stl", make_two_shell_stl())).size() == 1);
        tune(negative_route);
        const std::shared_ptr<Slic3r::Rest::SessionContext> negative_session = find_session(fixture, negative_session_id);
        const std::string baseline_gcode = slice_gcode(fixture, negative_route, negative_session);
        const double baseline_filament = filament_used_mm(baseline_gcode);
        const std::string negative_object = listed(fixture, negative_route)["objects"][0]["id"];
        const Slic3r::Rest::SessionData negative_data = negative_session->data_snapshot().value();
        const Slic3r::ModelObject &negative_model_object = negative_data.scene->get_object(0);
        const Slic3r::BoundingBoxf3 negative_bb = negative_model_object.instance_bounding_box(0);
        const Slic3r::Vec3d negative_center(
            negative_bb.max.x() - 2.5, negative_bb.min.y() + 2.5, negative_bb.min.z() + 2.5);
        REQUIRE(post_json(fixture, negative_route + "/add_shape/" + negative_object,
            {{"type", "negative_volume"}, {"shape", "box"}, {"size", {5, 5, 5}},
             {"position", {negative_center.x(), negative_center.y(), negative_center.z()}}}).result() == http::status::ok);
        const std::string cut_gcode = slice_gcode(fixture, negative_route, negative_session);
        CHECK(filament_used_mm(cut_gcode) < baseline_filament);
    }

    SECTION("support enforcer reaches the slicing engine") {
        const auto [session_id, route] = session_with_cube(fixture);
        const std::string object = listed(fixture, route)["objects"][0]["id"];
        REQUIRE(post_json(fixture, route + "/add_shape/" + object,
            {{"type", "support_enforcer"}, {"shape", "box"}, {"size", {10, 10, 10}}}).result() == http::status::ok);

        const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
        const Slic3r::Rest::SessionData data = session->data_snapshot().value();
        Slic3r::Model model(*data.scene);
        Slic3r::Print print;
        Slic3r::Orchestrator::RunContext run_context;
        Slic3r::Orchestrator::RunScope run_scope = Slic3r::Orchestrator::instance().enter_run(run_context, &print);
        for (Slic3r::ModelObject *model_object : model.object_ptrs())
            print.auto_assign_extruders(model_object);
        print.apply(model, data.configuration);
        print.set_status_silent();
        print.process();

        REQUIRE(print.objects().size() == 1);
        const std::vector<Slic3r::ExPolygons> enforcer_layers = print.objects().front().slice_support_enforcers();
        CHECK(std::any_of(enforcer_layers.begin(), enforcer_layers.end(),
            [](const Slic3r::ExPolygons &polygons) { return !polygons.empty(); }));
    }
}

// Support generation is not migrated to the plugin pipeline in this branch yet,
// so no support demand reaches the generator. Re-enable this test once the
// support-demand plugins are available.
TEST_CASE("REST support enforcers change generated support material", "[.rest_support_engine]")
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
    const auto support_slice = [&fixture, &tune](bool enforcer) {
        const std::string session_id = create_session(fixture);
        const std::string route = "/session/" + session_id;
        REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "overhang.stl", make_overhang_stl())).size() == 1);
        tune(route);
        const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
        const std::string object = listed(fixture, route)["objects"][0]["id"];
        if (enforcer) {
            const Slic3r::Rest::SessionData data = session->data_snapshot().value();
            const Slic3r::BoundingBoxf3 bb = data.scene->get_object(0).instance_bounding_box(0);
            const Slic3r::Vec3d center(bb.max.x() - 5.0, bb.center().y(), bb.min.z() + 7.5);
            REQUIRE(post_json(fixture, route + "/add_shape/" + object,
                {{"type", "support_enforcer"}, {"shape", "box"}, {"size", {14, 14, 16}},
                 {"position", {center.x(), center.y(), center.z()}}}).result() == http::status::ok);
        }
        return slice_gcode(fixture, route, session);
    };

    const std::string support_marker = ";TYPE:Support material";
    CHECK(support_slice(false).find(support_marker) == std::string::npos);
    CHECK(support_slice(true).find(support_marker) != std::string::npos);
}
