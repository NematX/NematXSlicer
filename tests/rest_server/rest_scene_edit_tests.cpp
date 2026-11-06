// REST scene-edit tests cover routing, atomic session publication and public IDs.
#include <catch2/catch.hpp>

#include "rest_models_test_helpers.hpp"
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
