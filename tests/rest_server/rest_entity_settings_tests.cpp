/*
 * Public HTTP tests for entity settings and layer-range lifecycle.
 *
 * Requests travel through the real loopback listener and inspect only public
 * responses except for the existing session hooks used to force busy and stale
 * commit states. Real slicing checks prove that accepted overrides reach the
 * engine, and project round-trips verify the serialized scene data.
 */

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include "entity_settings.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "rest_models_test_helpers.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;

namespace {

nlohmann::json listed(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target, const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

bool wait_for_status(
    const std::shared_ptr<Rest::SessionContext> &session,
    Rest::SessionStatus expected,
    std::chrono::seconds timeout = std::chrono::minutes(5))
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const Rest::SessionStatus status = session->snapshot().status;
        if (status == expected)
            return true;
        if (status == Rest::SessionStatus::failed)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return session->snapshot().status == expected;
}

std::string slice_gcode(
    const RestServerFixture &fixture,
    const std::string &route,
    const std::shared_ptr<Rest::SessionContext> &session)
{
    const HttpResponseMessage started = fixture.request(http::verb::post, route + "/slice");
    REQUIRE(started.result() == http::status::accepted);
    INFO("Slice error: " << session->snapshot().error_message);
    REQUIRE(wait_for_status(session, Rest::SessionStatus::sliced));
    const HttpResponseMessage result = fixture.request(http::verb::get, route + "/get_gcode");
    REQUIRE(result.result() == http::status::ok);
    return result.body();
}

double filament_used_mm(const std::string &gcode)
{
    const std::regex marker(R"(; filament used \[mm\] = ([0-9]+(?:\.[0-9]+)?))");
    std::smatch match;
    REQUIRE(std::regex_search(gcode, match, marker));
    return std::stod(match[1].str());
}

std::string profile_id(const nlohmann::json &profiles, const std::string &name)
{
    for (const nlohmann::json &profile : profiles)
        if (profile.at("name") == name)
            return profile.at("id").get<std::string>();
    return {};
}

std::string make_project_with_layer_range()
{
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "layer-range.3mf";
    Model model;
    ModelObject *object = model.add_object("layer-range", "cube.stl", make_cube(20, 20, 20));
    object->add_instance();
    ModelConfig range_config;
    range_config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    range_config.set_key_value("extruder", new ConfigOptionInt(0));
    object->layer_config_ranges.emplace(t_layer_height_range{2., 6.}, std::move(range_config));
    DynamicPrintConfig configuration;
    configuration.apply(FullPrintConfig::defaults());
    REQUIRE(store_3mf(path.string().c_str(), &model, &configuration));
    return read_file(path);
}

std::string layer_range_id(const nlohmann::json &scene, std::size_t object_index = 0, std::size_t range_index = 0)
{
    return scene.at("objects").at(object_index).at("layer_ranges").at(range_index).at("id").get<std::string>();
}

} // namespace

TEST_CASE("REST list_settings publishes entity setting scopes", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;
    const HttpResponseMessage response = fixture.request(http::verb::get, "/list_settings");
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json catalog = nlohmann::json::parse(response.body());
    REQUIRE(catalog.size() == 6);
    REQUIRE(catalog.at("entity").is_array());
    const auto extruder = std::find_if(catalog.at("entity").begin(), catalog.at("entity").end(),
        [](const nlohmann::json &definition) { return definition.at("id") == "extruder"; });
    REQUIRE(extruder != catalog.at("entity").end());
    CHECK(extruder->at("type") == "int");
    CHECK(extruder->at("default") == nlohmann::json{{"value", 0}});
    CHECK(extruder->at("min") == 0);
    CHECK(extruder->at("scopes") == nlohmann::json::array({"object", "part", "layer_range"}));
    REQUIRE(extruder->at("choices").size() == 10);
    for (std::size_t index = 0; index < extruder->at("choices").size(); ++index) {
        CHECK(extruder->at("choices").at(index).at("value").is_string());
        CHECK(extruder->at("choices").at(index).at("value") == std::to_string(index));
    }
    CHECK(extruder->at("choices").at(0).at("label") == "default");
    CHECK(extruder->at("choices").at(9) == nlohmann::json{{"value", "9"}, {"label", "9"}});

    const auto fill_density = std::find_if(catalog.at("print").begin(), catalog.at("print").end(),
        [](const nlohmann::json &definition) { return definition.at("id") == "fill_density"; });
    REQUIRE(fill_density != catalog.at("print").end());
    CHECK(fill_density->at("type") == "percent");
    CHECK(fill_density->at("default").contains("value"));
    CHECK(fill_density->at("scopes") == nlohmann::json::array({"object", "part", "layer_range"}));
}

TEST_CASE("REST entity settings read and write objects parts and layer ranges", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    const std::shared_ptr<Rest::SessionContext> session = find_session(fixture, session_id);
    nlohmann::json scene = listed(fixture, route);
    const std::string object_id = scene.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = scene.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const std::string instance_id = scene.at("objects").at(0).at("instances").at(0).at("id").get<std::string>();
    CHECK_FALSE(scene.at("objects").at(0).at("has_settings").get<bool>());
    CHECK_FALSE(scene.at("objects").at(0).at("parts").at(0).at("has_settings").get<bool>());
    CHECK(nlohmann::json::parse(fixture.request(http::verb::get, route + "/list_entity_settings/" + object_id).body())
        .at("setting_ids").empty());

    const std::uint64_t before_settings = session->snapshot().revision;
    const HttpResponseMessage object_update = post_json(fixture, route + "/set_entity_settings/" + object_id,
        {{"layer_height", {{"value", 0.25}}}});
    REQUIRE(object_update.result() == http::status::ok);
    CHECK(session->snapshot().revision == before_settings + 1);
    const nlohmann::json object_value = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + object_id + "/layer_height").body());
    CHECK(object_value.at("value").at("value") == Approx(0.25));
    CHECK(listed(fixture, route).at("objects").at(0).at("has_settings") == true);

    const HttpResponseMessage serialized = fixture.request(http::verb::post,
        route + "/set_entity_setting/" + part_id + "/fill_density/40%25");
    REQUIRE(serialized.result() == http::status::ok);
    CHECK(nlohmann::json::parse(serialized.body()).at("serialized_value") == "40%");
    const nlohmann::json part_value = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + part_id + "/fill_density").body());
    CHECK(part_value.at("value").at("value") == Approx(40.));
    CHECK(listed(fixture, route).at("objects").at(0).at("parts").at(0).at("has_settings") == true);

    const HttpResponseMessage part_removed = post_json(fixture, route + "/set_entity_settings/" + part_id,
        {{"fill_density", nullptr}});
    REQUIRE(part_removed.result() == http::status::ok);
    CHECK(nlohmann::json::parse(part_removed.body()).at("removed") == nlohmann::json::array({"fill_density"}));
    require_error(fixture.request(http::verb::get, route + "/get_entity_settings/" + part_id + "/fill_density"),
        http::status::not_found, "setting_not_set");
    const std::uint64_t after_removal = session->snapshot().revision;
    REQUIRE(post_json(fixture, route + "/set_entity_settings/" + part_id, {{"fill_density", nullptr}}).result() == http::status::ok);
    CHECK(session->snapshot().revision == after_removal);

    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + instance_id),
        http::status::unprocessable_entity, "invalid_entity_settings");
    const HttpResponseMessage negative = post_json(fixture, route + "/add_shape/" + object_id,
        {{"type", "negative_volume"}, {"shape", "box"}, {"size", {2, 2, 2}}});
    REQUIRE(negative.result() == http::status::ok);
    const std::string negative_id = nlohmann::json::parse(negative.body()).at("part_id").get<std::string>();
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + negative_id),
        http::status::unprocessable_entity, "invalid_entity_settings");

    const HttpResponseMessage range_response = post_json(fixture, route + "/add_layer_range/" + object_id,
        {{"min", 1.}, {"max", 3.}, {"layer_height", 0.2}});
    REQUIRE(range_response.result() == http::status::ok);
    const std::string range_id = nlohmann::json::parse(range_response.body()).at("id").get<std::string>();
    const nlohmann::json range_settings = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/list_entity_settings/" + range_id).body());
    CHECK(range_settings.at("setting_ids") == nlohmann::json::array({"extruder", "layer_height"}));
    const nlohmann::json range_height = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + range_id + "/layer_height").body());
    CHECK(range_height.at("value").at("value") == Approx(0.2));
    const HttpResponseMessage layer_removed = post_json(fixture, route + "/set_entity_settings/" + range_id,
        {{"layer_height", nullptr}});
    require_error(layer_removed, http::status::unprocessable_entity, "invalid_setting");
    const HttpResponseMessage object_scope_on_range = post_json(fixture, route + "/set_entity_settings/" + range_id,
        {{"brim_width", {{"value", 2.}}}});
    require_error(object_scope_on_range, http::status::unprocessable_entity, "invalid_setting");
}

TEST_CASE("REST layer ranges are entities with IDs that follow their bounds", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "first.stl", make_cube_stl())).size() == 1);
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "second.stl", make_cube_stl())).size() == 1);
    nlohmann::json scene = listed(fixture, route);
    const std::string first_object = scene.at("objects").at(0).at("id").get<std::string>();
    const std::string second_object = scene.at("objects").at(1).at("id").get<std::string>();
    const std::string first_range = nlohmann::json::parse(post_json(fixture, route + "/add_layer_range/" + first_object,
        {{"min", 0.}, {"max", 2.}, {"layer_height", 0.2}}).body()).at("id").get<std::string>();
    const std::string second_range = nlohmann::json::parse(post_json(fixture, route + "/add_layer_range/" + second_object,
        {{"min", 4.}, {"max", 6.}, {"layer_height", 0.2}}).body()).at("id").get<std::string>();
    REQUIRE_FALSE(first_range.empty());
    REQUIRE_FALSE(second_range.empty());
    require_error(post_json(fixture, route + "/add_layer_range/" + first_object,
        {{"min", 1.}, {"max", 3.}, {"layer_height", 0.2}}),
        http::status::unprocessable_entity, "invalid_layer_range");

    REQUIRE(post_json(fixture, route + "/set_layer_range/" + first_range,
        {{"min", 2.}, {"max", 4.}}).result() == http::status::ok);
    scene = listed(fixture, route);
    CHECK(layer_range_id(scene, 0) == first_range);
    CHECK(scene.at("objects").at(0).at("layer_ranges").at(0).at("min") == 2.);
    CHECK(scene.at("objects").at(0).at("layer_ranges").at(0).at("max") == 4.);
    const nlohmann::json current_range = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + first_range + "/layer_height").body());
    CHECK(current_range.at("value").at("value") == Approx(0.2));
    const std::string deleted_range = nlohmann::json::parse(post_json(fixture,
        route + "/add_layer_range/" + first_object,
        {{"min", 7.}, {"max", 8.}, {"layer_height", 0.2}}).body()).at("id").get<std::string>();
    const HttpResponseMessage deletion = fixture.request(http::verb::post, route + "/delete_entity/" + deleted_range);
    REQUIRE(deletion.result() == http::status::ok);
    CHECK(nlohmann::json::parse(deletion.body()).at("deleted") == nlohmann::json::array({deleted_range}));
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + deleted_range),
        http::status::not_found, "entity_not_found");

    const nlohmann::json merged_response = nlohmann::json::parse(post_json(fixture, route + "/merge",
        {{"ids", {first_object, second_object}}}).body());
    const std::string merged_object = merged_response.at("object_id").get<std::string>();
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + first_range),
        http::status::not_found, "entity_not_found");
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + second_range),
        http::status::not_found, "entity_not_found");
    scene = listed(fixture, route);
    const nlohmann::json &merged_ranges = scene.at("objects").at(0).at("layer_ranges");
    REQUIRE(merged_ranges.size() == 2);
    const std::string merged_range_one = merged_ranges.at(0).at("id").get<std::string>();
    const std::string merged_range_two = merged_ranges.at(1).at("id").get<std::string>();
    CHECK(merged_range_one != first_range);
    CHECK(merged_range_two != second_range);

    REQUIRE(post_json(fixture, route + "/split/" + merged_object, {{"to", "objects"}}).result() == http::status::ok);
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + merged_range_one),
        http::status::not_found, "entity_not_found");
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + merged_range_two),
        http::status::not_found, "entity_not_found");

    const nlohmann::json after_split = listed(fixture, route);
    const std::string target_object = after_split.at("objects").at(0).at("id").get<std::string>();
    const std::string delete_range = nlohmann::json::parse(post_json(fixture,
        route + "/add_layer_range/" + target_object,
        {{"min", 7.}, {"max", 9.}, {"layer_height", 0.2}}).body()).at("id").get<std::string>();
    REQUIRE(fixture.request(http::verb::post, route + "/delete_entity/" + target_object).result() == http::status::ok);
    require_error(fixture.request(http::verb::get, route + "/list_entity_settings/" + delete_range),
        http::status::not_found, "entity_not_found");

    const std::string project_session_id = create_session(fixture);
    const std::string project_route = "/session/" + project_session_id;
    const std::string project = make_project_with_layer_range();
    REQUIRE(object_ids(upload_model(fixture, project_route + "/add_model", "with-range.3mf", project)).size() == 1);
    scene = listed(fixture, project_route);
    REQUIRE(scene.at("objects").at(0).at("layer_ranges").size() == 1);
    CHECK_FALSE(layer_range_id(scene).empty());
    const HttpResponseMessage loaded = upload_model(fixture, project_route + "/load_project", "loaded-range.3mf", project);
    REQUIRE(object_ids(loaded).size() == 1);
    scene = listed(fixture, project_route);
    REQUIRE(scene.at("objects").at(0).at("layer_ranges").size() == 1);
    const std::string loaded_range = layer_range_id(scene);
    CHECK(fixture.request(http::verb::get, project_route + "/list_entity_settings/" + loaded_range).result() == http::status::ok);

    const std::string range_id = loaded_range;
    const std::string identity = R"({"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],"space":"platter"})";
    require_error(fixture.request(http::verb::post, project_route + "/transform/" + range_id, identity, "application/json"),
        http::status::unprocessable_entity, "invalid_transform");
    require_error(post_json(fixture, project_route + "/set_part_type/" + range_id, {{"type", "modifier"}}),
        http::status::unprocessable_entity, "invalid_part_type");
    require_error(post_json(fixture, project_route + "/rename/" + range_id, {{"name", "range"}}),
        http::status::unprocessable_entity, "invalid_name");
    require_error(post_json(fixture, project_route + "/add_shape/" + range_id,
        {{"type", "modifier"}, {"shape", "box"}}), http::status::unprocessable_entity, "invalid_shape");
    require_error(post_json(fixture, project_route + "/split/" + range_id, {{"to", "parts"}}),
        http::status::unprocessable_entity, "not_splittable");
    require_error(post_json(fixture, project_route + "/merge", {{"ids", {range_id, range_id}}}),
        http::status::unprocessable_entity, "invalid_merge");
    require_error(post_json(fixture, project_route + "/set_instance_count/" + range_id, {{"count", 1}}),
        http::status::unprocessable_entity, "invalid_instance_count");
    require_error(upload_model(fixture, project_route + "/add_part/" + range_id + "?type=modifier", "part.stl", make_cube_stl()),
        http::status::unprocessable_entity, "invalid_part_type");
}

TEST_CASE("REST entity setting edits are refused while slicing and on concurrent changes", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<std::shared_ptr<Rest::SessionContext>> observed_session =
        std::make_shared<std::shared_ptr<Rest::SessionContext>>();
    const std::shared_ptr<std::atomic<int>> commits = std::make_shared<std::atomic<int>>(0);
    fixture.set_scene_edit_observer([observed_session, commits]() {
        const std::shared_ptr<Rest::SessionContext> session = *observed_session;
        if (!session)
            return;
        const std::optional<Rest::SessionContext::MutationToken> token = session->begin_mutation();
        if (!token)
            return;
        Rest::SessionData competing = session->data_snapshot().value();
        competing.preferences["observer_commit"] = commits->fetch_add(1) + 1;
        session->commit_mutation(*token, std::move(competing), false, Rest::SessionContext::Clock::now());
    });
    fixture.start();
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    const std::shared_ptr<Rest::SessionContext> session = find_session(fixture, session_id);
    *observed_session = session;
    const nlohmann::json scene = listed(fixture, route);
    const std::string object_id = scene.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = scene.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const Rest::SessionContext::WorkToken work = session->begin_work(Rest::SessionContext::Clock::now()).value();
    const HttpResponseMessage malformed = fixture.request(http::verb::post, route + "/set_entity_settings/" + part_id,
        "{", "application/json");
    CHECK(malformed.result() == http::status::bad_request);
    require_error(post_json(fixture, route + "/set_entity_settings/" + part_id, {{"fill_density", {{"value", 20}}}}),
        http::status::conflict, "session_busy");
    REQUIRE(session->cancel_work(work, Rest::SessionContext::Clock::now()));

    const HttpResponseMessage stale_setting = post_json(fixture, route + "/set_entity_settings/" + part_id,
        {{"fill_density", {{"value", 25}}}});
    require_error(stale_setting, http::status::conflict, "session_changed");
    const HttpResponseMessage stale_range = post_json(fixture, route + "/add_layer_range/" + object_id,
        {{"min", 2.}, {"max", 4.}, {"layer_height", 0.2}});
    require_error(stale_range, http::status::conflict, "session_changed");
    CHECK(commits->load() == 2);
    CHECK(listed(fixture, route).at("objects").at(0).at("layer_ranges").empty());
    require_error(fixture.request(http::verb::get, route + "/get_entity_settings/" + part_id + "/fill_density"),
        http::status::not_found, "setting_not_set");
}

TEST_CASE("REST entity settings change the sliced result", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/fill_density/10%25").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/seam_visibility/0").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeters/1").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/top_solid_layers/1").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/bottom_solid_layers/1").result() == http::status::ok);
    const std::string object_id = listed(fixture, route).at("objects").at(0).at("id").get<std::string>();
    const nlohmann::json modifier_response = nlohmann::json::parse(post_json(fixture,
        route + "/add_shape/" + object_id,
        {{"type", "modifier"}, {"shape", "box"}, {"size", {12, 12, 12}}, {"position", {10, 10, 10}}}).body());
    const std::string modifier_id = modifier_response.at("part_id").get<std::string>();
    const std::shared_ptr<Rest::SessionContext> session = find_session(fixture, session_id);
    const std::string baseline = slice_gcode(fixture, route, session);
    const double baseline_filament = filament_used_mm(baseline);
    REQUIRE(post_json(fixture, route + "/set_entity_settings/" + modifier_id,
        {{"fill_density", {{"value", 80}}}}).result() == http::status::ok);
    const std::string modified = slice_gcode(fixture, route, session);
    CHECK(filament_used_mm(modified) > baseline_filament);

    const std::string range_id = nlohmann::json::parse(post_json(fixture, route + "/add_layer_range/" + object_id,
        {{"min", 5.}, {"max", 10.}, {"layer_height", 0.3}}).body()).at("id").get<std::string>();
    REQUIRE_FALSE(range_id.empty());
    const std::string ranged = slice_gcode(fixture, route, session);
    const std::regex layer_marker(R"(;Z:([0-9]+(?:\.[0-9]+)?)\n;MAX_LAYER_HEIGHT:([0-9]+(?:\.[0-9]+)?))");
    std::sregex_iterator layer(ranged.begin(), ranged.end(), layer_marker);
    const std::sregex_iterator end;
    bool found_range_height = false;
    for (; layer != end; ++layer) {
        const double z = std::stod((*layer)[1].str());
        const double height = std::stod((*layer)[2].str());
        if (z >= 5. && z <= 10. && std::abs(height - 0.3) < 0.001)
            found_range_height = true;
    }
    CHECK(found_range_height);

    RestServerFixture dual_fixture({}, true, true);
    dual_fixture.stop();
    std::ofstream(dual_fixture.profiles_directory() / "printer" / "dual-tool.ini")
        << "nozzle_diameter = 0.4,0.4\n"
        << "default_filament_profile = ,rest-filament-b\n";
    dual_fixture.start();
    const std::string dual_session_id = create_session(dual_fixture);
    const std::string dual_route = "/session/" + dual_session_id;
    const nlohmann::json profiles = nlohmann::json::parse(
        dual_fixture.request(http::verb::get, dual_route + "/list_profiles").body());
    const std::string dual_printer = profile_id(profiles.at("printer"), "dual-tool");
    const std::string print = profile_id(profiles.at("print"), "rest-print");
    REQUIRE_FALSE(dual_printer.empty());
    REQUIRE_FALSE(print.empty());
    REQUIRE(post_json(dual_fixture, dual_route + "/set_profile", {
        {"print", print}, {"printer", dual_printer}, {"filaments", nlohmann::json::array()},
        {"extruder", nlohmann::json::array()}
    }).result() == http::status::ok);
    REQUIRE(object_ids(upload_model(dual_fixture, dual_route + "/add_model", "dual-cube.stl", make_cube_stl())).size() == 1);
    const nlohmann::json dual_scene = listed(dual_fixture, dual_route);
    const std::string dual_part = dual_scene.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const std::shared_ptr<Rest::SessionContext> dual_session = find_session(dual_fixture, dual_session_id);
    const std::regex tool_one_command(R"((^|\n)T1\s)");
    const std::string dual_baseline = slice_gcode(dual_fixture, dual_route, dual_session);
    CHECK_FALSE(std::regex_search(dual_baseline, tool_one_command));
    REQUIRE(dual_fixture.request(http::verb::post, dual_route + "/set_entity_setting/" + dual_part + "/extruder/2").result() == http::status::ok);
    const std::string dual_gcode = slice_gcode(dual_fixture, dual_route, dual_session);
    CHECK(std::regex_search(dual_gcode, tool_one_command));
}

TEST_CASE("REST entity settings and layer ranges survive save_project and load_project", "[rest_entity_settings]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "persist.stl", make_cube_stl())).size() == 1);
    const nlohmann::json initial = listed(fixture, route);
    const std::string object_id = initial.at("objects").at(0).at("id").get<std::string>();
    const std::string part_id = initial.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    REQUIRE(post_json(fixture, route + "/set_entity_settings/" + object_id,
        {{"layer_height", {{"value", 0.25}}}}).result() == http::status::ok);
    REQUIRE(post_json(fixture, route + "/set_entity_settings/" + part_id,
        {{"fill_density", {{"value", 40}}}}).result() == http::status::ok);
    const std::string range_id = nlohmann::json::parse(post_json(fixture, route + "/add_layer_range/" + object_id,
        {{"min", 2.}, {"max", 6.}, {"layer_height", 0.3}}).body()).at("id").get<std::string>();
    REQUIRE_FALSE(range_id.empty());

    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    CHECK(saved.body().substr(0, 2) == "PK");
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "roundtrip.3mf", saved.body())).size() == 1);
    const nlohmann::json restored = listed(fixture, route);
    const std::string restored_object = restored.at("objects").at(0).at("id").get<std::string>();
    const std::string restored_part = restored.at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
    const std::string restored_range = layer_range_id(restored);
    REQUIRE(restored_range != range_id);
    const nlohmann::json object_value = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + restored_object + "/layer_height").body());
    const nlohmann::json part_value = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + restored_part + "/fill_density").body());
    const nlohmann::json range_value = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/get_entity_settings/" + restored_range + "/layer_height").body());
    CHECK(object_value.at("value").at("value") == Approx(0.25));
    CHECK(part_value.at("value").at("value") == Approx(40.));
    CHECK(range_value.at("value").at("value") == Approx(0.3));
    CHECK(nlohmann::json::parse(fixture.request(http::verb::get,
        route + "/get_entity_settings/" + restored_range + "/extruder").body()).at("value").at("value") == 0);
}
