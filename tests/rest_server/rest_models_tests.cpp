/*
 * Multipart integration tests for REST model and project uploads.
 *
 * The client deliberately writes small request fragments and chunked bodies so
 * the server must parse boundaries across transport reads instead of relying on
 * one complete in-memory request. Other cases cover rejected uploads (which
 * must never leave a temporary file), the import thread pool, the upload idle
 * timeout, and the bounded drain that keeps early error responses readable.
 * Project tests check that load_project replaces the scene, the configuration
 * and the entity IDs atomically, keeps unknown keys as custom settings, and
 * leaves the session untouched when the project is refused. Per-profile metadata
 * travels in the core's project sections, independently of the flat settings.
 */

#include <catch_main.hpp>
#include <nlohmann/json.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "engine.hpp"
#include "file_names.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "multipart_upload.hpp"
#include "rest_server_fixture.hpp"
#include "rest_models_test_helpers.hpp"
#include "session.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace RestServerTest;

namespace {

const std::vector<std::string> model_extensions = {".stl", ".obj", ".3mf"};

std::string make_obj_with_padding(std::size_t padding_bytes = 0)
{
    std::string obj;
    obj.reserve(padding_bytes + 80);
    while (obj.size() < padding_bytes)
        obj += "# padding for fragmented upload\n";
    obj += "v 0 0 0\n"
           "v 20 0 0\n"
           "v 0 20 0\n"
           "f 1 2 3\n";
    return obj;
}

// Read settings through the typed route, as a client would.
nlohmann::json read_settings(const RestServerFixture &fixture, const std::string &route, const nlohmann::json &ids)
{
    const HttpResponseMessage response = fixture.request(
        http::verb::post,
        route + "/get_settings",
        nlohmann::json{{"setting_ids", ids}}.dump(),
        "application/json");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

} // namespace

namespace {

// Build wire matrices explicitly rather than relying on Eigen's storage order.
nlohmann::json transform_body(const Slic3r::Transform3d &matrix, const std::string &space,
    const std::string &action = "compose", bool ensure_on_bed = false)
{
    nlohmann::json values = nlohmann::json::array();
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            values.push_back(matrix.matrix()(row,column));
    return {{"matrix", values}, {"space", space}, {"action", action}, {"ensure_on_bed", ensure_on_bed}};
}

Slic3r::Transform3d listed_matrix(const nlohmann::json &matrix)
{
    Slic3r::Transform3d result = Slic3r::Transform3d::Identity();
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            result.matrix()(row,column) = matrix.at(row * 4 + column).get<double>();
    return result;
}

nlohmann::json read_objects(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// A client has the whole download once Content-Length bytes arrived, but the
// server releases the temporary file only when its write completion runs on
// the network thread, slightly later. Poll instead of racing that release.
std::size_t wait_for_work_file_count(const RestServerFixture &fixture, std::size_t expected)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::size_t count = fixture.work_file_count();
    while (count != expected && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        count = fixture.work_file_count();
    }
    return count;
}

} // namespace

TEST_CASE("REST save_project round-trips a modified enriched project without changing its source", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    Slic3r::DynamicPrintConfig partial;
    partial.set_deserialize_strict("layer_height", "0.3");
    const std::string enriched = add_project_config_lines(make_3mf(2, &partial),
        "; future_print_key = 42\n; printer_future_key = pf\n; filament_future_key = 7\n");
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "original.3mf", enriched)).size() == 2);
    const std::string added = object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).front();
    Slic3r::Transform3d matrix = Slic3r::Transform3d::Identity();
    matrix.translation() = Slic3r::Vec3d(14, 27, 6);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + added,
        transform_body(matrix, "platter").dump(), "application/json").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_instance_count/" + added,
        R"({"count":3})", "application/json").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    Slic3r::Rest::SessionData secure = session->data_snapshot().value();
    secure.preferences = {{"units", "inches"}};
    for (const char *key : {"print_host", "printhost_apikey", "printhost_cafile", "printhost_port"})
        secure.configuration.set_key_value(key, new Slic3r::ConfigOptionString("secret-witness"));
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), secure, false,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const Slic3r::Rest::SessionData before = session->data_snapshot().value();
    const nlohmann::json listed = read_objects(fixture, route);
    const std::map<std::string, std::string> profiles = fixture.profile_files();
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    const Slic3r::Rest::SessionContext::Snapshot slicing = session->snapshot();
    const HttpResponseMessage during = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(during.result() == http::status::ok);
    CHECK(session->snapshot().status == slicing.status);
    CHECK(session->snapshot().revision == slicing.revision);
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    const Slic3r::Rest::SessionContext::Snapshot sliced = session->snapshot();
    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    CHECK(saved.body().substr(0, 2) == "PK");
    CHECK(saved[http::field::content_type] == "model/3mf");
    CHECK(saved[http::field::cache_control] == "no-store");
    CHECK(saved[http::field::content_disposition] == Slic3r::Rest::content_disposition_attachment("original.3mf"));
    CHECK(saved[http::field::content_length] == std::to_string(saved.body().size()));
    CHECK(session->snapshot().revision == sliced.revision);
    CHECK(session->snapshot().status == sliced.status);
    CHECK(session->snapshot().current_gcode == sliced.current_gcode);
    CHECK(session->viewer_snapshot() == viewer);
    CHECK(read_objects(fixture, route) == listed);
    CHECK(session->data_snapshot()->name == before.name);
    CHECK(session->data_snapshot()->scene == before.scene);
    CHECK(session->data_snapshot()->configuration.diff(before.configuration).empty());
    CHECK(session->data_snapshot()->profile_sections == before.profile_sections);
    CHECK(session->data_snapshot()->preferences == before.preferences);
    const std::string other_id = create_session(fixture);
    const std::string other_route = "/session/" + other_id;
    REQUIRE(object_ids(upload_model(fixture, other_route + "/load_project", "original.3mf", saved.body())).size() == 3);
    const Slic3r::Rest::SessionData after = find_session(fixture, other_id)->data_snapshot().value();
    Slic3r::DynamicPrintConfig expected(before.configuration);
    for (const char *key : {"print_host", "printhost_apikey", "printhost_cafile", "printhost_port"})
        expected.erase(key);
    Slic3r::DynamicPrintConfig actual(after.configuration);
    // Reloading a project saved from the same file keeps a stable preset name.
    for (const char *key : {"print_settings_id", "printer_settings_id", "filament_settings_id"}) {
        INFO(key);
        CHECK(actual.opt_serialize(key) == "original.3mf");
        actual.erase(key);
        expected.erase(key);
    }
    for (const std::string &key : expected.keys()) {
        if (!expected.option(key)->is_phony())
            continue;
        REQUIRE(actual.has(key));
        CHECK(actual.option(key)->is_phony());
        CHECK(actual.opt_serialize(key) == expected.opt_serialize(key));
        actual.erase(key);
    }
    for (const std::string &key : expected.keys())
        if (expected.option(key)->is_phony())
            expected.erase(key);
    CHECK(actual.diff(expected).empty());
    CHECK(after.name == before.name);
    CHECK(after.preferences == nlohmann::json::object());
    CHECK(after.profiles.print == before.profiles.print);
    CHECK(after.profiles.printer == before.profiles.printer);
    CHECK(after.profiles.filaments == before.profiles.filaments);
    CHECK(after.profiles.extruder == before.profiles.extruder);
    REQUIRE(after.scene->object_ptrs().size() == before.scene->object_ptrs().size());
    for (std::size_t index = 0; index < before.scene->object_ptrs().size(); ++index) {
        const Slic3r::ModelObject *a = before.scene->object_ptrs()[index];
        const Slic3r::ModelObject *b = after.scene->object_ptrs()[index];
        CHECK(a->name == b->name);
        REQUIRE(a->volumes.size() == b->volumes.size());
        REQUIRE(a->instances.size() == b->instances.size());
        for (std::size_t v = 0; v < a->volumes.size(); ++v) {
            CHECK(a->volumes[v]->name == b->volumes[v]->name);
            Slic3r::Transform3d expected_volume = a->volumes[v]->get_matrix();
            if (a->volumes.size() == 1)
                expected_volume.translation().setZero();
            CHECK(expected_volume.isApprox(b->volumes[v]->get_matrix(), 1e-6));
            CHECK(a->volumes[v]->mesh().facets_count() == b->volumes[v]->mesh().facets_count());
        }
        for (std::size_t i = 0; i < a->instances.size(); ++i) {
            Slic3r::Transform3d expected_instance = a->instances[i]->get_matrix();
            // The 3MF loader moves a single volume's translation into each instance without moving its geometry.
            if (a->volumes.size() == 1)
                expected_instance.translation() = (a->instances[i]->get_matrix() * a->volumes.front()->get_matrix()).translation();
            CHECK(expected_instance.isApprox(b->instances[i]->get_matrix(), 1e-6));
            for (std::size_t v = 0; v < a->volumes.size(); ++v)
                CHECK((a->instances[i]->get_matrix() * a->volumes[v]->get_matrix()).isApprox(
                    b->instances[i]->get_matrix() * b->volumes[v]->get_matrix(), 1e-6));
        }
    }
    const HttpResponseMessage saved_again = fixture.request(http::verb::get, other_route + "/save_project");
    REQUIRE(saved_again.result() == http::status::ok);
    const std::string third_id = create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, "/session/" + third_id + "/load_project",
        "original.3mf", saved_again.body())).size() == 3);
    const Slic3r::Rest::SessionData second_round = find_session(fixture, third_id)->data_snapshot().value();
    for (const char *key : {"print_settings_id", "printer_settings_id", "filament_settings_id"}) {
        INFO(key);
        CHECK(second_round.configuration.opt_serialize(key) == "original.3mf");
    }
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "saved.3mf";
    std::ofstream(path, std::ios::binary) << saved.body();
    Slic3r::DynamicPrintConfig reloaded;
    Slic3r::ConfigSubstitutionContext substitutions(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    Slic3r::Model model = Slic3r::Model::read_from_file(path.string(), &reloaded, &substitutions);
    for (const char *key : {"print_host", "printhost_apikey", "printhost_cafile", "printhost_port"}) {
        CHECK_FALSE(reloaded.has(key));
        CHECK(before.configuration.opt_string(key) == "secret-witness");
    }
    CHECK(fixture.profile_files() == profiles);
    CHECK(wait_for_work_file_count(fixture, 0) == 0);
}

TEST_CASE("REST save_project writes snapshot profile sections and replaces them on selection", "[rest_save_project][rest_profile_sections]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const Slic3r::Rest::SessionData initial = session->data_snapshot().value();
    REQUIRE_FALSE(initial.profile_sections.empty());
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).size() == 1);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    Slic3r::Rest::SessionData local = session->data_snapshot().value();
    local.configuration.set_deserialize_strict("printhost_password", "local-password-witness");
    local.configuration.set_deserialize_strict("printhost_client_cert_password", "local-certificate-witness");
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), std::move(local), false,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const Slic3r::Rest::SessionData before = session->data_snapshot().value();
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(response.result() == http::status::ok);
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "saved.3mf";
    std::ofstream(path, std::ios::binary) << response.body();
    Slic3r::DynamicPrintConfig configuration;
    Slic3r::ConfigSubstitutionContext substitutions(Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
    std::string sections;
    const Slic3r::Model model = Slic3r::Model::read_from_archive(path.u8string(), &configuration, &substitutions,
        Slic3r::Model::LoadAttribute::AddDefaultInstances, &sections);
    REQUIRE(model.object_ptrs().size() == 1);
    CHECK(sections == before.profile_sections);
    CHECK(configuration.opt_float("layer_height") == Approx(0.25));
    CHECK_FALSE(configuration.has("default_filament_profile"));
    CHECK_FALSE(configuration.has("printhost_password"));
    CHECK_FALSE(configuration.has("printhost_client_cert_password"));
    CHECK(sections.find("local-password-witness") == std::string::npos);
    CHECK(sections.find("local-certificate-witness") == std::string::npos);
    CHECK(session->data_snapshot()->profile_sections == before.profile_sections);
    const std::string loaded_id = create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, "/session/" + loaded_id + "/load_project", "saved.3mf", response.body())).size() == 1);
    CHECK(find_session(fixture, loaded_id)->data_snapshot()->profile_sections == before.profile_sections);

    const HttpResponseMessage listed = fixture.request(http::verb::get, route + "/list_profiles");
    REQUIRE(listed.result() == http::status::ok);
    const nlohmann::json profiles = nlohmann::json::parse(listed.body());
    std::string printer_id;
    for (const nlohmann::json &printer : profiles.at("printer"))
        if (printer.at("name") == "rest-printer-b")
            printer_id = printer.at("id").get<std::string>();
    REQUIRE_FALSE(printer_id.empty());
    const nlohmann::json selection = {{"print", initial.profiles.print}, {"printer", printer_id},
        {"extruder", nlohmann::json::array()}, {"filaments", nlohmann::json::array()}};
    REQUIRE(fixture.request(http::verb::post, route + "/set_profile", selection.dump(), "application/json").result() == http::status::ok);
    const Slic3r::Rest::SessionData selected = session->data_snapshot().value();
    REQUIRE(selected.profile_sections != before.profile_sections);
    boost::property_tree::ptree selected_sections;
    std::istringstream input(selected.profile_sections);
    boost::property_tree::read_ini(input, selected_sections);
    const boost::property_tree::ptree::const_assoc_iterator printer = selected_sections.find("printer:rest-printer-b");
    REQUIRE(printer != selected_sections.not_found());
    CHECK(printer->second.get<std::string>("default_filament_profile") == "rest-filament-b");
    const HttpResponseMessage saved_selection = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved_selection.result() == http::status::ok);
    std::ofstream(path, std::ios::binary) << saved_selection.body();
    Slic3r::DynamicPrintConfig selected_configuration;
    std::string saved_sections;
    Slic3r::Model::read_from_archive(path.u8string(), &selected_configuration, &substitutions,
        Slic3r::Model::LoadAttribute::AddDefaultInstances, &saved_sections);
    CHECK(saved_sections == selected.profile_sections);
}

TEST_CASE("REST save_project chooses portable attachment names", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = "/session/" + create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).size() == 1);
    const auto check_name = [&](const std::string &query, const std::string &expected) {
        const HttpResponseMessage response = fixture.request(http::verb::get, route + "/save_project" + query);
        REQUIRE(response.result() == http::status::ok);
        CHECK(response[http::field::content_disposition] == Slic3r::Rest::content_disposition_attachment(expected));
        CHECK(wait_for_work_file_count(fixture, 0) == 0);
    };
    check_name("", "project.3mf");
    REQUIRE(fixture.request(http::verb::post, route + "/set_name", R"({"name":"Bracket"})",
        "application/json").result() == http::status::ok);
    check_name("", "Bracket.3mf");
    check_name("?file_name=pi%C3%A8ce%3Av2", u8"pi\u00e8ce_v2.3mf");
    check_name("?file_name=nul", "_nul.3mf");
    check_name("?file_name=Part.3MF", "Part.3MF");
    check_name("?file_name=Support+bracket", "Support bracket.3mf");
    check_name("?file_name=a%2Bb", "a+b.3mf");
    check_name("?&&file%5Fname=Part&&", "Part.3mf");
    check_name("?file_name=...", "Bracket.3mf");
    REQUIRE(fixture.request(http::verb::post, route + "/set_name", R"({"name":"..."})",
        "application/json").result() == http::status::ok);
    check_name("?file_name=...", "project.3mf");
}

TEST_CASE("REST save_project refuses invalid requests without mutation or temporary files", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    require_error(fixture.request(http::verb::get, route + "/save_project"), http::status::conflict, "empty_scene");
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).size() == 1);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const Slic3r::Rest::SessionContext::Snapshot before = session->snapshot();
    const nlohmann::json listed = read_objects(fixture, route);
    require_error(fixture.request(http::verb::post, route + "/save_project", "bad"), http::status::not_found, "route_not_found");
    require_error(fixture.request(http::verb::get, route + "/save_project/extra"), http::status::not_found, "route_not_found");
    require_error(fixture.request(http::verb::get, route + "/save_project", "bad"), http::status::bad_request, "invalid_request");
    for (const std::string &query : {"?unknown=x", "?file_name=x&file%5Fname=y", "?file_name=%", "?file_name=%G1",
            "?file_name=", "?file_name", "?file_name=%00", "?file_name=x&unknown=y"})
        require_error(fixture.request(http::verb::get, route + "/save_project" + query), http::status::bad_request, "invalid_request");
    require_error(fixture.request_without_auth(http::verb::get, route + "/save_project"), http::status::unauthorized, "unauthorized");
    const std::string bob = fixture.login_token("Bob", "secret");
    require_error(fixture.request(http::verb::get, route + "/save_project", {}, {},
        {{"Authorization", "Bearer " + bob}}), http::status::not_found, "session_not_found");
    require_error(fixture.request(http::verb::get, "/session/missing/save_project"), http::status::not_found, "session_not_found");
    CHECK(session->snapshot().revision == before.revision);
    CHECK(session->snapshot().status == before.status);
    CHECK(read_objects(fixture, route) == listed);
    CHECK_FALSE(session->data_snapshot()->name.has_value());
    CHECK(fixture.work_file_count() == 0);
}

TEST_CASE("REST save_project snapshot survives deletion while the network stays responsive", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    // Shared with the observer: an import thread may still run it while a
    // failed test unwinds, after the test's own locals are gone.
    const std::shared_ptr<std::promise<void>> entered = std::make_shared<std::promise<void>>();
    const std::shared_ptr<std::promise<void>> release = std::make_shared<std::promise<void>>();
    const std::shared_future<void> ready = release->get_future().share();
    fixture.set_import_observer([entered, ready]() {
        entered->set_value();
        ready.wait_for(std::chrono::seconds(10));
    });
    fixture.start();
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    Slic3r::Rest::SessionData data = session->data_snapshot().value();
    data.scene = std::make_shared<Slic3r::Model>();
    data.scene->add_object("cube", "cube.stl", Slic3r::make_cube(10, 10, 10));
    data.scene->add_default_instances();
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    std::future<HttpResponseMessage> saved = std::async(std::launch::async, [&]() {
        return fixture.request(http::verb::get, route + "/save_project");
    });
    const std::future_status started = entered->get_future().wait_for(std::chrono::seconds(5));
    if (started != std::future_status::ready) {
        release->set_value();
        REQUIRE(started == std::future_status::ready);
    }
    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    const Slic3r::Rest::SessionEraseResult erased = fixture.sessions().erase(
        Slic3r::Rest::normalize_username("Alice"), id);
    release->set_value();
    CHECK(created.result() == http::status::created);
    CHECK(erased == Slic3r::Rest::SessionEraseResult::erased);
    const HttpResponseMessage response = saved.get();
    CHECK(response.result() == http::status::ok);
    CHECK(response.body().substr(0, 2) == "PK");
    CHECK(wait_for_work_file_count(fixture, 0) == 0);
}

TEST_CASE("REST save_project storage errors are generic and leave the session intact", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).size() == 1);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const Slic3r::Rest::SessionContext::Snapshot before = session->snapshot();
    const std::filesystem::path exports = fixture.work_directory() / "exports";
    std::filesystem::rename(exports, fixture.work_directory() / "exports-backup");
    std::ofstream(exports) << "not a directory";
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/save_project");
    require_error(response, http::status::internal_server_error, "internal_error");
    CHECK(response.body().find(fixture.work_directory().string()) == std::string::npos);
    CHECK(session->snapshot().revision == before.revision);
    CHECK(session->snapshot().status == before.status);
    CHECK(fixture.work_file_count() == 1);
}

TEST_CASE("REST save_project cleans abandoned responses and stale exports", "[rest_save_project]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::filesystem::path exports = fixture.work_directory() / "exports";
    std::ofstream(exports / "rest-export-stale.3mf") << "stale";
    std::ofstream(exports / "keep.txt") << "keep";
    fixture.start();
    CHECK_FALSE(std::filesystem::exists(exports / "rest-export-stale.3mf"));
    CHECK(read_file(exports / "keep.txt") == "keep");
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).size() == 1);
    asio::io_context context;
    Tcp::socket socket = fixture.connect_socket(context);
    const std::string request = "GET " + route + "/save_project HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer " +
        fixture.token() + "\r\nConnection: close\r\n\r\n";
    asio::write(socket, asio::buffer(request));
    char byte;
    socket.read_some(asio::buffer(&byte, 1));
    ErrorCode ignored;
    socket.set_option(asio::socket_base::linger(true, 0), ignored);
    socket.close(ignored);
    // keep.txt is the only remaining file once the export is released.
    CHECK(wait_for_work_file_count(fixture, 1) == 1);
    CHECK(read_file(exports / "keep.txt") == "keep");
}

TEST_CASE("REST startup reports export directory failures as paths.work errors", "[rest_save_project]")
{
    RestServerFixture fixture;
    fixture.stop();
    const std::filesystem::path exports = fixture.work_directory() / "exports";
    std::filesystem::rename(exports, fixture.work_directory() / "exports-backup");
    std::ofstream(exports) << "not a directory";
    CHECK_THROWS_AS(fixture.start(), std::invalid_argument);
    CHECK_THROWS_WITH(fixture.start(), Catch::Matchers::StartsWith("paths.work: cannot create the export directory:"));
}

TEST_CASE("REST transforms instances and objects atomically with explicit bases and bed placement", "[rest_transform]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    Slic3r::Rest::SessionData data = session->data_snapshot().value();
    data.scene = std::make_shared<Slic3r::Model>();
    Slic3r::ModelObject *object = data.scene->add_object("cube", "cube.stl", Slic3r::make_cube(10,10,10));
    Slic3r::Transform3d initial = Slic3r::Transform3d::Identity();
    initial.linear() << 0,-1,0, 1,0,0, 0,0,1;
    initial.translation() = Slic3r::Vec3d(10,20,3);
    object->add_instance(Slic3r::Geometry::Transformation(initial));
    object->add_instance()->set_offset(Slic3r::Vec3d(30,40,7));
    object->add_instance()->set_scaling_factor(Slic3r::Vec3d(2,1,1));
    std::vector<Slic3r::Rest::EntityBinding> bindings{
        {Slic3r::Rest::EntityKind::object, object->id().id},
        {Slic3r::Rest::EntityKind::volume, object->volumes.front()->id().id}
    };
    for (const Slic3r::ModelInstance *instance : object->instances)
        bindings.push_back({Slic3r::Rest::EntityKind::instance, instance->id().id});
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true,
        Slic3r::Rest::SessionRegistry::Clock::now(), bindings, ids));
    const nlohmann::json original = read_objects(fixture, route);
    const std::string object_id = original.at("objects").at(0).at("id");
    const std::string instance_id = original.at("objects").at(0).at("instances").at(0).at("id");
    const auto post = [&](const std::string &id, const nlohmann::json &body) {
        return fixture.request(http::verb::post, route + "/transform/" + id, body.dump(), "application/json");
    };
    Slic3r::Transform3d translation = Slic3r::Transform3d::Identity();
    translation.translation().x() = 5;
    nlohmann::json body = transform_body(translation, "platter");
    body.erase("action");
    body.erase("ensure_on_bed");
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::string result_id = session->viewer_snapshot()->result_id;
    const HttpResponseMessage global = post(instance_id, body);
    REQUIRE(global.result() == http::status::ok);
    CHECK(nlohmann::json::parse(global.body()) == nlohmann::json{
        {"id", instance_id}, {"action", "compose"}, {"space", "platter"}, {"matrix", body.at("matrix")}});
    nlohmann::json listed = read_objects(fixture, route);
    CHECK((listed_matrix(listed["objects"][0]["instances"][0]["matrix"]) * Slic3r::Vec3d(1,0,0)).isApprox(Slic3r::Vec3d(15,21,3)));
    CHECK(listed["objects"][0]["instances"][1] == original["objects"][0]["instances"][1]);
    CHECK(listed["objects"][0]["instances"][2] == original["objects"][0]["instances"][2]);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK_FALSE(session->current_gcode().has_value());
    CHECK(session->viewer_snapshot()->outdated);
    CHECK(session->viewer_snapshot()->result_id == result_id);
    REQUIRE(post(instance_id, transform_body(translation, "object")).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK((listed_matrix(listed["objects"][0]["instances"][0]["matrix"]) * Slic3r::Vec3d(1,0,0)).isApprox(Slic3r::Vec3d(15,26,3)));

    // The third instance's non-uniform scale makes a local rotation skewed; none may be published.
    Slic3r::Transform3d rotation = Slic3r::Transform3d::Identity();
    const double diagonal = std::sqrt(0.5);
    rotation.linear() << diagonal,-diagonal,0, diagonal,diagonal,0, 0,0,1;
    const std::uint64_t revision = session->snapshot().revision;
    require_error(post(object_id, transform_body(rotation, "object")), http::status::unprocessable_entity, "invalid_transform");
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == listed);

    REQUIRE(post(object_id, transform_body(translation, "platter")).result() == http::status::ok);
    const nlohmann::json translated = read_objects(fixture, route);
    for (std::size_t index = 0; index < 3; ++index) {
        const Slic3r::Transform3d previous = listed_matrix(listed["objects"][0]["instances"][index]["matrix"]);
        const Slic3r::Transform3d actual = listed_matrix(translated["objects"][0]["instances"][index]["matrix"]);
        CHECK((actual * Slic3r::Vec3d(1,2,3)).isApprox(translation * previous * Slic3r::Vec3d(1,2,3)));
        CHECK(translated["objects"][0]["instances"][index]["id"] == listed["objects"][0]["instances"][index]["id"]);
    }
    REQUIRE(post(instance_id, transform_body(rotation, "volume", "replace")).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK(listed_matrix(listed["objects"][0]["instances"][0]["matrix"]).matrix().isApprox(rotation.matrix()));
    CHECK(listed["objects"][0]["parts"] == original["objects"][0]["parts"]);
    CHECK(session->data_snapshot()->configuration.diff(data.configuration).empty());

    Slic3r::Transform3d lifted = Slic3r::Transform3d::Identity();
    lifted.translation().z() = 15;
    REQUIRE(post(object_id, transform_body(lifted, "platter", "replace")).result() == http::status::ok);
    listed = read_objects(fixture, route);
    for (const nlohmann::json &instance : listed["objects"][0]["instances"])
        CHECK(listed_matrix(instance.at("matrix")).matrix().isApprox(lifted.matrix()));
    REQUIRE(post(object_id, transform_body(lifted, "object", "replace", true)).result() == http::status::ok);
    const Slic3r::Rest::SessionData placed = session->data_snapshot().value();
    for (std::size_t index = 0; index < 3; ++index)
        CHECK(placed.scene->object_ptrs().front()->get_instance_min_z(index) == Approx(0).margin(1e-8));
    for (const std::string &id : ids)
        CHECK(session->resolve_entity(id).has_value());

    // Tilted and sunk geometry is placed by its actual lowest model point, not its origin.
    Slic3r::Transform3d tilted = Slic3r::Transform3d::Identity();
    tilted.linear() << 1,0,0, 0,diagonal,-diagonal, 0,diagonal,diagonal;
    tilted.translation().z() = -12;
    REQUIRE(post(instance_id, transform_body(tilted, "platter", "replace", true)).result() == http::status::ok);
    CHECK(session->data_snapshot()->scene->object_ptrs().front()->get_instance_min_z(0) == Approx(0).margin(1e-8));

    Slic3r::Transform3d mirrored = Slic3r::Transform3d::Identity();
    mirrored.linear() << -1,0,0, 0,2,0, 0,0,3;
    REQUIRE(post(instance_id, transform_body(mirrored, "volume", "replace")).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK((listed_matrix(listed["objects"][0]["instances"][0]["matrix"]) * Slic3r::Vec3d(1,2,3)).isApprox(Slic3r::Vec3d(-1,4,9)));
}

TEST_CASE("REST instance counts duplicate the current original and preserve retained IDs", "[rest_instances]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.max_instances_per_object = 5;
    RestServerFixture fixture(limits, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).front();
    object_ids(upload_model(fixture, route + "/add_model", "other.obj", make_obj_with_padding()));
    const std::string target = route + "/set_instance_count/" + object_id;
    nlohmann::json before = read_objects(fixture, route);
    const std::string original_id = before["objects"][0]["instances"][0]["id"];
    Slic3r::Transform3d original = Slic3r::Transform3d::Identity();
    original.linear() << 0,-1,0, 1,0,0, 0,0,1;
    original.translation() = Slic3r::Vec3d(10,20,7);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + original_id,
        transform_body(original, "platter", "replace").dump(), "application/json").result() == http::status::ok);
    before = read_objects(fixture, route);
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    const std::uint64_t revision = session->snapshot().revision;
    const auto set_count = [&](std::uint64_t count) {
        const HttpResponseMessage response = fixture.request(http::verb::post, target,
            nlohmann::json{{"count", count}}.dump(), "application/json");
        REQUIRE(response.result() == http::status::ok);
        const nlohmann::json body = nlohmann::json::parse(response.body());
        CHECK(body.size() == 3);
        CHECK(body.at("object_id") == object_id);
        CHECK(body.at("count") == count);
        return body.at("instance_ids").get<std::vector<std::string>>();
    };
    CHECK(set_count(1) == std::vector<std::string>{original_id});
    CHECK(session->snapshot().revision == revision);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
    CHECK(session->current_gcode().value() == std::filesystem::path("result.gcode"));
    CHECK(session->viewer_snapshot() == viewer);
    CHECK(read_objects(fixture, route) == before);

    const std::vector<std::string> three_ids = set_count(3);
    REQUIRE(three_ids.size() == 3);
    CHECK(three_ids.front() == original_id);
    CHECK(std::set<std::string>(three_ids.begin(), three_ids.end()).size() == 3);
    nlohmann::json listed = read_objects(fixture, route);
    REQUIRE(listed["objects"][0]["instances"].size() == 3);
    for (std::size_t index = 0; index < 3; ++index) {
        CHECK(listed["objects"][0]["instances"][index]["id"] == three_ids[index]);
        CHECK(listed["objects"][0]["instances"][index]["matrix"] == before["objects"][0]["instances"][0]["matrix"]);
    }
    CHECK(listed["objects"][0]["parts"] == before["objects"][0]["parts"]);
    CHECK(listed["objects"][1] == before["objects"][1]);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK_FALSE(session->current_gcode());
    CHECK(session->viewer_snapshot()->outdated);
    CHECK(session->viewer_snapshot()->result_id == viewer->result_id);
    Slic3r::Transform3d moved_copy = Slic3r::Transform3d::Identity();
    moved_copy.translation() = Slic3r::Vec3d(70,80,9);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + three_ids[1],
        transform_body(moved_copy, "platter", "replace").dump(), "application/json").result() == http::status::ok);
    original.linear() << 1,0,0, 0,0,-1, 0,1,0;
    original.translation() = Slic3r::Vec3d(30,40,11);
    REQUIRE(fixture.request(http::verb::post, route + "/transform/" + original_id,
        transform_body(original, "platter", "replace").dump(), "application/json").result() == http::status::ok);
    const nlohmann::json moved = read_objects(fixture, route);
    const std::vector<std::string> four_ids = set_count(4);
    REQUIRE(four_ids.size() == 4);
    CHECK(std::equal(three_ids.begin(), three_ids.end(), four_ids.begin()));
    CHECK(std::set<std::string>(four_ids.begin(), four_ids.end()).size() == 4);
    listed = read_objects(fixture, route);
    for (std::size_t index = 0; index < 3; ++index)
        CHECK(listed["objects"][0]["instances"][index] == moved["objects"][0]["instances"][index]);
    CHECK(listed["objects"][0]["instances"][3]["matrix"] == moved["objects"][0]["instances"][0]["matrix"]);
    CHECK(listed["objects"][0]["instances"][3]["matrix"] != moved["objects"][0]["instances"][1]["matrix"]);
    const std::uint64_t unchanged_revision = session->snapshot().revision;
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> outdated = session->viewer_snapshot();
    CHECK(set_count(4) == four_ids);
    CHECK(session->snapshot().revision == unchanged_revision);
    CHECK(session->viewer_snapshot() == outdated);
    CHECK(set_count(1) == std::vector<std::string>{original_id});
    const nlohmann::json reduced = read_objects(fixture, route);
    CHECK(reduced["objects"][0]["instances"] == nlohmann::json::array({moved["objects"][0]["instances"][0]}));
    CHECK(reduced["objects"][1] == before["objects"][1]);
    for (std::size_t index = 1; index < four_ids.size(); ++index) {
        CHECK_FALSE(session->resolve_entity(four_ids[index]));
        require_error(fixture.request(http::verb::post, route + "/transform/" + four_ids[index],
            transform_body(Slic3r::Transform3d::Identity(), "platter").dump(), "application/json"),
            http::status::not_found, "entity_not_found");
    }
    const std::vector<std::string> regrown = set_count(5);
    REQUIRE(regrown.size() == 5);
    CHECK(std::set<std::string>(regrown.begin(), regrown.end()).size() == 5);
    for (std::size_t index = 1; index < four_ids.size(); ++index)
        CHECK(std::find(regrown.begin(), regrown.end(), four_ids[index]) == regrown.end());
}

TEST_CASE("REST instance count errors respect the configured limit without mutations", "[rest_instances]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.max_instances_per_object = 3;
    RestServerFixture fixture(limits, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const std::string object_id = object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).front();
    const std::string target = route + "/set_instance_count/" + object_id;
    const nlohmann::json before = read_objects(fixture, route);
    const std::uint64_t revision = session->snapshot().revision;
    const std::vector<std::pair<nlohmann::json, unsigned int>> failures{
        {{{"count", 0}}, 422}, {{{"count", -1}}, 422}, {{{"count", 4}}, 422},
        {{{"count", 10000000}}, 422},
        {{{"count", std::numeric_limits<std::uint64_t>::max()}}, 422},
        {{{"count", std::numeric_limits<std::int64_t>::min()}}, 422},
        {{{"count", 1.5}}, 400}, {{{"count", 1.0}}, 400}, {{{"count", "2"}}, 400},
        {{{"count", true}}, 400}, {{{"count", nullptr}}, 400},
        {nlohmann::json::object(), 400}, {nlohmann::json::array({1}), 400},
        {{{"count", 2}, {"extra", 0}}, 400}
    };
    for (const std::pair<nlohmann::json, unsigned int> &failure : failures) {
        CAPTURE(failure.first.dump());
        const HttpResponseMessage response = fixture.request(http::verb::post, target, failure.first.dump(), "application/json");
        CHECK(response.result_int() == failure.second);
        CHECK(nlohmann::json::parse(response.body())["error"]["code"] ==
            (failure.second == 400 ? "invalid_request" : "invalid_instance_count"));
        CHECK(session->snapshot().revision == revision);
    }
    require_error(fixture.request(http::verb::post, target, "{", "application/json"), http::status::bad_request, "invalid_json");
    require_error(fixture.request(http::verb::post, target, {}, "application/json"), http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::post, target, "{\"count\":2}", "text/plain"), http::status::bad_request, "invalid_request");
    const std::string part_id = before["objects"][0]["parts"][0]["id"];
    const std::string instance_id = before["objects"][0]["instances"][0]["id"];
    for (const std::string &id : {part_id, instance_id})
        require_error(fixture.request(http::verb::post, route + "/set_instance_count/" + id,
            "{\"count\":2}", "application/json"), http::status::unprocessable_entity, "invalid_instance_count");
    require_error(fixture.request(http::verb::post, route + "/set_instance_count/missing",
        "{\"count\":2}", "application/json"), http::status::not_found, "entity_not_found");
    CHECK(fixture.request_without_auth(http::verb::post, target, "{\"count\":2}", "application/json").result() == http::status::unauthorized);
    const std::string bob_token = fixture.login_token("Bob", "secret");
    CHECK(fixture.request(http::verb::post, target, "{\"count\":2}", "application/json",
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::get, target).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, target + "/extra", "{\"count\":2}", "application/json").result() == http::status::not_found);
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == before);
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    const std::uint64_t busy_revision = session->snapshot().revision;
    for (int count : {1,2})
        require_error(fixture.request(http::verb::post, target, nlohmann::json{{"count", count}}.dump(), "application/json"),
            http::status::conflict, "session_busy");
    // An invalid request gets the same answer whether or not the session is slicing.
    require_error(fixture.request(http::verb::post, target, "{", "application/json"), http::status::bad_request, "invalid_json");
    require_error(fixture.request(http::verb::post, target, "{\"count\":0}", "application/json"),
        http::status::unprocessable_entity, "invalid_instance_count");
    CHECK(session->snapshot().revision == busy_revision);
    CHECK(read_objects(fixture, route) == before);
    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
}

TEST_CASE("REST volume transforms use reference axes and retain shared-instance geometry", "[rest_transform][rest_volume_transform]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    Slic3r::Rest::SessionData data = session->data_snapshot().value();
    data.scene = std::make_shared<Slic3r::Model>();
    Slic3r::ModelObject *object = data.scene->add_object("multipart", "part.stl", Slic3r::make_cube(10,10,10));
    object->add_volume(Slic3r::make_cube(4,4,4))->set_offset(Slic3r::Vec3d(5,0,0));
    Slic3r::Transform3d volume = Slic3r::Transform3d::Identity();
    volume.linear() << 1,0,0, 0,0,-1, 0,1,0;
    volume.translation() = Slic3r::Vec3d(1,2,3);
    object->volumes.front()->set_transformation(volume);
    object->add_instance()->set_offset(Slic3r::Vec3d(10,20,30));
    Slic3r::Transform3d rotated = Slic3r::Transform3d::Identity();
    rotated.linear() << 0,-1,0, 1,0,0, 0,0,1;
    rotated.translation() = Slic3r::Vec3d(40,50,60);
    object->add_instance(Slic3r::Geometry::Transformation(rotated));
    Slic3r::ModelObject *other = data.scene->add_object("other", "other.stl", Slic3r::make_cube(2,2,2));
    other->add_instance();
    std::vector<Slic3r::Rest::EntityBinding> bindings;
    for (const Slic3r::ModelObject *item : data.scene->object_ptrs()) {
        bindings.push_back({Slic3r::Rest::EntityKind::object, item->id().id});
        for (const Slic3r::ModelVolume *part : item->volumes)
            bindings.push_back({Slic3r::Rest::EntityKind::volume, part->id().id});
        for (const Slic3r::ModelInstance *instance : item->instances)
            bindings.push_back({Slic3r::Rest::EntityKind::instance, instance->id().id});
    }
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true,
        Slic3r::Rest::SessionRegistry::Clock::now(), bindings, ids));
    const nlohmann::json before = read_objects(fixture, route);
    const std::string part_id = before["objects"][0]["parts"][0]["id"];
    const std::string reference_id = before["objects"][0]["instances"][1]["id"];
    const std::string target = route + "/transform/" + part_id;
    const auto post = [&](const nlohmann::json &body) {
        return fixture.request(http::verb::post, target, body.dump(), "application/json");
    };
    const auto world_point = [](const nlohmann::json &listing, std::size_t instance) -> Slic3r::Vec3d {
        const Slic3r::Transform3d placement = listed_matrix(listing["objects"][0]["instances"][instance]["matrix"]);
        const Slic3r::Transform3d local = listed_matrix(listing["objects"][0]["parts"][0]["matrix"]);
        return placement * local * Slic3r::Vec3d(1,2,3);
    };
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::string result_id = session->viewer_snapshot()->result_id;
    Slic3r::Transform3d translation = Slic3r::Transform3d::Identity();
    translation.translation().x() = 5;
    nlohmann::json body = transform_body(translation, "platter");
    const HttpResponseMessage response = post(body);
    REQUIRE(response.result() == http::status::ok);
    CHECK(nlohmann::json::parse(response.body()) == nlohmann::json{
        {"id", part_id}, {"action", "compose"}, {"space", "platter"}, {"matrix", body.at("matrix")}});
    nlohmann::json listed = read_objects(fixture, route);
    CHECK(world_point(listed, 0).isApprox(Slic3r::Vec3d(17,19,35)));
    CHECK(world_point(listed, 1).isApprox(Slic3r::Vec3d(41,57,65)));
    CHECK(listed["objects"][0]["parts"][1] == before["objects"][0]["parts"][1]);
    CHECK(listed["objects"][0]["instances"] == before["objects"][0]["instances"]);
    CHECK(listed["objects"][1] == before["objects"][1]);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK_FALSE(session->current_gcode());
    CHECK(session->viewer_snapshot()->outdated);
    CHECK(session->viewer_snapshot()->result_id == result_id);

    body["instance"] = reference_id;
    REQUIRE(post(body).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK(world_point(listed, 0).isApprox(Slic3r::Vec3d(17,14,35)));
    CHECK(world_point(listed, 1).isApprox(Slic3r::Vec3d(46,57,65)));
    CHECK(listed["objects"][0]["instances"] == before["objects"][0]["instances"]);

    Slic3r::Transform3d absolute = Slic3r::Transform3d::Identity();
    absolute.linear() << 0,0,1, 0,1,0, -1,0,0;
    absolute.translation() = Slic3r::Vec3d(4,5,6);
    body = transform_body(absolute, "platter", "replace");
    body["instance"] = reference_id;
    REQUIRE(post(body).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK(world_point(listed, 1).isApprox(Slic3r::Vec3d(7,7,5)));
    CHECK(listed["objects"][0]["instances"] == before["objects"][0]["instances"]);
    body = transform_body(absolute, "object", "replace");
    REQUIRE(post(body).result() == http::status::ok);
    listed = read_objects(fixture, route);
    CHECK(listed_matrix(listed["objects"][0]["parts"][0]["matrix"]).matrix().isApprox(absolute.matrix()));
    CHECK(world_point(listed, 0).isApprox(Slic3r::Vec3d(17,27,35)));
    CHECK(world_point(listed, 1).isApprox(Slic3r::Vec3d(33,57,65)));
    body = transform_body(translation, "volume");
    REQUIRE(post(body).result() == http::status::ok);
    listed = read_objects(fixture, route);
    const Slic3r::Transform3d expected_local = absolute * translation;
    CHECK(listed_matrix(listed["objects"][0]["parts"][0]["matrix"]).matrix().isApprox(expected_local.matrix()));

    body = transform_body(Slic3r::Transform3d::Identity(), "object", "compose", true);
    REQUIRE(post(body).result() == http::status::ok);
    listed = read_objects(fixture, route);
    const Slic3r::Rest::SessionData placed = session->data_snapshot().value();
    for (std::size_t index = 0; index < 2; ++index) {
        CHECK(placed.scene->object_ptrs().front()->get_instance_min_z(index) == Approx(0).margin(1e-8));
        const Slic3r::Transform3d old_instance = listed_matrix(before["objects"][0]["instances"][index]["matrix"]);
        const Slic3r::Transform3d new_instance = listed_matrix(listed["objects"][0]["instances"][index]["matrix"]);
        CHECK(old_instance.linear().isApprox(new_instance.linear()));
        CHECK(old_instance.translation().x() == new_instance.translation().x());
        CHECK(old_instance.translation().y() == new_instance.translation().y());
        CHECK(listed["objects"][0]["instances"][index]["id"] == before["objects"][0]["instances"][index]["id"]);
    }
    CHECK(listed["objects"][0]["parts"][1] == before["objects"][0]["parts"][1]);
    CHECK(listed["objects"][1] == before["objects"][1]);
    CHECK(session->data_snapshot()->configuration.diff(data.configuration).empty());
    for (const std::string &id : ids)
        CHECK(session->resolve_entity(id).has_value());
}

TEST_CASE("REST volume transform errors are atomic across all parent instances", "[rest_transform][rest_volume_transform]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl()));
    Slic3r::Rest::SessionData candidate = session->data_snapshot().value();
    candidate.scene = std::make_shared<Slic3r::Model>(*candidate.scene);
    Slic3r::ModelObject *object = candidate.scene->object_ptrs().front();
    Slic3r::ModelInstance *scaled = object->add_instance();
    scaled->set_scaling_factor(Slic3r::Vec3d(2,1,1));
    Slic3r::ModelObject *other = candidate.scene->add_object("other", "other.stl", Slic3r::make_cube(2,2,2));
    other->add_instance();
    const std::vector<Slic3r::Rest::EntityBinding> bindings{
        {Slic3r::Rest::EntityKind::instance, scaled->id().id},
        {Slic3r::Rest::EntityKind::object, other->id().id},
        {Slic3r::Rest::EntityKind::volume, other->volumes.front()->id().id},
        {Slic3r::Rest::EntityKind::instance, other->instances.front()->id().id}
    };
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), candidate, true,
        Slic3r::Rest::SessionRegistry::Clock::now(), bindings, ids));
    const nlohmann::json before = read_objects(fixture, route);
    const std::string object_id = before["objects"][0]["id"];
    const std::string part_id = before["objects"][0]["parts"][0]["id"];
    const std::string instance_id = before["objects"][0]["instances"][0]["id"];
    const std::string other_instance = before["objects"][1]["instances"][0]["id"];
    const std::string target = route + "/transform/" + part_id;
    const auto post = [&](const nlohmann::json &body) {
        return fixture.request(http::verb::post, target, body.dump(), "application/json");
    };
    const Slic3r::Rest::SessionContext::WorkToken finished = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(finished, "current.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::uint64_t revision = session->snapshot().revision;
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    Slic3r::Transform3d rotation = Slic3r::Transform3d::Identity();
    const double diagonal = std::sqrt(0.5);
    rotation.linear() << diagonal,-diagonal,0, diagonal,diagonal,0, 0,0,1;
    require_error(post(transform_body(rotation, "object")), http::status::unprocessable_entity, "invalid_transform");
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == before);
    Slic3r::Transform3d singular = Slic3r::Transform3d::Identity();
    singular.linear()(0,0) = 0;
    require_error(post(transform_body(singular, "object")), http::status::unprocessable_entity, "invalid_transform");
    const nlohmann::json valid = transform_body(Slic3r::Transform3d::Identity(), "platter");
    for (const std::string &space : {"platter", "object", "volume"})
        for (const std::string &bad_reference : {object_id, part_id, other_instance}) {
            CAPTURE(space, bad_reference);
            nlohmann::json body = valid;
            body["space"] = space;
            body["instance"] = bad_reference;
            require_error(post(body), http::status::unprocessable_entity, "invalid_transform");
            CHECK(session->snapshot().revision == revision);
        }
    nlohmann::json body = valid;
    body["instance"] = "missing";
    require_error(post(body), http::status::not_found, "entity_not_found");
    body["instance"] = 1;
    require_error(post(body), http::status::bad_request, "invalid_request");
    body["instance"] = instance_id;
    for (const std::string &target_id : {object_id, instance_id})
        require_error(fixture.request(http::verb::post, route + "/transform/" + target_id,
            body.dump(), "application/json"), http::status::unprocessable_entity, "invalid_transform");
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == before);
    CHECK(session->viewer_snapshot() == viewer);
    CHECK(session->current_gcode().value() == std::filesystem::path("current.gcode"));
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    require_error(post(valid), http::status::conflict, "session_busy");
    CHECK(read_objects(fixture, route) == before);
    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
}

TEST_CASE("REST local mirroring preserves an arbitrary 3D rotation", "[rest_transform][rest_mirror_3d]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl()));
    Slic3r::Rest::SessionData candidate = session->data_snapshot().value();
    candidate.scene = std::make_shared<Slic3r::Model>(*candidate.scene);
    Slic3r::ModelObject *object = candidate.scene->object_ptrs().front();
    Slic3r::Transform3d rotated = Slic3r::Transform3d::Identity();
    rotated.linear() = Eigen::AngleAxisd(0.7, Slic3r::Vec3d(1,2,3).normalized()).toRotationMatrix();
    rotated.translation() = Slic3r::Vec3d(10,20,30);
    object->instances.front()->set_transformation(Slic3r::Geometry::Transformation(rotated));
    Slic3r::ModelInstance *second = object->add_instance();
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), candidate, true,
        Slic3r::Rest::SessionRegistry::Clock::now(),
        {{Slic3r::Rest::EntityKind::instance, second->id().id}}, ids));
    const nlohmann::json before = read_objects(fixture, route);
    const std::string instance_id = before["objects"][0]["instances"][0]["id"];
    Slic3r::Transform3d mirror = Slic3r::Transform3d::Identity();
    mirror.linear()(0,0) = -1;
    const nlohmann::json body = transform_body(mirror, "object");
    const HttpResponseMessage response = fixture.request(http::verb::post,
        route + "/transform/" + instance_id, body.dump(), "application/json");
    REQUIRE(response.result() == http::status::ok);
    CHECK(nlohmann::json::parse(response.body()).at("matrix") == body.at("matrix"));
    const nlohmann::json after = read_objects(fixture, route);
    const Slic3r::Transform3d actual = listed_matrix(after["objects"][0]["instances"][0]["matrix"]);
    const Slic3r::Transform3d expected = rotated * mirror;
    CHECK(actual.matrix().isApprox(expected.matrix()));
    for (const Slic3r::Vec3d &point : {Slic3r::Vec3d(1,2,3), Slic3r::Vec3d(-4,5,6)})
        CHECK((actual * point).isApprox(expected * point));
    CHECK(actual.linear().determinant() == Approx(-1));
    CHECK(after["objects"][0]["id"] == before["objects"][0]["id"]);
    CHECK(after["objects"][0]["parts"] == before["objects"][0]["parts"]);
    CHECK(after["objects"][0]["instances"][0]["id"] == instance_id);
    CHECK(after["objects"][0]["instances"][1] == before["objects"][0]["instances"][1]);
}

TEST_CASE("REST transform refusals leave scene revision and results untouched", "[rest_transform]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_stl()));
    const nlohmann::json before = read_objects(fixture, route);
    const std::string object_id = before["objects"][0]["id"];
    const std::string part_id = before["objects"][0]["parts"][0]["id"];
    const std::string target = route + "/transform/" + object_id;
    const nlohmann::json valid = transform_body(Slic3r::Transform3d::Identity(), "platter");
    const Slic3r::Rest::SessionContext::WorkToken finished = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(finished, "current.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::uint64_t revision = session->snapshot().revision;
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    std::vector<std::pair<nlohmann::json, unsigned int>> failures;
    nlohmann::json bad = valid;
    bad["pivot"] = {1,2,3};
    failures.emplace_back(bad, 400);
    bad = valid;
    bad["ensure_on_bed"] = "true";
    failures.emplace_back(bad, 400);
    bad = valid;
    bad.erase("space");
    failures.emplace_back(bad, 400);
    bad = valid;
    bad["matrix"] = {1,2,3};
    failures.emplace_back(bad, 422);
    bad = valid;
    bad["matrix"][0] = 0;
    failures.emplace_back(bad, 422);
    bad = valid;
    bad["matrix"][12] = 1;
    failures.emplace_back(bad, 422);
    bad = valid;
    bad["action"] = "unknown";
    failures.emplace_back(bad, 422);
    bad = valid;
    bad["space"] = "unknown";
    failures.emplace_back(bad, 422);
    for (const std::pair<nlohmann::json, unsigned int> &failure : failures) {
        const HttpResponseMessage response = fixture.request(http::verb::post, target, failure.first.dump(), "application/json");
        CHECK(response.result_int() == failure.second);
        CHECK(nlohmann::json::parse(response.body())["error"]["code"] == (failure.second == 400 ? "invalid_request" : "invalid_transform"));
    }
    const HttpResponseMessage malformed = fixture.request(http::verb::post, target, "{", "application/json");
    require_error(malformed, http::status::bad_request, "invalid_json");
    CHECK(nlohmann::json::parse(malformed.body()).at("error").at("details") ==
        nlohmann::json::array({nlohmann::json{{"reason", "invalid_json"}}}));
    require_error(fixture.request(http::verb::post, target, {}, "application/json"), http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::post, target, valid.dump(), "text/plain"), http::status::bad_request, "invalid_request");
    nlohmann::json wrong_reference = valid;
    wrong_reference["instance"] = object_id;
    require_error(fixture.request(http::verb::post, route + "/transform/" + part_id, wrong_reference.dump(), "application/json"),
        http::status::unprocessable_entity, "invalid_transform");
    require_error(fixture.request(http::verb::post, route + "/transform/missing", valid.dump(), "application/json"),
        http::status::not_found, "entity_not_found");
    CHECK(fixture.request(http::verb::get, target).result() == http::status::not_found);
    CHECK(fixture.request_without_auth(http::verb::post, target, valid.dump(), "application/json").result() == http::status::unauthorized);
    const std::string bob_token = fixture.login_token("Bob", "secret");
    CHECK(fixture.request(http::verb::post, target, valid.dump(), "application/json",
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == before);
    CHECK(session->viewer_snapshot() == viewer);
    CHECK(session->current_gcode().value() == std::filesystem::path("current.gcode"));
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    require_error(fixture.request(http::verb::post, target, valid.dump(), "application/json"), http::status::conflict, "session_busy");
    // An invalid request gets the same answer whether or not the session is slicing.
    nlohmann::json busy_invalid = valid;
    busy_invalid["space"] = "unknown";
    require_error(fixture.request(http::verb::post, target, busy_invalid.dump(), "application/json"),
        http::status::unprocessable_entity, "invalid_transform");
    busy_invalid = valid;
    busy_invalid["pivot"] = {1, 2, 3};
    require_error(fixture.request(http::verb::post, target, busy_invalid.dump(), "application/json"),
        http::status::bad_request, "invalid_request");
    CHECK(read_objects(fixture, route) == before);
    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
}

TEST_CASE("Multipart parser accepts fragmented STL body", "[rest_models]")
{
    TempDirectory directory;
    const std::string file = make_stl();
    const std::string body = make_multipart_body("triangle.stl", file);
    Slic3r::Rest::MultipartUploadParser parser(
        "multipart/form-data; boundary=" + test_boundary,
        directory.path(),
        1024 * 1024,
        16 * 1024,
        model_extensions);
    for (std::size_t offset = 0; offset < body.size(); offset += 7)
        parser.consume(body.data() + offset, std::min<std::size_t>(7, body.size() - offset));
    const std::shared_ptr<const Slic3r::Rest::HttpUpload> upload = parser.finish();
    REQUIRE(upload->byte_count == file.size());
    REQUIRE(read_file(upload->temporary_path) == file);
}

TEST_CASE("Multipart parser ignores the epilogue whatever the read split", "[rest_models]")
{
    TempDirectory directory;
    const std::string file = make_obj_with_padding();
    const std::string body = make_multipart_body("part.obj", file);
    REQUIRE(body.substr(body.size() - 4) == "--\r\n");
    Slic3r::Rest::MultipartUploadParser parser(
        "multipart/form-data; boundary=" + test_boundary,
        directory.path(),
        1024 * 1024,
        16 * 1024,
        model_extensions);
    // The closing "--" ends one read; its CRLF and an epilogue arrive later.
    parser.consume(body.data(), body.size() - 2);
    parser.consume("\r\n", 2);
    const std::string epilogue = "ignored epilogue\r\n";
    parser.consume(epilogue.data(), epilogue.size());
    const std::shared_ptr<const Slic3r::Rest::HttpUpload> upload = parser.finish();
    REQUIRE(upload->byte_count == file.size());
    REQUIRE(read_file(upload->temporary_path) == file);
}

TEST_CASE("Multipart parser stores uploads under random exclusive names", "[rest_models]")
{
    TempDirectory directory;
    const std::string body = make_multipart_body("part.obj", make_obj_with_padding());
    std::set<std::filesystem::path> paths;
    std::vector<std::shared_ptr<const Slic3r::Rest::HttpUpload>> uploads;
    for (int index = 0; index < 2; ++index) {
        Slic3r::Rest::MultipartUploadParser parser(
            "multipart/form-data; boundary=" + test_boundary,
            directory.path(),
            1024 * 1024,
            16 * 1024,
        model_extensions);
        parser.consume(body.data(), body.size());
        uploads.push_back(parser.finish());
        const std::filesystem::path path = uploads.back()->temporary_path;
        REQUIRE(path.parent_path() == directory.path());
        REQUIRE(path.filename().string().rfind("rest-upload-", 0) == 0);
        REQUIRE(path.extension() == ".obj");
        // The client file name never reaches the server path.
        REQUIRE(path.filename().string().find("part") == std::string::npos);
        paths.insert(path);
    }
    REQUIRE(paths.size() == 2);

    // Releasing the last owner deletes the temporary file.
    const std::filesystem::path first = uploads.front()->temporary_path;
    uploads.clear();
    REQUIRE_FALSE(std::filesystem::exists(first));
}

TEST_CASE("REST server removes stale uploads when it starts", "[rest_models]")
{
    RestServerFixture fixture;
    fixture.stop();
    const std::filesystem::path uploads = Slic3r::Rest::upload_directory(fixture.work_directory());
    std::filesystem::create_directories(uploads);
    std::ofstream(uploads / "rest-upload-stale.stl") << "left by a crash";
    std::ofstream(uploads / "unrelated.txt") << "not an upload";

    fixture.start();
    REQUIRE_FALSE(std::filesystem::exists(uploads / "rest-upload-stale.stl"));
    REQUIRE(std::filesystem::exists(uploads / "unrelated.txt"));
}

TEST_CASE("REST add_model imports STL OBJ and 3MF", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    UploadOptions fragmented;
    fragmented.fragment_size = 7;
    const std::vector<std::string> stl = object_ids(
        upload_model(fixture, target, "triangle.stl", make_stl(), fragmented));
    REQUIRE(stl.size() == 1);

    UploadOptions chunked;
    chunked.chunked = true;
    const std::vector<std::string> obj = object_ids(
        upload_model(fixture, target, "part.obj", make_obj_with_padding(), chunked));
    REQUIRE(obj.size() == 1);

    chunked.fragment_size = 11;
    const std::vector<std::string> three_mf = object_ids(
        upload_model(fixture, target, "project.3MF", make_3mf(2), chunked));
    REQUIRE(three_mf.size() == 2);

    // Every returned object ID is distinct, across objects and uploads.
    std::set<std::string> all_ids;
    for (const std::vector<std::string> *ids : {&stl, &obj, &three_mf})
        for (const std::string &id : *ids) {
            REQUIRE(!id.empty());
            REQUIRE(all_ids.insert(id).second);
        }

    // An epilogue after the closing boundary is ignored.
    const HttpResponseMessage epilogue = send_multipart(
        fixture,
        target,
        make_multipart_body("part.obj", make_obj_with_padding()) + "epilogue\r\n");
    REQUIRE(object_ids(epilogue).size() == 1);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model names entities after the client file", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string target = "/session/" + session_id + "/add_model";

    // A 3MF whose object and volume carry no name: the loader falls back to
    // "<file stem>_N", which must use the client file name.
    TempDirectory directory;
    const std::filesystem::path unnamed_path = directory.path() / "unnamed.3mf";
    Slic3r::Model unnamed;
    Slic3r::ModelObject *object = unnamed.add_object("", "", Slic3r::make_cube(5, 5, 5));
    object->volumes.front()->name.clear();
    object->add_instance();
    REQUIRE(Slic3r::store_3mf(unnamed_path.string().c_str(), &unnamed, nullptr));

    REQUIRE(object_ids(upload_model(fixture, target, "triangle.stl", make_stl())).size() == 1);
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", make_obj_with_padding())).size() == 1);
    REQUIRE(object_ids(upload_model(fixture, target, "bracket.3mf", read_file(unnamed_path))).size() == 1);

    const std::shared_ptr<Slic3r::Model> scene = find_session(fixture, session_id)->data_snapshot()->scene;
    std::vector<std::string> object_names;
    std::vector<std::string> volume_names;
    for (const Slic3r::ModelObject *model_object : scene->object_ptrs()) {
        object_names.push_back(model_object->name);
        CHECK(model_object->input_file.find("rest-upload-") == std::string::npos);
        for (const Slic3r::ModelVolume *volume : model_object->volumes) {
            volume_names.push_back(volume->name);
            CHECK(volume->source.input_file.find("rest-upload-") == std::string::npos);
        }
    }
    CHECK(object_names == std::vector<std::string>{"triangle.stl", "part.obj", "bracket_1"});
    CHECK(volume_names == std::vector<std::string>{"triangle.stl", "part.obj", "bracket_1"});
}

TEST_CASE("REST add_model ignores the project configuration of a 3MF", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = "/session/" + create_session(fixture);

    Slic3r::DynamicPrintConfig project;
    project.set_deserialize_strict("layer_height", "0.5");
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "project.3mf", make_3mf(1, &project))).size() == 1);

    const HttpResponseMessage layer_height = fixture.request(http::verb::get, route + "/get_setting/layer_height");
    REQUIRE(layer_height.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(layer_height.body()).at("serialized_value") == "0.2");
}

TEST_CASE("REST add_model rejects oversized and invalid uploads transactionally", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.max_upload_bytes = 32;
    RestServerFixture fixture(limits, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    UploadOptions chunked;
    chunked.chunked = true;
    chunked.fragment_size = 5;
    const HttpResponseMessage oversized = upload_model(
        fixture, target, "too-large.obj", make_obj_with_padding(128), chunked);
    require_error(oversized, http::status::payload_too_large, "request_too_large");
    REQUIRE(fixture.work_file_count() == 0);

    // The client keeps sending a large body after the early 413: the server
    // drains it, so the response is still readable instead of a reset.
    UploadOptions large_fragments;
    large_fragments.fragment_size = 64 * 1024;
    const HttpResponseMessage drained = upload_model(
        fixture, target, "large.obj", make_obj_with_padding(512 * 1024), large_fragments);
    require_error(drained, http::status::payload_too_large, "request_too_large");
    REQUIRE(fixture.work_file_count() == 0);

    UploadOptions small_fragments;
    small_fragments.fragment_size = 3;
    const HttpResponseMessage invalid = upload_model(fixture, target, "invalid.stl", "not a model", small_fragments);
    require_error(invalid, http::status::unprocessable_entity, "invalid_model");
    // The server-side temporary path never appears in a client message.
    REQUIRE(invalid.body().find("rest-upload-") == std::string::npos);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects truncated uploads", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string file = make_obj_with_padding(256);

    // The connection ends in the middle of the file data.
    UploadOptions interrupted;
    interrupted.body_bytes = make_multipart_body("part.obj", file).size() / 2;
    require_error(
        upload_model(fixture, target, "part.obj", file, interrupted),
        http::status::bad_request,
        "invalid_request",
        "truncated");
    REQUIRE(fixture.work_file_count() == 0);

    // The HTTP body is complete but the multipart closing boundary is missing.
    const std::string unterminated = make_part("form-data; name=\"file\"; filename=\"part.obj\"", file);
    require_error(
        send_multipart(fixture, target, unterminated),
        http::status::bad_request,
        "invalid_multipart",
        "no closing boundary");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects malformed multipart bodies", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string file = make_obj_with_padding();
    const std::string closing = "--" + test_boundary + "--\r\n";

    require_error(
        send_multipart(fixture, target, make_part("form-data; name=\"document\"; filename=\"part.obj\"", file) + closing),
        http::status::bad_request,
        "invalid_multipart",
        "must be named file");
    require_error(
        send_multipart(fixture, target, make_part("form-data; name=\"file\"", file) + closing),
        http::status::bad_request,
        "invalid_multipart",
        "file name is missing");
    const std::string file_part = make_part("form-data; name=\"file\"; filename=\"part.obj\"", file);
    require_error(
        send_multipart(fixture, target, file_part + file_part + closing),
        http::status::bad_request,
        "invalid_multipart",
        "Only one multipart file part");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model rejects unsupported multipart filenames", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    const std::string body = make_multipart_body("../../model.txt", "not accepted");
    UploadOptions headers_only;
    headers_only.fragment_size = 2;
    headers_only.body_bytes = body.find("\r\n\r\n") + 4;
    const HttpResponseMessage response = upload_model(
        fixture, target, "../../model.txt", "not accepted", headers_only);
    require_error(response, http::status::unprocessable_entity, "unsupported_model_format");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model authorizes before storing any byte", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string file = make_obj_with_padding();

    UploadOptions anonymous;
    anonymous.token = std::string();
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, anonymous),
        http::status::unauthorized,
        "unauthorized");

    UploadOptions invalid_token;
    invalid_token.token = std::string("not-a-token");
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, invalid_token),
        http::status::unauthorized,
        "unauthorized");

    UploadOptions other_owner;
    other_owner.token = fixture.login_token("Bob", "secret");
    require_error(
        upload_model(fixture, "/session/" + session_id + "/add_model", "part.obj", file, other_owner),
        http::status::not_found,
        "session_not_found");
    require_error(
        upload_model(fixture, "/session/session_missing/add_model", "part.obj", file),
        http::status::not_found,
        "session_not_found");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST add_model respects the slicing state and invalidates results", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string target = "/session/" + session_id + "/add_model";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);

    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    require_error(
        upload_model(fixture, target, "part.obj", make_obj_with_padding()),
        http::status::conflict,
        "session_busy");
    REQUIRE(fixture.work_file_count() == 0);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().empty());

    REQUIRE(session->finish_work(
        work.value(),
        "result.gcode",
        Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", make_obj_with_padding())).size() == 1);
    const Slic3r::Rest::SessionContext::Snapshot after = session->snapshot();
    REQUIRE(after.status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE_FALSE(after.current_gcode);
    REQUIRE(session->viewer_snapshot()->outdated);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 1);
}

TEST_CASE("REST add_model parses away from the network thread", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    // The observer holds the import until the test has been served another
    // request, which would deadlock if both ran on the network thread. The
    // bounded wait keeps a failed assertion from blocking the server shutdown.
    const std::shared_ptr<std::promise<void>> import_started = std::make_shared<std::promise<void>>();
    std::promise<void> release_import;
    std::shared_future<void> released = release_import.get_future().share();
    fixture.stop();
    fixture.set_import_observer([import_started, released]() {
        import_started->set_value();
        released.wait_for(std::chrono::seconds(10));
    });
    fixture.start();

    const std::string target = "/session/" + create_session(fixture) + "/add_model";
    std::future<HttpResponseMessage> upload = std::async(std::launch::async, [&fixture, &target]() {
        return upload_model(fixture, target, "part.obj", make_obj_with_padding());
    });
    std::future<void> started = import_started->get_future();
    REQUIRE(started.wait_for(std::chrono::seconds(10)) == std::future_status::ready);

    const HttpResponseMessage concurrent = fixture.request(http::verb::post, "/create_session");
    REQUIRE(concurrent.result() == http::status::created);

    release_import.set_value();
    REQUIRE(upload.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    REQUIRE(object_ids(upload.get()).size() == 1);
}

TEST_CASE("REST add_model uses an idle timeout for slow uploads", "[rest_models]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.timeout = std::chrono::seconds(1);
    RestServerFixture fixture(limits, true, true);
    const std::string target = "/session/" + create_session(fixture) + "/add_model";

    // The whole upload lasts longer than the timeout, but no gap exceeds it.
    UploadOptions slow;
    slow.fragment_size = 48;
    slow.fragment_delay = std::chrono::milliseconds(300);
    const std::string file = make_obj_with_padding(256);
    REQUIRE(make_multipart_body("part.obj", file).size() / slow.fragment_size >= 6);
    REQUIRE(object_ids(upload_model(fixture, target, "part.obj", file, slow)).size() == 1);
}

TEST_CASE("REST list_objects exposes stable IDs across uploads and project replacement", "[rest_objects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const HttpResponseMessage empty = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(empty.result() == http::status::ok);
    CHECK(nlohmann::json::parse(empty.body()) == nlohmann::json{{"objects", nlohmann::json::array()}});

    const std::vector<std::string> stl_ids = object_ids(upload_model(fixture, route + "/add_model", "triangle.stl", make_stl()));
    const HttpResponseMessage first = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(first.result() == http::status::ok);
    const nlohmann::json first_listing = nlohmann::json::parse(first.body());
    REQUIRE(first_listing.at("objects").size() == 1);
    CHECK(first_listing.at("objects").at(0).at("id") == stl_ids.front());
    CHECK(first_listing.at("objects").at(0).at("name") == "triangle.stl");
    REQUIRE(first_listing.at("objects").at(0).at("parts").size() == 1);
    REQUIRE(first_listing.at("objects").at(0).at("instances").size() == 1);
    CHECK(first_listing.at("objects").at(0).size() == 5);
    CHECK(first_listing.at("objects").at(0).at("parts").at(0).size() == 3);
    CHECK(first_listing.at("objects").at(0).at("instances").at(0).size() == 2);
    const std::uint64_t revision = session->snapshot().revision;
    CHECK(fixture.request(http::verb::get, route + "/list_objects").body() == first.body());
    CHECK(session->snapshot().revision == revision);

    const std::vector<std::string> obj_ids = object_ids(upload_model(fixture, route + "/add_model", "part.obj", make_obj_with_padding()));
    const HttpResponseMessage added = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(added.result() == http::status::ok);
    const nlohmann::json added_listing = nlohmann::json::parse(added.body());
    REQUIRE(added_listing.at("objects").size() == 2);
    CHECK(added_listing.at("objects").at(0) == first_listing.at("objects").at(0));
    CHECK(added_listing.at("objects").at(1).at("id") == obj_ids.front());

    const std::vector<std::string> project_ids = object_ids(upload_model(fixture, route + "/load_project", "project.3mf", make_3mf(2)));
    const HttpResponseMessage replaced = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(replaced.result() == http::status::ok);
    const nlohmann::json replacement = nlohmann::json::parse(replaced.body());
    REQUIRE(replacement.at("objects").size() == project_ids.size());
    std::set<std::string> old_ids;
    for (const nlohmann::json &object : added_listing.at("objects")) {
        old_ids.insert(object.at("id").get<std::string>());
        for (const char *children : {"parts", "instances"})
            for (const nlohmann::json &child : object.at(children))
                old_ids.insert(child.at("id").get<std::string>());
    }
    std::set<std::string> new_ids;
    for (std::size_t index = 0; index < project_ids.size(); ++index) {
        const nlohmann::json &object = replacement.at("objects").at(index);
        CHECK(object.at("id") == project_ids[index]);
        REQUIRE(new_ids.insert(project_ids[index]).second);
        for (const char *children : {"parts", "instances"})
            for (const nlohmann::json &child : object.at(children))
                REQUIRE(new_ids.insert(child.at("id").get<std::string>()).second);
    }
    for (const std::string &id : old_ids) {
        CHECK(new_ids.count(id) == 0);
        CHECK_FALSE(session->resolve_entity(id).has_value());
    }
    for (const std::string &id : new_ids)
        CHECK(session->resolve_entity(id).has_value());

    const Slic3r::Rest::SessionData before = session->data_snapshot().value();
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    const std::uint64_t busy_revision = session->snapshot().revision;
    const HttpResponseMessage busy = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(busy.result() == http::status::ok);
    CHECK(nlohmann::json::parse(busy.body()) == replacement);
    CHECK(session->snapshot().revision == busy_revision);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
    REQUIRE(session->cancel_work(work, Slic3r::Rest::SessionRegistry::Clock::now()));
    CHECK(session->data_snapshot()->configuration.diff(before.configuration).empty());
    CHECK(session->data_snapshot()->preferences == before.preferences);
    CHECK(session->data_snapshot()->scene == before.scene);

    CHECK(fixture.request_without_auth(http::verb::get, route + "/list_objects").result() == http::status::unauthorized);
    const std::string bob_token = fixture.login_token("Bob", "secret");
    CHECK(fixture.request(http::verb::get, route + "/list_objects", {}, {},
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::get, "/session/missing/list_objects").result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, route + "/list_objects").result() == http::status::not_found);
    CHECK(fixture.request(http::verb::get, route + "/list_objects", "{}", "application/json").result() == http::status::bad_request);
    CHECK(fixture.request(http::verb::get, route + "/list_objects/extra").result() == http::status::not_found);
    CHECK(fixture.work_file_count() == 0);

    // An incomplete internal mapping must produce an error, not create new IDs during a read.
    Slic3r::Rest::SessionData inconsistent = session->data_snapshot().value();
    inconsistent.scene = std::make_shared<Slic3r::Model>();
    inconsistent.scene->add_object("unmapped", "unmapped.stl", Slic3r::make_cube(1, 1, 1))->add_instance();
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), inconsistent, true,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::uint64_t invalid_revision = session->snapshot().revision;
    const HttpResponseMessage invalid = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(invalid.result() == http::status::internal_server_error);
    CHECK(nlohmann::json::parse(invalid.body()).at("error").at("code") == "internal_error");
    CHECK(session->snapshot().revision == invalid_revision);
}

// A client places a mesh point with the matrices of the listing alone:
// p_platter = M_instance × M_volume × p_mesh, matrices read row by row.
TEST_CASE("REST list_objects matrices recompose engine placements", "[rest_objects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;

    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "placed.3mf";
    Slic3r::Model model;
    Slic3r::ModelObject *object = model.add_object("placed", "placed.stl", Slic3r::make_cube(10, 10, 10));
    Slic3r::ModelVolume *modifier = object->add_volume(
        Slic3r::make_cube(2, 2, 2), Slic3r::ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Slic3r::Vec3d(1, 2, 3));
    Slic3r::Transform3d rotation = Slic3r::Transform3d::Identity();
    rotation.matrix()(0, 0) = 0;
    rotation.matrix()(0, 1) = -1;
    rotation.matrix()(1, 0) = 1;
    rotation.matrix()(1, 1) = 0;
    rotation.translation() = Slic3r::Vec3d(40, 50, 0);
    object->add_instance(Slic3r::Geometry::Transformation(rotation));
    REQUIRE(Slic3r::store_3mf(path.string().c_str(), &model, nullptr));
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "placed.3mf", read_file(path))).size() == 1);

    const HttpResponseMessage listed = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(listed.result() == http::status::ok);
    const nlohmann::json listed_object = nlohmann::json::parse(listed.body()).at("objects").at(0);
    const auto matrix_from_json = [](const nlohmann::json &values) {
        REQUIRE(values.size() == 16);
        Slic3r::Transform3d matrix;
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column)
                matrix.matrix()(row, column) = values.at(row * 4 + column).get<double>();
        return matrix;
    };
    const std::shared_ptr<Slic3r::Model> scene = find_session(fixture, session_id)->data_snapshot()->scene;
    const Slic3r::ModelObject *loaded = scene->object_ptrs().front();
    REQUIRE(listed_object.at("parts").size() == loaded->volumes.size());
    REQUIRE(listed_object.at("instances").size() == 1);
    const Slic3r::Transform3d instance = matrix_from_json(listed_object.at("instances").at(0).at("matrix"));
    const Slic3r::Vec3d mesh_point(4, 5, 6);
    for (std::size_t index = 0; index < loaded->volumes.size(); ++index) {
        INFO("volume " << index);
        const nlohmann::json &part = listed_object.at("parts").at(index);
        CHECK(part.at("name").get<std::string>().find("rest-upload-") == std::string::npos);
        const Slic3r::Vec3d from_listing = instance * matrix_from_json(part.at("matrix")) * mesh_point;
        const Slic3r::Vec3d from_engine =
            loaded->instances.front()->get_matrix() * loaded->volumes[index]->get_matrix() * mesh_point;
        CHECK(from_listing.isApprox(from_engine));
    }
    // The rotation must survive: the listed instance is not a pure translation.
    CHECK(instance.linear().isApprox(rotation.linear()));
}

TEST_CASE("REST load_project replaces a modified session", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string other_route = "/session/" + create_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);
    const std::map<std::string, std::string> profile_files = fixture.profile_files();
    const nlohmann::json defaults = read_settings(fixture, other_route, {"layer_height", "perimeters"});

    // Local edits, a scene object, preferences, and a finished result.
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/perimeters/7").result() == http::status::ok);
    const std::vector<std::string> old_ids = object_ids(
        upload_model(fixture, route + "/add_model", "part.obj", make_obj_with_padding()));
    {
        const std::optional<Slic3r::Rest::SessionContext::MutationToken> mutation = session->begin_mutation();
        Slic3r::Rest::SessionData data = session->data_snapshot().value();
        data.preferences = nlohmann::json{{"units", "mm"}};
        REQUIRE(session->commit_mutation(mutation.value(), std::move(data), false, Slic3r::Rest::SessionRegistry::Clock::now()));
    }
    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(session->finish_work(
        work.value(), "result.gcode", Slic3r::GCodeProcessorResult(), Slic3r::Rest::SessionRegistry::Clock::now()));

    // A partial project: only two known keys plus unknown keys of each origin.
    Slic3r::DynamicPrintConfig partial;
    partial.set_deserialize_strict("layer_height", "0.3");
    partial.set_deserialize_strict("post_process", "\"C:/witness/script.bat\"");
    const std::string project = add_project_config_lines(make_3mf(2, &partial),
        "; future_print_key = 42\n"
        "; printer_future_key = pf\n"
        "; retract_future_key = 1\n"
        "; filament_future_key = 7\n");
    const std::vector<std::string> new_ids = object_ids(
        upload_model(fixture, route + "/load_project", "project.3mf", project));
    REQUIRE(new_ids.size() == 2);

    // The scene and its IDs are replaced.
    for (const std::string &id : old_ids)
        REQUIRE_FALSE(session->resolve_entity(id).has_value());
    for (const std::string &id : new_ids) {
        REQUIRE(std::find(old_ids.begin(), old_ids.end(), id) == old_ids.end());
        REQUIRE(session->resolve_entity(id).has_value());
    }
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 2);

    // Present keys are applied, absent ones return to the system defaults,
    // and the local edits are gone. post_process is kept but never run.
    const nlohmann::json values = read_settings(fixture, route,
        {"layer_height", "perimeters", "post_process", "print_custom_setting", "filament_custom_setting"});
    REQUIRE(values.at("layer_height").at("value") == 0.3);
    REQUIRE(values.at("perimeters") == defaults.at("perimeters"));
    REQUIRE(values.at("post_process").at("value") == nlohmann::json::array({"C:/witness/script.bat"}));
    REQUIRE(nlohmann::json::parse(values.at("print_custom_setting").at("value").get<std::string>()) == nlohmann::json{
        {"print", {{"future_print_key", "42"}}},
        {"printer", {{"printer_future_key", "pf"}, {"retract_future_key", "1"}}}});
    const nlohmann::json filament_custom = values.at("filament_custom_setting").at("value");
    REQUIRE(filament_custom.size() == 1);
    REQUIRE(nlohmann::json::parse(filament_custom.at(0).get<std::string>()) ==
        nlohmann::json{{"filament_future_key", "7"}});

    // Session-only data and lifecycle: preferences kept, embedded profiles are
    // not catalog profiles, the result is outdated.
    const Slic3r::Rest::SessionData data = session->data_snapshot().value();
    REQUIRE(data.preferences == nlohmann::json{{"units", "mm"}});
    REQUIRE(data.profiles.print.empty());
    REQUIRE(data.profiles.printer.empty());
    const Slic3r::Rest::SessionContext::Snapshot snapshot = session->snapshot();
    REQUIRE(snapshot.status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE_FALSE(snapshot.current_gcode);
    REQUIRE(session->viewer_snapshot()->outdated);

    // Nothing leaks into another session or into the installed profiles.
    REQUIRE(read_settings(fixture, other_route, {"layer_height", "perimeters"}) == defaults);
    REQUIRE(fixture.profile_files() == profile_files);
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST load_project keeps separate profile metadata without trusting section settings", "[rest_projects][rest_profile_sections]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const std::string other_id = create_session(fixture);
    const Slic3r::Rest::SessionData other_before = find_session(fixture, other_id)->data_snapshot().value();
    const std::map<std::string, std::string> files_before = fixture.profile_files();
    Slic3r::DynamicPrintConfig configuration(session->data_snapshot()->configuration);
    configuration.set_deserialize_strict("print_settings_id", "Embedded print");
    configuration.set_deserialize_strict("printer_settings_id", "Embedded printer");
    configuration.set_deserialize_strict("filament_settings_id", "Embedded A;Embedded B");
    configuration.set_deserialize_strict("nozzle_diameter", "0.4,0.4");
    configuration.set_deserialize_strict("extruder_offset", "0x0,0x0");
    const std::string metadata = R"ini([print:Embedded print]
inherits = Print parent
compatible_printers_condition = nozzle_diameter[0]==0.4
post_process = section-script-witness

[printer:Embedded printer]
inherits = Printer parent
default_print_profile = Embedded print
default_filament_profile = Embedded A;Embedded B
printhost_apikey = section-local-witness

[filament:Embedded A]
inherits = Filament parent A
compatible_printers_condition = nozzle_diameter[0]==0.4

[filament:Embedded B]
inherits = Filament parent B
compatible_prints_condition = layer_height>0
)ini";
    TempDirectory directory;
    const std::filesystem::path path = directory.path() / "metadata.3mf";
    Slic3r::Model scene;
    scene.add_object("cube", "cube.stl", Slic3r::make_cube(20, 20, 20))->add_instance();
    REQUIRE(Slic3r::store_3mf(path.u8string().c_str(), &scene, &configuration,
        Slic3r::OptionStore3mf{}.set_profile_sections(metadata)));
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "metadata.3mf", read_file(path))).size() == 1);
    const Slic3r::Rest::SessionData loaded = session->data_snapshot().value();
    boost::property_tree::ptree sections;
    std::istringstream input(loaded.profile_sections);
    boost::property_tree::read_ini(input, sections);
    REQUIRE(sections.size() == 4);
    const boost::property_tree::ptree::const_assoc_iterator print = sections.find(
        "print:" + loaded.configuration.opt_string("print_settings_id"));
    const boost::property_tree::ptree::const_assoc_iterator printer = sections.find(
        "printer:" + loaded.configuration.opt_string("printer_settings_id"));
    REQUIRE(print != sections.not_found());
    REQUIRE(printer != sections.not_found());
    CHECK(print->second.get<std::string>("inherits") == "Print parent");
    CHECK(print->second.get<std::string>("compatible_printers_condition") == "nozzle_diameter[0]==0.4");
    CHECK(printer->second.get<std::string>("inherits") == "Printer parent");
    CHECK(printer->second.get<std::string>("default_print_profile") == "Embedded print");
    const std::vector<std::string> &filaments = loaded.configuration.option<Slic3r::ConfigOptionStrings>(
        "filament_settings_id")->get_values();
    REQUIRE(filaments.size() == 2);
    for (std::size_t slot = 0; slot < filaments.size(); ++slot) {
        CAPTURE(slot);
        const boost::property_tree::ptree::const_assoc_iterator filament = sections.find("filament:" + filaments[slot]);
        REQUIRE(filament != sections.not_found());
        CHECK(filament->second.get<std::string>("inherits") == (slot == 0 ? "Filament parent A" : "Filament parent B"));
        CHECK(filament->second.get<std::string>(slot == 0 ? "compatible_printers_condition" : "compatible_prints_condition") ==
            (slot == 0 ? "nozzle_diameter[0]==0.4" : "layer_height>0"));
    }
    CHECK(loaded.profile_sections.find("section-script-witness") == std::string::npos);
    CHECK(loaded.profile_sections.find("section-local-witness") == std::string::npos);
    CHECK(loaded.configuration.option<Slic3r::ConfigOptionStrings>("post_process")->empty());
    CHECK_FALSE(loaded.configuration.has("printhost_apikey"));
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    CHECK(session->data_snapshot()->profile_sections == loaded.profile_sections);
    CHECK(find_session(fixture, other_id)->data_snapshot()->profile_sections == other_before.profile_sections);
    CHECK(fixture.profile_files() == files_before);
}

TEST_CASE("REST load_project keeps installed profile IDs and accepts geometry only", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    const Slic3r::Rest::SessionData initial = session->data_snapshot().value();
    REQUIRE_FALSE(initial.profiles.print.empty());

    // A project naming the unchanged installed profiles selects them again.
    // This minimal flat project only names installed profiles. Metadata that
    // the project does not provide stays as defined in the catalog.
    Slic3r::DynamicPrintConfig named;
    for (const char *key : {"print_settings_id", "printer_settings_id", "filament_settings_id", "default_filament_profile"})
        named.set_key_value(key, initial.configuration.option(key)->clone());
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "project.3mf", make_3mf(1, &named))).size() == 1);
    const Slic3r::Rest::ProfileSelection selection = session->data_snapshot()->profiles;
    REQUIRE(selection.print == initial.profiles.print);
    REQUIRE(selection.printer == initial.profiles.printer);
    REQUIRE(selection.filaments == initial.profiles.filaments);

    // The same installed profiles with one modified value are no longer the
    // catalog profiles: the modified category loses its ID, the others keep it.
    Slic3r::DynamicPrintConfig modified_print = named;
    modified_print.set_deserialize_strict("layer_height", "0.3");
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "print.3mf", make_3mf(1, &modified_print))).size() == 1);
    const Slic3r::Rest::ProfileSelection print_changed = session->data_snapshot()->profiles;
    REQUIRE(print_changed.print.empty());
    REQUIRE(print_changed.printer == initial.profiles.printer);
    REQUIRE(print_changed.filaments == initial.profiles.filaments);

    Slic3r::DynamicPrintConfig modified_filament = named;
    modified_filament.set_deserialize_strict("filament_diameter", "2.85");
    REQUIRE(object_ids(upload_model(
        fixture, route + "/load_project", "filament.3mf", make_3mf(1, &modified_filament))).size() == 1);
    const Slic3r::Rest::ProfileSelection filament_changed = session->data_snapshot()->profiles;
    REQUIRE(filament_changed.print == initial.profiles.print);
    REQUIRE(filament_changed.printer == initial.profiles.printer);
    REQUIRE(filament_changed.filaments == std::vector<std::string>{""});

    // A 3MF without configuration resets every setting to the defaults.
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "geometry.3mf", make_3mf(3))).size() == 3);
    REQUIRE(read_settings(fixture, route, {"layer_height"}).at("layer_height").at("value") == 0.2);

    // Unnamed project objects take the "<file stem>_N" fallback of the client
    // file, never the server temporary file name.
    TempDirectory directory;
    const std::filesystem::path unnamed_path = directory.path() / "unnamed.3mf";
    Slic3r::Model unnamed;
    Slic3r::ModelObject *object = unnamed.add_object("", "", Slic3r::make_cube(5, 5, 5));
    object->volumes.front()->name.clear();
    object->add_instance();
    REQUIRE(Slic3r::store_3mf(unnamed_path.string().c_str(), &unnamed, nullptr));
    REQUIRE(object_ids(upload_model(fixture, route + "/load_project", "bracket.3mf", read_file(unnamed_path))).size() == 1);
    const std::shared_ptr<Slic3r::Model> loaded_scene = session->data_snapshot()->scene;
    const Slic3r::ModelObject *loaded = loaded_scene->object_ptrs().front();
    CHECK(loaded->name == "bracket_1");
    CHECK(loaded->volumes.front()->name == "bracket_1");
    CHECK(loaded->input_file.find("rest-upload-") == std::string::npos);
}

TEST_CASE("REST load_project refusals leave the session intact", "[rest_projects]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string target = route + "/load_project";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    const std::vector<std::string> ids = object_ids(
        upload_model(fixture, route + "/add_model", "part.obj", make_obj_with_padding()));
    const std::uint64_t revision = session->snapshot().revision;
    const std::string profile_sections = session->data_snapshot()->profile_sections;

    Slic3r::DynamicPrintConfig sla;
    sla.set_deserialize_strict("printer_technology", "SLA");
    Slic3r::DynamicPrintConfig incompatible;
    incompatible.set_deserialize_strict("use_firmware_retraction", "1");
    incompatible.set_deserialize_strict("wipe", "1");
    Slic3r::DynamicPrintConfig single_extruder;
    single_extruder.set_deserialize_strict("layer_height", "0.3");
    Slic3r::DynamicPrintConfig malformed_custom;
    malformed_custom.set_deserialize_strict("print_custom_setting", "not json");
    // The recomposition clamps global extruder options, so they must be
    // refused as loaded.
    Slic3r::DynamicPrintConfig global_extruder;
    global_extruder.set_deserialize_strict("perimeter_extruder", "3");
    // An empty nozzle list cannot be written by the engine; the raw line is
    // appended to the project configuration, where the last value wins.
    const std::string no_extruder = add_project_config_lines(
        make_3mf(1, &single_extruder), "; nozzle_diameter = \n");

    struct Refusal {
        std::string filename;
        std::string file;
        std::string code;
        std::string message;
    };
    const std::vector<Refusal> refusals = {
        {"sla.3mf", make_3mf(1, &sla), "invalid_project", "not an FFF project"},
        {"incompatible.3mf", make_3mf(1, &incompatible), "invalid_project", "not compatible"},
        {"extruder.3mf", make_3mf(1, &single_extruder, 3), "invalid_project", "uses extruder 3"},
        {"global_extruder.3mf", make_3mf(1, &global_extruder), "invalid_project",
            "uses extruder 3 in perimeter_extruder"},
        {"no_extruder.3mf", no_extruder, "invalid_project", "defines no extruder"},
        {"custom.3mf", make_3mf(1, &malformed_custom), "invalid_project", "custom setting"},
        {"corrupt.3mf", "not a zip archive", "invalid_project", ""},
        {"part.obj", make_obj_with_padding(), "unsupported_model_format", ".3mf"}
    };
    for (const Refusal &refusal : refusals) {
        INFO(refusal.filename);
        require_error(
            upload_model(fixture, target, refusal.filename, refusal.file),
            http::status::unprocessable_entity,
            refusal.code,
            refusal.message);
    }

    REQUIRE(session->snapshot().revision == revision);
    REQUIRE(session->data_snapshot()->profile_sections == profile_sections);
    REQUIRE(read_settings(fixture, route, {"layer_height"}).at("layer_height").at("value") == 0.25);
    REQUIRE(session->data_snapshot()->scene->object_ptrs().size() == 1);
    REQUIRE(session->resolve_entity(ids.front()).has_value());
    REQUIRE(fixture.work_file_count() == 0);

    // A running slicing job refuses the upload before it is stored.
    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    require_error(
        upload_model(fixture, target, "project.3mf", make_3mf()),
        http::status::conflict,
        "session_busy");
    REQUIRE(fixture.work_file_count() == 0);
}

TEST_CASE("REST clear_platter empties the scene and keeps the session settings", "[rest_platter]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);

    // Two objects, one with several instances, local settings and preferences.
    const std::string first_id = object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl())).front();
    object_ids(upload_model(fixture, route + "/add_model", "other.obj", make_obj_with_padding()));
    REQUIRE(fixture.request(http::verb::post, route + "/set_instance_count/" + first_id,
        "{\"count\":3}", "application/json").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    {
        Slic3r::Rest::SessionData data = session->data_snapshot().value();
        data.preferences = nlohmann::json{{"units", "mm"}};
        REQUIRE(session->commit_mutation(session->begin_mutation().value(), std::move(data), false,
            Slic3r::Rest::SessionRegistry::Clock::now()));
    }
    const Slic3r::Rest::SessionData before = session->data_snapshot().value();
    const nlohmann::json listed = read_objects(fixture, route);
    std::vector<std::string> old_ids;
    for (const nlohmann::json &object : listed.at("objects")) {
        old_ids.push_back(object.at("id"));
        for (const char *children : {"parts", "instances"})
            for (const nlohmann::json &child : object.at(children))
                old_ids.push_back(child.at("id"));
    }
    REQUIRE(old_ids.size() == 2 + 2 + 4);
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::string result_id = session->viewer_snapshot()->result_id;

    const HttpResponseMessage cleared = fixture.request(http::verb::post, route + "/clear_platter");
    REQUIRE(cleared.result() == http::status::ok);
    CHECK(nlohmann::json::parse(cleared.body()) == nlohmann::json{{"cleared_object_count", 2}});
    CHECK(read_objects(fixture, route) == nlohmann::json{{"objects", nlohmann::json::array()}});
    for (const std::string &id : old_ids) {
        CAPTURE(id);
        CHECK_FALSE(session->resolve_entity(id).has_value());
        require_error(fixture.request(http::verb::post, route + "/transform/" + id,
            transform_body(Slic3r::Transform3d::Identity(), "platter").dump(), "application/json"),
            http::status::not_found, "entity_not_found");
    }
    require_error(fixture.request(http::verb::post, route + "/set_instance_count/" + first_id,
        "{\"count\":2}", "application/json"), http::status::not_found, "entity_not_found");

    // Settings, profiles and preferences survive; the result is outdated.
    const Slic3r::Rest::SessionData after = session->data_snapshot().value();
    CHECK(after.configuration.diff(before.configuration).empty());
    CHECK(after.preferences == before.preferences);
    CHECK(after.profiles.print == before.profiles.print);
    CHECK(after.profiles.printer == before.profiles.printer);
    CHECK(after.profiles.filaments == before.profiles.filaments);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK_FALSE(session->current_gcode().has_value());
    REQUIRE(session->viewer_snapshot() != nullptr);
    CHECK(session->viewer_snapshot()->result_id == result_id);
    CHECK(session->viewer_snapshot()->outdated);

    // An already empty platter is not a change: nothing is committed.
    const std::uint64_t revision = session->snapshot().revision;
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    const HttpResponseMessage empty = fixture.request(http::verb::post, route + "/clear_platter");
    REQUIRE(empty.result() == http::status::ok);
    CHECK(nlohmann::json::parse(empty.body()) == nlohmann::json{{"cleared_object_count", 0}});
    CHECK(session->snapshot().revision == revision);
    CHECK(session->viewer_snapshot() == viewer);

    // The cleared session accepts new models with new IDs.
    const std::vector<std::string> new_ids = object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl()));
    REQUIRE(new_ids.size() == 1);
    CHECK(std::find(old_ids.begin(), old_ids.end(), new_ids.front()) == old_ids.end());
}

TEST_CASE("REST clear_platter preserves an empty failed session and its error message", "[rest_platter]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(session != nullptr);
    REQUIRE(session->data_snapshot()->scene->objects().empty());
    const std::optional<Slic3r::Rest::SessionContext::WorkToken> work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now());
    REQUIRE(work.has_value());
    const std::string message = "No printable objects.";
    REQUIRE(session->fail_work(work.value(), message, Slic3r::Rest::SessionRegistry::Clock::now()));
    const Slic3r::Rest::SessionContext::Snapshot before = session->snapshot();
    REQUIRE(before.status == Slic3r::Rest::SessionStatus::failed);
    REQUIRE(before.error_message == message);

    const HttpResponseMessage response = fixture.request(http::verb::post, route + "/clear_platter");
    REQUIRE(response.result() == http::status::ok);
    CHECK(nlohmann::json::parse(response.body()) == nlohmann::json{{"cleared_object_count", 0}});
    const Slic3r::Rest::SessionContext::Snapshot after = session->snapshot();
    CHECK(after.status == Slic3r::Rest::SessionStatus::failed);
    CHECK(after.error_message == message);
    CHECK(after.revision == before.revision);
    CHECK(after.current_gcode == before.current_gcode);
    CHECK(after.has_viewer == before.has_viewer);
    CHECK(read_objects(fixture, route) == nlohmann::json{{"objects", nlohmann::json::array()}});
}

TEST_CASE("REST clear_platter refusals leave the scene intact", "[rest_platter]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string target = route + "/clear_platter";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl()));
    const nlohmann::json before = read_objects(fixture, route);
    const std::uint64_t revision = session->snapshot().revision;

    require_error(fixture.request(http::verb::post, target, "{}", "application/json"),
        http::status::bad_request, "invalid_request");
    CHECK(fixture.request(http::verb::get, target).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, target + "/extra").result() == http::status::not_found);
    CHECK(fixture.request_without_auth(http::verb::post, target).result() == http::status::unauthorized);
    const std::string bob_token = fixture.login_token("Bob", "secret");
    CHECK(fixture.request(http::verb::post, target, {}, {},
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, "/session/missing/clear_platter").result() == http::status::not_found);
    CHECK(session->snapshot().revision == revision);
    CHECK(read_objects(fixture, route) == before);

    // During slicing a valid request is refused; an invalid body is still a 400.
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    require_error(fixture.request(http::verb::post, target), http::status::conflict, "session_busy");
    require_error(fixture.request(http::verb::post, target, "{}", "application/json"),
        http::status::bad_request, "invalid_request");
    CHECK(read_objects(fixture, route) == before);
    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
}

TEST_CASE("REST session names follow the GUI project name", "[rest_session_name]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const auto read_name = [&fixture](const std::string &route) {
        const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_name");
        REQUIRE(response.result() == http::status::ok);
        return nlohmann::json::parse(response.body()).at("name");
    };
    const auto set_name = [&fixture](const std::string &route, const nlohmann::json &name) {
        return fixture.request(http::verb::post, route + "/set_name",
            nlohmann::json{{"name", name}}.dump(), "application/json");
    };

    // A name given at creation, kept as given (only file names are sanitized).
    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session",
        nlohmann::json{{"name", u8"Pi\u00e8ce: v2/final"}}.dump(), "application/json");
    REQUIRE(created.result() == http::status::created);
    const nlohmann::json created_body = nlohmann::json::parse(created.body());
    CHECK(created_body.at("name") == u8"Pi\u00e8ce: v2/final");
    const std::string session_id = created_body.at("session_id");
    const std::string route = "/session/" + session_id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    CHECK(read_name(route) == u8"Pi\u00e8ce: v2/final");

    // Scene and setting changes, and clearing the platter, keep the name.
    object_ids(upload_model(fixture, route + "/add_model", "part.stl", make_stl()));
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/layer_height/0.25").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/clear_platter").result() == http::status::ok);
    CHECK(read_name(route) == u8"Pi\u00e8ce: v2/final");

    // Loading a project names the session after the file, as in the GUI.
    object_ids(upload_model(fixture, route + "/load_project", "support.bracket.3mf", make_3mf()));
    CHECK(read_name(route) == "support.bracket");

    // Renaming never touches the result: status, G-code and viewer remain.
    const Slic3r::Rest::SessionContext::WorkToken work = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->finish_work(work, "result.gcode", Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now()));
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    const std::uint64_t revision = session->snapshot().revision;
    const HttpResponseMessage renamed = set_name(route, "Bracket");
    REQUIRE(renamed.result() == http::status::ok);
    CHECK(nlohmann::json::parse(renamed.body()) == nlohmann::json{{"name", "Bracket"}});
    CHECK(read_name(route) == "Bracket");
    CHECK(session->snapshot().revision == revision + 1);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
    CHECK(session->current_gcode().has_value());
    CHECK(session->viewer_snapshot() == viewer);

    // The same name commits nothing; null removes the name.
    REQUIRE(set_name(route, "Bracket").result() == http::status::ok);
    CHECK(session->snapshot().revision == revision + 1);
    const HttpResponseMessage cleared = set_name(route, nullptr);
    REQUIRE(cleared.result() == http::status::ok);
    CHECK(nlohmann::json::parse(cleared.body()).at("name").is_null());
    CHECK(read_name(route).is_null());
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
}

TEST_CASE("REST session name refusals leave the session untouched", "[rest_session_name]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string session_id = create_session(fixture);
    const std::string route = "/session/" + session_id;
    const std::string target = route + "/set_name";
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, session_id);
    REQUIRE(fixture.request(http::verb::post, target, "{\"name\":\"Kept\"}", "application/json").result() ==
        http::status::ok);
    const std::uint64_t revision = session->snapshot().revision;

    require_error(fixture.request(http::verb::post, target), http::status::bad_request, "invalid_request");
    const std::vector<std::string> invalid_bodies = {
        "{}",
        "[]",
        "{\"name\":\"\"}",
        "{\"name\":3}",
        "{\"name\":\"Kept\",\"other\":1}",
        "{\"name\":\"tab\\there\"}",
        nlohmann::json{{"name", std::string(256, 'a')}}.dump(),
    };
    for (const std::string &body : invalid_bodies) {
        CAPTURE(body);
        require_error(fixture.request(http::verb::post, target, body, "application/json"),
            http::status::bad_request, "invalid_request");
    }
    require_error(fixture.request(http::verb::post, target, "{", "application/json"),
        http::status::bad_request, "invalid_json");
    for (const std::string &body : {std::string("{\"name\":\"\"}"), std::string("{\"name\":\"a\\u0007\"}")}) {
        CAPTURE(body);
        require_error(fixture.request(http::verb::post, "/create_session", body, "application/json"),
            http::status::bad_request, "invalid_request");
    }
    CHECK(fixture.request(http::verb::get, target).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, route + "/get_name").result() == http::status::not_found);
    require_error(fixture.request(http::verb::get, route + "/get_name", "{}", "application/json"),
        http::status::bad_request, "invalid_request");
    CHECK(fixture.request(http::verb::post, target + "/extra", "{\"name\":\"x\"}", "application/json").result() ==
        http::status::not_found);
    CHECK(fixture.request_without_auth(http::verb::get, route + "/get_name").result() == http::status::unauthorized);
    const std::string bob_token = fixture.login_token("Bob", "secret");
    CHECK(fixture.request(http::verb::get, route + "/get_name", {}, {},
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, target, "{\"name\":\"x\"}", "application/json",
        {{"Authorization", "Bearer " + bob_token}}).result() == http::status::not_found);
    CHECK(fixture.request(http::verb::post, "/session/missing/set_name", "{\"name\":\"x\"}",
        "application/json").result() == http::status::not_found);
    CHECK(session->snapshot().revision == revision);

    // During slicing a valid rename is refused; an invalid body is still a 400.
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    require_error(fixture.request(http::verb::post, target, "{\"name\":\"Other\"}", "application/json"),
        http::status::conflict, "session_busy");
    require_error(fixture.request(http::verb::post, target, "{\"name\":\"\"}", "application/json"),
        http::status::bad_request, "invalid_request");
    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
    CHECK(session->data_snapshot()->name == std::optional<std::string>("Kept"));
}
