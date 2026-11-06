// Deployment fixtures are validated against production configuration and profile loaders.
#include <catch_main.hpp>
#include "rest_models_test_helpers.hpp"
#include "rest_vendor_test_helpers.hpp"
#include "profiles.hpp"

#include <cstdlib>

using namespace RestServerTest;

TEST_CASE("REST deployment examples load and unknown keys fail at every level", "[rest_deployment]")
{
    TempDirectory root;
    const Slic3r::Rest::ServerConfig valid = make_valid_config(root.path());
    Slic3r::Rest::CommandLineOverrides paths;
    paths.resource_directory = valid.resource_directory;
    paths.profiles_directory = valid.profiles_directory;
    paths.work_directory = valid.work_directory;
    paths.accounts_file = valid.accounts_file;
    const Slic3r::Rest::ServerConfig example = Slic3r::Rest::load_server_config(
        std::filesystem::path(SLIC3R_REST_SOURCE_DIRECTORY) / "doc/rest-server/examples/server.json", paths);
    // Keep the operator example's buffered-body and header budgets at defaults;
    // streamed uploads have their own independent file-size limit.
    const Slic3r::Rest::ServerConfig defaults = Slic3r::Rest::ServerConfig::defaults();
    CHECK(example.max_request_body_bytes == defaults.max_request_body_bytes);
    CHECK(example.request_timeout == defaults.request_timeout);
    CHECK(example.max_upload_bytes == 256ULL * 1024ULL * 1024ULL);
    const std::filesystem::path file = root.path() / "bad.json";
    for (const std::string &level : {"", "listen", "paths", "limits", "timeouts", "auth", "sessions"}) {
        nlohmann::json document = nlohmann::json::object();
        if (level.empty())
            document["typo"] = 1;
        else
            document[level] = {{"typo", 1}};
        std::ofstream(file) << document.dump();
        CHECK_THROWS_AS(Slic3r::Rest::load_server_config(file, paths), std::invalid_argument);
    }
}

TEST_CASE("REST deployment Basic vendor profiles slice through public HTTP routes", "[rest_deployment]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path(), false);
    RestServerFixture fixture({}, true, false, "work", {}, root.path());
    const std::string route = "/session/" + create_session(fixture);
    const nlohmann::json profiles = nlohmann::json::parse(fixture.request(http::verb::get, route + "/list_profiles").body());
    nlohmann::json selection{{"print", ""}, {"printer", ""}, {"extruder", nlohmann::json::array()}, {"filaments", nlohmann::json::array()}};
    for (const char *category : {"print", "printer", "filaments"}) {
        REQUIRE(profiles[category].size() == 1);
        CHECK(profiles[category][0]["vendor"] == "Custom");
        if (std::string(category) == "filaments")
            selection[category].push_back(profiles[category][0]["id"]);
        else
            selection[category] = profiles[category][0]["id"];
    }
    REQUIRE(fixture.request(http::verb::post, route + "/set_profile", selection.dump(), "application/json").result() == http::status::ok);
    REQUIRE(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl()).result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/slice").result() == http::status::accepted);
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
    std::string status;
    do {
        const HttpResponseMessage response = fixture.request(http::verb::get, route + "/state");
        REQUIRE(response.result() == http::status::ok);
        status = nlohmann::json::parse(response.body())["etat"];
        if (status == "sliced" || status == "failed")
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::steady_clock::now() < deadline);
    REQUIRE(status == "sliced");
    CHECK(fixture.request(http::verb::get, route + "/get_gcode").result() == http::status::ok);
}
