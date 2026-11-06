// Deployment fixtures are validated against production configuration and profile loaders.
#include <catch_main.hpp>
#include <boost/nowide/cstdlib.hpp>
#include "rest_models_test_helpers.hpp"
#include "rest_deployment_test_helpers.hpp"
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

TEST_CASE("REST deployment Basic profiles flatten with resolved inheritance", "[rest_deployment]")
{
    ensure_engine_initialized();
    TempDirectory root;
    const std::filesystem::path source = std::filesystem::path(SLIC3R_REST_SOURCE_DIRECTORY) / "tests/data/rest_profiles/Basic.ini";
    flatten_basic_profiles(source, root.path());
    const std::filesystem::path printer = root.path() / "printer/Basic 0.4 nozzle.ini";
    REQUIRE(std::filesystem::is_regular_file(printer));
    CHECK(read_file(printer).find("nozzle_diameter = 0.4") != std::string::npos);
    CHECK(read_file(printer).find("inherits =") == std::string::npos);
    CHECK(std::filesystem::is_regular_file(root.path() / "print/0.20mm NORMAL .4N.ini"));
    CHECK(std::filesystem::is_regular_file(root.path() / "filament/Basic PLA @BASIC.ini"));
    const std::string print = read_file(root.path() / "print/0.20mm NORMAL .4N.ini");
    CHECK(print.find("layer_height = 0.2") != std::string::npos);
    CHECK(print.find("perimeters = 3") != std::string::npos);
    CHECK(print.find("inherits =") == std::string::npos);
    std::size_t files = 0;
    for (const std::filesystem::directory_entry &entry : std::filesystem::recursive_directory_iterator(root.path()))
        if (entry.is_regular_file())
            ++files;
    CHECK(files == 3); // Abstract parents must not become selectable profiles.
    // Optional output supports the installed smoke workflow without a production converter.
    if (const char *output = boost::nowide::getenv("REST_TEST_FLAT_PROFILES"))
        flatten_basic_profiles(source, std::filesystem::u8path(output));
}
