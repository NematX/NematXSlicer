/*
 * Loopback tests for the REST transport, the login route and bearer middleware.
 *
 * The fixture injects private routes into the library and never adds those
 * routes to the standalone executable. Configuration, size limits, timeouts,
 * connection limits and a full stop/start cycle are exercised here.
 */

#include <catch_main.hpp>
#include <nlohmann/json.hpp>

#include "rest_server_fixture.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;

TEST_CASE("REST save_project checks method before engine initialization", "[rest_server][rest_save_project]")
{
    RestServerFixture fixture;
    const HttpResponseMessage method = fixture.request(http::verb::post, "/session/missing/save_project", "bad");
    CHECK(method.result() == http::status::not_found);
    const HttpResponseMessage engine = fixture.request(http::verb::get, "/session/missing/save_project?bad=x", "bad");
    CHECK(engine.result() == http::status::service_unavailable);
    CHECK(nlohmann::json::parse(engine.body()).at("error").at("code") == "engine_not_initialized");
}

TEST_CASE("REST server handles loopback JSON and route errors", "[rest_server]")
{
    RestServerFixture fixture;

    const HttpResponseMessage valid = fixture.request(http::verb::post, "/__test__/json", "{}", "application/json");
    REQUIRE(valid.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(valid.body()).at("ok").get<bool>());

    const std::uint32_t calls_before_invalid = fixture.handler_calls();
    const HttpResponseMessage invalid = fixture.request(
        http::verb::post,
        "/__test__/json",
        "{invalid",
        "application/json");
    REQUIRE(invalid.result() == http::status::bad_request);
    const nlohmann::json invalid_json = nlohmann::json::parse(invalid.body());
    REQUIRE(invalid_json.at("error").at("code") == "invalid_json");
    REQUIRE(!invalid_json.at("error").at("request_id").get<std::string>().empty());
    REQUIRE(fixture.handler_calls() == calls_before_invalid);

    const HttpResponseMessage empty = fixture.request(http::verb::get, "/__test__/empty");
    REQUIRE(empty.result() == http::status::no_content);
    REQUIRE(empty.body().empty());

    const HttpResponseMessage unknown = fixture.request(http::verb::get, "/not-a-route");
    REQUIRE(unknown.result() == http::status::not_found);
    REQUIRE(nlohmann::json::parse(unknown.body()).at("error").at("code") == "route_not_found");

}

TEST_CASE("REST server handles login success and authentication errors", "[rest_server]")
{
    RestServerFixture fixture({}, true);

    const HttpResponseMessage success = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"ALICE","password":"secret"})",
        "application/json; charset=utf-8");
    REQUIRE(success.result() == http::status::ok);
    const nlohmann::json success_body = nlohmann::json::parse(success.body());
    REQUIRE(success_body.at("access_token").is_string());
    REQUIRE(!success_body.at("access_token").get<std::string>().empty());
    REQUIRE(success_body.at("token_type") == "Bearer");
    REQUIRE(success_body.at("expires_in") == 24 * 60 * 60);
    REQUIRE(success.body().find("secret") == std::string::npos);
    REQUIRE(success[http::field::cache_control] == "no-store");
    REQUIRE(success[http::field::pragma] == "no-cache");

    const HttpResponseMessage invalid_password = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"alice","password":"wrong"})",
        "application/json");
    const HttpResponseMessage unknown_user = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"unknown","password":"wrong"})",
        "application/json");
    REQUIRE(invalid_password.result() == http::status::unauthorized);
    REQUIRE(unknown_user.result() == http::status::unauthorized);
    const nlohmann::json invalid_password_body = nlohmann::json::parse(invalid_password.body());
    const nlohmann::json unknown_user_body = nlohmann::json::parse(unknown_user.body());
    REQUIRE(invalid_password_body.at("error").at("code") == "invalid_credentials");
    REQUIRE(unknown_user_body.at("error").at("code") == "invalid_credentials");
    REQUIRE(invalid_password_body.at("error").at("message") ==
        unknown_user_body.at("error").at("message"));
    REQUIRE(invalid_password.body().find("wrong") == std::string::npos);
    REQUIRE(unknown_user.body().find("unknown") == std::string::npos);
}

TEST_CASE("REST server validates login JSON and limits attempts", "[rest_server]")
{
    RestServerFixture fixture({}, true);

    const HttpResponseMessage malformed_json = fixture.request(
        http::verb::post,
        "/login",
        "{invalid",
        "application/json");
    REQUIRE(malformed_json.result() == http::status::bad_request);
    REQUIRE(nlohmann::json::parse(malformed_json.body()).at("error").at("code") == "invalid_json");

    const HttpResponseMessage missing_password = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"alice"})",
        "application/json");
    REQUIRE(missing_password.result() == http::status::bad_request);
    REQUIRE(nlohmann::json::parse(missing_password.body()).at("error").at("code") == "invalid_request");

    for (int attempt = 0; attempt < 5; ++attempt) {
        const HttpResponseMessage response = fixture.request(
            http::verb::post,
            "/login",
            R"({"username":"limited","password":"wrong"})",
            "application/json");
        REQUIRE(response.result() == http::status::unauthorized);
    }
    const HttpResponseMessage rate_limited = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"limited","password":"wrong"})",
        "application/json");
    REQUIRE(rate_limited.result() == http::status::too_many_requests);
    REQUIRE(nlohmann::json::parse(rate_limited.body()).at("error").at("code") == "login_rate_limited");
    REQUIRE(rate_limited.body().find("wrong") == std::string::npos);
    REQUIRE(rate_limited[http::field::retry_after] == "60");
}

TEST_CASE("REST server requires a valid Bearer token on protected routes", "[rest_server]")
{
    RestServerFixture fixture;
    const std::uint32_t calls_before = fixture.handler_calls();

    const HttpResponseMessage missing = fixture.request_without_auth(
        http::verb::get, "/__test__/json");
    const HttpResponseMessage missing_with_invalid_json = fixture.request_without_auth(
        http::verb::post,
        "/__test__/json",
        "{invalid",
        "application/json");
    const HttpResponseMessage invalid = fixture.request(
        http::verb::get,
        "/__test__/json",
        {},
        {},
        {{"Authorization", "Bearer invalid-token"}});
    const HttpResponseMessage malformed = fixture.request(
        http::verb::get,
        "/__test__/json",
        {},
        {},
        {{"Authorization", "Basic credentials"}});
    const HttpResponseMessage unknown_without_token = fixture.request_without_auth(
        http::verb::post,
        "/not-a-route",
        "{invalid",
        "application/json");
    const HttpResponseMessage unknown_with_token = fixture.request(
        http::verb::post,
        "/not-a-route",
        "{invalid",
        "application/json");

    REQUIRE(missing.result() == http::status::unauthorized);
    REQUIRE(missing_with_invalid_json.result() == http::status::unauthorized);
    REQUIRE(invalid.result() == http::status::unauthorized);
    REQUIRE(malformed.result() == http::status::unauthorized);
    REQUIRE(unknown_without_token.result() == http::status::unauthorized);
    REQUIRE(unknown_with_token.result() == http::status::not_found);
    REQUIRE(nlohmann::json::parse(unknown_with_token.body()).at("error").at("code") == "route_not_found");
    const nlohmann::json missing_body = nlohmann::json::parse(missing.body());
    const nlohmann::json invalid_body = nlohmann::json::parse(invalid.body());
    const nlohmann::json malformed_body = nlohmann::json::parse(malformed.body());
    REQUIRE(missing_body.at("error").at("code") == invalid_body.at("error").at("code"));
    REQUIRE(missing_body.at("error").at("code") == malformed_body.at("error").at("code"));
    REQUIRE(missing_body.at("error").at("message") == invalid_body.at("error").at("message"));
    REQUIRE(missing_body.at("error").at("message") == malformed_body.at("error").at("message"));
    REQUIRE(missing[http::field::www_authenticate] == "Bearer");
    REQUIRE(fixture.handler_calls() == calls_before);

    const HttpResponseMessage logout_without_token = fixture.request_without_auth(
        http::verb::post, "/logout");
    REQUIRE(logout_without_token.result() == http::status::unauthorized);
}

TEST_CASE("REST server rejects expired Bearer tokens", "[rest_server]")
{
    RestServerFixture::Limits limits;
    limits.token_ttl = std::chrono::seconds(1);
    RestServerFixture fixture(limits);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    const HttpResponseMessage expired = fixture.request(http::verb::get, "/__test__/json");
    REQUIRE(expired.result() == http::status::unauthorized);
}

TEST_CASE("REST server logout revokes only the presented token", "[rest_server]")
{
    RestServerFixture fixture;

    const HttpResponseMessage second_login = fixture.request(
        http::verb::post,
        "/login",
        R"({"username":"Alice","password":"secret"})",
        "application/json");
    REQUIRE(second_login.result() == http::status::ok);
    const std::string second_token = nlohmann::json::parse(second_login.body()).at("access_token");

    const HttpResponseMessage logout = fixture.request(http::verb::post, "/logout");
    REQUIRE(logout.result() == http::status::no_content);
    REQUIRE(logout.body().empty());

    const HttpResponseMessage revoked = fixture.request(http::verb::get, "/__test__/json");
    REQUIRE(revoked.result() == http::status::unauthorized);

    const HttpResponseMessage second_token_request = fixture.request(
        http::verb::get,
        "/__test__/json",
        {},
        {},
        {{"Authorization", "Bearer " + second_token}});
    REQUIRE(second_token_request.result() == http::status::ok);

    const HttpResponseMessage lowercase_scheme_request = fixture.request(
        http::verb::get,
        "/__test__/json",
        {},
        {},
        {{"Authorization", "bearer " + second_token}});
    REQUIRE(lowercase_scheme_request.result() == http::status::ok);
}

TEST_CASE("REST server creates authenticated sessions through the global routes", "[rest_server]")
{
    RestServerFixture fixture;

    const HttpResponseMessage settings_without_token = fixture.request_without_auth(
        http::verb::get, "/list_settings");
    REQUIRE(settings_without_token.result() == http::status::unauthorized);
    const HttpResponseMessage profiles_without_token = fixture.request_without_auth(
        http::verb::get, "/session/session_missing/list_profiles");
    REQUIRE(profiles_without_token.result() == http::status::unauthorized);

    const HttpResponseMessage settings_wrong_method = fixture.request(
        http::verb::post, "/list_settings");
    REQUIRE(settings_wrong_method.result() == http::status::not_found);

    // This fixture never initializes the engine, so the catalog reports that the
    // definitions are not finalized yet.
    const HttpResponseMessage settings_uninitialized = fixture.request(
        http::verb::get, "/list_settings");
    REQUIRE(settings_uninitialized.result() == http::status::service_unavailable);
    REQUIRE(nlohmann::json::parse(settings_uninitialized.body()).at("error").at("code") ==
        "engine_not_initialized");

    const HttpResponseMessage first = fixture.request(http::verb::post, "/create_session");
    const HttpResponseMessage second = fixture.request(http::verb::post, "/create_session");
    REQUIRE(first.result() == http::status::created);
    REQUIRE(second.result() == http::status::created);
    const std::string first_id = nlohmann::json::parse(first.body()).at("session_id");
    const std::string second_id = nlohmann::json::parse(second.body()).at("session_id");
    REQUIRE(first_id.rfind("session_", 0) == 0);
    REQUIRE(second_id.rfind("session_", 0) == 0);
    REQUIRE(first_id != second_id);

    const HttpResponseMessage profiles_uninitialized = fixture.request(
        http::verb::get, "/session/" + first_id + "/list_profiles");
    REQUIRE(profiles_uninitialized.result() == http::status::service_unavailable);
    const HttpResponseMessage set_profile_uninitialized = fixture.request(
        http::verb::post,
        "/session/" + first_id + "/set_profile",
        "{}",
        "application/json");
    REQUIRE(set_profile_uninitialized.result() == http::status::service_unavailable);
    const HttpResponseMessage get_setting_uninitialized = fixture.request(
        http::verb::get,
        "/session/" + first_id + "/get_setting/layer_height");
    REQUIRE(get_setting_uninitialized.result() == http::status::service_unavailable);

    // The body is optional and carries only the session name.
    REQUIRE(nlohmann::json::parse(first.body()).at("name").is_null());
    const HttpResponseMessage empty_object = fixture.request(
        http::verb::post,
        "/create_session",
        "{}",
        "application/json");
    REQUIRE(empty_object.result() == http::status::created);
    REQUIRE(nlohmann::json::parse(empty_object.body()).at("name").is_null());
    const HttpResponseMessage body_rejected = fixture.request(
        http::verb::post,
        "/create_session",
        "{\"other\":1}",
        "application/json");
    REQUIRE(body_rejected.result() == http::status::bad_request);
}

TEST_CASE("REST server decodes URL paths once without form plus conversion", "[rest_server]")
{
    RestServerFixture fixture;

    const std::string target = "/__test__/echo/a+b%20c%2Fini%3Dvalue?x=a+b%20c";
    const HttpResponseMessage response = fixture.request(http::verb::get, target);
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json body = nlohmann::json::parse(response.body());
    REQUIRE(body.at("path") == "/__test__/echo/a+b c/ini=value");
    REQUIRE(body.at("raw_target") == target);
    REQUIRE(body.at("segments").get<std::vector<std::string>>() ==
        std::vector<std::string>({"__test__", "echo", "a+b c/ini=value"}));
    REQUIRE(body.at("query") == "x=a+b%20c");
    REQUIRE(!body.at("has_json").get<bool>());
    REQUIRE(body.at("body_size") == 0);

    // An encoded slash stays inside its segment: the decoded path cannot
    // distinguish it from a separator, so routing must use the segment list.
    const HttpResponseMessage encoded_slash = fixture.request(
        http::verb::get, "/__test__/echo/a%2Fb");
    REQUIRE(encoded_slash.result() == http::status::ok);
    const nlohmann::json encoded_body = nlohmann::json::parse(encoded_slash.body());
    REQUIRE(encoded_body.at("segments").get<std::vector<std::string>>() ==
        std::vector<std::string>({"__test__", "echo", "a/b"}));

    const std::uint32_t calls_before_bad = fixture.handler_calls();
    const HttpResponseMessage truncated = fixture.request(http::verb::get, "/__test__/echo/bad%2");
    REQUIRE(truncated.result() == http::status::bad_request);
    REQUIRE(nlohmann::json::parse(truncated.body()).at("error").at("code") == "invalid_request");

    const HttpResponseMessage nul = fixture.request(http::verb::get, "/__test__/echo/bad%00name");
    REQUIRE(nul.result() == http::status::bad_request);
    REQUIRE(nlohmann::json::parse(nul.body()).at("error").at("code") == "invalid_request");
    REQUIRE(fixture.handler_calls() == calls_before_bad);
}

TEST_CASE("REST server loads JSON configuration and applies CLI overrides", "[rest_server]")
{
    TempDirectory root;
    std::filesystem::create_directories(root.path() / "resources");
    std::filesystem::create_directories(root.path() / "profiles");
    std::filesystem::create_directories(root.path() / "work");
    std::ofstream(root.path() / "accounts.json") << R"({"users":{}})";

    const std::filesystem::path config_file = root.path() / "server.json";
    std::ofstream output(config_file);
    output << R"({
        "listen": {"address": "127.0.0.1", "port": 8080},
        "paths": {
            "resources": "resources",
            "profiles": "profiles",
            "work": "work",
            "accounts": "accounts.json"
        },
        "limits": {"max_request_body_bytes": 2048, "max_upload_bytes": 4096, "max_header_bytes": 4096, "max_connections": 4, "import_threads": 3, "max_instances_per_object": 5},
        "timeouts": {"request_seconds": 5},
        "auth": {"token_ttl_seconds": 120, "login_window_seconds": 30, "max_login_attempts": 3},
        "sessions": {"expiration_seconds": 240}
    })";
    output.close();

    Slic3r::Rest::CommandLineOverrides overrides;
    overrides.port = 0;
    const Slic3r::Rest::ServerConfig config = Slic3r::Rest::load_server_config(config_file, overrides);
    REQUIRE(config.bind_address == "127.0.0.1");
    REQUIRE(config.port == 0);
    REQUIRE(config.resource_directory == root.path() / "resources");
    REQUIRE(config.profiles_directory == root.path() / "profiles");
    REQUIRE(config.max_request_body_bytes == 2048);
    REQUIRE(config.max_upload_bytes == 4096);
    REQUIRE(config.import_threads == 3);
    REQUIRE(config.max_instances_per_object == 5);
    REQUIRE(config.request_timeout == std::chrono::seconds(5));
    REQUIRE(config.token_ttl == std::chrono::seconds(120));
    REQUIRE(config.session_ttl == std::chrono::seconds(240));
    REQUIRE(config.login_window == std::chrono::seconds(30));
    REQUIRE(config.max_login_attempts == 3);
}

TEST_CASE("REST server configuration validation rejects bad paths and limits", "[rest_server]")
{
    TempDirectory root;
    const Slic3r::Rest::ServerConfig valid = make_valid_config(root.path());
    REQUIRE_NOTHROW(valid.validate());
    REQUIRE(Slic3r::Rest::ServerConfig::defaults().max_request_body_bytes == 16ULL * 1024ULL * 1024ULL);
    REQUIRE(Slic3r::Rest::ServerConfig::defaults().max_upload_bytes == 1ULL * 1024ULL * 1024ULL * 1024ULL);
    REQUIRE(Slic3r::Rest::ServerConfig::defaults().import_threads == 2);
    REQUIRE(Slic3r::Rest::ServerConfig::defaults().max_instances_per_object == 1000);

    Slic3r::Rest::ServerConfig missing_resource = valid;
    missing_resource.resource_directory = root.path() / "missing";
    REQUIRE_THROWS_AS(missing_resource.validate(), std::invalid_argument);

    const std::filesystem::path work_file = root.path() / "work_file";
    std::ofstream(work_file) << "not a directory";
    Slic3r::Rest::ServerConfig file_as_work = valid;
    file_as_work.work_directory = work_file;
    REQUIRE_THROWS_AS(file_as_work.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig missing_accounts_parent = valid;
    missing_accounts_parent.accounts_file = root.path() / "missing" / "accounts.json";
    REQUIRE_THROWS_AS(missing_accounts_parent.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_body_limit = valid;
    zero_body_limit.max_request_body_bytes = 0;
    REQUIRE_THROWS_AS(zero_body_limit.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_upload_limit = valid;
    zero_upload_limit.max_upload_bytes = 0;
    REQUIRE_THROWS_AS(zero_upload_limit.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_header_limit = valid;
    zero_header_limit.max_header_bytes = 0;
    REQUIRE_THROWS_AS(zero_header_limit.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_connections = valid;
    zero_connections.max_connections = 0;
    REQUIRE_THROWS_AS(zero_connections.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_instances = valid;
    zero_instances.max_instances_per_object = 0;
    REQUIRE_THROWS_AS(zero_instances.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_import_threads = valid;
    zero_import_threads.import_threads = 0;
    REQUIRE_THROWS_AS(zero_import_threads.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_timeout = valid;
    zero_timeout.request_timeout = std::chrono::seconds(0);
    REQUIRE_THROWS_AS(zero_timeout.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig missing_accounts = valid;
    missing_accounts.accounts_file = root.path() / "missing_accounts.json";
    REQUIRE_THROWS_AS(missing_accounts.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_token_ttl = valid;
    zero_token_ttl.token_ttl = std::chrono::seconds(0);
    REQUIRE_THROWS_AS(zero_token_ttl.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_session_ttl = valid;
    zero_session_ttl.session_ttl = std::chrono::seconds(0);
    REQUIRE_THROWS_AS(zero_session_ttl.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_login_attempts = valid;
    zero_login_attempts.max_login_attempts = 0;
    REQUIRE_THROWS_AS(zero_login_attempts.validate(), std::invalid_argument);
}

TEST_CASE("REST instance limit configuration rejects invalid values", "[rest_server]")
{
    TempDirectory root;
    make_valid_config(root.path());
    const std::filesystem::path config_file = root.path() / "server.json";
    const std::vector<nlohmann::json> invalid{0, -1, true, 1.5, "2", std::uint64_t{1} << 32};
    for (const nlohmann::json &value : invalid) {
        CAPTURE(value.dump());
        const nlohmann::json document{
            {"paths", {{"resources", "resources"}, {"profiles", "profiles"},
                       {"work", "work"}, {"accounts", "accounts.json"}}},
            {"limits", {{"max_instances_per_object", value}}}
        };
        std::ofstream(config_file) << document.dump();
        try {
            Slic3r::Rest::load_server_config(config_file);
            FAIL("Invalid instance limit accepted");
        } catch (const std::invalid_argument &error) {
            CHECK(std::string(error.what()).find("limits.max_instances_per_object") != std::string::npos);
        }
    }
}

TEST_CASE("REST server enforces configured body and header limits", "[rest_server]")
{
    RestServerFixture::Limits limits;
    limits.max_body_bytes = 64;
    limits.max_header_bytes = 1024;
    RestServerFixture fixture(limits);

    const std::string allowed(64, 'x');
    const HttpResponseMessage at_limit = fixture.request(
        http::verb::post, "/__test__/echo/body", allowed);
    REQUIRE(at_limit.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(at_limit.body()).at("body_size") == 64);

    const std::string too_large(65, 'x');
    const HttpResponseMessage body_rejected = fixture.request(
        http::verb::post, "/__test__/echo/body", too_large);
    REQUIRE(body_rejected.result() == http::status::payload_too_large);
    REQUIRE(nlohmann::json::parse(body_rejected.body()).at("error").at("code") == "request_too_large");

    const std::string large_header(2048, 'h');
    const HttpResponseMessage header_rejected = fixture.request(
        http::verb::get,
        "/__test__/json",
        {},
        {},
        {{"x-large", large_header}});
    REQUIRE(header_rejected.result() == http::status::request_header_fields_too_large);
    REQUIRE(nlohmann::json::parse(header_rejected.body()).at("error").at("code") == "request_header_too_large");
}

TEST_CASE("REST server closes stalled connections after the request timeout", "[rest_server]")
{
    RestServerFixture::Limits limits;
    limits.timeout = std::chrono::seconds(1);
    RestServerFixture fixture(limits);

    asio::io_context context;
    beast::tcp_stream stream(context);
    stream.connect(Tcp::endpoint(asio::ip::make_address("127.0.0.1"), fixture.bound_port()));
    const std::string partial = "GET /__test__/json HTTP/1.1\r\nHost: 127.0.0.1\r\n";
    asio::write(stream, asio::buffer(partial));
    stream.expires_after(std::chrono::seconds(3));

    std::array<char, 64> data{};
    ErrorCode error;
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    stream.read_some(asio::buffer(data), error);
    const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);

    REQUIRE(error == asio::error::eof);
    REQUIRE(elapsed < std::chrono::seconds(3));
}

// A buffered JSON body must arrive within one request deadline: sending one
// byte just before each idle delay would otherwise keep the connection alive
// indefinitely. The server closes at the deadline, so once the client stops
// dripping the connection is already gone instead of still waiting for data.
TEST_CASE("REST server applies one deadline to a slowly sent JSON body", "[rest_server]")
{
    RestServerFixture::Limits limits;
    limits.timeout = std::chrono::seconds(1);
    RestServerFixture fixture(limits);

    asio::io_context context;
    beast::tcp_stream stream(context);
    stream.connect(Tcp::endpoint(asio::ip::make_address("127.0.0.1"), fixture.bound_port()));
    const std::string header =
        "POST /__test__/json HTTP/1.1\r\n"
        "Host: 127.0.0.1\r\n"
        "Authorization: Bearer " + fixture.token() + "\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: 32\r\n\r\n";
    asio::write(stream, asio::buffer(header));
    for (int index = 0; index < 8; ++index) {
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        ErrorCode ignored;
        asio::write(stream, asio::buffer(std::string(" ")), ignored);
    }

    stream.expires_after(std::chrono::seconds(3));
    std::array<char, 64> data{};
    ErrorCode error;
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    stream.read_some(asio::buffer(data), error);
    const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);

    REQUIRE(error);
    REQUIRE(elapsed < std::chrono::milliseconds(500));
}

TEST_CASE("REST server enforces the connection limit", "[rest_server]")
{
    RestServerFixture::Limits limits;
    limits.max_connections = 1;
    limits.timeout = std::chrono::seconds(10);
    RestServerFixture fixture(limits);

    asio::io_context context;
    Tcp::socket occupied = fixture.connect_socket(context);
    // Keep the single allowed connection in a half-read state so it stays
    // registered while the second connection is attempted.
    const std::string partial = "GET /__test__/json HTTP/1.1\r\nHost: 127.0.0.1\r\n";
    asio::write(occupied, asio::buffer(partial));

    Tcp::socket rejected = fixture.connect_socket(context);
    http::request<http::string_body> message(http::verb::get, "/__test__/json", 11);
    message.set(http::field::host, "127.0.0.1");
    message.prepare_payload();
    ErrorCode write_error;
    http::write(rejected, message, write_error);

    beast::flat_buffer buffer;
    HttpResponseMessage response;
    ErrorCode read_error;
    http::read(rejected, buffer, response, read_error);
    REQUIRE(read_error);

    ErrorCode ignored;
    occupied.close(ignored);
    rejected.close(ignored);
}

TEST_CASE("REST server restarts cleanly", "[rest_server]")
{
    RestServerFixture fixture;
    REQUIRE(fixture.running());

    const HttpResponseMessage before = fixture.request(http::verb::get, "/__test__/json");
    REQUIRE(before.result() == http::status::ok);

    fixture.stop();
    REQUIRE(!fixture.running());
    REQUIRE(fixture.bound_port() == 0);

    fixture.start();
    REQUIRE(fixture.running());
    const HttpResponseMessage after = fixture.request(http::verb::get, "/__test__/json");
    REQUIRE(after.result() == http::status::ok);

    fixture.stop();
    REQUIRE(!fixture.running());
}
