/*
 * Loopback tests for the phase-two REST transport.
 *
 * The fixture injects private routes into the library and never adds those
 * routes to the standalone executable. Configuration, size limits, timeouts,
 * connection limits and a full stop/start cycle are exercised here.
 */

#include <boost/asio/connect.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/write.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <catch_main.hpp>
#include <nlohmann/json.hpp>

#include "rest_server.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace {

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;

using ErrorCode = boost::system::error_code;
using Tcp = asio::ip::tcp;
using HttpResponseMessage = http::response<http::string_body>;

class TempDirectory {
public:
    TempDirectory()
    {
        static std::atomic<std::uint64_t> next_id{1};
        root_ = std::filesystem::temp_directory_path() /
            ("superslicer_rest_" + std::to_string(next_id.fetch_add(1)));
        std::filesystem::create_directories(root_);
    }

    ~TempDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(root_, ignored);
    }

    const std::filesystem::path &path() const { return root_; }

private:
    std::filesystem::path root_;
};

Slic3r::Rest::ServerConfig make_valid_config(const std::filesystem::path &root)
{
    Slic3r::Rest::ServerConfig config = Slic3r::Rest::ServerConfig::defaults();
    config.bind_address = "127.0.0.1";
    config.port = 0;
    config.resource_directory = root / "resources";
    config.profiles_directory = root / "profiles";
    config.work_directory = root / "work";
    config.accounts_file = root / "accounts.json";
    std::filesystem::create_directories(config.resource_directory);
    std::filesystem::create_directories(config.profiles_directory);
    std::filesystem::create_directories(config.work_directory);
    return config;
}

class RestServerFixture {
public:
    struct Limits {
        std::uint64_t max_body_bytes = 1024 * 1024;
        std::uint64_t max_header_bytes = 16 * 1024;
        std::uint32_t max_connections = 64;
        std::chrono::seconds timeout{30};
    };

    explicit RestServerFixture(Limits limits = {})
        : root_directory_(),
          limits_(limits),
          server_(create_server())
    {
        server_->start();
    }

    ~RestServerFixture()
    {
        server_->stop();
    }

    RestServerFixture(const RestServerFixture &) = delete;
    RestServerFixture &operator=(const RestServerFixture &) = delete;

    void start() { server_->start(); }
    void stop() { server_->stop(); }
    bool running() const { return server_->running(); }
    std::uint16_t bound_port() const { return server_->bound_port(); }

    HttpResponseMessage request(
        http::verb method,
        const std::string &target,
        const std::string &body = {},
        const std::string &content_type = {},
        const std::vector<std::pair<std::string, std::string>> &extra_headers = {}) const
    {
        asio::io_context context;
        Tcp::socket socket = connect_socket(context);

        http::request<http::string_body> message(method, target, 11);
        message.set(http::field::host, "127.0.0.1");
        message.set(http::field::user_agent, "rest_server_tests");
        for (const std::pair<std::string, std::string> &header : extra_headers)
            message.set(header.first, header.second);
        if (!content_type.empty())
            message.set(http::field::content_type, content_type);
        message.body() = body;
        message.prepare_payload();
        http::write(socket, message);

        beast::flat_buffer buffer;
        HttpResponseMessage response;
        http::read(socket, buffer, response);
        ErrorCode ignored;
        socket.shutdown(Tcp::socket::shutdown_both, ignored);
        return response;
    }

    Tcp::socket connect_socket(asio::io_context &context) const
    {
        Tcp::resolver resolver(context);
        Tcp::socket socket(context);
        asio::connect(
            socket,
            resolver.resolve("127.0.0.1", std::to_string(server_->bound_port())));
        return socket;
    }

    std::uint32_t handler_calls() const
    {
        return handler_calls_.load();
    }

private:
    std::unique_ptr<Slic3r::Rest::RestServer> create_server()
    {
        const std::filesystem::path root = root_directory_.path();
        Slic3r::Rest::ServerConfig config = Slic3r::Rest::ServerConfig::defaults();
        config.bind_address = "127.0.0.1";
        config.port = 0;
        config.resource_directory = root / "resources";
        config.profiles_directory = root / "profiles";
        config.work_directory = root / "work";
        config.accounts_file = root / "accounts.json";
        config.max_request_body_bytes = limits_.max_body_bytes;
        config.max_header_bytes = limits_.max_header_bytes;
        config.max_connections = limits_.max_connections;
        config.request_timeout = limits_.timeout;
        std::filesystem::create_directories(config.resource_directory);
        std::filesystem::create_directories(config.profiles_directory);
        std::filesystem::create_directories(config.work_directory);

        Slic3r::Rest::RequestHandler handler = [this](const Slic3r::Rest::HttpRequest &request) {
            handler_calls_.fetch_add(1);
            if (request.path == "/__test__/json")
                return Slic3r::Rest::HttpResponse::json(200, nlohmann::json{{"ok", true}});
            if (request.path == "/__test__/empty")
                return Slic3r::Rest::HttpResponse::empty(204);
            if (request.path.rfind("/__test__/echo/", 0) == 0) {
                return Slic3r::Rest::HttpResponse::json(200, nlohmann::json{
                    {"path", request.path},
                    {"raw_target", request.raw_target},
                    {"segments", request.path_segments},
                    {"query", request.query},
                    {"has_json", request.json_body != nullptr},
                    {"body_size", request.body.size()}
                });
            }
            return Slic3r::Rest::HttpResponse::json(
                404, nlohmann::json{{"error", nlohmann::json{{"code", "private_route_not_found"}}}});
        };
        return std::make_unique<Slic3r::Rest::RestServer>(std::move(config), std::move(handler));
    }

    TempDirectory root_directory_;
    Limits limits_;
    std::unique_ptr<Slic3r::Rest::RestServer> server_;
    std::atomic<std::uint32_t> handler_calls_{0};
};

} // namespace

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

    // Business routes are denied until the authentication phase wires them: a
    // business-looking path must not reach any handler yet.
    const HttpResponseMessage login = fixture.request(http::verb::post, "/login", "{}", "application/json");
    REQUIRE(login.result() == http::status::not_found);
    REQUIRE(nlohmann::json::parse(login.body()).at("error").at("code") == "route_not_found");
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
        "limits": {"max_request_body_bytes": 2048, "max_header_bytes": 4096, "max_connections": 4},
        "timeouts": {"request_seconds": 5}
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
    REQUIRE(config.request_timeout == std::chrono::seconds(5));
}

TEST_CASE("REST server configuration validation rejects bad paths and limits", "[rest_server]")
{
    TempDirectory root;
    const Slic3r::Rest::ServerConfig valid = make_valid_config(root.path());
    REQUIRE_NOTHROW(valid.validate());
    REQUIRE(Slic3r::Rest::ServerConfig::defaults().max_request_body_bytes == 16ULL * 1024ULL * 1024ULL);

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

    Slic3r::Rest::ServerConfig zero_header_limit = valid;
    zero_header_limit.max_header_bytes = 0;
    REQUIRE_THROWS_AS(zero_header_limit.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_connections = valid;
    zero_connections.max_connections = 0;
    REQUIRE_THROWS_AS(zero_connections.validate(), std::invalid_argument);

    Slic3r::Rest::ServerConfig zero_timeout = valid;
    zero_timeout.request_timeout = std::chrono::seconds(0);
    REQUIRE_THROWS_AS(zero_timeout.validate(), std::invalid_argument);
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
