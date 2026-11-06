/*
 * Shared loopback fixture for REST server tests.
 *
 * Each fixture owns temporary paths, an authenticated account file, and a
 * real listener. Profile tests can opt into synthetic INI files without
 * changing the uninitialized-engine transport tests.
 */

#pragma once

#include <boost/asio/connect.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/write.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "auth.hpp"
#include "rest_server.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace RestServerTest {

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
        // CTest runs multiple executables: process-local counters would collide.
        root_ = std::filesystem::temp_directory_path() /
            ("superslicer_rest_" + boost::uuids::to_string(boost::uuids::random_generator()()));
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

inline Slic3r::Rest::ServerConfig make_valid_config(const std::filesystem::path &root)
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
    std::ofstream(config.accounts_file) << R"({"users":{}})";
    return config;
}

class RestServerFixture {
public:
    struct Limits {
        std::uint64_t max_body_bytes = 1024 * 1024;
        std::uint64_t max_upload_bytes = 1024 * 1024;
        std::uint64_t max_header_bytes = 16 * 1024;
        std::uint32_t max_connections = 64;
        std::uint32_t max_instances_per_object = 1000;
        std::uint32_t slice_threads = 2;
        std::chrono::seconds timeout{300};
        std::chrono::seconds token_ttl{24 * 60 * 60};
        std::chrono::seconds session_ttl{24 * 60 * 60};
    };

    explicit RestServerFixture(Limits limits = {}, bool with_account = true, bool with_profiles = false,
        std::filesystem::path work_directory_name = "work", std::optional<std::string> account_document = {},
        std::filesystem::path profiles_source = {})
        : root_directory_(),
          limits_(limits),
          with_account_(with_account),
          with_profiles_(with_profiles),
          work_directory_name_(std::move(work_directory_name)),
          account_document_(std::move(account_document)),
          profiles_source_(std::move(profiles_source)),
          server_(create_server())
    {
        server_->start();
        if (with_account_)
            refresh_token();
    }

    ~RestServerFixture()
    {
        server_->stop();
    }

    RestServerFixture(const RestServerFixture &) = delete;
    RestServerFixture &operator=(const RestServerFixture &) = delete;

    void start()
    {
        server_->start();
        if (with_account_)
            refresh_token();
    }
    void stop() { server_->stop(); }
    bool running() const { return server_->running(); }
    std::uint16_t bound_port() const { return server_->bound_port(); }

    HttpResponseMessage request(
        http::verb method,
        const std::string &target,
        const std::string &body = {},
        const std::string &content_type = {},
        const std::vector<std::pair<std::string, std::string>> &extra_headers = {},
        bool authenticate = true) const
    {
        asio::io_context context;
        Tcp::socket socket = connect_socket(context);

        http::request<http::string_body> message(method, target, 11);
        message.set(http::field::host, "127.0.0.1");
        message.set(http::field::user_agent, "rest_server_tests");
        if (authenticate && !default_token_.empty())
            message.set(http::field::authorization, "Bearer " + default_token_);
        for (const std::pair<std::string, std::string> &header : extra_headers)
            message.set(header.first, header.second);
        if (!content_type.empty())
            message.set(http::field::content_type, content_type);
        message.body() = body;
        message.prepare_payload();
        http::write(socket, message);

        beast::flat_buffer buffer;
        http::response_parser<http::string_body> parser;
        // Generated files include serialized settings as well as their output;
        // their size is independent of the upload/request-body limit.
        parser.body_limit(128 * 1024 * 1024);
        http::read(socket, buffer, parser);
        HttpResponseMessage response = parser.release();
        ErrorCode ignored;
        socket.shutdown(Tcp::socket::shutdown_both, ignored);
        return response;
    }

    HttpResponseMessage request_without_auth(
        http::verb method,
        const std::string &target,
        const std::string &body = {},
        const std::string &content_type = {},
        const std::vector<std::pair<std::string, std::string>> &extra_headers = {}) const
    {
        return request(method, target, body, content_type, extra_headers, false);
    }

    std::string login_token(const std::string &username, const std::string &password) const
    {
        const HttpResponseMessage response = request(
            http::verb::post,
            "/login",
            nlohmann::json{{"username", username}, {"password", password}}.dump(),
            "application/json",
            {},
            false);
        return nlohmann::json::parse(response.body()).at("access_token").get<std::string>();
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

    // Live registry of the running server, used to drive session states such
    // as an active slicing job that the HTTP routes cannot produce yet.
    Slic3r::Rest::SessionRegistry &sessions() const
    {
        return server_->session_registry();
    }

    // The server accepts the import hook only while stopped: call stop(), set
    // it, then start() again.
    void set_import_observer(std::function<void()> observer)
    {
        server_->set_import_observer(std::move(observer));
    }

    void set_write_observer(std::function<void(std::function<void()>)> observer)
    {
        server_->set_write_observer(std::move(observer));
    }

    // The slice hook follows the same stopped-only lifecycle as the import hook.
    void set_slice_observer(std::function<void()> observer)
    {
        server_->set_slice_observer(std::move(observer));
    }

    // The progress hook follows the same stopped-only lifecycle as the worker hooks.
    void set_progress_observer(std::function<void(const std::string &)> observer)
    {
        server_->set_progress_observer(std::move(observer));
    }

    const std::string &token() const
    {
        return default_token_;
    }

    std::filesystem::path work_directory() const
    {
        return root_directory_.path() / work_directory_name_;
    }

    // Tests install additional INIs only while the listener is stopped.
    std::filesystem::path profiles_directory() const
    {
        return root_directory_.path() / "profiles";
    }

    std::size_t work_file_count() const
    {
        std::size_t count = 0;
        const std::filesystem::path work = root_directory_.path() / work_directory_name_;
        for (const std::filesystem::directory_entry &entry :
             std::filesystem::recursive_directory_iterator(work))
            if (entry.is_regular_file())
                ++count;
        return count;
    }

    std::map<std::string, std::string> profile_files() const
    {
        std::map<std::string, std::string> files;
        const std::filesystem::path profiles = root_directory_.path() / "profiles";
        for (const std::filesystem::directory_entry &entry :
             std::filesystem::recursive_directory_iterator(profiles)) {
            if (!entry.is_regular_file())
                continue;
            std::ifstream input(entry.path(), std::ios::binary);
            const std::string relative = std::filesystem::relative(entry.path(), profiles).generic_string();
            files.emplace(relative, std::string(
                std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()));
        }
        return files;
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
        config.work_directory = root / work_directory_name_;
        config.accounts_file = root / "accounts.json";
        config.max_request_body_bytes = limits_.max_body_bytes;
        config.max_upload_bytes = limits_.max_upload_bytes;
        config.max_header_bytes = limits_.max_header_bytes;
        config.max_connections = limits_.max_connections;
        config.max_instances_per_object = limits_.max_instances_per_object;
        config.slice_threads = limits_.slice_threads;
        config.request_timeout = limits_.timeout;
        config.token_ttl = limits_.token_ttl;
        config.session_ttl = limits_.session_ttl;
        std::filesystem::create_directories(config.resource_directory);
        std::filesystem::create_directories(config.profiles_directory);
        std::filesystem::create_directories(config.work_directory);
        // Vendor tests exercise the public server with the same on-disk catalog.
        if (!profiles_source_.empty())
            std::filesystem::copy(profiles_source_, config.profiles_directory,
                std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
        if (with_profiles_) {
            // Empty synthetic INIs inherit the engine defaults; printer INIs
            // only connect each printer to its default filament profile.
            std::filesystem::create_directories(config.profiles_directory / "print");
            std::filesystem::create_directories(config.profiles_directory / "printer");
            std::filesystem::create_directories(config.profiles_directory / "filament");
            std::ofstream(config.profiles_directory / "print" / "rest-print.ini");
            std::ofstream(config.profiles_directory / "printer" / "rest-printer-a.ini")
                << "default_filament_profile = rest-filament-a\n";
            std::ofstream(config.profiles_directory / "printer" / "rest-printer-b.ini")
                << "default_filament_profile = rest-filament-b\n";
            std::ofstream(config.profiles_directory / "printer" / "a-sla-printer.ini")
                << "printer_technology = SLA\n";
            std::ofstream(config.profiles_directory / "filament" / "rest-filament-a.ini");
            std::ofstream(config.profiles_directory / "filament" / "rest-filament-b.ini");
        }
        if (account_document_) {
            // Launch tests use the exact document emitted by the account tool.
            std::ofstream(config.accounts_file) << *account_document_;
        } else if (with_account_) {
            Slic3r::Rest::Argon2Parameters parameters;
            parameters.memory_kib = 8U * 1024U;
            parameters.time_cost = 2;
            parameters.parallelism = 1;
            parameters.hash_bytes = 16;
            parameters.salt_bytes = 16;
            const std::string password_hash = Slic3r::Rest::hash_password("secret", parameters);
            const nlohmann::json accounts = {
                {"users", {
                    {"Alice", {{"password_hash", password_hash}}},
                    {"Bob", {{"password_hash", password_hash}}}
                }}
            };
            std::ofstream(config.accounts_file) << accounts.dump();
        } else {
            std::ofstream(config.accounts_file) << R"({"users":{}})";
        }

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

    void refresh_token()
    {
        default_token_ = login_token("Alice", "secret");
    }

    TempDirectory root_directory_;
    Limits limits_;
    bool with_account_;
    bool with_profiles_;
    std::filesystem::path work_directory_name_;
    std::optional<std::string> account_document_;
    std::filesystem::path profiles_source_;
    std::unique_ptr<Slic3r::Rest::RestServer> server_;
    std::string default_token_;
    std::atomic<std::uint32_t> handler_calls_{0};
};

} // namespace RestServerTest
