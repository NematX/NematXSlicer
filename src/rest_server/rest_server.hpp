/*
 * Public configuration and lifecycle API for the REST transport.
 *
 * The server owns its listener and worker thread. HTTP parsing and routing stay
 * in the implementation, while tests may inject a private handler without
 * adding any route to the standalone executable.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::Rest {

struct ServerConfig {
    std::string bind_address;
    std::uint16_t port;
    std::filesystem::path resource_directory;
    std::filesystem::path profiles_directory;
    std::filesystem::path work_directory;
    std::filesystem::path accounts_file;
    std::uint64_t max_request_body_bytes;
    std::uint64_t max_header_bytes;
    std::uint32_t max_connections;
    std::chrono::seconds request_timeout;

    static ServerConfig defaults();
    void validate() const;
};

struct CommandLineOverrides {
    std::optional<std::string> bind_address;
    std::optional<std::uint16_t> port;
    std::optional<std::filesystem::path> resource_directory;
    std::optional<std::filesystem::path> profiles_directory;
    std::optional<std::filesystem::path> work_directory;
    std::optional<std::filesystem::path> accounts_file;
};

ServerConfig load_server_config(
    const std::filesystem::path &config_file,
    const CommandLineOverrides &overrides = {});

struct HttpRequest {
    std::string method;
    std::string raw_target;
    // Decoded path segments, in order, with each segment decoded exactly once.
    // Routes must match on these segments: joining them into `path` loses the
    // boundary between an encoded slash inside a segment and a real separator.
    std::vector<std::string> path_segments;
    std::string path;
    std::string query;
    std::string request_id;
    std::map<std::string, std::string> headers;
    std::string body;
    std::shared_ptr<const nlohmann::json> json_body;
};

struct HttpResponse {
    unsigned int status = 200;
    std::string body;
    std::string content_type = "application/json; charset=utf-8";
    bool no_body = false;

    static HttpResponse json(unsigned int status, const nlohmann::json &payload);
    static HttpResponse error(
        unsigned int status,
        const std::string &code,
        const std::string &message,
        const std::string &request_id,
        const nlohmann::json *details = nullptr);
    static HttpResponse empty(unsigned int status);
};

using RequestHandler = std::function<HttpResponse(const HttpRequest &)>;

class RestServer {
public:
    explicit RestServer(ServerConfig config, RequestHandler private_test_handler = {});
    ~RestServer();

    RestServer(const RestServer &) = delete;
    RestServer &operator=(const RestServer &) = delete;

    void start();
    void stop() noexcept;
    bool running() const noexcept;
    std::uint16_t bound_port() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Slic3r::Rest
