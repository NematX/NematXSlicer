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

class SessionRegistry;
struct HttpUpload;

struct ServerConfig {
    std::string bind_address;
    std::uint16_t port;
    std::filesystem::path resource_directory;
    std::filesystem::path profiles_directory;
    std::filesystem::path work_directory;
    std::filesystem::path accounts_file;
    std::uint64_t max_request_body_bytes;
    std::uint64_t max_upload_bytes;
    std::uint64_t max_header_bytes;
    std::uint32_t max_connections;
    std::uint32_t max_instances_per_object;
    // Threads parsing uploaded models away from the network thread, so a long
    // import never delays other requests.
    std::uint32_t import_threads;
    // Excess slice requests wait in slicing state until a worker is available.
    std::uint32_t slice_threads;
    // Deadline for a whole buffered request, and idle delay between two reads
    // of a streamed upload.
    std::chrono::seconds request_timeout;
    std::chrono::seconds token_ttl;
    // Session inactivity delay; distinct from the bearer token TTL.
    std::chrono::seconds session_ttl;
    std::chrono::seconds login_window;
    std::uint32_t max_login_attempts;

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
    std::shared_ptr<const HttpUpload> upload;
};

// Completed multipart upload. The destructor deletes the temporary file, so
// the file lives exactly as long as the last owner of this object.
struct HttpUpload {
    std::filesystem::path temporary_path;
    std::string filename;
    std::string extension;
    std::uint64_t byte_count = 0;

    ~HttpUpload();
};

struct HttpResponse {
    unsigned int status = 200;
    std::string body;
    std::string content_type = "application/json; charset=utf-8";
    // A nonempty path replaces body; its file must survive asynchronous transport reads.
    std::filesystem::path body_file;
    // Transport retains this owner until completion or disconnect, then permits file cleanup.
    std::shared_ptr<const void> body_file_owner;
    bool no_body = false;
    // Extra headers set verbatim on the wire, for example Cache-Control on a
    // credential response.
    std::map<std::string, std::string> headers;
    bool stream_request = false;

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
    // In-process access to the live sessions, used by integration tests to put
    // a session in a state no public route can reach yet (for example a
    // running slicing job). Valid only while the server is running.
    SessionRegistry &session_registry() const;
    // Test hook called on the import thread before each model is parsed; a
    // test can block in it to prove the network thread keeps serving requests.
    // Must be set before start().
    void set_import_observer(std::function<void()> observer);
    // Test hook on the slicing worker before the engine starts; set before start().
    void set_slice_observer(std::function<void()> observer);
    // Test hook after a run accepts progress, outside the session lock; set before start().
    void set_progress_observer(std::function<void()> observer);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Slic3r::Rest
