/*
 * REST configuration, HTTP transport and request pipeline.
 *
 * Configuration is validated before the listener binds. Each connection reads
 * one bounded HTTP request, decodes its path once, validates JSON when declared,
 * passes only private test routes to the injected handler, and closes cleanly.
 * Business routes and authentication are added by later implementation phases.
 */

#include "rest_server.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cctype>
#include <fstream>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace Slic3r::Rest {

namespace {

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;

using ErrorCode = boost::system::error_code;
using Tcp = asio::ip::tcp;
using HttpRequestMessage = http::request<http::string_body>;
using HttpResponseMessage = http::response<http::string_body>;
using HttpRequestParser = http::request_parser<http::string_body>;

// Default buffered JSON body limit. The specification's 1 GiB upload limit
// belongs to the streamed multipart route, not to buffered JSON bodies.
constexpr std::uint64_t default_max_request_body_bytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_header_bytes = 16ULL * 1024ULL;
constexpr std::uint32_t default_max_connections = 64;
constexpr std::int64_t default_request_timeout_seconds = 30;

struct ParsedTarget {
    std::string path;
    std::vector<std::string> segments;
    std::string query;
};

std::string make_request_id()
{
    static std::atomic<std::uint64_t> next_id{1};
    return "req_" + std::to_string(next_id.fetch_add(1));
}

std::string lower_ascii(std::string value)
{
    for (char &character : value)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return value;
}

std::string json_media_type(const std::string &value)
{
    const std::size_t separator = value.find(';');
    return lower_ascii(value.substr(0, separator));
}

std::string json_error_message(const std::string &field, const std::string &message)
{
    return field + ": " + message;
}

HttpResponse make_error_response(
    unsigned int status,
    const std::string &code,
    const std::string &message,
    const std::string &request_id,
    const nlohmann::json *details = nullptr)
{
    return HttpResponse::error(status, code, message, request_id, details);
}

std::uint64_t read_unsigned_json(const nlohmann::json &value, const std::string &field)
{
    if (value.is_number_unsigned())
        return value.get<std::uint64_t>();
    if (value.is_number_integer()) {
        const std::int64_t integer_value = value.get<std::int64_t>();
        if (integer_value >= 0)
            return static_cast<std::uint64_t>(integer_value);
    }
    throw std::invalid_argument(json_error_message(field, "expected a non-negative integer"));
}

// Resolve a relative configuration path against the directory of the
// configuration file, never against the process working directory.
std::filesystem::path read_path_json(
    const nlohmann::json &value,
    const std::filesystem::path &base_directory,
    const std::string &field)
{
    if (!value.is_string())
        throw std::invalid_argument(json_error_message(field, "expected a path string"));

    const std::string path_text = value.get<std::string>();
    if (path_text.empty())
        throw std::invalid_argument(json_error_message(field, "must not be empty"));

    const std::filesystem::path path_value(path_text);
    if (path_value.is_absolute())
        return path_value;
    return base_directory / path_value;
}

void apply_json_config(
    ServerConfig &config,
    const nlohmann::json &document,
    const std::filesystem::path &base_directory)
{
    if (!document.is_object())
        throw std::invalid_argument("configuration root must be a JSON object");

    if (document.contains("listen")) {
        const nlohmann::json &listen = document.at("listen");
        if (!listen.is_object())
            throw std::invalid_argument("listen must be a JSON object");
        if (listen.contains("address")) {
            if (!listen.at("address").is_string())
                throw std::invalid_argument("listen.address must be a string");
            config.bind_address = listen.at("address").get<std::string>();
        }
        if (listen.contains("port")) {
            const std::uint64_t port = read_unsigned_json(listen.at("port"), "listen.port");
            if (port > std::numeric_limits<std::uint16_t>::max())
                throw std::invalid_argument("listen.port is outside the TCP port range");
            config.port = static_cast<std::uint16_t>(port);
        }
    }

    if (document.contains("paths")) {
        const nlohmann::json &paths = document.at("paths");
        if (!paths.is_object())
            throw std::invalid_argument("paths must be a JSON object");
        if (paths.contains("resources"))
            config.resource_directory = read_path_json(paths.at("resources"), base_directory, "paths.resources");
        if (paths.contains("profiles"))
            config.profiles_directory = read_path_json(paths.at("profiles"), base_directory, "paths.profiles");
        if (paths.contains("work"))
            config.work_directory = read_path_json(paths.at("work"), base_directory, "paths.work");
        if (paths.contains("accounts"))
            config.accounts_file = read_path_json(paths.at("accounts"), base_directory, "paths.accounts");
    }

    if (document.contains("limits")) {
        const nlohmann::json &limits = document.at("limits");
        if (!limits.is_object())
            throw std::invalid_argument("limits must be a JSON object");
        if (limits.contains("max_request_body_bytes"))
            config.max_request_body_bytes = read_unsigned_json(
                limits.at("max_request_body_bytes"), "limits.max_request_body_bytes");
        if (limits.contains("max_header_bytes"))
            config.max_header_bytes = read_unsigned_json(
                limits.at("max_header_bytes"), "limits.max_header_bytes");
        if (limits.contains("max_connections")) {
            const std::uint64_t max_connections = read_unsigned_json(
                limits.at("max_connections"), "limits.max_connections");
            if (max_connections > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("limits.max_connections is too large");
            config.max_connections = static_cast<std::uint32_t>(max_connections);
        }
    }

    if (document.contains("timeouts")) {
        const nlohmann::json &timeouts = document.at("timeouts");
        if (!timeouts.is_object())
            throw std::invalid_argument("timeouts must be a JSON object");
        if (timeouts.contains("request_seconds")) {
            const std::uint64_t seconds = read_unsigned_json(
                timeouts.at("request_seconds"), "timeouts.request_seconds");
            if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                throw std::invalid_argument("timeouts.request_seconds is too large");
            config.request_timeout = std::chrono::seconds(static_cast<std::int64_t>(seconds));
        }
    }
}

void apply_overrides(ServerConfig &config, const CommandLineOverrides &overrides)
{
    if (overrides.bind_address.has_value())
        config.bind_address = overrides.bind_address.value();
    if (overrides.port.has_value())
        config.port = overrides.port.value();
    if (overrides.resource_directory.has_value())
        config.resource_directory = overrides.resource_directory.value();
    if (overrides.profiles_directory.has_value())
        config.profiles_directory = overrides.profiles_directory.value();
    if (overrides.work_directory.has_value())
        config.work_directory = overrides.work_directory.value();
    if (overrides.accounts_file.has_value())
        config.accounts_file = overrides.accounts_file.value();
}

void validate_directory(const std::filesystem::path &path, const std::string &field)
{
    std::error_code error;
    const bool is_directory = std::filesystem::is_directory(path, error);
    if (error)
        throw std::invalid_argument(json_error_message(field, error.message()));
    if (!is_directory)
        throw std::invalid_argument(json_error_message(field, "must be an existing directory"));
}

void validate_parent_directory(const std::filesystem::path &path, const std::string &field)
{
    std::filesystem::path parent = path.parent_path();
    if (parent.empty())
        parent = std::filesystem::current_path();
    validate_directory(parent, field + ".parent");
}

int hex_value(char character)
{
    if (character >= '0' && character <= '9')
        return character - '0';
    if (character >= 'a' && character <= 'f')
        return character - 'a' + 10;
    if (character >= 'A' && character <= 'F')
        return character - 'A' + 10;
    return -1;
}

bool decode_path_segment(std::string_view encoded, std::string &decoded)
{
    decoded.clear();
    decoded.reserve(encoded.size());
    for (std::size_t index = 0; index < encoded.size(); ++index) {
        if (encoded[index] != '%') {
            decoded.push_back(encoded[index]);
            continue;
        }
        if (index + 2 >= encoded.size())
            return false;
        const int high = hex_value(encoded[index + 1]);
        const int low = hex_value(encoded[index + 2]);
        if (high < 0 || low < 0)
            return false;
        const char character = static_cast<char>((high << 4) | low);
        // A decoded NUL would truncate in any C API downstream and has no
        // legitimate use in a route segment.
        if (character == '\0')
            return false;
        decoded.push_back(character);
        index += 2;
    }
    return true;
}

// Split the raw target on '/' and decode each segment exactly once, keeping an
// encoded separator inside its segment.
bool parse_target(const std::string &target, ParsedTarget &parsed)
{
    if (target.empty() || target.front() != '/')
        return false;

    const std::size_t query_separator = target.find('?');
    const std::string raw_path = target.substr(0, query_separator);
    parsed.query = query_separator == std::string::npos
        ? std::string()
        : target.substr(query_separator + 1);
    parsed.path = "/";

    std::size_t segment_start = 1;
    while (segment_start <= raw_path.size()) {
        std::size_t segment_end = raw_path.find('/', segment_start);
        if (segment_end == std::string::npos)
            segment_end = raw_path.size();

        std::string decoded_segment;
        if (!decode_path_segment(
                std::string_view(raw_path).substr(segment_start, segment_end - segment_start),
                decoded_segment))
            return false;
        parsed.segments.push_back(decoded_segment);
        parsed.path += decoded_segment;

        if (segment_end == raw_path.size())
            break;
        parsed.path.push_back('/');
        segment_start = segment_end + 1;
    }
    return true;
}

std::string header_value(const HttpRequest &request, const std::string &name)
{
    const std::map<std::string, std::string>::const_iterator iterator = request.headers.find(name);
    if (iterator == request.headers.end())
        return {};
    return iterator->second;
}

// Copy method, target, headers and buffered body out of the parser. The target
// is decoded once here; JSON is parsed later by the route that declares it.
bool build_request_context(
    HttpRequestMessage &message,
    HttpRequest &request,
    HttpResponse &error_response,
    const std::string &request_id)
{
    ParsedTarget target;
    const boost::core::basic_string_view<char> target_view = message.target();
    request.raw_target = std::string(target_view.data(), target_view.size());
    if (!parse_target(request.raw_target, target)) {
        error_response = make_error_response(
            400, "invalid_request", "The request target is invalid.", request_id);
        return false;
    }

    const boost::core::basic_string_view<char> method_view = message.method_string();
    request.method = std::string(method_view.data(), method_view.size());
    request.path = target.path;
    request.path_segments = std::move(target.segments);
    request.query = target.query;
    request.request_id = request_id;
    // The parser message is not reused: the connection serves one request and is
    // then closed, so the buffered body can be moved instead of copied.
    request.body = std::move(message.body());
    for (HttpRequestMessage::const_iterator iterator = message.begin(); iterator != message.end(); ++iterator) {
        const boost::core::basic_string_view<char> name_view = iterator->name_string();
        const boost::core::basic_string_view<char> value_view = iterator->value();
        request.headers.emplace(
            lower_ascii(std::string(name_view.data(), name_view.size())),
            std::string(value_view.data(), value_view.size()));
    }

    const std::string content_type = json_media_type(header_value(request, "content-type"));
    if (content_type == "application/json") {
        try {
            request.json_body = std::make_shared<const nlohmann::json>(
                nlohmann::json::parse(request.body));
        } catch (const nlohmann::json::parse_error &) {
            const nlohmann::json details = nlohmann::json::array({
                nlohmann::json{{"reason", "invalid_json"}}
            });
            error_response = make_error_response(
                400,
                "invalid_json",
                "The request body is not valid JSON.",
                request_id,
                &details);
            return false;
        }
    }
    return true;
}

HttpResponse dispatch_request(
    const HttpRequest &request,
    const RequestHandler &private_test_handler,
    const std::string &request_id)
{
    // Production business routes and their authentication arrive in later
    // phases. Until then only the private test routes injected by the test
    // fixture are reachable, so every other path stays denied with 404.
    const bool test_route = private_test_handler && request.path.rfind("/__test__/", 0) == 0;
    if (!test_route)
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);

    try {
        return private_test_handler(request);
    } catch (const std::exception &) {
        return make_error_response(500, "internal_error", "An internal server error occurred.", request_id);
    } catch (...) {
        return make_error_response(500, "internal_error", "An internal server error occurred.", request_id);
    }
}

class HttpSession;

class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
    using CloseCallback = std::function<void(HttpSession *)>;

    HttpSession(
        Tcp::socket socket,
        std::shared_ptr<const ServerConfig> config,
        RequestHandler private_test_handler,
        CloseCallback close_callback)
        : socket_(std::move(socket)),
          config_(std::move(config)),
          private_test_handler_(std::move(private_test_handler)),
          close_callback_(std::move(close_callback)),
          timer_(socket_.get_executor())
    {
    }

    void start()
    {
        read_request();
    }

    void close()
    {
        if (closed_.exchange(true))
            return;
        ErrorCode ignored;
        timer_.cancel(ignored);
        socket_.shutdown(Tcp::socket::shutdown_both, ignored);
        socket_.close(ignored);
        // Move the callback out before invoking it: remove_session() may drop
        // the last reference to this session, and the std::function member must
        // not be destroyed while it is still executing.
        CloseCallback callback = std::move(close_callback_);
        if (callback)
            callback(this);
    }

private:
    // One deadline per socket operation: a client stalling while sending a
    // request or reading a response is closed instead of being queued.
    void arm_timeout()
    {
        timer_.expires_after(config_->request_timeout);
        const std::shared_ptr<HttpSession> self = shared_from_this();
        timer_.async_wait([self](const ErrorCode &error) {
            if (!error)
                self->close();
        });
    }

    void cancel_timeout()
    {
        ErrorCode ignored;
        timer_.cancel(ignored);
    }

    void read_request()
    {
        if (closed_.load())
            return;
        parser_.body_limit(config_->max_request_body_bytes);
        parser_.header_limit(config_->max_header_bytes);
        arm_timeout();
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_read(socket_, buffer_, parser_, [self](const ErrorCode &error, std::size_t bytes) {
            self->handle_read(error, bytes);
        });
    }

    void handle_read(const ErrorCode &error, std::size_t)
    {
        cancel_timeout();
        if (closed_.load())
            return;
        if (error == http::error::end_of_stream) {
            close();
            return;
        }
        if (error == http::error::body_limit) {
            write_response(make_error_response(
                413,
                "request_too_large",
                "The request body exceeds the configured size limit.",
                make_request_id()));
            return;
        }
        if (error == http::error::header_limit) {
            write_response(make_error_response(
                431,
                "request_header_too_large",
                "The request headers exceed the configured size limit.",
                make_request_id()));
            return;
        }
        if (error) {
            write_response(make_error_response(
                400,
                "invalid_request",
                "The HTTP request is invalid.",
                make_request_id()));
            return;
        }

        HttpRequestMessage &message = parser_.get();
        const std::string request_id = make_request_id();
        HttpRequest request;
        HttpResponse response;
        if (!build_request_context(message, request, response, request_id)) {
            write_response(response);
            return;
        }
        response = dispatch_request(request, private_test_handler_, request_id);
        write_response(response);
    }

    void write_response(const HttpResponse &response)
    {
        if (closed_)
            return;

        unsigned int status = response.status;
        if (status < 100 || status > 599)
            status = 500;
        response_message_.emplace(static_cast<http::status>(status), 11);
        response_message_->set(http::field::server, "SuperSlicer REST");
        response_message_->keep_alive(false);
        response_message_->body() = response.no_body ? std::string() : response.body;
        if (!response.no_body && !response.content_type.empty())
            response_message_->set(http::field::content_type, response.content_type);
        response_message_->prepare_payload();
        arm_timeout();
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_write(socket_, response_message_.value(), [self](
            const ErrorCode &error, std::size_t bytes) {
            self->handle_write(error, bytes);
        });
    }

    void handle_write(const ErrorCode &, std::size_t)
    {
        cancel_timeout();
        close();
    }

    Tcp::socket socket_;
    beast::flat_buffer buffer_;
    HttpRequestParser parser_;
    std::shared_ptr<const ServerConfig> config_;
    RequestHandler private_test_handler_;
    CloseCallback close_callback_;
    asio::steady_timer timer_;
    std::optional<HttpResponseMessage> response_message_;
    std::atomic<bool> closed_{false};
};

class ServerRuntime {
public:
    ServerRuntime(ServerConfig config, RequestHandler private_test_handler)
        : config_(std::move(config)),
          config_shared_(std::make_shared<const ServerConfig>(config_)),
          private_test_handler_(std::move(private_test_handler)),
          acceptor_(io_context_)
    {
    }

    ~ServerRuntime()
    {
        stop();
    }

    void start()
    {
        if (running_.load())
            throw std::logic_error("REST server is already running");

        config_.validate();
        std::error_code filesystem_error;
        std::filesystem::create_directories(config_.work_directory, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument(json_error_message(
                "paths.work", filesystem_error.message()));

        ErrorCode error;
        const asio::ip::address address = asio::ip::make_address(config_.bind_address, error);
        if (error)
            throw std::invalid_argument(json_error_message("listen.address", error.message()));

        io_context_.restart();
        acceptor_.open(address.is_v4() ? Tcp::v4() : Tcp::v6(), error);
        if (error)
            throw std::runtime_error("could not open REST listener: " + error.message());
        acceptor_.set_option(Tcp::acceptor::reuse_address(true), error);
        if (error) {
            const std::string message = error.message();
            acceptor_.close(error);
            throw std::runtime_error("could not configure REST listener: " + message);
        }
        acceptor_.bind(Tcp::endpoint(address, config_.port), error);
        if (error) {
            const std::string message = error.message();
            acceptor_.close(error);
            throw std::runtime_error("could not bind REST listener: " + message);
        }
        acceptor_.listen(asio::socket_base::max_listen_connections, error);
        if (error) {
            const std::string message = error.message();
            acceptor_.close(error);
            throw std::runtime_error("could not listen for REST connections: " + message);
        }

        bound_port_ = acceptor_.local_endpoint(error).port();
        if (error) {
            const std::string message = error.message();
            acceptor_.close(error);
            throw std::runtime_error("could not read REST listener port: " + message);
        }
        running_.store(true);
        const std::uint64_t generation = ++run_generation_;
        accept_next(generation);
        worker_ = std::thread([this]() {
            io_context_.run();
        });
    }

    void stop() noexcept
    {
        const bool was_running = running_.exchange(false);
        if (!was_running && !worker_.joinable())
            return;
        if (std::this_thread::get_id() == worker_.get_id())
            std::terminate(); // stop() must not be called from a request handler

        ++run_generation_;
        ErrorCode ignored;
        accept_retry_timer_.cancel(ignored);
        acceptor_.cancel(ignored);
        acceptor_.close(ignored);

        std::vector<std::shared_ptr<HttpSession>> sessions;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            sessions.assign(sessions_.begin(), sessions_.end());
        }
        for (const std::shared_ptr<HttpSession> &session : sessions)
            session->close();

        io_context_.stop();
        if (worker_.joinable())
            worker_.join();

        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.clear();
        bound_port_ = 0;
    }

    bool running() const noexcept
    {
        return running_.load();
    }

    std::uint16_t bound_port() const noexcept
    {
        return bound_port_;
    }

private:
    void accept_next(std::uint64_t generation)
    {
        if (!running_.load() || generation != run_generation_.load())
            return;
        acceptor_.async_accept([this, generation](const ErrorCode &error, Tcp::socket socket) {
            if (!running_.load() || generation != run_generation_.load())
                return;
            if (error) {
                // Retry after a short delay so a persistent accept failure
                // cannot spin the io thread at full speed.
                accept_retry_timer_.expires_after(std::chrono::milliseconds(100));
                accept_retry_timer_.async_wait([this, generation](const ErrorCode &timer_error) {
                    if (!timer_error)
                        accept_next(generation);
                });
                return;
            }

            bool accept_connection = false;
            {
                std::lock_guard<std::mutex> lock(sessions_mutex_);
                if (sessions_.size() < config_.max_connections)
                    accept_connection = true;
            }

            if (accept_connection) {
                const std::shared_ptr<HttpSession> session = std::make_shared<HttpSession>(
                    std::move(socket),
                    config_shared_,
                    private_test_handler_,
                    [this](HttpSession *closed_session) {
                        remove_session(closed_session);
                    });
                {
                    std::lock_guard<std::mutex> lock(sessions_mutex_);
                    sessions_.insert(session);
                }
                session->start();
            } else {
                ErrorCode ignored;
                socket.close(ignored);
            }
            accept_next(generation);
        });
    }

    void remove_session(HttpSession *session)
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (std::set<std::shared_ptr<HttpSession>>::iterator iterator = sessions_.begin();
             iterator != sessions_.end();
             ++iterator) {
            if (iterator->get() == session) {
                sessions_.erase(iterator);
                return;
            }
        }
    }

    ServerConfig config_;
    std::shared_ptr<const ServerConfig> config_shared_;
    RequestHandler private_test_handler_;
    asio::io_context io_context_;
    Tcp::acceptor acceptor_;
    asio::steady_timer accept_retry_timer_{io_context_};
    std::thread worker_;
    std::atomic<bool> running_{false};
    // Bumped on every start/stop so accept handlers and retry timers queued by
    // a previous run cannot reach the restarted acceptor.
    std::atomic<std::uint64_t> run_generation_{0};
    std::uint16_t bound_port_ = 0;
    std::mutex sessions_mutex_;
    std::set<std::shared_ptr<HttpSession>> sessions_;
};

} // namespace

ServerConfig ServerConfig::defaults()
{
    const std::filesystem::path root = std::filesystem::current_path();
    ServerConfig config;
    config.bind_address = "127.0.0.1";
    config.port = 8080;
    config.resource_directory = root / "resources";
    config.profiles_directory = config.resource_directory / "profiles";
    config.work_directory = root / "rest_work";
    config.accounts_file = root / "rest_accounts.json";
    config.max_request_body_bytes = default_max_request_body_bytes;
    config.max_header_bytes = default_max_header_bytes;
    config.max_connections = default_max_connections;
    config.request_timeout = std::chrono::seconds(default_request_timeout_seconds);
    return config;
}

void ServerConfig::validate() const
{
    if (bind_address.empty())
        throw std::invalid_argument("listen.address must not be empty");
    validate_directory(resource_directory, "paths.resources");
    validate_directory(profiles_directory, "paths.profiles");

    std::error_code filesystem_error;
    const bool work_exists = std::filesystem::exists(work_directory, filesystem_error);
    if (filesystem_error)
        throw std::invalid_argument(json_error_message("paths.work", filesystem_error.message()));
    if (work_exists) {
        const bool work_is_directory = std::filesystem::is_directory(work_directory, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument(json_error_message("paths.work", filesystem_error.message()));
        if (!work_is_directory)
            throw std::invalid_argument("paths.work must be a directory");
    } else {
        validate_parent_directory(work_directory, "paths.work");
    }

    if (accounts_file.empty())
        throw std::invalid_argument("paths.accounts must not be empty");
    validate_parent_directory(accounts_file, "paths.accounts");
    const bool accounts_exist = std::filesystem::exists(accounts_file, filesystem_error);
    if (filesystem_error)
        throw std::invalid_argument(json_error_message("paths.accounts", filesystem_error.message()));
    if (accounts_exist) {
        const bool accounts_is_file = std::filesystem::is_regular_file(accounts_file, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument(json_error_message("paths.accounts", filesystem_error.message()));
        if (!accounts_is_file)
            throw std::invalid_argument("paths.accounts must be a regular file");
    }

    if (max_request_body_bytes == 0)
        throw std::invalid_argument("limits.max_request_body_bytes must be positive");
    if (max_header_bytes == 0)
        throw std::invalid_argument("limits.max_header_bytes must be positive");
    if (max_connections == 0)
        throw std::invalid_argument("limits.max_connections must be positive");
    if (request_timeout.count() <= 0)
        throw std::invalid_argument("timeouts.request_seconds must be positive");
}

ServerConfig load_server_config(
    const std::filesystem::path &config_file,
    const CommandLineOverrides &overrides)
{
    ServerConfig config = ServerConfig::defaults();
    if (!config_file.empty()) {
        const std::filesystem::path absolute_config = std::filesystem::absolute(config_file);
        std::ifstream input(absolute_config);
        if (!input)
            throw std::invalid_argument("could not open configuration file: " + absolute_config.string());

        nlohmann::json document;
        try {
            input >> document;
        } catch (const nlohmann::json::parse_error &error) {
            throw std::invalid_argument(std::string("invalid configuration JSON: ") + error.what());
        }
        apply_json_config(config, document, absolute_config.parent_path());
    }
    apply_overrides(config, overrides);
    config.validate();
    return config;
}

HttpResponse HttpResponse::json(unsigned int response_status, const nlohmann::json &payload)
{
    HttpResponse response;
    response.status = response_status;
    response.body = payload.dump();
    response.content_type = "application/json; charset=utf-8";
    response.no_body = false;
    return response;
}

HttpResponse HttpResponse::error(
    unsigned int response_status,
    const std::string &code,
    const std::string &message,
    const std::string &request_id,
    const nlohmann::json *details)
{
    nlohmann::json error_payload = {
        {"code", code},
        {"message", message},
        {"request_id", request_id}
    };
    if (details != nullptr)
        error_payload["details"] = *details;
    return HttpResponse::json(response_status, nlohmann::json{{"error", error_payload}});
}

HttpResponse HttpResponse::empty(unsigned int response_status)
{
    HttpResponse response;
    response.status = response_status;
    response.body.clear();
    response.content_type.clear();
    response.no_body = true;
    return response;
}

class RestServer::Impl {
public:
    Impl(ServerConfig config, RequestHandler private_test_handler)
        : runtime_(std::move(config), std::move(private_test_handler))
    {
    }

    ServerRuntime runtime_;
};

RestServer::RestServer(ServerConfig config, RequestHandler private_test_handler)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(private_test_handler)))
{
}

RestServer::~RestServer() = default;

void RestServer::start()
{
    impl_->runtime_.start();
}

void RestServer::stop() noexcept
{
    impl_->runtime_.stop();
}

bool RestServer::running() const noexcept
{
    return impl_->runtime_.running();
}

std::uint16_t RestServer::bound_port() const noexcept
{
    return impl_->runtime_.bound_port();
}

} // namespace Slic3r::Rest
