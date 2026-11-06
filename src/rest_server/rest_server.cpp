/*
 * REST configuration, HTTP transport and request pipeline.
 *
 * Configuration and accounts are validated before the listener binds. Each
 * connection reads one bounded HTTP request, decodes its path once, and closes
 * after one response. `/login` is public; every other route requires a bearer
 * token before route resolution, then dispatches to logout, the global
 * settings/session routes, or the private test routes injected by the fixture.
 */

#include "rest_server.hpp"

#include "auth.hpp"
#include "file_names.hpp"
#include "multipart_upload.hpp"
#include "profiles.hpp"
#include "session.hpp"
#include "settings.hpp"
#include "temporary_file.hpp"
#include "transform.hpp"
#include "libslic3r/Format/3mf.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
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
using HttpRequestMessage = http::request<http::buffer_body>;
using HttpResponseMessage = http::response<http::string_body>;
using HttpRequestParser = http::request_parser<http::buffer_body>;
// Delivers a response that a route produces later on another thread. It may
// be called from any thread, exactly once.
using DeferredResponder = std::function<void(HttpResponse)>;
// Resolves one parsed request to a response; the transport owns no routing.
// An empty result means the route kept `respond` and will answer later.
using RestRouter = std::function<std::optional<HttpResponse>(
    HttpRequest &,
    const std::string &request_id,
    bool body_complete,
    const DeferredResponder &respond)>;

// Default buffered JSON body limit. The specification's 1 GiB upload limit
// belongs to the streamed multipart route, not to buffered JSON bodies.
constexpr std::uint64_t default_max_request_body_bytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_upload_bytes = 1ULL * 1024ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_header_bytes = 16ULL * 1024ULL;
constexpr std::uint32_t default_max_connections = 64;
constexpr std::uint32_t default_max_instances_per_object = 1000;
constexpr std::uint32_t default_import_threads = 2;
// After an early error response the refused request body is read and dropped
// within these bounds, so closing the socket sends FIN instead of a TCP reset
// that would hide the response from the client.
constexpr std::size_t max_drain_bytes = 1024 * 1024;
constexpr std::chrono::seconds drain_timeout{2};
constexpr std::int64_t default_request_timeout_seconds = 30;
constexpr std::int64_t default_token_ttl_seconds = 24 * 60 * 60;
constexpr std::int64_t default_session_ttl_seconds = 24 * 60 * 60;
constexpr std::int64_t default_login_window_seconds = 60;
constexpr std::uint32_t default_max_login_attempts = 5;

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

// One response for absent, invalid, expired and revoked tokens, so a caller
// cannot tell those cases apart.
HttpResponse make_unauthorized_response(const std::string &request_id)
{
    HttpResponse response = make_error_response(
        401,
        "unauthorized",
        "A valid Bearer token is required.",
        request_id);
    response.headers.emplace("WWW-Authenticate", "Bearer");
    return response;
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
        if (limits.contains("max_upload_bytes"))
            config.max_upload_bytes = read_unsigned_json(
                limits.at("max_upload_bytes"), "limits.max_upload_bytes");
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
        if (limits.contains("import_threads")) {
            const std::uint64_t import_threads = read_unsigned_json(
                limits.at("import_threads"), "limits.import_threads");
            if (import_threads > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("limits.import_threads is too large");
            config.import_threads = static_cast<std::uint32_t>(import_threads);
        }
        if (limits.contains("max_instances_per_object")) {
            const std::uint64_t maximum = read_unsigned_json(
                limits.at("max_instances_per_object"), "limits.max_instances_per_object");
            if (maximum > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("limits.max_instances_per_object is too large");
            config.max_instances_per_object = static_cast<std::uint32_t>(maximum);
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

    if (document.contains("auth")) {
        const nlohmann::json &auth = document.at("auth");
        if (!auth.is_object())
            throw std::invalid_argument("auth must be a JSON object");
        if (auth.contains("token_ttl_seconds")) {
            const std::uint64_t seconds = read_unsigned_json(
                auth.at("token_ttl_seconds"), "auth.token_ttl_seconds");
            if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                throw std::invalid_argument("auth.token_ttl_seconds is too large");
            config.token_ttl = std::chrono::seconds(static_cast<std::int64_t>(seconds));
        }
        if (auth.contains("login_window_seconds")) {
            const std::uint64_t seconds = read_unsigned_json(
                auth.at("login_window_seconds"), "auth.login_window_seconds");
            if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                throw std::invalid_argument("auth.login_window_seconds is too large");
            config.login_window = std::chrono::seconds(static_cast<std::int64_t>(seconds));
        }
        if (auth.contains("max_login_attempts")) {
            const std::uint64_t attempts = read_unsigned_json(
                auth.at("max_login_attempts"), "auth.max_login_attempts");
            if (attempts > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("auth.max_login_attempts is too large");
            config.max_login_attempts = static_cast<std::uint32_t>(attempts);
        }
    }

    if (document.contains("sessions")) {
        const nlohmann::json &sessions = document.at("sessions");
        if (!sessions.is_object())
            throw std::invalid_argument("sessions must be a JSON object");
        if (sessions.contains("expiration_seconds")) {
            const std::uint64_t seconds = read_unsigned_json(
                sessions.at("expiration_seconds"), "sessions.expiration_seconds");
            if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                throw std::invalid_argument("sessions.expiration_seconds is too large");
            config.session_ttl = std::chrono::seconds(static_cast<std::int64_t>(seconds));
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

// Extract the token from a `Bearer <token>` header. The scheme is matched
// case-insensitively and any whitespace after the token is refused.
std::optional<std::string> bearer_token(const HttpRequest &request)
{
    const std::string authorization = header_value(request, "authorization");
    if (authorization.size() < 7 ||
        lower_ascii(authorization.substr(0, 6)) != "bearer" ||
        authorization[6] != ' ')
        return std::nullopt;
    const std::string token = authorization.substr(7);
    if (token.empty() || token.find_first_of(" \t\r\n") != std::string::npos)
        return std::nullopt;
    return token;
}

// Parse the body only when the caller declared a JSON media type; other
// requests keep json_body null and are handled by their route.
bool parse_request_json(
    HttpRequest &request,
    HttpResponse &error_response,
    const std::string &request_id)
{
    if (json_media_type(header_value(request, "content-type")) != "application/json")
        return true;
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
    return true;
}

// Copy method, target and headers out of the parser. The body is supplied by the
// transport after either bounded buffering or multipart streaming completes.
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
    for (HttpRequestMessage::const_iterator iterator = message.begin(); iterator != message.end(); ++iterator) {
        const boost::core::basic_string_view<char> name_view = iterator->name_string();
        const boost::core::basic_string_view<char> value_view = iterator->value();
        request.headers.emplace(
            lower_ascii(std::string(name_view.data(), name_view.size())),
            std::string(value_view.data(), value_view.size()));
    }

    return true;
}

HttpResponse invalid_profile_request(const std::string &message, const std::string &request_id)
{
    // Profile IDs and compatibility failures are client-correctable selections.
    return make_error_response(422, "invalid_profile", message, request_id);
}

bool parse_profile_selection(
    HttpRequest &request,
    ProfileSelection &selection,
    HttpResponse &error_response,
    const std::string &request_id)
{
    // The route requires a complete, typed JSON selection before profile lookup.
    if (!parse_request_json(request, error_response, request_id))
        return false;
    if (request.json_body == nullptr || !request.json_body->is_object()) {
        error_response = make_error_response(
            400,
            "invalid_request",
            "The profile selection body must be a JSON object.",
            request_id);
        return false;
    }

    const nlohmann::json &body = *request.json_body;
    if (!body.contains("print") || !body.at("print").is_string() ||
        !body.contains("printer") || !body.at("printer").is_string() ||
        !body.contains("extruder") || !body.at("extruder").is_array() ||
        !body.contains("filaments") || !body.at("filaments").is_array()) {
        error_response = make_error_response(
            400,
            "invalid_request",
            "The profile selection must contain print, printer, extruder, and filaments.",
            request_id);
        return false;
    }

    selection.print = body.at("print").get<std::string>();
    selection.printer = body.at("printer").get<std::string>();
    for (const nlohmann::json &profile : body.at("extruder")) {
        if (!profile.is_string()) {
            error_response = make_error_response(
                400,
                "invalid_request",
                "The extruder profile list must contain strings.",
                request_id);
            return false;
        }
        selection.extruder.push_back(profile.get<std::string>());
    }
    for (const nlohmann::json &profile : body.at("filaments")) {
        if (!profile.is_string()) {
            error_response = make_error_response(
                400,
                "invalid_request",
                "The filament profile list must contain strings.",
                request_id);
            return false;
        }
        selection.filaments.push_back(profile.get<std::string>());
    }
    return true;
}

nlohmann::json profile_selection_json(const ProfileSelection &selection)
{
    // The catalog has already converted internal preset names to opaque IDs.
    return nlohmann::json{
        {"print", selection.print},
        {"printer", selection.printer},
        {"extruder", selection.extruder},
        {"filaments", selection.filaments}
    };
}

// Normalize setting failures into the route's client-correctable error shape.
HttpResponse invalid_setting_request(const std::string &message, const std::string &request_id)
{
    // Setting IDs, types, ranges and cardinalities are all client-correctable.
    return make_error_response(422, "invalid_setting", message, request_id);
}

// Parse the explicit ID list while preserving the distinction from malformed JSON.
bool parse_setting_ids(
    HttpRequest &request,
    std::vector<std::string> &setting_ids,
    HttpResponse &error_response,
    const std::string &request_id)
{
    if (!parse_request_json(request, error_response, request_id))
        return false;
    if (request.json_body == nullptr || !request.json_body->is_object() ||
        !request.json_body->contains("setting_ids") ||
        !request.json_body->at("setting_ids").is_array() ||
        request.json_body->at("setting_ids").empty()) {
        error_response = make_error_response(
            400,
            "invalid_request",
            "The get_settings body must contain a non-empty setting_ids array.",
            request_id);
        return false;
    }
    for (const nlohmann::json &setting_id : request.json_body->at("setting_ids")) {
        if (!setting_id.is_string()) {
            error_response = make_error_response(
                400,
                "invalid_request",
                "setting_ids must contain strings.",
                request_id);
            return false;
        }
        setting_ids.push_back(setting_id.get<std::string>());
    }
    return true;
}

// Preserve object order-independent updates for one atomic candidate mutation.
bool parse_setting_updates(
    HttpRequest &request,
    std::vector<std::pair<std::string, nlohmann::json>> &updates,
    HttpResponse &error_response,
    const std::string &request_id)
{
    if (!parse_request_json(request, error_response, request_id))
        return false;
    if (request.json_body == nullptr || !request.json_body->is_object()) {
        error_response = make_error_response(
            400,
            "invalid_request",
            "The set_settings body must be a JSON object.",
            request_id);
        return false;
    }
    for (nlohmann::json::const_iterator setting = request.json_body->begin();
         setting != request.json_body->end(); ++setting)
        updates.emplace_back(setting.key(), setting.value());
    return true;
}


// Services shared by every authenticated route for one request.
struct RouteServices {
    AuthService &auth;
    SessionRegistry &sessions;
    const ProfileCatalog *profiles;
    const RequestHandler &private_test_handler;
    // Worker threads for model parsing; jobs never touch the registry, only
    // the session they captured, so they may outlive the request dispatch.
    asio::thread_pool &imports;
    const std::function<void()> &import_observer;
    std::uint32_t max_instances_per_object;
    const std::filesystem::path &export_directory;
};

struct AuthenticatedRequest {
    std::string token;
    std::string username;
    AuthTimePoint now;
};

std::optional<AuthenticatedRequest> authenticate_request(
    const HttpRequest &request,
    const RouteServices &services,
    HttpResponse &error_response,
    const std::string &request_id)
{
    const std::optional<std::string> token = bearer_token(request);
    if (!token.has_value()) {
        error_response = make_unauthorized_response(request_id);
        return std::nullopt;
    }
    const AuthTimePoint now = std::chrono::system_clock::now();
    const std::optional<std::string> username = services.auth.username_for_token(*token, now);
    if (!username.has_value()) {
        error_response = make_unauthorized_response(request_id);
        return std::nullopt;
    }
    return AuthenticatedRequest{*token, *username, now};
}

// Routes whose body is a streamed multipart file instead of buffered JSON.
bool is_upload_route(const HttpRequest &request)
{
    return request.path_segments.size() == 3 &&
        request.path_segments[0] == "session" &&
        (request.path_segments[2] == "add_model" || request.path_segments[2] == "load_project");
}

// File formats accepted by an upload route: any model for add_model, only a
// 3MF project for load_project.
std::vector<std::string> upload_extensions(const HttpRequest &request)
{
    if (request.path_segments[2] == "load_project")
        return {".3mf"};
    return {".stl", ".obj", ".3mf"};
}


// Public login route. An unknown account and a wrong password return the same
// response, and the local password copy is wiped before returning.
HttpResponse handle_login(HttpRequest &request, const RouteServices &services, const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
HttpResponse parse_error;
if (!parse_request_json(request, parse_error, request_id))
    return parse_error;
if (request.json_body == nullptr || !request.json_body->is_object() ||
    !request.json_body->contains("username") ||
    !request.json_body->contains("password") ||
    !request.json_body->at("username").is_string() ||
    !request.json_body->at("password").is_string())
    return make_error_response(
        400,
        "invalid_request",
        "The login body must contain string username and password fields.",
        request_id);

const std::string username = request.json_body->at("username").get<std::string>();
std::string password = request.json_body->at("password").get<std::string>();
const AuthTimePoint now = std::chrono::system_clock::now();
const LoginResult result = services.auth.login(username, password, now);
clear_secret(password);
if (result.status == LoginStatus::rate_limited) {
    HttpResponse response = make_error_response(
        429,
        "login_rate_limited",
        "Too many login attempts.",
        request_id);
    response.headers.emplace(
        "Retry-After",
        std::to_string(services.auth.login_window().count()));
    return response;
}
// An unknown account and a wrong password return the same response.
if (result.status == LoginStatus::invalid_credentials)
    return make_error_response(
        401,
        "invalid_credentials",
        "The username or password is incorrect.",
        request_id);

const IssuedToken &token = *result.token;
const std::int64_t expires_in = std::chrono::duration_cast<std::chrono::seconds>(
    token.expires_at - now).count();
HttpResponse response = HttpResponse::json(200, nlohmann::json{
    {"access_token", token.value},
    {"token_type", "Bearer"},
    {"expires_in", expires_in}
});
response.headers.emplace("Cache-Control", "no-store");
response.headers.emplace("Pragma", "no-cache");
return response;
}

// Revoke only the bearer token that authenticated this request.
HttpResponse handle_logout(
    const HttpRequest &request,
    const RouteServices &services,
    const std::string &token,
    AuthTimePoint now,
    const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!services.auth.revoke_token(token, now))
    return make_unauthorized_response(request_id);
return HttpResponse::empty(204);
}

// Public catalog of finalized setting definitions.
HttpResponse handle_list_settings(const HttpRequest &request, const std::string &request_id)
{
if (request.method != "GET")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
if (!settings_catalog_ready())
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);
return HttpResponse::json(200, settings_catalog());
}

// Read the body of create_session or set_name: a JSON object whose only field
// is `name`, a string or null for no name. `required` makes the field
// mandatory. Returns the error response of an invalid body.
std::optional<HttpResponse> read_session_name(
    HttpRequest &request,
    bool required,
    std::optional<std::string> &name,
    const std::string &request_id)
{
    HttpResponse parse_error;
    if (!parse_request_json(request, parse_error, request_id))
        return parse_error;
    if (request.json_body == nullptr || !request.json_body->is_object())
        return make_error_response(400, "invalid_request", "The body must be a JSON object.", request_id);
    const nlohmann::json &body = *request.json_body;
    for (const auto &item : body.items())
        if (item.key() != "name")
            return make_error_response(400, "invalid_request", "Unknown field: " + item.key(), request_id);
    name.reset();
    if (!body.contains("name")) {
        if (required)
            return make_error_response(400, "invalid_request", "The body must contain the name field.", request_id);
        return std::nullopt;
    }
    const nlohmann::json &value = body.at("name");
    if (value.is_null())
        return std::nullopt;
    if (!value.is_string())
        return make_error_response(400, "invalid_request", "The name must be a string or null.", request_id);
    try {
        validate_session_name(value.get<std::string>());
    } catch (const std::invalid_argument &error) {
        return make_error_response(400, "invalid_request", error.what(), request_id);
    }
    name = value.get<std::string>();
    return std::nullopt;
}

nlohmann::json session_name_json(const std::optional<std::string> &name)
{
    return name ? nlohmann::json(name.value()) : nlohmann::json(nullptr);
}

// Create an owned session, seeded from the profile catalog defaults when the
// engine catalog is available. The body is empty or carries an optional name.
HttpResponse handle_create_session(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
std::optional<std::string> name;
if (!request.body.empty())
    if (std::optional<HttpResponse> error = read_session_name(request, false, name, request_id))
        return std::move(error.value());
SessionData session_data{std::make_shared<Model>(), {}, nlohmann::json::object()};
session_data.name = name;
if (services.profiles != nullptr) {
    // Defaults are built before the session is published; a broken
    // server-side catalog becomes an internal error, not a worker crash.
    ProfileApplication initial;
    try {
        initial = services.profiles->defaults();
    } catch (const std::exception &) {
        return make_error_response(
            500,
            "internal_error",
            "The server profile catalog could not provide defaults.",
            request_id);
    } catch (...) {
        return make_error_response(
            500,
            "internal_error",
            "The server profile catalog could not provide defaults.",
            request_id);
    }
    session_data.configuration = initial.configuration;
    session_data.profile_sections = initial.profile_sections;
    session_data.profiles = initial.selection;
}
const std::shared_ptr<SessionContext> session = services.sessions.create(
    username,
    SessionRegistry::Clock::now(),
    std::move(session_data));
return HttpResponse::json(201, nlohmann::json{
    {"session_id", session->session_id()}, {"name", session_name_json(name)}});
}

// List the installed profiles visible to one session. The shared catalog is
// only read here.
HttpResponse handle_list_profiles(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    const std::string &session_id = request.path_segments[1];
if (request.method != "GET")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
if (services.sessions.find(username, session_id, SessionRegistry::Clock::now()) == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (services.profiles == nullptr)
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);
return HttpResponse::json(200, services.profiles->list_profiles());
}

// Apply a full profile selection on a candidate configuration outside the
// session lock, then commit it under the session revision.
HttpResponse handle_set_profile(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    const std::string &session_id = request.path_segments[1];
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(
    username,
    session_id,
    SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (services.profiles == nullptr)
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);

ProfileSelection requested;
HttpResponse parse_error;
if (!parse_profile_selection(request, requested, parse_error, request_id))
    return parse_error;

const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
if (!mutation.has_value())
    return make_error_response(
        409,
        "session_busy",
        "The session is currently slicing.",
        request_id);

ProfileApplication application;
try {
    application = services.profiles->select(requested);
} catch (const std::invalid_argument &error) {
    return invalid_profile_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server profile catalog could not apply the selection.",
        request_id);
} catch (...) {
    return make_error_response(
        500,
        "internal_error",
        "The server profile catalog could not apply the selection.",
        request_id);
}

const std::optional<SessionData> current = session->data_snapshot();
if (!current.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
const nlohmann::json response_body = profile_selection_json(application.selection);
SessionData candidate = current.value();
candidate.configuration = std::move(application.configuration);
candidate.profile_sections = std::move(application.profile_sections);
candidate.profiles = std::move(application.selection);
if (!session->commit_mutation(
        mutation.value(),
        std::move(candidate),
        true,
        SessionRegistry::Clock::now()))
    return make_error_response(
        409,
        "session_changed",
        "The session changed while the profile selection was prepared.",
        request_id);
return HttpResponse::json(200, response_body);
}

// Read-only listing of the session scene. The session lock only covers the
// capture of the scene pointer and IDs; the JSON is built and sent without it.
HttpResponse handle_list_objects(
    const HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The list_objects route does not accept a request body.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    try {
        const std::optional<nlohmann::json> objects = session->list_objects();
        if (!objects.has_value())
            return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
        return HttpResponse::json(200, objects.value());
    } catch (const std::exception &) {
        return make_error_response(500, "internal_error", "The server could not read the scene.", request_id);
    }
}

// All result matrices are validated before editing the private scene; the revision guards publication.
HttpResponse handle_transform(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    // The request itself is validated before the session state, so an invalid
    // body gets the same answer whether or not the session is slicing.
    HttpResponse parse_error;
    if (!request.body.empty() && !parse_request_json(request, parse_error, request_id))
        return parse_error;
    if (request.json_body == nullptr)
        return make_error_response(400, "invalid_request", "The transform body must be valid JSON.", request_id);
    TransformRequest transform;
    try {
        transform = parse_transform_request(*request.json_body);
    } catch (const TransformError &error) {
        return make_error_response(error.status(), error.status() == 400 ? "invalid_request" : "invalid_transform", error.what(), request_id);
    }
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
    if (!entity)
        return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the transform was prepared.", request_id);
    try {
        if (transform.instance && entity->kind != EntityKind::volume)
            throw TransformError(422, "The instance field is only valid for a volume target.");
        SessionData candidate = current.value();
        candidate.scene = std::make_shared<Model>(*current->scene);
        std::vector<ModelInstance *> targets;
        ModelVolume *transformed_volume = nullptr;
        if (entity->kind == EntityKind::volume) {
            const std::pair<ModelObject *, ModelVolume *> target = find_transform_volume(*candidate.scene, entity.value());
            ModelObject *object = target.first;
            transformed_volume = target.second;
            ModelInstance *reference = object->instances.at(0);
            if (transform.instance) {
                const std::optional<EntityReference> reference_entity = session->resolve_entity(transform.instance.value());
                if (session->snapshot().revision != mutation->revision)
                    return make_error_response(409, "session_changed", "The session changed while the transform was prepared.", request_id);
                if (!reference_entity)
                    return make_error_response(404, "entity_not_found", "The reference instance does not exist.", request_id);
                if (reference_entity->kind != EntityKind::instance)
                    throw TransformError(422, "The instance field must identify an instance of the volume's object.");
                reference = find_transform_instances(*candidate.scene, reference_entity.value()).at(0);
                if (reference->get_object() != object)
                    throw TransformError(422, "The reference instance belongs to another object.");
            }
            const Transform3d matrix = volume_matrix(transformed_volume->get_matrix(), reference->get_matrix(), transform);
            for (const ModelInstance *instance : object->instances)
                validate_result(instance->get_matrix() * matrix);
            transformed_volume->set_transformation(matrix);
            targets = object->instances;
        } else {
            targets = find_transform_instances(*candidate.scene, entity.value());
            std::vector<Transform3d> matrices;
            matrices.reserve(targets.size());
            for (const ModelInstance *instance : targets)
                matrices.push_back(instance_matrix(instance->get_matrix(), transform));
            for (std::size_t index = 0; index < targets.size(); ++index)
                targets[index]->set_transformation(Geometry::Transformation(matrices[index]));
        }
        for (ModelObject *object : candidate.scene->object_ptrs()) {
            for (std::size_t index = 0; index < object->instances.size(); ++index) {
                ModelInstance *instance = object->instances[index];
                if (std::find(targets.begin(), targets.end(), instance) == targets.end())
                    continue;
                if (transform.ensure_on_bed) {
                    const double min_z = object->get_instance_min_z(index);
                    if (!std::isfinite(min_z) || min_z == std::numeric_limits<double>::max())
                        throw TransformError(422, "The instance has no valid model geometry for bed placement.");
                    instance->set_offset(Z, instance->get_offset(Z) - min_z);
                    if (transformed_volume != nullptr)
                        validate_result(instance->get_matrix() * transformed_volume->get_matrix());
                    else
                        validate_result(instance->get_matrix());
                }
                object->invalidate_bounding_box();
            }
        }
        const HttpResponse response = HttpResponse::json(200, nlohmann::json{
            {"id", request.path_segments[3]}, {"action", transform.action}, {"space", transform.space},
            {"matrix", request.json_body->at("matrix")}
        });
        if (!session->commit_mutation(mutation.value(), std::move(candidate), true, SessionRegistry::Clock::now()))
            return make_error_response(409, "session_changed", "The session changed while the transform was prepared.", request_id);
        return response;
    } catch (const TransformError &error) {
        return make_error_response(error.status(), error.status() == 400 ? "invalid_request" : "invalid_transform", error.what(), request_id);
    } catch (const std::exception &) {
        return make_error_response(500, "internal_error", "The server could not transform the scene.", request_id);
    }
}

// Instance removals and additions share the scene commit; a no-op only reads the current IDs.
HttpResponse handle_set_instance_count(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    // The request is validated before the session state (see handle_transform).
    HttpResponse parse_error;
    if (!request.body.empty() && !parse_request_json(request, parse_error, request_id))
        return parse_error;
    if (request.json_body == nullptr || !request.json_body->is_object() || request.json_body->size() != 1 ||
        !request.json_body->contains("count") || !request.json_body->at("count").is_number_integer())
        return make_error_response(400, "invalid_request", "The body must contain only the integer field count.", request_id);
    const nlohmann::json &value = request.json_body->at("count");
    if (!value.is_number_unsigned() && value.get<std::int64_t>() < 1)
        return make_error_response(422, "invalid_instance_count", "The instance count must be positive.", request_id);
    const std::uint64_t count = value.get<std::uint64_t>();
    if (count < 1 || count > services.max_instances_per_object)
        return make_error_response(422, "invalid_instance_count", "The instance count exceeds the permitted range.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the count was prepared.", request_id);
    if (!entity)
        return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
    if (entity->kind != EntityKind::object)
        return make_error_response(422, "invalid_instance_count", "The target must be an object.", request_id);
    try {
        std::optional<std::vector<std::string>> ids = session->instance_ids(mutation.value(), entity->engine_id);
        if (!ids)
            return make_error_response(409, "session_changed", "The session changed while the count was prepared.", request_id);
        if (count == ids->size())
            return HttpResponse::json(200, nlohmann::json{
                {"object_id", request.path_segments[3]}, {"count", count}, {"instance_ids", ids.value()}});
        SessionData candidate = current.value();
        candidate.scene = std::make_shared<Model>(*current->scene);
        ModelObject *object = find_transform_instances(*candidate.scene, entity.value()).at(0)->get_object();
        std::vector<EntityBinding> bindings;
        std::vector<EntityBinding> unbindings;
        ids->reserve(static_cast<std::size_t>(count));
        while (object->instances.size() < count) {
            const ModelInstance *instance = object->add_instance(*object->instances.at(0));
            bindings.push_back({EntityKind::instance, instance->id().id});
        }
        while (object->instances.size() > count) {
            unbindings.push_back({EntityKind::instance, object->instances.back()->id().id});
            object->delete_last_instance();
        }
        if (ids->size() > count)
            ids->resize(static_cast<std::size_t>(count));
        object->invalidate_bounding_box();
        std::vector<std::string> added_ids;
        if (!session->commit_mutation(mutation.value(), std::move(candidate), true,
                SessionRegistry::Clock::now(), bindings, added_ids, false, unbindings))
            return make_error_response(409, "session_changed", "The session changed while the count was prepared.", request_id);
        ids->insert(ids->end(), added_ids.begin(), added_ids.end());
        return HttpResponse::json(200, nlohmann::json{
            {"object_id", request.path_segments[3]}, {"count", count}, {"instance_ids", ids.value()}});
    } catch (const std::exception &) {
        return make_error_response(500, "internal_error", "The server could not change the instance count.", request_id);
    }
}

// Remove every object of the scene while keeping configuration, profiles and
// preferences. The empty scene and the removal of every entity ID are one
// commit, so no ID can resolve to a removed entity. Clearing an already empty
// scene changes nothing and does not commit: the result state (including a
// `failed` status) is left as it is. The body is checked before the slicing
// state so an invalid request always gets the same answer.
HttpResponse handle_clear_platter(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The clear_platter route does not accept a request body.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::size_t cleared = current->scene == nullptr ? 0 : current->scene->objects().size();
    if (cleared == 0)
        return HttpResponse::json(200, nlohmann::json{{"cleared_object_count", 0}});

    SessionData candidate = current.value();
    candidate.scene = std::make_shared<Model>();
    std::vector<std::string> bound_ids;
    if (!session->commit_mutation(mutation.value(), std::move(candidate), true,
            SessionRegistry::Clock::now(), {}, bound_ids, true))
        return make_error_response(409, "session_changed", "The session changed while the platter was cleared.", request_id);
    return HttpResponse::json(200, nlohmann::json{{"cleared_object_count", cleared}});
}

// Rename the session, or remove its name with null. The name never affects
// slicing: the result state is kept, and an unchanged name commits nothing.
// Like every mutation it is refused while the session is slicing, after the
// body is validated.
HttpResponse handle_set_name(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    std::optional<std::string> name;
    if (std::optional<HttpResponse> error = read_session_name(request, true, name, request_id))
        return std::move(error.value());
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (current->name == name)
        return HttpResponse::json(200, nlohmann::json{{"name", session_name_json(name)}});
    SessionData candidate = current.value();
    candidate.name = name;
    if (!session->commit_mutation(mutation.value(), std::move(candidate), false, SessionRegistry::Clock::now()))
        return make_error_response(409, "session_changed", "The session changed while it was renamed.", request_id);
    return HttpResponse::json(200, nlohmann::json{{"name", session_name_json(name)}});
}

// Read the session name; null when the session has none.
HttpResponse handle_get_name(
    const HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The get_name route does not accept a request body.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    return HttpResponse::json(200, nlohmann::json{{"name", session_name_json(current->name)}});
}

// Read one setting from an owned session without mutating it.
HttpResponse handle_get_setting(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
if (request.method != "GET")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
if (!request.body.empty())
    return make_error_response(
        400,
        "invalid_request",
        "The get_setting route does not accept a request body.",
        request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(
    username,
    request.path_segments[1],
    SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (!settings_catalog_ready())
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);
const std::optional<SessionData> data = session->data_snapshot();
if (!data.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
try {
    return HttpResponse::json(200, nlohmann::json{
        {"setting_id", request.path_segments[3]},
        {"serialized_value", setting_value_serialized(
            data.value().configuration, request.path_segments[3])}
    });
} catch (const std::invalid_argument &error) {
    return invalid_setting_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not read the setting.",
        request_id);
}
}

// Read several settings from one snapshot; the response is completed before it
// is sent.
HttpResponse handle_get_settings(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(
    username,
    request.path_segments[1],
    SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (!settings_catalog_ready())
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);
std::vector<std::string> setting_ids;
HttpResponse parse_error;
if (!parse_setting_ids(request, setting_ids, parse_error, request_id))
    return parse_error;
const std::optional<SessionData> data = session->data_snapshot();
if (!data.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
try {
    nlohmann::json values = nlohmann::json::object();
    for (const std::string &setting_id : setting_ids)
        values[setting_id] = setting_value_json(data.value().configuration, setting_id);
    return HttpResponse::json(200, values);
} catch (const std::invalid_argument &error) {
    return invalid_setting_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not read the settings.",
        request_id);
}
}

// Replace one setting through its serialized SuperSlicer value on a candidate
// session, then commit it under the session revision.
HttpResponse handle_set_setting(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
if (!request.body.empty())
    return make_error_response(
        400,
        "invalid_request",
        "The set_setting route does not accept a request body.",
        request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(
    username,
    request.path_segments[1],
    SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (!settings_catalog_ready())
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);

const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
if (!mutation.has_value())
    return make_error_response(
        409,
        "session_busy",
        "The session is currently slicing.",
        request_id);
const std::optional<SessionData> current = session->data_snapshot();
if (!current.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);

SessionData candidate = current.value();
std::string serialized_response;
try {
    set_setting_serialized(
        candidate.configuration,
        request.path_segments[3],
        request.path_segments[4]);
    validate_setting_configuration(current->configuration, candidate.configuration);
    serialized_response = setting_value_serialized(
        candidate.configuration,
        request.path_segments[3]);
} catch (const std::invalid_argument &error) {
    return invalid_setting_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not apply the setting.",
        request_id);
}
if (!session->commit_mutation(
        mutation.value(),
        std::move(candidate),
        setting_affects_slicing(request.path_segments[3]),
        SessionRegistry::Clock::now()))
    return make_error_response(
        409,
        "session_changed",
        "The session changed while the setting was prepared.",
        request_id);
try {
    return HttpResponse::json(200, nlohmann::json{
        {"setting_id", request.path_segments[3]},
        {"serialized_value", serialized_response}
    });
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not serialize the setting response.",
        request_id);
}
}

// Apply several typed setting updates atomically on a candidate session.
HttpResponse handle_set_settings(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
if (request.method != "POST")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(
    username,
    request.path_segments[1],
    SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (!settings_catalog_ready())
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);

std::vector<std::pair<std::string, nlohmann::json>> updates;
HttpResponse parse_error;
if (!parse_setting_updates(request, updates, parse_error, request_id))
    return parse_error;
if (updates.empty())
    return HttpResponse::json(200, nlohmann::json{{"updated", nlohmann::json::object()}});

const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
if (!mutation.has_value())
    return make_error_response(
        409,
        "session_busy",
        "The session is currently slicing.",
        request_id);
const std::optional<SessionData> current = session->data_snapshot();
if (!current.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);

SessionData candidate = current.value();
bool affects_slicing = false;
nlohmann::json response_values = nlohmann::json::object();
try {
    for (const std::pair<std::string, nlohmann::json> &update : updates) {
        const std::string serialized = setting_json_to_serialized(
            candidate.configuration, update.first, update.second);
        set_setting_serialized(candidate.configuration, update.first, serialized);
        affects_slicing = affects_slicing || setting_affects_slicing(update.first);
    }
    validate_setting_configuration(current->configuration, candidate.configuration);
    for (const std::pair<std::string, nlohmann::json> &update : updates)
        response_values[update.first] = setting_value_json(candidate.configuration, update.first);
} catch (const std::invalid_argument &error) {
    return invalid_setting_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not apply the settings.",
        request_id);
}
if (!session->commit_mutation(
        mutation.value(),
        std::move(candidate),
        affects_slicing,
        SessionRegistry::Clock::now()))
    return make_error_response(
        409,
        "session_changed",
        "The session changed while the settings were prepared.",
        request_id);

try {
    return HttpResponse::json(200, nlohmann::json{{"updated", response_values}});
} catch (const std::invalid_argument &error) {
    return invalid_setting_request(error.what(), request_id);
} catch (const std::exception &) {
    return make_error_response(
        500,
        "internal_error",
        "The server could not serialize the settings response.",
        request_id);
}
}

// Reserve an upload only after authentication and session ownership have been
// checked, so an unauthenticated client cannot make the server write a file.
HttpResponse handle_upload_preflight(
    const HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (request.path_segments[2] == "load_project" && services.profiles == nullptr)
        return make_error_response(
            503,
            "engine_not_initialized",
            "The REST engine is not initialized.",
            request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username,
        request.path_segments[1],
        SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (!session->begin_mutation().has_value())
        return make_error_response(
            409,
            "session_busy",
            "The session is currently slicing.",
            request_id);

    HttpResponse response;
    response.stream_request = true;
    return response;
}

// Replace every occurrence of `from` in `text`; used to keep server paths out
// of error messages returned to clients.
void replace_all(std::string &text, const std::string &from, const std::string &to)
{
    if (from.empty())
        return;
    for (std::size_t position = text.find(from); position != std::string::npos;
         position = text.find(from, position + to.size()))
        text.replace(position, from.size(), to);
}

// Map a name the loaders derived from the server temporary file to the client
// file name. STL/OBJ loaders use the temporary file name itself; the 3MF
// loader falls back to "<temporary stem>_N" for unnamed objects, which
// becomes "<client stem>_N". Other names come from the file and are kept.
std::string client_visible_name(const std::string &name, const HttpUpload &upload)
{
    const std::string temporary_name = upload.temporary_path.filename().string();
    if (name == temporary_name)
        return upload.filename;
    const std::string temporary_stem = upload.temporary_path.stem().string();
    if (!temporary_stem.empty() && name.compare(0, temporary_stem.size(), temporary_stem) == 0) {
        // String split rather than std::filesystem: the client name is UTF-8.
        const std::size_t dot = upload.filename.find_last_of('.');
        const std::string client_stem = dot == std::string::npos ? upload.filename : upload.filename.substr(0, dot);
        return client_stem + name.substr(temporary_stem.size());
    }
    return name;
}

// Remove every trace of the server temporary path from an imported object:
// object and volume names, and their source file references. Clients, the
// object listing and saved projects must only see the client file name.
void hide_upload_path(ModelObject &object, const HttpUpload &upload)
{
    const std::string source_path = upload.temporary_path.string();
    object.name = object.name.empty() ? upload.filename : client_visible_name(object.name, upload);
    if (object.input_file == source_path)
        object.input_file = upload.filename;
    for (ModelVolume *volume : object.volumes) {
        volume->name = client_visible_name(volume->name, upload);
        if (volume->source.input_file == source_path)
            volume->source.input_file = upload.filename;
    }
}

// Runs on an import thread. Parse the upload, add its objects to a private
// scene copy, and commit the scene together with the new entity IDs, so a
// failed import can neither leak mappings nor publish a half-built scene.
// Loader failures (std::runtime_error, including Slic3r::RuntimeError) are
// client errors; anything else is an internal error with a generic message.
// Only objects are imported: model-level 3MF data such as the project
// configuration or per-height custom G-code is discarded.
HttpResponse import_model(
    SessionContext &session,
    SessionContext::MutationToken mutation,
    const SessionData &current,
    const HttpUpload &upload,
    const std::string &request_id)
{
    const std::string source_path = upload.temporary_path.string();
    const std::string temporary_name = upload.temporary_path.filename().string();
    try {
        Model imported = Model::read_from_file(
            source_path,
            nullptr,
            nullptr,
            Model::LoadAttribute::AddDefaultInstances);
        const std::shared_ptr<Model> candidate_scene = std::make_shared<Model>(*current.scene);
        std::vector<std::size_t> object_binding_indices;
        std::vector<EntityBinding> bindings;

        for (ModelObject *imported_object : imported.object_ptrs()) {
            ModelObject *added_object = candidate_scene->add_object(*imported_object);
            hide_upload_path(*added_object, upload);

            object_binding_indices.push_back(bindings.size());
            bindings.push_back({EntityKind::object, static_cast<std::uint64_t>(added_object->id().id)});
            for (ModelVolume *volume : added_object->volumes)
                bindings.push_back({EntityKind::volume, static_cast<std::uint64_t>(volume->id().id)});
            for (ModelInstance *instance : added_object->instances)
                bindings.push_back({EntityKind::instance, static_cast<std::uint64_t>(instance->id().id)});
        }

        SessionData candidate = current;
        candidate.scene = candidate_scene;
        std::vector<std::string> bound_ids;
        if (!session.commit_mutation(
                mutation,
                std::move(candidate),
                true,
                SessionRegistry::Clock::now(),
                bindings,
                bound_ids))
            return make_error_response(
                409,
                "session_changed",
                "The session changed while the model was imported.",
                request_id);

        nlohmann::json object_ids = nlohmann::json::array();
        for (std::size_t binding_index : object_binding_indices)
            object_ids.push_back(bound_ids.at(binding_index));
        return HttpResponse::json(200, nlohmann::json{
            {"object_ids", object_ids},
            {"object_count", object_ids.size()}
        });
    } catch (const std::runtime_error &error) {
        std::string message = error.what();
        replace_all(message, source_path, upload.filename);
        replace_all(message, temporary_name, upload.filename);
        return make_error_response(422, "invalid_model", message, request_id);
    } catch (...) {
        return make_error_response(
            500,
            "internal_error",
            "The server could not import the model.",
            request_id);
    }
}

// Validate the completed upload on the network thread, then hand the parsing
// to the import pool. The job owns everything it uses (session, snapshot,
// temporary file), and answers through `respond` once the import is done.
std::optional<HttpResponse> handle_add_model(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id,
    const DeferredResponder &respond)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (request.upload == nullptr)
        return make_error_response(
            400,
            "invalid_multipart",
            "The request must contain one uploaded file.",
            request_id);

    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username,
        request.path_segments[1],
        SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation.has_value())
        return make_error_response(
            409,
            "session_busy",
            "The session is currently slicing.",
            request_id);
    std::optional<SessionData> current = session->data_snapshot();
    if (!current.has_value())
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);

    asio::post(services.imports, [
        session,
        token = mutation.value(),
        data = std::move(current.value()),
        upload = request.upload,
        request_id,
        respond,
        observer = services.import_observer]() {
        if (observer)
            observer();
        respond(import_model(*session, token, data, *upload, request_id));
    });
    return std::nullopt;
}

// Name the first integer extruder option ("extruder" or "*_extruder") whose
// 1-based index exceeds the extruders of the project; 0 means "default".
std::optional<std::string> extruder_out_of_range(
    const DynamicPrintConfig &config,
    std::size_t extruder_count,
    const std::string &owner)
{
    for (const std::string &key : config.keys()) {
        if (key != "extruder" && !boost::ends_with(key, "_extruder"))
            continue;
        const ConfigOptionInt *option = dynamic_cast<const ConfigOptionInt *>(config.option(key));
        if (option != nullptr && option->value > 0 && static_cast<std::size_t>(option->value) > extruder_count)
            return owner + " uses extruder " + std::to_string(option->value) + " in " + key +
                ", but the project defines " + std::to_string(extruder_count) + " extruder(s).";
    }
    return std::nullopt;
}

// Check the project configuration, every object, volume and layer range
// against the extruder count of the project. It must run on the configuration
// as loaded: the profile recomposition clamps the global *_extruder options
// to the extruder count and would hide an invalid index.
std::optional<std::string> project_extruder_error(
    const Model &scene,
    const DynamicPrintConfig &configuration,
    std::size_t count)
{
    std::optional<std::string> error = extruder_out_of_range(configuration, count, "The project configuration");
    for (const ModelObject *object : scene.object_ptrs()) {
        const std::string owner = "Object \"" + object->name + "\"";
        if (!error)
            error = extruder_out_of_range(object->config.get(), count, owner);
        for (const ModelVolume *volume : object->volumes)
            if (!error)
                error = extruder_out_of_range(volume->config.get(), count, owner);
        for (const std::pair<const t_layer_height_range, ModelConfig> &range : object->layer_config_ranges)
            if (!error)
                error = extruder_out_of_range(range.second.get(), count, owner);
    }
    return error;
}

// Runs on an import thread. Load a 3MF project into a new scene and a
// configuration recomposed from its embedded profiles, then replace the
// session scene, configuration and profile selection in one commit that also
// drops every previous entity ID. Session preferences are kept; the session
// takes the project file name. Loader and
// configuration errors are client errors (422 invalid_project); the session
// is left untouched by any refusal.
HttpResponse import_project(
    SessionContext &session,
    SessionContext::MutationToken mutation,
    const SessionData &current,
    const HttpUpload &upload,
    const ProfileCatalog &profiles,
    const std::string &request_id)
{
    const std::string source_path = upload.temporary_path.string();
    const std::string temporary_name = upload.temporary_path.filename().string();
    const auto invalid_project = [&request_id](const std::string &message) {
        return make_error_response(422, "invalid_project", message, request_id);
    };
    try {
        // The substitution context collects the keys the engine does not
        // know, so they can be preserved as custom settings. Only the Enable
        // rule records the unknown keys that the legacy handling drops.
        DynamicPrintConfig loaded;
        ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
        std::string profile_sections;
        Model scene = Model::read_from_archive(
            source_path,
            &loaded,
            &substitutions,
            Model::LoadAttribute::AddDefaultInstances,
            &profile_sections);
        // Unlike the archive reader, the project route requires a scene, as the generic model reader does.
        if (scene.object_ptrs().empty())
            return invalid_project("The supplied file couldn't be read because it's empty");
        substitutions.sort_and_remove_duplicates();
        if (Preset::printer_technology(loaded) != ptFFF)
            return invalid_project("The project is not an FFF project.");

        // A project replaces every slicing category: keys it does not contain
        // start from the system defaults, as for a profile load.
        DynamicPrintConfig project;
        project.apply(FullPrintConfig::defaults());
        project.apply(loaded, true);

        // The extruder count drives every per-extruder vector: an empty nozzle
        // list would make the normalization and the preset split below index
        // empty vectors. Extruder indices are checked before they are clamped.
        const ConfigOptionFloats *nozzle_diameter = project.option<ConfigOptionFloats>("nozzle_diameter");
        if (nozzle_diameter == nullptr || nozzle_diameter->empty())
            return invalid_project("The project defines no extruder.");
        if (const std::optional<std::string> error = project_extruder_error(scene, project, nozzle_diameter->size()))
            return invalid_project(error.value());

        Preset::normalize(project);
        preserve_unknown_project_settings(project, substitutions.get());
        validate_preset_custom_settings(project);
        ProfileApplication application = profiles.load_project(std::move(project), upload.filename, profile_sections);

        const std::string validation_error = DynamicPrintConfig(application.configuration).validate();
        if (!validation_error.empty())
            return invalid_project("The project configuration is invalid: " + validation_error);

        std::vector<std::size_t> object_binding_indices;
        std::vector<EntityBinding> bindings;
        for (ModelObject *object : scene.object_ptrs()) {
            hide_upload_path(*object, upload);
            object_binding_indices.push_back(bindings.size());
            bindings.push_back({EntityKind::object, static_cast<std::uint64_t>(object->id().id)});
            for (ModelVolume *volume : object->volumes)
                bindings.push_back({EntityKind::volume, static_cast<std::uint64_t>(volume->id().id)});
            for (ModelInstance *instance : object->instances)
                bindings.push_back({EntityKind::instance, static_cast<std::uint64_t>(instance->id().id)});
        }

        SessionData candidate;
        candidate.scene = std::make_shared<Model>(std::move(scene));
        candidate.configuration = std::move(application.configuration);
        candidate.profile_sections = std::move(application.profile_sections);
        candidate.preferences = current.preferences;
        candidate.profiles = std::move(application.selection);
        // As in the GUI, the loaded project names the session.
        candidate.name = session_name_from_file(upload.filename);
        std::vector<std::string> bound_ids;
        if (!session.commit_mutation(
                mutation,
                std::move(candidate),
                true,
                SessionRegistry::Clock::now(),
                bindings,
                bound_ids,
                true))
            return make_error_response(
                409,
                "session_changed",
                "The session changed while the project was loaded.",
                request_id);

        nlohmann::json object_ids = nlohmann::json::array();
        for (std::size_t binding_index : object_binding_indices)
            object_ids.push_back(bound_ids.at(binding_index));
        return HttpResponse::json(200, nlohmann::json{
            {"object_ids", object_ids},
            {"object_count", object_ids.size()}
        });
    } catch (const std::runtime_error &error) {
        std::string message = error.what();
        replace_all(message, source_path, upload.filename);
        replace_all(message, temporary_name, upload.filename);
        return invalid_project(message);
    } catch (...) {
        return make_error_response(
            500,
            "internal_error",
            "The server could not load the project.",
            request_id);
    }
}

// Same flow as add_model: validate on the network thread, then load the
// project on the import pool and answer through `respond`.
std::optional<HttpResponse> handle_load_project(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id,
    const DeferredResponder &respond)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (services.profiles == nullptr)
        return make_error_response(
            503,
            "engine_not_initialized",
            "The REST engine is not initialized.",
            request_id);
    if (request.upload == nullptr)
        return make_error_response(
            400,
            "invalid_multipart",
            "The request must contain one uploaded file.",
            request_id);

    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username,
        request.path_segments[1],
        SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation.has_value())
        return make_error_response(
            409,
            "session_busy",
            "The session is currently slicing.",
            request_id);
    std::optional<SessionData> current = session->data_snapshot();
    if (!current.has_value())
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);

    asio::post(services.imports, [
        session,
        token = mutation.value(),
        data = std::move(current.value()),
        upload = request.upload,
        profiles = services.profiles,
        request_id,
        respond,
        observer = services.import_observer]() {
        if (observer)
            observer();
        respond(import_project(*session, token, data, *upload, *profiles, request_id));
    });
    return std::nullopt;
}

// Decode once so encoded duplicate keys and NUL cannot bypass query validation.
bool parse_form_query(const std::string &query, std::map<std::string, std::string> &parameters)
{
    for (std::size_t cursor = 0; cursor < query.size();) {
        const std::size_t end = query.find('&', cursor);
        const std::string pair = query.substr(cursor, end == std::string::npos ? end : end - cursor);
        cursor = end == std::string::npos ? query.size() : end + 1;
        if (pair.empty())
            continue;
        const std::size_t equals = pair.find('=');
        std::string key;
        std::string value;
        std::string encoded_key = pair.substr(0, equals);
        std::string encoded_value = equals == std::string::npos ? std::string() : pair.substr(equals + 1);
        std::replace(encoded_key.begin(), encoded_key.end(), '+', ' ');
        std::replace(encoded_value.begin(), encoded_value.end(), '+', ' ');
        if (!decode_path_segment(encoded_key, key) || !decode_path_segment(encoded_value, value) ||
            !parameters.emplace(std::move(key), std::move(value)).second)
            return false;
    }
    return true;
}

// The response owns its private archive until the asynchronous transport releases it.
HttpResponse export_project(const SessionData &data, const std::filesystem::path &directory,
    const std::string &filename, const std::string &request_id)
{
    try {
        const std::shared_ptr<TemporaryFile> file = create_temporary_file(directory, "rest-export-", ".3mf");
        // store_3mf can mutate baked_transformation and objects, so it must not receive the committed scene.
        Model scene(*data.scene);
        DynamicPrintConfig configuration(data.configuration);
        for (const char *key : {"print_host", "printhost_apikey", "printhost_cafile", "printhost_port"})
            configuration.erase(key);
        const std::string path = file->path.u8string();
        if (!store_3mf(path.c_str(), &scene, &configuration,
                OptionStore3mf{}.set_fullpath_sources(false).set_profile_sections(data.profile_sections)))
            throw std::runtime_error("Project export failed.");
        HttpResponse response;
        response.content_type = "model/3mf";
        response.body_file = file->path;
        response.body_file_owner = file;
        response.headers.emplace("Content-Disposition", content_disposition_attachment(filename));
        response.headers.emplace("Cache-Control", "no-store");
        return response;
    } catch (...) {
        return make_error_response(500, "internal_error", "The server could not save the project.", request_id);
    }
}

// Capture committed data before dispatch so later mutations or deletion cannot change the export.
std::optional<HttpResponse> handle_save_project(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id, const DeferredResponder &respond)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (services.profiles == nullptr)
        return make_error_response(503, "engine_not_initialized", "The REST engine is not initialized.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The save_project route does not accept a request body.", request_id);
    std::map<std::string, std::string> parameters;
    if (!parse_form_query(request.query, parameters) || parameters.size() > 1 ||
        (!parameters.empty() && (parameters.begin()->first != "file_name" || parameters.begin()->second.empty())))
        return make_error_response(400, "invalid_request", "The save_project query is invalid.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    std::optional<SessionData> data = session->data_snapshot();
    if (!data)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (data->scene == nullptr || data->scene->object_ptrs().empty())
        return make_error_response(409, "empty_scene", "The session contains no objects.", request_id);
    std::optional<std::string> filename;
    if (!parameters.empty())
        filename = portable_file_name(parameters.begin()->second, ".3mf");
    if (!filename && data->name)
        filename = portable_file_name(*data->name, ".3mf");
    asio::post(services.imports, [data = std::move(*data), directory = services.export_directory,
        filename = filename.value_or("project.3mf"), request_id, respond, observer = services.import_observer]() {
        try {
            if (observer)
                observer();
            respond(export_project(data, directory, filename, request_id));
        } catch (...) {
            respond(make_error_response(500, "internal_error", "The server could not save the project.", request_id));
        }
    });
    return std::nullopt;
}

// Route injected by tests only; it is never registered in the standalone
// executable.
HttpResponse handle_private_test(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &request_id)
{
const bool test_route = services.private_test_handler && request.path.rfind("/__test__/", 0) == 0;
if (!test_route)
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);

HttpResponse parse_error;
if (!parse_request_json(request, parse_error, request_id))
    return parse_error;

try {
    return services.private_test_handler(request);
} catch (const std::exception &) {
    return make_error_response(500, "internal_error", "An internal server error occurred.", request_id);
} catch (...) {
    return make_error_response(500, "internal_error", "An internal server error occurred.", request_id);
}
}


// Route one request. `body_complete` is false only for the upload preflight,
// which runs after the headers so a refused upload is never written to disk.
std::optional<HttpResponse> dispatch_request(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &request_id,
    bool body_complete,
    const DeferredResponder &respond)
{
    if (request.path == "/login")
        return handle_login(request, services, request_id);

    // Every route except /login requires a valid bearer token before route
    // resolution, so an unknown path cannot be probed without credentials.
    HttpResponse authentication_error;
    const std::optional<AuthenticatedRequest> authentication = authenticate_request(
        request,
        services,
        authentication_error,
        request_id);
    if (!authentication.has_value())
        return authentication_error;

    if (!body_complete && is_upload_route(request))
        return handle_upload_preflight(request, services, authentication->username, request_id);

    if (request.path == "/logout")
        return handle_logout(request, services, authentication->token, authentication->now, request_id);
    if (request.path == "/list_settings")
        return handle_list_settings(request, request_id);
    if (request.path == "/create_session")
        return handle_create_session(request, services, authentication->username, request_id);

    // Session routes match decoded segments and resolve the owning account before
    // their handler reads or mutates the session.
    if (request.path_segments.size() >= 3 && request.path_segments[0] == "session") {
        const std::string &operation = request.path_segments[2];
        if (request.path_segments.size() == 3 && operation == "list_profiles")
            return handle_list_profiles(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "set_profile")
            return handle_set_profile(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "add_model")
            return handle_add_model(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "load_project")
            return handle_load_project(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "save_project")
            return handle_save_project(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "set_name")
            return handle_set_name(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "get_name")
            return handle_get_name(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "clear_platter")
            return handle_clear_platter(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "list_objects")
            return handle_list_objects(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "transform")
            return handle_transform(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "set_instance_count")
            return handle_set_instance_count(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "get_setting")
            return handle_get_setting(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "get_settings")
            return handle_get_settings(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 5 && operation == "set_setting")
            return handle_set_setting(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "set_settings")
            return handle_set_settings(request, services, authentication->username, request_id);
    }

    return handle_private_test(request, services, request_id);
}


class HttpSession;

// HTTP body limit shared by every route: the largest buffered JSON body or the
// file payload plus its multipart framing. The framing is the part headers
// (bounded by max_header_bytes in the multipart parser), a preamble of the same
// bound, and a few boundary lines; the file bytes are bounded by the parser.
std::uint64_t parser_body_limit(const ServerConfig &config)
{
    constexpr std::uint64_t boundary_lines_bytes = 4ULL * 1024ULL;
    const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t framing = config.max_header_bytes > (maximum - boundary_lines_bytes) / 2
        ? maximum
        : 2 * config.max_header_bytes + boundary_lines_bytes;
    const std::uint64_t upload_limit = config.max_upload_bytes > maximum - framing
        ? maximum
        : config.max_upload_bytes + framing;
    return std::max(config.max_request_body_bytes, upload_limit);
}

// One accepted connection serving exactly one request.
//
// Buffered requests (every JSON route) must arrive completely within one
// request deadline, so a client dripping bytes cannot hold a connection
// forever. A streamed upload may legitimately take longer than that deadline,
// so it uses the same delay as an idle limit between two reads instead.
// Routes may answer later through a DeferredResponder; no timer runs while the
// server itself is working on the request. After an early error response the
// unread request body is drained within bounds before the socket is closed.
class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
    using CloseCallback = std::function<void(HttpSession *)>;

    HttpSession(
        Tcp::socket socket,
        std::shared_ptr<const ServerConfig> config,
        RestRouter router,
        CloseCallback close_callback)
        : socket_(std::move(socket)),
          config_(std::move(config)),
          router_(std::move(router)),
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
    // Close the connection when `delay` elapses. Re-arming replaces the
    // previous deadline; its pending wait completes with operation_aborted.
    void arm_timeout(std::chrono::steady_clock::duration delay)
    {
        timer_.expires_after(delay);
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

    // The deadline armed here covers the headers and, for buffered requests,
    // the whole body as well.
    void read_request()
    {
        if (closed_.load())
            return;
        parser_.body_limit(parser_body_limit(*config_));
        parser_.header_limit(config_->max_header_bytes);
        arm_timeout(config_->request_timeout);
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_read_header(socket_, buffer_, parser_, [self](const ErrorCode &error, std::size_t bytes) {
            self->handle_header(error, bytes);
        });
    }

    // Build the request context from the headers. An upload is authorized
    // here, before any byte of its body is stored, and switches the session to
    // the idle timeout; other requests keep the running request deadline.
    void handle_header(const ErrorCode &error, std::size_t)
    {
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
        request_id_ = make_request_id();
        HttpResponse response;
        request_ = HttpRequest();
        if (!build_request_context(message, request_, response, request_id_)) {
            write_response(response);
            return;
        }

        if (is_upload_route(request_)) {
            response = route(false).value_or(HttpResponse());
            if (!response.stream_request) {
                write_response(response);
                return;
            }
            try {
                upload_parser_ = std::make_unique<MultipartUploadParser>(
                    header_value(request_, "content-type"),
                    upload_directory(config_->work_directory),
                    config_->max_upload_bytes,
                    config_->max_header_bytes,
                    upload_extensions(request_));
            } catch (const MultipartUploadError &upload_error) {
                write_response(make_error_response(
                    upload_error.status(),
                    upload_error.code(),
                    upload_error.what(),
                    request_id_));
                return;
            } catch (const std::exception &upload_error) {
                write_response(make_error_response(
                    400,
                    "invalid_multipart",
                    upload_error.what(),
                    request_id_));
                return;
            }
            idle_timeout_ = true;
        }
        read_body();
    }

    void read_body()
    {
        if (closed_.load())
            return;
        if (parser_.is_done()) {
            finish_request();
            return;
        }

        http::buffer_body::value_type &body = parser_.get().body();
        body.data = body_chunk_.data();
        body.size = body_chunk_.size();
        if (idle_timeout_)
            arm_timeout(config_->request_timeout);
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_read_some(socket_, buffer_, parser_, [self](const ErrorCode &error, std::size_t bytes) {
            self->handle_body(error, bytes);
        });
    }

    // Stream upload bytes to the multipart parser, or append JSON bytes to the
    // bounded buffer, then continue until the HTTP body is complete.
    void handle_body(const ErrorCode &error, std::size_t)
    {
        if (idle_timeout_)
            cancel_timeout();
        if (closed_.load())
            return;

        const http::buffer_body::value_type &body = parser_.get().body();
        const std::size_t remaining = std::min(body.size, body_chunk_.size());
        const std::size_t received = body_chunk_.size() - remaining;
        try {
            if (received != 0) {
                if (upload_parser_ != nullptr) {
                    upload_parser_->consume(body_chunk_.data(), received);
                } else {
                    if (received > config_->max_request_body_bytes -
                        std::min<std::uint64_t>(config_->max_request_body_bytes, buffered_body_.size())) {
                        write_response(make_error_response(
                            413,
                            "request_too_large",
                            "The request body exceeds the configured size limit.",
                            request_id_));
                        return;
                    }
                    buffered_body_.append(body_chunk_.data(), received);
                }
            }
        } catch (const MultipartUploadError &upload_error) {
            write_response(make_error_response(
                upload_error.status(),
                upload_error.code(),
                upload_error.what(),
                request_id_));
            return;
        } catch (const std::exception &upload_error) {
            write_response(make_error_response(
                400,
                "invalid_multipart",
                upload_error.what(),
                request_id_));
            return;
        }

        if (error && error != http::error::need_buffer) {
            if (error == http::error::body_limit)
                write_response(make_error_response(
                    413,
                    "request_too_large",
                    "The request body exceeds the configured size limit.",
                    request_id_));
            else
                write_response(make_error_response(
                    400,
                    "invalid_request",
                    "The request body is truncated or invalid.",
                    request_id_));
            return;
        }
        if (parser_.is_done())
            finish_request();
        else
            read_body();
    }

    // The body is complete: stop the client timeout and route the request. A
    // deferred route answers later through the responder.
    void finish_request()
    {
        cancel_timeout();
        try {
            if (upload_parser_ != nullptr) {
                request_.upload = upload_parser_->finish();
                upload_parser_.reset();
            } else {
                request_.body = std::move(buffered_body_);
            }
        } catch (const MultipartUploadError &upload_error) {
            write_response(make_error_response(
                upload_error.status(),
                upload_error.code(),
                upload_error.what(),
                request_id_));
            return;
        } catch (const std::exception &error) {
            write_response(make_error_response(
                400,
                "invalid_multipart",
                error.what(),
                request_id_));
            return;
        }

        const std::optional<HttpResponse> response = route(true);
        // Credentials must not stay in the request buffers after routing. A
        // deferred import keeps its own reference to the uploaded file.
        request_.json_body.reset();
        request_.upload.reset();
        clear_secret(request_.body);
        request_.body.clear();
        if (response.has_value())
            write_response(response.value());
    }

    // Call the router and turn any escaping exception into a 500 response.
    std::optional<HttpResponse> route(bool body_complete)
    {
        try {
            return router_(request_, request_id_, body_complete, make_responder());
        } catch (const std::exception &) {
        } catch (...) {
        }
        return make_error_response(
            500,
            "internal_error",
            "An internal server error occurred.",
            request_id_);
    }

    // The responder may be called from an import thread: it posts the write
    // back to the connection's executor, which owns the socket.
    DeferredResponder make_responder()
    {
        const std::shared_ptr<HttpSession> self = shared_from_this();
        return [self](HttpResponse response) {
            asio::post(self->socket_.get_executor(), [self, response = std::move(response)]() {
                self->write_response(response);
            });
        };
    }

    void set_response_fields(http::response_header<> &message, const HttpResponse &response)
    {
        message.set(http::field::server, "SuperSlicer REST");
        message.set(http::field::connection, "close");
        if (!response.no_body && !response.content_type.empty())
            message.set(http::field::content_type, response.content_type);
        for (const std::pair<const std::string, std::string> &header : response.headers)
            message.set(header.first, header.second);
    }

    void write_response(const HttpResponse &response)
    {
        if (closed_)
            return;
        upload_parser_.reset();
        request_.upload.reset();

        unsigned int status = response.status;
        if (status < 100 || status > 599)
            status = 500;
        if (!response.body_file.empty()) {
            file_response_owner_ = response.body_file_owner;
            file_response_message_.emplace(static_cast<http::status>(status), 11);
            ErrorCode error;
            file_response_message_->body().open(response.body_file.u8string().c_str(), beast::file_mode::scan, error);
            if (error) {
                file_response_message_.reset();
                file_response_owner_.reset();
                write_response(make_error_response(500, "internal_error", "The server could not send the file.", request_id_));
                return;
            }
            set_response_fields(*file_response_message_, response);
            file_response_message_->prepare_payload();
            arm_timeout(config_->request_timeout);
            const std::shared_ptr<HttpSession> self = shared_from_this();
            http::async_write(socket_, *file_response_message_, [self](const ErrorCode &error, std::size_t bytes) {
                self->handle_write(error, bytes);
            });
            return;
        }
        response_message_.emplace(static_cast<http::status>(status), 11);
        set_response_fields(*response_message_, response);
        response_message_->body() = response.no_body ? std::string() : response.body;
        response_message_->prepare_payload();
        arm_timeout(config_->request_timeout);
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_write(socket_, response_message_.value(), [self](
            const ErrorCode &error, std::size_t bytes) {
            self->handle_write(error, bytes);
        });
    }

    void handle_write(const ErrorCode &error, std::size_t)
    {
        cancel_timeout();
        file_response_message_.reset();
        file_response_owner_.reset();
        if (error || parser_.is_done()) {
            close();
            return;
        }
        drain_request();
    }

    // The response was sent before the client finished its request body.
    // Closing now with unread input would make the OS send a TCP reset, and
    // many clients would then report "connection reset" instead of the
    // response. Half-close the sending side and discard the remaining input
    // until the client stops, the byte bound is reached, or the delay expires.
    void drain_request()
    {
        ErrorCode ignored;
        socket_.shutdown(Tcp::socket::shutdown_send, ignored);
        drained_bytes_ = 0;
        arm_timeout(drain_timeout);
        read_drain();
    }

    void read_drain()
    {
        const std::shared_ptr<HttpSession> self = shared_from_this();
        socket_.async_read_some(asio::buffer(body_chunk_), [self](const ErrorCode &error, std::size_t bytes) {
            if (self->closed_.load())
                return;
            self->drained_bytes_ += bytes;
            if (error || self->drained_bytes_ >= max_drain_bytes) {
                self->close();
                return;
            }
            self->read_drain();
        });
    }

    Tcp::socket socket_;
    beast::flat_buffer buffer_;
    HttpRequestParser parser_;
    HttpRequest request_;
    std::string request_id_;
    std::string buffered_body_;
    std::array<char, 64 * 1024> body_chunk_{};
    std::unique_ptr<MultipartUploadParser> upload_parser_;
    std::shared_ptr<const ServerConfig> config_;
    RestRouter router_;
    CloseCallback close_callback_;
    asio::steady_timer timer_;
    std::optional<HttpResponseMessage> response_message_;
    // Reverse destruction closes the read handle before the temporary file is deleted on Windows.
    std::shared_ptr<const void> file_response_owner_;
    std::optional<http::response<http::file_body>> file_response_message_;
    // True once an upload is accepted: the timeout becomes an idle limit.
    bool idle_timeout_ = false;
    std::size_t drained_bytes_ = 0;
    std::atomic<bool> closed_{false};
};

class ServerRuntime {
public:
    ServerRuntime(ServerConfig config, RestRouter router, RequestHandler private_test_handler)
        : config_(std::move(config)),
          config_shared_(std::make_shared<const ServerConfig>(config_)),
          router_(std::move(router)),
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
        // The catalog is immutable after startup and is borrowed by HTTP
        // sessions until stop() joins the worker and releases it.
        if (settings_catalog_ready())
            profile_catalog_ = std::make_unique<ProfileCatalog>(config_.profiles_directory);
        session_registry_ = std::make_unique<SessionRegistry>(config_.session_ttl);
        AuthConfig auth_config;
        auth_config.token_ttl = config_.token_ttl;
        auth_config.login_window = config_.login_window;
        auth_config.max_login_attempts = config_.max_login_attempts;
        auth_service_ = std::make_shared<AuthService>(config_.accounts_file, auth_config);
        std::error_code filesystem_error;
        std::filesystem::create_directories(config_.work_directory, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument(json_error_message(
                "paths.work", filesystem_error.message()));
        remove_stale_uploads(upload_directory(config_.work_directory));
        export_directory_ = config_.work_directory / "exports";
        std::filesystem::create_directories(export_directory_, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument("paths.work: cannot create the export directory: " + filesystem_error.message());
        try {
            remove_stale_temporary_files(export_directory_, "rest-export-");
        } catch (const std::filesystem::filesystem_error &failure) {
            throw std::invalid_argument("paths.work: cannot list the export directory: " + failure.code().message());
        }

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
        import_pool_ = std::make_unique<asio::thread_pool>(config_.import_threads);
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

        // Running imports finish before the network thread stops: their
        // responses are posted to io_context_ and their captured connections
        // must be released while it still exists.
        if (import_pool_ != nullptr)
            import_pool_->join();
        io_context_.stop();
        if (worker_.joinable())
            worker_.join();

        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.clear();
        import_pool_.reset();
        auth_service_.reset();
        profile_catalog_.reset();
        session_registry_.reset();
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

    // Services addressed by the routes for one request; valid while the runtime
    // is running and the worker has not been joined.
    RouteServices route_services() const
    {
        return RouteServices{
            *auth_service_,
            *session_registry_,
            profile_catalog_.get(),
            private_test_handler_,
            *import_pool_,
            import_observer_,
            config_.max_instances_per_object,
            export_directory_};
    }

    void set_import_observer(std::function<void()> observer)
    {
        if (running_.load())
            throw std::logic_error("The import observer must be set before the REST server starts");
        import_observer_ = std::move(observer);
    }

    SessionRegistry &session_registry() const
    {
        if (session_registry_ == nullptr)
            throw std::logic_error("REST sessions are only available while the server is running");
        return *session_registry_;
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
                    router_,
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
    std::filesystem::path export_directory_;
    std::shared_ptr<const ServerConfig> config_shared_;
    std::shared_ptr<AuthService> auth_service_;
    std::unique_ptr<ProfileCatalog> profile_catalog_;
    RestRouter router_;
    // Created in start(), released in stop() after the sessions are closed;
    // HttpSession borrows it.
    std::unique_ptr<SessionRegistry> session_registry_;
    RequestHandler private_test_handler_;
    std::function<void()> import_observer_;
    // Created in start(), joined in stop() before the network thread stops.
    std::unique_ptr<asio::thread_pool> import_pool_;
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
    config.max_upload_bytes = default_max_upload_bytes;
    config.max_header_bytes = default_max_header_bytes;
    config.max_connections = default_max_connections;
    config.max_instances_per_object = default_max_instances_per_object;
    config.import_threads = default_import_threads;
    config.request_timeout = std::chrono::seconds(default_request_timeout_seconds);
    config.token_ttl = std::chrono::seconds(default_token_ttl_seconds);
    config.session_ttl = std::chrono::seconds(default_session_ttl_seconds);
    config.login_window = std::chrono::seconds(default_login_window_seconds);
    config.max_login_attempts = default_max_login_attempts;
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
    } else {
        throw std::invalid_argument("paths.accounts must be an existing file");
    }

    if (max_request_body_bytes == 0)
        throw std::invalid_argument("limits.max_request_body_bytes must be positive");
    if (max_upload_bytes == 0)
        throw std::invalid_argument("limits.max_upload_bytes must be positive");
    if (max_header_bytes == 0)
        throw std::invalid_argument("limits.max_header_bytes must be positive");
    if (max_connections == 0)
        throw std::invalid_argument("limits.max_connections must be positive");
    if (max_instances_per_object == 0)
        throw std::invalid_argument("limits.max_instances_per_object must be positive");
    if (import_threads == 0)
        throw std::invalid_argument("limits.import_threads must be positive");
    if (request_timeout.count() <= 0)
        throw std::invalid_argument("timeouts.request_seconds must be positive");
    if (token_ttl.count() <= 0)
        throw std::invalid_argument("auth.token_ttl_seconds must be positive");
    if (session_ttl.count() <= 0)
        throw std::invalid_argument("sessions.expiration_seconds must be positive");
    if (login_window.count() <= 0)
        throw std::invalid_argument("auth.login_window_seconds must be positive");
    if (max_login_attempts == 0)
        throw std::invalid_argument("auth.max_login_attempts must be positive");
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
        : runtime_(
              std::move(config),
              [this](
                  HttpRequest &request,
                  const std::string &request_id,
                  bool body_complete,
                  const DeferredResponder &respond) {
                  return dispatch_request(request, runtime_.route_services(), request_id, body_complete, respond);
              },
              std::move(private_test_handler))
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

SessionRegistry &RestServer::session_registry() const
{
    return impl_->runtime_.session_registry();
}

void RestServer::set_import_observer(std::function<void()> observer)
{
    impl_->runtime_.set_import_observer(std::move(observer));
}

} // namespace Slic3r::Rest
