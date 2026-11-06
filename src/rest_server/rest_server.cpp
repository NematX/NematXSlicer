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
#include "profiles.hpp"
#include "session.hpp"
#include "settings.hpp"

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
// Resolves one parsed request to a response; the transport owns no routing.
using RestRouter = std::function<HttpResponse(HttpRequest &, const std::string &request_id)>;

// Default buffered JSON body limit. The specification's 1 GiB upload limit
// belongs to the streamed multipart route, not to buffered JSON bodies.
constexpr std::uint64_t default_max_request_body_bytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_header_bytes = 16ULL * 1024ULL;
constexpr std::uint32_t default_max_connections = 64;
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
};


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

// Create an owned session, seeded from the profile catalog defaults when the
// engine catalog is available. The route accepts no request body.
HttpResponse handle_create_session(
    const HttpRequest &request,
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
        "The create_session route does not accept a request body.",
        request_id);
SessionData session_data{std::make_shared<Model>(), {}, nlohmann::json::object()};
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
    session_data.profiles = initial.selection;
}
const std::shared_ptr<SessionContext> session = services.sessions.create(
    username,
    SessionRegistry::Clock::now(),
    std::move(session_data));
return HttpResponse::json(201, nlohmann::json{{"session_id", session->session_id()}});
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
const nlohmann::json response_body = profile_selection_json(application.selection);
SessionData candidate = current.value();
candidate.configuration = std::move(application.configuration);
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


HttpResponse dispatch_request(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &request_id)
{
    if (request.path == "/login")
        return handle_login(request, services, request_id);

    // Every route except /login requires a valid bearer token before route
    // resolution, so an unknown path cannot be probed without credentials.
    const std::optional<std::string> token = bearer_token(request);
    if (!token.has_value())
        return make_unauthorized_response(request_id);

    const AuthTimePoint now = std::chrono::system_clock::now();
    const std::optional<std::string> username = services.auth.username_for_token(*token, now);
    if (!username.has_value())
        return make_unauthorized_response(request_id);

    if (request.path == "/logout")
        return handle_logout(request, services, *token, now, request_id);
    if (request.path == "/list_settings")
        return handle_list_settings(request, request_id);
    if (request.path == "/create_session")
        return handle_create_session(request, services, *username, request_id);

    // Session routes match decoded segments and resolve the owning account before
    // their handler reads or mutates the session.
    if (request.path_segments.size() >= 3 && request.path_segments[0] == "session") {
        const std::string &operation = request.path_segments[2];
        if (request.path_segments.size() == 3 && operation == "list_profiles")
            return handle_list_profiles(request, services, *username, request_id);
        if (request.path_segments.size() == 3 && operation == "set_profile")
            return handle_set_profile(request, services, *username, request_id);
        if (request.path_segments.size() == 4 && operation == "get_setting")
            return handle_get_setting(request, services, *username, request_id);
        if (request.path_segments.size() == 3 && operation == "get_settings")
            return handle_get_settings(request, services, *username, request_id);
        if (request.path_segments.size() == 5 && operation == "set_setting")
            return handle_set_setting(request, services, *username, request_id);
        if (request.path_segments.size() == 3 && operation == "set_settings")
            return handle_set_settings(request, services, *username, request_id);
    }

    return handle_private_test(request, services, request_id);
}


class HttpSession;

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
        try {
            response = router_(request, request_id);
        } catch (const std::exception &) {
            response = make_error_response(
                500,
                "internal_error",
                "An internal server error occurred.",
                request_id);
        } catch (...) {
            response = make_error_response(
                500,
                "internal_error",
                "An internal server error occurred.",
                request_id);
        }
        // Credentials must not stay in the request buffers after routing.
        request.json_body.reset();
        clear_secret(request.body);
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
        for (const std::pair<const std::string, std::string> &header : response.headers)
            response_message_->set(header.first, header.second);
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
    RestRouter router_;
    CloseCallback close_callback_;
    asio::steady_timer timer_;
    std::optional<HttpResponseMessage> response_message_;
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
        return RouteServices{*auth_service_, *session_registry_, profile_catalog_.get(), private_test_handler_};
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
    std::shared_ptr<const ServerConfig> config_shared_;
    std::shared_ptr<AuthService> auth_service_;
    std::unique_ptr<ProfileCatalog> profile_catalog_;
    RestRouter router_;
    // Created in start(), released in stop() after the sessions are closed;
    // HttpSession borrows it.
    std::unique_ptr<SessionRegistry> session_registry_;
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
    if (max_header_bytes == 0)
        throw std::invalid_argument("limits.max_header_bytes must be positive");
    if (max_connections == 0)
        throw std::invalid_argument("limits.max_connections must be positive");
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
              [this](HttpRequest &request, const std::string &request_id) {
                  return dispatch_request(request, runtime_.route_services(), request_id);
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

} // namespace Slic3r::Rest
