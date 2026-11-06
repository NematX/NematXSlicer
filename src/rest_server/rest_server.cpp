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
#include "engine.hpp"
#include "file_names.hpp"
#include "multipart_upload.hpp"
#include "profiles.hpp"
#include "progress.hpp"
#include "scene_edit.hpp"
#include "session.hpp"
#include "settings.hpp"
#include "temporary_file.hpp"
#include "transform.hpp"
#include "viewer_json.hpp"
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
#include <initializer_list>
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

// Reject misspellings at each configuration level instead of silently using defaults.
void reject_unknown_keys(const nlohmann::json &object, const std::string &level,
    std::initializer_list<const char *> allowed);

// Default buffered JSON body limit. The specification's 1 GiB upload limit
// belongs to the streamed multipart route, not to buffered JSON bodies.
constexpr std::uint64_t default_max_request_body_bytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_upload_bytes = 1ULL * 1024ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t default_max_header_bytes = 16ULL * 1024ULL;
constexpr std::uint32_t default_max_connections = 64;
constexpr std::uint32_t default_max_instances_per_object = 1000;
constexpr std::uint32_t default_import_threads = 2;
constexpr std::uint32_t default_slice_threads = 2;
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

// Clamp cleanup cadence without overflowing milliseconds when the configured TTL is very long.
std::chrono::milliseconds session_expiration_interval(std::chrono::seconds ttl)
{
    const std::chrono::seconds capped = std::min(ttl, std::chrono::seconds(240));
    return std::clamp(std::chrono::duration_cast<std::chrono::milliseconds>(capped) / 4,
        std::chrono::milliseconds(100), std::chrono::milliseconds(60000));
}

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

    const std::filesystem::path path_value = std::filesystem::u8path(path_text);
    if (path_value.is_absolute())
        return path_value;
    return base_directory / path_value;
}

void reject_unknown_keys(const nlohmann::json &object, const std::string &level,
    std::initializer_list<const char *> allowed)
{
    for (nlohmann::json::const_iterator entry = object.begin(); entry != object.end(); ++entry) {
        bool known = false;
        for (const char *key : allowed)
            known = known || entry.key() == key;
        if (!known)
            throw std::invalid_argument("unknown configuration key: " + level + "." + entry.key());
    }
}

void apply_json_config(
    ServerConfig &config,
    const nlohmann::json &document,
    const std::filesystem::path &base_directory)
{
    if (!document.is_object())
        throw std::invalid_argument("configuration root must be a JSON object");
    reject_unknown_keys(document, "configuration", {"listen", "paths", "limits", "timeouts", "auth", "sessions"});

    if (document.contains("listen")) {
        const nlohmann::json &listen = document.at("listen");
        if (!listen.is_object())
            throw std::invalid_argument("listen must be a JSON object");
        reject_unknown_keys(listen, "listen", {"address", "port"});
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
        reject_unknown_keys(paths, "paths", {"resources", "profiles", "work", "accounts"});
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
        reject_unknown_keys(limits, "limits", {"max_request_body_bytes", "max_upload_bytes", "max_header_bytes",
            "max_connections", "import_threads", "slice_threads", "max_instances_per_object"});
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
        if (limits.contains("slice_threads")) {
            const std::uint64_t slice_threads = read_unsigned_json(
                limits.at("slice_threads"), "limits.slice_threads");
            if (slice_threads > std::numeric_limits<std::uint32_t>::max())
                throw std::invalid_argument("limits.slice_threads is too large");
            config.slice_threads = static_cast<std::uint32_t>(slice_threads);
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
        reject_unknown_keys(timeouts, "timeouts", {"request_seconds"});
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
        reject_unknown_keys(auth, "auth", {"token_ttl_seconds", "login_window_seconds", "max_login_attempts"});
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
        reject_unknown_keys(sessions, "sessions", {"expiration_seconds"});
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


// Submission and shutdown share this lock so no job can arrive after pool joining.
class SliceJobs {
public:
    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = false;
    }

    bool submit(asio::thread_pool &pool, const std::shared_ptr<SessionContext> &session,
        std::function<void(std::uint64_t)> job)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_)
            return false;
        const std::uint64_t id = ++next_id_;
        active_.emplace(id, session);
        try {
            asio::post(pool, [job = std::move(job), id]() { job(id); });
        } catch (...) {
            active_.erase(id);
            throw;
        }
        return true;
    }

    // The session arbitrates cancellation against publication; the job stays
    // registered until it has published, so shutdown can still cancel it.
    void complete(std::uint64_t id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_.erase(id);
    }

    // Shutdown uses the same session cancellation as cancel_slicing: queued
    // work ends at once, started work stops its engine run.
    void cancel()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        for (const std::pair<const std::uint64_t, std::shared_ptr<SessionContext>> &entry : active_)
            entry.second->request_cancel(SessionRegistry::Clock::now());
    }

private:
    std::mutex mutex_;
    std::uint64_t next_id_ = 0;
    std::map<std::uint64_t, std::shared_ptr<SessionContext>> active_;
    bool stopping_ = false;
};

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
    const std::function<void()> &scene_edit_observer;
    std::uint32_t max_instances_per_object;
    const std::filesystem::path &export_directory;
    asio::thread_pool &slices;
    SliceJobs &slice_jobs;
    const std::function<void()> &slice_observer;
    const std::function<void(const std::string &)> &progress_observer;
    const std::filesystem::path &result_directory;
    const std::filesystem::path &work_directory;
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
    return !request.path_segments.empty() && request.path_segments[0] == "session" &&
        ((request.path_segments.size() == 3 &&
            (request.path_segments[2] == "add_model" || request.path_segments[2] == "load_project")) ||
         (request.path_segments.size() == 4 && request.path_segments[2] == "add_part"));
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
std::optional<HttpResponse> handle_create_session(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id,
    const DeferredResponder &respond)
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
    // Catalog copies run on the existing import pool. Publish only after the
    // defaults succeed; stop() joins this pool before releasing the registry.
    asio::post(services.imports, [profiles = services.profiles, sessions = &services.sessions,
        username, name, request_id, respond, data = std::move(session_data), observer = services.import_observer]() mutable {
        try {
            if (observer)
                observer();
            ProfileApplication initial = profiles->defaults();
            data.configuration = std::move(initial.configuration);
            data.profile_sections = std::move(initial.profile_sections);
            data.profiles = std::move(initial.selection);
            const std::shared_ptr<SessionContext> session = sessions->create(username,
                SessionRegistry::Clock::now(), std::move(data));
            respond(HttpResponse::json(201, nlohmann::json{
                {"session_id", session->session_id()}, {"name", session_name_json(name)}}));
        } catch (...) {
            respond(make_error_response(500, "internal_error",
                "The server profile catalog could not provide defaults.", request_id));
        }
    });
    return std::nullopt;
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
// Shared strict percent-decoding/duplicate detection for form-style queries.
bool parse_form_query(const std::string &query, std::map<std::string, std::string> &parameters);

HttpResponse handle_list_profiles(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id)
{
    const std::string &session_id = request.path_segments[1];
if (request.method != "GET")
    return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
if (!request.body.empty())
    return make_error_response(400, "invalid_request", "list_profiles requires an empty body.", request_id);
std::map<std::string, std::string> parameters;
if (!parse_form_query(request.query, parameters))
    return make_error_response(400, "invalid_request", "Invalid profile query.", request_id);
for (const auto &[key, value] : parameters)
    if (value.empty() || (key != "printer" && key != "print" && key != "compatible_only") ||
        (key == "compatible_only" && value != "0" && value != "1"))
        return make_error_response(400, "invalid_request", "Invalid profile query.", request_id);
const std::shared_ptr<SessionContext> session = services.sessions.find(username, session_id, SessionRegistry::Clock::now());
if (session == nullptr)
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
if (services.profiles == nullptr)
    return make_error_response(
        503,
        "engine_not_initialized",
        "The REST engine is not initialized.",
        request_id);
const std::optional<SessionData> data = session->data_snapshot();
if (!data.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
const bool explicit_printer = parameters.count("printer") != 0;
const bool explicit_print = parameters.count("print") != 0;
ProfileCatalog::CompatibilityContext context;
context.printer = services.profiles->printer_by_id(explicit_printer ? parameters.at("printer") : data->profiles.printer);
context.print = services.profiles->print_by_id(explicit_print ? parameters.at("print") : data->profiles.print);
if ((explicit_printer && context.printer == nullptr) || (explicit_print && context.print == nullptr) ||
    (context.printer != nullptr && context.printer->printer_technology() != ptFFF))
    return make_error_response(422, "invalid_profile", "The selected profile is unavailable or not FFF.", request_id);
// Without query overrides, use the live configuration, even when a catalog ID
// remains after set_settings. Keep catalog name/vendor identity for named lists.
std::optional<Preset> session_printer;
std::optional<Preset> session_print;
if (!explicit_printer) {
    if (context.printer != nullptr)
        session_printer.emplace(*context.printer);
    else
        session_printer.emplace(SessionCompatibilityPreset(Preset::TYPE_PRINTER, data->configuration));
    session_printer->config = data->configuration;
    context.printer = &*session_printer;
}
if (!explicit_print) {
    if (context.print != nullptr)
        session_print.emplace(*context.print);
    else
        session_print.emplace(SessionCompatibilityPreset(Preset::TYPE_FFF_PRINT, data->configuration));
    session_print->config = data->configuration;
    context.print = &*session_print;
}
return HttpResponse::json(200, services.profiles->list_profiles(context,
    parameters.count("compatible_only") != 0 && parameters.at("compatible_only") == "1"));
}

// Apply a full profile selection on a candidate configuration outside the
// session lock, then commit it under the session revision.
std::optional<HttpResponse> handle_set_profile(
    HttpRequest &request,
    const RouteServices &services,
    const std::string &username,
    const std::string &request_id,
    const DeferredResponder &respond)
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

const std::optional<SessionData> current = session->data_snapshot();
if (!current.has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
// Capture the revision and snapshot before dispatch. Concurrent mutation or
// deletion wins; expensive selection never holds the network thread or a lock.
asio::post(services.imports, [session, token = *mutation, candidate = *current,
    profiles = services.profiles, requested = std::move(requested), request_id, respond,
    observer = services.import_observer]() mutable {
const HttpResponse response = [&]() -> HttpResponse {
ProfileApplication application;
try {
    if (observer)
        observer();
    application = profiles->select(requested);
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

if (!session->data_snapshot().has_value())
    return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
const nlohmann::json response_body = profile_selection_json(application.selection);
candidate.configuration = std::move(application.configuration);
candidate.profile_sections = std::move(application.profile_sections);
candidate.profiles = std::move(application.selection);
if (!session->commit_mutation(
         token,
        std::move(candidate),
        true,
        SessionRegistry::Clock::now()))
    return make_error_response(
        409,
        "session_changed",
        "The session changed while the profile selection was prepared.",
        request_id);
return HttpResponse::json(200, response_body);
}();
respond(response);
});
return std::nullopt;
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

// Parse a scene-edit JSON object with a closed set of fields.
std::optional<HttpResponse> read_scene_edit_body(HttpRequest &request, std::initializer_list<const char *> fields,
    const std::string &request_id)
{
    HttpResponse parse_error;
    if (!request.body.empty() && !parse_request_json(request, parse_error, request_id))
        return parse_error;
    if (request.json_body == nullptr || !request.json_body->is_object())
        return make_error_response(400, "invalid_request", "The body must be a JSON object.", request_id);
    for (const auto &item : request.json_body->items())
        if (std::find_if(fields.begin(), fields.end(), [&item](const char *field) { return item.key() == field; }) == fields.end())
            return make_error_response(400, "invalid_request", "Unknown field: " + item.key(), request_id);
    return std::nullopt;
}

// Commit one private scene edit and translate its typed exception to the public error contract.
HttpResponse scene_edit_error(const SceneEditError &error, const std::string &request_id)
{
    return make_error_response(error.status(), error.code(), error.what(), request_id);
}

HttpResponse handle_set_part_type(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"type"}, request_id))
        return *error;
    if (request.json_body->size() != 1 || !request.json_body->contains("type") || !request.json_body->at("type").is_string())
        return make_error_response(400, "invalid_request", "The body must contain only the string field type.", request_id);
    const std::optional<ModelVolumeType> type = parse_part_type(request.json_body->at("type").get<std::string>());
    if (!type)
        return make_error_response(422, "invalid_part_type", "The part type is unknown.", request_id);
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
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    if (entity->kind != EntityKind::volume)
        return make_error_response(422, "invalid_part_type", "The target must be a part.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        const std::pair<ModelObject *, ModelVolume *> target = find_transform_volume(*candidate.scene, *entity);
        const SceneChange change = set_part_type(*target.first, *target.second, *type);
        if (!change.changed)
            return HttpResponse::json(200, {{"id", request.path_segments[3]}, {"type", part_type_name(*type)}});
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), change.affects_slicing, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        return HttpResponse::json(200, {{"id", request.path_segments[3]}, {"type", part_type_name(*type)}});
    } catch (const SceneEditError &error) {
        return scene_edit_error(error, request_id);
    } catch (...) {
        return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id);
    }
}

HttpResponse handle_rename(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"name"}, request_id))
        return *error;
    if (request.json_body->size() != 1 || !request.json_body->contains("name") || !request.json_body->at("name").is_string())
        return make_error_response(400, "invalid_request", "The body must contain only the string field name.", request_id);
    const std::string name = request.json_body->at("name").get<std::string>();
    try { validate_session_name(name); }
    catch (const std::invalid_argument &error) { return make_error_response(400, "invalid_request", error.what(), request_id); }
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
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    if (entity->kind == EntityKind::instance)
        return make_error_response(422, "invalid_name", "Instances cannot be renamed.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        SceneChange change;
        if (entity->kind == EntityKind::volume)
            change = rename_volume(*find_transform_volume(*candidate.scene, *entity).second, name);
        else
            change = rename_object(*find_transform_instances(*candidate.scene, *entity).at(0)->get_object(), name);
        if (!change.changed)
            return HttpResponse::json(200, {{"id", request.path_segments[3]}, {"name", name}});
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), change.affects_slicing, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        return HttpResponse::json(200, {{"id", request.path_segments[3]}, {"name", name}});
    } catch (const SceneEditError &error) {
        return scene_edit_error(error, request_id);
    } catch (...) {
        return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id);
    }
}

HttpResponse handle_delete_entity(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    bool invalidate_cut = false;
    if (!request.body.empty()) {
        if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"invalidate_cut"}, request_id))
            return *error;
        if (request.json_body->size() != 1 || !request.json_body->contains("invalidate_cut") ||
            !request.json_body->at("invalidate_cut").is_boolean())
            return make_error_response(400, "invalid_request", "The body may contain only the boolean field invalidate_cut.", request_id);
        invalidate_cut = request.json_body->at("invalidate_cut").get<bool>();
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
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        SceneChange change;
        if (entity->kind == EntityKind::volume) {
            const std::pair<ModelObject *, ModelVolume *> target = find_transform_volume(*candidate.scene, *entity);
            change = delete_volume(*target.first, *target.second, invalidate_cut);
        } else {
            std::vector<ModelInstance *> targets = find_transform_instances(*candidate.scene, *entity);
            ModelObject *object = targets.at(0)->get_object();
            if (entity->kind == EntityKind::object)
                change = delete_object(*candidate.scene, *object, invalidate_cut);
            else {
                const size_t index = size_t(std::find(object->instances.begin(), object->instances.end(), targets.at(0)) - object->instances.begin());
                change = delete_instance(*object, index);
            }
        }
        const std::optional<std::vector<std::string>> removed = session->entity_ids(*mutation, change.unbindings);
        if (!removed)
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), change.affects_slicing, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        return HttpResponse::json(200, {{"deleted", *removed}});
    } catch (const SceneEditError &error) {
        return scene_edit_error(error, request_id);
    } catch (...) {
        return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id);
    }
}

std::optional<Vec3d> read_vec3(const nlohmann::json &body, const char *field)
{
    if (!body.contains(field))
        return std::nullopt;
    const nlohmann::json &value = body.at(field);
    if (!value.is_array() || value.size() != 3)
        throw SceneEditError(422, "invalid_shape", std::string(field) + " must contain three numbers.");
    Vec3d result;
    for (size_t index = 0; index < 3; ++index) {
        if (!value.at(index).is_number())
            throw SceneEditError(422, "invalid_shape", std::string(field) + " must contain three numbers.");
        result[int(index)] = value.at(index).get<double>();
    }
    return result;
}

HttpResponse handle_add_shape(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"type", "shape", "size", "position", "instance"}, request_id))
        return *error;
    const nlohmann::json &body = *request.json_body;
    if (!body.contains("type") || !body.at("type").is_string() || !body.contains("shape") || !body.at("shape").is_string() ||
        (body.contains("instance") && !body.at("instance").is_string()))
        return make_error_response(400, "invalid_request", "The body requires string type and shape fields.", request_id);
    const std::optional<ModelVolumeType> type = parse_part_type(body.at("type").get<std::string>());
    if (!type)
        return make_error_response(422, "invalid_shape", "The shape part type is unknown.", request_id);
    ShapeRequest shape{*type, body.at("shape").get<std::string>()};
    try { shape.size = read_vec3(body, "size"); shape.position = read_vec3(body, "position"); }
    catch (const SceneEditError &error) { return scene_edit_error(error, request_id); }
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
    if (!entity)
        return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
    if (entity->kind != EntityKind::object)
        return make_error_response(422, "invalid_shape", "The target must be an object.", request_id);
    std::optional<EntityReference> reference;
    if (body.contains("instance")) {
        reference = session->resolve_entity(body.at("instance").get<std::string>());
        if (!reference)
            return make_error_response(404, "entity_not_found", "The reference instance does not exist.", request_id);
    }
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        ModelObject *object = find_transform_instances(*candidate.scene, *entity).at(0)->get_object();
        size_t instance_index = 0;
        if (reference) {
            if (reference->kind != EntityKind::instance)
                throw SceneEditError(422, "invalid_shape", "The instance field must identify an instance of the target object.");
            ModelInstance *instance = find_transform_instances(*candidate.scene, *reference).at(0);
            if (instance->get_object() != object)
                throw SceneEditError(422, "invalid_shape", "The reference instance belongs to another object.");
            instance_index = size_t(std::find(object->instances.begin(), object->instances.end(), instance) - object->instances.begin());
        }
        const SceneChange change = add_shape(*object, instance_index, shape);
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), true, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        return HttpResponse::json(200, {{"part_id", ids.at(0)}, {"type", part_type_name(*type)}, {"shape", shape.shape}});
    } catch (const SceneEditError &error) { return scene_edit_error(error, request_id); }
    catch (...) { return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id); }
}

HttpResponse handle_split(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"to"}, request_id)) return *error;
    if (request.json_body->size() != 1 || !request.json_body->contains("to") || !request.json_body->at("to").is_string())
        return make_error_response(400, "invalid_request", "The body must contain only the string field to.", request_id);
    const std::string to = request.json_body->at("to").get<std::string>();
    if (to != "objects" && to != "parts")
        return make_error_response(422, "not_splittable", "The split target must be objects or parts.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation) return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
    if (!entity) return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        SceneChange change;
        if (to == "objects") {
            if (entity->kind != EntityKind::object)
                throw SceneEditError(422, "not_splittable", "Only an object can split into objects.");
            ModelObject *object = find_transform_instances(*candidate.scene, *entity).at(0)->get_object();
            change = split_object(*candidate.scene, *object);
        } else {
            ModelObject *object = nullptr;
            ModelVolume *volume = nullptr;
            if (entity->kind == EntityKind::volume) {
                const std::pair<ModelObject *, ModelVolume *> target = find_transform_volume(*candidate.scene, *entity);
                object = target.first; volume = target.second;
            } else if (entity->kind == EntityKind::object) {
                object = find_transform_instances(*candidate.scene, *entity).at(0)->get_object();
                if (object->volumes.size() != 1)
                    throw SceneEditError(422, "not_splittable", "An object target must contain one part.");
                volume = object->volumes.front();
            } else throw SceneEditError(422, "not_splittable", "Instances cannot be split.");
            const ConfigOptionFloats *nozzles = dynamic_cast<const ConfigOptionFloats *>(candidate.configuration.option("nozzle_diameter"));
            change = split_volume(*object, *volume, nozzles == nullptr ? 1U : unsigned(nozzles->get_values().size()));
        }
        const std::optional<std::vector<std::string>> removed = session->entity_ids(*mutation, change.unbindings);
        if (!removed) return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), true, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        if (to == "objects") {
            nlohmann::json object_ids = nlohmann::json::array();
            size_t index = 0;
            while (index < change.bindings.size()) { object_ids.push_back(ids[index]); ++index;
                while (index < change.bindings.size() && change.bindings[index].kind != EntityKind::object) ++index; }
            return HttpResponse::json(200, {{"removed", *removed}, {"object_ids", object_ids}});
        }
        return HttpResponse::json(200, {{"removed", *removed}, {"part_ids", ids}});
    } catch (const SceneEditError &error) { return scene_edit_error(error, request_id); }
    catch (...) { return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id); }
}

HttpResponse handle_merge(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST") return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session) return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (std::optional<HttpResponse> error = read_scene_edit_body(request, {"ids", "name"}, request_id)) return *error;
    const nlohmann::json &body = *request.json_body;
    if (!body.contains("ids") || !body.at("ids").is_array() ||
        (body.contains("name") && !body.at("name").is_string()))
        return make_error_response(400, "invalid_request", "The body requires an ids array and an optional string name.", request_id);
    for (const nlohmann::json &id : body.at("ids")) if (!id.is_string())
        return make_error_response(400, "invalid_request", "Every merge ID must be a string.", request_id);
    const std::string name = body.contains("name") ? body.at("name").get<std::string>() : "Merged";
    try { validate_session_name(name); }
    catch (const std::invalid_argument &error) { return make_error_response(400, "invalid_request", error.what(), request_id); }
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation) return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    const std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    std::vector<EntityReference> entities;
    for (const nlohmann::json &id : body.at("ids")) {
        const std::optional<EntityReference> entity = session->resolve_entity(id.get<std::string>());
        if (!entity) return make_error_response(404, "entity_not_found", "An entity does not exist.", request_id);
        entities.push_back(*entity);
    }
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
    try {
        SessionData candidate = *current;
        candidate.scene = std::make_shared<Model>(*current->scene);
        std::vector<ModelObject *> sources;
        for (const EntityReference &entity : entities) {
            if (entity.kind != EntityKind::object) throw SceneEditError(422, "invalid_merge", "Only objects can be merged.");
            sources.push_back(find_transform_instances(*candidate.scene, entity).at(0)->get_object());
        }
        const SceneChange change = merge_objects(*candidate.scene, sources, name);
        const std::optional<std::vector<std::string>> removed = session->entity_ids(*mutation, change.unbindings);
        if (!removed) return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        if (services.scene_edit_observer)
            services.scene_edit_observer();
        std::vector<std::string> ids;
        if (!session->commit_mutation(*mutation, std::move(candidate), true, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the edit was prepared.", request_id);
        nlohmann::json part_ids = nlohmann::json::array();
        for (size_t index = 0; index < change.bindings.size(); ++index)
            if (change.bindings[index].kind == EntityKind::volume) part_ids.push_back(ids[index]);
        return HttpResponse::json(200, {{"removed", *removed}, {"object_id", ids.at(0)}, {"part_ids", part_ids}});
    } catch (const SceneEditError &error) { return scene_edit_error(error, request_id); }
    catch (...) { return make_error_response(500, "internal_error", "The server could not edit the scene.", request_id); }
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
    if (request.path_segments[2] == "add_part") {
        std::map<std::string, std::string> parameters;
        if (!parse_form_query(request.query, parameters) || parameters.size() != 1 || parameters.count("type") == 0)
            return make_error_response(400, "invalid_request", "The add_part query must contain only type.", request_id);
        const std::optional<ModelVolumeType> type = parse_part_type(parameters.at("type"));
        if (!type || !is_loadable_part_type(*type))
            return make_error_response(422, "invalid_part_type", "This part type cannot be loaded.", request_id);
        // Snapshot first: an entity resolved afterwards may be newer than the
        // snapshot, so a missing match is answered like an unknown entity.
        const std::optional<SessionData> data = session->data_snapshot();
        if (!data)
            return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
        const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
        if (!entity)
            return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
        if (entity->kind != EntityKind::object)
            return make_error_response(422, "invalid_part_type", "The target must be an object.", request_id);
        std::vector<ModelInstance *> instances;
        try {
            if (data->scene != nullptr)
                instances = find_transform_instances(*data->scene, *entity);
        } catch (const std::logic_error &) {
        }
        if (instances.empty())
            return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
        const ModelObject *object = instances.front()->get_object();
        if (object->is_cut() && (*type == ModelVolumeType::MODEL_PART || *type == ModelVolumeType::NEGATIVE_VOLUME))
            return make_error_response(422, "cut_object", "A solid or negative part cannot be added to a cut object.", request_id);
    }
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
    const std::string source_path = upload.temporary_path.u8string();
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
    // Model readers expect UTF-8, including for Windows work directories.
    const std::string source_path = upload.temporary_path.u8string();
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

// Import one uploaded mesh into an existing object on the import pool.
HttpResponse import_part(SessionContext &session, SessionContext::MutationToken mutation, SessionData current,
    EntityReference entity, ModelVolumeType type, const HttpUpload &upload, const std::string &request_id)
{
    // Use the same UTF-8 path for loading and for sanitizing loader errors.
    const std::string source_path = upload.temporary_path.u8string();
    const std::string temporary_name = upload.temporary_path.filename().string();
    try {
        Model imported = Model::read_from_file(source_path, nullptr, nullptr, Model::LoadAttribute::AddDefaultInstances);
        current.scene = std::make_shared<Model>(*current.scene);
        ModelObject *target = find_transform_instances(*current.scene, entity).at(0)->get_object();
        const SceneChange change = add_part(*target, std::move(imported), type, upload.filename);
        std::vector<std::string> ids;
        if (!session.commit_mutation(mutation, std::move(current), true, SessionRegistry::Clock::now(),
                change.bindings, ids, false, change.unbindings, change.rebindings))
            return make_error_response(409, "session_changed", "The session changed while the part was imported.", request_id);
        return HttpResponse::json(200, {{"part_id", ids.at(0)}, {"type", part_type_name(type)}});
    } catch (const SceneEditError &error) {
        return scene_edit_error(error, request_id);
    } catch (const std::runtime_error &error) {
        std::string message = error.what();
        replace_all(message, source_path, upload.filename);
        replace_all(message, temporary_name, upload.filename);
        return make_error_response(422, "invalid_model", message, request_id);
    } catch (...) {
        return make_error_response(500, "internal_error", "The server could not import the part.", request_id);
    }
}

std::optional<HttpResponse> handle_add_part(HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id, const DeferredResponder &respond)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (request.upload == nullptr)
        return make_error_response(400, "invalid_multipart", "The request must contain one uploaded file.", request_id);
    std::map<std::string, std::string> parameters;
    if (!parse_form_query(request.query, parameters) || parameters.size() != 1 || parameters.count("type") == 0)
        return make_error_response(400, "invalid_request", "The add_part query must contain only type.", request_id);
    const std::optional<ModelVolumeType> type = parse_part_type(parameters.at("type"));
    if (!type || !is_loadable_part_type(*type))
        return make_error_response(422, "invalid_part_type", "This part type cannot be loaded.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now());
    if (!session)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<SessionContext::MutationToken> mutation = session->begin_mutation();
    if (!mutation)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    std::optional<SessionData> current = session->data_snapshot();
    if (!current)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::optional<EntityReference> entity = session->resolve_entity(request.path_segments[3]);
    if (!entity)
        return make_error_response(404, "entity_not_found", "The entity does not exist.", request_id);
    if (entity->kind != EntityKind::object)
        return make_error_response(422, "invalid_part_type", "The target must be an object.", request_id);
    if (session->snapshot().revision != mutation->revision)
        return make_error_response(409, "session_changed", "The session changed while the part was prepared.", request_id);
    ModelObject *object = find_transform_instances(*current->scene, *entity).at(0)->get_object();
    if (object->is_cut() && (*type == ModelVolumeType::MODEL_PART || *type == ModelVolumeType::NEGATIVE_VOLUME))
        return make_error_response(422, "cut_object", "A solid or negative part cannot be added to a cut object.", request_id);
    asio::post(services.imports, [session, token = *mutation, data = std::move(*current), entity = *entity,
        type = *type, upload = request.upload, request_id, respond, observer = services.import_observer]() mutable {
        if (observer) observer();
        if (!session->data_snapshot()) {
            respond(make_error_response(404, "session_not_found", "The session does not exist.", request_id));
            return;
        }
        respond(import_part(*session, token, std::move(data), entity, type, *upload, request_id));
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
    // Match the model reader and hide_upload_path's UTF-8 source references.
    const std::string source_path = upload.temporary_path.u8string();
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

// The only accepted query of a download route is an optional, nonempty
// `file_name`. False for any other key, a repeated or malformed one.
bool read_file_name_query(const std::string &query, std::optional<std::string> &requested_name)
{
    std::map<std::string, std::string> parameters;
    if (!parse_form_query(query, parameters) || parameters.size() > 1 ||
        (!parameters.empty() && (parameters.begin()->first != "file_name" || parameters.begin()->second.empty())))
        return false;
    requested_name.reset();
    if (!parameters.empty())
        requested_name = parameters.begin()->second;
    return true;
}

// Download name: the requested name, else the session name, else "project",
// each made portable; a name with nothing usable left falls back to the next.
std::string download_file_name(const std::optional<std::string> &requested_name,
    const std::optional<std::string> &session_name, const std::string &extension)
{
    std::optional<std::string> name;
    if (requested_name)
        name = portable_file_name(*requested_name, extension);
    if (!name && session_name)
        name = portable_file_name(*session_name, extension);
    return name.value_or("project" + extension);
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
    std::optional<std::string> requested_name;
    if (!read_file_name_query(request.query, requested_name))
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
    std::string filename = download_file_name(requested_name, data->name, ".3mf");
    asio::post(services.imports, [data = std::move(*data), directory = services.export_directory,
        filename = std::move(filename), request_id, respond, observer = services.import_observer]() {
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

// The accepted response keeps the state contract even when progress is unavailable.
nlohmann::json slice_state_json(const SessionContext::Snapshot &state)
{
    const SliceProgress progress = state.status == SessionStatus::slicing ? state.progress : SliceProgress{};
    return {{"etat", session_status_name(state.status)},
        {"primary_message", state.status == SessionStatus::failed ? state.error_message : progress.main_message},
        {"primary_percentage", progress.main_percentage}, {"secondary_message", progress.secondary_message},
        {"secondary_percentage", progress.secondary_percentage}};
}

// State remains readable without an initialized engine or a current result.
HttpResponse handle_state(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The state route does not accept a request body.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    return HttpResponse::json(200, slice_state_json(session->snapshot()));
}

// Keep worker failures and cancellation within the owning session's work generation.
void run_slice(const std::shared_ptr<SessionContext> &session, SessionContext::WorkToken token,
    SessionData data, const std::shared_ptr<TemporaryDirectory> &directory,
    const std::shared_ptr<SliceControl> &control, const std::function<void()> &observer,
    const std::filesystem::path &work, SliceJobs &jobs, std::uint64_t id)
{
    // Work cancelled while queued already ended in `ready`: skip the engine.
    if (!session->start_work(token)) {
        jobs.complete(id);
        return;
    }
    // Reports after an accepted cancellation end in `ready` (session arbitration);
    // a cancelled engine run surfaces here as an exception.
    std::shared_ptr<const ViewerSnapshot> discarded_viewer;
    try {
        if (observer)
            observer();
        // Engine apply and extruder assignment must never mutate the committed shared scene.
        SliceResult result = Engine::instance().slice(Model(*data.scene), std::move(data.configuration),
            directory->path / "output.gcode", control.get());
        // Built here, off the network thread, once per result for every page.
        std::shared_ptr<const ViewerHeader> header = make_viewer_header(result.viewer);
        session->finish_work(token, std::move(result.output_file), std::move(result.viewer),
            SessionRegistry::Clock::now(), directory, &discarded_viewer, std::move(header));
    } catch (const std::exception &error) {
        session->fail_work(token, sanitized_slice_error(error.what(), directory->path, work),
            SessionRegistry::Clock::now());
    } catch (...) {
        session->fail_work(token, "The server could not slice the project.", SessionRegistry::Clock::now());
    }
    jobs.complete(id);
}

// Admit work before queueing so queued runs already block mutations and expiration.
HttpResponse handle_slice(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (services.profiles == nullptr)
        return make_error_response(503, "engine_not_initialized", "The REST engine is not initialized.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The slice route does not accept a request body.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    std::optional<SessionData> data = session->data_snapshot();
    if (!data)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (data->scene == nullptr || data->scene->object_ptrs().empty())
        return make_error_response(409, "empty_scene", "The session contains no objects.", request_id);
    // The control exists before admission so cancel_slicing always reaches it.
    const std::shared_ptr<SliceControl> control = std::make_shared<SliceControl>();
    const std::optional<SessionContext::WorkToken> token = session->begin_work(
        SessionRegistry::Clock::now(), [control]() { control->request_cancel(); });
    if (!token)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    data = session->data_snapshot();
    if (!data || data->scene == nullptr || data->scene->object_ptrs().empty()) {
        session->fail_work(*token, "The REST slice model is empty.", SessionRegistry::Clock::now());
        return HttpResponse::json(202, slice_state_json(session->snapshot()));
    }
    std::shared_ptr<TemporaryDirectory> directory;
    try {
        directory = create_temporary_directory(services.result_directory, "rest-result-");
    } catch (...) {
        session->fail_work(*token, "The server could not prepare the result storage.", SessionRegistry::Clock::now());
        return HttpResponse::json(202, slice_state_json(session->snapshot()));
    }
    try {
        control->set_progress_callback([session, token = *token, directory_path = directory->path,
            work = services.work_directory, session_id = request.path_segments[1],
            observer = services.progress_observer](const PrintStatus::SlicingStatus &status) {
            // TODO(progress rework): forward legacy callbacks without GUI coalescing until the progress model is replaced.
            std::optional<ProgressUpdate> update = progress_update(status, directory_path, work);
            if (!update)
                return;
            if (session->update_progress(token, update->secondary, std::move(update->message), update->percentage) && observer)
                observer(session_id);
        });
        // Capture the accepted state before a fast worker can publish its completed result.
        HttpResponse response = HttpResponse::json(202, slice_state_json(session->snapshot()));
        SliceJobs &jobs = services.slice_jobs;
        if (!jobs.submit(services.slices, session, [session, token = *token, data = std::move(*data),
            directory, control, observer = services.slice_observer, work = services.work_directory,
            &jobs](std::uint64_t id) mutable {
                run_slice(session, token, std::move(data), directory, control, observer, work, jobs, id);
            })) {
            session->cancel_work(*token, SessionRegistry::Clock::now());
            return HttpResponse::json(202, slice_state_json(session->snapshot()));
        }
        return response;
    } catch (...) {
        session->fail_work(*token, "The server could not schedule the slicing job.", SessionRegistry::Clock::now());
        return HttpResponse::json(202, slice_state_json(session->snapshot()));
    }
}

// Stream the current G-code. The published snapshot owns the result
// directory, so a transfer already started keeps its file even if the session
// is mutated, resliced or deleted meanwhile. A read of the session: no 503.
HttpResponse handle_get_gcode(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The get_gcode route does not accept a request body.", request_id);
    std::optional<std::string> requested_name;
    if (!read_file_name_query(request.query, requested_name))
        return make_error_response(400, "invalid_request", "The get_gcode query is invalid.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const std::shared_ptr<const ViewerSnapshot> result = session->current_result();
    if (result == nullptr) {
        // A deletion between lookup and read reports the missing session, not a missing result.
        if (session->snapshot().retired)
            return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
        return make_error_response(409, "no_gcode", "No current G-code is available.", request_id);
    }
    // The name is cosmetic: reading it apart from the result is harmless.
    const std::optional<SessionData> data = session->data_snapshot();
    HttpResponse response;
    response.content_type = "text/x.gcode";
    response.body_file = result->gcode_file;
    response.body_file_owner = result;
    response.headers.emplace("Content-Disposition", content_disposition_attachment(
        download_file_name(requested_name, data ? data->name : std::nullopt, ".gcode")));
    response.headers.emplace("Cache-Control", "no-store");
    return response;
}

// Parse a decimal query value made of digits only (no sign, no space),
// rejecting values that do not fit.
bool parse_query_count(const std::string &text, std::size_t &value)
{
    if (text.empty() || text.size() > 19)
        return false;
    std::uint64_t parsed = 0;
    for (char character : text) {
        if (character < '0' || character > '9')
            return false;
        parsed = parsed * 10 + static_cast<std::uint64_t>(character - '0');
    }
    if (parsed > std::numeric_limits<std::size_t>::max())
        return false;
    value = static_cast<std::size_t>(parsed);
    return true;
}

struct ViewerQuery {
    std::size_t offset = 0;
    std::size_t limit = 10000;
    std::optional<std::string> result_id;
};

// Accepted keys are offset, limit (1..100000) and result_id, each at most
// once; a later page (offset > 0) must name the result it continues.
bool read_viewer_query(const std::string &query, ViewerQuery &parsed)
{
    std::map<std::string, std::string> parameters;
    if (!parse_form_query(query, parameters))
        return false;
    for (const std::pair<const std::string, std::string> &parameter : parameters) {
        if (parameter.first == "offset") {
            if (!parse_query_count(parameter.second, parsed.offset))
                return false;
        } else if (parameter.first == "limit") {
            if (!parse_query_count(parameter.second, parsed.limit) || parsed.limit < 1 || parsed.limit > 100000)
                return false;
        } else if (parameter.first == "result_id") {
            if (parameter.second.empty())
                return false;
            parsed.result_id = parameter.second;
        } else {
            return false;
        }
    }
    return parsed.offset == 0 || parsed.result_id.has_value();
}

// Runs on an import thread: a page of up to 100000 moves is built and
// serialized away from the network thread, from the captured immutable
// snapshot, so a page never mixes two results.
HttpResponse viewer_page_response(const ViewerSnapshot &snapshot, const ViewerQuery &query,
    const std::string &request_id)
{
    try {
        const ViewerPage page = make_viewer_page(*snapshot.viewer, query.offset, query.limit);
        const std::size_t end = std::min(page.total_moves, query.offset + page.moves.size());
        const ViewerHeader empty_header{nlohmann::json::object(), nlohmann::json::array(), {}};
        const ViewerHeader &header = snapshot.header != nullptr ? *snapshot.header : empty_header;
        return HttpResponse::json(200, nlohmann::json{
            {"schema_version", 1},
            {"result_id", snapshot.result_id},
            {"outdated", snapshot.outdated},
            {"metadata", header.metadata},
            {"layers", header.layers},
            {"moves", viewer_moves_json(page, header)},
            {"pagination", {
                {"offset", query.offset},
                {"limit", query.limit},
                {"total_moves", page.total_moves},
                {"next_offset", query.offset < page.total_moves && end < page.total_moves ?
                    nlohmann::json(end) : nlohmann::json(nullptr)}}}});
    } catch (const std::exception &) {
        return make_error_response(500, "internal_error", "The server could not build the viewer page.", request_id);
    }
}

// Read one page of the last retained result, current or outdated. A read of
// the session: no 503.
std::optional<HttpResponse> handle_get_gcode_for_view(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id, const DeferredResponder &respond)
{
    if (request.method != "GET")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The get_gcode_for_view route does not accept a request body.", request_id);
    ViewerQuery query;
    if (!read_viewer_query(request.query, query))
        return make_error_response(400, "invalid_request", "The get_gcode_for_view query is invalid.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    std::shared_ptr<const ViewerSnapshot> snapshot = session->viewer_snapshot();
    if (snapshot == nullptr) {
        // A deletion between lookup and read reports the missing session.
        if (session->snapshot().retired)
            return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
        return make_error_response(409, "no_viewer_result", "No slicing result has been retained.", request_id);
    }
    if (query.result_id && *query.result_id != snapshot->result_id)
        return make_error_response(409, "result_replaced", "The requested result has been replaced.", request_id);
    asio::post(services.imports, [snapshot = std::move(snapshot), query, request_id, respond,
        observer = services.import_observer]() {
        try {
            if (observer)
                observer();
            respond(viewer_page_response(*snapshot, query, request_id));
        } catch (...) {
            respond(make_error_response(500, "internal_error", "The server could not build the viewer page.", request_id));
        }
    });
    return std::nullopt;
}

// Request that the session's slicing stop. Queued work ends at once in
// `ready`; started work keeps `slicing` until the engine stops. A repeated
// request during the stop is accepted again. Without active work: 409.
HttpResponse handle_cancel_slicing(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "POST")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The cancel_slicing route does not accept a request body.", request_id);
    const std::shared_ptr<SessionContext> session = services.sessions.find(
        username, request.path_segments[1], SessionRegistry::Clock::now());
    if (session == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (session->request_cancel(SessionRegistry::Clock::now()) == SessionContext::CancelResult::not_active)
        return make_error_response(409, "not_slicing", "No slicing operation is active.", request_id);
    return HttpResponse::json(202, slice_state_json(session->snapshot()));
}

// Retained downloads survive retirement; synchronous DELETE cleanup may occupy the request thread in V1.
HttpResponse handle_delete_session(const HttpRequest &request, const RouteServices &services,
    const std::string &username, const std::string &request_id)
{
    if (request.method != "DELETE")
        return make_error_response(404, "route_not_found", "The route does not exist.", request_id);
    if (!request.body.empty())
        return make_error_response(400, "invalid_request", "The delete route does not accept a request body.", request_id);
    // Apply expiration first so a late DELETE gets 404 before erase arbitrates busy versus erased.
    if (services.sessions.find(username, request.path_segments[1], SessionRegistry::Clock::now()) == nullptr)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    const SessionEraseResult result = services.sessions.erase(username, request.path_segments[1]);
    if (result == SessionEraseResult::not_found)
        return make_error_response(404, "session_not_found", "The session does not exist.", request_id);
    if (result == SessionEraseResult::busy)
        return make_error_response(409, "session_busy", "The session is currently slicing.", request_id);
    return HttpResponse::empty(204);
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
        return handle_create_session(request, services, authentication->username, request_id, respond);

    // Session routes match decoded segments and resolve the owning account before
    // their handler reads or mutates the session.
    if (request.path_segments.size() >= 3 && request.path_segments[0] == "session") {
        const std::string &operation = request.path_segments[2];
        if (request.path_segments.size() == 3 && operation == "list_profiles")
            return handle_list_profiles(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "set_profile")
            return handle_set_profile(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "add_model")
            return handle_add_model(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 4 && operation == "add_part")
            return handle_add_part(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "load_project")
            return handle_load_project(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "save_project")
            return handle_save_project(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "slice")
            return handle_slice(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "get_gcode_for_view")
            return handle_get_gcode_for_view(request, services, authentication->username, request_id, respond);
        if (request.path_segments.size() == 3 && operation == "get_gcode")
            return handle_get_gcode(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "cancel_slicing")
            return handle_cancel_slicing(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "state")
            return handle_state(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "delete")
            return handle_delete_session(request, services, authentication->username, request_id);
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
        if (request.path_segments.size() == 4 && operation == "set_part_type")
            return handle_set_part_type(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "rename")
            return handle_rename(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "delete_entity")
            return handle_delete_entity(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "add_shape")
            return handle_add_shape(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 4 && operation == "split")
            return handle_split(request, services, authentication->username, request_id);
        if (request.path_segments.size() == 3 && operation == "merge")
            return handle_merge(request, services, authentication->username, request_id);
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
        std::function<void(std::function<void()>)> write_observer,
        CloseCallback close_callback)
        : socket_(std::move(socket)),
          config_(std::move(config)),
          router_(std::move(router)),
          write_observer_(std::move(write_observer)),
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
            file_serializer_.emplace(*file_response_message_);
            file_serializer_->limit(64 * 1024);
            if (write_observer_) {
                // Retain the open file and owner while a test delays the first
                // write. A weak continuation cannot outlive server shutdown.
                const std::weak_ptr<HttpSession> weak = shared_from_this();
                write_observer_([weak]() {
                    if (const auto session = weak.lock())
                        asio::post(session->socket_.get_executor(), [session]() {
                            if (!session->closed_) {
                                session->arm_timeout(session->config_->request_timeout);
                                session->write_file_chunk();
                            }
                        });
                });
                return;
            }
            arm_timeout(config_->request_timeout);
            write_file_chunk();
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

    // Each progressing write gets a fresh idle budget; a blocked socket still expires.
    void write_file_chunk()
    {
        const std::shared_ptr<HttpSession> self = shared_from_this();
        http::async_write_some(socket_, *file_serializer_, [self](const ErrorCode &error, std::size_t bytes) {
            if (error || self->closed_ || self->file_serializer_->is_done()) {
                self->handle_write(error, bytes);
                return;
            }
            if (bytes != 0)
                self->arm_timeout(self->config_->request_timeout);
            self->write_file_chunk();
        });
    }

    std::function<void(std::function<void()>)> write_observer_;

    void handle_write(const ErrorCode &error, std::size_t)
    {
        cancel_timeout();
        file_serializer_.reset();
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
    std::optional<http::response_serializer<http::file_body>> file_serializer_;
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

        result_directory_ = config_.work_directory / "results";
        std::filesystem::create_directories(result_directory_, filesystem_error);
        if (filesystem_error)
            throw std::invalid_argument("paths.work: cannot create the result directory: " + filesystem_error.message());
        try {
            remove_stale_temporary_files(result_directory_, "rest-result-");
        } catch (const std::filesystem::filesystem_error &failure) {
            throw std::invalid_argument("paths.work: cannot list the result directory: " + failure.code().message());
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
        slice_pool_ = std::make_unique<asio::thread_pool>(config_.slice_threads);
        slice_jobs_.reset();
        running_.store(true);
        const std::uint64_t generation = ++run_generation_;
        accept_next(generation);
        schedule_expiration(generation);
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
        {
            std::lock_guard<std::mutex> lock(expiration_mutex_);
            expiration_timer_.cancel(ignored);
        }
        acceptor_.cancel(ignored);
        acceptor_.close(ignored);

        std::vector<std::shared_ptr<HttpSession>> sessions;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            sessions.assign(sessions_.begin(), sessions_.end());
        }
        for (const std::shared_ptr<HttpSession> &session : sessions)
            session->close();

        slice_jobs_.cancel();
        if (slice_pool_ != nullptr)
            slice_pool_->join();

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
        slice_pool_.reset();
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

    // Copied after startup so launch diagnostics do not borrow catalog storage.
    std::vector<std::string> startup_warnings() const
    {
        return profile_catalog_ == nullptr ? std::vector<std::string>() : profile_catalog_->warnings();
    }

    // Services addressed by the routes for one request; valid while the runtime
    // is running and the worker has not been joined.
    RouteServices route_services()
    {
        return RouteServices{
            *auth_service_,
            *session_registry_,
            profile_catalog_.get(),
            private_test_handler_,
            *import_pool_,
            import_observer_,
            scene_edit_observer_,
            config_.max_instances_per_object,
            export_directory_,
            *slice_pool_,
            slice_jobs_,
            slice_observer_,
            progress_observer_,
            result_directory_,
            config_.work_directory};
    }

    void set_import_observer(std::function<void()> observer)
    {
        if (running_.load())
            throw std::logic_error("The import observer must be set before the REST server starts");
        import_observer_ = std::move(observer);
    }

    void set_scene_edit_observer(std::function<void()> observer)
    {
        if (running_.load())
            throw std::logic_error("The scene-edit observer must be set before the REST server starts");
        scene_edit_observer_ = std::move(observer);
    }

    void set_write_observer(std::function<void(std::function<void()>)> observer)
    {
        if (running_.load())
            throw std::logic_error("The write observer must be set before the REST server starts");
        write_observer_ = std::move(observer);
    }

    void set_slice_observer(std::function<void()> observer)
    {
        if (running_.load())
            throw std::logic_error("The slice observer must be set before the REST server starts");
        slice_observer_ = std::move(observer);
    }

    void set_progress_observer(std::function<void(const std::string &)> observer)
    {
        if (running_.load())
            throw std::logic_error("The progress observer must be set before the REST server starts");
        progress_observer_ = std::move(observer);
    }

    SessionRegistry &session_registry() const
    {
        if (session_registry_ == nullptr)
            throw std::logic_error("REST sessions are only available while the server is running");
        return *session_registry_;
    }

private:
    // Serialize timer submission with stop so no sweep can enter the import pool after joining starts.
    void schedule_expiration(std::uint64_t generation)
    {
        std::lock_guard<std::mutex> lock(expiration_mutex_);
        if (!running_.load() || generation != run_generation_.load())
            return;
        expiration_timer_.expires_after(session_expiration_interval(config_.session_ttl));
        expiration_timer_.async_wait([this, generation](const ErrorCode &error) {
            {
                std::lock_guard<std::mutex> lock(expiration_mutex_);
                if (error || !running_.load() || generation != run_generation_.load())
                    return;
                SessionRegistry *registry = session_registry_.get();
                asio::post(*import_pool_, [registry]() { registry->expire(SessionRegistry::Clock::now()); });
            }
            schedule_expiration(generation);
        });
    }

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
                    write_observer_,
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
    std::filesystem::path result_directory_;
    std::shared_ptr<const ServerConfig> config_shared_;
    std::shared_ptr<AuthService> auth_service_;
    std::unique_ptr<ProfileCatalog> profile_catalog_;
    RestRouter router_;
    // Created in start(), released in stop() after the sessions are closed;
    // HttpSession borrows it.
    std::unique_ptr<SessionRegistry> session_registry_;
    RequestHandler private_test_handler_;
    std::function<void()> import_observer_;
    std::function<void()> scene_edit_observer_;
    std::function<void(std::function<void()>)> write_observer_;
    // Created in start(), joined in stop() before the network thread stops.
    std::unique_ptr<asio::thread_pool> import_pool_;
    std::unique_ptr<asio::thread_pool> slice_pool_;
    SliceJobs slice_jobs_;
    std::function<void()> slice_observer_;
    std::function<void(const std::string &)> progress_observer_;
    asio::io_context io_context_;
    Tcp::acceptor acceptor_;
    asio::steady_timer accept_retry_timer_{io_context_};
    asio::steady_timer expiration_timer_{io_context_};
    std::mutex expiration_mutex_;
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
    config.slice_threads = default_slice_threads;
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
    if (slice_threads == 0)
        throw std::invalid_argument("limits.slice_threads must be positive");
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
    const CommandLineOverrides &overrides,
    const std::optional<std::filesystem::path> &default_resources)
{
    ServerConfig config = ServerConfig::defaults();
    // Host defaults have the lowest priority; JSON and CLI still override them.
    if (default_resources)
        config.resource_directory = *default_resources;
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

std::vector<std::string> RestServer::startup_warnings() const
{
    return impl_->runtime_.startup_warnings();
}

SessionRegistry &RestServer::session_registry() const
{
    return impl_->runtime_.session_registry();
}

void RestServer::set_import_observer(std::function<void()> observer)
{
    impl_->runtime_.set_import_observer(std::move(observer));
}

void RestServer::set_scene_edit_observer(std::function<void()> observer)
{
    impl_->runtime_.set_scene_edit_observer(std::move(observer));
}

void RestServer::set_write_observer(std::function<void(std::function<void()>)> observer)
{
    impl_->runtime_.set_write_observer(std::move(observer));
}

void RestServer::set_slice_observer(std::function<void()> observer)
{
    impl_->runtime_.set_slice_observer(std::move(observer));
}

void RestServer::set_progress_observer(std::function<void(const std::string &)> observer)
{
    impl_->runtime_.set_progress_observer(std::move(observer));
}

} // namespace Slic3r::Rest
