// E2E clients use public HTTP routes only. Never access the session registry,
// engine results or mutation tokens here. Work-directory inventories are the
// sole state exception, for detecting leaked uploads, exports and results.
#pragma once

#include "rest_models_test_helpers.hpp"

#include <set>

namespace RestServerTest::E2E {

// Bounds detect stuck work, not normal slow execution under external CPU load.
inline constexpr std::chrono::seconds work_timeout{300};

// A real TCP exchange with a deadline even while a worker delays its response.
inline HttpResponseMessage exchange(const RestServerFixture &fixture, const std::string &token,
    http::verb method, const std::string &target, const std::string &body = {},
    const std::string &content_type = "application/json")
{
    asio::io_context context;
    beast::tcp_stream stream(context);
    stream.connect(Tcp::endpoint(asio::ip::make_address("127.0.0.1"), fixture.bound_port()));
    stream.expires_after(work_timeout);
    http::request<http::string_body> message(method, target, 11);
    message.set(http::field::host, "127.0.0.1");
    message.set(http::field::connection, "close");
    if (!token.empty())
        message.set(http::field::authorization, "Bearer " + token);
    if (!body.empty())
        message.set(http::field::content_type, content_type);
    message.body() = body;
    message.prepare_payload();
    beast::flat_buffer buffer;
    http::response_parser<http::string_body> parser;
    parser.body_limit(64 * 1024 * 1024);
    ErrorCode error;
    http::async_write(stream, message, [&](ErrorCode written, std::size_t) {
        if (written) {
            error = written;
            return;
        }
        http::async_read(stream, buffer, parser, [&](ErrorCode read, std::size_t) { error = read; });
    });
    context.run();
    REQUIRE_FALSE(error);
    return parser.release();
}

inline std::string login(const RestServerFixture &fixture, const std::string &user,
    const std::string &password = "secret")
{
    const HttpResponseMessage response = exchange(fixture, {}, http::verb::post, "/login",
        nlohmann::json{{"username", user}, {"password", password}}.dump());
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body()).at("access_token");
}

// Every action explicitly carries this account's token, never the fixture default.
struct Client {
    const RestServerFixture &fixture;
    std::string token;

    HttpResponseMessage get(const std::string &route) const
    { return exchange(fixture, token, http::verb::get, route); }
    HttpResponseMessage post(const std::string &route, const nlohmann::json &body = nullptr) const
    { return exchange(fixture, token, http::verb::post, route, body.is_null() ? "" : body.dump()); }
    HttpResponseMessage del(const std::string &route) const
    { return exchange(fixture, token, http::verb::delete_, route); }
    HttpResponseMessage upload(const std::string &route, const std::string &name, const std::string &bytes) const
    { return exchange(fixture, token, http::verb::post, route, make_multipart_body(name, bytes),
        "multipart/form-data; boundary=" + test_boundary); }
};

inline nlohmann::json json_ok(const HttpResponseMessage &response)
{
    INFO(response.body());
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Poll conditions, not fixed sleeps; the short pause only limits polling traffic.
template<class Predicate>
bool poll(Predicate predicate, std::chrono::seconds timeout = work_timeout)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + timeout;
    do {
        if (predicate())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

inline std::vector<nlohmann::json> wait_state(const Client &client, const std::string &route,
    const std::string &expected)
{
    std::vector<nlohmann::json> observed;
    REQUIRE(poll([&]() {
        observed.push_back(json_ok(client.get(route + "/state")));
        INFO(observed.back().dump());
        REQUIRE(observed.back().at("etat") != "failed");
        return observed.back().at("etat") == expected;
    }));
    return observed;
}

// Each page has an independent delta baseline; retain one result ID throughout.
inline nlohmann::json read_viewer(const Client &client, const std::string &route)
{
    nlohmann::json result = json_ok(client.get(route + "/get_gcode_for_view?limit=1000"));
    nlohmann::json page = result;
    nlohmann::json moves = nlohmann::json::array();
    const std::string id = result.at("result_id");
    for (;;) {
        REQUIRE(page.at("result_id") == id);
        REQUIRE(page.at("metadata") == result.at("metadata"));
        REQUIRE(page.at("layers") == result.at("layers"));
        nlohmann::json previous = nlohmann::json::object();
        for (const nlohmann::json &sparse : page.at("moves")) {
            for (auto field = sparse.begin(); field != sparse.end(); ++field)
                previous[field.key()] = field.value();
            REQUIRE(previous.size() == 19);
            moves.push_back(previous);
        }
        const nlohmann::json next = page.at("pagination").at("next_offset");
        if (next.is_null())
            break;
        REQUIRE(next.get<std::size_t>() == moves.size());
        REQUIRE_FALSE(page.at("moves").empty());
        page = json_ok(client.get(route + "/get_gcode_for_view?limit=1000&offset=" +
            std::to_string(next.get<std::size_t>()) + "&result_id=" + id));
    }
    REQUIRE(moves.size() == result.at("pagination").at("total_moves").get<std::size_t>());
    result["moves"] = std::move(moves);
    return result;
}

struct WorkInventory {
    std::set<std::string> uploads, exports, results, files;
    // A partial scan must not pass cleanup assertions, even if its sets are empty.
    std::error_code error;
};

// Include directories as well as files, so an empty leaked result also fails.
// before_increment is a test seam to remove an entry deterministically just
// before descent, exercising the same race as cancellation/expiration cleanup.
inline WorkInventory work_inventory(const RestServerFixture &fixture,
    const std::function<void(const std::filesystem::path &)> &before_increment = {})
{
    WorkInventory inventory;
    std::filesystem::recursive_directory_iterator iterator(fixture.work_directory(), inventory.error);
    const std::filesystem::recursive_directory_iterator end;
    while (!inventory.error && iterator != end) {
        const std::filesystem::directory_entry &entry = *iterator;
        const std::string name = entry.path().filename().string();
        // Both paths are absolute beneath the same root; no filesystem lookup
        // is needed to form a relative name while directories may disappear.
        const std::string relative = entry.path().lexically_relative(fixture.work_directory()).generic_string();
        if (name.rfind("rest-upload-", 0) == 0)
            inventory.uploads.insert(relative);
        if (name.rfind("rest-export-", 0) == 0)
            inventory.exports.insert(relative);
        if (name.rfind("rest-result-", 0) == 0)
            inventory.results.insert(relative);
        if (entry.is_regular_file(inventory.error))
            inventory.files.insert(relative);
        if (inventory.error)
            break;
        if (before_increment)
            before_increment(entry.path());
        iterator.increment(inventory.error);
    }
    return inventory;
}

} // namespace RestServerTest::E2E
