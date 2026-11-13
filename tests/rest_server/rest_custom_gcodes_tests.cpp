// REST custom G-code tests cover validation, list replacement, project persistence, races and slicing output.
#include <catch2/catch.hpp>

#include "custom_gcodes.hpp"
#include "rest_models_test_helpers.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;
using namespace Slic3r;
using namespace Slic3r::Rest;

namespace {

// Send one authenticated JSON mutation through the loopback REST fixture.
HttpResponseMessage post_json(const RestServerFixture &fixture, const std::string &target,
    const nlohmann::json &body)
{
    return fixture.request(http::verb::post, target, body.dump(), "application/json");
}

// Upload a cube and return its session route.
std::string session_with_cube(RestServerFixture &fixture)
{
    const std::string route = "/session/" + create_session(fixture);
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return route;
}

// Return the first object's part ID from the object listing.
std::string first_part_id(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_objects");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body()).at("objects").at(0).at("parts").at(0).at("id").get<std::string>();
}

// Commit session configuration options directly, as the settings routes would.
void set_options(const std::shared_ptr<SessionContext> &session,
    const std::vector<std::pair<std::string, std::string>> &options)
{
    SessionData data = session->data_snapshot().value();
    for (const std::pair<std::string, std::string> &option : options)
        data.configuration.set_deserialize_strict(option.first, option.second);
    const SessionContext::MutationToken token = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(token, std::move(data), true, SessionContext::Clock::now()));
}

// Install a two-extruder printer profile; the fixture restarts, so call it before creating sessions.
void install_dual_tool(RestServerFixture &fixture)
{
    fixture.stop();
    std::ofstream(fixture.profiles_directory() / "printer" / "dual-tool.ini")
        << "nozzle_diameter = 0.4,0.4\n"
        << "default_filament_profile = ,rest-filament-b\n";
    fixture.start();
}

// Return the ID of the named profile, or an empty string.
std::string profile_id(const nlohmann::json &profiles, const std::string &name)
{
    for (const nlohmann::json &profile : profiles)
        if (profile.at("name") == name)
            return profile.at("id").get<std::string>();
    return {};
}

// Create a session on the two-extruder profile holding one cube and return its route.
std::string dual_session_with_cube(RestServerFixture &fixture)
{
    const std::string route = "/session/" + create_session(fixture);
    const nlohmann::json profiles = nlohmann::json::parse(
        fixture.request(http::verb::get, route + "/list_profiles").body());
    const std::string printer = profile_id(profiles.at("printer"), "dual-tool");
    const std::string print = profile_id(profiles.at("print"), "rest-print");
    REQUIRE_FALSE(printer.empty());
    REQUIRE_FALSE(print.empty());
    REQUIRE(post_json(fixture, route + "/set_profile", {
        {"print", print}, {"printer", printer}, {"filaments", nlohmann::json::array()},
        {"extruder", nlohmann::json::array()}
    }).result() == http::status::ok);
    REQUIRE(object_ids(upload_model(fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    return route;
}

// Return the JSON body of a successful GET.
nlohmann::json get_custom_gcodes(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_custom_gcodes");
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Return the JSON body of a successful set.
nlohmann::json set_custom_gcodes(const RestServerFixture &fixture, const std::string &route,
    const nlohmann::json &items)
{
    const HttpResponseMessage response = post_json(fixture, route + "/set_custom_gcodes", {{"items", items}});
    INFO("Response: " << response.body());
    REQUIRE(response.result() == http::status::ok);
    return nlohmann::json::parse(response.body());
}

// Return the error object of a refused set after checking status and code.
nlohmann::json set_error(const RestServerFixture &fixture, const std::string &route, const nlohmann::json &body,
    http::status status, const std::string &code)
{
    const HttpResponseMessage response = post_json(fixture, route + "/set_custom_gcodes", body);
    INFO("Response: " << response.body());
    require_error(response, status, code);
    return nlohmann::json::parse(response.body()).at("error");
}

// Return true when the warnings array of a response holds the reason (and item, when given).
bool has_warning(const nlohmann::json &json, const std::string &reason, std::optional<std::size_t> item = {})
{
    for (const nlohmann::json &warning : json.at("warnings"))
        if (warning.at("reason") == reason && warning.contains("item") == item.has_value() &&
            (!item || warning.at("item") == item.value()))
            return true;
    return false;
}

// Wait for a session to publish the requested slicing terminal state.
bool wait_for_status(const std::shared_ptr<SessionContext> &session, SessionStatus status)
{
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        const SessionStatus current = session->snapshot().status;
        if (current == status)
            return true;
        if (current == SessionStatus::failed)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return session->snapshot().status == status;
}

// Slice one prepared session and return its current G-code text.
std::string slice_gcode(const RestServerFixture &fixture, const std::string &route,
    const std::shared_ptr<SessionContext> &session)
{
    REQUIRE(fixture.request(http::verb::post, route + "/slice").result() == http::status::accepted);
    INFO("Slice error: " << session->snapshot().error_message);
    REQUIRE(wait_for_status(session, SessionStatus::sliced));
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/get_gcode");
    REQUIRE(response.result() == http::status::ok);
    return response.body();
}

// Count the lines of the text that equal the given line exactly.
std::size_t count_lines(const std::string &text, const std::string &line)
{
    std::size_t count = 0;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find('\n', start);
        if (text.compare(start, end == std::string::npos ? std::string::npos : end - start, line) == 0)
            ++count;
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return count;
}

// Return the G-code of the layer printed at the given Z, from its Z marker to the next layer change.
std::string layer_block(const std::string &gcode, const std::string &z)
{
    const std::size_t begin = gcode.find(";Z:" + z + "\n");
    REQUIRE(begin != std::string::npos);
    return gcode.substr(begin, gcode.find(";LAYER_CHANGE", begin) - begin);
}

// Return the position of the first extrusion move after the given position.
std::size_t first_extrusion(const std::string &gcode, std::size_t from)
{
    static const std::regex extrusion("\nG1 X[^\n]* E[0-9.]+\n");
    std::smatch match;
    const std::string tail = gcode.substr(from);
    REQUIRE(std::regex_search(tail, match, extrusion));
    return from + match.position(0);
}

} // namespace

// Verify 400 shape errors, 422 value errors with details, 404 and 401.
TEST_CASE("REST custom G-codes validate requests", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const nlohmann::json valid_item = {{"type", "template"}, {"z", 5.}};

    for (const nlohmann::json &body : std::vector<nlohmann::json>{
            nlohmann::json::array(),
            {{"items", nlohmann::json::array()}, {"extra", 1}},
            {{"nothing", 1}},
            {{"items", "list"}},
            {{"items", {1}}},
            {{"items", {{{"z", 5.}}}}},
            {{"items", {{{"type", 3}, {"z", 5.}}}}},
            {{"items", {{{"type", "template"}}}}},
            {{"items", {{{"type", "template"}, {"z", "5"}}}}},
            {{"items", {{{"type", "template"}, {"z", 5.}, {"gcode", "M117"}}}}},
            {{"items", {{{"type", "pause"}, {"z", 5.}, {"gcode", "M117"}}}}},
            {{"items", {{{"type", "pause"}, {"z", 5.}}}}},
            {{"items", {{{"type", "pause"}, {"z", 5.}, {"message", 4}}}}},
            {{"items", {{{"type", "custom"}, {"z", 5.}}}}},
            {{"items", {{{"type", "custom"}, {"z", 5.}, {"message", "x"}}}}},
            {{"items", {{{"type", "color_change"}, {"z", 5.}, {"extruder", 1.5}}}}},
            {{"items", {{{"type", "color_change"}, {"z", 5.}, {"color", 5}}}}},
            {{"items", {{{"type", "color_change"}, {"z", 5.}, {"message", "x"}}}}},
            {{"items", {{{"type", "tool_change"}, {"z", 5.}}}}},
            {{"items", {{{"type", "bogus"}}}}},
            {{"items", {{{"type", "bogus"}, {"z", "5"}}}}},
            {{"items", {{{"type", "color_change"}, {"z", 5.}, {"extruder", 18446744073709551615ULL}}}}},
            {{"items", {{{"type", "color_change"}, {"z", 5.}, {"extruder", -3000000000LL}}}}},
            {{"items", {valid_item, {{"type", "template"}, {"z", 6.}, {"extruder", 1}}}}}}) {
        INFO("Body: " << body.dump());
        const HttpResponseMessage response = post_json(fixture, route + "/set_custom_gcodes", body);
        INFO("Response: " << response.body());
        require_error(response, http::status::bad_request, "invalid_request");
    }
    require_error(fixture.request(http::verb::post, route + "/set_custom_gcodes", "{", "application/json"),
        http::status::bad_request, "invalid_json");
    require_error(fixture.request(http::verb::post, route + "/set_custom_gcodes"), http::status::bad_request,
        "invalid_request");

    // Value errors name the failing item and a stable reason.
    const std::vector<std::pair<nlohmann::json, std::pair<std::size_t, std::string>>> refusals = {
        {{{"items", {{{"type", "bogus"}, {"z", 5.}}}}}, {0, "unknown_type"}},
        {{{"items", {valid_item, {{"type", "bogus"}, {"z", 6.}}}}}, {1, "unknown_type"}},
        {{{"items", {{{"type", "template"}, {"z", 0.}}}}}, {0, "invalid_z"}},
        {{{"items", {valid_item, {{"type", "template"}, {"z", -2.}}}}}, {1, "invalid_z"}},
        {{{"items", {valid_item, {{"type", "template"}, {"z", 100.}}}}}, {1, "above_objects"}},
        {{{"items", {valid_item, valid_item}}}, {1, "duplicate_z"}},
        {{{"items", {{{"type", "tool_change"}, {"z", 5.}, {"extruder", 1}}}}}, {0, "mode_tool_change"}},
        {{{"items", {{{"type", "color_change"}, {"z", 5.}, {"extruder", 2}}}}}, {0, "extruder_range"}},
        {{{"items", {{{"type", "color_change"}, {"z", 5.}, {"color", "blue"}}}}}, {0, "invalid_color"}},
        {{{"items", {{{"type", "pause"}, {"z", 5.}, {"message", ""}}}}}, {0, "empty_message"}},
        {{{"items", {{{"type", "custom"}, {"z", 5.}, {"gcode", ""}}}}}, {0, "empty_gcode"}}};
    for (const std::pair<nlohmann::json, std::pair<std::size_t, std::string>> &refusal : refusals) {
        INFO("Body: " << refusal.first.dump());
        const nlohmann::json error = set_error(fixture, route, refusal.first, http::status::unprocessable_entity,
            "invalid_custom_gcode");
        REQUIRE(error.at("details").size() == 1);
        CHECK(error.at("details").at(0) == nlohmann::json({{"item", refusal.second.first},
            {"reason", refusal.second.second}}));
    }

    // Sequential printing refuses a non-empty list but accepts an empty one.
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    set_options(session, {{"print_plan_builder", "ordering.plan_builder.by_object"}});
    const nlohmann::json sequential = set_error(fixture, route, {{"items", {valid_item}}},
        http::status::unprocessable_entity, "invalid_custom_gcode");
    CHECK_FALSE(sequential.contains("details"));
    set_custom_gcodes(fixture, route, nlohmann::json::array());

    // Rejected requests leave the list untouched; reads need a body-less GET.
    CHECK(get_custom_gcodes(fixture, route).at("items").empty());
    require_error(fixture.request(http::verb::get, route + "/get_custom_gcodes", "{}", "application/json"),
        http::status::bad_request, "invalid_request");
    require_error(fixture.request(http::verb::get, route + "/get_custom_gcodes?x=1"), http::status::bad_request,
        "invalid_request");
    require_error(fixture.request(http::verb::post, route + "/get_custom_gcodes"), http::status::not_found,
        "route_not_found");
    require_error(fixture.request(http::verb::get, route + "/set_custom_gcodes"), http::status::not_found,
        "route_not_found");
    require_error(fixture.request(http::verb::get, "/session/unknown/get_custom_gcodes"), http::status::not_found,
        "session_not_found");
    require_error(post_json(fixture, "/session/unknown/set_custom_gcodes", {{"items", nlohmann::json::array()}}),
        http::status::not_found, "session_not_found");
    CHECK(fixture.request_without_auth(http::verb::get, route + "/get_custom_gcodes").result() ==
        http::status::unauthorized);
    CHECK(fixture.request_without_auth(http::verb::post, route + "/set_custom_gcodes", "{}", "application/json")
        .result() == http::status::unauthorized);
}

// Verify full replacement, sorting, defaults, unchanged revisions, warnings and the current mode.
TEST_CASE("REST custom G-codes replace the whole list", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    set_options(session, {{"color_change_gcode", "M600"}, {"pause_print_gcode", "M601"},
        {"template_custom_gcode", "M117 template"}});
    const std::uint64_t fresh_revision = session->snapshot().revision;
    CHECK(set_custom_gcodes(fixture, route, nlohmann::json::array()).at("items").empty());
    CHECK(session->snapshot().revision == fresh_revision);

    const nlohmann::json first_items = {
        {{"type", "pause"}, {"z", 10.}, {"message", "Insert nuts"}},
        {{"type", "color_change"}, {"z", 5.}, {"color", "#ff0000"}},
        {{"type", "custom"}, {"z", 15.}, {"gcode", "M117 Hello"}},
        {{"type", "template"}, {"z", 12.5}},
        {{"type", "color_change"}, {"z", 18.}}};
    const nlohmann::json first = set_custom_gcodes(fixture, route, first_items);
    CHECK(first.at("mode") == "single_extruder");
    CHECK(first.at("current_mode") == "single_extruder");
    CHECK(first.at("warnings").empty());
    REQUIRE(first.at("items").size() == 5);
    CHECK(first.at("items").at(0) == nlohmann::json({{"z", 5.}, {"type", "color_change"}, {"extruder", 1},
        {"color", "#FF0000"}}));
    CHECK(first.at("items").at(1) == nlohmann::json({{"z", 10.}, {"type", "pause"}, {"message", "Insert nuts"}}));
    CHECK(first.at("items").at(2) == nlohmann::json({{"z", 12.5}, {"type", "template"}}));
    CHECK(first.at("items").at(3) == nlohmann::json({{"z", 15.}, {"type", "custom"}, {"gcode", "M117 Hello"}}));
    CHECK(first.at("items").at(4).at("type") == "color_change");
    CHECK(first.at("items").at(4).at("extruder") == 1);
    CHECK(get_custom_gcodes(fixture, route) == first);

    // The same list is not a new revision, whatever the order of its elements.
    const std::uint64_t revision = session->snapshot().revision;
    CHECK(set_custom_gcodes(fixture, route, first_items) == first);
    CHECK(session->snapshot().revision == revision);

    // A different list replaces the previous one completely and changes the revision.
    const nlohmann::json second = set_custom_gcodes(fixture, route, {{{"type", "template"}, {"z", 6.}}});
    CHECK(second.at("items") == nlohmann::json::array({{{"z", 6.}, {"type", "template"}}}));
    CHECK(session->snapshot().revision > revision);
    CHECK(get_custom_gcodes(fixture, route) == second);

    // An empty printer G-code is accepted and reported.
    set_options(session, {{"template_custom_gcode", ""}});
    const nlohmann::json warned = get_custom_gcodes(fixture, route);
    CHECK(has_warning(warned, "empty_printer_gcode", 0));
    CHECK(warned.at("warnings").size() == 1);
    set_options(session, {{"pause_print_gcode", ""}, {"gcode_flavor", "mach3"}});
    const nlohmann::json pause_warned = set_custom_gcodes(fixture, route,
        {{{"type", "pause"}, {"z", 6.}, {"message", "Wait"}}});
    CHECK(has_warning(pause_warned, "empty_printer_gcode", 0));

    // Emptying the list is a change once, then a no-op.
    const nlohmann::json emptied = set_custom_gcodes(fixture, route, nlohmann::json::array());
    CHECK(emptied.at("items").empty());
    CHECK(emptied.at("warnings").empty());
    const std::uint64_t emptied_revision = session->snapshot().revision;
    CHECK(set_custom_gcodes(fixture, route, nlohmann::json::array()) == emptied);
    CHECK(session->snapshot().revision == emptied_revision);

    // A scene that shrinks below an element keeps it but reports it.
    set_custom_gcodes(fixture, route, {{{"type", "template"}, {"z", 15.}}});
    set_options(session, {{"template_custom_gcode", "M117 template"}});
    const std::string part_id = first_part_id(fixture, route);
    REQUIRE(post_json(fixture, route + "/transform/" + part_id, {{"matrix", {.5, 0., 0., 0., 0., .5, 0., 0., 0., 0., .5, 0.,
        0., 0., 0., 1.}}, {"space", "object"}, {"action", "replace"}, {"ensure_on_bed", true}})
        .result() == http::status::ok);
    CHECK(has_warning(get_custom_gcodes(fixture, route), "above_objects", 0));

    // The current mode follows the engine rule on every read, on a real two-extruder session.
    RestServerFixture dual_fixture({}, true, true);
    install_dual_tool(dual_fixture);
    const std::string dual_route = dual_session_with_cube(dual_fixture);
    const std::string dual_part = first_part_id(dual_fixture, dual_route);
    const nlohmann::json dual_set = set_custom_gcodes(dual_fixture, dual_route, {{{"type", "color_change"}, {"z", 5.}}});
    CHECK(dual_set.at("mode") == "multi_as_single");
    CHECK(dual_set.at("current_mode") == "multi_as_single");

    // A part on extruder 2 makes two extruders print (the fallback region keeps the default one):
    // the stored multi_as_single list now has color changes the engine ignores.
    REQUIRE(dual_fixture.request(http::verb::post, dual_route + "/set_entity_setting/" + dual_part + "/extruder/2")
        .result() == http::status::ok);
    const nlohmann::json dual_get = get_custom_gcodes(dual_fixture, dual_route);
    CHECK(dual_get.at("mode") == "multi_as_single");
    CHECK(dual_get.at("current_mode") == "multi_extruder");
    CHECK(has_warning(dual_get, "mode_mismatch"));

    // A layer range reaching 10000 mm must not trip the engine rounding assert when the mode is computed.
    const nlohmann::json dual_scene = nlohmann::json::parse(
        dual_fixture.request(http::verb::get, dual_route + "/list_objects").body());
    const std::string dual_object = dual_scene.at("objects").at(0).at("id").get<std::string>();
    REQUIRE(post_json(dual_fixture, dual_route + "/add_layer_range/" + dual_object,
        {{"min", 0.}, {"max", 10000.}, {"layer_height", 0.2}, {"extruder", 1}}).result() == http::status::ok);
    CHECK(get_custom_gcodes(dual_fixture, dual_route).at("current_mode") == "multi_extruder");

    // Tool changes depend on the mode recomputed at the call.
    set_error(dual_fixture, dual_route, {{"items", {{{"type", "tool_change"}, {"z", 5.}, {"extruder", 2}}}}},
        http::status::unprocessable_entity, "invalid_custom_gcode");
    const nlohmann::json multi = set_custom_gcodes(dual_fixture, dual_route,
        {{{"type", "color_change"}, {"z", 5.}, {"extruder", 2}, {"color", "#00ff00"}}});
    CHECK(multi.at("mode") == "multi_extruder");
    CHECK(multi.at("items").at(0).at("extruder") == 2);
    CHECK_FALSE(has_warning(multi, "mode_mismatch"));
}

// Verify save_project and load_project keep the list and its mode.
TEST_CASE("REST custom G-codes persist in projects", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const nlohmann::json before = set_custom_gcodes(fixture, route, {
        {{"type", "color_change"}, {"z", 5.}, {"color", "#123456"}},
        {{"type", "pause"}, {"z", 10.}, {"message", "Insert nuts"}},
        {{"type", "custom"}, {"z", 15.}, {"gcode", "M117 Hello"}},
        {{"type", "template"}, {"z", 7.}}});
    REQUIRE(before.at("items").size() == 4);

    const HttpResponseMessage saved = fixture.request(http::verb::get, route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    const std::string restored_route = "/session/" + create_session(fixture);
    REQUIRE(get_custom_gcodes(fixture, restored_route).at("items").empty());
    REQUIRE(object_ids(upload_model(fixture, restored_route + "/load_project", "gcodes.3mf", saved.body())).size() == 1);
    CHECK(get_custom_gcodes(fixture, restored_route) == before);
}

// Verify slicing refusal, reads during slicing, and a concurrent change during a synchronous set.
TEST_CASE("REST custom G-codes are refused while slicing and on concurrent changes", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    const nlohmann::json body = {{"items", {{{"type", "template"}, {"z", 5.}}}}};

    const SessionContext::WorkToken work = session->begin_work(SessionContext::Clock::now()).value();
    require_error(post_json(fixture, route + "/set_custom_gcodes", body), http::status::conflict, "session_busy");
    CHECK(get_custom_gcodes(fixture, route).at("items").empty());
    REQUIRE(session->cancel_work(work, SessionContext::Clock::now()));
    set_custom_gcodes(fixture, route, body.at("items"));

    fixture.stop();
    const std::shared_ptr<std::shared_ptr<SessionContext>> observed =
        std::make_shared<std::shared_ptr<SessionContext>>();
    fixture.set_scene_edit_observer([observed]() {
        const std::shared_ptr<SessionContext> current = *observed;
        if (!current)
            return;
        const std::optional<SessionContext::MutationToken> token = current->begin_mutation();
        if (!token)
            return;
        SessionData data = current->data_snapshot().value();
        data.preferences["custom_gcode_observer"] = true;
        current->commit_mutation(*token, std::move(data), false, SessionContext::Clock::now());
    });
    fixture.start();
    const std::string observed_route = session_with_cube(fixture);
    *observed = find_session(fixture, observed_route.substr(9));
    require_error(post_json(fixture, observed_route + "/set_custom_gcodes", body), http::status::conflict,
        "session_changed");
    CHECK(get_custom_gcodes(fixture, observed_route).at("items").empty());
}

// Verify configured and firmware-default pauses and ordered markers at their target layers.
TEST_CASE("REST custom G-codes reach the G-code", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string route = session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    bool firmware_default = false;
    SECTION("Configured pause script displays its message and pauses") {}
    SECTION("Empty pause script uses the selected firmware") { firmware_default = true; }
    set_options(session, {{"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"color_change_gcode", "M600"},
        {"pause_print_gcode", firmware_default ? "" : "M117 {pause_message}\\nM601"},
        {"gcode_firmware_plugin", "gcode.firmware.marlin2"}, {"seam_visibility", "0"}});
    set_custom_gcodes(fixture, route, {
        {{"type", "color_change"}, {"z", 5.}},
        {{"type", "pause"}, {"z", 10.}, {"message", "REST pause"}},
        {{"type", "custom"}, {"z", 15.}, {"gcode", "M117 REST custom"}},
        {{"type", "custom"}, {"z", 7.05}, {"gcode", "M117 REST low"}},
        {{"type", "custom"}, {"z", 7.1}, {"gcode", "M117 REST high"}}});
    const std::string gcode = slice_gcode(fixture, route, session);

    // The color change comes after the layer change and before the first extrusion of layer 5.
    const std::size_t color_layer = gcode.find(";Z:5\n");
    REQUIRE(color_layer != std::string::npos);
    CHECK(count_lines(gcode, "M600") == 1);
    const std::size_t color_change = gcode.find("\nM600\n");
    REQUIRE(color_change != std::string::npos);
    CHECK(color_change > color_layer);
    CHECK(color_change < first_extrusion(gcode, color_layer));
    CHECK(gcode.find(";COLOR_CHANGE,T0,") < color_change);

    const std::string pause_layer = layer_block(gcode, "10");
    CHECK(count_lines(gcode, "M117 REST pause") == (firmware_default ? 0 : 1));
    CHECK(count_lines(gcode, "M601") == (firmware_default ? 0 : 1));
    CHECK(count_lines(pause_layer, "M117 REST pause") == (firmware_default ? 0 : 1));
    CHECK(count_lines(pause_layer, "M601") == (firmware_default ? 0 : 1));
    CHECK(count_lines(gcode, "M0 REST pause") == (firmware_default ? 1 : 0));
    CHECK(count_lines(pause_layer, "M0 REST pause") == (firmware_default ? 1 : 0));

    CHECK(count_lines(gcode, "M117 REST custom") == 1);
    CHECK(count_lines(layer_block(gcode, "15"), "M117 REST custom") == 1);

    // Both elements fall in layer 7.2 (7.0 < z <= 7.2), in increasing source height order.
    CHECK(count_lines(gcode, "M117 REST low") == 1);
    CHECK(count_lines(gcode, "M117 REST high") == 1);
    const std::string shared_layer = layer_block(gcode, "7.2");
    REQUIRE(count_lines(shared_layer, "M117 REST low") == 1);
    REQUIRE(count_lines(shared_layer, "M117 REST high") == 1);
    CHECK(shared_layer.find("M117 REST low") < shared_layer.find("M117 REST high"));
}

// Verify a tool change on a two-extruder printer that prints the whole model with one extruder,
// and that it applies in addition to a custom G-code of the same layer.
TEST_CASE("REST custom G-code tool changes reach the G-code", "[rest_custom_gcode]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    install_dual_tool(fixture);
    const std::string route = dual_session_with_cube(fixture);
    const std::shared_ptr<SessionContext> session = find_session(fixture, route.substr(9));
    set_options(session, {{"layer_height", "0.2"}, {"first_layer_height", "0.2"}, {"seam_visibility", "0"}});
    const nlohmann::json stored = set_custom_gcodes(fixture, route, {
        {{"type", "tool_change"}, {"z", 10.}, {"extruder", 2}},
        {{"type", "custom"}, {"z", 9.95}, {"gcode", "M117 REST same layer"}}});
    CHECK(stored.at("mode") == "multi_as_single");
    const std::string gcode = slice_gcode(fixture, route, session);
    const std::string layer = layer_block(gcode, "10");
    CHECK(count_lines(layer, "T1") == 1);
    CHECK(count_lines(layer, "M117 REST same layer") == 1);
    CHECK(count_lines(gcode, "M117 REST same layer") == 1);
}
