// Real slices and bounded worker gates verify asynchronous state and result ownership.

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include "progress.hpp"
#include "rest_models_test_helpers.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace RestServerTest;

namespace {

constexpr std::chrono::seconds async_timeout{60};
constexpr std::chrono::seconds gate_timeout{10};

struct SessionRoute {
    std::string id;
    std::string route;
    std::shared_ptr<Slic3r::Rest::SessionContext> session;
};

// A worker waits for release only briefly, so failed assertions cannot hang fixture shutdown.
class ObserverGate {
public:
    explicit ObserverGate(std::uint32_t expected_entries)
        : expected_entries_(expected_entries),
          entries_ready_(entries_ready_promise_.get_future().share()),
          release_ready_(release_promise_.get_future().share())
    {
    }

    void enter()
    {
        const std::uint32_t entries = entry_count_.fetch_add(1) + 1;
        if (entries == expected_entries_)
            entries_ready_promise_.set_value();
        release_ready_.wait_for(gate_timeout);
    }

    bool wait_for_entries() const
    {
        return entries_ready_.wait_for(gate_timeout) == std::future_status::ready;
    }

    void release()
    {
        bool expected = false;
        if (released_.compare_exchange_strong(expected, true))
            release_promise_.set_value();
    }

    std::uint32_t entry_count() const
    {
        return entry_count_.load();
    }

private:
    std::uint32_t expected_entries_;
    std::atomic<std::uint32_t> entry_count_{0};
    std::promise<void> entries_ready_promise_;
    std::shared_future<void> entries_ready_;
    std::promise<void> release_promise_;
    std::shared_future<void> release_ready_;
    std::atomic<bool> released_{false};
};

// This guard is declared after the fixture so unwinding releases workers first.
class ObserverRelease {
public:
    explicit ObserverRelease(std::shared_ptr<ObserverGate> gate) : gate_(std::move(gate)) {}
    ~ObserverRelease() { gate_->release(); }

private:
    std::shared_ptr<ObserverGate> gate_;
};

bool wait_until(const std::function<bool()> &predicate)
{
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + async_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return predicate();
}

SessionRoute create_cube_session(RestServerFixture &fixture)
{
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    REQUIRE(object_ids(upload_model(
        fixture, route + "/add_model", "cube.stl", make_cube_stl())).size() == 1);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    REQUIRE(session != nullptr);
    return SessionRoute{id, route, session};
}

nlohmann::json response_json(const HttpResponseMessage &response)
{
    return nlohmann::json::parse(response.body());
}

void require_accepted_state(const nlohmann::json &state)
{
    REQUIRE(state.size() == 5);
    CHECK(state.at("etat") == "slicing");
    CHECK(state.at("primary_message") == "");
    CHECK(state.at("primary_percentage") == -1);
    CHECK(state.at("secondary_message") == "");
    CHECK(state.at("secondary_percentage") == -1);
}

bool wait_for_status(
    const std::shared_ptr<Slic3r::Rest::SessionContext> &session,
    Slic3r::Rest::SessionStatus expected)
{
    return wait_until([&]() {
        return session->snapshot().status == expected;
    });
}

void run_successful_slice(const RestServerFixture &fixture, const SessionRoute &session_route)
{
    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::sliced));
}

std::vector<std::filesystem::path> result_directories(const RestServerFixture &fixture)
{
    std::vector<std::filesystem::path> directories;
    const std::filesystem::path results = fixture.work_directory() / "results";
    std::error_code error;
    if (!std::filesystem::is_directory(results, error))
        return directories;
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(results))
        if (entry.is_directory() && entry.path().filename().string().rfind("rest-result-", 0) == 0)
            directories.push_back(entry.path());
    return directories;
}

bool wait_for_result_count(const RestServerFixture &fixture, std::size_t expected)
{
    return wait_until([&]() { return result_directories(fixture).size() == expected; });
}

bool contains_temporary_file(const RestServerFixture &fixture)
{
    const std::filesystem::path work = fixture.work_directory();
    std::error_code error;
    if (!std::filesystem::is_directory(work, error))
        return false;
    for (const std::filesystem::directory_entry &entry :
         std::filesystem::recursive_directory_iterator(work)) {
        if (entry.is_regular_file() && entry.path().filename().string().find(".tmp") != std::string::npos)
            return true;
    }
    return false;
}

nlohmann::json identity_transform()
{
    return {
        {"matrix", {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}},
        {"space", "platter"}
    };
}

} // namespace

TEST_CASE("REST progress helper maps producer updates without session state", "[rest_state]")
{
    TempDirectory root;
    const std::filesystem::path work = root.path() / "work";
    const std::filesystem::path directory = work / "results" / "rest-result-progress";

    const std::optional<Slic3r::Rest::ProgressUpdate> default_update = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(27, ""), directory, work);
    REQUIRE(default_update.has_value());
    CHECK_FALSE(default_update->secondary);
    CHECK(default_update->message.empty());
    CHECK(default_update->percentage == 27);

    const std::optional<Slic3r::Rest::ProgressUpdate> main_update = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(
            42, "Preparing print", Slic3r::PrintStatus::SlicingStatus::MAIN_STATE),
        directory,
        work);
    REQUIRE(main_update.has_value());
    CHECK_FALSE(main_update->secondary);
    CHECK(main_update->message == "Preparing print");
    CHECK(main_update->percentage == 42);

    const std::optional<Slic3r::Rest::ProgressUpdate> secondary_update = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(
            250, "Generating G-code layer %s / %s", {"3", "10"},
            Slic3r::PrintStatus::SlicingStatus::SECONDARY_STATE),
        directory,
        work);
    REQUIRE(secondary_update.has_value());
    CHECK(secondary_update->secondary);
    CHECK(secondary_update->message == "Generating G-code layer 3 / 10");
    CHECK(secondary_update->percentage == 100);

    const std::optional<Slic3r::Rest::ProgressUpdate> both_channels = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(
            INT_MIN, "Optimizing skirt & brim %s%%", {"45"},
            Slic3r::PrintStatus::SlicingStatus::MAIN_STATE |
                Slic3r::PrintStatus::SlicingStatus::SECONDARY_STATE),
        directory,
        work);
    REQUIRE(both_channels.has_value());
    CHECK(both_channels->secondary);
    CHECK(both_channels->message == "Optimizing skirt & brim 45%");
    CHECK(both_channels->percentage == -1);

    const std::optional<Slic3r::Rest::ProgressUpdate> positional = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(50, "Layer %1% of %2%", {"3", "10"}), directory, work);
    REQUIRE(positional.has_value());
    CHECK(positional->message == "Layer 3 of 10");

    const std::optional<Slic3r::Rest::ProgressUpdate> malformed = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(50, "Malformed %"), directory, work);
    REQUIRE(malformed.has_value());
    CHECK(malformed->message == "Malformed %");

    const std::optional<Slic3r::Rest::ProgressUpdate> sanitized = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(60, "Read %s", {(directory / "output.gcode").generic_u8string()}),
        directory, work);
    REQUIRE(sanitized.has_value());
    CHECK(sanitized->message == "Read <output>/output.gcode");
}

TEST_CASE("REST progress helper ignores every producer control event", "[rest_state]")
{
    using SlicingStatus = Slic3r::PrintStatus::SlicingStatus;
    TempDirectory root;
    const std::filesystem::path work = root.path() / "work";
    const std::filesystem::path directory = work / "results" / "rest-result-progress";
    const std::array<unsigned int, 8> control_flags{
        SlicingStatus::RELOAD_SCENE,
        SlicingStatus::RELOAD_SLA_SUPPORT_POINTS,
        SlicingStatus::RELOAD_SLA_PREVIEW,
        SlicingStatus::UPDATE_PRINT_STEP_WARNINGS,
        SlicingStatus::UPDATE_PRINT_OBJECT_STEP_WARNINGS,
        SlicingStatus::SLICING_ENDED,
        SlicingStatus::GCODE_ENDED,
        SlicingStatus::FORCE_SHOW};

    for (const unsigned int flag : control_flags) {
        CHECK_FALSE(Slic3r::Rest::progress_update(
            SlicingStatus(75, (directory / "output.gcode").u8string(), flag), directory, work).has_value());
        CHECK_FALSE(Slic3r::Rest::progress_update(
            SlicingStatus(75, (directory / "output.gcode").u8string(), flag | SlicingStatus::SECONDARY_STATE),
            directory,
            work).has_value());
    }
}

TEST_CASE("REST progress sanitization replaces overlapping UTF-8 output prefixes", "[rest_state]")
{
    TempDirectory root;
    const std::filesystem::path work = root.path() / std::filesystem::u8path("work-\xc3\xa9");
    const std::filesystem::path directory =
        work / std::filesystem::u8path("r\xc3\xa9sults") / "rest-result-secret";
    std::filesystem::create_directories(directory);
    const std::string message = "directory=" + directory.u8string() +
        ";directory-generic=" + directory.generic_u8string() +
        ";work=" + work.u8string() +
        ";work-generic=" + work.generic_u8string() +
        ";repeated=" + directory.u8string() + "|" + work.generic_u8string();

    CHECK(Slic3r::Rest::sanitized_slice_error(message, directory, work) ==
        "directory=<output>;directory-generic=<output>;work=<output>;work-generic=<output>;"
        "repeated=<output>|<output>");
    const std::optional<Slic3r::Rest::ProgressUpdate> update = Slic3r::Rest::progress_update(
        Slic3r::PrintStatus::SlicingStatus(50, "%s", {message}), directory, work);
    REQUIRE(update.has_value());
    CHECK(update->message ==
        "directory=<output>;directory-generic=<output>;work=<output>;work-generic=<output>;"
        "repeated=<output>|<output>");
}

TEST_CASE("REST state exposes session progress and clears terminal channels", "[rest_slice][rest_state]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    REQUIRE(session != nullptr);

    const nlohmann::json ready_expected{
        {"etat", "ready"}, {"primary_message", ""}, {"primary_percentage", -1},
        {"secondary_message", ""}, {"secondary_percentage", -1}};
    const HttpResponseMessage ready = fixture.request(http::verb::get, route + "/state");
    REQUIRE(ready.result() == http::status::ok);
    CHECK(response_json(ready) == ready_expected);

    // Inject deterministic channels because real producer percentages have no ordering guarantee.
    const Slic3r::Rest::SessionContext::WorkToken active = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->update_progress(active, false, "Slicing objects", 37));
    REQUIRE(session->update_progress(active, true, "Generating infill", 62));
    const nlohmann::json slicing_expected{
        {"etat", "slicing"}, {"primary_message", "Slicing objects"}, {"primary_percentage", 37},
        {"secondary_message", "Generating infill"}, {"secondary_percentage", 62}};
    const HttpResponseMessage slicing = fixture.request(http::verb::get, route + "/state");
    REQUIRE(slicing.result() == http::status::ok);
    CHECK(response_json(slicing) == slicing_expected);

    REQUIRE(session->cancel_work(active, Slic3r::Rest::SessionRegistry::Clock::now()));
    const HttpResponseMessage canceled = fixture.request(http::verb::get, route + "/state");
    REQUIRE(canceled.result() == http::status::ok);
    CHECK(response_json(canceled) == ready_expected);

    const Slic3r::Rest::SessionContext::WorkToken failing = session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->update_progress(failing, false, "Slicing objects", 75));
    REQUIRE(session->update_progress(failing, true, "Generating infill", 90));
    REQUIRE(session->fail_work(
        failing, "The server could not slice the project.", Slic3r::Rest::SessionRegistry::Clock::now()));
    const nlohmann::json failed_expected{
        {"etat", "failed"}, {"primary_message", "The server could not slice the project."},
        {"primary_percentage", -1}, {"secondary_message", ""}, {"secondary_percentage", -1}};
    const HttpResponseMessage failed = fixture.request(http::verb::get, route + "/state");
    REQUIRE(failed.result() == http::status::ok);
    CHECK(response_json(failed) == failed_expected);
}

TEST_CASE("REST state validates authentication route method body and ownership", "[rest_slice][rest_state]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id + "/state";

    require_error(
        fixture.request_without_auth(http::verb::get, route, "{", "application/json"),
        http::status::unauthorized,
        "unauthorized");
    require_error(
        fixture.request(http::verb::post, route),
        http::status::not_found,
        "route_not_found");
    require_error(
        fixture.request(http::verb::get, route, "{", "application/json"),
        http::status::bad_request,
        "invalid_request");

    const std::string bob_token = fixture.login_token("Bob", "secret");
    require_error(
        fixture.request(
            http::verb::get,
            route,
            {},
            {},
            {{"Authorization", "Bearer " + bob_token}}),
        http::status::not_found,
        "session_not_found");
    require_error(
        fixture.request(http::verb::get, "/session/missing/state"),
        http::status::not_found,
        "session_not_found");
    require_error(
        fixture.request(http::verb::get, route + "/extra"),
        http::status::not_found,
        "route_not_found");
}

TEST_CASE("REST completed cube slice reports default terminal progress", "[rest_slice][rest_state]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::sliced));

    const HttpResponseMessage state = fixture.request(http::verb::get, session_route.route + "/state");
    REQUIRE(state.result() == http::status::ok);
    const nlohmann::json expected{
        {"etat", "sliced"}, {"primary_message", ""}, {"primary_percentage", -1},
        {"secondary_message", ""}, {"secondary_percentage", -1}};
    CHECK(response_json(state) == expected);
}

TEST_CASE("REST real slice publishes readable sanitized progress outside the session lock", "[rest_slice][rest_state]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    const std::shared_ptr<std::atomic<unsigned int>> calls = std::make_shared<std::atomic<unsigned int>>(0);
    ObserverRelease release(gate);
    fixture.set_progress_observer([gate, calls]() {
        if (calls->fetch_add(1) == 0)
            gate->enter();
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(gate->wait_for_entries());

    std::future<HttpResponseMessage> state_request = std::async(std::launch::async, [&fixture, route = session_route.route]() {
        return fixture.request(http::verb::get, route + "/state");
    });
    const std::future_status state_status = state_request.wait_for(std::chrono::seconds(3));
    std::optional<HttpResponseMessage> state_response;
    if (state_status == std::future_status::ready)
        state_response = state_request.get();

    // Release before fatal checks so a lock regression cannot strand the slicing worker.
    gate->release();
    REQUIRE(state_status == std::future_status::ready);
    REQUIRE(state_response.has_value());
    REQUIRE(state_response->result() == http::status::ok);
    const nlohmann::json state = response_json(*state_response);
    CHECK(state.at("etat") == "slicing");
    REQUIRE(state.at("primary_message").is_string());
    REQUIRE(state.at("secondary_message").is_string());
    REQUIRE(state.at("primary_percentage").is_number_integer());
    REQUIRE(state.at("secondary_percentage").is_number_integer());
    CHECK(state.at("primary_percentage").get<int>() >= -1);
    CHECK(state.at("primary_percentage").get<int>() <= 100);
    CHECK(state.at("secondary_percentage").get<int>() >= -1);
    CHECK(state.at("secondary_percentage").get<int>() <= 100);
    const std::string messages = state.at("primary_message").get<std::string>() +
        state.at("secondary_message").get<std::string>();
    CHECK(messages.find(fixture.work_directory().string()) == std::string::npos);
    CHECK(messages.find(fixture.work_directory().u8string()) == std::string::npos);

    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::sliced));
    const HttpResponseMessage terminal_response = fixture.request(
        http::verb::get, session_route.route + "/state");
    REQUIRE(terminal_response.result() == http::status::ok);
    CHECK(response_json(terminal_response) == nlohmann::json{
        {"etat", "sliced"}, {"primary_message", ""}, {"primary_percentage", -1},
        {"secondary_message", ""}, {"secondary_percentage", -1}});
}

TEST_CASE("REST slice exports a real cube and publishes one owned result", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);

    run_successful_slice(fixture, session_route);

    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    REQUIRE(session != nullptr);
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    REQUIRE(viewer != nullptr);
    REQUIRE(viewer->viewer != nullptr);
    CHECK_FALSE(viewer->outdated);
    CHECK_FALSE(viewer->viewer->moves.empty());
    REQUIRE(session->current_gcode().has_value());
    CHECK(session->current_gcode()->filename() == "output.gcode");
    CHECK(session->current_gcode()->parent_path().filename().string().rfind("rest-result-", 0) == 0);
    CHECK(session->current_gcode()->parent_path().parent_path() == fixture.work_directory() / "results");

    CHECK(read_file(session->current_gcode().value()).find("G1") != std::string::npos);
    CHECK(wait_for_result_count(fixture, 1));
    CHECK_FALSE(contains_temporary_file(fixture));
}

TEST_CASE("REST slice accepts one job per session while its worker is blocked", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(gate->wait_for_entries());
    const HttpResponseMessage duplicate = fixture.request(http::verb::post, session_route.route + "/slice");
    require_error(duplicate, http::status::conflict, "session_busy");
    CHECK(gate->entry_count() == 1);
    CHECK(session_route.session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);

    gate->release();
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::sliced));
}

TEST_CASE("REST slice blocks mutations but permits snapshot reads", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);
    const HttpResponseMessage objects = fixture.request(http::verb::get, session_route.route + "/list_objects");
    REQUIRE(objects.result() == http::status::ok);
    const std::string object_id = response_json(objects).at("objects").at(0).at("id");

    REQUIRE(fixture.request(http::verb::post, session_route.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    require_error(
        fixture.request(
            http::verb::post,
            session_route.route + "/transform/" + object_id,
            identity_transform().dump(),
            "application/json"),
        http::status::conflict,
        "session_busy");
    require_error(
        fixture.request(http::verb::post, session_route.route + "/set_setting/layer_height/0.25"),
        http::status::conflict,
        "session_busy");
    CHECK(fixture.request(http::verb::get, session_route.route + "/list_objects").result() == http::status::ok);
    CHECK(fixture.request(http::verb::get, session_route.route + "/get_setting/layer_height").result() == http::status::ok);
    CHECK(fixture.request(http::verb::get, session_route.route + "/save_project").result() == http::status::ok);

    gate->release();
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::sliced));
}

TEST_CASE("REST slice keeps registry work busy and restarts TTL at completion", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.session_ttl = std::chrono::seconds(1);
    RestServerFixture fixture(limits, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;

    REQUIRE(fixture.request(http::verb::post, session_route.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    CHECK(fixture.sessions().erase(Slic3r::Rest::normalize_username("Alice"), session_route.id) ==
        Slic3r::Rest::SessionEraseResult::busy);
    CHECK(fixture.sessions().expire(Slic3r::Rest::SessionRegistry::Clock::now() + std::chrono::hours(1)) == 0);

    gate->release();
    REQUIRE(wait_until([&]() {
        return session->snapshot().status == Slic3r::Rest::SessionStatus::sliced;
    }));
    const Slic3r::Rest::SessionContext::TimePoint finished = session->snapshot().last_activity;
    CHECK(fixture.sessions().expire(finished + std::chrono::milliseconds(999)) == 0);
    CHECK(fixture.sessions().expire(finished + std::chrono::seconds(1)) == 1);
    CHECK(fixture.sessions().size() == 0);
}

TEST_CASE("REST slice replaces result directories after external snapshots release them", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    const std::shared_ptr<std::atomic<unsigned int>> calls = std::make_shared<std::atomic<unsigned int>>(0);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate, calls]() {
        if (calls->fetch_add(1) == 1)
            gate->enter();
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;

    run_successful_slice(fixture, session_route);
    std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> held = session->viewer_snapshot();
    REQUIRE(held != nullptr);
    const std::filesystem::path first_directory = held->gcode_file.parent_path();
    const std::string first_result_id = held->result_id;
    REQUIRE(std::filesystem::is_directory(first_directory));

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(gate->wait_for_entries());
    CHECK_FALSE(session->current_gcode().has_value());
    REQUIRE(session->viewer_snapshot() != nullptr);
    CHECK(session->viewer_snapshot()->outdated);
    CHECK(session->viewer_snapshot()->result_id == first_result_id);
    gate->release();
    REQUIRE(wait_for_status(session, Slic3r::Rest::SessionStatus::sliced));
    CHECK(session->viewer_snapshot()->result_id != first_result_id);
    const std::filesystem::path second_directory = session->current_gcode()->parent_path();
    CHECK(second_directory != first_directory);
    CHECK(wait_for_result_count(fixture, 2));
    CHECK(std::filesystem::is_directory(first_directory));
    held.reset();
    CHECK(wait_until([&]() { return !std::filesystem::exists(first_directory); }));
    CHECK(wait_for_result_count(fixture, 1));

    run_successful_slice(fixture, session_route);
    CHECK(wait_until([&]() { return !std::filesystem::exists(second_directory); }));
    CHECK(wait_for_result_count(fixture, 1));
}

TEST_CASE("REST slice destroys a displaced result outside session and job locks", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute first = create_cube_session(fixture);
    const SessionRoute second = create_cube_session(fixture);
    const std::shared_ptr<ObserverGate> owner_gate = std::make_shared<ObserverGate>(1);
    std::future<Slic3r::Rest::SessionContext::Snapshot> snapshot_probe;
    std::future<HttpResponseMessage> second_slice;
    ObserverRelease release(owner_gate);

    const Slic3r::Rest::SessionContext::WorkToken prior_work = first.session->begin_work(
        Slic3r::Rest::SessionRegistry::Clock::now()).value();
    std::shared_ptr<const void> prior_owner(
        new std::uint8_t(0),
        [owner_gate](const void *value) {
            owner_gate->enter();
            delete static_cast<const std::uint8_t *>(value);
        });
    REQUIRE(first.session->finish_work(
        prior_work,
        "synthetic.gcode",
        Slic3r::GCodeProcessorResult(),
        Slic3r::Rest::SessionRegistry::Clock::now(),
        prior_owner));
    prior_owner.reset();

    const HttpResponseMessage first_accepted = fixture.request(http::verb::post, first.route + "/slice");
    REQUIRE(first_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(first_accepted));
    REQUIRE(owner_gate->wait_for_entries());

    const std::shared_ptr<Slic3r::Rest::SessionContext> first_session = first.session;
    snapshot_probe = std::async(std::launch::async, [first_session]() {
        return first_session->snapshot();
    });
    second_slice = std::async(std::launch::async, [&fixture, route = second.route]() {
        return fixture.request(http::verb::post, route + "/slice");
    });
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    const std::future_status snapshot_status = snapshot_probe.wait_until(deadline);
    const std::future_status second_status = second_slice.wait_until(deadline);

    // Release before assertions so a regression cannot strand either async probe during unwinding.
    owner_gate->release();
    REQUIRE(snapshot_status == std::future_status::ready);
    REQUIRE(second_status == std::future_status::ready);
    CHECK(snapshot_probe.get().status == Slic3r::Rest::SessionStatus::sliced);
    const HttpResponseMessage second_accepted = second_slice.get();
    REQUIRE(second_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(second_accepted));
    REQUIRE(wait_for_status(first.session, Slic3r::Rest::SessionStatus::sliced));
    REQUIRE(wait_for_status(second.session, Slic3r::Rest::SessionStatus::sliced));
}

TEST_CASE("REST slice failure retains only the old outdated result", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    run_successful_slice(fixture, session_route);
    const std::string old_result_id = session->viewer_snapshot()->result_id;
    const std::filesystem::path old_directory = session->current_gcode()->parent_path();

    Slic3r::Rest::SessionData invalid = session->data_snapshot().value();
    invalid.configuration.set_deserialize_strict("layer_height", "1.0");
    REQUIRE(session->commit_mutation(
        session->begin_mutation().value(),
        std::move(invalid),
        true,
        Slic3r::Rest::SessionRegistry::Clock::now()));
    REQUIRE(session->viewer_snapshot()->outdated);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(wait_for_status(session, Slic3r::Rest::SessionStatus::failed));
    const std::string failure = session->snapshot().error_message;
    CHECK_FALSE(failure.empty());
    CHECK(failure.find(fixture.work_directory().string()) == std::string::npos);
    CHECK_FALSE(session->current_gcode().has_value());
    REQUIRE(session->viewer_snapshot() != nullptr);
    CHECK(session->viewer_snapshot()->result_id == old_result_id);
    CHECK(session->viewer_snapshot()->outdated);
    CHECK(std::filesystem::is_directory(old_directory));
    REQUIRE(wait_for_result_count(fixture, 1));
    const std::vector<std::filesystem::path> remaining = result_directories(fixture);
    REQUIRE(remaining.size() == 1);
    CHECK(remaining.front() == old_directory);
}

TEST_CASE("REST slice refuses an empty scene without changing it", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const std::string id = create_session(fixture);
    const std::string route = "/session/" + id;
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = find_session(fixture, id);
    const Slic3r::Rest::SessionContext::Snapshot before = session->snapshot();

    require_error(
        fixture.request(http::verb::post, route + "/slice"),
        http::status::conflict,
        "empty_scene");
    const Slic3r::Rest::SessionContext::Snapshot after = session->snapshot();
    CHECK(after.revision == before.revision);
    CHECK(after.status == before.status);
    CHECK(after.work_generation == before.work_generation);
    CHECK(result_directories(fixture).empty());
}

TEST_CASE("REST slice runs two sessions simultaneously with two workers", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(2);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute first = create_cube_session(fixture);
    const SessionRoute second = create_cube_session(fixture);

    const HttpResponseMessage first_accepted = fixture.request(http::verb::post, first.route + "/slice");
    const HttpResponseMessage second_accepted = fixture.request(http::verb::post, second.route + "/slice");
    REQUIRE(first_accepted.result() == http::status::accepted);
    REQUIRE(second_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(first_accepted));
    require_accepted_state(response_json(second_accepted));
    REQUIRE(gate->wait_for_entries());
    CHECK(gate->entry_count() == 2);
    gate->release();

    REQUIRE(wait_for_status(first.session, Slic3r::Rest::SessionStatus::sliced));
    REQUIRE(wait_for_status(second.session, Slic3r::Rest::SessionStatus::sliced));
    CHECK(wait_for_result_count(fixture, 2));
}

TEST_CASE("REST slice queues every accepted session behind one worker", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.slice_threads = 1;
    RestServerFixture fixture(limits, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute first = create_cube_session(fixture);
    const SessionRoute second = create_cube_session(fixture);
    const SessionRoute third = create_cube_session(fixture);

    const HttpResponseMessage first_accepted = fixture.request(http::verb::post, first.route + "/slice");
    REQUIRE(first_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(first_accepted));
    REQUIRE(gate->wait_for_entries());
    const HttpResponseMessage second_accepted = fixture.request(http::verb::post, second.route + "/slice");
    const HttpResponseMessage third_accepted = fixture.request(http::verb::post, third.route + "/slice");
    REQUIRE(second_accepted.result() == http::status::accepted);
    REQUIRE(third_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(second_accepted));
    require_accepted_state(response_json(third_accepted));
    CHECK(first.session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
    CHECK(second.session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
    CHECK(third.session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
    CHECK(gate->entry_count() == 1);

    gate->release();
    REQUIRE(wait_for_status(first.session, Slic3r::Rest::SessionStatus::sliced));
    REQUIRE(wait_for_status(second.session, Slic3r::Rest::SessionStatus::sliced));
    REQUIRE(wait_for_status(third.session, Slic3r::Rest::SessionStatus::sliced));
    CHECK(wait_for_result_count(fixture, 3));
}

TEST_CASE("REST slice never executes a configured post-process command", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    const std::filesystem::path nonexistent = fixture.work_directory() / "missing-post-process-command";
    Slic3r::Rest::SessionData data = session->data_snapshot().value();
    data.configuration.set_key_value(
        "post_process",
        new Slic3r::ConfigOptionStrings({nonexistent.string()}));
    REQUIRE(session->commit_mutation(
        session->begin_mutation().value(),
        std::move(data),
        true,
        Slic3r::Rest::SessionRegistry::Clock::now()));

    run_successful_slice(fixture, session_route);
    CHECK_FALSE(std::filesystem::exists(nonexistent));
}

TEST_CASE("REST slice contains and sanitizes observer failures", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::filesystem::path disclosed =
        fixture.work_directory() / "results" / "rest-result-secret" / "output.gcode";
    fixture.set_slice_observer([disclosed]() {
        throw std::runtime_error(
            "observer failed for " + disclosed.string() + " and " + disclosed.generic_string());
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::failed));
    const std::string message = session_route.session->snapshot().error_message;
    CHECK(message.find("<output>") != std::string::npos);
    CHECK(message.find(fixture.work_directory().string()) == std::string::npos);
    CHECK(message.find(fixture.work_directory().generic_string()) == std::string::npos);
    CHECK(wait_for_result_count(fixture, 0));
}

TEST_CASE("REST slice sanitizes repeated UTF-8 work paths in observer failures", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true, std::filesystem::u8path("work-\xc3\xa9"));
    fixture.stop();
    const std::filesystem::path work = fixture.work_directory();
    const std::filesystem::path disclosed = work / "results" / "rest-result-secret" / "output.gcode";
    fixture.set_slice_observer([work, disclosed]() {
        throw std::runtime_error(
            "observer failed for " + work.u8string() + " and " + work.generic_u8string() +
            " under " + disclosed.u8string() + " then " + work.u8string());
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    require_accepted_state(response_json(accepted));
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::failed));
    const std::string message = session_route.session->snapshot().error_message;
    CHECK(message.find("<output>") != std::string::npos);
    CHECK(message.find(work.u8string()) == std::string::npos);
    CHECK(message.find(work.generic_u8string()) == std::string::npos);
    CHECK(wait_for_result_count(fixture, 0));
}

TEST_CASE("REST stop cancels running and queued slices before joining pools", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.slice_threads = 1;
    RestServerFixture fixture(limits, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute running_route = create_cube_session(fixture);
    const SessionRoute queued_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> running = running_route.session;
    const std::shared_ptr<Slic3r::Rest::SessionContext> queued = queued_route.session;

    const HttpResponseMessage running_accepted = fixture.request(http::verb::post, running_route.route + "/slice");
    REQUIRE(running_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(running_accepted));
    REQUIRE(gate->wait_for_entries());
    const HttpResponseMessage queued_accepted = fixture.request(http::verb::post, queued_route.route + "/slice");
    REQUIRE(queued_accepted.result() == http::status::accepted);
    require_accepted_state(response_json(queued_accepted));
    std::future<void> stopped = std::async(std::launch::async, [&fixture]() { fixture.stop(); });
    const bool stop_requested = wait_until([&fixture]() { return !fixture.running(); });
    if (!stop_requested)
        gate->release();
    REQUIRE(stop_requested);
    gate->release();
    REQUIRE(stopped.wait_for(gate_timeout) == std::future_status::ready);
    stopped.get();

    CHECK(running->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK(queued->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    // The queued job ended at shutdown and never reached the worker hook.
    CHECK(gate->entry_count() == 1);
    CHECK_FALSE(running->current_gcode().has_value());
    CHECK_FALSE(queued->current_gcode().has_value());
    CHECK(wait_for_result_count(fixture, 0));
}

TEST_CASE("REST slice validates authentication method body ownership and state", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);

    require_error(
        fixture.request_without_auth(http::verb::post, session_route.route + "/slice", "{}", "application/json"),
        http::status::unauthorized,
        "unauthorized");
    require_error(
        fixture.request(http::verb::get, session_route.route + "/slice"),
        http::status::not_found,
        "route_not_found");
    require_error(
        fixture.request(http::verb::post, session_route.route + "/slice", "{}", "application/json"),
        http::status::bad_request,
        "invalid_request");
    require_error(
        fixture.request(http::verb::post, "/session/missing/slice", "{}", "application/json"),
        http::status::bad_request,
        "invalid_request");
    const std::string bob_token = fixture.login_token("Bob", "secret");
    require_error(
        fixture.request(
            http::verb::post,
            session_route.route + "/slice",
            {},
            {},
            {{"Authorization", "Bearer " + bob_token}}),
        http::status::not_found,
        "session_not_found");
    require_error(
        fixture.request(http::verb::post, "/session/missing/slice"),
        http::status::not_found,
        "session_not_found");
}

TEST_CASE("REST result owner survives registry deletion until a viewer releases it", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    run_successful_slice(fixture, session_route);
    std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> held = session->viewer_snapshot();
    REQUIRE(held != nullptr);
    REQUIRE(held->gcode_owner != nullptr);
    const std::filesystem::path directory = held->gcode_file.parent_path();

    CHECK(fixture.sessions().erase(Slic3r::Rest::normalize_username("Alice"), session_route.id) ==
        Slic3r::Rest::SessionEraseResult::erased);
    CHECK(std::filesystem::is_directory(directory));
    held.reset();
    CHECK(wait_until([&]() { return !std::filesystem::exists(directory); }));
}

TEST_CASE("REST startup removes stale result directories but preserves unrelated files", "[rest_slice]")
{
    RestServerFixture fixture;
    fixture.stop();
    const std::filesystem::path results = fixture.work_directory() / "results";
    const std::filesystem::path stale = results / "rest-result-stale";
    std::filesystem::create_directories(stale);
    std::ofstream(stale / "output.gcode") << "stale";
    std::ofstream(results / "keep.txt") << "keep";

    fixture.start();
    CHECK_FALSE(std::filesystem::exists(stale));
    CHECK(read_file(results / "keep.txt") == "keep");
}

TEST_CASE("REST restart preserves a result while an old snapshot owns it", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> held;
    {
        const SessionRoute session_route = create_cube_session(fixture);
        run_successful_slice(fixture, session_route);
        held = session_route.session->viewer_snapshot();
    }
    REQUIRE(held != nullptr);
    const std::filesystem::path directory = held->gcode_file.parent_path();
    fixture.stop();
    fixture.start();
    CHECK(std::filesystem::is_directory(directory));
    CHECK(read_file(held->gcode_file).find("G1") != std::string::npos);
    held.reset();
    CHECK(wait_until([&]() { return !std::filesystem::exists(directory); }));
}

TEST_CASE("REST slice reports sanitized result-storage failures", "[rest_slice]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);
    const std::filesystem::path results = fixture.work_directory() / "results";
    std::filesystem::remove_all(results);
    std::ofstream(results) << "not a directory";

    const HttpResponseMessage accepted = fixture.request(http::verb::post, session_route.route + "/slice");
    REQUIRE(accepted.result() == http::status::accepted);
    CHECK(response_json(accepted) == nlohmann::json{
        {"etat", "failed"}, {"primary_message", "The server could not prepare the result storage."},
        {"primary_percentage", -1}, {"secondary_message", ""}, {"secondary_percentage", -1}});
    REQUIRE(wait_for_status(session_route.session, Slic3r::Rest::SessionStatus::failed));
    const std::string message = session_route.session->snapshot().error_message;
    CHECK(message == "The server could not prepare the result storage.");
    CHECK(message.find(fixture.work_directory().string()) == std::string::npos);
    CHECK(message.find("rest-result-") == std::string::npos);
    CHECK(result_directories(fixture).empty());
}

namespace {

nlohmann::json cancel_state(const RestServerFixture &fixture, const std::string &route)
{
    const HttpResponseMessage response = fixture.request(http::verb::post, route + "/cancel_slicing");
    REQUIRE(response.result() == http::status::accepted);
    return response_json(response);
}

} // namespace

TEST_CASE("REST cancel_slicing stops a running engine and allows a new slice", "[rest_cancel]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    // The first progress update proves the engine runs; the gate then holds it.
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    // Shared like the gate: workers may still call the observer while a failed
    // test unwinds, after the test's own locals are gone.
    const std::shared_ptr<std::atomic<bool>> hold = std::make_shared<std::atomic<bool>>(true);
    fixture.set_progress_observer([gate, hold]() {
        if (hold->exchange(false))
            gate->enter();
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;

    REQUIRE(fixture.request(http::verb::post, session_route.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    CHECK(cancel_state(fixture, session_route.route).at("etat") == "slicing");
    // The run is still stopping: the state stays slicing, mutations and new
    // slices are refused, and a repeated cancellation is accepted.
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
    require_error(fixture.request(http::verb::post, session_route.route + "/slice"),
        http::status::conflict, "session_busy");
    require_error(fixture.request(http::verb::post, session_route.route + "/set_setting/layer_height/0.25"),
        http::status::conflict, "session_busy");
    CHECK(cancel_state(fixture, session_route.route).at("etat") == "slicing");

    gate->release();
    REQUIRE(wait_for_status(session, Slic3r::Rest::SessionStatus::ready));
    CHECK_FALSE(session->current_gcode().has_value());
    CHECK(session->viewer_snapshot() == nullptr);
    CHECK(wait_for_result_count(fixture, 0));
    require_error(fixture.request(http::verb::post, session_route.route + "/cancel_slicing"),
        http::status::conflict, "not_slicing");

    run_successful_slice(fixture, session_route);
    CHECK(wait_for_result_count(fixture, 1));
}

TEST_CASE("REST cancel_slicing keeps the previous result outdated", "[rest_cancel]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    // Shared like the gate, for the same unwinding reason as above.
    const std::shared_ptr<std::atomic<bool>> hold = std::make_shared<std::atomic<bool>>(false);
    fixture.set_slice_observer([gate, hold]() {
        if (hold->load())
            gate->enter();
    });
    fixture.start();
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    run_successful_slice(fixture, session_route);
    const std::string old_result_id = session->viewer_snapshot()->result_id;
    const std::filesystem::path old_directory = session->current_gcode()->parent_path();

    // Started but not yet in the engine: the remembered request stops it there.
    hold->store(true);
    REQUIRE(fixture.request(http::verb::post, session_route.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    CHECK(cancel_state(fixture, session_route.route).at("etat") == "slicing");
    gate->release();

    REQUIRE(wait_for_status(session, Slic3r::Rest::SessionStatus::ready));
    CHECK(session->snapshot().error_message.empty());
    REQUIRE(session->viewer_snapshot() != nullptr);
    CHECK(session->viewer_snapshot()->result_id == old_result_id);
    CHECK(session->viewer_snapshot()->outdated);
    REQUIRE(wait_for_result_count(fixture, 1));
    CHECK(result_directories(fixture).front() == old_directory);
}

TEST_CASE("REST cancel_slicing ends queued work at once without running it", "[rest_cancel]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.slice_threads = 1;
    RestServerFixture fixture(limits, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(1);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute running = create_cube_session(fixture);
    const SessionRoute queued = create_cube_session(fixture);

    REQUIRE(fixture.request(http::verb::post, running.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    REQUIRE(fixture.request(http::verb::post, queued.route + "/slice").result() == http::status::accepted);
    const nlohmann::json cancelled = cancel_state(fixture, queued.route);
    CHECK(cancelled.at("etat") == "ready");
    CHECK(queued.session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    require_error(fixture.request(http::verb::post, queued.route + "/cancel_slicing"),
        http::status::conflict, "not_slicing");

    gate->release();
    REQUIRE(wait_for_status(running.session, Slic3r::Rest::SessionStatus::sliced));
    // Wait until the skipped queued job has been dequeued, then check it never
    // reached the worker hook nor produced a result.
    run_successful_slice(fixture, running);
    CHECK(gate->entry_count() == 2);
    CHECK(queued.session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    CHECK(wait_for_result_count(fixture, 1));
}

TEST_CASE("REST cancel_slicing of one session does not stop another", "[rest_cancel]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<ObserverGate> gate = std::make_shared<ObserverGate>(2);
    ObserverRelease release(gate);
    fixture.set_slice_observer([gate]() { gate->enter(); });
    fixture.start();
    const SessionRoute first = create_cube_session(fixture);
    const SessionRoute second = create_cube_session(fixture);

    REQUIRE(fixture.request(http::verb::post, first.route + "/slice").result() == http::status::accepted);
    REQUIRE(fixture.request(http::verb::post, second.route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait_for_entries());
    CHECK(cancel_state(fixture, first.route).at("etat") == "slicing");
    gate->release();

    REQUIRE(wait_for_status(first.session, Slic3r::Rest::SessionStatus::ready));
    REQUIRE(wait_for_status(second.session, Slic3r::Rest::SessionStatus::sliced));
    CHECK(wait_for_result_count(fixture, 1));
}

TEST_CASE("REST cancel_slicing refuses requests without active work", "[rest_cancel]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    const SessionRoute session_route = create_cube_session(fixture);
    const std::shared_ptr<Slic3r::Rest::SessionContext> session = session_route.session;
    const std::string target = session_route.route + "/cancel_slicing";

    require_error(fixture.request(http::verb::post, target), http::status::conflict, "not_slicing");
    run_successful_slice(fixture, session_route);
    const std::uint64_t revision = session->snapshot().revision;
    require_error(fixture.request(http::verb::post, target), http::status::conflict, "not_slicing");
    CHECK(session->snapshot().revision == revision);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);

    const Slic3r::Rest::SessionContext::WorkToken failing =
        session->begin_work(Slic3r::Rest::SessionRegistry::Clock::now()).value();
    REQUIRE(session->fail_work(failing, "failed", Slic3r::Rest::SessionRegistry::Clock::now()));
    require_error(fixture.request(http::verb::post, target), http::status::conflict, "not_slicing");

    require_error(fixture.request(http::verb::get, target), http::status::not_found, "route_not_found");
    require_error(fixture.request(http::verb::post, target + "/extra"), http::status::not_found, "route_not_found");
    require_error(fixture.request(http::verb::post, target, "{}", "application/json"),
        http::status::bad_request, "invalid_request");
    require_error(fixture.request_without_auth(http::verb::post, target), http::status::unauthorized, "unauthorized");
    const std::string bob_token = fixture.login_token("Bob", "secret");
    require_error(fixture.request(http::verb::post, target, {}, {}, {{"Authorization", "Bearer " + bob_token}}),
        http::status::not_found, "session_not_found");
    require_error(fixture.request(http::verb::post, "/session/missing/cancel_slicing"),
        http::status::not_found, "session_not_found");
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::failed);
}