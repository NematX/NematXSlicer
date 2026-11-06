// Review regressions use public loopback HTTP; worker hooks are only barriers.
// Slow profile preparation must not block the network or overwrite newer state.
#include <catch2/catch.hpp>
#include "profiles.hpp"
#include "rest_models_test_helpers.hpp"
#include "rest_vendor_test_helpers.hpp"

#include <atomic>
#include <future>

using namespace RestServerTest;

namespace {

// Resolve by full name and fail visibly if a fixture entry is missing.
std::string id_for(const nlohmann::json &listed, const char *category, const std::string &name)
{
    for (const nlohmann::json &entry : listed.at(category))
        if (entry.at("name") == name)
            return entry.at("id").get<std::string>();
    FAIL("Missing profile " + name);
    return {};
}

// Assert existence as well as the compatibility value, never an empty loop.
void check_compatible(const nlohmann::json &listed, const char *category, const std::string &name, bool expected)
{
    const std::string id = id_for(listed, category, name);
    for (const nlohmann::json &entry : listed.at(category))
        if (entry.at("id") == id)
            CHECK(entry.at("compatible") == expected);
}

// Arm once after fixture setup. The timeout is a stuck-test safeguard, not a
// performance threshold; release remains safe during assertion unwinding.
struct ProfileGate {
    std::atomic<bool> armed{false};
    std::atomic<bool> released{false};
    std::promise<void> entered_promise, release_promise;
    std::shared_future<void> entered = entered_promise.get_future().share();
    std::shared_future<void> release_ready = release_promise.get_future().share();
    void observe()
    {
        if (!armed.exchange(false))
            return;
        entered_promise.set_value();
        release_ready.wait_for(std::chrono::seconds(120));
    }
    void release()
    {
        if (!released.exchange(true))
            release_promise.set_value();
    }
};

} // namespace

TEST_CASE("Profile defaults and selection leave HTTP responsive and reject stale commits", "[rest_profile_review]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path(), false);
    RestServerFixture fixture({}, true, false, "work", {}, root.path());
    const std::shared_ptr<ProfileGate> gate = std::make_shared<ProfileGate>();
    fixture.stop();
    fixture.set_import_observer([gate]() { gate->observe(); });
    fixture.start();
    const std::string route = "/session/" + create_session(fixture);
    const nlohmann::json listed = nlohmann::json::parse(fixture.request(http::verb::get, route + "/list_profiles").body());
    const std::string selection = nlohmann::json{{"print", id_for(listed, "print", "0.20mm NORMAL .4N")},
        {"printer", id_for(listed, "printer", "Basic 0.4 nozzle")}, {"extruder", nlohmann::json::array()},
        {"filaments", nlohmann::json::array({id_for(listed, "filaments", "Basic PLA @BASIC")})}}.dump();
    bool defaults = false, change = false, remove = false, shutdown = false;
    SECTION("defaults") { defaults = true; }
    SECTION("selection") {}
    SECTION("concurrent mutation") { change = true; }
    SECTION("concurrent deletion") { remove = true; }
    SECTION("shutdown while preparing selection") { shutdown = true; }
    gate->armed = true;
    std::future<HttpResponseMessage> slow = std::async(std::launch::async, [&]() {
        return defaults ? fixture.request(http::verb::post, "/create_session") :
            fixture.request(http::verb::post, route + "/set_profile", selection, "application/json");
    });
    const bool entered = gate->entered.wait_for(std::chrono::seconds(60)) == std::future_status::ready;
    CHECK(entered);
    // A separate request must complete while preparation remains at the barrier.
    std::future<HttpResponseMessage> state = std::async(std::launch::async, [&]() {
        return fixture.request(http::verb::get, route + "/state");
    });
    // Destroy before either future so an assertion never leaves a worker gated.
    struct Release {
        std::shared_ptr<ProfileGate> gate;
        ~Release() { gate->release(); }
    } release{gate};
    const bool responsive = state.wait_for(std::chrono::seconds(30)) == std::future_status::ready;
    CHECK(responsive);
    if (!entered || !responsive) {
        gate->release();
        slow.wait();
        state.wait();
        return;
    }
    CHECK(state.get().result() == http::status::ok);
    if (change)
        REQUIRE(fixture.request(http::verb::post, route + "/set_name", R"({"name":"newer"})", "application/json").result() == http::status::ok);
    if (remove)
        REQUIRE(fixture.request(http::verb::delete_, route + "/delete").result() == http::status::no_content);
    if (shutdown) {
        std::future<void> stopped = std::async(std::launch::async, [&]() { fixture.stop(); });
        CHECK(stopped.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout);
        gate->release();
        REQUIRE(stopped.wait_for(std::chrono::seconds(300)) == std::future_status::ready);
        stopped.get();
        // The closed connection may produce an EOF rather than an HTTP response.
        try { slow.get(); } catch (const boost::system::system_error &) {}
        CHECK_FALSE(fixture.running());
        return;
    }
    gate->release();
    REQUIRE(slow.wait_for(std::chrono::seconds(300)) == std::future_status::ready);
    const HttpResponseMessage response = slow.get();
    CHECK(response.result() == (defaults ? http::status::created :
        remove ? http::status::not_found : change ? http::status::conflict : http::status::ok));
    if (change) {
        CHECK(nlohmann::json::parse(response.body())["error"]["code"] == "session_changed");
        CHECK(nlohmann::json::parse(fixture.request(http::verb::get, route + "/get_name").body())["name"] == "newer");
    }
}

TEST_CASE("Profile compatibility reads edited session values but overrides use catalog presets", "[rest_profile_review]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path());
    std::filesystem::create_directories(root.path() / "print");
    std::ofstream(root.path() / "print/Named.ini") << "compatible_printers = Test 0.4\n";
    std::ofstream(root.path() / "print/User print.ini") << "inherits = Test print @TEST\nlayer_height = 0.2\n";
    std::filesystem::create_directories(root.path() / "printer");
    std::ofstream(root.path() / "printer/User printer.ini")
        << "inherits = Test 0.4\nnozzle_diameter = 0.4\ndefault_filament_profile = Test PLA @TEST\n";
    RestServerFixture fixture({}, true, false, "work", {}, root.path());
    const std::string route = "/session/" + create_session(fixture);
    const nlohmann::json all = nlohmann::json::parse(fixture.request(http::verb::get, route + "/list_profiles").body());
    bool user = false;
    SECTION("system preset identity") {}
    SECTION("user preset retains inherited vendor") { user = true; }
    const std::string printer = id_for(all, "printer", user ? "User printer" : "Test 0.4");
    const std::string print = id_for(all, "print", user ? "User print" : "Test print @TEST");
    const std::string body = nlohmann::json{{"print", print}, {"printer", printer}, {"extruder", nlohmann::json::array()},
        {"filaments", nlohmann::json::array({id_for(all, "filaments", "Test PLA @TEST")})}}.dump();
    REQUIRE(fixture.request(http::verb::post, route + "/set_profile", body, "application/json").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_setting/nozzle_diameter/0.25").result() == http::status::ok);
    REQUIRE(fixture.request(http::verb::post, route + "/set_settings", R"({"layer_height":{"value":0.1}})", "application/json").result() == http::status::ok);
    const nlohmann::json edited = nlohmann::json::parse(fixture.request(http::verb::get, route + "/list_profiles").body());
    check_compatible(edited, "print", "Test print @TEST", false);
    check_compatible(edited, "print", "Test fine .25N", true);
    check_compatible(edited, "print", "Named", !user); // Keep the printer name for named lists.
    check_compatible(edited, "filaments", "Test PLA @TEST", false);
    const nlohmann::json printer_override = nlohmann::json::parse(fixture.request(http::verb::get,
        route + "/list_profiles?printer=" + printer).body());
    check_compatible(printer_override, "print", "Test print @TEST", true);
    check_compatible(printer_override, "filaments", "Test PLA @TEST", false);
    const nlohmann::json print_override = nlohmann::json::parse(fixture.request(http::verb::get,
        route + "/list_profiles?print=" + print).body());
    check_compatible(print_override, "print", "Test fine .25N", true);
    check_compatible(print_override, "filaments", "Test PLA @TEST", true);
}

TEST_CASE("Vendor and flat user presets load under UTF-8 paths and uppercase INI is validated", "[rest_profile_review]")
{
    ensure_engine_initialized();
    TempDirectory root;
    const std::filesystem::path profiles = root.path() / std::filesystem::u8path(u8"profils-é");
    write_vendor_profiles(profiles, false);
    std::filesystem::rename(profiles / "vendor/Basic.ini", profiles / "vendor/Basic.INI");
    std::filesystem::create_directories(profiles / "print");
    std::ofstream(profiles / "print" / std::filesystem::u8path(u8"Utilisateur-é.ini"))
        << "inherits = 0.20mm NORMAL .4N\nlayer_height = 0.15\n";
    Slic3r::Rest::ProfileCatalog catalog(profiles);
    const nlohmann::json listed = catalog.list_profiles();
    Slic3r::Rest::ProfileSelection selection;
    selection.print = id_for(listed, "print", u8"Utilisateur-é");
    selection.printer = id_for(listed, "printer", "Basic 0.4 nozzle");
    selection.filaments = {id_for(listed, "filaments", "Basic PLA @BASIC")};
    CHECK(catalog.select(selection).configuration.opt_float("layer_height") == Approx(0.15));
    RestServerFixture fixture({}, true, false, std::filesystem::u8path(u8"travail-é"), {}, profiles);
    const std::string route = "/session/" + create_session(fixture);
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_profiles");
    REQUIRE(response.result() == http::status::ok);
    CHECK_FALSE(id_for(nlohmann::json::parse(response.body()), "print", u8"Utilisateur-é").empty());
    std::ofstream(profiles / "vendor/Invalid.INI") << "[print:invalid]\nlayer_height = 0.2\n";
    CHECK_THROWS_WITH(Slic3r::Rest::ProfileCatalog(profiles), Catch::Matchers::Contains("Invalid.INI"));
}
