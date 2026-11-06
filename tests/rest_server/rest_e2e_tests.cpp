// Public-route workflows on a loopback TCP listener. Hooks only hold workers
// at barriers; all session, configuration and result assertions use HTTP.
#include <catch_main.hpp>
#include "rest_e2e_client.hpp"

#include <condition_variable>
#include <mutex>

using namespace RestServerTest;
using namespace RestServerTest::E2E;

namespace {

// Hold each session once, regardless of which engine/TBB thread reports it.
// No hook reads or changes server/session state.
class Gate {
public:
    explicit Gate(std::size_t expected) : expected_(expected) {}
    void enter(const std::string &session_id)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!sessions_.insert(session_id).second)
            return;
        changed_.notify_all();
        if (!changed_.wait_for(lock, work_timeout, [&]() { return released_; }))
            timed_out_ = true;
    }
    bool wait(std::chrono::milliseconds timeout = work_timeout, std::size_t entries = 0)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const std::size_t target = entries == 0 ? expected_ : entries;
        return changed_.wait_for(lock, timeout, [&]() { return sessions_.size() >= target; });
    }
    void release()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }
    bool timed_out()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return timed_out_;
    }
private:
    std::mutex mutex_;
    std::condition_variable changed_;
    std::set<std::string> sessions_;
    std::size_t expected_;
    bool released_ = false, timed_out_ = false;
};

// Release before fixture shutdown even if a REQUIRE unwinds the test.
struct Release {
    std::shared_ptr<Gate> gate;
    ~Release() { gate->release(); }
};

std::string session(const Client &client, const std::string &name = "E2E");
std::string cube(const Client &client, const std::string &route);
nlohmann::json settings(const Client &client, const std::string &route);
nlohmann::json translation(double x, const std::string &space);
void sliced(const Client &client, const std::string &route);
void clean(const RestServerFixture &fixture, std::size_t results);
std::string profile(const nlohmann::json &catalog, const std::string &category, const std::string &name);
void same_geometry(const nlohmann::json &a, const nlohmann::json &b);

std::string session(const Client &client, const std::string &name)
{
    const HttpResponseMessage response = client.post("/create_session", {{"name", name}});
    REQUIRE(response.result() == http::status::created);
    return "/session/" + nlohmann::json::parse(response.body()).at("session_id").get<std::string>();
}

std::string cube(const Client &client, const std::string &route)
{
    const HttpResponseMessage response = client.upload(route + "/add_model", "cube.stl", make_cube_stl());
    return object_ids(response).front();
}

nlohmann::json settings(const Client &client, const std::string &route)
{
    return json_ok(client.post(route + "/get_settings", {{"setting_ids", {
        "layer_height", "overhangs_width_speed", "filament_travel_pa", "print_custom_setting"}}}));
}

nlohmann::json translation(double x, const std::string &space)
{
    return {{"matrix", {1,0,0,x, 0,1,0,0, 0,0,1,0, 0,0,0,1}},
        {"space", space}, {"action", "compose"}};
}

void sliced(const Client &client, const std::string &route)
{
    REQUIRE(client.post(route + "/slice").result() == http::status::accepted);
    wait_state(client, route, "sliced");
}

void clean(const RestServerFixture &fixture, std::size_t results)
{
    std::error_code last_error;
    const bool completed = poll([&]() {
        const WorkInventory inventory = work_inventory(fixture);
        if (inventory.error)
            last_error = inventory.error;
        return !inventory.error && inventory.uploads.empty() && inventory.exports.empty() && inventory.results.size() == results &&
            inventory.files.size() == results;
    });
    INFO("Last incomplete work inventory: " << last_error.message());
    REQUIRE(completed);
    // The checked complete inventory already counted files; a second throwing
    // traversal could race cleanup again after the predicate has succeeded.
}

std::string profile(const nlohmann::json &catalog, const std::string &category, const std::string &name)
{
    for (const nlohmann::json &entry : catalog.at(category))
        if (entry.at("name") == name) return entry.at("id");
    FAIL("Installed profile missing: " + name);
    return {};
}

// The 3MF loader recenters volumes into instances. Compare composed world
// transforms, not that non-unique factorization; entity IDs are session-local.
void same_geometry(const nlohmann::json &a, const nlohmann::json &b)
{
    REQUIRE(a.at("objects").size() == b.at("objects").size());
    for (std::size_t object = 0; object < a.at("objects").size(); ++object) {
        const nlohmann::json &left = a.at("objects")[object], &right = b.at("objects")[object];
        REQUIRE(left.at("instances").size() == right.at("instances").size());
        REQUIRE(left.at("parts").size() == right.at("parts").size());
        for (std::size_t instance = 0; instance < left.at("instances").size(); ++instance)
            for (std::size_t part = 0; part < left.at("parts").size(); ++part) {
                const nlohmann::json &li = left["instances"][instance]["matrix"], &ri = right["instances"][instance]["matrix"];
                const nlohmann::json &lp = left["parts"][part]["matrix"], &rp = right["parts"][part]["matrix"];
                REQUIRE(li.size() == 16);
                REQUIRE(ri.size() == 16);
                REQUIRE(lp.size() == 16);
                REQUIRE(rp.size() == 16);
                for (std::size_t row = 0; row < 4; ++row)
                    for (std::size_t column = 0; column < 4; ++column) {
                        double before = 0, after = 0;
                        for (std::size_t k = 0; k < 4; ++k) {
                            before += li[row * 4 + k].get<double>() * lp[k * 4 + column].get<double>();
                            after += ri[row * 4 + k].get<double>() * rp[k * 4 + column].get<double>();
                        }
                        CHECK(before == Approx(after).margin(1e-6));
                    }
            }
    }
}

} // namespace

TEST_CASE("E2E progress barrier counts sessions not callback threads", "[rest_e2e_helpers]")
{
    const std::shared_ptr<Gate> gate = std::make_shared<Gate>(2);
    std::future<void> first = std::async(std::launch::async, [gate]() { gate->enter("A"); });
    std::future<void> second;
    // Declare after the futures so unwinding releases blocked callbacks first.
    Release release{gate};
    REQUIRE(gate->wait(work_timeout, 1));
    std::future<void> duplicate = std::async(std::launch::async, [gate]() { gate->enter("A"); });
    REQUIRE(duplicate.wait_for(work_timeout) == std::future_status::ready);
    duplicate.get();
    CHECK_FALSE(gate->wait(std::chrono::milliseconds(0)));
    second = std::async(std::launch::async, [gate]() { gate->enter("B"); });
    REQUIRE(gate->wait());
    gate->release();
    REQUIRE(first.wait_for(work_timeout) == std::future_status::ready);
    REQUIRE(second.wait_for(work_timeout) == std::future_status::ready);
    first.get();
    second.get();
    CHECK_FALSE(gate->timed_out());
}

TEST_CASE("E2E inventory retries deletion during directory descent", "[rest_e2e_helpers]")
{
    RestServerFixture fixture;
    const std::filesystem::path result = fixture.work_directory() / "rest-result-race";
    std::filesystem::create_directories(result);
    std::ofstream(result / "output.gcode") << "G1 X1\n";
    bool removed = false;
    const WorkInventory interrupted = work_inventory(fixture, [&](const std::filesystem::path &path) {
        if (path == result) {
            std::filesystem::remove_all(result);
            removed = true;
        }
    });
    REQUIRE(removed);
    REQUIRE(interrupted.error);
    // A fresh scan, not the partial one, establishes that cleanup completed.
    const WorkInventory complete = work_inventory(fixture);
    REQUIRE_FALSE(complete.error);
    CHECK(complete.files.empty());
    CHECK(complete.results.empty());
    clean(fixture, 0);
}

TEST_CASE("E1 complete public REST workflow", "[rest_e2e][E1]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<Gate> gate = std::make_shared<Gate>(1);
    Release release{gate};
    fixture.set_slice_observer([gate]() { gate->enter("nominal"); });
    fixture.start();
    const Client alice{fixture, login(fixture, "Alice")};
    const nlohmann::json catalog = json_ok(alice.get("/list_settings"));
    REQUIRE(catalog.size() == 5);
    std::size_t definitions = 0;
    for (const char *category : {"print", "printer", "filament", "extruder", "preference"}) {
        REQUIRE(catalog.at(category).is_array());
        definitions += catalog.at(category).size();
    }
    REQUIRE(definitions > 0);
    const std::string route = session(alice, "Before project load");
    const nlohmann::json profiles = json_ok(alice.get(route + "/list_profiles"));
    const nlohmann::json selection = {{"print", profile(profiles, "print", "rest-print")},
        {"printer", profile(profiles, "printer", "rest-printer-b")},
        {"filaments", nlohmann::json::array()}, {"extruder", nlohmann::json::array()}};
    const nlohmann::json selected = json_ok(alice.post(route + "/set_profile", selection));
    CHECK(selected.at("printer") == selection.at("printer"));
    CHECK(selected.at("extruder").empty());
    CHECK(selected.at("filaments") == nlohmann::json::array({profile(profiles, "filaments", "rest-filament-b")}));
    const nlohmann::json defaults = settings(alice, route);
    const nlohmann::json changes = {{"layer_height", {{"value", 0.25}}},
        {"overhangs_width_speed", {{"value", 30}, {"enabled", false}, {"percent", true}}},
        {"filament_travel_pa", {{"value", {12}}, {"enabled", {false}}, {"percent", {false}}}}};
    json_ok(alice.post(route + "/set_settings", changes));
    const nlohmann::json changed = settings(alice, route);
    for (auto field = changes.begin(); field != changes.end(); ++field)
        CHECK(changed.at(field.key()) == field.value());
    REQUIRE(alice.post(route + "/set_settings", {{"layer_height", {{"value", 0.3}}},
        {"filament_travel_pa", {{"value", {12,13}}}}}).result() == http::status::unprocessable_entity);
    CHECK(settings(alice, route) == changed);
    json_ok(alice.post(route + "/set_profile", selection));
    CHECK(settings(alice, route) == defaults);

    cube(alice, route);
    const HttpResponseMessage cube_project = alice.get(route + "/save_project");
    REQUIRE(cube_project.result() == http::status::ok);
    Slic3r::DynamicPrintConfig foreign;
    foreign.set_key_value("layer_height", new Slic3r::ConfigOptionFloat(0.33));
    object_ids(alice.upload(route + "/add_model", "ignored.3mf", make_3mf(1, &foreign)));
    CHECK(settings(alice, route) == defaults);
    CHECK(json_ok(alice.get(route + "/list_objects")).at("objects").size() == 2);
    const std::string project = add_project_config_lines(cube_project.body(),
        "; layer_height = 0.25\n; e2e_foreign_key = preserved\n");
    object_ids(alice.upload(route + "/load_project", "E2E.3mf", project));
    CHECK(json_ok(alice.get(route + "/get_name")).at("name") == "E2E");
    nlohmann::json objects = json_ok(alice.get(route + "/list_objects"));
    REQUIRE(objects.at("objects").size() == 1);
    const std::string object_id = objects["objects"][0]["id"];
    const std::string instance_id = objects["objects"][0]["instances"][0]["id"];
    const std::string part_id = objects["objects"][0]["parts"][0]["id"];
    CHECK(std::set<std::string>{object_id, instance_id, part_id}.size() == 3);
    json_ok(alice.post(route + "/transform/" + object_id, translation(10, "platter")));
    json_ok(alice.post(route + "/transform/" + instance_id, translation(5, "platter")));
    json_ok(alice.post(route + "/transform/" + part_id, translation(1, "volume")));
    std::vector<std::string> three;
    for (int count : {3,1,2}) {
        const nlohmann::json counted = json_ok(alice.post(route + "/set_instance_count/" + object_id, {{"count", count}}));
        const std::vector<std::string> ids = counted.at("instance_ids");
        REQUIRE(ids.size() == std::size_t(count));
        CHECK(ids.front() == instance_id);
        if (count == 3) three = ids;
        if (count == 2) CHECK(ids[1] != three[1]);
        objects = json_ok(alice.get(route + "/list_objects"));
        for (const nlohmann::json &instance : objects["objects"][0]["instances"])
            CHECK(instance.at("matrix") == objects["objects"][0]["instances"][0]["matrix"]);
    }
    const nlohmann::json before = settings(alice, route);
    CHECK(before.at("print_custom_setting").at("value").get<std::string>().find("e2e_foreign_key") != std::string::npos);
    const HttpResponseMessage saved = alice.get(route + "/save_project");
    REQUIRE(saved.result() == http::status::ok);
    CHECK(json_ok(alice.get(route + "/list_objects")) == objects);
    const std::string reloaded = session(alice, "reload");
    object_ids(alice.upload(reloaded + "/load_project", "E2E.3mf", saved.body()));
    same_geometry(objects, json_ok(alice.get(reloaded + "/list_objects")));
    CHECK(settings(alice, reloaded) == before);
    REQUIRE(alice.post(route + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait());
    CHECK(json_ok(alice.get(route + "/state")).at("etat") == "slicing");
    gate->release();
    wait_state(alice, route, "sliced");
    CHECK_FALSE(gate->timed_out());
    sliced(alice, reloaded);
    const HttpResponseMessage gcode = alice.get(route + "/get_gcode");
    REQUIRE(gcode.result() == http::status::ok);
    CHECK(gcode.body().find("\nG1 ") != std::string::npos);
    CHECK(gcode.body().find("; layer_height = 0.25") != std::string::npos);
    CHECK(gcode[http::field::content_disposition].find("E2E.gcode") != beast::string_view::npos);
    const nlohmann::json viewer = read_viewer(alice, route);
    REQUIRE(viewer.at("layers").size() > 2);
    CHECK(viewer["layers"][2]["z_mm"].get<double>() - viewer["layers"][1]["z_mm"].get<double>() == Approx(0.25).margin(1e-4));
    CHECK(viewer["metadata"]["objects"].size() == 2);
    double time = 0;
    for (const nlohmann::json &move : viewer.at("moves")) {
        CHECK(move.at("time_s").get<double>() >= time);
        time = move.at("time_s");
    }
    json_ok(alice.post(route + "/set_setting/layer_height/0.3"));
    require_error(alice.get(route + "/get_gcode"), http::status::conflict, "no_gcode");
    const nlohmann::json stale = read_viewer(alice, route);
    CHECK(stale.at("result_id") == viewer.at("result_id"));
    CHECK(stale.at("outdated") == true);
    sliced(alice, route);
    const nlohmann::json replaced = read_viewer(alice, route);
    CHECK(replaced.at("outdated") == false);
    CHECK(replaced.at("result_id") != viewer.at("result_id"));
    require_error(alice.get(route + "/get_gcode_for_view?offset=1&result_id=" + viewer.at("result_id").get<std::string>()),
        http::status::conflict, "result_replaced");
    CHECK(json_ok(alice.post(route + "/clear_platter")).at("cleared_object_count") == 1);
    require_error(alice.post(route + "/slice"), http::status::conflict, "empty_scene");
    REQUIRE(alice.del(route + "/delete").result() == http::status::no_content);
    require_error(alice.get(route + "/state"), http::status::not_found, "session_not_found");
    clean(fixture, 1); // Only the live reloaded session retains a result.
    REQUIRE(alice.del(reloaded + "/delete").result() == http::status::no_content);
    clean(fixture, 0);
    REQUIRE(alice.post("/logout").result() == http::status::no_content);
    REQUIRE(alice.get("/list_settings").result() == http::status::unauthorized);
}

TEST_CASE("E2 accounts authentication and ownership", "[rest_e2e][E2]")
{
    ensure_engine_initialized();
    SECTION("bad passwords are rate limited even when the sixth password is correct") {
        RestServerFixture fixture({}, true, true);
        for (int attempt = 0; attempt < 5; ++attempt)
            REQUIRE(exchange(fixture, {}, http::verb::post, "/login", R"({"username":"Bob","password":"wrong"})").result() ==
                http::status::unauthorized);
        REQUIRE(exchange(fixture, {}, http::verb::post, "/login", R"({"username":"Bob","password":"secret"})").result() ==
            http::status::too_many_requests);
    }
    SECTION("token expiration") {
        RestServerFixture::Limits limits;
        limits.token_ttl = std::chrono::seconds(2);
        RestServerFixture fixture(limits, true, true);
        const Client alice{fixture, login(fixture, "Alice")};
        REQUIRE(alice.get("/list_settings").result() == http::status::ok);
        REQUIRE(poll([&]() { return alice.get("/list_settings").result() == http::status::unauthorized; }));
    }
    SECTION("targeted logout and isolated sessions") {
        RestServerFixture fixture({}, true, true);
        const Client alice{fixture, login(fixture, "Alice")};
        const Client second{fixture, login(fixture, "Alice")};
        const Client bob{fixture, login(fixture, "Bob")};
        const std::string a = session(alice, "Alice-only"), b = session(bob, "Bob-only");
        const nlohmann::json before = settings(alice, a);
        json_ok(bob.post(b + "/set_setting/layer_height/0.3"));
        json_ok(bob.post(b + "/set_name", {{"name", "Bob-changed"}}));
        CHECK(settings(alice, a) == before);
        CHECK(json_ok(alice.get(a + "/get_name")).at("name") == "Alice-only");
        for (const std::string &operation : {"state", "get_name", "list_objects", "list_profiles", "save_project", "get_gcode", "get_gcode_for_view"}) {
            const HttpResponseMessage foreign = bob.get(a + "/" + operation);
            const HttpResponseMessage missing = bob.get("/session/missing/" + operation);
            REQUIRE(foreign.result() == http::status::not_found);
            nlohmann::json left = nlohmann::json::parse(foreign.body()), right = nlohmann::json::parse(missing.body());
            left["error"].erase("request_id");
            right["error"].erase("request_id");
            CHECK(left == right);
        }
        // Valid bodies reach ownership checks for every mutating public route.
        const nlohmann::json profiles = json_ok(alice.get(a + "/list_profiles"));
        const std::vector<std::pair<std::string, nlohmann::json>> operations = {
            {"get_settings", {{"setting_ids", {"layer_height"}}}},
            {"set_settings", {{"layer_height", {{"value", 0.3}}}}},
            {"set_setting/layer_height/0.3", nullptr},
            {"set_name", {{"name", "Forbidden"}}},
            {"set_profile", {{"print", profile(profiles, "print", "rest-print")},
                {"printer", profile(profiles, "printer", "rest-printer-a")},
                {"filaments", nlohmann::json::array()}, {"extruder", nlohmann::json::array()}}},
            {"transform/missing", translation(0, "platter")},
            {"set_instance_count/missing", {{"count", 2}}},
            {"clear_platter", nullptr}, {"slice", nullptr}, {"cancel_slicing", nullptr}
        };
        for (const std::pair<std::string, nlohmann::json> &operation : operations) {
            const HttpResponseMessage foreign = bob.post(a + "/" + operation.first, operation.second);
            const HttpResponseMessage missing = bob.post("/session/missing/" + operation.first, operation.second);
            REQUIRE(foreign.result() == http::status::not_found);
            REQUIRE(missing.result() == http::status::not_found);
            nlohmann::json left = nlohmann::json::parse(foreign.body()), right = nlohmann::json::parse(missing.body());
            left["error"].erase("request_id");
            right["error"].erase("request_id");
            CHECK(left == right);
        }
        require_error(bob.get(a + "/get_setting/layer_height"), http::status::not_found, "session_not_found");
        require_error(bob.del(a + "/delete"), http::status::not_found, "session_not_found");
        for (const char *operation : {"add_model", "load_project"})
            require_error(bob.upload(a + "/" + operation, "cube.3mf", make_3mf()),
                http::status::not_found, "session_not_found");
        CHECK(settings(alice, a) == before);
        CHECK(json_ok(alice.get(a + "/get_name")).at("name") == "Alice-only");
        REQUIRE(alice.post("/logout").result() == http::status::no_content);
        REQUIRE(alice.get(a + "/state").result() == http::status::unauthorized);
        REQUIRE(second.get(a + "/state").result() == http::status::ok);
    }
}

TEST_CASE("E3 simultaneous engines cancel independently", "[rest_e2e][E3]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    fixture.stop();
    const std::shared_ptr<Gate> gate = std::make_shared<Gate>(2);
    Release release{gate};
    fixture.set_progress_observer([gate](const std::string &id) { gate->enter(id); });
    fixture.start();
    const Client alice{fixture, login(fixture, "Alice")}, bob{fixture, login(fixture, "Bob")};
    const std::string a = session(alice), b = session(bob);
    const std::string ai = cube(alice, a), bi = cube(bob, b);
    json_ok(alice.post(a + "/set_instance_count/" + ai, {{"count", 4}}));
    json_ok(bob.post(b + "/set_instance_count/" + bi, {{"count", 4}}));
    REQUIRE(alice.post(a + "/slice").result() == http::status::accepted);
    REQUIRE(bob.post(b + "/slice").result() == http::status::accepted);
    REQUIRE(gate->wait());
    for (const nlohmann::json &state : {json_ok(alice.get(a + "/state")), json_ok(bob.get(b + "/state"))}) {
        CHECK(state.at("etat") == "slicing");
        CHECK((state.at("primary_percentage").get<int>() >= 0 || state.at("secondary_percentage").get<int>() >= 0));
    }
    REQUIRE(alice.post(a + "/cancel_slicing").result() == http::status::accepted);
    CHECK(json_ok(alice.get(a + "/state")).at("etat") == "slicing");
    gate->release();
    wait_state(alice, a, "ready");
    wait_state(bob, b, "sliced");
    CHECK_FALSE(gate->timed_out());
    REQUIRE(bob.get(b + "/get_gcode").result() == http::status::ok);
    require_error(alice.get(a + "/get_gcode"), http::status::conflict, "no_gcode");
    clean(fixture, 1);
    sliced(alice, a);
    clean(fixture, 2);
    REQUIRE(alice.del(a + "/delete").result() == http::status::no_content);
    REQUIRE(bob.del(b + "/delete").result() == http::status::no_content);
    clean(fixture, 0);
}

TEST_CASE("E4 post processing is inert from all configuration sources", "[rest_e2e][E4]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);
    TempDirectory scripts;
    const std::filesystem::path witness = scripts.path() / "witness.txt";
#ifdef _WIN32
    const std::filesystem::path script = scripts.path() / "witness.cmd";
    std::ofstream(script) << "@echo witness > \"" << witness.string() << "\"\r\n";
#else
    const std::filesystem::path script = scripts.path() / "witness.sh";
    std::ofstream(script) << "#!/bin/sh\nprintf witness > '" << witness.string() << "'\n";
    std::filesystem::permissions(script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);
#endif
    fixture.stop();
    std::ofstream(fixture.profiles_directory() / "print" / "e2e-witness.ini") << "post_process = " << script.generic_string() << "\n";
    fixture.start();
    const Client alice{fixture, login(fixture, "Alice")};
    const std::string route = session(alice);
    cube(alice, route);
    const HttpResponseMessage project = alice.get(route + "/save_project");
    REQUIRE(project.result() == http::status::ok);
    const nlohmann::json profiles = json_ok(alice.get(route + "/list_profiles"));
    json_ok(alice.post(route + "/set_profile", {{"print", profile(profiles, "print", "e2e-witness")},
        {"printer", profile(profiles, "printer", "rest-printer-a")},
        {"filaments", nlohmann::json::array()}, {"extruder", nlohmann::json::array()}}));
    CHECK(json_ok(alice.get(route + "/get_setting/post_process")).at("serialized_value").get<std::string>().find(script.generic_string()) != std::string::npos);
    sliced(alice, route);
    REQUIRE(alice.get(route + "/get_gcode").result() == http::status::ok);
    CHECK_FALSE(std::filesystem::exists(witness));
    json_ok(alice.post(route + "/set_profile", {{"print", profile(profiles, "print", "rest-print")},
        {"printer", profile(profiles, "printer", "rest-printer-a")},
        {"filaments", nlohmann::json::array()}, {"extruder", nlohmann::json::array()}}));
    std::string encoded;
    for (unsigned char character : script.generic_string()) {
        const char digits[] = "0123456789ABCDEF";
        encoded += '%';
        encoded += digits[character >> 4];
        encoded += digits[character & 15];
    }
    json_ok(alice.post(route + "/set_setting/post_process/" + encoded));
    CHECK(json_ok(alice.get(route + "/get_setting/post_process")).at("serialized_value").get<std::string>().find(script.generic_string()) != std::string::npos);
    sliced(alice, route);
    REQUIRE(alice.get(route + "/get_gcode").result() == http::status::ok);
    CHECK_FALSE(std::filesystem::exists(witness));
    object_ids(alice.upload(route + "/load_project", "witness.3mf", add_project_config_lines(project.body(),
        "; post_process = " + script.generic_string() + "\n")));
    CHECK(json_ok(alice.get(route + "/get_setting/post_process")).at("serialized_value").get<std::string>().find(script.generic_string()) != std::string::npos);
    sliced(alice, route);
    REQUIRE(alice.get(route + "/get_gcode").result() == http::status::ok);
    CHECK_FALSE(std::filesystem::exists(witness));
    REQUIRE(alice.del(route + "/delete").result() == http::status::no_content);
    clean(fixture, 0);
}

TEST_CASE("E5 expiration shutdown and restart", "[rest_e2e][E5]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.session_ttl = std::chrono::seconds(2);
    RestServerFixture fixture(limits, true, true);
    SECTION("inactive result is swept without session requests") {
        const Client alice{fixture, login(fixture, "Alice")};
        const std::string route = session(alice);
        cube(alice, route);
        sliced(alice, route);
        const WorkInventory inventory = work_inventory(fixture);
        REQUIRE_FALSE(inventory.error);
        REQUIRE(inventory.results.size() == 1);
        clean(fixture, 0);
        require_error(alice.get(route + "/state"), http::status::not_found, "session_not_found");
    }
    SECTION("work suspends expiration then restarts its inactivity window") {
        fixture.stop();
        const std::shared_ptr<Gate> gate = std::make_shared<Gate>(1);
        Release release{gate};
        fixture.set_slice_observer([gate]() { gate->enter("expiring"); });
        fixture.start();
        const Client alice{fixture, login(fixture, "Alice")};
        const std::string route = session(alice);
        cube(alice, route);
        REQUIRE(alice.post(route + "/slice").result() == http::status::accepted);
        REQUIRE(gate->wait());
        const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
        REQUIRE(poll([&]() {
            CHECK(json_ok(alice.get(route + "/state")).at("etat") == "slicing");
            return std::chrono::steady_clock::now() - started >= std::chrono::seconds(4);
        }));
        gate->release();
        wait_state(alice, route, "sliced");
        REQUIRE(alice.get(route + "/get_gcode").result() == http::status::ok);
        clean(fixture, 0);
        require_error(alice.get(route + "/state"), http::status::not_found, "session_not_found");
        CHECK_FALSE(gate->timed_out());
    }
    SECTION("stop joins active worker cleans storage and forgets tokens and sessions") {
        fixture.stop();
        const std::shared_ptr<Gate> gate = std::make_shared<Gate>(1);
        Release release{gate};
        fixture.set_progress_observer([gate](const std::string &id) { gate->enter(id); });
        fixture.start();
        const Client old{fixture, login(fixture, "Alice")};
        const std::string route = session(old);
        cube(old, route);
        REQUIRE(old.post(route + "/slice").result() == http::status::accepted);
        REQUIRE(gate->wait());
        std::promise<void> stopping;
        std::future<void> stop = std::async(std::launch::async, [&]() {
            stopping.set_value();
            fixture.stop();
        });
        REQUIRE(stopping.get_future().wait_for(work_timeout) == std::future_status::ready);
        CHECK(stop.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);
        gate->release();
        REQUIRE(stop.wait_for(work_timeout) == std::future_status::ready);
        stop.get();
        clean(fixture, 0);
        fixture.start();
        REQUIRE(old.get(route + "/state").result() == http::status::unauthorized);
        const Client alice{fixture, login(fixture, "Alice")};
        require_error(alice.get(route + "/state"), http::status::not_found, "session_not_found");
        const std::string fresh = session(alice);
        cube(alice, fresh);
        sliced(alice, fresh);
        REQUIRE(alice.del(fresh + "/delete").result() == http::status::no_content);
        clean(fixture, 0);
        CHECK_FALSE(gate->timed_out());
    }
}

TEST_CASE("E6 rejected operations are atomic and leave no files", "[rest_e2e][E6]")
{
    ensure_engine_initialized();
    RestServerFixture::Limits limits;
    limits.max_upload_bytes = 32 * 1024;
    limits.max_instances_per_object = 4;
    RestServerFixture fixture(limits, true, true);
    const Client alice{fixture, login(fixture, "Alice")};
    const std::string route = session(alice), id = cube(alice, route);
    const nlohmann::json before_objects = json_ok(alice.get(route + "/list_objects"));
    const nlohmann::json before_settings = settings(alice, route);
    // Check immediately after each rejection, not only after the whole sequence.
    const auto unchanged = [&]() {
        CHECK(json_ok(alice.get(route + "/list_objects")) == before_objects);
        CHECK(settings(alice, route) == before_settings);
        clean(fixture, 0);
    };
    REQUIRE(alice.upload(route + "/add_model", "large.stl", std::string(40 * 1024, 'x')).result() == http::status::payload_too_large);
    unchanged();
    REQUIRE(alice.upload(route + "/add_model", "bad.txt", "invalid").result() == http::status::unprocessable_entity);
    unchanged();
    REQUIRE(exchange(fixture, alice.token, http::verb::post, route + "/add_model", "broken",
        "multipart/form-data; boundary=" + test_boundary).result() == http::status::bad_request);
    unchanged();
    require_error(alice.upload(route + "/load_project", "bad.3mf", "not a zip"), http::status::unprocessable_entity, "invalid_project");
    unchanged();
    nlohmann::json bad = translation(0, "platter");
    bad["matrix"][0] = 0;
    REQUIRE(alice.post(route + "/transform/" + id, bad).result() == http::status::unprocessable_entity);
    unchanged();
    REQUIRE(alice.post(route + "/set_instance_count/" + id, {{"count", 5}}).result() == http::status::unprocessable_entity);
    unchanged();
    REQUIRE(alice.post(route + "/set_settings", {{"layer_height", {{"value", "invalid"}}}}).result() == http::status::unprocessable_entity);
    unchanged();
}
