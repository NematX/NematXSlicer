/*
 * Integration tests for the REST profile catalog and session selection.
 *
 * The fixture uses empty profile INIs so the finalized engine supplies the
 * configuration values, while printer INIs explicitly connect default
 * filament profiles to exercise the catalog mapping and isolation rules.
 */

#include <catch_main.hpp>
#include <nlohmann/json.hpp>

#include "engine.hpp"
#include "profiles.hpp"
#include "rest_profiles_test_access.hpp"
#include "rest_server_fixture.hpp"
#include "rest_vendor_test_helpers.hpp"
#include "rest_models_test_helpers.hpp"
#include <boost/nowide/cstdlib.hpp>
#include <chrono>
#include <iostream>

#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace RestServerTest;

namespace {

std::string profile_id(const nlohmann::json &profiles, const std::string &name)
{
    for (const nlohmann::json &profile : profiles)
        if (profile.at("name") == name)
            return profile.at("id").get<std::string>();
    return {};
}

std::string selection_body(
    const std::string &print,
    const std::string &printer,
    const std::vector<std::string> &extruder,
    const std::vector<std::string> &filaments)
{
    return nlohmann::json{
        {"print", print},
        {"printer", printer},
        {"extruder", extruder},
        {"filaments", filaments}
    }.dump();
}

} // namespace

TEST_CASE("REST default profile configuration passes engine validation", "[rest_profiles]")
{
    ensure_engine_initialized();
    TempDirectory profiles_root;
    std::filesystem::create_directories(profiles_root.path() / "print");
    std::filesystem::create_directories(profiles_root.path() / "printer");
    std::filesystem::create_directories(profiles_root.path() / "filament");
    std::ofstream(profiles_root.path() / "print" / "rest-print.ini");
    std::ofstream(profiles_root.path() / "printer" / "rest-printer-a.ini")
        << "default_filament_profile = rest-filament-a\n";
    std::ofstream(profiles_root.path() / "printer" / "a-sla-printer.ini")
        << "printer_technology = SLA\n";
    std::ofstream(profiles_root.path() / "filament" / "rest-filament-a.ini");

    Slic3r::Rest::ProfileCatalog catalog(profiles_root.path());
    const nlohmann::json listed = catalog.list_profiles();
    REQUIRE(listed.at("printer").size() == 1);
    REQUIRE(profile_id(listed.at("printer"), "a-sla-printer").empty());
    const std::string fff_printer = profile_id(listed.at("printer"), "rest-printer-a");
    REQUIRE(!fff_printer.empty());
    const Slic3r::Rest::ProfileApplication defaults = catalog.defaults();
    REQUIRE(defaults.selection.printer == fff_printer);
    Slic3r::DynamicPrintConfig configuration(defaults.configuration);
    REQUIRE(configuration.validate().empty());
}

TEST_CASE("REST vendor catalogs resolve inheritance preserve metadata and merge first wins", "[rest_vendor]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path());
    std::filesystem::create_directories(root.path() / "print");
    std::ofstream(root.path() / "print/User.ini") << "inherits = 0.20mm NORMAL .4N\nlayer_height = 0.15\n";
    std::ofstream(root.path() / "print/0.20mm NORMAL .4N.ini") << "layer_height = 0.9\n";
    Slic3r::Rest::ProfileCatalog catalog(root.path());
    const nlohmann::json listed = catalog.list_profiles();
    for (const char *category : {"print", "printer", "filaments"})
        for (const nlohmann::json &entry : listed.at(category)) {
            CHECK(entry.at("name").get<std::string>().front() != '*');
            if (entry.at("name") == "User")
                CHECK_FALSE(entry.contains("vendor"));
            else
                CHECK(entry.at("vendor") == (entry.at("name").get<std::string>().rfind("Test", 0) == 0 ? "TestVendor" : "Custom"));
        }
    Slic3r::Rest::ProfileSelection basic;
    basic.printer = profile_id(listed.at("printer"), "Basic 0.4 nozzle");
    basic.print = profile_id(listed.at("print"), "0.20mm NORMAL .4N");
    basic.filaments = {profile_id(listed.at("filaments"), "Basic PLA @BASIC")};
    REQUIRE_FALSE(basic.printer.empty());
    REQUIRE_FALSE(basic.print.empty());
    REQUIRE_FALSE(basic.filaments.front().empty());
    CHECK(profile_id(listed.at("filaments"), "Basic PLA").empty());
    const Slic3r::Rest::ProfileApplication application = catalog.select(basic);
    CHECK(application.configuration.opt_float("layer_height") == Approx(0.2));
    CHECK(application.configuration.opt_int("perimeters") == 3);
    CHECK(application.configuration.opt_float("nozzle_diameter", 0) == Approx(0.4));
    basic.print = profile_id(listed.at("print"), "User");
    CHECK(catalog.select(basic).configuration.opt_float("layer_height") == Approx(0.15));
    Slic3r::Rest::ProfileSelection test;
    test.printer = profile_id(listed.at("printer"), "Test 0.4");
    test.print = profile_id(listed.at("print"), "Test print @TEST");
    test.filaments = {profile_id(listed.at("filaments"), "Test PLA @TEST")};
    const Slic3r::Rest::ProfileApplication test_application = catalog.select(test);
    CHECK(test_application.configuration.opt_string("print_custom_setting").find("test_future_key") != std::string::npos);
    // A later vendor uses the same names but different values. The original wins.
    std::string duplicate;
    { std::ifstream input(root.path() / "vendor/Test.ini");
      duplicate.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()); }
    const std::size_t value = duplicate.find("layer_height = 0.2");
    REQUIRE(value != std::string::npos);
    duplicate.replace(value, std::string("layer_height = 0.2").size(), "layer_height = 0.3");
    std::ofstream(root.path() / "vendor/Test2.ini") << duplicate;
    Slic3r::Rest::ProfileCatalog merged(root.path());
    REQUIRE(merged.warnings().size() == 1);
    CHECK(merged.warnings().front().find("Test print @TEST") != std::string::npos);
    CHECK(merged.select(test).configuration.opt_float("layer_height") == Approx(0.2));
    std::ofstream(root.path() / "vendor/Invalid.ini") << "[print:invalid]\nlayer_height = 0.2\n";
    CHECK_THROWS_WITH(Slic3r::Rest::ProfileCatalog(root.path()), Catch::Matchers::Contains("Invalid.ini"));
}

TEST_CASE("REST vendor bundles ignored by the GUI warn without preventing startup", "[rest_vendor]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path(), false);
    const std::filesystem::path templates = std::filesystem::path(SLIC3R_REST_SOURCE_DIRECTORY) /
        "resources/profiles/Templates.ini";
    REQUIRE(std::filesystem::is_regular_file(templates));
    // Test both the shipped name and a skipped first file in filename order.
    for (const char *name : {"Templates.ini", "00-Templates.ini"}) {
        const std::filesystem::path target = root.path() / "vendor" / name;
        std::filesystem::copy_file(templates, target);
        Slic3r::Rest::ProfileCatalog catalog(root.path());
        REQUIRE(catalog.warnings().size() == 1);
        CHECK(catalog.warnings().front() == std::string("profiles/vendor/") + name +
            ": ignored, no printer model or template metadata");
        CHECK_FALSE(profile_id(catalog.list_profiles()["printer"], "Basic 0.4 nozzle").empty());
        RestServerFixture fixture({}, true, false, "work", {}, root.path());
        const std::string route = "/session/" + create_session(fixture);
        const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_profiles");
        REQUIRE(response.result() == http::status::ok);
        CHECK_FALSE(profile_id(nlohmann::json::parse(response.body())["filaments"], "Basic PLA @BASIC").empty());
        std::filesystem::remove(target);
    }
    std::ofstream(root.path() / "vendor/Broken.ini") << "[vendor\nname = Broken\n";
    CHECK_THROWS_WITH(Slic3r::Rest::ProfileCatalog(root.path()), Catch::Matchers::Contains("Broken.ini"));
}

TEST_CASE("REST vendor compatibility checks printer print and anonymous session configurations", "[rest_vendor]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path());
    Slic3r::Rest::ProfileCatalog catalog(root.path());
    const nlohmann::json all = catalog.list_profiles();
    const std::string normal = profile_id(all.at("print"), "Test print @TEST");
    const std::string fine = profile_id(all.at("print"), "Test fine .25N");
    const std::string printer4 = profile_id(all.at("printer"), "Test 0.4");
    const std::string printer25 = profile_id(all.at("printer"), "Test 0.25");
    for (const bool small : {false, true}) {
        Slic3r::Rest::ProfileCatalog::CompatibilityContext context{
            catalog.printer_by_id(small ? printer25 : printer4), catalog.print_by_id(small ? fine : normal)};
        REQUIRE(context.printer != nullptr);
        REQUIRE(context.print != nullptr);
        const nlohmann::json listed = catalog.list_profiles(context, false);
        for (const nlohmann::json &entry : listed.at("print")) {
            if (entry.at("id") == normal)
                CHECK(entry.at("compatible") == !small);
            if (entry.at("id") == fine)
                CHECK(entry.at("compatible") == small);
        }
        for (const nlohmann::json &entry : listed.at("filaments"))
            if (entry.at("name") == "Test PLA @TEST")
                CHECK(entry.at("compatible") == !small);
        const nlohmann::json filtered = catalog.list_profiles(context, true);
        for (const char *category : {"print", "filaments"})
            for (const nlohmann::json &entry : filtered.at(category))
                CHECK(entry.at("compatible") == true);
        CHECK(filtered.at("printer") == all.at("printer"));
        Slic3r::Rest::SessionCompatibilityPreset anonymous(Slic3r::Preset::TYPE_PRINTER, context.printer->config);
        context.printer = &anonymous;
        const nlohmann::json embedded = catalog.list_profiles(context, false);
        for (const nlohmann::json &entry : embedded.at("print"))
            if (entry.at("id") == normal)
                CHECK(entry.at("compatible") == !small);
    }
}

TEST_CASE("REST list_profiles compatibility query validates before session and resolves catalog overrides", "[rest_vendor]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path());
    RestServerFixture fixture({}, true, false, "work", {}, root.path());
    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string route = "/session/" + nlohmann::json::parse(created.body()).at("session_id").get<std::string>() + "/list_profiles";
    const HttpResponseMessage response = fixture.request(http::verb::get, route);
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json listed = nlohmann::json::parse(response.body());
    for (const char *category : {"print", "filaments"})
        for (const nlohmann::json &entry : listed.at(category))
            CHECK(entry.at("compatible").is_boolean());
    for (const nlohmann::json &entry : listed.at("printer"))
        CHECK_FALSE(entry.contains("compatible"));
    const std::string printer4 = profile_id(listed.at("printer"), "Test 0.4");
    const std::string printer25 = profile_id(listed.at("printer"), "Test 0.25");
    const std::string normal = profile_id(listed.at("print"), "Test print @TEST");
    const std::string fine = profile_id(listed.at("print"), "Test fine .25N");
    const nlohmann::json large = nlohmann::json::parse(fixture.request(http::verb::get,
        route + "?printer=" + printer4 + "&print=" + normal).body());
    const nlohmann::json small = nlohmann::json::parse(fixture.request(http::verb::get,
        route + "?printer=" + printer25 + "&print=" + fine).body());
    const HttpResponseMessage printer_only_response = fixture.request(http::verb::get, route + "?printer=" + printer25);
    REQUIRE(printer_only_response.result() == http::status::ok);
    const nlohmann::json printer_only = nlohmann::json::parse(printer_only_response.body());
    for (const nlohmann::json &entry : printer_only["print"]) {
        if (entry["id"] == fine)
            CHECK(entry["compatible"] == true);
        if (entry["id"] == normal)
            CHECK(entry["compatible"] == false);
    }
    CHECK(large != small);
    for (const nlohmann::json &entry : small.at("filaments"))
        if (entry.at("name") == "Test PLA @TEST")
            CHECK(entry.at("compatible") == false);
    for (const char *category : {"print", "filaments"}) {
        const nlohmann::json filtered = nlohmann::json::parse(fixture.request(http::verb::get,
            route + "?printer=" + printer4 + "&print=" + normal + "&compatible_only=1").body());
        for (const nlohmann::json &entry : filtered.at(category))
            CHECK(entry.at("compatible") == true);
    }
    CHECK(nlohmann::json::parse(fixture.request(http::verb::get, route + "?compatible_only=0").body()) == listed);
    for (const std::string &query : {"?printer=", "?print=", "?compatible_only=2", "?other=1", "?printer=x&printer=y"}) {
        const HttpResponseMessage error = fixture.request(http::verb::get, route + query);
        CHECK(error.result() == http::status::bad_request);
        CHECK(nlohmann::json::parse(error.body())["error"]["code"] == "invalid_request");
        CHECK(fixture.request(http::verb::get, "/session/missing/list_profiles" + query).result() == http::status::bad_request);
    }
    for (const std::string &query : {"?printer=missing", "?print=missing"}) {
        const HttpResponseMessage error = fixture.request(http::verb::get, route + query);
        CHECK(error.result() == http::status::unprocessable_entity);
        CHECK(nlohmann::json::parse(error.body())["error"]["code"] == "invalid_profile");
    }
    CHECK(fixture.request(http::verb::get, route, "{}", "application/json").result() == http::status::bad_request);
    CHECK(fixture.request(http::verb::post, route).result() == http::status::not_found);
    const std::vector<std::pair<std::string, std::string>> bob = {{"Authorization", "Bearer " + fixture.login_token("Bob", "secret")}};
    CHECK(fixture.request(http::verb::get, route, {}, {}, bob).result() == http::status::not_found);
}

TEST_CASE("REST complete vendor catalog copy costs", "[.rest_profiles_bench]")
{
    ensure_engine_initialized();
    const char *source = boost::nowide::getenv("REST_VENDOR_PROFILES_DIR");
    REQUIRE(source != nullptr);
    TempDirectory root;
    std::filesystem::create_directories(root.path() / "vendor");
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(std::filesystem::u8path(source)))
        if (entry.is_regular_file() && entry.path().extension() == ".ini")
            std::filesystem::copy_file(entry.path(), root.path() / "vendor" / entry.path().filename());
    const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Slic3r::Rest::ProfileCatalog catalog(root.path());
    const std::chrono::steady_clock::time_point loaded = std::chrono::steady_clock::now();
    const Slic3r::Rest::ProfileSelection selection = catalog.defaults().selection;
    const std::chrono::steady_clock::time_point select_begin = std::chrono::steady_clock::now();
    for (int index = 0; index < 20; ++index)
        REQUIRE_FALSE(catalog.select(selection).selection.printer.empty());
    const std::chrono::steady_clock::time_point defaults_begin = std::chrono::steady_clock::now();
    for (int index = 0; index < 20; ++index)
        REQUIRE_FALSE(catalog.defaults().selection.printer.empty());
    const std::chrono::steady_clock::time_point list_begin = std::chrono::steady_clock::now();
    const nlohmann::json listed = catalog.list_profiles();
    const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // The copy alone, then the reduced bundle a default selection needs.
    const Slic3r::PresetBundle &bundle = Slic3r::Rest::ProfileCatalogTestAccess::bundle(catalog);
    const std::chrono::steady_clock::time_point copy_begin = std::chrono::steady_clock::now();
    for (int index = 0; index < 20; ++index) {
        const Slic3r::PresetBundle copy = bundle;
        REQUIRE(copy.filaments.size() == bundle.filaments.size());
    }
    const std::chrono::steady_clock::time_point subset_begin = std::chrono::steady_clock::now();
    Slic3r::PresetBundle::PresetSubset names;
    std::size_t reduced_presets = 0;
    for (int index = 0; index < 20; ++index) {
        names.printers = {bundle.printers.get_selected_preset_name()};
        names.prints = {bundle.fff_prints.get_selected_preset_name()};
        names.filaments = bundle.compatible_filament_names(
            bundle.printers.get_selected_preset(), &bundle.fff_prints.get_selected_preset());
        const Slic3r::PresetBundle reduced = bundle.subset(names);
        reduced_presets = reduced.fff_prints.size() + reduced.filaments.size() + reduced.printers.size();
    }
    const std::chrono::steady_clock::time_point subset_end = std::chrono::steady_clock::now();
    std::cout << "vendor benchmark: copy_mean_ms="
        << std::chrono::duration<double, std::milli>(subset_begin - copy_begin).count() / 20
        << " subset_mean_ms=" << std::chrono::duration<double, std::milli>(subset_end - subset_begin).count() / 20
        << " catalog_presets=" << bundle.fff_prints.size() + bundle.filaments.size() + bundle.printers.size()
        << " reduced_presets=" << reduced_presets << " compatible_filaments=" << names.filaments.size() << std::endl;

    // Every printer with its default print: the worst case has the most
    // compatible filaments.
    std::map<std::string, std::string> print_ids;
    for (const nlohmann::json &print : listed["print"])
        print_ids.emplace(print["name"].get<std::string>(), print["id"].get<std::string>());
    double slowest_ms = 0.;
    const Slic3r::Preset *slowest = nullptr;
    double total_ms = 0.;
    std::size_t selections = 0;
    std::size_t most_filaments = 0;
    for (const nlohmann::json &printer : listed["printer"]) {
        const Slic3r::Preset *preset = catalog.printer_by_id(printer["id"].get<std::string>());
        const std::map<std::string, std::string>::const_iterator print =
            print_ids.find(preset->config.opt_string("default_print_profile"));
        if (print == print_ids.end())
            continue;
        Slic3r::Rest::ProfileSelection request;
        request.printer = printer["id"].get<std::string>();
        request.print = print->second;
        const std::chrono::steady_clock::time_point one_begin = std::chrono::steady_clock::now();
        try {
            catalog.select(request);
        } catch (const std::invalid_argument &) {
            // An incompatible default print still measures the work done.
        }
        const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - one_begin).count();
        if (elapsed > slowest_ms)
            slowest = preset;
        slowest_ms = std::max(slowest_ms, elapsed);
        total_ms += elapsed;
        ++selections;
        most_filaments = std::max(most_filaments, bundle.compatible_filament_names(*preset,
            bundle.fff_prints.find_preset(print->first, false, false)).size());
    }
    std::cout << "vendor benchmark: printers_selected=" << selections << " select_all_mean_ms="
        << (selections == 0 ? 0. : total_ms / selections) << " select_all_max_ms=" << slowest_ms
        << " most_compatible_filaments=" << most_filaments << std::endl;
    if (slowest != nullptr) {
        const Slic3r::Preset *print = bundle.fff_prints.find_preset(slowest->config.opt_string("default_print_profile"), false, false);
        const std::chrono::steady_clock::time_point names_begin = std::chrono::steady_clock::now();
        Slic3r::PresetBundle::PresetSubset worst;
        worst.printers = {slowest->name};
        worst.prints = {print->name};
        worst.filaments = bundle.compatible_filament_names(*slowest, print);
        const std::chrono::steady_clock::time_point worst_subset_begin = std::chrono::steady_clock::now();
        const Slic3r::PresetBundle reduced = bundle.subset(worst);
        const std::chrono::steady_clock::time_point worst_end = std::chrono::steady_clock::now();
        std::cout << "vendor benchmark: slowest=\"" << slowest->name << "\" extruders="
            << slowest->config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter")->size()
            << " filaments=" << worst.filaments.size() << " names_ms="
            << std::chrono::duration<double, std::milli>(worst_subset_begin - names_begin).count() << " subset_ms="
            << std::chrono::duration<double, std::milli>(worst_end - worst_subset_begin).count() << std::endl;
    }
    std::cout << "vendor benchmark: load_ms=" << std::chrono::duration<double, std::milli>(loaded - begin).count()
        << " select_mean_ms=" << std::chrono::duration<double, std::milli>(defaults_begin - select_begin).count() / 20
        << " defaults_mean_ms=" << std::chrono::duration<double, std::milli>(list_begin - defaults_begin).count() / 20
        << " compatibility_list_ms=" << std::chrono::duration<double, std::milli>(end - list_begin).count()
        << " warnings=" << catalog.warnings().size() << " profiles="
        << listed["printer"].size() + listed["print"].size() + listed["filaments"].size() << std::endl;
}

TEST_CASE("REST vendor project identities survive unchanged and anonymous profiles evaluate conditions over TCP", "[rest_vendor]")
{
    ensure_engine_initialized();
    TempDirectory root;
    write_vendor_profiles(root.path());
    Slic3r::Rest::ProfileCatalog catalog(root.path());
    const nlohmann::json listed = catalog.list_profiles();
    Slic3r::Rest::ProfileSelection basic;
    basic.printer = profile_id(listed["printer"], "Basic 0.4 nozzle");
    basic.print = profile_id(listed["print"], "0.20mm NORMAL .4N");
    basic.filaments = {profile_id(listed["filaments"], "Basic PLA @BASIC")};
    const Slic3r::Rest::ProfileApplication original = catalog.select(basic);
    RestServerFixture fixture({}, true, false, "work", {}, root.path());
    const std::string route = "/session/" + create_session(fixture);
    REQUIRE(upload_model(fixture, route + "/load_project", "basic.3mf", make_3mf(1, &original.configuration)).result() == http::status::ok);
    const Slic3r::Rest::ProfileApplication unchanged = catalog.load_project(original.configuration, "basic.3mf");
    CHECK(unchanged.selection.print == basic.print);
    CHECK(unchanged.selection.printer == basic.printer);
    CHECK(unchanged.selection.filaments == basic.filaments);
    Slic3r::DynamicPrintConfig changed = original.configuration;
    changed.set_deserialize_strict("layer_height", "0.15");
    changed.set_deserialize_strict("nozzle_diameter", "0.25");
    REQUIRE(upload_model(fixture, route + "/load_project", "changed.3mf", make_3mf(1, &changed)).result() == http::status::ok);
    const Slic3r::Rest::ProfileApplication embedded = catalog.load_project(changed, "changed.3mf");
    CHECK(embedded.selection.print.empty());
    CHECK(embedded.selection.printer.empty());
    const HttpResponseMessage response = fixture.request(http::verb::get, route + "/list_profiles");
    REQUIRE(response.result() == http::status::ok);
    const nlohmann::json compatible = nlohmann::json::parse(response.body());
    bool saw_normal = false;
    bool saw_fine = false;
    for (const nlohmann::json &entry : compatible["print"]) {
        if (entry["name"] == "Test print @TEST") {
            saw_normal = true;
            CHECK(entry["compatible"] == false);
        }
        if (entry["name"] == "Test fine .25N") {
            saw_fine = true;
            CHECK(entry["compatible"] == true);
        }
    }
    REQUIRE(saw_normal);
    REQUIRE(saw_fine);
}

TEST_CASE("REST profile IDs are opaque and selections are isolated", "[rest_profiles]")
{
    ensure_engine_initialized();
    RestServerFixture fixture({}, true, true);

    const HttpResponseMessage first = fixture.request(http::verb::post, "/create_session");
    const HttpResponseMessage second = fixture.request(http::verb::post, "/create_session");
    REQUIRE(first.result() == http::status::created);
    REQUIRE(second.result() == http::status::created);
    const std::string first_id = nlohmann::json::parse(first.body()).at("session_id");
    const std::string second_id = nlohmann::json::parse(second.body()).at("session_id");
    const std::string first_route = "/session/" + first_id;
    const std::string second_route = "/session/" + second_id;
    const std::map<std::string, std::string> profile_files_before = fixture.profile_files();

    REQUIRE(fixture.request_without_auth(http::verb::get, first_route + "/list_profiles").result() ==
        http::status::unauthorized);

    const HttpResponseMessage listed = fixture.request(http::verb::get, first_route + "/list_profiles");
    REQUIRE(listed.result() == http::status::ok);
    const nlohmann::json catalog = nlohmann::json::parse(listed.body());
    REQUIRE(catalog.size() == 4);
    REQUIRE(catalog.at("extruder").empty());
    REQUIRE(catalog.at("print").size() == 1);
    REQUIRE(catalog.at("printer").size() == 2);
    REQUIRE(catalog.at("filaments").size() == 2);
    REQUIRE(profile_id(catalog.at("printer"), "a-sla-printer").empty());
    REQUIRE(fixture.profile_files() == profile_files_before);

    const std::string print = profile_id(catalog.at("print"), "rest-print");
    const std::string printer_a = profile_id(catalog.at("printer"), "rest-printer-a");
    const std::string printer_b = profile_id(catalog.at("printer"), "rest-printer-b");
    const std::string filament_a = profile_id(catalog.at("filaments"), "rest-filament-a");
    const std::string filament_b = profile_id(catalog.at("filaments"), "rest-filament-b");
    REQUIRE(!print.empty());
    REQUIRE(!printer_a.empty());
    REQUIRE(!printer_b.empty());
    REQUIRE(!filament_a.empty());
    REQUIRE(!filament_b.empty());
    REQUIRE(print != "rest-print");
    REQUIRE(printer_a != "rest-printer-a");
    REQUIRE(filament_a != "rest-filament-a");
    for (const char *category : {"print", "printer", "filaments"})
        for (const nlohmann::json &profile : catalog.at(category)) {
            REQUIRE(profile.size() == (std::string(category) == "printer" ? 2 : 3));
            REQUIRE(profile.contains("id"));
            REQUIRE(profile.contains("name"));
            REQUIRE(profile.at("id") != profile.at("name"));
        }

    const HttpResponseMessage first_default = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {}, {}),
        "application/json");
    REQUIRE(first_default.result() == http::status::ok);
    const nlohmann::json first_default_selection = nlohmann::json::parse(first_default.body());
    REQUIRE(first_default_selection.at("printer") == printer_a);
    REQUIRE(first_default_selection.at("filaments") ==
        nlohmann::json::array({filament_a}));
    REQUIRE(fixture.profile_files() == profile_files_before);

    const HttpResponseMessage second_default = fixture.request(
        http::verb::post,
        second_route + "/set_profile",
        selection_body(print, printer_b, {}, {}),
        "application/json");
    REQUIRE(second_default.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(second_default.body()).at("filaments") ==
        nlohmann::json::array({filament_b}));

    const HttpResponseMessage first_again = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {}, {}),
        "application/json");
    REQUIRE(first_again.result() == http::status::ok);
    const nlohmann::json first_effective = nlohmann::json::parse(first_again.body());
    REQUIRE(first_effective.at("print") == print);
    REQUIRE(first_effective.at("printer") == printer_a);
    REQUIRE(first_effective.at("extruder").empty());
    REQUIRE(first_effective.at("filaments") == nlohmann::json::array({filament_a}));

    const HttpResponseMessage listed_again = fixture.request(
        http::verb::get,
        first_route + "/list_profiles");
    REQUIRE(listed_again.result() == http::status::ok);
    REQUIRE(nlohmann::json::parse(listed_again.body()) == catalog);

    const HttpResponseMessage explicit_selection = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {}, {filament_a}),
        "application/json");
    REQUIRE(explicit_selection.result() == http::status::ok);

    const HttpResponseMessage independent_extruder = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {"unused"}, {filament_a}),
        "application/json");
    REQUIRE(independent_extruder.result() == http::status::unprocessable_entity);

    const HttpResponseMessage wrong_count = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {}, {filament_a, filament_b}),
        "application/json");
    REQUIRE(wrong_count.result() == http::status::unprocessable_entity);

    const HttpResponseMessage unknown_profile = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body("profile_missing", printer_a, {}, {filament_a}),
        "application/json");
    REQUIRE(unknown_profile.result() == http::status::unprocessable_entity);

    const HttpResponseMessage malformed = fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        R"({"print":"profile_missing"})",
        "application/json");
    REQUIRE(malformed.result() == http::status::bad_request);
    REQUIRE(fixture.running());

    const std::string bob_token = fixture.login_token("Bob", "secret");
    const std::vector<std::pair<std::string, std::string>> bob_auth = {
        {"Authorization", "Bearer " + bob_token}
    };
    REQUIRE(fixture.request(
        http::verb::get,
        first_route + "/list_profiles",
        {},
        {},
        bob_auth).result() == http::status::not_found);
    REQUIRE(fixture.request(
        http::verb::post,
        first_route + "/set_profile",
        selection_body(print, printer_a, {}, {filament_a}),
        "application/json",
        bob_auth).result() == http::status::not_found);
    REQUIRE(fixture.request(
        http::verb::get,
        "/session/session_missing/list_profiles").result() == http::status::not_found);
    REQUIRE(fixture.request(
        http::verb::post,
        "/session/session_missing/set_profile",
        selection_body(print, printer_a, {}, {filament_a}),
        "application/json").result() == http::status::not_found);
}

TEST_CASE("REST profile catalog remains usable with no installed profiles", "[rest_profiles]")
{
    ensure_engine_initialized();
    RestServerFixture fixture;

    const HttpResponseMessage created = fixture.request(http::verb::post, "/create_session");
    REQUIRE(created.result() == http::status::created);
    const std::string session_id = nlohmann::json::parse(created.body()).at("session_id");
    const HttpResponseMessage listed = fixture.request(
        http::verb::get,
        "/session/" + session_id + "/list_profiles");
    REQUIRE(listed.result() == http::status::ok);
    REQUIRE(fixture.running());
}

TEST_CASE("REST profile catalog rejects invalid custom settings", "[rest_profiles]")
{
    ensure_engine_initialized();
    TempDirectory profiles_root;
    std::filesystem::create_directories(profiles_root.path() / "print");
    std::filesystem::create_directories(profiles_root.path() / "printer");
    std::filesystem::create_directories(profiles_root.path() / "filament");
    std::ofstream(profiles_root.path() / "print" / "rest-print.ini")
        << "print_custom_setting = {\n";

    const auto load_catalog = [&]() {
        Slic3r::Rest::ProfileCatalog catalog(profiles_root.path());
    };
    REQUIRE_THROWS_AS(load_catalog(), std::runtime_error);
}

TEST_CASE("Wizard profile loading fills defaults and separates preset categories", "[rest_profiles][wizard]")
{
    ensure_engine_initialized();
    TempDirectory profiles_root;
    Slic3r::PresetBundle bundle;
    Slic3r::PresetsConfigSubstitutions substitutions;
    Slic3r::PresetCollection *collections[] = {&bundle.fff_prints, &bundle.printers, &bundle.filaments};
    for (Slic3r::PresetCollection *collection : collections) {
        const std::string section = collection->section_name();
        std::filesystem::create_directories(profiles_root.path() / section);
        collection->load_presets(
            profiles_root.path().string(), section, substitutions,
            Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
    }

    std::unique_ptr<Slic3r::DynamicPrintConfig> configuration(Slic3r::DynamicPrintConfig::new_from_defaults_keys({
        "gcode_flavor", "bed_shape", "bed_custom_texture", "bed_custom_model",
        "nozzle_diameter", "filament_diameter", "temperature", "bed_temperature"
    }));
    configuration->set_deserialize_strict("nozzle_diameter", "0.6");
    configuration->set_deserialize_strict("filament_diameter", "2.85");
    configuration->set_deserialize_strict("temperature", "235");
    configuration->set_key_value("print_custom_setting", new Slic3r::ConfigOptionString(
        R"({"print":{"future":"print value"},"printer":{"future":"printer value"}})"));
    configuration->set_key_value("filament_custom_setting", new Slic3r::ConfigOptionStrings{
        R"({"future":"filament value"})"
    });
    REQUIRE_FALSE(configuration->has("layer_height"));
    REQUIRE_FALSE(configuration->has("perimeters"));

    bundle.load_config_from_wizard("wizard-profile", *configuration);

    for (const Slic3r::PresetCollection *collection : collections) {
        const Slic3r::Preset &preset = collection->get_selected_preset();
        CAPTURE(collection->section_name());
        REQUIRE(preset.name == "wizard-profile");
        CHECK(preset.config.keys() == collection->default_preset().config.keys());
        REQUIRE(std::filesystem::is_regular_file(preset.file));

        Slic3r::DynamicPrintConfig saved;
        saved.load_from_ini(preset.file, Slic3r::ForwardCompatibilitySubstitutionRule::Disable);
        CHECK(saved.has("layer_height") == (collection == &bundle.fff_prints));
        CHECK(saved.has("nozzle_diameter") == (collection == &bundle.printers));
        CHECK(saved.has("filament_diameter") == (collection == &bundle.filaments));
        if (collection == &bundle.fff_prints) {
            REQUIRE(saved.has("layer_height"));
            REQUIRE(saved.has("perimeters"));
            CHECK(saved.opt_serialize("layer_height") == collection->default_preset().config.opt_serialize("layer_height"));
            CHECK(saved.opt_serialize("perimeters") == collection->default_preset().config.opt_serialize("perimeters"));
            CHECK(nlohmann::json::parse(saved.opt_string("print_custom_setting")) ==
                nlohmann::json::parse(R"({"print":{"future":"print value"}})"));
        } else if (collection == &bundle.printers) {
            REQUIRE(saved.has("nozzle_diameter"));
            CHECK(saved.opt_serialize("nozzle_diameter") == "0.6");
            CHECK(nlohmann::json::parse(saved.opt_string("print_custom_setting")) ==
                nlohmann::json::parse(R"({"printer":{"future":"printer value"}})"));
        } else {
            REQUIRE(saved.has("filament_diameter"));
            CHECK(saved.opt_serialize("filament_diameter") == "2.85");
            CHECK(saved.opt_serialize("temperature") == "235");
            const Slic3r::ConfigOptionStrings *custom =
                saved.option<Slic3r::ConfigOptionStrings>("filament_custom_setting");
            REQUIRE(custom != nullptr);
            REQUIRE(custom->size() == 1);
            CHECK(nlohmann::json::parse(custom->get_at(0)) ==
                nlohmann::json::parse(R"({"future":"filament value"})"));
        }
    }
}

TEST_CASE("REST profile configuration preserves unknown profile settings", "[rest_profiles]")
{
    ensure_engine_initialized();
    TempDirectory profiles_root;
    std::filesystem::create_directories(profiles_root.path() / "print");
    std::filesystem::create_directories(profiles_root.path() / "printer");
    std::filesystem::create_directories(profiles_root.path() / "filament");
    std::ofstream(profiles_root.path() / "print" / "rest-print.ini")
        << "future_print_setting = print raw\n"
        << "filament_diameter = 1.75\n"
        << "print_custom_setting = {\"print\":{\"future_print_setting\":\"explicit\"}}\n";
    std::ofstream(profiles_root.path() / "printer" / "rest-printer.ini")
        << "future_printer_setting = printer raw\n"
        << "layer_height = 0.3\n";
    std::ofstream(profiles_root.path() / "filament" / "rest-filament.ini")
        << "future_filament_setting = filament raw\n";

    Slic3r::Rest::ProfileCatalog catalog(profiles_root.path());
    const nlohmann::json listed = catalog.list_profiles();
    const Slic3r::Rest::ProfileApplication application = catalog.select({
        profile_id(listed.at("print"), "rest-print"),
        profile_id(listed.at("printer"), "rest-printer"),
        {},
        {profile_id(listed.at("filaments"), "rest-filament")}
    });
    const nlohmann::json print_setting = nlohmann::json::parse(
        application.configuration.opt_string("print_custom_setting"));
    REQUIRE(print_setting.at("print").at("future_print_setting") == "explicit");
    REQUIRE(print_setting.at("print").at("filament_diameter") == "1.75");
    REQUIRE(print_setting.at("printer").at("future_printer_setting") == "printer raw");
    REQUIRE(print_setting.at("printer").at("layer_height") == "0.3");

    const Slic3r::ConfigOptionStrings *filament_settings =
        application.configuration.option<Slic3r::ConfigOptionStrings>("filament_custom_setting");
    REQUIRE(filament_settings != nullptr);
    REQUIRE(filament_settings->size() == 1);
    const nlohmann::json filament_setting = nlohmann::json::parse(filament_settings->get_at(0));
    REQUIRE(filament_setting.at("future_filament_setting") == "filament raw");

    Slic3r::DynamicPrintConfig roundtrip;
    roundtrip.load_from_ini_string(
        "print_custom_setting = " + application.configuration.opt_serialize("print_custom_setting"),
        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(roundtrip.opt_string("print_custom_setting") == application.configuration.opt_string("print_custom_setting"));
}

TEST_CASE("REST profile defaults resolve each printer filament slot", "[rest_profiles]")
{
    ensure_engine_initialized();
    TempDirectory profiles_root;
    std::filesystem::create_directories(profiles_root.path() / "print");
    std::filesystem::create_directories(profiles_root.path() / "printer");
    std::filesystem::create_directories(profiles_root.path() / "filament");
    std::ofstream(profiles_root.path() / "print" / "rest-print.ini");
    std::ofstream(profiles_root.path() / "printer" / "rest-printer.ini")
        << "nozzle_diameter = 0.4,0.4\n"
        << "default_filament_profile = ,rest-filament-b\n";
    std::ofstream(profiles_root.path() / "filament" / "rest-filament-a.ini");
    std::ofstream(profiles_root.path() / "filament" / "rest-filament-b.ini");

    Slic3r::Rest::ProfileCatalog catalog(profiles_root.path());
    const nlohmann::json listed = catalog.list_profiles();
    const Slic3r::Rest::ProfileApplication application = catalog.defaults();
    REQUIRE(application.selection.filaments == std::vector<std::string>({
        profile_id(listed.at("filaments"), "rest-filament-b"),
        profile_id(listed.at("filaments"), "rest-filament-b")
    }));
}
