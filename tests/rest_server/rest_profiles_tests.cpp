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
#include "rest_server_fixture.hpp"

#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace RestServerTest;

namespace {

void ensure_engine_initialized()
{
    Slic3r::Rest::Engine &engine = Slic3r::Rest::Engine::instance();
    if (engine.initialized())
        return;

    static TempDirectory engine_root;
    Slic3r::Rest::EnginePaths paths;
    paths.resource_directory = SLIC3R_REST_RESOURCE_DIRECTORY;
    paths.data_directory = engine_root.path() / "engine_data";
    engine.initialize(paths);
}

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
            REQUIRE(profile.size() == 2);
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
