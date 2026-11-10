///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

#include "preset_test_helpers.hpp"
#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/PresetBundle.hpp"

/*
These tests preserve unrecognized profile settings as text for saving and reloading.
The text is not used as an active slicing option. Print/printer groups stay distinct;
filament values keep their positions. Known profile metadata remains separate.
Other cases reject malformed JSON and prevent known save restrictions from being bypassed.

User/vendor profile cases
  Load profiles containing unknown or misplaced keys
  Save and reload the retained JSON without a spurious value-substitution dialog
Project cases
  Split or combine print/printer groups and per-filament values
  Reload known profile metadata through its separate INI sections
Invalid-data cases: reject bad JSON; omit keys forbidden from retained JSON

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- std::filesystem::create_directory()
        |-- root.path()
        |-- std::ofstream()
        |-- bundle.fff_prints.load_presets()
        |-- u8string()
        `-- saved.load_from_ini()
*/

using namespace Slic3r;
using Json = nlohmann::json;

TEST_CASE("Unknown options are retained as raw substitutions", "[Config][preset_preservation]")
{
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::EnableSilent);
    config.set_deserialize("future_setting", "raw value", substitutions);
    REQUIRE(substitutions.get().size() == 1);
    CHECK(substitutions.get().front().opt_def == nullptr);
    CHECK(substitutions.get().front().old_name == "future_setting");
    CHECK(substitutions.get().front().old_value == "raw value");
    CHECK_FALSE(config.has("future_setting"));
}

TEST_CASE("Unknown project values keep their origin and filament slots", "[Config][preset_preservation]")
{
    DynamicPrintConfig config;
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.6");
    config.set_key_value("print_custom_setting", new ConfigOptionString(R"({"print":{"explicit":"kept"}})"));
    ConfigSubstitutions substitutions;
    substitutions.emplace_back("filament_split", "1,2");
    substitutions.emplace_back("filament_strings", R"("a;b";"c")");
    substitutions.emplace_back("filament_scalar", "5");
    substitutions.emplace_back("machine_limit", "3");
    substitutions.emplace_back("tool_future", "4,4");
    substitutions.emplace_back("explicit", "captured");
    substitutions.emplace_back("plain_future", "x");
    preserve_unknown_project_settings(config, substitutions);
    const Json expected = {{"print", {{"explicit", "kept"}, {"filament_scalar", "5"}, {"plain_future", "x"}}},
                           {"printer", {{"machine_limit", "3"}, {"tool_future", "4,4"}}}};
    CHECK(Json::parse(config.opt_string("print_custom_setting")) == expected);
    const std::vector<std::string> &slots = config.option<ConfigOptionStrings>("filament_custom_setting")->get_values();
    REQUIRE(slots.size() == 2);
    const Json first = {{"filament_split", "1"}, {"filament_strings", R"("a;b")"}};
    const Json second = {{"filament_split", "2"}, {"filament_strings", "c"}};
    CHECK(Json::parse(slots[0]) == first);
    CHECK(Json::parse(slots[1]) == second);
}

TEST_CASE("Misplaced preset keys are retained without replacing explicit data", "[Config][preset_preservation]")
{
    PresetBundle bundle;
    DynamicPrintConfig config = bundle.fff_prints.default_preset().config;
    config.set_deserialize_strict("nozzle_diameter", "0.6");
    config.set_deserialize_strict("print_custom_setting", R"({"print":{"future":"explicit"}})");
    ConfigSubstitutions captured;
    captured.emplace_back("future", "captured");
    CHECK(Preset::remove_invalid_keys(config, bundle.fff_prints.default_preset().config, &captured) == "nozzle_diameter");
    preserve_unknown_preset_settings(config, Preset::TYPE_FFF_PRINT, captured);
    CHECK_FALSE(config.has("nozzle_diameter"));
    const Json expected = {{"print", {{"future", "explicit"}, {"nozzle_diameter", "0.6"}}}};
    CHECK(Json::parse(config.opt_string("print_custom_setting")) == expected);
    CHECK(print_custom_setting_for_origin(config, "printer").empty());
}

TEST_CASE("Retained raw data cannot bypass known persistence exclusions", "[Config][preset_preservation]")
{
    PresetBundle bundle;
    DynamicPrintConfig config = bundle.fff_prints.default_preset().config;
    config.set_deserialize_strict("printhost_apikey", "test-local-only");
    config.set_deserialize_strict("default_print_profile", "printer metadata");
    REQUIRE_FALSE(config.writes_key("printhost_apikey", ConfigWriteTarget::Project));
    REQUIRE_FALSE(config.writes_key("default_print_profile", ConfigWriteTarget::GCode));
    ConfigSubstitutions captured;
    const std::string removed = Preset::remove_invalid_keys(config, bundle.fff_prints.default_preset().config, &captured);
    CHECK(removed.find("printhost_apikey") != std::string::npos);
    CHECK(removed.find("default_print_profile") != std::string::npos);
    CHECK(captured.empty());
    preserve_unknown_preset_settings(config, Preset::TYPE_FFF_PRINT, captured);
    CHECK(config.opt_string("print_custom_setting").empty());
}

TEST_CASE("User preset loading preserves raw data without a substitution dialog", "[Config][preset_preservation]")
{
    PresetTest::TemporaryDirectory root;
    std::filesystem::create_directory(root.path() / "print");
    std::ofstream(root.path() / "print" / "Future.ini") << "future_setting = raw value\nnozzle_diameter = 0.6\n";
    for (const ForwardCompatibilitySubstitutionRule rule : {ForwardCompatibilitySubstitutionRule::Enable,
             ForwardCompatibilitySubstitutionRule::EnableSilent}) {
        PresetBundle bundle;
        PresetsConfigSubstitutions substitutions;
        bundle.fff_prints.load_presets(root.path().u8string(), "print", substitutions, rule);
        CHECK(substitutions.empty());
        REQUIRE(bundle.fff_prints.select_preset_by_name("Future", true));
        const Preset &selected = bundle.fff_prints.get_selected_preset();
        const Json data = Json::parse(selected.config.opt_string("print_custom_setting"));
        CHECK(data.at("print").at("future_setting") == "raw value");
        CHECK(data.at("print").at("nozzle_diameter") == "0.6");
        CHECK_FALSE(selected.is_dirty);
        CHECK(PresetCollection::dirty_options(&bundle.fff_prints.get_edited_preset(), &selected, true).empty());
        DynamicPrintConfig saved;
        selected.config.save((root.path() / "saved.ini").u8string(), ConfigWriteTarget::Local);
        saved.load_from_ini((root.path() / "saved.ini").u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(saved.opt_string("print_custom_setting") == selected.config.opt_string("print_custom_setting"));
    }
}

TEST_CASE("Project profile metadata remains separate from retained unknown values", "[Config][preset_preservation]")
{
    PresetBundle bundle;
    bundle.fff_prints.get_edited_preset().config.set_deserialize_strict("inherits", "print parent");
    bundle.printers.get_edited_preset().config.set_deserialize_strict("inherits", "printer parent");
    bundle.fff_prints.get_edited_preset().config.set_deserialize_strict("print_custom_setting", R"({"print":{"future":"A"}})");
    bundle.printers.get_edited_preset().config.set_deserialize_strict("print_custom_setting", R"({"printer":{"future":"P"}})");
    bundle.filaments.get_edited_preset().config.option<ConfigOptionStrings>("filament_custom_setting")->set({R"({"future":"F"})"});
    bundle.update_multi_material_filament_presets();
    const DynamicPrintConfig combined = bundle.full_config();
    const std::string sections = bundle.project_profile_sections();
    CHECK(sections.find("inherits = print parent") != std::string::npos);
    CHECK(sections.find("inherits = printer parent") != std::string::npos);
    const Json expected = {{"print", {{"future", "A"}}}, {"printer", {{"future", "P"}}}};
    CHECK(Json::parse(combined.opt_string("print_custom_setting")) == expected);
    PresetBundle loaded;
    loaded.load_config_model("project.3mf", combined, sections);
    CHECK(loaded.fff_prints.get_edited_preset().inherits() == "print parent");
    CHECK(loaded.printers.get_edited_preset().inherits() == "printer parent");
    CHECK(Json::parse(loaded.full_config().opt_string("print_custom_setting")) == expected);
    CHECK(loaded.filaments.get_edited_preset().config.opt_serialize("filament_custom_setting") ==
          bundle.filaments.get_edited_preset().config.opt_serialize("filament_custom_setting"));
}

TEST_CASE("Vendor bundles retain future data but report real substitutions", "[Config][preset_preservation]")
{
    PresetTest::TemporaryDirectory root;
    const std::filesystem::path file = root.path() / "Future.ini";
    std::ofstream(file) << R"ini([vendor]
name = Future vendor
config_version = 1.0.0

[printer_model:Test]
name = Test
variants = 0.4

[print:Future print]
alias = Future alias
renamed_from = Old print
future_key = raw
nozzle_diameter = 0.6
)ini";
    for (const ForwardCompatibilitySubstitutionRule rule : {ForwardCompatibilitySubstitutionRule::Enable,
             ForwardCompatibilitySubstitutionRule::EnableSilent}) {
        PresetBundle bundle;
        const std::pair<PresetsConfigSubstitutions, size_t> loaded = bundle.load_configbundle(
            file.u8string(), PresetBundle::LoadSystem, rule);
        CHECK(loaded.first.empty());
        REQUIRE(bundle.fff_prints.select_preset_by_name("Future print", true));
        const Preset &selected = bundle.fff_prints.get_selected_preset();
        CHECK(selected.is_system);
        CHECK_FALSE(selected.is_dirty);
        CHECK(PresetCollection::dirty_options(&bundle.fff_prints.get_edited_preset(), &selected, true).empty());
        const Json expected = {{"print", {{"future_key", "raw"}, {"nozzle_diameter", "0.6"}}}};
        CHECK(Json::parse(selected.config.opt_string("print_custom_setting")) == expected);
        CHECK(selected.alias == "Future alias");
        CHECK(selected.renamed_from == std::vector<std::string>{"Old print"});
    }
    std::ofstream(file, std::ios::app) << R"ini(
[printer:Future printer]
printer_model = Test
printer_variant = 0.4
machine_limits_usage = unrecognized_future_usage
)ini";
    PresetBundle logged, silent;
    CHECK_FALSE(logged.load_configbundle(file.u8string(), PresetBundle::LoadSystem,
        ForwardCompatibilitySubstitutionRule::Enable).first.empty());
    CHECK(silent.load_configbundle(file.u8string(), PresetBundle::LoadSystem,
        ForwardCompatibilitySubstitutionRule::EnableSilent).first.empty());
}

TEST_CASE("Malformed retained settings fail instead of silently discarding data", "[Config][preset_preservation]")
{
    PresetBundle bundle;
    for (const char *value : {"{", "[]", R"({"other":{}})", R"({"print":{"future":1}})"}) {
        DynamicPrintConfig config = bundle.fff_prints.default_preset().config;
        config.set_deserialize_strict("print_custom_setting", value);
        CHECK_THROWS_AS(validate_preset_custom_settings(config), ConfigurationError);
        CHECK_THROWS_AS(bundle.fff_prints.load_preset("", "Invalid", std::move(config)), ConfigurationError);
    }
}
