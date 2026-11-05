///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <fstream>
#include <string>

#include "preset_test_helpers.hpp"
#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/PresetBundle.hpp"

/*
These tests load vendor profiles from an explicit directory without changing data_dir().
A vendor bundle is an INI file containing that manufacturer's profiles.
Small files check inherited settings, filename order, duplicate names and bundle errors.
Unknown text is preserved without reporting a replacement of a known option value.

Operation notes:
- write_vendor: create a parent profile and a child containing an unknown key
- load_system_presets_from: load the files with the GUI's profile rules
- Check the resulting settings, profile state and diagnostics

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- write_vendor()
        |-- root.path()
        |-- bundle.load_system_presets_from()
        |-- u8string()
        |-- bundle.fff_prints.select_preset_by_name()
        `-- PresetCollection::dirty_options()
*/

using namespace Slic3r;

namespace {

// Write a vendor bundle with a parent used only for inheritance and a child containing an unknown key.
// The caller chooses layer_height to distinguish duplicate definitions.
void write_vendor(const std::filesystem::path &file, const char *height);

void write_vendor(const std::filesystem::path &file, const char *height)
{
    std::ofstream(file) << "[vendor]\nname = Test vendor\nconfig_version = 1.0.0\n\n"
        "[print:*base*]\nlayer_height = " << height << "\n\n"
        "[print:Test print @TEST]\ninherits = *base*\ntest_future_key = raw\n";
}

} // namespace

TEST_CASE("GUI system bundle loading preserves future data without dirty presets", "[preset_vendors][preset_gui_path]")
{
    PresetTest::TemporaryDirectory root;
    write_vendor(root.path() / "Test.ini", "0.2");
    for (const ForwardCompatibilitySubstitutionRule rule : {ForwardCompatibilitySubstitutionRule::EnableSystemSilent,
             ForwardCompatibilitySubstitutionRule::EnableSilent, ForwardCompatibilitySubstitutionRule::Enable}) {
        PresetBundle bundle;
        const std::pair<PresetsConfigSubstitutions, std::string> loaded = bundle.load_system_presets_from(root.path().u8string(), rule);
        REQUIRE(loaded.second.empty());
        CHECK(loaded.first.empty());
        REQUIRE(bundle.fff_prints.select_preset_by_name("Test print @TEST", true));
        const Preset &selected = bundle.fff_prints.get_selected_preset();
        CHECK(selected.is_system);
        CHECK_FALSE(selected.is_dirty);
        CHECK_FALSE(bundle.fff_prints.get_edited_preset().is_dirty);
        CHECK(PresetCollection::dirty_options(&bundle.fff_prints.get_edited_preset(), &selected, true).empty());
        CHECK(selected.config.opt_float("layer_height") == Approx(0.2));
        CHECK(selected.config.opt_string("print_custom_setting").find("test_future_key") != std::string::npos);
    }
}

TEST_CASE("Vendor directory order determines which duplicate definition wins", "[preset_vendors]")
{
    PresetTest::TemporaryDirectory root;
    write_vendor(root.path() / "Z-last.ini", "0.3");
    write_vendor(root.path() / "A-first.ini", "0.2");
    PresetBundle bundle;
    const std::pair<PresetsConfigSubstitutions, std::string> loaded = bundle.load_system_presets_from(
        root.path().u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(loaded.second.find("Test print @TEST") != std::string::npos);
    REQUIRE(bundle.fff_prints.select_preset_by_name("Test print @TEST", true));
    CHECK(bundle.fff_prints.get_selected_preset().config.opt_float("layer_height") == Approx(0.2));
    REQUIRE(bundle.fff_prints.get_selected_preset().vendor != nullptr);
    CHECK(bundle.fff_prints.get_selected_preset().vendor->id == "A-first");
}

TEST_CASE("Vendor directory loads reset stale presets and identify failed files", "[preset_vendors]")
{
    PresetTest::TemporaryDirectory root, empty;
    write_vendor(root.path() / "Test.ini", "0.2");
    std::ofstream(root.path() / "Broken.ini") << "[print:Invalid]\nlayer_height = 0.2\n";
    PresetBundle bundle;
    const std::pair<PresetsConfigSubstitutions, std::string> loaded = bundle.load_system_presets_from(
        root.path().u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(loaded.second.find("Broken.ini") != std::string::npos);
    CHECK(bundle.fff_prints.find_preset("Test print @TEST", false, false) != nullptr);
    CHECK(bundle.load_system_presets_from(empty.path().u8string(), ForwardCompatibilitySubstitutionRule::EnableSilent).second.empty());
    CHECK(bundle.fff_prints.has_defaults_only());
    CHECK(bundle.vendors.empty());
}
