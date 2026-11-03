// Checks the effective host definitions used by the filament layout without constructing wx tabs.
#include <catch2/catch.hpp>
#include <boost/filesystem.hpp>
#include "plugin_test_helpers.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ContainerUtils.hpp" // Preset.hpp requires lower_bound_by_predicate.
#include "libslic3r/Config/Preset.hpp"

// Inspect borrowed finalized definitions and the source layout path; assertions check ownership, bounds and existence without changing presets.
TEST_CASE("Simple filament options belong to the effective filament preset", "[plugins][host-simple-filament]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto &definition = Slic3r::PrintConfigDef::instance();
    for (const auto *key : {"temperature", "first_layer_temperature", "bed_temperature", "first_layer_bed_temperature"}) {
        const auto *option = definition.get(key);
        REQUIRE(option);
        CHECK(option->type == Slic3r::coInts);
        CHECK(definition.option_keys(RAW_PRESET_TYPE_FFF_FILAMENT).count(key) == 1);
        CHECK(definition.option_keys(RAW_PRESET_TYPE_FFF_PRINT).count(key) == 0);
    }
    for (const auto *key : {"temperature", "first_layer_temperature"}) {
        const auto *option = definition.get(key);
        REQUIRE(option);
        CHECK(option->min == 0);
        CHECK(option->max == 1500);
    }
    const boost::filesystem::path layout = boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() /
        "resources" / "ui_layout" / "default";
    const auto expected = layout / (Slic3r::Preset::type_name(Slic3r::Preset::TYPE_FFF_FILAMENT) + ".as");
    CHECK(boost::filesystem::is_regular_file(expected));
}
