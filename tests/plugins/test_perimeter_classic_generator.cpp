#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include <algorithm>
#include <string>
#include <vector>

namespace {

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

} // namespace

// Read registered perimeter descriptions without mutation and check execution priority separately from the classic algorithm's preference.
TEST_CASE("ClassicPerimeterGenerator is registered and selectable skeleton", "[plugins][perimeter][classic-generator]")
{
    // This test protects the STEP_PERIMETER skeleton: the plugin appears as a
    // selectable perimeter generator and runs through the host perimeter tree
    // without crashing. Geometry comparisons live in the dedicated regression
    // tests.
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Plugin *plugin = Slic3r::Orchestrator::instance().get_plugin(CLASSIC_PERIMETER_GENERATOR);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_id() == CLASSIC_PERIMETER_GENERATOR);
    CHECK(plugin->get_name() == "Classic perimeter generator");
    CHECK(plugin->get_step() == STEP_PERIMETER);
    CHECK(plugin->get_priority() == 0);
    CHECK(plugin->get_exclusive_group_priority() == 5);

    const DynamicPrintConfig config = perimeter_config({{"perimeters", "1"}});
    const ExPolygon surface = rectangle_expolygon(-10., -10., 10., 10.);
    const PerimeterRunCapture generated =
        run_perimeter_case(config, {CLASSIC_PERIMETER_GENERATOR}, surface, 0);

    REQUIRE(external_perimeter_count(generated) > 0);
    REQUIRE_FALSE(generated.fill_surfaces.empty());
    REQUIRE_FALSE(generated.fill_no_overlap_surfaces.empty());

    // The classic skeleton is no longer expected to match the deliberately
    // simple child-area formula. It already follows the first pieces of the
    // classic "next onion" logic, so this test only protects the shared
    // perimeter-step contract: remaining no-overlap areas and fill/anchor areas
    // must form valid leaf domains for the later infill steps.
    require_leaf_fill_area_consistency(generated);
}
TEST_CASE("Classic perimeter generator owns its settings and activation rules",
          "[plugins][perimeter][classic][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.generator.classic");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 15);
    for (const char *expected : { "gap_fill_enabled", "thin_perimeters", "thin_walls", "thin_walls_merge" })
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), expected) != defined_keys.end());

    const ConfigOptionDef *def = PrintConfigDef::instance().get("thin_walls");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_SLICING);
    def = PrintConfigDef::instance().get("thin_walls_merge");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Object);
    CHECK(PrintConfig::defaults().option("thin_walls") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("thin_walls") != nullptr);

    // Selecting another generator removes the generated implementation reason,
    // so the plugin's own activation rules alone decide the result here.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled = [&orchestrator, &config](const char *target) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1))
            if (application.target_key == target)
                return application.enabled;
        return false;
    };

    config.set_deserialize("step_perimeter_plugin", "perimeter.generator.simple");
    config.option("perimeter_generator")->set_int(int(PerimeterGeneratorType::Classic));
    config.option<ConfigOptionInt>("perimeters")->value = 0;
    CHECK_FALSE(enabled("thin_walls"));
    config.option<ConfigOptionInt>("perimeters")->value = 2;
    CHECK(enabled("thin_walls"));
    CHECK(enabled("gap_fill_enabled"));

    config.option("perimeter_generator")->set_int(int(PerimeterGeneratorType::Arachne));
    CHECK_FALSE(enabled("thin_walls"));
    CHECK_FALSE(enabled("gap_fill_enabled"));
}
