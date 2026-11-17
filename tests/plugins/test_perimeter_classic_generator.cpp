#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "plugin_e2e_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include <algorithm>
#include <string>
#include <vector>

/*
These tests check the classic provider's registration, per-shell role/flow and
the default preference Classic, Arachne, then Simple. Focused fixtures include contours and
holes with distinct external/internal widths. The export matrix selects classic
and both simple providers explicitly, using only small STL fixtures.

    TEST_CASE
    |-- run_perimeter_case(): capture the generated shell tree
    |-- require_loop_role_and_flow_per_shell(): compare roles and physical flow
    `-- linear_cases() / export_gcode_cli(): verify each provider's declared inputs

Activation changes are scoped to each export scenario and source captures own
their data. Persisted provider choices and the absent-selector fallback are
checked independently from the generated geometry.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

class ScopedPerimeterExportPlugins {
public:
    // Retain the working default pipeline and activate one explicitly selected provider.
    explicit ScopedPerimeterExportPlugins(const char *provider) : host(Orchestrator::instance()) {
        for (Plugin *plugin : host.active_plugins())
            previous.push_back(plugin);
        REQUIRE(host.set_plugin_active(provider, true));
        REQUIRE(host.set_plugin_active("ordering.extrusion_tree.advanced", true));
    }
    // Restore the full activation set even when an export assertion fails.
    ~ScopedPerimeterExportPlugins() {
        host.clear_active_plugins();
        for (Plugin *plugin : previous)
            host.set_plugin_active(plugin, true);
    }
private:
    Orchestrator &host;
    std::vector<Plugin *> previous;
};

} // namespace

TEST_CASE("Perimeter generators pass their linear STL export catalogues",
          "[plugins][plugin-e2e][perimeter-e2e][perimeter]")
{
    namespace E2E = Test::PluginE2E;
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    std::vector<const char *> providers{CLASSIC_PERIMETER_GENERATOR, SIMPLE_PERIMETER_GENERATOR, ARACHNE_PERIMETER_GENERATOR};
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
    providers.push_back(PYTHON_SIMPLE_PERIMETER_GENERATOR);
#endif
    const TriangleMesh mesh = E2E::stl_fixture(true);
    for (const char *provider : providers) {
        INFO(provider);
        ScopedPerimeterExportPlugins active(provider);
        DynamicPrintConfig baseline = DynamicPrintConfig::full_print_config();
        baseline.set_deserialize_strict({{"layer_height", "0.2"}, {"first_layer_height", "0.2"},
            {"step_perimeter_plugin", provider}, {"step_gcode_plugin", "gcode.printing_plan_file_writer"},
            {"gcode_firmware_plugin", "gcode.firmware.marlin2"}});
        const std::string ordering_key = Steps::plugin_exclusive_group_option_key(host, STEP_ORDERING, "ordering.extrusion_tree");
        REQUIRE_FALSE(ordering_key.empty());
        baseline.set_deserialize_strict(ordering_key, "ordering.extrusion_tree.advanced");
        std::vector<E2E::Domain> domains;
        const auto add = [&domains](const std::string &key, const std::vector<std::string> &values) {
            E2E::Domain domain{{key}, {}};
            for (const std::string &value : values)
                domain.cases.push_back({key + "=" + value, {{key, value}}});
            domains.push_back(std::move(domain));
        };
        add("perimeters", {"1", "3"});
        add("external_perimeter_extrusion_width", {"0.4", "0.6"});
        add("perimeter_extrusion_width", {"0.4", "0.6"});
        domains.push_back({{"perimeter_extruder", "nozzle_diameter", "filament_diameter", "extrusion_multiplier",
                            "min_layer_height", "max_layer_height"}, {
            {"second perimeter tool", {{"perimeter_extruder", "2"}, {"nozzle_diameter", "0.4,0.4"},
                {"filament_diameter", "1.75,1.75"}, {"extrusion_multiplier", "1,1"},
                {"min_layer_height", "5%,5%"}, {"max_layer_height", "75%,75%"}}}}});
        if (std::string(provider) == ARACHNE_PERIMETER_GENERATOR)
            add("perimeter_reverse", {"0", "1"});
        if (std::string(provider) == CLASSIC_PERIMETER_GENERATOR) {
            for (const char *key : {"overhangs", "perimeter_round_corners", "thin_walls", "thin_walls_merge",
                                    "gap_fill_enabled", "gap_fill_last"})
                add(key, {"0", "1"});
            for (const char *key : {"overhangs_flow_ratio", "thin_perimeters", "thin_perimeters_all", "gap_fill_overlap"})
                add(key, {"50%", "100%"});
            for (const char *key : {"overhangs_width", "overhangs_extrusion_spacing", "thin_walls_min_width",
                                    "thin_walls_overlap", "gap_fill_min_width", "gap_fill_max_width",
                                    "gap_fill_min_length", "gap_fill_min_area", "gap_fill_extension"})
                add(key, {"0", "50%"});
        }
        for (const E2E::Case &test : E2E::linear_cases(host, {provider}, baseline, domains)) {
            INFO(test.name);
            Print print;
            Model model;
            Test::init_print({mesh}, print, model, E2E::apply_case(baseline, test));
            print.process();
            const boost::filesystem::path path = Test::GCode::temporary_gcode_path();
            std::string code;
            try {
                const std::string exported = host.export_gcode_cli(print, path.string());
                code = Test::GCode::read_text_file(exported);
                Test::GCode::remove_output_pair(path);
            } catch (...) {
                Test::GCode::remove_output_pair(path);
                throw;
            }
            CHECK(code.find(";TYPE:External perimeter") != std::string::npos);
            CHECK(code.find("G1 ") != std::string::npos);
            CHECK(code.find(" E") != std::string::npos);
        }
    }
}

TEST_CASE("Classic perimeter loops use the role and flow of their shell",
          "[plugins][perimeter][classic-generator]")
{
    // Only shell 0 is the external perimeter. The deeper shells are internal
    // perimeters and must carry the internal role and flow, otherwise they are
    // printed at the external perimeter speed and width. The test runs on layer
    // 1 with explicitly different widths: on the first layer
    // first_layer_extrusion_width gives every role the same flow.
    const DynamicPrintConfig config = distinct_perimeter_width_config(3);
    for (const ExPolygon &surface : {rectangle_expolygon(-10., -10., 10., 10.), rectangle_with_hole_expolygon()}) {
        const PerimeterRunCapture generated =
            run_perimeter_case(config, {CLASSIC_PERIMETER_GENERATOR}, surface, 1);
        require_loop_role_and_flow_per_shell(generated);
    }
}

TEST_CASE("Perimeter defaults prefer Classic then Arachne then Simple", "[plugins][perimeter][classic-generator]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const std::vector<Plugin *> active = orchestrator.get_active_plugins_for_step(STEP_PERIMETER);
    for (const char *id : {SIMPLE_PERIMETER_GENERATOR, CLASSIC_PERIMETER_GENERATOR, ARACHNE_PERIMETER_GENERATOR}) {
        Plugin *plugin = orchestrator.get_plugin(id);
        REQUIRE(plugin != nullptr);
        REQUIRE(std::find(active.begin(), active.end(), plugin) != active.end());
    }

    // A profile without a stored generator resolves the option default.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto resolved = [&orchestrator, &config]() {
        const std::vector<Plugin *> plugins =
            Steps::selected_or_active_plugins_for_step(orchestrator, STEP_PERIMETER, &config);
        REQUIRE(plugins.size() == 1);
        return plugins.front()->get_id();
    };
    const ConfigOptionDef *def = PrintConfigDef::instance().get("step_perimeter_plugin");
    REQUIRE(def != nullptr);
    CHECK(def->default_value->serialize() == CLASSIC_PERIMETER_GENERATOR);
    CHECK(resolved() == CLASSIC_PERIMETER_GENERATOR);

    // Profiles that choose another generator keep it.
    config.set_deserialize("step_perimeter_plugin", SIMPLE_PERIMETER_GENERATOR);
    CHECK(resolved() == SIMPLE_PERIMETER_GENERATOR);
    config.set_deserialize("step_perimeter_plugin", ARACHNE_PERIMETER_GENERATOR);
    CHECK(resolved() == ARACHNE_PERIMETER_GENERATOR);
    config.set_deserialize("step_perimeter_plugin", CLASSIC_PERIMETER_GENERATOR);
    CHECK(resolved() == CLASSIC_PERIMETER_GENERATOR);
    config.erase("step_perimeter_plugin");
    CHECK(resolved() == CLASSIC_PERIMETER_GENERATOR);

    // Available alternatives follow their rank, not the vector's input order.
    Plugin *classic = orchestrator.get_plugin(CLASSIC_PERIMETER_GENERATOR);
    Plugin *arachne = orchestrator.get_plugin(ARACHNE_PERIMETER_GENERATOR);
    Plugin *simple = orchestrator.get_plugin(SIMPLE_PERIMETER_GENERATOR);
    CHECK(Steps::default_provider_from_group({simple, arachne, classic}) == classic);
    CHECK(Steps::default_provider_from_group({simple, arachne}) == arachne);
    CHECK(Steps::default_provider_from_group({simple}) == simple);
}

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
    CHECK(plugin->get_exclusive_group_priority() == 0);

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
