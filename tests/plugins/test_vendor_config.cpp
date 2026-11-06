#include <catch2/catch.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/make_shared.hpp>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ContainerUtils.hpp"
#include "libslic3r/Config/PresetBundle.hpp"
#include "libslic3r/Config/AppConfig.hpp"
#include "libslic3r/Utils.hpp"

/*
Verify bundle default requests and historical name/value conversions for config loading.
Runtime initialization supplies registered options before independent source maps are converted.
Bundle tests create isolated files, load presets, then check collection values and diagnostics separately
from source-file preservation. Bundle sources and isolated data directories live under
build/temp/vendor_config and are removed after each test, restoring the previous data directory.
The test-owned collection defaults may change; schema definitions and plugin selections do not.

Test paths and their responsibilities:
  Ordinary conversion TEST_CASE - check destinations and serialized values for 21 independent source maps.
    ensure_plugin_test_runtime_initialized - initialize the shared option/plugin runtime.
    PrintConfigDef::handle_legacy_map - convert each map without schema filtering.
  Bundle TEST_CASEs - compare loaded values, substitutions and optional captured warnings.
    VendorBundleFiles - own source files and isolate the application's data directory.
      load - write a source and call PresetBundle::load_configbundle without saving presets;
        compare file names and bytes to ensure loading leaves sources and directories unchanged.
      load_system - prepare saved system profiles through normal vendor setup.
    VendorWarnings - capture warnings and restore the previous logging level on destruction.
    vendor_config - locate saved collection values rather than an edited preset copy.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- bundle.fff_prints.default_preset()
        |-- width->set_phony()
        |-- PrintConfigDef::instance()
        |-- bundle.printers.default_preset()
        |-- retract->set()
        `-- retract->set_can_be_disabled()
*/

using namespace Slic3r;

namespace {
class VendorBundleFiles {
public:
    // Initialize the plugin runtime and isolate the data directory in an owned project-local directory.
    VendorBundleFiles();
    // Restore the previous data directory and remove only this instance's files.
    ~VendorBundleFiles();
    // Write one source, load without saving, and verify unchanged bytes and absence of generated files.
    std::pair<PresetsConfigSubstitutions, size_t> load(PresetBundle &bundle, const std::string &text,
        ForwardCompatibilitySubstitutionRule rule = Enable, bool system = false, bool from_prusa = false,
        const std::string &expected_filament_selection = std::string());
    // Load the test vendor with its SLA model enabled through normal setup, populating renamed-system maps without access changes.
    void load_system(PresetBundle &bundle, const std::string &text);
private:
    std::string previous;
    boost::filesystem::path directory;
    size_t sequence = 0;
};

// Find a saved preset, not an edited copy, and fail at an assertion before dereferencing a missing preset.
const DynamicPrintConfig &vendor_config(const PresetCollection &collection, const std::string &name);
// Snapshot actual file names and bytes below the isolated data directory to detect unexpected saves or modifications.
std::map<std::string, std::string> vendor_files_snapshot(const boost::filesystem::path &directory);

class VendorWarnings {
public:
    // Capture warnings while preserving the application's previous log level.
    VendorWarnings();
    // Remove this sink and restore the previous logging level, including after failed assertions.
    ~VendorWarnings();
    // Flush and return captured log messages.
    std::string text();
private:
    using Sink = boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend>;
    boost::shared_ptr<std::ostringstream> messages;
    boost::shared_ptr<Sink> sink;
    unsigned level;
};
}

// One source at a time exposes the exact destination and converted text, without schema filtering.
TEST_CASE("Fixed legacy branches preserve ordinary serialized results", "[plugins][config][vendor-config-names]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const struct LegacyCase {
        const char *source;
        const char *input;
        const char *destination;
        const char *output;
    } cases[] = {
        {"octoprint_host", "printer.local", "print_host", "printer.local"},
        {"octoprint_cafile", "cert.pem", "printhost_cafile", "cert.pem"},
        {"octoprint_apikey", "test-token", "printhost_apikey", "test-token"},
        {"elefant_foot_compensation", "0.25", "first_layer_size_compensation", "-0.25"},
        {"elefant_foot_compensation", "0", "first_layer_size_compensation", "0"},
        {"elefant_foot_compensation", "-0.25", "first_layer_size_compensation", "-0.25"},
        {"elefant_foot_min_width", "0.3", "elephant_foot_min_width", "0.3"},
        {"z_steps_per_mm", "4", "z_step", "0.25"},
        {"z_steps_per_mm", "0", "z_step", "0"},
        {"z_steps_per_mm", "-4", "z_step", "-4"},
        {"infill_not_connected", "1", "infill_connection", "notconnected"},
        {"infill_not_connected", "0", "infill_connection", "connected"},
        {"preset_name", "sentinel", "preset_names", "sentinel"},
        {"print_machine_envelope", "1", "machine_limits_usage", "emit_to_gcode"},
        {"print_machine_envelope", "0", "machine_limits_usage", "time_estimate_only"},
        {"retract_lift_not_last_layer", "1", "retract_lift_top", "Not on top"},
        {"retract_lift_not_last_layer", "0", "retract_lift_top", "All surfaces"},
        {"support_material_solid_first_layer", "1", "raft_first_layer_density", "100"},
        {"support_material_solid_first_layer", "0", "raft_first_layer_density", "100"},
        {"external_perimeters_vase", "1", "seam_slope_type", "all"},
        {"external_perimeters_vase", "0", "seam_slope_type", "none"},
    };
    for (const LegacyCase &entry : cases) {
        CAPTURE(entry.source, entry.input);
        std::unordered_map<t_config_option_key, std::pair<t_config_option_key, std::string>> values;
        values[entry.source] = {entry.source, entry.input};
        PrintConfigDef::handle_legacy_map(values, false);
        REQUIRE(values.size() == 1);
        REQUIRE(values.count(entry.source) == 1);
        CHECK(values.at(entry.source).first == entry.destination);
        CHECK(values.at(entry.source).second == entry.output);
    }
}

// Each pair collides by one identity before migration, preserving only its final text or whole option.
TEST_CASE("Bundle default requests follow local identity order", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    const auto result = files.load(bundle,
        "[print:reset]\nlayer_height=0.73\nperimeters_extruder=4\nseam_slope_type=all\n"
        "default_settings=layer_height;perimeter_extruder;seam_slope_type;test_config_nature_normal;seam_visibility\n"
        "[print:text]\ndefault_settings=layer_height;perimeters_extruder;seam_slope_type\n"
        "layer_height=0.61\nperimeter_extruder=3\nseam_slope_type=all\n"
        "[printer:canonical]\nz_step=0.25\ndefault_settings=z_step\n"
        "[printer:canonical_text]\ndefault_settings=z_step\nz_step=0.25\n");
    CHECK(result.second == 4);
    CHECK(result.first.empty());
    const DynamicPrintConfig &reset = vendor_config(bundle.fff_prints, "reset");
    const DynamicPrintConfig &text = vendor_config(bundle.fff_prints, "text");
    CHECK(reset.opt_float("layer_height") == bundle.fff_prints.default_preset().config.opt_float("layer_height"));
    CHECK(reset.opt_int("perimeter_extruder") == 1);
    CHECK(reset.option("seam_slope_type")->serialize() == bundle.fff_prints.default_preset().config.option("seam_slope_type")->serialize());
    CHECK(reset.opt_int("test_config_nature_normal") == 7);
    CHECK_FALSE(reset.opt_bool("seam_visibility"));
    CHECK(text.opt_float("layer_height") == Approx(0.61));
    CHECK(text.opt_int("perimeter_extruder") == 3);
    CHECK(text.option("seam_slope_type")->serialize() == "all");
    CHECK(vendor_config(bundle.printers, "canonical").opt_float("z_step") == bundle.printers.default_preset().config.opt_float("z_step"));
    CHECK(vendor_config(bundle.printers, "canonical_text").opt_float("z_step") == Approx(0.25));
    CHECK_FALSE(reset.has("default_settings"));
    CHECK(reset.opt_string("print_custom_variables").find("default_settings") == std::string::npos);
}

// Unregistered historical names stay unknown requests, while ordinary historical values still convert.
TEST_CASE("Bundle requests do not invent aliases for historical conversions", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    const auto result = files.load(bundle,
        "[printer:unknown_request]\nz_step=0.25\ndefault_settings=z_steps_per_mm\n");
    CHECK(result.second == 1);
    CHECK(vendor_config(bundle.printers, "unknown_request").opt_float("z_step") == Approx(0.25));
    REQUIRE(result.first.size() == 1);
    REQUIRE(result.first.front().substitutions.size() == 1);
    const ConfigSubstitution &record = result.first.front().substitutions.front();
    CHECK(record.old_name == "z_steps_per_mm");
    CHECK(record.opt_def == nullptr);
    CHECK(record.new_value.get() == nullptr);
    const auto ordinary = files.load(bundle,
        "[printer:ordinary_legacy]\nz_steps_per_mm=4\ndefault_settings=retract_length\n");
    CHECK(ordinary.second == 1);
    CHECK(ordinary.first.empty());
    CHECK(vendor_config(bundle.printers, "ordinary_legacy").opt_float("z_step") == Approx(0.25));
}

// Private and partial parents contribute only their written keys, and descendants inherit values rather than requests.
TEST_CASE("Bundle defaults retain partial inheritance and last parent precedence", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    const double baseline = bundle.fff_prints.default_preset().config.opt_float("layer_height");
    const auto result = files.load(bundle,
        "[print:*first]\nlayer_height=0.49\nperimeters=5\n"
        "[print:*reset]\ndefault_settings=layer_height\n"
        "[print:*nested]\ninherits=*first;*reset\n"
        "[print:child]\ninherits=*nested\ndefault_settings=\n"
        "[print:last_text]\ninherits=*reset;*first\n"
        "[print:local]\ninherits=*nested\nlayer_height=0.62\n"
        "[print:old]\ninherits=*first\nperimeters=6\n"
        "[print:local_reset]\ninherits=*first\ndefault_settings=perimeters\n");
    CHECK(result.second == 5);
    CHECK(result.first.empty());
    CHECK(vendor_config(bundle.fff_prints, "child").opt_float("layer_height") == baseline);
    CHECK(vendor_config(bundle.fff_prints, "child").opt_int("perimeters") == 5);
    CHECK(vendor_config(bundle.fff_prints, "last_text").opt_float("layer_height") == Approx(0.49));
    CHECK(vendor_config(bundle.fff_prints, "local").opt_float("layer_height") == Approx(0.62));
    CHECK(vendor_config(bundle.fff_prints, "old").opt_int("perimeters") == 6);
    CHECK(vendor_config(bundle.fff_prints, "local_reset").opt_int("perimeters") == bundle.fff_prints.default_preset().config.opt_int("perimeters"));
    CHECK(bundle.fff_prints.find_preset("*nested", false, false) == nullptr);
    CHECK(vendor_config(bundle.fff_prints, "child").opt_string("inherits").empty());
}

// Exact states include values hidden by phony serialization and disabled vector entries, without shared ownership.
TEST_CASE("Bundle copies preserve whole collection options independently", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    PresetBundle reference;
    DynamicPrintConfig &defaults = bundle.fff_prints.default_preset().config;
    ConfigOptionFloatOrPercent *width = defaults.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width");
    REQUIRE(width != nullptr);
    width->value = 137;
    width->percent = true;
    width->set_phony(true);
    defaults.option<ConfigOptionString>("print_custom_variables")->value = "literal=${default}";
    const ConfigOption *definition = PrintConfigDef::instance().get("perimeter_extrusion_width")->default_value.get();
    REQUIRE(PrintConfigDef::instance().get("perimeter_extrusion_width")->can_phony);
    REQUIRE(static_cast<const ConfigOptionFloatOrPercent *>(definition)->value != 137);
    DynamicPrintConfig &printer_defaults = bundle.printers.default_preset().config;
    ConfigOptionFloats *retract = printer_defaults.option<ConfigOptionFloats>("retract_length");
    REQUIRE(retract != nullptr);
    retract->set({2.75});
    retract->set_can_be_disabled();
    retract->set_enabled(false, 0);
    const auto result = files.load(bundle,
        "[print:parent]\ndefault_settings=layer_height;fill_density;perimeter_extrusion_width;print_custom_variables;seam_position\n"
        "[print:one]\ninherits=parent\n[print:two]\ninherits=parent\n"
        "[printer:vector]\nnozzle_diameter=0.4,0.6\ndefault_settings=retract_length\n");
    CHECK(result.first.empty());
    Preset *first = bundle.fff_prints.find_preset("one", false, false);
    REQUIRE(first != nullptr);
    DynamicPrintConfig &one = first->config;
    const DynamicPrintConfig &two = vendor_config(bundle.fff_prints, "two");
    const auto *copied = one.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width");
    REQUIRE(copied != nullptr);
    CHECK(copied->value == 137);
    CHECK(copied->percent);
    CHECK(copied->is_phony());
    CHECK(copied->flags == width->flags);
    CHECK(one.opt_float("layer_height") == defaults.opt_float("layer_height"));
    CHECK(one.option<ConfigOptionPercent>("fill_density")->value == defaults.option<ConfigOptionPercent>("fill_density")->value);
    CHECK(one.option("seam_position")->get_int() == defaults.option("seam_position")->get_int());
    CHECK(one.opt_string("print_custom_variables") == "literal=${default}");
    const auto *vector = vendor_config(bundle.printers, "vector").option<ConfigOptionFloats>("retract_length");
    REQUIRE(vector != nullptr);
    REQUIRE(vector->size() == 2);
    CHECK(vector->get_at(0) == 2.75);
    CHECK(vector->get_at(1) == 2.75);
    CHECK_FALSE(vector->is_enabled(0));
    CHECK(vector->flags == retract->flags);
    // New elements repeat the first value but take the option-wide activation, not the first element's activation.
    CHECK(vector->is_enabled(1) == retract->ConfigOption::is_enabled());
    one.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width")->value = 99;
    CHECK(two.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width")->value == 137);
    CHECK(vendor_config(bundle.fff_prints, "parent").option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width")->value == 137);
    CHECK(width->value == 137);
    CHECK(reference.fff_prints.default_preset().config.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width")->value != 137);
    CHECK(static_cast<const ConfigOptionFloatOrPercent *>(definition)->value != 137);
}

// Only syntactically valid lists expand; unknown members do not cancel valid members of the same list.
TEST_CASE("Bundle request grammar and admission stay local", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    const auto result = files.load(bundle,
        "[print:trim]\nlayer_height=0.77\ndefault_settings= layer_height ; layer_height \n"
        "[print:empty]\nlayer_height=0.71\ndefault_settings=   \n"
        "[print:malformed]\nlayer_height=0.72\ndefault_settings=layer_height;;perimeters\n"
        "[print:reserved]\nlayer_height=0.73\ndefault_settings=all;layer_height\n"
        "[print:all]\nlayer_height=0.74\ndefault_settings=all\n"
        "[print:mixed]\nlayer_height=0.75\ndefault_settings=layer_height;unknown_key;seam_travel;brim_type;print_version;inherits;alias;renamed_from;default_settings;nozzle_diameter;printer_technology;print_settings_id;print_custom_variables\n"
        "[printer:missing]\ndefault_settings=elefant_foot_min_width;preset_name;octoprint_host;octoprint_cafile;octoprint_apikey\n");
    CHECK(result.second == 7);
    CHECK(vendor_config(bundle.fff_prints, "trim").opt_float("layer_height") == bundle.fff_prints.default_preset().config.opt_float("layer_height"));
    CHECK(vendor_config(bundle.fff_prints, "empty").opt_float("layer_height") == Approx(0.71));
    CHECK(vendor_config(bundle.fff_prints, "malformed").opt_float("layer_height") == Approx(0.72));
    CHECK(vendor_config(bundle.fff_prints, "reserved").opt_float("layer_height") == Approx(0.73));
    CHECK(vendor_config(bundle.fff_prints, "all").opt_float("layer_height") == Approx(0.74));
    CHECK(vendor_config(bundle.fff_prints, "mixed").opt_float("layer_height") == bundle.fff_prints.default_preset().config.opt_float("layer_height"));
    size_t diagnostics = 0;
    bool nonapplicable_technology = false;
    for (const PresetConfigSubstitutions &group : result.first)
        for (const ConfigSubstitution &record : group.substitutions) {
            ++diagnostics;
            CHECK(record.opt_def == nullptr);
            CHECK(record.new_value.get() == nullptr);
            if (record.old_name == "printer_technology")
                nonapplicable_technology = true;
            CHECK(record.old_name != "print_settings_id");
            CHECK(record.old_name != "print_custom_variables");
        }
    CHECK(diagnostics == 18);
    CHECK(nonapplicable_technology);
    CHECK_FALSE(vendor_config(bundle.fff_prints, "mixed").has("printer_technology"));
    CHECK(vendor_config(bundle.fff_prints, "mixed").opt_string("print_custom_variables").find("printer_technology") == std::string::npos);
    const auto silent = files.load(bundle, "[print:silent_admission]\ndefault_settings=printer_technology\n", EnableSilent);
    CHECK(silent.first.empty());
    CHECK_FALSE(vendor_config(bundle.fff_prints, "silent_admission").has("printer_technology"));
    CHECK_THROWS(files.load(bundle, "[print:duplicate]\nlayer_height=0.2\nlayer_height=0.3\n"));
}

// Discarding a source before conversion prevents only that source's derived outputs, not separately written settings.
TEST_CASE("Bundle requests discard replaced migration sources", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    const auto result = files.load(bundle,
        "[print:discard]\nseam_position=nearest\ndefault_settings=seam_position\n"
        "[print:explicit]\nseam_position=nearest\nseam_angle_cost=73\ndefault_settings=seam_position\n"
        "[print:survive]\ndefault_settings=seam_position\nseam_position=nearest\n"
        "[print:ordinary]\nseam_position=nearest\nprint_custom_variables=marker=${default}\n");
    CHECK(result.first.empty());
    const DynamicPrintConfig &defaults = bundle.fff_prints.default_preset().config;
    CHECK(vendor_config(bundle.fff_prints, "discard").option("seam_angle_cost")->serialize() == defaults.option("seam_angle_cost")->serialize());
    CHECK(vendor_config(bundle.fff_prints, "explicit").opt_float("seam_angle_cost") == 73);
    CHECK(vendor_config(bundle.fff_prints, "survive").option("seam_angle_cost")->serialize() == vendor_config(bundle.fff_prints, "ordinary").option("seam_angle_cost")->serialize());
    CHECK(vendor_config(bundle.fff_prints, "survive").option("seam_position")->serialize() == vendor_config(bundle.fff_prints, "ordinary").option("seam_position")->serialize());
    CHECK(vendor_config(bundle.fff_prints, "ordinary").opt_string("print_custom_variables") == "marker=${default}");
}

// Invalid local text never reveals a valid parent and restores the collection's full state after parser flag changes.
TEST_CASE("Active bundle invalid values use effective defaults under every policy", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    for (ForwardCompatibilitySubstitutionRule rule : {Enable, EnableSilent, Disable, EnableSystemSilent, EnableSilentDisableSystem}) {
        CAPTURE(rule);
        PresetBundle bundle;
        auto *effective = bundle.fff_prints.default_preset().config.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width");
        REQUIRE(effective != nullptr);
        effective->value = 143;
        effective->percent = true;
        effective->set_phony(true);
        REQUIRE(static_cast<const ConfigOptionFloatOrPercent *>(PrintConfigDef::instance().get("perimeter_extrusion_width")->default_value.get())->value != 143);
        const auto result = files.load(bundle,
            "[print:*parent]\nperimeter_extrusion_width=0.82\n[print:invalid]\ninherits=*parent\n"
            "default_settings=layer_height\nperimeter_extrusion_width=not-a-width\n", rule);
        const auto *actual = vendor_config(bundle.fff_prints, "invalid").option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width");
        REQUIRE(actual != nullptr);
        CHECK(actual->value == 143);
        CHECK(actual->percent);
        CHECK(actual->is_phony());
        CHECK(actual->flags == effective->flags);
        if (rule == EnableSilent || rule == EnableSilentDisableSystem)
            CHECK(result.first.empty());
        else {
            REQUIRE(result.first.size() == 1);
            REQUIRE(result.first.front().substitutions.size() == 1);
            const ConfigSubstitution &substitution = result.first.front().substitutions.front();
            CHECK(substitution.old_value == "not-a-width");
            const auto *replacement = dynamic_cast<const ConfigOptionFloatOrPercent *>(substitution.new_value.get());
            REQUIRE(replacement != nullptr);
            CHECK(replacement->value == 143);
            CHECK(replacement->percent);
            CHECK(replacement->is_phony());
            CHECK(replacement->flags == effective->flags);
            CHECK(replacement != actual);
            CHECK(replacement != effective);
        }
    }
    PresetBundle strict;
    CHECK_THROWS(files.load(strict, "[print:old]\nperimeter_extrusion_width=not-a-width\n", Disable));
    CHECK_THROWS(files.load(strict, "[print:empty]\ndefault_settings=\nperimeter_extrusion_width=not-a-width\n", Disable));
    CHECK_THROWS(files.load(strict, "[print:unknown]\ndefault_settings=unresolved\nperimeter_extrusion_width=not-a-width\n", Disable));
    PresetBundle fresh;
    CHECK(fresh.fff_prints.default_preset().config.option<ConfigOptionFloatOrPercent>("perimeter_extrusion_width")->value != 143);
}

// The wizard's exact strict-system core call keeps an empty result while warning and applying effective defaults.
TEST_CASE("Strict system bundle requests warn without returning substitutions", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    bundle.fff_prints.default_preset().config.option<ConfigOptionBool>("seam_visibility")->value = true;
    REQUIRE_FALSE(static_cast<const ConfigOptionBool *>(PrintConfigDef::instance().get("seam_visibility")->default_value.get())->value);
    VendorWarnings warnings;
    const auto result = files.load(bundle,
        "[vendor]\nname=Test\nconfig_version=1.0.0\ntemplates_profile=1\n"
        "[print:system]\ndefault_settings=layer_height;unknown_system_request;printer_technology\nseam_visibility=not-a-bool\n"
        "[print:unresolved]\ndefault_settings=all\n", Disable, true);
    CHECK(result.second == 2);
    CHECK(result.first.empty());
    CHECK(vendor_config(bundle.fff_prints, "system").opt_bool("seam_visibility"));
    CHECK(warnings.text().find("not-a-bool") != std::string::npos);
    CHECK(warnings.text().find("unknown_system_request") != std::string::npos);
    CHECK(warnings.text().find("unapplied printer_technology=") != std::string::npos);
    CHECK(warnings.text().find("all") != std::string::npos);
    CHECK_THROWS(files.load(bundle,
        "[vendor]\nname=Test\nconfig_version=1.0.0\ntemplates_profile=1\n"
        "[print:old]\nseam_visibility=not-a-bool\n", Disable, true));
}

// Candidate technology uses the native option table, and an invalid declaration blocks inherited SLA even after a copy is replaced.
TEST_CASE("Bundle printer defaults select native technology before request order", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    VendorWarnings warnings;
    const auto result = files.load(bundle,
        "[printer:*sla]\nprinter_technology=SLA\ndefault_settings=printer_technology;printer_custom_variables\n"
        "[printer:*plain_sla]\nprinter_technology=SLA\n"
        "[printer:*native_parent]\nprinter_technology=SLS\ndefault_settings=printer_technology\n"
        "[printer:same_native]\ninherits=*native_parent\nprinter_technology=SLA\n"
        "[printer:inherited]\ninherits=*sla\ndefault_settings=\n"
        "[printer:before]\ndefault_settings=printer_technology\nprinter_technology=SLA\n"
        "[printer:after]\nprinter_technology=SLA\ndefault_settings=printer_technology\n"
        "[printer:invalid_before]\ninherits=*plain_sla\ndefault_settings=printer_technology\nprinter_technology=MILL\n"
        "[printer:invalid_after]\ninherits=*plain_sla\nprinter_technology=LASER\ndefault_settings=printer_technology\n"
        "[printer:sls]\nprinter_technology=SLS\ndefault_settings=printer_technology\n"
        "[printer:cnc]\nprinter_technology=CNC\ndefault_settings=printer_technology\n"
        "[printer:lsr]\nprinter_technology=LSR\ndefault_settings=printer_technology\n"
        "[printer:native_sls]\nprinter_technology=SLS\ndefault_settings=printer_custom_variables\n"
        "[printer:native_cnc]\nprinter_technology=CNC\ndefault_settings=printer_custom_variables\n"
        "[printer:native_lsr]\nprinter_technology=LSR\ndefault_settings=printer_custom_variables\n"
        "[printer:fff]\nprinter_technology=FFF\ndefault_settings=printer_technology\n");
    CHECK(result.second == 13);
    CHECK(warnings.text().find("printer:same_native changes inherited printer technology") == std::string::npos);
    size_t count = 0;
    for (const auto &group : result.first)
        count += group.substitutions.size();
    CHECK(count == 2);
    for (const auto &group : result.first) {
        REQUIRE(group.substitutions.size() == 1);
        CHECK(group.substitutions.front().opt_def == PrintConfigDef::instance().get("printer_technology"));
        const ConfigOption *replacement = group.substitutions.front().new_value.get();
        REQUIRE(replacement != nullptr);
        CHECK(replacement->get_int() == ptFFF);
    }
    for (const char *name : {"inherited", "before", "after", "sls", "cnc", "lsr", "same_native"}) {
        CAPTURE(name);
        CHECK(vendor_config(bundle.printers, name).option("printer_technology")->get_int() == ptSLA);
        CHECK_FALSE(vendor_config(bundle.printers, name).has("nozzle_diameter"));
    }
    for (const char *name : {"invalid_before", "invalid_after", "fff"}) {
        CAPTURE(name);
        CHECK(vendor_config(bundle.printers, name).option("printer_technology")->get_int() == ptFFF);
        CHECK(vendor_config(bundle.printers, name).has("nozzle_diameter"));
    }
    const std::pair<const char *, const char *> native[] = {
        {"native_sls", "SLS"}, {"native_cnc", "CNC"}, {"native_lsr", "LSR"}};
    for (const std::pair<const char *, const char *> &entry : native) {
        CHECK(vendor_config(bundle.printers, entry.first).option("printer_technology")->serialize() == entry.second);
        CHECK_FALSE(vendor_config(bundle.printers, entry.first).has("nozzle_diameter"));
    }
    // No inherited active marker or surviving copy can mask a regression of the actual-request marker.
    std::pair<PresetsConfigSubstitutions, size_t> marker;
    REQUIRE_NOTHROW(marker = files.load(bundle,
        "[printer:marker]\ndefault_settings=printer_technology\nprinter_technology=MILL\n", Disable));
    REQUIRE(marker.first.size() == 1);
    CHECK(vendor_config(bundle.printers, "marker").option("printer_technology")->get_int() == ptFFF);
}

// Category references include filament and SLA defaults; a changed descendant technology does not replay ancestor requests.
TEST_CASE("Bundle category defaults and resolved technology inheritance stay separate", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    bundle.printers.default_preset(0).config.option<ConfigOptionString>("printer_custom_variables")->value = "fff-value";
    bundle.printers.default_preset(1).config.option<ConfigOptionString>("printer_custom_variables")->value = "sla-value";
    VendorWarnings warnings;
    const auto result = files.load(bundle,
        "[filament:f]\nfilament_diameter=2.85\ndefault_settings=filament_diameter\n"
        "[sla_print:p]\nlayer_height=0.7\ndefault_settings=layer_height\n"
        "[sla_material:m]\ndefault_settings=material_colour\n"
        "[printer:*fff]\nprinter_technology=FFF\ndefault_settings=printer_custom_variables;nozzle_diameter\n"
        "[printer:changed]\ninherits=*fff\nprinter_technology=SLA\n"
        "[printer:local]\ninherits=*fff\nprinter_technology=SLA\nprinter_custom_variables=local-value\n", Enable, false, false, "f");
    CHECK(result.first.empty());
    CHECK(warnings.text().find("printer:changed changes inherited printer technology") != std::string::npos);
    CHECK(vendor_config(bundle.filaments, "f").option<ConfigOptionFloats>("filament_diameter")->get_at(0) == bundle.filaments.default_preset().config.option<ConfigOptionFloats>("filament_diameter")->get_at(0));
    CHECK(vendor_config(bundle.sla_prints, "p").opt_float("layer_height") == bundle.sla_prints.default_preset().config.opt_float("layer_height"));
    CHECK(vendor_config(bundle.sla_materials, "m").opt_string("material_colour") == bundle.sla_materials.default_preset().config.opt_string("material_colour"));
    CHECK(vendor_config(bundle.printers, "changed").opt_string("printer_custom_variables") == "fff-value");
    CHECK_FALSE(vendor_config(bundle.printers, "changed").has("nozzle_diameter"));
    CHECK(vendor_config(bundle.printers, "local").opt_string("printer_custom_variables") == "local-value");
    PresetBundle legacy;
    files.load(legacy, "[filament:f]\nfilament_diameter=1.75\n", Enable, false, false, "f");
    CHECK(bundle.filaments.get_selected_preset_name() == legacy.filaments.get_selected_preset_name());
}

// Renamed saved parents select schema from saved state only; empty and unresolved lists preserve historical loading.
TEST_CASE("Bundle saved renamed system technology ignores edited state", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    files.load_system(bundle,
        "[vendor]\nname=Test\nconfig_version=1.0.0\n"
        "[printer_model:SLAModel]\nname=SLA Model\ntechnology=SLA\nvariants=default\n"
        "[printer:Saved SLA]\nprinter_technology=SLA\nprinter_model=SLAModel\nprinter_variant=default\nrenamed_from=Old SLA;*Old SLA\nprinter_custom_variables=saved-value\n");
    const std::string *renamed = bundle.printers.get_preset_name_renamed("Old SLA");
    REQUIRE(renamed != nullptr);
    REQUIRE(*renamed == "Saved SLA");
    bundle.printers.select_preset_by_name("Saved SLA", true);
    REQUIRE(bundle.printers.get_selected_preset_name() == "Saved SLA");
    // Keep the edited FFF configuration complete so compatibility updates can inspect its extruder settings.
    bundle.printers.get_edited_preset().config = bundle.printers.default_preset(0).config;
    const Preset *saved = bundle.printers.find_preset("Saved SLA", false, false);
    const Preset *edited = bundle.printers.find_preset("Saved SLA", false, true);
    REQUIRE(saved != nullptr);
    REQUIRE(edited != nullptr);
    REQUIRE(saved != edited);
    REQUIRE(saved->config.option("printer_technology")->get_int() == ptSLA);
    REQUIRE(edited->config.option("printer_technology")->get_int() == ptFFF);
    bundle.update_compatible(PresetSelectCompatibleType::Never);
    const auto result = files.load(bundle,
        "[printer:resolved]\ninherits=Old SLA\ndefault_settings=printer_technology\n"
        "[printer:empty]\ninherits=Old SLA\ndefault_settings=\n"
        "[printer:unknown]\ninherits=Old SLA\ndefault_settings=unresolved\n");
    CHECK(result.second == 3);
    CHECK(vendor_config(bundle.printers, "resolved").option("printer_technology")->get_int() == ptSLA);
    CHECK(vendor_config(bundle.printers, "resolved").opt_string("inherits") == "Old SLA");
    CHECK(vendor_config(bundle.printers, "resolved").opt_string("printer_custom_variables") == bundle.printers.default_preset(1).config.opt_string("printer_custom_variables"));
    CHECK(vendor_config(bundle.printers, "empty").option("printer_technology")->get_int() == ptFFF);
    CHECK(vendor_config(bundle.printers, "unknown").option("printer_technology")->get_int() == ptFFF);
    CHECK(vendor_config(bundle.printers, "empty").opt_string("inherits") == "Old SLA");
    CHECK(vendor_config(bundle.printers, "unknown").opt_string("inherits") == "Old SLA");
    CHECK(bundle.printers.get_edited_preset().config.option("printer_technology")->get_int() == ptFFF);
    const auto invalid = files.load(bundle,
        "[printer:invalid_saved]\ninherits=Old SLA\ndefault_settings=printer_technology\nprinter_technology=MILL\n");
    REQUIRE(invalid.first.size() == 1);
    CHECK(vendor_config(bundle.printers, "invalid_saved").option("printer_technology")->get_int() == ptFFF);
    const auto local = files.load(bundle,
        "[printer:*Old SLA]\nprinter_technology=FFF\n"
        "[printer:local_alias_child]\ninherits=*Old SLA\ndefault_settings=printer_technology\n");
    CHECK(local.first.empty());
    // System names include former names and keep their existing priority over local parent names.
    CHECK(vendor_config(bundle.printers, "local_alias_child").option("printer_technology")->get_int() == ptSLA);
}

// No-effect requests preserve existing conversion flags, and private request failures are reported at the origin only.
TEST_CASE("Bundle no-effect lists preserve historical conversion and source diagnostics", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle ordinary;
    PresetBundle empty;
    const std::string text = "[print:old]\nseam_position=nearest\nlayer_height=0.29\nprint_custom_variables=literal=${default}\n";
    const auto original = files.load(ordinary, text, Enable, false, true);
    const auto noop = files.load(empty, text + "default_settings=\n", Enable, false, true);
    CHECK(original.first.size() == noop.first.size());
    const DynamicPrintConfig &expected = vendor_config(ordinary.fff_prints, "old");
    const DynamicPrintConfig &actual = vendor_config(empty.fff_prints, "old");
    REQUIRE(expected.keys() == actual.keys());
    for (const std::string &key : expected.keys()) {
        CAPTURE(key);
        CHECK(actual.option(key)->serialize() == expected.option(key)->serialize());
        CHECK(actual.option(key)->flags == expected.option(key)->flags);
    }
    PresetBundle bundle;
    const auto private_result = files.load(bundle,
        "[print:*private]\ndefault_settings=unknown_private\n"
        "[print:first]\ninherits=*private\ndefault_settings=layer_height\n"
        "[print:second]\ninherits=*private\ndefault_settings=perimeters\n");
    REQUIRE(private_result.first.size() == 1);
    CHECK(private_result.first.front().preset_name == "*private");
    REQUIRE(private_result.first.front().substitutions.size() == 1);
    CHECK(private_result.first.front().substitutions.front().old_name == "unknown_private");
    const DynamicPrintConfig &config = vendor_config(bundle.fff_prints, "first");
    CHECK_FALSE(config.has("unknown_private"));
    CHECK_FALSE(config.has("default_settings"));
    CHECK(config.opt_string("print_custom_variables").find("unknown_private") == std::string::npos);
}

// System and local parents arbitrate only their contributed technology in source order before local defaults are copied.
TEST_CASE("Bundle mixed saved and local parents share one technology winner", "[plugins][config][vendor-config]")
{
    VendorBundleFiles files;
    PresetBundle bundle;
    DynamicPrintConfig &fff = bundle.printers.default_preset(0).config;
    DynamicPrintConfig &sla = bundle.printers.default_preset(1).config;
    REQUIRE(fff.has("printer_notes"));
    REQUIRE(sla.has("printer_notes"));
    fff.option<ConfigOptionString>("printer_custom_variables")->value = "fff-request-default";
    sla.option<ConfigOptionString>("printer_custom_variables")->value = "sla-request-default";
    fff.option<ConfigOptionString>("printer_notes")->value = "fff-resolved-parent";
    sla.option<ConfigOptionString>("printer_notes")->value = "sla-resolved-parent";
    files.load_system(bundle,
        "[vendor]\nname=Test\nconfig_version=1.0.0\n"
        "[printer_model:SLAModel]\nname=SLA Model\ntechnology=SLA\nvariants=default\n"
        "[printer_model:FFFModel]\nname=FFF Model\ntechnology=FFF\nvariants=default\n"
        "[printer:Saved SLA]\nprinter_technology=SLA\nprinter_model=SLAModel\nprinter_variant=default\nrenamed_from=Former SLA\nprinter_notes=saved-sla-notes\n"
        "[printer:Saved FFF]\nprinter_technology=FFF\nprinter_model=FFFModel\nprinter_variant=default\nrenamed_from=Former FFF\nprinter_notes=saved-fff-notes\n");
    REQUIRE(bundle.printers.get_preset_name_renamed("Former SLA") != nullptr);
    REQUIRE(bundle.printers.get_preset_name_renamed("Former FFF") != nullptr);
    const struct MixedParentsCase {
        const char *name;
        const char *saved;
        const char *local;
        bool local_clone;
        bool saved_last;
    } cases[] = {
        {"round1_sla_fff_text_local", "Former SLA", "FFF", false, false},
        {"round1_sla_fff_text_system", "Former SLA", "FFF", false, true},
        {"round1_sla_fff_clone_local", "Former SLA", "FFF", true, false},
        {"round1_sla_fff_clone_system", "Former SLA", "FFF", true, true},
        {"round1_fff_sla_text_local", "Former FFF", "SLA", false, false},
        {"round1_fff_sla_text_system", "Former FFF", "SLA", false, true},
        {"round1_fff_sla_clone_local", "Former FFF", "SLA", true, false},
        {"round1_fff_sla_clone_system", "Former FFF", "SLA", true, true},
    };
    for (const MixedParentsCase &entry : cases) {
        CAPTURE(entry.name);
        const bool local_fff = std::string(entry.local) == "FFF";
        const bool expected_fff = entry.saved_last ? !local_fff : local_fff;
        const std::string parents = entry.saved_last ? std::string("*local;") + entry.saved : std::string(entry.saved) + ";*local";
        const std::string source = std::string("[printer:*local]\nprinter_technology=") + entry.local +
            "\ndefault_settings=printer_notes" + (entry.local_clone ? ";printer_technology" : "") +
            "\n[printer:*partial]\nprinter_variant=partial-variant\n[printer:" + entry.name + "]\ninherits=" +
            parents + ";*partial\ndefault_settings=printer_custom_variables\n";
        const auto result = files.load(bundle, source);
        CHECK(result.second == 1);
        CHECK(result.first.empty());
        const DynamicPrintConfig &config = vendor_config(bundle.printers, entry.name);
        CHECK(config.option("printer_technology")->get_int() == (expected_fff ? ptFFF : ptSLA));
        CHECK(config.has("nozzle_diameter") == expected_fff);
        CHECK(config.opt_string("printer_custom_variables") == (expected_fff ? "fff-request-default" : "sla-request-default"));
        CHECK(config.opt_string("printer_notes") == (local_fff ? "fff-resolved-parent" : "sla-resolved-parent"));
        CHECK(config.opt_string("printer_variant") == "partial-variant");
        CHECK(config.opt_string("printer_model").empty());
    }
    const auto inactive = files.load(bundle,
        "[printer:*plain]\nprinter_technology=FFF\n"
        "[printer:round1_old_mixed]\ninherits=*plain;Former SLA\ndefault_settings=\n"
        "[printer:round1_old_system]\ninherits=Former SLA\n"
        "[printer:round1_unknown_system]\ninherits=Former SLA\ndefault_settings=unresolved_round1\n");
    CHECK(inactive.second == 3);
    REQUIRE(inactive.first.size() == 1);
    REQUIRE(inactive.first.front().substitutions.size() == 1);
    CHECK(inactive.first.front().substitutions.front().opt_def == nullptr);
    CHECK(inactive.first.front().substitutions.front().new_value.get() == nullptr);
    for (const char *name : {"round1_old_mixed", "round1_old_system", "round1_unknown_system"}) {
        CAPTURE(name);
        const DynamicPrintConfig &config = vendor_config(bundle.printers, name);
        CHECK(config.option("printer_technology")->get_int() == ptFFF);
        CHECK(config.has("nozzle_diameter"));
    }
}

namespace {
VendorBundleFiles::VendorBundleFiles()
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    previous = has_data_dir() ? data_dir() : std::string();
    directory = boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
        "build" / "temp" / "vendor_config" / boost::filesystem::unique_path("bundle-%%%%-%%%%");
    boost::filesystem::create_directories(directory / "data");
    set_data_dir((directory / "data").string());
}

VendorBundleFiles::~VendorBundleFiles()
{
    set_data_dir(previous);
    boost::system::error_code error;
    boost::filesystem::remove_all(directory, error);
}

std::pair<PresetsConfigSubstitutions, size_t> VendorBundleFiles::load(PresetBundle &bundle, const std::string &text,
    ForwardCompatibilitySubstitutionRule rule, bool system, bool from_prusa, const std::string &expected_filament_selection)
{
    const boost::filesystem::path source = directory / ("source-" + std::to_string(sequence++) + ".ini");
    {
        std::ofstream stream(source.string(), std::ios::binary);
        stream << text;
        REQUIRE(stream.good());
    }
    std::vector<std::string> selected = {bundle.printers.get_selected_preset_name(),
        bundle.fff_prints.get_selected_preset_name(), bundle.filaments.get_selected_preset_name(),
        bundle.sla_prints.get_selected_preset_name(), bundle.sla_materials.get_selected_preset_name()};
    // Importing the first filament retains the loader's existing automatic selection of that filament.
    if (!expected_filament_selection.empty())
        selected[2] = expected_filament_selection;
    const std::unordered_set<Plugin *> active = Orchestrator::instance().active_plugins();
    const std::map<std::string, std::string> before = vendor_files_snapshot(directory / "data");
    std::pair<PresetsConfigSubstitutions, size_t> result;
    const PresetBundle::LoadConfigBundleAttributes flags = system ? PresetBundle::LoadConfigBundleAttributes{PresetBundle::LoadSystem} :
        from_prusa ? PresetBundle::LoadConfigBundleAttributes{PresetBundle::ConvertFromPrusa} : PresetBundle::LoadConfigBundleAttributes{};
    result = bundle.load_configbundle(source.string(), flags, rule);
    std::ifstream stream(source.string(), std::ios::binary);
    const std::string after((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    CHECK(after == text);
    CHECK(vendor_files_snapshot(directory / "data") == before);
    const std::vector<std::string> after_selection = {bundle.printers.get_selected_preset_name(),
        bundle.fff_prints.get_selected_preset_name(), bundle.filaments.get_selected_preset_name(),
        bundle.sla_prints.get_selected_preset_name(), bundle.sla_materials.get_selected_preset_name()};
    CHECK(after_selection == selected);
    CHECK(Orchestrator::instance().active_plugins() == active);
    return result;
}

void VendorBundleFiles::load_system(PresetBundle &bundle, const std::string &text)
{
    bundle.setup_directories();
    const boost::filesystem::path source = directory / "data" / "vendor" / "Test.ini";
    {
        std::ofstream stream(source.string(), std::ios::binary);
        stream << text;
        REQUIRE(stream.good());
    }
    const std::map<std::string, std::string> before = vendor_files_snapshot(directory / "data");
    AppConfig config(AppConfig::EAppMode::Editor);
    config.set_variant("Test", "SLAModel", "default", true);
    PresetsConfigSubstitutions substitutions;
    REQUIRE_NOTHROW(substitutions = bundle.load_presets(config, Disable));
    CHECK(substitutions.empty());
    CHECK(vendor_files_snapshot(directory / "data") == before);
}

std::map<std::string, std::string> vendor_files_snapshot(const boost::filesystem::path &directory)
{
    std::map<std::string, std::string> files;
    for (const boost::filesystem::directory_entry &entry : boost::filesystem::recursive_directory_iterator(directory)) {
        if (!boost::filesystem::is_regular_file(entry.path()))
            continue;
        std::ifstream stream(entry.path().string(), std::ios::binary);
        REQUIRE(stream.good());
        files[entry.path().string()] = std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
    }
    return files;
}

const DynamicPrintConfig &vendor_config(const PresetCollection &collection, const std::string &name)
{
    const Preset *preset = collection.find_preset(name, false, false);
    REQUIRE(preset != nullptr);
    return preset->config;
}

VendorWarnings::VendorWarnings()
    : messages(boost::make_shared<std::ostringstream>()), sink(boost::make_shared<Sink>()), level(get_logging_level())
{
    sink->locked_backend()->add_stream(messages);
    boost::log::core::get()->add_sink(sink);
    set_logging_level(std::max(level, 2u));
}

VendorWarnings::~VendorWarnings()
{
    boost::log::core::get()->remove_sink(sink);
    set_logging_level(level);
}

std::string VendorWarnings::text()
{
    sink->flush();
    return messages->str();
}
}
