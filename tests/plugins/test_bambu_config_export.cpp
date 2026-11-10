#include <catch2/catch.hpp>

#include <algorithm>
#include <clocale>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/Preset.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Format/BBConfig.hpp"
#include "libslic3r/Format/BambuConfigExport.hpp"
#include "libslic3r/Format/CustomVariables.hpp"
#include "libslic3r/Format/OrcaConfigKeys.hpp"

/*
Bambu configuration export and import tests
===========================================

The export tests ([bambu][config-export]) translate a SuperSlicer configuration and compare the Orca
keys and values with literal expectations written by hand from Orca's definitions, never computed from
the translation tables under test. The import tests ([bbconfig]) check the corrections of the Bambu
import that share these tables. The two Bambu settings added to the firmware (nozzle type, plate type)
and the printer identity have their own definition test.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- Test::Plugins::ensure_plugin_test_runtime_initialized()
        |-- PrintConfigDef::instance()
        |-- model_id->default_value->serialize()
        |-- nozzle_type->default_value->serialize()
        |-- nozzle_type->enum_def->values()
        `-- strings()
*/

namespace {
using namespace Slic3r;

// A local configuration definition permits independent destination permissions without changing global settings.
class ProjectFilteredConfig : public DynamicConfig {
public:
    ConfigDef definitions;
    // Return the definitions owned by this test configuration.
    const ConfigDef *def() const override;
};

// Borrow this object's definitions for option lookup; no ownership moves, allocation or failure occurs.
const ConfigDef *ProjectFilteredConfig::def() const { return &definitions; }

// Keep the numeric C locale of the calling thread while a test temporarily changes its decimal separator.
class NumericLocaleRestore {
public:
    // Save the current numeric locale for restoration at scope exit.
    NumericLocaleRestore();
    // Restore the saved numeric locale, including when an assertion aborts the test.
    ~NumericLocaleRestore();
private:
    std::string previous;
};

// Copy the current borrowed locale name into owned storage before tests change it; allocation can fail.
NumericLocaleRestore::NumericLocaleRestore() : previous(std::setlocale(LC_NUMERIC, nullptr)) {}

// Restore the previously saved locale without throwing; setlocale retains ownership of its result.
NumericLocaleRestore::~NumericLocaleRestore() { std::setlocale(LC_NUMERIC, previous.c_str()); }

// A full configuration (plugin settings included) with some values replaced.
DynamicPrintConfig make_config(std::initializer_list<std::pair<const char *, const char *>> values,
                               size_t extruders = 1)
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    if (extruders > 1) {
        config.set_num_extruders(unsigned(extruders));
        Preset::normalize(config);
    }
    for (const std::pair<const char *, const char *> &value : values)
        config.set_deserialize_strict(value.first, value.second);
    return config;
}

// The entry of an Orca key, or nullptr when it was not written.
const BambuConfigEntry *find_entry(const BambuConfigTranslation &translation, const std::string &key)
{
    const auto it = translation.entries.find(key);
    return it == translation.entries.end() ? nullptr : &it->second;
}

// The values of an entry, empty when absent.
std::vector<std::string> values_of(const BambuConfigTranslation &translation, const std::string &key)
{
    const BambuConfigEntry *entry = find_entry(translation, key);
    return entry != nullptr ? entry->values : std::vector<std::string>();
}

std::vector<std::string> strings(std::initializer_list<const char *> list)
{
    return std::vector<std::string>(list.begin(), list.end());
}

bool has_warning(const BambuConfigTranslation &translation, const std::string &part)
{
    return std::any_of(translation.warnings.begin(), translation.warnings.end(),
                       [&part](const std::string &warning) { return warning.find(part) != std::string::npos; });
}

// The configuration written, read back from a Bambu JSON file, through the real import.
DynamicPrintConfig import_json(const std::string &json)
{
    const boost::filesystem::path path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("bambu_export_test_%%%%%%%%.json");
    {
        std::ofstream file(path.string(), std::ios::binary);
        file << json;
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    REQUIRE(read_json_file_bambu(std_path(path.string()), config, substitutions, false));
    boost::filesystem::remove(path);
    return config;
}

// Import serialized Bambu settings into a full configuration.
DynamicPrintConfig import_settings(const std::map<std::string, std::string> &settings,
                                   DynamicPrintConfig config = DynamicPrintConfig::full_print_config())
{
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    REQUIRE(convert_settings_from_bambu(settings, config, substitutions, false));
    return config;
}

} // namespace

TEST_CASE("Settings are renamed to their Orca key and written in the Orca form",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"perimeters", "3"},
                                             {"fill_density", "15%"},
                                             {"top_solid_layers", "5"},
                                             {"layer_height", "0.2"},
                                             {"retract_length", "0.8"},
                                             {"brim_speed", "45"},
                                             {"spiral_vase", "1"},
                                             {"first_layer_height", "0.3"},
                                             {"printer_technology", "FFF"},
                                             {"bed_shape", "0x0,200x0,200x200,0x200"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "wall_loops") == strings({"3"}));
    CHECK(values_of(translation, "sparse_infill_density") == strings({"15%"}));
    CHECK(values_of(translation, "top_shell_layers") == strings({"5"}));
    CHECK(values_of(translation, "layer_height") == strings({"0.2"}));
    CHECK(values_of(translation, "retraction_length") == strings({"0.8"}));
    CHECK(values_of(translation, "skirt_speed") == strings({"45"}));
    CHECK(values_of(translation, "spiral_mode") == strings({"1"}));
    CHECK(values_of(translation, "initial_layer_print_height") == strings({"0.3"}));
    CHECK(values_of(translation, "printer_technology") == strings({"FFF"}));
    CHECK(values_of(translation, "printable_area") == strings({"0x0", "200x0", "200x200", "0x200"}));
    // The names of the import table are never written.
    for (const char *ss_key : {"perimeters", "fill_density", "top_solid_layers", "retract_length", "spiral_vase", "bed_shape", "brim_speed"})
        CHECK(find_entry(translation, ss_key) == nullptr);
    CHECK(find_entry(translation, "wall_loops")->kind == BambuValueKind::Scalar);
    CHECK(find_entry(translation, "printable_area")->kind == BambuValueKind::NumericVector);
}

TEST_CASE("A setting without an Orca equivalent is omitted and counted once",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"resolution_internal", "0.0125"}, {"avoid_crossing_not_first_layer", "1"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    // None of them is an Orca setting, so none keeps its SuperSlicer name.
    for (const char *ss_key : {"resolution_internal", "avoid_crossing_not_first_layer", "gcode_flavor", "platter_fan", "printer_model_id"})
        CHECK(find_entry(translation, ss_key) == nullptr);
    const std::vector<std::string>::difference_type counts = std::count_if(translation.warnings.begin(), translation.warnings.end(),
        [](const std::string &warning) { return warning.find("have no Orca equivalent") != std::string::npos; });
    CHECK(counts == 1);
    // Every key written is a key that Orca defines.
    for (const auto &[key, entry] : translation.entries)
        CHECK(find_orca_config_key(key) != nullptr);
}

TEST_CASE("A SuperSlicer setting does not take the name of an Orca setting that the import renames",
          "[bambu][config-export]")
{
    // Orca's skirt_speed is SuperSlicer's brim_speed: SuperSlicer's own skirt_speed is another setting.
    DynamicPrintConfig config = make_config({{"skirt_speed", "30"}, {"brim_speed", "45"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "skirt_speed") == strings({"45"}));
}

TEST_CASE("Enumeration values are written as Orca names, not as the inverse of the migration table",
          "[bambu][config-export]")
{
    SECTION("same name")
    {
        // "rectilinear" is also what the import maps "zig-zag" to, but the export must not write "zig-zag".
        DynamicPrintConfig config = make_config({{"fill_pattern", "rectilinear"}, {"top_fill_pattern", "monotonic"}, {"perimeter_generator", "arachne"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "sparse_infill_pattern") == strings({"rectilinear"}));
        CHECK(values_of(translation, "top_surface_pattern") == strings({"monotonic"}));
        CHECK(values_of(translation, "wall_generator") == strings({"arachne"}));
    }
    SECTION("renamed values")
    {
        DynamicPrintConfig config = make_config({{"fill_pattern", "stars"},
                                                 {"seam_position", "rear"},
                                                 {"complete_objects", "1"},
                                                 {"thumbnails_format", "BIQU"},
                                                 {"gcode_flavor", "sprinter"},
                                                 {"machine_limits_usage", "emit_to_gcode"},
                                                 {"gcode_label_objects", "octoprint"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "sparse_infill_pattern") == strings({"tri-hexagon"}));
        CHECK(values_of(translation, "seam_position") == strings({"back"}));
        CHECK(values_of(translation, "print_sequence") == strings({"by object"}));
        CHECK(values_of(translation, "thumbnails_format") == strings({"BTT_TFT"}));
        CHECK(values_of(translation, "gcode_flavor") == strings({"reprap"}));
        CHECK(values_of(translation, "emit_machine_limits_to_gcode") == strings({"1"}));
        CHECK(values_of(translation, "gcode_label_objects") == strings({"1"}));
    }
    SECTION("vectors of renamed values")
    {
        DynamicPrintConfig config = make_config({{"filament_type", "FLEX;PLA"}, {"retract_lift_top", "Not on top;Only on top"}}, 2);
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "filament_type") == strings({"TPU", "PLA"}));
        CHECK(find_entry(translation, "filament_type")->kind == BambuValueKind::StringVector);
        CHECK(values_of(translation, "retract_lift_enforce") == strings({"Bottom Only", "Top Only"}));
    }
}

TEST_CASE("A value that Orca cannot read omits the setting and adds a warning",
          "[bambu][config-export]")
{
    // "cost" is SuperSlicer's own seam position.
    DynamicPrintConfig config = make_config({{"seam_position", "cost"}, {"infill_every_layers", "3"}, {"layer_height", "0.2"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(find_entry(translation, "seam_position") == nullptr);
    CHECK(has_warning(translation, "seam_position: the value 'cost' is not accepted by Orca's seam_position"));
    // 3 layers have no exact Orca equivalent: Orca only combines 10 layers.
    CHECK(find_entry(translation, "infill_combination") == nullptr);
    CHECK(has_warning(translation, "infill_every_layers: the value '3' is not accepted by Orca's infill_combination"));
    // The other settings are not affected.
    CHECK(values_of(translation, "layer_height") == strings({"0.2"}));
}

TEST_CASE("Vectors have one element per filament and scalars stay scalars",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"temperature", "200,210,220"},
                                             {"filament_colour", "#FF0000;#00FF00;#0000FF"},
                                             {"nozzle_diameter", "0.4,0.4,0.6"},
                                             {"extrusion_multiplier", "1,0.95,1"}}, 3);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "nozzle_temperature") == strings({"200", "210", "220"}));
    CHECK(values_of(translation, "filament_colour") == strings({"#FF0000", "#00FF00", "#0000FF"}));
    CHECK(values_of(translation, "nozzle_diameter") == strings({"0.4", "0.4", "0.6"}));
    CHECK(values_of(translation, "filament_flow_ratio") == strings({"1", "0.95", "1"}));
}

TEST_CASE("The plate temperatures are written for the six Orca plates",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"bed_temperature", "60,70"}, {"first_layer_bed_temperature", "65,75"}}, 2);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    for (const char *plate : {"supertack_plate", "cool_plate", "textured_cool_plate", "eng_plate", "hot_plate", "textured_plate"}) {
        INFO(plate);
        CHECK(values_of(translation, std::string(plate) + "_temp") == strings({"60", "70"}));
        CHECK(values_of(translation, std::string(plate) + "_temp_initial_layer") == strings({"65", "75"}));
    }
    CHECK(find_entry(translation, "bed_temperature") == nullptr);
    CHECK(find_entry(translation, "first_layer_bed_temperature") == nullptr);
}

TEST_CASE("G-code templates keep their text and only change the name of their setting",
          "[bambu][config-export]")
{
    const char *start = "G28 ; version name from\nM104 S[first_layer_temperature] {if layer_height > 0.1}M117 \"x\"{endif}\n";
    DynamicPrintConfig config = make_config({{"start_gcode", start}, {"end_gcode", "M84"}, {"layer_gcode", "G92 E0"}});
    config.set_deserialize_strict("start_filament_gcode", "; filament start\"");
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "machine_start_gcode") == strings({start}));
    CHECK(values_of(translation, "machine_end_gcode") == strings({"M84"}));
    CHECK(values_of(translation, "layer_change_gcode") == strings({"G92 E0"}));
    CHECK(find_entry(translation, "start_gcode") == nullptr);
    CHECK(find_entry(translation, "machine_start_gcode")->kind == BambuValueKind::Scalar);
    CHECK(find_entry(translation, "filament_start_gcode")->kind == BambuValueKind::StringVector);
}

TEST_CASE("Connection settings never reach the Orca files",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"print_host", "user:password@printer.local"},
                                             {"printhost_apikey", "secret-key"},
                                             {"printhost_password", "secret-password"},
                                             {"printhost_user", "secret-user"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(find_entry(translation, "print_host") == nullptr);
    const std::string json = bambu_project_settings_json(translation, "02.08.01.55");
    std::string block;
    for (const auto &[key, value] : bambu_config_block(translation))
        block += key + " = " + value + "\n";
    for (const std::string &output : {json, block})
        for (const char *secret : {"user:password", "printer.local", "secret-key", "secret-password", "secret-user"}) {
            INFO(secret);
            CHECK(output.find(secret) == std::string::npos);
        }
}

TEST_CASE("A disabled setting is not written",
          "[bambu][config-export]")
{
    {
        DynamicPrintConfig config = make_config({{"bridge_angle", "!0"}});
        CHECK(find_entry(translate_config_to_bambu(config), "bridge_angle") == nullptr);
    }
    {
        DynamicPrintConfig config = make_config({{"bridge_angle", "15"}});
        CHECK(values_of(translate_config_to_bambu(config), "bridge_angle") == strings({"15"}));
    }
}

TEST_CASE("The chamber fan settings of the Bambu firmware are written under their Orca names",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"platter_fan", "1"},
                                             {"support_air_filtration", "1"},
                                             {"platter_fan_speed", "70,80"},
                                             {"activate_air_filtration", "1,0"},
                                             {"during_print_exhaust_fan_speed", "60,65"},
                                             {"complete_print_exhaust_fan_speed", "80,85"}}, 2);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "auxiliary_fan") == strings({"1"}));
    CHECK(values_of(translation, "support_air_filtration") == strings({"1"}));
    CHECK(values_of(translation, "additional_cooling_fan_speed") == strings({"70", "80"}));
    CHECK(values_of(translation, "activate_air_filtration") == strings({"1", "0"}));
    CHECK(values_of(translation, "during_print_exhaust_fan_speed") == strings({"60", "65"}));
    CHECK(values_of(translation, "complete_print_exhaust_fan_speed") == strings({"80", "85"}));
    CHECK(find_entry(translation, "platter_fan") == nullptr);
    CHECK(find_entry(translation, "platter_fan_speed") == nullptr);
}

TEST_CASE("The firmware printer variables are written as Orca settings",
          "[bambu][config-export]")
{
    SECTION("both variables")
    {
        DynamicPrintConfig config = make_config({{"printer_custom_variables", "other = 5\nscan_first_layer = True\nenable_power_loss_recovery = enable\n"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "scan_first_layer") == strings({"1"}));
        CHECK(values_of(translation, "enable_power_loss_recovery") == strings({"enable"}));
        CHECK(find_entry(translation, "other") == nullptr);
    }
    SECTION("the textual rules of the firmware")
    {
        // "1.0" is not "1": the firmware does not scan, so the export writes 0.
        DynamicPrintConfig config = make_config({{"printer_custom_variables", "scan_first_layer = 1.0\nenable_power_loss_recovery = disable"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "scan_first_layer") == strings({"0"}));
        CHECK(values_of(translation, "enable_power_loss_recovery") == strings({"disable"}));
    }
    SECTION("absent or invalid")
    {
        DynamicPrintConfig config = make_config({{"printer_custom_variables", "enable_power_loss_recovery = sometimes"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(find_entry(translation, "scan_first_layer") == nullptr);
        CHECK(find_entry(translation, "enable_power_loss_recovery") == nullptr);
        CHECK(has_warning(translation, "enable_power_loss_recovery"));
    }
}

TEST_CASE("The nozzle type and plate type created by the Bambu firmware are written",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"nozzle_type", "hardened_steel"},
                                             {"curr_bed_type", "Textured PEI Plate"},
                                             {"printer_model_id", "BL-P001"},
                                             {"printer_model", "Bambu Lab X1 Carbon"},
                                             {"nozzle_diameter", "0.4,0.4"}}, 2);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "nozzle_type") == strings({"hardened_steel", "hardened_steel"}));
    CHECK(find_entry(translation, "nozzle_type")->kind == BambuValueKind::NumericVector);
    CHECK(values_of(translation, "curr_bed_type") == strings({"Textured PEI Plate"}));
    CHECK(values_of(translation, "printer_model") == strings({"Bambu Lab X1 Carbon"}));
    // The model id is only for the slice information: it is in no output.
    CHECK(find_entry(translation, "printer_model_id") == nullptr);
    CHECK(bambu_project_settings_json(translation, "1").find("BL-P001") == std::string::npos);
    for (const auto &[key, value] : bambu_config_block(translation))
        CHECK(value.find("BL-P001") == std::string::npos);
}

TEST_CASE("The firmware settings are written whatever firmware is selected, and only when the configuration has them",
          "[bambu][config-export]")
{
    SECTION("another firmware selected")
    {
        DynamicPrintConfig config = make_config({{"gcode_flavor", "marlin2"}, {"nozzle_type", "brass"}, {"platter_fan", "1"}});
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "nozzle_type") == strings({"brass"}));
        CHECK(values_of(translation, "auxiliary_fan") == strings({"1"}));
    }
    SECTION("settings absent from the configuration")
    {
        Test::Plugins::ensure_plugin_test_runtime_initialized();
        DynamicPrintConfig config;
        config.set_deserialize_strict("layer_height", "0.2");
        const BambuConfigTranslation translation = translate_config_to_bambu(config);
        CHECK(values_of(translation, "layer_height") == strings({"0.2"}));
        for (const char *key : {"nozzle_type", "curr_bed_type", "auxiliary_fan", "additional_cooling_fan_speed", "scan_first_layer"})
            CHECK(find_entry(translation, key) == nullptr);
    }
}

TEST_CASE("The conversions of the import are written backwards when the inverse is exact",
          "[bambu][config-export]")
{
    SECTION("ironing")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"ironing", "0"}, {"ironing_type", "top"}})), "ironing_type") == strings({"no ironing"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"ironing", "1"}, {"ironing_type", "solid"}})), "ironing_type") == strings({"solid"}));
    }
    SECTION("brim")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"brim_ears", "1"}, {"brim_width", "5"}})), "brim_type") == strings({"brim_ears"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"brim_ears", "0"}, {"brim_width", "5"}})), "brim_type") == strings({"outer_only"}));
    }
    SECTION("remaining times")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"remaining_times", "1"}, {"remaining_times_type", "m73"}})), "disable_m73") == strings({"0"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"remaining_times", "0"}, {"remaining_times_type", "m73"}})), "disable_m73") == strings({"1"}));
        // Another method has no Orca equivalent.
        const BambuConfigTranslation other = translate_config_to_bambu(make_config({{"remaining_times", "1"}, {"remaining_times_type", "m117"}}));
        CHECK(find_entry(other, "disable_m73") == nullptr);
        CHECK(has_warning(other, "remaining_times: the value '1' has no exact Orca equivalent"));
    }
    SECTION("arc fitting")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"arc_fitting", "bambu"}})), "enable_arc_fitting") == strings({"enabled"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"arc_fitting", "disabled"}})), "enable_arc_fitting") == strings({"disabled"}));
        CHECK(find_entry(translate_config_to_bambu(make_config({{"arc_fitting", "emit_center"}})), "enable_arc_fitting") == nullptr);
    }
    SECTION("percentages that Orca stores as ratios")
    {
        const BambuConfigTranslation translation = translate_config_to_bambu(make_config({{"bridge_flow_ratio", "95%"},
                                                                                           {"fill_top_flow_ratio", "100%"},
                                                                                           {"first_layer_flow_ratio", "110%"},
                                                                                           {"print_extrusion_multiplier", "98%"}}));
        CHECK(values_of(translation, "bridge_flow") == strings({"0.95"}));
        CHECK(values_of(translation, "top_solid_infill_flow_ratio") == strings({"1"}));
        CHECK(values_of(translation, "bottom_solid_infill_flow_ratio") == strings({"1.1"}));
        CHECK(values_of(translation, "print_flow_ratio") == strings({"0.98"}));
    }
    SECTION("first layer compensation changes sign")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"first_layer_size_compensation", "-0.2"}})), "elefant_foot_compensation") == strings({"0.2"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"first_layer_size_compensation", "0"}})), "elefant_foot_compensation") == strings({"0"}));
        const BambuConfigTranslation positive = translate_config_to_bambu(make_config({{"first_layer_size_compensation", "0.2"}}));
        CHECK(find_entry(positive, "elefant_foot_compensation") == nullptr);
        CHECK(has_warning(positive, "first_layer_size_compensation"));
    }
    SECTION("bridges")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"bridge_type", "nozzle"}})), "thick_bridges") == strings({"1"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"bridge_type", "flow"}})), "thick_bridges") == strings({"0"}));
        CHECK(find_entry(translate_config_to_bambu(make_config({{"bridge_type", "height"}})), "thick_bridges") == nullptr);
    }
    SECTION("infill combination")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"infill_every_layers", "1"}})), "infill_combination") == strings({"0"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"infill_every_layers", "10"}})), "infill_combination") == strings({"1"}));
    }
    SECTION("vertical shell thickness")
    {
        CHECK(values_of(translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "disabled"}})), "ensure_vertical_shell_thickness") == strings({"none"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "enabled_old"}, {"solid_over_perimeters", "2"}})), "ensure_vertical_shell_thickness") == strings({"ensure_critical_only"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "enabled_old"}, {"solid_over_perimeters", "4"}})), "ensure_vertical_shell_thickness") == strings({"ensure_moderate"}));
        CHECK(values_of(translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "enabled_old"}, {"solid_over_perimeters", "0"}})), "ensure_vertical_shell_thickness") == strings({"ensure_all"}));
        CHECK(find_entry(translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "partial"}})), "ensure_vertical_shell_thickness") == nullptr);
        const BambuConfigTranslation unsupported = translate_config_to_bambu(make_config({{"ensure_vertical_shell_thickness", "enabled_old"}, {"solid_over_perimeters", "3"}}));
        CHECK(find_entry(unsupported, "ensure_vertical_shell_thickness") == nullptr);
        CHECK(has_warning(unsupported, "ensure_vertical_shell_thickness"));
    }
}

TEST_CASE("The project settings JSON is a flat sorted object of strings with its headers",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"perimeters", "3"}, {"temperature", "200,210"}, {"start_gcode", "G28\nM84"}}, 2);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    const std::string text = bambu_project_settings_json(translation, "02.08.01.55");
    const nlohmann::json root = nlohmann::json::parse(text);
    REQUIRE(root.is_object());
    CHECK(root.at("version") == "02.08.01.55");
    CHECK(root.at("name") == "project_settings");
    CHECK(root.at("from") == "project");
    CHECK(root.at("wall_loops").is_string());
    CHECK(root.at("wall_loops") == "3");
    REQUIRE(root.at("nozzle_temperature").is_array());
    CHECK(root.at("nozzle_temperature") == nlohmann::json::array({"200", "210"}));
    // A template keeps its line break (as a JSON escape in the file).
    CHECK(root.at("machine_start_gcode") == "G28\nM84");
    CHECK(text.find("\"machine_start_gcode\": \"G28\\nM84\"") != std::string::npos);
    // Every value is a string or an array of strings.
    for (const auto &item : root.items()) {
        INFO(item.key());
        if (item.value().is_array())
            for (const nlohmann::json &element : item.value())
                CHECK(element.is_string());
        else
            CHECK(item.value().is_string());
    }
    // Sorted keys: "from" < "name" < "version", and "layer_height" < "wall_loops".
    CHECK(text.find("\"from\"") < text.find("\"name\""));
    CHECK(text.find("\"name\"") < text.find("\"version\""));
    CHECK(text.find("\"layer_height\"") < text.find("\"wall_loops\""));
}

TEST_CASE("A template that is not valid UTF-8 is written with its bad bytes replaced",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({});
    config.opt_string("start_gcode", true) = std::string("G28 ; caf\xE9 \xFF end");
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    std::string text;
    REQUIRE_NOTHROW(text = bambu_project_settings_json(translation, "1"));
    const nlohmann::json root = nlohmann::json::parse(text);
    // The key stays, the bad bytes become U+FFFD.
    REQUIRE(root.contains("machine_start_gcode"));
    CHECK(root.at("machine_start_gcode") == "G28 ; caf\xEF\xBF\xBD \xEF\xBF\xBD end");
}

TEST_CASE("The CONFIG_BLOCK lines follow Orca's format",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"perimeters", "3"},
                                             {"temperature", "200,210"},
                                             {"filament_type", "PLA;PETG"},
                                             {"filament_settings_id", "Bambu PLA Basic @BBL X1C;Generic PETG"},
                                             {"bed_shape", "0x0,200x0"},
                                             {"curr_bed_type", "Textured PEI Plate"},
                                             {"retract_lift_top", "Only on top;All surfaces"}}, 2);
    config.opt_string("start_gcode", true) = "G28 ; \"home\" \\ back\r\nM84\n";
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    const std::vector<std::pair<std::string, std::string>> lines = bambu_config_block(translation);
    std::map<std::string, std::string> by_key(lines.begin(), lines.end());
    CHECK(by_key.at("wall_loops") == "3");
    // Numbers are joined by commas.
    CHECK(by_key.at("nozzle_temperature") == "200,210");
    CHECK(by_key.at("printable_area") == "0x0,200x0");
    // Enumerations too, and a name with spaces is not quoted.
    CHECK(by_key.at("curr_bed_type") == "Textured PEI Plate");
    CHECK(by_key.at("retract_lift_enforce") == "Top Only,All Surfaces");
    // Strings are joined by semicolons, quoted when they hold a space or a special character.
    CHECK(by_key.at("filament_type") == "PLA;PETG");
    CHECK(by_key.at("filament_settings_id") == "\"Bambu PLA Basic @BBL X1C\";\"Generic PETG\"");
    // A template is on one line: no raw line break can end the comment.
    CHECK(by_key.at("machine_start_gcode") == "G28 ; \\\"home\\\" \\\\ back\\r\\nM84\\n");
    CHECK(std::is_sorted(lines.begin(), lines.end()));
    for (const auto &[key, value] : lines)
        CHECK(value.find('\n') == std::string::npos);
}

TEST_CASE("The default configuration of three filaments is written in Orca types only",
          "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({}, 3);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(translation.entries.size() > 150);
    for (const auto &[key, entry] : translation.entries) {
        INFO(key);
        const OrcaConfigKey *orca = find_orca_config_key(key);
        REQUIRE(orca != nullptr);
        // Geometry, normal/silent machine limits, command lists and purge-volume pairs are not filament arrays.
        if (entry.kind != BambuValueKind::Scalar && orca->type != OrcaOptionType::Points &&
            orca->type != OrcaOptionType::PointsGroups && key.rfind("machine_", 0) != 0 &&
            key != "post_process" && key != "wiping_volumes_extruders" && key != "wiping_volumes_matrix" &&
            key != "wipe_tower_x" && key != "wipe_tower_y")
            CHECK(entry.values.size() == 3);
        CHECK((entry.kind == BambuValueKind::Scalar) == !orca_option_is_vector(orca->type));
        if (orca->type == OrcaOptionType::Bool || orca->type == OrcaOptionType::Bools)
            for (const std::string &value : entry.values)
                CHECK((value == "0" || value == "1"));
        if (orca->type == OrcaOptionType::Enum || orca->type == OrcaOptionType::Enums)
            for (const std::string &value : entry.values)
                CHECK((std::string("|") + orca->enum_values + "|").find("|" + value + "|") != std::string::npos);
    }
    // The vectors have three elements, one per filament.
    CHECK(values_of(translation, "nozzle_temperature").size() == 3);
    CHECK(values_of(translation, "nozzle_type").size() == 3);
    CHECK(values_of(translation, "hot_plate_temp").size() == 3);
    CHECK(values_of(translation, "wipe_tower_x").size() == 1);
    CHECK(values_of(translation, "wipe_tower_y").size() == 1);
    CHECK(values_of(translation, "curr_bed_type") == strings({"Cool Plate"}));
    CHECK(values_of(translation, "nozzle_type") == strings({"undefine", "undefine", "undefine"}));
    // Every JSON value reads back.
    CHECK_NOTHROW(nlohmann::json::parse(bambu_project_settings_json(translation, "02.08.01.55")));
}

TEST_CASE("The table of renamed values only names values that Orca accepts",
          "[bambu][config-export]")
{
    REQUIRE_FALSE(bambu_export_value_rules().empty());
    for (const BambuExportValueRule &rule : bambu_export_value_rules()) {
        INFO(rule.ss_key << " " << rule.ss_value << " -> " << rule.orca_value);
        const std::optional<std::string> orca_key = bambu_export_key(rule.ss_key);
        REQUIRE(orca_key.has_value());
        const OrcaConfigKey *orca = find_orca_config_key(*orca_key);
        REQUIRE(orca != nullptr);
        if (orca->type == OrcaOptionType::Bool || orca->type == OrcaOptionType::Bools)
            CHECK((std::string(rule.orca_value) == "0" || std::string(rule.orca_value) == "1"));
        else if (orca->type == OrcaOptionType::Enum || orca->type == OrcaOptionType::Enums)
            CHECK((std::string("|") + orca->enum_values + "|").find(std::string("|") + rule.orca_value + "|") != std::string::npos);
        // A rename that does nothing is a mistake.
        CHECK(std::string(rule.ss_value) != rule.orca_value);
    }
}

TEST_CASE("The generated list of Orca keys is sorted and complete",
          "[bambu][config-export]")
{
    CHECK(orca_config_key_count() == 808);
    CHECK(std::is_sorted(orca_config_keys(), orca_config_keys() + orca_config_key_count(),
                         [](const OrcaConfigKey &left, const OrcaConfigKey &right) { return std::strcmp(left.key, right.key) < 0; }));
    CHECK(find_orca_config_key("wall_loops") != nullptr);
    CHECK(find_orca_config_key("wall_loops")->type == OrcaOptionType::Int);
    CHECK(find_orca_config_key("nozzle_type")->type == OrcaOptionType::Enums);
    CHECK(std::string(find_orca_config_key("curr_bed_type")->enum_values) ==
          "Default Plate|Supertack Plate|Cool Plate|Engineering Plate|High Temp Plate|Textured PEI Plate|Textured Cool Plate");
    // The SuperSlicer name of a renamed key is not an Orca key.
    CHECK(find_orca_config_key("perimeters") == nullptr);
    CHECK(find_orca_config_key("print_host") != nullptr);
}

TEST_CASE("The exported project settings are read back by the Bambu import",
          "[bambu][config-export]")
{
    // A secondary check on settings with a one to one correspondence: templates, conversions and
    // non reversible values are covered by their own tests.
    DynamicPrintConfig config = make_config({{"perimeters", "3"}, {"fill_density", "15%"}, {"top_solid_layers", "5"}, {"retract_length", "0.8"},
                                             {"temperature", "200"}});
    BambuConfigTranslation translation = translate_config_to_bambu(config);
    BambuConfigTranslation kept;
    for (const char *key : {"wall_loops", "sparse_infill_density", "top_shell_layers", "retraction_length", "nozzle_temperature"})
        kept.entries[key] = translation.entries.at(key);
    DynamicPrintConfig imported = import_json(bambu_project_settings_json(kept, "02.08.01.55"));
    CHECK(imported.opt_serialize("perimeters") == "3");
    CHECK(imported.opt_serialize("fill_density") == "15%");
    CHECK(imported.opt_serialize("top_solid_layers") == "5");
    CHECK(imported.opt_serialize("retract_length") == "0.8");
    CHECK(imported.opt_serialize("temperature") == "200");
}

TEST_CASE("The Bambu firmware creates the printer identity, nozzle type and plate type settings",
          "[bambu][config-export]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ConfigOptionDef *model_id = PrintConfigDef::instance().get("printer_model_id");
    REQUIRE(model_id != nullptr);
    CHECK(model_id->type == coString);
    CHECK(model_id->default_value->serialize().empty());
    CHECK(model_id->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);

    const ConfigOptionDef *nozzle_type = PrintConfigDef::instance().get("nozzle_type");
    REQUIRE(nozzle_type != nullptr);
    CHECK(nozzle_type->type == coEnum);
    CHECK(nozzle_type->default_value->serialize() == "undefine");
    CHECK(nozzle_type->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER);
    REQUIRE(nozzle_type->enum_def.get() != nullptr);
    CHECK(nozzle_type->enum_def->values() == strings({"undefine", "hardened_steel", "stainless_steel", "tungsten_carbide", "brass"}));

    const ConfigOptionDef *bed_type = PrintConfigDef::instance().get("curr_bed_type");
    REQUIRE(bed_type != nullptr);
    CHECK(bed_type->type == coEnum);
    CHECK(bed_type->default_value->serialize() == "Cool Plate");
    CHECK(bed_type->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    REQUIRE(bed_type->enum_def.get() != nullptr);
    CHECK(bed_type->enum_def->values() == strings({"Cool Plate", "Engineering Plate", "High Temp Plate", "Textured PEI Plate",
                                                   "Textured Cool Plate", "Supertack Plate"}));

    // They are read by the Bambu firmware only: the host greys them with the other firmware settings.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    for (const char *firmware : {"bambu", "marlin2"}) {
        config.set_deserialize_strict("gcode_flavor", firmware);
        for (const auto &[key, scope] : {std::pair<const char *, raw_option_preset_type>{"printer_model_id", RAW_PRESET_TYPE_FFF_PRINTER},
                                         {"nozzle_type", RAW_PRESET_TYPE_FFF_PRINTER},
                                         {"curr_bed_type", RAW_PRESET_TYPE_FFF_PRINT}}) {
            INFO(key << " " << firmware);
            bool found = false;
            for (const Orchestrator::GuiRuleApplication &rule : Orchestrator::instance().gui_rules().evaluate(config, scope, -1)) {
                if (rule.target_key == key) {
                    found = true;
                    CHECK(rule.enabled == (std::string(firmware) == "bambu"));
                }
            }
            CHECK(found);
        }
    }
}

TEST_CASE("The custom variables text is read like the placeholder parser",
          "[bambu][config-export]")
{
    const std::map<std::string, std::string> variables = parse_custom_variable_lines(
        "  a = 1 \r\nb=\"two words\"\nc\nd = 3\nd = 4\n = no name\n");
    CHECK(variables.at("a") == "1");
    CHECK(variables.at("b") == "two words");
    CHECK(variables.count("c") == 0);
    CHECK(variables.at("d") == "4");
}

TEST_CASE("The Bambu translation tables are built once and keep their size",
          "[bbconfig]")
{
    const BBConfiguration::Tables &first = BBConfiguration::tables();
    const size_t keys = first.key_translation_map.size();
    const size_t values = first.value_translation_map.size();
    // Two imports in a row (each used to grow a table).
    import_settings({{"wall_loops", "2"}});
    import_settings({{"wall_loops", "2"}});
    const BBConfiguration::Tables &second = BBConfiguration::tables();
    CHECK(&first == &second);
    CHECK(second.key_translation_map.size() == keys);
    CHECK(second.value_translation_map.size() == values);
    CHECK(second.special_gcode_replace.size() == 2);
    CHECK(second.identifier_gcode_replace.size() == 6);
}

TEST_CASE("A template imported from Bambu keeps the words that are not Bambu settings",
          "[bbconfig]")
{
    // "version", "from" and "name" used to be deleted, "top_surface_line_width" was damaged by the
    // rename of "line_width", and "bed_temperature_initial_layer_vector" was never replaced.
    DynamicPrintConfig config = import_settings({
        {"machine_start_gcode", "M104 S{version} ; from name\n; top_surface_line_width line_width\n"
                                "G1 initial_extruder_id {first_layer_temperature[initial_extruder_id]}\n"
                                "M140 S[bed_temperature_initial_layer_single] {bed_temperature_initial_layer_single}\n"
                                "M141 {bed_temperature_initial_layer} {bed_temperature_initial_layer_vector}\n"
                                "M104 S[temperature_initial_layer] {temperature_initial_layer}\n"}});
    CHECK(config.opt_string("start_gcode") ==
          "M104 S{version} ; from name\n; top_infill_extrusion_width extrusion_width\n"
          "G1 initial_extruder {first_layer_temperature[initial_extruder]}\n"
          "M140 S{first_layer_bed_temperature[initial_extruder]} {first_layer_bed_temperature[initial_extruder]}\n"
          "M141 {first_layer_bed_temperature} {\"\"}\n"
          "M104 S{first_layer_temperature[initial_extruder]} {first_layer_temperature[initial_extruder]}\n");
}

TEST_CASE("infill_combination is renamed and converted to a number of layers",
          "[bbconfig]")
{
    CHECK(import_settings({{"infill_combination", "0"}}).opt_serialize("infill_every_layers") == "1");
    CHECK(import_settings({{"infill_combination", "1"}}).opt_serialize("infill_every_layers") == "10");
    // The JSON reader goes through the same renaming and conversion.
    BambuConfigTranslation translation;
    translation.entries["infill_combination"] = BambuConfigEntry{BambuValueKind::Scalar, {"1"}};
    CHECK(import_json(bambu_project_settings_json(translation, "1")).opt_serialize("infill_every_layers") == "10");
}

TEST_CASE("The overhang angle of Bambu is in degrees",
          "[bbconfig]")
{
    // tan(45 degrees) is 1 and tan(30 degrees) is 0.577..., times the 0.2 mm layer height.
    DynamicPrintConfig base = make_config({{"layer_height", "0.2"}});
    CHECK(import_settings({{"make_overhang_printable", "1"}, {"make_overhang_printable_angle", "45"}}, base).opt_serialize("overhangs_max_slope") == "0.2");
    CHECK(import_settings({{"make_overhang_printable", "1"}, {"make_overhang_printable_angle", "30"}}, base).opt_serialize("overhangs_max_slope") == "0.115");
}

TEST_CASE("The firmware variables of a Bambu file are appended to the printer custom variables",
          "[bbconfig]")
{
    DynamicPrintConfig base = make_config({{"printer_custom_variables", "mine = 1"}});
    DynamicPrintConfig config = import_settings({{"scan_first_layer", "1"}, {"enable_power_loss_recovery", "enable"}}, base);
    // The text that was there stays; the variables are added, and read as the firmware reads them.
    CHECK(config.opt_string("printer_custom_variables") == "mine = 1\nenable_power_loss_recovery = enable\nscan_first_layer = 1\n");
    // The same import again adds nothing.
    config = import_settings({{"scan_first_layer", "1"}, {"enable_power_loss_recovery", "enable"}}, config);
    CHECK(config.opt_string("printer_custom_variables") == "mine = 1\nenable_power_loss_recovery = enable\nscan_first_layer = 1\n");
    // An older Orca file holds a bool.
    config = import_settings({{"enable_power_loss_recovery", "0"}}, make_config({}));
    CHECK(config.opt_string("printer_custom_variables") == "enable_power_loss_recovery = disable\n");
}

TEST_CASE("The value table applies to the elements of a Bambu vector",
          "[bbconfig]")
{
    BambuConfigTranslation translation;
    translation.entries["filament_type"] = BambuConfigEntry{BambuValueKind::StringVector, {"TPU", "PLA"}};
    translation.entries["retract_lift_enforce"] = BambuConfigEntry{BambuValueKind::NumericVector, {"Bottom Only", "Top Only"}};
    DynamicPrintConfig config = import_json(bambu_project_settings_json(translation, "1"));
    const ConfigOptionStrings *types = config.option<ConfigOptionStrings>("filament_type");
    REQUIRE(types != nullptr);
    CHECK(types->get_values() == strings({"FLEX", "PLA"}));
    const ConfigOptionStrings *lifts = config.option<ConfigOptionStrings>("retract_lift_top");
    REQUIRE(lifts != nullptr);
    CHECK(lifts->get_values() == strings({"Not on top", "Only on top"}));
}

TEST_CASE("The arc fitting of an Orca file is read for its enumeration and for the older bool",
          "[bbconfig]")
{
    CHECK(import_settings({{"enable_arc_fitting", "enabled"}}).opt_serialize("arc_fitting") == "bambu");
    CHECK(import_settings({{"enable_arc_fitting", "disabled"}}).opt_serialize("arc_fitting") == "disabled");
    CHECK(import_settings({{"enable_arc_fitting", "1"}}).opt_serialize("arc_fitting") == "bambu");
    CHECK(import_settings({{"enable_arc_fitting", "0"}}).opt_serialize("arc_fitting") == "disabled");
    CHECK(import_settings({{"enable_arc_fitting", "sparse_infill"}}).opt_serialize("arc_fitting") == "bambu");
    CHECK(import_settings({{"enable_arc_fitting", "future_mode"}}).opt_serialize("arc_fitting") == "bambu");
}

TEST_CASE("Decimal exports keep the dot under a comma numeric locale", "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"layer_height", "0.2"}, {"nozzle_diameter", "0.4"},
                                             {"bridge_flow_ratio", "95.5%"}, {"brim_width", "0.4"}});
    NumericLocaleRestore restore;
    bool comma_locale = false;
    for (const char *name : {"French_France.1252", "fr_FR.UTF-8", "fr_FR", "de_DE.UTF-8"}) {
        if (std::setlocale(LC_NUMERIC, name) != nullptr && std::string(std::localeconv()->decimal_point) == ",") {
            comma_locale = true;
            break;
        }
    }
    // The case needs an installed comma locale; a minimal system without one has nothing to check here.
    if (!comma_locale) {
        WARN("No comma numeric locale is installed: the locale-independence check is not run.");
        return;
    }
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "layer_height") == strings({"0.2"}));
    CHECK(values_of(translation, "nozzle_diameter") == strings({"0.4"}));
    CHECK(values_of(translation, "bridge_flow") == strings({"0.955"}));
    CHECK(values_of(translation, "brim_type") == strings({"outer_only"}));
}

TEST_CASE("Inner and outer brims share Orca's single width only when representable", "[bambu][config-export]")
{
    const struct BrimCase { const char *outer; const char *inner; const char *ears; const char *type; const char *width; } cases[] = {
        {"0", "0", "0", "no_brim", "0"}, {"5", "0", "0", "outer_only", "5"},
        {"0", "4", "0", "inner_only", "4"}, {"3", "3", "0", "outer_and_inner", "3"},
        {"5", "0", "1", "brim_ears", "5"}
    };
    for (const BrimCase &item : cases) {
        INFO(item.type);
        const BambuConfigTranslation translation = translate_config_to_bambu(make_config({
            {"brim_width", item.outer}, {"brim_width_interior", item.inner}, {"brim_ears", item.ears}}));
        CHECK(values_of(translation, "brim_type") == strings({item.type}));
        CHECK(values_of(translation, "brim_width") == strings({item.width}));
    }
    const BambuConfigTranslation unequal = translate_config_to_bambu(make_config({{"brim_width", "5"}, {"brim_width_interior", "3"}, {"brim_ears", "0"}}));
    CHECK(find_entry(unequal, "brim_type") == nullptr);
    CHECK(find_entry(unequal, "brim_width") == nullptr);
    CHECK(has_warning(unequal, "brim_ears"));
    CHECK(has_warning(unequal, "brim_width"));
}

TEST_CASE("Disabled elements retain their slot only in nullable Orca vectors", "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"extrusion_multiplier", "1,0.95"}, {"temperature", "200,210"},
                                             {"filament_settings_id", "nil;profile"}}, 2);
    CHECK(values_of(translate_config_to_bambu(config), "filament_settings_id") == strings({"nil", "profile"}));
    config.option("extrusion_multiplier")->set_can_be_disabled()->set_enabled(false, 1);
    config.option("temperature")->set_can_be_disabled()->set_enabled(false, 1);
    config.option("filament_settings_id")->set_can_be_disabled()->set_enabled(false, 1);
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(values_of(translation, "filament_flow_ratio") == strings({"1", "nil"}));
    CHECK(find_entry(translation, "nozzle_temperature") == nullptr);
    CHECK(has_warning(translation, "temperature: the value 'nil'"));
    CHECK(find_entry(translation, "filament_settings_id") == nullptr);
    CHECK(has_warning(translation, "filament_settings_id: disabled elements"));
}

TEST_CASE("Project permissions filter both settings and derived printer variables", "[bambu][config-export]")
{
    ProjectFilteredConfig config;
    config.definitions.add("layer_height", coFloat)->nature = ConfigOptionNature::NotProject;
    config.definitions.add("printer_custom_variables", coString)->nature = ConfigOptionNature::NotProject;
    config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    config.set_key_value("printer_custom_variables", new ConfigOptionString("scan_first_layer = 1\nenable_power_loss_recovery = enable"));
    REQUIRE(config.writes_key("layer_height", ConfigWriteTarget::GCode));
    REQUIRE_FALSE(config.writes_key("layer_height", ConfigWriteTarget::Project));
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    const nlohmann::json root = nlohmann::json::parse(bambu_project_settings_json(translation, "1"));
    CHECK_FALSE(root.contains("layer_height"));
    CHECK_FALSE(root.contains("scan_first_layer"));
    CHECK_FALSE(root.contains("enable_power_loss_recovery"));
    const std::vector<std::pair<std::string, std::string>> lines = bambu_config_block(translation);
    const std::map<std::string, std::string> block(lines.begin(), lines.end());
    CHECK(block.at("layer_height") == "0.2");
    CHECK(block.at("scan_first_layer") == "1");
    CHECK(block.at("enable_power_loss_recovery") == "enable");
}

TEST_CASE("Unsupported thumbnail dimensions are omitted without a value warning", "[bambu][config-export]")
{
    DynamicPrintConfig config = make_config({{"thumbnails", "32x32,400x300"}});
    const BambuConfigTranslation translation = translate_config_to_bambu(config);
    CHECK(find_entry(translation, "thumbnails") == nullptr);
    CHECK_FALSE(has_warning(translation, "thumbnails"));
    const BambuConfigTranslation defaults = translate_config_to_bambu(make_config({}));
    CHECK_FALSE(has_warning(defaults, "thumbnails"));
    REQUIRE(config.erase("thumbnails"));
    const BambuConfigTranslation without_thumbnails = translate_config_to_bambu(config);
    // Sum omission counts from borrowed warnings without changing them or taking ownership; value warnings
    // are excluded, and stoul propagates malformed or out-of-range prefixes instead of hiding a broken message.
    const auto unmapped_count = [](const BambuConfigTranslation &result) {
        size_t count = 0;
        for (const std::string &warning : result.warnings)
            if (warning.find(" SuperSlicer settings have no Orca equivalent") != std::string::npos)
                count += std::stoul(warning);
        return count;
    };
    CHECK(unmapped_count(translation) == unmapped_count(without_thumbnails) + 1);
}
