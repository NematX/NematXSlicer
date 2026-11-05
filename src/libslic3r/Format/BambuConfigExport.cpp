///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BambuConfigExport.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <system_error>

#include <nlohmann/json.hpp>
#include <fast_float/fast_float.h>

#include "BBConfig.hpp"
#include "CustomVariables.hpp"
#include "OrcaConfigKeys.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/LocalesUtils.hpp"

/*
Bambu configuration export
==========================

The public interface is described in BambuConfigExport.hpp. This file holds the rules.

How a SuperSlicer setting becomes an Orca setting (translate_config_to_bambu, for every key in order):

    1. The key is refused when its nature does not allow the G-code destination (this is what keeps
       `print_host` and the preset-only settings out), or when the option is disabled.
    2. A conversion rule (k_key_rules) may exist for the key. It names the Orca key, and may convert the
       value (it can read other settings) or repeat it for every extruder. One SuperSlicer key may feed
       several Orca keys: `bed_temperature` fills the six plate temperatures.
       Without a rule, the Orca key comes from bambu_export_key(): the import key table read backwards
       (BBConfig.cpp), the explicit renames of k_extra_renames, or the key itself when Orca defines it.
       No Orca key: the setting has no equivalent and is only counted.
    3. Each element is converted: by the rule, else by the small table of renamed values
       (k_value_rules), else kept as it is.
    4. store_entry() checks the elements against the Orca type of the key (OrcaConfigKeys.cpp): a bool
       is 0 or 1, an integer has no fraction, a percent is a number followed by %, an enumeration value is
       one of Orca's names... A value that fails omits the whole setting and adds a warning.
        Per-filament scalar values are repeated for the actual number of extruders; wipe-tower positions
        remain one-element per-plate vectors.
    5. The two printer variables `scan_first_layer` and `enable_power_loss_recovery`, kept by the Bambu
       firmware in `printer_custom_variables`, are written under their own names.

Conversion rules, one per case that the import (BBConfig.cpp, PrintConfigDef legacy conversions) does:

    Exact inverse written:
      ironing_type + ironing           "no ironing" when ironing is off
      brim_ears + brim_width + brim_width_interior: brim_type and one effective width;
                                       unequal active inner and outer widths cannot be represented

Operation notes:
- remaining_times(+_type)          disable_m73 (inverted), only for the m73 method
- arc_fitting                      enable_arc_fitting disabled|enabled (bambu method only)
- first_layer_size_compensation    elefant_foot_compensation, sign inverted for nonpositive values only
- bridge_flow_ratio, fill_top_flow_ratio, first_layer_flow_ratio, print_extrusion_multiplier
- a percentage becomes the ratio Orca stores (95% :  0.95)
- bridge_type                      thick_bridges (nozzle :  1, flow :  0)
- infill_every_layers              infill_combination (1 :  0, 10 :  1), via the value table
- retract_lift_top, complete_objects, machine_limits_usage, gcode_label_objects, gcode_flavor,
- seam_position, thumbnails_format, filament_type, the infill patterns
- renamed values, via the value table
- ensure_vertical_shell_thickness  disabled :  none; enabled_old :  ensure_critical_only (2 perimeters),
- ensure_moderate (4) or ensure_all (0); other thresholds are omitted
- Omitted (the SuperSlicer setting has no Orca key, so it is counted as "no equivalent"):
- the `*_extrusion_spacing` settings, resolution_internal, bridge_overlap, avoid_crossing_not_first_layer,
- small_perimeter_max_length, support_material_bottom_interface_pattern, overhangs_max_slope and its
- bridge settings (Orca's make_overhang_printable has no SuperSlicer slope), remaining_times_type.

Representative direct calls (branches are not a timeline):

    translate_config_to_bambu()
        |-- translate_option()
        `-- translate_custom_variables()

    translate_option()
        |-- bambu_export_key()
        |-- option_texts()
        |-- convert_text()
        |-- find_orca_config_key()
        `-- store_entry()

    store_entry()
        |-- find_orca_config_key()
        `-- validate_element()
*/

namespace Slic3r {

namespace {

// The conversion of one element of a SuperSlicer value; it may read the other settings of the
// configuration. Returning nothing means that Orca has no exact form for the value.
typedef std::optional<std::string> (*ValueConversion)(const ConfigBase &config, const std::string &text);

// A SuperSlicer setting that needs more than a rename: its own Orca key and/or a conversion.
struct KeyRule
{
    const char     *ss_key;
    const char     *orca_key;     // nullptr: the key found by bambu_export_key()
    ValueConversion convert;      // nullptr: no conversion
    bool            per_extruder; // a scalar written once for every extruder
};

// Write the translation of every writable setting of the configuration.
void translate_option(const ConfigBase &config, const std::string &ss_key, const ConfigOption &option,
                      BambuConfigTranslation &translation, size_t &unmapped_count);

// Write the Orca printer variables that the Bambu firmware reads from printer_custom_variables.
void translate_custom_variables(const ConfigBase &config, BambuConfigTranslation &translation);

// Return owned element texts, preserving disabled vector slots as Orca's `nil` marker.
std::vector<std::string> option_texts(const ConfigOption &option);

// Convert one element: by the rule, else by the table of renamed values, else unchanged.
std::optional<std::string> convert_text(const ConfigBase &config, const std::string &ss_key, const KeyRule *rule,
                                        const std::string &text);

// Check the elements against the Orca type of the key, then store them; add a warning when they do not fit.
void store_entry(BambuConfigTranslation &translation, const std::string &warning_prefix, const std::string &orca_key,
                 std::vector<std::string> texts, bool project_allowed = true);

// Return the text of an element in the Orca type, or nothing when the type cannot hold it.
std::optional<std::string> validate_element(const OrcaConfigKey &orca, const std::string &text);

// Return whether the text is a number (digits, optional sign, fraction and exponent; no percent sign).
bool is_number(const std::string &text);

// Return the serialization of another setting of the configuration, empty when it does not exist.
std::string serialized_setting(const ConfigBase &config, const std::string &key);

// Conversions used by k_key_rules, see the header of this file.
std::optional<std::string> convert_ironing_type(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_brim_ears(const ConfigBase &config, const std::string &text);
// Return the single brim width Orca can represent, or nothing for unequal inner and outer widths.
std::optional<std::string> convert_brim_width(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_ensure_vertical_shell_thickness(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_remaining_times(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_arc_fitting(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_negate(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_percent_to_ratio(const ConfigBase &config, const std::string &text);
std::optional<std::string> convert_bridge_type(const ConfigBase &config, const std::string &text);

// Return the SuperSlicer key -> Orca key table (the import table read backwards), built once.
const std::map<std::string, std::string> &import_table_inverse();

// Return the Orca keys that the import table maps to another SuperSlicer key, built once: a SuperSlicer
// setting of that name is another setting than Orca's, so it cannot keep its name.
const std::set<std::string> &keys_renamed_by_import();

// Escape a scalar for the CONFIG_BLOCK like Orca's escape_string_cstyle: one line, no quotes.
std::string escape_scalar(const std::string &text);

// Escape one element of a string vector for the CONFIG_BLOCK like Orca's escape_strings_cstyle.
std::string escape_string_element(const std::string &text, bool only_element);

// SuperSlicer settings created by plugins, or renamed since the import table, that Orca names differently.
const std::pair<const char *, const char *> k_extra_renames[] = {
    {"platter_fan", "auxiliary_fan"},
    {"platter_fan_speed", "additional_cooling_fan_speed"},
};

// The six plates of Orca, in the order of its temperature keys.
// Every plate receives the temperature of SuperSlicer's single bed temperature.
const KeyRule k_key_rules[] = {
    {"bed_temperature", "supertack_plate_temp", nullptr, false},
    {"bed_temperature", "cool_plate_temp", nullptr, false},
    {"bed_temperature", "textured_cool_plate_temp", nullptr, false},
    {"bed_temperature", "eng_plate_temp", nullptr, false},
    {"bed_temperature", "hot_plate_temp", nullptr, false},
    {"bed_temperature", "textured_plate_temp", nullptr, false},
    {"first_layer_bed_temperature", "supertack_plate_temp_initial_layer", nullptr, false},
    {"first_layer_bed_temperature", "cool_plate_temp_initial_layer", nullptr, false},
    {"first_layer_bed_temperature", "textured_cool_plate_temp_initial_layer", nullptr, false},
    {"first_layer_bed_temperature", "eng_plate_temp_initial_layer", nullptr, false},
    {"first_layer_bed_temperature", "hot_plate_temp_initial_layer", nullptr, false},
    {"first_layer_bed_temperature", "textured_plate_temp_initial_layer", nullptr, false},
    {"nozzle_type", "nozzle_type", nullptr, true},
    {"ironing_type", nullptr, &convert_ironing_type, false},
    {"brim_ears", "brim_type", &convert_brim_ears, false},
    {"brim_width", "brim_width", &convert_brim_width, false},
    {"brim_ears", "brim_ears", nullptr, false},
    {"ensure_vertical_shell_thickness", nullptr, &convert_ensure_vertical_shell_thickness, false},
    {"remaining_times", "disable_m73", &convert_remaining_times, false},
    {"arc_fitting", "enable_arc_fitting", &convert_arc_fitting, false},
    {"first_layer_size_compensation", "elefant_foot_compensation", &convert_negate, false},
    {"bridge_flow_ratio", nullptr, &convert_percent_to_ratio, false},
    {"fill_top_flow_ratio", nullptr, &convert_percent_to_ratio, false},
    // Approximation: Orca's bottom-solid flow is broader than SuperSlicer's first-layer-only flow.
    {"first_layer_flow_ratio", nullptr, &convert_percent_to_ratio, false},
    {"print_extrusion_multiplier", "print_flow_ratio", &convert_percent_to_ratio, false},
    {"bridge_type", "thick_bridges", &convert_bridge_type, false},
};

// SuperSlicer values that Orca names differently. Every Orca value is one of the names in Orca's enumeration
// (or a bool 0/1), which the tests check against the generated list of Orca keys.
const std::vector<BambuExportValueRule> k_value_rules = {
    // "stars" is the SuperSlicer name of the pattern that Orca calls "tri-hexagon".
    {"fill_pattern", "stars", "tri-hexagon"},
    {"top_fill_pattern", "stars", "tri-hexagon"},
    {"bottom_fill_pattern", "stars", "tri-hexagon"},
    {"solid_fill_pattern", "stars", "tri-hexagon"},
    {"seam_position", "rear", "back"},
    {"filament_type", "FLEX", "TPU"},
    {"gcode_flavor", "sprinter", "reprap"},
    {"thumbnails_format", "BIQU", "BTT_TFT"},
    {"retract_lift_top", "All surfaces", "All Surfaces"},
    {"retract_lift_top", "Not on top", "Bottom Only"},
    {"retract_lift_top", "Only on top", "Top Only"},
    {"complete_objects", "0", "by layer"},
    {"complete_objects", "1", "by object"},
    {"machine_limits_usage", "time_estimate_only", "0"},
    {"machine_limits_usage", "emit_to_gcode", "1"},
    {"gcode_label_objects", "disabled", "0"},
    {"gcode_label_objects", "octoprint", "1"},
    {"infill_every_layers", "1", "0"},
    {"infill_every_layers", "10", "1"},
};

} // namespace

// Read non-owning configuration options allowed in G-code into owned entries and warnings; unsupported
// values are omitted, allocation failures propagate, and each entry retains its Project permission.
BambuConfigTranslation translate_config_to_bambu(const ConfigBase &config)
{
    BambuConfigTranslation translation;
    size_t                 unmapped_count = 0;
    for (const std::string &key : config.keys()) {
        // The destination filter is what keeps connection settings (print_host...) out of the files.
        if (!config.writes_key(key, ConfigWriteTarget::GCode))
            continue;
        const ConfigOption *option = config.optptr(key);
        if (option != nullptr && option->is_enabled())
            translate_option(config, key, *option, translation, unmapped_count);
    }
    translate_custom_variables(config, translation);
    if (unmapped_count > 0)
        translation.warnings.push_back(std::to_string(unmapped_count) + " SuperSlicer settings have no Orca equivalent and were not written.");
    return translation;
}

namespace {

// Read the enabled option without taking ownership, apply key/value rules and filament repetition, and
// append owned entries or omission warnings; missing keys are counted and allocation errors propagate.
void translate_option(const ConfigBase &config, const std::string &ss_key, const ConfigOption &option,
                      BambuConfigTranslation &translation, size_t &unmapped_count)
{
    // The rules of this key, or one generic rule when it has none.
    std::vector<const KeyRule *> rules;
    for (const KeyRule &rule : k_key_rules)
        if (ss_key == rule.ss_key)
            rules.push_back(&rule);
    const std::optional<std::string> generic_key = bambu_export_key(ss_key);
    if (rules.empty()) {
        if (!generic_key)
            ++unmapped_count;
        else
            rules.push_back(nullptr);
    }
    if (rules.empty())
        return;

    const std::vector<std::string> source = option_texts(option);
    for (const KeyRule *rule : rules) {
        const std::string orca_key = rule != nullptr && rule->orca_key != nullptr ? std::string(rule->orca_key) :
                                     generic_key ? *generic_key : std::string();
        if (orca_key.empty())
            continue;
        std::vector<std::string> texts;
        bool                     convertible = true;
        for (const std::string &text : source) {
            const std::optional<std::string> converted = convert_text(config, ss_key, rule, text);
            if (!converted) {
                translation.warnings.push_back(ss_key + ": the value '" + text + "' has no exact Orca equivalent, " + orca_key + " was not written.");
                convertible = false;
                break;
            }
            texts.push_back(*converted);
        }
        if (!convertible)
            continue;
        const OrcaConfigKey *orca = find_orca_config_key(orca_key);
        // A literal string "nil" is ordinary text; reject disabled string slots using their enabled flags.
        if (orca != nullptr && !orca->nullable && option.type() == coStrings) {
            const ConfigOptionVectorBase &vector = static_cast<const ConfigOptionVectorBase &>(option);
            for (size_t idx = 0; idx < vector.size(); ++idx)
                if (!vector.is_enabled(int32_t(idx)))
                    convertible = false;
            if (!convertible) {
                translation.warnings.push_back(ss_key + ": disabled elements are not accepted by Orca's " + orca_key);
                continue;
            }
        }
        if ((rule != nullptr && rule->per_extruder) ||
            (option.is_scalar() && orca != nullptr && orca_option_is_vector(orca->type) &&
             orca_key != "wipe_tower_x" && orca_key != "wipe_tower_y")) {
            // Wipe-tower positions are per plate; the other scalar-to-vector settings are per filament.
            const ConfigOption *nozzles = config.optptr("nozzle_diameter");
            texts.resize(nozzles != nullptr ? std::max<size_t>(nozzles->size(), 1) : 1, texts.empty() ? std::string() : texts.front());
        }
        store_entry(translation, ss_key + ": ", orca_key, std::move(texts), config.writes_key(ss_key, ConfigWriteTarget::Project));
    }
}

// Copy raw strings or serialized numeric values from a borrowed option; disabled vector elements are
// retained as `nil` for later nullable validation, and allocations may fail without changing the option.
std::vector<std::string> option_texts(const ConfigOption &option)
{
    std::vector<std::string> texts;
    if (option.is_scalar()) {
        // A string is written raw: its own serialization would escape the line breaks of a template.
        texts.push_back(option.type() == coString ? static_cast<const ConfigOptionString &>(option).value : option.serialize());
        return texts;
    }
    const ConfigOptionVectorBase &vector = static_cast<const ConfigOptionVectorBase &>(option);
    for (size_t idx = 0; idx < vector.size(); ++idx) {
        if (!vector.is_enabled(int32_t(idx))) {
            texts.emplace_back("nil");
            continue;
        }
        texts.push_back(option.type() == coStrings ? static_cast<const ConfigOptionStrings &>(option).get_at(idx) :
                                                     vector.serialize_at(int(idx)));
    }
    return texts;
}

std::optional<std::string> convert_text(const ConfigBase &config, const std::string &ss_key, const KeyRule *rule,
                                        const std::string &text)
{
    if (rule != nullptr && rule->convert != nullptr)
        return rule->convert(config, text);
    for (const BambuExportValueRule &value_rule : k_value_rules)
        if (ss_key == value_rule.ss_key && text == value_rule.ss_value)
            return std::string(value_rule.orca_value);
    return text;
}

// Validate owned text elements against Orca's type and nullable flag, then move them into an entry with
// its Project permission; invalid values append a warning, unknown keys are ignored, allocations may fail.
void store_entry(BambuConfigTranslation &translation, const std::string &warning_prefix, const std::string &orca_key,
                 std::vector<std::string> texts, bool project_allowed)
{
    const OrcaConfigKey *orca = find_orca_config_key(orca_key);
    if (orca == nullptr)
        return;
    const bool orca_vector = orca_option_is_vector(orca->type);
    if (!orca_vector && texts.size() != 1) {
        translation.warnings.push_back(warning_prefix + orca_key + " holds one value in Orca and the setting has " +
                                       std::to_string(texts.size()) + ", it was not written.");
        return;
    }
    for (std::string &text : texts) {
        const std::optional<std::string> valid = validate_element(*orca, text);
        if (!valid) {
            translation.warnings.push_back(warning_prefix + "the value '" + text + "' is not accepted by Orca's " + orca_key + ", it was not written.");
            return;
        }
        text = *valid;
    }
    BambuConfigEntry entry;
    entry.kind = !orca_vector ? BambuValueKind::Scalar :
                 (orca->type == OrcaOptionType::Strings ? BambuValueKind::StringVector : BambuValueKind::NumericVector);
    entry.values = std::move(texts);
    entry.project_allowed = project_allowed;
    translation.entries[orca_key] = std::move(entry);
}

// Check borrowed text against Orca's definition and return an owned accepted spelling: nullable numeric
// `nil` markers and literal strings are distinct. Invalid values return nothing; allocations may fail.
std::optional<std::string> validate_element(const OrcaConfigKey &orca, const std::string &text)
{
    if (text == "nil" && orca.type != OrcaOptionType::String && orca.type != OrcaOptionType::Strings)
        return orca.nullable ? std::optional<std::string>(text) : std::nullopt;
    switch (orca.type) {
    case OrcaOptionType::Bool:
    case OrcaOptionType::Bools:
        return text == "0" || text == "1" ? std::optional<std::string>(text) : std::nullopt;
    case OrcaOptionType::Int:
    case OrcaOptionType::Ints: {
        int         value = 0;
        const char *end   = text.data() + text.size();
        const std::from_chars_result result = std::from_chars(text.data(), end, value);
        return !text.empty() && result.ec == std::errc() && result.ptr == end ? std::optional<std::string>(text) : std::nullopt;
    }
    case OrcaOptionType::Float:
    case OrcaOptionType::Floats:
        return is_number(text) ? std::optional<std::string>(text) : std::nullopt;
    case OrcaOptionType::Percent:
    case OrcaOptionType::Percents:
    case OrcaOptionType::FloatOrPercent:
    case OrcaOptionType::FloatsOrPercents: {
        const bool percent = !text.empty() && text.back() == '%';
        if (!is_number(percent ? text.substr(0, text.size() - 1) : text))
            return std::nullopt;
        // A percent type always shows its sign; a float-or-percent keeps the form it has.
        const bool needs_sign = orca.type == OrcaOptionType::Percent || orca.type == OrcaOptionType::Percents;
        return percent || !needs_sign ? text : text + "%";
    }
    case OrcaOptionType::String:
    case OrcaOptionType::Strings:
        return text;
    case OrcaOptionType::Enum:
    case OrcaOptionType::Enums: {
        const std::string names = std::string("|") + orca.enum_values + "|";
        return names.find("|" + text + "|") != std::string::npos ? std::optional<std::string>(text) : std::nullopt;
    }
    case OrcaOptionType::Point:
    case OrcaOptionType::Points: {
        // "XxY": two numbers.
        const size_t separator = text.find('x', 1);
        return separator != std::string::npos && is_number(text.substr(0, separator)) && is_number(text.substr(separator + 1)) ?
                   std::optional<std::string>(text) : std::nullopt;
    }
    case OrcaOptionType::PointsGroups:
        return std::nullopt;
    }
    return std::nullopt;
}

// Parse the complete borrowed decimal text with fast_float, independently of the C locale and without
// allocating; only the supported numeric syntax succeeds, malformed or out-of-range input returns false.
bool is_number(const std::string &text)
{
    if (text.empty() || !(std::isdigit(static_cast<unsigned char>(text.front())) || text.front() == '-' || text.front() == '.'))
        return false;
    double      value = 0.;
    const char *end   = text.data() + text.size();
    const fast_float::from_chars_result result = fast_float::from_chars(text.data(), end, value);
    return result.ec == std::errc() && result.ptr == end;
}

std::string serialized_setting(const ConfigBase &config, const std::string &key)
{
    const ConfigOption *option = config.optptr(key);
    return option != nullptr ? option->serialize() : std::string();
}

std::optional<std::string> convert_ironing_type(const ConfigBase &config, const std::string &text)
{
    return serialized_setting(config, "ironing") == "0" ? std::string("no ironing") : text;
}

// Read borrowed inner/outer brim widths with decimal-point parsing; return an owned Orca brim type,
// prioritizing ears, or nothing when unequal nonzero widths cannot share Orca's single width.
std::optional<std::string> convert_brim_ears(const ConfigBase &config, const std::string &text)
{
    if (text == "1")
        return std::string("brim_ears");
    const std::string outer_text = serialized_setting(config, "brim_width");
    const std::string inner_text = serialized_setting(config, "brim_width_interior");
    const double outer = outer_text.empty() ? 0. : string_to_double_decimal_point(outer_text);
    const double inner = inner_text.empty() ? 0. : string_to_double_decimal_point(inner_text);
    if (outer > 0. && inner > 0.)
        return outer == inner ? std::optional<std::string>("outer_and_inner") : std::nullopt;
    return std::string(inner > 0. ? "inner_only" : outer > 0. ? "outer_only" : "no_brim");
}

// Read borrowed serialized widths and return the effective single Orca width; unequal active inner and
// outer widths return nothing, ears use the outer width, and invalid numeric configurations may throw.
std::optional<std::string> convert_brim_width(const ConfigBase &config, const std::string &text)
{
    const std::string inner_text = serialized_setting(config, "brim_width_interior");
    const double outer = text.empty() ? 0. : string_to_double_decimal_point(text);
    const double inner = inner_text.empty() ? 0. : string_to_double_decimal_point(inner_text);
    if (serialized_setting(config, "brim_ears") == "1")
        return text;
    if (outer > 0. && inner > 0. && outer != inner)
        return std::nullopt;
    return outer > 0. || inner <= 0. ? text : inner_text;
}

// Map borrowed shell mode and threshold settings to an owned exact Orca mode; unsupported thresholds
// return nothing instead of widening the shell, and allocation failures propagate without config changes.
std::optional<std::string> convert_ensure_vertical_shell_thickness(const ConfigBase &config, const std::string &text)
{
    if (text == "disabled")
        return std::string("none");
    if (text != "enabled_old")
        return std::nullopt;
    const std::string perimeters = serialized_setting(config, "solid_over_perimeters");
    return perimeters == "2" ? std::string("ensure_critical_only") :
           perimeters == "4" ? std::optional<std::string>("ensure_moderate") :
           perimeters == "0" ? std::optional<std::string>("ensure_all") : std::nullopt;
}

std::optional<std::string> convert_remaining_times(const ConfigBase &config, const std::string &text)
{
    // No remaining times is "disable M73" whatever the method; with remaining times, only the plain M73 method is Orca's.
    if (text == "0")
        return std::string("1");
    return serialized_setting(config, "remaining_times_type") == "m73" ? std::optional<std::string>("0") : std::nullopt;
}

std::optional<std::string> convert_arc_fitting(const ConfigBase &, const std::string &text)
{
    if (text == "disabled")
        return std::string("disabled");
    return text == "bambu" ? std::optional<std::string>("enabled") : std::nullopt;
}

// Invert a borrowed nonpositive compensation without locale-sensitive parsing; positive or malformed
// values have no exact import inverse and return nothing, while accepted strings are copied into the result.
std::optional<std::string> convert_negate(const ConfigBase &, const std::string &text)
{
    if (!is_number(text))
        return std::nullopt;
    if (text.front() == '-')
        return text.substr(1);
    return string_to_double_decimal_point(text) == 0. ? std::optional<std::string>(text) : std::nullopt;
}

// Parse a borrowed percent with decimal-point helpers and format an owned ratio in the classic locale;
// malformed values return nothing, and stream/allocation failures propagate without modifying the input.
std::optional<std::string> convert_percent_to_ratio(const ConfigBase &, const std::string &text)
{
    if (text.empty() || text.back() != '%' || !is_number(text.substr(0, text.size() - 1)))
        return std::nullopt;
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(10) << string_to_double_decimal_point(text.substr(0, text.size() - 1)) / 100.;
    return stream.str();
}

std::optional<std::string> convert_bridge_type(const ConfigBase &, const std::string &text)
{
    if (text == "nozzle")
        return std::string("1");
    return text == "flow" ? std::optional<std::string>("0") : std::nullopt;
}

// Read the borrowed custom-variable string only when G-code permits it, copying recognized firmware
// variables with the source's Project permission; invalid values warn and allocation failures propagate.
void translate_custom_variables(const ConfigBase &config, BambuConfigTranslation &translation)
{
    const ConfigOption *option = config.optptr("printer_custom_variables");
    if (option == nullptr || option->type() != coString || !option->is_enabled() ||
        !config.writes_key("printer_custom_variables", ConfigWriteTarget::GCode))
        return;
    const std::map<std::string, std::string> variables =
        parse_custom_variable_lines(static_cast<const ConfigOptionString &>(*option).value);
    // The firmware reads "1" or "true" (any case) as a scan request and anything else as none.
    const auto scan = variables.find("scan_first_layer");
    if (scan != variables.end()) {
        std::string text = scan->second;
        std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) { return char(std::tolower(c)); });
        store_entry(translation, "printer_custom_variables: ", "scan_first_layer", {text == "1" || text == "true" ? "1" : "0"},
                    config.writes_key("printer_custom_variables", ConfigWriteTarget::Project));
    }
    const auto recovery = variables.find("enable_power_loss_recovery");
    if (recovery != variables.end())
        store_entry(translation, "printer_custom_variables: ", "enable_power_loss_recovery", {recovery->second},
                    config.writes_key("printer_custom_variables", ConfigWriteTarget::Project));
}

const std::map<std::string, std::string> &import_table_inverse()
{
    static const std::map<std::string, std::string> inverse = [] {
        std::map<std::string, std::string> table;
        for (const auto &[bambu_key, ss_key] : BBConfiguration::tables().key_translation_map)
            if (!ss_key.empty() && ss_key != bambu_key)
                table.emplace(ss_key, bambu_key);
        return table;
    }();
    return inverse;
}

const std::set<std::string> &keys_renamed_by_import()
{
    static const std::set<std::string> renamed = [] {
        std::set<std::string> keys;
        for (const auto &[bambu_key, ss_key] : BBConfiguration::tables().key_translation_map)
            if (ss_key != bambu_key)
                keys.insert(bambu_key);
        return keys;
    }();
    return renamed;
}

std::string escape_scalar(const std::string &text)
{
    std::string escaped;
    escaped.reserve(text.size());
    for (const char c : text) {
        if (c == '\r')
            escaped += "\\r";
        else if (c == '\n')
            escaped += "\\n";
        else if (c == '\\')
            escaped += "\\\\";
        else if (c == '"')
            escaped += "\\\"";
        else
            escaped += c;
    }
    return escaped;
}

std::string escape_string_element(const std::string &text, const bool only_element)
{
    const bool quoted = (only_element && text.empty()) || text.find_first_of(" \t;\\\"\r\n") != std::string::npos;
    if (!quoted)
        return text;
    std::string escaped = "\"";
    for (const char c : text) {
        if (c == '\\')
            escaped += "\\\\";
        else if (c == '"')
            escaped += "\\\"";
        else if (c == '\r')
            escaped += "\\r";
        else if (c == '\n')
            escaped += "\\n";
        else
            escaped += c;
    }
    return escaped + "\"";
}

} // namespace

// Copy borrowed Project-allowed entries into a sorted JSON object with caller-supplied version headers;
// invalid UTF-8 bytes are replaced, returned storage is owned, and allocation failures propagate.
std::string bambu_project_settings_json(const BambuConfigTranslation &translation, const std::string &version)
{
    // nlohmann::json keeps its object keys sorted, which is the order Orca writes.
    nlohmann::json root = nlohmann::json::object();
    for (const auto &[key, entry] : translation.entries) {
        if (!entry.project_allowed)
            continue;
        if (entry.kind == BambuValueKind::Scalar) {
            root[key] = entry.values.empty() ? std::string() : entry.values.front();
        } else {
            nlohmann::json array = nlohmann::json::array();
            for (const std::string &value : entry.values)
                array.push_back(value);
            root[key] = std::move(array);
        }
    }
    root["version"] = version;
    root["name"]    = "project_settings";
    root["from"]    = "project";
    // One tab per level and UTF-8 as it is, like Orca; an invalid byte is replaced instead of throwing.
    return root.dump(1, '\t', false, nlohmann::json::error_handler_t::replace);
}

std::vector<std::pair<std::string, std::string>> bambu_config_block(const BambuConfigTranslation &translation)
{
    std::vector<std::pair<std::string, std::string>> lines;
    lines.reserve(translation.entries.size());
    for (const auto &[key, entry] : translation.entries) {
        std::string value;
        if (entry.kind == BambuValueKind::Scalar) {
            value = entry.values.empty() ? std::string() : escape_scalar(entry.values.front());
        } else {
            const char *separator = entry.kind == BambuValueKind::StringVector ? ";" : ",";
            for (size_t idx = 0; idx < entry.values.size(); ++idx) {
                if (idx > 0)
                    value += separator;
                value += entry.kind == BambuValueKind::StringVector ? escape_string_element(entry.values[idx], entry.values.size() == 1) :
                                                                       entry.values[idx];
            }
        }
        lines.emplace_back(key, std::move(value));
    }
    return lines;
}

const std::vector<BambuExportValueRule> &bambu_export_value_rules()
{
    return k_value_rules;
}

// Resolve a borrowed SuperSlicer name through the inverse key table and Orca key list, returning an owned
// name or nothing for unsupported keys (including thumbnails); table construction/allocation may throw.
std::optional<std::string> bambu_export_key(const std::string &ss_key)
{
    // No verified Orca string encoding is available here for SuperSlicer's point-vector thumbnails.
    if (ss_key == "thumbnails")
        return std::nullopt;
    // The import renames the Orca key into the SuperSlicer one; read it backwards when Orca still has that name.
    // Orca's own name for the filament colors (the import table points its default_filament_colour at it).
    if (ss_key == "filament_colour")
        return ss_key;
    const std::map<std::string, std::string> &inverse = import_table_inverse();
    if (const auto renamed = inverse.find(ss_key); renamed != inverse.end() && find_orca_config_key(renamed->second) != nullptr)
        return renamed->second;
    for (const auto &[from, to] : k_extra_renames)
        if (ss_key == from)
            return std::string(to);
    // Same name: only when Orca defines it and the import does not give that Orca name to another setting.
    if (find_orca_config_key(ss_key) != nullptr && keys_renamed_by_import().count(ss_key) == 0)
        return ss_key;
    return std::nullopt;
}

} // namespace Slic3r
