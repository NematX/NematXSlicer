/*
 * REST setting value codec.
 *
 * Converts stored session values into typed JSON and back into the engine's
 * serialized representation, enforcing the catalog type, indicator, cardinality
 * and range rules before any candidate configuration is committed.
 */

#include "settings.hpp"
#include "settings_internal.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_types.h"
#include "libslic3r/Api/plugin/c/slic3r_slicing_step.h"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace Slic3r::Rest {

namespace {

using Json = nlohmann::json;
using detail::json_value;
using detail::scalar_type;

// Search the finalized FFF ownership sets; preference definitions are not part
// of PrintConfigDef and therefore cannot be resolved here yet.
const ConfigOptionDef *find_setting_definition(const std::string &setting_id)
{
    const PrintConfigDef &definition = PrintConfigDef::instance();
    const std::set<std::string> &print_keys = definition.option_keys(RAW_PRESET_TYPE_FFF_PRINT);
    if (print_keys.find(setting_id) != print_keys.end())
        return definition.get(setting_id);
    const std::set<std::string> &filament_keys = definition.option_keys(RAW_PRESET_TYPE_FFF_FILAMENT);
    if (filament_keys.find(setting_id) != filament_keys.end())
        return definition.get(setting_id);
    const std::set<std::string> &extruder_keys = definition.option_keys(RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER);
    if (extruder_keys.find(setting_id) != extruder_keys.end())
        return definition.get(setting_id);
    const std::set<std::string> &printer_keys = definition.option_keys(RAW_PRESET_TYPE_FFF_PRINTER);
    if (printer_keys.find(setting_id) != printer_keys.end())
        return definition.get(setting_id);
    return nullptr;
}

// Reject unavailable definitions before any route reads or mutates a session.
const ConfigOptionDef &require_setting_definition(const std::string &setting_id)
{
    if (!settings_catalog_ready())
        throw std::logic_error("REST settings require finalized configuration definitions");
    const ConfigOptionDef *definition = find_setting_definition(setting_id);
    if (definition == nullptr)
        throw std::invalid_argument("The setting is unknown or inapplicable: " + setting_id);
    return *definition;
}

// Extruder-sized settings follow the nozzle vector in the session config.
std::size_t extruder_count(const DynamicPrintConfig &configuration)
{
    const ConfigOption *nozzle_diameter = configuration.option("nozzle_diameter");
    if (nozzle_diameter != nullptr && nozzle_diameter->is_vector() && nozzle_diameter->size() > 0)
        return nozzle_diameter->size();
    return 1;
}

// Fixed vectors use their stored size; extruder vectors use the selected printer.
std::size_t setting_value_count(
    const ConfigOptionDef &definition,
    const ConfigOption &option,
    const DynamicPrintConfig &configuration)
{
    if (!definition.is_vector_extruder)
        return option.size();
    return extruder_count(configuration);
}

// Read the session value or materialize the engine default without changing the session.
const ConfigOption &setting_option(
    const DynamicPrintConfig &configuration,
    const ConfigOptionDef &definition,
    std::unique_ptr<ConfigOption> &default_option)
{
    const ConfigOption *option = configuration.option(definition.opt_key);
    if (option != nullptr)
        return *option;
    default_option.reset(definition.create_default_option());
    if (default_option == nullptr)
        throw std::invalid_argument("The setting has no usable value: " + definition.opt_key);
    return *default_option;
}

// Convert one stored scalar element into its typed wire representation.
Json scalar_value_json(
    const ConfigOptionDef &definition,
    const ConfigOption &option,
    std::size_t index)
{
    switch (definition.type) {
    case coBool:
        return static_cast<const ConfigOptionBool &>(option).value;
    case coBools:
        return static_cast<const ConfigOptionBools &>(option).get_at(index) != 0;
    case coInt:
        return static_cast<const ConfigOptionInt &>(option).value;
    case coInts:
        return static_cast<const ConfigOptionInts &>(option).get_at(index);
    case coFloat:
        return static_cast<const ConfigOptionFloat &>(option).value;
    case coFloats:
        return static_cast<const ConfigOptionFloats &>(option).get_at(index);
    case coPercent:
        return static_cast<const ConfigOptionPercent &>(option).value;
    case coPercents:
        return static_cast<const ConfigOptionPercents &>(option).get_at(index);
    case coFloatOrPercent:
        return static_cast<const ConfigOptionFloatOrPercent &>(option).value;
    case coFloatsOrPercents:
        return static_cast<const ConfigOptionFloatsOrPercents &>(option).get_at(index).value;
    case coString:
        return static_cast<const ConfigOptionString &>(option).value;
    case coStrings:
        return static_cast<const ConfigOptionStrings &>(option).get_at(index);
    case coPoint:
        return json_value(static_cast<const ConfigOptionPoint &>(option).value);
    case coPoints:
        return json_value(static_cast<const ConfigOptionPoints &>(option).get_at(index));
    case coPoint3:
        return json_value(static_cast<const ConfigOptionPoint3 &>(option).value);
    case coGraph:
        return json_value(static_cast<const ConfigOptionGraph &>(option).value);
    case coGraphs:
        return json_value(static_cast<const ConfigOptionGraphs &>(option).get_at(index));
    case coEnum: {
        if (!definition.enum_def)
            throw std::invalid_argument("The enum has no value mapping: " + definition.opt_key);
        const std::optional<std::reference_wrapper<const std::string>> value =
            definition.enum_def->enum_to_value(option.get_int());
        if (!value.has_value())
            throw std::invalid_argument("The enum value is not defined: " + definition.opt_key);
        return value.value().get();
    }
    default:
        throw std::invalid_argument("The setting type is not supported: " + definition.opt_key);
    }
}

// Preserve the percent indicator independently from the numeric value.
bool scalar_percent(const ConfigOptionDef &definition, const ConfigOption &option, std::size_t index)
{
    if (definition.type == coFloatOrPercent)
        return static_cast<const ConfigOptionFloatOrPercent &>(option).percent;
    if (definition.type == coFloatsOrPercents)
        return static_cast<const ConfigOptionFloatsOrPercents &>(option).get_at(index).percent;
    return false;
}

// Build scalar, vector, and parallel indicator fields from one session option.
Json setting_value_json_impl(
    const ConfigOptionDef &definition,
    const ConfigOption &option,
    const DynamicPrintConfig &configuration)
{
    const bool vector = (static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0;
    const std::size_t count = vector ? setting_value_count(definition, option, configuration) : 1;
    Json result = Json::object();
    if (vector) {
        Json values = Json::array();
        Json enabled = Json::array();
        Json percents = Json::array();
        for (std::size_t index = 0; index < count; ++index) {
            values.push_back(scalar_value_json(definition, option, index));
            if (definition.can_be_disabled)
                enabled.push_back(option.is_enabled(static_cast<int32_t>(index)));
            if (definition.type == coFloatsOrPercents)
                percents.push_back(scalar_percent(definition, option, index));
        }
        result["value"] = std::move(values);
        if (definition.can_be_disabled)
            result["enabled"] = std::move(enabled);
        if (definition.type == coFloatsOrPercents)
            result["percent"] = std::move(percents);
    } else {
        result["value"] = scalar_value_json(definition, option, 0);
        if (definition.can_be_disabled)
            result["enabled"] = option.is_enabled();
        if (definition.type == coFloatOrPercent)
            result["percent"] = scalar_percent(definition, option, 0);
    }
    return result;
}

// Serialize a JSON number only after confirming that it is numeric.
std::string json_number(const Json &value, const std::string &field)
{
    if (!value.is_number())
        throw std::invalid_argument(field + " must be a number");
    return value.dump();
}

// Keep integer conversion within the 32-bit configuration option range.
std::string json_integer(const Json &value, const std::string &field)
{
    if (value.is_number_integer()) {
        const std::int64_t integer_value = value.get<std::int64_t>();
        if (integer_value < std::numeric_limits<std::int32_t>::min() ||
            integer_value > std::numeric_limits<std::int32_t>::max())
            throw std::invalid_argument(field + " is outside the integer range");
        return std::to_string(integer_value);
    }
    if (value.is_number_unsigned()) {
        const std::uint64_t integer_value = value.get<std::uint64_t>();
        if (integer_value > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
            throw std::invalid_argument(field + " is outside the integer range");
        return std::to_string(integer_value);
    }
    throw std::invalid_argument(field + " must be an integer");
}

// Apply the catalog bounds before the configuration parser sees a candidate.
void validate_number_range(const ConfigOptionDef &definition, const Json &value, const std::string &field)
{
    if (!value.is_number())
        return;
    const double number = value.get<double>();
    if (number < definition.min || number > definition.max)
        throw std::invalid_argument(field + " is outside the allowed range");
}

// Convert a point array to the separators accepted by ConfigOptionPoint types.
std::string point_value(const Json &value, std::size_t dimensions, const std::string &field)
{
    if (!value.is_array() || value.size() != dimensions)
        throw std::invalid_argument(field + " must contain exactly " + std::to_string(dimensions) + " numbers");
    std::string result;
    for (std::size_t index = 0; index < dimensions; ++index) {
        if (index > 0)
            result += dimensions == 2 ? "x" : ",";
        result += json_number(value.at(index), field);
    }
    return result;
}

// Graph JSON exposes usable points and type; the engine backing range is not in the V1 contract.
std::string graph_value(const Json &value, const std::string &field)
{
    if (!value.is_object() || !value.contains("type") || !value.at("type").is_string() ||
        !value.contains("points") || !value.at("points").is_array())
        throw std::invalid_argument(field + " must contain type and points");
    const std::string type = value.at("type").get<std::string>();
    std::uint8_t type_id = 0;
    if (type == "linear")
        type_id = GraphData::LINEAR;
    else if (type == "square")
        type_id = GraphData::SQUARE;
    else if (type == "spline")
        type_id = GraphData::SPLINE;
    else
        throw std::invalid_argument(field + ".type is invalid");

    const Json &points = value.at("points");
    // GraphData serializes begin/end indices, but the REST value exposes only data(),
    // so submitted points intentionally become the complete usable range.
    std::string result = "0:" + std::to_string(points.size()) + ":" + std::to_string(type_id);
    for (std::size_t index = 0; index < points.size(); ++index) {
        const Json &point = points.at(index);
        if (!point.is_array() || point.size() != 2)
            throw std::invalid_argument(field + ".points must contain pairs");
        result += ":" + json_number(point.at(0), field) + "x" + json_number(point.at(1), field);
    }
    return result;
}

// Convert one typed value and its indicators to the engine's serialized form.
std::string scalar_value_serialized(
    const ConfigOptionDef &definition,
    const Json &value,
    bool enabled,
    bool percent,
    const std::string &field)
{
    std::string result;
    if (!enabled)
        result = scalar_type(definition.type) == coString ? "!:" : "!";
    switch (scalar_type(definition.type)) {
    case coBool:
        if (!value.is_boolean())
            throw std::invalid_argument(field + " must be a boolean");
        result += value.get<bool>() ? "1" : "0";
        break;
    case coInt:
        result += json_integer(value, field);
        break;
    case coFloat:
    case coPercent:
    case coFloatOrPercent:
        validate_number_range(definition, value, field);
        result += json_number(value, field);
        if (scalar_type(definition.type) == coPercent ||
            (scalar_type(definition.type) == coFloatOrPercent && percent))
            result += "%";
        break;
    case coString:
        if (!value.is_string())
            throw std::invalid_argument(field + " must be a string");
        result += escape_string_cstyle(value.get<std::string>());
        break;
    case coEnum:
        if (!value.is_string() || !definition.has_enum_value(value.get<std::string>()))
            throw std::invalid_argument(field + " is not an allowed enum value");
        result += value.get<std::string>();
        break;
    case coPoint:
        result += point_value(value, 2, field);
        break;
    case coPoint3:
        result += point_value(value, 3, field);
        break;
    case coGraph:
        result += graph_value(value, field);
        break;
    default:
        throw std::invalid_argument(field + " has an unsupported type");
    }
    return result;
}

// Validate scalar or parallel enabled/percent indicators against the setting definition.
std::vector<bool> indicator_values(
    const ConfigOptionDef &definition,
    const Json &entry,
    const char *name,
    std::size_t count,
    bool required)
{
    const std::string field(name);
    if (!definition.can_be_disabled && field == "enabled") {
        if (entry.contains(field))
            throw std::invalid_argument(field + " is not applicable to this setting");
        return std::vector<bool>(count, true);
    }
    if (required && !entry.contains(field))
        throw std::invalid_argument(field + " is required for this setting");
    if (!entry.contains(field))
        return std::vector<bool>(count, false);

    const Json &value = entry.at(field);
    std::vector<bool> result;
    if (count == 1 && !((static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0)) {
        if (!value.is_boolean())
            throw std::invalid_argument(field + " must be a boolean");
        result.push_back(value.get<bool>());
        return result;
    }
    if (!value.is_array() || value.size() != count)
        throw std::invalid_argument(field + " must match the setting value cardinality");
    for (const Json &item : value) {
        if (!item.is_boolean())
            throw std::invalid_argument(field + " must contain booleans");
        result.push_back(item.get<bool>());
    }
    return result;
}

// Validate the typed object, cardinality, indicators, and element values as one candidate.
std::string typed_value_serialized(
    const ConfigOptionDef &definition,
    const Json &entry,
    const DynamicPrintConfig &configuration)
{
    if (!entry.is_object() || !entry.contains("value"))
        throw std::invalid_argument("A setting value must be an object containing value");
    for (Json::const_iterator field = entry.begin(); field != entry.end(); ++field) {
        if (field.key() != "value" && field.key() != "enabled" && field.key() != "percent")
            throw std::invalid_argument("Unknown field in setting value: " + field.key());
    }

    const bool vector = (static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0;
    const ConfigOptionType type = scalar_type(definition.type);
    const Json &values = entry.at("value");
    std::size_t count = 1;
    if (vector) {
        if (!values.is_array())
            throw std::invalid_argument("value must be an array");
        count = values.size();
        if (definition.is_vector_extruder && count != extruder_count(configuration))
            throw std::invalid_argument("The setting value count does not match the extruders");
    } else if (values.is_array() && type != coPoint && type != coPoint3) {
        throw std::invalid_argument("value must be scalar");
    }

    const bool float_or_percent = scalar_type(definition.type) == coFloatOrPercent;
    if (!float_or_percent && entry.contains("percent"))
        throw std::invalid_argument("percent is not applicable to this setting");
    std::vector<bool> enabled = indicator_values(
        definition, entry, "enabled", count, definition.can_be_disabled);
    std::vector<bool> percent = float_or_percent
        ? indicator_values(definition, entry, "percent", count, true)
        : std::vector<bool>(count, false);

    if (vector && scalar_type(definition.type) == coString) {
        std::vector<std::string> strings;
        strings.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            if (!values.at(index).is_string())
                throw std::invalid_argument("value must contain strings");
            strings.push_back(values.at(index).get<std::string>());
        }
        return escape_strings_cstyle(strings, enabled);
    }

    std::string result;
    for (std::size_t index = 0; index < count; ++index) {
        if (index > 0)
            result += ",";
        const Json &value = vector ? values.at(index) : values;
        const std::string value_field = vector
            ? "value[" + std::to_string(index) + "]"
            : "value";
        result += scalar_value_serialized(
            definition,
            value,
            enabled[index],
            percent[index],
            value_field);
    }
    return result;
}

// Ensure the stored option shape still matches its finalized definition.
void validate_setting_option(
    const ConfigOptionDef &definition,
    const ConfigOption &option,
    const DynamicPrintConfig &configuration,
    const std::optional<std::size_t> expected_extruder_count = std::nullopt)
{
    const bool vector = (static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0;
    if (vector != option.is_vector())
        throw std::invalid_argument("The setting has an incompatible stored value: " + definition.opt_key);
    const std::size_t expected_count = expected_extruder_count.value_or(extruder_count(configuration));
    if (definition.is_vector_extruder && option.size() != expected_count)
        throw std::invalid_argument("The setting value count does not match the extruders");
}

// Recheck numeric bounds after the engine parser has materialized the candidate.
void validate_setting_range(
    const ConfigOptionDef &definition,
    const ConfigOption &option,
    const DynamicPrintConfig &configuration)
{
    const ConfigOptionType type = scalar_type(definition.type);
    if (type != coInt && type != coFloat && type != coPercent && type != coFloatOrPercent)
        return;
    const Json value = setting_value_json_impl(definition, option, configuration).at("value");
    if (value.is_array()) {
        for (const Json &item : value)
            validate_number_range(definition, item, definition.opt_key);
    } else {
        validate_number_range(definition, value, definition.opt_key);
    }
}

// Detect disabled markers at scalar or vector item boundaries.
bool starts_disabled_item(const std::string &serialized_value, char separator)
{
    if (!serialized_value.empty() && serialized_value.front() == '!')
        return true;
    for (std::size_t separator_index = serialized_value.find(separator);
         separator_index != std::string::npos;
         separator_index = serialized_value.find(separator, separator_index + 1)) {
        if (separator_index + 1 < serialized_value.size() && serialized_value[separator_index + 1] == '!')
            return true;
    }
    return false;
}

// Non-disableable settings must not reach parsers that assert on a leading marker.
void validate_serialized_indicators(const ConfigOptionDef &definition, const std::string &serialized_value)
{
    if (definition.can_be_disabled)
        return;
    const bool vector = (static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0;
    const ConfigOptionType type = scalar_type(definition.type);
    if (type == coString) {
        if (vector ? starts_disabled_item(serialized_value, ';') : serialized_value.rfind("!:", 0) == 0)
            throw std::invalid_argument("The setting cannot be disabled: " + definition.opt_key);
    } else if (starts_disabled_item(serialized_value, vector ? ',' : '\0')) {
        throw std::invalid_argument("The setting cannot be disabled: " + definition.opt_key);
    }
}

} // namespace

// Only an explicit STEP_NONE marks a setting whose change does not invalidate
// the slicing result; host definitions default to STEP_ANY.
bool setting_affects_slicing(const std::string &setting_id)
{
    const ConfigOptionDef &definition = require_setting_definition(setting_id);
    return definition.invalidates_step != STEP_NONE;
}

// The engine validator reports only the first cross-setting error. A session
// may already hold an invalid combination (for example from a loaded profile or
// project), so a mutation is refused only when it produces a new first error;
// unrelated edits on an already invalid configuration stay possible.
void validate_setting_configuration(
    const DynamicPrintConfig &previous,
    const DynamicPrintConfig &candidate)
{
    DynamicPrintConfig candidate_copy(candidate);
    const std::string error = candidate_copy.validate();
    if (error.empty())
        return;
    DynamicPrintConfig previous_copy(previous);
    if (previous_copy.validate() == error)
        return;
    throw std::invalid_argument("The setting combination is invalid: " + error);
}

nlohmann::json setting_value_json(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id)
{
    const ConfigOptionDef &definition = require_setting_definition(setting_id);
    std::unique_ptr<ConfigOption> default_option;
    const ConfigOption &option = setting_option(configuration, definition, default_option);
    validate_setting_option(definition, option, configuration);
    return setting_value_json_impl(definition, option, configuration);
}

std::string setting_value_serialized(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id)
{
    const ConfigOptionDef &definition = require_setting_definition(setting_id);
    std::unique_ptr<ConfigOption> default_option;
    const ConfigOption &option = setting_option(configuration, definition, default_option);
    validate_setting_option(definition, option, configuration);
    return option.serialize();
}

std::string setting_json_to_serialized(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id,
    const nlohmann::json &value)
{
    const ConfigOptionDef &definition = require_setting_definition(setting_id);
    if (definition.readonly)
        throw std::invalid_argument("The setting is read-only: " + setting_id);
    return typed_value_serialized(definition, value, configuration);
}

void set_setting_serialized(
    DynamicPrintConfig &configuration,
    const std::string &setting_id,
    const std::string &serialized_value)
{
    const ConfigOptionDef &definition = require_setting_definition(setting_id);
    if (definition.readonly)
        throw std::invalid_argument("The setting is read-only: " + setting_id);
    // A nozzle update must not change the cardinality used to validate itself.
    const std::size_t expected_extruder_count = extruder_count(configuration);
    validate_serialized_indicators(definition, serialized_value);
    try {
        configuration.set_deserialize_strict(setting_id, serialized_value);
    } catch (const std::exception &error) {
        throw std::invalid_argument("Invalid value for " + setting_id + ": " + error.what());
    }
    const ConfigOption *option = configuration.option(setting_id);
    if (option == nullptr)
        throw std::invalid_argument("The setting could not be stored: " + setting_id);
    validate_setting_option(definition, *option, configuration, expected_extruder_count);
    validate_setting_range(definition, *option, configuration);
}

const ConfigOptionDef &require_entity_setting_definition(const std::string &key)
{
    if (!settings_catalog_ready())
        throw std::logic_error("REST entity settings require finalized configuration definitions");
    const ConfigOptionDef *definition = PrintConfigDef::instance().get(key);
    if (definition == nullptr)
        throw std::invalid_argument("The setting is unknown: " + key);
    return *definition;
}

nlohmann::json entity_setting_value_json(
    const ConfigOption &option,
    const ConfigOptionDef &definition,
    const DynamicPrintConfig &session)
{
    validate_setting_option(definition, option, session, extruder_count(session));
    validate_setting_range(definition, option, session);
    return setting_value_json_impl(definition, option, session);
}

std::string entity_setting_json_to_serialized(
    const ConfigOptionDef &definition,
    const nlohmann::json &value,
    const DynamicPrintConfig &session)
{
    if (definition.readonly)
        throw std::invalid_argument("The setting is read-only: " + definition.opt_key);
    return typed_value_serialized(definition, value, session);
}

void set_entity_setting_serialized(
    DynamicPrintConfig &target,
    const ConfigOptionDef &definition,
    const std::string &serialized,
    const DynamicPrintConfig &session)
{
    if (definition.readonly)
        throw std::invalid_argument("The setting is read-only: " + definition.opt_key);
    const std::size_t expected_extruder_count = extruder_count(session);
    validate_serialized_indicators(definition, serialized);
    try {
        target.set_deserialize_strict(definition.opt_key, serialized);
    } catch (const std::exception &error) {
        throw std::invalid_argument("Invalid value for " + definition.opt_key + ": " + error.what());
    }
    const ConfigOption *option = target.option(definition.opt_key);
    if (option == nullptr)
        throw std::invalid_argument("The setting could not be stored: " + definition.opt_key);
    validate_setting_option(definition, *option, session, expected_extruder_count);
    validate_setting_range(definition, *option, session);
}

} // namespace Slic3r::Rest
