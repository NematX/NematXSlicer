/*
 * REST configuration catalog projection.
 *
 * Finalized PrintConfig definitions are grouped by preset ownership and
 * converted to the JSON metadata consumed by clients before a session exists.
 * The catalog only exposes FFF settings; SLA and other technologies are out of
 * scope for V1. Two limitations are deliberate for now: a vector setting is
 * reported either as an extruder array or as a variable-length array (fixed
 * cardinalities are not emitted yet), and the preference category stays empty
 * until the headless server projects AppConfig definitions.
 */

#include "settings.hpp"

#include <nlohmann/json.hpp>

#include <cfloat>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_types.h"

namespace Slic3r::Rest {

namespace {

using Json = nlohmann::json;

// Wire names match the specification; COUNT is an internal sentinel and keeps
// the linear fallback.
const char *graph_type_name(GraphData::GraphType type)
{
    switch (type) {
    case GraphData::SQUARE:
        return "square";
    case GraphData::LINEAR:
        return "linear";
    case GraphData::SPLINE:
        return "spline";
    case GraphData::COUNT:
        break;
    }
    return "linear";
}

// Config values converted to the JSON primitives used by the catalog: scalars,
// points and graphs.
Json json_value(const std::string &value)
{
    return value;
}

Json json_value(bool value)
{
    return value;
}

Json json_value(unsigned char value)
{
    return value != 0;
}

Json json_value(std::int32_t value)
{
    return value;
}

Json json_value(double value)
{
    return value;
}

Json json_value(const Vec2d &value)
{
    return Json::array({value.x(), value.y()});
}

Json json_value(const Vec3d &value)
{
    return Json::array({value.x(), value.y(), value.z()});
}

Json json_value(const GraphData &value)
{
    Json points = Json::array();
    for (const Vec2d &point : value.data())
        points.push_back(Json::array({point.x(), point.y()}));
    return Json{{"type", graph_type_name(value.type)}, {"points", std::move(points)}};
}

// Vector elements converted one by one, in index order.
template<typename T>
Json vector_json_value(const ConfigOptionVector<T> &option)
{
    Json values = Json::array();
    for (const T &value : option.get_values())
        values.push_back(json_value(value));
    return values;
}

// An extruder array default carries a single common element with scalar
// indicators (specification 5.2); fixed and variable arrays carry the full
// value.
template<typename T>
Json default_vector_value(const ConfigOptionVector<T> &option, bool extruder_array)
{
    if (!extruder_array)
        return vector_json_value(option);
    // A definition without a default element keeps a null default instead of
    // reading past the vector.
    return option.get_values().empty() ? Json() : json_value(option.get_at(0));
}

// The vector flag is stored in the type value; the catalog exposes the element
// type.
ConfigOptionType scalar_type(ConfigOptionType type)
{
    if ((static_cast<int>(type) & static_cast<int>(coVectorType)) == 0)
        return type;
    return static_cast<ConfigOptionType>(static_cast<int>(type) - static_cast<int>(coVectorType));
}

const char *setting_type_name(ConfigOptionType type)
{
    switch (scalar_type(type)) {
    case coBool:
        return "bool";
    case coInt:
        return "int";
    case coFloat:
        return "float";
    case coPercent:
        return "percent";
    case coFloatOrPercent:
        return "float_or_percent";
    case coString:
        return "string";
    case coEnum:
        return "enum";
    case coPoint:
        return "point2";
    case coPoint3:
        return "point3";
    case coGraph:
        return "graph";
    default:
        // Unknown or not-yet-mapped types fall back to their serialized string.
        return "string";
    }
}

// Build the `default` value object: `value`, plus `percent` for float-or-percent
// types and `enabled` when the setting can be disabled. A scalar carries scalar
// indicators; a non-extruder array carries parallel arrays; an extruder array
// carries one common element.
Json default_value(const ConfigOptionDef &definition)
{
    if (!definition.default_value)
        return nullptr;

    const ConfigOption *option = definition.default_value.get();
    Json result = Json::object();
    switch (definition.type) {
    case coBool:
        result["value"] = static_cast<const ConfigOptionBool *>(option)->value;
        break;
    case coBools:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionBools *>(option), definition.is_vector_extruder);
        break;
    case coInt:
        result["value"] = static_cast<const ConfigOptionInt *>(option)->value;
        break;
    case coInts:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionInts *>(option), definition.is_vector_extruder);
        break;
    case coFloat:
        result["value"] = static_cast<const ConfigOptionFloat *>(option)->value;
        break;
    case coFloats:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionFloats *>(option), definition.is_vector_extruder);
        break;
    case coPercent:
        result["value"] = static_cast<const ConfigOptionPercent *>(option)->value;
        break;
    case coPercents:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionPercents *>(option), definition.is_vector_extruder);
        break;
    case coFloatOrPercent: {
        const ConfigOptionFloatOrPercent *value =
            static_cast<const ConfigOptionFloatOrPercent *>(option);
        result["value"] = value->value;
        result["percent"] = value->percent;
        break;
    }
    case coFloatsOrPercents: {
        const ConfigOptionFloatsOrPercents *value =
            static_cast<const ConfigOptionFloatsOrPercents *>(option);
        if (definition.is_vector_extruder) {
            if (value->get_values().empty()) {
                result["value"] = nullptr;
                result["percent"] = nullptr;
            } else {
                const FloatOrPercent &item = value->get_at(0);
                result["value"] = item.value;
                result["percent"] = item.percent;
            }
        } else {
            result["value"] = Json::array();
            result["percent"] = Json::array();
            for (const FloatOrPercent &item : value->get_values()) {
                result["value"].push_back(item.value);
                result["percent"].push_back(item.percent);
            }
        }
        break;
    }
    case coString:
        result["value"] = static_cast<const ConfigOptionString *>(option)->value;
        break;
    case coStrings:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionStrings *>(option), definition.is_vector_extruder);
        break;
    case coPoint:
        result["value"] = json_value(static_cast<const ConfigOptionPoint *>(option)->value);
        break;
    case coPoints:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionPoints *>(option), definition.is_vector_extruder);
        break;
    case coPoint3:
        result["value"] = json_value(static_cast<const ConfigOptionPoint3 *>(option)->value);
        break;
    case coGraph:
        result["value"] = json_value(static_cast<const ConfigOptionGraph *>(option)->value);
        break;
    case coGraphs:
        result["value"] = default_vector_value(
            *static_cast<const ConfigOptionGraphs *>(option), definition.is_vector_extruder);
        break;
    case coEnum:
        result["value"] = option->serialize();
        break;
    default:
        result["value"] = option->serialize();
        break;
    }
    if (definition.can_be_disabled) {
        if (definition.is_vector_extruder) {
            result["enabled"] = option->is_enabled(0);
        } else if (option->is_vector()) {
            Json enabled = Json::array();
            for (std::size_t index = 0; index < option->size(); ++index)
                enabled.push_back(option->is_enabled(static_cast<int32_t>(index)));
            result["enabled"] = std::move(enabled);
        } else {
            result["enabled"] = option->is_enabled();
        }
    }
    return result;
}

// One catalog entry. `precision` and `min`/`max` are emitted only when they
// differ from the definition defaults (6 and ±FLT_MAX). The full label is used
// because catalog clients have no GUI context. Vector settings are reported as
// an extruder array or a variable-length array; fixed cardinalities are not
// emitted yet.
Json setting_definition(const ConfigOptionDef &definition)
{
    Json result{
        {"id", definition.opt_key},
        {"type", setting_type_name(definition.type)},
        {"label", definition.get_full_label()},
        {"default", default_value(definition)},
        {"read_only", definition.readonly},
        {"can_be_disabled", definition.can_be_disabled}
    };
    if (!definition.sidetext.empty())
        result["unit"] = definition.sidetext;
    if (!definition.tooltip.empty())
        result["description"] = definition.tooltip;
    if (definition.precision != 6)
        result["precision"] = definition.precision;
    if (definition.min != -FLT_MAX)
        result["min"] = definition.min;
    if (definition.max != FLT_MAX)
        result["max"] = definition.max;
    if ((static_cast<int>(definition.type) & static_cast<int>(coVectorType)) != 0)
        result["array"] = definition.is_vector_extruder ? "extruder" : "variable";
    if (definition.enum_def && definition.enum_def->has_values()) {
        result["choices"] = Json::array();
        const std::vector<std::string> &labels = definition.enum_def->labels();
        const std::vector<std::string> &values = definition.enum_def->values();
        for (std::size_t index = 0; index < values.size(); ++index) {
            result["choices"].push_back({
                {"value", values[index]},
                {"label", index < labels.size() ? labels[index] : values[index]}
            });
        }
    }
    return result;
}

using CategoryKeys = std::map<std::string, std::set<std::string>>;

// The four FFF preset sets define print, filament, extruder and printer
// ownership. Preference definitions come from AppConfig, which the headless
// server does not load yet, so the category stays empty.
CategoryKeys category_keys(const PrintConfigDef &definition)
{
    CategoryKeys categories;
    categories["preference"] = {};
    categories["print"] = definition.option_keys(RAW_PRESET_TYPE_FFF_PRINT);
    categories["filament"] = definition.option_keys(RAW_PRESET_TYPE_FFF_FILAMENT);
    categories["extruder"] = definition.option_keys(RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER);
    categories["printer"] = definition.option_keys(RAW_PRESET_TYPE_FFF_PRINTER);
    return categories;
}

} // namespace

bool settings_catalog_ready() noexcept
{
    return PrintConfigDef::instance().is_finalized();
}

// Build the catalog from the finalized definitions. The route answers 503
// before calling this; the throw is the last-resort guard.
nlohmann::json settings_catalog()
{
    const PrintConfigDef &definition = PrintConfigDef::instance();
    if (!definition.is_finalized())
        throw std::logic_error("REST settings catalog requires finalized configuration definitions");

    nlohmann::json result = nlohmann::json::object();
    const CategoryKeys categories = category_keys(definition);
    for (const std::string &category_name : {"preference", "print", "filament", "extruder", "printer"}) {
        result[category_name] = nlohmann::json::array();
        const std::set<std::string> &keys = categories.find(category_name)->second;
        for (const std::string &key : keys) {
            const ConfigOptionDef *option = definition.get(key);
            if (option != nullptr)
                result[category_name].push_back(setting_definition(*option));
        }
    }
    return result;
}

} // namespace Slic3r::Rest
