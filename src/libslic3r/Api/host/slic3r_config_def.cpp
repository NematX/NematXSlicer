///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Configuration definition introspection
======================================

The public registration record is also used as a compact read-only snapshot.
Most ConfigOptionDef fields can be copied directly, while strings, arrays and
the serialized default need backing storage. One thread-local snapshot owns
that backing storage until the same thread asks for another definition. C++
and Python convenience wrappers copy it immediately into ordinary values.

Configuration keys are stored in PrintConfigDef's ordered map. Their addresses
remain stable, so enumeration can return borrowed key pointers without building
another temporary collection.
*/

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"

#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace Slic3r {

// The classification word shares the existing slot without moving the remaining reserved storage.
static_assert(offsetof(raw_config_option_def, nature) == offsetof(raw_config_option_def, contains_plugin_id) + sizeof(int32_t),
              "nature must immediately follow contains_plugin_id");
static_assert(sizeof(raw_config_option_def::nature) == sizeof(uint32_t), "nature must remain a 32-bit word");
static_assert(offsetof(raw_config_option_def, nature) + sizeof(raw_config_option_def::nature) == offsetof(raw_config_option_def, reserved_u64),
              "nature must end at the remaining reserved words");
namespace {

struct ConfigOptionDefSnapshot;

static raw_config_option_type raw_option_type(ConfigOptionType type) noexcept;
static raw_container_type raw_container(ConfigOptionContainerType type) noexcept;
static raw_graph_type raw_graph_interpolation(GraphData::GraphType type) noexcept;
static const char *optional_string(const std::string &value) noexcept;
static void rebuild_string_pointers(const std::vector<std::string> &strings,
                                    std::vector<const char *> &pointers);
static void populate_graph_snapshot(ConfigOptionDefSnapshot &snapshot, const GraphSettings &source);
// Copy definition metadata into owned backing storage and bind its raw snapshot pointers.
static void populate_snapshot(ConfigOptionDefSnapshot &snapshot, const ConfigOptionDef &source);

/* Own every allocation referenced by one returned raw_config_option_def. */
struct ConfigOptionDefSnapshot
{
    raw_config_option_def raw = raw_config_option_def_init();
    std::string opt_key;
    std::string gui_flags;
    std::string label;
    std::string full_label;
    std::string tooltip;
    std::string translation_domain;
    std::string sidetext;
    std::string cli;
    std::string ratio_over;
    std::string default_serialized_value;
    std::vector<std::string> aliases;
    std::vector<std::string> shortcuts;
    std::vector<std::string> dependencies;
    std::vector<const char *> alias_pointers;
    std::vector<const char *> shortcut_pointers;
    std::vector<const char *> dependency_pointers;
    std::vector<std::string> enum_values;
    std::vector<std::string> enum_labels;
    std::vector<key_value_string_pair_t> enum_pairs;
    raw_graph_settings graph_settings = {};
    std::string graph_title;
    std::string graph_description;
    std::string graph_y_label;
    std::string graph_x_label;
    std::string graph_null_label;
    std::string graph_label_min_x;
    std::string graph_label_max_x;
    std::string graph_label_min_y;
    std::string graph_label_max_y;
    std::vector<raw_graph_type> graph_allowed_types;
    std::vector<c_vec2d> graph_reset_points;
    std::vector<c_vec2d> graph_enforced_values;
};

thread_local ConfigOptionDefSnapshot config_option_def_snapshot;

static raw_config_option_type raw_option_type(ConfigOptionType type) noexcept
{
    switch (type) {
    case coBool:             return RAW_CO_BOOL;
    case coInt:              return RAW_CO_INT;
    case coFloat:            return RAW_CO_FLOAT;
    case coPercent:          return RAW_CO_PERCENT;
    case coFloatOrPercent:   return RAW_CO_FLOAT_OR_PERCENT;
    case coString:           return RAW_CO_STRING;
    case coPoint:            return RAW_CO_POINT;
    case coPoint3:           return RAW_CO_POINT3;
    case coEnum:             return RAW_CO_ENUM;
    case coGraph:            return RAW_CO_GRAPH;
    case coBools:            return RAW_CO_VECTOR_BOOL;
    case coInts:             return RAW_CO_VECTOR_INT;
    case coFloats:           return RAW_CO_VECTOR_FLOAT;
    case coPercents:         return RAW_CO_VECTOR_PERCENT;
    case coFloatsOrPercents: return RAW_CO_VECTOR_FLOAT_OR_PERCENT;
    case coStrings:          return RAW_CO_VECTOR_STRING;
    case coPoints:           return RAW_CO_VECTOR_POINT;
    case coGraphs:           return RAW_CO_VECTOR_GRAPH;
    case coNone:
    default:                 return RAW_CO_NONE;
    }
}

static raw_container_type raw_container(ConfigOptionContainerType type) noexcept
{
    switch (type) {
    case ConfigOptionContainerType::Project: return RAW_CONTAINER_TYPE_PROJECT;
    case ConfigOptionContainerType::Plater:  return RAW_CONTAINER_TYPE_PLATER;
    case ConfigOptionContainerType::Object:  return RAW_CONTAINER_TYPE_OBJECT;
    case ConfigOptionContainerType::Layer:   return RAW_CONTAINER_TYPE_LAYER;
    case ConfigOptionContainerType::Region:  return RAW_CONTAINER_TYPE_REGION;
    case ConfigOptionContainerType::None:
    default:                                 return RAW_CONTAINER_TYPE_NONE;
    }
}

static raw_graph_type raw_graph_interpolation(GraphData::GraphType type) noexcept
{
    switch (type) {
    case GraphData::GraphType::SQUARE: return RAW_GRAPH_TYPE_SQUARE;
    case GraphData::GraphType::LINEAR: return RAW_GRAPH_TYPE_LINEAR;
    case GraphData::GraphType::SPLINE: return RAW_GRAPH_TYPE_SPLINE;
    case GraphData::GraphType::COUNT:  return RAW_GRAPH_TYPE_NONE;
    }
    return RAW_GRAPH_TYPE_NONE;
}

static const char *optional_string(const std::string &value) noexcept
{
    return value.empty() ? nullptr : value.c_str();
}

static void rebuild_string_pointers(const std::vector<std::string> &strings,
                                    std::vector<const char *> &pointers)
{
    pointers.clear();
    pointers.reserve(strings.size());
    for (const std::string &value : strings)
        pointers.push_back(value.c_str());
}

/* Build the pointer-backed graph subrecord owned by the enclosing snapshot. */
static void populate_graph_snapshot(ConfigOptionDefSnapshot &snapshot, const GraphSettings &source)
{
    snapshot.graph_title = source.title;
    snapshot.graph_description = source.description;
    snapshot.graph_y_label = source.y_label;
    snapshot.graph_x_label = source.x_label;
    snapshot.graph_null_label = source.null_label;
    snapshot.graph_label_min_x = source.label_min_x;
    snapshot.graph_label_max_x = source.label_max_x;
    snapshot.graph_label_min_y = source.label_min_y;
    snapshot.graph_label_max_y = source.label_max_y;

    snapshot.graph_allowed_types.reserve(source.allowed_types.size());
    for (GraphData::GraphType type : source.allowed_types)
        snapshot.graph_allowed_types.push_back(raw_graph_interpolation(type));
    if (source.has_reset_vals) {
        snapshot.graph_reset_points.reserve(source.reset_vals.graph_points.size());
        for (const Vec2d &point : source.reset_vals.graph_points)
            snapshot.graph_reset_points.push_back({point.x(), point.y()});
    }
    snapshot.graph_enforced_values.reserve(source.enforced_values.size());
    for (const Vec2d &point : source.enforced_values)
        snapshot.graph_enforced_values.push_back({point.x(), point.y()});

    raw_graph_settings &out = snapshot.graph_settings;
    out.title = optional_string(snapshot.graph_title);
    out.description = optional_string(snapshot.graph_description);
    out.y_label = optional_string(snapshot.graph_y_label);
    out.x_label = optional_string(snapshot.graph_x_label);
    out.null_label = optional_string(snapshot.graph_null_label);
    out.min_x = source.min_x;
    out.max_x = source.max_x;
    out.step_x = source.step_x;
    out.min_y = source.min_y;
    out.max_y = source.max_y;
    out.step_y = source.step_y;
    out.label_min_x = optional_string(snapshot.graph_label_min_x);
    out.label_max_x = optional_string(snapshot.graph_label_max_x);
    out.label_min_y = optional_string(snapshot.graph_label_min_y);
    out.label_max_y = optional_string(snapshot.graph_label_max_y);
    out.allowed_types = snapshot.graph_allowed_types.data();
    out.allowed_type_count = uint32_t(snapshot.graph_allowed_types.size());
    out.reset_values.begin_index = source.has_reset_vals ? uint32_t(source.reset_vals.begin_idx) : 0u;
    out.reset_values.end_index = source.has_reset_vals ? uint32_t(source.reset_vals.end_idx) : 0u;
    out.reset_values.type = source.has_reset_vals ?
        raw_graph_interpolation(source.reset_vals.type) : RAW_GRAPH_TYPE_NONE;
    out.reset_values.points = snapshot.graph_reset_points.data();
    out.reset_values.point_count = uint32_t(snapshot.graph_reset_points.size());
    out.enforced_values = snapshot.graph_enforced_values.data();
    out.enforced_value_count = uint32_t(snapshot.graph_enforced_values.size());
}

// Replace snapshot-owned backing storage from a borrowed definition and expose normalized raw fields; allocation failures propagate and old snapshot pointers are invalidated.
static void populate_snapshot(ConfigOptionDefSnapshot &snapshot, const ConfigOptionDef &source)
{
    snapshot = ConfigOptionDefSnapshot{};

    snapshot.opt_key = source.opt_key;
    snapshot.gui_flags = source.gui_flags;
    snapshot.label = source.label;
    snapshot.full_label = source.full_label;
    snapshot.tooltip = source.tooltip;
    snapshot.translation_domain = source.translation_domain;
    snapshot.sidetext = source.sidetext;
    snapshot.cli = source.cli;
    snapshot.ratio_over = source.ratio_over;
    if (source.default_value)
        snapshot.default_serialized_value = source.default_value->serialize();

    snapshot.aliases.assign(source.aliases.begin(), source.aliases.end());
    snapshot.shortcuts.assign(source.shortcut.begin(), source.shortcut.end());
    snapshot.dependencies.assign(source.depends_on.begin(), source.depends_on.end());
    rebuild_string_pointers(snapshot.aliases, snapshot.alias_pointers);
    rebuild_string_pointers(snapshot.shortcuts, snapshot.shortcut_pointers);
    rebuild_string_pointers(snapshot.dependencies, snapshot.dependency_pointers);

    if (source.enum_def) {
        snapshot.enum_values = source.enum_def->values();
        if (source.enum_def->has_labels())
            snapshot.enum_labels = source.enum_def->labels();
        const size_t pair_count = std::max(snapshot.enum_values.size(), snapshot.enum_labels.size());
        snapshot.enum_pairs.reserve(pair_count);
        for (size_t index = 0; index < pair_count; ++index) {
            const char *value = index < snapshot.enum_values.size() ? snapshot.enum_values[index].c_str() : nullptr;
            const char *label = index < snapshot.enum_labels.size() ? snapshot.enum_labels[index].c_str() : nullptr;
            snapshot.enum_pairs.push_back({value, label});
        }
    }

    if (source.graph_settings)
        populate_graph_snapshot(snapshot, *source.graph_settings);

    raw_config_option_def &out = snapshot.raw;
    out.opt_key = snapshot.opt_key.c_str();
    out.type = raw_option_type(source.type);
    out.category = static_cast<raw_option_category>(source.category);
    out.level = static_cast<raw_option_level>(source.plugin_api_level);
    out.container_type = raw_container(source.container_type);
    out.option_preset_type = static_cast<raw_option_preset_type>(source.option_preset_type);
    out.gui_type = static_cast<raw_gui_type>(source.gui_type);
    out.printer_technology = static_cast<raw_printer_technology>(source.printer_technology);
    out.invalidates_step = source.invalidates_step;
    out.is_optional = source.is_optional ? 1 : 0;
    out.gui_flags = optional_string(snapshot.gui_flags);
    out.label = optional_string(snapshot.label);
    out.full_label = optional_string(snapshot.full_label);
    out.tooltip = optional_string(snapshot.tooltip);
    out.translation_domain = optional_string(snapshot.translation_domain);
    out.sidetext = optional_string(snapshot.sidetext);
    out.cli = optional_string(snapshot.cli);
    out.ratio_over = optional_string(snapshot.ratio_over);
    out.multiline = source.multiline ? 1 : 0;
    out.full_width = source.full_width ? 1 : 0;
    out.is_code = source.is_code ? 1 : 0;
    out.is_vector_extruder = source.is_vector_extruder ? 1 : 0;
    out.readonly = source.readonly ? 1 : 0;
    out.can_phony = source.can_phony ? 1 : 0;
    out.can_be_disabled = source.can_be_disabled ? 1 : 0;
    out.height = source.height;
    out.width = source.width;
    out.label_width = source.label_width;
    out.aligned_label_left = source.aligned_label_left ? 1 : 0;
    out.sidetext_width = source.sidetext_width;

    out.has_min = source.min != -FLT_MAX;
    out.min_value = source.min;
    out.has_max = source.max != FLT_MAX;
    out.max_value = source.max;
    out.has_max_literal = source.max_literal.value != 0.0 || source.max_literal.percent;
    out.max_literal_value = source.max_literal.value;
    out.max_literal_is_percent = source.max_literal.percent ? 1 : 0;
    out.precision = source.precision;
    out.mode = static_cast<raw_config_option_mode>(source.mode);
    out.flags = source.plugin_api_flags;
    out.aliases = {snapshot.alias_pointers.data(), uint32_t(snapshot.alias_pointers.size())};
    out.shortcut = {snapshot.shortcut_pointers.data(), uint32_t(snapshot.shortcut_pointers.size())};
    out.depends_on = {snapshot.dependency_pointers.data(), uint32_t(snapshot.dependency_pointers.size())};
    out.default_serialized_value = source.default_value ? snapshot.default_serialized_value.c_str() : nullptr;
    out.enum_def.value_label_pairs = {snapshot.enum_pairs.data(), uint32_t(snapshot.enum_pairs.size())};
    out.contains_plugin_id = source.contains_plugin_id ? 1 : 0;
    out.nature = static_cast<uint32_t>(source.nature);
    out.graph_settings = source.graph_settings ? &snapshot.graph_settings : nullptr;
}

} // namespace

void link_config_definition_api() noexcept
{
    // The call site is a link anchor only. The exported C functions below are
    // reached dynamically and otherwise look unused to static-library linkers.
}

} // namespace Slic3r

extern "C" {

uint32_t orchestrator_get_option_keys(const orchestrator_handle *orchestrator,
                                      raw_config_option_type type_filter,
                                      raw_config_option_key *out,
                                      uint32_t capacity)
{
    if (orchestrator == nullptr)
        return 0;

    uint32_t count = 0;
    for (const std::pair<const std::string, Slic3r::ConfigOptionDef> &entry :
         Slic3r::PrintConfigDef::instance().options) {
        const raw_config_option_type type = Slic3r::raw_option_type(entry.second.type);
        if (type_filter != RAW_CO_NONE && type != type_filter)
            continue;
        if (out != nullptr && count < capacity)
            out[count] = {entry.first.c_str(), type};
        ++count;
    }
    return count;
}

int32_t orchestrator_get_option_def(const orchestrator_handle *orchestrator,
                                    const char *key,
                                    raw_config_option_def *out)
{
    if (out == nullptr)
        return 0;
    std::memset(out, 0, sizeof(*out));
    if (orchestrator == nullptr || key == nullptr)
        return 0;

    try {
        const Slic3r::ConfigOptionDef *definition = Slic3r::PrintConfigDef::instance().get(key);
        if (definition == nullptr)
            return 0;
        Slic3r::populate_snapshot(Slic3r::config_option_def_snapshot, *definition);
        *out = Slic3r::config_option_def_snapshot.raw;
        return 1;
    } catch (...) {
        std::memset(out, 0, sizeof(*out));
        return 0;
    }
}

} // extern "C"
