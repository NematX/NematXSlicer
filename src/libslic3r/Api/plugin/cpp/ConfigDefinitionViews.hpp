///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_ConfigDefinitionViews_hpp_
#define slic3r_Api_plugin_cpp_ConfigDefinitionViews_hpp_

/*
Configuration definition values
===============================

The C API returns raw_config_option_def as a short-lived thread-local snapshot.
These value types copy every pointer-backed field immediately, allowing plugin
code to retain, copy and compare definitions without depending on host memory.
They are read-only descriptions; changing a member never changes PrintConfigDef.
*/

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"

namespace slic3r_api {

struct ConfigOptionKey
{
    std::string key;
    raw_config_option_type type = RAW_CO_NONE;
};

struct ConfigOptionEnumEntry
{
    std::optional<std::string> value;
    std::optional<std::string> label;
};

struct ConfigGraphData
{
    uint32_t begin_index = 0;
    uint32_t end_index = 0;
    raw_graph_type type = RAW_GRAPH_TYPE_NONE;
    std::vector<ConfigPoint> points;
};

/* An owning copy of the graph editor metadata attached to one definition. */
struct ConfigGraphSettings
{
    explicit ConfigGraphSettings(const raw_graph_settings &raw);

    std::optional<std::string> title;
    std::optional<std::string> description;
    std::optional<std::string> y_label;
    std::optional<std::string> x_label;
    std::optional<std::string> null_label;
    double min_x = 0.0;
    double max_x = 0.0;
    double step_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double step_y = 0.0;
    std::optional<std::string> label_min_x;
    std::optional<std::string> label_max_x;
    std::optional<std::string> label_min_y;
    std::optional<std::string> label_max_y;
    std::vector<raw_graph_type> allowed_types;
    ConfigGraphData reset_values;
    std::vector<ConfigPoint> enforced_values;
};

/* An owning C++ copy of definition metadata, excluding contains_plugin_id. */
struct ConfigOptionDefinition
{
    // Copy a borrowed raw definition into independently owned metadata.
    explicit ConfigOptionDefinition(const raw_config_option_def &raw);

    std::string opt_key;
    raw_config_option_type type = RAW_CO_NONE;
    raw_option_category category = RAW_OPTION_CATEGORY_NONE;
    raw_option_level level = RAW_OPTION_LEVEL_BASIC;
    raw_container_type container_type = RAW_CONTAINER_TYPE_NONE;
    raw_option_preset_type option_preset_type = RAW_PRESET_TYPE_NONE;
    raw_gui_type gui_type = RAW_GUI_TYPE_UNDEFINED;
    raw_printer_technology printer_technology = RAW_PT_NONE;
    slicing_step_t invalidates_step = STEP_NONE;
    bool is_optional = false;
    std::optional<std::string> gui_flags;
    std::optional<std::string> label;
    std::optional<std::string> full_label;
    std::optional<std::string> tooltip;
    std::optional<std::string> translation_domain;
    std::optional<std::string> sidetext;
    std::optional<std::string> cli;
    std::optional<std::string> ratio_over;
    bool multiline = false;
    bool full_width = false;
    bool is_code = false;
    bool is_vector_extruder = false;
    bool readonly = false;
    bool can_phony = false;
    bool can_be_disabled = false;
    int32_t height = -1;
    int32_t width = -1;
    int32_t label_width = -1;
    bool aligned_label_left = false;
    int32_t sidetext_width = -1;
    std::optional<double> min_value;
    std::optional<double> max_value;
    std::optional<double> max_literal_value;
    bool max_literal_is_percent = false;
    int32_t precision = 0;
    raw_config_option_mode mode = RAW_CONFIG_OPTION_MODE_NONE;
    RawConfigOptionFlags flags = 0;
    uint32_t nature = RAW_CONFIG_NATURE_NORMAL;
    std::vector<std::string> aliases;
    std::vector<std::string> shortcut;
    std::vector<std::string> depends_on;
    std::optional<std::string> default_serialized_value;
    std::vector<ConfigOptionEnumEntry> enum_entries;
    std::optional<ConfigGraphSettings> graph_settings;
};

inline ConfigGraphSettings::ConfigGraphSettings(const raw_graph_settings &raw) :
    min_x(raw.min_x),
    max_x(raw.max_x),
    step_x(raw.step_x),
    min_y(raw.min_y),
    max_y(raw.max_y),
    step_y(raw.step_y)
{
    const std::pair<const char *, std::optional<std::string> *> optional_strings[] = {
        {raw.title, &title},
        {raw.description, &description},
        {raw.y_label, &y_label},
        {raw.x_label, &x_label},
        {raw.null_label, &null_label},
        {raw.label_min_x, &label_min_x},
        {raw.label_max_x, &label_max_x},
        {raw.label_min_y, &label_min_y},
        {raw.label_max_y, &label_max_y},
    };
    for (const std::pair<const char *, std::optional<std::string> *> &entry : optional_strings)
        if (entry.first != nullptr)
            *entry.second = std::string(entry.first);

    if (raw.allowed_types != nullptr)
        allowed_types.assign(raw.allowed_types, raw.allowed_types + raw.allowed_type_count);
    reset_values.begin_index = raw.reset_values.begin_index;
    reset_values.end_index = raw.reset_values.end_index;
    reset_values.type = raw.reset_values.type;
    if (raw.reset_values.points != nullptr) {
        reset_values.points.reserve(raw.reset_values.point_count);
        for (uint32_t idx = 0; idx < raw.reset_values.point_count; ++idx)
            reset_values.points.push_back({raw.reset_values.points[idx].x, raw.reset_values.points[idx].y});
    }
    if (raw.enforced_values != nullptr) {
        enforced_values.reserve(raw.enforced_value_count);
        for (uint32_t idx = 0; idx < raw.enforced_value_count; ++idx)
            enforced_values.push_back({raw.enforced_values[idx].x, raw.enforced_values[idx].y});
    }
}

// Copy a borrowed raw definition into independently owned metadata.
inline ConfigOptionDefinition::ConfigOptionDefinition(const raw_config_option_def &raw) :
    opt_key(raw.opt_key == nullptr ? std::string() : std::string(raw.opt_key)),
    type(raw.type),
    category(raw.category),
    level(raw.level),
    container_type(raw.container_type),
    option_preset_type(raw.option_preset_type),
    gui_type(raw.gui_type),
    printer_technology(raw.printer_technology),
    invalidates_step(raw.invalidates_step),
    is_optional(raw.is_optional != 0),
    multiline(raw.multiline != 0),
    full_width(raw.full_width != 0),
    is_code(raw.is_code != 0),
    is_vector_extruder(raw.is_vector_extruder != 0),
    readonly(raw.readonly != 0),
    can_phony(raw.can_phony != 0),
    can_be_disabled(raw.can_be_disabled != 0),
    height(raw.height),
    width(raw.width),
    label_width(raw.label_width),
    aligned_label_left(raw.aligned_label_left != 0),
    sidetext_width(raw.sidetext_width),
    max_literal_is_percent(raw.max_literal_is_percent != 0),
    precision(raw.precision),
    mode(raw.mode),
    flags(raw.flags),
    nature(raw.nature)
{
    const std::pair<const char *, std::optional<std::string> *> optional_strings[] = {
        {raw.gui_flags, &gui_flags},
        {raw.label, &label},
        {raw.full_label, &full_label},
        {raw.tooltip, &tooltip},
        {raw.translation_domain, &translation_domain},
        {raw.sidetext, &sidetext},
        {raw.cli, &cli},
        {raw.ratio_over, &ratio_over},
        {raw.default_serialized_value, &default_serialized_value},
    };
    for (const std::pair<const char *, std::optional<std::string> *> &entry : optional_strings)
        if (entry.first != nullptr)
            *entry.second = std::string(entry.first);

    if (raw.has_min)
        min_value = raw.min_value;
    if (raw.has_max)
        max_value = raw.max_value;
    if (raw.has_max_literal)
        max_literal_value = raw.max_literal_value;

    const std::pair<const_strings_t, std::vector<std::string> *> string_arrays[] = {
        {raw.aliases, &aliases},
        {raw.shortcut, &shortcut},
        {raw.depends_on, &depends_on},
    };
    for (const std::pair<const_strings_t, std::vector<std::string> *> &array : string_arrays) {
        array.second->reserve(array.first.size);
        for (uint32_t index = 0; index < array.first.size; ++index)
            if (array.first.items != nullptr && array.first.items[index] != nullptr)
                array.second->emplace_back(array.first.items[index]);
    }

    const key_value_string_pair_array_t &raw_entries = raw.enum_def.value_label_pairs;
    if (raw_entries.items != nullptr) {
        enum_entries.reserve(raw_entries.count);
        for (uint32_t index = 0; index < raw_entries.count; ++index) {
            const key_value_string_pair_t &entry = raw_entries.items[index];
            ConfigOptionEnumEntry copied;
            if (entry.value != nullptr)
                copied.value = std::string(entry.value);
            if (entry.label != nullptr)
                copied.label = std::string(entry.label);
            enum_entries.push_back(std::move(copied));
        }
    }

    if (raw.graph_settings != nullptr)
        graph_settings.emplace(*raw.graph_settings);
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_ConfigDefinitionViews_hpp_
