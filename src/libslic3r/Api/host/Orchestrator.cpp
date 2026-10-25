///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* The orchestrator owns plugin registration and run contexts, and resolves service invalidation to executable producers. */
#include "Orchestrator.hpp"
#include "GCodeReaderService.hpp"
#include "GCodeThumbnailRenderer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"
#include "libslic3r/Api/plugin/c/slic3r_gcode_script.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Plugins/BridgeDetector.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Thread.hpp"
#include "libslic3r/UiLayoutMerger.hpp"
#include "libslic3r/Utils.hpp"

#include "ApiHostUtils.hpp"
#include "ClipperShapes.hpp"
#include "Plugin.hpp"

namespace Slic3r {

// Resolve a borrowed plugin to its executable invalidation producer, returning STEP_NONE for unsupported steps.
static slicing_step_t plan_owner_step(const Plugin &plugin);

static bool graph_coordinate_equal(double lhs, double rhs);
static bool graph_enforced_values_equal(const GraphSettings &lhs, const GraphSettings &rhs);

// Derive the public report state from registrations and issues after every
// mutation. A package may register useful plugins before a later registration
// fails, which is reported as LoadedWithErrors instead of hiding those plugins.
static void update_plugin_package_load_state(PluginPackageLoadReport &report)
{
    if (report.issues.empty()) {
        report.state = PluginPackageLoadState::Loaded;
    } else if (report.registered_plugin_ids.empty()) {
        report.state = PluginPackageLoadState::Failed;
    } else {
        report.state = PluginPackageLoadState::LoadedWithErrors;
    }
}

Orchestrator *orchestrator_from_handle(orchestrator_handle *me) {
    return me == nullptr ? &Orchestrator::instance() : reinterpret_cast<Orchestrator *>(me);
}

const Orchestrator *orchestrator_from_handle(const orchestrator_handle *me) {
    return me == nullptr ? &Orchestrator::instance() : reinterpret_cast<const Orchestrator *>(me);
}

static ConfigOptionContainerType config_option_container_type(raw_container_type type)
{
    switch (type) {
    case RAW_CONTAINER_TYPE_PROJECT: return ConfigOptionContainerType::Project;
    case RAW_CONTAINER_TYPE_PLATER:  return ConfigOptionContainerType::Plater;
    case RAW_CONTAINER_TYPE_OBJECT:  return ConfigOptionContainerType::Object;
    case RAW_CONTAINER_TYPE_LAYER:   return ConfigOptionContainerType::Layer;
    case RAW_CONTAINER_TYPE_REGION:  return ConfigOptionContainerType::Region;
    case RAW_CONTAINER_TYPE_NONE:
    default:                         return ConfigOptionContainerType::None;
    }
}

static ConfigOptionType config_option_type(raw_config_option_type type)
{
    switch (type) {
    case RAW_CO_NONE:                    return coNone;
    case RAW_CO_BOOL:                    return coBool;
    case RAW_CO_INT:                     return coInt;
    case RAW_CO_FLOAT:                   return coFloat;
    case RAW_CO_FLOAT_OR_PERCENT:        return coFloatOrPercent;
    case RAW_CO_PERCENT:                 return coPercent;
    case RAW_CO_STRING:                  return coString;
    case RAW_CO_POINT:                   return coPoint;
    case RAW_CO_ENUM:                    return coEnum;
    case RAW_CO_GRAPH:                   return coGraph;
    case RAW_CO_VECTOR_BOOL:             return coBools;
    case RAW_CO_VECTOR_INT:              return coInts;
    case RAW_CO_VECTOR_FLOAT:            return coFloats;
    case RAW_CO_VECTOR_PERCENT:          return coPercents;
    case RAW_CO_VECTOR_FLOAT_OR_PERCENT: return coFloatsOrPercents;
    case RAW_CO_VECTOR_STRING:           return coStrings;
    case RAW_CO_VECTOR_POINT:            return coPoints;
    case RAW_CO_VECTOR_GRAPH:            return coGraphs;
    case RAW_CO_POINT3:                  return coPoint3;
    case RAW_CO_VECTOR_ENUM:
    default:                             return coNone;
    }
}

static bool raw_graph_type_valid(raw_graph_type type) noexcept
{
    return type == RAW_GRAPH_TYPE_SQUARE ||
           type == RAW_GRAPH_TYPE_LINEAR ||
           type == RAW_GRAPH_TYPE_SPLINE;
}

static GraphData::GraphType graph_type_from_raw(raw_graph_type type)
{
    switch (type) {
    case RAW_GRAPH_TYPE_SQUARE: return GraphData::GraphType::SQUARE;
    case RAW_GRAPH_TYPE_SPLINE: return GraphData::GraphType::SPLINE;
    case RAW_GRAPH_TYPE_NONE:
    case RAW_GRAPH_TYPE_LINEAR:
    default:                    return GraphData::GraphType::LINEAR;
    }
}

/* Validate pointer/count pairs and graph ranges before copying plugin memory. */
static bool raw_graph_settings_valid(const raw_config_option_def &def)
{
    const raw_graph_settings *settings = def.graph_settings;
    if (settings == nullptr)
        return true;
    if (def.type != RAW_CO_GRAPH && def.type != RAW_CO_VECTOR_GRAPH)
        return false;
    if ((settings->allowed_type_count > 0 && settings->allowed_types == nullptr) ||
        (settings->reset_values.point_count > 0 && settings->reset_values.points == nullptr) ||
        (settings->enforced_value_count > 0 && settings->enforced_values == nullptr) ||
        settings->enforced_value_count > 2)
        return false;

    // GraphDialog constructs bounded spin controls and divides by each step
    // while editing. Both axes therefore need finite, non-empty ranges and
    // finite positive increments before the definition reaches the GUI.
    if (!std::isfinite(settings->min_x) || !std::isfinite(settings->max_x) ||
        !std::isfinite(settings->step_x) || settings->min_x >= settings->max_x ||
        settings->step_x <= 0.0 ||
        !std::isfinite(settings->min_y) || !std::isfinite(settings->max_y) ||
        !std::isfinite(settings->step_y) || settings->min_y >= settings->max_y ||
        settings->step_y <= 0.0)
        return false;

    std::set<raw_graph_type> unique_types;
    for (uint32_t idx = 0; idx < settings->allowed_type_count; ++idx) {
        const raw_graph_type type = settings->allowed_types[idx];
        if (!raw_graph_type_valid(type) || !unique_types.insert(type).second)
            return false;
    }

    const raw_graph_data &reset = settings->reset_values;
    if (reset.begin_index > reset.end_index || reset.end_index > reset.point_count)
        return false;
    if (reset.point_count == 0) {
        if (reset.begin_index != 0 || reset.end_index != 0 ||
            (reset.type != RAW_GRAPH_TYPE_NONE && !raw_graph_type_valid(reset.type)))
            return false;
    } else if (!raw_graph_type_valid(reset.type)) {
        return false;
    }
    for (uint32_t idx = 0; idx < reset.point_count; ++idx) {
        const c_vec2d &point = reset.points[idx];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            (idx > 0 && reset.points[idx - 1].x > point.x))
            return false;
    }
    for (uint32_t idx = 0; idx < settings->enforced_value_count; ++idx) {
        const c_vec2d &point = settings->enforced_values[idx];
        if ((!std::isfinite(point.x) && !std::isnan(point.x)) || !std::isfinite(point.y))
            return false;
    }
    return true;
}

/* Copy graph editor metadata into the host-owned ConfigOptionDef. */
static void populate_graph_settings_from_raw(ConfigOptionDef &out, const raw_graph_settings &source)
{
    out.graph_settings = std::make_shared<GraphSettings>();
    GraphSettings &destination = *out.graph_settings;
    destination.title = source.title == nullptr ? std::string() : std::string(source.title);
    destination.description = source.description == nullptr ? std::string() : std::string(source.description);
    destination.y_label = source.y_label == nullptr ? std::string() : std::string(source.y_label);
    destination.x_label = source.x_label == nullptr ? std::string() : std::string(source.x_label);
    destination.null_label = source.null_label == nullptr ? std::string() : std::string(source.null_label);
    destination.min_x = source.min_x;
    destination.max_x = source.max_x;
    destination.step_x = source.step_x;
    destination.min_y = source.min_y;
    destination.max_y = source.max_y;
    destination.step_y = source.step_y;
    destination.label_min_x = source.label_min_x == nullptr ? std::string() : std::string(source.label_min_x);
    destination.label_max_x = source.label_max_x == nullptr ? std::string() : std::string(source.label_max_x);
    destination.label_min_y = source.label_min_y == nullptr ? std::string() : std::string(source.label_min_y);
    destination.label_max_y = source.label_max_y == nullptr ? std::string() : std::string(source.label_max_y);

    destination.allowed_types.reserve(source.allowed_type_count);
    for (uint32_t idx = 0; idx < source.allowed_type_count; ++idx)
        destination.allowed_types.push_back(graph_type_from_raw(source.allowed_types[idx]));

    const raw_graph_data &reset = source.reset_values;
    destination.has_reset_vals = reset.type != RAW_GRAPH_TYPE_NONE;
    destination.reset_vals.begin_idx = reset.begin_index;
    destination.reset_vals.end_idx = reset.end_index;
    destination.reset_vals.type = graph_type_from_raw(reset.type);
    destination.reset_vals.graph_points.reserve(reset.point_count);
    for (uint32_t idx = 0; idx < reset.point_count; ++idx)
        destination.reset_vals.graph_points.emplace_back(reset.points[idx].x, reset.points[idx].y);

    destination.enforced_values.reserve(source.enforced_value_count);
    for (uint32_t idx = 0; idx < source.enforced_value_count; ++idx)
        destination.enforced_values.emplace_back(source.enforced_values[idx].x, source.enforced_values[idx].y);
}

static bool string_vector_equal(const std::vector<std::string> &lhs, const std::vector<std::string> &rhs)
{
    return lhs == rhs;
}

static bool optional_double_equal(const double lhs, const double rhs)
{
    return std::abs(lhs - rhs) < 1e-12;
}

static std::string config_option_def_default_serialized(const ConfigOptionDef &def)
{
    return def.default_value ? def.default_value->serialize() : std::string();
}

static bool enum_def_equal(const ConfigOptionDef &lhs, const ConfigOptionDef &rhs)
{
    if (lhs.enum_def.get() == nullptr || rhs.enum_def.get() == nullptr)
        return lhs.enum_def.get() == nullptr && rhs.enum_def.get() == nullptr;

    return lhs.enum_def->values() == rhs.enum_def->values() &&
           lhs.enum_def->labels() == rhs.enum_def->labels();
}

/* Treat matching NaN graph endpoint sentinels as the same coordinate. */
static bool graph_coordinate_equal(double lhs, double rhs)
{
    return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
}

/* Compare enforced graph endpoints without applying NaN's IEEE inequality. */
static bool graph_enforced_values_equal(const GraphSettings &lhs, const GraphSettings &rhs)
{
    if (lhs.enforced_values.size() != rhs.enforced_values.size())
        return false;

    for (size_t idx = 0; idx < lhs.enforced_values.size(); ++idx) {
        const Vec2d &left = lhs.enforced_values[idx];
        const Vec2d &right = rhs.enforced_values[idx];
        if (!graph_coordinate_equal(left.x(), right.x()) ||
            !graph_coordinate_equal(left.y(), right.y()))
            return false;
    }
    return true;
}

static bool graph_settings_equal(const ConfigOptionDef &lhs, const ConfigOptionDef &rhs)
{
    if (!lhs.graph_settings || !rhs.graph_settings)
        return !lhs.graph_settings && !rhs.graph_settings;

    const GraphSettings &left = *lhs.graph_settings;
    const GraphSettings &right = *rhs.graph_settings;
    return left.title == right.title && left.description == right.description &&
           left.y_label == right.y_label && left.x_label == right.x_label &&
           left.null_label == right.null_label &&
           optional_double_equal(left.min_x, right.min_x) &&
           optional_double_equal(left.max_x, right.max_x) &&
           optional_double_equal(left.step_x, right.step_x) &&
           optional_double_equal(left.min_y, right.min_y) &&
           optional_double_equal(left.max_y, right.max_y) &&
           optional_double_equal(left.step_y, right.step_y) &&
           left.label_min_x == right.label_min_x && left.label_max_x == right.label_max_x &&
           left.label_min_y == right.label_min_y && left.label_max_y == right.label_max_y &&
           left.allowed_types == right.allowed_types &&
           left.has_reset_vals == right.has_reset_vals &&
           (!left.has_reset_vals || left.reset_vals == right.reset_vals) &&
           graph_enforced_values_equal(left, right);
}

static bool config_option_def_compatible(const ConfigOptionDef &existing,
                                         const ConfigOptionDef &candidate,
                                         std::string *reason)
{
    const auto fail = [reason](const char *field) {
        if (reason != nullptr)
            *reason = field;
        return false;
    };

    if (existing.type != candidate.type) return fail("type");
    if (existing.plugin_api_level != candidate.plugin_api_level) return fail("level");
    if (existing.plugin_api_flags != candidate.plugin_api_flags) return fail("flags");
    if (existing.contains_plugin_id != candidate.contains_plugin_id) return fail("contains_plugin_id");
    if (existing.category != candidate.category) return fail("category");
    if (existing.gui_type != candidate.gui_type) return fail("gui_type");
    if (existing.printer_technology != candidate.printer_technology) return fail("printer_technology");
    if (existing.container_type != candidate.container_type) return fail("container_type");
    if (existing.option_preset_type != candidate.option_preset_type) return fail("option_preset_type");
    if (existing.invalidates_step != candidate.invalidates_step) return fail("invalidates_step");
    if (existing.is_optional != candidate.is_optional) return fail("is_optional");
    if (existing.can_be_disabled != candidate.can_be_disabled) return fail("can_be_disabled");
    if (existing.multiline != candidate.multiline) return fail("multiline");
    if (existing.full_width != candidate.full_width) return fail("full_width");
    if (existing.is_code != candidate.is_code) return fail("is_code");
    if (existing.is_vector_extruder != candidate.is_vector_extruder) return fail("is_vector_extruder");
    if (existing.readonly != candidate.readonly) return fail("readonly");
    if (existing.can_phony != candidate.can_phony) return fail("can_phony");
    if (existing.aligned_label_left != candidate.aligned_label_left) return fail("aligned_label_left");
    if (existing.gui_flags != candidate.gui_flags) return fail("gui_flags");
    if (existing.label != candidate.label) return fail("label");
    if (existing.full_label != candidate.full_label) return fail("full_label");
    if (existing.tooltip != candidate.tooltip) return fail("tooltip");
    if (existing.translation_domain != candidate.translation_domain) return fail("translation_domain");
    if (existing.sidetext != candidate.sidetext) return fail("sidetext");
    if (existing.cli != candidate.cli) return fail("cli");
    if (existing.ratio_over != candidate.ratio_over) return fail("ratio_over");
    if (existing.height != candidate.height) return fail("height");
    if (existing.width != candidate.width) return fail("width");
    if (existing.label_width != candidate.label_width) return fail("label_width");
    if (existing.sidetext_width != candidate.sidetext_width) return fail("sidetext_width");
    if (!optional_double_equal(existing.min, candidate.min)) return fail("min");
    if (!optional_double_equal(existing.max, candidate.max)) return fail("max");
    if (!optional_double_equal(existing.max_literal.value, candidate.max_literal.value) ||
        existing.max_literal.percent != candidate.max_literal.percent) return fail("max_literal");
    if (existing.precision != candidate.precision) return fail("precision");
    if (existing.mode != candidate.mode) return fail("mode");
    if (!string_vector_equal(existing.aliases, candidate.aliases)) return fail("aliases");
    if (!string_vector_equal(existing.shortcut, candidate.shortcut)) return fail("shortcut");
    if (!string_vector_equal(existing.depends_on, candidate.depends_on)) return fail("depends_on");
    if (!enum_def_equal(existing, candidate)) return fail("enum_def");
    if (!graph_settings_equal(existing, candidate)) return fail("graph_settings");
    if (config_option_def_default_serialized(existing) != config_option_def_default_serialized(candidate))
        return fail("default_value");

    return true;
}

static bool same_option_ownership_scope(const Orchestrator::ConfigOptionOwner &existing,
                                        const Plugin &candidate)
{
    // Dynamic config keys are global once they enter PrintConfigDef. Without a
    // small ownership rule, two unrelated plugins could accidentally share a
    // key and then disagree about its type, default value, GUI placement or
    // invalidation behavior.
    //
    // The one intentional exception is an exclusive group: those plugins are
    // alternatives for the same job, so sharing a compatible key is how they
    // expose a stable setting regardless of which implementation is selected.
    if (existing.plugin_id == candidate.get_id())
        return true;

    return !existing.exclusive_group.empty() &&
           existing.exclusive_group == candidate.get_exclusive_group();
}

static bool is_valid_translation_domain(const std::string &domain)
{
    if (domain.empty())
        return false;

    for (const unsigned char character : domain)
        if (!std::isalnum(character) && character != '.' && character != '_' && character != '-')
            return false;
    return true;
}

static bool is_path_inside(const boost::filesystem::path &directory,
                           const boost::filesystem::path &package_root)
{
    // Compare canonical path components instead of string prefixes. A prefix
    // would incorrectly accept a sibling such as "plugin-extra", and cannot
    // detect a locale directory reached through a junction outside the package.
    boost::filesystem::path::iterator directory_part = directory.begin();
    boost::filesystem::path::iterator root_part = package_root.begin();
    for (; root_part != package_root.end(); ++root_part, ++directory_part) {
        if (directory_part == directory.end() || *directory_part != *root_part)
            return false;
    }
    return true;
}

static void populate_config_option_def_from_raw(ConfigOptionDef &out,
                                                const raw_config_option_def *def,
                                                const std::string &translation_domain)
{
    out.opt_key = def->opt_key;
    out.type = config_option_type(def->type);
    out.plugin_api_level = uint32_t(def->level);
    out.plugin_api_flags = def->flags;
    out.contains_plugin_id = def->contains_plugin_id != 0;
    out.category = static_cast<OptionCategory>(def->category);
    out.gui_type = static_cast<ConfigOptionDef::GUIType>(def->gui_type);
    out.printer_technology = static_cast<PrinterTechnology>(def->printer_technology);
    out.container_type = config_option_container_type(def->container_type);
    out.option_preset_type = static_cast<uint32_t>(def->option_preset_type);
    out.invalidates_step = def->invalidates_step;

    out.is_optional = def->is_optional != 0;
    out.multiline = def->multiline != 0;
    out.full_width = def->full_width != 0;
    out.is_code = def->is_code != 0;
    out.is_vector_extruder = def->is_vector_extruder != 0;
    out.readonly = def->readonly != 0;
    out.can_phony = def->can_phony != 0;
    out.can_be_disabled = def->can_be_disabled != 0;
    out.aligned_label_left = def->aligned_label_left != 0;
    out.is_script = false;

    if (def->gui_flags)
        out.gui_flags = def->gui_flags;
    if (def->label)
        out.label = def->label;
    if (def->full_label)
        out.full_label = def->full_label;
    if (def->tooltip)
        out.tooltip = def->tooltip;
    out.translation_domain = translation_domain;
    if (def->sidetext)
        out.sidetext = def->sidetext;
    if (def->cli)
        out.cli = def->cli;
    if (def->ratio_over)
        out.ratio_over = def->ratio_over;

    out.height = def->height;
    out.width = def->width;
    out.label_width = def->label_width;
    out.sidetext_width = def->sidetext_width;

    out.min = def->has_min ? def->min_value : -FLT_MAX;
    out.max = def->has_max ? def->max_value : FLT_MAX;
    out.max_literal = def->has_max_literal
        ? FloatOrPercent{def->max_literal_value, def->max_literal_is_percent != 0}
        : FloatOrPercent{0., false};

    out.precision = def->precision;
    out.mode = static_cast<ConfigOptionMode>(def->mode);

    for (size_t i = 0; i < def->aliases.size; ++i) {
        const char *s = def->aliases.items[i];
        if (s)
            out.aliases.emplace_back(s);
    }

    for (size_t i = 0; i < def->shortcut.size; ++i) {
        const char *s = def->shortcut.items[i];
        if (s)
            out.shortcut.emplace_back(s);
    }

    for (size_t i = 0; i < def->depends_on.size; ++i) {
        const char *s = def->depends_on.items[i];
        if (s)
            out.depends_on.emplace_back(s);
    }

    const bool has_pair_enum = def->enum_def.value_label_pairs.items != nullptr &&
        def->enum_def.value_label_pairs.count > 0;

    if (has_pair_enum) {
        std::vector<std::string> values;
        std::vector<std::pair<std::string, std::string>> values_labels;

        values.reserve(def->enum_def.value_label_pairs.count);
        values_labels.reserve(def->enum_def.value_label_pairs.count);
        for (size_t i = 0; i < def->enum_def.value_label_pairs.count; ++i) {
            const key_value_string_pair_t &p = def->enum_def.value_label_pairs.items[i];
            values.emplace_back(p.value ? p.value : "");
            values_labels.emplace_back(values.back(), p.label ? p.label : "");
        }

        if (!values.empty()) {
            // Plugin-provided enums may omit gui_type. In that case, keep the
            // old GUI behavior by exposing them as a closed combo box.
            const ConfigOptionDef::GUIType enum_gui_type = out.gui_type == ConfigOptionDef::GUIType::undefined ?
                ConfigOptionDef::GUIType::select_close :
                out.gui_type;
            if (enum_gui_type == ConfigOptionDef::GUIType::select_close) {
                // Closed scripted enums need the string -> int map before the
                // default value is deserialized below. This is especially
                // important for exclusive-step plugin selectors: GUI rules read
                // the enum as its integer index.
                out.set_enum_as_closed_for_scripted_enum(values_labels);
                out.gui_type = ConfigOptionDef::GUIType::select_close;
            } else if (!values_labels.empty()) {
                // Open enums can keep their string values directly; labels are
                // used only for display when provided.
                out.set_enum_values(enum_gui_type, values_labels);
            } else {
                out.set_enum_values(enum_gui_type, values);
            }
        }
    }

    if (def->graph_settings != nullptr)
        populate_graph_settings_from_raw(out, *def->graph_settings);

    ConfigOption *temp_default_option = out.create_empty_option();
    if (def->default_serialized_value != nullptr)
        temp_default_option->deserialize(def->default_serialized_value);
    // ConfigOptionDef has distinct ownership setters because vector defaults
    // also carry their extruder-sized marker. Select the matching overload
    // after deserialization instead of routing every option through the scalar
    // assertion.
    if (temp_default_option->is_vector())
        out.set_default_value(static_cast<ConfigOptionVectorBase *>(temp_default_option));
    else
        out.set_default_value(temp_default_option);
}

// Map a borrowed plugin's service step to its executable producer for invalidation; unknown steps return STEP_NONE.
static slicing_step_t plan_owner_step(const Plugin &plugin)
{
    const slicing_step_t step = plugin.get_step();
    switch (step) {
    case INFILL_PATTERN:
    case INFILL_SURFACE_RECIPE_MODIFIER:
        return STEP_INFILL;
    case BRIDGE_DETECTOR:
    case PERIMETER_GENERATION_MODULE:
        return STEP_PERIMETER;
    case SEAM_PLACER:
        return STEP_ORDERING;
    case WIPE_TOWER_WIPE:
        return STEP_WIPETOWER;
    case GCODE_FIRMWARE:
    case GCODE_THUMBNAIL:
    case GCODE_READER:
        return STEP_GCODE;
    default:
        return Steps::execution_index(step) < Steps::execution_order().size() ?
            step : STEP_NONE;
    }
}

static raw_config_option_def config_option_def_with_resolved_invalidation(const raw_config_option_def &def,
                                                                          const Plugin *plugin)
{
    raw_config_option_def resolved = def;

    // A service plugin runs inside its owning pipeline producer. Resolve the
    // default there so config changes never schedule a non-executable service.
    if (plugin != nullptr && resolved.invalidates_step == STEP_NONE) {
        const slicing_step_t owner_step = plan_owner_step(*plugin);
        resolved.invalidates_step = owner_step == STEP_NONE ? STEP_ANY : owner_step;
    }

    return resolved;
}

static bool validate_used_config_key_definition(const Plugin &,
                                                const Plugin::UsedConfigKey &used_key,
                                                std::string &error_message)
{
    // used_config_keys() is the plugin's read-side contract. The plugin may
    // read a built-in option or an option created by a dependency, but it must
    // describe the value representation it expects. This catches stale plugins
    // before they run with a ConfigOption subclass they do not understand.
    if (used_key.key.empty()) {
        error_message = "declares an empty used config key.";
        return false;
    }

    const ConfigOptionType expected_type = config_option_type(used_key.type);
    if (expected_type == coNone) {
        error_message = "declares used option '" + used_key.key +
                        "' without a supported value type.";
        return false;
    }

    const ConfigOptionDef *def = PrintConfigDef::instance().get(used_key.key);
    if (def == nullptr) {
        error_message = "uses option '" + used_key.key +
                        "', but no active plugin or built-in config defines it.";
        return false;
    }

    if (def->type != expected_type) {
        error_message = "expects option '" + used_key.key +
                        "' to have a different value type.";
        return false;
    }

    if (used_key.container_type != RAW_CONTAINER_TYPE_NONE &&
        def->container_type != config_option_container_type(used_key.container_type)) {
        error_message = "expects option '" + used_key.key +
                        "' to live in a different config container.";
        return false;
    }

    if (used_key.option_preset_type != RAW_PRESET_TYPE_NONE &&
        def->option_preset_type != uint32_t(used_key.option_preset_type)) {
        error_message = "expects option '" + used_key.key +
                        "' to belong to a different preset type.";
        return false;
    }

    return true;
}

// A setting read by a plugin contributes to that plugin's output, so changing
// it must replay the pipeline step that runs the plugin, even when the setting
// definition predates the plugin pipeline and still points at a later legacy
// step. Service plugins use the executable step that owns them. A key without
// a definition is skipped; this only happens for a plugin activated after
// initialize_plugins() validated the used keys.
static void tighten_used_config_key_invalidations(ConfigDef &definition, const Plugin &plugin)
{
    const slicing_step_t owner_step = plan_owner_step(plugin);
    if (owner_step == STEP_NONE)
        return;

    for (const Plugin::UsedConfigKey &used_key : plugin.get_used_config_keys()) {
        const t_optiondef_map::iterator option = definition.options.find(used_key.key);
        if (option != definition.options.end())
            option->second.invalidates_step = Steps::tightened_invalidation_step(
                option->second.invalidates_step, owner_step);
    }
}

static std::chrono::milliseconds elapsed_ms(const std::chrono::steady_clock::time_point &start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
}

} // namespace Slic3r

extern "C" {

option_def_error_code orchestrator_create_option_def(orchestrator_handle *me, const raw_config_option_def *def) {
    if (def == nullptr)
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    // removed for now
    // if (def->struct_size < offsetof(raw_config_option_def, reserved_u64))
    //    return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    if (def->opt_key == nullptr || def->default_serialized_value == nullptr ||
        def->container_type == RAW_CONTAINER_TYPE_NONE || def->option_preset_type == RAW_PRESET_TYPE_NONE ||
        def->printer_technology == RAW_PT_NONE || !Slic3r::raw_graph_settings_valid(*def))
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    try {
        return Slic3r::orchestrator_from_handle(me)->create_new_print_config(def);
    } catch (...) { return OPTION_DEF_ERROR_INTERNAL; }
}

option_def_error_code orchestrator_modify_option_def(orchestrator_handle *me, const raw_option_def_patch *patch) {
    if (patch == nullptr)
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    try {
        return Slic3r::orchestrator_from_handle(me)->modify_print_config(patch);
    } catch (...) { return OPTION_DEF_ERROR_INTERNAL; }
}

int32_t orchestrator_selected_plugin_used_config_keys(orchestrator_handle *me,
                                                      const config_handle *config,
                                                      slicing_step_t step,
                                                      raw_used_config_key *keys)
{
    Slic3r::Orchestrator *orchestrator = Slic3r::orchestrator_from_handle(me);
    const Slic3r::ConfigBase *config_base = Slic3r::ApiHost::to_config(config);
    const std::vector<Slic3r::Plugin *> plugins =
        Slic3r::Steps::selected_or_active_plugins_for_step(*orchestrator, step, config_base);

    // Exclusive steps yield one selected plugin; non-exclusive steps yield all
    // active plugins. Build one stable union so callers do not have to care
    // which execution model the queried step uses.
    std::vector<const Slic3r::Plugin::UsedConfigKey *> used_keys;
    for (const Slic3r::Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;

        const std::vector<Slic3r::Plugin::UsedConfigKey> &plugin_keys = plugin->get_used_config_keys();
        for (const Slic3r::Plugin::UsedConfigKey &candidate : plugin_keys) {
            const std::vector<const Slic3r::Plugin::UsedConfigKey *>::const_iterator found =
                std::find_if(used_keys.begin(),
                             used_keys.end(),
                             [&candidate](const Slic3r::Plugin::UsedConfigKey *existing) {
                                 return existing != nullptr &&
                                        existing->key == candidate.key &&
                                        existing->type == candidate.type &&
                                        existing->container_type == candidate.container_type &&
                                        existing->option_preset_type == candidate.option_preset_type;
                             });
            if (found == used_keys.end())
                used_keys.push_back(&candidate);
        }
    }

    if (keys != nullptr) {
        for (size_t idx = 0; idx < used_keys.size(); ++idx) {
            keys[idx].key = used_keys[idx]->key.c_str();
            keys[idx].type = used_keys[idx]->type;
            keys[idx].container_type = used_keys[idx]->container_type;
            keys[idx].option_preset_type = used_keys[idx]->option_preset_type;
        }
    }
    return int32_t(used_keys.size());
}
}

namespace Slic3r {

// Initialize the host registries and reserve built-in runtime identities.
// The singleton owns this state until process shutdown.
Orchestrator::Orchestrator()
{
    link_config_definition_api();
    // Zero is a real schema identity, not an allocation failure. Registering it
    // here lets generic callers resolve the portable Reader schema by name
    // while keeping the complete lower half unavailable to plugin allocation.
    const bool portable_reader_registered = this->register_static_runtime_id(
        GCODE_READER_STATE_ID_TYPE, "slic3r.portable", GCODE_READER_STATE_ID_PORTABLE);
    assert(portable_reader_registered);
    (void) portable_reader_registered;

    // Script events keep their compact historical values in PrintingPlan.
    // Reserving every stable name in the generic registry gives built-in and
    // plugin-defined scripts the same lookup path without making the numeric
    // value persistent outside this process.
    const std::pair<const char *, orchestrator_runtime_id> builtin_scripts[] = {
        {"start_gcode", GCODE_SCRIPT_TYPE_START_GCODE},
        {"end_gcode", GCODE_SCRIPT_TYPE_END_GCODE},
        {"extrusion_custom_gcode_script", GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM},
        {"start_filament_gcode", GCODE_SCRIPT_TYPE_START_FILAMENT_GCODE},
        {"end_filament_gcode", GCODE_SCRIPT_TYPE_END_FILAMENT_GCODE},
        {"before_layer_gcode", GCODE_SCRIPT_TYPE_BEFORE_LAYER_GCODE},
        {"layer_gcode", GCODE_SCRIPT_TYPE_LAYER_GCODE},
        {"toolchange_gcode", GCODE_SCRIPT_TYPE_TOOLCHANGE_GCODE},
        {"between_objects_gcode", GCODE_SCRIPT_TYPE_BETWEEN_OBJECTS_GCODE},
        {"feature_gcode", GCODE_SCRIPT_TYPE_FEATURE_GCODE},
        {"color_change_gcode", GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE},
        {"pause_print_gcode", GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE},
        {"template_custom_gcode", GCODE_SCRIPT_TYPE_TEMPLATE_CUSTOM_GCODE},
    };
    bool builtin_scripts_registered = true;
    for (const std::pair<const char *, orchestrator_runtime_id> &script : builtin_scripts)
        builtin_scripts_registered = this->register_static_runtime_id(
            GCODE_SCRIPT_ID_TYPE, script.first, script.second) && builtin_scripts_registered;
    assert(builtin_scripts_registered);
    (void) builtin_scripts_registered;
    // Built-in seam painting is registered in the same table as plugin generic
    // facet annotations. The GUI still creates a fixed toolbar button for seam
    // today, but using the registry keeps the storage key and future plugin
    // path identical from the model's point of view.
    m_generic_facets_annotations.emplace_back(builtin_seam_facets_annotation_definition());
}

// Return the function-local singleton; its plugin objects remain valid until shutdown.
Orchestrator &Orchestrator::instance() {
    static Orchestrator s_instance;
    static bool s_default_bridge_detector_registered = []() {
        slic3r_api::BridgeDetectorPlugin::register_bridge_detector_plugin(
            reinterpret_cast<orchestrator_handle *>(&s_instance));
        return true;
    }();
    (void) s_default_bridge_detector_registered;
    return s_instance;
}

// Validate the instance, create host-owned storage, and publish its metadata.
// Registration failure does not publish a partially constructed plugin.
bool Orchestrator::register_plugin(plugin_instance plugin) {
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::unique_ptr<Plugin> new_plugin;
    try {
        const PluginRegistrationSource *source = m_plugin_registration_sources.empty() ?
            nullptr : &m_plugin_registration_sources.back();
        new_plugin.reset(new Plugin(plugin,
                                    source != nullptr && source->external_plugin ? std::string() : "Slic3r",
                                    source != nullptr ? source->package_root : std::string()));
    } catch (const std::exception &error) {
        BOOST_LOG_TRIVIAL(error) << "Cannot register plugin: " << error.what() << std::endl;
        if (!m_plugin_registration_sources.empty() &&
            m_plugin_registration_sources.back().external_plugin) {
            PluginPackageLoadIssue issue;
            issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
            issue.detail = error.what();
            report_plugin_package_load_issue(m_plugin_registration_sources.back().package_id, std::move(issue));
        }
        return false;
    }

    const std::string new_id = new_plugin->get_id();
    const Plugin *check_exists = get_plugin(new_id);
    if (check_exists) {
        BOOST_LOG_TRIVIAL(error) << "Plugin with id " << new_id << " already exists, cannot register plugin"
                                 << std::endl;
        if (!m_plugin_registration_sources.empty() &&
            m_plugin_registration_sources.back().external_plugin) {
            PluginPackageLoadIssue issue;
            issue.code = PluginPackageLoadErrorCode::RegistrationFailed;
            issue.plugin_id = new_id;
            issue.detail = "A plugin with this id is already registered.";
            report_plugin_package_load_issue(m_plugin_registration_sources.back().package_id, std::move(issue));
        }
        return false;
    }
    m_registered_plugins.emplace_back(std::move(new_plugin));
    if (!m_plugin_registration_sources.empty() &&
        m_plugin_registration_sources.back().external_plugin) {
        PluginPackageLoadReport &report = m_plugin_package_load_reports[
            m_plugin_registration_sources.back().package_id];
        if (std::find(report.registered_plugin_ids.begin(), report.registered_plugin_ids.end(), new_id) ==
            report.registered_plugin_ids.end())
            report.registered_plugin_ids.push_back(new_id);
    }
    BOOST_LOG_TRIVIAL(debug) << "Registered plugin '" << new_id << "' in " << elapsed_ms(start).count() << " ms.";
    return true;
}

// Replace any previous report and begin collecting load results for one package.
void Orchestrator::begin_plugin_package_load(const std::string &package_id,
                                             const std::string &package_path,
                                             bool allows_no_plugins)
{
    if (package_id.empty())
        return;
    PluginPackageLoadReport report;
    report.package_id = package_id;
    report.package_path = package_path;
    report.allows_no_plugins = allows_no_plugins;
    m_plugin_package_load_reports[package_id] = std::move(report);
}

// Append one diagnostic and update the package state without unloading plugins.
void Orchestrator::report_plugin_package_load_issue(const std::string &package_id,
                                                    PluginPackageLoadIssue issue)
{
    if (package_id.empty())
        return;
    PluginPackageLoadReport &report = m_plugin_package_load_reports[package_id];
    report.package_id = package_id;
    report.issues.push_back(std::move(issue));
    update_plugin_package_load_state(report);
}

// Complete a report and synthesize the no-plugin error when the package is not allowed to be empty.
void Orchestrator::finish_plugin_package_load(const std::string &package_id)
{
    const std::map<std::string, PluginPackageLoadReport>::iterator found =
        m_plugin_package_load_reports.find(package_id);
    if (found == m_plugin_package_load_reports.end())
        return;

    PluginPackageLoadReport &report = found->second;
    if (report.issues.empty() && report.registered_plugin_ids.empty() && !report.allows_no_plugins) {
        PluginPackageLoadIssue issue;
        issue.code = PluginPackageLoadErrorCode::NoPluginsRegistered;
        issue.detail = "The package completed registration without registering a plugin instance.";
        report.issues.push_back(std::move(issue));
    }
    update_plugin_package_load_state(report);
}

// Discard reports only; registered plugin objects and activation state are preserved.
void Orchestrator::clear_plugin_package_load_reports()
{
    m_plugin_package_load_reports.clear();
}

// Return a borrowed report pointer; it is invalidated when reports are cleared.
const PluginPackageLoadReport *Orchestrator::plugin_package_load_report(const std::string &package_id) const
{
    const std::map<std::string, PluginPackageLoadReport>::const_iterator found =
        m_plugin_package_load_reports.find(package_id);
    return found == m_plugin_package_load_reports.end() ? nullptr : &found->second;
}

// Transfer the scope token so only one object can close the registration scope.
Orchestrator::PluginRegistrationScope::PluginRegistrationScope(PluginRegistrationScope &&other) noexcept
    : m_orchestrator(std::exchange(other.m_orchestrator, nullptr))
{
}

// Close this scope before taking the source token, preserving one active owner.
Orchestrator::PluginRegistrationScope &Orchestrator::PluginRegistrationScope::operator=(PluginRegistrationScope &&other) noexcept
{
    if (this != &other) {
        if (m_orchestrator != nullptr)
            m_orchestrator->end_plugin_registration_scope();
        m_orchestrator = std::exchange(other.m_orchestrator, nullptr);
    }
    return *this;
}

// Close the registration scope when its owning token is destroyed.
Orchestrator::PluginRegistrationScope::~PluginRegistrationScope()
{
    if (m_orchestrator != nullptr)
        m_orchestrator->end_plugin_registration_scope();
}

// Record the package context until the returned scope is destroyed.
Orchestrator::PluginRegistrationScope Orchestrator::plugin_registration_scope(std::string package_root,
                                                                                bool external_plugin)
{
    PluginRegistrationSource source;
    if (!package_root.empty()) {
        source.package_root = boost::filesystem::absolute(boost::filesystem::path(package_root))
                                  .lexically_normal()
                                  .generic_string();
        source.package_id = boost::filesystem::path(source.package_root).filename().string();
    }
    source.external_plugin = external_plugin;
    m_plugin_registration_sources.emplace_back(std::move(source));
    return PluginRegistrationScope(this);
}

// Remove the most recent registration context; the scope stack must not be empty.
void Orchestrator::end_plugin_registration_scope()
{
    assert(!m_plugin_registration_sources.empty());
    m_plugin_registration_sources.pop_back();
}

// Validate a package-relative catalog path and retain one catalog per domain.
// Return -1 for invalid input, -2 for a conflicting domain, 0 for a duplicate, and 1 when added.
int32_t Orchestrator::register_translation_catalog(const char *domain, const char *locale_directory)
{
    if (domain == nullptr || locale_directory == nullptr || m_plugin_registration_sources.empty())
        return -1;

    const std::string domain_value(domain);
    const boost::filesystem::path locale_path(locale_directory);
    const PluginRegistrationSource &source = m_plugin_registration_sources.back();
    if (!is_valid_translation_domain(domain_value) || locale_path.empty() || locale_path.is_absolute() ||
        source.package_root.empty())
        return -1;

    for (boost::filesystem::path::iterator it = locale_path.begin(); it != locale_path.end(); ++it)
        if (*it == "..")
            return -1;

    boost::system::error_code error;
    const boost::filesystem::path package_root =
        boost::filesystem::canonical(boost::filesystem::path(source.package_root), error);
    if (error)
        return -1;

    const boost::filesystem::path absolute_directory =
        boost::filesystem::canonical(package_root / locale_path, error);
    if (error || !is_path_inside(absolute_directory, package_root))
        return -1;

    const std::string directory_value = absolute_directory.generic_string();
    for (const TranslationCatalog &existing : m_translation_catalogs) {
        if (existing.domain != domain_value)
            continue;
        return existing.locale_directory == directory_value ? 0 : -2;
    }

    m_translation_catalogs.push_back(TranslationCatalog{domain_value, directory_value, package_root.generic_string()});
    return 1;
}

// Use the explicit domain when present, otherwise inherit the initializing plugin's domain.
std::string Orchestrator::resolved_translation_domain(const char *requested_domain) const
{
    if (requested_domain != nullptr && requested_domain[0] != '\0')
        return requested_domain;
    return m_initializing_plugin != nullptr ? m_initializing_plugin->get_translation_domain() : std::string();
}

// Treat the host domain as available and require a registered catalog for other domains.
bool Orchestrator::is_translation_domain_available(const std::string &domain, const std::string &package_root) const
{
    if (domain.empty() || domain == "Slic3r")
        return true;

    for (const TranslationCatalog &catalog : m_translation_catalogs)
        if (catalog.domain == domain && catalog.package_root == package_root)
            return true;
    return false;
}

// Copy borrowed plugin pointers into a snapshot so callers cannot alter the owning vector.
std::vector<Plugin *> Orchestrator::registered_plugins() const
{
    std::vector<Plugin *> plugins;
    plugins.reserve(m_registered_plugins.size());
    for (const std::unique_ptr<Plugin> &plugin : m_registered_plugins)
        plugins.push_back(plugin.get());
    return plugins;
}

// Allocate one compact id for a namespaced service step. The stable name is
// kept in a map-backed record so pointers returned by the C API remain valid
// when later steps are registered.
slicing_step_t Orchestrator::register_step(const char *namespaced_name,
                                           slicing_step_t invalidates_step)
{
    if (namespaced_name == nullptr || namespaced_name[0] == '\0' ||
        invalidates_step == STEP_NONE ||
        uint32_t(invalidates_step) >= uint32_t(SLICING_STEP_CUSTOM_BEGIN))
        return STEP_NONE;

    if (const DynamicStepInfo *existing = this->step_info(namespaced_name))
        return existing->invalidates_step == invalidates_step ? existing->id : STEP_NONE;

    if (m_next_dynamic_step > uint32_t(SLICING_STEP_CUSTOM_END))
        return STEP_NONE;

    DynamicStepInfo info;
    info.id = static_cast<slicing_step_t>(m_next_dynamic_step++);
    info.name = namespaced_name;
    info.invalidates_step = invalidates_step;
    const slicing_step_t id = info.id;
    m_dynamic_step_infos.emplace(id, std::move(info));
    return id;
}

// Intern one name in the registry selected by id_type. Dynamic counters are
// independent per type, therefore a numeric id is meaningful only together
// with the type supplied here.
orchestrator_runtime_id Orchestrator::register_runtime_id(
    const char *id_type, const char *namespaced_name)
{
    if (id_type == nullptr || id_type[0] == '\0' ||
        namespaced_name == nullptr || namespaced_name[0] == '\0')
        return ORCHESTRATOR_RUNTIME_ID_INVALID;

    RuntimeIdRegistry &registry = m_runtime_id_registries[id_type];
    const std::map<std::string, orchestrator_runtime_id>::const_iterator existing =
        registry.ids_by_name.find(namespaced_name);
    if (existing != registry.ids_by_name.end())
        return existing->second;
    if (registry.next_dynamic_id == ORCHESTRATOR_RUNTIME_ID_INVALID)
        return ORCHESTRATOR_RUNTIME_ID_INVALID;

    const orchestrator_runtime_id id = registry.next_dynamic_id++;
    registry.ids_by_name.emplace(namespaced_name, id);
    registry.names_by_id.emplace(id, namespaced_name);
    return id;
}

// Resolve an id without creating either its type registry or a new name.
const std::string *Orchestrator::runtime_id_name(
    const char *id_type, orchestrator_runtime_id id) const
{
    if (id_type == nullptr || id_type[0] == '\0')
        return nullptr;
    const std::map<std::string, RuntimeIdRegistry>::const_iterator registry =
        m_runtime_id_registries.find(id_type);
    if (registry == m_runtime_id_registries.end())
        return nullptr;
    const std::map<orchestrator_runtime_id, std::string>::const_iterator name =
        registry->second.names_by_id.find(id);
    return name == registry->second.names_by_id.end() ? nullptr : &name->second;
}

// Reserve a host-defined id in the lower half. Both directions are checked so
// a duplicate is accepted only when it describes exactly the same mapping.
bool Orchestrator::register_static_runtime_id(
    const char *id_type, const char *namespaced_name, orchestrator_runtime_id id)
{
    if (id_type == nullptr || id_type[0] == '\0' ||
        namespaced_name == nullptr || namespaced_name[0] == '\0' ||
        id >= ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN)
        return false;

    RuntimeIdRegistry &registry = m_runtime_id_registries[id_type];
    const std::map<std::string, orchestrator_runtime_id>::const_iterator existing_id =
        registry.ids_by_name.find(namespaced_name);
    const std::map<orchestrator_runtime_id, std::string>::const_iterator existing_name =
        registry.names_by_id.find(id);
    if (existing_id != registry.ids_by_name.end() || existing_name != registry.names_by_id.end())
        return existing_id != registry.ids_by_name.end() && existing_id->second == id &&
               existing_name != registry.names_by_id.end() && existing_name->second == namespaced_name;

    registry.ids_by_name.emplace(namespaced_name, id);
    registry.names_by_id.emplace(id, namespaced_name);
    return true;
}

// Resolve runtime step metadata by its compact id.
const Orchestrator::DynamicStepInfo *Orchestrator::step_info(slicing_step_t step) const
{
    const std::map<slicing_step_t, DynamicStepInfo>::const_iterator found =
        m_dynamic_step_infos.find(step);
    return found == m_dynamic_step_infos.end() ? nullptr : &found->second;
}

// Resolve runtime step metadata by the stable name shared by providers and
// consumers. Registration is infrequent, so a small linear lookup avoids a
// second ownership map for the same strings.
const Orchestrator::DynamicStepInfo *Orchestrator::step_info(const char *namespaced_name) const
{
    if (namespaced_name == nullptr)
        return nullptr;
    for (const std::pair<const slicing_step_t, DynamicStepInfo> &entry : m_dynamic_step_infos)
        if (entry.second.name == namespaced_name)
            return &entry.second;
    return nullptr;
}

// Validate and retain one fragment; duplicate IDs are rejected to keep the logical slot unique.
bool Orchestrator::add_ui_fragment(const char *target_file,
                                   const char *fragment_id,
                                   const char *content,
                                   int32_t priority)
{
    if (target_file == nullptr || target_file[0] == '\0' ||
        fragment_id == nullptr || fragment_id[0] == '\0' ||
        content == nullptr || content[0] == '\0')
        return false;

    for (const PluginUiFragment &fragment : m_ui_fragments)
        if (fragment.target_file == target_file && fragment.fragment_id == fragment_id) {
            // The fragment id names the logical UI slot, not the plugin that
            // registered it. This lets a built-in layout or one plugin say "the
            // controls for this feature are already present" and suppress later
            // copies. If the second copy is different, the application can still
            // run with the first fragment, but developers need a warning: two
            // providers now disagree on the controls hidden behind one id.
            if (fragment.content != content) {
                std::ostringstream message;
                message << "UI fragment '" << fragment_id << "' for '" << target_file
                        << "' was already registered with different content";
                if (!fragment.plugin_id.empty())
                    message << " by plugin '" << fragment.plugin_id << "'";
                if (m_initializing_plugin != nullptr)
                    message << "; plugin '" << m_initializing_plugin->get_id()
                            << "' attempted to register another version";
                if (m_initializing_plugin != nullptr && !m_initializing_plugin->get_exclusive_group().empty())
                    message << " from exclusive group '" << m_initializing_plugin->get_exclusive_group() << "'";
                if (!fragment.exclusive_group.empty())
                    message << " while the kept fragment belongs to exclusive group '" << fragment.exclusive_group << "'";
                message << ". The first fragment is kept.";
                const std::string message_text = message.str();
                BOOST_LOG_TRIVIAL(warning) << message_text;
                if (m_initializing_plugin != nullptr)
                    this->add_plugin_message(PluginMessageLevel::Warning,
                                             m_initializing_plugin,
                                             STEP_NONE,
                                             message_text.c_str());
            }
            return false;
        }

    PluginUiFragment fragment;
    fragment.target_file = target_file;
    fragment.fragment_id = fragment_id;
    fragment.content = content;
    if (m_initializing_plugin != nullptr) {
        fragment.plugin_id = m_initializing_plugin->get_id();
        fragment.exclusive_group = m_initializing_plugin->get_exclusive_group();
    }
    fragment.priority = priority;
    fragment.order = m_next_ui_fragment_order++;
    m_ui_fragments.emplace_back(std::move(fragment));
    return true;
}

std::vector<Orchestrator::PluginUiFragment>
Orchestrator::ui_fragments_for_file(const std::string &target_file) const
{
    // Return copies because sorting and later registry changes must not mutate caller-visible storage.
    std::vector<PluginUiFragment> out;
    for (const PluginUiFragment &fragment : m_ui_fragments)
        if (fragment.target_file == target_file)
            out.emplace_back(fragment);

    std::sort(out.begin(), out.end(), [](const PluginUiFragment &lhs, const PluginUiFragment &rhs) {
        if (lhs.priority != rhs.priority)
            return lhs.priority < rhs.priority;
        if (lhs.order != rhs.order)
            return lhs.order < rhs.order;
        return lhs.fragment_id < rhs.fragment_id;
    });
    return out;
}

// Merge the registered fragments into a base layout without excluding any fragment ID.
std::string Orchestrator::merged_ui_layout(const std::string &target_file, const std::string &base_content) const
{
    static const std::unordered_set<std::string> empty_implemented_fragment_ids;
    return this->merged_ui_layout(target_file, base_content, empty_implemented_fragment_ids);
}

std::string Orchestrator::merged_ui_layout(const std::string &target_file,
                                           const std::string &base_content,
                                           const std::unordered_set<std::string> &implemented_fragment_ids) const
{
    // Skip IDs already implemented by the base layout or another provider.
    const std::vector<PluginUiFragment> fragments = this->ui_fragments_for_file(target_file);
    if (fragments.empty())
        return base_content;

    UiLayoutMerger merger(target_file);
    merger.set_base(base_content);
    for (const PluginUiFragment &fragment : fragments) {
        if (implemented_fragment_ids.find(fragment.fragment_id) != implemented_fragment_ids.end())
            continue;
        merger.add_fragment(fragment.fragment_id, fragment.content, fragment.priority, fragment.order);
    }
    return merger.merged();
}

// Owner attributed to rules registered through the C API while a plugin is
// initializing; "core" outside plugin initialization.
std::string Orchestrator::current_gui_rule_owner() const
{
    return m_initializing_plugin != nullptr ? m_initializing_plugin->get_id() : std::string(CORE_GUI_RULE_OWNER);
}

// Assign or reuse a type ID after validating the descriptor; incompatible reuse is rejected.
slic3r_property_type Orchestrator::register_property(const char *namespaced_name,
                                                     uint32_t byte_count,
                                                     uint32_t alignment)
{
    if (namespaced_name == nullptr || namespaced_name[0] == '\0' || byte_count == 0 || alignment == 0)
        return SLIC3R_PROPERTY_TYPE_INVALID;

    /*
    The orchestrator owns the string-to-id mapping for custom properties. The
    numeric id is only a compact runtime handle; callers must register by name
    again for each orchestrator so parallel plugin configurations do not share
    mutable global state.
    */
    if (const PropertyInfo *existing = this->property_info(namespaced_name)) {
        return existing->byte_count == byte_count && existing->alignment == alignment ?
            existing->type :
            SLIC3R_PROPERTY_TYPE_INVALID;
    }

    PropertyInfo info;
    info.type = m_next_custom_property_type++;
    if (info.type == SLIC3R_PROPERTY_TYPE_INVALID)
        info.type = m_next_custom_property_type++;
    info.name = namespaced_name;
    info.byte_count = byte_count;
    info.alignment = alignment;
    m_custom_property_infos.emplace_back(std::move(info));
    return m_custom_property_infos.back().type;
}

// Reuse the generic property registry for the extrusion-property ABI type.
extrusion_property_type Orchestrator::register_custom_extrusion_property(const char *namespaced_name,
                                                                         uint32_t byte_count,
                                                                         uint32_t alignment)
{
    return static_cast<extrusion_property_type>(this->register_property(namespaced_name, byte_count, alignment));
}

// Validate and retain one annotation definition; conflicting duplicate keys are rejected.
bool Orchestrator::register_generic_facets_annotation(GenericFacetsAnnotationDefinition def)
{
    if (def.key.empty() || def.label.empty() || def.enforce_label.empty() || def.block_label.empty())
        return false;

    if (def.icon_filename.empty() && def.icon_svg.empty())
        return false;

    const bool has_explicit_translation_domain = !def.translation_domain.empty();
    def.translation_domain = this->resolved_translation_domain(def.translation_domain.c_str());
    if (has_explicit_translation_domain &&
        !this->is_translation_domain_available(
            def.translation_domain,
            m_initializing_plugin != nullptr ? m_initializing_plugin->get_package_root() : std::string()))
        return false;

    // Duplicate keys are allowed only when they describe the same painting, so
    // a plugin can call registration more than once without changing the GUI or
    // model storage associated with that stable key.
    for (const GenericFacetsAnnotationDefinition &existing : m_generic_facets_annotations) {
        if (existing.key != def.key)
            continue;

        if (existing.label == def.label &&
            existing.enforce_label == def.enforce_label &&
            existing.block_label == def.block_label &&
            existing.translation_domain == def.translation_domain &&
            existing.icon_filename == def.icon_filename &&
            existing.icon_svg == def.icon_svg)
            return true;

        if (m_initializing_plugin != nullptr) {
            m_initializing_plugin_failed = true;
            m_initializing_plugin_failure = "Generic facet annotation '" + def.key +
                "' was already registered with different labels.";
        }
        return false;
    }

    m_generic_facets_annotations.emplace_back(std::move(def));
    return true;
}

const Orchestrator::CustomExtrusionPropertyInfo*
Orchestrator::custom_extrusion_property_info(extrusion_property_type type) const
{
    // The property APIs share one descriptor table.
    return this->property_info(type);
}

const Orchestrator::PropertyInfo*
Orchestrator::property_info(slic3r_property_type type) const
{
    for (const PropertyInfo &info : m_custom_property_infos)
        if (info.type == type)
            return &info;
    return nullptr;
}

const Orchestrator::CustomExtrusionPropertyInfo*
Orchestrator::custom_extrusion_property_info(const char *namespaced_name) const
{
    // Name lookup uses the same table as numeric lookup.
    return this->property_info(namespaced_name);
}

const Orchestrator::PropertyInfo*
Orchestrator::property_info(const char *namespaced_name) const
{
    if (namespaced_name == nullptr)
        return nullptr;

    for (const PropertyInfo &info : m_custom_property_infos)
        if (info.name == namespaced_name)
            return &info;
    return nullptr;
}

plugin_host_context Orchestrator::prepare_plugin_host_context(slicing_step_t step,
                                                              Plugin *plugin,
                                                              Print *print)
{
    // These handles are borrowed; the caller must keep the plugin and print alive for the run.
    plugin_host_context context = {};
    context.orchestrator = this;
    context.print = print;
    context.plugin = plugin;
    context.step = step;
    return context;
}

plugin_run_context Orchestrator::prepare_plugin_run_context(slicing_step_t step,
                                                            Plugin *plugin,
                                                            plugin_host_context *host_context) {
    // Plugin storage stays owned by this orchestrator; host_context is optional and borrowed.
    plugin_run_context context = {};
    context.step = step;
    context.print = host_context != nullptr && host_context->print != nullptr ?
        reinterpret_cast<const print_handle *>(host_context->print) : nullptr;
    context.plugin_storage = reinterpret_cast<storage_handle *>(&this->plugin_storage()[plugin]);
    context.host_context = host_context;
    context.is_cancelled = orchestrator_plugin_is_cancelled;
    context.report_warning = orchestrator_plugin_report_warning;
    context.report_error = orchestrator_plugin_report_error;
    context.report_progress = orchestrator_plugin_report_progress;
    return context;
}

void Orchestrator::track_plugin_setup(Plugin &plugin, Print &print, const slicing_step_t step)
{
    print.track_plugin_setup(plugin, step);
}

bool Orchestrator::clear_plugin_participants(Print &print, const slicing_step_t first_invalid_step)
{
    const size_t first_invalid_index = Steps::execution_index(first_invalid_step);
    assert(first_invalid_index < Steps::execution_order().size());
    if (first_invalid_index >= Steps::execution_order().size())
        return false;

    // Cleanup removes successful entries from Print, so iterate a copy rather than that changing list.
    std::vector<Print::PluginCleanupParticipant> participants =
        print.plugin_cleanup_participants();
    bool all_clear_succeeded = true;
    // Consumers may retain producer-owned handles, so unwind steps and setup order in reverse.
    std::stable_sort(
        participants.begin(), participants.end(), [](const Print::PluginCleanupParticipant &lhs,
                                                      const Print::PluginCleanupParticipant &rhs) {
            const size_t lhs_index = Steps::execution_index(lhs.step);
            const size_t rhs_index = Steps::execution_index(rhs.step);
            if (lhs_index != rhs_index)
                return lhs_index > rhs_index;
            return lhs.setup_order > rhs.setup_order;
        });

    for (const Print::PluginCleanupParticipant &participant : participants) {
        if (participant.plugin == nullptr ||
            Steps::execution_index(participant.step) < first_invalid_index)
            continue;

        std::map<Plugin *, PluginStorage>::iterator storage =
            m_plugin_storage.find(participant.plugin);
        if (storage == m_plugin_storage.end()) {
            this->add_plugin_message(PluginMessageLevel::Error, participant.plugin, participant.step,
                                     "Cannot clear Print state: plugin storage is missing.");
            all_clear_succeeded = false;
            continue;
        }

        int32_t clear_code = PLUGIN_ERROR_OK;
        try {
            clear_code = participant.plugin->clear(
                reinterpret_cast<const print_handle *>(&print),
                reinterpret_cast<storage_handle *>(&storage->second));
        } catch (const std::exception &error) {
            clear_code = this->store_error_message(error.what());
        } catch (...) {
            clear_code = this->store_error_message("Unknown exception while clearing Print state");
        }
        // Failed entries stay tracked for retry instead of being mistaken for released caches.
        if (clear_code == PLUGIN_ERROR_OK) {
            print.erase_plugin_cleanup_participant(participant.plugin, participant.step);
        } else {
            all_clear_succeeded = false;
            // The registry owns the text: reserved, unknown or negative codes
            // resolve to a generic message instead of an empty diagnostic.
            const std::string message = this->error_message(clear_code);
            BOOST_LOG_TRIVIAL(error) << "Plugin '" << participant.plugin->get_id()
                                     << "' failed to clear Print state (code " << clear_code
                                     << "): " << message;
            this->add_plugin_message(PluginMessageLevel::Error, participant.plugin, participant.step, message.c_str());
        }
    }
    return all_clear_succeeded;
}

// Run one plugin with the same two-phase barrier used by object-oriented
// slicing steps. Each worker receives a private host/run context while the
// caller-owned payload address remains identical in both per-run phases.
raw_plugin_execution_status Orchestrator::execute_plugin(
    Plugin &plugin,
    Print *print,
    const raw_plugin_run_payload *payloads,
    uint32_t run_count,
    bool cancel_on_error)
{
    if (!this->owns_plugin(&plugin) || (run_count != 0 && payloads == nullptr))
        return RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT;
    if (plugin.has_clear() && print == nullptr)
        return RAW_PLUGIN_EXECUTION_INVALID_ARGUMENT;
    if (!this->is_plugin_active(&plugin))
        return RAW_PLUGIN_EXECUTION_INACTIVE;
    if (this->is_plugin_cancelled() ||
        (print != nullptr && print->canceled()))
        return RAW_PLUGIN_EXECUTION_CANCELLED;

    std::atomic_bool execution_error { false };
    plugin_host_context host_context = this->prepare_plugin_host_context(plugin.get_step(), &plugin, print);
    host_context.object_count = run_count;
    host_context.execution_error = &execution_error;
    host_context.cancel_on_error = cancel_on_error;
    plugin_run_context run_context = this->prepare_plugin_run_context(
        plugin.get_step(), &plugin, &host_context);

    plugin.setup(run_context, run_count);
    if (execution_error.load(std::memory_order_relaxed))
        return RAW_PLUGIN_EXECUTION_PLUGIN_ERROR;
    if (this->is_plugin_cancelled() ||
        (print != nullptr && print->canceled()))
        return RAW_PLUGIN_EXECUTION_CANCELLED;

    parallel_for(size_t(0), size_t(run_count),
        [&plugin, &payloads, &run_context, &host_context](size_t idx) {
            plugin_run_context context_copy = run_context;
            plugin_host_context host_context_copy = host_context;
            host_context_copy.object_idx = idx;
            context_copy.host_context = &host_context_copy;
            if (context_copy.is_cancelled != nullptr &&
                context_copy.is_cancelled(context_copy.host_context))
                return;
            context_copy.data = payloads[idx].data;
            plugin.setup_run(context_copy);
        });

    if (execution_error.load(std::memory_order_relaxed))
        return RAW_PLUGIN_EXECUTION_PLUGIN_ERROR;
    if (this->is_plugin_cancelled() ||
        (print != nullptr && print->canceled()))
        return RAW_PLUGIN_EXECUTION_CANCELLED;

    parallel_for(size_t(0), size_t(run_count),
        [&plugin, &payloads, &run_context, &host_context](size_t idx) {
            plugin_run_context context_copy = run_context;
            plugin_host_context host_context_copy = host_context;
            host_context_copy.object_idx = idx;
            context_copy.host_context = &host_context_copy;
            if (context_copy.is_cancelled != nullptr &&
                context_copy.is_cancelled(context_copy.host_context))
                return;
            context_copy.data = payloads[idx].data;
            plugin.run(context_copy);
        });

    if (execution_error.load(std::memory_order_relaxed))
        return RAW_PLUGIN_EXECUTION_PLUGIN_ERROR;
    return this->is_plugin_cancelled() ||
                   (print != nullptr && print->canceled()) ?
        RAW_PLUGIN_EXECUTION_CANCELLED : RAW_PLUGIN_EXECUTION_SUCCESS;
}

void Orchestrator::add_plugin_message(PluginMessageLevel level,
                                      const Plugin *plugin,
                                      slicing_step_t step,
                                      const char *message,
                                      int32_t diagnostic_id)
{
    // Plugins can report messages while slicing runs on a worker thread, while
    // the Plater consumes them later from the UI thread. The queue keeps only
    // copied strings, so plugin-owned buffers may disappear immediately after
    // the callback returns.
    PluginMessage plugin_message;
    plugin_message.level = level;
    // Idle diagnostics without a plugin belong to the host; a real step remains unattributed.
    plugin_message.plugin_id = plugin != nullptr ? plugin->get_id() :
        (step == STEP_NONE ? "<host>" : "<unknown>");
    plugin_message.step = step;
    plugin_message.diagnostic_id = diagnostic_id;
    plugin_message.message = message != nullptr && message[0] != '\0' ?
        message :
        "(empty plugin message)";

    std::lock_guard<std::mutex> lock(m_plugin_messages_mutex);
    m_plugin_messages.emplace_back(std::move(plugin_message));
}

std::vector<Orchestrator::PluginMessage> Orchestrator::consume_plugin_messages()
{
    // The GUI drains the queue instead of peeking at it so each plugin message
    // is displayed exactly once, even if several status updates arrive for the
    // same slicing step.
    std::lock_guard<std::mutex> lock(m_plugin_messages_mutex);
    std::vector<PluginMessage> messages;
    messages.swap(m_plugin_messages);
    return messages;
}

int32_t Orchestrator::store_error_message(const char *message)
{
    if (message == nullptr || message[0] == '\0')
        return PLUGIN_ERROR_GENERAL;

    try {
        std::lock_guard<std::mutex> lock(m_error_messages_mutex);
        // Ids are append-only and never reused, so two concurrent callbacks keep
        // distinct messages. Saturation falls back to the generic failure code.
        if (m_next_error_message_id <= 0 ||
            m_next_error_message_id == (std::numeric_limits<int32_t>::max)())
            return PLUGIN_ERROR_GENERAL;
        m_error_messages.emplace_back(message);
        return m_next_error_message_id++;
    } catch (...) {
        // Diagnostic storage must not throw across the C ABI.
        return PLUGIN_ERROR_GENERAL;
    }
}

std::string Orchestrator::error_message(int32_t id) const
{
    if (id == PLUGIN_ERROR_GENERAL)
        return "Plugin reported an error without a diagnostic.";
    if (id < PLUGIN_ERROR_FIRST_DYNAMIC)
        // Reserved but unused ids, and anything below zero, use the generic text.
        return "Plugin reported an unknown error code.";

    std::lock_guard<std::mutex> lock(m_error_messages_mutex);
    const size_t index = size_t(id) - size_t(PLUGIN_ERROR_FIRST_DYNAMIC);
    if (index >= m_error_messages.size())
        return "Plugin reported an unknown diagnostic id.";
    return m_error_messages[index];
}

// Read the host cancellation flag without changing it.
bool Orchestrator::is_plugin_cancelled() const { return m_plugin_cancel_requested.load(std::memory_order_relaxed); }

// Publish cancellation for running plugin code to observe at its next cancellation point.
void Orchestrator::request_plugin_cancel() { m_plugin_cancel_requested.store(true, std::memory_order_relaxed); }

// Clear cancellation before starting an independent operation.
void Orchestrator::reset_plugin_cancel() { m_plugin_cancel_requested.store(false, std::memory_order_relaxed); }

bool Orchestrator::validate_plugin_activation(const std::vector<std::string> &plugin_ids,
                                              std::string &error_message) const
{
    // The plugin dialog needs to reject a bad activation set before it writes
    // activated.ini and asks the user to restart. At that moment the plugins
    // are loaded but not initialized, so their real raw_config_option_def
    // objects may not exist yet. The only contract available cheaply is the
    // declared list of keys returned by defined_config_keys().
    //
    // This preflight checks dependencies and declared configuration ownership:
    // - it verifies that every requested plugin is loaded;
    // - it requires every dependency to belong to the complete requested set;
    // - it rejects a key already owned by an unrelated plugin;
    // - it rejects a key already present in the built-in/global config unless
    //   it was introduced by the same compatible exclusive group.
    //
    // It does not decide whether the definitions are byte-for-byte compatible.
    // That stricter validation still happens in create_new_print_config() while
    // the active plugin initializes and publishes the actual option definition.
    std::set<std::string> selected_ids(plugin_ids.begin(), plugin_ids.end());
    const std::map<std::string, std::string> activation_errors = plugin_activation_errors(selected_ids);
    if (!activation_errors.empty()) {
        error_message.clear();
        for (const auto &[id, reason] : activation_errors) {
            if (!error_message.empty()) error_message += "\n";
            error_message += reason;
        }
        return false;
    }
    std::map<std::string, ConfigOptionOwner> future_owners = m_config_option_owners;

    for (const std::string &plugin_id : selected_ids) {
        const Plugin *plugin = this->get_plugin(plugin_id);
        if (plugin == nullptr) {
            error_message = "Plugin '" + plugin_id + "' is not loaded.";
            return false;
        }

        for (const Plugin::DefinedConfigKey &defined_key : plugin->get_defined_config_keys()) {
            const std::string &key = defined_key.key;
            const std::map<std::string, ConfigOptionOwner>::const_iterator existing_owner = future_owners.find(key);
            if (existing_owner != future_owners.end()) {
                if (same_option_ownership_scope(existing_owner->second, *plugin))
                    continue;

                error_message = "Plugin '" + plugin->get_id() + "' defines option '" + key +
                    "', but that option is already owned by plugin '" + existing_owner->second.plugin_id + "'.";
                return false;
            }

            if (PrintConfigDef::instance().get(key) != nullptr) {
                error_message = "Plugin '" + plugin->get_id() + "' defines option '" + key +
                    "', but that option already exists outside its exclusive group.";
                return false;
            }

            future_owners.emplace(key, ConfigOptionOwner{plugin->get_id(), plugin->get_exclusive_group()});
        }
    }

    return true;
}

option_def_error_code Orchestrator::create_new_print_config(const raw_config_option_def *def) {
    const raw_config_option_def resolved_def =
        config_option_def_with_resolved_invalidation(*def, m_initializing_plugin);
    def = &resolved_def;

    const bool has_explicit_translation_domain =
        def->translation_domain != nullptr && def->translation_domain[0] != '\0';
    const std::string translation_domain = this->resolved_translation_domain(def->translation_domain);
    if (has_explicit_translation_domain &&
        !this->is_translation_domain_available(
            translation_domain,
            m_initializing_plugin != nullptr ? m_initializing_plugin->get_package_root() : std::string()))
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    //PrintOptionPresetType preset_type = static_cast<PrintOptionPresetType>(def->option_preset_type);
    //PrintOptionContainer container = static_cast<PrintOptionContainer>(def->container_type);
    const ConfigOptionType type = config_option_type(def->type);
    assert(type != coNone);

    // This function is the definitive validation point for plugin settings.
    // Unlike validate_plugin_activation(), it sees the whole raw_config_option_def
    // and can compare every user-visible and serialization-relevant field.
    //
    // Several alternative plugins may publish the same setting, but only when
    // they are explicit alternatives in the same exclusive group. A compatible
    // duplicate from another group is still an error: a plugin may depend on
    // an option owned by another plugin, but it must not re-declare that
    // option. The dependency should provide the option definition; the
    // dependent plugin should only read the already-owned key.
    if (const ConfigOptionDef *existing = PrintConfigDef::instance().get(def->opt_key)) {
        ConfigOptionDef candidate;
        populate_config_option_def_from_raw(candidate, def, translation_domain);

        std::string reason;
        if (config_option_def_compatible(*existing, candidate, &reason)) {
            if (m_initializing_plugin == nullptr)
                return OPTION_DEF_ERROR_OK;

            const std::map<std::string, ConfigOptionOwner>::const_iterator owner_it =
                m_config_option_owners.find(def->opt_key);
            if (owner_it != m_config_option_owners.end() &&
                same_option_ownership_scope(owner_it->second, *m_initializing_plugin))
                return OPTION_DEF_ERROR_OK;

            std::ostringstream message;
            message << "Plugin config option '" << def->opt_key
                    << "' is already defined outside exclusive group '"
                    << m_initializing_plugin->get_exclusive_group() << "'.";
            if (owner_it != m_config_option_owners.end())
                message << " Existing owner is plugin '" << owner_it->second.plugin_id << "'.";
            else
                message << " Existing definition is not owned by a plugin in this group.";
            message << " Plugin '" << m_initializing_plugin->get_id() << "' will be disabled.";
            const std::string message_text = message.str();
            BOOST_LOG_TRIVIAL(error) << message_text;
            this->add_plugin_message(PluginMessageLevel::Error,
                                     m_initializing_plugin,
                                     STEP_NONE,
                                     message_text.c_str());
            m_initializing_plugin_failed = true;
            m_initializing_plugin_failure = message_text;
            return OPTION_DEF_ERROR_ALREADY_EXISTS;
        }

        std::ostringstream message;
        message << "Plugin config option '" << def->opt_key
                << "' is already defined with incompatible " << reason << ".";
        if (m_initializing_plugin != nullptr)
            message << " Plugin '" << m_initializing_plugin->get_id() << "' will be disabled.";
        const std::string message_text = message.str();
        BOOST_LOG_TRIVIAL(error) << message_text;
        if (m_initializing_plugin != nullptr) {
            this->add_plugin_message(PluginMessageLevel::Error,
                                     m_initializing_plugin,
                                     STEP_NONE,
                                     message_text.c_str());
            m_initializing_plugin_failed = true;
            m_initializing_plugin_failure = message_text;
        }
        return OPTION_DEF_ERROR_ALREADY_EXISTS;
    }

    ConfigOptionDef &out = *PrintConfigDef::instance_mutable().add(def->opt_key, type);
    populate_config_option_def_from_raw(out, def, translation_domain);

    // GUI activation rules may have been registered before this definition.
    // Attach the queued rules now that the setting exists.
    m_gui_rule_registry.attach_pending_for_setting(out.opt_key);

    // A plugin option that is not declared in used_config_keys never receives a
    // generated activation rule: it would stay editable whatever the selected
    // implementation is. Only plugins that already participate in the used-key
    // contract are reported, so host-created group selectors stay silent.
    if (m_initializing_plugin != nullptr) {
        const std::vector<Plugin::UsedConfigKey> &used_keys = m_initializing_plugin->get_used_config_keys();
        if (!used_keys.empty()) {
            const bool declared = std::any_of(used_keys.begin(), used_keys.end(),
                [&out](const Plugin::UsedConfigKey &used_key) { return used_key.key == out.opt_key; });
            if (!declared)
                BOOST_LOG_TRIVIAL(warning)
                    << "Plugin '" << m_initializing_plugin->get_id() << "' created option '"
                    << out.opt_key << "' without declaring it in used_config_keys; no activation rule "
                    << "will be generated for it.";
        }
    }

    // publish it?
    PrintConfigDef::instance_mutable().option_keys(def->option_preset_type).insert(out.opt_key);
    if(def->option_preset_type == RAW_PRESET_TYPE_FFF_FILAMENT_OVERRIDE) {
        assert(false); // please do'nt yet, not made for that
    }
    if(def->option_preset_type == RAW_PRESET_TYPE_SLA_MATERIAL_OVERRIDE) {
        assert(false); // please do'nt yet, not made for that
    }
    if(def->option_preset_type == RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER
        || def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS
        || def->option_preset_type == RAW_PRESET_TYPE_FFF_TOOL_MILLING) {
        PrintConfigDef::instance_mutable().option_keys(RAW_PRESET_TYPE_FFF_PRINTER).insert(out.opt_key);
    }
    if(def->option_preset_type == RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER_RETRACTION) {
        PrintConfigDef::instance_mutable().option_keys(RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER).insert(out.opt_key);
        PrintConfigDef::instance_mutable().option_keys(RAW_PRESET_TYPE_FFF_PRINTER).insert(out.opt_key);
    }
    add_to_prusa_export_to_remove_keys(out.opt_key);
    if (m_initializing_plugin != nullptr)
        m_config_option_owners[out.opt_key] = ConfigOptionOwner{
            m_initializing_plugin->get_id(),
            m_initializing_plugin->get_exclusive_group()
        };
    return OPTION_DEF_ERROR_OK;
}

namespace {

using PluginEnumValues = std::vector<std::pair<std::string, std::string>>;

bool plugin_enum_has_value(const PluginEnumValues &values, const std::string &value)
{
    return std::any_of(values.begin(), values.end(),
                       [&value](const std::pair<std::string, std::string> &entry) {
                           return entry.first == value;
                       });
}

// Snapshot the visible value/label pairs of one enum definition.
PluginEnumValues option_def_enum_values(const ConfigOptionDef &def)
{
    PluginEnumValues out;
    if (def.enum_def.get() == nullptr)
        return out;

    const std::vector<std::string> &values = def.enum_def->values();
    const std::vector<std::string> &labels = def.enum_def->labels();
    out.reserve(values.size());
    for (size_t index = 0; index < values.size(); ++index)
        out.emplace_back(values[index], index < labels.size() ? labels[index] : values[index]);
    return out;
}

} // namespace

// Record one deferred option definition patch. Nothing is applied until
// finalize_plugin_option_defs() runs, so patches stay valid when a target key is
// created by a later plugin.
option_def_error_code Orchestrator::modify_print_config(const raw_option_def_patch *patch)
{
    if (patch == nullptr || patch->opt_key == nullptr || patch->opt_key[0] == '\0')
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;

    OptionDefPatch stored;
    stored.kind = patch->kind;
    stored.opt_key = patch->opt_key;
    stored.plugin_id = m_initializing_plugin != nullptr ? m_initializing_plugin->get_id() : std::string();

    switch (patch->kind) {
    case RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES: {
        if (patch->enum_values.items == nullptr || patch->enum_values.count == 0)
            return OPTION_DEF_ERROR_INVALID_ARGUMENT;
        for (uint32_t index = 0; index < patch->enum_values.count; ++index) {
            const key_value_string_pair_t &pair = patch->enum_values.items[index];
            if (pair.value == nullptr || pair.value[0] == '\0')
                return OPTION_DEF_ERROR_INVALID_ARGUMENT;
            stored.enum_values.emplace_back(pair.value, pair.label != nullptr ? pair.label : pair.value);
        }
        stored.prepend = patch->prepend != 0;
        break;
    }
    case RAW_OPTION_DEF_PATCH_REMOVE_ENUM_VALUES: {
        if (patch->remove_values.items == nullptr || patch->remove_values.size == 0)
            return OPTION_DEF_ERROR_INVALID_ARGUMENT;
        for (size_t index = 0; index < patch->remove_values.size; ++index) {
            const char *value = patch->remove_values.items[index];
            if (value == nullptr || value[0] == '\0')
                return OPTION_DEF_ERROR_INVALID_ARGUMENT;
            stored.remove_values.emplace_back(value);
        }
        break;
    }
    case RAW_OPTION_DEF_PATCH_SET_DEFAULT:
        if (patch->default_serialized_value == nullptr)
            return OPTION_DEF_ERROR_INVALID_ARGUMENT;
        stored.default_serialized_value = patch->default_serialized_value;
        break;
    case RAW_OPTION_DEF_PATCH_REPLICATE_ENUM:
        if (patch->source_option_key == nullptr || patch->source_option_key[0] == '\0')
            return OPTION_DEF_ERROR_INVALID_ARGUMENT;
        stored.source_option_key = patch->source_option_key;
        break;
    default:
        return OPTION_DEF_ERROR_INVALID_ARGUMENT;
    }

    m_option_def_patches[stored.opt_key].emplace_back(std::move(stored));
    return OPTION_DEF_ERROR_OK;
}

// Tighten the invalidation of settings read by active plugins, then apply every
// recorded patch in dependency order, prune plugin-id lists and rebuild the
// scripted enum maps. Missing target keys are ignored.
void Orchestrator::finalize_plugin_option_defs()
{
    ConfigDef &definition = PrintConfigDef::instance_mutable();

    // initialize_plugins() has already disabled every plugin whose used-key
    // contract failed, so only real consumers move a definition. This must run
    // before the early return below, which only concerns enum patches.
    for (const std::unique_ptr<Plugin> &plugin : m_registered_plugins)
        if (this->is_plugin_active(plugin.get()))
            tighten_used_config_key_invalidations(definition, *plugin);

    // Resolve a key after its dependencies: replication patches create edges to
    // the source option, so the source is finalized first (leaves first in the
    // dependency tree). A key already on the current path breaks the cycle
    // softly: the edge is ignored and each option still resolves once.
    std::map<std::string, PluginEnumValues> base_values;
    std::set<std::string> target_keys;
    for (const std::pair<const std::string, ConfigOptionDef> &entry : definition.options) {
        if (!entry.second.contains_plugin_id)
            continue;
        target_keys.insert(entry.first);
        base_values[entry.first] = option_def_enum_values(entry.second);
    }
    for (const std::pair<const std::string, std::vector<OptionDefPatch>> &entry : m_option_def_patches) {
        target_keys.insert(entry.first);
        const std::map<std::string, PluginEnumValues>::iterator base = base_values.find(entry.first);
        if (base == base_values.end()) {
            const ConfigOptionDef *def = definition.get(entry.first);
            if (def != nullptr)
                base_values[entry.first] = option_def_enum_values(*def);
        }
    }
    if (target_keys.empty())
        return;

    std::map<std::string, PluginEnumValues> resolved;
    std::set<std::string> resolving;

    std::function<const PluginEnumValues &(const std::string &)> resolve =
        [&](const std::string &key) -> const PluginEnumValues & {
            const std::map<std::string, PluginEnumValues>::iterator done = resolved.find(key);
            if (done != resolved.end())
                return done->second;

            static const PluginEnumValues s_empty;
            if (!resolving.insert(key).second) {
                BOOST_LOG_TRIVIAL(warning)
                    << "Plugin option definition dependency cycle detected at '" << key
                    << "'; the cyclic edge is ignored.";
                return s_empty;
            }

            PluginEnumValues effect;
            const std::map<std::string, PluginEnumValues>::const_iterator base = base_values.find(key);
            if (base != base_values.end()) {
                effect = base->second;
            } else if (const ConfigOptionDef *source_def = definition.get(key)) {
                // A replication source that is not itself a patch target still
                // starts from its current definition values.
                effect = option_def_enum_values(*source_def);
            }

            std::string requested_default;
            const std::map<std::string, std::vector<OptionDefPatch>>::const_iterator patches =
                m_option_def_patches.find(key);
            if (patches != m_option_def_patches.end()) {
                // Replication replaces the whole list, so it must run before
                // every contribution no matter which plugin initialized first.
                // Otherwise a contribution recorded before the target existed
                // would be erased by a later replication patch.
                for (const OptionDefPatch &patch : patches->second) {
                    if (patch.kind != RAW_OPTION_DEF_PATCH_REPLICATE_ENUM)
                        continue;

                    // A source that is still being resolved is an ancestor in
                    // the dependency tree: ignore the edge and keep the current
                    // values instead of erasing them.
                    if (resolving.find(patch.source_option_key) != resolving.end()) {
                        BOOST_LOG_TRIVIAL(warning)
                            << "Ignored cyclic plugin option definition edge '"
                            << key << "' -> '" << patch.source_option_key << "'.";
                        continue;
                    }
                    // Snapshot the source now that its own patches applied.
                    effect = resolve(patch.source_option_key);
                }

                for (const OptionDefPatch &patch : patches->second) {
                    switch (patch.kind) {
                    case RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES:
                        for (const std::pair<std::string, std::string> &entry : patch.enum_values) {
                            if (plugin_enum_has_value(effect, entry.first))
                                continue;
                            if (patch.prepend)
                                effect.insert(effect.begin(), entry);
                            else
                                effect.emplace_back(entry);
                        }
                        break;
                    case RAW_OPTION_DEF_PATCH_REMOVE_ENUM_VALUES:
                        effect.erase(
                            std::remove_if(effect.begin(), effect.end(),
                                           [&patch](const std::pair<std::string, std::string> &entry) {
                                               return std::find(patch.remove_values.begin(),
                                                                patch.remove_values.end(),
                                                                entry.first) != patch.remove_values.end();
                                           }),
                            effect.end());
                        break;
                    case RAW_OPTION_DEF_PATCH_SET_DEFAULT:
                        requested_default = patch.default_serialized_value;
                        break;
                    case RAW_OPTION_DEF_PATCH_REPLICATE_ENUM:
                        // Applied in the first pass.
                        break;
                    default:
                        break;
                    }
                }
            }

            ConfigOptionDef *def = nullptr;
            t_optiondef_map::iterator def_it = definition.options.find(key);
            if (def_it != definition.options.end())
                def = &def_it->second;

            const bool contains_plugin_id = def != nullptr && def->contains_plugin_id;
            if (contains_plugin_id) {
                effect.erase(
                    std::remove_if(effect.begin(), effect.end(),
                                   [this](const std::pair<std::string, std::string> &entry) {
                                       const Plugin *plugin = this->get_plugin(entry.first);
                                       return plugin == nullptr || !this->is_plugin_active(plugin);
                                   }),
                    effect.end());

                if (effect.empty()) {
                    const std::string message = "Plugin-id option '" + key +
                        "' has no active value; the setting is left empty.";
                    BOOST_LOG_TRIVIAL(error) << message;
                    this->add_plugin_message(PluginMessageLevel::Error, nullptr, STEP_NONE, message.c_str());
                }
            }

            if (def != nullptr && def->enum_def.get() != nullptr) {
                // Plugin-created enums own a scripted map that must be rebuilt
                // whenever their value set changes.
                if (def->enum_is_scripted() || contains_plugin_id) {
                    // Read the current default before rebuilding the map: a
                    // ConfigOptionEnumGeneric borrows the map pointer, which
                    // set_enum_as_closed_for_scripted_enum() releases.
                    std::string desired = requested_default;
                    if (desired.empty() && def->default_value)
                        desired = def->default_value->serialize();
                    if (!desired.empty() && desired.front() == '!')
                        desired.erase(desired.begin());

                    def->set_enum_as_closed_for_scripted_enum(effect);

                    if (!plugin_enum_has_value(effect, desired))
                        desired = effect.empty() ? std::string() : effect.front().first;

                    if (!desired.empty()) {
                        ConfigOption *default_option = def->create_empty_option();
                        if (default_option->deserialize(desired))
                            def->set_default_value(default_option);
                        else
                            delete default_option;
                    } else {
                        // The rebuilt list is empty: replace the default too so
                        // it does not keep borrowing the released map.
                        def->set_default_value(def->create_empty_option());
                    }
                } else {
                    BOOST_LOG_TRIVIAL(warning)
                        << "Ignored plugin option patches for '" << key
                        << "': the setting is not a plugin-defined enum.";
                }
            }

            resolving.erase(key);
            const std::map<std::string, PluginEnumValues>::iterator stored =
                resolved.emplace(key, std::move(effect)).first;
            return stored->second;
        };

    for (const std::string &key : target_keys)
        resolve(key);
}

// Return a priority-ordered snapshot of every plugin registered for a step.
std::vector<Plugin *> Orchestrator::get_all_plugins_for_step(slicing_step_t step) const {
    std::vector<Plugin *> list;
    for (const std::unique_ptr<Plugin> &plugin : m_registered_plugins) {
        if (plugin->get_step() == step)
            list.push_back(plugin.get());
    }
    std::stable_sort(list.begin(), list.end(), [](const Plugin *lhs, const Plugin *rhs) {
        return lhs->get_priority() < rhs->get_priority();
    });
    return list;
}

// Filter the step snapshot to plugins currently marked active.
std::vector<Plugin *> Orchestrator::get_active_plugins_for_step(slicing_step_t step) const {
    std::vector<Plugin *> list;
    for (Plugin *plugin : this->get_all_plugins_for_step(step))
        if (this->is_plugin_active(plugin))
            list.push_back(plugin);
    return list;
}

// Return active plugins in the step's current execution order.
std::vector<Plugin *> Orchestrator::get_current_plugins_for_step(slicing_step_t step) const {
    auto it = m_plugins_by_step.find(step);
    if (it == m_plugins_by_step.end())
        return {};

    std::vector<Plugin *> plugins;
    plugins.reserve(it->second.size());
    for (Plugin *plugin : it->second)
        if (this->is_plugin_active(plugin))
            plugins.push_back(plugin);
    return plugins;
}

// Perform a linear ID lookup and return a borrowed pointer when found.
const Plugin *Orchestrator::get_plugin(const std::string &plugin_id) const {
    const Plugin *plugin = nullptr;
    for (const std::unique_ptr<Plugin> &plugin_test : m_registered_plugins) {
        if (plugin_test->get_id() == plugin_id) {
            plugin = plugin_test.get();
            break;
        }
    }
    return plugin;
}

// Perform the mutable form of the ID lookup without transferring ownership.
Plugin *Orchestrator::get_plugin(const std::string &plugin_id) {
    Plugin *plugin = nullptr;
    for (const std::unique_ptr<Plugin> &plugin_test : m_registered_plugins) {
        if (plugin_test->get_id() == plugin_id) {
            plugin = plugin_test.get();
            break;
        }
    }
    return plugin;
}

// Validate an opaque public plugin handle without trusting memory owned by a
// different orchestrator.
bool Orchestrator::owns_plugin(const Plugin *plugin) const
{
    if (plugin == nullptr)
        return false;
    for (const std::unique_ptr<Plugin> &registered : m_registered_plugins)
        if (registered.get() == plugin)
            return true;
    return false;
}

// Test membership in the active set without dereferencing the handle.
bool Orchestrator::is_plugin_active(const Plugin *plugin) const
{
    return plugin != nullptr && m_active_plugins.find(const_cast<Plugin *>(plugin)) != m_active_plugins.end();
}

// Resolve the ID and test the resulting handle without changing activation.
bool Orchestrator::is_plugin_active(const std::string &plugin_id) const
{
    return this->is_plugin_active(this->get_plugin(plugin_id));
}

// Empty the active set and discard dependency-blocking diagnostics.
void Orchestrator::clear_active_plugins()
{
    m_active_plugins.clear();
    m_blocked_plugin_activations.clear();
}

// Insert or erase a non-null handle. This internal overload assumes that the
// caller obtained the plugin from this orchestrator; it does not validate
// ownership itself.
bool Orchestrator::set_plugin_active(Plugin *plugin, bool active)
{
    if (plugin == nullptr)
        return false;

    if (active)
        m_active_plugins.insert(plugin);
    else
        m_active_plugins.erase(plugin);
    return true;
}

// Resolve the ID and delegate activation to the handle overload.
bool Orchestrator::set_plugin_active(const std::string &plugin_id, bool active)
{
    return this->set_plugin_active(this->get_plugin(plugin_id), active);
}

std::vector<std::string> Orchestrator::active_plugin_dependency_errors() const
{
    // Sort and deduplicate diagnostics independently of pointer ordering in
    // the active set, so startup logs and GUI messages remain deterministic.
    std::set<std::string> errors;
    for (const Plugin *plugin : m_active_plugins) {
        for (const std::string &dependency_id : plugin->get_dependencies()) {
            const Plugin *dependency = get_plugin(dependency_id);
            if (dependency == nullptr)
                errors.insert("Active plugin '" + plugin->get_id() + "' requires plugin '" + dependency_id + "', which is not loaded.");
            else if (!is_plugin_active(dependency))
                errors.insert("Active plugin '" + plugin->get_id() + "' requires plugin '" + dependency_id + "', which is inactive.");
        }
    }
    return {errors.begin(), errors.end()};
}

// Expand dependencies iteratively; on a missing plugin, clear the output and return false.
bool Orchestrator::plugin_dependency_closure(const std::vector<std::string> &ids,
                                            std::vector<std::string> &closure, std::string &error) const
{
    std::set<std::string> visited;
    std::vector<std::string> pending = ids;
    closure.clear();
    error.clear();
    for (size_t index = 0; index < pending.size(); ++index) {
        const std::string id = pending[index];
        if (!visited.insert(id).second)
            continue;
        const Plugin *plugin = get_plugin(id);
        if (plugin == nullptr) {
            error = "Required plugin '" + id + "' is not loaded.";
            return false;
        }
        for (const std::string &dependency : plugin->get_dependencies()) {
            if (get_plugin(dependency) == nullptr) {
                error = "Plugin '" + id + "' requires plugin '" + dependency + "', which is not loaded.";
                return false;
            }
            pending.push_back(dependency);
        }
    }
    closure.assign(visited.begin(), visited.end());
    return true;
}

// Validate and merge one activation-solidarity declaration without partially
// mutating the accumulated groups when the declaration is invalid.
bool Orchestrator::register_activation_group(const std::string &id,
    const std::vector<std::string> &members, std::string &error)
{
    error.clear();
    std::set<std::string> unique;
    if (id.empty() || members.size() < 2) {
        error = "An activation group needs a nonempty ID and at least two distinct members.";
        return false;
    }
    for (const std::string &member : members) {
        if (member.empty() || !unique.insert(member).second) {
            error = "Activation group '" + id + "' contains an empty or repeated member: '" + member + "'.";
            return false;
        }
    }
    // Validate the entire declaration before merging, so invalid calls cannot
    // partially extend a previously valid group.
    const std::map<std::string, std::set<std::string>>::const_iterator existing = m_activation_groups.find(id);
    if (existing != m_activation_groups.end())
        unique.insert(existing->second.begin(), existing->second.end());
    m_activation_groups.insert_or_assign(id, std::move(unique));
    return true;
}

// Report groups containing IDs that are not registered yet.
std::map<std::string, std::string> Orchestrator::incomplete_activation_groups() const
{
    std::map<std::string, std::string> errors;
    for (const auto &[group, members] : m_activation_groups) {
        std::string missing;
        for (const std::string &member : members)
            if (get_plugin(member) == nullptr)
                missing += "\n" + member;
        if (!missing.empty())
            errors[group] = "Activation group '" + group + "' has missing plugins:" + missing;
    }
    return errors;
}

// Validate dependencies and group membership for a proposed complete selection.
std::map<std::string, std::string> Orchestrator::plugin_activation_errors(
    const std::set<std::string> &selected) const
{
    std::map<std::string, std::string> errors;
    for (const std::string &id : selected) {
        const Plugin *plugin = get_plugin(id);
        if (plugin == nullptr) {
            errors[id] = "Plugin '" + id + "' is not loaded.";
            continue;
        }
        for (const std::string &dependency : plugin->get_dependencies()) {
            if (selected.count(dependency) == 0 || get_plugin(dependency) == nullptr) {
                if (!errors[id].empty()) errors[id] += "\n";
                errors[id] += "Plugin '" + id + "' requires plugin '" + dependency +
                    (get_plugin(dependency) == nullptr ? "', which is not loaded." : "', which is inactive.");
            }
        }
    }
    for (const auto &[group, members] : m_activation_groups) {
        std::string unavailable;
        for (const std::string &member : members)
            if (selected.count(member) == 0 || get_plugin(member) == nullptr)
                unavailable += "\n" + member + (get_plugin(member) == nullptr ? " (not loaded)" : " (inactive)");
        if (unavailable.empty()) continue;
        for (const std::string &member : members) {
            if (selected.count(member) == 0) continue;
            if (!errors[member].empty()) errors[member] += "\n";
            errors[member] += "Plugin '" + member + "' belongs to activation group '" + group +
                "', whose required members are unavailable:" + unavailable;
        }
    }
    return errors;
}

Orchestrator::ActivationChange Orchestrator::propose_plugin_activation(
    const std::vector<std::string> &current, const std::vector<std::string> &requested, bool active) const
{
    ActivationChange result;
    const std::set<std::string> original(current.begin(), current.end());
    const std::set<std::string> explicit_ids(requested.begin(), requested.end());
    std::set<std::string> affected = explicit_ids;
    size_t previous_size;
    do {
        previous_size = affected.size();
        // Expand whole groups before traversing dependency edges. Repeating
        // handles overlapping groups and dependencies that join other groups.
        for (const auto &[group, members] : m_activation_groups) {
            if (std::none_of(members.begin(), members.end(), [&affected](const std::string &id) {
                    return affected.count(id) != 0;
                })) continue;
            result.groups.insert(group);
            affected.insert(members.begin(), members.end());
            result.solidarity_changes.insert(members.begin(), members.end());
        }
        if (active) {
            const std::set<std::string> pass = affected;
            for (const std::string &id : pass) {
                const Plugin *plugin = get_plugin(id);
                if (plugin == nullptr) continue;
                for (const std::string &dependency : plugin->get_dependencies()) {
                    affected.insert(dependency);
                    result.dependency_changes.insert(dependency);
                }
            }
        } else {
            std::set<std::string> remaining = original;
            for (const std::string &id : affected) remaining.erase(id);
            for (const auto &[id, reason] : plugin_activation_errors(remaining)) {
                // Preserve unrelated unavailable configured IDs for repair.
                if (get_plugin(id) == nullptr) continue;
                affected.insert(id);
                result.dependency_changes.insert(id);
            }
        }
    } while (affected.size() != previous_size);

    result.selected = original;
    if (active) {
        for (const auto &[id, reason] : plugin_activation_errors(affected)) {
            if (!result.error.empty()) result.error += "\n";
            result.error += reason;
        }
        if (result.error.empty()) result.selected.insert(affected.begin(), affected.end());
    } else {
        for (const std::string &id : affected) result.selected.erase(id);
    }
    // The confirmation lists only changes beyond the explicit checkbox action.
    for (const std::string &id : affected) {
        const bool additional = explicit_ids.count(id) == 0 &&
            (active ? original.count(id) == 0 : original.count(id) != 0);
        if (!additional) {
            result.solidarity_changes.erase(id);
            result.dependency_changes.erase(id);
        } else if (result.solidarity_changes.count(id) != 0) {
            result.dependency_changes.erase(id);
        }
    }
    return result;
}

void Orchestrator::block_unsatisfied_plugin_dependencies(const std::vector<std::string> &requested)
{
    m_blocked_plugin_activations.clear();
    // Missing requested members never entered m_active_plugins. Include them
    // once so an entirely absent but requested group still produces a notice.
    std::set<std::string> missing_requests;
    for (const std::string &id : requested) {
        if (get_plugin(id) != nullptr) continue;
        for (const auto &[group, members] : m_activation_groups)
            if (members.count(id) != 0) missing_requests.insert(id);
    }
    // Remove one complete generation at a time: a consumer of a rejected
    // plugin is rejected on the next pass, regardless of registration order.
    bool changed;
    do {
        std::set<std::string> selected;
        for (const Plugin *plugin : m_active_plugins) selected.insert(plugin->get_id());
        selected.insert(missing_requests.begin(), missing_requests.end());
        missing_requests.clear();
        const std::map<std::string, std::string> rejected = plugin_activation_errors(selected);
        changed = !rejected.empty();
        for (const auto &[id, reason] : rejected) {
            set_plugin_active(id, false);
            m_blocked_plugin_activations[id] = reason;
        }
    } while (changed);
}

// Insert the plugin into a step while respecting dependency order and priority.
void Orchestrator::add_plugin_to_step(Plugin *plugin, slicing_step_t step) {
    if (!plugin) {
        return;
    }
    auto it_unique = m_plugins_by_step.find(step);
    if (it_unique != m_plugins_by_step.end()) {
        // replace
        it_unique->second = {plugin};
    } else {
        // create new
        std::vector<Plugin *> &plugins = m_plugins_by_step[step];
        // check not already inside, if so remove it.
        plugins.erase(std::remove(plugins.begin(), plugins.end(), plugin), plugins.end());
        // search best place
        // search from end until a dep is found
        // we have to put it between plugins[i-1] and plugins[i]
        size_t i = plugins.size();
        const std::vector<std::string> &deps = plugin->get_dependencies();
        for (; i > 0; i--) {
            const std::string candidate_id = plugins[i - 1]->get_id();
            for (const std::string &dep : deps) {
                if (candidate_id == dep) {
                    goto found_dep;
                }
            }
        }
    found_dep:
        // now move forward until the  priority is higher than our plugin's
        for (; i < plugins.size(); i++) {
            if (plugins[i]->get_priority() > plugin->get_priority()) {
                break;
            }
        }
        plugins.insert(plugins.begin() + i, plugin);
    }
}

// Delegate one synchronous print through the configured step pipeline.
void Orchestrator::slice(Print &print) {
    Steps::StepPipeline::run_slice(*this, print);
}

std::string Orchestrator::export_gcode(Print &print,
                                       const std::string &path_template,
                                       GCodeProcessorResult *result,
                                       ThumbnailsGeneratorCallback thumbnail_cb)
{
    // output everything to a G-code file
    // The following call may die if the output_filename_format template substitution fails.
    const std::string path = print.output_filepath(path_template);
    // Re-export through the worker path: the UI cancellation callback would wait for this very thread.
    print.request_replay_from_worker(Steps::Checkpoint::Plan);
    GCodeThumbnailRenderer thumbnail_renderer(print, std::move(thumbnail_cb));
    const std::string reader_plugin_id = Steps::StepPipeline::run_gcode(
        *this, print, path, thumbnail_renderer.c_renderer(), result);

    if (result != nullptr) {
        result->reader_plugin_id = reader_plugin_id;
        if (print.conflict_result())
            result->conflict_result = *print.conflict_result();
    }

    print.set_status(100, "", PrintBase::SlicingStatus::DEFAULT | PrintBase::SlicingStatus::SECONDARY_STATE);
    print.set_status(100, L("Gcode done"), PrintBase::SlicingStatus::FlagBits::GCODE_ENDED);
    return path;
}

// Initialize active plugins and then validate their read-side configuration contracts.
void Orchestrator::initialize_plugins() {
    for (const std::unique_ptr<Plugin> &plugin_ptr : m_registered_plugins) {
        if (this->is_plugin_active(plugin_ptr.get())) {
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            // Plugin initialization is allowed to register dynamic options,
            // GUI fragments and GUI rules. Keep the current plugin in the
            // orchestrator so those registration paths can attach diagnostics
            // to the responsible plugin and, for option definitions, disable
            // only that plugin if validation fails.
            m_initializing_plugin = plugin_ptr.get();
            m_initializing_plugin_failed = false;
            m_initializing_plugin_failure.clear();
            plugin_ptr->initialize(reinterpret_cast<storage_handle *>(&m_plugin_storage[plugin_ptr.get()]));
            if (m_initializing_plugin_failed) {
                this->set_plugin_active(plugin_ptr.get(), false);
                BOOST_LOG_TRIVIAL(error) << "Disabled plugin '" << plugin_ptr->get_id()
                                         << "' after initialization failure: "
                                         << m_initializing_plugin_failure;
            } else {
                BOOST_LOG_TRIVIAL(debug) << "Initialized plugin '" << plugin_ptr->get_id() << "' in "
                                         << elapsed_ms(start).count() << " ms.";
            }
            m_initializing_plugin = nullptr;
            m_initializing_plugin_failed = false;
            m_initializing_plugin_failure.clear();
        }
    }

    for (const std::unique_ptr<Plugin> &plugin_ptr : m_registered_plugins) {
        if (!this->is_plugin_active(plugin_ptr.get()))
            continue;

        // Validate read-side config contracts after every active plugin had a
        // chance to publish its option definitions. This allows a plugin to
        // depend on settings created by another active plugin without owning or
        // re-declaring those settings itself.
        for (const Plugin::UsedConfigKey &used_key : plugin_ptr->get_used_config_keys()) {
            std::string failure;
            if (!validate_used_config_key_definition(*plugin_ptr, used_key, failure)) {
                this->set_plugin_active(plugin_ptr.get(), false);
                const std::string message = failure + " Plugin will be disabled.";
                BOOST_LOG_TRIVIAL(error) << "Plugin '" << plugin_ptr->get_id() << "': " << message;
                // This validation runs after plugin initialization, so plugin
                // code cannot report the failure itself. Queue the diagnostic
                // here to make the automatic deactivation visible in the GUI.
                this->add_plugin_message(PluginMessageLevel::Error,
                                         plugin_ptr.get(),
                                         plugin_ptr->get_step(),
                                         message.c_str());
                break;
            }
        }
    }
}

// Start with empty plugin-owned storage.
PluginStorage::PluginStorage() = default;
// Every Entry in the registry destroys its object, so destroying the map releases all allocations.
PluginStorage::~PluginStorage() = default;

// Move the whole registry out under the lock and let the local map destroy the objects after unlocking;
// call only when no callback or handle reader can still use them.
void PluginStorage::clear() {
    std::unordered_map<void *, Entry> released;
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        released.swap(m_entries);
        m_first_extrusion = nullptr;
        m_last_extrusion = nullptr;
    }
}

// Read the registry under the lock; null or untracked exported handles return false without inspecting the object.
bool PluginStorage::contains(void *ptr) const {
    const std::lock_guard<std::mutex> lock(m_mutex);
    return ptr != nullptr && m_entries.find(ptr) != m_entries.end();
}

// Read the tracked allocation count under the lock; the count may change as soon as another callback allocates or frees.
size_t PluginStorage::size() const {
    const std::lock_guard<std::mutex> lock(m_mutex);
    return m_entries.size();
}

// Copy extrusion pointers in insertion order while holding the registry lock; callers must keep these objects alive during inspection, and vector allocation can throw.
std::vector<const ExtrusionEntity *> PluginStorage::stored_extrusions() const
{
    const std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<const ExtrusionEntity *> result;
    for (const Entry *entry = m_first_extrusion; entry != nullptr; entry = entry->next_extrusion)
        result.push_back(static_cast<const ExtrusionEntity *>(entry->object));
    return result;
}

// Append to the tail of the extrusion chain so iteration follows insertion order.
// Preconditions: m_mutex is held and the entry is not in the chain (a freshly stored extrusion).
void PluginStorage::link_extrusion(Entry &entry)
{
    entry.is_extrusion = true;
    entry.prev_extrusion = m_last_extrusion;
    entry.next_extrusion = nullptr;
    if (m_last_extrusion != nullptr)
        m_last_extrusion->next_extrusion = &entry;
    else
        m_first_extrusion = &entry;
    m_last_extrusion = &entry;
}

// Splice the entry out of the doubly linked extrusion chain, fixing the chain ends when needed.
// Preconditions: m_mutex is held and the entry is a linked extrusion entry (is_extrusion is true).
void PluginStorage::unlink_extrusion(Entry &entry)
{
    if (entry.prev_extrusion != nullptr)
        entry.prev_extrusion->next_extrusion = entry.next_extrusion;
    else
        m_first_extrusion = entry.next_extrusion;
    if (entry.next_extrusion != nullptr)
        entry.next_extrusion->prev_extrusion = entry.prev_extrusion;
    else
        m_last_extrusion = entry.prev_extrusion;
    entry.prev_extrusion = nullptr;
    entry.next_extrusion = nullptr;
    entry.is_extrusion = false;
}

// Look the handle up, detach its entry (and chain link) under the lock, then destroy the object once the
// lock is released so a destructor never runs under it. The entry is invisible to contains(), size() and
// stored_extrusions() from the unlock on; a reader still using the object while another callback frees it
// is not protected (forbidden by the run-context contract).
// Null, unknown or already freed handles are programming errors: assert in Debug, return false otherwise.
bool PluginStorage::free(void *ptr) {
    std::unordered_map<void *, Entry>::node_type node;
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        const std::unordered_map<void *, Entry>::iterator found = ptr == nullptr ? m_entries.end() : m_entries.find(ptr);
        if (found == m_entries.end()) {
            assert(false);
            return false;
        }
        if (found->second.is_extrusion)
            unlink_extrusion(found->second);
        node = m_entries.extract(found);
    }
    // `node` dies here, outside the lock: its Entry destroys the object.
    return true;
}

} // namespace Slic3r
