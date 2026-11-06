/*
 * Entity-local setting rules for objects, printable parts, and layer ranges.
 *
 * The REST target stores only explicit overrides; inherited effective values
 * remain in the session configuration and are not presented as entity values.
 * Each write is validated on a DynamicPrintConfig copy before ModelConfig is
 * updated, while layer-range ID changes are described for an atomic session
 * commit.
 */

#include "entity_settings.hpp"

#include "settings.hpp"
#include "libslic3r/Config/ConfigOption.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Slicing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

struct SettingScopeKeys {
    std::set<std::string> object;
    std::set<std::string> part;
    std::set<std::string> layer_range;
};

bool is_fff_preset(std::uint32_t preset)
{
    switch (static_cast<raw_option_preset_type>(preset)) {
    case RAW_PRESET_TYPE_FFF_PRINT:
    case RAW_PRESET_TYPE_FFF_FILAMENT:
    case RAW_PRESET_TYPE_FFF_FILAMENT_OVERRIDE:
    case RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER:
    case RAW_PRESET_TYPE_FFF_TOOL_EXTRUDER_RETRACTION:
    case RAW_PRESET_TYPE_FFF_TOOL_MILLING:
    case RAW_PRESET_TYPE_FFF_PRINTER:
    case RAW_PRESET_TYPE_FFF_PRINTER_MACHINE_LIMITS:
        return true;
    default:
        return false;
    }
}

bool allowed_definition(SettingScope scope, const std::string &key, const ConfigOptionDef *definition)
{
    if (definition == nullptr)
        return false;
    if (definition->category == OptionCategory::none && key != "extruder" && key != "layer_height")
        return false;
    return scope == SettingScope::object || definition->category != OptionCategory::support;
}

// Build stable scope sets after plugin and host definitions are finalized.
SettingScopeKeys make_scope_keys()
{
    if (!settings_catalog_ready())
        throw std::logic_error("REST entity settings require finalized configuration definitions");

    const PrintConfigDef &definitions = PrintConfigDef::instance();
    const t_config_option_keys region_keys = PrintRegionConfig().keys();
    const t_config_option_keys object_keys = PrintObjectConfig().keys();
    std::set<std::string> region;
    std::set<std::string> object_only;
    region.insert(region_keys.begin(), region_keys.end());
    for (const std::string &key : object_keys)
        if (std::find(region_keys.begin(), region_keys.end(), key) == region_keys.end())
            object_only.insert(key);

    for (const std::pair<const t_config_option_key, ConfigOptionDef> &entry : definitions.options) {
        const ConfigOptionDef &definition = entry.second;
        if (!is_fff_preset(definition.option_preset_type))
            continue;
        if (definition.container_type == ConfigOptionContainerType::Layer ||
            definition.container_type == ConfigOptionContainerType::Region)
            region.insert(entry.first);
        else if (definition.container_type == ConfigOptionContainerType::Object)
            object_only.insert(entry.first);
    }

    SettingScopeKeys result;
    for (const std::string &key : region) {
        if (allowed_definition(SettingScope::part, key, definitions.get(key))) {
            result.part.insert(key);
            result.layer_range.insert(key);
        }
    }
    for (const std::string &key : object_only)
        if (allowed_definition(SettingScope::object, key, definitions.get(key)))
            result.object.insert(key);
    for (const std::string &key : region)
        if (allowed_definition(SettingScope::object, key, definitions.get(key)))
            result.object.insert(key);

    if (definitions.get("extruder") != nullptr) {
        result.object.insert("extruder");
        result.part.insert("extruder");
        result.layer_range.insert("extruder");
    }
    if (definitions.get("layer_height") != nullptr) {
        result.object.insert("layer_height");
        result.layer_range.insert("layer_height");
    }
    return result;
}

// Function-local initialization is thread-safe and occurs on the first catalog or entity request.
const SettingScopeKeys &all_scope_keys()
{
    static const SettingScopeKeys keys = make_scope_keys();
    return keys;
}

int range_extruder(const DynamicPrintConfig &configuration)
{
    const ConfigOptionInt *extruder = dynamic_cast<const ConfigOptionInt *>(configuration.option("extruder"));
    if (extruder == nullptr)
        throw SceneEditError(422, "invalid_setting", "A layer range must define an extruder.");
    return extruder->value;
}

double range_layer_height(const DynamicPrintConfig &configuration)
{
    const ConfigOptionFloat *height = dynamic_cast<const ConfigOptionFloat *>(configuration.option("layer_height"));
    if (height == nullptr)
        throw SceneEditError(422, "invalid_setting", "A layer range must define a layer height.");
    return height->value;
}

// Validate the required height override using the selected session's nozzle limits.
void validate_range_layer_height(const DynamicPrintConfig &configuration, const DynamicPrintConfig &session)
{
    const int extruder = range_extruder(configuration);
    const double height = range_layer_height(configuration);
    const std::pair<double, double> limits = layer_height_limits(session, extruder);
    if (!std::isfinite(height) || height < limits.first - EPSILON || height > limits.second + EPSILON)
        throw SceneEditError(422, "invalid_layer_range", "The layer height is outside the nozzle's allowed range.");
}

} // namespace

// Count configured nozzle slots, defaulting to one when none are present.
std::size_t session_extruder_count(const DynamicPrintConfig &session)
{
    const ConfigOption *nozzle_diameter = session.option("nozzle_diameter");
    if (nozzle_diameter != nullptr && nozzle_diameter->is_vector() && nozzle_diameter->size() > 0)
        return nozzle_diameter->size();
    return 1;
}

const std::set<std::string> &scope_keys(SettingScope scope)
{
    const SettingScopeKeys &keys = all_scope_keys();
    switch (scope) {
    case SettingScope::object:
        return keys.object;
    case SettingScope::part:
        return keys.part;
    case SettingScope::layer_range:
        return keys.layer_range;
    }
    throw std::logic_error("Unknown entity setting scope.");
}

bool setting_in_scope(SettingScope scope, const std::string &key)
{
    const std::set<std::string> &keys = scope_keys(scope);
    return keys.find(key) != keys.end();
}

std::vector<std::string> setting_scopes(const std::string &key)
{
    std::vector<std::string> result;
    if (setting_in_scope(SettingScope::object, key))
        result.push_back("object");
    if (setting_in_scope(SettingScope::part, key))
        result.push_back("part");
    if (setting_in_scope(SettingScope::layer_range, key))
        result.push_back("layer_range");
    return result;
}

SettingsTarget find_settings_target(Model &scene, const EntityReference &entity)
{
    if (entity.kind == EntityKind::object) {
        for (ModelObject *object : scene.object_ptrs())
            if (static_cast<std::uint64_t>(object->id().id) == entity.engine_id)
                return {SettingScope::object, &object->config, object, std::nullopt};
        throw std::logic_error("The settings object is not in the scene snapshot.");
    }
    if (entity.kind == EntityKind::volume) {
        const std::pair<ModelObject *, ModelVolume *> target = find_transform_volume(scene, entity);
        if (target.second->type() != ModelVolumeType::MODEL_PART &&
            target.second->type() != ModelVolumeType::PARAMETER_MODIFIER)
            throw SceneEditError(422, "invalid_entity_settings", "Only solid parts and modifiers have entity settings.");
        return {SettingScope::part, &target.second->config, target.first, std::nullopt};
    }
    if (entity.kind == EntityKind::layer_range) {
        const std::pair<ModelObject *, t_layer_config_ranges::iterator> target = find_layer_range(scene, entity);
        return {SettingScope::layer_range, &target.second->second, target.first, entity.range};
    }
    throw SceneEditError(422, "invalid_entity_settings", "Instances do not have entity settings.");
}

std::vector<std::string> defined_setting_ids(const SettingsTarget &target)
{
    std::vector<std::string> result;
    for (const std::string &key : target.config->keys())
        if (setting_in_scope(target.scope, key) && PrintConfigDef::instance().get(key) != nullptr)
            result.push_back(key);
    return result;
}

nlohmann::json read_entity_setting(
    const SettingsTarget &target,
    const std::string &key,
    const DynamicPrintConfig &session)
{
    if (!setting_in_scope(target.scope, key))
        throw SceneEditError(422, "invalid_setting", "This setting is not available for the target entity.");
    const ConfigOptionDef &definition = require_entity_setting_definition(key);
    const ConfigOption *option = target.config->option(key);
    if (option == nullptr)
        throw SceneEditError(404, "setting_not_set", "The entity does not define this setting.");
    return entity_setting_value_json(*option, definition, session);
}

EntitySettingsChange write_entity_settings(
    SettingsTarget &target,
    const std::vector<std::pair<std::string, std::optional<std::string>>> &updates,
    const DynamicPrintConfig &session)
{
    const DynamicPrintConfig previous = target.config->get();
    DynamicPrintConfig candidate(previous);
    for (const std::pair<std::string, std::optional<std::string>> &update : updates) {
        const std::string &key = update.first;
        if (!setting_in_scope(target.scope, key))
            throw SceneEditError(422, "invalid_setting", "This setting is not available for the target entity.");
        const ConfigOptionDef &definition = require_entity_setting_definition(key);
        if (update.second.has_value()) {
            set_entity_setting_serialized(candidate, definition, update.second.value(), session);
            if (key == "extruder") {
                const int extruder = candidate.opt_int(key);
                if (extruder < 0 || static_cast<std::size_t>(extruder) > session_extruder_count(session))
                    throw SceneEditError(422, "invalid_setting", "The extruder must be between zero and the session extruder count.");
            }
        } else {
            if (target.scope == SettingScope::layer_range && (key == "layer_height" || key == "extruder"))
                throw SceneEditError(422, "invalid_setting", "Layer ranges must retain layer_height and extruder.");
            candidate.erase(key);
        }
    }

    if (target.scope == SettingScope::layer_range)
        validate_range_layer_height(candidate, session);
    EntitySettingsChange change;
    if (candidate != previous) {
        target.config->assign_config(std::move(candidate));
        change.changed = true;
    } else {
        change.changed = false;
    }
    return change;
}

std::pair<double, double> layer_height_limits(const DynamicPrintConfig &session, int extruder)
{
    if (extruder < 0 || static_cast<std::size_t>(extruder) > session_extruder_count(session))
        throw SceneEditError(422, "invalid_layer_range", "The extruder is outside the session extruder count.");
    const int nozzle_index = extruder > 0 ? extruder - 1 : 0;
    const double minimum = Slicing::min_layer_height_from_nozzle(session, static_cast<std::uint16_t>(nozzle_index));
    const double maximum = Slicing::max_layer_height_from_nozzle(session, static_cast<std::uint16_t>(nozzle_index));
    return {minimum, maximum};
}

void validate_layer_range_bounds(
    const t_layer_config_ranges &ranges,
    const t_layer_height_range &candidate,
    const std::optional<t_layer_height_range> &replaced)
{
    if (!std::isfinite(candidate.first) || !std::isfinite(candidate.second) ||
        candidate.first < 0. || candidate.first >= candidate.second)
        throw SceneEditError(422, "invalid_layer_range", "Layer range bounds must be finite and satisfy 0 <= min < max.");
    for (const std::pair<const t_layer_height_range, ModelConfig> &range : ranges) {
        if (replaced.has_value() && range.first == replaced.value())
            continue;
        if (candidate.first < range.first.second && range.first.first < candidate.second)
            throw SceneEditError(422, "invalid_layer_range", "Layer ranges cannot overlap.");
    }
}

EntitySettingsChange add_layer_range(
    ModelObject &object,
    double min,
    double max,
    double layer_height,
    int extruder,
    const DynamicPrintConfig &session)
{
    if (extruder < 0 || static_cast<std::size_t>(extruder) > session_extruder_count(session))
        throw SceneEditError(422, "invalid_setting", "The extruder must be between zero and the session extruder count.");
    const t_layer_height_range range{min, max};
    validate_layer_range_bounds(object.layer_config_ranges, range, std::nullopt);
    const std::pair<double, double> limits = layer_height_limits(session, extruder);
    if (!std::isfinite(layer_height) || layer_height < limits.first - EPSILON || layer_height > limits.second + EPSILON)
        throw SceneEditError(422, "invalid_layer_range", "The layer height is outside the nozzle's allowed range.");

    ModelConfig configuration;
    configuration.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    configuration.set_key_value("extruder", new ConfigOptionInt(extruder));
    if (!object.layer_config_ranges.emplace(range, std::move(configuration)).second)
        throw SceneEditError(422, "invalid_layer_range", "A layer range with these bounds already exists.");
    EntitySettingsChange change;
    change.bindings.push_back({EntityKind::layer_range, object.id().id, range});
    return change;
}

EntitySettingsChange set_layer_range_bounds(
    ModelObject &object,
    const t_layer_height_range &current,
    double min,
    double max)
{
    const t_layer_config_ranges::iterator existing = object.layer_config_ranges.find(current);
    if (existing == object.layer_config_ranges.end())
        throw std::logic_error("The layer range is not in the scene snapshot.");
    const t_layer_height_range replacement{min, max};
    if (replacement == current) {
        EntitySettingsChange unchanged;
        unchanged.changed = false;
        return unchanged;
    }
    validate_layer_range_bounds(object.layer_config_ranges, replacement, current);
    ModelConfig configuration = std::move(existing->second);
    object.layer_config_ranges.erase(existing);
    if (!object.layer_config_ranges.emplace(replacement, std::move(configuration)).second)
        throw SceneEditError(422, "invalid_layer_range", "A layer range with these bounds already exists.");
    EntitySettingsChange change;
    change.rebindings.push_back({EntityKind::layer_range, object.id().id, object.id().id, current, replacement});
    return change;
}

EntitySettingsChange delete_layer_range(ModelObject &object, const t_layer_height_range &range)
{
    const t_layer_height_range removed_range = range;
    const t_layer_config_ranges::iterator existing = object.layer_config_ranges.find(removed_range);
    if (existing == object.layer_config_ranges.end())
        throw std::logic_error("The layer range is not in the scene snapshot.");
    object.layer_config_ranges.erase(existing);
    EntitySettingsChange change;
    change.unbindings.push_back({EntityKind::layer_range, object.id().id, removed_range});
    return change;
}

} // namespace Slic3r::Rest
