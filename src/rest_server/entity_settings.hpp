/*
 * Entity-local setting rules for objects, printable parts, and layer ranges.
 *
 * Scope keys come from finalized FFF configuration definitions, including
 * plugin options owned by the engine's object/region containers. The module
 * reads and validates partial ModelConfig overrides, then publishes a complete
 * candidate only after every update passes. Layer ranges use their owner and
 * exact height bounds as REST entity identity.
 */

#pragma once

#include "scene_edit.hpp"

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r::Rest {

enum class SettingScope {
    object,
    part,
    layer_range
};

// Allowed setting keys per scope, computed once from finalized definitions.
const std::set<std::string> &scope_keys(SettingScope scope);
bool setting_in_scope(SettingScope scope, const std::string &key);
std::vector<std::string> setting_scopes(const std::string &key);

struct SettingsTarget {
    SettingScope scope;
    ModelConfig *config;
    ModelObject *object;
    std::optional<t_layer_height_range> range;
};

struct EntitySettingsChange {
    bool changed = true;
    std::vector<EntityBinding> bindings;
    std::vector<EntityBinding> unbindings;
    std::vector<EntityRebinding> rebindings;
};

// Resolve the entity inside a private scene copy and reject non-settings targets.
SettingsTarget find_settings_target(Model &scene, const EntityReference &entity);
std::vector<std::string> defined_setting_ids(const SettingsTarget &target);
nlohmann::json read_entity_setting(
    const SettingsTarget &target,
    const std::string &key,
    const DynamicPrintConfig &session);
EntitySettingsChange write_entity_settings(
    SettingsTarget &target,
    const std::vector<std::pair<std::string, std::optional<std::string>>> &updates,
    const DynamicPrintConfig &session);

// Count extruder slots in the session configuration, defaulting to one when absent.
std::size_t session_extruder_count(const DynamicPrintConfig &session);
// Nozzle limits are evaluated against the session's selected printer config.
std::pair<double, double> layer_height_limits(const DynamicPrintConfig &session, int extruder);
void validate_layer_range_bounds(
    const t_layer_config_ranges &ranges,
    const t_layer_height_range &candidate,
    const std::optional<t_layer_height_range> &replaced);
EntitySettingsChange add_layer_range(
    ModelObject &object,
    double min,
    double max,
    double layer_height,
    int extruder,
    const DynamicPrintConfig &session);
EntitySettingsChange set_layer_range_bounds(
    ModelObject &object,
    const t_layer_height_range &current,
    double min,
    double max);
EntitySettingsChange delete_layer_range(ModelObject &object, const t_layer_height_range &range);

} // namespace Slic3r::Rest
