/*
 * REST configuration catalog projection.
 *
 * The catalog exposes finalized libslic3r option definitions as stable JSON
 * metadata without exposing the mutable configuration objects themselves.
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace Slic3r {
class ConfigOption;
class ConfigOptionDef;
class DynamicPrintConfig;
}

namespace Slic3r::Rest {

// True once PrintConfigDef::finalize() has run, meaning the catalog can be built.
bool settings_catalog_ready() noexcept;
// Build the catalog. Throws std::logic_error when the definitions are not
// finalized; the route answers 503 before calling it.
nlohmann::json settings_catalog();

bool setting_affects_slicing(const std::string &setting_id);
// Throws std::invalid_argument when the candidate introduces a cross-setting
// error that the previous configuration did not already have.
void validate_setting_configuration(
    const DynamicPrintConfig &previous,
    const DynamicPrintConfig &candidate);
nlohmann::json setting_value_json(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id);
std::string setting_value_serialized(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id);
std::string setting_json_to_serialized(
    const DynamicPrintConfig &configuration,
    const std::string &setting_id,
    const nlohmann::json &value);
void set_setting_serialized(
    DynamicPrintConfig &configuration,
    const std::string &setting_id,
    const std::string &serialized_value);

// Entity settings use finalized option definitions directly, without filtering
// them through the four preset catalogs used by the session-level routes.
const ConfigOptionDef &require_entity_setting_definition(const std::string &key);
nlohmann::json entity_setting_value_json(
    const ConfigOption &option,
    const ConfigOptionDef &definition,
    const DynamicPrintConfig &session);
std::string entity_setting_json_to_serialized(
    const ConfigOptionDef &definition,
    const nlohmann::json &value,
    const DynamicPrintConfig &session);
void set_entity_setting_serialized(
    DynamicPrintConfig &target,
    const ConfigOptionDef &definition,
    const std::string &serialized,
    const DynamicPrintConfig &session);

} // namespace Slic3r::Rest
