///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugin_hpp_
#define slic3r_Plugin_hpp_

#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin.h"
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

// Plugin copies a validated C function table for the slicer application.
// The plugin object and its executable code remain owned by the plugin.
// Call initialize(), then setup(), setup_run() and run().
// These phases prepare the plugin before executing its jobs.
// These calls associate the host's execution state with their calling thread.
// clear() releases a Print's cached data at a saved pipeline boundary or during Print destruction.
// It receives the Print identity and the same persistent storage used during setup().

namespace Slic3r {

class Plugin;

/*
Record that one plugin received setup() for the given step, so a later Print
invalidation can ask it to release the state it created. Only plugins with a
clear callback are tracked; repeated calls for the same plugin and step keep a
single entry.
*/
void note_plugin_setup(plugin_host_context *host_context,
                       const Plugin &plugin,
                       slicing_step_t step);

class Plugin
{
public:
    struct UsedConfigKey
    {
        std::string key;
        raw_config_option_type type = RAW_CO_NONE;
        raw_container_type container_type = RAW_CONTAINER_TYPE_NONE;
        raw_option_preset_type option_preset_type = RAW_PRESET_TYPE_NONE;
    };

    struct DefinedConfigKey
    {
        // The type is resolved from UsedConfigKey during registration, before
        // inactive plugins have published their complete option definitions.
        std::string key;
        raw_config_option_type type = RAW_CO_NONE;
    };

protected:
    // The copied function table borrows plugin state; unavailable optional functions remain null.
    plugin_vtable m_validated_vtable = {};
    plugin_instance m_c_api;
    std::string m_id;
    std::string m_translation_domain;
    // The registration root remains available while initialize() registers
    // option definitions, after the dynamic-library loader scope has ended.
    std::string m_package_root;
    std::string m_name;
    std::string m_description;
    std::string m_exclusive_group;
    slicing_step_t m_step;
    std::vector<std::string> m_dependencies;
    std::vector<UsedConfigKey> m_used_config_keys;
    std::vector<DefinedConfigKey> m_defined_config_keys;
    int m_priority;
    int32_t m_exclusive_group_priority = 100;

public:
    // Check and copy the function table and descriptions while borrowing the plugin object.
    // Slicing data is unchanged; invalid descriptions or allocation failures throw before registration.
    Plugin(plugin_instance c_api,
           std::string default_translation_domain,
           std::string package_root);

    const std::string& get_id() const noexcept { return m_id; }
    const std::string& get_translation_domain() const noexcept { return m_translation_domain; }
    const std::string& get_package_root() const noexcept { return m_package_root; }
    const std::string& get_name() const noexcept { return m_name; }
    const std::string& get_description() const noexcept { return m_description; }
    const std::string& get_exclusive_group() const noexcept { return m_exclusive_group; }
    slicing_step_t get_step() const noexcept { return m_step; }
    const std::vector<std::string>& get_dependencies() const noexcept { return m_dependencies; }
    const std::vector<UsedConfigKey>& get_used_config_keys() const noexcept { return m_used_config_keys; }
    const std::vector<DefinedConfigKey>& get_defined_config_keys() const noexcept { return m_defined_config_keys; }
    int get_priority() const noexcept { return m_priority; }
    // Return the copied rank among alternative plugins.
    int32_t get_exclusive_group_priority() const noexcept { return m_exclusive_group_priority; }
    // True when the plugin opted into per-Print cleanup through clear().
    bool has_clear() const noexcept { return m_c_api.vt->clear != nullptr; }
    // Initialize with borrowed storage and return the plugin's diagnostic id.
    int32_t initialize(storage_handle *storage) const {
        return m_c_api.vt->initialize(m_c_api.ctx, storage);
    }
    // Select the execution on this thread, register Print cleanup, then call the plugin's setup().
    void setup(const plugin_run_context &context, uint32_t run_count) const;
    // Select the supplied execution on this thread while the plugin prepares one job.
    void setup_run(const plugin_run_context &context) const;
    // Select the supplied execution on this thread while the plugin runs one job.
    // Count every run() call, whatever its outcome.
    void run(const plugin_run_context &context) const;
    // An absent callback succeeds without touching the plugin's storage.
    // Returns PLUGIN_ERROR_OK, or a diagnostic id the host resolves through
    // Orchestrator::error_message().
    int32_t clear(const print_handle *print, storage_handle *storage) const {
        if (m_c_api.vt->clear != nullptr)
            return m_c_api.vt->clear(m_c_api.ctx, print, storage);
        return PLUGIN_ERROR_OK;
    }

};

} // namespace Slic3r



#endif // slic3r_plugin_hpp_
