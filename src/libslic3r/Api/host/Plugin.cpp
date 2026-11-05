///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "Plugin.hpp"

#include "Orchestrator.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>

/*
This file wraps a plugin's functions and copies its descriptions for the application.
A plugin supplies functions to the host, the slicer application, through a C
ABI (binary calling interface). A callback is one such function; its context
pointer identifies the plugin object without transferring ownership.

Construct Plugin from plugin_instance, which pairs that context with
plugin_vtable, a function table with an explicit byte count. The constructor validates required
functions, copies available functions and descriptive strings, and reads no
optional field beyond the supplied size.

Read identity and group membership with get_id() and get_exclusive_group().
get_priority() controls execution ordering; get_exclusive_group_priority()
ranks alternatives for the same job. After registration, initialize() returns
a diagnostic id; setup(), setup_run() and run() prepare and execute jobs.
All calls still borrow plugin-owned context and code, which must remain alive.
The host context links a callback to one execution's progress and stop state.
setup(), setup_run() and run() install this link on their calling thread.
Nested calls to host services then observe that execution's stop state.

Operation notes:
- initialize / setup / setup_run / run

Representative direct calls (branches are not a timeline):

    Plugin::Plugin()
        |-- validate_plugin_instance()
        |-- std::memcpy()
        |-- read_plugin_used_config_keys()
        `-- read_plugin_defined_config_keys()

    Plugin::setup()
        |-- bind_run()
        |-- note_plugin_setup()
        `-- m_c_api.vt->setup()

    Plugin::run()
        |-- bind_run()
        `-- m_c_api.vt->run()
*/

namespace Slic3r {

namespace {

const int32_t MAX_PLUGIN_CONFIG_KEYS = 1024;

// Check the borrowed table's size and required functions without calling the plugin or reading optional fields.
// Missing functions throw without changing plugin state.
void validate_plugin_instance(const plugin_instance &c_api);

void validate_plugin_instance(const plugin_instance &c_api)
{
    if (c_api.vt == nullptr)
        throw std::runtime_error("Plugin instance has no vtable.");
    if (c_api.vt->struct_size < offsetof(plugin_vtable, run) + sizeof(c_api.vt->run))
        throw std::runtime_error("Plugin vtable has an incomplete required prefix.");
    if (c_api.vt->get_id == nullptr || c_api.vt->get_name == nullptr ||
        c_api.vt->get_description == nullptr ||
        c_api.vt->get_exclusive_group == nullptr ||
        c_api.vt->get_step == nullptr ||
        c_api.vt->get_dependencies == nullptr || c_api.vt->get_priority == nullptr ||
        c_api.vt->used_config_keys == nullptr || c_api.vt->defined_config_keys == nullptr ||
        c_api.vt->initialize == nullptr || c_api.vt->setup == nullptr ||
        c_api.vt->setup_run == nullptr || c_api.vt->run == nullptr)
        throw std::runtime_error("Plugin vtable has missing callbacks.");
}

void read_plugin_used_config_keys(const plugin_instance &c_api,
                                  plugin_used_config_keys_fn callback,
                                  std::vector<Plugin::UsedConfigKey> &out)
{
    // Plugin key lists use a C-style double-call convention. The cap prevents a
    // broken plugin from reserving an arbitrary amount of memory during startup.
    const int32_t key_count = callback(c_api.ctx, nullptr);
    if (key_count < 0 || key_count > MAX_PLUGIN_CONFIG_KEYS)
        throw std::runtime_error("Plugin returned an invalid used_config_keys count.");

    if (key_count == 0)
        return;

    std::vector<raw_used_config_key> keys;
    keys.assign(size_t(key_count), raw_used_config_key{});
    const int32_t written_count = callback(c_api.ctx, keys.data());
    if (written_count < 0 || written_count > key_count)
        throw std::runtime_error("Plugin wrote an invalid used_config_keys count.");

    for (int32_t i = 0; i < written_count; ++i) {
        const raw_used_config_key &key = keys[size_t(i)];
        if (key.key == nullptr)
            continue;
        if (key.type == RAW_CO_NONE)
            throw std::runtime_error(std::string("Plugin declares used config key '") + key.key +
                                     "' without a value type.");
        out.push_back(Plugin::UsedConfigKey{
            key.key,
            key.type,
            key.container_type,
            key.option_preset_type
        });
    }
}

void read_plugin_defined_config_keys(const plugin_instance &c_api,
                                     plugin_defined_config_keys_fn callback,
                                     const std::vector<Plugin::UsedConfigKey> &used_keys,
                                     std::vector<Plugin::DefinedConfigKey> &out)
{
    const int32_t key_count = callback(c_api.ctx, nullptr);
    if (key_count < 0 || key_count > MAX_PLUGIN_CONFIG_KEYS)
        throw std::runtime_error("Plugin returned an invalid defined_config_keys count.");

    if (key_count == 0)
        return;

    std::vector<const char *> keys;
    keys.assign(size_t(key_count), nullptr);
    const int32_t written_count = callback(c_api.ctx, keys.data());
    if (written_count < 0 || written_count > key_count)
        throw std::runtime_error("Plugin wrote an invalid defined_config_keys count.");

    // Defined keys intentionally reuse the richer used-key declaration. This
    // keeps the lightweight activation callback ABI stable while making a
    // setting's type available before an inactive plugin is initialized.
    for (int32_t i = 0; i < written_count; ++i) {
        const char *defined_key = keys[size_t(i)];
        if (defined_key == nullptr)
            continue;

        const Plugin::UsedConfigKey *matching_used_key = nullptr;
        for (const Plugin::UsedConfigKey &used_key : used_keys) {
            if (used_key.key != defined_key)
                continue;
            if (matching_used_key != nullptr && matching_used_key->type != used_key.type)
                throw std::runtime_error(
                    std::string("Plugin declares defined config key '") + defined_key +
                    "' with conflicting used_config_keys types.");
            matching_used_key = &used_key;
        }

        if (matching_used_key == nullptr)
            throw std::runtime_error(
                std::string("Plugin declares defined config key '") + defined_key +
                "' without a matching used_config_keys entry.");

        out.push_back(Plugin::DefinedConfigKey{defined_key, matching_used_key->type});
    }
}

} // namespace

Plugin::Plugin(plugin_instance c_api,
               std::string default_translation_domain,
               std::string package_root)
    : m_c_api(c_api)
    , m_package_root(std::move(package_root))
{
    validate_plugin_instance(c_api);
    std::memcpy(&m_validated_vtable, c_api.vt, offsetof(plugin_vtable, run) + sizeof(c_api.vt->run));
    if (c_api.vt->struct_size >= offsetof(plugin_vtable, clear) + sizeof(c_api.vt->clear))
        m_validated_vtable.clear = c_api.vt->clear;
    if (c_api.vt->struct_size >= offsetof(plugin_vtable, get_exclusive_group_priority) + sizeof(c_api.vt->get_exclusive_group_priority))
        m_validated_vtable.get_exclusive_group_priority = c_api.vt->get_exclusive_group_priority;
    m_validated_vtable.struct_size = sizeof(plugin_vtable);
    m_c_api.vt = &m_validated_vtable;
    c_api.vt = &m_validated_vtable;

    const char *plugin_id = c_api.vt->get_id(c_api.ctx);
    const char *plugin_name = c_api.vt->get_name(c_api.ctx);
    const char *plugin_description = c_api.vt->get_description(c_api.ctx);
    const char *plugin_exclusive_group = c_api.vt->get_exclusive_group(c_api.ctx);
    this->m_id = plugin_id != nullptr ? plugin_id : "";
    this->m_translation_domain = std::move(default_translation_domain);
    if (this->m_translation_domain.empty())
        this->m_translation_domain = this->m_id;
    this->m_name = plugin_name != nullptr && plugin_name[0] != '\0' ? plugin_name : this->m_id;
    this->m_description = plugin_description != nullptr ? plugin_description : "";
    // NULL preserves the legacy singleton fallback. An explicit empty string
    // opts out for addressable services, whose caller selects an id directly
    // instead of reading a generated configuration selector.
    this->m_exclusive_group = plugin_exclusive_group != nullptr ?
        plugin_exclusive_group : this->m_id;
    this->m_step = c_api.vt->get_step(c_api.ctx);
    // Service kinds have no checkpoint-owned execution lifetime to pair with a clear callback.
    if (c_api.vt->clear != nullptr &&
        Steps::execution_index(this->m_step) >= Steps::execution_order().size())
        throw std::runtime_error(
            "Plugin '" + this->m_id +
            "' declares a clear callback without a supported pipeline cleanup boundary.");
    this->m_priority = c_api.vt->get_priority(c_api.ctx);
    if (c_api.vt->get_exclusive_group_priority != nullptr)
        m_exclusive_group_priority = c_api.vt->get_exclusive_group_priority(c_api.ctx);
    const_strings_t cstrings = c_api.vt->get_dependencies(c_api.ctx);
    if (cstrings.items) {
        for (size_t i = 0; i < cstrings.size; ++i) {
            m_dependencies.emplace_back(cstrings.items[i]);
        }
    }

    read_plugin_used_config_keys(c_api, c_api.vt->used_config_keys, m_used_config_keys);
    read_plugin_defined_config_keys(
        c_api, c_api.vt->defined_config_keys, m_used_config_keys, m_defined_config_keys);
}

void Plugin::setup(const plugin_run_context &context, uint32_t run_count) const
{
    Orchestrator::RunScope run_scope = Orchestrator::instance().bind_run(context.host_context);
    // Track before calling setup so partially created caches also receive checkpoint cleanup.
    note_plugin_setup(context.host_context, *this, context.step);
    m_c_api.vt->setup(m_c_api.ctx, &context, run_count);
}

void Plugin::setup_run(const plugin_run_context &context) const
{
    Orchestrator::RunScope run_scope = Orchestrator::instance().bind_run(context.host_context);
    m_c_api.vt->setup_run(m_c_api.ctx, &context);
}

void Plugin::run(const plugin_run_context &context) const
{
    Orchestrator::RunScope run_scope = Orchestrator::instance().bind_run(context.host_context);
    m_c_api.vt->run(m_c_api.ctx, &context);
}

void note_plugin_setup(plugin_host_context *host_context,
                       const Plugin &plugin,
                       slicing_step_t step)
{
    if (host_context == nullptr || host_context->orchestrator == nullptr ||
        host_context->print == nullptr || !plugin.has_clear())
        return;
    // Only the participation record changes; the registered plugin descriptor remains unchanged.
    host_context->orchestrator->track_plugin_setup(
        const_cast<Plugin &>(plugin), *host_context->print, step);
}

} // namespace Slic3r
