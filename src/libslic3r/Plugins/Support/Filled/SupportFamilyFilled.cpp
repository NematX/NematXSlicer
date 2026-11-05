///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportFamilyFilled.hpp"
#include "FilledSupportGenerator.hpp"

#include <iterator>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Plugins/Support/Common/SupportPublication.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Common/SupportToolpathRoles.hpp"

/* The filled family skips other styles and objects without final demand, then generates and publishes slabs.
   Initialization copies automatic toolpath preferences to the host. PluginBase reports generation errors.
   The process-lifetime provider retains no per-object geometry; the orchestrator owns the run storage. */

namespace slic3r_api::Support::Filled {
namespace {
const raw_used_config_key k_filled_keys[] = {
    {"raft_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
};
}

SupportFamilyFilled &SupportFamilyFilled::instance(orchestrator_handle *orchestrator)
{
    static SupportFamilyFilled provider(orchestrator);
    return provider;
}

void SupportFamilyFilled::inilialize_impl(storage_handle *) const
{
    Common::declare_toolpath_preferences(m_orchestrator, {"filled"},
        Common::legacy_toolpath_preferences(Common::k_legacy_sparse_id));
}

int32_t SupportFamilyFilled::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = Common::append_used_config_keys(keys, 0, Common::support_geometry_keys());
    return Common::append_used_config_keys(keys, count, Common::UsedKeyList{k_filled_keys, std::size(k_filled_keys)});
}

void SupportFamilyFilled::run_impl(const plugin_run_context *context) const
{
    Common::run_support_family(context, m_orchestrator, progress(), Common::SupportStyle::Filled,
                               Common::SupportStyle::Filled, &generate_support);
}

void register_support_family_filled_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, SupportFamilyFilled::instance(orchestrator).c_instance());
}
}
