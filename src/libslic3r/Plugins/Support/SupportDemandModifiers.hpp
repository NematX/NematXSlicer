///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_support_supportdemandmodifiers_hpp_
#define plugins_support_supportdemandmodifiers_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Support { namespace SupportDemandModifiersPlugin {

// STEP_SUPPORT_DEMAND refinement plugin.
//
// It applies support modifier volumes after the default overhang demand:
// support enforcers add demand inside their covered islands, support blockers
// remove demand from the final island-keyed support demand set.
class SupportDemandModifiers : public PluginBase
{
public:
    static SupportDemandModifiers &instance(orchestrator_handle *orch);

private:
    SupportDemandModifiers(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    // Reserves one progress unit per object island when support modifier volumes exist.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_support_demand_modifiers_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Support::SupportDemandModifiersPlugin

#endif // plugins_support_supportdemandmodifiers_hpp_
