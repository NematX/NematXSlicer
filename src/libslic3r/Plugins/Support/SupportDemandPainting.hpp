///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_support_supportdemandpainting_hpp_
#define plugins_support_supportdemandpainting_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Support { namespace SupportDemandPaintingPlugin {

// STEP_SUPPORT_DEMAND refinement plugin.
//
// It applies FDM support painting after automatic overhang demand and before
// support modifier volumes. Painted enforcers are applied first, then painted
// blockers subtract from the result.
class SupportDemandPainting : public PluginBase
{
public:
    static SupportDemandPainting &instance(orchestrator_handle *orch);

private:
    SupportDemandPainting(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    // Reserves one progress unit per object island when support painting exists.
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_support_demand_painting_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Support::SupportDemandPaintingPlugin

#endif // plugins_support_supportdemandpainting_hpp_
