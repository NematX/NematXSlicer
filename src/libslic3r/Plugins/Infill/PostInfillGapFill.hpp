///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_postinfillgapfill_hpp_
#define plugins_infill_postinfillgapfill_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace PostInfillGapFillPlugin {

// Register this built-in with register_post_infill_gap_fill_plugin(). It completes
// solid, non-bridge surface subtrees after the selected infill pattern has run.
// The source recipe remains on their root, so full-fill volume correction sees
// the original pattern and its gap fill as one surface.
class PostInfillGapFill : public PluginBase
{
public:
    static PostInfillGapFill &instance(orchestrator_handle *orch);

private:
    PostInfillGapFill(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Report the regional activation and geometry settings consumed by this pass.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_post_infill_gap_fill_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::PostInfillGapFillPlugin

#endif // plugins_infill_postinfillgapfill_hpp_
