///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_perimeter_loopdirection_hpp_
#define plugins_perimeter_loopdirection_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace LoopDirectionPlugin {

// STEP_POST_PERIMETER plugin that gives every closed perimeter loop its printing
// direction: perimeter_direction for contours and holes, reversed on odd layers by
// perimeter_reverse, or for the overhanging loops of a steep overhang by
// overhangs_reverse. It runs after overhang detection, whatever generator made the
// loops, and forbids later steps to reverse them. It defines perimeter_reverse.
class LoopDirection : public PluginBase
{
public:
    static LoopDirection &instance(orchestrator_handle *orch);

private:
    LoopDirection(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_loop_direction_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::LoopDirectionPlugin

#endif // plugins_perimeter_loopdirection_hpp_
