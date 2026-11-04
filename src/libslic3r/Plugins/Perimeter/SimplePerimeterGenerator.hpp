///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_perimeter_simpleperimetergenerator_hpp_
#define plugins_perimeter_simpleperimetergenerator_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace SimplePerimeterGeneratorPlugin {

// Fixed-width STEP_PERIMETER plugin. For each perimeter node it offsets the
// node area into concentric closed loops, one shell per node: shell 0 is the
// external perimeter (external role and flow), the following shells are
// internal perimeters (internal role and flow, spaced at the internal
// spacing). It publishes, for each node, the area left inside the loops and the
// slightly larger fill/anchor area that the later steps use.
class SimplePerimeterGenerator : public PluginBase
{
public:
    static SimplePerimeterGenerator &instance(orchestrator_handle *orch);

private:
    SimplePerimeterGenerator(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the common perimeter execution priority without changing state or transferring ownership; the call cannot fail.
    int32_t priority_impl() const noexcept override;
    // Prefer the simple algorithm among enabled perimeter alternatives.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    const char *progress_message_format_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_simple_perimeter_generator_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin

#endif // plugins_perimeter_simpleperimetergenerator_hpp_
