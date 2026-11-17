///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ArachnePerimeterGenerator.hpp
=============================

Declares the Arachne perimeter generator plugin (STEP_PERIMETER, id
"perimeter.generator.arachne").

The host calls register_arachne_perimeter_generator_plugin() once and then
drives setup_run() and run() for every printable layer island. Each run walks
the host perimeter tree and produces one variable-width Arachne wall pass per
node, so PerimeterGenerationModule plugins can still request extra perimeters or
split child areas between nodes. The loop direction post-process, not this
plugin, gives the closed loops their printing direction.
*/

#ifndef plugins_perimeter_arachneperimetergenerator_hpp_
#define plugins_perimeter_arachneperimetergenerator_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace ArachnePerimeterGeneratorPlugin {

// STEP_PERIMETER implementation backed by libArachne's variable-width wall
// generator. It groups the current layer-island regions, then delegates the
// perimeter tree traversal to the host through run_region_group(). Each host
// node receives one Arachne wall pass, so PerimeterGenerationModule plugins can
// still adjust branch counts and split child areas.
class ArachnePerimeterGenerator : public PluginBase
{
public:
    static ArachnePerimeterGenerator &instance(orchestrator_handle *orch);

private:
    ArachnePerimeterGenerator(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the common perimeter execution priority without changing state or transferring ownership; the call cannot fail.
    int32_t priority_impl() const noexcept override;
    // Return Arachne's rank among alternative perimeter algorithms.
    int32_t exclusive_group_priority_impl() const noexcept override { return 10; }
    const char *progress_message_format_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_arachne_perimeter_generator_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin

#endif // plugins_perimeter_arachneperimetergenerator_hpp_
