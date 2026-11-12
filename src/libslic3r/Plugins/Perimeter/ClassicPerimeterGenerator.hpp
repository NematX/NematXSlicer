///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ClassicPerimeterGenerator.hpp
=============================

Declares the classic perimeter generator plugin (STEP_PERIMETER, id
"perimeter.generator.classic").

register_classic_perimeter_generator_plugin() publishes the generator, and the
host then calls setup_run() and run() for every printable layer island. run()
builds the concentric perimeter tree from the host-provided node areas and
publishes the resulting loops and properties, letting PerimeterGenerationModule
plugins adjust counts and child areas between nodes. The plugin owns the classic
generator settings and defines them, with their activation rules, in the .cpp.
*/

#ifndef plugins_perimeter_classicperimetergenerator_hpp_
#define plugins_perimeter_classicperimetergenerator_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace ClassicPerimeterGeneratorPlugin {

// Classic concentric perimeter builder. Every host node is turned into the
// perimeters of its area, and the resulting loops, areas and extrusion
// properties are published for the host and the perimeter modules.
class ClassicPerimeterGenerator : public PluginBase
{
public:
    static ClassicPerimeterGenerator &instance(orchestrator_handle *orch);

private:
    ClassicPerimeterGenerator(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the common perimeter execution priority without changing state or transferring ownership; the call cannot fail.
    int32_t priority_impl() const noexcept override;
    // Prefer Classic over Arachne and Simple when resolving a default provider.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_classic_perimeter_generator_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::ClassicPerimeterGeneratorPlugin

#endif // plugins_perimeter_classicperimetergenerator_hpp_
