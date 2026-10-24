///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_xysizecompensation_hpp_
#define plugins_xysizecompensation_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
Register this native plugin to apply regional XY and first-layer compensation
after volume slicing. Its settings belong to region configurations. The host
calls run_impl() once per object; geometry and publication use only the public
SDK. Registration is separate from activation, for example:

    register_simple_xy_size_compensation_plugin(orchestrator);
*/

namespace slic3r_api { namespace XYSizeCompensationPlugin {

// Post-slicing XY size compensation using only the public C++ geometry SDK.
// The no-collapse routine is a plugin-private port built on generic SDK geometry primitives.
class SimpleXYSizeCompensation final : public PluginBase
{
public:
    // Borrow the provider instance bound to the host's orchestrator.
    static SimpleXYSizeCompensation &instance(orchestrator_handle *orch);

private:
    // Bind registration state; per-run geometry belongs to the caller storage.
    explicit SimpleXYSizeCompensation(orchestrator_handle *orch) : PluginBase(orch) {}

    // Return the stable provider identifier.
    const char *id_impl() const noexcept override;
    // Return borrowed display text.
    const char *name_impl() const noexcept override;
    // Describe the regional FFF compensation behavior.
    const char *description_impl() const noexcept override;
    // Execute after raw volume slicing and before perimeter preparation.
    slicing_step_t step_impl() const noexcept override;
    // Return the empty dependency list; priority sets post-slicing order.
    const char *const *dependencies_impl() const noexcept override;
    // Run dimensional compensation before ordinary post-slicing edits.
    int32_t priority_impl() const noexcept override;
    // Copy declared inputs, or return their count when keys is null.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Copy the five owned setting keys, or return their count when keys is null.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Register regional options once before slicing begins.
    void inilialize_impl(storage_handle *storage) const override;
    // Compute owned layer replacements and publish them after validating destinations.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the native post-slicing plugin with the host orchestrator.
void register_simple_xy_size_compensation_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::XYSizeCompensationPlugin

#endif // plugins_xysizecompensation_hpp_
