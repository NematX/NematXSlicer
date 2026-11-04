///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_polyholes_hpp_
#define plugins_polyholes_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace PolyholesPlugin {

// Official post-slicing plugin that replaces circular holes by polygonal holes
// tuned for FDM extrusion. It scans raw layer-region slices, groups matching
// holes through consecutive layers, then writes the replacement polygons back
// into the raw slices and asks the host to rebuild layer slices/islands.
class Polyholes : public PluginBase
{
public:
    static Polyholes &instance(orchestrator_handle *orch);
    static const char *exclusive_group_ui_fragment() noexcept;
    static const char *print_ui_fragment() noexcept;

private:
    Polyholes(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Prefer this algorithm among enabled alternatives for polygonal holes.
    int32_t exclusive_group_priority_impl() const noexcept override { return 0; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    // Replace round slice holes without clearing the plugin's shared storage.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the singleton plugin in an orchestrator or external plugin loader.
void register_polyholes_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::PolyholesPlugin

#endif // plugins_polyholes_hpp_
