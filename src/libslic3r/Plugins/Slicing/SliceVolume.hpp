///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_slicing_slicevolume_hpp_
#define plugins_slicing_slicevolume_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace SliceVolumePlugin {

// Official STEP_SLICING plugin that slices object volumes into raw LayerRegion slices.
//
// The plugin intentionally stops at raw slices. Surface creation belongs to the
// following surface-generation step, so callers should expect this plugin to
// populate LayerRegion::raw_slices only.
class SliceVolume : public PluginBase
{
public:
    static SliceVolume &instance(orchestrator_handle *orch);

private:
    SliceVolume(orchestrator_handle *orch) : PluginBase(orch) {}
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the singleton plugin in an orchestrator. This is used both by the
// built-in host registration path and by the optional DLL entry point below.
void register_slice_volume_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::SliceVolumePlugin

#endif // plugins_slicing_slicevolume_hpp_
