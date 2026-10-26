///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* This built-in support family handles grid and snug styles. It reads demand
   from object islands and runs native geometry calculations, then publishes
   auxiliary slices, support properties and typed fill surfaces through the
   plugin API. Extrusion generation belongs to a later support-toolpath step. */
#ifndef plugins_support_grid_family_hpp_
#define plugins_support_grid_family_hpp_
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/ClipperUtils.hpp"

namespace slic3r_api::Support::Grid {
namespace Internal {
/* Every area of one auxiliary layer is printed by exactly one role. A support layer holds three
   kinds of surfaces: the upper ones (contacts touching the object above and the interfaces below
   them), the lower ones (contacts touching the object below and the interfaces above them) and the
   sparse base that carries the load down to the bed. They must stay disjoint, otherwise the same
   area is extruded twice. When the gap between the object above and the object below is thinner
   than the support layers, one area is both upper and lower: the upper surface keeps it, as the
   native generator merged the lower contact into the upper one. */
// Decide whether the native generator would have merged a contact area into the base layer, which
// makes the family publish that contact as base; a bridging contact never merges.
bool merged_into_base(bool merge, bool base_takes_contact, bool z_has_base, bool bridging);
// Remove from the surfaces every area the covering surfaces already print, so each area keeps one owner.
Slic3r::Polygons resolve_overlap(const Slic3r::Polygons &surfaces, const Slic3r::Polygons &covering);
}
class SupportFamilyGrid final : public PluginBase {
public:
    // Return the process-lifetime built-in provider.
    static SupportFamilyGrid &instance(orchestrator_handle *orchestrator);
private:
    // Connect the provider to the host orchestrator.
    explicit SupportFamilyGrid(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
    // Return the stable activation id.
    const char *id_impl() const noexcept override { return "support.family.grid"; }
    // Return the name shown in plugin lists.
    const char *name_impl() const noexcept override { return "Grid and snug support family"; }
    // Describe the geometric output of this family.
    const char *description_impl() const noexcept override { return "Build grid and snug support layers with typed surfaces."; }
    // Select the per-object support generation step.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // Allow any demand producer without requiring a particular provider.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Run the family at the default support priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Declare settings used by the geometry and flow calculations.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Filter the object, calculate support and publish typed auxiliary layers.
    void run_impl(const plugin_run_context *context) const override;
};
// Register the built-in grid and snug support provider.
void register_support_family_grid_plugin(orchestrator_handle *orchestrator);
}
#endif
