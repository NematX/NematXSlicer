///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_holesizecompensation_hpp_
#define plugins_holesizecompensation_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

/*
Register the convex-hole compensation provider before initializing settings:

    register_hole_size_compensation_plugin(orchestrator);

Its regional distance and area threshold apply at post-slicing priority -1100,
before XY offsets. The detail helper accepts one setting per borrowed regional
slice set and returns owned replacements in caller storage.
*/

namespace slic3r_api { namespace HoleSizeCompensationPlugin {

// Post-slicing compensation of convex holes (hole_size_compensation and
// hole_size_threshold), using only the public C++ geometry SDK. It runs before
// the XY size compensation, so it measures the holes as sliced.
class HoleSizeCompensation final : public PluginBase
{
public:
    // Borrow the provider instance bound to the host orchestrator.
    static HoleSizeCompensation &instance(orchestrator_handle *orch);

private:
    // Retain the orchestrator while per-run geometry stays in caller storage.
    explicit HoleSizeCompensation(orchestrator_handle *orch) : PluginBase(orch) {}

    // Return the stable provider identifier.
    const char *id_impl() const noexcept override;
    // Return borrowed display text.
    const char *name_impl() const noexcept override;
    // Describe convex-hole compensation for the settings catalogue.
    const char *description_impl() const noexcept override;
    // Execute on sliced regions before perimeter preparation.
    slicing_step_t step_impl() const noexcept override;
    // Return the empty dependency list; priority places this before XY edits.
    const char *const *dependencies_impl() const noexcept override;
    // Run before the XY compensation provider.
    int32_t priority_impl() const noexcept override;
    // Copy the used setting keys, or return their count when keys is null.
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    // Copy owned keys, or return their count when keys is null.
    int32_t defined_config_keys(const char **keys) const noexcept override;
    // Register both regional setting definitions once at startup.
    void inilialize_impl(storage_handle *storage) const override;
    // Compute each layer's owned replacements and publish only after validation.
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the native post-slicing plugin with the host orchestrator.
void register_hole_size_compensation_plugin(orchestrator_handle *orch);

namespace detail {

// The settings of one region on one layer, already converted to scaled units.
struct RegionHoleSettings
{
    // hole_size_compensation: how far the wall of a convex hole moves.
    // Negative enlarges the hole (removes material), positive shrinks it.
    coord_t delta = 0;
    // hole_size_threshold, in scaled units squared (0: no threshold).
    double threshold_area = 0.;
};

// Compensate the convex holes of one layer. region_slices[i] and settings[i]
// describe region i: each region moves its part of a hole wall by its value,
// reduced by its threshold. The regional rules are those of the SDK
// compensate_convex_holes() (HoleCompensation.hpp).
// Returns the new slices of every region, in the same order, and sets
// `changed` to false when no region was modified (the result is then a copy).
std::vector<StoredExPolygonCollection> compensate_holes(storage_handle *storage,
                                                         const std::vector<RegionHoleSettings> &settings,
                                                         const std::vector<ExPolygonCollection> &region_slices,
                                                         bool &changed);

} // namespace detail

}} // namespace slic3r_api::HoleSizeCompensationPlugin

#endif // plugins_holesizecompensation_hpp_
