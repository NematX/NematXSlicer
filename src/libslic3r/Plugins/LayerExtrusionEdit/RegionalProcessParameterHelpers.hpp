///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_LayerExtrusionEdit_RegionalProcessParameterHelpers_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_RegionalProcessParameterHelpers_hpp_

#include <cstdint>
#include <functional>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

/*
Regional process-parameter helpers
==================================

Process settings may differ between the regions attached to one
LayerRegionIsland, while its extrusion tree may cross several of those regions.
This module turns RegionSettings geometry into immutable partitions suitable
for STEP_LAYER_EXTRUSION_EDIT.

The first stage groups raw configuration tuples and translates their masks to
the object instance coordinates already used by PrintingPlan. The second stage
lets a process plugin resolve each tuple to its final stored float and merges
zones that produce the same value. Callers may then inspect a temporary clone
without changing the plan, or split and annotate the real plan leaf.
DEPOSIT leaves request stationary volume at one point. They select a projected
area without splitting; missing or conflicting areas use the island's first
region value retained alongside the projection.

The producer guarantees that RegionSettings masks are disjoint and cover the
source island. These helpers intentionally do not diagnose or repair malformed
partitions.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace RegionalProcessParameterHelpers {

using ProcessParameterHelpers::EPropertySpeedFieldEditor;

struct RegionalSourceKey
{
    const layer_region_island_handle *region_island = nullptr;
    size_t printing_catalog_idx = size_t(-1);

    bool operator<(const RegionalSourceKey &rhs) const;
};

struct RegionalSettingsArea
{
    RegionalSettingsArea(const Config &config, StoredExPolygonCollection &&area);
    RegionalSettingsArea(RegionalSettingsArea &&) noexcept = default;
    RegionalSettingsArea &operator=(RegionalSettingsArea &&) noexcept = default;

    RegionalSettingsArea(const RegionalSettingsArea &) = delete;
    RegionalSettingsArea &operator=(const RegionalSettingsArea &) = delete;

    Config config;
    StoredExPolygonCollection area;
};

struct RegionalSettingsPartition
{
    bool is_uniform() const { return areas.empty(); }

    Config uniform_config;
    std::vector<RegionalSettingsArea> areas;
};

struct RegionalProcessPartition
{
    bool requires_split() const
    {
        return areas.size() > 1 && areas.size() == values.size();
    }

    float uniform_value = -1.f;
    float fallback_value = -1.f;
    std::vector<StoredExPolygonCollection> areas;
    std::vector<float> values;
};

using RegionalValueResolver = std::function<float(const Config &)>;
using RegionalValuePredicate = std::function<bool(float)>;

// Read a deposit's single point in the immutable partition. Return the matching
// value, or the first-region fallback for missing/conflicting masks. No split or
// ownership transfer occurs; invalid point access propagates.
float deposit_value(const ExtrusionEntity &leaf, const RegionalProcessPartition &partition);

// Build one raw configuration partition in PrintingPlan instance coordinates.
RegionalSettingsPartition build_regional_settings_partition(
    storage_handle *storage,
    const LayerRegionIsland &region_island,
    const RegionSettings::OptionKeyGroup &option_keys,
    const c_point instance_shift = {0,0});

// Resolve every raw tuple and merge areas whose final stored float is equal.
RegionalProcessPartition project_regional_process_values(
    storage_handle *storage,
    const RegionalSettingsPartition &settings,
    const RegionalValueResolver &resolve_value);

// Test touched regional values by splitting a clone, or selecting a deposit's point without splitting.
bool regional_partition_leaf_matches(
    storage_handle *storage,
    const MutableExtrusionEntity &leaf,
    const RegionalProcessPartition &partition,
    const RegionalValuePredicate &matches_value);

// Assign regional values to split fragments or a single unsplit deposit selected by its point.
// Returns false when the partition has only one final value and no split is needed.
bool apply_regional_process_partition(
    MutableExtrusionEntity leaf,
    const RegionalProcessPartition &partition,
    EPropertySpeedFieldEditor &editor);

}}} // namespace slic3r_api::LayerExtrusionEdit::RegionalProcessParameterHelpers

#endif // slic3r_Plugins_LayerExtrusionEdit_RegionalProcessParameterHelpers_hpp_
