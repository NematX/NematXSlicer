///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Regional process-parameter partitions
=====================================

The implementation performs all PluginStorage allocations during the serialized
setup phase. Raw RegionSettings zones are translated once for their object
instance, then projected and united by exact final float. Parallel plugin runs
only borrow those immutable masks when splitting independent extrusion leaves.

For setup-time questions such as autospeed eligibility, a leaf is deep-cloned
into PluginStorage and the same splitter is run on the clone. This preserves
the exact arc-aware clipping behavior without changing PrintingPlan.
DEPOSIT leaves carry stationary volume at one point; area containment selects
their value without clipping, with a retained first-region fallback.
*/

#include "RegionalProcessParameterHelpers.hpp"

#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>
#include <utility>

namespace slic3r_api { namespace LayerExtrusionEdit { namespace RegionalProcessParameterHelpers {
namespace {

// Find the physical island whose finite slice bounds a region-island partition.
LayerIsland owning_layer_island(const LayerRegionIsland &region_island);

// Translate ExPolygon geometry through mutable oriented paths, then rebuild holes.
StoredExPolygonCollection translated_area(storage_handle *storage,
                                          const ExPolygonCollection &source,
                                          c_point shift);

// Select a validated deposit's point in projected masks, or return the first-region value when no unique mask contains it.
float deposit_value(const ExtrusionEntity &leaf, const RegionalProcessPartition &partition);

LayerIsland owning_layer_island(const LayerRegionIsland &region_island)
{
    if (!region_island.valid() || region_island.region_count() == 0)
        throw std::runtime_error("Cannot locate the LayerIsland of an invalid LayerRegionIsland.");

    // Region-islands do not expose a parent pointer through the plugin ABI, so
    // locate the owner by scanning the source layer during serialized setup.
    const Layer layer = region_island.region(0).layer();
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        const LayerIsland island = layer.island(island_idx);
        for (uint32_t region_island_idx = 0;
             region_island_idx < island.region_island_count();
             ++region_island_idx) {
            if (island.region_island(region_island_idx).handle() == region_island.handle())
                return island;
        }
    }

    throw std::runtime_error("Printing extrusion source LayerRegionIsland has no owning LayerIsland.");
}

StoredExPolygonCollection translated_area(storage_handle *storage,
                                          const ExPolygonCollection &source,
                                          c_point shift)
{
    if (shift.x == 0 && shift.y == 0)
        return source.clone(storage);

    // Flatten contours and holes into oriented paths because StoredExPolygon
    // intentionally exposes its point sequences as read-only views.
    StoredPolygonCollection translated_paths(storage);
    for (const ExPolygon expolygon : source) {
        translated_paths.push_back(expolygon.contour());
        for (const Polygon hole : expolygon.holes())
            translated_paths.push_back(hole);
    }

    // Moving the first path to the end after translation visits every original
    // path exactly once without requiring mutable collection element views.
    const uint32_t path_count = translated_paths.size();
    for (uint32_t path_idx = 0; path_idx < path_count; ++path_idx) {
        StoredPolygon path = translated_paths.extract(0);
        path.translate(double(shift.x), double(shift.y));
        translated_paths.push_back_move(std::move(path));
    }

    ClipperContext clipper(storage);
    return clipper_union(clipper(translated_paths.readonly())).to_expolygon_collection();
}

// Inspect immutable masks using the borrowed one-point deposit; return a copied value, using first-region fallback for missing or conflicting areas, without splitting or transferring ownership.
float deposit_value(const ExtrusionEntity &leaf, const RegionalProcessPartition &partition)
{
    const c_point point = leaf.point(0);
    float value = partition.fallback_value;
    bool found = false;
    for (size_t area_idx = 0; area_idx < partition.areas.size(); ++area_idx) {
        bool contains = false;
        for (const ExPolygon area : partition.areas[area_idx].readonly())
            if (area.contains(point)) {
                contains = true;
                break;
            }
        if (contains) {
            if (found)
                return partition.fallback_value;
            value = partition.values[area_idx];
            found = true;
        }
    }
    return value;
}

} // namespace

bool RegionalSourceKey::operator<(const RegionalSourceKey &rhs) const
{
    const std::less<const layer_region_island_handle *> less_handle;
    if (region_island != rhs.region_island)
        return less_handle(region_island, rhs.region_island);
    return printing_catalog_idx < rhs.printing_catalog_idx;
}

RegionalSettingsArea::RegionalSettingsArea(const Config &config_value,
                                           StoredExPolygonCollection &&area_value)
    : config(config_value), area(std::move(area_value))
{}

RegionalSettingsPartition build_regional_settings_partition(
    storage_handle *storage,
    const LayerRegionIsland &region_island,
    const RegionSettings::OptionKeyGroup &option_keys,
    const c_point instance_shift /* = {0,0}*/)
{
    if (storage == nullptr || !region_island.valid() || region_island.region_count() == 0 ||
        option_keys.empty())
        throw std::invalid_argument("Regional process partition requires storage, regions, and option keys.");

    RegionalSettingsPartition partition;
    partition.uniform_config = region_island.region(0).print_region().config();
    if (region_island.region_count() == 1)
        return partition;

    // RegionSettings performs the raw tuple grouping and clips only the
    // regions attached to this LayerRegionIsland against its physical island.
    const LayerIsland island = owning_layer_island(region_island);
    std::vector<RegionSettings::OptionKeyGroup> option_groups;
    option_groups.push_back(option_keys);
    RegionSettings settings(storage, partition.uniform_config, std::move(option_groups));
    for (uint32_t region_idx = 0; region_idx < region_island.region_count(); ++region_idx)
        settings.add_region(region_island.region(region_idx));
    settings.segregate(island.slice());

    const char *const primary_key = option_keys.front().c_str();
    const RegionSettings::AreaMap &raw_areas = settings.get_areas(primary_key);
    if (raw_areas.size() == 1) {
        const std::vector<LayerRegion> &regions =
            settings.get_regions(primary_key, raw_areas.begin()->first);
        if (regions.empty())
            throw std::runtime_error("Uniform regional process partition has no source region.");
        partition.uniform_config = regions.front().print_region().config();
        return partition;
    }

    // Materialize every raw zone in the same coordinates as its PrintingPlan
    // extrusion. The representative config is valid for the whole tuple group.
    partition.areas.reserve(raw_areas.size());
    for (const std::pair<const RegionSettingsValue, RegionSettingsClip> &entry : raw_areas) {
        const std::vector<LayerRegion> &regions = settings.get_regions(primary_key, entry.first);
        if (regions.empty() || entry.second.is_accept_all())
            throw std::runtime_error("Regional process partition has no concrete source area.");
        partition.areas.emplace_back(
            regions.front().print_region().config(),
            translated_area(storage, entry.second.expolygons(), instance_shift));
    }
    return partition;
}

// Resolve borrowed raw regional configs into owned immutable masks and scalar values, retaining the first-region fallback; invalid inputs or non-finite values throw before publication.
RegionalProcessPartition project_regional_process_values(
    storage_handle *storage,
    const RegionalSettingsPartition &settings,
    const RegionalValueResolver &resolve_value)
{
    if (storage == nullptr || !resolve_value)
        throw std::invalid_argument("Regional process projection requires storage and a resolver.");

    RegionalProcessPartition partition;
    partition.fallback_value = resolve_value(settings.uniform_config);
    if (!std::isfinite(partition.fallback_value))
        throw std::runtime_error("Regional process fallback resolved to a non-finite float.");
    if (settings.is_uniform()) {
        partition.uniform_value = partition.fallback_value;
        return partition;
    }

    // Raw tuples may differ in settings that do not affect the current role.
    // Merge them by the exact float that the process plugin intends to store.
    std::map<float, StoredExPolygonCollection> grouped_areas;
    for (const RegionalSettingsArea &settings_area : settings.areas) {
        const float value = resolve_value(settings_area.config);
        if (!std::isfinite(value))
            throw std::runtime_error("Regional process value resolved to a non-finite float.");

        std::map<float, StoredExPolygonCollection>::iterator grouped = grouped_areas.find(value);
        if (grouped == grouped_areas.end()) {
            grouped = grouped_areas.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(value),
                std::forward_as_tuple(storage)).first;
        }
        grouped->second.append_copy_from(settings_area.area.readonly());
    }

    if (grouped_areas.size() == 1) {
        partition.uniform_value = grouped_areas.begin()->first;
        return partition;
    }

    // Union each final-value bucket once. The resulting masks remain immutable
    // while parallel layer runs split unrelated leaves.
    partition.areas.reserve(grouped_areas.size());
    partition.values.reserve(grouped_areas.size());
    for (std::pair<const float, StoredExPolygonCollection> &entry : grouped_areas) {
        ClipperContext clipper(storage);
        StoredExPolygonCollection area =
            clipper_union(clipper(entry.second.readonly())).to_expolygon_collection();
        partition.values.push_back(entry.first);
        partition.areas.push_back(std::move(area));
    }
    return partition;
}

// Inspect borrowed geometry via a storage-owned split clone or one deposit point, returning whether a resolved value matches; invalid inputs and split failures throw without changing the plan.
bool regional_partition_leaf_matches(
    storage_handle *storage,
    const MutableExtrusionEntity &leaf,
    const RegionalProcessPartition &partition,
    const RegionalValuePredicate &matches_value)
{
    if (storage == nullptr || !leaf.valid() || !matches_value)
        throw std::invalid_argument("Regional leaf inspection requires storage, a leaf, and a predicate.");
    if (!partition.requires_split())
        return matches_value(partition.uniform_value);
    if (leaf.is_deposit())
        return matches_value(deposit_value(leaf.readonly(), partition));

    // Split a deep clone so setup can ask which regional values the exact arc
    // geometry touches without changing the plan before parallel execution.
    StoredExtrusionEntity clone(storage, leaf.readonly());
    std::vector<ExPolygonCollection> area_views;
    area_views.reserve(partition.areas.size());
    for (const StoredExPolygonCollection &area : partition.areas)
        area_views.push_back(area.readonly());

    const std::vector<ExtrusionAreaFragment> fragments =
        clone.mutable_view().split_leaf_by_areas(area_views);
    for (const ExtrusionAreaFragment &fragment : fragments) {
        if (fragment.area_index >= partition.values.size())
            throw std::runtime_error("Regional clone split returned an invalid area index.");
        if (matches_value(partition.values[fragment.area_index]))
            return true;
    }
    return false;
}

// Annotate plan-owned split fragments or the unsplit deposit with the selected regional value; uniform partitions return false, split failures throw, and no geometry is cloned for deposits.
bool apply_regional_process_partition(
    MutableExtrusionEntity leaf,
    const RegionalProcessPartition &partition,
    EPropertySpeedFieldEditor &editor)
{
    if (!partition.requires_split())
        return false;
    // Stationary volume belongs to one point and must never be duplicated by clipping.
    if (leaf.is_deposit()) {
        editor.set_value(leaf, deposit_value(leaf.readonly(), partition));
        return true;
    }

    // The splitter preserves traversal order and source geometry. Assigning
    // through EPropertySpeedFieldEditor changes only its selected process field.
    std::vector<ExPolygonCollection> area_views;
    area_views.reserve(partition.areas.size());
    for (const StoredExPolygonCollection &area : partition.areas)
        area_views.push_back(area.readonly());

    const std::vector<ExtrusionAreaFragment> fragments = leaf.split_leaf_by_areas(area_views);
    for (const ExtrusionAreaFragment &fragment : fragments) {
        if (fragment.area_index >= partition.values.size())
            throw std::runtime_error("Regional process split returned an invalid area index.");
        editor.set_value(fragment.entity, partition.values[fragment.area_index]);
    }
    return true;
}

}}} // namespace slic3r_api::LayerExtrusionEdit::RegionalProcessParameterHelpers
