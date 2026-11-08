///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_XYSizeCompensationHelpers_hpp_
#define slic3r_XYSizeCompensationHelpers_hpp_

#include <cstddef>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
 * Geometry core of the XY size compensation plugin, for one layer at a time.
 *
 * Vocabulary used by this file and XYSizeCompensation.cpp:
 * - layer: one horizontal slice of the object.
 * - region: the part of a layer printed with one set of print settings. An
 *   object usually has one region; modifiers or per-volume settings add more.
 *   Regions of a layer never overlap, and the host stores one polygon set
 *   ("slices") per region.
 * - island: one connected piece of material on the layer (an outer contour and
 *   its holes). One island may be split between several regions.
 * - mask: the part of one island that belongs to one region.
 * - outer delta: how far the outside contour of the material moves. Positive
 *   adds material, negative removes it.
 * - inner delta: same, for the walls of holes. Positive fills holes in.
 * Distances are in scaled integer units (see scale_d / unscaled).
 */

namespace slic3r_api { namespace XYSizeCompensationPlugin { namespace detail {

// The compensation settings of one region on one layer. They are the same for
// every island of that layer, so they are stored once per region, not per island.
struct RegionSettings
{
    coord_t outer_delta = 0;
    coord_t inner_delta = 0;
    // True on the first layers when narrow walls must not disappear when the
    // compensation removes material ("elephant foot" protection).
    bool no_collapse = false;
    // Wall-width threshold below which no-collapse stops withdrawing material.
    // It does not guarantee a final thickness or thicken existing thin walls.
    // Only set when protection is needed; zero otherwise.
    double minimum_contour_width_mm = 0.;
};

// One island of the layer and how it is split between the layer's regions.
// region_masks has one entry per region of the layer, in the same order as the
// settings vector; a region that has no material in this island has an empty mask.
// The masks of an island do not overlap and together cover the island.
// All geometry is owned by the caller's storage, which must outlive this object.
struct IslandInput
{
    explicit IslandInput(storage_handle *storage) : original(storage) {}

    StoredExPolygon original;
    std::vector<StoredExPolygonCollection> region_masks;
};

// Result for one layer, ready to replace the host's region slices.
struct LayerOutput
{
    explicit LayerOutput(storage_handle *storage) : slices(storage) {}

    // All material of the compensated layer, every region merged. Used for
    // coverage diagnostics and tests; the host rebuilds islands from regions.
    StoredExPolygonCollection slices;
    // The same material split per region, in the order of the settings vector.
    std::vector<StoredExPolygonCollection> regions;
    // Area, in scaled units squared, of new material that no region claimed
    // and that was given to a region by the fallback rule.
    double fallback_area = 0.;
};

// Fallback material above this area is reported as a warning; below it, it is
// treated as Clipper rounding. Matches the host's insignificant-overlap tolerance.
constexpr double k_insignificant_area = 10. * double(SCALED_EPSILON) * double(SCALED_EPSILON);

// Convert millimeters to an exact, Clipper-safe integer displacement.
coord_t checked_scaled_delta(double millimeters);

// Compensate every island of one layer, merge the results, then give each piece
// of the final material to exactly one region: material that already existed
// stays in its region, and new material goes to the region that grew it.
// settings[i] and island.region_masks[i] describe the same region i.
LayerOutput compensate_layer(storage_handle *storage,
                             const std::vector<RegionSettings> &settings,
                             const std::vector<IslandInput> &islands);

}}} // namespace slic3r_api::XYSizeCompensationPlugin::detail

#endif // slic3r_XYSizeCompensationHelpers_hpp_
