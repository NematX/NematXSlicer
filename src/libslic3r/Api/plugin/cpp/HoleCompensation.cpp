///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "HoleCompensation.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/HoleGeometry.hpp"

/*
This SDK routine gives hole-compensation providers one regional geometry core.
It merges the borrowed layer material to find real voids, then asks the caller
for each region's signed distance. Removals stay in their region; additions
cannot cover existing material and are assigned in region order.

    compensate_convex_holes()
    |-- union_of(): find the merged layer material
    |-- hole_void() / offset_of(): build bands around convex holes
    |-- intersection_of(): limit a region's claim to its wall
    `-- difference_of() / union_of(): publish independent regional replacements

Inputs stay read-only, callbacks run synchronously, and output collections belong
to caller storage. Errors propagate without publishing into the host's slices.
There is no shared mutable state; callbacks must support the caller's thread use.
*/

namespace slic3r_api {
namespace {

// A region acts on a hole when its material lies within this distance of the
// hole wall. The wall of a hole is the border of the merged layer material, so
// the regions that form it touch it exactly; the margin only absorbs rounding.
constexpr coord_t k_touch_margin = 10 * SCALED_EPSILON;

// Merge borrowed regional collections into an owned collection, skipping empty inputs.
StoredExPolygonCollection union_of(storage_handle *storage, const std::vector<ExPolygonCollection> &collections);
// Offset borrowed material into caller storage; empty input yields an empty collection.
StoredExPolygonCollection offset_of(storage_handle *storage, const ExPolygonCollection &geometry, coord_t delta);
// Copy only material shared by the two borrowed inputs.
StoredExPolygonCollection intersection_of(storage_handle *storage, const ExPolygonCollection &subject,
                                          const ExPolygonCollection &clip);
// Copy subject material outside clip; inputs remain unchanged.
StoredExPolygonCollection difference_of(storage_handle *storage, const ExPolygonCollection &subject,
                                        const ExPolygonCollection &clip);
// Turn a borrowed clockwise hole into an owned filled contour.
StoredExPolygonCollection hole_void(storage_handle *storage, const Polygon &hole);

} // namespace

std::vector<StoredExPolygonCollection> compensate_convex_holes(storage_handle *storage,
                                                               const std::vector<ExPolygonCollection> &region_slices,
                                                               const ConvexHoleDelta &delta_of,
                                                               bool &changed)
{
    if (storage == nullptr || !delta_of)
        throw std::invalid_argument("Hole compensation needs a storage and a distance function.");
    const std::size_t region_count = region_slices.size();
    changed = false;

    // The holes are those of the merged layer material.
    const StoredExPolygonCollection material = union_of(storage, region_slices);

    // Keep removals and additions separate until claims can be resolved in region order.
    std::vector<StoredExPolygonCollection> removed;
    std::vector<StoredExPolygonCollection> added;
    for (std::size_t idx = 0; idx < region_count; ++idx) {
        removed.emplace_back(storage);
        added.emplace_back(storage);
    }

    for (const ExPolygon island : material.readonly()) {
        for (const Polygon hole : island.holes()) {
            if (!is_convex_hole(hole))
                continue;
            const ConvexHoleInfo info{ hole, std::abs(hole.area()), approximate_hole_diameter(hole) };
            const StoredExPolygonCollection inside = hole_void(storage, hole);
            if (inside.empty())
                continue;
            // A thin band finds the regions whose material forms the hole wall.
            const StoredExPolygonCollection wall_zone = offset_of(storage, inside.readonly(), k_touch_margin);

            for (std::size_t region = 0; region < region_count; ++region) {
                if (region_slices[region].empty())
                    continue;
                const coord_t delta = delta_of(region, info);
                if (delta == 0)
                    continue;
                const StoredExPolygonCollection wall =
                    intersection_of(storage, wall_zone.readonly(), region_slices[region]);
                if (wall.empty())
                    continue;

                if (delta < 0) {
                    // Enlarging removes only this region's material around the hole.
                    const StoredExPolygonCollection grown = offset_of(storage, inside.readonly(), -delta);
                    StoredExPolygonCollection ring = difference_of(storage, grown.readonly(), inside.readonly());
                    removed[region].append_move_from(ring);
                } else {
                    // Shrinking assigns new material only within this wall's reach.
                    const StoredExPolygonCollection shrunk = offset_of(storage, inside.readonly(), -delta);
                    const StoredExPolygonCollection ring = difference_of(storage, inside.readonly(), shrunk.readonly());
                    const StoredExPolygonCollection reach = offset_of(storage, wall.readonly(), delta + k_touch_margin);
                    StoredExPolygonCollection part = intersection_of(storage, ring.readonly(), reach.readonly());
                    added[region].append_move_from(part);
                }
            }
        }
    }

    // New material cannot cover an inner island or an earlier region's claim.
    std::vector<StoredExPolygonCollection> result;
    result.reserve(region_count);
    StoredExPolygonCollection claimed = material.readonly().clone(storage);
    for (std::size_t region = 0; region < region_count; ++region) {
        StoredExPolygonCollection slices = removed[region].empty() ?
            region_slices[region].clone(storage) :
            difference_of(storage, region_slices[region], removed[region].readonly());
        changed |= !removed[region].empty();
        if (!added[region].empty()) {
            const StoredExPolygonCollection new_material =
                difference_of(storage, added[region].readonly(), claimed.readonly());
            if (!new_material.empty()) {
                changed = true;
                slices = union_of(storage, { slices.readonly(), new_material.readonly() });
                claimed = union_of(storage, { claimed.readonly(), new_material.readonly() });
            }
        }
        result.push_back(std::move(slices));
    }
    return result;
}

namespace {

StoredExPolygonCollection union_of(storage_handle *storage, const std::vector<ExPolygonCollection> &collections)
{
    ClipperOperand combined(storage);
    bool has_paths = false;
    for (const ExPolygonCollection &collection : collections) {
        if (collection.empty())
            continue;
        ClipperOperand next(storage, collection);
        if (!has_paths) {
            combined = std::move(next);
            has_paths = true;
        } else {
            combined.concat_replace(next);
        }
    }
    if (!has_paths)
        return StoredExPolygonCollection(storage);
    return clipper_union(storage, combined).to_expolygon_collection();
}

StoredExPolygonCollection offset_of(storage_handle *storage, const ExPolygonCollection &geometry, coord_t delta)
{
    if (geometry.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand subject(storage, geometry);
    return clipper_offset(storage, subject, double(delta)).to_expolygon_collection();
}

StoredExPolygonCollection intersection_of(storage_handle *storage, const ExPolygonCollection &subject,
                                          const ExPolygonCollection &clip)
{
    if (subject.empty() || clip.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_intersection(storage, subject_operand, clip_operand).to_expolygon_collection();
}

StoredExPolygonCollection difference_of(storage_handle *storage, const ExPolygonCollection &subject,
                                        const ExPolygonCollection &clip)
{
    if (subject.empty())
        return StoredExPolygonCollection(storage);
    if (clip.empty())
        return subject.clone(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_diff(storage, subject_operand, clip_operand).to_expolygon_collection();
}

StoredExPolygonCollection hole_void(storage_handle *storage, const Polygon &hole)
{
    StoredPolygon filled(storage);
    filled.copy_from(hole);
    filled.make_counter_clockwise();
    StoredPolygonCollection polygons(storage);
    polygons.push_back(filled.readonly());
    ClipperOperand operand(storage, polygons.readonly());
    return clipper_union(storage, operand).to_expolygon_collection();
}

} // namespace

} // namespace slic3r_api
