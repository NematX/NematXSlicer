///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PolygonOps_hpp_
#define slic3r_Api_plugin_cpp_PolygonOps_hpp_

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api {

/*
Polygon operations built on the plugin API
==========================================

This header holds C++ helpers that compose existing plugin API calls into one
polygon operation, for every plugin to share. They add no C function, so there
is no ABI change and no Python mirror: they exist for C++ plugins only.

Mental model
------------
A helper takes a read-only view of a collection and a storage, and returns a
StoredPolygonCollection owned by that storage. The input is never modified.
Intermediate handles are created in the same storage and are released before
the helper returns; only the returned collection outlives the call, and it
must not outlive the storage.

polygons_simplify()
-------------------
Replacement of the native Slic3r::polygons_simplify(polygons, tolerance, false)
for plugins. The native function cleans each ring alone; so does this one:
  1. every ring is simplified on its own with the Douglas-Peucker algorithm
     (polygons_ensure_valid on a one-ring collection, which is what the native
     function runs, with the same truncation of the tolerance to a coord_t); a
     ring left with fewer than 3 points disappears;
  2. that ring alone goes through a NonZero union (clipper_union), which is what
     ClipperLib::SimplifyPolygons executes in the native function: the self
     intersections created by the simplification (or already in the ring) are
     resolved into counter-clockwise rings. When the simplified ring was
     clockwise, the rings of its result are reversed back to clockwise, as the
     native function does, so the winding of each ring keeps its sign;
  3. one clipper_union over all the cleaned rings gives rings that do not overlap.
     The signs kept in step 2 matter here: a clockwise ring (a hole) inside a
     counter-clockwise one cancels its winding instead of adding to it.

The output is a flat ring list (contours counter-clockwise, holes clockwise), as
the native output is. Under the NonZero fill rule it describes the same region as
the native output.

Differences from the native function (accepted):
  - the order of the rings and the first point of each ring differ: compare the
    results as regions or canonical rings, not with operator==;
  - rings that overlap or touch after the cleaning are merged into one ring by
    step 3 (the native function keeps them as separate rings);
  - a clockwise ring that has no enclosing contour in the set is a filled area
    for the NonZero rule: the native function keeps it clockwise, the final union
    returns it counter-clockwise (same region);
  - areas can differ below the resolution of the simplification (a few scaled
    units of the Clipper integer grid);
  - there is no strictly_simple mode (the plugin callers do not use it).
*/

// Simplify the rings of polygons with the tolerance (scaled units, Douglas-Peucker distance), each ring
// alone, then merge the result into non-overlapping rings. The result is a new collection owned by
// storage; polygons is not modified. An empty input gives an empty collection. See the header comment
// for the differences from the native Slic3r::polygons_simplify.
inline StoredPolygonCollection polygons_simplify(storage_handle *storage, const PolygonCollection &polygons, double tolerance)
{
    ClipperContext clipper(storage);
    // The cleaned rings of every input ring, with the orientation the native function gives them.
    StoredPolygonCollection cleaned_rings(storage);
    for (uint32_t ring_idx = 0; ring_idx < polygons.size(); ++ring_idx) {
        // A copy of the ring alone: ensure_valid edits its collection in place and may delete the ring.
        StoredPolygonCollection ring(storage);
        ring.push_back(polygons.at(ring_idx));
        ring.ensure_valid(coord_t(tolerance));
        if (ring.empty())
            continue;
        const bool clockwise = ring.at(0).is_clockwise();
        // The operand is an adapter over `ring`, which stays alive until the union result is materialized.
        StoredPolygonCollection cleaned = clipper_union(clipper(ring.readonly())).to_polygon_collection();
        if (!clockwise) {
            cleaned_rings.append_move_from(cleaned);
            continue;
        }
        // The union returns counter-clockwise rings: restore the clockwise orientation of a hole.
        while (!cleaned.empty()) {
            StoredPolygon reversed = cleaned.extract(cleaned.size() - 1);
            reversed.reverse();
            cleaned_rings.push_back_move(std::move(reversed));
        }
    }
    return clipper_union(clipper(cleaned_rings.readonly())).to_polygon_collection();
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PolygonOps_hpp_
