///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ExtrusionGenerationWinding_hpp_
#define slic3r_ExtrusionGenerationWinding_hpp_

#include <algorithm>

#include "ExtrusionEntityOrdering.hpp"

/* Normalize generated adhesion contours before ordering chooses their entries.
   Read stored closure and winding rather than legacy path/collection types.
   This generation policy may reverse geometry even when later ordering cannot;
   identities, properties, permissions, arc data and endpoint Z stay on the nodes.

       const GenerationWindingResult result = normalize_generation_winding(tree, true);
   An explicitly reversible target authorizes inversion of its complete command
   phases and fixed descendants. Otherwise inspect deferred_contours: fragmented
   command phases are left unchanged, not silently reported as normalized. A sole
   closed child can receive the reversal through non-reversible wrappers while
   surrounding empty events remain at their original indices.
   The caller exclusively owns the generated tree. Orientation allocates before
   each contour's mutation; no borrowed geometry or resource is retained.
*/

namespace Slic3r { namespace ExtrusionTreeTransforms {

struct GenerationWindingResult
{
    size_t reversed_contours = 0;
    size_t deferred_contours = 0;
};

namespace GenerationWindingDetail {

// Borrow stored geometry only: do not dispatch to legacy collection/loop/Nop
// projections. Linearization matches the generation policy's orientation test.
inline void collect_points(const ExtrusionEntity &entity, Points &out)
{
    if (const ArcPolyline *path = entity.polyline_or_null()) {
        const Polyline linear = path->to_polyline();
        out.insert(out.end(), linear.points.begin(), linear.points.end());
        return;
    }
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child)
            collect_points(*child, out);
}

// Non-reversible generation contours retain the historical pure-geometry policy.
// Reversible units bypass these barriers and reverse their complete command phases.
inline bool has_barrier(const ExtrusionEntity &entity)
{
    if (has_one_shot_extrusion_event(entity) ||
        (entity.ExtrusionEntity::empty() && entity.has_properties()))
        return true;
    for (const ExtrusionEntityUPtr &child : entity.children())
        if (child && has_barrier(*child))
            return true;
    return false;
}

// Generation has explicitly requested winding, as legacy Loop::reverse did.
// This is not an ordering permission check. The prevalidated subtree has no
// event barriers; preserve nodes, properties and flags while reversing arcs/Z.
inline void reverse_geometry(ExtrusionEntity &entity)
{
    if (ArcPolyline *path = entity.polyline_or_null()) {
        path->reverse();
        return;
    }
    const ExtrusionEntity &view = entity;
    if (view.children().empty())
        return;
    for (const ExtrusionEntityUPtr &child : view.children())
        if (child)
            reverse_geometry(*child);
    // Stored children are already present, so this does not convert content.
    ExtrusionEntity::Children &children = entity.children();
    std::reverse(children.begin(), children.end());
}

} // namespace GenerationWindingDetail

// Normalize borrowed generation output only. An explicitly reversible target
// authorizes its complete inversion, including commands and fixed descendants.
// Selection and geometry use stored content, not virtual loop/projection queries.
// The reversible branch calls virtual reverse (including Collection cache
// invalidation); the historical pure-geometry branch avoids it. No permission or
// resource is changed. Orientation allocation precedes a contour's writes.
// Non-reversible barriers are reported, not rejected or moved;
// callers must not silently claim such contours have been normalized.
inline GenerationWindingResult normalize_generation_winding(ExtrusionEntity &entity, const bool clockwise)
{
    GenerationWindingResult result;
    if (!entity.ExtrusionEntity::is_loop()) {
        const ExtrusionEntity &view = entity;
        for (const ExtrusionEntityUPtr &child : view.children()) {
            if (!child)
                continue;
            const GenerationWindingResult nested = normalize_generation_winding(*child, clockwise);
            result.reversed_contours += nested.reversed_contours;
            result.deferred_contours += nested.deferred_contours;
        }
        return result;
    }

    Points points;
    GenerationWindingDetail::collect_points(entity, points);
    if (Polygon(std::move(points)).is_counter_clockwise() != clockwise)
        return result;

    // A reversible target is an authorized unit: do not delegate away its events.
    // Otherwise empty siblings stay around the sole closed geometric child.
    ExtrusionEntity *target = &entity;
    while (!target->can_reverse() && target->polyline_or_null() == nullptr) {
        ExtrusionEntity *only = nullptr;
        size_t count = 0;
        const ExtrusionEntity &view = *target;
        for (const ExtrusionEntityUPtr &child : view.children())
            if (child && !child->ExtrusionEntity::empty()) {
                only = child.get();
                ++count;
            }
        if (count != 1 || !only->ExtrusionEntity::is_loop())
            break;
        target = only;
    }
    if (target->can_reverse()) {
        target->reverse();
        result.reversed_contours = 1;
        return result;
    }
    // A one-shot on the target itself stays at the same closed-contour entry.
    // Only strict descendants can change execution phase under child reversal;
    // command state is represented by properties, not by a legacy node type.
    bool barrier = false;
    const ExtrusionEntity &view = *target;
    for (const ExtrusionEntityUPtr &child : view.children())
        if (child && GenerationWindingDetail::has_barrier(*child)) {
            barrier = true;
            break;
        }
    if (barrier) {
        result.deferred_contours = 1;
        return result;
    }
    const Point entry = entity.ExtrusionEntity::first_point();
    GenerationWindingDetail::reverse_geometry(*target);
    assert(entity.ExtrusionEntity::first_point() == entry);
    assert(entity.ExtrusionEntity::is_loop());
    result.reversed_contours = 1;
    return result;
}

}} // namespace Slic3r::ExtrusionTreeTransforms

#endif // slic3r_ExtrusionGenerationWinding_hpp_
