///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ExtrusionSeamPlanning_hpp_
#define slic3r_ExtrusionSeamPlanning_hpp_

#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "ExtrusionEntityOrdering.hpp"

/* Read-only seam descriptions for native and host ordering code. A legal entry
   is a cyclic start that stored geometry, rotation permissions and event scopes
   allow. legal_seam_entry first validates a request against the whole contour,
   then describes a local rotation, child rotation, split or unchanged entry.
   A sole eligible contour may receive the seam through structural wrappers;
   open composite children remain atomic and event barriers keep child order.

   Planning borrows the tree and uses temporary polylines to preview native split
   rounding, preserving arcs and Z. It does not mutate nodes or consume permission.
   Indices describe this exact tree: recompute immediately before materialization,
   with no concurrent structural edits. Allocation and geometry errors propagate.

       const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(entity, requested, true);
       if (plan) consume_entry(plan->point);

   is_directly_legal_seam instead checks exact membership for analyses whose own
   indices/scores must refer to the requested point, not a projected replacement.
*/

namespace Slic3r {

// A read-only description for the existing local rotations/splits. Indices are
// borrowed from the current tree: recompute before mutation, never cache across
// an ordering pass. Delegation retains the parent's route and all empty siblings.
struct ExtrusionSeamPlan
{
    enum class Operation { KeepStart, RotateLocal, RotateChildren, SplitChild };
    Point point;
    Operation operation = Operation::KeepStart;
    std::vector<size_t> delegation;
    size_t child_index = 0;
    // Integer projection is not idempotent. Feed this input to the native split,
    // whose exact output is point; never feed point back through a projection.
    Point split_point = Point(0, 0);
};

namespace ExtrusionSeamPlanningDetail {

// Measure XY distance in floating point, avoiding scaled-integer subtraction overflow.
inline double distance_squared(const Point &lhs, const Point &rhs)
{
    // Subtract in floating point, as the existing SDK projection does.
    return (lhs.cast<double>() - rhs.cast<double>()).squaredNorm();
}

struct Projection
{
    Point point;
    double distance_squared = (std::numeric_limits<double>::max)();
    bool valid = false;
};

// Find the nearest stored point on native lines/arcs, breaking ties by child order.
inline Projection project(const ExtrusionEntity &entity, const Point &requested)
{
    Projection best;
    if (const ArcPolyline *path = entity.polyline_or_null()) {
        if (path->size() == 1)
            return {path->front(), distance_squared(path->front(), requested), true};
        const Geometry::ArcWelder::PathSegmentProjection projected =
            Geometry::ArcWelder::point_to_path_projection(path->get_arc(), requested);
        if (projected.valid())
            best = {projected.point, projected.distance2, true};
    } else {
        for (const ExtrusionEntityUPtr &child : entity.children()) {
            assert(child);
            if (!child)
                continue;
            const Projection candidate = project(*child, requested);
            if (candidate.valid && (!best.valid || candidate.distance_squared < best.distance_squared))
                best = candidate;
        }
    }
    return best;
}

// Match the existing split's endpoint snapping without adding a new tolerance.
inline Point snap_endpoint(const ArcPolyline &path, const Point &point)
{
    if (point.coincides_with_epsilon(path.front()))
        return path.front();
    if (point.coincides_with_epsilon(path.back()))
        return path.back();
    return point;
}

struct SplitPreview
{
    Point input;
    Point point;
};

// Preview the existing split on temporary geometry only. No owned entity,
// property or persistent fragment is created or changed during planning.
inline SplitPreview preview_split(const ArcPolyline &path, const Point &projected)
{
    const Point input = snap_endpoint(path, projected);
    if (input == path.front() || input == path.back())
        return {input, input};
    ArcPolyline before;
    ArcPolyline after;
    Point split_input = input;
    path.split_at(split_input, before, after);
    if (before.size() < 2)
        return {path.front(), path.front()};
    if (after.size() < 2)
        return {path.back(), path.back()};
    assert(before.back() == after.front());
    return {input, after.front()};
}

// requested has already passed the whole-contour projection contract.
inline ExtrusionSeamPlan plan_on_contour(const ExtrusionEntity &entity, const Point &requested)
{
    const Point start = entity.ExtrusionEntity::first_point();
    ExtrusionSeamPlan keep{start};
    if (const ArcPolyline *path = entity.polyline_or_null()) {
        const Projection projected = project(entity, requested);
        const SplitPreview split = preview_split(*path, projected.point);
        return {split.point, split.point == start ? ExtrusionSeamPlan::Operation::KeepStart :
                                                   ExtrusionSeamPlan::Operation::RotateLocal,
                {}, 0, split.input};
    }

    size_t only_geometry = 0;
    size_t geometry_count = 0;
    for (size_t index = 0; index < entity.child_count(); ++index) {
        if (!entity.child(index).ExtrusionEntity::empty()) {
            only_geometry = index;
            ++geometry_count;
        }
    }
    if (geometry_count == 1) {
        const ExtrusionEntity &child = entity.child(only_geometry);
        if (is_seam_unit(child) && can_choose_child_seam(entity, child, true)) {
            ExtrusionSeamPlan nested = plan_on_contour(child, requested);
            nested.delegation.insert(nested.delegation.begin(), only_geometry);
            return nested;
        }
        // A singleton contour governs its own permission, including a local
        // closed polyline: refusing delegation must not fall back to splitting it.
        return keep;
    }

    if (has_seam_barrier(entity))
        return keep;

    std::optional<ExtrusionSeamPlan> best;
    double best_distance = (std::numeric_limits<double>::max)();
    for (size_t index = 0; index < entity.child_count(); ++index) {
        const ExtrusionEntity &child = entity.child(index);
        if (child.ExtrusionEntity::empty())
            continue;

        ExtrusionSeamPlan candidate{child.ExtrusionEntity::first_point(),
                                   ExtrusionSeamPlan::Operation::RotateChildren, {}, index};
        if (const ArcPolyline *path = child.polyline_or_null()) {
            const Projection projected = project(child, requested);
            if (projected.valid && (child.can_rotate() || !child.ExtrusionEntity::is_loop())) {
                const SplitPreview split = preview_split(*path, projected.point);
                candidate.point = split.point;
                candidate.split_point = split.input;
                if (candidate.point == path->back() && candidate.point != path->front()) {
                    // Keep empty children in stored order, but begin with the next
                    // actual path so the published point equals the real entry.
                    size_t next = (index + 1) % entity.child_count();
                    while (entity.child(next).ExtrusionEntity::empty())
                        next = (next + 1) % entity.child_count();
                    candidate.child_index = next;
                    candidate.point = entity.child(next).ExtrusionEntity::first_point();
                } else if (candidate.point != path->front()) {
                    candidate.operation = ExtrusionSeamPlan::Operation::SplitChild;
                }
            }
        }
        // Composite children are atomic outside sole-contour delegation: only
        // their existing entry is offered, never a recursive rotation of an open path.
        if (candidate.point == start)
            candidate = keep;
        const double distance = distance_squared(candidate.point, requested);
        if (!best || distance < best_distance) {
            best = std::move(candidate);
            best_distance = distance;
        }
    }
    return best ? std::move(*best) : keep;
}

} // namespace ExtrusionSeamPlanningDetail

// Validate against the full current contour before restricting to materializable
// entries. Off-contour and ineligible requests remain errors, even behind a barrier.
inline std::optional<ExtrusionSeamPlan> legal_seam_entry(const ExtrusionEntity &entity,
                                                       const Point &requested,
                                                       const bool ancestors_allow_seams)
{
    if (!ancestors_allow_seams || !is_seam_unit(entity))
        return std::nullopt;
    const ExtrusionSeamPlanningDetail::Projection projected = ExtrusionSeamPlanningDetail::project(entity, requested);
    const double tolerance_squared = double(SCALED_EPSILON) * double(SCALED_EPSILON);
    if (!projected.valid || projected.distance_squared > tolerance_squared)
        return std::nullopt;
    return ExtrusionSeamPlanningDetail::plan_on_contour(entity, requested);
}

// Custom native analyses keep their own indices and scores. They may use this
// exact membership check, not score a projected point with another point's index.
inline bool is_directly_legal_seam(const ExtrusionEntity &entity, const Point &requested,
                                  const bool ancestors_allow_seams)
{
    const std::optional<ExtrusionSeamPlan> plan = legal_seam_entry(entity, requested, ancestors_allow_seams);
    return plan && plan->point == requested;
}

} // namespace Slic3r

#endif // slic3r_ExtrusionSeamPlanning_hpp_
