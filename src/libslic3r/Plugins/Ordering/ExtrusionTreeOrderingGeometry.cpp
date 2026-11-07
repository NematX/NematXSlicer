///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Api/plugin/cpp/properties/BridgeProperties.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ExtrusionEntity.hpp"

namespace slic3r_api { namespace Ordering { namespace TreeOrderingGeometry {
namespace {

/*
Extrusion-tree ordering geometry
================================

Ordering providers decide where a path should begin and whether it should run
forward or backward. This module performs those decisions without adding a
second policy layer. Tree reversal preserves every existing handle. Loop
rotation projects an already validated seam onto lines or arcs, splits only the
containing path when necessary, and keeps segment metadata aligned.

The same module also connects plan geometry to its source LayerRegion. A
PrintingExtrusion owns a transformed clone for printing, while its
LayerRegionIsland retains the unshifted source extrusion and region slices.
Matching those two source-side objects avoids mixing coordinate systems before
the selected region is passed to a SeamPlacer.

A collection marked with BridgeAtomicStrand represents one continuous split
bridge row. Treat it as a single candidate: do not spatially sort its children or
choose their directions independently. Any permitted reversal applies to the
whole sequence through reverse_tree(), then final ordering locks its descendant
permissions. The marker is not permission to reverse an otherwise fixed path.


Representative direct calls:

    atomic_strand()
        `-- entity.get() [BridgeAtomicStrand property]

    reverse_tree()
        |-- entity.reverse() [path leaf]
        `-- reverse_tree() [recursive child paths before reversing child order]
*/

/* Find the nearest point on any local line or arc in one entity subtree. */
bool projected_point_on_entity(const ExtrusionEntity &entity,
                               c_point requested,
                               c_point &projected,
                               double &distance_squared,
                               const plugin_run_context *run_ctx);

/* Rotate a loop stored as one local polyline by splitting at its seam. */
void rotate_local_loop_to_seam(storage_handle *scratch_storage,
                               MutableExtrusionEntity loop,
                               c_point seam,
                               const plugin_run_context *run_ctx);

/* Compare two points without converting their scaled coordinates. */
double squared_distance(c_point lhs, c_point rhs);

/*
Return whether any source path overlaps or lies inside one region slice.

Temporary linearized paths and Clipper outputs belong to scratch_storage. The
input tree and region remain borrowed and are never mutated.
*/
bool entity_intersects_region(storage_handle *scratch_storage,
                              const ExtrusionEntity &entity,
                              const LayerRegion &region,
                              c_point shift);

bool projected_point_on_entity(const ExtrusionEntity &entity,
                               const c_point requested,
                               c_point &projected,
                               double &distance_squared_out,
                               const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    bool found_projection = false;
    double best_distance = (std::numeric_limits<double>::max)();
    c_point best_point = {};

    if (entity.point_count() > 0) {
        /* ArcWelder performs the same projection for straight and curved segments. */
        const Slic3r::ExtrusionEntity *core_entity =
            reinterpret_cast<const Slic3r::ExtrusionEntity *>(entity.handle());
        const Slic3r::ArcPolyline path = core_entity->as_polyline();
        const Slic3r::Geometry::ArcWelder::PathSegmentProjection projection =
            Slic3r::Geometry::ArcWelder::point_to_path_projection(
                path.get_arc(), Slic3r::Point(requested.x, requested.y));
        if (projection.valid()) {
            best_point = c_point{projection.point.x(), projection.point.y()};
            best_distance = projection.distance2;
            found_projection = true;
        }
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        c_point child_point = {};
        double child_distance = (std::numeric_limits<double>::max)();
        if (projected_point_on_entity(
                entity.child(child_idx), requested, child_point, child_distance, run_ctx) &&
            (!found_projection || child_distance < best_distance)) {
            best_point = child_point;
            best_distance = child_distance;
            found_projection = true;
        }
    }

    if (found_projection) {
        projected = best_point;
        distance_squared_out = best_distance;
    }
    return found_projection;
}

void rotate_local_loop_to_seam(storage_handle *scratch_storage,
                               MutableExtrusionEntity loop,
                               const c_point seam,
                               const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    StoredExtrusionEntity before(scratch_storage, loop.readonly());
    StoredExtrusionEntity after(scratch_storage, loop.readonly());
    if (extrusion_polyline_split_at_point(
            loop.handle(), seam, before.mutable_handle(), after.mutable_handle()) == 0)
        throw std::runtime_error(
            "The local extrusion loop could not be split at its seam.");

    c_point projected = {};
    if (before.point_count() > 0)
        projected = before.local_back();
    else if (after.point_count() > 0)
        projected = after.local_front();
    else
        throw std::runtime_error(
            "The local extrusion loop produced no seam projection.");

    const double tolerance_squared =
        double(SCALED_EPSILON) * double(SCALED_EPSILON);
    if (squared_distance(seam, projected) > tolerance_squared)
        throw std::runtime_error(
            "The seam placer returned a point outside the local loop.");
    if (points_equal(projected, loop.local_front()) ||
        points_equal(projected, loop.local_back()))
        return;

    /* Recompose suffix + prefix while retaining arcs and endpoint Z offsets. */
    std::vector<c_extrusion_segment> rotated_segments = after.segments();
    const std::vector<c_extrusion_segment> prefix_segments = before.segments();
    rotated_segments.insert(
        rotated_segments.end(), prefix_segments.begin(), prefix_segments.end());
    if (!loop.set_segments(rotated_segments))
        throw std::runtime_error(
            "The local extrusion loop rejected its rotated segments.");
}

double squared_distance(const c_point lhs, const c_point rhs)
{
    const double dx = double(lhs.x) - double(rhs.x);
    const double dy = double(lhs.y) - double(rhs.y);
    return dx * dx + dy * dy;
}

bool entity_intersects_region(storage_handle *scratch_storage,
                              const ExtrusionEntity &entity,
                              const LayerRegion &region,
                              const c_point shift)
{
    const ExPolygonCollection slices = region.slices();
    if (entity.has_polyline() && !slices.empty()) {
        // Region matching needs the complete geometric arc, unlike fixed-direction seam
        // selection which deliberately considers only stored vertices. This is
        // private built-in code, so it may inspect and linearize the native
        // source path without changing the public extrusion tree.
        const Slic3r::ExtrusionEntity *source =
            reinterpret_cast<const Slic3r::ExtrusionEntity *>(entity.handle());
        const Slic3r::Polyline linearized = source->as_polyline().to_polyline();
        StoredPolyline path(scratch_storage);
        for (const Slic3r::Point &point : linearized.points)
            path.push_back(c_point{point.x() - shift.x, point.y() - shift.y});

        const StoredPolylineCollection overlap =
            clipper_intersection_polyline_expolygons(
                scratch_storage, path.readonly(), slices);
        for (uint32_t overlap_idx = 0; overlap_idx < overlap.size(); ++overlap_idx) {
            if (!overlap[overlap_idx].empty())
                return true;
        }

        // A path reduced to boundary contacts may produce no line fragment.
        // Point containment keeps such a geometrically associated region from
        // being lost without widening the test to an arbitrary tolerance.
        for (uint32_t point_idx = 0; point_idx < path.size(); ++point_idx) {
            for (uint32_t slice_idx = 0; slice_idx < slices.size(); ++slice_idx) {
                if (slices[slice_idx].contains(path[point_idx]))
                    return true;
            }
        }
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        if (entity_intersects_region(
                scratch_storage, entity.child(child_idx), region, shift))
            return true;
    }
    return false;
}

} // namespace

bool atomic_strand(const ExtrusionEntity &entity)
{
    const BridgeAtomicStrand *property=entity.get(BridgeAtomicStrand::key);
    return property!=nullptr && property->atomic_orientation!=0;
}

void reverse_tree(MutableExtrusionEntity entity, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (entity.point_count() > 0) {
        throw_if_cancelled(run_ctx);
        if (!entity.reverse())
            throw std::runtime_error("The extrusion polyline could not be reversed.");
        return;
    }

    /* Reverse every path first, then reverse child order as one unit. */
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        reverse_tree(entity.child_mutable(child_idx), run_ctx);
    }

    std::vector<const extrusion_entity_handle *> desired;
    std::vector<const extrusion_entity_handle *> current;
    desired.reserve(entity.child_count());
    current.reserve(entity.child_count());
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        current.push_back(entity.child(child_idx).handle());
    }
    for (uint32_t child_idx = entity.child_count(); child_idx > 0; --child_idx) {
        throw_if_cancelled(run_ctx);
        desired.push_back(entity.child(child_idx - 1).handle());
    }

    for (uint32_t target_idx = 0; target_idx < desired.size(); ++target_idx) {
        throw_if_cancelled(run_ctx);
        if (current[target_idx] == desired[target_idx])
            continue;
        uint32_t source_idx = target_idx + 1;
        while (source_idx < current.size() && current[source_idx] != desired[target_idx]) {
            throw_if_cancelled(run_ctx);
            ++source_idx;
        }
        throw_if_cancelled(run_ctx);
        if (source_idx == current.size() ||
            entity.move_child_from(target_idx, entity, source_idx) != target_idx)
            throw std::runtime_error(
                "The extrusion collection could not be reversed.");
        const extrusion_entity_handle *moved = current[source_idx];
        current.erase(current.begin() + source_idx);
        current.insert(current.begin() + target_idx, moved);
    }
}

void rotate_loop_to_seam(storage_handle *scratch_storage,
                         MutableExtrusionEntity loop,
                         const c_point seam,
                         const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (loop.point_count() > 0) {
        rotate_local_loop_to_seam(scratch_storage, loop, seam, run_ctx);
        return;
    }
    if (loop.child_count() == 0)
        throw std::runtime_error("A seam cannot be materialized on an empty loop.");

    /*
    Structural wrappers may place property-only events beside one geometric
    child. When that child is already a complete loop, rotate its local or
    nested geometry directly. Splitting it at the wrapper level would retain
    every provisional seam as another persistent child after the final pass.

    A parent with more than one non-empty child remains a genuinely composed
    loop. Its child boundaries may carry different properties, so the generic
    reordering and closing-fragment algorithm below must preserve them.
    */
    MutableExtrusionEntity nested_loop;
    bool has_other_geometry = false;
    for (uint32_t child_idx = 0; child_idx < loop.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        MutableExtrusionEntity child = loop.child_mutable(child_idx);
        if (child.empty())
            continue;
        if (nested_loop.valid() || !child.readonly().is_loop()) {
            has_other_geometry = true;
            break;
        }
        nested_loop = child;
    }
    if (nested_loop.valid() && !has_other_geometry) {
        rotate_loop_to_seam(scratch_storage, nested_loop, seam, run_ctx);
        return;
    }

    /* Select the direct child nearest to the seam; child order breaks ties. */
    uint32_t containing_idx = EXTRUSION_INDEX_INVALID;
    c_point projected = {};
    double best_distance = (std::numeric_limits<double>::max)();
    for (uint32_t child_idx = 0; child_idx < loop.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        c_point child_projection = {};
        double child_distance = (std::numeric_limits<double>::max)();
        if (projected_point_on_entity(
                loop.child(child_idx), seam, child_projection, child_distance, run_ctx) &&
            child_distance < best_distance) {
            containing_idx = child_idx;
            projected = child_projection;
            best_distance = child_distance;
        }
    }
    const double tolerance_squared =
        double(SCALED_EPSILON) * double(SCALED_EPSILON);
    if (containing_idx == EXTRUSION_INDEX_INVALID ||
        best_distance > tolerance_squared)
        throw std::runtime_error(
            "The seam placer returned a point outside the extrusion loop.");

    uint32_t start_idx = containing_idx;
    std::unique_ptr<StoredExtrusionEntity> closing_piece;
    MutableExtrusionEntity child = loop.child_mutable(containing_idx);
    if (child.point_count() > 0) {
        if (points_equal(projected, child.local_front())) {
            start_idx = containing_idx;
        } else if (points_equal(projected, child.local_back())) {
            start_idx = (containing_idx + 1) % loop.child_count();
        } else {
            /* Keep the source handle for the suffix and append a cloned prefix. */
            StoredExtrusionEntity before(scratch_storage, child.readonly());
            StoredExtrusionEntity after(scratch_storage, child.readonly());
            if (extrusion_polyline_split_at_point(
                    child.handle(), projected,
                    before.mutable_handle(), after.mutable_handle()) == 0 ||
                extrusion_move_from(child.mutable_handle(), after.mutable_handle()) == 0)
                throw std::runtime_error(
                    "The extrusion loop could not be split at its seam.");
            closing_piece.reset(new StoredExtrusionEntity(std::move(before)));
            start_idx = containing_idx;
        }
    } else {
        rotate_loop_to_seam(scratch_storage, child, projected, run_ctx);
    }

    /* Move the original prefix behind the selected child without cloning it. */
    for (uint32_t moved_count = 0; moved_count < start_idx; ++moved_count) {
        throw_if_cancelled(run_ctx);
        const uint32_t moved_idx = loop.move_child_from(loop.child_count(), loop, 0);
        if (moved_idx == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error(
                "The extrusion loop rejected its seam rotation.");
    }
    throw_if_cancelled(run_ctx);
    if (closing_piece != nullptr &&
        loop.append_child_move(closing_piece->mutable_view()) == EXTRUSION_INDEX_INVALID)
        throw std::runtime_error(
            "The extrusion loop rejected its closing seam fragment.");
}

void disable_ordering_flags_recursively(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        disable_ordering_flags_recursively(entity.child_mutable(child_idx), run_ctx);
    }
    throw_if_cancelled(run_ctx);
    entity.disable_sort();
    entity.disable_reverse();
}

LayerRegion resolve_source_layer_region(storage_handle *scratch_storage,
                                        const PrintingExtrusion &extrusion)
{
    if (scratch_storage == nullptr || !extrusion.valid())
        throw std::invalid_argument(
            "LayerRegion resolution needs storage and a valid PrintingExtrusion.");

    // The source island groups both candidate regions and role-specific source
    // extrusions. The plan root is a clone and may already carry an instance
    // transform, so it is deliberately not used for this spatial test.
    const LayerRegionIsland source_island = extrusion.region_island();
    const extrusion_entity_handle *source_handle = source_island.valid() ?
        source_island.extrusion(extrusion.role()) : nullptr;
    if (source_handle == nullptr)
        throw std::runtime_error(
            "A PrintingExtrusion has no source geometry for LayerRegion resolution.");

    const ExtrusionEntity source(source_handle);
    // Island order is the public deterministic tie-break when one path touches
    // several regions. Return a borrowed view; scratch data may be cleared.
    for (uint32_t region_idx = 0; region_idx < source_island.region_count(); ++region_idx) {
        const LayerRegion region = source_island.region(region_idx);
        if (region.valid() &&
            entity_intersects_region(scratch_storage, source, region, {}))
            return region;
    }

    throw std::runtime_error(
        "A PrintingExtrusion does not intersect any LayerRegion from its source island.");
}

std::vector<LayerRegion> resolve_source_layer_regions(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &loop)
{
    if (scratch_storage == nullptr || !extrusion.valid() || !loop.is_loop())
        throw std::invalid_argument(
            "Loop LayerRegion resolution needs storage, a PrintingExtrusion, and an atomic loop.");

    const LayerRegionIsland source_island = extrusion.region_island();
    if (!source_island.valid() || source_island.region_count() == 0)
        throw std::runtime_error(
            "A PrintingExtrusion loop has no source LayerRegionIsland.");

    const LayerRegion first_region = source_island.region(0);
    if (!first_region.valid())
        throw std::runtime_error(
            "A PrintingExtrusion loop has an invalid first source LayerRegion.");
    const Object object = first_region.layer().object();
    if (!object.valid())
        throw std::runtime_error(
            "A PrintingExtrusion loop has an invalid source object instance.");
    const c_point shift = plan.object_instance_shift(extrusion);

    std::vector<LayerRegion> result;
    result.reserve(source_island.region_count());
    for (uint32_t region_idx = 0; region_idx < source_island.region_count(); ++region_idx) {
        const LayerRegion region = source_island.region(region_idx);
        if (region.valid() &&
            entity_intersects_region(scratch_storage, loop, region, shift))
            result.push_back(region);
    }
    if (result.empty())
        throw std::runtime_error(
            "A PrintingExtrusion loop does not intersect any LayerRegion from its source island.");
    return result;
}

}}} // namespace slic3r_api::Ordering::TreeOrderingGeometry
