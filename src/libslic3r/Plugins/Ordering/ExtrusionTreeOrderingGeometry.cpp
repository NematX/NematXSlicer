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
#include "libslic3r/ExtrusionSeamPlanning.hpp"

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

    rotate_loop_to_seam()
        `-- rotate_loop_to_legal_seam() [use the same guarded geometry path]

    legal_seam_point() / rotate_loop_to_legal_seam()
        |-- legal_seam_entry() [validate the request and preview exact split output]
        `-- follow delegation, then rotate local geometry or direct children
            `-- split a direct leaf only when no event would be duplicated

Scratch storage owns temporary paths; input trees remain borrowed during each
call. Cancellation and invalid loop geometry propagate to the ordering provider.
*/

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

c_point legal_seam_point(const ExtrusionEntity &loop,
                         const c_point requested,
                         const bool ancestors_allow_seams,
                         const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!loop.valid())
        throw std::invalid_argument("Seam planning needs a valid extrusion tree.");
    const Slic3r::ExtrusionEntity *core = reinterpret_cast<const Slic3r::ExtrusionEntity *>(loop.handle());
    const std::optional<Slic3r::ExtrusionSeamPlan> plan =
        Slic3r::legal_seam_entry(*core, Slic3r::Point(requested.x, requested.y), ancestors_allow_seams);
    if (!plan)
        throw std::runtime_error("The seam request is outside the contour or its rotation permissions.");
    throw_if_cancelled(run_ctx);
    return {plan->point.x(), plan->point.y()};
}

void rotate_loop_to_legal_seam(storage_handle *scratch_storage,
                              MutableExtrusionEntity loop,
                              const c_point requested,
                              const bool ancestors_allow_seams,
                              const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!loop.valid())
        throw std::invalid_argument("Seam materialization needs a valid extrusion tree.");
    const Slic3r::ExtrusionEntity *core = reinterpret_cast<const Slic3r::ExtrusionEntity *>(loop.handle());
    const std::optional<Slic3r::ExtrusionSeamPlan> plan =
        Slic3r::legal_seam_entry(*core, Slic3r::Point(requested.x, requested.y), ancestors_allow_seams);
    if (!plan)
        throw std::runtime_error("The seam request is outside the contour or its rotation permissions.");

    MutableExtrusionEntity target = loop;
    for (const size_t index : plan->delegation) {
        throw_if_cancelled(run_ctx);
        target = target.child_mutable(uint32_t(index));
    }
    const c_point seam{plan->point.x(), plan->point.y()};
    const c_point split_point{plan->split_point.x(), plan->split_point.y()};
    using Operation = Slic3r::ExtrusionSeamPlan::Operation;
    if (plan->operation == Operation::RotateLocal) {
        rotate_local_loop_to_seam(scratch_storage, target, split_point, run_ctx);
    } else if (plan->operation != Operation::KeepStart) {
        std::unique_ptr<StoredExtrusionEntity> closing_piece;
        if (plan->operation == Operation::SplitChild) {
            throw_if_cancelled(run_ctx);
            MutableExtrusionEntity child = target.child_mutable(uint32_t(plan->child_index));
            const Slic3r::ExtrusionEntity *child_core = reinterpret_cast<const Slic3r::ExtrusionEntity *>(child.handle());
            if (Slic3r::has_one_shot_extrusion_event(*child_core))
                throw std::runtime_error("A seam split must not duplicate a one-shot extrusion event.");
            StoredExtrusionEntity before(scratch_storage, child.readonly());
            StoredExtrusionEntity after(scratch_storage, child.readonly());
            if (extrusion_polyline_split_at_point(child.handle(), split_point, before.mutable_handle(), after.mutable_handle()) == 0 ||
                before.point_count() < 2 || after.point_count() < 2 ||
                !points_equal(before.local_back(), seam) || !points_equal(after.local_front(), seam))
                throw std::runtime_error("The seam split did not materialize the planned entry.");
            throw_if_cancelled(run_ctx);
            if (extrusion_move_from(child.mutable_handle(), after.mutable_handle()) == 0)
                throw std::runtime_error("The seam suffix could not retain the source handle.");
            closing_piece.reset(new StoredExtrusionEntity(std::move(before)));
        }
        for (size_t index = 0; index < plan->child_index; ++index) {
            throw_if_cancelled(run_ctx);
            if (target.move_child_from(target.child_count(), target, 0) == EXTRUSION_INDEX_INVALID)
                throw std::runtime_error("The extrusion loop rejected its planned child rotation.");
        }
        throw_if_cancelled(run_ctx);
        if (closing_piece && target.append_child_move(closing_piece->mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("The extrusion loop rejected its planned closing fragment.");
    }
    if (!points_equal(loop.front(), seam))
        throw std::runtime_error("The extrusion loop entry differs from its legal seam plan.");
}

bool can_choose_child_seam(const ExtrusionEntity &parent,
                           const ExtrusionEntity &child,
                           const bool ancestors_allow_seams)
{
    const Slic3r::ExtrusionEntity *core_parent = reinterpret_cast<const Slic3r::ExtrusionEntity *>(parent.handle());
    const Slic3r::ExtrusionEntity *core_child = reinterpret_cast<const Slic3r::ExtrusionEntity *>(child.handle());
    return core_parent && core_child &&
        Slic3r::can_choose_child_seam(*core_parent, *core_child, ancestors_allow_seams);
}

void disable_rotation_recursively(MutableExtrusionEntity entity, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        disable_rotation_recursively(entity.child_mutable(index), run_ctx);
    entity.set_can_rotate(false);
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
    // Keep the internal compatibility entry point, not a second unguarded algorithm.
    rotate_loop_to_legal_seam(scratch_storage, loop, seam, true, run_ctx);
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
    entity.set_can_rotate(false);
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
