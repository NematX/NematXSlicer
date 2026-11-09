///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_
#define slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Geometry queries and tree mutations for built-in extrusion-tree orderers.

These operations preserve existing entity handles whenever possible. Loop
rotation accepts seams on lines and arcs, splits only the segment containing
the seam, and preserves segment Z offsets. The caller owns the policy deciding
which seam, direction, or ordering should be selected.

A marked bridge strand is one continuous unit even when splitting gave it child
paths. Test atomic_strand() before descending into a tree; any permitted reversal
uses reverse_tree() on the complete strand, then final ordering can lock its flags.

Example after choosing to reverse a MutableExtrusionEntity strand:
    if (atomic_strand(strand.readonly()) && strand.readonly().reversible())
        reverse_tree(strand, run_ctx);
*/

namespace slic3r_api { namespace Ordering { namespace TreeOrderingGeometry {

// Return whether this node has a nonzero strand marker; callers must not reorder
// or reverse its descendants independently, even when the collection is reversible.
bool atomic_strand(const ExtrusionEntity &entity);

/* Reverse every local path and child order so the complete tree runs backward. */
void reverse_tree(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx = nullptr);

/* Compatibility entry point for guarded legal rotation. The actual entry can be
   the current start behind a barrier, a direct composite's existing entry, or a
   snapped leaf boundary; it need not equal the provider's requested point. */
void rotate_loop_to_seam(storage_handle *scratch_storage,
                         MutableExtrusionEntity loop,
                         c_point seam,
                         const plugin_run_context *run_ctx = nullptr);

/* Borrow loop, validate the original provider point, then return a legal entry
   without mutation or consumption. No scratch storage is retained. Invalid
   handles, permissions or off-contour requests throw; cancellation propagates.
   Ancestor permission is explicit and never inferred from legacy types. */
c_point legal_seam_point(const ExtrusionEntity &loop,
                         c_point requested,
                         bool ancestors_allow_seams,
                         const plugin_run_context *run_ctx = nullptr);

/* Recompute and materialize the legal description immediately before mutation;
   retain rotation permission for further geometry passes. Scratch storage owns
   temporary split paths; existing suffix/child handles survive success. The
   caller excludes concurrent edits. Invalid requests throw before mutation;
   cancellation and later allocation failures propagate without rollback. */
void rotate_loop_to_legal_seam(storage_handle *scratch_storage,
                              MutableExtrusionEntity loop,
                              c_point requested,
                              bool ancestors_allow_seams,
                              const plugin_run_context *run_ctx = nullptr);

/* Borrow direct parent/child views and read the shared continuity rule. Carry
   the result through nested wrappers for provider binding and route creation. */
bool can_choose_child_seam(const ExtrusionEntity &parent,
                           const ExtrusionEntity &child,
                           bool ancestors_allow_seams);

/* Consume only rotation on a borrowed finalized subtree without allocation or
   geometry edits. Cancellation may stop consumption between nodes. */
void disable_rotation_recursively(MutableExtrusionEntity entity,
                                  const plugin_run_context *run_ctx = nullptr);

/* Remove sorting, reversal and rotation permissions from finalized descendants. */
void disable_ordering_flags_recursively(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx = nullptr);

/*
Resolve the first source LayerRegion touched by a PrintingExtrusion.

The geometric test uses the unshifted source extrusion retained by
LayerRegionIsland, not the instance-shifted plan clone. This keeps both the
path and region slices in the source object's coordinate system. Region order
supplies the deterministic result when a path crosses several regions. The
scratch storage owns only temporary Clipper results and may be cleared after
this call; the returned LayerRegion remains borrowed from the source island.
*/
LayerRegion resolve_source_layer_region(storage_handle *scratch_storage,
                                        const PrintingExtrusion &extrusion);

/*
Resolve every source LayerRegion touched by one atomic loop from the plan.

loop uses platter coordinates, while the region slices use object coordinates;
the PrintingExtrusion instance shift is removed only for this test. Results
preserve source-island order and remain borrowed after scratch data is freed.
*/
std::vector<LayerRegion> resolve_source_layer_regions(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &loop);

}}} // namespace slic3r_api::Ordering::TreeOrderingGeometry

#endif // slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_
