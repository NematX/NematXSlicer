///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultExtrusionTreeOrdering.hpp"

#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Plugins/PrintingPlan/EntryPointProperty.h"
#include "ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/ExtrusionSeamPlanning.hpp"

namespace slic3r_api { namespace Ordering { namespace DefaultOrderingPlugin {
namespace {

/*
Default extrusion-tree ordering
===============================

The preceding STEP_ORDERING providers have selected the high-level order of
groups, layers, tools, and PrintingExtrusion roots. This provider fixes only
the order and direction of the children inside those roots. Non-sortable
subtrees remain atomic, so properties and forced sequencing stay attached to
their original subtree.

The provider walks the complete PrintingPlan sequentially and carries the
position where the previous extrusion finished. For each sortable node, it
repeatedly selects the remaining child whose legal entry point is closest to
that position, moves the child into the next output slot, and prepares it for
printing. Sortable children are processed recursively. Open reversible paths
may be started at either end, while closed loops are rotated to their closest
stored vertex.

Only existing vertices are used as loop entry candidates. This avoids creating
new segments and preserves the additional geometry stored by the extrusion
representation, such as arcs and per-point Z offsets. Once a sortable node has
been converted into a concrete sequence, its sorting, reversal and rotation flags
are disabled so downstream code cannot silently change the selected order. The
exact front and back of every processed root are published through
EntryPointProperty for the next root and for later pipeline stages.

The normal call flow is:

    DefaultExtrusionTreeOrdering::run_impl()
    `-- walk PrintingPlan groups, layers, tools, and extrusion roots
        |-- order_sortable_extrusion_children()
        |   |-- best_entry_candidate() for each remaining child
        |   |   |-- sortable node: inspect its children recursively
        |   |   |-- loop: collect stored vertices and choose the nearest one
        |   |   `-- open path: compare front() and back() when reversible
        |   |-- move the selected child into the fixed prefix
        |   `-- prepare_child_for_entry()
        |       |-- rotate_loop_to_legal_seam() for loops
        |       |-- reverse() for a selected reverse candidate
        |       `-- recurse into sortable children
        `-- publish EntryPointProperty{root.front(), root.back()}

This is a deterministic nearest-entry heuristic, not a global optimizer. Its
result depends on the current start position and on the order of equal-cost
children.
Seam candidates must satisfy exact stored closure and independent rotation
permission. Shared legal-entry planning rejects vertices inside an atomic open
composite or across an event barrier. Finalizing a seam consumes rotation only;
no borrowed tree view or scratch path survives the callback.
*/

const char *k_no_dependencies[] = { nullptr };
const char *k_group_extrusions = "ordering.extrusion_tree";

struct ExtrusionEntryCandidate
{
    c_point point = {};
    double score = (std::numeric_limits<double>::max)();
    bool reverse_before_printing = false;
    bool valid = false;
};

/*
Order the direct children of one sortable node from the given current point.

This is the core algorithm of the plugin. It greedily selects the child that
can start closest to the current nozzle position, prepares that child to really
start there, and moves it into the fixed prefix of the same parent.
*/
void order_sortable_extrusion_children(storage_handle *storage,
                                       MutableExtrusionEntity entity,
                                       c_point start_near,
                                       const plugin_run_context *run_ctx);

/* Prepare one selected child before it is considered fixed in its parent. */
void prepare_child_for_entry(storage_handle *storage,
                             MutableExtrusionEntity child,
                             const ExtrusionEntryCandidate &candidate,
                             const plugin_run_context *run_ctx);

/*
Find the best legal start point a child can present to a sortable parent.

Open reversible paths may start from either end. Loops may start from any stored
vertex. Sortable children expose the best candidate of their descendants because
they will be ordered recursively after they are selected.
*/
ExtrusionEntryCandidate best_entry_candidate(
    const ExtrusionEntity &entity,
    c_point current_point,
    const plugin_run_context *run_ctx);

/*
Collect loop entry candidates from the stored vertices of a loop tree.

The plugin intentionally does not split paths at projected closest points. It
uses existing vertices only, which keeps arc and Z-offset data aligned with the
polyline API.
*/
void collect_loop_entry_points(
    const ExtrusionEntity &entity,
    std::vector<c_point> &points,
    const plugin_run_context *run_ctx);

/* Squared distance is enough for nearest-candidate comparison and avoids sqrt. */
double distance_to_square(c_point lhs, c_point rhs);

/* Count roots in the complete plan before the sequential ordering pass. */
uint32_t extrusion_count(
    const ConstPrintingPlan &plan,
    const plugin_run_context *run_ctx);

void order_sortable_extrusion_children(storage_handle *storage,
                                       MutableExtrusionEntity entity,
                                       c_point start_near,
                                       const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    c_point current_point = start_near;
    uint32_t fixed_count = 0;

    while (fixed_count < entity.child_count()) {
        throw_if_cancelled(run_ctx);
        /*
        Children before fixed_count are already ordered and must not move again.
        Children from fixed_count to the end are candidates for the next printed
        subtree.
        */
        uint32_t selected_idx = EXTRUSION_INDEX_INVALID;
        ExtrusionEntryCandidate selected_candidate;

        for (uint32_t child_idx = fixed_count; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            const ExtrusionEntryCandidate candidate =
                best_entry_candidate(entity.child(child_idx), current_point, run_ctx);
            if (!candidate.valid)
                continue;
            if (selected_idx == EXTRUSION_INDEX_INVALID || candidate.score < selected_candidate.score) {
                selected_idx = child_idx;
                selected_candidate = candidate;
            }
        }

        if (selected_idx == EXTRUSION_INDEX_INVALID)
            break;

        /*
        Move the selected child into the fixed prefix before mutating it. That
        way recursive ordering or loop rotation works on the child at its final
        parent index.
        */
        throw_if_cancelled(run_ctx);
        const uint32_t moved_idx = entity.move_child_from(fixed_count, entity, selected_idx);
        assert(moved_idx != EXTRUSION_INDEX_INVALID);
        MutableExtrusionEntity child = entity.child_mutable(fixed_count);

        prepare_child_for_entry(storage, child, selected_candidate, run_ctx);
        throw_if_cancelled(run_ctx);
        if (!child.empty())
            current_point = child.back();
        ++fixed_count;
    }

    /*
    This node now contains a concrete printing sequence. Disable later automatic
    sorting, reversal and rotation so downstream code observes the order selected here.
    */
    throw_if_cancelled(run_ctx);
    entity.disable_sort();
    entity.disable_reverse();
    entity.set_can_rotate(false);
}

void prepare_child_for_entry(storage_handle *storage,
                             MutableExtrusionEntity child,
                             const ExtrusionEntryCandidate &candidate,
                             const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    assert(candidate.valid);

    if (child.readonly().sortable()) {
        /*
        A sortable child can choose its own first descendant. Recursing here
        turns the child subtree into a fixed sequence starting from the point
        chosen by the parent.
        */
        order_sortable_extrusion_children(storage, child, candidate.point, run_ctx);
        return;
    }

    /*
    Non-sortable children are atomic. The only legal preparation is changing
    their entry point without changing their internal ordering: rotate a loop or
    reverse an explicitly reversible open path.
    */
    if (child.readonly().is_seam_unit()) {
        TreeOrderingGeometry::rotate_loop_to_legal_seam(storage, child, candidate.point, true, run_ctx);
        TreeOrderingGeometry::disable_rotation_recursively(child, run_ctx);
    } else if (candidate.reverse_before_printing) {
        throw_if_cancelled(run_ctx);
        child.reverse();
    }
}

ExtrusionEntryCandidate best_entry_candidate(
    const ExtrusionEntity &entity,
    c_point current_point,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    ExtrusionEntryCandidate best;
    if (entity.empty())
        return best;

    if (entity.sortable()) {
        /*
        A sortable child does not have a fixed first point yet. Ask its children
        what entry point they could present after recursive ordering, and report
        the best of those possibilities to this parent.
        */
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            ExtrusionEntryCandidate candidate = best_entry_candidate(
                entity.child(child_idx), current_point, run_ctx);
            if (candidate.valid && (!best.valid || candidate.score < best.score)) {
                candidate.reverse_before_printing = false;
                best = candidate;
            }
        }
        return best;
    }

    if (entity.is_seam_unit()) {
        std::vector<c_point> candidates;
        collect_loop_entry_points(entity, candidates, run_ctx);
        if (candidates.empty())
            candidates.push_back(entity.front());

        /*
        Loop entry is chosen from existing vertices only. This keeps the
        operation cheap and retains the policy's exact stored-vertex scores,
        rather than substituting a projected point for a different candidate.
        */
        for (c_point point : candidates) {
            throw_if_cancelled(run_ctx);
            const Slic3r::ExtrusionEntity *core = reinterpret_cast<const Slic3r::ExtrusionEntity *>(entity.handle());
            if (!Slic3r::is_directly_legal_seam(*core, Slic3r::Point(point.x, point.y), true))
                continue;
            ExtrusionEntryCandidate candidate;
            candidate.point = point;
            candidate.score = distance_to_square(point, current_point);
            candidate.valid = true;
            if (!best.valid || candidate.score < best.score)
                best = candidate;
        }
        return best;
    }

    /*
    Open paths normally start at front(). If the whole entity is reversible,
    back() is also a legal start, and prepare_child_for_entry() will reverse it
    only if that candidate wins.
    */
    best.point = entity.front();
    best.score = distance_to_square(best.point, current_point);
    best.valid = true;

    if (entity.reversible()) {
        throw_if_cancelled(run_ctx);
        ExtrusionEntryCandidate reversed;
        reversed.point = entity.back();
        reversed.score = distance_to_square(reversed.point, current_point);
        reversed.reverse_before_printing = true;
        reversed.valid = true;
        if (reversed.score < best.score)
            best = reversed;
    }

    return best;
}

void collect_loop_entry_points(
    const ExtrusionEntity &entity,
    std::vector<c_point> &points,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (entity.point_count() > 0) {
        const bool skip_duplicate_last =
            entity.point_count() > 1 && points_equal(entity.local_front(), entity.local_back());
        const uint32_t point_count = skip_duplicate_last ? entity.point_count() - 1 : entity.point_count();
        /*
        Closed polylines usually repeat the first vertex as the last vertex.
        Exposing both as candidates would give two equivalent starts with
        different split behavior, so the duplicate last point is skipped.
        */
        for (uint32_t point_idx = 0; point_idx < point_count; ++point_idx) {
            throw_if_cancelled(run_ctx);
            points.push_back(entity.point(point_idx));
        }
        return;
    }

    // Collection loop: candidates are the stored vertices of every child path.
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        collect_loop_entry_points(entity.child(child_idx), points, run_ctx);
    }
}

double distance_to_square(c_point lhs, c_point rhs)
{
    const double dx = double(lhs.x) - double(rhs.x);
    const double dy = double(lhs.y) - double(rhs.y);
    return dx * dx + dy * dy;
}

uint32_t extrusion_count(
    const PrintingPlan &plan,
    const plugin_run_context *run_ctx)
{
    uint32_t count = 0;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_ctx);
                count += layer.tool_group(tool_idx).extrusion_count();
            }
        }
    }
    return count;
}

class DefaultExtrusionTreeOrdering : public PluginBase
{
public:
    static DefaultExtrusionTreeOrdering &instance(orchestrator_handle *orch)
    {
        static DefaultExtrusionTreeOrdering s_instance(orch);
        return s_instance;
    }

    explicit DefaultExtrusionTreeOrdering(orchestrator_handle *orch) :
        PluginBase(orch),
        m_entry_point_property(entry_point_property_key(orch))
    {
    }

private:
    const char *id_impl() const noexcept override
    {
        return "ordering.extrusion_tree.default";
    }
    const char *name_impl() const noexcept override { return "Default extrusion-tree ordering"; }
    const char *description_impl() const noexcept override
    {
        return "Fixes sortable extrusion trees and publishes their exact entry and exit points.";
    }
    const char *exclusive_group_impl() const noexcept override { return k_group_extrusions; }
    slicing_step_t step_impl() const noexcept override { return STEP_ORDERING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Return the common extrusion-ordering priority. Nothing is changed or transferred, and the call cannot fail.
    int32_t priority_impl() const noexcept override { return 1050; }
    // Return the default algorithm's preference. Nothing is changed or transferred, and the call cannot fail.
    int32_t exclusive_group_priority_impl() const noexcept override { return 20; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Ordering extrusion trees: %u / %u roots";
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_extrusion_ordering *ctx = plugin_ctx_as_extrusion_ordering(run_ctx);
        assert(ctx != nullptr);
        assert(ctx == nullptr || ctx->plan != nullptr);
        if (ctx == nullptr || ctx->plan == nullptr)
            return;

        assert(run_ctx != nullptr);
        assert(run_ctx->plugin_storage != nullptr);
        if (run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;

        throw_if_cancelled(run_ctx);
        PrintingPlan plan(ctx->plan);
        progress().add_max(extrusion_count(plan, run_ctx));
        c_point current_point = {};
        bool has_current_point = false;

        for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
            throw_if_cancelled(run_ctx);
            PrintingGroup group = plan.group(group_idx);
            for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
                throw_if_cancelled(run_ctx);
                PrintingLayerGroup layer = group.layer_group(layer_idx);
                for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                    throw_if_cancelled(run_ctx);
                    PrintingToolGroup tool = layer.tool_group(tool_idx);
                    for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                        throw_if_cancelled(run_ctx);
                        MutableExtrusionEntity root = tool.extrusion(extrusion_idx).mutable_root();
                        if (root.empty()) {
                            throw_if_cancelled(run_ctx);
                            m_entry_point_property.remove(root);
                            progress().increment();
                            continue;
                        }

                        /*
                        The pre-sort estimate gives the first root a useful
                        approach direction. Once one root has been fixed, its
                        exact exit is a better start for every following root.
                        */
                        if (!has_current_point) {
                            const EntryPointProperty *estimate =
                                m_entry_point_property.get(root);
                            if (estimate != nullptr)
                                current_point = estimate->entry;
                            has_current_point = true;
                        }

                        if (root.readonly().sortable())
                            order_sortable_extrusion_children(
                                run_ctx->plugin_storage, root, current_point, run_ctx);

                        throw_if_cancelled(run_ctx);
                        EntryPointProperty &entry_points =
                            m_entry_point_property.get_or_add(root);
                        entry_points.entry = root.front();
                        entry_points.exit = root.back();
                        current_point = entry_points.exit;
                        progress().increment();
                    }
                }
            }
        }
    }

    PluginPropertyKey<EntryPointProperty> m_entry_point_property;
};

} // namespace

void register_default_extrusion_tree_ordering_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, DefaultExtrusionTreeOrdering::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::Ordering::DefaultOrderingPlugin
