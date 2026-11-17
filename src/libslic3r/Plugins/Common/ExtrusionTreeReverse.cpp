///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ExtrusionTreeReverse.hpp"

#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace ExtrusionTree {
namespace {

void reverse(MutableExtrusionEntity entity, const bool keep_loops, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (entity.point_count() > 0) {
        throw_if_cancelled(run_ctx);
        if (!entity.reverse())
            throw std::runtime_error("The extrusion polyline could not be reversed.");
        return;
    }

    /* Reverse every path first, then reverse child order as one unit. The children of a loop are its pieces and turn
       with it; a closed child of any other node ends where it starts, so turning it around would not help. */
    const bool keep_closed_children = keep_loops && !entity.readonly().is_loop();
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        if (keep_closed_children && entity.child(child_idx).is_closed())
            continue;
        reverse(entity.child_mutable(child_idx), keep_loops, run_ctx);
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

} // namespace

void reverse_tree(MutableExtrusionEntity entity, const plugin_run_context *run_ctx)
{
    reverse(entity, false, run_ctx);
}

void reverse_tree_keeping_loops(MutableExtrusionEntity entity, const plugin_run_context *run_ctx)
{
    reverse(entity, true, run_ctx);
}

}} // namespace slic3r_api::ExtrusionTree
