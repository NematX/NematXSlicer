///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Post-slicing step host side: callbacks by which plugins rewrite layer geometry after slicing. The three
// mutators (assign islands, recompute slices from islands, recompute slices and islands from layer regions)
// are transactional natively and contain any failure through Detail::contain_callback_failure(); the four
// borrowers only hand out host-owned objects. All callbacks are noexcept because they are called through
// the C ABI.

#include "PostSlicingStep.hpp"

#include <utility>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::ApiHost::Steps {
namespace {

// Replace the islands of a layer by the contents of in_out_islands. set_islands() is transactional: on
// failure (bad_alloc) the layer and the source collection are unchanged; the failure is handed to
// contain_callback_failure(), which stops the step. The source is cleared only after success. When the
// plugin stop is requested or the print is cancelled, it returns at once: the layer and the source are
// untouched and no message is added (a cancelled print is not an error; the runner rethrows it).
void layer_assign_islands_by_moving_contents(layer_handle *me, expolygon_collection_handle *in_out_islands) noexcept
{
    if (me == nullptr || in_out_islands == nullptr)
        return;

    Layer &layer = *reinterpret_cast<Layer *>(me);
    if (Orchestrator::instance().is_plugin_cancelled() || layer.object()->print()->canceled())
        return;

    ExPolygons *islands = reinterpret_cast<ExPolygons *>(in_out_islands);
    try {
        ApiInternal::LayerAccess::set_islands(*reinterpret_cast<Layer *>(me), std::move(*islands));
    } catch (...) {
        Slic3r::Steps::Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_SLICING);
        return;
    }
    islands->clear();
}

// Rebuild the layer slices from its islands. The native call is transactional: on failure the layer is
// unchanged and contain_callback_failure() stops the step. Like the other mutators, it returns at once
// when the plugin stop is requested or the print is cancelled.
void layer_recompute_slices_from_islands(layer_handle *me) noexcept
{
    if (me == nullptr ||
        Orchestrator::instance().is_plugin_cancelled() ||
        reinterpret_cast<Layer *>(me)->object()->print()->canceled())
        return;
    try {
        ApiInternal::LayerAccess::recompute_slices_from_islands(*reinterpret_cast<Layer *>(me));
    } catch (...) {
        Slic3r::Steps::Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_SLICING);
    }
}

// Rebuild the layer slices and islands from the layer regions. Same failure behaviour as above: the layer
// is unchanged and the step is stopped. Like the other mutators, it returns at once when the plugin stop
// is requested or the print is cancelled.
void layer_recompute_slices_and_islands_from_layer_region(layer_handle *me) noexcept
{
    if (me == nullptr ||
        Orchestrator::instance().is_plugin_cancelled() ||
        reinterpret_cast<Layer *>(me)->object()->print()->canceled())
        return;
    try {
        ApiInternal::LayerAccess::recompute_slices_from_layer_regions(*reinterpret_cast<Layer *>(me));
    } catch (...) {
        Slic3r::Steps::Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_SLICING);
    }
}

// The four borrowers below only cast handles and look up existing objects, so they cannot throw: they are
// noexcept without a try block. They return NULL for a NULL handle. The returned handles are borrowed from
// the host, which keeps ownership; they are valid only during the current plugin call.

// Borrow the layer's slices collection (owned by the Layer).
expolygon_collection_handle *layer_borrow_mutable_slices(layer_handle *me) noexcept
{
    return me == nullptr ? nullptr :
                           reinterpret_cast<expolygon_collection_handle *>(
                               &ApiInternal::LayerAccess::slices_mutable(*reinterpret_cast<Layer *>(me)));
}

// Borrow the slices collection of a layer region (owned by the LayerRegion).
expolygon_collection_handle *layer_region_borrow_mutable_slices(const layer_region_handle *me) noexcept {
    return me == nullptr ?
        nullptr :
        reinterpret_cast<expolygon_collection_handle *>(&ApiInternal::LayerRegionAccess::slices_mutable(
            *const_cast<LayerRegion *>(reinterpret_cast<const LayerRegion *>(me))));
}

// Borrow the slice polygon of a layer island (owned by the LayerSliceIsland).
expolygon_handle *layer_island_borrow_mutable_slice(layer_island_handle *me) noexcept
{
    return me == nullptr ? nullptr :
                           reinterpret_cast<expolygon_handle *>(
                               &ApiInternal::LayerIslandAccess::slice_mutable(*reinterpret_cast<LayerSliceIsland *>(me)));
}

// Borrow layer idx of the object (owned by the PrintObject). idx must be below the object's layer count:
// it is not checked, an out-of-range index is a plugin error.
layer_handle *object_borrow_mutable_layer(const object_handle *me, uint32_t idx) noexcept {
    return me == nullptr ?
        nullptr :
        reinterpret_cast<layer_handle *>(
            &const_cast<PrintObject *>(reinterpret_cast<const PrintObject *>(me))->layer(static_cast<size_t>(idx)));
}

} // namespace

run_ctx_post_slicing make_post_slicing_run_context(Print &print, size_t object_idx)
{
    run_ctx_post_slicing context_step = {};
    context_step.print = reinterpret_cast<const print_handle *>(&print);
    context_step.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
    context_step.layer_assign_islands_by_moving_contents = layer_assign_islands_by_moving_contents;
    context_step.layer_recompute_slices_from_islands = layer_recompute_slices_from_islands;
    context_step.object_borrow_mutable_layer = object_borrow_mutable_layer;
    context_step.layer_recompute_slices_and_islands_from_layer_region = layer_recompute_slices_and_islands_from_layer_region;
    context_step.layer_borrow_mutable_slices = layer_borrow_mutable_slices;
    context_step.layer_region_borrow_mutable_slices = layer_region_borrow_mutable_slices;
    context_step.layer_island_borrow_mutable_slice = layer_island_borrow_mutable_slice;
    return context_step;
}

} // namespace Slic3r::ApiHost::Steps
