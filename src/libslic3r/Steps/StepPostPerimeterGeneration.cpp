
// Post-perimeter step: host callbacks given to post-perimeter plugins through run_ctx_post_perimeter_generation.
// A plugin borrows the mutable extrusion roots of a region island, or replaces the fill / free-fill areas
// of an island. Callbacks cross the C ABI, so none may let an exception out: the pure borrower is noexcept
// because it cannot throw, the two writers run their throwing work in a try block and hand a failure to
// Detail::contain_callback_failure(), which records it and stops the step. Once a stop is requested, the
// writers return 0 at once without touching the island and without a new message.

/* StepSelectorTraits.hpp defines the selector metadata this module registers with Orchestrator, the plugin manager.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step
       Detail::run_object_step_plugins
   Plugins read print settings and edit island extrusion roots and infill areas through the callbacks described above. */
#include "StepPostPerimeterGeneration.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include <utility>

#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_perimeter.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

#include "StepRunner.hpp"

namespace Slic3r::Steps::StepPostPerimeterGeneration {

// Install the post-perimeter selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_PERIMETER;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_POST_PERIMETER, std::move(traits));
    assert(registered == 1);
    (void)registered;
}
namespace {

LayerSliceIsland *to_layer_island(const layer_island_handle *handle)
{
    return const_cast<LayerSliceIsland *>(reinterpret_cast<const LayerSliceIsland *>(handle));
}

LayerRegionIsland *to_layer_region_island(const layer_region_island_handle *handle)
{
    return const_cast<LayerRegionIsland *>(reinterpret_cast<const LayerRegionIsland *>(handle));
}

const ExPolygons *to_expolygons(const expolygon_collection_handle *handle)
{
    return reinterpret_cast<const ExPolygons *>(handle);
}

ExtrusionRole bucket_role_from_raw(raw_extrusion_role role)
{
    if ((role & RAW_EXTRUSION_ROLE_THIN) != 0 || role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return LayerRegionIsland::GAP_FILLS;
    if ((role & RAW_EXTRUSION_ROLE_INFILL) != 0)
        return LayerRegionIsland::INFILLS;
    if ((role & RAW_EXTRUSION_ROLE_IRONING) != 0)
        return LayerRegionIsland::IRONINGS;
    if ((role & RAW_EXTRUSION_ROLE_MILL) != 0)
        return LayerRegionIsland::MILLS;
    if ((role & RAW_EXTRUSION_ROLE_SUPPORT) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ? LayerRegionIsland::SUPPORT_INTERFACE :
                                                           LayerRegionIsland::SUPPORT;
    return LayerRegionIsland::PERIMETERS;
}

// Borrow the mutable extrusion root of a bucket of a region island, or NULL when the island is NULL or the
// bucket is absent or empty (has_extrusion() is false for an empty bucket). Cannot throw: has_extrusion()
// is a map lookup and mutable_extrusion() only runs on a key that exists, so nothing is allocated and no
// try block is needed.
extrusion_entity_handle *get_region_island_mutable_extrusion_callback(
    const layer_region_island_handle *region_island_handle,
    raw_extrusion_role role) noexcept
{
    LayerRegionIsland *region_island = to_layer_region_island(region_island_handle);
    if (region_island == nullptr)
        return nullptr;

    // Post-perimeter plugins edit buckets produced by earlier steps. Missing
    // buckets stay absent, so a plugin can safely probe optional extrusion
    // classes without creating empty LayerRegionIsland entries.
    const ExtrusionRole bucket_role = bucket_role_from_raw(role);
    if (!region_island->has_extrusion(bucket_role))
        return nullptr;

    ExtrusionEntity &collection = region_island->mutable_extrusion(bucket_role);
    return reinterpret_cast<extrusion_entity_handle *>(&collection);
}

// Replace the infill areas of an island with a validated copy of areas_handle (NULL clears them).
// Returns 1 on success. Returns 0 without touching anything, and without stopping the step, for a NULL
// island; returns 0 at once (no change, no new message) when the plugin stop is already requested.
// Otherwise the copy, ensure_valid() and the bounding-box computation inside set_infill_areas() may throw
// (bad_alloc); they all run before the island is written, so on a contained failure the island is
// unchanged and contain_callback_failure() stops the step. A print cancellation is not checked here: the
// runner rethrows the cancellation at the phase boundary.
int32_t set_island_fill_areas_callback(const layer_island_handle *island_handle,
                                       const expolygon_collection_handle *areas_handle) noexcept
{
    LayerSliceIsland *island = to_layer_island(island_handle);
    if (island == nullptr || Orchestrator::instance().is_plugin_cancelled())
        return 0;

    try {
        ExPolygons areas = areas_handle == nullptr ? ExPolygons{} : *to_expolygons(areas_handle);
        ensure_valid(areas);
        ApiInternal::LayerIslandAccess::set_infill_areas(*island, std::move(areas));
        return 1;
    } catch (...) {
        Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_PERIMETER);
        return 0;
    }
}

// Same contract as set_island_fill_areas_callback() for the free (no-encroachment) areas: the copy and
// ensure_valid() may throw and run first; the final write is a non-throwing move, so a failure leaves the
// island unchanged, returns 0 and stops the step. A NULL island returns 0 without stopping; an already
// requested plugin stop returns 0 at once with no change and no message.
int32_t set_island_fill_free_areas_callback(const layer_island_handle *island_handle,
                                            const expolygon_collection_handle *areas_handle) noexcept
{
    LayerSliceIsland *island = to_layer_island(island_handle);
    if (island == nullptr || Orchestrator::instance().is_plugin_cancelled())
        return 0;

    try {
        ExPolygons areas = areas_handle == nullptr ? ExPolygons{} : *to_expolygons(areas_handle);
        ensure_valid(areas);
        ApiInternal::LayerIslandAccess::infill_free_areas_mutable(*island) = std::move(areas);
        return 1;
    } catch (...) {
        Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_PERIMETER);
        return 0;
    }
}

} // namespace

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string &)
{
    return true;
}

bool validate_post(const Print &, std::string &)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_POST_PERIMETER,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_post_perimeter_generation context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        context.get_region_island_mutable_extrusion = &get_region_island_mutable_extrusion_callback;
        context.set_island_fill_areas = &set_island_fill_areas_callback;
        context.set_island_fill_free_areas = &set_island_fill_free_areas_callback;
        return context;
    });
}

} // namespace Slic3r::Steps::StepPostPerimeterGeneration
