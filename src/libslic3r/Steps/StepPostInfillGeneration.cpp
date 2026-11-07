
// Post-infill step: host callback given to post-infill plugins through run_ctx_post_infill_generation.
// A plugin borrows the mutable extrusion root of an infill-owned bucket of a region island and appends its
// generated subtrees to it. The callback may create the root, so it can throw (bad_alloc): it is noexcept,
// hands a failure to Detail::contain_callback_failure() (which stops the step) and returns NULL. Once a
// stop is requested it returns NULL at once. After the plugins finish, empty roots are removed.

/* StepSelectorTraits.hpp defines the selector metadata this module registers with Orchestrator, the plugin manager.
   Call graph:
     register_selector_traits
       Orchestrator::register_builtin_step_selector
     run_step
       Detail::run_object_step_plugin
       remove_empty_region_islands
   Plugins edit infill extrusion roots through the callback described above; empty results are removed afterward. */
#include "StepPostInfillGeneration.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintRegion.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepPostInfillGeneration {

// Install the post-infill selector declaration on the supplied host.
void register_selector_traits(Orchestrator &orchestrator);

void register_selector_traits(Orchestrator &orchestrator)
{
    StepSelectorTraits traits;
    traits.category = RAW_OPTION_CATEGORY_INFILL;
    traits.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    const int32_t registered = orchestrator.register_builtin_step_selector(STEP_POST_INFILL, std::move(traits));
    assert(registered == 1);
    (void)registered;
}
namespace {

LayerRegionIsland *to_layer_region_island(const layer_region_island_handle *handle)
{
    return const_cast<LayerRegionIsland *>(reinterpret_cast<const LayerRegionIsland *>(handle));
}

bool role_is_post_infill_owned(raw_extrusion_role role)
{
    // This step runs after infill generation. A plugin here may only edit
    // extrusion trees that were produced for infill-related work. Other
    // extrusion classes are handled earlier or later in the pipeline, so we
    // reject them here instead of giving a plugin a pointer to the wrong bucket.
    return (role & RAW_EXTRUSION_ROLE_INFILL) != 0 ||
           (role & RAW_EXTRUSION_ROLE_IRONING) != 0 ||
           role == RAW_EXTRUSION_ROLE_GAP_FILL;
}

ExtrusionRole bucket_role_from_raw(raw_extrusion_role role)
{
    // A LayerRegionIsland does not store one root per exact extrusion role.
    // Instead it stores a few large roots: all sparse/solid/bridge infill
    // paths share INFILLS, ironing paths share IRONINGS, and narrow residual
    // paths share GAP_FILLS. The exact role is still stored on each child path.
    if ((role & RAW_EXTRUSION_ROLE_IRONING) != 0)
        return LayerRegionIsland::IRONINGS;
    if (role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return LayerRegionIsland::GAP_FILLS;
    return LayerRegionIsland::INFILLS;
}

// Borrow the mutable extrusion root of the bucket matching role, creating the empty root if needed.
// Returns NULL for a NULL island, a role not owned by this step, or a contained failure: creating the
// root can throw (bad_alloc), which is handed to contain_callback_failure() and stops the step. Once the
// plugin stop is already requested it also returns NULL, without creating anything or adding a message.
// A print cancellation is not checked here: the runner rethrows the cancellation at the phase boundary.
extrusion_entity_handle *get_region_island_mutable_extrusion_callback(
    const layer_region_island_handle *region_island_handle,
    raw_extrusion_role role) noexcept
{
    LayerRegionIsland *region_island = to_layer_region_island(region_island_handle);
    if (region_island == nullptr || !role_is_post_infill_owned(role))
        return nullptr;

    // The returned object is the host-owned root collection for this bucket.
    // It may be empty when the plugin is about to append the first generated
    // child subtree. Empty roots are removed by the step cleanup after the
    // plugin returns, so a harmless probe does not permanently dirty the tree.
    if (Orchestrator::instance().is_plugin_cancelled())
        return nullptr;

    try {
        const ExtrusionRole bucket_role = bucket_role_from_raw(role);
        ExtrusionEntity &collection = region_island->mutable_extrusion(bucket_role);
        return reinterpret_cast<extrusion_entity_handle *>(&collection);
    } catch (...) {
        Detail::contain_callback_failure(Orchestrator::instance(), STEP_POST_INFILL);
        return nullptr;
    }
}

void remove_empty_region_islands(Print &print)
{
    // Plugins may ask for a destination LayerRegionIsland, then decide that no
    // output has to be published there. The host removes groups that have no
    // fill surfaces and no real extrusion after each plugin finishes, when no
    // plugin still holds borrowed handles from the current callback.
    for (PrintObject &object : print.objects())
        for (Layer &layer : object.layers())
            for (LayerSliceIsland &island : layer.islands()) {
                LayerRegionIslandUPtrs &region_islands = island.mutable_regions_islands();
                for (LayerRegionIslandUPtr &region_island : region_islands)
                    if (region_island != nullptr)
                        region_island->remove_empty_extrusions();
                region_islands.erase(
                    std::remove_if(region_islands.begin(),
                                   region_islands.end(),
                                   [](const LayerRegionIslandUPtr &region_island) {
                                       return region_island != nullptr &&
                                              region_island->fill_surfaces().empty() &&
                                              !region_island->has_extrusions();
                                   }),
                    region_islands.end());
            }
}

} // namespace

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    // Several post-infill plugins can run one after another. This is useful for
    // small finishing passes: one plugin may add residual gap fill, another may
    // prepare ironing, and another may simplify or tag the generated infill.
    // They all receive the same object-level payload and run in priority order.
    std::vector<Plugin *> plugins =
        selected_or_active_plugins_for_step(orchestrator, STEP_POST_INFILL, &print.full_print_config());
    for (Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;

        Detail::run_object_step_plugin(
            orchestrator,
            print,
            STEP_POST_INFILL,
            *plugin,
            print.objects().size(),
            [&print](const size_t object_idx) {
            run_ctx_post_infill_generation context = {};
            context.print = reinterpret_cast<const print_handle *>(&print);
            context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
            context.get_region_island_mutable_extrusion = &get_region_island_mutable_extrusion_callback;
            return context;
        });
        remove_empty_region_islands(print);
        print.pipeline_progress().complete_plugin(STEP_POST_INFILL, plugin->get_id());
    }
}

} // namespace Slic3r::Steps::StepPostInfillGeneration
