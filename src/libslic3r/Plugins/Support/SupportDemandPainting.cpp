///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SupportDemandPainting.hpp"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Plugins/Support/Common/SupportDemand.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

/*
SupportDemandPainting
=====================

This plugin applies the support-enforcer and support-blocker areas painted on
the object to the support-demand polygons. It runs at STEP_SUPPORT_DEMAND,
before support geometry is generated, and operates on the demand left by
other support-demand plugins.

The normal execution flow is:

    setup_run_impl()
    `-- reserve progress for every layer island when support painting exists

    run_impl()
    |-- project the painted support facets into per-layer polygons
    `-- for every layer island:
        |-- intersect enforcer painting with the island's overhang and union it with demand
        `-- enlarge blocker painting slightly and subtract it from demand

Enforcer painting can create demand where no demand existed, but it is clipped
to the overhang of the current island (the part that the layer below does not
cover; nothing on the first layer), and is ignored while the object prints no support (Common::object_support_enabled()).
Blocker painting changes only existing demand. Enforcers
are applied before blockers, so a blocker can suppress demand created by an
enforcer in the same layer. The small blocker offset prevents boundary-touching
polygons from surviving a boolean difference because of rounding differences.

The projected painting is computed once per object and reused for all islands
at the corresponding layer. The resulting polygon collection replaces the
support demand of that island. This stage modifies support demand
only; it does not generate or remove support extrusions directly.
*/

namespace slic3r_api { namespace Support { namespace SupportDemandPaintingPlugin {

namespace {

const char *k_support_demand_painting_id = "support.demand.painting";
const char *k_dependencies[] = { nullptr };

// Clips layer painting to the overhang of an island, the part that lower (the layer below, null on the first layer) does not cover,
// and adds it through the common demand publisher.
void add_painting_to_island(const plugin_run_context *run_ctx, const LayerIsland &island, const Layer *lower, const ClipperContext &clipper,
                            const std::vector<StoredPolygonCollection> &painting, uint32_t layer_idx);
// Removes layer blocker painting through the common enlarged subtraction.
void remove_painting_from_island(const plugin_run_context *run_ctx, const LayerIsland &island, const ClipperContext &clipper,
                                 const std::vector<StoredPolygonCollection> &painting, uint32_t layer_idx);

bool object_has_support_painting(const Object &object)
{
    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx)
        if (object.volume(volume_idx).has_painting(RAW_FACET_PAINTING_FDM_SUPPORT))
            return true;
    return false;
}

bool has_layer_painting(const std::vector<StoredPolygonCollection> &by_layer, uint32_t layer_idx)
{
    return layer_idx < by_layer.size() && !by_layer[layer_idx].empty();
}

// Intersects borrowed painting for a valid layer with its island, then publishes its union with demand through Common; missing painting leaves demand unchanged and cancellation/geometry/allocation failures propagate.
void add_painting_to_island(const plugin_run_context *run_ctx,
                            const LayerIsland &island,
                            const Layer *lower,
                            const ClipperContext &clipper,
                            const std::vector<StoredPolygonCollection> &painting,
                            uint32_t layer_idx)
{
    // The first layer stands on the bed: nothing there asks for support.
    if (lower == nullptr || !has_layer_painting(painting, layer_idx))
        return;

    // Only the overhang asks for support: a facet sloping down across several layers is projected into each of them, partly
    // over the layer below.
    ClipperOperand enforced = clipper_diff(clipper_intersection(clipper(island.slice()), clipper(painting[layer_idx])),
                                           Common::accumulate_lower_slices(clipper, clipper(), *lower));
    Common::add_to_demand(run_ctx, island, clipper, std::move(enforced));
}

// Passes a borrowed layer's blocker painting to Common's enlarged subtraction; missing painting leaves demand unchanged, context must allow demand mutation, and cancellation/geometry/allocation failures propagate.
void remove_painting_from_island(const plugin_run_context *run_ctx,
                                 const LayerIsland &island,
                                 const ClipperContext &clipper,
                                 const std::vector<StoredPolygonCollection> &painting,
                                 uint32_t layer_idx)
{
    if (!has_layer_painting(painting, layer_idx))
        return;

    Common::remove_from_demand(run_ctx, island, clipper, clipper(painting[layer_idx]));
}

} // namespace

SupportDemandPainting &
SupportDemandPainting::instance(orchestrator_handle *orch)
{
    static SupportDemandPainting s_instance(orch);
    return s_instance;
}

const char *SupportDemandPainting::id_impl() const noexcept
{
    return k_support_demand_painting_id;
}

const char *SupportDemandPainting::name_impl() const noexcept
{
    return "Support demand painting";
}

const char *SupportDemandPainting::description_impl() const noexcept
{
    return "Apply support enforcer and blocker painting to support demand polygons.";
}

slicing_step_t SupportDemandPainting::step_impl() const noexcept
{
    return STEP_SUPPORT_DEMAND;
}

const char *const *SupportDemandPainting::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t SupportDemandPainting::priority_impl() const noexcept
{
    return 5;
}

const char *SupportDemandPainting::progress_message_format_impl() const noexcept
{
    return "Support demand painting: %u / %u islands";
}

// Borrows a demand payload and reserves progress for all object islands when painting exists; missing payload does nothing and cancellation propagates.
void SupportDemandPainting::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (object_has_support_painting(object))
        progress().add_max(Common::island_work_count(object));
}

void SupportDemandPainting::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (!object_has_support_painting(object)) {
        progress().finish_run();
        return;
    }

    storage_handle *storage = run_ctx->plugin_storage;
    // With support disabled the painted enforcers ask for nothing; the painted blockers still apply.
    std::vector<StoredPolygonCollection> enforcers = Common::object_support_enabled(object) ? project_painting_to_polygons(
        storage, object, RAW_FACET_PAINTING_FDM_SUPPORT, RAW_FACET_PAINTING_ENFORCER) : std::vector<StoredPolygonCollection>();
    std::vector<StoredPolygonCollection> blockers = project_painting_to_polygons(
        storage, object, RAW_FACET_PAINTING_FDM_SUPPORT, RAW_FACET_PAINTING_BLOCKER);

    ClipperContext clipper(storage);
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);

        const Layer layer = object.layer(layer_idx);
        const std::optional<Layer> lower = layer_idx > 0 ? std::optional<Layer>(object.layer(layer_idx - 1)) : std::nullopt;
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);

            const LayerIsland island = layer.island(island_idx);
            add_painting_to_island(run_ctx, island, lower ? &*lower : nullptr, clipper, enforcers, layer_idx);
            remove_painting_from_island(run_ctx, island, clipper, blockers, layer_idx);
            progress().increment();
        }
    }

    progress().finish_run();
}

void register_support_demand_painting_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SupportDemandPainting::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Support::SupportDemandPaintingPlugin

#ifdef SUPPORT_DEMAND_PAINTING_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Support::SupportDemandPaintingPlugin::register_support_demand_painting_plugin(orch);
}
#endif // SUPPORT_DEMAND_PAINTING_PLUGIN_DLL
