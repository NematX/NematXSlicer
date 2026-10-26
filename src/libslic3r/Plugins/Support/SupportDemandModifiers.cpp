///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SupportDemandModifiers.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"

/*
SupportDemandModifiers
======================

This plugin applies support-enforcer and support-blocker volumes to the
support-demand polygons produced for an object. It works on the demand stage,
before support geometry is generated, so it changes where support may be
requested rather than deleting already generated support extrusions.

The normal execution flow is:

    setup_run_impl()
    `-- reserve progress for every object layer island when a relevant volume exists

    run_impl()
    |-- collect the object's layer Z positions and slicing parameters
    |-- slice relevant modifier volumes in object-volume order
    `-- for every layer island:
        |-- intersect enforcers with the island and union them with existing demand
        `-- subtract blockers from the resulting demand

Each modifier volume is sliced once and its polygons are reused for all layer
islands at the same Z. Enforcers can create demand where none existed, but only
inside the object island. Blockers affect only existing demand. A small offset
is applied to blocker polygons before subtraction so boundary-touching areas
are not left as support demand because of polygon-rounding differences.

The island's support demand is the source of truth: the plugin replaces it
after each boolean operation. Volume
priority is therefore determined by object-volume order: later modifiers see
the demand produced by earlier ones, and a later blocker can remove an earlier
enforcer's result.
*/

namespace slic3r_api { namespace Support { namespace SupportDemandModifiersPlugin {

namespace {

const char *k_support_demand_modifiers_id = "support.demand.modifiers";
const char *k_dependencies[] = { nullptr };

bool is_support_modifier_type(raw_volume_type type)
{
    return type == RAW_VOLUME_TYPE_SUPPORT_ENFORCER || type == RAW_VOLUME_TYPE_SUPPORT_BLOCKER;
}

bool object_has_support_modifier_volume(const Object &object)
{
    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx)
        if (is_support_modifier_type(object.volume(volume_idx).type()))
            return true;
    return false;
}

uint32_t island_work_count(const Object &object)
{
    uint32_t count = 0;
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx)
        count += object.layer(layer_idx).island_count();
    return count;
}

struct SlicedSupportModifier
{
    raw_volume_type type = RAW_VOLUME_TYPE_INVALID;
    std::vector<StoredExPolygonCollection> slices;
};

bool has_any_layer_slices(const std::vector<StoredExPolygonCollection> &by_layer)
{
    for (const StoredExPolygonCollection &layer_slices : by_layer)
        if (!layer_slices.empty())
            return true;
    return false;
}

std::vector<SlicedSupportModifier> slice_support_modifier_volumes_in_order(const Object &object,
                                                                           const std::vector<float> &slice_zs,
                                                                           const c_mesh_slicing_params &base_params,
                                                                           storage_handle *storage,
                                                                           const plugin_run_context *run_ctx)
{
    std::vector<SlicedSupportModifier> out;

    for (uint32_t volume_idx = 0; volume_idx < object.volume_count(); ++volume_idx) {
        throw_if_cancelled(run_ctx);
        const Volume volume = object.volume(volume_idx);
        const raw_volume_type type = volume.type();
        if (!is_support_modifier_type(type))
            continue;

        std::vector<StoredExPolygonCollection> slices =
            Common::slice_support_modifier_volume(volume, slice_zs, base_params, storage, run_ctx);
        if (!has_any_layer_slices(slices))
            continue;

        SlicedSupportModifier modifier;
        modifier.type = type;
        modifier.slices = std::move(slices);
        out.emplace_back(std::move(modifier));
    }

    return out;
}

bool has_layer_slices(const std::vector<StoredExPolygonCollection> &by_layer, uint32_t layer_idx)
{
    return layer_idx < by_layer.size() && !by_layer[layer_idx].empty();
}

ClipperOperand clip_layer_slices(const std::vector<StoredExPolygonCollection> &by_layer,
                                 uint32_t layer_idx,
                                 const ClipperContext &clipper)
{
    assert(has_layer_slices(by_layer, layer_idx));
    return clipper(by_layer[layer_idx]);
}

void set_demand_from_operand(const LayerIsland &island,
                             ClipperOperand &&operand,
                             const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    StoredExPolygonCollection polygons = operand.to_expolygon_collection();
    throw_if_cancelled(run_ctx);
    polygons.ensure_valid();
    throw_if_cancelled(run_ctx);
    expolygons_move(support_demand_get_island_mutable(run_ctx, island.handle()), polygons.mutable_handle());
    throw_if_cancelled(run_ctx);
}

void add_enforcers_to_island(const LayerIsland &island,
                             const ClipperContext &clipper,
                             const std::vector<StoredExPolygonCollection> &enforcers,
                             uint32_t layer_idx,
                             const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!has_layer_slices(enforcers, layer_idx))
        return;

    ClipperOperand enforced =
        clipper_intersection(clipper(island.slice()), clip_layer_slices(enforcers, layer_idx, clipper));
    throw_if_cancelled(run_ctx);
    if (enforced.empty())
        return;

    const ExPolygonCollection existing = island.support_demand();
    if (!existing.empty()) {
        enforced = clipper_union2(clipper(existing), enforced);
        throw_if_cancelled(run_ctx);
    }
    set_demand_from_operand(island, std::move(enforced), run_ctx);
}

void remove_blockers_from_island(const LayerIsland &island,
                                 const ClipperContext &clipper,
                                 const std::vector<StoredExPolygonCollection> &blockers,
                                 uint32_t layer_idx,
                                 const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!has_layer_slices(blockers, layer_idx))
        return;

    const ExPolygonCollection existing = island.support_demand();
    if (existing.empty())
        return;

    ClipperOperand blocked =
        clipper_offset(clip_layer_slices(blockers, layer_idx, clipper), 1000. * double(SCALED_EPSILON));
    throw_if_cancelled(run_ctx);
    ClipperOperand remaining = clipper_diff(clipper(existing), blocked);
    throw_if_cancelled(run_ctx);
    set_demand_from_operand(island, std::move(remaining), run_ctx);
}

} // namespace

SupportDemandModifiers &
SupportDemandModifiers::instance(orchestrator_handle *orch)
{
    static SupportDemandModifiers s_instance(orch);
    return s_instance;
}

const char *SupportDemandModifiers::id_impl() const noexcept
{
    return k_support_demand_modifiers_id;
}

const char *SupportDemandModifiers::name_impl() const noexcept
{
    return "Support demand modifiers";
}

const char *SupportDemandModifiers::description_impl() const noexcept
{
    return "Apply support enforcer and blocker modifier volumes to support demand polygons.";
}

slicing_step_t SupportDemandModifiers::step_impl() const noexcept
{
    return STEP_SUPPORT_DEMAND;
}

const char *const *SupportDemandModifiers::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t SupportDemandModifiers::priority_impl() const noexcept
{
    return 10;
}

const char *SupportDemandModifiers::progress_message_format_impl() const noexcept
{
    return "Support demand modifiers: %u / %u islands";
}

void SupportDemandModifiers::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (object_has_support_modifier_volume(object))
        progress().add_max(island_work_count(object));
}

void SupportDemandModifiers::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_support_demand *ctx = plugin_ctx_as_support_demand(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (!object_has_support_modifier_volume(object)) {
        progress().finish_run();
        return;
    }

    storage_handle *storage = run_ctx->plugin_storage;
    const Print print(ctx->print);
    const std::vector<float> slice_zs = Common::layer_slice_zs(object);
    const c_mesh_slicing_params params = Common::support_modifier_slicing_params(print, object);
    throw_if_cancelled(run_ctx);
    const std::vector<SlicedSupportModifier> modifiers =
        slice_support_modifier_volumes_in_order(object, slice_zs, params, storage, run_ctx);
    throw_if_cancelled(run_ctx);

    if (modifiers.empty()) {
        progress().finish_run();
        return;
    }

    ClipperContext clipper(storage);
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);

        const Layer layer = object.layer(layer_idx);
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);

            const LayerIsland island = layer.island(island_idx);
            for (const SlicedSupportModifier &modifier : modifiers) {
                throw_if_cancelled(run_ctx);
                if (modifier.type == RAW_VOLUME_TYPE_SUPPORT_ENFORCER)
                    add_enforcers_to_island(island, clipper, modifier.slices, layer_idx, run_ctx);
                else if (modifier.type == RAW_VOLUME_TYPE_SUPPORT_BLOCKER)
                    remove_blockers_from_island(island, clipper, modifier.slices, layer_idx, run_ctx);
            }
            progress().increment();
        }
    }

    progress().finish_run();
}

void register_support_demand_modifiers_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SupportDemandModifiers::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Support::SupportDemandModifiersPlugin

#ifdef SUPPORT_DEMAND_MODIFIERS_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Support::SupportDemandModifiersPlugin::register_support_demand_modifiers_plugin(orch);
}
#endif // SUPPORT_DEMAND_MODIFIERS_PLUGIN_DLL
