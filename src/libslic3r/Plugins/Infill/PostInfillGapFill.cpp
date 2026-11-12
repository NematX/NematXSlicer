///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PostInfillGapFill.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Common/ResidualGapFill.hpp"

/* Complete solid infill without regenerating its pattern. A source surface and
   the island's free infill area bound every addition. Inherited path attributes
   describe occupied spacing; subtracting it leaves the narrow residuals consumed
   by the SDK medial-axis factory. Regional clips apply activation and thresholds
   even when several modifier regions share one surface subtree.

       run_impl -> fill_layer: independent layer trees and scratch storage
           fill_region -> fill_surface: recover the stamped source and recipe
               GapFill::collect_paths / coverage: measure paths before insertion
               GapFill::generate: open residuals and build variable-width paths
               publish: order additions after old content, retain the root stamp

   Old paths and their private properties remain unchanged. New paths inherit
   only the surface recipe, allowing the later volume corrector to measure both.
   Successful empty pattern results are eligible too. Exceptions and cancellation
   propagate through PluginBase's C ABI boundary; no borrowed view is retained
   between plugin calls. Each parallel worker owns its temporary geometry. */

namespace slic3r_api { namespace Infill { namespace PostInfillGapFillPlugin {
namespace {

using GapFill::k_rounded_section;
const char *k_no_dependencies[] = { nullptr };
const raw_used_config_key k_used_keys[] = {
    {"infill_filled_top", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_filled_solid", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_filled_bottom", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"gap_fill_enabled", RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"gap_fill_min_area", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"gap_fill_min_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"gap_fill_max_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"gap_fill_min_length", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"gap_fill_extension", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT},
    {"resolution", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

// Select the existing activation option; bridges and sparse surfaces have no pass.
const char *activation_key(raw_surface_type type);
// Rebuild the nominal non-bridge flow from the final recipe, retaining its nozzle.
c_flow recipe_flow(const EPropertyInfill &recipe, const LayerRegion &region);
// Complete one stamped subtree, keeping its recipe on the direct bucket child.
void fill_surface(storage_handle *, MutableExtrusionEntity, const Surface &, const LayerIsland &,
    const RegionSettings &, const LayerRegion &, coord_t, const plugin_run_context *);
// Resolve regional clips once, then borrow the bucket even when it has no geometry.
void fill_region(storage_handle *, const run_ctx_post_infill_generation &, const LayerIsland &,
    const LayerRegionIsland &, coord_t, const plugin_run_context *);
// Parallel worker: layers own disjoint extrusion trees and temporary storage.
void fill_layer(uint32_t, storage_handle *, const run_ctx_post_infill_generation *, const plugin_run_context *);

// Apply the recipe multiplier exactly once, only to newly generated attributes.
class MultiplyFlowVisitor : public ExtrusionTreeVisitor<>
{
public:
    explicit MultiplyFlowVisitor(float mult) : m_mult(mult) {}
    // Scale stored attributes rather than materializing inherited properties.
    void enter_node(MutableExtrusionEntity entity) override
    {
        if (EPropertyAttributes *a = entity.get_mutable(EPropertyAttributes::key)) {
            c_extrusion_property_attributes &raw = *a;
            raw.mm3_per_mm *= m_mult;
            raw.width *= m_mult;
        }
    }
private:
    float m_mult;
};

const char *activation_key(raw_surface_type type)
{
    if (!(type & RAW_SURFACE_TYPE_DENS_SOLID) || (type & RAW_SURFACE_TYPE_MOD_BRIDGE))
        return nullptr;
    if (type & RAW_SURFACE_TYPE_POS_TOP)
        return "infill_filled_top";
    if (type & RAW_SURFACE_TYPE_POS_BOTTOM)
        return "infill_filled_bottom";
    return "infill_filled_solid";
}

c_flow recipe_flow(const EPropertyInfill &recipe, const LayerRegion &region)
{
    const raw_extrusion_role role = recipe.surface_type() & RAW_SURFACE_TYPE_POS_TOP ?
        RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL : RAW_EXTRUSION_ROLE_SOLID_INFILL;
    c_flow flow = region.flow(role);
    // The recipe stores the spacing between paths; the width adds back the
    // overlap of their rounded ends, as in GapFill::coverage.
    const double h = unscaled(recipe.height());
    const double w = unscaled(recipe.spacing()) + h * k_rounded_section * recipe.spacing_ratio();
    flow.height = recipe.height();
    flow.spacing = recipe.spacing();
    flow.spacing_ratio = recipe.spacing_ratio();
    flow.width = scale_i(w);
    flow.mm3_per_mm = h * (w - h * k_rounded_section);
    flow.is_bridge = 0;
    if (!(h > 0. && w > 0. && flow.mm3_per_mm > 0. && flow.nozzle_diameter > 0))
        throw std::runtime_error("Invalid infill recipe flow for gap fill");
    return flow;
}

void fill_surface(storage_handle *storage, MutableExtrusionEntity tree, const Surface &surface,
    const LayerIsland &island, const RegionSettings &settings, const LayerRegion &region,
    coord_t resolution, const plugin_run_context *run_ctx)
{
    const EPropertyInfill recipe = *tree.get(EPropertyInfill::key);
    const char *key = activation_key(recipe.surface_type());
    if (!key || !(recipe.density() > 0.) || !(recipe.flow_mult() > 0.))
        return;
    const auto &areas = settings.get_areas("gap_fill_enabled");
    if (std::none_of(areas.begin(), areas.end(), [key](const auto &entry) {
            return entry.first.get_bool("gap_fill_enabled") && entry.first.get_bool(key);
        }))
        return;
    const ClipperContext clip(storage);
    // An absent no-overlap mask means no clipping, as in the data-tree contract.
    const ExPolygonCollection free = island.infill_no_overlap_areas();
    StoredExPolygonCollection domain = free.empty() ? StoredExPolygonCollection(storage, surface.expolygon()) :
        clipper_intersection(clip(surface.expolygon()), clip(free)).to_expolygon_collection();
    if (domain.empty())
        return;
    std::vector<GapFill::Path> paths;
    GapFill::collect_paths(tree.readonly(), paths);
    StoredPolygonCollection occupied = GapFill::coverage(storage, paths, recipe.spacing_ratio(), GapFill::Footprint::Spacing);
    StoredExPolygonCollection residual = clipper_diff(clip(domain), clip(occupied)).to_expolygon_collection();
    if (residual.empty())
        return;
    StoredExtrusionEntity additions(storage);
    for (const auto &entry : areas) {
        if (!entry.first.get_bool("gap_fill_enabled") || !entry.first.get_bool(key))
            continue;
        StoredExPolygonCollection allowed = entry.second.intersections(domain.readonly());
        StoredExPolygonCollection remaining = entry.second.intersections(residual.readonly());
        const c_flow flow = recipe_flow(recipe, region);
        const GapFill::Limits limits = GapFill::resolve_limits([&entry](const char *name, double base) {
            return entry.first.get_effective_value(base, name);
        }, unscaled(flow.width), resolution);
        StoredExtrusionEntity gaps = GapFill::generate(storage, remaining, allowed, flow,
            limits, RAW_EXTRUSION_ROLE_GAP_FILL, run_ctx);
        if (!gaps.readonly().empty() && additions.append_child_move(gaps.mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Cannot stage infill gap fill");
    }
    if (additions.readonly().empty())
        return;
    MultiplyFlowVisitor multiply(recipe.flow_mult());
    multiply.traverse(additions.mutable_view());

    // The old content keeps all its private properties. Its surface recipe moves
    // up to the tree root, the bucket child read by the full-fill volume corrector,
    // so that pass measures the old content and the new gap fill together.
    MutableExtrusionEntity queue = GapFill::ordered_boundary(tree);
    tree.get_or_add(EPropertyInfill::key) = recipe;
    tree.child_mutable(0).remove(EPropertyInfill::key);
    GapFill::append_additions(queue, additions);
}

void fill_region(storage_handle *storage, const run_ctx_post_infill_generation &ctx,
    const LayerIsland &island, const LayerRegionIsland &region, coord_t resolution,
    const plugin_run_context *run_ctx)
{
    if (region.region_count() == 0 || region.fill_surface_count() == 0)
        return;
    RegionSettings settings(storage, region.region(0).print_region().config(), {{
        "gap_fill_enabled", "infill_filled_top", "infill_filled_solid", "infill_filled_bottom",
        "gap_fill_min_area", "gap_fill_min_width", "gap_fill_max_width", "gap_fill_min_length", "gap_fill_extension"}});
    for (uint32_t r = 0; r < region.region_count(); ++r)
        settings.add_region(region.region(r));
    settings.segregate(island.slice());
    bool enabled = false;
    for (const auto &entry : settings.get_areas("gap_fill_enabled"))
        enabled |= entry.first.get_bool("gap_fill_enabled") && (entry.first.get_bool("infill_filled_top") ||
            entry.first.get_bool("infill_filled_solid") || entry.first.get_bool("infill_filled_bottom"));
    if (!enabled)
        return;
    std::unordered_map<uint64_t, Surface> sources;
    for (const Surface surface : region.fill_surfaces())
        sources.emplace(surface.id(), surface);
    MutableExtrusionEntity root(ctx.get_region_island_mutable_extrusion(region.handle(), RAW_EXTRUSION_ROLE_INTERNAL_INFILL));
    if (!root.valid())
        return;
    for (uint32_t c = 0; c < root.child_count(); ++c) {
        throw_if_cancelled(run_ctx);
        MutableExtrusionEntity tree = root.child_mutable(c);
        const EPropertyInfill *recipe = tree.get(EPropertyInfill::key);
        if (!recipe || !activation_key(recipe->surface_type()))
            continue;
        const auto source = sources.find(recipe->surface_id());
        if (source != sources.end())
            fill_surface(storage, tree, source->second, island, settings, region.region(0), resolution, run_ctx);
    }
}

void fill_layer(uint32_t idx, storage_handle *storage, const run_ctx_post_infill_generation *ctx,
    const plugin_run_context *run_ctx)
{
    const Layer layer = Object(ctx->object).layer(idx);
    const coord_t resolution = std::max<coord_t>(SCALED_EPSILON,
        scale_i(Print(ctx->print).config().get("resolution").get_float()));
    for (uint32_t i = 0; i < layer.island_count(); ++i) {
        throw_if_cancelled(run_ctx);
        const LayerIsland island = layer.island(i);
        for (uint32_t r = 0; r < island.region_island_count(); ++r)
            fill_region(storage, *ctx, island, island.region_island(r), resolution, run_ctx);
    }
}

} // namespace

PostInfillGapFill &PostInfillGapFill::instance(orchestrator_handle *orch)
{
    static PostInfillGapFill instance(orch);
    return instance;
}

const char *PostInfillGapFill::id_impl() const noexcept { return "infill.post_process.gap_fill"; }
const char *PostInfillGapFill::name_impl() const noexcept { return "Post-infill gap fill"; }
const char *PostInfillGapFill::description_impl() const noexcept
{ return "Complete narrow uncovered areas of solid infill after pattern generation."; }
slicing_step_t PostInfillGapFill::step_impl() const noexcept { return STEP_POST_INFILL; }
const char *const *PostInfillGapFill::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t PostInfillGapFill::priority_impl() const noexcept { return 0; }

int32_t PostInfillGapFill::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys)
        std::copy(std::begin(k_used_keys), std::end(k_used_keys), keys);
    return int32_t(std::size(k_used_keys));
}

void PostInfillGapFill::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    if (!ctx || !ctx->object || !ctx->print || !ctx->get_region_island_mutable_extrusion)
        return;
    parallel_for_storage_with_progress(0, Object(ctx->object).layer_count(), run_ctx, nullptr, fill_layer, ctx, run_ctx);
}

void register_post_infill_gap_fill_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, PostInfillGapFill::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Infill::PostInfillGapFillPlugin
