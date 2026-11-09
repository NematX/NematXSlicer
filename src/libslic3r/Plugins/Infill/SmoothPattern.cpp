///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

// Keep the passes synchronized with FillSmooth.cpp.
#include "SmoothPattern.hpp"

#include <algorithm>
#include <iterator>

#include "libslic3r/Api/plugin/c/slic3r_config_types.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"

/*
Smooth (Ironing) pattern plugin
===============================

`smooth` fills a top, bottom or solid surface in two passes with the ported
rectilinear fillers:

- the deposit pass lays a monotonic fill at the recipe density with
  (1 - fill_smooth_distribution) of the flow. When the recipe has no-overlap
  areas and is not a bridge, it only fills them, grown by half the overlap;
- the ironing pass covers the whole surface a quarter turn later (when the fill
  angle may cross), with lines fill_smooth_width times closer, monotonic, with
  the ironing role and fill_smooth_distribution of the flow. Each line carries
  less flow as the lines get closer, so the pass keeps its share of the volume.

Both settings are read from the region of the surface. The fillers follow the
recipe C_INFILL_FLAG_CAN_ANGLE_CROSS for their layer rotation. Gap fill and the
full-fill volume correction are left to the post-infill plugins, which correct
both passes together.

Published tree, with the native flags. Every path is monotonic, so none is
reversible:

    root (fixed order: deposit before ironing)
    |-- deposit pass (sortable unless the recipe is MONOTONIC)
    |   `-- one group per area (fixed order) -- paths
    `-- ironing pass (fixed order)
        `-- one group (sortable) -- paths

The normal call flow is:

    SmoothPattern::run_impl()
    |-- read_settings()
    `-- fill_smooth_surface()
        |-- pass_recipe() for each pass
        |-- deposit_areas()
        `-- fill_pass() for each pass
            `-- set_order()
*/

namespace slic3r_api { namespace Infill {
namespace {

namespace rect_work = PluginCommon::polygon_work;
namespace rect_fills = RectilinearPatternsPlugin::fills;

const char *const k_fill_smooth_width_key = "fill_smooth_width";
const char *const k_fill_smooth_distribution_key = "fill_smooth_distribution";

// Built-in options carry no container or preset type to check.
const raw_used_config_key k_used_config_keys[] = {
    { k_fill_smooth_width_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_fill_smooth_distribution_key, RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

// Ironing settings of the region that owns the surface.
SmoothSettings read_settings(const layer_region_handle *region);
// Recipe of one pass: its density, and the flow of each line scaled so the pass keeps its share of the volume.
raw_infill_pattern_params pass_recipe(const raw_infill_pattern_params &recipe, float density, float share);
// The no-overlap areas grown by half the overlap, or the whole surface for bridges and recipes without them.
StoredExPolygonCollection deposit_areas(storage_handle *storage, const run_ctx_infill_pattern &ctx,
                                        const ExPolygon &surface);
// Fill each area and append the pass collection, one group per filled area, to the root.
void fill_pass(storage_handle *storage, const rect_fills::FillRectilinear &filler, const ExPolygonCollection &areas,
               const raw_infill_pattern_params &params, bool keep_pass_order, StoredExtrusionEntity &root,
               const plugin_run_context *run_ctx);
// Give a filled collection the native sortable and reversible state.
void set_order(StoredExtrusionEntity &collection, bool keep_order);
class SmoothPattern;
// One stateless provider serves independent surface calls.
SmoothPattern &instance(orchestrator_handle *orchestrator);

SmoothSettings read_settings(const layer_region_handle *region)
{
    const Config config = LayerRegion(region).print_region().config();
    SmoothSettings settings;
    settings.width = config.effective_float_or_percent_or_default(k_fill_smooth_width_key, 1., settings.width);
    settings.distribution = config.percent_or_default(k_fill_smooth_distribution_key, settings.distribution);
    return settings;
}

raw_infill_pattern_params pass_recipe(const raw_infill_pattern_params &recipe, float density, float share)
{
    // Same float operations as FillSmooth::perform_single_fill().
    raw_infill_pattern_params params = recipe;
    params.density = density;
    params.flow_mult *= recipe.density;
    params.flow_mult /= params.density;
    params.flow_mult *= share;
    return params;
}

StoredExPolygonCollection deposit_areas(storage_handle *storage, const run_ctx_infill_pattern &ctx,
                                        const ExPolygon &surface)
{
    if (ctx.params->flow.is_bridge != 0 || ctx.no_overlap_areas == nullptr ||
        ExPolygonCollection(ctx.no_overlap_areas).empty())
        return StoredExPolygonCollection(storage, surface);
    const ClipperOperand half_overlap = clipper_offset(
        ClipperOperand(storage, ExPolygonCollection(ctx.no_overlap_areas)), scale_d(ctx.params->overlap / 2));
    return clipper_intersection(ClipperOperand(storage, surface), half_overlap).to_expolygon_collection();
}

void fill_pass(storage_handle *storage, const rect_fills::FillRectilinear &filler, const ExPolygonCollection &areas,
               const raw_infill_pattern_params &params, bool keep_pass_order, StoredExtrusionEntity &root,
               const plugin_run_context *run_ctx)
{
    StoredExtrusionEntity pass(storage);
    for (const ExPolygon area : areas) {
        throw_if_cancelled(run_ctx);
        const rect_work::Polylines lines = filler.fill_surface(area, params);
        StoredExtrusionEntity group(storage);
        for (const rect_work::Polyline &line : lines) {
            if (line.points.size() < 2)
                continue;
            StoredExtrusionEntity path = PatternHelpers::make_path(storage, line.points, params);
            group.append_child_move(path.mutable_view());
        }
        if (group.empty())
            continue;
        set_order(group, filler.no_sort());
        pass.append_child_move(group.mutable_view());
    }
    if (pass.empty())
        return;
    set_order(pass, keep_pass_order);
    root.append_child_move(pass.mutable_view());
}

void set_order(StoredExtrusionEntity &collection, bool keep_order)
{
    // Adding the first child makes an API collection sortable by default.
    if (keep_order) {
        collection.disable_sort();
        collection.disable_reverse();
    } else {
        collection.set_flags(collection.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    }
}

class SmoothPattern : public PluginBase
{
public:
    explicit SmoothPattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }

private:
    const char *id_impl() const noexcept override { return "smooth"; }
    const char *name_impl() const noexcept override { return "Ironing"; }
    const char *description_impl() const noexcept override { return "Ironing-style smooth infill pattern."; }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override {
        static const char *none[] = {nullptr};
        return none;
    }
    int32_t priority_impl() const noexcept override { return 17; }
    const char *progress_message_format_impl() const noexcept override { return "Generating ironing infill: %u / %u surfaces"; }
    void inilialize_impl(storage_handle *) const override {
        PatternHelpers::add_self_to_pattern_list(m_orchestrator, "smooth", "Ironing",
            {"top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern"});
    }
    void run_impl(const plugin_run_context *run_ctx) const override {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            ctx->primary_region == nullptr || run_ctx->plugin_storage == nullptr)
            return;
        fill_smooth_surface(*ctx, run_ctx->plugin_storage, run_ctx, read_settings(ctx->primary_region));
        progress().increment();
    }
};

SmoothPattern &instance(orchestrator_handle *orchestrator)
{
    static SmoothPattern pattern(orchestrator);
    return pattern;
}
} // namespace

void fill_smooth_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                         const SmoothSettings &settings)
{
    throw_if_cancelled(run_ctx);
    const raw_infill_pattern_params &recipe = *ctx.params;
    const ExPolygon surface = Surface(ctx.surface).expolygon();

    // Both passes are monotonic, which fixes the path directions; only the deposit pass
    // keeps the recipe ordering of its areas.
    raw_infill_pattern_params deposit = pass_recipe(recipe, recipe.density, 1.f - float(settings.distribution));
    deposit.infill_flags |= C_INFILL_FLAG_MONOTONIC;
    raw_infill_pattern_params ironing =
        pass_recipe(recipe, recipe.density / float(settings.width), float(settings.distribution));
    ironing.infill_flags |= C_INFILL_FLAG_MONOTONIC;
    ironing.extrusion_role = RAW_EXTRUSION_ROLE_IRONING_INFILL;

    StoredExtrusionEntity root(storage);
    rect_fills::FillMonotonic deposit_filler;
    rect_fills::configure_filler(deposit_filler, deposit, storage);
    const StoredExPolygonCollection deposit_area = deposit_areas(storage, ctx, surface);
    fill_pass(storage, deposit_filler, deposit_area.readonly(), deposit,
              (recipe.infill_flags & C_INFILL_FLAG_MONOTONIC) != 0, root, run_ctx);

    rect_fills::FillRectilinear ironing_filler;
    rect_fills::configure_filler(ironing_filler, ironing, storage);
    if (ironing_filler.can_angle_cross)
        ironing_filler.angle += float(PatternHelpers::k_half_pi);
    const StoredExPolygonCollection ironing_area(storage, surface);
    fill_pass(storage, ironing_filler, ironing_area.readonly(), ironing, true, root, run_ctx);

    throw_if_cancelled(run_ctx);
    if (root.empty())
        return;
    set_order(root, true);
    MutableExtrusionEntity(ctx.output).append_child_move(root.mutable_view());
}

void register_smooth_pattern_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, instance(orchestrator).c_instance());
}
}} // namespace slic3r_api::Infill
