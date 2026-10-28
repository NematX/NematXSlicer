///|/ Copyright (c) 2026 NematX
///|/
///|/ Released under the GNU Affero General Public License, version 3 or later.

/*
Hole compensation by diameter
=============================

Region setting hole_size_compensations_curve: a curve giving, for a hole
diameter (mm), an XY compensation (mm) added to the convex holes of that
diameter. Negative enlarges the hole, positive shrinks it, as with the host's
hole_size_compensation, to which it adds up. An empty curve does nothing.

The plugin runs at STEP_POST_SLICING, priority -1050:
- after slicing.hole_size_compensation (-1100), so the diameter is measured on
  the hole already moved by hole_size_compensation (at most twice that value
  larger or smaller; the legacy slicer measured it before);
- before slicing.xy_size_compensation (-1000), which then adds
  xy_inner_size_compensation.

The regional rules (which holes, which regions, how a shared hole wall moves)
are those of the SDK compensate_convex_holes(), shared with the host plugin.
This file only reads the curves and publishes the result.
*/

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_slicing.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/HoleCompensation.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace nematx_hole_size_curve {
namespace {

using namespace slic3r_api;

constexpr const char *k_plugin_id = "nematx_hole_size_curve";
constexpr const char *k_curve_key = "hole_size_compensations_curve";
const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_config_keys[] = {
    { k_curve_key, RAW_CO_GRAPH, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};
const char *const k_defined_config_keys[] = { k_curve_key };

// The curve of a region, or no value when it has no point (or is disabled).
std::optional<GraphData> region_curve(const LayerRegion &region)
{
    const graph_data_handle *handle = region.print_region().config().graph_or_default(k_curve_key, nullptr);
    if (handle == nullptr)
        return std::nullopt;
    const GraphData curve(handle);
    if (curve.size() == 0)
        return std::nullopt;
    return curve;
}

// Replace the host's region slices of one layer with the compensated ones.
void publish_layer(const run_ctx_post_slicing &ctx, uint32_t layer_index,
                   std::vector<StoredExPolygonCollection> &regions, const plugin_run_context *run_ctx)
{
    if (ctx.object_borrow_mutable_layer == nullptr || ctx.layer_region_borrow_mutable_slices == nullptr ||
        ctx.layer_recompute_slices_and_islands_from_layer_region == nullptr)
        throw std::runtime_error("Post-slicing host callbacks required to publish the hole curve compensation are unavailable.");
    layer_handle *mutable_layer = ctx.object_borrow_mutable_layer(ctx.object, layer_index);
    if (mutable_layer == nullptr)
        throw std::runtime_error("The post-slicing host could not provide a mutable layer.");
    std::vector<expolygon_collection_handle *> destinations;
    for (uint32_t region_idx = 0; region_idx < regions.size(); ++region_idx) {
        layer_region_handle *mutable_region = layer_get_region_mutable(mutable_layer, region_idx);
        expolygon_collection_handle *destination = mutable_region == nullptr ? nullptr :
            ctx.layer_region_borrow_mutable_slices(mutable_region);
        if (destination == nullptr)
            throw std::runtime_error("The post-slicing host could not provide mutable region slices.");
        destinations.push_back(destination);
    }
    throw_if_cancelled(run_ctx);
    // Move only after every destination has been obtained.
    for (size_t idx = 0; idx < regions.size(); ++idx)
        expolygons_move(destinations[idx], regions[idx].mutable_handle());
    ctx.layer_recompute_slices_and_islands_from_layer_region(mutable_layer);
}

class HoleSizeCurve final : public PluginBase
{
public:
    explicit HoleSizeCurve(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Hole compensation by diameter"; }
    const char *description_impl() const noexcept override
    {
        return "Adds an XY compensation read from a curve of the hole diameter to convex holes.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_POST_SLICING; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // After slicing.hole_size_compensation (-1100), before slicing.xy_size_compensation (-1000).
    int32_t priority_impl() const noexcept override { return -1050; }

    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
        return int32_t(std::size(k_used_config_keys));
    }
    int32_t defined_config_keys(const char **keys) const noexcept override
    {
        if (keys != nullptr)
            std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
        return int32_t(std::size(k_defined_config_keys));
    }

    void inilialize_impl(storage_handle *) const override
    {
        // Graph editor settings of the legacy slicer.
        static const raw_graph_type allowed_types[] = { RAW_GRAPH_TYPE_LINEAR, RAW_GRAPH_TYPE_SQUARE };
        raw_graph_settings graph = {};
        graph.title = "Compensation per hole diameter";
        graph.description = "Add this compensation to the existing one (only for convex holes)";
        graph.x_label = "Hole diameter (mm)";
        graph.y_label = "XY compensation (mm)";
        graph.null_label = "No compensation";
        graph.label_min_x = "Hole min size";
        graph.label_max_x = "Hole max size";
        graph.label_min_y = "Min compensation";
        graph.label_max_y = "Max compensation";
        graph.min_x = 0;
        graph.max_x = 10;
        graph.step_x = 0.01;
        graph.min_y = -10;
        graph.max_y = 10;
        graph.step_y = 0.001;
        graph.allowed_types = allowed_types;
        graph.allowed_type_count = uint32_t(std::size(allowed_types));
        graph.reset_values.type = RAW_GRAPH_TYPE_NONE;

        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_curve_key;
        def.type = RAW_CO_GRAPH;
        def.category = RAW_OPTION_CATEGORY_SLICING;
        def.level = RAW_OPTION_LEVEL_ADVANCED;
        def.container_type = RAW_CONTAINER_TYPE_REGION;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_POST_SLICING;
        def.label = def.full_label = "XY holes curve compensation";
        def.tooltip = "This string is edited by a Dialog and contains XY compensations for different hole diameters."
                      "\nThe graph is in mm of compensation per mm of diameter.";
        def.mode = raw_config_option_mode(RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI);
        // Empty linear curve (begin 0, end 0, linear): no compensation.
        def.default_serialized_value = "0:0:1";
        def.graph_settings = &graph;
        const option_def_error_code result = orchestrator_create_option_def(m_orchestrator, &def);
        if (result != OPTION_DEF_ERROR_OK)
            throw std::runtime_error("Cannot define hole_size_compensations_curve (option definition error " +
                                     std::to_string(int(result)) + ").");
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_post_slicing *ctx = plugin_ctx_as_post_slicing(run_ctx);
        if (ctx == nullptr || ctx->object == nullptr || run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            throw std::runtime_error("The hole curve compensation requires a post-slicing object and plugin storage.");

        const Object object(ctx->object);
        for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const Layer layer = object.layer(layer_idx);
            std::vector<std::optional<GraphData>> curves;
            std::vector<ExPolygonCollection> slices;
            bool needed = false;
            for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
                const LayerRegion region = layer.region(region_idx);
                curves.push_back(region_curve(region));
                slices.push_back(region.slices());
                needed |= curves.back().has_value() && !slices.back().empty();
            }
            if (!needed)
                continue;

            bool changed = false;
            std::vector<StoredExPolygonCollection> result = compensate_convex_holes(run_ctx->plugin_storage, slices,
                [&curves](size_t region, const ConvexHoleInfo &hole) {
                    if (!curves[region])
                        return coord_t(0);
                    const double millimeters = curves[region]->interpolate(unscaled(hole.diameter));
                    if (!std::isfinite(millimeters) || std::abs(millimeters) > 1000.)
                        throw std::out_of_range("hole_size_compensations_curve gives an invalid compensation.");
                    return coord_t(std::llround(scale_d(millimeters)));
                },
                changed);
            if (changed)
                publish_layer(*ctx, layer_idx, result, run_ctx);
        }
    }
};

} // namespace
} // namespace nematx_hole_size_curve

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    static nematx_hole_size_curve::HoleSizeCurve plugin(orchestrator);
    orchestrator_register_plugin(orchestrator, plugin.c_instance());
}

// Keep this header last so its ABI registration includes every public API header above.
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
