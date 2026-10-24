///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
 * XY size compensation: grow or shrink the sliced FFF layers in the XY plane.
 *
 * Settings (all per region, so modifiers can use different values):
 * - xy_size_compensation: moves the outside contour of the object (mm).
 * - xy_inner_size_compensation: moves the walls of holes (mm).
 * - first_layer_size_compensation (+ _layers): extra amount added to both on
 *   the first layers, fading out over the configured number of layers.
 * - first_layer_size_compensation_no_collapse: on those first layers, never
 *   remove a narrow wall completely.
 * See XYSizeCompensationHelpers.hpp for the words layer, region, island, mask.
 *
 * How the host calls this plugin:
 *   register_simple_xy_size_compensation_plugin()   registers the plugin
 *   inilialize_impl()                               defines the settings above
 *   run_impl()                                      once per object, at STEP_POST_SLICING,
 *                                                   after slicing and before perimeters
 *
 * What run_impl() does for each layer of the object:
 *   1. read_layer_settings(): read each region's settings (cheap, no geometry).
 *      Skip the layer when nothing has to move.
 *   2. build_islands(): split the layer into islands, and each island into the
 *      parts owned by each region.
 *   3. compensate_layer() (XYSizeCompensationHelpers.cpp): compute the new
 *      material of each region.
 *   4. publish_layer(): replace the host's region slices with the result. The
 *      host then rebuilds its own island list from them.
 *
 * Thread safety: the host may run several objects at the same time. Each run
 * only touches its own object, and the shared plugin_storage locks its own
 * allocations, so runs need no extra lock. Never clear plugin_storage here:
 * other runs may still own geometry in it.
 */

#include "XYSizeCompensation.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_slicing.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/XYSizeCompensationHelpers.hpp"

namespace slic3r_api { namespace XYSizeCompensationPlugin {
namespace {

using detail::IslandInput;
using detail::RegionSettings;
using detail::checked_scaled_delta;
using detail::compensate_layer;
using slic3r_api::ExPolygon;
using slic3r_api::ExPolygonCollection;
using slic3r_api::StoredExPolygonCollection;

constexpr const char *k_plugin_id = "slicing.xy_size_compensation";
constexpr const char *k_no_dependencies[] = { nullptr };

constexpr const char *k_outer_key = "xy_size_compensation";
constexpr const char *k_inner_key = "xy_inner_size_compensation";
constexpr const char *k_first_layer_key = "first_layer_size_compensation";
constexpr const char *k_first_layer_count_key = "first_layer_size_compensation_layers";
constexpr const char *k_no_collapse_key = "first_layer_size_compensation_no_collapse";
constexpr const char *k_raft_layers_key = "raft_layers";

const raw_used_config_key k_used_config_keys[] = {
    // Plugin-owned settings have explicit metadata. Built-in flow inputs below retain NONE as the API
    // wildcard because their native definitions do not publish these fields.
    { k_outer_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_inner_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_first_layer_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_first_layer_count_key, RAW_CO_INT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_no_collapse_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { "external_perimeter_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "external_perimeter_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "external_perimeter_extrusion_change_odd_layers", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "external_perimeter_overlap", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "perimeter_extruder", RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "first_layer_extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_width", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "extrusion_spacing", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "filament_max_overlap", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_raft_layers_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

const char *const k_defined_config_keys[] = {
    k_outer_key,
    k_inner_key,
    k_first_layer_key,
    k_first_layer_count_key,
    k_no_collapse_key
};

// Build a region's mask inside one island. Only an empty island is skipped:
// Clipper accepts any other ring as input, so no stricter validity check is
// needed. A region with no slices produces an empty mask.
StoredExPolygonCollection intersect_island_with_region(storage_handle *storage,
                                                       const ExPolygon &island,
                                                       const ExPolygonCollection &region_slices)
{
    if (island.contour().empty() || region_slices.empty())
        return StoredExPolygonCollection(storage);
    slic3r_api::ClipperOperand island_operand(storage, island);
    slic3r_api::ClipperOperand region_operand(storage, region_slices);
    return slic3r_api::clipper_intersection(storage, island_operand, region_operand).to_expolygon_collection();
}

void define_region_option(orchestrator_handle *orchestrator,
                          const char *key,
                          raw_config_option_type type,
                          raw_option_level level,
                          const char *label,
                          const char *full_label,
                          const char *tooltip,
                          const char *default_value,
                          const char *sidetext,
                          int32_t precision,
                          raw_config_option_mode mode,
                          bool has_min = false,
                          double min_value = 0.,
                          bool has_max = false,
                          double max_value = 0.)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = type;
    def.category = RAW_OPTION_CATEGORY_SLICING;
    def.level = level;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_POST_SLICING;
    def.label = label;
    def.full_label = full_label;
    def.tooltip = tooltip;
    def.sidetext = sidetext;
    def.precision = precision;
    def.has_min = has_min ? 1 : 0;
    def.min_value = min_value;
    def.has_max = has_max ? 1 : 0;
    def.max_value = max_value;
    def.mode = mode;
    def.default_serialized_value = default_value;
    orchestrator_create_option_def(orchestrator, &def);
}

double first_layer_addition(const Config &region_config,
                            uint32_t layer_index,
                            int32_t first_layer_count,
                            int32_t raft_layers)
{
    const double configured = region_config.float_or_default(k_first_layer_key, 0.);
    if (layer_index >= uint32_t(first_layer_count) || int32_t(layer_index) < raft_layers)
        return 0.;
    // Match the legacy object-local formula: raft layers are skipped, but k is not renumbered.
    return double(first_layer_count - int32_t(layer_index)) * configured / double(first_layer_count);
}

void publish_layer(const run_ctx_post_slicing &ctx,
                   uint32_t layer_index,
                   detail::LayerOutput &output,
                   const plugin_run_context *run_ctx)
{
    if (ctx.object_borrow_mutable_layer == nullptr ||
        ctx.layer_region_borrow_mutable_slices == nullptr ||
        ctx.layer_recompute_slices_and_islands_from_layer_region == nullptr)
        throw std::runtime_error("Post-slicing host callbacks required to publish XY compensation are unavailable.");

    layer_handle *mutable_layer = ctx.object_borrow_mutable_layer(ctx.object, layer_index);
    if (mutable_layer == nullptr)
        throw std::runtime_error("The post-slicing host could not provide a mutable layer for XY compensation.");

    std::vector<expolygon_collection_handle *> destinations;
    destinations.reserve(output.regions.size());
    for (uint32_t region_idx = 0; region_idx < output.regions.size(); ++region_idx) {
        layer_region_handle *mutable_region = layer_get_region_mutable(mutable_layer, region_idx);
        expolygon_collection_handle *destination = mutable_region == nullptr ? nullptr :
            ctx.layer_region_borrow_mutable_slices(mutable_region);
        if (destination == nullptr)
            throw std::runtime_error("The post-slicing host could not provide mutable region slices for XY compensation.");
        destinations.push_back(destination);
    }

    throw_if_cancelled(run_ctx);
    // The complete replacement collections are already SDK-owned by this run.
    // Move them into the host only after every destination has been validated.
    for (std::size_t idx = 0; idx < output.regions.size(); ++idx)
        expolygons_move(destinations[idx], output.regions[idx].mutable_handle());
    ctx.layer_recompute_slices_and_islands_from_layer_region(mutable_layer);
}

// Everything read_layer_settings() learns about one layer, without touching
// any geometry.
struct LayerSettings
{
    // One entry per region of the layer, in the host's region order.
    std::vector<RegionSettings> regions;
    // False when no region that has material on this layer moves its
    // boundary: the layer can then be left exactly as it is.
    bool needs_compensation = false;
    // Index when exactly one region has material; otherwise no value.
    std::optional<uint32_t> single_material_region;
};

// Read the compensation settings of each region for this layer and convert
// them to scaled distances. This only reads configuration and region state
// from the host and allocates nothing in plugin_storage.
LayerSettings read_layer_settings(const Layer &layer, uint32_t layer_idx, int32_t raft_layers)
{
    LayerSettings result;
    const uint32_t region_count = layer.region_count();
    result.regions.resize(region_count);
    uint32_t material_region_count = 0;
    for (uint32_t region_idx = 0; region_idx < region_count; ++region_idx) {
        const LayerRegion region = layer.region(region_idx);
        const Config region_config = region.print_region().config();
        const bool has_material = !region.slices().empty();

        // The first-layer amount fades out over first_layer_count layers.
        const int32_t first_layer_count = region_config.int_or_default(k_first_layer_count_key, 1);
        if (first_layer_count < 1 || first_layer_count > 30)
            throw std::out_of_range("The XY compensation first-layer count must be between 1 and 30.");
        const double first_add = first_layer_addition(region_config, layer_idx, first_layer_count, raft_layers);

        RegionSettings &settings = result.regions[region_idx];
        settings.outer_delta = checked_scaled_delta(region_config.float_or_default(k_outer_key, 0.) + first_add);
        settings.inner_delta = checked_scaled_delta(region_config.float_or_default(k_inner_key, 0.) + first_add);
        settings.no_collapse = int32_t(layer_idx) < first_layer_count && int32_t(layer_idx) >= raft_layers &&
                               region_config.bool_or_default(k_no_collapse_key, true);

        // External-perimeter width plus spacing defines the protection threshold.
        // The routine limits withdrawal near this width and leaves thinner walls
        // unchanged; it does not thicken them. Only material-bearing regions
        // that request a withdrawal need this value.
        const bool removes_material = settings.outer_delta < 0 || settings.inner_delta < 0;
        if (has_material && settings.no_collapse && removes_material) {
            const c_flow external_flow = region.flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
            settings.minimum_contour_width_mm = unscaled(double(external_flow.width) + double(external_flow.spacing));
        }

        if (!has_material)
            continue;
        // A region without material cannot change the layer, whatever its settings.
        result.needs_compensation |= settings.outer_delta != 0 || settings.inner_delta != 0;
        ++material_region_count;
        result.single_material_region = region_idx;
    }
    if (material_region_count != 1)
        result.single_material_region.reset();
    return result;
}

// Split the layer into islands, and each island into the part owned by each
// region (its "mask"). The result lives in plugin_storage.
//
// The islands are recomputed as the exact union of the region slices instead
// of reusing the host's cached island list, which may contain small
// safety-offset artifacts.
std::vector<IslandInput> build_islands(storage_handle *storage,
                                       const Layer &layer,
                                       const LayerSettings &settings,
                                       const plugin_run_context *run_ctx)
{
    const uint32_t region_count = layer.region_count();

    // All material of the layer, merged into islands.
    StoredExPolygonCollection all_material(storage);
    for (uint32_t region_idx = 0; region_idx < region_count; ++region_idx)
        all_material.append_copy_from(layer.region(region_idx).slices());
    slic3r_api::ClipperOperand material_operand(storage, all_material.readonly());
    const StoredExPolygonCollection islands_geometry =
        slic3r_api::clipper_union(storage, material_operand).to_expolygon_collection();

    std::vector<IslandInput> islands;
    islands.reserve(islands_geometry.size());
    for (const ExPolygon source : islands_geometry) {
        throw_if_cancelled(run_ctx);
        IslandInput island(storage);
        island.original.copy_from(source);
        island.region_masks.reserve(region_count);
        for (uint32_t region_idx = 0; region_idx < region_count; ++region_idx) {
            if (settings.single_material_region) {
                // Only one region has material, so it owns every island whole:
                // no intersection needed.
                island.region_masks.emplace_back(storage);
                if (region_idx == *settings.single_material_region)
                    island.region_masks.back().push_back(source);
            } else {
                island.region_masks.push_back(intersect_island_with_region(
                    storage, source, layer.region(region_idx).slices()));
            }
        }
        islands.push_back(std::move(island));
    }
    return islands;
}

// The fallback in compensate_layer() assigns unclaimed material by contact,
// then candidate priority. Tiny remainders are rounding; significant miter
// artifacts at regional boundaries are a known limit of the algorithm, so they
// are kept but reported to the user.
void warn_about_fallback(const detail::LayerOutput &output, uint32_t layer_idx, const plugin_run_context *run_ctx)
{
    if (output.fallback_area <= detail::k_insignificant_area)
        return;
    const std::string message = "XY size compensation: " +
        std::to_string(unscaled(unscaled(output.fallback_area))) + " mm2 of new material on layer " +
        std::to_string(layer_idx) + " had no owning region and was assigned using the regional fallback rule.";
    report_warning(run_ctx, message.c_str());
}

} // namespace

SimpleXYSizeCompensation &SimpleXYSizeCompensation::instance(orchestrator_handle *orch)
{
    static SimpleXYSizeCompensation singleton(orch);
    return singleton;
}

const char *SimpleXYSizeCompensation::id_impl() const noexcept { return k_plugin_id; }
const char *SimpleXYSizeCompensation::name_impl() const noexcept { return "Simple XY size compensation"; }
const char *SimpleXYSizeCompensation::description_impl() const noexcept
{
    return "Applies independent outer and inner XY size compensation to FFF layer regions.";
}
slicing_step_t SimpleXYSizeCompensation::step_impl() const noexcept { return STEP_POST_SLICING; }
const char *const *SimpleXYSizeCompensation::dependencies_impl() const noexcept { return k_no_dependencies; }
int32_t SimpleXYSizeCompensation::priority_impl() const noexcept { return -1000; }

int32_t SimpleXYSizeCompensation::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t SimpleXYSizeCompensation::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
    return int32_t(std::size(k_defined_config_keys));
}

void SimpleXYSizeCompensation::inilialize_impl(storage_handle *) const
{
    // These are plugin-owned REGION options. Their legacy visibility is retained without vendor-only flags.
    define_region_option(m_orchestrator, k_outer_key, RAW_CO_FLOAT, RAW_OPTION_LEVEL_ADVANCED,
        "Outer", "Outer XY size compensation",
        "Change the outside size of each region in millimeters. Positive values add material; negative values remove it.",
        "0", "mm", 6, RAW_CONFIG_OPTION_MODE_EXPERT);
    define_region_option(m_orchestrator, k_inner_key, RAW_CO_FLOAT, RAW_OPTION_LEVEL_ADVANCED,
        "Inner", "Inner XY size compensation",
        "Change hole size independently in millimeters. Positive values add material into holes; negative values remove material.",
        "0", "mm", 6, RAW_CONFIG_OPTION_MODE_EXPERT);
    define_region_option(m_orchestrator, k_first_layer_key, RAW_CO_FLOAT, RAW_OPTION_LEVEL_ADVANCED,
        "First layer", "XY First layer compensation",
        "The first layer will be grown / shrunk in the XY plane by the configured value "
        "to compensate for the 1st layer squish aka an Elephant Foot effect. (should be negative = inwards = remove area)",
        "0", "mm", 6, RAW_CONFIG_OPTION_MODE_ADV_EXP);
    define_region_option(m_orchestrator, k_first_layer_count_key, RAW_CO_INT, RAW_OPTION_LEVEL_ADVANCED,
        "Height in layers", "XY First layer compensation height in layers",
        "Number of object-local layers over which first-layer size compensation tapers.",
        "1", "layers", 0, RAW_CONFIG_OPTION_MODE_ADV_EXP, true, 1., true, 30.);
    define_region_option(m_orchestrator, k_no_collapse_key, RAW_CO_BOOL, RAW_OPTION_LEVEL_ADVANCED,
        "No collapse", "XY First layer compensation: no collapse",
        "Preserve narrow first-layer walls using the external-perimeter width. This applies to the configured first layers.",
        "1", nullptr, 0, RAW_CONFIG_OPTION_MODE_ADV_EXP);
}

void SimpleXYSizeCompensation::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_slicing *ctx = plugin_ctx_as_post_slicing(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
        throw std::runtime_error("XY size compensation requires a valid post-slicing object and plugin storage.");

    const Object object(ctx->object);
    // Raft layers are part of the object's layer list but are not the first
    // layers of the part itself: first-layer compensation skips them.
    const int32_t raft_layers = std::max(0, object.config().int_or_default(k_raft_layers_key, 0));
    storage_handle *storage = run_ctx->plugin_storage;

    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer = object.layer(layer_idx);
        if (layer.region_count() == 0)
            continue;

        // 1. Settings only: decide whether this layer changes at all.
        const LayerSettings settings = read_layer_settings(layer, layer_idx, raft_layers);
        if (!settings.needs_compensation)
            continue;

        // 2. Islands of the layer, and the part of each island owned by each region.
        const std::vector<IslandInput> islands = build_islands(storage, layer, settings, run_ctx);
        if (islands.empty())
            continue;

        // 3. Compute the compensated material of each region.
        detail::LayerOutput output = compensate_layer(storage, settings.regions, islands);
        warn_about_fallback(output, layer_idx, run_ctx);

        // 4. Replace the host's region slices with the result.
        publish_layer(*ctx, layer_idx, output, run_ctx);
    }
}

void register_simple_xy_size_compensation_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SimpleXYSizeCompensation::instance(orch).c_instance());
}

}} // namespace slic3r_api::XYSizeCompensationPlugin
