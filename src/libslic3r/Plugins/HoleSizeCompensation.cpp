///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
 * Hole size compensation: resize the convex holes of the sliced FFF layers.
 *
 * Settings (per region, so modifiers can use different values):
 * - hole_size_compensation: how far the wall of a convex hole moves (mm).
 *   Negative enlarges the hole, positive shrinks it. Holes are usually printed
 *   a bit too small, so a small negative value is the common case.
 * - hole_size_threshold: holes up to this area (mm²) get the full value; the
 *   value then decreases to zero for holes of four times this area.
 *
 * Only convex holes (bores) are changed: a hole with a concave corner is a
 * cut-out whose size is not a diameter. The rules are those of the legacy
 * slicer (PrintObject::_shrink_contour_holes), see HoleGeometry.hpp.
 *
 * Order: this plugin runs at STEP_POST_SLICING before the XY size compensation
 * (slicing.xy_size_compensation), so convexity and area are measured on the
 * holes as sliced. The XY plugin then moves every hole wall by
 * xy_inner_size_compensation; on a convex hole both values add up, as in the
 * legacy slicer (which applied them in a single offset).
 *
 * A hole whose wall is shared by several regions: each region moves its own
 * part of the wall with its own value, so the wall can show a small step
 * (the difference of the two values) where the regions meet.
 *
 *   run_impl()
 *     |-- read regional distances and thresholds from the raw layer
 *     |-- detail::compensate_holes(): find convex voids of merged material
 *     |   |-- remove only each region's material when growing a void
 *     |   `-- assign new material beside the region forming a shrinking void
 *     `-- publish_layer(): move replacements after checking all destinations
 *
 * Input views remain borrowed for the call; output polygons belong to caller
 * storage. Objects can run concurrently while that storage protects allocation.
 * Geometry and allocation errors propagate to the plugin's error reporting.
 */

#include "HoleSizeCompensation.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_slicing.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/HoleGeometry.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace HoleSizeCompensationPlugin {
namespace {

constexpr const char *k_plugin_id = "slicing.hole_size_compensation";
constexpr const char *k_no_dependencies[] = { nullptr };

constexpr const char *k_delta_key = "hole_size_compensation";
constexpr const char *k_threshold_key = "hole_size_threshold";

const raw_used_config_key k_used_config_keys[] = {
    { k_delta_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_threshold_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT }
};

const char *const k_defined_config_keys[] = { k_delta_key, k_threshold_key };

// A region acts on a hole when its material lies within this distance of the
// hole wall. The wall of a hole is the border of the merged layer material, so
// the regions that form it touch it exactly; the margin only absorbs rounding.
constexpr coord_t k_touch_margin = 10 * SCALED_EPSILON;

using detail::RegionHoleSettings;

// ---- Small Clipper helpers ----------------------------------------------------

StoredExPolygonCollection union_of(storage_handle *storage, const std::vector<ExPolygonCollection> &collections)
{
    ClipperOperand combined(storage);
    bool has_paths = false;
    for (const ExPolygonCollection &collection : collections) {
        if (collection.empty())
            continue;
        ClipperOperand next(storage, collection);
        if (!has_paths) {
            combined = std::move(next);
            has_paths = true;
        } else {
            combined.concat_replace(next);
        }
    }
    if (!has_paths)
        return StoredExPolygonCollection(storage);
    return clipper_union(storage, combined).to_expolygon_collection();
}

StoredExPolygonCollection offset_of(storage_handle *storage, const ExPolygonCollection &geometry, coord_t delta)
{
    if (geometry.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand subject(storage, geometry);
    return clipper_offset(storage, subject, double(delta)).to_expolygon_collection();
}

StoredExPolygonCollection intersection_of(storage_handle *storage, const ExPolygonCollection &subject,
                                          const ExPolygonCollection &clip)
{
    if (subject.empty() || clip.empty())
        return StoredExPolygonCollection(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_intersection(storage, subject_operand, clip_operand).to_expolygon_collection();
}

StoredExPolygonCollection difference_of(storage_handle *storage, const ExPolygonCollection &subject,
                                        const ExPolygonCollection &clip)
{
    if (subject.empty())
        return StoredExPolygonCollection(storage);
    if (clip.empty())
        return subject.clone(storage);
    ClipperOperand subject_operand(storage, subject);
    ClipperOperand clip_operand(storage, clip);
    return clipper_diff(storage, subject_operand, clip_operand).to_expolygon_collection();
}

// The inside of a hole as a filled shape: the hole ring, reversed to the
// counter-clockwise orientation of a contour.
StoredExPolygonCollection hole_void(storage_handle *storage, const Polygon &hole)
{
    StoredPolygon filled(storage);
    filled.copy_from(hole);
    filled.make_counter_clockwise();
    StoredPolygonCollection polygons(storage);
    polygons.push_back(filled.readonly());
    ClipperOperand operand(storage, polygons.readonly());
    return clipper_union(storage, operand).to_expolygon_collection();
}

// Convert millimetres to a scaled distance, rejecting values that cannot be a
// hole compensation (they would only come from a corrupted profile).
coord_t scaled_hole_delta(double millimeters)
{
    if (!std::isfinite(millimeters) || std::abs(millimeters) > 1000.)
        throw std::out_of_range("hole_size_compensation must be a finite value of at most 1000 mm.");
    return coord_t(std::llround(scale_d(millimeters)));
}

// ---- Settings and publication ---------------------------------------------------

void define_region_option(orchestrator_handle *orchestrator, const char *key, const char *label,
                          const char *full_label, const char *tooltip, const char *default_value,
                          const char *sidetext)
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = key;
    def.type = RAW_CO_FLOAT;
    def.category = RAW_OPTION_CATEGORY_SLICING;
    def.level = RAW_OPTION_LEVEL_ADVANCED;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_POST_SLICING;
    def.label = label;
    def.full_label = full_label;
    def.tooltip = tooltip;
    def.sidetext = sidetext;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = default_value;
    orchestrator_create_option_def(orchestrator, &def);
}

// Replace the host's region slices of one layer with the compensated ones.
void publish_layer(const run_ctx_post_slicing &ctx, uint32_t layer_index,
                   std::vector<StoredExPolygonCollection> &regions, const plugin_run_context *run_ctx)
{
    if (ctx.object_borrow_mutable_layer == nullptr || ctx.layer_region_borrow_mutable_slices == nullptr ||
        ctx.layer_recompute_slices_and_islands_from_layer_region == nullptr)
        throw std::runtime_error("Post-slicing host callbacks required to publish hole compensation are unavailable.");

    layer_handle *mutable_layer = ctx.object_borrow_mutable_layer(ctx.object, layer_index);
    if (mutable_layer == nullptr)
        throw std::runtime_error("The post-slicing host could not provide a mutable layer for hole compensation.");

    std::vector<expolygon_collection_handle *> destinations;
    destinations.reserve(regions.size());
    for (uint32_t region_idx = 0; region_idx < regions.size(); ++region_idx) {
        layer_region_handle *mutable_region = layer_get_region_mutable(mutable_layer, region_idx);
        expolygon_collection_handle *destination = mutable_region == nullptr ? nullptr :
            ctx.layer_region_borrow_mutable_slices(mutable_region);
        if (destination == nullptr)
            throw std::runtime_error("The post-slicing host could not provide mutable region slices for hole compensation.");
        destinations.push_back(destination);
    }

    throw_if_cancelled(run_ctx);
    // Move only after every destination has been obtained.
    for (std::size_t idx = 0; idx < regions.size(); ++idx)
        expolygons_move(destinations[idx], regions[idx].mutable_handle());
    ctx.layer_recompute_slices_and_islands_from_layer_region(mutable_layer);
}

} // namespace

// ---- Geometry core ----------------------------------------------------------------

namespace detail {

std::vector<StoredExPolygonCollection> compensate_holes(storage_handle *storage,
                                                         const std::vector<RegionHoleSettings> &settings,
                                                         const std::vector<ExPolygonCollection> &region_slices,
                                                         bool &changed)
{
    if (storage == nullptr || settings.size() != region_slices.size())
        throw std::invalid_argument("Hole compensation needs a storage and one setting per region.");
    const size_t region_count = region_slices.size();
    changed = false;

    // The holes are those of the merged layer material: a region that only
    // surrounds another region does not make a hole.
    const StoredExPolygonCollection material = union_of(storage, region_slices);

    // Per region: material to remove (around enlarged holes) and to add
    // (inside shrunk holes).
    std::vector<StoredExPolygonCollection> removed;
    std::vector<StoredExPolygonCollection> added;
    for (size_t idx = 0; idx < region_count; ++idx) {
        removed.emplace_back(storage);
        added.emplace_back(storage);
    }

    for (const ExPolygon island : material.readonly()) {
        for (const Polygon hole : island.holes()) {
            if (!is_convex_hole(hole))
                continue;
            const double area = std::abs(hole.area());
            const StoredExPolygonCollection inside = hole_void(storage, hole);
            if (inside.empty())
                continue;
            // The material along the hole wall, to find the regions forming it.
            const StoredExPolygonCollection wall_zone = offset_of(storage, inside.readonly(), k_touch_margin);

            for (size_t region = 0; region < region_count; ++region) {
                if (settings[region].delta == 0 || region_slices[region].empty())
                    continue;
                const coord_t delta = coord_t(std::llround(
                    double(settings[region].delta) * hole_threshold_factor(area, settings[region].threshold_area)));
                if (delta == 0)
                    continue;
                const StoredExPolygonCollection wall =
                    intersection_of(storage, wall_zone.readonly(), region_slices[region]);
                if (wall.empty())
                    continue; // this region does not form the wall of the hole

                if (delta < 0) {
                    // Enlarge: the ring between the hole and the hole grown by
                    // |delta|. Only this region's material in it is removed.
                    const StoredExPolygonCollection grown = offset_of(storage, inside.readonly(), -delta);
                    StoredExPolygonCollection ring = difference_of(storage, grown.readonly(), inside.readonly());
                    removed[region].append_move_from(ring);
                } else {
                    // Shrink: the ring between the hole and the hole shrunk by
                    // delta, limited to what this region's wall can reach.
                    const StoredExPolygonCollection shrunk = offset_of(storage, inside.readonly(), -delta);
                    const StoredExPolygonCollection ring = difference_of(storage, inside.readonly(), shrunk.readonly());
                    const StoredExPolygonCollection reach = offset_of(storage, wall.readonly(), delta + k_touch_margin);
                    StoredExPolygonCollection part = intersection_of(storage, ring.readonly(), reach.readonly());
                    added[region].append_move_from(part);
                }
            }
        }
    }

    // Apply: removals first, then additions in region order. An addition never
    // covers existing material (an island inside the hole, for example) nor
    // material already given to an earlier region.
    std::vector<StoredExPolygonCollection> result;
    result.reserve(region_count);
    StoredExPolygonCollection claimed = material.readonly().clone(storage);
    for (size_t region = 0; region < region_count; ++region) {
        StoredExPolygonCollection slices = removed[region].empty() ?
            region_slices[region].clone(storage) :
            difference_of(storage, region_slices[region], removed[region].readonly());
        changed |= !removed[region].empty();
        if (!added[region].empty()) {
            const StoredExPolygonCollection new_material =
                difference_of(storage, added[region].readonly(), claimed.readonly());
            if (!new_material.empty()) {
                changed = true;
                slices = union_of(storage, { slices.readonly(), new_material.readonly() });
                claimed = union_of(storage, { claimed.readonly(), new_material.readonly() });
            }
        }
        result.push_back(std::move(slices));
    }
    return result;
}

} // namespace detail

// ---- Plugin -------------------------------------------------------------------------

HoleSizeCompensation &HoleSizeCompensation::instance(orchestrator_handle *orch)
{
    static HoleSizeCompensation singleton(orch);
    return singleton;
}

const char *HoleSizeCompensation::id_impl() const noexcept { return k_plugin_id; }
const char *HoleSizeCompensation::name_impl() const noexcept { return "Hole size compensation"; }
const char *HoleSizeCompensation::description_impl() const noexcept
{
    return "Grows or shrinks the convex holes of FFF layer regions.";
}
slicing_step_t HoleSizeCompensation::step_impl() const noexcept { return STEP_POST_SLICING; }
const char *const *HoleSizeCompensation::dependencies_impl() const noexcept { return k_no_dependencies; }
// Before slicing.xy_size_compensation (-1000): holes are measured as sliced.
int32_t HoleSizeCompensation::priority_impl() const noexcept { return -1100; }

int32_t HoleSizeCompensation::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t HoleSizeCompensation::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_defined_config_keys), std::end(k_defined_config_keys), keys);
    return int32_t(std::size(k_defined_config_keys));
}

void HoleSizeCompensation::inilialize_impl(storage_handle *) const
{
    define_region_option(m_orchestrator, k_delta_key, "XY compensation", "XY holes compensation",
        "The convex holes will be grown / shrunk in the XY plane by the configured value"
        " (negative = inwards = remove area, positive = outwards = add area, should be negative as the holes are always a bit smaller irl)."
        " This might be useful for fine-tuning hole sizes."
        "\nThis setting behaves the same as 'Inner XY size compensation' but only for convex shapes. It's added to 'Inner XY size compensation', it does not replace it. ",
        "0", "mm");
    define_region_option(m_orchestrator, k_threshold_key, "Threshold", "XY holes threshold",
        "Maximum area for the hole where the hole_size_compensation will apply fully."
        " After that, it will decrease down to 0 for four times this area."
        " Set to 0 to let the hole_size_compensation apply fully for all detected holes",
        "100", "mm²");
}

void HoleSizeCompensation::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_slicing *ctx = plugin_ctx_as_post_slicing(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
        throw std::runtime_error("Hole compensation requires a valid post-slicing object and plugin storage.");

    const Object object(ctx->object);
    storage_handle *storage = run_ctx->plugin_storage;
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer = object.layer(layer_idx);
        const uint32_t region_count = layer.region_count();

        // Settings only: skip the layer when no region with material moves its holes.
        std::vector<RegionHoleSettings> settings(region_count);
        std::vector<ExPolygonCollection> slices;
        slices.reserve(region_count);
        bool needed = false;
        for (uint32_t region_idx = 0; region_idx < region_count; ++region_idx) {
            const LayerRegion region = layer.region(region_idx);
            const Config config = region.print_region().config();
            settings[region_idx].delta = scaled_hole_delta(config.float_or_default(k_delta_key, 0.));
            settings[region_idx].threshold_area = scale_d(scale_d(config.float_or_default(k_threshold_key, 100.)));
            slices.push_back(region.slices());
            needed |= settings[region_idx].delta != 0 && !slices.back().empty();
        }
        if (!needed)
            continue;

        bool changed = false;
        std::vector<StoredExPolygonCollection> result = detail::compensate_holes(storage, settings, slices, changed);
        if (changed)
            publish_layer(*ctx, layer_idx, result, run_ctx);
    }
}

void register_hole_size_compensation_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, HoleSizeCompensation::instance(orch).c_instance());
}

}} // namespace slic3r_api::HoleSizeCompensationPlugin
