///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "LoopDirection.hpp"

#include <cstdint>
#include <iterator>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_perimeter.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Plugins/Common/ExtrusionTreeReverse.hpp"

/*
Perimeter loop direction
========================

This plugin provides the STEP_POST_PERIMETER pass that decides in which
direction each closed perimeter loop is printed. Generators publish loops in
whatever direction their geometry gives; this pass is the only place where the
three direction settings of a region apply:

- perimeter_direction: counter-clockwise or clockwise, separately for contours
  and holes (the HOLE perimeter flag tells them apart);
- perimeter_reverse: every loop is reversed on odd layers;
- overhangs_reverse, when perimeter_reverse is off: on odd layers, an island with
  a steep overhang reverses its loops that have an overhang path. The overhang is
  steep when the island area outside the layer below is still there after a
  shrink by half of overhangs_reverse_threshold; overhang paths are the ones that
  perimeter.post_process.detect_overhang marked, so this plugin runs after it.

A loop is the first entity of the tree that carries the LOOP perimeter flag and
whose points close; its fragments (overhang splits) reverse as one path. Gap
fill and thin walls are left as they are. A loop that got its direction is no
longer reversible, so ordering may still choose its seam but not its direction.

The settings are read from the first region of each region island: generators
never group regions that disagree on them.

    LoopDirection::run_impl()
    `-- walk object layers, islands and region islands
        |-- read the direction settings of the region island
        |-- odd layer with overhangs_reverse: has_steep_overhang()
        `-- orient_loops() on the perimeter root
            |-- find each loop
            |-- reverse_tree() when its direction differs from the wanted one
            `-- forbid its reversal
*/

namespace slic3r_api { namespace Perimeter { namespace LoopDirectionPlugin {

namespace {

const char *k_loop_direction_id = "perimeter.post_process.loop_direction";
const char *k_dependencies[] = { "perimeter.post_process.detect_overhang", nullptr };

const char *k_perimeter_direction_key = "perimeter_direction";
const char *k_perimeter_reverse_key = "perimeter_reverse";
const char *k_overhangs_reverse_key = "overhangs_reverse";
const char *k_overhangs_reverse_threshold_key = "overhangs_reverse_threshold";

const raw_used_config_key k_used_config_keys[] = {
    { k_perimeter_direction_key, RAW_CO_ENUM, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_perimeter_reverse_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_overhangs_reverse_key, RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_overhangs_reverse_threshold_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

const char *const k_defined_config_keys[] = {
    k_perimeter_reverse_key
};

// Values of the perimeter_direction enum, in its option order.
constexpr int32_t k_ccw_cw = 0;
constexpr int32_t k_ccw_ccw = 1;
constexpr int32_t k_cw_ccw = 2;

constexpr uint16_t k_loop_flag = uint16_t(C_EXTRUSION_PERIMETER_FLAG_LOOP);
constexpr uint16_t k_hole_flag = uint16_t(C_EXTRUSION_PERIMETER_FLAG_HOLE);

// The direction rules of one region island.
struct Directions
{
    bool ccw_contour = true;
    bool ccw_hole = false;
    // Reverse every loop, for perimeter_reverse on an odd layer.
    bool reverse_all = false;
    // Reverse the loops with an overhang path, for overhangs_reverse on an odd layer with a steep overhang.
    bool reverse_overhangs = false;
};

void initialize_loop_direction_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = k_perimeter_reverse_key;
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_PERIMETER;
    def.category = RAW_OPTION_CATEGORY_PERIMETER;
    def.label = "Perimeter reversal on even layers";
    def.tooltip = "On even layers, all perimeter loops are reversed (it disables the overhang reversal, so it doesn't double-reverse).That setting will likely create defects on the perimeters, so it's only useful is for materials that have some direction-dependent properties (stress lines).";
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(orchestrator, &def);
}

void add_loop_direction_gui_rules(orchestrator_handle *orchestrator)
{
    // Perimeter reversal has loops to reverse only with a non-zero perimeter count.
    GuiRule::activate({ k_perimeter_reverse_key })
        .when(non_zero("perimeters"))
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator);
}

const EPropertyAttributes *first_attributes(const ExtrusionEntity &entity)
{
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        return attributes;
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        if (const EPropertyAttributes *attributes = first_attributes(entity.child(idx)))
            return attributes;
    return nullptr;
}

// Gap fill and thin walls may close on themselves, but they are not perimeter shells.
bool is_gap_fill_or_thin_wall(const ExtrusionEntity &entity)
{
    const EPropertyAttributes *attributes = first_attributes(entity);
    if (attributes == nullptr)
        return false;
    const raw_extrusion_role role = raw_extrusion_role(attributes->extrusion_role());
    return role == RAW_EXTRUSION_ROLE_GAP_FILL || (role & RAW_EXTRUSION_ROLE_THIN) != 0;
}

bool has_overhang_path(const ExtrusionEntity &entity)
{
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key)) {
        const uint32_t role = attributes->extrusion_role();
        if ((role & RAW_EXTRUSION_ROLE_PERIMETER) != 0 && (role & RAW_EXTRUSION_ROLE_BRIDGE) != 0)
            return true;
    }
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        if (has_overhang_path(entity.child(idx)))
            return true;
    return false;
}

bool is_counter_clockwise(const std::vector<c_point> &points)
{
    double twice_area = 0.;
    for (size_t idx = 0; idx < points.size(); ++idx) {
        const c_point &a = points[idx];
        const c_point &b = points[(idx + 1) % points.size()];
        twice_area += double(a.x) * double(b.y) - double(b.x) * double(a.y);
    }
    return twice_area > 0.;
}

void forbid_reversal(MutableExtrusionEntity entity)
{
    entity.disable_reverse();
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        forbid_reversal(entity.child_mutable(idx));
}

void orient_loops(MutableExtrusionEntity entity, const Directions &directions, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const ExtrusionEntity view = entity.readonly();
    const EPropertyPerimeter *perimeter = view.get(EPropertyPerimeter::key);
    if (perimeter != nullptr && (perimeter->perimeter_flags() & k_loop_flag) != 0 && view.is_closed()) {
        if (is_gap_fill_or_thin_wall(view))
            return;
        const bool hole = (perimeter->perimeter_flags() & k_hole_flag) != 0;
        bool want_ccw = hole ? directions.ccw_hole : directions.ccw_contour;
        if (directions.reverse_all || (directions.reverse_overhangs && has_overhang_path(view)))
            want_ccw = !want_ccw;
        if (is_counter_clockwise(view.collect_points()) != want_ccw)
            ExtrusionTree::reverse_tree(entity, run_ctx);
        forbid_reversal(entity);
        return;
    }
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx)
        orient_loops(entity.child_mutable(idx), directions, run_ctx);
}

// Whether the island area outside the layer below survives a shrink by half of the threshold.
bool has_steep_overhang(storage_handle *storage, const LayerIsland &island, const Layer &lower, coordf_t threshold)
{
    ClipperContext clipper(storage);
    const ClipperOperand overhang = clipper_diff(clipper(island.slice()), clipper(lower.slices()));
    return !clipper_offset(overhang, -0.5 * threshold).empty();
}

Directions directions_for(storage_handle *storage, const LayerIsland &island, const LayerRegionIsland &region_island,
                          uint32_t layer_idx, const Object &object)
{
    const Config config = region_island.region(0).print_region().config();
    const int32_t direction = config.enum_or_default(k_perimeter_direction_key, k_ccw_cw);
    Directions out;
    out.ccw_contour = direction == k_ccw_cw || direction == k_ccw_ccw;
    out.ccw_hole = direction == k_cw_ccw || direction == k_ccw_ccw;
    if (layer_idx % 2 == 0)
        return out;
    if (config.bool_or_default(k_perimeter_reverse_key, false)) {
        out.reverse_all = true;
    } else if (config.bool_or_default(k_overhangs_reverse_key, false)) {
        const double perimeter_width = unscaled(region_island.region(0).flow(RAW_EXTRUSION_ROLE_INTERNAL_PERIMETER).width);
        const coordf_t threshold =
            scale_d(config.effective_float_or_percent_or_default(k_overhangs_reverse_threshold_key, perimeter_width, 0.));
        out.reverse_overhangs = has_steep_overhang(storage, island, object.layer(layer_idx - 1), threshold);
    }
    return out;
}

} // namespace

LoopDirection &LoopDirection::instance(orchestrator_handle *orch)
{
    static LoopDirection s_instance(orch);
    return s_instance;
}

const char *LoopDirection::id_impl() const noexcept
{
    return k_loop_direction_id;
}

const char *LoopDirection::name_impl() const noexcept
{
    return "Perimeter loop direction";
}

const char *LoopDirection::description_impl() const noexcept
{
    return "Prints each perimeter loop in the direction set by perimeter_direction, perimeter_reverse and "
           "overhangs_reverse.";
}

slicing_step_t LoopDirection::step_impl() const noexcept
{
    return STEP_POST_PERIMETER;
}

const char *const *LoopDirection::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t LoopDirection::priority_impl() const noexcept
{
    // After every built-in post-process that changes perimeter paths (overhang detection is at 50).
    return 100;
}

int32_t LoopDirection::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_used_config_keys); ++idx)
            keys[idx] = k_used_config_keys[idx];
    return int32_t(std::size(k_used_config_keys));
}

int32_t LoopDirection::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

const char *LoopDirection::progress_message_format_impl() const noexcept
{
    return "Perimeter loop direction: %u / %u layers";
}

void LoopDirection::inilialize_impl(storage_handle *) const
{
    initialize_loop_direction_settings(m_orchestrator);
    add_loop_direction_gui_rules(m_orchestrator);
}

void LoopDirection::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_perimeter_generation *ctx = plugin_ctx_as_post_perimeter_generation(run_ctx);
    if (ctx != nullptr && ctx->object != nullptr)
        progress().add_max(Object(ctx->object).layer_count());
}

void LoopDirection::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_perimeter_generation *ctx = plugin_ctx_as_post_perimeter_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || run_ctx == nullptr || run_ctx->plugin_storage == nullptr ||
        ctx->get_region_island_mutable_extrusion == nullptr)
        return;

    const Object object(ctx->object);
    for (uint32_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer = object.layer(layer_idx);
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            const LayerIsland island = layer.island(island_idx);
            for (uint32_t region_island_idx = 0; region_island_idx < island.region_island_count(); ++region_island_idx) {
                throw_if_cancelled(run_ctx);
                const LayerRegionIsland region_island = island.region_island(region_island_idx);
                if (region_island.region_count() == 0)
                    continue;
                extrusion_entity_handle *root =
                    ctx->get_region_island_mutable_extrusion(region_island.handle(), RAW_EXTRUSION_ROLE_PERIMETER);
                if (root == nullptr)
                    continue;
                const Directions directions =
                    directions_for(run_ctx->plugin_storage, island, region_island, layer_idx, object);
                orient_loops(MutableExtrusionEntity(root), directions, run_ctx);
            }
        }
        progress().increment();
    }
}

void register_loop_direction_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, LoopDirection::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Perimeter::LoopDirectionPlugin

#ifdef LOOP_DIRECTION_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Perimeter::LoopDirectionPlugin::register_loop_direction_plugin(orch);
}
#endif // LOOP_DIRECTION_PLUGIN_DLL
