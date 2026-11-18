///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SupportDemandBridgeRemoval.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_perimeter.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Plugins/Support/Common/SupportBridgeStrokes.hpp"
#include "libslic3r/Plugins/Support/Common/SupportDemand.hpp"

/*
Support-demand bridge removal
=============================

This plugin refines the support demand for the `dont_support_bridges` object
setting. It runs in STEP_POST_PERIMETER, once the perimeters are final, because
it finds bridges from the perimeter and gap-fill extrusions; the demand itself
was written by STEP_SUPPORT_DEMAND right after slicing. It edits the islands'
support demand before support generation; it does not remove generated support
extrusions. Its priority puts it after the plugins that add perimeters over
overhangs and before the ones that only alter their look (fuzzy skin) or tag
them (overhang detection).

For each object layer after the first, the plugin examines perimeter and gap
fill extrusion trees in every island. Straight segments are split at their
intersections with the lower-layer support mask. An unsupported interval is
treated as a bridge only when both of its extended endpoints reach lower
support. The resulting stroke area is then subtracted from the island's
existing support demand through the step context.
Each branch resolves its nearest extrusion attributes, so internal perimeters and
gap fills use their own printed widths rather than one external-perimeter width.
Thin subtraction leftovers are removed only when they touch actual bridge strokes
and stay inside their one-line-width neighbourhood; independent demand is retained.

The normal call flow is:

    SupportDemandBridgeRemoval::inilialize_impl()
    `-- create dont_support_bridges and its GUI activation rules

    SupportDemandBridgeRemoval::setup_run_impl()
    `-- reserve progress for object islands after layer zero

    SupportDemandBridgeRemoval::run_impl()
    `-- read the object setting
        |-- disabled?
        |   `-- leave support demand unchanged
        `-- walk object layers after the first
            `-- for each layer island
                |-- bridge_areas_for_island()
                |   |-- visit perimeter and gap-fill extrusion trees
                |   |-- Common::grown_lower_support(): expand lower-layer support by the probe distance
                |   `-- Common::append_entity_strokes() (SupportBridgeStrokes.hpp)
                |       |-- split straight segments at support intersections
                |       `-- keep unsupported intervals anchored at both ends
                 `-- remove_bridges_from_island_demand()
                     |-- subtract bridge areas from the island's demand
                     `-- Common::remove_leftovers_along_strokes(): clean only bounded touching thin pieces

Bridged infill is not handled here: the infill is generated after this step, so
support.demand.bridge_infill_removal removes the demand under its bridge lines at
STEP_POST_INFILL, with the same rule (shared in Common/SupportBridgeStrokes.hpp).

Only a bridge anchored at both ends loses its demand: a straight segment with a
free end (a cantilever) keeps it. The demand is removed whenever dont_support_bridges
is on, whether or not it comes from an enforcer: the demand of the enforcers is not
distinguished from the demand of the overhangs (known limit of this version).

Curved extrusion segments are ignored by the bridge-area detector. The
operation is limited to the current island and its immediately lower layer;
support generation and any later support post-processing remain separate
pipeline responsibilities.
*/

namespace slic3r_api { namespace Support { namespace SupportDemandBridgeRemovalPlugin {

namespace {

const char *k_support_demand_bridge_removal_id = "support.demand.bridge_removal";
const char *k_dependencies[] = { nullptr };
const raw_used_config_key k_used_config_keys[] = {
    { "dont_support_bridges", RAW_CO_BOOL, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT },
    { "support_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

struct BridgeRemovalConfig
{
    bool dont_support_bridges = false;
};

// Add one role's bridge strokes using inherited/local printed widths and return the widest contributing stroke (scaled).
double append_bridge_areas_from_region_island(storage_handle *storage,
                                             const ExPolygonCollection &lower_support,
                                             const Common::BridgeLineFlow &flow,
                                             const ClipperContext &clipper,
                                             const LayerRegionIsland &region_island,
                                             raw_extrusion_role role,
                                             ClipperOperand &bridges);
// Return united perimeter/gap-fill strokes in storage and their maximum contributing width through line_width.
ClipperOperand bridge_areas_for_island(storage_handle *storage,
                                      const Layer &lower_layer,
                                      const LayerIsland &island,
                                      const ClipperContext &clipper,
                                      double &line_width);
// Replace the island's demand only after stroke subtraction and bounded thin-leftover cleanup have succeeded.
void remove_bridges_from_island_demand(const plugin_run_context *run_ctx,
                                      storage_handle *storage,
                                      const Layer &lower_layer,
                                      const LayerIsland &island,
                                      const ClipperContext &clipper);

BridgeRemovalConfig read_object_config(const Object &object)
{
    const Config object_config = object.config();
    BridgeRemovalConfig out;
    out.dont_support_bridges = object_config.get("dont_support_bridges").get_bool();
    return out;
}

double append_bridge_areas_from_region_island(storage_handle *storage,
                                            const ExPolygonCollection &lower_support,
                                            const Common::BridgeLineFlow &flow,
                                            const ClipperContext &clipper,
                                            const LayerRegionIsland &region_island,
                                            raw_extrusion_role role,
                                            ClipperOperand &bridges)
{
    if (!region_island.has_extrusion(role))
        return 0.;

    const ExtrusionEntity root(region_island.extrusion(role));
    return Common::append_entity_strokes(storage, lower_support, flow, clipper, root, bridges);
}

ClipperOperand bridge_areas_for_island(storage_handle *storage,
                                      const Layer &lower_layer,
                                      const LayerIsland &island,
                                      const ClipperContext &clipper,
                                      double &line_width)
{
    ClipperOperand bridges = clipper.empty();
    line_width = 0.;
    const Common::BridgeLineFlow flow = Common::bridge_line_flow(island, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, false);
    StoredExPolygonCollection lower_support = Common::grown_lower_support(clipper, lower_layer, flow);

    for (uint32_t region_island_idx = 0; region_island_idx < island.region_island_count(); ++region_island_idx) {
        const LayerRegionIsland region_island = island.region_island(region_island_idx);
        line_width = std::max(line_width, append_bridge_areas_from_region_island(storage, lower_support, flow, clipper, region_island,
            RAW_EXTRUSION_ROLE_PERIMETER, bridges));
        line_width = std::max(line_width, append_bridge_areas_from_region_island(storage, lower_support, flow, clipper, region_island,
            RAW_EXTRUSION_ROLE_GAP_FILL, bridges));
    }

    return bridges.empty() ? std::move(bridges) : clipper_union(bridges);
}

void remove_bridges_from_island_demand(const plugin_run_context *run_ctx,
                                       storage_handle *storage,
                                       const Layer &lower_layer,
                                       const LayerIsland &island,
                                       const ClipperContext &clipper)
{
    const ExPolygonCollection existing = island.support_demand();
    if (existing.empty())
        return;

    double line_width = 0.;
    ClipperOperand bridges = bridge_areas_for_island(storage, lower_layer, island, clipper, line_width);
    if (bridges.empty())
        return;

    ClipperOperand remaining = clipper_diff_with_safety_offset(clipper(existing), bridges);
    Common::remove_leftovers_along_strokes(clipper, bridges, line_width, remaining);
    StoredExPolygonCollection cleaned_demand = remaining.to_expolygon_collection();
    expolygons_move(support_demand_get_island_mutable(run_ctx, island.handle()), cleaned_demand.mutable_handle());
}

} // namespace

SupportDemandBridgeRemoval &
SupportDemandBridgeRemoval::instance(orchestrator_handle *orch)
{
    static SupportDemandBridgeRemoval s_instance(orch);
    return s_instance;
}

const char *SupportDemandBridgeRemoval::id_impl() const noexcept
{
    return k_support_demand_bridge_removal_id;
}

const char *SupportDemandBridgeRemoval::name_impl() const noexcept
{
    return "Support demand bridge removal";
}

const char *SupportDemandBridgeRemoval::description_impl() const noexcept
{
    return "Remove support demand under bridge spans when bridge support is disabled.";
}

slicing_step_t SupportDemandBridgeRemoval::step_impl() const noexcept
{
    return STEP_POST_PERIMETER;
}

const char *const *SupportDemandBridgeRemoval::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t SupportDemandBridgeRemoval::priority_impl() const noexcept
{
    // After ExtraPerimetersOnOverhangs (-20) and ExtraPerimeterOverhangWave (-19),
    // which add perimeters over overhangs; before FuzzySkin (0) and DetectOverhang (50).
    return -10;
}

int32_t SupportDemandBridgeRemoval::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t SupportDemandBridgeRemoval::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        keys[0] = k_used_config_keys[0].key;
    return 1;
}

const char *SupportDemandBridgeRemoval::progress_message_format_impl() const noexcept
{
    return "Support demand bridge removal: %u / %u islands";
}

const char *SupportDemandBridgeRemoval::print_ui_fragment() noexcept
{
    return "page:Support material\n"
           "group:Options for support material and raft\n"
           "setting:insert$afterline$Contact XY separation:dont_support_bridges\n";
}

void SupportDemandBridgeRemoval::inilialize_impl(storage_handle *) const
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "dont_support_bridges";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_OBJECT;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Don't support bridges";
    def.category = RAW_OPTION_CATEGORY_SUPPORT;
    def.invalidates_step = STEP_POST_PERIMETER;
    def.tooltip = "Experimental option for preventing support material from being generated under bridged areas.";
    def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_PRUSA;
    def.default_serialized_value = "1";
    orchestrator_create_option_def(m_orchestrator, &def);

    orchestrator_add_ui_fragment(m_orchestrator,
                                 "print.ui",
                                 k_support_demand_bridge_removal_id,
                                 SupportDemandBridgeRemoval::print_ui_fragment(),
                                 0);

    GuiRule::activate("dont_support_bridges")
        .when(GuiCondition::true_("support_material"))
        .add_to(m_orchestrator);
}

// Borrows a post-perimeter payload and reserves progress above layer zero when bridge removal is enabled; missing payload does nothing and cancellation propagates.
void SupportDemandBridgeRemoval::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_perimeter_generation *ctx = plugin_ctx_as_post_perimeter_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    const BridgeRemovalConfig config = read_object_config(object);
    if (config.dont_support_bridges)
        progress().add_max(Common::island_work_count(object, 1));
}

void SupportDemandBridgeRemoval::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_perimeter_generation *ctx = plugin_ctx_as_post_perimeter_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    const BridgeRemovalConfig config = read_object_config(object);
    if (!config.dont_support_bridges) {
        progress().finish_run();
        return;
    }

    storage_handle *storage = run_ctx->plugin_storage;
    ClipperContext clipper(storage);
    for (uint32_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);

        const Layer layer = object.layer(layer_idx);
        const Layer lower_layer = object.layer(layer_idx - 1);
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);

            const LayerIsland island = layer.island(island_idx);
            remove_bridges_from_island_demand(run_ctx, storage, lower_layer, island, clipper);
            progress().increment();
        }
    }

    progress().finish_run();
}

void register_support_demand_bridge_removal_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SupportDemandBridgeRemoval::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Support::SupportDemandBridgeRemovalPlugin

#ifdef SUPPORT_DEMAND_BRIDGE_REMOVAL_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Support::SupportDemandBridgeRemovalPlugin::register_support_demand_bridge_removal_plugin(orch);
}
#endif // SUPPORT_DEMAND_BRIDGE_REMOVAL_PLUGIN_DLL
