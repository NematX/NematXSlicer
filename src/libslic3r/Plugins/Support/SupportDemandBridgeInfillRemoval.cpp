///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SupportDemandBridgeInfillRemoval.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <map>
#include <utility>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_infill.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_demand.h"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/Support/Common/SupportBridgeStrokes.hpp"

/*
Support-demand bridge infill removal
====================================

This plugin refines the support demand for the `dont_support_bridges` object
setting, which support.demand.bridge_removal defines. That plugin removes the
demand under the bridged perimeters and gap fills at STEP_POST_PERIMETER; this one
does the same for the bridge infill, which is generated later. It runs in
STEP_POST_INFILL, once the infill extrusions are final, and edits the islands'
support demand (editable until STEP_SUPPORT) before support generation; it does not
remove generated support extrusions. Its priority is 0. It reads the printed bridge
lines, so it must run after any STEP_POST_INFILL plugin that changes the geometry of
the bridge infill; today the other plugins of the step only change the flow
(PostFullFillVolume). Like the demand it edits, it acts whenever `dont_support_bridges`
is on: a print without support has no demand to edit.

What counts as a bridge
-----------------------

The plugin looks at the printed lines. It applies to the straight segments of the
bridge infill of every island (the infill lines over the void, role BRIDGE_INFILL:
infill, solid, bridge, external) the rule that support.demand.bridge_removal applies to
the perimeters: a segment piece whose middle is not over the lower layer and whose two
extended ends are over it is a bridge, and the area its line covers loses its demand.
Common/SupportBridgeStrokes.hpp holds the rule. Consecutive collinear segments of a line
are merged before the rule applies. The internal bridge infill (a bridge over the sparse
infill of the layer below) lies on the lower slices, so it is supported and never a
bridge; it is not examined. The width of a line is its own printed width (the attributes
of the line), which gives its stroke and the extension of its ends; the support probe
offset (half a nozzle) comes from the bridging solid infill flow of the regions of the island: the
bottom bridges are printed with the solid infill role and its extruder.

A cantilever or a floating surface keeps its demand, because its lines are not
anchored at both ends. The result depends on the printed direction of the lines: the
plugin pipeline sets no bridge direction (the infill generator receives no detected
angle), so the bridges are printed at the fill angle, and a line that runs from a
pillar to a free side is not a bridge.

Leftovers along the strokes
---------------------------

The strokes of the bridge infill and the ones that support.demand.bridge_removal subtracted
from the same demand at STEP_POST_PERIMETER do not meet exactly: they can leave slivers
narrower than a line, which the support family widens into a wall. After the subtraction, the
plugin opens the remaining demand by half the widest line width of the island (erosion then
dilation with mitered joins, so the convex corners stay) and takes the connected pieces of what
the opening removes. A piece is dropped only when it touches the strokes (within a few
SCALED_EPSILON: the residue of the subtraction touches them) and lies entirely within one line
width of the strokes. Every other piece, thin demand elsewhere included, stays.

Known limits
------------

The demand of the enforcers is not distinguished from the demand of the overhangs: an
enforced area under a bridge line loses its demand too (known limit of this version).

The normal call flow is:

    SupportDemandBridgeInfillRemoval::setup_run_impl()
    `-- reserve progress for the object layers after layer zero

    SupportDemandBridgeInfillRemoval::run_impl()
    `-- dont_support_bridges disabled?
        |-- yes: leave support demand unchanged
        `-- no: one parallel job per object layer after the first
            `-- remove_bridged_infill_from_layer()
                |-- a cache of the grown lower support, by probe offset
                `-- for each island with support demand
                    |-- bridge_infill_strokes()
                    |   `-- for each bridge infill leaf of the island:
                    |       Common::append_polyline_strokes()
                    |-- subtract the strokes from the island's demand
                    `-- remove the leftovers thinner than a line along the strokes

The jobs write the demand of different islands only, and read the data of their
own layer and of the layer below, so they do not share any mutable data.
*/

namespace slic3r_api { namespace Support { namespace SupportDemandBridgeInfillRemovalPlugin {

namespace {

const char *k_support_demand_bridge_infill_removal_id = "support.demand.bridge_infill_removal";
// support.demand.bridge_removal defines dont_support_bridges.
const char *k_dependencies[] = { "support.demand.bridge_removal", nullptr };
const raw_used_config_key k_used_config_keys[] = {
    { "dont_support_bridges", RAW_CO_BOOL, RAW_CONTAINER_TYPE_OBJECT, RAW_PRESET_TYPE_FFF_PRINT }
};

// The grown lower support of a layer job, by probe offset: the islands of a layer share it unless their nozzles differ.
using LowerSupportCache = std::map<double, StoredExPolygonCollection>;

bool read_dont_support_bridges(const Object &object)
{
    return object.config().get("dont_support_bridges").get_bool();
}

// Whether a printed line is a bridge over the void: bridge infill of a bottom surface, not the internal bridge over the sparse infill.
bool is_bridge_over_void(raw_extrusion_role role)
{
    constexpr raw_extrusion_role required = RAW_EXTRUSION_ROLE_INFILL | RAW_EXTRUSION_ROLE_BRIDGE | RAW_EXTRUSION_ROLE_EXTERNAL;
    return (role & required) == required;
}

// Add the strokes of the bridge lines of an extrusion tree. The attributes of a line (role, width) come from the nearest ancestor that has
// some. A line without a positive width takes the dimensions of the island flow. `max_line_width` grows to the widest stroke used (scaled).
void append_bridge_infill_strokes(storage_handle *storage,
                                  const ExPolygonCollection &lower_support,
                                  const Common::BridgeLineFlow &island_flow,
                                  const ClipperContext &clipper,
                                  const ExtrusionEntity &entity,
                                  const EPropertyAttributes *attributes,
                                  ClipperOperand &strokes,
                                  double &max_line_width)
{
    if (const EPropertyAttributes *local = entity.get(EPropertyAttributes::key))
        attributes = local;
    if (entity.has_polyline() && attributes != nullptr && is_bridge_over_void(attributes->extrusion_role())) {
        Common::BridgeLineFlow line_flow = island_flow;
        // EPropertyAttributes::width() is a setter: the printed width (mm) is the field of the C struct it wraps.
        const double printed_width = attributes->c_extrusion_property_attributes::width;
        if (printed_width > 0)
            line_flow = Common::bridge_line_flow_of_width(island_flow, coord_t(std::llround(scale_d(printed_width))));
        max_line_width = std::max(max_line_width, 2. * line_flow.half_width);
        Common::append_polyline_strokes(storage, lower_support, line_flow, clipper, entity, strokes);
    }
    for (const ExtrusionEntity child : entity.children())
        append_bridge_infill_strokes(storage, lower_support, island_flow, clipper, child, attributes, strokes, max_line_width);
}

// The areas covered by the bridge lines of the infill of an island, united; empty when it has none. `line_width` receives the widest
// stroke width used (scaled).
ClipperOperand bridge_infill_strokes(storage_handle *storage,
                                     const Layer &lower_layer,
                                     const LayerIsland &island,
                                     const ClipperContext &clipper,
                                     LowerSupportCache &lower_supports,
                                     double &line_width)
{
    ClipperOperand strokes = clipper.empty();
    line_width = 0.;
    // The bottom bridges are printed with the solid infill role and the solid infill extruder (DefaultInfillGenerator), so the nozzle of
    // the probe offset is the one of the bridging flow of the solid infill.
    const Common::BridgeLineFlow flow =
        Common::bridge_line_flow(island, raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL | RAW_EXTRUSION_ROLE_SOLID), true);
    auto lower_support = lower_supports.find(flow.support_probe_offset);
    if (lower_support == lower_supports.end())
        lower_support = lower_supports.emplace(flow.support_probe_offset,
                                               Common::grown_lower_support(clipper, lower_layer, flow)).first;

    for (uint32_t region_island_idx = 0; region_island_idx < island.region_island_count(); ++region_island_idx) {
        const LayerRegionIsland region_island = island.region_island(region_island_idx);
        if (!region_island.has_extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)))
            continue;
        const ExtrusionEntity root(region_island.extrusion(raw_extrusion_role(RAW_EXTRUSION_ROLE_INFILL)));
        append_bridge_infill_strokes(storage, lower_support->second, flow, clipper, root, nullptr, strokes, line_width);
    }
    return strokes.empty() ? std::move(strokes) : clipper_union(strokes);
}

// Remove from `remaining` the leftovers of its subtraction with the strokes: the connected pieces of what an opening by half the line
// width removes (the parts thinner than a line) that touch the strokes and lie entirely within one line width of them.
void remove_leftovers_along_strokes(const ClipperContext &clipper,
                                    const ClipperOperand &strokes,
                                    double line_width,
                                    ClipperOperand &remaining)
{
    // The residue of demand - strokes touches the strokes: a few SCALED_EPSILON absorb the rounding.
    const double tolerance = 4. * double(SCALED_EPSILON);
    // A classic opening: mitered joins keep the convex corners of the remaining demand.
    const ClipperOperand opened = clipper_offset2(remaining, -0.5 * line_width, 0.5 * line_width, CLIPPER_JOIN_MITER, 3.);
    const ClipperOperand thin_parts = clipper_diff(remaining, opened);
    if (thin_parts.empty())
        return;

    const ClipperOperand near_strokes = clipper_offset(strokes, line_width + tolerance, CLIPPER_JOIN_SQUARE, 0.);
    ClipperOperand removable = clipper.empty();
    const StoredExPolygonCollection pieces = thin_parts.to_expolygon_collection();
    for (ExPolygon piece : pieces) {
        const ClipperOperand shape = clipper(piece);
        const ClipperOperand reach = clipper_offset(shape, tolerance, CLIPPER_JOIN_SQUARE, 0.);
        if (clipper_intersection(reach, strokes).empty())
            continue;
        if (!clipper_diff(shape, near_strokes).empty())
            continue;
        removable.concat_replace(shape);
    }
    if (!removable.empty())
        remaining = clipper_diff(remaining, removable);
}

// Subtract the bridge strokes of the island from its demand, then the leftovers along them, and publish the result.
void remove_bridged_infill_from_island_demand(const plugin_run_context *run_ctx,
                                              storage_handle *storage,
                                              const Layer &lower_layer,
                                              const LayerIsland &island,
                                              const ClipperContext &clipper,
                                              LowerSupportCache &lower_supports)
{
    const ExPolygonCollection existing = island.support_demand();
    if (existing.empty())
        return;

    double line_width = 0.;
    ClipperOperand strokes = bridge_infill_strokes(storage, lower_layer, island, clipper, lower_supports, line_width);
    if (strokes.empty())
        return;

    ClipperOperand remaining = clipper_diff_with_safety_offset(clipper(existing), strokes);
    remove_leftovers_along_strokes(clipper, strokes, line_width, remaining);
    StoredExPolygonCollection cleaned_demand = remaining.to_expolygon_collection();
    expolygons_move(support_demand_get_island_mutable(run_ctx, island.handle()), cleaned_demand.mutable_handle());
}

// One parallel job: the islands of one object layer. The scratch storage holds the temporary geometry of the job; the new demand is moved into the islands.
void remove_bridged_infill_from_layer(uint32_t layer_idx,
                                      storage_handle *scratch_storage,
                                      const plugin_run_context *run_ctx,
                                      const Object *object)
{
    ClipperContext clipper(scratch_storage);
    LowerSupportCache lower_supports;
    const Layer layer = object->layer(layer_idx);
    const Layer lower_layer = object->layer(layer_idx - 1);
    for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
        throw_if_cancelled(run_ctx);
        remove_bridged_infill_from_island_demand(run_ctx, scratch_storage, lower_layer, layer.island(island_idx), clipper,
                                                 lower_supports);
    }
}

} // namespace

SupportDemandBridgeInfillRemoval &
SupportDemandBridgeInfillRemoval::instance(orchestrator_handle *orch)
{
    static SupportDemandBridgeInfillRemoval s_instance(orch);
    return s_instance;
}

const char *SupportDemandBridgeInfillRemoval::id_impl() const noexcept
{
    return k_support_demand_bridge_infill_removal_id;
}

const char *SupportDemandBridgeInfillRemoval::name_impl() const noexcept
{
    return "Support demand bridge infill removal";
}

const char *SupportDemandBridgeInfillRemoval::description_impl() const noexcept
{
    return "Remove support demand under bridge infill lines when bridge support is disabled.";
}

slicing_step_t SupportDemandBridgeInfillRemoval::step_impl() const noexcept
{
    return STEP_POST_INFILL;
}

const char *const *SupportDemandBridgeInfillRemoval::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t SupportDemandBridgeInfillRemoval::priority_impl() const noexcept
{
    return 0;
}

int32_t SupportDemandBridgeInfillRemoval::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t SupportDemandBridgeInfillRemoval::defined_config_keys(const char **) const noexcept
{
    return 0;
}

const char *SupportDemandBridgeInfillRemoval::progress_message_format_impl() const noexcept
{
    return "Support demand bridge infill removal: %u / %u layers";
}

// Borrows a post-infill payload and reserves progress for the layers above layer zero when bridge infill removal is enabled; missing payload does nothing and cancellation propagates.
void SupportDemandBridgeInfillRemoval::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (read_dont_support_bridges(object) && object.layer_count() > 1)
        progress().add_max(object.layer_count() - 1);
}

// Runs one parallel job per object layer above the first when dont_support_bridges is on; otherwise it leaves the demand untouched.
// Cancellation and the first failure of a job propagate after the jobs end; a missing payload does nothing.
void SupportDemandBridgeInfillRemoval::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_infill_generation *ctx = plugin_ctx_as_post_infill_generation(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    const Object object(ctx->object);
    if (!read_dont_support_bridges(object) || object.layer_count() <= 1) {
        progress().finish_run();
        return;
    }

    parallel_for_storage_with_progress(1,
                                       object.layer_count(),
                                       run_ctx,
                                       &progress(),
                                       remove_bridged_infill_from_layer,
                                       run_ctx,
                                       &object);
    progress().finish_run();
}

// Registers the process-wide instance with the orchestrator (the loader and the tests call it).
void register_support_demand_bridge_infill_removal_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, SupportDemandBridgeInfillRemoval::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Support::SupportDemandBridgeInfillRemovalPlugin

#ifdef SUPPORT_DEMAND_BRIDGE_INFILL_REMOVAL_PLUGIN_DLL
extern "C" void register_plugin(orchestrator_handle *orch)
{
    slic3r_api::Support::SupportDemandBridgeInfillRemovalPlugin::register_support_demand_bridge_infill_removal_plugin(orch);
}
#endif // SUPPORT_DEMAND_BRIDGE_INFILL_REMOVAL_PLUGIN_DLL
