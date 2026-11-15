///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CreateRetractionLift.cpp
========================

Implements the compact-scope retraction-lift provider declared in
CreateRetractionLift.hpp: the print_retract_lift option definition and its
activation rule, the two passes that turn the requested lift into wipe and
travel geometry, and the registration entry point. The header describes the
pass contract and the prerequisites on transition scopes.
*/

#include "CreateRetractionLift.hpp"
#include "libslic3r/Plugins/Common/SourceLayerIndices.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ExtrusionScopeHelpers.hpp"
#include "TravelPhaseHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

/*
Compact-scope retraction-lift implementation
============================================

The plugin has two parallel passes. setup_run() applies one progressive lift to
all outgoing wipe leaves and publishes one scalar description of the layer's final
scope. After the host barrier, run() streams local scope pairs and shapes each
target travel. Only the first scope of a layer may consume the preceding
layer's scalar summary.

    setup_run_impl: raise each scope's outgoing wipe and publish its final Z
        prepare_scope_wipe_lift: measure all wipe leaves before computing the ramp
            prepare_wipe_lifts: collect Wipe leaves in execution order
            lift_wipe: insert the ramp start and keep a running maximum of path Z plus ramp across leaves
    run_impl: shape incoming travel using the preceding scope's final Z

Owner views supplied by PrintingEntityPropertyTraversal provide the tool,
region and object instance. Printable endpoint state is recovered directly
from scope.content(), so the private scope property remains a one-byte set of
structural flags.
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CreateRetractionLiftPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_create_retraction_lift_plugin(orchestrator_handle *orchestrator);

namespace {

using ScopeEntity = PrintingEntity<PrintingExtrusionScopeProperty>;
namespace TP = TravelPhase;
using TP::segment_planar_length;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    nullptr
};

const RegionSettings::OptionKeyGroup k_lift_region_keys = {
    "print_retract_lift"
};

const char *const k_defined_config_keys[] = {
    "print_retract_lift"
};

void initialize_create_retraction_lift_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definition owned by this module. It fixes the option type, override
    scope, default and GUI metadata; the defined key is published in the print
    presets, so profiles read and write it by name like any built-in setting.
    Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "print_retract_lift";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Z-lift override";
    def.tooltip = "Set the new lift-z value for this override. 0 will disable the z-lift. -& to disable. May only work on Height range modifiers.";
    def.sidetext = "mm";
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!0";
    orchestrator_create_option_def(orchestrator, &def);
}

/* Properties inherited while locating printable or process geometry. */
struct EffectiveState
{
    coord_t z_offset = 0;
    std::optional<EPropertyAttributes> attributes;
    std::optional<EPropertyModifier> modifier;
};

/* Last printable endpoint and the source policy attached to that endpoint. */
struct SourceEndpoint
{
    MutableExtrusionEntity leaf;
    c_point point{};
    c_point interior_point{};
    coord_t absolute_z = 0;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    EPropertyModifier modifier{};
    bool has_interior_point = false;
};

/* Scalar source facts needed after the owning layer is no longer accessible. */
struct SourceLiftState
{
    bool valid = false;
    bool outgoing_transition = false;
    bool outgoing_toolchange = false;
    bool retracted = false;
    bool disable_lift = false;
    bool source_is_top = false;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    double requested_lift_mm = 0.0;
    coord_t unlifted_end_z = 0;
    coord_t lifted_end_z = 0;
};

/* One independently published summary for a final ordered layer. */
struct LayerLiftSummary
{
    bool prepared = false;
    SourceLiftState last;
};

/* Built-in provider which materializes progressive wipe and travel Z lift. */
class CreateRetractionLift final : public PluginBase
{
public:
    static CreateRetractionLift &instance(orchestrator_handle *orchestrator);
    explicit CreateRetractionLift(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    const char *progress_message_format_impl() const noexcept override;

    /*
    Prepare the shared indexing used by the two per-layer passes.

    This method runs once and must stay inexpensive: it assigns one stable
    summary slot to every PrintingLayerGroup, but does not inspect or modify
    extrusion trees.
    */
    void setup_impl(const plugin_run_context *run_ctx,
                    uint32_t run_count) const override;

    /*
    Perform the first, parallel pass for one PrintingLayerGroup.

    The pass applies progressive wipe lift only to Wipe geometry owned by the
    current layer, then publishes a scalar description of that layer's final
    scope. It never reads another layer's summary or extrusion tree. The host
    completes every setup_run() before invoking any run().
    */
    void setup_run_impl(const plugin_run_context *run_ctx) const override;

    /*
    Perform the second, parallel pass for one PrintingLayerGroup.

    After the setup-run barrier, this pass may read immutable summaries from
    earlier layers. It applies lift to travel slots owned by the current layer,
    using direct scope pairs locally and the preceding summary only for the
    first local scope.
    */
    void run_impl(const plugin_run_context *run_ctx) const override;

    /* Convert stable group/layer indexes into the summary array. */
    size_t flat_layer_index(uint32_t group_idx, uint32_t layer_idx) const;

    /* Return the final scope summary of the nearest preceding non-empty layer. */
    const SourceLiftState *previous_layer_source(
        size_t flat_idx, const plugin_run_context *run_ctx) const;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
    mutable bool m_setup_valid = false;
    mutable std::vector<std::vector<size_t>> m_flat_indices;
    mutable std::vector<LayerLiftSummary> m_layers;
    mutable SourceLayerIndices m_source_indices;
};

/* Add scaled Z values and reject overflow before editing geometry. */
coord_t checked_add_z(coord_t value, coord_t offset);

/* Resolve direct lift-relevant properties over an inherited state. */
EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent);

/* Convert the traversal's autonomous snapshot to this plugin's local state. */
EffectiveState traversal_state(const ExtrusionPropertyState &properties);

/* Describe the properties which every lift scope traversal must inherit. */
ExtrusionPropertyStateDefinition lift_state_definition();

/* Find the final printable leaf, optionally starting with entity resolved. */
bool find_last_printable_geometry(MutableExtrusionEntity entity,
                                  coord_t print_z,
                                  const EffectiveState &parent,
                                  SourceEndpoint &endpoint,
                                  bool entity_already_resolved,
                                  const plugin_run_context *run_ctx);

/* Find whether a phase contains a semantic Retract request. */
bool contains_semantic_retract(
    const ExtrusionEntity &entity,
    std::optional<EPropertyAttributes> inherited_attributes,
    const plugin_run_context *run_ctx);

/* Resolve the final absolute Z reached by any geometry in one phase. */
std::optional<coord_t> last_geometric_z(MutableExtrusionEntity entity,
                                        coord_t print_z,
                                        const EffectiveState &parent,
                                        const plugin_run_context *run_ctx);

/* Resolve the source-region lift override or the extruder fallback. */
double source_lift(const ScopeEntity &source,
                   const PrintingPlan &plan,
                   const SourceEndpoint &endpoint,
                   const Config &print_config,
                   const plugin_run_context *run_ctx);

/* Describe one outgoing scope without retaining owner or entity handles. */
SourceLiftState describe_source(
    const ScopeEntity &source,
    const PrintingPlan &plan,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx);

/* Lift added along a whole travel. It rises linearly from start to top until rise_end,
   stays at top, then falls linearly to zero from fall_start to length. Distances are
   planar and accumulated over the ordered travel leaves. */
struct LiftProfile
{
    coord_t start = 0;
    coord_t top = 0;
    distf_t rise_end = 0.0;
    distf_t fall_start = 0.0;
    distf_t length = 0.0;
};

/* Return the lift requested by a profile at one planar distance along the travel. */
long double lift_at(const LiftProfile &profile, distf_t distance);

/* Sum planar segment lengths of the first end leaves in the order used by apply_lift_profile. */
distf_t planar_length_before(const std::vector<TP::TravelLeaf> &leaves, size_t end);

/* Make distance, measured with host leaf lengths, a leaf boundary of the ordered travel.
   The leaf containing it is split into two ordered children which inherit its properties;
   a junction within SCALED_EPSILON is reused. Returns the index of the first leaf after the
   boundary; failed edits throw. */
size_t split_travel_at(std::vector<TP::TravelLeaf> &leaves, distf_t distance,
                       const plugin_run_context *run_ctx);

/* Add the profile's lift to every point of the ordered travel leaves; overflow or a failed edit throws. */
void apply_lift_profile(const std::vector<TP::TravelLeaf> &leaves, const LiftProfile &profile,
                        const plugin_run_context *run_ctx);

/* Materialize a vertical or ramped lift on the ordered leaves of one reserved travel. */
void lift_travel(std::vector<TP::TravelLeaf> leaves,
                 coord_t lift,
                 coord_t inherited_start_lift,
                 bool ramping,
                 double slope_degrees,
                 const plugin_run_context *run_ctx);

/* Insert the ramp start without changing leaf properties, then clamp path Z plus ramp to the preceding wipe Z.
   traversed is the shared planar distance in/out and is advanced here; previous carries the last Z across leaves. Failed edits or overflow throw. */
void lift_wipe(MutableExtrusionEntity entity,
               coord_t lift,
               distf_t lift_length,
               distf_t total,
               distf_t &traversed,
               std::optional<coord_t> &previous,
               const plugin_run_context *run_ctx);

/* Append borrowed Wipe leaves in execution order, resolving inherited roles without changing geometry. */
void prepare_wipe_lifts(MutableExtrusionEntity entity,
                        std::vector<MutableExtrusionEntity> &wipes,
                        std::optional<EPropertyAttributes> inherited_attributes,
                        const plugin_run_context *run_ctx);

/* Measure the outgoing wipe and apply a capped shared ramp with a running maximum that keeps Z non-decreasing across leaves; invalid settings or failed edits throw. */
void prepare_scope_wipe_lift(
    const ScopeEntity &scope,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx);

/* Collect the continuous Travel leaves of a reserved travel and return its absolute start Z.
   A travel without geometry, discontinuous leaves or a non-Travel role throws. */
coord_t travel_start_z(
    const ScopeEntity &target,
    const ExtrusionScope::OrderedExtrusionScope &scope,
    std::vector<TP::TravelLeaf> &leaves,
    const plugin_run_context *run_ctx);

/*
Apply one source scope's lift policy to the following scope's travel.

The function first verifies that the source and target describe the same
transition. A lift is considered only when the source contains a semantic
retraction and the target owns a materialized travel. It then applies the
source role, modifier, toolchange, layer-height, minimum-travel and printer-Z
limits, combines the requested lift with any Z already left by a lifted wipe,
and writes the resulting vertical or ramped profile into target.travel.

The source state and printable content are read-only. Only the target travel
geometry is modified, and its final Z remains the original destination Z.
source_layer_index is the minimum rank in the target copies' merged object/auxiliary
source sequences; only rank zero enables the first-layer override.
*/
void apply_transition_lift(
    const SourceLiftState &source,
    const ScopeEntity &target,
    uint32_t source_layer_index,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx);

coord_t checked_add_z(const coord_t value, const coord_t offset)
{
    if ((offset > 0 && value > (std::numeric_limits<coord_t>::max)() - offset) ||
        (offset < 0 && value < (std::numeric_limits<coord_t>::min)() - offset))
        throw std::overflow_error("A retraction lift exceeds the coord_t range.");
    return value + offset;
}

EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent)
{
    EffectiveState state = parent;
    if (const EPropertyZOffset *z_offset = entity.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        state.attributes = *attributes;
    if (const EPropertyModifier *modifier = entity.get(EPropertyModifier::key))
        state.modifier = *modifier;
    return state;
}

EffectiveState traversal_state(const ExtrusionPropertyState &properties)
{
    EffectiveState state;
    if (const EPropertyZOffset *z_offset = properties.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes =
            properties.get(EPropertyAttributes::key))
        state.attributes = *attributes;
    if (const EPropertyModifier *modifier = properties.get(EPropertyModifier::key))
        state.modifier = *modifier;
    return state;
}

ExtrusionPropertyStateDefinition lift_state_definition()
{
    ExtrusionPropertyStateDefinition definition;
    definition.track(EPropertyZOffset::key);
    definition.track(EPropertyAttributes::key);
    definition.track(EPropertyModifier::key);
    return definition;
}

// Copy the last printable endpoint from borrowed children, including deposits without interior probes; missing attributes, process roles, cancellation or Z overflow throw without mutation.
bool find_last_printable_geometry(
    MutableExtrusionEntity entity,
    const coord_t print_z,
    const EffectiveState &parent,
    SourceEndpoint &endpoint,
    const bool entity_already_resolved,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);
    ExtrusionScope::LeafEndpoints endpoints;
    if (ExtrusionScope::leaf_endpoints(entity.readonly(), endpoints)) {
        if (!state.attributes)
            throw std::runtime_error(
                "A lift source contains geometry without extrusion attributes.");
        const raw_extrusion_role role = state.attributes->extrusion_role();
        if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
            RAW_EXTRUSION_ROLE_IS_RETRACT(role) ||
            RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
            throw std::runtime_error("Printable scope content contains process geometry.");

        endpoint.leaf = entity;
        endpoint.point = endpoints.last;
        endpoint.absolute_z = checked_add_z(
            checked_add_z(print_z, state.z_offset), endpoints.last_z);
        endpoint.role = role;
        endpoint.modifier = state.modifier.value_or(EPropertyModifier{});
        const distf_t distance = std::min(
            distf_t(SCALED_EPSILON), entity.local_length() / 2.0);
        if (distance > 0.0) {
            endpoint.interior_point = entity.point_from_end(distance);
            endpoint.has_interior_point = true;
        }
        return true;
    }

    for (uint32_t child_idx = entity.child_count(); child_idx > 0; --child_idx) {
        throw_if_cancelled(run_ctx);
        if (find_last_printable_geometry(entity.child_mutable(child_idx - 1),
                                         print_z, state, endpoint, false, run_ctx))
            return true;
    }
    return false;
}

bool contains_semantic_retract(
    const ExtrusionEntity &entity,
    std::optional<EPropertyAttributes> inherited_attributes,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (const EPropertyAttributes *direct = entity.get(EPropertyAttributes::key))
        inherited_attributes = *direct;
    const EPropertyExtrusionAxis *axis = entity.get(EPropertyExtrusionAxis::key);
    if (inherited_attributes && axis != nullptr &&
        RAW_EXTRUSION_ROLE_IS_RETRACT(inherited_attributes->extrusion_role()) &&
        axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO)
        return true;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        if (contains_semantic_retract(
                entity.child(child_idx), inherited_attributes, run_ctx))
            return true;
    }
    return false;
}

// Scan borrowed children for the final segmented or deposit Z, returning no value without geometry; cancellation and checked Z overflow throw without tree mutation.
std::optional<coord_t> last_geometric_z(
    MutableExtrusionEntity entity,
    const coord_t print_z,
    const EffectiveState &parent,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveState state = resolved_state(entity.readonly(), parent);
    std::optional<coord_t> result;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        const std::optional<coord_t> child_result = last_geometric_z(
            entity.child_mutable(child_idx), print_z, state, run_ctx);
        if (child_result)
            result = child_result;
    }
    ExtrusionScope::LeafEndpoints endpoints;
    if (ExtrusionScope::leaf_endpoints(entity.readonly(), endpoints)) {
        result = checked_add_z(
            checked_add_z(print_z, state.z_offset), endpoints.last_z);
    }
    return result;
}

// Resolve borrowed source-region lift settings, falling back to the first region for unmatched deposits; disabled settings use printer lift, and cancellation or unresolved ordinary geometry throws.
double source_lift(const ScopeEntity &source,
                   const PrintingPlan &plan,
                   const SourceEndpoint &endpoint,
                   const Config &print_config,
                   const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const extruder_index_t extruder_id = source.tool_group.extruder_id();
    const double printer_lift = print_config.vector_float_or_default(
        "retract_lift", extruder_id, 0.0);
    const LayerRegionIsland region_island =
        source.printing_extrusion.region_island();
    if (!region_island.valid() || region_island.region_count() == 0)
        return printer_lift;

    const c_point shift = plan.object_instance_shift(source.printing_extrusion);
    c_point lookup_point{
        endpoint.point.x - shift.x,
        endpoint.point.y - shift.y
    };
    RegionSettingsPointResult lookup = RegionSettings::lookup_at_point(
        region_island, k_lift_region_keys, lookup_point);
    throw_if_cancelled(run_ctx);
    if (!lookup.found() && endpoint.has_interior_point) {
        lookup_point = c_point{
            endpoint.interior_point.x - shift.x,
            endpoint.interior_point.y - shift.y
        };
        lookup = RegionSettings::lookup_at_point(
            region_island, k_lift_region_keys, lookup_point);
        throw_if_cancelled(run_ctx);
    }
    if (!lookup.found() && endpoint.leaf.is_deposit()) {
        const Config config = region_island.region(0).print_region().config();
        return config.float_or_default("print_retract_lift", printer_lift);
    }
    if (!lookup.found())
        throw std::runtime_error(
            "The lift source point does not select an unambiguous region.");
    return lookup.value.is_enabled("print_retract_lift") ?
        lookup.value.get_float("print_retract_lift") : printer_lift;
}

SourceLiftState describe_source(
    const ScopeEntity &source,
    const PrintingPlan &plan,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const ExtrusionScope::OrderedExtrusionScope scope(source.entity, key);
    SourceLiftState result;
    result.valid = true;
    result.outgoing_transition = scope.has_outgoing_transition();
    result.outgoing_toolchange = scope.has_outgoing_toolchange();
    result.extruder_id = source.tool_group.extruder_id();
    if (!result.outgoing_transition)
        return result;

    const EffectiveState scope_state = traversal_state(source.effective_properties);
    MutableExtrusionEntity content = scope.content();
    SourceEndpoint endpoint;
    if (!find_last_printable_geometry(content, source.layer_group.print_z(),
                                      scope_state, endpoint,
                                      content.handle() == scope.root().handle(),
                                      run_ctx))
        throw std::runtime_error("A lift source scope has no printable geometry.");

    result.unlifted_end_z = endpoint.absolute_z;
    result.lifted_end_z = endpoint.absolute_z;
    result.disable_lift = endpoint.modifier.disable_lift != 0;
    result.source_is_top =
        endpoint.role == RAW_EXTRUSION_ROLE_TOP_SOLID_INFILL ||
        RAW_EXTRUSION_ROLE_HAS(endpoint.role, RAW_EXTRUSION_ROLE_IRONING);
    result.retracted = contains_semantic_retract(
        scope.after().readonly(), std::nullopt, run_ctx);
    if (!result.retracted)
        return result;

    result.requested_lift_mm = source_lift(
        source, plan, endpoint, print_config, run_ctx);
    if (!std::isfinite(result.requested_lift_mm) ||
        result.requested_lift_mm < 0.0)
        throw std::runtime_error("Lift settings produced an invalid Z request.");
    const std::optional<coord_t> wipe_end = last_geometric_z(
        scope.after(), source.layer_group.print_z(), scope_state, run_ctx);
    if (wipe_end)
        result.lifted_end_z = *wipe_end;
    return result;
}

long double lift_at(const LiftProfile &profile, const distf_t distance)
{
    if (distance <= profile.rise_end) {
        const double factor = profile.rise_end <= distf_t(SCALED_EPSILON) ? 1.0 :
            std::clamp(double(distance / profile.rise_end), 0.0, 1.0);
        return static_cast<long double>(profile.start) +
            static_cast<long double>(profile.top - profile.start) * factor;
    }
    if (distance >= profile.fall_start) {
        const distf_t fall = profile.length - profile.fall_start;
        const double factor = fall <= distf_t(SCALED_EPSILON) ? 0.0 :
            std::clamp(double((profile.length - distance) / fall), 0.0, 1.0);
        return static_cast<long double>(profile.top) * factor;
    }
    return static_cast<long double>(profile.top);
}

distf_t planar_length_before(const std::vector<TP::TravelLeaf> &leaves, const size_t end)
{
    // One running sum, as in apply_lift_profile, so a boundary equals the distance of its junction point.
    distf_t traversed = 0.0;
    for (size_t leaf_idx = 0; leaf_idx < end; ++leaf_idx) {
        const MutableExtrusionEntity &entity = leaves[leaf_idx].entity;
        for (uint32_t segment_idx = 0; segment_idx < entity.segment_count(); ++segment_idx)
            traversed += segment_planar_length(entity.segment(segment_idx));
    }
    return traversed;
}

size_t split_travel_at(std::vector<TP::TravelLeaf> &leaves,
                       const distf_t distance,
                       const plugin_run_context *run_ctx)
{
    distf_t before = 0.0;
    for (size_t leaf_idx = 0; leaf_idx < leaves.size(); ++leaf_idx) {
        throw_if_cancelled(run_ctx);
        if (distance <= before + distf_t(SCALED_EPSILON))
            return leaf_idx;
        MutableExtrusionEntity leaf = leaves[leaf_idx].entity;
        const distf_t after = before + leaf.local_length();
        if (distance < after - distf_t(SCALED_EPSILON)) {
            MutableExtrusionEntity first = leaf.emplace_ordered_leaf(
                OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
            if (!first.valid() || leaf.child_count() != 2)
                throw std::runtime_error("Unable to create ordered ramped-travel phases.");
            MutableExtrusionEntity second = leaf.child_mutable(1);
            throw_if_cancelled(run_ctx);
            if (extrusion_polyline_split_at_distance(
                    second.handle(), distance - before,
                    first.mutable_handle(), second.mutable_handle()) == 0)
                throw std::runtime_error("Unable to split a travel for ramped lift.");
            TP::TravelLeaf tail = leaves[leaf_idx];
            tail.entity = second;
            leaves[leaf_idx].entity = first;
            leaves.insert(leaves.begin() + std::ptrdiff_t(leaf_idx + 1), tail);
            return leaf_idx + 1;
        }
        before = after;
    }
    return leaves.size();
}

void apply_lift_profile(const std::vector<TP::TravelLeaf> &leaves,
                        const LiftProfile &profile,
                        const plugin_run_context *run_ctx)
{
    distf_t traversed = 0.0;
    for (const TP::TravelLeaf &leaf : leaves) {
        MutableExtrusionEntity entity = leaf.entity;
        // The first point of a leaf repeats the previous junction, so the running distance continues unchanged.
        for (uint32_t point_idx = 0; point_idx < entity.point_count(); ++point_idx) {
            throw_if_cancelled(run_ctx);
            if (point_idx > 0)
                traversed += segment_planar_length(entity.segment(point_idx - 1));
            const long double value =
                static_cast<long double>(entity.z_offset(point_idx)) + lift_at(profile, traversed);
            if (value < static_cast<long double>((std::numeric_limits<coord_t>::min)()) ||
                value > static_cast<long double>((std::numeric_limits<coord_t>::max)()))
                throw std::overflow_error("A ramped travel lift exceeds the coord_t range.");
            throw_if_cancelled(run_ctx);
            if (!entity.set_z_offset(point_idx, coord_t(std::llround(value))))
                throw std::runtime_error("Unable to apply a Z ramp to a travel phase.");
        }
    }
}

void lift_travel(
    std::vector<TP::TravelLeaf> leaves,
    const coord_t lift,
    const coord_t inherited_start_lift,
    const bool ramping,
    const double slope_degrees,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if ((lift <= 0 && inherited_start_lift <= 0) || leaves.empty())
        return;

    const coord_t effective_lift = std::max(lift, inherited_start_lift);
    if (!ramping || slope_degrees >= 90.0) {
        for (size_t leaf_idx = 0; leaf_idx < leaves.size(); ++leaf_idx) {
            throw_if_cancelled(run_ctx);
            MutableExtrusionEntity travel = leaves[leaf_idx].entity;
            std::vector<c_extrusion_segment> lifted = travel.segments();
            if (lifted.empty())
                continue;

            // Explicit vertical segments at both ends of the travel expose the complete lift in the plan.
            c_extrusion_segment raise{};
            raise.point_a = lifted.front().point_a;
            raise.point_b = lifted.front().point_a;
            raise.z_offset_a = checked_add_z(
                lifted.front().z_offset_a, inherited_start_lift);
            raise.z_offset_b = checked_add_z(
                lifted.front().z_offset_a, effective_lift);
            raise.orientation = RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN;

            c_extrusion_segment lower{};
            lower.point_a = lifted.back().point_b;
            lower.point_b = lifted.back().point_b;
            lower.z_offset_a = checked_add_z(
                lifted.back().z_offset_b, effective_lift);
            lower.z_offset_b = lifted.back().z_offset_b;
            lower.orientation = RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN;

            for (c_extrusion_segment &segment : lifted) {
                throw_if_cancelled(run_ctx);
                segment.z_offset_a = checked_add_z(segment.z_offset_a, effective_lift);
                segment.z_offset_b = checked_add_z(segment.z_offset_b, effective_lift);
            }
            throw_if_cancelled(run_ctx);
            if (leaf_idx == 0)
                lifted.insert(lifted.begin(), raise);
            if (leaf_idx + 1 == leaves.size())
                lifted.push_back(lower);
            throw_if_cancelled(run_ctx);
            if (!travel.set_segments(lifted))
                throw std::runtime_error("Unable to materialize a complete travel lift.");
        }
        return;
    }

    distf_t length = 0.0;
    for (const TP::TravelLeaf &leaf : leaves)
        length += leaf.entity.local_length();
    if (length <= distf_t(SCALED_EPSILON))
        return;

    // A zero slope uses a symmetric triangle. A positive angle limits each
    // ramp and leaves a plateau when the travel is long enough.
    distf_t ramp_length = length * 0.5;
    if (slope_degrees > 0.0) {
        const double radians = slope_degrees * std::acos(-1.0) / 180.0;
        const double requested_mm =
            unscaled(effective_lift) / std::tan(radians);
        if (std::isfinite(requested_mm) && requested_mm > 0.0)
            ramp_length = std::min(length * 0.5, scale_d(requested_mm));
    }
    ramp_length = std::clamp<distf_t>(
        ramp_length, distf_t(SCALED_EPSILON), length * 0.5);

    // Ramp ends become leaf boundaries, so they are exact points of the lifted path.
    const size_t rise_end = split_travel_at(leaves, ramp_length, run_ctx);
    const size_t fall_start = length - 2.0 * ramp_length > distf_t(SCALED_EPSILON) ?
        split_travel_at(leaves, length - ramp_length, run_ctx) : rise_end;
    LiftProfile profile;
    profile.start = inherited_start_lift;
    profile.top = effective_lift;
    profile.rise_end = planar_length_before(leaves, rise_end);
    profile.fall_start = planar_length_before(leaves, fall_start);
    profile.length = planar_length_before(leaves, leaves.size());
    apply_lift_profile(leaves, profile, run_ctx);
}

void lift_wipe(MutableExtrusionEntity entity,
               const coord_t lift,
               const distf_t lift_length,
               const distf_t total,
               distf_t &traversed,
               std::optional<coord_t> &previous,
               const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (lift <= 0 || lift_length <= 0.0 || entity.point_count() < 2)
        return;
    const distf_t start = std::max<distf_t>(0.0, total - lift_length);
    if (start - traversed > distf_t(SCALED_EPSILON) &&
        traversed + entity.local_length() - start > distf_t(SCALED_EPSILON)) {
        // Rejoin the split polylines on the same leaf so its retraction target still covers the entire motion.
        StoredExtrusionEntity before(run_ctx->plugin_storage);
        StoredExtrusionEntity after(run_ctx->plugin_storage);
        if (extrusion_polyline_split_at_distance(entity.handle(), start - traversed,
                before.mutable_handle(), after.mutable_handle()) == 0)
            throw std::runtime_error("Unable to insert the wipe lift ramp start.");
        std::vector<c_extrusion_segment> segments = before.segments();
        const std::vector<c_extrusion_segment> tail = after.segments();
        segments.insert(segments.end(), tail.begin(), tail.end());
        if (!entity.set_segments(segments))
            throw std::runtime_error("Unable to preserve the split wipe geometry.");
    }
    for (uint32_t point_idx = 0; point_idx < entity.point_count(); ++point_idx) {
        throw_if_cancelled(run_ctx);
        if (point_idx > 0)
            traversed += segment_planar_length(entity.segment(point_idx - 1));
        const double factor = traversed <= start ? 0.0 :
            std::clamp(double((traversed - start) /
                              std::max<distf_t>(lift_length, 1.0)),
                       0.0, 1.0);
        const coord_t added = coord_t(std::llround(double(lift) * factor));
        coord_t raised = checked_add_z(entity.z_offset(point_idx), added);
        if (previous)
            raised = std::max(*previous, raised);
        throw_if_cancelled(run_ctx);
        if (!entity.set_z_offset(point_idx, raised))
            throw std::runtime_error("Unable to apply progressive wipe lift.");
        previous = raised;
    }
}

void prepare_wipe_lifts(
    MutableExtrusionEntity entity,
    std::vector<MutableExtrusionEntity> &wipes,
    std::optional<EPropertyAttributes> inherited_attributes,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!entity.valid())
        return;
    if (const EPropertyAttributes *direct = entity.get(EPropertyAttributes::key))
        inherited_attributes = *direct;

    if (entity.segment_count() != 0 && inherited_attributes &&
        RAW_EXTRUSION_ROLE_IS_WIPE(inherited_attributes->extrusion_role()))
        wipes.push_back(entity);

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        prepare_wipe_lifts(entity.child_mutable(child_idx), wipes, inherited_attributes,
                           run_ctx);
    }
}

void prepare_scope_wipe_lift(
    const ScopeEntity &scope_entity,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const ExtrusionScope::OrderedExtrusionScope scope(scope_entity.entity, key);
    if (!scope.has_outgoing_transition())
        return;
    coord_t layer_height = scale_i(0.2);
    const LayerRegionIsland region_island =
        scope_entity.printing_extrusion.region_island();
    if (region_island.valid() && region_island.region_count() != 0)
        layer_height = region_island.region(0).layer().height();
    std::vector<MutableExtrusionEntity> wipes;
    prepare_wipe_lifts(scope.after(), wipes, std::nullopt, run_ctx);
    if (wipes.empty())
        return;
    distf_t total = 0.0;
    for (const MutableExtrusionEntity &wipe : wipes)
        total += wipe.local_length();
    const extruder_index_t extruder_id = scope_entity.tool_group.extruder_id();
    const double lift_mm = print_config.vector_effective_float_or_percent_or_default(
        "wipe_lift", extruder_id, unscaled(layer_height), 0.0);
    const double length_mm = print_config.vector_effective_float_or_percent_or_default(
        "wipe_lift_length", extruder_id, unscaled(total), 0.0);
    if (!std::isfinite(lift_mm) || lift_mm < 0.0 ||
        !std::isfinite(length_mm) || length_mm < 0.0)
        throw std::runtime_error("Wipe lift settings produced an invalid value.");
    const distf_t lift_length = std::min(total, scale_d(length_mm));
    distf_t traversed = 0.0;
    std::optional<coord_t> previous;
    for (MutableExtrusionEntity &wipe : wipes)
        lift_wipe(wipe, scale_i(lift_mm), lift_length, total, traversed, previous, run_ctx);
}

coord_t travel_start_z(
    const ScopeEntity &target,
    const ExtrusionScope::OrderedExtrusionScope &scope,
    std::vector<TP::TravelLeaf> &leaves,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!TP::collect_travel_leaves(scope.travel(),
            traversal_state(target.effective_properties).z_offset, leaves) ||
        leaves.empty())
        throw std::runtime_error(
            "A lifted transition requires one continuous materialized travel.");
    for (const TP::TravelLeaf &leaf : leaves)
        if (!RAW_EXTRUSION_ROLE_IS_TRAVEL(leaf.role))
            throw std::runtime_error("A lifted travel has an incompatible role.");

    const c_extrusion_segment first = leaves.front().entity.segment(0);
    return checked_add_z(
        checked_add_z(target.layer_group.print_z(), leaves.front().z_offset),
        first.z_offset_a);
}

void apply_transition_lift(
    const SourceLiftState &source,
    const ScopeEntity &target,
    const uint32_t source_layer_index,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const ExtrusionScope::OrderedExtrusionScope target_scope(target.entity, key);
    if (source.outgoing_transition != target_scope.has_incoming_transition())
        throw std::runtime_error(
            "Adjacent scopes disagree about their lift transition.");
    if (!target_scope.has_incoming_transition())
        return;
    if (source.outgoing_toolchange != target_scope.has_incoming_toolchange())
        throw std::runtime_error(
            "Adjacent scopes disagree about their lift tool change.");

    MutableExtrusionEntity travel = target_scope.travel();
    if (!source.retracted || !travel.valid() ||
        (travel.segment_count() == 0 && travel.child_count() == 0))
        return;

    std::vector<TP::TravelLeaf> leaves;
    const coord_t initial_travel_z = travel_start_z(target, target_scope, leaves, run_ctx);
    const extruder_index_t extruder_id = source.extruder_id == EXTRUDER_ID_UNKNOWN ?
        target.tool_group.extruder_id() : source.extruder_id;
    double lift_mm = source.requested_lift_mm;
    const bool first_layer_override = source_layer_index == 0 &&
        print_config.vector_bool_or_default(
            "retract_lift_first_layer", extruder_id, false);
    const std::string top_policy = print_config.vector_string_or_default(
        "retract_lift_top", extruder_id, std::string());
    bool role_allows_lift = true;
    if (top_policy == "Not on top")
        role_allows_lift = !source.source_is_top;
    else if (top_policy == "Only on top")
        role_allows_lift = source.source_is_top;
    if (source.disable_lift ||
        (!first_layer_override && !source.outgoing_toolchange &&
         !role_allows_lift))
        lift_mm = 0.0;

    const double current_z_mm = unscaled(target.layer_group.print_z());
    const double above = print_config.vector_float_or_default(
        "retract_lift_above", extruder_id, 0.0);
    const double below = print_config.vector_float_or_default(
        "retract_lift_below", extruder_id, 0.0);
    if (!first_layer_override &&
        ((above > 0.0 && current_z_mm < above) ||
         (below > 0.0 && current_z_mm > below)))
        lift_mm = 0.0;

    const double minimum_travel = print_config.vector_float_or_default(
        "retract_lift_before_travel", extruder_id, 0.0);
    distf_t travel_length = 0.0;
    for (const TP::TravelLeaf &leaf : leaves)
        travel_length += leaf.entity.local_length();
    if (unscaled(travel_length) <= minimum_travel)
        lift_mm = 0.0;
    const double max_print_height = print_config.float_or_default(
        "max_print_height", 0.0);
    if (max_print_height > 0.0)
        lift_mm = std::min(
            lift_mm, std::max(0.0, max_print_height - current_z_mm));
    if (!std::isfinite(lift_mm) || lift_mm < 0.0)
        throw std::runtime_error("Lift filters produced an invalid Z request.");

    const coord_t inherited_start_lift = std::max<coord_t>(
        0, source.lifted_end_z - initial_travel_z);
    const bool ramping = print_config.vector_bool_or_default(
        "travel_ramping_lift", extruder_id, false);
    const double slope = print_config.vector_float_or_default(
        "travel_slope", extruder_id, 0.0);
    if (!std::isfinite(slope))
        throw std::runtime_error("Travel lift slope is not finite.");
    throw_if_cancelled(run_ctx);
    lift_travel(std::move(leaves), scale_i(lift_mm), inherited_start_lift,
                ramping, slope, run_ctx);
}

CreateRetractionLift &CreateRetractionLift::instance(
    orchestrator_handle *orchestrator)
{
    static CreateRetractionLift plugin(orchestrator);
    return plugin;
}

CreateRetractionLift::CreateRetractionLift(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_scope_property(printing_extrusion_scope_property_key(orchestrator))
{}

const char *CreateRetractionLift::id_impl() const noexcept
{
    return "layer_extrusion_edit.lift.default";
}

const char *CreateRetractionLift::name_impl() const noexcept
{
    return "Create retraction lift";
}

const char *CreateRetractionLift::description_impl() const noexcept
{
    return "Materializes retract-related wipe and travel Z lift.";
}

const char *CreateRetractionLift::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.lift";
}

slicing_step_t CreateRetractionLift::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *CreateRetractionLift::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t CreateRetractionLift::priority_impl() const noexcept
{
    return -40;
}

int32_t CreateRetractionLift::used_config_keys(
    raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"retract_lift", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_retract_lift", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_lift_above", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_lift_below", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_lift_first_layer", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_lift_top", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_lift_before_travel", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"travel_ramping_lift", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"travel_slope", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_lift", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_lift_length", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"max_print_height", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

int32_t CreateRetractionLift::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void CreateRetractionLift::inilialize_impl(storage_handle *) const
{
    initialize_create_retraction_lift_settings(m_orchestrator);
}

const char *CreateRetractionLift::progress_message_format_impl() const noexcept
{
    return "Creating retraction lifts: %u / %u layers";
}

void CreateRetractionLift::setup_impl(
    const plugin_run_context *run_ctx,
    const uint32_t run_count) const
{
    throw_if_cancelled(run_ctx);
    // Rebuild only the small group/layer-to-summary mapping. Geometry remains
    // untouched here so expensive work can stay parallel in the later passes.
    m_setup_valid = false;
    m_flat_indices.clear();
    m_layers.clear();
    m_source_indices.clear();
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->plan == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    if (ctx->print != nullptr)
        m_source_indices = source_layer_indices(Print(ctx->print), plan);
    size_t flat_idx = 0;
    m_flat_indices.resize(plan.group_count());
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        const PrintingGroup group = plan.group(group_idx);
        m_flat_indices[group_idx].resize(group.layer_group_count());
        for (uint32_t layer_idx = 0;
             layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            m_flat_indices[group_idx][layer_idx] = flat_idx++;
        }
    }
    if (flat_idx != run_count)
        throw std::runtime_error(
            "Lift run count does not match PrintingPlan layers.");
    m_layers.resize(flat_idx);
    m_setup_valid = true;
}

size_t CreateRetractionLift::flat_layer_index(
    const uint32_t group_idx,
    const uint32_t layer_idx) const
{
    if (!m_setup_valid || group_idx >= m_flat_indices.size() ||
        layer_idx >= m_flat_indices[group_idx].size())
        throw std::out_of_range(
            "CreateRetractionLift received an invalid layer index.");
    return m_flat_indices[group_idx][layer_idx];
}

const SourceLiftState *CreateRetractionLift::previous_layer_source(
    const size_t flat_idx, const plugin_run_context *run_ctx) const
{
    for (size_t idx = flat_idx; idx > 0; --idx) {
        throw_if_cancelled(run_ctx);
        const LayerLiftSummary &summary = m_layers[idx - 1];
        if (!summary.prepared)
            throw std::runtime_error(
                "A lift summary was read before the setup-run barrier.");
        if (summary.last.valid)
            return &summary.last;
    }
    return nullptr;
}

void CreateRetractionLift::setup_run_impl(
    const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || ctx->print == nullptr ||
        ctx->plan == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    PrintingLayerGroup layer(ctx->layer_group);
    const Config print_config = Print(ctx->print).config();
    LayerLiftSummary summary;
    summary.prepared = true;

    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        lift_state_definition(),
        [this, &summary, &print_config, &plan, run_ctx]
        (ScopeEntity *previous, ScopeEntity *next) {
            throw_if_cancelled(run_ctx);
            // Every matching scope appears exactly once as `next`. Prepare its
            // outgoing wipe here so a following travel observes the
            // final Z left by that wipe.
            if (next != nullptr)
                prepare_scope_wipe_lift(
                    *next, print_config, m_scope_property, run_ctx);

            // Only the terminal callback has `previous` without `next`. Store
            // that final scope as the layer summary because it is the sole
            // source that the first scope of a later layer may need. The
            // summary contains scalar state only; no scope handle escapes.
            if (previous != nullptr && next == nullptr)
                summary.last = describe_source(
                    *previous, plan, print_config, m_scope_property, run_ctx);
        },
        MatchingEntityDescendants::Skip);
    throw_if_cancelled(run_ctx);
    traversal.process(layer);
    throw_if_cancelled(run_ctx);
    m_layers[flat_layer_index(ctx->group_idx, ctx->layer_group_idx)] = summary;
    progress().add_max(1);
}

void CreateRetractionLift::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || ctx->print == nullptr ||
        ctx->plan == nullptr)
        return;

    const size_t flat_idx = flat_layer_index(
        ctx->group_idx, ctx->layer_group_idx);
    if (!m_layers[flat_idx].prepared)
        throw std::runtime_error("A lift layer has no prepared summary.");
    const SourceLiftState *preceding = previous_layer_source(flat_idx, run_ctx);
    PrintingLayerGroup layer(ctx->layer_group);
    PrintingPlan plan(ctx->plan);
    const Config print_config = Print(ctx->print).config();

    // For scopes A, B and C in this layer, traversal calls (nullptr, A),
    // (A, B), (B, C), then (C, nullptr). The callback applies lift to the
    // travel entering `next`; no travel enters the terminal nullptr boundary.
    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        lift_state_definition(),
        [this, preceding, &print_config, &plan,
         source_index = m_source_indices.at(ctx->group_idx).at(ctx->layer_group_idx).index, run_ctx]
        (ScopeEntity *previous, ScopeEntity *next) {
            throw_if_cancelled(run_ctx);
            // The terminal callback has a source but no target travel to edit.
            if (next == nullptr)
                return;

            // Both scopes belong to this layer. Read the complete source state
            // directly from `previous`, then apply its lift policy to the
            // reserved travel phase which enters `next`.
            if (previous != nullptr) {
                const SourceLiftState source = describe_source(
                    *previous, plan, print_config, m_scope_property, run_ctx);
                apply_transition_lift(
                    source, *next, source_index, print_config,
                    m_scope_property, run_ctx);
                return;
            }

            // The first local scope has no local `previous`. When a preceding
            // non-empty layer exists, use the immutable scalar description of
            // its final scope as the source for this inter-layer transition.
            if (preceding != nullptr) {
                apply_transition_lift(
                    *preceding, *next, source_index, print_config,
                    m_scope_property, run_ctx);
            }
        },
        MatchingEntityDescendants::Skip);
    throw_if_cancelled(run_ctx);
    traversal.process(layer);
    throw_if_cancelled(run_ctx);
    progress().increment();
}

} // namespace

void register_create_retraction_lift_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, CreateRetractionLift::instance(orchestrator).c_instance());
    const plugin_instance member = CreateRetractionLift::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Retraction lift";
    definition.tooltip = "Selects how Z lift is applied to retracted travels.";
    CreateRetractionLift::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::CreateRetractionLiftPlugin
