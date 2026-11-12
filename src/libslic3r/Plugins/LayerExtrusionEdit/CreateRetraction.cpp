///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Semantic retraction planning for compact PrintingPlan scopes
============================================================

The transition producer has already decided where process boundaries exist.
This module resolves the retraction settings at those boundaries and appends
firmware-neutral E-axis requests to the existing ordered phases. Local scope
pairs use their owner views directly. Layer-edge snapshots contain endpoint
values and provenance into immutable sliced geometry, never extrusion-tree
views. Parallel workers mutate only their own layer's process phases.

    setup_run_impl -> publish endpoint and regional provenance snapshots
    run_impl -> decide_boundary: preview the selected built-in travel route
        -> perimeter crossing test: suppress only hidden ordinary XY travels
        -> write_outgoing_retraction / write_incoming_retraction: E-axis events

The preview reuses the perimeter travel provider's router before any wipe is
created. Missing provenance and unknown travel providers retain retraction;
explicit requests, tool changes and configured layer changes are not suppressed.
*/

#include "CreateRetraction.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "ExtrusionScopeHelpers.hpp"
#include "PerimeterTravelPlanner.hpp"
#include "libslic3r/Api/plugin/cpp/properties/RetractionCrossingPolicy.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_extrusion_edit.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CreateRetractionPlugin {
// Register the permanent borrowed plugins and copy their alternative-selection setting descriptions into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_create_retraction_plugins(orchestrator_handle *orchestrator);

namespace {

using ScopeEntity = PrintingEntity<PrintingExtrusionScopeProperty>;

const char *const k_scope_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    nullptr
};

const char *const k_no_dependencies[] = {nullptr};

const RegionSettings::OptionKeyGroup k_retraction_region_keys = {
    "print_retract_length"
};

const char *const k_defined_config_keys[] = {
    "print_retract_length"
};

void initialize_create_retraction_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definition owned by this module. It fixes the option type, override
    scope, default and GUI metadata; the defined key is published in the print
    presets, so profiles read and write it by name like any built-in setting.
    Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "print_retract_length";
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.category = RAW_OPTION_CATEGORY_FILAMENT;
    def.label = "Retraction length";
    def.tooltip = "Override the retract_length setting from the printer config. Used for calibration.";
    def.can_be_disabled = 1;
    def.has_min = 1;
    def.min_value = 0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!0";
    orchestrator_create_option_def(orchestrator, &def);
}

/* One absolute endpoint in scaled PrintingPlan coordinates. */
struct PlannedPosition
{
    coord_t x = 0;
    coord_t y = 0;
    coord_t z = 0;
};

/* Properties inherited while descending from a PrintingExtrusion root. */
struct EffectiveState
{
    coord_t z_offset = 0;
    std::optional<EPropertyModifier> modifier;
};

/* Geometric endpoint and its tiny inside probe for regional lookup. */
struct GeometricEndpoint
{
    MutableExtrusionEntity leaf;
    PlannedPosition position;
    EPropertyModifier modifier{};
    c_point interior_point{};
    bool has_interior_point = false;
};

/* Scalar facts required to decide one side of a transition. */
struct BoundaryState
{
    bool valid = false;
    PlannedPosition position;
    EPropertyModifier modifier{};
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    double normal_retract_target = 0.0;
    TravelConnection::TravelEndpoint travel;
    bool first_object_layer = false;
    bool has_hidden_interior = false;
    bool exclude_top_surfaces = false;
};

/* Immutable values published by one setup_run worker. */
struct LayerBoundarySummary
{
    bool prepared = false;
    BoundaryState first;
    BoundaryState last;
    bool first_incoming_toolchange = false;
    bool last_outgoing_toolchange = false;
};

/* Complete process decision calculated identically from either layer side. */
struct RetractionDecision
{
    bool requested = false;
    bool toolchange_retraction = false;
    double target = 0.0;
    double restart_extra = 0.0;
};

/* Add scaled coordinates without allowing signed overflow. */
coord_t checked_add(coord_t lhs, coord_t rhs);

/* Resolve direct Z and modifier properties over inherited state. */
EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent);

/* Convert the traversal's autonomous snapshot to this plugin's local state. */
EffectiveState traversal_state(const ExtrusionPropertyState &properties);

/* Describe the properties which every retraction scope traversal must inherit. */
ExtrusionPropertyStateDefinition retraction_state_definition();

/* Find the first content leaf without borrowing later siblings' role. */
bool find_first_geometric_role(
    MutableExtrusionEntity entity,
    std::optional<raw_extrusion_role> inherited_role,
    std::optional<raw_extrusion_role> &result,
    bool entity_already_resolved,
    const plugin_run_context *run_ctx);

/* Resolve the first role from printable content, never from technical phases. */
std::optional<raw_extrusion_role> first_content_role(
    const ScopeEntity &target,
    const ExtrusionScope::OrderedExtrusionScope &scope,
    const plugin_run_context *run_ctx);

/* Resolve the first geometric leaf, optionally starting with entity resolved. */
bool find_first_geometry(MutableExtrusionEntity entity, coord_t print_z,
                         const EffectiveState &parent,
                         GeometricEndpoint &endpoint,
                         bool entity_already_resolved,
                         const plugin_run_context *run_ctx);

/* Resolve the last geometric leaf, optionally starting with entity resolved. */
bool find_last_geometry(MutableExtrusionEntity entity, coord_t print_z,
                        const EffectiveState &parent,
                        GeometricEndpoint &endpoint,
                        bool entity_already_resolved,
                        const plugin_run_context *run_ctx);

/* Resolve one scope endpoint while retaining only its effective leaf state. */
GeometricEndpoint scope_endpoint(const ScopeEntity &scope,
                                 const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                                 bool first,
                                 const plugin_run_context *run_ctx);

/* Read the source-region override for the normal retraction target. */
double normal_retract_target(const Config &print_config,
                             const PrintingPlan &plan,
                             const ScopeEntity &source,
                             const GeometricEndpoint &endpoint,
                             const plugin_run_context *run_ctx);

/* Convert a scope side into the scalar decision format. */
BoundaryState boundary_state(const ScopeEntity &scope,
                             const Config &print_config,
                             const PrintingPlan &plan,
                             const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                             bool first,
                             bool resolve_normal_target,
                             const plugin_run_context *run_ctx,
                             const PluginPropertyKey<RetractionCrossingPolicy> *policy_key = nullptr);

/* Return direct XY distance for discontinuity checks and unknown providers. */
double planar_gap_mm(const BoundaryState &source,
                     const BoundaryState &target);

/* Validate matching scope flags and report a real tool selection. */
bool boundary_toolchange(const PrintingExtrusionScopeProperty &source,
                         const PrintingExtrusionScopeProperty &target);

/* Resolve one transition into semantic E-axis and tool-selection requests. */
RetractionDecision decide_boundary(const Config &print_config,
                                   const BoundaryState &source,
                                   const BoundaryState &target,
                                   bool layer_change,
                                   bool actual_toolchange,
                                   AvoidCrossingPerimetersTravelPlugin::PerimeterTravelPlanner &planner,
                                   bool known_travel_provider,
                                   bool perimeter_avoiding);

/* Configure one empty event with a process role and zero physical flow. */
void set_event_attributes(MutableExtrusionEntity event,
                          raw_extrusion_role role);

/* Append the source-owned semantic Retract request. */
void write_outgoing_retraction(const ScopeEntity &source,
                               const RetractionDecision &decision,
                               const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                               const plugin_run_context *run_ctx);

/* Append the target-owned Unretract request after tool-group selection events. */
void write_incoming_retraction(const ScopeEntity &target,
                               const RetractionDecision &decision,
                               const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                               const plugin_run_context *run_ctx);

/* Count marked roots for progress without retaining their handles. */
uint32_t scope_count(PrintingLayerGroup &layer,
                     const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                     const plugin_run_context *run_ctx);

/* Append the end-of-print Retract event to the plan after sequence. */
void append_terminal_retraction(storage_handle *storage,
                                PrintingPlan &plan,
                                double target,
                                const plugin_run_context *run_ctx);

/* Parallel provider that writes retraction operations around scope pairs. */
class CreateRetraction final : public PluginBase
{
public:
    static CreateRetraction &instance(orchestrator_handle *orchestrator);
    explicit CreateRetraction(orchestrator_handle *orchestrator);

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
    void setup_impl(const plugin_run_context *run_ctx,
                    uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    /* Convert final group/layer indexes into the scalar summary array. */
    size_t flat_layer_index(uint32_t group_idx, uint32_t layer_idx) const;

    /* Find the nearest preceding non-empty prepared layer. */
    const LayerBoundarySummary *previous_nonempty_layer(
        size_t flat_idx, const plugin_run_context *run_ctx) const;

    /* Find the nearest following non-empty prepared layer. */
    const LayerBoundarySummary *next_nonempty_layer(
        size_t flat_idx, const plugin_run_context *run_ctx) const;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
    PluginPropertyKey<RetractionCrossingPolicy> m_crossing_policy;
    mutable bool m_setup_valid = false;
    mutable std::vector<std::vector<size_t>> m_flat_indices;
    mutable std::vector<LayerBoundarySummary> m_layers;
};

/* Sequential provider that owns the explicit end-of-print boundary. */
class CreateTerminalRetraction final : public PluginBase
{
public:
    static CreateTerminalRetraction &instance(orchestrator_handle *orchestrator);
    explicit CreateTerminalRetraction(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
};

coord_t checked_add(const coord_t lhs, const coord_t rhs)
{
    if ((rhs > 0 && lhs > (std::numeric_limits<coord_t>::max)() - rhs) ||
        (rhs < 0 && lhs < (std::numeric_limits<coord_t>::min)() - rhs))
        throw std::overflow_error("A retraction endpoint exceeds coord_t.");
    return lhs + rhs;
}

EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent)
{
    EffectiveState state = parent;
    if (const EPropertyZOffset *z_offset = entity.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyModifier *modifier = entity.get(EPropertyModifier::key))
        state.modifier = *modifier;
    return state;
}

EffectiveState traversal_state(const ExtrusionPropertyState &properties)
{
    EffectiveState state;
    if (const EPropertyZOffset *z_offset = properties.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyModifier *modifier = properties.get(EPropertyModifier::key))
        state.modifier = *modifier;
    return state;
}

ExtrusionPropertyStateDefinition retraction_state_definition()
{
    ExtrusionPropertyStateDefinition definition;
    definition.track(EPropertyAttributes::key);
    definition.track(EPropertyZOffset::key);
    definition.track(EPropertyModifier::key);
    return definition;
}

// Resolve the first segmented or deposit leaf's role through borrowed children; cancellation throws, and an absent role remains absent without tree mutation.
bool find_first_geometric_role(
    MutableExtrusionEntity entity,
    const std::optional<raw_extrusion_role> inherited_role,
    std::optional<raw_extrusion_role> &result,
    const bool entity_already_resolved,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    std::optional<raw_extrusion_role> role = inherited_role;
    if (!entity_already_resolved) {
        const EPropertyAttributes *attributes =
            entity.get(EPropertyAttributes::key);
        if (attributes != nullptr)
            role = raw_extrusion_role(attributes->extrusion_role());
    }

    // Geometry ends the search even when no effective role exists. A later
    // sibling cannot describe the first leaf's semantic process role.
    if (entity.segment_count() != 0 || entity.is_deposit()) {
        result = role;
        return true;
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        if (find_first_geometric_role(
                entity.child_mutable(child_idx), role, result, false, run_ctx))
            return true;
    }
    return false;
}

std::optional<raw_extrusion_role> first_content_role(
    const ScopeEntity &target,
    const ExtrusionScope::OrderedExtrusionScope &scope,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EPropertyAttributes *inherited_attributes =
        target.effective_properties.get(EPropertyAttributes::key);
    const std::optional<raw_extrusion_role> inherited_role = inherited_attributes ?
        std::optional<raw_extrusion_role>(raw_extrusion_role(
            inherited_attributes->extrusion_role())) : std::nullopt;
    MutableExtrusionEntity content = scope.content();
    std::optional<raw_extrusion_role> result;
    if (!find_first_geometric_role(
            content, inherited_role, result,
            content.handle() == scope.root().handle(), run_ctx))
        throw std::runtime_error(
            "A retraction target scope has no printable geometry.");
    return result;
}

// Resolve borrowed geometry in forward order and copy its endpoint and modifier; deposits have no interior probe, and cancellation or Z overflow throws without mutation.
bool find_first_geometry(MutableExtrusionEntity entity, const coord_t print_z,
                         const EffectiveState &parent,
                         GeometricEndpoint &endpoint,
                         const bool entity_already_resolved,
                         const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);
    ExtrusionScope::LeafEndpoints endpoints;
    if (ExtrusionScope::leaf_endpoints(entity.readonly(), endpoints)) {
        endpoint.leaf = entity;
        endpoint.position = PlannedPosition{
            endpoints.first.x,
            endpoints.first.y,
            checked_add(checked_add(print_z, state.z_offset), endpoints.first_z)};
        endpoint.modifier = state.modifier.value_or(EPropertyModifier{});
        const distf_t distance = std::min(
            distf_t(SCALED_EPSILON), entity.local_length() / 2.0);
        if (distance > 0.0) {
            endpoint.interior_point = entity.point_from_begin(distance);
            endpoint.has_interior_point = true;
        }
        return true;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        throw_if_cancelled(run_ctx);
        if (find_first_geometry(entity.child_mutable(child_idx), print_z,
                                state, endpoint, false, run_ctx))
            return true;
    }
    return false;
}

// Resolve borrowed geometry backwards and copy its endpoint and modifier; deposits have no interior probe, and cancellation or Z overflow throws without mutation.
bool find_last_geometry(MutableExtrusionEntity entity, const coord_t print_z,
                        const EffectiveState &parent,
                        GeometricEndpoint &endpoint,
                        const bool entity_already_resolved,
                        const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);
    ExtrusionScope::LeafEndpoints endpoints;
    if (ExtrusionScope::leaf_endpoints(entity.readonly(), endpoints)) {
        endpoint.leaf = entity;
        endpoint.position = PlannedPosition{
            endpoints.last.x,
            endpoints.last.y,
            checked_add(checked_add(print_z, state.z_offset), endpoints.last_z)};
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
        if (find_last_geometry(entity.child_mutable(child_idx - 1), print_z,
                               state, endpoint, false, run_ctx))
            return true;
    }
    return false;
}

GeometricEndpoint scope_endpoint(
    const ScopeEntity &scope,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const bool first,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const ExtrusionScope::OrderedExtrusionScope ordered(scope.entity, key);
    const EffectiveState scope_state = traversal_state(scope.effective_properties);
    MutableExtrusionEntity content = ordered.content();
    const bool content_is_scope = content.handle() == ordered.root().handle();
    GeometricEndpoint endpoint;
    const bool found = first ?
        find_first_geometry(content, scope.layer_group.print_z(), scope_state,
                            endpoint, content_is_scope, run_ctx) :
        find_last_geometry(content, scope.layer_group.print_z(), scope_state,
                           endpoint, content_is_scope, run_ctx);
    if (!found)
        throw std::runtime_error("A prepared retraction scope has no printable geometry.");
    return endpoint;
}

// Read borrowed regional settings at the source point, using the island's first region for unmatched deposits; return printer defaults when disabled, and throw for cancellation or unresolved ordinary geometry.
double normal_retract_target(const Config &print_config,
                             const PrintingPlan &plan,
                             const ScopeEntity &source,
                             const GeometricEndpoint &endpoint,
                             const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const extruder_index_t extruder_id = source.tool_group.extruder_id();
    const double printer_target = print_config.vector_float_or_default(
        "retract_length", extruder_id, 0.0);
    const LayerRegionIsland region_island = source.printing_extrusion.region_island();
    if (!region_island.valid() || region_island.region_count() == 0)
        return printer_target;

    const c_point shift = plan.object_instance_shift(source.printing_extrusion);
    c_point lookup_point{
        endpoint.position.x - shift.x,
        endpoint.position.y - shift.y
    };
    RegionSettingsPointResult lookup = RegionSettings::lookup_at_point(
        region_island, k_retraction_region_keys, lookup_point);
    throw_if_cancelled(run_ctx);
    if (!lookup.found() && endpoint.has_interior_point) {
        lookup_point = c_point{
            endpoint.interior_point.x - shift.x,
            endpoint.interior_point.y - shift.y
        };
        lookup = RegionSettings::lookup_at_point(
            region_island, k_retraction_region_keys, lookup_point);
        throw_if_cancelled(run_ctx);
    }
    if (!lookup.found() && endpoint.leaf.is_deposit()) {
        const Config config = region_island.region(0).print_region().config();
        return config.float_or_default("print_retract_length", printer_target);
    }
    if (!lookup.found())
        throw std::runtime_error(
            "The retraction source point does not select an unambiguous region.");
    return lookup.value.is_enabled("print_retract_length") ?
        lookup.value.get_float("print_retract_length") : printer_target;
}

BoundaryState boundary_state(
    const ScopeEntity &scope,
    const Config &print_config,
    const PrintingPlan &plan,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const bool first,
    const bool resolve_normal_target,
    const plugin_run_context *run_ctx,
    const PluginPropertyKey<RetractionCrossingPolicy> *policy_key)
{
    throw_if_cancelled(run_ctx);
    const GeometricEndpoint endpoint = scope_endpoint(scope, key, first, run_ctx);
    BoundaryState state;
    state.valid = true;
    state.position = endpoint.position;
    state.modifier = endpoint.modifier;
    state.extruder_id = scope.tool_group.extruder_id();
    if (policy_key != nullptr) {
        const RetractionCrossingPolicy *policy = policy_key->get(scope.printing_extrusion.root());
        state.exclude_top_surfaces = policy != nullptr && policy->exclude_top_surfaces != 0;
    }
    // Immutable sliced provenance may be read by either side of a layer edge;
    // no extrusion-tree handle escapes this endpoint snapshot.
    state.travel.position = {endpoint.position.x, endpoint.position.y, endpoint.position.z, true};
    state.travel.region_island = scope.printing_extrusion.region_island();
    state.travel.printing_catalog_id = scope.printing_extrusion.printing_catalog_id();
    state.travel.interior_point = endpoint.interior_point;
    state.travel.has_interior_point = endpoint.has_interior_point;
    if (state.travel.region_island.valid() && state.travel.region_island.region_count() > 0) {
        const LayerRegion region = state.travel.region_island.region(0);
        state.first_object_layer = region.layer().bottom_z() <= SCALED_EPSILON;
        const Config regional = region.print_region().config();
        state.has_hidden_interior = regional.get("fill_density").get_float() > 0. ||
            regional.bool_or_default("avoid_crossing_perimeters", false);
    }
    if (resolve_normal_target)
        state.normal_retract_target = normal_retract_target(
            print_config, plan, scope, endpoint, run_ctx);
    return state;
}

double planar_gap_mm(const BoundaryState &source,
                     const BoundaryState &target)
{
    const long double dx =
        static_cast<long double>(target.position.x) - source.position.x;
    const long double dy =
        static_cast<long double>(target.position.y) - source.position.y;
    return unscaled(double(std::sqrt(dx * dx + dy * dy)));
}

bool boundary_toolchange(const PrintingExtrusionScopeProperty &source,
                         const PrintingExtrusionScopeProperty &target)
{
    const bool outgoing = printing_extrusion_scope_has_flag(
        source, PRINTING_EXTRUSION_SCOPE_OUTGOING_TOOLCHANGE);
    const bool incoming = printing_extrusion_scope_has_flag(
        target, PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE);
    if (outgoing != incoming)
        throw std::runtime_error(
            "Adjacent scopes disagree about their tool-change transition.");
    return outgoing;
}

RetractionDecision decide_boundary(const Config &print_config,
                                   const BoundaryState &source,
                                   const BoundaryState &target,
                                   const bool layer_change,
                                   const bool actual_toolchange,
                                   AvoidCrossingPerimetersTravelPlugin::PerimeterTravelPlanner &planner,
                                   const bool known_travel_provider,
                                   const bool perimeter_avoiding)
{
    if (!source.valid || !target.valid)
        throw std::invalid_argument("Retraction decisions need two valid scope endpoints.");

    RetractionDecision decision;
    decision.toolchange_retraction = actual_toolchange ||
        source.modifier.toolchange_retraction != 0 ||
        target.modifier.toolchange_retraction != 0;
    const extruder_index_t retracting_extruder = source.extruder_id != EXTRUDER_ID_UNKNOWN ?
        source.extruder_id : target.extruder_id;
    double gap = planar_gap_mm(source, target);
    const double minimum_gap = print_config.vector_float_or_default(
        "retract_before_travel", retracting_extruder, 0.0);
    const bool retract_on_layer = print_config.vector_bool_or_default(
        "retract_layer_change", retracting_extruder, false);
    const bool discontinuous = gap >= unscaled(SCALED_EPSILON);
    const bool forced = decision.toolchange_retraction ||
        source.modifier.enforce_retraction != 0 ||
        target.modifier.enforce_retraction != 0 ||
        (layer_change && retract_on_layer);
    const bool crossing_only = print_config.bool_or_default("only_retract_when_crossing_perimeters", false) &&
        !(target.first_object_layer && print_config.bool_or_default("enforce_retract_first_layer", true));
    std::vector<c_point> route;
    if (!forced && discontinuous && known_travel_provider && (perimeter_avoiding || crossing_only)) {
        route = perimeter_avoiding ? planner.path(source.travel, target.travel, retracting_extruder) :
            TravelConnection::straight_path(source.travel, target.travel, retracting_extruder);
        gap = 0.;
        for (size_t i = 1; i < route.size(); ++i)
            gap += unscaled(c_point_distance_to(route[i - 1], route[i]));
    }
    decision.requested = forced || (discontinuous && gap >= minimum_gap);
    if (source.modifier.disable_retraction != 0 ||
        target.modifier.disable_retraction != 0)
        decision.requested = false;
    if (!decision.requested)
        return decision;

    // Explicit requests and tool/layer transitions retain their own semantics.
    // Only an ordinary XY-travel request may be suppressed as hidden interior.
    if (!forced && crossing_only && target.has_hidden_interior && known_travel_provider &&
        !planner.crosses(source.travel, target.travel, retracting_extruder, route, target.exclude_top_surfaces))
        return RetractionDecision{};

    decision.target = decision.toolchange_retraction ?
        print_config.vector_float_or_default(
            "retract_length_toolchange", retracting_extruder, 0.0) :
        source.normal_retract_target;
    decision.restart_extra = print_config.vector_float_or_default(
        decision.toolchange_retraction ? "retract_restart_extra_toolchange" :
                                         "retract_restart_extra",
        target.extruder_id, 0.0);
    if (!std::isfinite(decision.target) || decision.target < 0.0 ||
        !std::isfinite(decision.restart_extra))
        throw std::runtime_error("Retraction settings produced an invalid E-axis request.");
    return decision;
}

void set_event_attributes(MutableExtrusionEntity event,
                          const raw_extrusion_role role)
{
    EPropertyAttributes &attributes = event.get_or_add(EPropertyAttributes::key);
    attributes.extrusion_role(role)
        .mm3_per_mm(0.0)
        .width(0.f)
        .height(0.f)
        .no_seam_enabled(false);
}

void write_outgoing_retraction(
    const ScopeEntity &source,
    const RetractionDecision &decision,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!decision.requested || decision.target <= 0.0)
        return;
    const ExtrusionScope::OrderedExtrusionScope scope(source.entity, key);
    throw_if_cancelled(run_ctx);
    MutableExtrusionEntity event = ExtrusionScope::append_phase_leaf(scope.after());
    throw_if_cancelled(run_ctx);
    set_event_attributes(event, RAW_EXTRUSION_ROLE_RETRACT);
    throw_if_cancelled(run_ctx);
    event.get_or_add(EPropertyExtrusionAxis::key).retract_to(
        decision.target, decision.toolchange_retraction);
}

void write_incoming_retraction(
    const ScopeEntity &target,
    const RetractionDecision &decision,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (!decision.requested || decision.target <= 0.0)
        return;

    const ExtrusionScope::OrderedExtrusionScope scope(target.entity, key);
    const std::optional<raw_extrusion_role> role = first_content_role(
        target, scope, run_ctx);
    throw_if_cancelled(run_ctx);
    MutableExtrusionEntity event = ExtrusionScope::append_phase_leaf(scope.before());
    throw_if_cancelled(run_ctx);
    set_event_attributes(event, role.value_or(RAW_EXTRUSION_ROLE_NONE) |
        RAW_EXTRUSION_ROLE_UNRETRACT);
    throw_if_cancelled(run_ctx);
    event.get_or_add(EPropertyExtrusionAxis::key).unretract(
        decision.restart_extra, decision.toolchange_retraction);
}

uint32_t scope_count(
    PrintingLayerGroup &layer,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    uint32_t count = 0;
    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        key,
        [&count, run_ctx](ScopeEntity *, ScopeEntity *next) {
            throw_if_cancelled(run_ctx);
            if (next != nullptr)
                ++count;
        },
        MatchingEntityDescendants::Skip);
    throw_if_cancelled(run_ctx);
    traversal.process(layer);
    throw_if_cancelled(run_ctx);
    return count;
}

void append_terminal_retraction(storage_handle *storage,
                                PrintingPlan &plan,
                                const double target,
                                const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (storage == nullptr || target <= 0.0)
        return;
    StoredExtrusionEntity event(storage);
    event.set_can_rotate(false);
    throw_if_cancelled(run_ctx);
    set_event_attributes(event.mutable_view(), RAW_EXTRUSION_ROLE_RETRACT);
    throw_if_cancelled(run_ctx);
    event.get_or_add(EPropertyExtrusionAxis::key).retract_to(target, false);
    throw_if_cancelled(run_ctx);
    if (!plan.events().append_after_move(event.mutable_view()).valid())
        throw std::runtime_error("Unable to append the terminal retraction event.");
}

CreateRetraction &CreateRetraction::instance(orchestrator_handle *orchestrator)
{
    static CreateRetraction plugin(orchestrator);
    return plugin;
}

CreateRetraction::CreateRetraction(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_scope_property(printing_extrusion_scope_property_key(orchestrator)),
    m_crossing_policy(retraction_crossing_policy_key(orchestrator))
{}

const char *CreateRetraction::id_impl() const noexcept
{
    return "layer_extrusion_edit.retraction.default";
}

const char *CreateRetraction::name_impl() const noexcept
{
    return "Create retraction";
}

const char *CreateRetraction::description_impl() const noexcept
{
    return "Adds semantic retract and unretract events to prepared transitions.";
}

const char *CreateRetraction::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.retraction";
}

slicing_step_t CreateRetraction::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *CreateRetraction::dependencies_impl() const noexcept
{
    return k_scope_dependencies;
}

int32_t CreateRetraction::priority_impl() const noexcept
{
    return -90;
}

int32_t CreateRetraction::used_config_keys(raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"retract_before_travel", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_layer_change", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_length", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_length_toolchange", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_restart_extra", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_restart_extra_toolchange", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_retract_length", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"only_retract_when_crossing_perimeters", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"enforce_retract_first_layer", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"fill_density", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    const int32_t count = int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
    return count + AvoidCrossingPerimetersTravelPlugin::PerimeterTravelPlanner::used_config_keys(
        keys == nullptr ? nullptr : keys + count);
}

int32_t CreateRetraction::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void CreateRetraction::inilialize_impl(storage_handle *) const
{
    initialize_create_retraction_settings(m_orchestrator);
}

const char *CreateRetraction::progress_message_format_impl() const noexcept
{
    return "Creating retractions: %u / %u scopes";
}

void CreateRetraction::setup_impl(const plugin_run_context *run_ctx,
                                  const uint32_t run_count) const
{
    throw_if_cancelled(run_ctx);
    m_setup_valid = false;
    m_flat_indices.clear();
    m_layers.clear();
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->plan == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    size_t flat_idx = 0;
    m_flat_indices.resize(plan.group_count());
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        const PrintingGroup group = plan.group(group_idx);
        m_flat_indices[group_idx].resize(group.layer_group_count());
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            m_flat_indices[group_idx][layer_idx] = flat_idx++;
        }
    }
    if (flat_idx != run_count)
        throw std::runtime_error("Retraction run count does not match PrintingPlan layers.");
    m_layers.resize(flat_idx);
    m_setup_valid = true;
}

size_t CreateRetraction::flat_layer_index(
    const uint32_t group_idx, const uint32_t layer_idx) const
{
    if (!m_setup_valid || group_idx >= m_flat_indices.size() ||
        layer_idx >= m_flat_indices[group_idx].size())
        throw std::out_of_range("CreateRetraction received an invalid layer index.");
    return m_flat_indices[group_idx][layer_idx];
}

const LayerBoundarySummary *CreateRetraction::previous_nonempty_layer(
    const size_t flat_idx, const plugin_run_context *run_ctx) const
{
    for (size_t idx = flat_idx; idx > 0; --idx) {
        throw_if_cancelled(run_ctx);
        const LayerBoundarySummary &summary = m_layers[idx - 1];
        if (!summary.prepared)
            throw std::runtime_error("A retraction summary was read before the host barrier.");
        if (summary.first.valid)
            return &summary;
    }
    return nullptr;
}

const LayerBoundarySummary *CreateRetraction::next_nonempty_layer(
    const size_t flat_idx, const plugin_run_context *run_ctx) const
{
    for (size_t idx = flat_idx + 1; idx < m_layers.size(); ++idx) {
        throw_if_cancelled(run_ctx);
        const LayerBoundarySummary &summary = m_layers[idx];
        if (!summary.prepared)
            throw std::runtime_error("A retraction summary was read before the host barrier.");
        if (summary.first.valid)
            return &summary;
    }
    return nullptr;
}

void CreateRetraction::setup_run_impl(const plugin_run_context *run_ctx) const
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
    LayerBoundarySummary summary;
    summary.prepared = true;
    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        retraction_state_definition(),
        [this, &summary, &print_config, &plan, run_ctx](
            ScopeEntity *previous, ScopeEntity *next) {
            throw_if_cancelled(run_ctx);
            if (previous == nullptr && next != nullptr) {
                summary.first = boundary_state(
                    *next, print_config, plan, m_scope_property, true, false, run_ctx, &m_crossing_policy);
                summary.first_incoming_toolchange = printing_extrusion_scope_has_flag(
                    *next->property,
                    PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE);
            }
            if (previous != nullptr && next == nullptr) {
                summary.last = boundary_state(
                    *previous, print_config, plan, m_scope_property, false, true, run_ctx, &m_crossing_policy);
                summary.last_outgoing_toolchange = printing_extrusion_scope_has_flag(
                    *previous->property,
                    PRINTING_EXTRUSION_SCOPE_OUTGOING_TOOLCHANGE);
            }
        },
        MatchingEntityDescendants::Skip);
    throw_if_cancelled(run_ctx);
    traversal.process(layer);
    throw_if_cancelled(run_ctx);
    m_layers[flat_layer_index(ctx->group_idx, ctx->layer_group_idx)] = summary;
    progress().add_max(scope_count(layer, m_scope_property, run_ctx));
}

void CreateRetraction::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || ctx->print == nullptr ||
        ctx->plan == nullptr)
        return;

    const size_t flat_idx = flat_layer_index(ctx->group_idx, ctx->layer_group_idx);
    if (!m_layers[flat_idx].prepared)
        throw std::runtime_error("A retraction layer has no prepared summary.");
    const LayerBoundarySummary *preceding = previous_nonempty_layer(flat_idx, run_ctx);
    const LayerBoundarySummary *following = next_nonempty_layer(flat_idx, run_ctx);
    PrintingLayerGroup layer(ctx->layer_group);
    PrintingPlan plan(ctx->plan);
    const Config print_config = Print(ctx->print).config();
    AvoidCrossingPerimetersTravelPlugin::PerimeterTravelPlanner planner(Print(ctx->print), plan, run_ctx,
        ctx->group_idx, layer.print_z());
    const orchestrator_plugin_handle *travel_provider = orchestrator_select_plugin(
        m_orchestrator, print_config.handle(), STEP_LAYER_EXTRUSION_EDIT, "layer_extrusion_edit.travel");
    const char *travel_id = orchestrator_plugin_id(travel_provider);
    const bool perimeter_avoiding = travel_id != nullptr &&
        std::strcmp(travel_id, "layer_extrusion_edit.travel.avoid_crossing_perimeters") == 0;
    const bool known_travel_provider = perimeter_avoiding || (travel_id != nullptr &&
        std::strcmp(travel_id, "layer_extrusion_edit.travel.default") == 0);

    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        retraction_state_definition(),
        [this, &print_config, &plan, &planner, known_travel_provider, perimeter_avoiding,
         &progress = progress(), preceding, following, run_ctx]
        (ScopeEntity *previous, ScopeEntity *next) {
            throw_if_cancelled(run_ctx);
            if (previous == nullptr && next != nullptr) {
                if (preceding != nullptr) {
                    const bool incoming_toolchange = printing_extrusion_scope_has_flag(
                        *next->property,
                        PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE);
                    if (preceding->last_outgoing_toolchange != incoming_toolchange)
                        throw std::runtime_error(
                            "Inter-layer scopes disagree about their tool-change transition.");
                    const BoundaryState target = boundary_state(
                        *next, print_config, plan, m_scope_property, true, false, run_ctx, &m_crossing_policy);
                    const RetractionDecision decision = decide_boundary(
                        print_config, preceding->last, target, true,
                        incoming_toolchange, planner, known_travel_provider, perimeter_avoiding);
                    write_incoming_retraction(*next, decision, m_scope_property, run_ctx);
                }
                progress.increment();
                return;
            }

            if (previous != nullptr && next != nullptr) {
                const bool outgoing_transition = printing_extrusion_scope_has_flag(
                    *previous->property,
                    PRINTING_EXTRUSION_SCOPE_OUTGOING_TRANSITION);
                const bool incoming_transition = printing_extrusion_scope_has_flag(
                    *next->property,
                    PRINTING_EXTRUSION_SCOPE_INCOMING_TRANSITION);
                if (outgoing_transition != incoming_transition)
                    throw std::runtime_error(
                        "Adjacent scopes disagree about their retraction transition.");
                if (outgoing_transition) {
                    const BoundaryState source = boundary_state(
                        *previous, print_config, plan, m_scope_property, false, true, run_ctx, &m_crossing_policy);
                    const BoundaryState target = boundary_state(
                        *next, print_config, plan, m_scope_property, true, false, run_ctx, &m_crossing_policy);
                    const RetractionDecision decision = decide_boundary(
                        print_config, source, target, false,
                        boundary_toolchange(*previous->property, *next->property),
                        planner, known_travel_provider, perimeter_avoiding);
                    write_outgoing_retraction(*previous, decision, m_scope_property, run_ctx);
                    write_incoming_retraction(*next, decision, m_scope_property, run_ctx);
                }
                progress.increment();
                return;
            }

            if (previous != nullptr && following != nullptr) {
                const bool outgoing_toolchange = printing_extrusion_scope_has_flag(
                    *previous->property,
                    PRINTING_EXTRUSION_SCOPE_OUTGOING_TOOLCHANGE);
                if (outgoing_toolchange != following->first_incoming_toolchange)
                    throw std::runtime_error(
                        "Inter-layer scopes disagree about their tool-change transition.");
                const BoundaryState source = boundary_state(
                    *previous, print_config, plan, m_scope_property, false, true, run_ctx, &m_crossing_policy);
                const RetractionDecision decision = decide_boundary(
                    print_config, source, following->first, true,
                    outgoing_toolchange, planner, known_travel_provider, perimeter_avoiding);
                write_outgoing_retraction(*previous, decision, m_scope_property, run_ctx);
            }
        },
        MatchingEntityDescendants::Skip);
    throw_if_cancelled(run_ctx);
    traversal.process(layer);
}

CreateTerminalRetraction &CreateTerminalRetraction::instance(
    orchestrator_handle *orchestrator)
{
    static CreateTerminalRetraction plugin(orchestrator);
    return plugin;
}

CreateTerminalRetraction::CreateTerminalRetraction(
    orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_scope_property(printing_extrusion_scope_property_key(orchestrator))
{}

const char *CreateTerminalRetraction::id_impl() const noexcept
{
    return "extrusion_edit.terminal_retraction.default";
}

const char *CreateTerminalRetraction::name_impl() const noexcept
{
    return "Create terminal retraction";
}

const char *CreateTerminalRetraction::description_impl() const noexcept
{
    return "Adds the final semantic retraction to the PrintingPlan after sequence.";
}

const char *CreateTerminalRetraction::exclusive_group_impl() const noexcept
{
    return "extrusion_edit.terminal_retraction";
}

slicing_step_t CreateTerminalRetraction::step_impl() const noexcept
{
    return STEP_EXTRUSION_EDIT;
}

const char *const *CreateTerminalRetraction::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t CreateTerminalRetraction::priority_impl() const noexcept
{
    return -100;
}

int32_t CreateTerminalRetraction::used_config_keys(
    raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"retract_length", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"print_retract_length", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

void CreateTerminalRetraction::run_impl(
    const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_extrusion_edition *ctx =
        plugin_ctx_as_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        run_ctx->plugin_storage == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    const Config print_config = Print(ctx->print).config();
    bool found_terminal = false;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
                m_scope_property,
                retraction_state_definition(),
                [this, &found_terminal, &print_config, &plan, run_ctx]
                (ScopeEntity *, ScopeEntity *next) {
                    throw_if_cancelled(run_ctx);
                    if (next == nullptr || !printing_extrusion_scope_has_flag(
                            *next->property, PRINTING_EXTRUSION_SCOPE_TERMINAL))
                        return;
                    if (found_terminal)
                        throw std::runtime_error(
                            "The PrintingPlan contains several terminal extrusion scopes.");
                    found_terminal = true;
                    const BoundaryState source = boundary_state(
                        *next, print_config, plan, m_scope_property, false, true, run_ctx);
                    if (source.modifier.disable_retraction == 0)
                        append_terminal_retraction(
                            run_ctx->plugin_storage, plan,
                            source.normal_retract_target, run_ctx);
                },
                MatchingEntityDescendants::Skip);
            throw_if_cancelled(run_ctx);
            traversal.process(layer);
            throw_if_cancelled(run_ctx);
        }
    }
}

} // namespace

void register_create_retraction_plugins(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, CreateRetraction::instance(orchestrator).c_instance());
    const plugin_instance member = CreateRetraction::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Retraction planning";
    definition.tooltip = "Selects how semantic retractions are placed around ordered travels.";
    CreateRetraction::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
    orchestrator_register_plugin(
        orchestrator, CreateTerminalRetraction::instance(orchestrator).c_instance());
    const plugin_instance terminal_member = CreateTerminalRetraction::instance(orchestrator).c_instance();
    raw_exclusive_group_def terminal_definition = raw_exclusive_group_def_init();
    terminal_definition.exclusive_group = terminal_member.vt->get_exclusive_group(terminal_member.ctx);
    terminal_definition.label = "Terminal retraction";
    terminal_definition.tooltip = "Selects how the final printable scope leaves filament retracted.";
    CreateTerminalRetraction::instance(orchestrator).register_exclusive_group(STEP_EXTRUSION_EDIT, terminal_definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::CreateRetractionPlugin
