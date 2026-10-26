///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#include "ToolChangeUnretract.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/ExtrusionScopeHelpers.hpp"
#include "libslic3r/Plugins/PrintingPlan/PrintingExtrusionScopeProperty.h"

namespace slic3r_api { namespace WipeTower { namespace ToolChangeUnretractPlugin {
namespace {

using ScopeEntity = PrintingEntity<PrintingExtrusionScopeProperty>;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    nullptr
};

/* Settings read by the unretraction rewrite, in the print and regional scopes. */
const raw_used_config_key k_used_keys[] = {
    {"retract_restart_wipe_toolchange", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"retract_length_toolchange", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"retract_restart_extra_toolchange", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"deretract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"retract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"filament_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"use_firmware_retraction", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"single_extruder_multi_material", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"solid_infill_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"support_material_speed", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"wipe_tower_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

/* Effective properties inherited while walking one scope content. */
struct EffectiveState
{
    coord_t z_offset = 0;
    std::optional<EPropertyAttributes> attributes;
};

/* First printable leaf of the tower content, with its flow and absolute Z. */
struct WipeLeaf
{
    MutableExtrusionEntity entity;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    double mm3_per_mm = 0.0;
    coord_t base_z = 0;
};

/* Built-in provider which rewrites the tower tool-change unretraction. */
class ToolChangeUnretract final : public PluginBase
{
public:
    static ToolChangeUnretract &instance(orchestrator_handle *orchestrator);
    explicit ToolChangeUnretract(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;

    // [group_idx][layer_group_idx][tool_group_idx]: the visit is a return of its tool.
    mutable std::vector<std::vector<std::vector<bool>>> m_return_flags;

    // Print default region config, and through its parents the default object and print values.
    mutable Config m_default_region_config;
};

/* Add two scaled values without allowing signed coordinate overflow. */
coord_t checked_add(coord_t lhs, coord_t rhs);

/* Resolve direct wipe-relevant properties over the inherited parent state. */
EffectiveState resolved_state(const ExtrusionEntity &entity, const EffectiveState &parent);

/* Convert the traversal's autonomous snapshot to this plugin's local state. */
EffectiveState traversal_state(const ExtrusionPropertyState &properties);

/* Describe the properties which every wipe scope traversal must inherit. */
ExtrusionPropertyStateDefinition wipe_state_definition();

/* Find the first printable leaf of a borrowed content tree. */
bool find_first_printable_leaf(MutableExtrusionEntity entity, coord_t print_z,
                               const EffectiveState &parent, WipeLeaf &leaf,
                               bool entity_already_resolved);

/* Resolve the first printable leaf and phase-relative base Z of one scope. */
bool target_first_leaf(const ScopeEntity &scope,
                       const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                       WipeLeaf &leaf, coord_t &phase_base_z);

/* Locate one semantic E-axis event inside a small ordered phase. */
MutableExtrusionEntity find_axis_event(MutableExtrusionEntity entity, raw_extrusion_role role);

/* Reverse line and arc segments without changing their represented geometry. */
std::vector<c_extrusion_segment> reversed_segments(
    const std::vector<c_extrusion_segment> &segments);

/* Express one leaf's local Z offsets in the coordinate system of a phase. */
std::vector<c_extrusion_segment> rebased_segments(const WipeLeaf &leaf, coord_t phase_base_z);

/* Truncate a segment path to at most length, interpolating the last point. */
std::vector<c_extrusion_segment> clip_path(const std::vector<c_extrusion_segment> &segments,
                                           distf_t length);

/* Repeat out-and-back trips over the followed leaf until the requested length is covered. */
std::vector<c_extrusion_segment> round_trip_path(
    const std::vector<c_extrusion_segment> &forward, distf_t requested_length);

/* Assign process attributes shared by the mechanical wipe leaves. */
void set_wipe_attributes(MutableExtrusionEntity entity, raw_extrusion_role role);

/* Resolve one speed option over a fallback base, keeping the base when it is missing or zero. */
double resolved_ratio_value(const Config &config, const char *key, double ratio);

/* Return the XY speed ceiling of the tower configuration, or wipe_tower_speed. */
double tower_speed_cap(const PrintingPlan &plan, const PrintingToolGroup &tool_group,
                       const PrintingExtrusion &tower_extrusion,
                       const Config &default_region_config, const Config &print_config);

/* Rewrite a borrowed tower scope's tool-change unretraction wipe in place; invalid geometry may return or throw, cancellation propagates, and no scope ownership is transferred. */
void edit_scope(const ScopeEntity &scope, const PrintingLayerGroup &layer,
                const std::vector<bool> &return_flags, const Config &print_config,
                const Config &default_region_config, const PrintingPlan &plan,
                const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                const plugin_run_context *run);

coord_t checked_add(const coord_t lhs, const coord_t rhs)
{
    if ((rhs > 0 && lhs > (std::numeric_limits<coord_t>::max)() - rhs) ||
        (rhs < 0 && lhs < (std::numeric_limits<coord_t>::min)() - rhs))
        throw std::overflow_error("A wipe Z coordinate exceeds coord_t.");
    return lhs + rhs;
}

EffectiveState resolved_state(const ExtrusionEntity &entity, const EffectiveState &parent)
{
    EffectiveState state = parent;
    if (const EPropertyZOffset *z_offset = entity.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        state.attributes = *attributes;
    return state;
}

EffectiveState traversal_state(const ExtrusionPropertyState &properties)
{
    EffectiveState state;
    if (const EPropertyZOffset *z_offset = properties.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes = properties.get(EPropertyAttributes::key))
        state.attributes = *attributes;
    return state;
}

ExtrusionPropertyStateDefinition wipe_state_definition()
{
    ExtrusionPropertyStateDefinition definition;
    definition.track(EPropertyZOffset::key);
    definition.track(EPropertyAttributes::key);
    return definition;
}

bool find_first_printable_leaf(MutableExtrusionEntity entity, const coord_t print_z,
                               const EffectiveState &parent, WipeLeaf &leaf,
                               const bool entity_already_resolved)
{
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);
    if (entity.segment_count() > 0) {
        if (!state.attributes)
            return false;
        const raw_extrusion_role role = state.attributes->extrusion_role();
        if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
            RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
            return false;
        leaf.entity = entity;
        leaf.role = role;
        leaf.mm3_per_mm = state.attributes->c_extrusion_property_attributes::mm3_per_mm;
        leaf.base_z = checked_add(print_z, state.z_offset);
        return true;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (find_first_printable_leaf(entity.child_mutable(child_idx), print_z, state, leaf, false))
            return true;
    return false;
}

bool target_first_leaf(const ScopeEntity &scope,
                       const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                       WipeLeaf &leaf, coord_t &phase_base_z)
{
    const LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope ordered(scope.entity, key);
    const EffectiveState scope_state = traversal_state(scope.effective_properties);
    phase_base_z = checked_add(scope.layer_group.print_z(), scope_state.z_offset);
    MutableExtrusionEntity content = ordered.content();
    return find_first_printable_leaf(content, scope.layer_group.print_z(), scope_state, leaf,
                                     content.handle() == ordered.root().handle());
}

MutableExtrusionEntity find_axis_event(MutableExtrusionEntity entity,
                                       const raw_extrusion_role role)
{
    if (!entity.valid())
        return MutableExtrusionEntity();
    const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key);
    if (attributes != nullptr &&
        RAW_EXTRUSION_ROLE_HAS(attributes->extrusion_role(), role) &&
        entity.get(EPropertyExtrusionAxis::key) != nullptr)
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        MutableExtrusionEntity found = find_axis_event(entity.child_mutable(child_idx), role);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

std::vector<c_extrusion_segment> reversed_segments(
    const std::vector<c_extrusion_segment> &segments)
{
    std::vector<c_extrusion_segment> reversed;
    reversed.reserve(segments.size());
    for (std::vector<c_extrusion_segment>::const_reverse_iterator it = segments.rbegin();
         it != segments.rend(); ++it) {
        c_extrusion_segment segment = *it;
        std::swap(segment.point_a, segment.point_b);
        std::swap(segment.z_offset_a, segment.z_offset_b);
        if (segment.orientation == RAW_EXTRUSION_ARC_ORIENTATION_CW)
            segment.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CCW;
        else if (segment.orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW)
            segment.orientation = RAW_EXTRUSION_ARC_ORIENTATION_CW;
        reversed.push_back(segment);
    }
    return reversed;
}

std::vector<c_extrusion_segment> rebased_segments(const WipeLeaf &leaf,
                                                  const coord_t phase_base_z)
{
    std::vector<c_extrusion_segment> result = leaf.entity.segments();
    const int64_t shift = int64_t(leaf.base_z) - int64_t(phase_base_z);
    for (c_extrusion_segment &segment : result) {
        const int64_t z_a = int64_t(segment.z_offset_a) + shift;
        const int64_t z_b = int64_t(segment.z_offset_b) + shift;
        if (z_a < int64_t((std::numeric_limits<coord_t>::min)()) ||
            z_a > int64_t((std::numeric_limits<coord_t>::max)()) ||
            z_b < int64_t((std::numeric_limits<coord_t>::min)()) ||
            z_b > int64_t((std::numeric_limits<coord_t>::max)()))
            throw std::overflow_error("A rebased wipe Z offset exceeds coord_t.");
        segment.z_offset_a = coord_t(z_a);
        segment.z_offset_b = coord_t(z_b);
    }
    return result;
}

/* Truncate a segment path to at most length, interpolating the last point. */
std::vector<c_extrusion_segment> clip_path(const std::vector<c_extrusion_segment> &segments,
                                           const distf_t length)
{
    std::vector<c_extrusion_segment> clipped;
    if (length <= distf_t(SCALED_EPSILON))
        return clipped;
    distf_t remaining = length;
    for (const c_extrusion_segment &segment : segments) {
        const distf_t segment_length = c_point_distance_to(segment.point_a, segment.point_b);
        if (segment_length <= distf_t(SCALED_EPSILON))
            continue;
        if (remaining >= segment_length - distf_t(SCALED_EPSILON)) {
            clipped.push_back(segment);
            remaining -= segment_length;
            continue;
        }
        // An interpolated endpoint keeps the segment start's Z offset; tower leaves carry a
        // constant offset, so no visible step is introduced.
        const double ratio = double(remaining) / double(segment_length);
        c_extrusion_segment part = segment;
        part.point_b = c_point{
            coord_t(std::llround(double(segment.point_a.x) +
                                 (double(segment.point_b.x) - double(segment.point_a.x)) * ratio)),
            coord_t(std::llround(double(segment.point_a.y) +
                                 (double(segment.point_b.y) - double(segment.point_a.y)) * ratio))};
        part.z_offset_b = segment.z_offset_a;
        clipped.push_back(part);
        remaining = 0.;
        break;
    }
    return clipped;
}

std::vector<c_extrusion_segment> round_trip_path(
    const std::vector<c_extrusion_segment> &forward, const distf_t requested_length)
{
    if (forward.empty() || requested_length <= distf_t(SCALED_EPSILON))
        return {};

    // Out and back along the leaf start, total length requested_length, ending on the leaf's
    // first point: out for half the length, then the same way back. A leaf shorter than half
    // the length is traversed by whole out-and-back trips plus a final partial one, so a closed
    // leaf behaves like an open one and the wipe density stays at the planned value.
    distf_t forward_length = 0.0;
    for (const c_extrusion_segment &segment : forward)
        forward_length += c_point_distance_to(segment.point_a, segment.point_b);
    if (forward_length <= distf_t(SCALED_EPSILON))
        return {};

    std::vector<c_extrusion_segment> path;
    const distf_t half = requested_length / 2.;
    if (forward_length >= half - distf_t(SCALED_EPSILON)) {
        path = clip_path(forward, half);
        const std::vector<c_extrusion_segment> back = reversed_segments(path);
        path.insert(path.end(), back.begin(), back.end());
        return path;
    }

    const size_t trips = size_t(std::floor(double(requested_length) /
                                            double(2. * forward_length)));
    for (size_t trip = 0; trip < trips; ++trip) {
        path.insert(path.end(), forward.begin(), forward.end());
        const std::vector<c_extrusion_segment> back = reversed_segments(forward);
        path.insert(path.end(), back.begin(), back.end());
    }
    const distf_t remaining =
        requested_length - double(trips) * 2. * forward_length;
    if (remaining > distf_t(SCALED_EPSILON)) {
        const std::vector<c_extrusion_segment> out = clip_path(forward, remaining / 2.);
        path.insert(path.end(), out.begin(), out.end());
        const std::vector<c_extrusion_segment> back = reversed_segments(out);
        path.insert(path.end(), back.begin(), back.end());
    }
    return path;
}

void set_wipe_attributes(MutableExtrusionEntity entity, const raw_extrusion_role role)
{
    entity.get_or_add(EPropertyAttributes::key)
        .extrusion_role(role)
        .mm3_per_mm(0.0)
        .width(0.f)
        .height(0.f)
        .no_seam_enabled(false);
}

double resolved_ratio_value(const Config &config, const char *key, const double ratio)
{
    // Mirror DefaultSpeed: a percentage is taken over the fallback base and a missing, disabled
    // or zero option keeps that base.
    const double value = config.effective_float_or_percent_or_default(key, ratio, ratio);
    return value > 0.0 ? value : ratio;
}

double tower_speed_cap(const PrintingPlan &plan, const PrintingToolGroup &tool_group,
                       const PrintingExtrusion &tower_extrusion,
                       const Config &default_region_config, const Config &print_config)
{
    // The manager names the shared tower exactly "Wipe tower"; an instance tower appends its
    // owner. The shared tower resolves the print defaults, an instance tower the object printed
    // in the same visit. Both chains fall back to wipe_tower_speed when nothing resolves.
    Config region_config = default_region_config;
    const std::string tower_name = plan.object_instance(
        tower_extrusion.printing_catalog_id()).name();
    if (tower_name != "Wipe tower" && !tower_name.empty())
        for (uint32_t idx = 0; idx < tool_group.extrusion_count(); ++idx) {
            const LayerRegionIsland island = tool_group.extrusion(idx).region_island();
            if (!island.valid() || island.region_count() == 0)
                continue;
            region_config = island.region(0).print_region().config();
            break;
        }

    // Resolve the same percentage chain as DefaultSpeed. A pure percentage without an absolute
    // base resolves to zero, and the whole cap then falls back to wipe_tower_speed.
    const double solid = resolved_ratio_value(region_config, "solid_infill_speed", 0.0);
    const double infill = resolved_ratio_value(region_config, "infill_speed", solid);
    const double support = resolved_ratio_value(region_config, "support_material_speed", 0.0);
    const double cap = std::max(infill, support);
    return cap > 0.0 ? cap : print_config.float_or_default("wipe_tower_speed", 0.0);
}

void edit_scope(const ScopeEntity &scope, const PrintingLayerGroup &layer,
                const std::vector<bool> &return_flags, const Config &print_config,
                const Config &default_region_config, const PrintingPlan &plan,
                const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
                const plugin_run_context *run)
{
    throw_if_cancelled(run);
    const LayerExtrusionEdit::ExtrusionScope::OrderedExtrusionScope ordered(scope.entity, key);
    if (!ordered.has_incoming_toolchange() || !ordered.before().valid())
        return;
    const extruder_index_t tool = scope.tool_group.extruder_id();
    if (tool < 0)
        return;

    // The visit flag was cached by setup; locate this scope's visit in its layer.
    uint32_t visit_index = 0;
    while (visit_index < layer.tool_group_count() &&
           layer.tool_group(visit_index).handle() != scope.tool_group.handle())
        ++visit_index;
    if (visit_index >= return_flags.size() || !return_flags[visit_index])
        return;

    // The ABI returns percent options normalized (20% -> 0.2).
    const double pct = print_config.vector_percent_or_default(
        "retract_restart_wipe_toolchange", uint32_t(tool), 0.0);
    if (!(pct > 0.0))
        return;

    WipeLeaf leaf;
    coord_t phase_base_z = 0;
    if (!target_first_leaf(scope, key, leaf, phase_base_z) ||
        !RAW_EXTRUSION_ROLE_HAS(leaf.role, RAW_EXTRUSION_ROLE_WIPE_TOWER))
        return;

    MutableExtrusionEntity unretract = find_axis_event(
        ordered.before(), RAW_EXTRUSION_ROLE_UNRETRACT);
    if (!unretract.valid() || unretract.segment_count() != 0)
        return;
    const EPropertyExtrusionAxis *axis = unretract.get(EPropertyExtrusionAxis::key);
    if (axis == nullptr || axis->operation != C_EXTRUSION_AXIS_OPERATION_UNRETRACT ||
        axis->toolchange == 0 || axis->value != 0.0)
        return;

    const double restart_extra = axis->restart_extra;
    const double retract_length = print_config.vector_float_or_default(
        "retract_length_toolchange", uint32_t(tool), 0.0);
    const double filament_diameter = print_config.vector_float_or_default(
        "filament_diameter", uint32_t(tool), 0.0);
    if (!(retract_length >= 0.0) || !(filament_diameter > 0.0) ||
        !(leaf.mm3_per_mm > 0.0) || leaf.entity.segment_count() == 0)
        return;

    // Wiped filament and the path length that carries it at the followed leaf's flow.
    const double wiped_filament = pct * retract_length + std::max(0.0, restart_extra);
    if (!(wiped_filament > 0.0))
        return;
    const double filament_area = PI * filament_diameter * filament_diameter * 0.25;
    const distf_t requested_length = scale_d(
        wiped_filament * filament_area / leaf.mm3_per_mm);
    if (requested_length <= distf_t(SCALED_EPSILON))
        return;

    const double deretract_speed = print_config.vector_float_or_default(
        "deretract_speed", uint32_t(tool), 0.0);
    const double retract_speed = print_config.vector_float_or_default(
        "retract_speed", uint32_t(tool), 0.0);
    const double filament_speed = deretract_speed > 0.0 ? deretract_speed : retract_speed;
    const double cap = tower_speed_cap(
        plan, scope.tool_group, scope.printing_extrusion, default_region_config, print_config);
    double speed = cap;
    if (filament_speed > 0.0)
        speed = std::min(unscaled(requested_length) * filament_speed / wiped_filament, cap);
    if (!(speed > 0.0) || !std::isfinite(speed))
        return;

    const std::vector<c_extrusion_segment> forward = rebased_segments(leaf, phase_base_z);
    const std::vector<c_extrusion_segment> path = round_trip_path(forward, requested_length);
    if (path.empty())
        return;

    // Keep whatever preceded the unretraction, then the partial in place and the wipe leaf in
    // its place. With a full wipe the event itself becomes the geometric wipe leaf.
    const double partial_filament = (1.0 - pct) * retract_length;
    MutableExtrusionEntity wipe;
    if (partial_filament > 0.0) {
        EPropertyExtrusionAxis *mutable_axis =
            unretract.get_mutable(EPropertyExtrusionAxis::key);
        if (mutable_axis == nullptr)
            return;
        mutable_axis->unretract_partial(partial_filament, true);
        wipe = unretract.emplace_ordered_leaf(
            OrderedLeafPosition::After, ExistingPropertyPlacement::MoveWithExistingContent);
        if (!wipe.valid())
            throw std::runtime_error("Unable to append the tool-change unretraction wipe.");
    } else {
        wipe = unretract;
    }

    if (!wipe.set_segments(path))
        throw std::runtime_error("Unable to define the tool-change unretraction wipe path.");
    set_wipe_attributes(
        wipe, RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_UNRETRACT);
    wipe.get_or_add(EPropertySpeed::key)
        .speed(float(speed))
        .acceleration(-1.f)
        .pressure_advance(-1.f)
        .fan_speed(-1.f);
    wipe.get_or_add(EPropertyExtrusionAxis::key).unretract(restart_extra, true);
}

ToolChangeUnretract &ToolChangeUnretract::instance(orchestrator_handle *orchestrator)
{
    // The C ABI stores a non-owning pointer, so keep the plugin alive for the orchestrator.
    static ToolChangeUnretract plugin(orchestrator);
    return plugin;
}

ToolChangeUnretract::ToolChangeUnretract(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_scope_property(printing_extrusion_scope_property_key(orchestrator))
{}

const char *ToolChangeUnretract::id_impl() const noexcept
{
    return "wipe_tower.toolchange_unretract.default";
}

const char *ToolChangeUnretract::name_impl() const noexcept
{
    return "Tool-change unretraction wipe";
}

const char *ToolChangeUnretract::description_impl() const noexcept
{
    return "Wipes the tool-change unretraction over the start of the wipe tower path.";
}

const char *ToolChangeUnretract::exclusive_group_impl() const noexcept
{
    return "";
}

slicing_step_t ToolChangeUnretract::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *ToolChangeUnretract::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t ToolChangeUnretract::priority_impl() const noexcept
{
    // After CreateRetraction (-90) and CreateRetractionWipe (-80), before the entry state and
    // the travel providers.
    return -70;
}

int32_t ToolChangeUnretract::used_config_keys(raw_used_config_key *keys) const noexcept
{
    const int32_t count = int32_t(sizeof(k_used_keys) / sizeof(k_used_keys[0]));
    if (keys != nullptr)
        std::copy(k_used_keys, k_used_keys + count, keys);
    return count;
}

const char *ToolChangeUnretract::progress_message_format_impl() const noexcept
{
    return "Wiping tool-change unretractions: %u / %u layers";
}

void ToolChangeUnretract::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    // One sequential prefix scan builds the tool history before the parallel layer runs; the
    // change detection mirrors the wipe tower service: an unresolved or unconfigured visit is
    // skipped without resetting the previous concrete tool.
    m_return_flags.clear();
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        return;
    const Print print(ctx->print);
    const Config print_config = print.config();
    const uint32_t tool_count = print_config.size("nozzle_diameter", 0);
    m_default_region_config = print.default_region_config();

    const ConstPrintingPlan plan(ctx->plan);
    std::vector<bool> already_left(tool_count, false);
    extruder_index_t previous = EXTRUDER_ID_UNKNOWN;
    m_return_flags.resize(plan.group_count());
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        m_return_flags[group_idx].resize(group.layer_group_count());
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            std::vector<bool> flags(layer.tool_group_count(), false);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const extruder_index_t tool = layer.tool_group(tool_idx).extruder_id();
                if (tool < 0 || uint32_t(tool) >= tool_count)
                    continue;
                if (previous != EXTRUDER_ID_UNKNOWN && previous != tool) {
                    already_left[uint32_t(previous)] = true;
                    if (already_left[uint32_t(tool)])
                        flags[tool_idx] = true;
                }
                previous = tool;
            }
            m_return_flags[group_idx][layer_idx] = std::move(flags);
        }
    }
}

void ToolChangeUnretract::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (plugin_ctx_as_layer_extrusion_edition(run_ctx) != nullptr)
        progress().add_max(1);
}

void ToolChangeUnretract::run_impl(const plugin_run_context *run_ctx) const
{
    // Runs may be concurrent. Only the cached flags of this layer are read, and only this
    // layer's scope phases are edited.
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        ctx->layer_group == nullptr)
        return;
    if (ctx->group_idx >= m_return_flags.size() ||
        ctx->layer_group_idx >= m_return_flags[ctx->group_idx].size())
        return;

    const Config print_config = Print(ctx->print).config();
    // Firmware retraction cannot restore a partial amount, and the SEMM swap has no tool-change
    // unretraction at this boundary.
    if (print_config.bool_or_default("single_extruder_multi_material", false) ||
        print_config.bool_or_default("use_firmware_retraction", false))
        return;

    const std::vector<bool> &return_flags = m_return_flags[ctx->group_idx][ctx->layer_group_idx];
    PrintingLayerGroup layer(ctx->layer_group);
    const PrintingPlan plan(ctx->plan);
    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        wipe_state_definition(),
        [this, &layer, &return_flags, &print_config, &plan, run_ctx](
            ScopeEntity *, ScopeEntity *next) {
            if (next != nullptr)
                edit_scope(*next, layer, return_flags, print_config, m_default_region_config,
                           plan, m_scope_property, run_ctx);
        },
        MatchingEntityDescendants::Skip);
    traversal.process(layer);
    progress().increment();
}

} // namespace

void register_tool_change_unretract_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, ToolChangeUnretract::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::WipeTower::ToolChangeUnretractPlugin
