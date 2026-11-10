///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CreateRetractionWipe.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ExtrusionScopeHelpers.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingEntityPropertyTraversal.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "AvoidCrossing/LayerInputCache.hpp"
#include "AvoidCrossing/PerimeterCrossing.hpp"

/*
Retraction-wipe implementation
==============================

CreateRetractionWipe distributes the part of a retraction remaining after the
stationary amount configured by retract_before_wipe over recently printed
motion. It runs in the ordered layer-edit step after retraction requests have
been created.

ExtrusionScopeHelpers.hpp defines a scope as printable content with optional
transition phases: travel moves to the scope, before holds its approach events,
content holds printable motion, and after holds departure events. In each
adjacent pair visited in output order, source means the earlier scope and
target means the later scope.

Incoming work may turn the target's unretract request into an inside-start
approach. Outgoing work may keep part of the source's retraction stationary
and distribute the rest over a wipe, with a final stationary request if the
wipe is too short. When the last printed leaf is not a loop, the wipe walks
source content backwards and stops before a bridge, a discontinuity or a leaf
without role attributes; only that connected tail is kept.
A final loop instead continues forward
from its seam without visiting older leaves. A loop parent retains its ordered
fragments, whose lengths together define one turn. Segment vectors preserve lines,
arcs and endpoint Z offsets while the wipe is assembled and clipped.
Inside-start and inside-end movements use the complete loop's orientation and
hole flag to select the material side, while leaf endpoints locate the nozzle.
An outgoing fragmented loop stops before its first bridge fragment. Such a
partial turn is an open forward path, without loop repetition or turn rounding;
a terminal bridge still prevents the outgoing wipe entirely.
External loop prefixes retain the extra perimeter wipe distance, but only a
complete turn supplies the orientation needed for an inside-end movement.
No source geometry or neighbour pointer is retained after the callback.

When wipe_only_crossing is enabled, the target's region identifies the layer
and its tool group selects the nozzle radius and material areas. The straight
gap is tested after removing the target instance's XY shift from both ends.
Global skirt and brim paths have no instance shift and stay in platter XY.
This is only a gate for adding a wipe; it never routes the gap around objects.

PerimeterCrossing prepares two versions of the target's filled areas. For an
object it clips to the selected tool's areas, insets one version by 1.5 nozzle
radii, and removes top surfaces from both. For support it expands the support
layer's own slices by 1.5 radii, without tool clipping or top removal; it does
not collect object layers already printed at that height. Both versions are
simplified with the nozzle radius. The wipe gate queries the offset version's
boundaries, not extrusion centre lines; a false result does not prove that
the gap lies inside material.

Each run owns copied layer inputs and a crossing cache per target layer and
tool. LayerInputCache resolves public handles and copies geometry using the
context's persistent plugin storage for temporary API allocations. That storage
is shared by parallel calls; the returned geometry copies own their buffers
independently of it. The plan and run context are borrowed for the
run; inputs outlive the crossing caches that refer to them. No cache is shared
between workers or refreshed after print edits. Cancellation propagates.

    run_impl: visit adjacent printable scopes in output order
    |-- add_inside_start: move an eligible unretract into an approach to the seam
    `-- add_outgoing_wipe: distribute the source retract over its printed tail
        |-- source_wipe_path: collect the reversed connected tail, or a forward final loop
        |   `-- append_forward_loop: join ordered fragments and sum one complete turn
        |-- WipeCrossingChecker::crosses: gate wipes on the direct gap if requested
        |   |-- LayerInputCache::get: resolve and copy the target layer
        |   |-- PerimeterCrossing::prepare: prepare target areas for the tool
        |   `-- PerimeterCrossing::can_cross_perimeter: test the offset boundaries
        `-- fit_wipe_length: repeat loops before clipping and assigning wipe motion
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace CreateRetractionWipePlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_create_retraction_wipe_plugin(orchestrator_handle *orchestrator);

namespace {

using ScopeEntity = PrintingEntity<PrintingExtrusionScopeProperty>;
namespace AC = AvoidCrossing;

const char *const k_dependencies[] = {
    "layer_extrusion_edit.transition_scope.default",
    nullptr
};

/* Effective properties inherited while walking one printable scope. */
struct EffectiveState
{
    coord_t z_offset = 0;
    std::optional<EPropertyAttributes> attributes;
    std::optional<EPropertyPerimeter> perimeter;
};

/* Metadata for the final or first printable leaf of one scope. */
struct WipeLeaf
{
    MutableExtrusionEntity entity;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_NONE;
    uint16_t perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_NONE;
    coord_t base_z = 0;
    bool loop = false;
};

/* Arc-preserving path from the reversed open tail or a loop's usable forward turn. */
struct WipePath
{
    std::vector<c_extrusion_segment> segments;
    WipeLeaf source;
    bool has_source = false;
    bool stopped = false;
    distf_t loop_length = 0.;
    // True only when the collected geometry contains the whole loop, allowing repetition, turn rounding and an inside end.
    bool complete_turn = false;
};

/* Own the worker's layer-input cache and crossing caches indexed by layer and tool. */
class WipeCrossingChecker
{
public:
    // Copy the settings view and borrow the plan and run context for this worker; own all copied crossing inputs.
    WipeCrossingChecker(const Config &print_config,
                        const PrintingPlan &plan,
                        const plugin_run_context *run_ctx);

    // Test the direct gap in target coordinates against offset boundaries; missing target regions, a null layer handle or empty leaves return false.
    // Copy inputs on first use; an absent layer throws, and exceptions from cancellation or geometry preparation propagate.
    bool crosses(const WipeLeaf &source, const ScopeEntity &target_scope,
                 const WipeLeaf &target);

private:
    /* Identify one crossing cache by its target layer and active extruder. */
    struct Key
    {
        const layer_handle *layer = nullptr;
        extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;

        // Order public layer handles and tools for worker-local cache lookup.
        bool operator<(const Key &rhs) const
        {
            return std::less<const layer_handle *>()(layer, rhs.layer) ||
                (layer == rhs.layer && extruder_id < rhs.extruder_id);
        }
    };

    Config m_print_config;
    const PrintingPlan &m_plan;
    const plugin_run_context *m_run_ctx = nullptr;
    AC::LayerInputCache m_inputs;
    std::map<Key, AC::PerimeterCrossing> m_crossings;
};

/* Built-in provider which converts semantic retraction into wipe motion. */
class CreateRetractionWipe final : public PluginBase
{
public:
    static CreateRetractionWipe &instance(orchestrator_handle *orchestrator);
    explicit CreateRetractionWipe(orchestrator_handle *orchestrator);

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
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<PrintingExtrusionScopeProperty> m_scope_property;
};

/* Add two scaled values without allowing signed coordinate overflow. */
coord_t checked_add(coord_t lhs, coord_t rhs);

/* Resolve direct wipe-relevant properties over the inherited parent state. */
EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent);

/* Convert the traversal's autonomous snapshot to this plugin's local state. */
EffectiveState traversal_state(const ExtrusionPropertyState &properties);

/* Describe the properties which every wipe scope traversal must inherit. */
ExtrusionPropertyStateDefinition wipe_state_definition();

/* Reverse line and arc segments without changing their represented geometry. */
std::vector<c_extrusion_segment> reversed_segments(
    const std::vector<c_extrusion_segment> &segments);

/* Express one leaf's local Z offsets in the coordinate system of a phase. */
std::vector<c_extrusion_segment> rebased_segments(
    const WipeLeaf &leaf, coord_t phase_base_z);

/* Append ordered loop leaves and sum their true lengths, optionally stopping geometry before the first bridge while retaining the last leaf's metadata. Missing attributes, process geometry or an out-of-range Z offset throws. */
void append_forward_loop(MutableExtrusionEntity entity, coord_t print_z,
                         coord_t phase_base_z, const EffectiveState &state,
                         WipePath &path, bool stop_before_bridge = false);

/* Walk source content backwards, collecting a resolved loop's forward prefix before a bridge or the connected reversed open tail; a terminal bridge prevents wiping. */
void append_reverse_wipe_path(MutableExtrusionEntity entity,
                              coord_t print_z,
                              coord_t phase_base_z,
                              const EffectiveState &parent,
                              WipePath &path,
                              bool entity_already_resolved = false);

/* Build the source wipe path without collecting a separate leaf history. */
WipePath source_wipe_path(
    const ScopeEntity &scope,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key);

/* Find the first printable leaf and optionally copy its complete loop once at the first composed loop node, relative to phase_base_z. Invalid process geometry in a collected loop throws. */
bool find_first_printable_leaf(MutableExtrusionEntity entity,
                               coord_t print_z,
                               coord_t phase_base_z,
                               const EffectiveState &parent,
                               WipeLeaf &leaf,
                               bool entity_already_resolved = false,
                               std::vector<c_extrusion_segment> *loop_segments = nullptr);

/* Resolve the first leaf and phase-relative base Z of one target scope, optionally copying the complete loop for an inside approach. */
bool target_first_leaf(
    const ScopeEntity &scope,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    WipeLeaf &leaf,
    coord_t &phase_base_z,
    std::vector<c_extrusion_segment> *loop_segments = nullptr);

/* Locate one semantic E-axis event inside a small ordered phase. */
MutableExtrusionEntity find_axis_event(MutableExtrusionEntity entity,
                                       raw_extrusion_role role);

/* Repeat a complete loop enough times for true-length clipping. */
std::vector<c_extrusion_segment> fit_wipe_length(
    const std::vector<c_extrusion_segment> &source,
    bool loop,
    distf_t requested_length);

/* Compute a point on the material side of an external perimeter seam using the complete loop and its hole flag; degenerate geometry returns no point. */
std::optional<c_point> loop_inside_point(const WipeLeaf &leaf,
                                       const std::vector<c_extrusion_segment> &segments,
                                       coord_t depth);

/* Extend a non-returning loop wipe toward material, using the complete source loop for its orientation; a failed geometry assignment throws. */
void append_inside_end(MutableExtrusionEntity wipe,
                       const WipeLeaf &source,
                       const std::vector<c_extrusion_segment> &source_segments,
                       coord_t depth);

/* Add an inside-to-seam approach for an external loop, collecting its complete geometry only after the setting and unretract allow it. Ineligible or missing geometry returns unchanged; an out-of-range Z offset or a failed segment assignment throws. The borrowed target keeps ownership. */
void add_inside_start(
    const ScopeEntity &target,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key);

/* Assign process attributes shared by mechanical and retracting wipe leaves. */
void set_wipe_attributes(MutableExtrusionEntity entity,
                         raw_extrusion_role role);

/* Return the configured wipe speed or its travel-speed fallback. */
double wipe_speed(const Config &print_config, extruder_index_t extruder_id);

/* Convert an eligible source Retract into initial E motion, a wipe and residual E; ineligible transitions return unchanged. The scope tree is changed in place without transferring ownership; path collection, crossing checks and failed geometry edits may throw, as may a lost Retract. */
void add_outgoing_wipe(
    const ScopeEntity &source,
    const ScopeEntity *target,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    WipeCrossingChecker &crossing_checker);

coord_t checked_add(const coord_t lhs, const coord_t rhs)
{
    if ((rhs > 0 && lhs > (std::numeric_limits<coord_t>::max)() - rhs) ||
        (rhs < 0 && lhs < (std::numeric_limits<coord_t>::min)() - rhs))
        throw std::overflow_error("A wipe Z coordinate exceeds coord_t.");
    return lhs + rhs;
}

EffectiveState resolved_state(const ExtrusionEntity &entity,
                              const EffectiveState &parent)
{
    EffectiveState state = parent;
    if (const EPropertyZOffset *z_offset = entity.get(EPropertyZOffset::key))
        state.z_offset = z_offset->get();
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        state.attributes = *attributes;
    if (const EPropertyPerimeter *perimeter = entity.get(EPropertyPerimeter::key))
        state.perimeter = *perimeter;
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
    if (const EPropertyPerimeter *perimeter = properties.get(EPropertyPerimeter::key))
        state.perimeter = *perimeter;
    return state;
}

ExtrusionPropertyStateDefinition wipe_state_definition()
{
    ExtrusionPropertyStateDefinition definition;
    definition.track(EPropertyZOffset::key);
    definition.track(EPropertyAttributes::key);
    definition.track(EPropertyPerimeter::key);
    return definition;
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

std::vector<c_extrusion_segment> rebased_segments(
    const WipeLeaf &leaf, const coord_t phase_base_z)
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

void append_forward_loop(MutableExtrusionEntity entity, const coord_t print_z,
                         const coord_t phase_base_z, const EffectiveState &state,
                         WipePath &path, const bool stop_before_bridge)
{
    for (uint32_t idx = 0; idx < entity.child_count(); ++idx) {
        MutableExtrusionEntity child = entity.child_mutable(idx);
        append_forward_loop(child, print_z, phase_base_z,
                            resolved_state(child.readonly(), state), path, stop_before_bridge);
    }
    if (entity.segment_count() == 0)
        return;
    if (!state.attributes)
        throw std::runtime_error("A printable loop fragment has no role attributes.");
    const raw_extrusion_role role = state.attributes->extrusion_role();
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
        RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
        throw std::runtime_error("Printable scope content contains process geometry.");
    const uint16_t flags = state.perimeter ? state.perimeter->perimeter_flags() :
        C_EXTRUSION_PERIMETER_FLAG_NONE;
    WipeLeaf leaf{entity, role, flags,
                  checked_add(print_z, state.z_offset), true};
    // A bridge cuts the usable forward path, but later leaves still identify the nozzle position.
    if (stop_before_bridge && RAW_EXTRUSION_ROLE_IS_BRIDGE(role))
        path.stopped = true;
    if (!path.stopped) {
        std::vector<c_extrusion_segment> segments = rebased_segments(leaf, phase_base_z);
        path.segments.insert(path.segments.end(), segments.begin(), segments.end());
        path.loop_length += entity.local_length();
    }
    path.source = leaf;
    path.has_source = true;
}

void append_reverse_wipe_path(
    MutableExtrusionEntity entity,
    const coord_t print_z,
    const coord_t phase_base_z,
    const EffectiveState &parent,
    WipePath &path,
    const bool entity_already_resolved)
{
    if (path.stopped)
        return;
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);

    // A loop node with several children is one composed turn; inherited LOOP flags must not make each fragment a whole loop.
    // Single-child wrappers continue to the leaf or the node holding the fragments.
    if (!path.has_source && entity.child_count() > 1 && state.perimeter &&
        (state.perimeter->perimeter_flags() & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0) {
        append_forward_loop(entity, print_z, phase_base_z, state, path, true);
        // Only the immediately preceding printable leaf decides whether a bridge prevents wiping.
        if (path.has_source && RAW_EXTRUSION_ROLE_IS_BRIDGE(path.source.role)) {
            path.has_source = false;
            path.segments.clear();
        }
        path.complete_turn = path.has_source && !path.stopped;
        path.stopped = true;
        return;
    }

    // Reverse child order mirrors the machine's immediately preceding path.
    for (uint32_t child_idx = entity.child_count(); child_idx > 0; --child_idx) {
        append_reverse_wipe_path(entity.child_mutable(child_idx - 1), print_z,
                                 phase_base_z, state, path);
        if (path.stopped)
            return;
    }
    if (entity.segment_count() == 0)
        return;
    if (!state.attributes) {
        path.stopped = true;
        return;
    }

    const raw_extrusion_role role = state.attributes->extrusion_role();
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
        RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
        throw std::runtime_error("Printable scope content contains process geometry.");
    if (RAW_EXTRUSION_ROLE_IS_BRIDGE(role)) {
        path.stopped = true;
        return;
    }

    const uint16_t perimeter_flags = state.perimeter ?
        state.perimeter->perimeter_flags() : C_EXTRUSION_PERIMETER_FLAG_NONE;
    WipeLeaf leaf{
        entity,
        role,
        perimeter_flags,
        checked_add(print_z, state.z_offset),
        (perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0
    };
    std::vector<c_extrusion_segment> leaf_segments =
        rebased_segments(leaf, phase_base_z);
    if (leaf_segments.empty())
        return;

    // A final loop continues forward from its seam; older leaves are irrelevant.
    if (!path.has_source && leaf.loop) {
        path.source = leaf;
        path.has_source = true;
        path.segments = std::move(leaf_segments);
        path.loop_length = entity.local_length();
        path.complete_turn = true;
        path.stopped = true;
        return;
    }

    std::vector<c_extrusion_segment> reversed = reversed_segments(leaf_segments);
    if (!path.segments.empty() &&
        c_point_distance_to(path.segments.back().point_b,
                            reversed.front().point_a) >= SCALED_EPSILON) {
        path.stopped = true;
        return;
    }
    if (!path.has_source) {
        path.source = leaf;
        path.has_source = true;
    }
    path.segments.insert(path.segments.end(), reversed.begin(), reversed.end());
}

WipePath source_wipe_path(
    const ScopeEntity &scope,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key)
{
    const ExtrusionScope::OrderedExtrusionScope ordered(scope.entity, key);
    const EffectiveState scope_state = traversal_state(scope.effective_properties);
    const coord_t phase_base_z = checked_add(
        scope.layer_group.print_z(), scope_state.z_offset);
    MutableExtrusionEntity content = ordered.content();
    WipePath path;
    append_reverse_wipe_path(content, scope.layer_group.print_z(), phase_base_z,
                             scope_state, path,
                             content.handle() == ordered.root().handle());
    return path;
}

bool find_first_printable_leaf(
    MutableExtrusionEntity entity,
    const coord_t print_z,
    const coord_t phase_base_z,
    const EffectiveState &parent,
    WipeLeaf &leaf,
    const bool entity_already_resolved,
    std::vector<c_extrusion_segment> *loop_segments)
{
    const EffectiveState state = entity_already_resolved ?
        parent : resolved_state(entity.readonly(), parent);
    // Collect at the first composed loop node on descent, not again at its descendants or ancestors.
    std::vector<c_extrusion_segment> *child_loop_segments = loop_segments;
    if (loop_segments != nullptr && entity.child_count() > 1 && state.perimeter &&
        (state.perimeter->perimeter_flags() & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0) {
        WipePath loop;
        append_forward_loop(entity, print_z, phase_base_z, state, loop);
        *loop_segments = std::move(loop.segments);
        child_loop_segments = nullptr;
    }
    if (entity.segment_count() > 0) {
        if (!state.attributes)
            return false;
        const raw_extrusion_role role = state.attributes->extrusion_role();
        if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
            RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
            return false;
        const uint16_t perimeter_flags = state.perimeter ?
            state.perimeter->perimeter_flags() : C_EXTRUSION_PERIMETER_FLAG_NONE;
        leaf = WipeLeaf{
            entity,
            role,
            perimeter_flags,
            checked_add(print_z, state.z_offset),
            (perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_LOOP) != 0
        };
        if (loop_segments != nullptr && leaf.loop)
            *loop_segments = entity.segments();
        return true;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (find_first_printable_leaf(entity.child_mutable(child_idx), print_z,
                                      phase_base_z, state, leaf, false, child_loop_segments)) {
            return true;
        }
    return false;
}

bool target_first_leaf(
    const ScopeEntity &scope,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    WipeLeaf &leaf,
    coord_t &phase_base_z,
    std::vector<c_extrusion_segment> *loop_segments)
{
    const ExtrusionScope::OrderedExtrusionScope ordered(scope.entity, key);
    const EffectiveState scope_state = traversal_state(scope.effective_properties);
    phase_base_z = checked_add(scope.layer_group.print_z(), scope_state.z_offset);
    MutableExtrusionEntity content = ordered.content();
    return find_first_printable_leaf(content, scope.layer_group.print_z(),
                                     phase_base_z, scope_state, leaf,
                                     content.handle() == ordered.root().handle(), loop_segments);
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
        MutableExtrusionEntity found = find_axis_event(
            entity.child_mutable(child_idx), role);
        if (found.valid())
            return found;
    }
    return MutableExtrusionEntity();
}

std::vector<c_extrusion_segment> fit_wipe_length(
    const std::vector<c_extrusion_segment> &source,
    const bool loop,
    const distf_t requested_length)
{
    if (source.empty() || requested_length <= distf_t(SCALED_EPSILON))
        return {};
    if (!loop)
        return source;

    // Chord length is a conservative lower bound. Repeating by this bound
    // guarantees enough true arc length before the host performs exact clipping.
    distf_t one_turn_chord = 0.0;
    for (const c_extrusion_segment &segment : source)
        one_turn_chord += c_point_distance_to(segment.point_a, segment.point_b);
    if (one_turn_chord <= distf_t(SCALED_EPSILON))
        return {};

    std::vector<c_extrusion_segment> result;
    for (distf_t accumulated = 0.0; accumulated < requested_length;
         accumulated += one_turn_chord)
        result.insert(result.end(), source.begin(), source.end());
    return result;
}

std::optional<c_point> loop_inside_point(const WipeLeaf &leaf,
                                       const std::vector<c_extrusion_segment> &segments,
                                       const coord_t depth)
{
    if (!leaf.loop || depth <= 0 || segments.size() < 2)
        return std::nullopt;
    const c_point seam = segments.front().point_a;
    const c_point next = segments.front().point_b;
    const c_point previous = segments.back().point_a;

    const long double outgoing_x = static_cast<long double>(next.x) - seam.x;
    const long double outgoing_y = static_cast<long double>(next.y) - seam.y;
    const long double incoming_x = static_cast<long double>(seam.x) - previous.x;
    const long double incoming_y = static_cast<long double>(seam.y) - previous.y;
    const long double outgoing_length = std::sqrt(
        outgoing_x * outgoing_x + outgoing_y * outgoing_y);
    const long double incoming_length = std::sqrt(
        incoming_x * incoming_x + incoming_y * incoming_y);
    if (outgoing_length <= SCALED_EPSILON || incoming_length <= SCALED_EPSILON)
        return std::nullopt;

    long double signed_area_twice = 0.0L;
    for (const c_extrusion_segment &segment : segments)
        signed_area_twice += static_cast<long double>(segment.point_a.x) * segment.point_b.y -
                             static_cast<long double>(segment.point_b.x) * segment.point_a.y;
    const bool counter_clockwise = signed_area_twice > 0.0L;
    const bool hole =
        (leaf.perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_HOLE) != 0;
    const bool material_is_left = hole ? !counter_clockwise : counter_clockwise;
    const long double side = material_is_left ? 1.0L : -1.0L;
    long double normal_x = side *
        (-outgoing_y / outgoing_length - incoming_y / incoming_length);
    long double normal_y = side *
        (outgoing_x / outgoing_length + incoming_x / incoming_length);
    long double normal_length = std::sqrt(
        normal_x * normal_x + normal_y * normal_y);
    if (normal_length <= 1e-12L) {
        normal_x = side * -outgoing_y / outgoing_length;
        normal_y = side * outgoing_x / outgoing_length;
        normal_length = 1.0L;
    }

    const long double x = static_cast<long double>(seam.x) +
        normal_x * static_cast<long double>(depth) / normal_length;
    const long double y = static_cast<long double>(seam.y) +
        normal_y * static_cast<long double>(depth) / normal_length;
    if (x < static_cast<long double>((std::numeric_limits<coord_t>::min)()) ||
        x > static_cast<long double>((std::numeric_limits<coord_t>::max)()) ||
        y < static_cast<long double>((std::numeric_limits<coord_t>::min)()) ||
        y > static_cast<long double>((std::numeric_limits<coord_t>::max)()))
        return std::nullopt;
    return c_point{coord_t(std::llround(x)), coord_t(std::llround(y))};
}

void append_inside_end(MutableExtrusionEntity wipe,
                       const WipeLeaf &source,
                       const std::vector<c_extrusion_segment> &source_segments,
                       const coord_t depth)
{
    if (!source.loop || depth <= 0 || wipe.segment_count() == 0)
        return;
    long double signed_area_twice = 0.0L;
    for (const c_extrusion_segment &segment : source_segments)
        signed_area_twice += static_cast<long double>(segment.point_a.x) * segment.point_b.y -
                             static_cast<long double>(segment.point_b.x) * segment.point_a.y;
    const bool counter_clockwise = signed_area_twice > 0.0L;
    const bool hole =
        (source.perimeter_flags & C_EXTRUSION_PERIMETER_FLAG_HOLE) != 0;
    const long double side =
        (hole ? !counter_clockwise : counter_clockwise) ? 1.0L : -1.0L;

    std::vector<c_extrusion_segment> segments = wipe.segments();
    const c_extrusion_segment &last = segments.back();
    const long double tangent_x =
        static_cast<long double>(last.point_b.x) - last.point_a.x;
    const long double tangent_y =
        static_cast<long double>(last.point_b.y) - last.point_a.y;
    const long double tangent_length = std::sqrt(
        tangent_x * tangent_x + tangent_y * tangent_y);
    if (tangent_length <= SCALED_EPSILON)
        return;

    const long double x = static_cast<long double>(last.point_b.x) -
        side * tangent_y * depth / tangent_length;
    const long double y = static_cast<long double>(last.point_b.y) +
        side * tangent_x * depth / tangent_length;
    if (x < static_cast<long double>((std::numeric_limits<coord_t>::min)()) ||
        x > static_cast<long double>((std::numeric_limits<coord_t>::max)()) ||
        y < static_cast<long double>((std::numeric_limits<coord_t>::min)()) ||
        y > static_cast<long double>((std::numeric_limits<coord_t>::max)()))
        return;

    c_extrusion_segment inward = {};
    inward.point_a = last.point_b;
    inward.point_b = c_point{coord_t(std::llround(x)), coord_t(std::llround(y))};
    inward.z_offset_a = last.z_offset_b;
    inward.z_offset_b = last.z_offset_b;
    inward.orientation = RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN;
    segments.push_back(inward);
    if (!wipe.set_segments(segments))
        throw std::runtime_error("Unable to append the inside-end wipe movement.");
}

void add_inside_start(
    const ScopeEntity &target,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key)
{
    const ExtrusionScope::OrderedExtrusionScope scope(target.entity, key);
    if (!scope.has_incoming_transition())
        return;

    const extruder_index_t extruder_id = target.tool_group.extruder_id();
    if (!print_config.vector_bool_or_default(
            "wipe_inside_start", extruder_id, false))
        return;

    MutableExtrusionEntity unretract = find_axis_event(
        scope.before(), RAW_EXTRUSION_ROLE_UNRETRACT);
    if (!unretract.valid())
        return;
    const EPropertyExtrusionAxis *axis =
        unretract.get(EPropertyExtrusionAxis::key);
    if (axis == nullptr ||
        axis->operation != C_EXTRUSION_AXIS_OPERATION_UNRETRACT ||
        axis->toolchange != 0)
        return;

    WipeLeaf target_leaf;
    std::vector<c_extrusion_segment> loop_segments;
    coord_t phase_base_z = 0;
    if (!target_first_leaf(target, key, target_leaf, phase_base_z, &loop_segments) ||
        !target_leaf.loop ||
        !RAW_EXTRUSION_ROLE_HAS(target_leaf.role, RAW_EXTRUSION_ROLE_EXTERNAL))
        return;

    const double nozzle_diameter = print_config.vector_float_or_default(
        "nozzle_diameter", extruder_id, 0.4);
    const double configured_depth = print_config.vector_percent_or_default(
        "wipe_inside_depth", extruder_id, 0.5) * nozzle_diameter;
    const coord_t depth = scale_i(
        configured_depth > 0.0 ? configured_depth : nozzle_diameter * 0.5);
    const std::optional<c_point> inside = loop_inside_point(target_leaf, loop_segments, depth);
    if (!inside || target_leaf.entity.segment_count() == 0)
        return;

    const c_extrusion_segment first = target_leaf.entity.segment(0);
    if (c_point_distance_to(*inside, first.point_a) <= SCALED_EPSILON)
        return;
    const int64_t z_offset = int64_t(target_leaf.base_z) - phase_base_z +
        first.z_offset_a;
    if (z_offset < int64_t((std::numeric_limits<coord_t>::min)()) ||
        z_offset > int64_t((std::numeric_limits<coord_t>::max)()))
        throw std::overflow_error("The inside-start Z offset exceeds coord_t.");

    c_extrusion_segment approach = {};
    approach.point_a = *inside;
    approach.point_b = first.point_a;
    approach.z_offset_a = coord_t(z_offset);
    approach.z_offset_b = coord_t(z_offset);
    approach.orientation = RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN;
    if (!unretract.set_segments({approach}))
        throw std::runtime_error("Unable to define the inside-start approach.");
    set_wipe_attributes(
        unretract,
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_UNRETRACT);
    unretract.get_or_add(EPropertySpeed::key)
        .speed(float(wipe_speed(print_config, extruder_id)))
        .acceleration(-1.f)
        .pressure_advance(-1.f);
}

void set_wipe_attributes(MutableExtrusionEntity entity,
                         const raw_extrusion_role role)
{
    entity.get_or_add(EPropertyAttributes::key)
        .extrusion_role(role)
        .mm3_per_mm(0.0)
        .width(0.f)
        .height(0.f)
        .no_seam_enabled(false);
}

double wipe_speed(const Config &print_config, const extruder_index_t extruder_id)
{
    const double configured = print_config.vector_float_or_default(
        "wipe_speed", extruder_id, 0.0);
    return configured > 0.0 ? configured :
        print_config.float_or_default("travel_speed", 0.0) * 0.8;
}

WipeCrossingChecker::WipeCrossingChecker(
    const Config &print_config,
    const PrintingPlan &plan,
    const plugin_run_context *run_ctx) :
    m_print_config(print_config),
    m_plan(plan),
    m_run_ctx(run_ctx)
{}

bool WipeCrossingChecker::crosses(
    const WipeLeaf &source,
    const ScopeEntity &target_scope,
    const WipeLeaf &target)
{
    const LayerRegionIsland region_island =
        target_scope.printing_extrusion.region_island();
    if (!region_island.valid() || region_island.region_count() == 0 ||
        source.entity.segment_count() == 0 || target.entity.segment_count() == 0)
        return false;

    const Layer layer_view = region_island.region(0).layer();
    if (layer_view.handle() == nullptr)
        return false;

    const extruder_index_t extruder_id = target_scope.tool_group.extruder_id();
    const Key cache_key{layer_view.handle(), extruder_id};
    // Keep copied geometry stable while the per-tool crossing cache borrows it.
    const AC::Input &input = m_inputs.get(
        Print(plugin_ctx_as_layer_extrusion_edition(m_run_ctx)->print), layer_view, m_run_ctx);
    AC::PerimeterCrossing &crossing = m_crossings[cache_key];

    const double nozzle_diameter = m_print_config.vector_float_or_default(
        "nozzle_diameter", extruder_id, 0.4);
    const coord_t nozzle_radius = std::max<coord_t>(
        1, scale_i(nozzle_diameter * 0.5));
    crossing.prepare(m_run_ctx->plugin_storage, *input.target, extruder_id,
        nozzle_radius, [this]() { throw_if_cancelled(m_run_ctx); });

    // Crossing areas use target-local XY; global auxiliary paths have a zero shift.
    const c_point shift = m_plan.object_instance_shift(target_scope.printing_extrusion);
    const c_extrusion_segment source_last =
        source.entity.segment(source.entity.segment_count() - 1);
    const c_extrusion_segment target_first = target.entity.segment(0);
    PluginCommon::work::Polyline direct;
    direct.points = {
        PluginCommon::work::Point{source_last.point_b.x - shift.x,
                      source_last.point_b.y - shift.y},
        PluginCommon::work::Point{target_first.point_a.x - shift.x,
                      target_first.point_a.y - shift.y}
    };
    return crossing.can_cross_perimeter(direct, true);
}

void add_outgoing_wipe(
    const ScopeEntity &source,
    const ScopeEntity *target,
    const Config &print_config,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key,
    WipeCrossingChecker &crossing_checker)
{
    const ExtrusionScope::OrderedExtrusionScope scope(source.entity, key);
    if (!scope.has_outgoing_transition())
        return;
    const extruder_index_t extruder_id = source.tool_group.extruder_id();
    if (!print_config.vector_bool_or_default("wipe", extruder_id, false))
        return;

    MutableExtrusionEntity retract = find_axis_event(
        scope.after(), RAW_EXTRUSION_ROLE_RETRACT);
    if (!retract.valid())
        return;
    const EPropertyExtrusionAxis *axis = retract.get(EPropertyExtrusionAxis::key);
    if (axis == nullptr ||
        axis->operation != C_EXTRUSION_AXIS_OPERATION_RETRACT_TO ||
        !std::isfinite(axis->value) || axis->value <= 0.0)
        return;
    const double total_retraction = axis->value;
    const bool toolchange = axis->toolchange != 0;

    const double speed = wipe_speed(print_config, extruder_id);
    const double retract_speed = print_config.vector_float_or_default(
        "retract_speed", extruder_id, 0.0);
    if (!(speed > 0.0) || !(retract_speed > 0.0))
        return;
    const double retraction_per_mm =
        0.95 * std::floor(retract_speed + 0.5) / speed;
    if (!(retraction_per_mm > 0.0))
        return;

    WipePath path = source_wipe_path(source, key);
    if (!path.has_source || path.segments.empty())
        return;
    if (print_config.vector_bool_or_default(
            "wipe_only_crossing", extruder_id, false)) {
        if (target == nullptr)
            return;
        WipeLeaf target_leaf;
        coord_t unused_phase_base_z = 0;
        if (!target_first_leaf(*target, key, target_leaf, unused_phase_base_z) ||
            !crossing_checker.crosses(path.source, *target, target_leaf))
            return;
    }

    const double base_length_mm = total_retraction / retraction_per_mm;
    double wipe_length_mm =
        print_config.vector_effective_float_or_percent_or_default(
            "wipe_min", extruder_id, base_length_mm, base_length_mm);
    if (path.source.loop &&
        RAW_EXTRUSION_ROLE_HAS(path.source.role, RAW_EXTRUSION_ROLE_EXTERNAL))
        wipe_length_mm += print_config.vector_float_or_default(
            "wipe_extra_perimeter", extruder_id, 0.0);
    const bool return_to_seam = print_config.vector_bool_or_default(
        "wipe_return", extruder_id, false);
    double requested_outbound_mm = return_to_seam && !path.complete_turn ?
        wipe_length_mm * 0.5 : wipe_length_mm;
    if (return_to_seam && path.complete_turn) {
        const double turn_mm = unscaled(path.loop_length);
        if (turn_mm > EPSILON)
            requested_outbound_mm = std::ceil(wipe_length_mm / turn_mm) * turn_mm;
    }

    const distf_t requested_outbound = scale_d(requested_outbound_mm);
    std::vector<c_extrusion_segment> fitted = fit_wipe_length(
        path.segments, path.complete_turn, requested_outbound);
    if (fitted.empty())
        return;

    MutableExtrusionEntity wipe = ExtrusionScope::append_phase_leaf(scope.after());
    if (!wipe.set_segments(fitted))
        throw std::runtime_error("Unable to append wipe geometry after retraction.");
    if (wipe.local_length() > requested_outbound + distf_t(SCALED_EPSILON) &&
        !wipe.clip_end(wipe.local_length() - requested_outbound))
        throw std::runtime_error("Unable to clip wipe geometry to its configured length.");
    if (return_to_seam && !path.complete_turn) {
        std::vector<c_extrusion_segment> round_trip = wipe.segments();
        const std::vector<c_extrusion_segment> return_path =
            reversed_segments(round_trip);
        round_trip.insert(round_trip.end(), return_path.begin(), return_path.end());
        if (!wipe.set_segments(round_trip))
            throw std::runtime_error("Unable to append the wipe return path.");
    }
    if (!return_to_seam && path.complete_turn &&
        print_config.vector_bool_or_default(
            "wipe_inside_end", extruder_id, false)) {
        const double nozzle_diameter = print_config.vector_float_or_default(
            "nozzle_diameter", extruder_id, 0.4);
        const double configured_depth = print_config.vector_percent_or_default(
            "wipe_inside_depth", extruder_id, 0.5) * nozzle_diameter;
        append_inside_end(wipe, path.source, path.segments, scale_i(
            configured_depth > 0.0 ? configured_depth : nozzle_diameter * 0.5));
    }

    set_wipe_attributes(
        wipe, RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    wipe.get_or_add(EPropertySpeed::key)
        .speed(float(speed))
        .acceleration(-1.f)
        .pressure_advance(-1.f);

    // Structural insertion may have moved the original event into a child.
    // Reacquire it before changing the initial E-only target.
    retract = find_axis_event(scope.after(), RAW_EXTRUSION_ROLE_RETRACT);
    if (!retract.valid())
        throw std::runtime_error("The source Retract disappeared while adding its wipe.");
    EPropertyExtrusionAxis *mutable_axis =
        retract.get_mutable(EPropertyExtrusionAxis::key);
    if (mutable_axis == nullptr)
        throw std::runtime_error("The source Retract lost its E-axis request.");
    const double before_percent = print_config.vector_percent_or_default(
        "retract_before_wipe", extruder_id, 0.0);
    const double initial_retraction = std::clamp(
        total_retraction * before_percent, 0.0, total_retraction);
    mutable_axis->value = initial_retraction;

    const double movable_retraction =
        std::max(0.0, total_retraction - initial_retraction);
    const distf_t retracting_distance = scale_d(
        movable_retraction / retraction_per_mm);
    const distf_t total_wipe_length = wipe.local_length();
    const double wiped_retraction = std::min(
        movable_retraction, unscaled(total_wipe_length) * retraction_per_mm);
    if (retracting_distance > distf_t(SCALED_EPSILON) &&
        retracting_distance < total_wipe_length - distf_t(SCALED_EPSILON)) {
        MutableExtrusionEntity retracting_wipe = wipe.emplace_ordered_leaf(
            OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
        if (!retracting_wipe.valid() || wipe.child_count() != 2)
            throw std::runtime_error("Unable to create the retracting wipe phase.");
        MutableExtrusionEntity motion_only_wipe = wipe.child_mutable(1);
        if (extrusion_polyline_split_at_distance(
                motion_only_wipe.handle(), retracting_distance,
                retracting_wipe.mutable_handle(),
                motion_only_wipe.mutable_handle()) == 0)
            throw std::runtime_error("Unable to split wipe at the retraction target.");
        set_wipe_attributes(
            retracting_wipe,
            RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
            RAW_EXTRUSION_ROLE_RETRACT);
        retracting_wipe.get_or_add(EPropertyExtrusionAxis::key)
            .retract_to(total_retraction, toolchange);
    } else if (retracting_distance > distf_t(SCALED_EPSILON)) {
        set_wipe_attributes(
            wipe,
            RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
            RAW_EXTRUSION_ROLE_RETRACT);
        wipe.get_or_add(EPropertyExtrusionAxis::key)
            .retract_to(initial_retraction + wiped_retraction, toolchange);
    }

    if (total_retraction - wiped_retraction > initial_retraction + EPSILON) {
        MutableExtrusionEntity residual =
            ExtrusionScope::append_phase_leaf(scope.after());
        set_wipe_attributes(residual, RAW_EXTRUSION_ROLE_RETRACT);
        residual.get_or_add(EPropertyExtrusionAxis::key)
            .retract_to(total_retraction, toolchange);
    }
}

CreateRetractionWipe &CreateRetractionWipe::instance(
    orchestrator_handle *orchestrator)
{
    static CreateRetractionWipe plugin(orchestrator);
    return plugin;
}

CreateRetractionWipe::CreateRetractionWipe(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_scope_property(printing_extrusion_scope_property_key(orchestrator))
{}

const char *CreateRetractionWipe::id_impl() const noexcept
{
    return "layer_extrusion_edit.wipe.default";
}

const char *CreateRetractionWipe::name_impl() const noexcept
{
    return "Create retraction wipe";
}

const char *CreateRetractionWipe::description_impl() const noexcept
{
    return "Distributes eligible semantic retractions over the preceding printed path.";
}

const char *CreateRetractionWipe::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.wipe";
}

slicing_step_t CreateRetractionWipe::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *CreateRetractionWipe::dependencies_impl() const noexcept
{
    return k_dependencies;
}

int32_t CreateRetractionWipe::priority_impl() const noexcept
{
    return -80;
}

int32_t CreateRetractionWipe::used_config_keys(
    raw_used_config_key *keys) const noexcept
{
    static const raw_used_config_key used_keys[] = {
        {"wipe", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_min", RAW_CO_VECTOR_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_return", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_only_crossing", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_before_wipe", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"retract_speed", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"travel_speed", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_inside_start", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_inside_end", RAW_CO_VECTOR_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_inside_depth", RAW_CO_VECTOR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
        {"wipe_extra_perimeter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
    };
    if (keys != nullptr)
        std::copy(std::begin(used_keys), std::end(used_keys), keys);
    return int32_t(sizeof(used_keys) / sizeof(used_keys[0]));
}

const char *CreateRetractionWipe::progress_message_format_impl() const noexcept
{
    return "Creating retraction wipes: %u / %u layers";
}

void CreateRetractionWipe::setup_run_impl(
    const plugin_run_context *run_ctx) const
{
    if (plugin_ctx_as_layer_extrusion_edition(run_ctx) != nullptr)
        progress().add_max(1);
}

void CreateRetractionWipe::run_impl(const plugin_run_context *run_ctx) const
{
    const run_ctx_layer_extrusion_edition *ctx =
        plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr || ctx->print == nullptr)
        return;

    PrintingLayerGroup layer(ctx->layer_group);
    const Config print_config = Print(ctx->print).config();
    if (ctx->plan == nullptr)
        return;
    PrintingPlan plan(ctx->plan);
    WipeCrossingChecker crossing_checker(print_config, plan, run_ctx);
    PrintingEntityPropertyTraversal<PrintingExtrusionScopeProperty> traversal(
        m_scope_property,
        wipe_state_definition(),
        [this, &print_config, &crossing_checker]
        (ScopeEntity *previous, ScopeEntity *next) {
            if (next != nullptr)
                add_inside_start(*next, print_config, m_scope_property);
            if (previous != nullptr)
                add_outgoing_wipe(
                    *previous, next, print_config, m_scope_property,
                    crossing_checker);
        },
        MatchingEntityDescendants::Skip);
    traversal.process(layer);
    progress().increment();
}

} // namespace

void register_create_retraction_wipe_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, CreateRetractionWipe::instance(orchestrator).c_instance());
    const plugin_instance member = CreateRetractionWipe::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Retraction wipe";
    definition.tooltip = "Selects how retraction is distributed over movement on printed material.";
    CreateRetractionWipe::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin
