///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Default target deceleration for travels
=======================================

Speed and acceleration providers first publish complete process properties on
the ordered PrintingPlan. This plugin then groups consecutive compatible
Travel leaves into one virtual trajectory. It may split the leaf containing
the deceleration boundary, while every later leaf uses the following printed
extrusion's acceleration. This matches the legacy G-code planner without
making tree boundaries visible to the motion calculation.

Each parallel run mutates only its own PrintingLayerGroup. setup() scans the
already resolved speeds once so the first travel of every layer can still see
the movement speed that preceded that independently processed layer.

Synthetic catalog entries have no source object and an unset instance index.
Their region-less output stays in movement order but has no regional
target-deceleration setting; it therefore cannot supply a deceleration target.
Real-object output must retain its source regions.
*/

#include "DefaultTravelDeceleration.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "ExtrusionProcessParameterHelpers.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_polyline.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_layer_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/RegionSettingsViews.hpp"

namespace slic3r_api { namespace LayerExtrusionEdit { namespace DefaultTravelDecelerationPlugin {
// Register the permanent borrowed plugin and copy its alternative-selection setting description into the application.
// Print data is unchanged; allocation failures or rejected descriptions throw.
void register_default_travel_deceleration_plugin(orchestrator_handle *orchestrator);

namespace {

using namespace ProcessParameterHelpers;

constexpr double PI_VALUE = 3.14159265358979323846;

const char *const k_no_dependencies[] = { nullptr };

const raw_used_config_key k_used_config_keys[] = {
    {"gcode_min_length", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE},
    {"travel_deceleration_use_target", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE}
};

const RegionSettings::OptionKeyGroup k_travel_deceleration_region_keys{
    "travel_deceleration_use_target"
};

const char *const k_defined_config_keys[] = {
    "gcode_min_length",
    "travel_deceleration_use_target"
};

const char *const k_gcode_min_length_aliases[] = { "min_length" };

using slic3r_api::GuiRule;
using slic3r_api::non_zero;

void initialize_default_travel_deceleration_settings(orchestrator_handle *orchestrator)
{
    /*
    Option definitions owned by this module. Each entry fixes the option type,
    override scope, default and GUI metadata; defined keys are published in the
    print presets, so profiles read and write them by name like any built-in
    setting. Called once from inilialize_impl().
    */
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = "gcode_min_length";
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_PLATER;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.label = "Minimum extrusion length";
    def.tooltip = "When outputting gcode, this setting ensure that there is almost no commands more than this value apart."
        " Be sure to also use max_gcode_per_second instead, as it's much better when you have very different speeds for features"
        " (Too many too small commands may overload the firmware / connection)."
        "\nDisabled if set to 0.";
    def.sidetext = "mm or %";
    def.has_min = 1;
    def.min_value = 0.0;
    def.precision = 6;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.can_be_disabled = 1;
    def.default_serialized_value = "0.02";
    def.aliases = const_strings_t{ k_gcode_min_length_aliases, 1 };
    orchestrator_create_option_def(orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = "travel_deceleration_use_target";
    def.type = RAW_CO_BOOL;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.category = RAW_OPTION_CATEGORY_SPEED;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.invalidates_step = STEP_GCODE;
    def.label = "Decelerate with target acceleration";
    def.full_label = "Use target acceleration for travel deceleration";
    def.tooltip = "If selected, the deceleration of a travel will use the acceleration value of the extrusion that will be printed after it (if any) ";
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "1";
    orchestrator_create_option_def(orchestrator, &def);

    // Travel deceleration applies once a default acceleration is set for the print.
    GuiRule::activate({ "travel_deceleration_use_target" })
        .when(non_zero("default_acceleration"))
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator);
}

struct LayerEntrySpeed
{
    float speed_mm_per_s = -1.f;
    bool known = false;
};

struct OrderedLeaf
{
    MutableExtrusionEntity entity;
    EffectiveTreeState state;
    LayerRegionIsland region_island;
    c_point instance_shift = {0,0};
    distf_t length = 0.0;
};

struct TravelRun
{
    size_t begin = 0;
    size_t end = 0;
    float speed = -1.f;
    float acceleration = -1.f;
    distf_t length = 0.0;
    uint32_t segment_count = 0;
    double last_segment_length = 0.0;
};

enum class DecelerationAction
{
    None,
    WholeTravel,
    SplitTravel
};

struct DecelerationDecision
{
    DecelerationAction action = DecelerationAction::None;
    distf_t split_distance = 0.0;
};

/* Measure one straight or curved segment in scaled planar coordinates. */
double planar_segment_length(const c_extrusion_segment &segment);

/* Identify a geometric leaf whose effective role is Travel. */
bool is_travel_leaf(const OrderedLeaf &leaf);

/* Update the last segmented or deposit speed while scanning one immutable tree. */
void scan_last_speed(const MutableExtrusionEntity &entity,
                     const EffectiveTreeState &parent_state,
                     LayerEntrySpeed &last_speed,
                     const plugin_run_context *run_ctx);

/* Cache the movement speed visible immediately before every layer-group. */
void prepare_layer_entry_speeds(PrintingPlan &plan,
                                std::vector<std::vector<LayerEntrySpeed>> &entry_speeds,
                                const plugin_run_context *run_ctx);

/* Flatten one tree into geometric leaves with all inherited process values resolved. */
void collect_ordered_leaves(
    MutableExtrusionEntity entity,
    const EffectiveTreeState &parent_state,
    const LayerRegionIsland &region_island,
    const c_point instance_shift,
    std::vector<OrderedLeaf> &leaves,
    const plugin_run_context *run_ctx);

/* Flatten one tool-group across tree and PrintingExtrusion boundaries. */
// Retain synthetic movement without regional source lookup; reject missing real-object sources.
std::vector<OrderedLeaf> ordered_tool_leaves(
    PrintingPlan &plan,
    PrintingToolGroup tool,
    PluginProgress &progress,
    const plugin_run_context *run_ctx);

/* Build the maximal homogeneous Travel run beginning at one flattened leaf. */
TravelRun collect_travel_run(
    const std::vector<OrderedLeaf> &leaves,
    size_t begin,
    const plugin_run_context *run_ctx);

/* Resolve the regional enable switch at the final point of one Travel run. */
// Return false for a region-less synthetic target rather than using object settings.
bool target_deceleration_is_enabled(const std::vector<OrderedLeaf> &leaves,
                                    const TravelRun &travel,
                                    const OrderedLeaf &target,
                                    const plugin_run_context *run_ctx);

/* Reproduce the legacy kinematic checks and select a split position when useful. */
DecelerationDecision deceleration_decision(const TravelRun &travel,
                                           const OrderedLeaf &target,
                                           float previous_speed,
                                           bool use_target_deceleration,
                                           const Config &config);

/* Apply the target acceleration directly to every leaf in one Travel range. */
void apply_travel_range_acceleration(const std::vector<OrderedLeaf> &leaves,
                                     size_t begin,
                                     size_t end,
                                     float target_acceleration,
                                     EPropertySpeedFieldEditor &editor,
                                     const plugin_run_context *run_ctx);

/* Split one leaf at a local distance and return its final phase. */
MutableExtrusionEntity split_travel_for_deceleration(MutableExtrusionEntity travel,
                                                      distf_t split_distance,
                                                      const plugin_run_context *run_ctx);

/* Project a run-wide split distance onto its leaves and annotate the suffix. */
void apply_split_travel_acceleration(const std::vector<OrderedLeaf> &leaves,
                                     const TravelRun &run,
                                     distf_t split_distance,
                                     float target_acceleration,
                                     const plugin_run_context *run_ctx);

/* Process homogeneous Travel runs within one tool-group. */
void process_tool_group(const Print &print,
                        PrintingPlan &plan,
                        PrintingToolGroup tool,
                        LayerEntrySpeed &previous,
                        PluginProgress &progress,
                        const plugin_run_context *run_ctx);

/* Apply target deceleration to eligible travels of one independently processed layer. */
void process_layer(const Print &print,
                   PrintingPlan &plan,
                   PrintingLayerGroup &layer,
                   const LayerEntrySpeed &entry_speed,
                   PluginProgress &progress,
                   const plugin_run_context *run_ctx);

class DefaultTravelDeceleration : public PluginBase
{
public:
    static DefaultTravelDeceleration &instance(orchestrator_handle *orchestrator);
    explicit DefaultTravelDeceleration(orchestrator_handle *orchestrator);

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
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    mutable bool m_setup_valid = false;
    mutable std::vector<std::vector<LayerEntrySpeed>> m_entry_speeds;
};

double planar_segment_length(const c_extrusion_segment &segment)
{
    const double dx = double(segment.point_b.x) - double(segment.point_a.x);
    const double dy = double(segment.point_b.y) - double(segment.point_a.y);
    const double chord = std::hypot(dx, dy);
    if (segment.radius == 0.f)
        return chord;

    const double radius = std::abs(double(segment.radius));
    if (radius <= 0.0 || chord > radius * 2.0 + double(SCALED_EPSILON))
        throw std::invalid_argument("Travel deceleration encountered an arc with an invalid radius.");

    // Positive radii describe the short arc. A negative radius selects the
    // complementary long arc between the same two endpoints.
    const double ratio = std::clamp(chord / (2.0 * radius), 0.0, 1.0);
    double angle = 2.0 * std::asin(ratio);
    if (segment.radius < 0.f)
        angle = 2.0 * PI_VALUE - angle;
    return radius * angle;
}

bool is_travel_leaf(const OrderedLeaf &leaf)
{
    return leaf.state.has_attributes &&
        RAW_EXTRUSION_ROLE_IS_TRAVEL(leaf.state.attributes.extrusion_role());
}

// Scan borrowed segmented and deposit leaves for the final positive speed; update only the scalar snapshot, retain it for missing speed, and throw on cancellation.
void scan_last_speed(const MutableExtrusionEntity &entity,
                     const EffectiveTreeState &parent_state,
                     LayerEntrySpeed &last_speed,
                     const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            scan_last_speed(entity.child_mutable(child_idx), state, last_speed, run_ctx);
        }
        return;
    }
    if (entity.segment_count() == 0 && !entity.is_deposit())
        return;

    // A movement with no explicit speed keeps the firmware's previous speed.
    // Valid completed speed providers normally make every geometric leaf
    // positive, but retaining the prior state keeps this prefix scan neutral.
    if (state.speed > 0.f && std::isfinite(state.speed)) {
        last_speed.speed_mm_per_s = state.speed;
        last_speed.known = true;
    }
}

void prepare_layer_entry_speeds(PrintingPlan &plan,
                                std::vector<std::vector<LayerEntrySpeed>> &entry_speeds,
                                const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    entry_speeds.clear();
    entry_speeds.resize(plan.group_count());
    LayerEntrySpeed last_speed = {};

    // The scan follows the same group/layer/tool/extrusion order as the file
    // writer. The resulting scalar snapshots are safe for parallel readers.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        entry_speeds[group_idx].resize(group.layer_group_count());
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            entry_speeds[group_idx][layer_idx] = last_speed;
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_ctx);
                PrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                    throw_if_cancelled(run_ctx);
                    scan_last_speed(tool.extrusion(extrusion_idx).mutable_root(),
                                    EffectiveTreeState{}, last_speed, run_ctx);
                }
            }
        }
    }
}

// Append borrowed segmented and deposit leaf views in execution order without tree mutation; invalid lengths or cancellation throw, and the plan retains ownership.
void collect_ordered_leaves(
    MutableExtrusionEntity entity,
    const EffectiveTreeState &parent_state,
    const LayerRegionIsland &region_island,
    const c_point instance_shift,
    std::vector<OrderedLeaf> &leaves,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const EffectiveTreeState state = effective_state(entity, parent_state);
    if (entity.child_count() > 0) {
        for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
            throw_if_cancelled(run_ctx);
            collect_ordered_leaves(
                entity.child_mutable(child_idx), state, region_island,
                instance_shift, leaves, run_ctx);
        }
        return;
    }
    if (entity.segment_count() == 0 && !entity.is_deposit())
        return;

    const distf_t length = entity.local_length();
    if (!std::isfinite(length) || length < 0.0)
        throw std::runtime_error("Travel deceleration encountered an invalid leaf length.");
    throw_if_cancelled(run_ctx);
    leaves.push_back(OrderedLeaf{
        entity, state, region_island, instance_shift, length
    });
}

// Borrow plan-owned trees and return borrowed ordered leaf views, including synthetic movement with no source shift; invalid real sources or cancellation throw without geometry mutation.
std::vector<OrderedLeaf> ordered_tool_leaves(
    PrintingPlan &plan,
    PrintingToolGroup tool,
    PluginProgress &progress,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    std::vector<OrderedLeaf> leaves;
    if (tool.extruder_id() == EXTRUDER_ID_UNKNOWN)
        throw std::runtime_error("Travel deceleration encountered a tool group without an extruder.");
    for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
        throw_if_cancelled(run_ctx);
        PrintingExtrusion extrusion = tool.extrusion(extrusion_idx);
        const bool synthetic = is_regionless_synthetic(plan, extrusion);
        const LayerRegionIsland region_island = extrusion.region_island();
        if (!synthetic && (!region_island.valid() || region_island.region_count() == 0))
            throw std::runtime_error(
                "Travel deceleration encountered an extrusion without source regions.");
        const c_point instance_shift = synthetic ? c_point{0,0} : plan.object_instance_shift(extrusion);
        collect_ordered_leaves(
            extrusion.mutable_root(), EffectiveTreeState{}, region_island,
            instance_shift, leaves, run_ctx);
        throw_if_cancelled(run_ctx);
        progress.increment();
    }
    return leaves;
}

TravelRun collect_travel_run(
    const std::vector<OrderedLeaf> &leaves,
    const size_t begin,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    assert(begin < leaves.size() && is_travel_leaf(leaves[begin]));
    const float speed = leaves[begin].state.speed;
    const float acceleration = leaves[begin].state.acceleration;
    TravelRun run{begin, begin, speed, acceleration, 0.0, 0, 0.0};

    // Tree and PrintingExtrusion boundaries do not alter machine movement.
    // A process-value change does, so it starts a separate kinematic run.
    while (run.end < leaves.size() && is_travel_leaf(leaves[run.end]) &&
           leaves[run.end].state.speed == speed &&
           leaves[run.end].state.acceleration == acceleration) {
        throw_if_cancelled(run_ctx);
        const OrderedLeaf &leaf = leaves[run.end];
        run.length += leaf.length;
        run.segment_count += leaf.entity.segment_count();
        run.last_segment_length = planar_segment_length(
            leaf.entity.segment(leaf.entity.segment_count() - 1));
        ++run.end;
    }

    if (!std::isfinite(run.length) || run.length < 0.0)
        throw std::runtime_error("Travel deceleration produced an invalid run length.");
    return run;
}

// Read borrowed regional settings at a travel endpoint; unmatched deposits use the island's first region, synthetic targets return false, and ordinary lookup failures or cancellation throw without mutation.
bool target_deceleration_is_enabled(const std::vector<OrderedLeaf> &leaves,
                                    const TravelRun &travel,
                                    const OrderedLeaf &target,
                                    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    assert(travel.begin < travel.end && travel.end <= leaves.size());
    if (!target.region_island.valid())
        return false;
    const OrderedLeaf &last_travel = leaves[travel.end - 1];
    const c_extrusion_segment last_segment =
        last_travel.entity.segment(last_travel.entity.segment_count() - 1);

    const Object object = target.region_island.region(0).layer().object();

    // Region slices use object coordinates while PrintingPlan paths already
    // include the selected instance shift. Convert only the lookup points;
    // the extrusion geometry itself must remain untouched.
    const c_point endpoint{
        last_segment.point_b.x - target.instance_shift.x,
        last_segment.point_b.y - target.instance_shift.y
    };
    RegionSettingsPointResult lookup = RegionSettings::lookup_at_point(
        target.region_island, k_travel_deceleration_region_keys, endpoint);
    throw_if_cancelled(run_ctx);
    if (lookup.found())
        return lookup.value.get_bool("travel_deceleration_use_target");

    // A deposit has no interior path to probe when the point is outside or ambiguous.
    if (target.entity.is_deposit())
        return target.region_island.region(0).print_region().config().bool_or_default(
            "travel_deceleration_use_target", false);

    // A boundary belongs to both neighboring ExPolygons. Move a tiny distance
    // into the following printed path so the selected value comes from the
    // region the machine is about to enter, including when that path is curved.
    const distf_t probe_distance = std::min(
        distf_t(SCALED_EPSILON), target.length / 2.0);
    if (probe_distance > 0.0) {
        const c_point plan_probe = target.entity.point_from_begin(probe_distance);
        const c_point source_probe{
            plan_probe.x - target.instance_shift.x,
            plan_probe.y - target.instance_shift.y
        };
        lookup = RegionSettings::lookup_at_point(
            target.region_island, k_travel_deceleration_region_keys, source_probe);
        throw_if_cancelled(run_ctx);
        if (lookup.found())
            return lookup.value.get_bool("travel_deceleration_use_target");
    }

    if (lookup.status == RegionSettingsPointStatus::Ambiguous)
        throw std::runtime_error(
            "Travel deceleration target remains on a boundary with conflicting regional settings.");
    throw std::runtime_error(
        "Travel deceleration target point is outside all of its source regions.");
}

DecelerationDecision deceleration_decision(const TravelRun &travel,
                                           const OrderedLeaf &target,
                                           const float previous_speed,
                                           const bool use_target_deceleration,
                                           const Config &config)
{
    if (!use_target_deceleration)
        return {};

    const double travel_speed = travel.speed;
    const double travel_acceleration = travel.acceleration;
    const double target_speed = target.state.speed;
    const double target_acceleration = target.state.acceleration;
    if (!(travel_speed > 0.0) || !std::isfinite(travel_speed) ||
        !(travel_acceleration > 0.0) || !std::isfinite(travel_acceleration) ||
        !(target_speed > 0.0) || !std::isfinite(target_speed) ||
        !(target_acceleration > 0.0) || !std::isfinite(target_acceleration))
        return {};

    const distf_t length = travel.length;
    if (!std::isfinite(length) || length <= double(SCALED_EPSILON) ||
        travel_acceleration <= target_acceleration ||
        !(previous_speed > 0.f) || !std::isfinite(previous_speed))
        return DecelerationDecision{DecelerationAction::WholeTravel, 0.0};

    // Compute the planar distances spent approaching travel speed and then
    // approaching the following extrusion speed. Units and rounding match the
    // legacy GCode.cpp calculation exactly.
    const double previous_to_travel = previous_speed >= travel_speed ? 0.0 :
        travel_speed - previous_speed;
    const double accelerate_seconds = previous_to_travel / travel_acceleration;
    const distf_t accelerate_distance = scale_d(
        accelerate_seconds * (travel_speed - previous_to_travel / 2.0));

    const double travel_to_target = target_speed >= travel_speed ? 0.0 :
        travel_speed - target_speed;
    const double decelerate_seconds = travel_to_target / target_acceleration;
    const distf_t decelerate_distance = scale_d(
        decelerate_seconds * (travel_speed - travel_to_target / 2.0));

    const double direct_speed_difference = std::abs(double(previous_speed) - target_speed);
    const double direct_acceleration = direct_speed_difference *
        (double(previous_speed) + target_speed) / (2.0 * length);
    if (!std::isfinite(accelerate_distance) || accelerate_distance < 0.0 ||
        !std::isfinite(decelerate_distance) || decelerate_distance < 0.0 ||
        !std::isfinite(direct_acceleration))
        throw std::runtime_error("Travel deceleration produced invalid kinematic distances.");

    double target_width = 0.0;
    if (target.state.has_attributes) {
        target_width = target.state.attributes.c_extrusion_property_attributes::width;
        if (!std::isfinite(target_width) || target_width < 0.0)
            target_width = 0.0;
    }
    const double configured_minimum_mm = config.effective_float_or_percent_or_default(
        "gcode_min_length", target_width, 0.0);
    const distf_t configured_minimum = configured_minimum_mm > 0.0 ?
        scale_d(configured_minimum_mm) : 0.0;
    const distf_t minimum_deceleration = std::max({
        distf_t(SCALED_EPSILON), decelerate_distance / 10.0, configured_minimum
    });

    const bool cannot_split = length < minimum_deceleration ||
        direct_acceleration * 1.1 > target_acceleration ||
        decelerate_distance < distf_t(SCALED_EPSILON);
    if (cannot_split)
        return DecelerationDecision{DecelerationAction::WholeTravel, 0.0};

    // Long multi-segment travels decelerate over their final segment whenever
    // that segment is long enough. Otherwise reserve the required distance
    // from the end; short/simple travels use the legacy inflection ratio.
    distf_t split_distance = 0.0;
    const distf_t needed_deceleration = decelerate_distance + minimum_deceleration;
    if (travel.segment_count > 1 &&
        length > accelerate_distance + needed_deceleration) {
        split_distance = travel.last_segment_length < needed_deceleration ?
            length - needed_deceleration : length - travel.last_segment_length;
    } else {
        const double ratio = (accelerate_distance + 1.0) /
            (accelerate_distance + decelerate_distance + 1.0);
        split_distance = length * ratio;
    }

    if (!std::isfinite(split_distance) ||
        split_distance <= distf_t(SCALED_EPSILON) ||
        split_distance >= length - distf_t(SCALED_EPSILON))
        return DecelerationDecision{DecelerationAction::WholeTravel, 0.0};
    return DecelerationDecision{DecelerationAction::SplitTravel, split_distance};
}

void apply_travel_range_acceleration(const std::vector<OrderedLeaf> &leaves,
                                     const size_t begin,
                                     const size_t end,
                                     const float target_acceleration,
                                     EPropertySpeedFieldEditor &editor,
                                     const plugin_run_context *run_ctx)
{
    assert(begin <= end && end <= leaves.size());
    for (size_t leaf_idx = begin; leaf_idx < end; ++leaf_idx) {
        throw_if_cancelled(run_ctx);
        editor.set_value(leaves[leaf_idx].entity, target_acceleration);
    }
}

MutableExtrusionEntity split_travel_for_deceleration(MutableExtrusionEntity travel,
                                                      const distf_t split_distance,
                                                      const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    MutableExtrusionEntity first_phase = travel.emplace_ordered_leaf(
        OrderedLeafPosition::Before, ExistingPropertyPlacement::KeepOnParent);
    if (!first_phase.valid() || travel.child_count() != 2)
        throw std::runtime_error("Unable to create the ordered travel-deceleration phases.");
    MutableExtrusionEntity final_phase = travel.child_mutable(1);

    // The host split preserves ArcPolyline radii, directions, and point Z
    // offsets. The source aliases the second output safely because the host
    // computes both temporary polylines before replacing either destination.
    throw_if_cancelled(run_ctx);
    if (extrusion_polyline_split_at_distance(
            final_phase.handle(), split_distance,
            first_phase.mutable_handle(), final_phase.mutable_handle()) == 0)
        throw std::runtime_error("Unable to split a travel at its deceleration boundary.");

    return final_phase;
}

void apply_split_travel_acceleration(const std::vector<OrderedLeaf> &leaves,
                                     const TravelRun &run,
                                     const distf_t split_distance,
                                     const float target_acceleration,
                                     const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    EPropertySpeedFieldEditor editor(EPropertySpeedField::Acceleration);
    distf_t distance_before_leaf = 0.0;

    // Locate the virtual boundary without concatenating or copying any
    // polyline. A boundary close to an existing leaf junction reuses that
    // junction and avoids creating a negligible fragment.
    for (size_t leaf_idx = run.begin; leaf_idx < run.end; ++leaf_idx) {
        throw_if_cancelled(run_ctx);
        const distf_t distance_after_leaf = distance_before_leaf + leaves[leaf_idx].length;
        if (split_distance <= distance_before_leaf + distf_t(SCALED_EPSILON)) {
            apply_travel_range_acceleration(
                leaves, leaf_idx, run.end, target_acceleration, editor, run_ctx);
            return;
        }
        if (split_distance < distance_after_leaf - distf_t(SCALED_EPSILON)) {
            MutableExtrusionEntity final_phase = split_travel_for_deceleration(
                leaves[leaf_idx].entity, split_distance - distance_before_leaf, run_ctx);
            throw_if_cancelled(run_ctx);
            editor.set_value(final_phase, target_acceleration);
            apply_travel_range_acceleration(
                leaves, leaf_idx + 1, run.end, target_acceleration, editor, run_ctx);
            return;
        }
        distance_before_leaf = distance_after_leaf;
    }

    // deceleration_decision() returns an interior boundary. Reaching the end
    // therefore indicates inconsistent accumulated lengths rather than a
    // harmless no-op.
    throw std::runtime_error("Travel deceleration boundary is outside its Travel run.");
}

void process_tool_group(const Print &print,
                        PrintingPlan &plan,
                        PrintingToolGroup tool,
                        LayerEntrySpeed &previous,
                        PluginProgress &progress,
                        const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const std::vector<OrderedLeaf> leaves = ordered_tool_leaves(
        plan, tool, progress, run_ctx);
    size_t leaf_idx = 0;
    while (leaf_idx < leaves.size()) {
        throw_if_cancelled(run_ctx);
        const OrderedLeaf &leaf = leaves[leaf_idx];
        if (!is_travel_leaf(leaf)) {
            if (leaf.state.speed > 0.f && std::isfinite(leaf.state.speed)) {
                previous.speed_mm_per_s = leaf.state.speed;
                previous.known = true;
            }
            ++leaf_idx;
            continue;
        }

        const TravelRun run = collect_travel_run(leaves, leaf_idx, run_ctx);
        if (run.end < leaves.size()) {
            const OrderedLeaf &target = leaves[run.end];
            if (!is_travel_leaf(target) && target.state.has_attributes &&
                target.state.speed > 0.f && target.state.acceleration > 0.f) {
                const DecelerationDecision decision = deceleration_decision(
                    run, target, previous.known ? previous.speed_mm_per_s : -1.f,
                    target_deceleration_is_enabled(leaves, run, target, run_ctx),
                    print.config());
                if (decision.action == DecelerationAction::WholeTravel) {
                    EPropertySpeedFieldEditor editor(EPropertySpeedField::Acceleration);
                    apply_travel_range_acceleration(
                        leaves, run.begin, run.end, target.state.acceleration, editor,
                        run_ctx);
                } else if (decision.action == DecelerationAction::SplitTravel) {
                    apply_split_travel_acceleration(
                        leaves, run, decision.split_distance, target.state.acceleration,
                        run_ctx);
                }
            }
        }

        // A homogeneous run ends with its common speed active. This becomes
        // the preceding speed if another differently configured Travel run
        // follows immediately.
        if (run.speed > 0.f && std::isfinite(run.speed)) {
            previous.speed_mm_per_s = run.speed;
            previous.known = true;
        }
        leaf_idx = run.end;
    }
}

void process_layer(const Print &print,
                   PrintingPlan &plan,
                   PrintingLayerGroup &layer,
                   const LayerEntrySpeed &entry_speed,
                   PluginProgress &progress,
                   const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    LayerEntrySpeed previous = entry_speed;
    for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
        throw_if_cancelled(run_ctx);
        process_tool_group(
            print, plan, layer.tool_group(tool_idx), previous, progress, run_ctx);
    }
}

DefaultTravelDeceleration &DefaultTravelDeceleration::instance(orchestrator_handle *orchestrator)
{
    static DefaultTravelDeceleration plugin(orchestrator);
    return plugin;
}

DefaultTravelDeceleration::DefaultTravelDeceleration(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{}

const char *DefaultTravelDeceleration::id_impl() const noexcept
{
    return "layer_extrusion_edit.travel_deceleration.default";
}

const char *DefaultTravelDeceleration::name_impl() const noexcept
{
    return "Default travel target deceleration";
}

const char *DefaultTravelDeceleration::description_impl() const noexcept
{
    return "Uses the next extrusion acceleration for the final phase of eligible travels.";
}

const char *DefaultTravelDeceleration::exclusive_group_impl() const noexcept
{
    return "layer_extrusion_edit.travel_deceleration";
}

slicing_step_t DefaultTravelDeceleration::step_impl() const noexcept
{
    return STEP_LAYER_EXTRUSION_EDIT;
}

const char *const *DefaultTravelDeceleration::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t DefaultTravelDeceleration::priority_impl() const noexcept
{
    return 15;
}

int32_t DefaultTravelDeceleration::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t DefaultTravelDeceleration::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < std::size(k_defined_config_keys); ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return int32_t(std::size(k_defined_config_keys));
}

void DefaultTravelDeceleration::inilialize_impl(storage_handle *) const
{
    initialize_default_travel_deceleration_settings(m_orchestrator);
}

const char *DefaultTravelDeceleration::progress_message_format_impl() const noexcept
{
    return "Applying target travel deceleration: %u / %u trees";
}

void DefaultTravelDeceleration::setup_impl(const plugin_run_context *run_ctx, uint32_t) const
{
    throw_if_cancelled(run_ctx);
    m_setup_valid = false;
    m_entry_speeds.clear();
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    assert(ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr)
        return;

    PrintingPlan plan(ctx->plan);
    prepare_layer_entry_speeds(plan, m_entry_speeds, run_ctx);
    throw_if_cancelled(run_ctx);
    m_setup_valid = true;
}

void DefaultTravelDeceleration::setup_run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    if (ctx == nullptr || ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);
    progress().add_max(extrusion_tree_count(PrintingLayerGroup(ctx->layer_group)));
}

void DefaultTravelDeceleration::run_impl(const plugin_run_context *run_ctx) const
{
    if (!m_setup_valid)
        return;
    const run_ctx_layer_extrusion_edition *ctx = plugin_ctx_as_layer_extrusion_edition(run_ctx);
    assert(ctx != nullptr && ctx->print != nullptr && ctx->plan != nullptr &&
           ctx->layer_group != nullptr);
    if (ctx == nullptr || ctx->print == nullptr || ctx->plan == nullptr ||
        ctx->layer_group == nullptr)
        return;
    throw_if_cancelled(run_ctx);
    if (ctx->group_idx >= m_entry_speeds.size() ||
        ctx->layer_group_idx >= m_entry_speeds[ctx->group_idx].size())
        throw std::runtime_error("Travel-deceleration layer location is outside its setup cache.");

    PrintingPlan plan(ctx->plan);
    PrintingLayerGroup layer_group(ctx->layer_group);
    process_layer(
        Print(ctx->print), plan, layer_group,
        m_entry_speeds[ctx->group_idx][ctx->layer_group_idx], progress(), run_ctx);
}

} // namespace

void register_default_travel_deceleration_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, DefaultTravelDeceleration::instance(orchestrator).c_instance());
    const plugin_instance member = DefaultTravelDeceleration::instance(orchestrator).c_instance();
    raw_exclusive_group_def definition = raw_exclusive_group_def_init();
    definition.exclusive_group = member.vt->get_exclusive_group(member.ctx);
    definition.label = "Travel deceleration";
    definition.tooltip = "Selects how travel movements approach the speed of their following extrusion.";
    DefaultTravelDeceleration::instance(orchestrator).register_exclusive_group(STEP_LAYER_EXTRUSION_EDIT, definition);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::DefaultTravelDecelerationPlugin
