///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
PrintingPlan movement-time estimation
=====================================

This source mirrors the movement geometry consumed by G-code generation, but
keeps timing deliberately simple. It resolves inherited speed, flow and Z metadata,
measures each segment at constant speed, and inserts a straight synthetic
travel whenever two consecutive leaves are separated by at least the shared
scaled continuity epsilon.
Stationary deposits consume volume at speed times inherited mm3_per_mm and
establish the same machine position as a segmented leaf.
Detailed results number geometric leaves only; stationary deposits and synthetic
gaps remain fixed, unattributed time. Both output forms sum in traversal order.

    estimate() / estimate_detailed()
    `-- estimate_plan(): carry machine position across layers and printing groups
        `-- estimate_layer(): preserve tool visits and extrusion order
            `-- estimate_extrusion() -> estimate_entity(): resolve inherited fields
                |-- visit children before a node's own geometry
                |-- charge gaps and stationary deposits as fixed time
                `-- measure planar arcs and Z displacement at constant speed

The estimator copies travel speed from configuration and borrows the plan only
for the call. It changes neither geometry nor properties and caches no views.
Invalid geometry or missing effective rates throw before a result is returned.
*/

#include "PrintingPlanTimeEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace slic3r_api {
namespace {

struct Position3d
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct EstimatorState
{
    float speed_mm_per_s = -1.f;
    double mm3_per_mm = 0.0;
    coord_t z_offset = 0;
};

// Running result of one layer. `duration` is the total charged to the layer and is
// accumulated in traversal order, so estimate() keeps its historical value.
// The optional detail members receive the same increments, split by origin.
struct LayerAccumulator
{
    double duration = 0.0;
    // Fixed deposits and synthetic connectors (including layer entry), or null.
    double *unattributed = nullptr;
    // Per-leaf durations, or null when only the total is wanted.
    std::vector<PrintingLeafTime> *leaves = nullptr;
    // Location of the extrusion being visited.
    uint32_t tool_idx = 0;
    uint32_t extrusion_idx = 0;
    uint32_t next_leaf_rank = 0;
};

// Return XY arc/line length in mm; invalid arc radii throw invalid_argument.
double planar_segment_length(const c_extrusion_segment &segment);
// Return the Euclidean distance between machine positions in mm.
double distance_3d(const Position3d &lhs, const Position3d &rhs);
// Convert scaled XY and absolute plus inherited/local Z into machine coordinates.
Position3d segment_position(c_point point, coord_t print_z, coord_t z_offset);
// Traverse borrowed children and own geometry, accumulating gaps and deposits;
// advance position without plan mutation, rejecting invalid effective rates.
void estimate_entity(const ExtrusionEntity &entity,
                     const EstimatorState &parent_state,
                     coord_t print_z,
                     double travel_speed,
                     std::optional<Position3d> &position,
                     LayerAccumulator &layer);
// Reset geometric ranks for one extrusion and accumulate its borrowed tree.
void estimate_extrusion(const ConstPrintingExtrusion &extrusion,
                        coord_t print_z,
                        double travel_speed,
                        std::optional<Position3d> &position,
                        LayerAccumulator &layer);
// Visit tools and extrusions in file order for both total and detailed results.
void estimate_layer(const ConstPrintingLayerGroup &layer_group,
                    double travel_speed,
                    std::optional<Position3d> &position,
                    LayerAccumulator &layer);
// Fill either optional output while carrying position across all groups/layers.
// Layer entry gaps are charged to their destination; totals retain sum order.
void estimate_plan(const ConstPrintingPlan &plan,
                   double travel_speed,
                   std::vector<PrintingLayerTimeEstimate> *totals,
                   std::vector<PrintingLayerTimeDetail> *details);

double planar_segment_length(const c_extrusion_segment &segment)
{
    const double dx = double(segment.point_b.x) - double(segment.point_a.x);
    const double dy = double(segment.point_b.y) - double(segment.point_a.y);
    const double chord = std::hypot(dx, dy);
    if (segment.radius == 0.f)
        return unscaled(chord);

    const double radius = std::abs(double(segment.radius));
    if (radius <= 0.0 || chord > radius * 2.0 + double(SCALED_EPSILON))
        throw std::invalid_argument("PrintingPlan contains an arc with an invalid radius.");

    // Positive radii select the short arc; negative radii select the remaining
    // long arc between the same endpoints.
    const double ratio = std::clamp(chord / (2.0 * radius), 0.0, 1.0);
    double angle = 2.0 * std::asin(ratio);
    if (segment.radius < 0.f)
        angle = 2.0 * PI - angle;
    return unscaled(radius * angle);
}

double distance_3d(const Position3d &lhs, const Position3d &rhs)
{
    return std::sqrt((rhs.x - lhs.x) * (rhs.x - lhs.x) +
                     (rhs.y - lhs.y) * (rhs.y - lhs.y) +
                     (rhs.z - lhs.z) * (rhs.z - lhs.z));
}

Position3d segment_position(c_point point, coord_t print_z, coord_t z_offset)
{
    return Position3d{unscaled(point.x), unscaled(point.y), unscaled(print_z + z_offset)};
}

void estimate_entity(const ExtrusionEntity &entity,
                     const EstimatorState &parent_state,
                     coord_t print_z,
                     double travel_speed,
                     std::optional<Position3d> &position,
                     LayerAccumulator &layer)
{
    // Resolve the same inherited process fields that a firmware session sees
    // before descending into children or measuring this node's local path.
    EstimatorState state = parent_state;
    if (const EPropertySpeed *speed = entity.get(EPropertySpeed::key))
        if (speed->speed_mm_per_s > 0.f)
            state.speed_mm_per_s = speed->speed_mm_per_s;
    if (const EPropertyZOffset *z = entity.get(EPropertyZOffset::key))
        state.z_offset = z->get();
    if (const EPropertyAttributes *attributes = entity.get(EPropertyAttributes::key))
        state.mm3_per_mm = attributes->mm3_per_mm();

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        estimate_entity(entity.child(child_idx), state, print_z, travel_speed, position, layer);

    // Timing reaches fan selection before G-code pre-validation, so reject
    // missing deposit positions and invalid volumes before they affect duration.
    if (entity.is_deposit()) {
        const double volume = entity.get(EPropertySpecialCommand::key)->extra_data;
        if (entity.point_count() != 1 || !std::isfinite(volume) || volume <= 0.0)
            throw std::invalid_argument("A deposit needs exactly one point and a finite positive volume.");
    }
    if (entity.segment_count() == 0 && !entity.is_deposit())
        return;
    if (state.speed_mm_per_s <= 0.f || !std::isfinite(state.speed_mm_per_s))
        throw std::invalid_argument("A printable PrintingPlan leaf has no positive effective speed.");

    const Position3d first_position = segment_position(
        entity.point(0), print_z, state.z_offset + entity.z_offset(0));

    // The first known point establishes machine position for free. Every later
    // gap large enough to survive travel-plugin snapping receives a synthetic
    // connector at configured travel speed.
    const double continuity_epsilon = unscaled(double(SCALED_EPSILON));
    if (position && distance_3d(*position, first_position) >= continuity_epsilon) {
        if (travel_speed <= 0.0 || !std::isfinite(travel_speed))
            throw std::invalid_argument("Synthetic PrintingPlan travel needs a positive travel_speed.");
        const double connector = distance_3d(*position, first_position) / travel_speed;
        layer.duration += connector;
        if (layer.unattributed)
            *layer.unattributed += connector;
    }
    position = first_position;

    // Multiplier scales both the volume and its rate, so it cancels from duration.
    if (entity.is_deposit()) {
        if (state.mm3_per_mm <= 0.0 || !std::isfinite(state.mm3_per_mm))
            throw std::invalid_argument("A deposit needs positive effective mm3_per_mm.");
        const double deposit_duration = entity.get(EPropertySpecialCommand::key)->extra_data /
            (double(state.speed_mm_per_s) * state.mm3_per_mm);
        layer.duration += deposit_duration;
        if (layer.unattributed)
            *layer.unattributed += deposit_duration;
        return;
    }
    // The leaf's own duration is accumulated separately for the detailed result;
    // the layer total still receives every segment in order.
    double leaf_duration = 0.0;

    for (uint32_t segment_idx = 0; segment_idx < entity.segment_count(); ++segment_idx) {
        const c_extrusion_segment segment = entity.segment(segment_idx);
        const Position3d destination = segment_position(
            segment.point_b, print_z, state.z_offset + segment.z_offset_b);
        const double planar = planar_segment_length(segment);
        const double dz = destination.z - position->z;
        const double segment_duration = std::hypot(planar, dz) / double(state.speed_mm_per_s);
        layer.duration += segment_duration;
        leaf_duration += segment_duration;
        position = destination;
    }
    if (layer.leaves)
        layer.leaves->push_back(PrintingLeafTime{layer.tool_idx, layer.extrusion_idx, layer.next_leaf_rank,
                                                 leaf_duration, state.speed_mm_per_s});
    ++layer.next_leaf_rank;
}

void estimate_extrusion(const ConstPrintingExtrusion &extrusion,
                        coord_t print_z,
                        double travel_speed,
                        std::optional<Position3d> &position,
                        LayerAccumulator &layer)
{
    layer.next_leaf_rank = 0;
    estimate_entity(extrusion.root(), EstimatorState{}, print_z, travel_speed, position, layer);
}

void estimate_layer(const ConstPrintingLayerGroup &layer_group,
                    double travel_speed,
                    std::optional<Position3d> &position,
                    LayerAccumulator &layer)
{
    for (uint32_t tool_idx = 0; tool_idx < layer_group.tool_group_count(); ++tool_idx) {
        const ConstPrintingToolGroup tool = layer_group.tool_group(tool_idx);
        for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
            layer.tool_idx = tool_idx;
            layer.extrusion_idx = extrusion_idx;
            estimate_extrusion(tool.extrusion(extrusion_idx), layer_group.print_z(),
                               travel_speed, position, layer);
        }
    }
}

void estimate_plan(const ConstPrintingPlan &plan,
                   double travel_speed,
                   std::vector<PrintingLayerTimeEstimate> *totals,
                   std::vector<PrintingLayerTimeDetail> *details)
{
    std::optional<Position3d> position;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerTimeDetail detail;
            detail.group_idx = group_idx;
            detail.layer_group_idx = layer_idx;
            LayerAccumulator layer;
            if (details) {
                layer.unattributed = &detail.unattributed_seconds;
                layer.leaves = &detail.leaves;
            }
            estimate_layer(group.layer_group(layer_idx), travel_speed, position, layer);
            if (totals)
                totals->push_back(PrintingLayerTimeEstimate{group_idx, layer_idx, layer.duration});
            if (details)
                details->push_back(std::move(detail));
        }
    }
}

} // namespace

PrintingPlanTimeEstimator::PrintingPlanTimeEstimator(const Config &print_config) :
    m_travel_speed_mm_per_s(print_config.float_or_default("travel_speed", 0.0))
{}

std::vector<PrintingLayerTimeEstimate> PrintingPlanTimeEstimator::estimate(
    const ConstPrintingPlan &plan) const
{
    std::vector<PrintingLayerTimeEstimate> estimates;
    estimate_plan(plan, m_travel_speed_mm_per_s, &estimates, nullptr);
    return estimates;
}

std::vector<PrintingLayerTimeDetail> PrintingPlanTimeEstimator::estimate_detailed(
    const ConstPrintingPlan &plan) const
{
    std::vector<PrintingLayerTimeDetail> details;
    estimate_plan(plan, m_travel_speed_mm_per_s, nullptr, &details);
    return details;
}

} // namespace slic3r_api
