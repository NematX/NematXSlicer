///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Api_plugin_cpp_PrintingPlanTimeEstimator_hpp_
#define slic3r_Api_plugin_cpp_PrintingPlanTimeEstimator_hpp_

#include <cstdint>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

namespace slic3r_api {

/*
Fallback duration estimator for an ordered PrintingPlan.

The estimator measures only movements whose duration can be inferred from the
plan: extrusion/travel geometry, stationary deposits and synthetic travels
between separate leaves.
It deliberately ignores acceleration, scripts, retractions, and machine-side
delays. The result is therefore an estimate, not an authoritative G-code time.

One result is returned for every PrintingLayerGroup in group/layer order. The
machine position is retained between results so the travel entering a layer is
charged to that destination layer.

Construct from print configuration, then call estimate(plan) for totals or
estimate_detailed(plan) for geometric timings plus fixed unattributed time.
Configuration is copied as travel speed; borrowed plan views never escape a call.
Both calls are read-only and throw invalid_argument for malformed geometry or
missing effective rates. The caller must not edit the plan during measurement.
*/
struct PrintingLayerTimeEstimate
{
    uint32_t group_idx = 0;
    uint32_t layer_group_idx = 0;
    double duration_seconds = 0.0;
};

// Duration of one geometric leaf (an extrusion entity that owns segments) at its
// effective speed. The leaf is located by its tool group, its extrusion in that
// tool group and its rank: leaves of one extrusion are numbered 0, 1, ... in the
// order the estimator finishes them (children before the entity's own segments).
struct PrintingLeafTime
{
    uint32_t tool_idx = 0;
    uint32_t extrusion_idx = 0;
    uint32_t leaf_rank = 0;
    // Time spent on this leaf's own segments only.
    double duration_seconds = 0.0;
    // Effective (possibly inherited) speed used to measure the leaf.
    float speed_mm_per_s = 0.f;
};

// Breakdown of one layer's estimate. `unattributed_seconds` covers fixed deposits
// and synthetic connectors charged to the layer (including layer entry).
// Cooling slowdown does not edit these durations. The sum of this
// value and all leaf durations equals PrintingLayerTimeEstimate::duration_seconds
// of the same layer, up to floating-point summation order (relative 1e-9).
struct PrintingLayerTimeDetail
{
    uint32_t group_idx = 0;
    uint32_t layer_group_idx = 0;
    double unattributed_seconds = 0.0;
    std::vector<PrintingLeafTime> leaves;
};

class PrintingPlanTimeEstimator
{
public:
    // Copy travel speed without retaining the borrowed configuration.
    explicit PrintingPlanTimeEstimator(const Config &print_config);

    // Return one total per layer, preserving serialized plan order and position.
    std::vector<PrintingLayerTimeEstimate> estimate(const ConstPrintingPlan &plan) const;
    // Same traversal as estimate(), but returning the per-leaf breakdown.
    std::vector<PrintingLayerTimeDetail> estimate_detailed(const ConstPrintingPlan &plan) const;

private:
    double m_travel_speed_mm_per_s = 0.0;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PrintingPlanTimeEstimator_hpp_
