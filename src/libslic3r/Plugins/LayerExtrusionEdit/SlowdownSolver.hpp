///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_LayerExtrusionEdit_SlowdownSolver_hpp_
#define slic3r_Plugins_LayerExtrusionEdit_SlowdownSolver_hpp_

#include <vector>

/*
Short-layer slowdown solver (pure numerical core)
=================================================

Private header of the host plugin `layer_extrusion_edit.speed.slowdown`. It
knows nothing about printing plans, configuration or the plugin SDK: the
plugin adapter converts one layer into the plain structures below, calls
solve(), and writes the returned speeds back.

Vocabulary
----------
- line:     one slowable move of the plan, i.e. one geometric leaf (a "sheet" of
            the extrusion tree). It is the equivalent of one legacy
            `_EXTRUDE_SET_SPEED` ... `_EXTRUDE_END` block of the former
            CoolingBuffer: a single path whose moves all share one speed. Its
            length is constant (`seconds * speed`), so changing its speed
            stretches its duration proportionally.
- fixed:    time that the solver never changes. A line is fixed when it is not
            `adjustable` (overhang, bridge, travel... decided by the adapter) or
            when it already runs at or below its minimum speed, or when its
            maximum speed reduction leaves no room to slow it further (the
            minimum speed and the reduction cap both bound a line's duration).
- unassigned: time of the layer that no line owns (fixed deposits, travel entering
             the layer and synthetic connectors between leaves); never slowed.

Behaviour
---------
It reproduces the non-proportional logic of the legacy CoolingBuffer
(`calculate_layer_slowdown` + `extruder_range_slow_down_non_proportional`):
extruders are handled by increasing `threshold_seconds`; while the layer is
shorter than the threshold, the fastest lines are slowed first down to the next
speed ("equalize from the top"), never below `min_print_speed` nor beyond
`max_speed_reduction`, and everything goes to the minimum speed when the
threshold cannot be reached.

Two deliberate differences from the legacy code:
1. With several thresholds the current durations (after the slowdowns already
   applied for lower thresholds) are used, not the durations frozen at the
   start of the layer.
2. The stretch obtained by slowing lines to a target speed is capped by each
   line's maximum duration (`min(time * (speed / target - 1), time_max - time)`),
   and the target speed is solved with that same capped formula, so the
   threshold is reached whenever it is reachable.

The solver is deterministic: ties between lines keep their input order and ties
between extruders keep their input order.

Build a Layer with one Extruder per tool, then call solve(layer). The returned
lines retain the input shape; write speeds only where LineResult::slowed is true.
The input remains unchanged, all work is local, and invalid numbers throw before
solving. No plan handles or mutable global state are retained.

    Layer layer;
    layer.extruders.push_back(extruder);
    const Result result = solve(layer);
*/

namespace slic3r_api { namespace LayerExtrusionEdit { namespace SlowdownSolver {

// One slowable move: its duration and speed before the solver runs.
struct Line
{
    // Duration at `speed_mm_per_s` (seconds, > 0 for an adjustable line).
    double seconds = 0.0;
    float speed_mm_per_s = 0.f;
    // False for lines that must keep their speed (they still count in the time).
    bool adjustable = false;
};

// All the moves printed by one extruder during the layer.
struct Extruder
{
    // Layer-time goal in seconds (>= 0); 0 disables the slowdown for this extruder.
    double threshold_seconds = 0.0;
    // Lowest speed allowed (mm/s); 0 means no lower bound.
    double min_print_speed = 0.0;
    // Largest relative speed reduction in [0, 1]; 0 and 1 both mean "no bound".
    double max_speed_reduction = 0.0;
    // Time of this extruder that is not described by `lines`.
    double fixed_seconds = 0.0;
    std::vector<Line> lines;
};

// One layer as seen by the solver.
struct Layer
{
    double unassigned_seconds = 0.0;
    std::vector<Extruder> extruders;
};

// Outcome for one input line.
struct LineResult
{
    // True when the line must receive `speed_mm_per_s`; false lines are untouched.
    bool slowed = false;
    float speed_mm_per_s = 0.f;
};

struct Result
{
    // Same shape as Layer::extruders[i].lines.
    std::vector<std::vector<LineResult>> lines;
    // Layer duration once the returned speeds are applied (float-rounded speeds).
    double total_seconds = 0.0;
};

// Compute the slowed speeds of one layer. Throws std::invalid_argument, with an
// explanatory message, when a value is negative, not finite or out of range.
Result solve(const Layer &layer);

}}} // namespace slic3r_api::LayerExtrusionEdit::SlowdownSolver

#endif // slic3r_Plugins_LayerExtrusionEdit_SlowdownSolver_hpp_
