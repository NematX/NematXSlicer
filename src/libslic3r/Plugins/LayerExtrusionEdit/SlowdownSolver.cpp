///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
Short-layer slowdown solver: implementation
===========================================

Vocabulary (details in SlowdownSolver.hpp): a "line" is one slowable move of the
plan (a path whose moves share one speed); "fixed" time is time the solver never
changes (non-adjustable lines, lines already at their limits, time outside any
line); "unassigned" time belongs to no line (fixed deposits and travels). The goal of a layer is
`threshold_seconds`; the "stretch" is the number of seconds still missing.
The two deliberate differences with the legacy CoolingBuffer are listed in the
header and marked "difference 1" / "difference 2" where they apply.

How solve() works
-----------------
1. validate() rejects impossible numbers, then build_work() converts the input
   into doubles with, for every adjustable line, its constant length
   (`seconds * speed`) and its maximum duration `time_max`:
       time_max = max(time, length / min_print_speed)          (infinite if the
                                                                 minimum is 0)
       time_max = min(time_max, time / (1 - max_speed_reduction))  (if 0 < r < 1)
   A line whose `time_max` is not meaningfully larger than its duration is not
   slowable and is treated as fixed (it already runs at its minimum speed or at
   the maximum reduction, or it is not adjustable at all).
2. slow_layer() visits the extruders that can be slowed by increasing
   threshold. For each one, if the CURRENT layer duration (x 1.001, the legacy
   tolerance) is below the threshold, it slows this extruder and all the ones
   with higher thresholds:
   - to their minimum speed when even that does not reach the threshold;
   - otherwise with equalize(), which lowers the fastest lines first.
3. equalize() repeats: find the fastest slowable speed `speed`, the next lower
   speed `next` among all lines, and slow every line faster than
   max(next, min_print_speed) down to that speed, until the missing time (the
   "stretch") is produced. When the stretch is reached before the next speed
   level, solve_speed() finds the exact common target speed. Extruders whose
   min_print_speed is 0 have no speed floor: their slowable lines then share one
   duration factor (slow_proportionally_by_stretch()), found so that lines which
   hit their reduction cap early do not make the goal fall short.
4. build_result() turns the slowed lines into float speeds (the type stored in
   the plan) and recomputes the layer duration with those rounded speeds.

All the work is done on local copies; the input is never modified.

    solve()
    |-- validate(): reject invalid durations, bounds and speeds
    |-- build_work(): copy lines and derive maximum durations
    |-- slow_layer(): visit increasing thresholds using current layer time
    |   |-- slow_to_minimum(): saturate when the goal is unreachable
    |   `-- equalize(): lower fastest lines until the missing time is gained
    |       |-- solve_speed(): bisect the capped gain for a common speed
    |       `-- slow_proportionally_by_stretch(): handle absent speed floors
    `-- build_result(): round output speeds and recompute their actual duration
*/
#include "SlowdownSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace slic3r_api { namespace LayerExtrusionEdit { namespace SlowdownSolver {
namespace {

// Same absolute speed tolerance as the legacy code (EPSILON in libslic3r.h).
constexpr double k_speed_epsilon = 1e-4;
// A layer within 0.1 % of its threshold is considered long enough (legacy).
constexpr double k_threshold_tolerance = 1.001;
// A line must be extensible by more than this relative amount to be slowable;
// it avoids slowing a line that already runs at its minimum speed because of
// floating-point noise in `length / min_print_speed`.
constexpr double k_extension_tolerance = 1e-9;
constexpr double k_unbounded = std::numeric_limits<double>::infinity();
// Bisections stop when the bracket is narrower than this relative width: speeds
// are stored as float (24-bit mantissa, relative step 6e-8), so a finer result
// could not be written anyway. 60 halvings are a hard upper bound.
constexpr double k_bisection_tolerance = 1e-7;
constexpr int k_bisection_steps = 60;

// Mutable copy of one line.
struct WorkLine
{
    double length = 0.0;
    double time = 0.0;
    double time_max = 0.0;
    double speed = 0.0;
    // Slowable in principle (adjustable and able to get slower).
    bool candidate = false;
    bool slowed = false;
};

// Mutable copy of one extruder.
struct WorkExtruder
{
    double threshold = 0.0;
    double min_print_speed = 0.0;
    double reduction = 0.0;
    double fixed = 0.0;
    std::vector<WorkLine> lines;
    // Indexes of the candidate lines, fastest first (stable for equal speeds).
    std::vector<size_t> by_speed;
    // [span_begin, span_end) of by_speed: the lines currently at the top speed.
    size_t span_begin = 0;
    size_t span_end = 0;
    // True while the extruder belongs to the range being slowed.
    bool in_range = false;
};

using ExtruderRange = std::vector<WorkExtruder *>;

// Reject nonfinite, negative or out-of-range input with a field-specific error.
void require(bool ok, const char *what);
// Validate the numeric input before allocating mutable work.
void validate(const Layer &layer);
// Copy lines and derive their maximum durations and stable fastest-first order.
std::vector<WorkExtruder> build_work(const Layer &layer);
// Apply increasing tool thresholds using the current duration after each step.
// Tools with no goal or no slowable line add fixed time and never enter the range.
void slow_layer(double unassigned, std::vector<WorkExtruder> &work);
// Sum fixed time and all current line durations across the layer.
double layer_seconds(double unassigned, const std::vector<WorkExtruder> &work);
// Return the duration if all currently selected lines saturate their bounds.
double maximum_layer_seconds(double unassigned, const std::vector<WorkExtruder> &work);
// Saturate every adjustable line of the selected range at its duration bound.
void slow_to_minimum(ExtruderRange::iterator first, ExtruderRange::iterator last);
// Lower fastest lines first until the missing time is gained or bounds saturate.
void equalize(ExtruderRange::iterator first, ExtruderRange::iterator last, double stretch);
// Return the gain at a common speed, capped by every line's maximum duration.
double stretch_to_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double speed);
// Bisect the capped gain to find a common speed producing the requested stretch.
// Gain decreases with speed; unlike the legacy fixed-point search, each trial
// respects duration caps before deciding which side of the bracket to keep.
double solve_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double lowest, double stretch);
// Return the capped gain from multiplying adjustable durations by one factor.
double proportional_gain(ExtruderRange::iterator first, ExtruderRange::iterator last, double rate);
// Solve a shared duration factor when no minimum speed bounds the range. The
// capped gain grows with the factor, even when some lines saturate early.
// If all lines are bounded, the largest time_max/time gives an upper bracket;
// unreachable goals saturate every line. If a line is unbounded, the uncapped
// factor is a lower bound and doubling finds the upper bracket.
void slow_proportionally_by_stretch(ExtruderRange::iterator first, ExtruderRange::iterator last, double stretch);
// Lower faster lines to the target speed without exceeding their duration bounds.
void slow_to_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double speed);
// Sum the current durations of lines that can still be slowed.
double adjustable_seconds(ExtruderRange::iterator first, ExtruderRange::iterator last);
// Multiply adjustable durations by the factor, capped by each line's maximum.
void slow_proportionally(ExtruderRange::iterator first, ExtruderRange::iterator last, double rate);
// Round output speeds to float and recompute timing from the written values.
Result build_result(const Layer &layer, const std::vector<WorkExtruder> &work);
// Identify a candidate whose duration has not yet saturated its bound.
bool adjustable_now(const WorkLine &line);

void require(bool ok, const char *what)
{
    if (!ok)
        throw std::invalid_argument(std::string("Invalid slowdown input: ") + what + ".");
}

void validate(const Layer &layer)
{
    require(std::isfinite(layer.unassigned_seconds) && layer.unassigned_seconds >= 0.0, "unassigned seconds");
    for (const Extruder &extruder : layer.extruders) {
        require(std::isfinite(extruder.threshold_seconds) && extruder.threshold_seconds >= 0.0, "threshold");
        require(std::isfinite(extruder.min_print_speed) && extruder.min_print_speed >= 0.0, "min_print_speed");
        require(std::isfinite(extruder.max_speed_reduction) &&
                extruder.max_speed_reduction >= 0.0 && extruder.max_speed_reduction <= 1.0, "max_speed_reduction");
        require(std::isfinite(extruder.fixed_seconds) && extruder.fixed_seconds >= 0.0, "fixed seconds");
        for (const Line &line : extruder.lines) {
            require(std::isfinite(line.seconds) && line.seconds >= 0.0, "line seconds");
            require(std::isfinite(line.speed_mm_per_s) && line.speed_mm_per_s >= 0.f, "line speed");
        }
    }
}

bool adjustable_now(const WorkLine &line)
{
    return line.candidate && line.time < line.time_max;
}

std::vector<WorkExtruder> build_work(const Layer &layer)
{
    std::vector<WorkExtruder> work(layer.extruders.size());
    for (size_t e = 0; e < work.size(); ++e) {
        const Extruder &in = layer.extruders[e];
        WorkExtruder &out = work[e];
        out.threshold = in.threshold_seconds;
        out.min_print_speed = in.min_print_speed;
        out.reduction = in.max_speed_reduction;
        out.fixed = in.fixed_seconds;
        out.lines.resize(in.lines.size());
        for (size_t i = 0; i < in.lines.size(); ++i) {
            WorkLine &line = out.lines[i];
            line.time = in.lines[i].seconds;
            line.speed = double(in.lines[i].speed_mm_per_s);
            line.length = line.time * line.speed;
            line.time_max = line.time;
            if (!in.lines[i].adjustable || line.time <= 0.0 || line.speed <= 0.0)
                continue;
            line.time_max = out.min_print_speed == 0.0 ? k_unbounded :
                std::max(line.time, line.length / out.min_print_speed);
            // A reduction of 0 (disabled) or 1 (100 %) bounds nothing.
            if (out.reduction > 0.0 && out.reduction < 1.0)
                line.time_max = std::min(line.time_max, line.time / (1.0 - out.reduction));
            line.candidate = line.time_max > line.time * (1.0 + k_extension_tolerance);
            if (!line.candidate)
                line.time_max = line.time;
            else
                out.by_speed.push_back(i);
        }
        std::stable_sort(out.by_speed.begin(), out.by_speed.end(), [&out](size_t a, size_t b) {
            return out.lines[a].speed > out.lines[b].speed;
        });
    }
    return work;
}

double layer_seconds(double unassigned, const std::vector<WorkExtruder> &work)
{
    double total = unassigned;
    for (const WorkExtruder &extruder : work) {
        total += extruder.fixed;
        for (const WorkLine &line : extruder.lines)
            total += line.time;
    }
    return total;
}

double maximum_layer_seconds(double unassigned, const std::vector<WorkExtruder> &work)
{
    double total = unassigned;
    for (const WorkExtruder &extruder : work) {
        total += extruder.fixed;
        for (const WorkLine &line : extruder.lines)
            total += (extruder.in_range && adjustable_now(line)) ? line.time_max : line.time;
    }
    return total;
}

void slow_layer(double unassigned, std::vector<WorkExtruder> &work)
{
    ExtruderRange by_threshold;
    for (WorkExtruder &extruder : work)
        if (extruder.threshold > 0.0 && !extruder.by_speed.empty())
            by_threshold.push_back(&extruder);
    std::stable_sort(by_threshold.begin(), by_threshold.end(), [](const WorkExtruder *a, const WorkExtruder *b) {
        return a->threshold < b->threshold;
    });

    for (size_t k = 0; k < by_threshold.size(); ++k) {
        const double threshold = by_threshold[k]->threshold;
        // Difference 1 with the legacy code: current durations, not frozen ones.
        const double total = layer_seconds(unassigned, work);
        if (total * k_threshold_tolerance >= threshold)
            continue;
        // This extruder and every one with a higher threshold are slowed together.
        for (size_t j = 0; j < by_threshold.size(); ++j)
            by_threshold[j]->in_range = j >= k;
        ExtruderRange::iterator first = by_threshold.begin() + std::ptrdiff_t(k);
        if (maximum_layer_seconds(unassigned, work) > threshold)
            equalize(first, by_threshold.end(), threshold - total);
        else
            slow_to_minimum(first, by_threshold.end());
    }
}

void slow_to_minimum(ExtruderRange::iterator first, ExtruderRange::iterator last)
{
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            WorkLine &line = (*it)->lines[index];
            if (!adjustable_now(line))
                continue;
            line.time = line.time_max;
            line.speed = line.length / line.time;
            line.slowed = true;
        }
}

void equalize(ExtruderRange::iterator first, ExtruderRange::iterator last, double stretch)
{
    ExtruderRange order(first, last);
    // Fastest slowable speed over the whole range.
    double speed = 0.0;
    for (WorkExtruder *extruder : order) {
        extruder->span_begin = extruder->span_end = 0;
        speed = std::max(speed, extruder->lines[extruder->by_speed.front()].speed);
    }
    if (speed <= 0.0)
        return;
    // Extruders with the highest lower bound first, as in the legacy sort.
    std::stable_sort(order.begin(), order.end(), [](const WorkExtruder *a, const WorkExtruder *b) {
        return (1.0 - a->reduction) * a->min_print_speed > (1.0 - b->reduction) * b->min_print_speed;
    });

    for (;;) {
        // Span of each extruder's lines running at (about) the current top speed.
        for (WorkExtruder *extruder : order) {
            extruder->span_end = extruder->span_begin;
            while (extruder->span_end < extruder->by_speed.size() &&
                   extruder->lines[extruder->by_speed[extruder->span_end]].speed > speed - k_speed_epsilon)
                ++extruder->span_end;
        }
        // Next lower speed among the lines left below the span.
        double next_speed = 0.0;
        for (const WorkExtruder *extruder : order)
            if (extruder->span_end < extruder->by_speed.size())
                next_speed = std::max(next_speed, extruder->lines[extruder->by_speed[extruder->span_end]].speed);

        for (ExtruderRange::iterator it = order.begin(); it != order.end();) {
            const WorkExtruder &extruder = **it;
            if (extruder.min_print_speed == 0.0) {
                // No lower bound: all remaining lines share one proportional factor.
                slow_proportionally_by_stretch(it, order.end(), stretch);
                return;
            }
            double limit = std::max(next_speed, extruder.min_print_speed);
            const double reachable = stretch_to_speed(it, order.end(), limit);
            const bool done = reachable >= stretch;
            if (done)
                limit = solve_speed(it, order.end(), limit, stretch);
            else
                stretch -= reachable;
            slow_to_speed(it, order.end(), limit);
            if (done)
                return;
            // Extruders with the same bounds were slowed by the call above.
            ExtruderRange::iterator following = it + 1;
            while (following != order.end() &&
                   (*following)->min_print_speed > extruder.min_print_speed - k_speed_epsilon &&
                   (*following)->reduction < extruder.reduction + k_speed_epsilon)
                ++following;
            it = following;
        }
        if (next_speed == 0.0)
            break; // nothing left to slow down
        for (WorkExtruder *extruder : order)
            extruder->span_begin = extruder->span_end;
        speed = next_speed;
    }
}

double stretch_to_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double speed)
{
    double stretch = 0.0;
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            const WorkLine &line = (*it)->lines[index];
            if (line.speed > speed)
                stretch += std::max(0.0, std::min(line.time * (line.speed / speed - 1.0),
                                                  line.time_max - line.time));
        }
    return stretch;
}

double solve_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double lowest, double stretch)
{
    double high = lowest;
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed)
            high = std::max(high, (*it)->lines[index].speed);
    double low = lowest;
    for (int step = 0; step < k_bisection_steps && high - low > k_bisection_tolerance * high; ++step) {
        const double middle = 0.5 * (low + high);
        if (stretch_to_speed(first, last, middle) >= stretch)
            low = middle;
        else
            high = middle;
    }
    return low;
}

void slow_to_speed(ExtruderRange::iterator first, ExtruderRange::iterator last, double speed)
{
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            WorkLine &line = (*it)->lines[index];
            if (line.speed <= speed)
                continue;
            const double stretched = line.time * (line.speed / speed);
            if (stretched >= line.time_max) {
                line.time = line.time_max;
                line.speed = line.length / line.time;
            } else {
                line.time = stretched;
                line.speed = speed;
            }
            line.slowed = true;
        }
}

double adjustable_seconds(ExtruderRange::iterator first, ExtruderRange::iterator last)
{
    double total = 0.0;
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed)
            if (adjustable_now((*it)->lines[index]))
                total += (*it)->lines[index].time;
    return total;
}

void slow_proportionally(ExtruderRange::iterator first, ExtruderRange::iterator last, double rate)
{
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            WorkLine &line = (*it)->lines[index];
            if (!adjustable_now(line))
                continue;
            line.time = rate == k_unbounded ? line.time_max : std::min(line.time_max, line.time * rate);
            line.speed = line.length / line.time;
            line.slowed = true;
        }
}

double proportional_gain(ExtruderRange::iterator first, ExtruderRange::iterator last, double rate)
{
    double gain = 0.0;
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            const WorkLine &line = (*it)->lines[index];
            if (adjustable_now(line))
                gain += std::min(line.time_max, line.time * rate) - line.time;
        }
    return gain;
}

void slow_proportionally_by_stretch(ExtruderRange::iterator first, ExtruderRange::iterator last, double stretch)
{
    const double adjustable = adjustable_seconds(first, last);
    if (adjustable <= 0.0)
        return;
    bool unbounded = false;
    double maximum_gain = 0.0;
    double maximum_ratio = 1.0;
    for (ExtruderRange::iterator it = first; it != last; ++it)
        for (size_t index : (*it)->by_speed) {
            const WorkLine &line = (*it)->lines[index];
            if (!adjustable_now(line))
                continue;
            if (line.time_max == k_unbounded) {
                unbounded = true;
            } else {
                maximum_gain += line.time_max - line.time;
                maximum_ratio = std::max(maximum_ratio, line.time_max / line.time);
            }
        }

    double high = (adjustable + stretch) / adjustable;
    if (!unbounded) {
        if (maximum_gain < stretch)
            high = k_unbounded;
        else
            high = maximum_ratio;
    } else {
        for (int step = 0; step < k_bisection_steps && proportional_gain(first, last, high) < stretch; ++step)
            high *= 2.0;
    }
    if (high != k_unbounded) {
        double low = 1.0;
        for (int step = 0; step < k_bisection_steps && high - low > k_bisection_tolerance * high; ++step) {
            const double middle = 0.5 * (low + high);
            if (proportional_gain(first, last, middle) >= stretch)
                high = middle;
            else
                low = middle;
        }
    }
    slow_proportionally(first, last, high);
}
Result build_result(const Layer &layer, const std::vector<WorkExtruder> &work)
{
    Result result;
    result.lines.resize(work.size());
    result.total_seconds = layer.unassigned_seconds;
    for (size_t e = 0; e < work.size(); ++e) {
        result.total_seconds += work[e].fixed;
        result.lines[e].resize(work[e].lines.size());
        for (size_t i = 0; i < work[e].lines.size(); ++i) {
            const WorkLine &line = work[e].lines[i];
            LineResult &out = result.lines[e][i];
            if (!line.slowed) {
                out.speed_mm_per_s = layer.extruders[e].lines[i].speed_mm_per_s;
                result.total_seconds += line.time;
                continue;
            }
            out.slowed = true;
            out.speed_mm_per_s = float(line.speed);
            result.total_seconds += line.length / double(out.speed_mm_per_s);
        }
    }
    return result;
}

} // namespace

Result solve(const Layer &layer)
{
    validate(layer);
    std::vector<WorkExtruder> work = build_work(layer);
    slow_layer(layer.unassigned_seconds, work);
    return build_result(layer, work);
}

}}} // namespace slic3r_api::LayerExtrusionEdit::SlowdownSolver
