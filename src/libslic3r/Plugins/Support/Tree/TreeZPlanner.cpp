///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "TreeZPlanner.hpp"

#include <algorithm>
#include <limits>
#include <vector>

/* Implementation of the branch slab planner (see TreeZPlanner.hpp).

   plan_branch_slabs() advances from the lower extreme one slab at a time. Each slab height comes from slab_height(), which
   bisects over the grid: a candidate height fits when the largest rate of the object layers that the slab reaches, times
   the height, does not exceed the maximum overhang. Heights fit downward closed, so the bisection finds the largest one.
   The last slab is the rest below the upper extreme; when that rest is thinner than the minimum, rebalance_tail() backs
   off the last boundaries and cuts their span again. The cut is the even one when it fits, else the one that grid_cut()
   finds by dynamic programming over the grid, which can make the top slab the constrained one and spread the rest below. */

namespace Slic3r::TreeFamily {
namespace {
// The planning grid when the machine Z step is finer: 0.01 mm in scaled units (one unit is one nanometer).
constexpr coord_t planning_step = 10000;
// The limits that every slab of a plan must satisfy: the object layers with their rates, the maximum overhang, the
// height bounds (already on the grid) and the grid itself.
struct Limits {
    const std::vector<OverhangLayer> &layers;
    coord_t max_overhang, lowest, highest, grid;
};

// Rounds a non-negative length up to a whole number of grid steps.
coord_t round_up(coord_t value, coord_t grid);
// Rounds a non-negative length down to a whole number of grid steps.
coord_t round_down(coord_t value, coord_t grid);
// Height of the slab that starts at z, the largest grid height in [lowest, highest] that fits its window; lowest when none fits.
coord_t slab_height(coord_t z, coord_t to, const Limits &limits);
// Whether a slab of the given height starting at z fits: the rate of its window, times the height, stays within the maximum overhang.
bool slab_height_fits(coord_t height, coord_t z, coord_t to, const Limits &limits);
// Largest rate among the object layers that overlap the window (from, to); layers are sorted and the search starts at the first one above from.
double window_rate(coord_t from, coord_t to, const std::vector<OverhangLayer> &layers);
// Replaces the boundaries from the last one on, when the rest below to is thinner than the minimum: the span from the nearest kept
// boundary to to is cut again, backing off one more boundary until a cut fits. Without any fitting cut, the last boundary is
// dropped and the span above the one before it becomes a single slab.
void rebalance_tail(std::vector<coord_t> &tops, coord_t from, coord_t to, const Limits &limits);
// The even cut when it fits, else the grid cut. The tops are returned bottom first, the last one is to.
std::vector<coord_t> fitting_cut(coord_t start, coord_t to, const Limits &limits);
// The even cut of [start, to] into the fewest slabs whose heights are within [lowest, highest] and whose overhangs fit; empty
// when no count gives such a cut.
std::vector<coord_t> even_cut(coord_t start, coord_t to, const Limits &limits);
// The cut of [start, to] at the grid positions that dynamic programming finds, under the conditions of even_cut(); empty when
// no cut fits.
std::vector<coord_t> grid_cut(coord_t start, coord_t to, const Limits &limits);
} // namespace

coord_t planning_grid(coord_t z_step)
{
    return std::max(z_step, planning_step);
}

// Cut [from, to] into slabs of the planned heights; the rest below to is added last, or rebalanced with the slabs below it.
std::vector<coord_t> plan_branch_slabs(coord_t from, coord_t to, const std::vector<OverhangLayer> &layers, coord_t max_overhang,
                                       coord_t min_height, coord_t max_height, coord_t z_step)
{
    const coord_t grid = planning_grid(z_step);
    const coord_t lowest = std::max(grid, round_up(min_height, grid));
    const coord_t highest = std::max(lowest, round_down(max_height, grid));
    const Limits limits{ layers, max_overhang, lowest, highest, grid };
    std::vector<coord_t> tops;
    coord_t z = from;
    for (;;) {
        const coord_t height = slab_height(z, to, limits);
        if (to - z <= height) break;
        z += height;
        tops.push_back(z);
    }
    if (to - z < lowest && !tops.empty())
        rebalance_tail(tops, from, to, limits);
    else
        tops.push_back(to);
    return tops;
}

namespace {
coord_t round_up(coord_t value, coord_t grid)
{
    return (value + grid - 1) / grid * grid;
}

coord_t round_down(coord_t value, coord_t grid)
{
    return value / grid * grid;
}

coord_t slab_height(coord_t z, coord_t to, const Limits &limits)
{
    if (!slab_height_fits(limits.lowest, z, to, limits))
        return limits.lowest;
    // Steps above lowest: fitting is the largest step known to fit, failing the smallest step known not to.
    coord_t fitting = 0, failing = (limits.highest - limits.lowest) / limits.grid + 1;
    while (failing - fitting > 1) {
        const coord_t middle = (fitting + failing) / 2;
        if (slab_height_fits(limits.lowest + middle * limits.grid, z, to, limits))
            fitting = middle;
        else
            failing = middle;
    }
    return limits.lowest + fitting * limits.grid;
}

bool slab_height_fits(coord_t height, coord_t z, coord_t to, const Limits &limits)
{
    return window_rate(z, std::min(z + height, to), limits.layers) * double(height) <= double(limits.max_overhang);
}

double window_rate(coord_t from, coord_t to, const std::vector<OverhangLayer> &layers)
{
    std::vector<OverhangLayer>::const_iterator layer = std::upper_bound(layers.begin(), layers.end(), from,
        [](coord_t z, const OverhangLayer &candidate) { return z < candidate.top_z; });
    double rate = 0;
    for (; layer != layers.end() && layer->bottom_z < to; ++layer)
        rate = std::max(rate, layer->rate);
    return rate;
}

void rebalance_tail(std::vector<coord_t> &tops, coord_t from, coord_t to, const Limits &limits)
{
    const size_t count = tops.size();
    for (size_t kept = count; kept-- > 0;) {
        const coord_t start = kept > 0 ? tops[kept - 1] : from;
        const std::vector<coord_t> cut = fitting_cut(start, to, limits);
        if (!cut.empty()) {
            tops.resize(kept);
            tops.insert(tops.end(), cut.begin(), cut.end());
            return;
        }
    }
    tops.resize(count - 1);
    tops.push_back(to);
}

std::vector<coord_t> fitting_cut(coord_t start, coord_t to, const Limits &limits)
{
    std::vector<coord_t> cut = even_cut(start, to, limits);
    return cut.empty() ? grid_cut(start, to, limits) : cut;
}

std::vector<coord_t> even_cut(coord_t start, coord_t to, const Limits &limits)
{
    const coord_t span = to - start;
    const coord_t fewest = std::max<coord_t>(1, (span + limits.highest - 1) / limits.highest);
    for (coord_t count = fewest; count <= span / limits.lowest; ++count) {
        std::vector<coord_t> tops;
        coord_t bottom = start;
        bool fits = true;
        for (coord_t piece = 1; piece <= count && fits; ++piece) {
            const coord_t top = piece == count ? to : start + round_down(span * piece / count, limits.grid);
            const coord_t length = top - bottom;
            fits = length >= limits.lowest && length <= limits.highest &&
                window_rate(bottom, top, limits.layers) * double(length) <= double(limits.max_overhang);
            tops.push_back(top);
            bottom = top;
        }
        if (fits)
            return tops;
    }
    return {};
}

// The positions are the grid steps from start and the grid steps back from to, so a constrained slab can end exactly at to.
// Slabs are the transitions between positions whose lengths are within [lowest, highest] and whose overhangs fit; the first
// transition that reaches a position is kept, so every reached position has a path back to start. The rate of a window grows
// as its upper end moves up, so each window takes its layers in order.
std::vector<coord_t> grid_cut(coord_t start, coord_t to, const Limits &limits)
{
    std::vector<coord_t> positions{ start, to };
    for (coord_t z = start + limits.grid; z < to; z += limits.grid)
        positions.push_back(z);
    for (coord_t z = to - limits.grid; z > start; z -= limits.grid)
        positions.push_back(z);
    std::sort(positions.begin(), positions.end());
    positions.erase(std::unique(positions.begin(), positions.end()), positions.end());
    constexpr size_t unreached = std::numeric_limits<size_t>::max();
    std::vector<size_t> previous(positions.size(), unreached);
    previous[0] = 0;
    const size_t last = positions.size() - 1;
    for (size_t from = 0; from < last; ++from) {
        if (previous[from] == unreached)
            continue;
        std::vector<OverhangLayer>::const_iterator layer = std::upper_bound(limits.layers.begin(), limits.layers.end(), positions[from],
            [](coord_t z, const OverhangLayer &candidate) { return z < candidate.top_z; });
        double rate = 0.;
        for (size_t upper = from + 1; upper <= last; ++upper) {
            const coord_t length = positions[upper] - positions[from];
            if (length > limits.highest)
                break;
            for (; layer != limits.layers.end() && layer->bottom_z < positions[upper]; ++layer)
                rate = std::max(rate, layer->rate);
            if (length >= limits.lowest && previous[upper] == unreached && rate * double(length) <= double(limits.max_overhang))
                previous[upper] = from;
        }
    }
    std::vector<coord_t> tops;
    if (previous[last] == unreached)
        return tops;
    for (size_t idx = last; idx != 0; idx = previous[idx])
        tops.push_back(positions[idx]);
    std::reverse(tops.begin(), tops.end());
    return tops;
}
} // namespace
} // namespace Slic3r::TreeFamily
