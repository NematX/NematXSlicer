///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
#include "SupportCounters.hpp"

#include <cstdlib>
#include <cstring>

/* Storage and test controls for support diagnostics. Enablement reads the environment once at startup;
   geometry workers update only relaxed atomics. Tests override the plain enable flag only between joined runs. */

namespace slic3r_api::Support::Common::Counters {
// Reads the process environment once without owning its string; only the exact value "1" enables counts, with no geometry effects.
bool enabled = [] {
    const char *value = std::getenv("SLIC3R_SUPPORT_COUNTERS");
    return value != nullptr && std::strcmp(value, "1") == 0;
}();
Totals totals;

// Stores zero into each process-owned atomic using relaxed ordering; callers must join work first, and no failure occurs.
void reset() noexcept
{
    for (std::atomic<uint64_t> *counter : { &totals.publication_calls, &totals.groups, &totals.islands, &totals.slabs,
         &totals.max_islands_slabs, &totals.intersections, &totals.differences, &totals.unions, &totals.meshes,
         &totals.slicing_calls, &totals.z_slices, &totals.refine_rounds, &totals.tree_bounds_moved,
         &totals.host_bounds_moved, &totals.modulation_layers, &totals.height_changes, &totals.overlap_pairs })
        counter->store(0, std::memory_order_relaxed);
}

// Loads process totals independently in relaxed order into an owned name/value list; join work for a complete run, and allocation failures propagate.
std::vector<std::pair<const char *, uint64_t>> snapshot()
{
    return {
        {"publication_calls", totals.publication_calls.load(std::memory_order_relaxed)},
        {"groups", totals.groups.load(std::memory_order_relaxed)},
        {"islands", totals.islands.load(std::memory_order_relaxed)},
        {"slabs", totals.slabs.load(std::memory_order_relaxed)},
        {"max_islands_slabs", totals.max_islands_slabs.load(std::memory_order_relaxed)},
        {"intersections", totals.intersections.load(std::memory_order_relaxed)},
        {"differences", totals.differences.load(std::memory_order_relaxed)},
        {"unions", totals.unions.load(std::memory_order_relaxed)},
        {"meshes", totals.meshes.load(std::memory_order_relaxed)},
        {"slicing_calls", totals.slicing_calls.load(std::memory_order_relaxed)},
        {"z_slices", totals.z_slices.load(std::memory_order_relaxed)},
        {"refine_rounds", totals.refine_rounds.load(std::memory_order_relaxed)},
        {"tree_bounds_moved", totals.tree_bounds_moved.load(std::memory_order_relaxed)},
        {"host_bounds_moved", totals.host_bounds_moved.load(std::memory_order_relaxed)},
        {"modulation_layers", totals.modulation_layers.load(std::memory_order_relaxed)},
        {"height_changes", totals.height_changes.load(std::memory_order_relaxed)},
        {"overlap_pairs", totals.overlap_pairs.load(std::memory_order_relaxed)}
    };
}

// Changes only the process enable flag and returns its old value without failure; callers must stop all counted work and restore it after the test.
bool set_enabled_for_tests(bool value) noexcept
{
    const bool previous = enabled;
    enabled = value;
    return previous;
}
} // namespace slic3r_api::Support::Common::Counters
