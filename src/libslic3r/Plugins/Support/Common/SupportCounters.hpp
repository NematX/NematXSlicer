///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Process-wide support diagnostics, enabled at startup by exactly SLIC3R_SUPPORT_COUNTERS=1.
   Relaxed atomic totals aggregate joined plugin and host work without changing geometry. Reset and snapshot
   belong between completed runs; snapshots are independent loads, not a transaction. The test override changes
   enablement only while all counted work is stopped, and must restore the previous value before other tests run. */
#ifndef plugins_support_common_counters_hpp_
#define plugins_support_common_counters_hpp_

#include <atomic>
#include <cstdint>
#include <utility>
#include <vector>

namespace slic3r_api::Support::Common::Counters {
extern bool enabled;
struct Totals {
    // Publication calls, including calls with no slabs.
    std::atomic<uint64_t> publication_calls{0};
    // Publication (Z,height) groups whose combined polygons are nonempty.
    std::atomic<uint64_t> groups{0};
    // Sum of published island counts over groups.
    std::atomic<uint64_t> islands{0};
    // Entries in published groups, including empty slabs in those groups.
    std::atomic<uint64_t> slabs{0};
    // Largest island count times slab-entry count in one published group.
    std::atomic<uint64_t> max_islands_slabs{0};
    // Publication per-island intersection calls, including empty inputs.
    std::atomic<uint64_t> intersections{0};
    // Publication overlap difference calls; empty surfaces or covering short-circuit and do not count.
    std::atomic<uint64_t> differences{0};
    // Publication per-island union calls, including empty inputs and native_expolygons normalization; excludes layer-slice unions.
    std::atomic<uint64_t> unions{0};
    // Organic branch path meshes built, including meshes with no requested slice inside their Z span.
    std::atomic<uint64_t> meshes{0};
    // Organic branch mesh slicing calls; a mesh with no intersecting requested Z makes no call.
    std::atomic<uint64_t> slicing_calls{0};
    // Sum of requested Z values over organic mesh slicing calls.
    std::atomic<uint64_t> z_slices{0};
    // Organic slab section-check rounds, including the terminating round with no cuts.
    std::atomic<uint64_t> refine_rounds{0};
    // Tree alignment changed tops, printable bottoms and bottom_z values per pass, including repeated moves.
    std::atomic<uint64_t> tree_bounds_moved{0};
    // STEP_SUPPORT host alignment changed auxiliary-layer tops and bottoms.
    std::atomic<uint64_t> host_bounds_moved{0};
    // Legacy toolpath layer/role executions, including those without overlap candidates.
    std::atomic<uint64_t> modulation_layers{0};
    // Nonempty legacy modulation fragments assigned a height different from their source path.
    std::atomic<uint64_t> height_changes{0};
    // Collected legacy modulation layer pairs per role pass, not XY intersections; empty footprints do not count.
    std::atomic<uint64_t> overlap_pairs{0};
};
extern Totals totals;
// Adds work to a borrowed counter only when enabled; relaxed updates never throw or own geometry.
inline void add(std::atomic<uint64_t> &counter, uint64_t count = 1) noexcept
{
    if (enabled) counter.fetch_add(count, std::memory_order_relaxed);
}
// Retains the largest observed value only when enabled, without owning inputs or throwing.
inline void maximum(std::atomic<uint64_t> &counter, uint64_t value) noexcept
{
    if (enabled) {
        uint64_t old = counter.load(std::memory_order_relaxed);
        while (old < value && !counter.compare_exchange_weak(old, value, std::memory_order_relaxed)) {}
    }
}
// Clears all totals between joined runs, independently of enablement.
void reset() noexcept;
// Returns named totals in stable order between joined runs.
std::vector<std::pair<const char *, uint64_t>> snapshot();
// Overrides enablement between joined runs and returns the value the test must restore.
bool set_enabled_for_tests(bool value) noexcept;
} // namespace slic3r_api::Support::Common::Counters
#endif
