// Native fill publication retains priority wrappers and per-extruder bucket ownership.
// Call store_fill after generation; cached trees are owned, while surfaces and regions are borrowed.
// Example: FillCacheDetail::store_fill(caches, island, regions);

#ifndef slic3r_FillCache_hpp_
#define slic3r_FillCache_hpp_

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace Slic3r {

class ExtrusionEntity;
class LayerRegion;
class LayerSliceIsland;
class Surface;

namespace FillCacheDetail {

// Consume native caches into initially empty infill buckets, preserving priority-specific copies and transfers.
// Requires a nonempty outer priority vector and region set, with borrowed sparse or solid surfaces. Priority rows may be empty.
// Cache roots must already contain ExtrusionEntity::Children, including empty caches.
// One priority transfers only children from a single cache, or nonempty wrappers from several caches; all caches may be empty.
// Multiple priorities transfer singleton wrappers and copy intermediate groups built from rows containing several caches.
// Only in that branch, each multi-cache row needs a nonempty result and each counted family must reach a nonempty root.
// Equal extruders may share a root. Normal return clears all cache owners, including wrappers whose children were transferred.
// Borrows outlive the call. Allocation failures propagate and may leave caches consumed and buckets partly populated.
void store_fill(std::vector<std::vector<std::pair<const Surface *, std::unique_ptr<ExtrusionEntity>>>> &fills_by_priority,
                LayerSliceIsland &island, std::set<const LayerRegion *> regions);

} // namespace FillCacheDetail
} // namespace Slic3r

#endif
