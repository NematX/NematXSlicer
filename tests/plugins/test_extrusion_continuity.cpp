/* Extrusion-tree tests check ordered continuity, wrapper simplification and collection access.
   Concurrent collection readers share immutable owned children and compare const accessors against those children.
   Freshly appended collections exercise first-read pointer-cache construction; assertions run only after workers join. */
#include <catch2/catch.hpp>

#include <atomic>
#include <exception>
#include <memory>
#include <thread>
#include <vector>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"

namespace {

// Checks const collection views against its owned children without changing the collection.
bool collection_reads_match(const Slic3r::ExtrusionEntityCollection &collection);

// Borrows a nonempty collection of nonempty paths and compares all const views with its immutable owned children;
// returns false for a size, pointer, iteration or endpoint mismatch, owns nothing and propagates accessor failures.
bool collection_reads_match(const Slic3r::ExtrusionEntityCollection &collection)
{
    const Slic3r::ExtrusionEntity::Children &children = collection.children();
    const Slic3r::ExtrusionEntitiesPtr &entities = collection.entities();
    if (entities.size() != children.size()) return false;
    for (size_t idx = 0; idx < children.size(); ++idx)
        if (entities[idx] != children[idx].get()) return false;
    if (collection.size() != children.size() || collection.empty()) return false;
    Slic3r::ExtrusionEntitiesPtr::const_iterator it = collection.begin();
    const Slic3r::ExtrusionEntitiesPtr::const_iterator end = collection.end();
    for (size_t idx = 0; idx < children.size(); ++idx, ++it) {
        if (it == end || *it != children[idx].get()) return false;
    }
    if (it != end) return false;
    Slic3r::ExtrusionEntitiesPtr::const_iterator cit = collection.cbegin();
    const Slic3r::ExtrusionEntitiesPtr::const_iterator cend = collection.cend();
    for (size_t idx = 0; idx < children.size(); ++idx, ++cit) {
        if (cit == cend || *cit != children[idx].get()) return false;
    }
    return cit == cend && collection.first_point() == children.front()->first_point() &&
        collection.last_point() == children.back()->last_point();
}

Slic3r::ExtrusionPath path_between(const Slic3r::Point &from, const Slic3r::Point &to)
{
    Slic3r::ExtrusionPath path(
        Slic3r::ExtrusionAttributes(
            Slic3r::ExtrusionRole::Perimeter,
            Slic3r::ExtrusionFlow(0.1, 0.4f, 0.2f)),
        nullptr,
        true);
    path.polyline().append(from);
    path.polyline().append(to);
    return path;
}

} // namespace

// Creates owned collections before starting readers, gates each cold read on all eight workers being ready, and checks worker-local
// results after joining. Children outlive every reader; accessor exceptions are retained for the test thread, and startup failures join workers.
TEST_CASE("Concurrent const collection access matches owned children", "[plugins][extrusion][extrusion-collection-threads]")
{
    constexpr size_t collection_count = 200, child_count = 256, worker_count = 8;
    std::vector<std::unique_ptr<Slic3r::ExtrusionEntityCollection>> collections;
    collections.reserve(collection_count);
    for (size_t round = 0; round < collection_count; ++round) {
        std::unique_ptr<Slic3r::ExtrusionEntityCollection> collection = std::make_unique<Slic3r::ExtrusionEntityCollection>();
        for (size_t idx = 0; idx < child_count; ++idx)
            collection->append(path_between(Slic3r::Point(int(idx), 0), Slic3r::Point(int(idx + 1), 0)));
        // Appending leaves the pointer cache empty; no const collection accessor runs before the synchronized readers.
        collections.push_back(std::move(collection));
    }
    std::atomic<size_t> ready{0}, phase{0};
    std::atomic<bool> stop{false};
    std::vector<size_t> mismatches(worker_count, 0);
    std::vector<std::exception_ptr> errors(worker_count);
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    try {
        for (size_t worker = 0; worker < worker_count; ++worker) {
            // Reads each collection only after the release gate, records failures in this worker's slots and owns no geometry;
            // each round reaches the next barrier even after an accessor exception so the coordinator can finish and join.
            workers.emplace_back([&, worker] {
                for (size_t round = 0; round < collection_count; ++round) {
                    ready.fetch_add(1, std::memory_order_release);
                    while (phase.load(std::memory_order_acquire) < round + 1 && !stop.load(std::memory_order_acquire))
                        std::this_thread::yield();
                    if (stop.load(std::memory_order_acquire)) return;
                    try {
                        const Slic3r::ExtrusionEntityCollection &collection = *collections[round];
                        for (size_t pass = 0; pass < 3; ++pass)
                            if (!collection_reads_match(collection)) ++mismatches[worker];
                    } catch (...) {
                        if (!errors[worker]) errors[worker] = std::current_exception();
                    }
                }
            });
        }
    } catch (...) {
        stop.store(true, std::memory_order_release);
        for (std::thread &worker : workers) worker.join();
        throw;
    }
    for (size_t round = 0; round < collection_count; ++round) {
        while (ready.load(std::memory_order_acquire) < (round + 1) * worker_count)
            std::this_thread::yield();
        phase.store(round + 1, std::memory_order_release);
    }
    for (std::thread &worker : workers) worker.join();
    for (size_t worker = 0; worker < worker_count; ++worker) {
        INFO("reader " << worker);
        if (errors[worker]) std::rethrow_exception(errors[worker]);
        CHECK(mismatches[worker] == 0);
    }
}

TEST_CASE("Extrusion continuity is computed from tree order", "[plugins][extrusion][continuity]")
{
    const Slic3r::Point p0(0, 0);
    const Slic3r::Point p1(100, 0);
    const Slic3r::Point p2(200, 0);
    const Slic3r::Point p3(300, 0);

    SECTION("empty entities and leaf polylines are continuous")
    {
        // A leaf has no child order to validate. It is therefore continuous by
        // nature unless a caller explicitly marks it sortable, which is an
        // invalid state for local-polyline entities and rejected by the C API.
        Slic3r::ExtrusionEntity empty(true);
        Slic3r::ExtrusionPath leaf = path_between(p0, p1);

        CHECK(empty.is_continuous());
        CHECK(leaf.is_continuous());
    }

    SECTION("sortable collections are never continuous")
    {
        // Sorting gives path planners permission to reorder children. The
        // current children happen to touch, but the collection is not a forced
        // continuous path while m_can_sort is enabled.
        Slic3r::ExtrusionEntityCollection sortable;
        sortable.append(path_between(p0, p1));
        sortable.append(path_between(p1, p2));

        CHECK(sortable.can_sort());
        CHECK_FALSE(sortable.is_continuous());
    }

    SECTION("non-sortable collections are continuous when children touch")
    {
        // Disabling sorting makes the current child order authoritative. The
        // continuity check then walks non-empty children and verifies that each
        // child ends exactly where the next one starts.
        Slic3r::ExtrusionEntityCollection ordered(false, true);
        ordered.append(path_between(p0, p1));
        ordered.append(path_between(p1, p2));

        CHECK_FALSE(ordered.can_sort());
        CHECK(ordered.is_continuous());
    }

    SECTION("a gap between two children breaks continuity")
    {
        Slic3r::ExtrusionEntityCollection ordered(false, true);
        ordered.append(path_between(p0, p1));
        ordered.append(path_between(p2, p3));

        CHECK_FALSE(ordered.is_continuous());
    }

    SECTION("a continuous closed generic collection is a loop")
    {
        Slic3r::ExtrusionEntityCollection loop(false, true);
        loop.append(path_between(p0, p1));
        loop.append(path_between(p1, p0));

        CHECK(loop.is_continuous());
        CHECK(loop.is_loop());
    }

    SECTION("a non-continuous child makes the parent non-continuous")
    {
        // Continuity is recursive. A parent cannot become continuous just
        // because its child collection has matching first/back points if that
        // child remains sortable internally.
        Slic3r::ExtrusionEntityCollection sortable_child;
        sortable_child.append(path_between(p0, p1));
        sortable_child.append(path_between(p1, p2));

        Slic3r::ExtrusionEntityCollection parent(false, true);
        parent.append(sortable_child);

        CHECK_FALSE(sortable_child.is_continuous());
        CHECK_FALSE(parent.is_continuous());
    }
}

TEST_CASE("Extrusion tree simplification removes only transparent collection wrappers",
          "[plugins][extrusion][simplify]")
{
    const Slic3r::Point p0(0, 0);
    const Slic3r::Point p1(100, 0);
    const Slic3r::Point p2(200, 0);
    const Slic3r::Point p3(300, 0);
    const Slic3r::Point p4(400, 0);

    SECTION("empty transparent collection child is removed")
    {
        // Empty sortable collections sometimes appear as temporary plugin
        // output. If they carry no state, deleting them cannot change the
        // printed geometry or inherited properties.
        Slic3r::ExtrusionEntityCollection root;
        root.append(std::make_unique<Slic3r::ExtrusionEntityCollection>());

        CHECK(simplify_extrusion_tree(root));

        CHECK(root.child_count() == 0);
    }

    SECTION("transparent collection child is spliced into its parent")
    {
        // The wrapper has the same ordering/reversing behavior as its parent
        // and no explicit properties. Its children can therefore inherit from
        // the parent directly after the wrapper is removed.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).first_point() == p0);
        CHECK(root.child(1).first_point() == p2);
    }

    SECTION("unknown explicit child property blocks simplification")
    {
        // A property present on the wrapper but absent from the parent would no
        // longer be inherited by the grandchildren after splicing. The wrapper
        // must stay in the tree.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("different explicit child property blocks simplification")
    {
        // Properties are inherited state. A wrapper that overrides the parent
        // with different bytes is semantically meaningful and cannot be removed.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(20.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("same explicit child property allows simplification")
    {
        // Repeating the exact same property payload on the wrapper does not
        // change what the grandchildren observe. The duplicate wrapper can be
        // removed and the parent remains the source of the inherited property.
        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.add_property(Slic3r::ExtrusionPropertySpeed(10.f));
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).first_point() == p0);
        CHECK(root.child(0).get_property<Slic3r::ExtrusionPropertySpeed>() == nullptr);
        REQUIRE(root.get_property<Slic3r::ExtrusionPropertySpeed>() != nullptr);
        CHECK(root.get_property<Slic3r::ExtrusionPropertySpeed>()->speed_mm_per_s == 10.f);
    }

    SECTION("ordering flags protect collection wrappers")
    {
        // Removing a wrapper with a different sorting contract would give the
        // parent permission to reorder children that were previously protected,
        // or the other way around. Such wrappers are not transparent.
        Slic3r::ExtrusionEntityCollection wrapper(false, true);
        wrapper.append(path_between(p0, p1));
        wrapper.append(path_between(p2, p3));
        REQUIRE(wrapper.is_collection());

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 1);
        CHECK(root.child(0).is_collection());
    }

    SECTION("grandchild properties survive splicing")
    {
        // Simplification moves existing child objects; it must not clone them
        // into new leaves or drop properties carried by deeper nodes.
        Slic3r::ExtrusionPath first = path_between(p0, p1);
        first.add_property(Slic3r::ExtrusionPropertySpeed(30.f));

        Slic3r::ExtrusionEntityCollection wrapper;
        wrapper.append(std::move(first));
        wrapper.append(path_between(p2, p3));

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(wrapper));

        CHECK(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        const Slic3r::ExtrusionPropertySpeed *speed =
            root.child(0).get_property<Slic3r::ExtrusionPropertySpeed>();
        REQUIRE(speed != nullptr);
        CHECK(speed->speed_mm_per_s == 30.f);
    }

    SECTION("loops and continuous multipaths are left intact")
    {
        // Closed and continuous non-sortable structures encode printable path
        // order. Even if they contain several child paths, they are not plain
        // structural collections and are therefore outside this simplifier.
        Slic3r::ExtrusionEntityCollection loop(false, true);
        loop.append(path_between(p0, p1));
        loop.append(path_between(p1, p0));
        REQUIRE(loop.is_loop());

        Slic3r::ExtrusionEntityCollection multipath(false, true);
        multipath.append(path_between(p2, p3));
        multipath.append(path_between(p3, p4));
        REQUIRE(multipath.is_continuous());
        REQUIRE_FALSE(multipath.is_loop());

        Slic3r::ExtrusionEntityCollection root;
        root.append(std::move(loop));
        root.append(std::move(multipath));

        CHECK_FALSE(simplify_extrusion_tree(root));

        REQUIRE(root.child_count() == 2);
        CHECK(root.child(0).is_loop());
        CHECK(root.child(0).child_count() == 2);
        CHECK(root.child(1).is_continuous());
        CHECK(root.child(1).child_count() == 2);
    }
}
