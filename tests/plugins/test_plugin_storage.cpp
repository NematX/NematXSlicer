/* PluginStorage tracks allocations created through the C API. The registry is
   shared, but each worker edits only its own objects. The stress test starts
   eight threads together, keeps handles alive while other threads allocate and
   free, and checks the final registry after joining. The execution test exercises
   the same sharing through four parallel STEP_SUPPORT plugin runs. These tests check
   exact counts and membership; exposing an unlocked race depends on scheduling.

   The registry tests below check the constant-time free: every stored type, mixed batches freed out of
   order (a counted extrusion subclass proves which object was destroyed), configs whose handle differs
   from the object address, the insertion order of stored_extrusions(), clear() followed by reuse, and
   concurrent frees. A 100,000-object case checks exact content; its timing only lives in the hidden
   [plugin-storage-benchmark] case. Freeing a null, unknown or already freed handle asserts in Debug
   (the project only builds Debug and Catch2 has no death test), so that path is not tested dynamically. */
#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace {
using namespace Slic3r;
std::atomic_uint completed_objects {0};

class StorageProbe final : public slic3r_api::PluginBase
{
public:
    // Connect the recording provider to the test orchestrator.
    explicit StorageProbe(orchestrator_handle *orch) : PluginBase(orch) {}
private:
    // Identify the provider for activation in this test only.
    const char *id_impl() const noexcept override { return "test.plugin_storage.support"; }
    // Exercise the object runner without adding support geometry.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // Run this allocation probe at the default priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // No other plugin is needed for the allocation probe.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Check one object's payload and repeatedly clip temporary polygons in the shared storage; failures use the plugin error channel.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(run_ctx);
        if (payload == nullptr || payload->object == nullptr)
            throw std::runtime_error("Missing support object for storage probe");
        const ExPolygons areas {ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                                   Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}))};
        const slic3r_api::ClipperContext clipper(run_ctx->plugin_storage);
        for (size_t idx = 0; idx < 256; ++idx) {
            const slic3r_api::StoredExPolygonCollection clipped = slic3r_api::clipper_offset(
                clipper(slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&areas))),
                scale_d(0.01)).to_expolygon_collection();
            if (clipped.size() != 1)
                throw std::runtime_error("Unexpected temporary clipping result");
        }
        completed_objects.fetch_add(1, std::memory_order_relaxed);
    }
};
} // namespace

namespace Slic3r::Test::Plugins {
// Register the allocation probe before option finalization; registration failure stops test initialization.
void register_plugin_storage_test_provider(orchestrator_handle *orch)
{
    static StorageProbe provider(orch);
    if (!reinterpret_cast<Orchestrator *>(orch)->register_plugin(provider.c_instance()))
        throw std::runtime_error("Cannot register plugin storage probe");
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("Shared plugin storage registers and frees mixed allocations concurrently", "[plugins][plugin-storage]")
{
    constexpr size_t workers = 8;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    void *anchor = storage_new_polygon(handle);
    void *config = storage_new_config(handle);
    REQUIRE(storage.contains(anchor));
    REQUIRE(storage.contains(config));
    std::array<void *, workers> retained {};
    std::array<std::exception_ptr, workers> failures {};
    std::vector<std::thread> threads;
    std::mutex start_mutex;
    std::condition_variable start_signal;
    size_t ready = 0;
    bool start = false;
    for (size_t worker = 0; worker < workers; ++worker)
        threads.emplace_back([&, worker] {
            {
                std::unique_lock<std::mutex> lock(start_mutex);
                ++ready;
                start_signal.notify_all();
                start_signal.wait(lock, [&] { return start; });
            }
            try {
                retained[worker] = storage_new_polygon(handle);
                for (size_t iteration = 0; iteration < 256; ++iteration) {
                    std::vector<void *> allocations {
                        storage_new_polygon(handle), storage_new_polyline(handle), storage_new_expolygon(handle),
                        storage_new_polygons(handle), storage_new_polylines(handle), storage_new_expolygons(handle),
                        storage_new_surface_collection(handle), storage_new_config(handle), extrusion_create_empty(handle)
                    };
                    const slic3r_api::ClipperContext clipper(handle);
                    slic3r_api::ClipperOperand shapes = clipper.empty();
                    for (void *allocation : allocations)
                        if (!storage.contains(allocation))
                            throw std::runtime_error("Allocation absent from shared registry");
                    if (!storage.contains(const_cast<clipper_shapes_handle *>(shapes.handle())) ||
                        storage.size() < allocations.size() + 1)
                        throw std::runtime_error("Clipper allocation absent from shared registry");
                    for (void *allocation : allocations)
                        if (!storage.free(allocation))
                            throw std::runtime_error("Cannot release shared registry allocation");
                }
            } catch (...) {
                failures[worker] = std::current_exception();
            }
        });
    {
        std::unique_lock<std::mutex> lock(start_mutex);
        start_signal.wait(lock, [&] { return ready == workers; });
        start = true;
    }
    start_signal.notify_all();
    for (std::thread &thread : threads)
        thread.join();
    for (const std::exception_ptr &failure : failures)
        if (failure)
            std::rethrow_exception(failure);
    CHECK(storage.size() == workers + 2);
    CHECK(storage.contains(anchor));
    CHECK(storage.contains(config));
    for (void *allocation : retained)
        CHECK(storage.contains(allocation));
    storage.clear();
    CHECK(storage.size() == 0);
    CHECK_FALSE(storage.contains(anchor));
    CHECK_FALSE(storage.contains(config));
    CHECK_FALSE(storage.contains(nullptr));
}

TEST_CASE("Four parallel plugin runs share storage without retaining clipping temporaries", "[plugins][plugin-storage]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    struct Deactivate {
        // Leave the probe inactive even when an assertion fails.
        ~Deactivate() { Orchestrator::instance().set_plugin_active("test.plugin_storage.support", false); }
    } deactivate;
    REQUIRE(orchestrator.set_plugin_active("test.plugin_storage.support", true));
    completed_objects = 0;
    Plugin *plugin = orchestrator.get_plugin("test.plugin_storage.support");
    REQUIRE(plugin != nullptr);
    PluginStorage &storage = orchestrator.plugin_storage()[plugin];
    const size_t initial_count = storage.size();
    constexpr size_t run_count = 4;
    std::array<run_ctx_generate_support, run_count> support_payloads {};
    std::array<raw_plugin_run_payload, run_count> run_payloads {};
    for (size_t idx = 0; idx < run_count; ++idx) {
        // The storage probe only needs a distinct, non-null object identity;
        // the host runner carries the payload address to each parallel call.
        support_payloads[idx].object = reinterpret_cast<const object_handle *>(&support_payloads[idx]);
        run_payloads[idx].data = &support_payloads[idx];
    }
    REQUIRE(orchestrator.execute_plugin(*plugin, nullptr, run_payloads.data(), run_count) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(completed_objects.load() == 4);
    CHECK(storage.size() == initial_count);
}

namespace {
// Extrusion whose destructor records its id, proving which stored object a free destroyed and that
// the virtual destructor runs when the registry deletes through ExtrusionEntity.
class CountedExtrusion final : public ExtrusionEntity
{
public:
    CountedExtrusion(int id, std::vector<int> *destroyed_log, std::atomic_size_t *destroyed_count = nullptr)
        : ExtrusionEntity(true), m_id(id), m_log(destroyed_log), m_count(destroyed_count) {}
    ~CountedExtrusion()
    {
        if (m_log != nullptr)
            m_log->push_back(m_id);
        if (m_count != nullptr)
            m_count->fetch_add(1, std::memory_order_relaxed);
    }
    int id() const { return m_id; }

private:
    int m_id;
    std::vector<int> *m_log;
    std::atomic_size_t *m_count;
};

// Three well separated points starting at (id, id): Polygon asserts on a single point and drops a last point too close to the first.
Points marked_points(int id)
{
    return Points {Point(id, id), Point(id + 1000, id), Point(id, id + 1000)};
}

// Store a CountedExtrusion and return the borrowed pointer.
const ExtrusionEntity *store_counted(PluginStorage &storage, int id, std::vector<int> *log)
{
    return storage.store<ExtrusionEntity>(std::make_unique<CountedExtrusion>(id, log));
}

// Store a polygon whose first point is (id, id), so its content identifies it.
Polygon *store_marked_polygon(PluginStorage &storage, int id)
{
    return storage.store(std::make_unique<Polygon>(marked_points(id)));
}

// Ids of the stored extrusions, in the order stored_extrusions() reports them.
std::vector<int> extrusion_ids(const PluginStorage &storage)
{
    std::vector<int> ids;
    for (const ExtrusionEntity *entity : storage.stored_extrusions()) {
        const CountedExtrusion *counted = dynamic_cast<const CountedExtrusion *>(entity);
        REQUIRE(counted != nullptr);
        ids.push_back(counted->id());
    }
    return ids;
}

// Store, find, count and free one object, then check it is gone and the count is back to zero.
template<class T> void check_round_trip(std::unique_ptr<T> item, void *handle)
{
    PluginStorage storage;
    T *object = item.get();
    void *key = handle != nullptr ? handle : static_cast<void *>(object);
    CHECK(storage.store(std::move(item), handle) == object);
    CHECK(storage.contains(key));
    CHECK(storage.size() == 1);
    CHECK(storage.free(key));
    CHECK_FALSE(storage.contains(key));
    CHECK(storage.size() == 0);
}
} // namespace

TEST_CASE("Plugin storage stores, finds and frees every supported type", "[plugins][plugin-storage]")
{
    check_round_trip(std::make_unique<Polyline>(), nullptr);
    check_round_trip(std::make_unique<Polygon>(), nullptr);
    check_round_trip(std::make_unique<ExPolygon>(), nullptr);
    check_round_trip(std::make_unique<std::vector<Polyline>>(), nullptr);
    check_round_trip(std::make_unique<Polygons>(), nullptr);
    check_round_trip(std::make_unique<std::vector<ExPolygon>>(), nullptr);
    check_round_trip(std::make_unique<SurfaceCollection>(), nullptr);
    check_round_trip<ExtrusionEntity>(std::make_unique<ExtrusionEntity>(true), nullptr);
    check_round_trip(std::make_unique<TriangleMesh>(make_cube(1., 1., 1.)), nullptr);
    {
        // A config is registered under its adjusted ConfigBase handle, not the object address.
        std::unique_ptr<DynamicConfig> config = std::make_unique<DynamicConfig>();
        void *handle = ApiHost::to_config_handle(config.get());
        // The handle differs from the object address and is the only registered key.
        CHECK(handle != static_cast<void *>(config.get()));
        PluginStorage storage;
        void *address = config.get();
        CHECK(storage.store(std::move(config), handle) != nullptr);
        CHECK(storage.contains(handle));
        CHECK_FALSE(storage.contains(address));
        CHECK(storage.free(handle));
        CHECK(storage.size() == 0);
    }
    {
        // ClipperShapes only exist behind the C API.
        PluginStorage storage;
        clipper_shapes_handle *shapes = clipper_shapes_create_empty(reinterpret_cast<storage_handle *>(&storage));
        REQUIRE(shapes != nullptr);
        CHECK(storage.contains(shapes));
        CHECK(storage.size() == 1);
        CHECK(storage.free(shapes));
        CHECK_FALSE(storage.contains(shapes));
        CHECK(storage.size() == 0);
    }
}

TEST_CASE("Plugin storage frees a mixed batch out of order and destroys the right objects", "[plugins][plugin-storage]")
{
    constexpr int count = 300;
    // Declared before the storage so the log outlives every object the storage destroys.
    std::vector<int> destroyed;
    PluginStorage storage;
    // Item i has type i % 5: polygon, polyline, config, extrusion, expolygon. `handles[i]` is the key to free.
    std::vector<void *> handles(count);
    for (int id = 0; id < count; ++id) {
        switch (id % 5) {
        case 0: handles[id] = store_marked_polygon(storage, id); break;
        case 1: handles[id] = storage.store(std::make_unique<Polyline>(marked_points(id))); break;
        case 2: {
            std::unique_ptr<DynamicConfig> config = std::make_unique<DynamicConfig>();
            config->set_key_value("marker", new ConfigOptionInt(id));
            void *handle = ApiHost::to_config_handle(config.get());
            storage.store(std::move(config), handle);
            handles[id] = handle;
            break;
        }
        case 3: handles[id] = const_cast<ExtrusionEntity *>(store_counted(storage, id, &destroyed)); break;
        default: handles[id] = storage.store(std::make_unique<ExPolygon>(Polygon(marked_points(id)))); break;
        }
    }
    REQUIRE(storage.size() == count);

    std::vector<int> order(count);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 shuffle_engine(12345);
    std::shuffle(order.begin(), order.end(), shuffle_engine);
    // Free the first 200 of the shuffled order; the other 100 must stay intact.
    std::vector<bool> freed(count, false);
    std::vector<int> expected_destroyed;
    for (size_t step = 0; step < 200; ++step) {
        const int id = order[step];
        REQUIRE(storage.free(handles[id]));
        freed[id] = true;
        if (id % 5 == 3)
            expected_destroyed.push_back(id);
        CHECK_FALSE(storage.contains(handles[id]));
        CHECK(destroyed == expected_destroyed);
    }
    CHECK(storage.size() == count - 200);
    for (int id = 0; id < count; ++id) {
        CHECK(storage.contains(handles[id]) == !freed[id]);
        if (freed[id])
            continue;
        // Surviving objects keep their own content.
        switch (id % 5) {
        case 0: CHECK(static_cast<const Polygon *>(handles[id])->points.front() == Point(id, id)); break;
        case 1: CHECK(static_cast<const Polyline *>(handles[id])->points.front() == Point(id, id)); break;
        case 2: CHECK(ApiHost::to_config(static_cast<config_handle *>(handles[id]))->option<ConfigOptionInt>("marker")->value == id); break;
        case 3: CHECK(dynamic_cast<const CountedExtrusion *>(static_cast<const ExtrusionEntity *>(handles[id]))->id() == id); break;
        default: CHECK(static_cast<const ExPolygon *>(handles[id])->contour.points.front() == Point(id, id)); break;
        }
    }
    // clear() destroys the surviving extrusions too: every extrusion is destroyed exactly once.
    storage.clear();
    CHECK(storage.size() == 0);
    std::sort(destroyed.begin(), destroyed.end());
    std::vector<int> all_extrusions;
    for (int id = 3; id < count; id += 5)
        all_extrusions.push_back(id);
    CHECK(destroyed == all_extrusions);
}

TEST_CASE("Plugin storage frees many configs out of order", "[plugins][plugin-storage]")
{
    constexpr int count = 500;
    PluginStorage storage;
    storage_handle *api = reinterpret_cast<storage_handle *>(&storage);
    std::vector<config_handle *> handles(count);
    for (int idx = 0; idx < count; ++idx) {
        handles[idx] = storage_new_config(api);
        REQUIRE(handles[idx] != nullptr);
        dynamic_cast<DynamicConfig *>(ApiHost::to_config(handles[idx]))->set_key_value("marker", new ConfigOptionInt(idx));
    }
    REQUIRE(storage.size() == count);
    std::vector<int> order(count);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 shuffle_engine(777);
    std::shuffle(order.begin(), order.end(), shuffle_engine);
    std::vector<bool> freed(count, false);
    for (size_t step = 0; step < order.size(); ++step) {
        const int idx = order[step];
        CHECK(storage.free(handles[idx]));
        freed[idx] = true;
        CHECK(storage.size() == count - step - 1);
        // Every ten steps, all survivors must still answer with their own marker.
        if (step % 10 == 0)
            for (int other = 0; other < count; ++other) {
                CHECK(storage.contains(handles[other]) == !freed[other]);
                if (!freed[other])
                    CHECK(ApiHost::to_config(handles[other])->option<ConfigOptionInt>("marker")->value == other);
            }
    }
    CHECK(storage.size() == 0);
}

TEST_CASE("Plugin storage keeps extrusion order after frees at head, middle and tail", "[plugins][plugin-storage]")
{
    // Declared before the storage: the storage destroys its remaining extrusions when it dies, so the log must outlive it.
    std::vector<int> destroyed;
    PluginStorage storage;
    std::vector<const ExtrusionEntity *> items;
    for (int id = 0; id < 10; ++id) {
        items.push_back(store_counted(storage, id, &destroyed));
        // Other types between extrusions must not appear in the extrusion list.
        store_marked_polygon(storage, id);
    }
    CHECK(extrusion_ids(storage) == std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));

    REQUIRE(storage.free(const_cast<ExtrusionEntity *>(items[0])));
    CHECK(extrusion_ids(storage) == std::vector<int>({1, 2, 3, 4, 5, 6, 7, 8, 9}));
    REQUIRE(storage.free(const_cast<ExtrusionEntity *>(items[5])));
    CHECK(extrusion_ids(storage) == std::vector<int>({1, 2, 3, 4, 6, 7, 8, 9}));
    REQUIRE(storage.free(const_cast<ExtrusionEntity *>(items[9])));
    CHECK(extrusion_ids(storage) == std::vector<int>({1, 2, 3, 4, 6, 7, 8}));
    CHECK(destroyed == std::vector<int>({0, 5, 9}));

    // New extrusions go to the tail.
    items.push_back(store_counted(storage, 10, &destroyed));
    items.push_back(store_counted(storage, 11, &destroyed));
    CHECK(extrusion_ids(storage) == std::vector<int>({1, 2, 3, 4, 6, 7, 8, 10, 11}));

    // Free everything but extrusion 6, then that one: the list ends empty and no entry is null.
    for (int id : {1, 2, 3, 4, 7, 8, 10, 11})
        REQUIRE(storage.free(const_cast<ExtrusionEntity *>(items[id])));
    CHECK(extrusion_ids(storage) == std::vector<int>({6}));
    for (const ExtrusionEntity *entity : storage.stored_extrusions())
        CHECK(entity != nullptr);
    REQUIRE(storage.free(const_cast<ExtrusionEntity *>(items[6])));
    CHECK(storage.stored_extrusions().empty());
    // The chain is reusable after being emptied.
    store_counted(storage, 12, &destroyed);
    CHECK(extrusion_ids(storage) == std::vector<int>({12}));
}

TEST_CASE("Plugin storage supports partial frees, clear, new allocations and frees", "[plugins][plugin-storage]")
{
    // Declared before the storage: the storage destroys its remaining extrusions when it dies, so the log must outlive it.
    std::vector<int> destroyed;
    PluginStorage storage;
    std::vector<ExtrusionEntity *> extrusions;
    std::vector<Polygon *> polygons;
    for (int id = 0; id < 50; ++id) {
        extrusions.push_back(const_cast<ExtrusionEntity *>(store_counted(storage, id, &destroyed)));
        polygons.push_back(store_marked_polygon(storage, id));
    }
    for (int id = 0; id < 50; id += 2) {
        REQUIRE(storage.free(extrusions[id]));
        REQUIRE(storage.free(polygons[id]));
    }
    CHECK(storage.size() == 50);
    storage.clear();
    CHECK(storage.size() == 0);
    CHECK(storage.stored_extrusions().empty());
    CHECK(destroyed.size() == 50);

    // Fresh allocations after clear() start a fresh list and registry.
    destroyed.clear();
    extrusions.clear();
    polygons.clear();
    for (int id = 100; id < 130; ++id) {
        extrusions.push_back(const_cast<ExtrusionEntity *>(store_counted(storage, id, &destroyed)));
        polygons.push_back(store_marked_polygon(storage, id));
    }
    CHECK(storage.size() == 60);
    CHECK(extrusion_ids(storage).front() == 100);
    CHECK(extrusion_ids(storage).back() == 129);
    for (size_t idx = 0; idx < extrusions.size(); ++idx) {
        CHECK(storage.free(extrusions[idx]));
        CHECK(storage.free(polygons[idx]));
    }
    CHECK(storage.size() == 0);
    CHECK(storage.stored_extrusions().empty());
    CHECK(destroyed.size() == 30);
}

TEST_CASE("Plugin storage frees concurrently while other threads allocate", "[plugins][plugin-storage]")
{
    constexpr size_t workers = 8;
    constexpr int per_worker = 1500;
    // Declared before the storage so the counter outlives every object the storage destroys.
    std::atomic_size_t destroyed {0};
    PluginStorage storage;
    std::array<std::exception_ptr, workers> failures {};
    std::vector<std::thread> threads;
    for (size_t worker = 0; worker < workers; ++worker)
        threads.emplace_back([&, worker] {
            try {
                std::vector<void *> mine;
                std::vector<Polygon *> polygons;
                for (int idx = 0; idx < per_worker; ++idx) {
                    const int id = int(worker) * per_worker + idx;
                    mine.push_back(storage.store<ExtrusionEntity>(std::make_unique<CountedExtrusion>(id, nullptr, &destroyed)));
                    polygons.push_back(store_marked_polygon(storage, id));
                    mine.push_back(polygons.back());
                    // Free an earlier object of this thread while the others keep allocating.
                    if (idx % 3 == 2) {
                        const size_t victim = size_t(idx) * 2 - 4;
                        if (!storage.free(mine[victim]) || !storage.free(mine[victim + 1]))
                            throw std::runtime_error("Cannot free concurrently");
                        mine[victim] = mine[victim + 1] = nullptr;
                    }
                }
                std::mt19937 shuffle_engine{unsigned(worker)};
                std::shuffle(mine.begin(), mine.end(), shuffle_engine);
                for (void *handle : mine)
                    if (handle != nullptr && !storage.free(handle))
                        throw std::runtime_error("Cannot free concurrently");
            } catch (...) {
                failures[worker] = std::current_exception();
            }
        });
    for (std::thread &thread : threads)
        thread.join();
    for (const std::exception_ptr &failure : failures)
        if (failure)
            std::rethrow_exception(failure);
    CHECK(storage.size() == 0);
    CHECK(storage.stored_extrusions().empty());
    CHECK(destroyed.load() == workers * per_worker);
}

namespace {
// Store `count` marked polygons, free them in `order` (a permutation), checking every object's exact
// content just before its free and the survivors' count after each free.
void check_bulk_free(PluginStorage &storage, int count, const std::vector<int> &order, std::vector<Polygon *> &polygons)
{
    REQUIRE(storage.size() == size_t(count));
    for (size_t step = 0; step < order.size(); ++step) {
        const int id = order[step];
        REQUIRE(polygons[id]->points.size() == 3);
        REQUIRE(polygons[id]->points.front() == Point(id, id));
        REQUIRE(storage.free(polygons[id]));
        REQUIRE(storage.size() == size_t(count) - step - 1);
    }
}
} // namespace

TEST_CASE("Plugin storage frees 100,000 items in reverse and random order with exact content", "[plugins][plugin-storage]")
{
    constexpr int count = 100000;
    PluginStorage storage;
    std::vector<Polygon *> polygons(count);
    std::vector<int> order(count);
    for (int pass = 0; pass < 2; ++pass) {
        for (int id = 0; id < count; ++id)
            polygons[id] = store_marked_polygon(storage, id);
        std::iota(order.begin(), order.end(), 0);
        if (pass == 0)
            std::reverse(order.begin(), order.end());
        else {
            std::mt19937 shuffle_engine(2024);
            std::shuffle(order.begin(), order.end(), shuffle_engine);
        }
        check_bulk_free(storage, count, order, polygons);
        CHECK(storage.size() == 0);
    }
}

TEST_CASE("Plugin storage free timing", "[.][plugin-storage-benchmark]")
{
    // Timing only: printed, never asserted, since wall time depends on the machine.
    constexpr int count = 100000;
    PluginStorage storage;
    std::vector<Polygon *> polygons(count);
    std::vector<int> order(count);
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 shuffle_engine(1);
    std::shuffle(order.begin(), order.end(), shuffle_engine);
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    for (int id = 0; id < count; ++id)
        polygons[id] = store_marked_polygon(storage, id);
    const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (int id : order)
        storage.free(polygons[id]);
    const std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::fprintf(stderr, "plugin-storage-benchmark: store %d: %.1f ms, free in random order: %.1f ms\n", count,
                 std::chrono::duration<double, std::milli>(t1 - t0).count(),
                 std::chrono::duration<double, std::milli>(t2 - t1).count());
}
