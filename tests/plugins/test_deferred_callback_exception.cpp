///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <atomic>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch.hpp>
#include <oneapi/tbb/task_arena.h>

#include "libslic3r/Api/host/ClipperShapes.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Api/plugin/cpp/DeferredCallbackException.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"

using slic3r_api::DeferredCallbackException;

TEST_CASE("Deferred callback exceptions preserve the primary foreign-callback failure",
          "[plugins][callback-boundary]")
{
    DeferredCallbackException deferred;
    CHECK_FALSE(deferred.captured());
    CHECK_FALSE(deferred.capture_current());

    try {
        throw std::runtime_error("primary callback failure");
    } catch (...) {
        CHECK(deferred.capture_current());
    }

    try {
        throw std::logic_error("secondary cleanup failure");
    } catch (...) {
        CHECK_FALSE(deferred.capture_current());
    }

    REQUIRE(deferred.captured());
    try {
        deferred.rethrow_if_captured();
        FAIL("The retained callback exception was not rethrown.");
    } catch (const std::runtime_error &error) {
        CHECK(std::string(error.what()) == "primary callback failure");
    }
}

TEST_CASE("Deferred callback exceptions keep exactly one exception when several threads capture", "[plugins][callback-boundary]")
{
    DeferredCallbackException deferred;
    constexpr int thread_count = 8;
    std::atomic<bool> go{ false };
    std::atomic<int> winners{ 0 };
    std::atomic<int> winner_id{ -1 };
    std::vector<std::thread> threads;
    for (int id = 0; id < thread_count; ++id)
        threads.emplace_back([&, id] {
            while (! go.load()) {}
            try {
                throw std::runtime_error("thread " + std::to_string(id));
            } catch (...) {
                if (deferred.capture_current()) {
                    winner_id = id;
                    ++winners;
                }
            }
        });
    go = true;
    for (std::thread &thread : threads)
        thread.join();

    CHECK(winners.load() == 1);
    REQUIRE(deferred.captured());
    CHECK_THROWS_WITH(deferred.rethrow_if_captured(), "thread " + std::to_string(winner_id.load()));
}

namespace {
// A run context that reports the given flag as its cancellation state.
plugin_run_context cancellation_context(std::atomic<bool> &cancelled)
{
    plugin_run_context context = {};
    context.host_context = reinterpret_cast<plugin_host_context *>(&cancelled);
    context.is_cancelled = [](plugin_host_context *host) {
        return reinterpret_cast<std::atomic<bool> *>(host)->load() ? 1 : 0;
    };
    return context;
}

// A progress counter that expects `jobs` work units and reports to no host.
void init_progress(slic3r_api::PluginProgress &progress, uint32_t jobs)
{
    progress.reset(nullptr, 1, nullptr);
    progress.add_max(jobs);
}
} // namespace

TEST_CASE("The parallel loop wrapper does not run a job of an already cancelled context and rethrows after the call",
          "[plugins][callback-boundary]")
{
    std::atomic<bool> cancelled{ true };
    const plugin_run_context context = cancellation_context(cancelled);
    slic3r_api::PluginProgress progress;
    init_progress(progress, 16);
    std::atomic<int> ran{ 0 };
    CHECK_THROWS_AS(slic3r_api::parallel_for_storage_with_progress(0, 16, &context, &progress,
                        [&ran](uint32_t, storage_handle *) { ++ran; }),
                    slic3r_api::PluginCancelled);
    CHECK(ran.load() == 0);
    CHECK(progress.done() == 0);
}

// One thread runs the jobs one after the other, so the counts are exact. The failing job is chosen by its start rank (a ticket), not
// by its index, so the test does not depend on the order in which TBB splits the range.
TEST_CASE("The parallel loop wrapper rethrows the first exception, skips the later jobs and counts only finished jobs",
          "[plugins][callback-boundary]")
{
    tbb::task_arena single_thread(1);
    single_thread.execute([] {
        slic3r_api::PluginProgress progress;
        init_progress(progress, 10);
        std::atomic<int> started{ 0 };
        std::atomic<int> completed{ 0 };
        CHECK_THROWS_WITH(slic3r_api::parallel_for_storage_with_progress(0, 10, nullptr, &progress,
                              [&](uint32_t, storage_handle *) {
                                  const int ticket = ++started;
                                  if (ticket == 4)
                                      throw std::runtime_error("job 4 failed");
                                  if (ticket > 4)
                                      throw std::runtime_error("a job after the failure ran");
                                  ++completed;
                              }),
                          "job 4 failed");
        // Jobs 1 to 3 finished, job 4 failed, the six others returned before running their worker.
        CHECK(started.load() == 4);
        CHECK(completed.load() == 3);
        CHECK(progress.done() == 3);

        // The same loop with a worker that does not fail runs every job and counts them all.
        progress.reset(nullptr, 1, nullptr);
        progress.add_max(10);
        std::atomic<int> ran{ 0 };
        CHECK_NOTHROW(slic3r_api::parallel_for_storage_with_progress(0, 10, nullptr, &progress,
                          [&ran](uint32_t, storage_handle *) { ++ran; }));
        CHECK(ran.load() == 10);
        CHECK(progress.done() == 10);
    });
}

// Every job fails whatever the order in which TBB starts them: only the first captured exception is rethrown, and the progress of
// a job that did not finish is never counted.
TEST_CASE("The parallel loop wrapper rethrows one exception when every job fails", "[plugins][callback-boundary]")
{
    slic3r_api::PluginProgress progress;
    init_progress(progress, 64);
    std::atomic<int> started{ 0 };
    CHECK_THROWS_AS(slic3r_api::parallel_for_storage_with_progress(0, 64, nullptr, &progress,
                        [&started](uint32_t, storage_handle *) {
                            ++started;
                            throw std::invalid_argument("failing job");
                        }),
                    std::invalid_argument);
    CHECK(started.load() >= 1);
    CHECK(started.load() <= 64);
    CHECK(progress.done() == 0);
}

namespace {
// An object kept in a scratch storage whose destruction is observable through a flag. ClipperShapes has a virtual destructor, so the
// storage deletes it through its base type like any stored shape.
class StorageWitness : public Slic3r::ApiClipper::ClipperShapes
{
public:
    explicit StorageWitness(std::atomic<bool> &destroyed) : m_destroyed(destroyed) {}
    ~StorageWitness() override { m_destroyed = true; }
    void add_to_clipper(Slic3r::ClipperLib::Clipper &, Slic3r::ClipperLib::PolyType, bool) const override {}
    Slic3r::ClipperLib::Paths to_paths() const override { return {}; }
    Slic3r::Polygons to_polygons() const override { return {}; }
    Slic3r::ExPolygons to_expolygons() const override { return {}; }
    bool empty() const override { return true; }
    uint32_t path_count() const override { return 0; }
    Slic3r::BoundingBox bounding_box() const override { return {}; }

private:
    std::atomic<bool> &m_destroyed;
};

// Store a witness in the scratch storage of the current job.
void store_witness(storage_handle *scratch, std::atomic<bool> &destroyed)
{
    reinterpret_cast<Slic3r::PluginStorage *>(scratch)->store<Slic3r::ApiClipper::ClipperShapes>(
        std::make_unique<StorageWitness>(destroyed));
}
} // namespace

TEST_CASE("The scratch storage of a failing job is destroyed before the caller handles the exception", "[plugins][callback-boundary]")
{
    std::atomic<bool> destroyed{ false };
    bool destroyed_when_handled = false;
    bool handled = false;
    slic3r_api::PluginProgress progress;
    init_progress(progress, 1);
    try {
        slic3r_api::parallel_for_storage_with_progress(0, 1, nullptr, &progress,
            [&destroyed](uint32_t, storage_handle *scratch) {
                store_witness(scratch, destroyed);
                throw std::runtime_error("job failed after storing");
            });
    } catch (const std::runtime_error &) {
        handled = true;
        destroyed_when_handled = destroyed.load();
    }
    CHECK(handled);
    CHECK(destroyed_when_handled);
}

TEST_CASE("The C callback of the parallel loop wrapper lets no exception out", "[plugins][callback-boundary]")
{
    std::atomic<bool> destroyed{ false };
    const auto worker = [&destroyed](uint32_t, storage_handle *scratch) {
        store_witness(scratch, destroyed);
        throw std::runtime_error("job failed");
    };
    using Worker = std::decay_t<decltype(worker)>;
    slic3r_api::PluginProgress progress;
    init_progress(progress, 1);
    slic3r_api::detail::ParallelForStorageWithProgressData<Worker> data{ nullptr, &progress, worker, {} };
    Slic3r::PluginStorage scratch;

    // The bridge is what the host loop calls: it must return normally and keep the failure for the caller.
    CHECK_NOTHROW(slic3r_api::detail::parallel_for_storage_with_progress_bridge<Worker>(
        0, reinterpret_cast<storage_handle *>(&scratch), &data));
    CHECK(data.failure.captured());
    CHECK(progress.done() == 0);
    CHECK_THROWS_AS(data.failure.rethrow_if_captured(), std::runtime_error);

    // Once failed, a second call returns at once without running the worker (the witness would be stored again).
    CHECK(scratch.size() == 1);
    CHECK_NOTHROW(slic3r_api::detail::parallel_for_storage_with_progress_bridge<Worker>(
        1, reinterpret_cast<storage_handle *>(&scratch), &data));
    CHECK(scratch.size() == 1);
}

TEST_CASE("Deferred callback exceptions are copyable values", "[plugins][callback-boundary]")
{
    DeferredCallbackException source;
    try {
        throw std::runtime_error("copied failure");
    } catch (...) {
        source.capture_current();
    }

    DeferredCallbackException copy(source);
    CHECK(copy.captured());
    CHECK_THROWS_WITH(copy.rethrow_if_captured(), "copied failure");

    DeferredCallbackException empty;
    DeferredCallbackException assigned;
    assigned = source;
    CHECK(assigned.captured());
    assigned = empty;
    CHECK_FALSE(assigned.captured());
    CHECK_NOTHROW(assigned.rethrow_if_captured());

    // Self-assignment keeps the state and does not deadlock.
    DeferredCallbackException &alias = source;
    source = alias;
    CHECK(source.captured());
}
