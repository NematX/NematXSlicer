///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <atomic>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

/*
These tests call a native test plugin, ExecutionProbe, through the real host wrappers.
Concurrent prints share plugin storage but use separate progress counters and stop state.
Other cases check shared storage after RunScope ends, hosts without a RunContext,
and restoration of the previous context after nested calls.

Test cases

Operation notes:
- run_probe: initialize and register the test plugin
- Build Print and RunContext objects, then prepare their callback data
- Plugin::setup / setup_run / run: exercise the host wrappers
- ExecutionProbe: record progress and observe contexts on nested workers
- Finish workers before destroying callback data or execution scopes
- Print cleanup: release that Print's cached allocations after the scope has ended

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- run_probe()
        |-- Orchestrator::instance()
        |-- orchestrator.reset_plugin_cancel()
        |-- orchestrator.prepare_plugin_host_context()
        |-- orchestrator.prepare_plugin_run_context()
        `-- plugin.setup()
*/

namespace {

using namespace Slic3r;

struct LoopProbe
{
    Orchestrator::RunContext *expected = nullptr;
    std::atomic_uint matches { 0 };
    std::atomic_uint failures { 0 };
};

struct Payload
{
    int work = 1;
    bool fail = false;
    slic3r_api::PluginProgress *progress = nullptr;
    void *allocation = nullptr;
    LoopProbe nested;
};

// Check the worker's execution context. Record failures instead of throwing through C.
void observe_storage_worker(uint32_t, storage_handle *, void *user_data) noexcept;
// Use the same context check for the parallel loop that supplies no temporary storage.
void observe_worker(uint32_t index, void *user_data) noexcept;
// Initialize plugin definitions and register the test plugin for the process lifetime.
// It adds no configuration options.
Plugin &run_probe();

// The cache models the current contract: one shared storage, with entries partitioned by Print identity.
class ExecutionProbe final : public slic3r_api::PluginBase
{
public:
    // Borrow the application-wide plugin manager.
    explicit ExecutionProbe(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    // Keep the test plugin's ID separate from production plugin IDs.
    const char *id_impl() const noexcept override { return "test.plugin_run_context"; }
    // Use a pipeline step that supports checkpoint cleanup.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // The test plugin needs no other plugin.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Use the default execution priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // Opt into the same per-Print cleanup as production cached plugins.
    bool clear_enabled_impl() const noexcept override { return true; }
    // Create one retained polygon per Print while preparing this execution's progress counters.
    void setup_impl(const plugin_run_context *context, uint32_t) const override
    {
        Payload &payload = *static_cast<Payload *>(context->data);
        progress().add_max(10);
        payload.progress = &progress();
        std::lock_guard<std::mutex> lock(cache_mutex_);
        Cache &cache = caches_[context->print];
        if (cache.allocation == nullptr) {
            cache.storage = context->plugin_storage;
            cache.allocation = storage_new_polygon(cache.storage);
        }
        payload.allocation = cache.allocation;
    }
    // Check that a preparation worker sees setup's counters and execution context.
    void setup_run_impl(const plugin_run_context *context) const override
    {
        const Payload &payload = *static_cast<Payload *>(context->data);
        const plugin_host_context binding = Orchestrator::instance().prepare_plugin_host_context(STEP_NONE, nullptr);
        if (payload.progress != &progress() || binding.run_context != context->host_context->run_context)
            throw std::runtime_error("Preparation worker lost its plugin execution binding");
    }
    // Advance only this execution, verify nested worker binding, and optionally report a fatal plugin error.
    void run_impl(const plugin_run_context *context) const override
    {
        Payload &payload = *static_cast<Payload *>(context->data);
        payload.progress = &progress();
        progress().increment_by(payload.work);
        payload.nested.expected = static_cast<Orchestrator::RunContext *>(context->host_context->run_context);
        slic3r_parallel_for_storage(0, 4, &payload.nested, observe_storage_worker);
        if (payload.fail)
            slic3r_api::report_error(context, "execution probe failure");
    }
    // Release only the supplied Print's allocation, using the same shared storage that setup received.
    int32_t clear_impl(const print_handle *print, storage_handle *storage) const override
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        const std::map<const print_handle *, Cache>::iterator found = caches_.find(print);
        if (found != caches_.end()) {
            if (found->second.storage != storage)
                throw std::runtime_error("Checkpoint cleanup received a different plugin storage");
            storage_free(storage, found->second.allocation);
            caches_.erase(found);
        }
        return PLUGIN_ERROR_OK;
    }

    struct Cache
    {
        storage_handle *storage = nullptr;
        void *allocation = nullptr;
    };
    mutable std::mutex cache_mutex_;
    mutable std::map<const print_handle *, Cache> caches_;
};

void observe_storage_worker(uint32_t, storage_handle *, void *user_data) noexcept
{
    LoopProbe &probe = *static_cast<LoopProbe *>(user_data);
    try {
        const plugin_host_context context = Orchestrator::instance().prepare_plugin_host_context(STEP_NONE, nullptr);
        if (context.run_context == probe.expected)
            probe.matches.fetch_add(1, std::memory_order_relaxed);
        else
            probe.failures.fetch_add(1, std::memory_order_relaxed);
    } catch (...) {
        probe.failures.fetch_add(1, std::memory_order_relaxed);
    }
}

void observe_worker(uint32_t index, void *user_data) noexcept
{
    observe_storage_worker(index, nullptr, user_data);
}

Plugin &run_probe()
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    static ExecutionProbe probe(orchestrator.handle());
    static const bool registered = orchestrator.register_plugin(probe.c_instance());
    if (!registered)
        throw std::runtime_error("Cannot register execution-context probe");
    return *orchestrator.get_plugin("test.plugin_run_context");
}

} // namespace

TEST_CASE("Concurrent plugin executions isolate progress and the first stop cause", "[plugins][run_context]")
{
    Plugin &plugin = run_probe();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Orchestrator::RunContext first_run, second_run;
    std::atomic_int first_percent { -1 }, second_percent { -1 };
    Print first_print, second_print;
    first_print.set_status_callback([&first_percent](const PrintBase::SlicingStatus &status) {
        first_percent.store(status.percent, std::memory_order_relaxed);
    });
    second_print.set_status_callback([&second_percent](const PrintBase::SlicingStatus &status) {
        second_percent.store(status.percent, std::memory_order_relaxed);
    });
    Payload first, second;
    first.work = 3;
    first.fail = true;
    second.work = 7;
    {
        Orchestrator::RunScope first_scope = orchestrator.enter_run(first_run, &first_print);
        Orchestrator::RunScope second_scope = orchestrator.enter_run(second_run, &second_print);
        plugin_host_context first_host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &first_print);
        plugin_host_context second_host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &second_print);
        plugin_run_context first_context = orchestrator.prepare_plugin_run_context(STEP_SUPPORT, &plugin, &first_host);
        plugin_run_context second_context = orchestrator.prepare_plugin_run_context(STEP_SUPPORT, &plugin, &second_host);
        first_context.data = &first;
        second_context.data = &second;
        REQUIRE(first_context.plugin_storage == second_context.plugin_storage);
        REQUIRE(first_context.progress_context != nullptr);
        REQUIRE(first_context.progress_context != second_context.progress_context);
        plugin.setup(first_context, 1);
        plugin.setup(second_context, 1);
        std::promise<void> start;
        const std::shared_future<void> ready = start.get_future().share();
        std::future<void> a = std::async(std::launch::async, [&] {
            ready.wait();
            plugin.setup_run(first_context);
            plugin.run(first_context);
        });
        std::future<void> b = std::async(std::launch::async, [&] {
            ready.wait();
            plugin.setup_run(second_context);
            plugin.run(second_context);
        });
        start.set_value();
        a.get();
        b.get();

        REQUIRE(first.progress != nullptr);
        REQUIRE(second.progress != nullptr);
        CHECK(first.progress->done() == 3);
        CHECK(second.progress->done() == 7);
        CHECK(first.progress->max() == 10);
        CHECK(second.progress->max() == 10);
        CHECK(first_percent.load() == 30);
        CHECK(second_percent.load() == 70);
        CHECK(first.nested.matches.load() == 4);
        CHECK(second.nested.matches.load() == 4);
        CHECK(first.nested.failures.load() == 0);
        CHECK(second.nested.failures.load() == 0);
        CHECK(first_run.plugin_cancel_requested());
        CHECK_FALSE(second_run.plugin_cancel_requested());
        CHECK(first_context.is_cancelled(first_context.host_context) == 1);
        CHECK(second_context.is_cancelled(second_context.host_context) == 0);
        CHECK(first_run.plugin_stop_cause().plugin == &plugin);
        CHECK(first_run.plugin_stop_cause().kind == Orchestrator::PluginStopCause::Kind::Plugin);
        CHECK(orchestrator.is_plugin_cancelled(&first_print));
        CHECK_FALSE(orchestrator.is_plugin_cancelled(&second_print));
        const std::atomic_bool no_own_error { false };
        CHECK_THROWS_WITH(Steps::Detail::throw_if_plugin_stopped(orchestrator, first_print,
            STEP_SUPPORT, plugin, no_own_error, "run"), Catch::Matchers::Contains("after plugin 'test.plugin_run_context' reported an error"));
        CHECK_NOTHROW(Steps::Detail::throw_if_plugin_stopped(orchestrator, second_print,
            STEP_SUPPORT, plugin, no_own_error, "run"));
        first_run.request_plugin_cancel_after_host_failure();
        CHECK(first_run.plugin_stop_cause().kind == Orchestrator::PluginStopCause::Kind::Plugin);
        second_run.request_plugin_cancel_after_host_failure();
        CHECK(second_run.plugin_stop_cause().kind == Orchestrator::PluginStopCause::Kind::Host);
        orchestrator.reset_plugin_cancel(&second_print);
        CHECK_FALSE(second_run.plugin_cancel_requested());
        CHECK(second_run.plugin_stop_cause().kind == Orchestrator::PluginStopCause::Kind::Unknown);
        CHECK(first_run.plugin_cancel_requested());
    }
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
    orchestrator.consume_plugin_messages();
}

TEST_CASE("Print cleanup retains shared plugin storage after execution scopes end", "[plugins][run_context][clear]")
{
    Plugin &plugin = run_probe();
    Orchestrator &orchestrator = Orchestrator::instance();
    Orchestrator::RunContext run;
    std::unique_ptr<Print> print = std::make_unique<Print>();
    Payload payload;
    storage_handle *storage = nullptr;
    {
        Orchestrator::RunScope scope = orchestrator.enter_run(run, print.get());
        plugin_host_context host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, print.get());
        plugin_run_context context = orchestrator.prepare_plugin_run_context(STEP_SUPPORT, &plugin, &host);
        context.data = &payload;
        storage = context.plugin_storage;
        plugin.setup(context, 1);
        REQUIRE(is_local_storage(storage, payload.allocation) == 1);
    }
    CHECK(is_local_storage(storage, payload.allocation) == 1);
    REQUIRE(orchestrator.clear_plugin_participants(*print, STEP_SUPPORT));
    CHECK(is_local_storage(storage, payload.allocation) == 0);
    {
        Orchestrator::RunScope scope = orchestrator.enter_run(run, print.get());
        plugin_host_context host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, print.get());
        plugin_run_context context = orchestrator.prepare_plugin_run_context(STEP_SUPPORT, &plugin, &host);
        context.data = &payload;
        plugin.setup(context, 1);
        CHECK(context.plugin_storage == storage);
        REQUIRE(is_local_storage(storage, payload.allocation) == 1);
    }
    print.reset();
    CHECK(is_local_storage(storage, payload.allocation) == 0);
}

TEST_CASE("Unbound hosts retain plugin-local progress and shared storage", "[plugins][run_context][fallback]")
{
    Plugin &plugin = run_probe();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Print print;
    Payload payload;
    plugin_host_context host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &print);
    plugin_run_context context = orchestrator.prepare_plugin_run_context(STEP_SUPPORT, &plugin, &host);
    context.data = &payload;
    CHECK(host.run_context == nullptr);
    CHECK(context.progress_context == nullptr);
    plugin.setup(context, 1);
    plugin.setup_run(context);
    plugin.run(context);
    REQUIRE(payload.progress != nullptr);
    CHECK(payload.progress->done() == 1);
    CHECK(payload.progress->max() == 10);
    CHECK(payload.nested.matches.load() == 4);
    CHECK(payload.nested.failures.load() == 0);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}

TEST_CASE("Print bindings and copied workers restore nested execution scopes", "[plugins][run_context][workers]")
{
    Plugin &plugin = run_probe();
    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    Orchestrator::RunContext outer, inner;
    Print print;
    {
        Orchestrator::RunScope outer_scope = orchestrator.enter_run(outer, &print);
        {
            Orchestrator::RunScope inner_scope = orchestrator.enter_run(inner);
            CHECK(orchestrator.prepare_plugin_host_context(STEP_NONE, nullptr).run_context == &inner);
            // A worker without a thread binding must still resolve the explicitly supplied Print.
            std::future<void *> inherited = std::async(std::launch::async, [&] {
                const plugin_host_context host = orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &print);
                orchestrator.request_plugin_cancel_after_host_failure(&print);
                return host.run_context;
            });
            CHECK(inherited.get() == &outer);
            CHECK(outer.plugin_stop_cause().kind == Orchestrator::PluginStopCause::Kind::Host);
            CHECK_FALSE(inner.plugin_cancel_requested());
            LoopProbe loop;
            loop.expected = &inner;
            slic3r_parallel_for(0, 4, &loop, observe_worker);
            CHECK(loop.matches.load() == 4);
            CHECK(loop.failures.load() == 0);
        }
        CHECK(orchestrator.prepare_plugin_host_context(STEP_NONE, nullptr).run_context == &outer);
        {
            Orchestrator::RunScope nested = orchestrator.enter_run(outer, &print);
            CHECK(orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &print).run_context == &outer);
        }
        CHECK(orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &print).run_context == &outer);
    }
    CHECK(orchestrator.prepare_plugin_host_context(STEP_SUPPORT, &plugin, &print).run_context == nullptr);
    CHECK(orchestrator.prepare_plugin_host_context(STEP_NONE, nullptr).run_context == nullptr);
    CHECK_FALSE(orchestrator.is_plugin_cancelled());
}
