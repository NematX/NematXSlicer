///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Thread.hpp"

#include <exception>

#include <boost/log/trivial.hpp>

#include "Orchestrator.hpp"

/*
These C helpers run plugin work on the TBB thread pool and manage plugin storage.
The parallel loops use the calling thread's execution context on each worker.
They return after their workers finish. The caller keeps user_data and the
execution context alive until that return.
slic3r_parallel_for_storage gives each item its own temporary allocation storage.
Results that must outlive the item must be moved or copied to longer-lived storage.

storage_*: manage allocations in the supplied storage, independently of loop lifetime

A C caller cannot recover by catching an exception from these loops.
Unwinding through their C entry points can skip compiler-generated cleanup
and leave a TBB context registered, breaking later work. The failure handler
therefore logs without throwing and calls std::terminate().

Callbacks must not let C++ exceptions escape through this C interface (see slic3r_utils.h).
If a callback or host operation throws out of a parallel loop, log the failure and terminate.
Continuing after unwinding through the loop's C calling boundary is not supported.

Operation notes:
- slic3r_parallel_for / slic3r_parallel_for_storage
- Capture the calling execution
- parallel_for: install it while calling each item
- Wait for all items before returning

Representative direct calls (branches are not a timeline):

    slic3r_parallel_for()
        |-- Slic3r::Orchestrator::instance()
        |-- orchestrator.prepare_plugin_host_context()
        |-- Slic3r::parallel_for()
        `-- terminate_on_callback_exception()

    slic3r_parallel_for_storage()
        |-- Slic3r::Orchestrator::instance()
        |-- orchestrator.prepare_plugin_host_context()
        |-- Slic3r::parallel_for()
        `-- terminate_on_callback_exception()
*/


namespace {
[[noreturn]] void terminate_on_callback_exception(const char *function) noexcept
{
    try {
        BOOST_LOG_TRIVIAL(error) << "an exception crossed " << function << "; terminating.";
    } catch (...) {
    }
    std::terminate();
}
} // namespace

// Runs fn(index, user_data) for every index in [begin, end) on the TBB pool and returns once all calls have finished (the join).
// user_data is borrowed: it is only passed through and must stay valid during the call. A null fn returns at once. A callback that
// is already cancelled by its parent TBB group may be skipped. The callback must not let an exception out (see the net above).
void slic3r_parallel_for(uint32_t begin, uint32_t end, void *user_data, void (*fn)(uint32_t index, void *user_data)) {
    if (fn == nullptr)
        return;

    try {
        Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
        const plugin_host_context inherited = orchestrator.prepare_plugin_host_context(STEP_NONE, nullptr);
        Slic3r::parallel_for(begin, end, [&](size_t i) {
            Slic3r::Orchestrator::RunScope scope = orchestrator.bind_run(&inherited);
            fn(uint32_t(i), user_data);
        });
    } catch (...) {
        terminate_on_callback_exception("slic3r_parallel_for");
    }
}

// Same as slic3r_parallel_for(), and each call gets a fresh scratch PluginStorage that lives exactly as long as the call: it is
// destroyed when the callback returns, so no handle allocated from it may be kept. user_data is borrowed as above and a null fn
// returns at once.
void slic3r_parallel_for_storage(uint32_t begin,
                                 uint32_t end,
                                 void *user_data,
                                 slic3r_parallel_for_storage_fn fn)
{
    if (fn == nullptr)
        return;

    try {
        Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
        const plugin_host_context inherited = orchestrator.prepare_plugin_host_context(STEP_NONE, nullptr);
        Slic3r::parallel_for(begin, end, [&](size_t i) {
            Slic3r::Orchestrator::RunScope scope = orchestrator.bind_run(&inherited);
            // A fresh scratch storage per item gives plugin code a simple lifetime:
            // every temporary handle created from scratch_storage dies when this
            // callback returns. Long-lived results must be copied or moved through
            // a step callback before that point.
            Slic3r::PluginStorage scratch_storage;
            fn(uint32_t(i), reinterpret_cast<storage_handle *>(&scratch_storage), user_data);
        });
    } catch (...) {
        terminate_on_callback_exception("slic3r_parallel_for_storage");
    }
}
int32_t slic3r_safe_rand(int32_t max)
{
    return Slic3r::safe_rand(max);
}

/* release all objects that were created in the storage */
void storage_clear(storage_handle *me) {
    Slic3r::PluginStorage *storage = reinterpret_cast<Slic3r::PluginStorage *>(me);
    storage->clear();
}
/* return 1 if the handle_to_check is an element of the storage, 0 otherwise */
int32_t is_local_storage(storage_handle *me, void *handle_to_check) {
    Slic3r::PluginStorage *storage = reinterpret_cast<Slic3r::PluginStorage *>(me);
    return storage->contains(handle_to_check);
}
/* release an element with handle_to_free to check if it's correct */
int32_t storage_free(storage_handle *me, void *handle_to_free) {
    Slic3r::PluginStorage *storage = reinterpret_cast<Slic3r::PluginStorage *>(me);
    return storage->free(handle_to_free);
}

int32_t storage_size(storage_handle *me) {
    Slic3r::PluginStorage *storage = reinterpret_cast<Slic3r::PluginStorage *>(me);
    return uint32_t(storage->size());
}
