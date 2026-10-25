///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Thread.hpp"

#include <exception>

#include <boost/log/trivial.hpp>

#include "Orchestrator.hpp"

/* tbb */
// The callbacks of the two loops below must not let an exception out (see slic3r_utils.h). The loops are extern "C", so the
// compiler may assume they never throw and skip the cleanup of the frames an exception unwinds through (the TBB context of the loop
// stays registered), which crashes later. An exception that reaches the loop anyway (a plugin that breaks the contract, or a host
// allocation failure) therefore ends the process in a defined way: the failure is logged without throwing, then std::terminate() runs.
// This safety net is verified by inspection only: terminating the process cannot be tested without a subprocess, and no test starts one.
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
        Slic3r::parallel_for(begin, end, [&](size_t i) { fn(uint32_t(i), user_data); });
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
        Slic3r::parallel_for(begin, end, [&](size_t i) {
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
