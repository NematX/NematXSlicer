///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_
#define slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_

/*
Deferred exceptions at C callback boundaries
============================================

A C or ctypes callback must never let a C++ exception unwind through the
foreign caller. DeferredCallbackException lets such a callback catch the
exception, remember the first failure, return normally through the C ABI, and
rethrow only after execution has returned to a C++ frame.

The object deliberately does not interpret the exception. Its owner decides
whether the captured value means cancellation, an invalid argument, or an
ordinary failure, and may perform immediate side effects such as requesting
cooperative cancellation. Keeping that policy outside this helper allows the
same boundary mechanism to serve callbacks with different result protocols.

Typical use:

    DeferredCallbackException deferred;
    c_api_call([](void *context) noexcept {
        DeferredCallbackException &failure =
            *static_cast<DeferredCallbackException *>(context);
        if (failure.captured())
            return;
        try {
            callback_work();
        } catch (...) {
            failure.capture_current();
        }
    }, &deferred);
    deferred.rethrow_if_captured();

capture_current() must be called from a catch handler. The first exception is
retained so a later cleanup failure cannot hide the operation's primary cause.

The object is thread-safe, so the jobs of a parallel loop can share one instance.
captured() is a lock-free atomic read for the early return at the top of each
job; the exception_ptr itself is guarded by a mutex that is only held to store
or copy the pointer, never while a job runs or while the exception is rethrown.
*/

#include <atomic>
#include <exception>
#include <mutex>
#include <utility>

namespace slic3r_api {

class DeferredCallbackException
{
public:
    DeferredCallbackException() = default;

    // Copies the state of `other` (the captured exception, if any), so the class stays a copyable value of the
    // public plugin API although it owns a mutex. The source is locked while it is read.
    DeferredCallbackException(const DeferredCallbackException &other)
    {
        std::lock_guard<std::mutex> lock(other.m_mutex);
        m_exception = other.m_exception;
        m_captured.store(other.m_exception != nullptr, std::memory_order_release);
    }

    // Both mutexes are locked together (std::scoped_lock avoids a deadlock between two threads assigning in
    // opposite directions); a self-assignment does nothing.
    DeferredCallbackException &operator=(const DeferredCallbackException &other)
    {
        if (this == &other)
            return *this;
        std::scoped_lock lock(m_mutex, other.m_mutex);
        m_exception = other.m_exception;
        m_captured.store(m_exception != nullptr, std::memory_order_release);
        return *this;
    }

    // Capture the exception currently handled by the caller. Return true only
    // when this call stored the first exception; false means that an earlier
    // exception already owns the outcome or that no exception is being handled.
    // Safe to call from several threads: exactly one call returns true.
    bool capture_current() noexcept
    {
        std::exception_ptr current = std::current_exception();
        if (current == nullptr)
            return false;

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_exception != nullptr)
            return false;

        m_exception = std::move(current);
        m_captured.store(true, std::memory_order_release);
        return true;
    }

    // True once an exception was stored; a cheap check for a job's early return.
    bool captured() const noexcept { return m_captured.load(std::memory_order_acquire); }

    // Call this only after the foreign invocation has returned. Rethrowing
    // while still inside the C callback would violate the boundary this class
    // is intended to protect.
    void rethrow_if_captured() const
    {
        if (! captured())
            return;
        // Copy under the lock, rethrow after releasing it.
        std::exception_ptr stored;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            stored = m_exception;
        }
        std::rethrow_exception(stored);
    }

private:
    std::atomic<bool> m_captured{ false };
    mutable std::mutex m_mutex;
    std::exception_ptr m_exception;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_
