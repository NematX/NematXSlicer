///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include <algorithm>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "libslic3r/PrintBase.hpp"

/*
These tests check step timestamps without slicing geometry.
A timestamp is a growing transition number, not a time of day.
Each concurrent worker owns a separate step state and mutex, like independent prints.

Sequential test
Concurrent test
  Collect numbers from all workers
  Check growth within each worker and uniqueness across all workers

Operation notes:
- set_started / set_done / invalidate: collect each transition number
- Check that each transition advances the number

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- TimestampSource::next_timestamp()
        |-- state.set_started()
        |-- state.state_with_timestamp()
        |-- state.set_done()
        |-- state.is_done()
        `-- state.invalidate()
*/

namespace {

// Expose the allocator also used by Print's pipeline state without constructing a Print.
class TimestampSource : public Slic3r::PrintStateBase
{
public:
    using PrintStateBase::next_timestamp;
};

enum class TestStep { work };

} // namespace

TEST_CASE("Print step timestamps advance on start completion and invalidation", "[print_state][timestamps]")
{
    using namespace Slic3r;
    PrintState<TestStep, 1> state;
    std::mutex mutex;
    const PrintStateBase::TimeStamp before = TimestampSource::next_timestamp();

    REQUIRE(state.set_started(TestStep::work, mutex, [] {}));
    const PrintStateBase::TimeStamp started = state.state_with_timestamp(TestStep::work, mutex).timestamp;
    CHECK(started > before);
    const PrintStateBase::TimeStamp done = state.set_done(TestStep::work, mutex, [] {}).first;
    CHECK(done > started);
    CHECK(state.is_done(TestStep::work, mutex));

    // Invalidation normally runs under the owning Print's state mutex.
    std::lock_guard<std::mutex> lock(mutex);
    REQUIRE(state.invalidate(TestStep::work, [] {}));
    const PrintStateBase::StateWithTimeStamp invalidated = state.state_with_timestamp_unguarded(TestStep::work);
    CHECK(invalidated.state == PrintStateBase::State::Invalidated);
    CHECK(invalidated.timestamp > done);
    CHECK_FALSE(state.invalidate(TestStep::work, [] {}));
    CHECK(state.state_with_timestamp_unguarded(TestStep::work).timestamp == invalidated.timestamp);
}

TEST_CASE("Independent print states allocate unique timestamps concurrently", "[print_state][timestamps][concurrency]")
{
    using namespace Slic3r;
    constexpr size_t workers = 8;
    constexpr size_t cycles = 64;
    std::mutex start_mutex;
    std::condition_variable start_signal;
    size_t ready = 0;
    bool start = false;
    std::vector<std::future<std::vector<PrintStateBase::TimeStamp>>> jobs;

    for (size_t worker = 0; worker < workers; ++worker) {
        jobs.emplace_back(std::async(std::launch::async, [&] {
            {
                std::unique_lock<std::mutex> lock(start_mutex);
                ++ready;
                start_signal.notify_all();
                start_signal.wait(lock, [&] { return start; });
            }
            PrintState<TestStep, 1> state;
            std::mutex state_mutex;
            std::vector<PrintStateBase::TimeStamp> timestamps;
            timestamps.reserve(cycles * 4);
            for (size_t cycle = 0; cycle < cycles; ++cycle) {
                timestamps.push_back(TimestampSource::next_timestamp());
                if (!state.set_started(TestStep::work, state_mutex, [] {}))
                    throw std::runtime_error("Independent print step did not start");
                timestamps.push_back(state.state_with_timestamp(TestStep::work, state_mutex).timestamp);
                timestamps.push_back(state.set_done(TestStep::work, state_mutex, [] {}).first);
                std::lock_guard<std::mutex> lock(state_mutex);
                if (!state.invalidate(TestStep::work, [] {}))
                    throw std::runtime_error("Independent print step did not invalidate");
                timestamps.push_back(state.state_with_timestamp_unguarded(TestStep::work).timestamp);
            }
            return timestamps;
        }));
    }
    {
        std::unique_lock<std::mutex> lock(start_mutex);
        start_signal.wait(lock, [&] { return ready == workers; });
        start = true;
    }
    start_signal.notify_all();

    std::vector<PrintStateBase::TimeStamp> all;
    for (std::future<std::vector<PrintStateBase::TimeStamp>> &job : jobs) {
        const std::vector<PrintStateBase::TimeStamp> timestamps = job.get();
        REQUIRE(timestamps.size() == cycles * 4);
        CHECK(std::adjacent_find(timestamps.begin(), timestamps.end(),
            std::greater_equal<PrintStateBase::TimeStamp>()) == timestamps.end());
        all.insert(all.end(), timestamps.begin(), timestamps.end());
    }
    REQUIRE(all.size() == workers * cycles * 4);
    std::sort(all.begin(), all.end());
    CHECK(all.front() > 0);
    CHECK(std::adjacent_find(all.begin(), all.end()) == all.end());
}
