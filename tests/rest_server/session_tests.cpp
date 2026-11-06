/*
 * Session registry and lifecycle tests for the REST server.
 *
 * These tests use a controlled steady-clock timeline to verify ownership,
 * revision-checked mutations, stale worker rejection, result invalidation,
 * immutable viewer snapshots, stable entity IDs, and expiration behavior.
 * Race-prone transitions are synchronized with promises, not sleeps.
 */

#include <catch_main.hpp>

#include "session.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace {

using Registry = Slic3r::Rest::SessionRegistry;
using Session = Slic3r::Rest::SessionContext;
using SessionData = Slic3r::Rest::SessionData;
using SessionEraseResult = Slic3r::Rest::SessionEraseResult;

Session::TimePoint at_hours(std::int64_t hours)
{
    return Session::TimePoint{} + std::chrono::hours(hours);
}

Slic3r::GCodeProcessorResult viewer_result()
{
    Slic3r::GCodeProcessorResult result;
    result.filename = "result.gcode";
    return result;
}

} // namespace

TEST_CASE("session lookup enforces ownership without touching activity", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> alice = registry.create("alice", at_hours(0));
    const std::shared_ptr<Session> bob = registry.create("bob", at_hours(0));
    REQUIRE(alice->session_id() != bob->session_id());

    const Session::Snapshot before = alice->snapshot();
    REQUIRE(registry.find("bob", alice->session_id(), at_hours(12)) == nullptr);
    REQUIRE(alice->snapshot().last_activity == before.last_activity);
    REQUIRE(registry.find("alice", alice->session_id(), at_hours(12)) == alice);
    REQUIRE(alice->snapshot().last_activity == at_hours(12));
}

TEST_CASE("session mutations commit atomically and preserve no-op results", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    const std::optional<Session::WorkToken> work = session->begin_work(at_hours(1));
    REQUIRE(work.has_value());
    REQUIRE(session->finish_work(
        work.value(), std::filesystem::path("first.gcode"), viewer_result(), at_hours(2)));
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> original_viewer = session->viewer_snapshot();
    REQUIRE(original_viewer != nullptr);
    REQUIRE(original_viewer->viewer != nullptr);
    REQUIRE(!original_viewer->outdated);
    const std::shared_ptr<const Slic3r::GCodeProcessorResult> original_moves = original_viewer->viewer;

    std::optional<Session::MutationToken> first_token = session->begin_mutation();
    std::optional<Session::MutationToken> stale_token = session->begin_mutation();
    REQUIRE(first_token.has_value());
    REQUIRE(stale_token.has_value());

    Session::Snapshot before_noop = session->snapshot();
    REQUIRE(session->commit_mutation(first_token.value(), SessionData{}, false, at_hours(3)));
    REQUIRE(!session->commit_mutation(stale_token.value(), SessionData{}, true, at_hours(4)));
    const Session::Snapshot after_noop = session->snapshot();
    REQUIRE(after_noop.status == Slic3r::Rest::SessionStatus::sliced);
    REQUIRE(after_noop.current_gcode);
    REQUIRE(after_noop.revision > before_noop.revision);
    REQUIRE(session->viewer_snapshot()->result_id == original_viewer->result_id);
    REQUIRE(!session->viewer_snapshot()->outdated);

    const std::optional<Session::MutationToken> invalidating = session->begin_mutation();
    REQUIRE(invalidating.has_value());
    REQUIRE(session->commit_mutation(invalidating.value(), SessionData{}, true, at_hours(5)));
    REQUIRE(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
    REQUIRE(!session->current_gcode().has_value());
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> outdated_viewer = session->viewer_snapshot();
    REQUIRE(outdated_viewer != nullptr);
    REQUIRE(outdated_viewer->result_id == original_viewer->result_id);
    REQUIRE(outdated_viewer->outdated);
    // Marking a viewer outdated must not deep-copy the engine move list.
    REQUIRE(outdated_viewer->viewer == original_moves);
}

TEST_CASE("stale work cannot publish after a later generation", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    const std::optional<Session::WorkToken> first = session->begin_work(at_hours(1));
    REQUIRE(first.has_value());
    REQUIRE(!session->begin_work(at_hours(2)).has_value());
    REQUIRE(!session->begin_mutation().has_value());
    REQUIRE(session->cancel_work(first.value(), at_hours(3)));

    const std::optional<Session::WorkToken> second = session->begin_work(at_hours(4));
    REQUIRE(second.has_value());
    REQUIRE(!session->finish_work(
        first.value(), std::filesystem::path("stale.gcode"), viewer_result(), at_hours(5)));
    REQUIRE(session->finish_work(
        second.value(), std::filesystem::path("second.gcode"), viewer_result(), at_hours(6)));
    REQUIRE(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
}

TEST_CASE("failed work keeps the previous viewer outdated", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    const std::optional<Session::WorkToken> first = session->begin_work(at_hours(1));
    REQUIRE(first.has_value());
    REQUIRE(session->finish_work(
        first.value(), std::filesystem::path("first.gcode"), viewer_result(), at_hours(2)));
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    REQUIRE(viewer != nullptr);
    REQUIRE(!viewer->outdated);

    const std::optional<Session::WorkToken> second = session->begin_work(at_hours(3));
    REQUIRE(second.has_value());
    REQUIRE(session->fail_work(second.value(), "slicing failed", at_hours(4)));

    const Session::Snapshot snapshot = session->snapshot();
    REQUIRE(snapshot.status == Slic3r::Rest::SessionStatus::failed);
    REQUIRE(snapshot.error_message == "slicing failed");
    REQUIRE(!snapshot.current_gcode);
    REQUIRE(snapshot.has_viewer);
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> outdated = session->viewer_snapshot();
    REQUIRE(outdated->result_id == viewer->result_id);
    REQUIRE(outdated->outdated);
    REQUIRE(outdated->viewer == viewer->viewer);
}

TEST_CASE("expiration is controlled, suspended during work, and deletion releases the registry", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> active = registry.create("alice", at_hours(0));
    const std::shared_ptr<Session> idle = registry.create("alice", at_hours(99));

    const std::optional<Session::WorkToken> work = active->begin_work(at_hours(1));
    REQUIRE(work.has_value());
    REQUIRE(registry.expire(at_hours(100)) == 0);
    REQUIRE(registry.find("alice", active->session_id(), at_hours(100)) == active);
    REQUIRE(registry.erase("alice", active->session_id()) == SessionEraseResult::busy);
    REQUIRE(active->cancel_work(work.value(), at_hours(101)));

    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> held_viewer = idle->viewer_snapshot();
    REQUIRE(held_viewer == nullptr);
    REQUIRE(registry.erase("alice", idle->session_id()) == SessionEraseResult::erased);
    REQUIRE(registry.find("alice", idle->session_id(), at_hours(1)) == nullptr);

    REQUIRE(registry.expire(at_hours(126)) == 1);
    REQUIRE(registry.find("alice", active->session_id(), at_hours(127)) == nullptr);
    REQUIRE(registry.size() == 0);
}

TEST_CASE("expiration restarts when work completes", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    const std::optional<Session::WorkToken> work = session->begin_work(at_hours(1));
    REQUIRE(work.has_value());
    REQUIRE(registry.expire(at_hours(25)) == 0);
    REQUIRE(session->finish_work(
        work.value(), std::filesystem::path("done.gcode"), viewer_result(), at_hours(2)));

    REQUIRE(registry.expire(at_hours(25)) == 0);
    REQUIRE(registry.expire(at_hours(27)) == 1);
    REQUIRE(registry.size() == 0);
}

TEST_CASE("a held viewer snapshot survives session deletion", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    std::weak_ptr<Session> weak_session;
    std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> held_viewer;
    std::string session_id;
    {
        const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
        weak_session = session;
        session_id = session->session_id();

        const std::optional<Session::WorkToken> work = session->begin_work(at_hours(1));
        REQUIRE(work.has_value());
        REQUIRE(session->finish_work(
            work.value(), std::filesystem::path("held.gcode"), viewer_result(), at_hours(2)));
        held_viewer = session->viewer_snapshot();
        REQUIRE(held_viewer != nullptr);
        REQUIRE(registry.erase("alice", session_id) == SessionEraseResult::erased);
    }

    // The registry dropped its reference; only the held snapshot keeps the
    // engine result alive.
    REQUIRE(weak_session.expired());
    REQUIRE(held_viewer != nullptr);
    REQUIRE(held_viewer->viewer != nullptr);
    REQUIRE(held_viewer->viewer->filename == "result.gcode");
    REQUIRE(registry.find("alice", session_id, at_hours(3)) == nullptr);
}

TEST_CASE("entity IDs are stable until the entity is removed", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    const std::optional<std::string> object_id = session->bind_entity(Slic3r::Rest::EntityKind::object, 10);
    REQUIRE(object_id.has_value());
    REQUIRE(session->bind_entity(Slic3r::Rest::EntityKind::object, 10) == object_id);
    REQUIRE(session->bind_entity(Slic3r::Rest::EntityKind::instance, 10) != object_id);
    REQUIRE(session->resolve_entity(object_id.value())->engine_id == 10);
    REQUIRE(session->unbind_entity(object_id.value()));
    REQUIRE(!session->resolve_entity(object_id.value()).has_value());
    REQUIRE(session->bind_entity(Slic3r::Rest::EntityKind::object, 10) != object_id);
}

TEST_CASE("a mutation racing a work start is rejected", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    std::optional<Session::MutationToken> mutation;
    bool commit_succeeded = true;
    std::promise<void> mutation_started;
    std::promise<void> work_started;
    const std::shared_future<void> mutation_started_future = mutation_started.get_future();
    const std::shared_future<void> work_started_future = work_started.get_future();

    std::thread mutator([&]() {
        mutation = session->begin_mutation();
        mutation_started.set_value();
        work_started_future.wait();
        commit_succeeded = session->commit_mutation(
            mutation.value(), SessionData{}, true, at_hours(3));
    });

    mutation_started_future.wait();
    const std::optional<Session::WorkToken> work = session->begin_work(at_hours(2));
    work_started.set_value();
    mutator.join();

    REQUIRE(mutation.has_value());
    REQUIRE(work.has_value());
    REQUIRE(!commit_succeeded);
    REQUIRE(session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
}

TEST_CASE("concurrent work starts elect a single generation", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));

    std::promise<void> gate;
    const std::shared_future<void> gate_future = gate.get_future();
    std::atomic<int> started{0};
    const auto attempt = [&]() {
        gate_future.wait();
        if (session->begin_work(at_hours(1)).has_value())
            started.fetch_add(1);
    };
    std::thread first(attempt);
    std::thread second(attempt);
    gate.set_value();
    first.join();
    second.join();

    REQUIRE(started.load() == 1);
    REQUIRE(session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);
}

TEST_CASE("concurrent commits with one token elect a single commit", "[rest_session]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::MutationToken token = session->begin_mutation().value();

    std::promise<void> gate;
    const std::shared_future<void> gate_future = gate.get_future();
    std::atomic<int> committed{0};
    const auto attempt = [&]() {
        gate_future.wait();
        if (session->commit_mutation(token, SessionData{}, true, at_hours(2)))
            committed.fetch_add(1);
    };
    std::thread first(attempt);
    std::thread second(attempt);
    gate.set_value();
    first.join();
    second.join();

    REQUIRE(committed.load() == 1);
    REQUIRE(session->snapshot().revision == token.revision + 1);
}
