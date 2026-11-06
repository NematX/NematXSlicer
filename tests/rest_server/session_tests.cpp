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
#include "libslic3r/TriangleMesh.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <set>
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

TEST_CASE("session progress channels update independently and normalize values", "[rest_session][rest_state]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken work = session->begin_work(at_hours(1)).value();

    REQUIRE(session->update_progress(work, false, "Slicing objects", 25));
    REQUIRE(session->update_progress(work, true, "Generating infill", 40));
    Session::Snapshot snapshot = session->snapshot();
    CHECK(snapshot.progress.main_message == "Slicing objects");
    CHECK(snapshot.progress.main_percentage == 25);
    CHECK(snapshot.progress.secondary_message == "Generating infill");
    CHECK(snapshot.progress.secondary_percentage == 40);

    REQUIRE(session->update_progress(work, false, "", std::numeric_limits<int>::max()));
    REQUIRE(session->update_progress(work, true, "", 0));
    snapshot = session->snapshot();
    CHECK(snapshot.progress.main_message == "Slicing objects");
    CHECK(snapshot.progress.main_percentage == 100);
    CHECK(snapshot.progress.secondary_message == "Generating infill");
    CHECK(snapshot.progress.secondary_percentage == 0);

    REQUIRE(session->update_progress(work, true, "", std::numeric_limits<int>::min()));
    REQUIRE(session->update_progress(work, false, "Waiting for output", -2));
    snapshot = session->snapshot();
    CHECK(snapshot.progress.main_message == "Waiting for output");
    CHECK(snapshot.progress.main_percentage == -1);
    CHECK(snapshot.progress.secondary_message.empty());
    CHECK(snapshot.progress.secondary_percentage == -1);
}

TEST_CASE("session progress rejects inactive stale and retired work", "[rest_session][rest_state]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken first = session->begin_work(at_hours(1)).value();
    REQUIRE(session->update_progress(first, false, "first", 10));
    REQUIRE(session->cancel_work(first, at_hours(2)));

    CHECK_FALSE(session->update_progress(first, false, "inactive", 20));
    const Session::WorkToken second = session->begin_work(at_hours(3)).value();
    REQUIRE(session->update_progress(second, true, "current", 30));
    CHECK_FALSE(session->update_progress(first, false, "stale", 40));
    Session::Snapshot snapshot = session->snapshot();
    CHECK(snapshot.progress.main_message.empty());
    CHECK(snapshot.progress.main_percentage == -1);
    CHECK(snapshot.progress.secondary_message == "current");
    CHECK(snapshot.progress.secondary_percentage == 30);

    REQUIRE(session->cancel_work(second, at_hours(4)));
    CHECK_FALSE(session->update_progress(second, true, "inactive", 50));
    REQUIRE(registry.erase("alice", session->session_id()) == SessionEraseResult::erased);
    CHECK_FALSE(session->update_progress(second, true, "retired", 60));
    snapshot = session->snapshot();
    CHECK(snapshot.retired);
    CHECK(snapshot.progress.main_message.empty());
    CHECK(snapshot.progress.main_percentage == -1);
    CHECK(snapshot.progress.secondary_message.empty());
    CHECK(snapshot.progress.secondary_percentage == -1);
}

TEST_CASE("session lifecycle transitions reset every progress field", "[rest_session][rest_state]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const auto check_default_progress = [](const Session::Snapshot &snapshot) {
        CHECK(snapshot.progress.main_message.empty());
        CHECK(snapshot.progress.main_percentage == -1);
        CHECK(snapshot.progress.secondary_message.empty());
        CHECK(snapshot.progress.secondary_percentage == -1);
    };

    const Session::WorkToken finished = session->begin_work(at_hours(1)).value();
    check_default_progress(session->snapshot());
    REQUIRE(session->update_progress(finished, false, "main", 10));
    REQUIRE(session->update_progress(finished, true, "secondary", 20));
    REQUIRE(session->finish_work(
        finished, std::filesystem::path("done.gcode"), viewer_result(), at_hours(2)));
    check_default_progress(session->snapshot());

    const Session::WorkToken failed = session->begin_work(at_hours(3)).value();
    check_default_progress(session->snapshot());
    REQUIRE(session->update_progress(failed, false, "main", 30));
    REQUIRE(session->update_progress(failed, true, "secondary", 40));
    REQUIRE(session->fail_work(failed, "slice failed", at_hours(4)));
    check_default_progress(session->snapshot());

    const Session::WorkToken canceled = session->begin_work(at_hours(5)).value();
    check_default_progress(session->snapshot());
    REQUIRE(session->update_progress(canceled, false, "main", 50));
    REQUIRE(session->update_progress(canceled, true, "secondary", 60));
    REQUIRE(session->cancel_work(canceled, at_hours(6)));
    check_default_progress(session->snapshot());

    REQUIRE(registry.erase("alice", session->session_id()) == SessionEraseResult::erased);
    check_default_progress(session->snapshot());
}

TEST_CASE("concurrent progress producers preserve both channels", "[rest_session][rest_state]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken work = session->begin_work(at_hours(1)).value();
    std::promise<void> gate;
    const std::shared_future<void> start = gate.get_future().share();
    std::atomic<bool> main_updated{false};
    std::atomic<bool> secondary_updated{false};

    std::thread main_producer([&]() {
        start.wait();
        main_updated.store(session->update_progress(work, false, "overall", 70));
    });
    std::thread secondary_producer([&]() {
        start.wait();
        secondary_updated.store(session->update_progress(work, true, "detail", 80));
    });
    gate.set_value();
    main_producer.join();
    secondary_producer.join();

    REQUIRE(main_updated.load());
    REQUIRE(secondary_updated.load());
    const Session::Snapshot snapshot = session->snapshot();
    CHECK(snapshot.progress.main_message == "overall");
    CHECK(snapshot.progress.main_percentage == 70);
    CHECK(snapshot.progress.secondary_message == "detail");
    CHECK(snapshot.progress.secondary_percentage == 80);
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

TEST_CASE("session lookup expires exactly at the inactivity deadline", "[rest_session][rest_delete]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    CHECK(registry.find("alice", session->session_id(), at_hours(24)) == nullptr);
    CHECK(registry.size() == 0);
    CHECK(session->snapshot().retired);
    CHECK(registry.find("alice", session->session_id(), at_hours(25)) == nullptr);
}

TEST_CASE("session lookup suspends expiration until work has ended", "[rest_session][rest_delete]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken work = session->begin_work(at_hours(1)).value();
    REQUIRE(registry.find("alice", session->session_id(), at_hours(48)) == session);
    REQUIRE(session->cancel_work(work, at_hours(49)));
    CHECK(registry.find("alice", session->session_id(), at_hours(73)) == nullptr);
    CHECK(registry.size() == 0);
}

TEST_CASE("foreign session lookup cannot remove an expired session", "[rest_session][rest_delete]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    CHECK(registry.find("bob", session->session_id(), at_hours(24)) == nullptr);
    CHECK(registry.size() == 1);
    CHECK_FALSE(session->snapshot().retired);
    CHECK(session->snapshot().last_activity == at_hours(0));
    CHECK(registry.find("alice", session->session_id(), at_hours(24)) == nullptr);
    CHECK(registry.size() == 0);
}

TEST_CASE("expiration releases every result in a mixed session batch", "[rest_session][rest_delete]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<std::atomic<int>> released = std::make_shared<std::atomic<int>>(0);
    for (int index = 0; index < 3; ++index) {
        const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
        const Session::WorkToken work = session->begin_work(at_hours(1)).value();
        std::shared_ptr<const void> owner(new int(index), [released](const void *value) {
            released->fetch_add(1);
            delete static_cast<const int *>(value);
        });
        REQUIRE(session->finish_work(work, "result.gcode", viewer_result(), at_hours(1), std::move(owner)));
    }
    const std::shared_ptr<Session> recent = registry.create("alice", at_hours(24));
    CHECK(registry.expire(at_hours(25)) == 3);
    CHECK(released->load() == 3);
    CHECK(registry.size() == 1);
    CHECK(registry.find("alice", recent->session_id(), at_hours(25)) == recent);
    CHECK_FALSE(recent->snapshot().retired);
}

TEST_CASE("retired result cleanup does not hold the registry lock", "[rest_session][rest_delete]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const std::shared_ptr<std::promise<void>> entered = std::make_shared<std::promise<void>>();
    const std::shared_ptr<std::promise<void>> release = std::make_shared<std::promise<void>>();
    const std::shared_future<void> released = release->get_future().share();
    const std::shared_future<void> cleaning = entered->get_future().share();
    const Session::WorkToken work = session->begin_work(at_hours(1)).value();
    std::shared_ptr<const void> owner(new int(0), [entered, released](const void *value) {
        entered->set_value();
        released.wait_for(std::chrono::seconds(10));
        delete static_cast<const int *>(value);
    });
    REQUIRE(session->finish_work(work, "result.gcode", viewer_result(), at_hours(2), std::move(owner)));
    std::function<void()> retire;
    SECTION("explicit deletion")
    {
        retire = [&]() { registry.erase("alice", session->session_id()); };
    }
    SECTION("expired lookup")
    {
        retire = [&]() { registry.find("alice", session->session_id(), at_hours(26)); };
    }
    SECTION("expiration sweep")
    {
        retire = [&]() { registry.expire(at_hours(26)); };
    }
    std::future<void> retired = std::async(std::launch::async, retire);
    const bool cleanup_started = cleaning.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    std::future<std::size_t> size = std::async(std::launch::async, [&]() { return registry.size(); });
    const bool readable = size.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
    release->set_value();
    retired.get();
    const std::size_t remaining = size.get();
    REQUIRE(cleanup_started);
    CHECK(readable);
    CHECK(remaining == 0);
    CHECK(session->snapshot().retired);
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

TEST_CASE("layer-range entity IDs follow exact owner bounds atomically", "[rest_session][rest_layer_ranges]")
{
    Registry registry(std::chrono::hours(24));
    SessionData data;
    data.scene = std::make_shared<Slic3r::Model>();
    Slic3r::ModelObject *object = data.scene->add_object("part", "part.stl", Slic3r::make_cube(1, 1, 1));
    object->add_instance();
    const Slic3r::t_layer_height_range original{2., 4.};
    const Slic3r::t_layer_height_range replacement{4., 6.};
    object->layer_config_ranges.emplace(original, Slic3r::ModelConfig{});
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const std::vector<Slic3r::Rest::EntityBinding> bindings{
        {Slic3r::Rest::EntityKind::object, static_cast<std::uint64_t>(object->id().id)},
        {Slic3r::Rest::EntityKind::volume, static_cast<std::uint64_t>(object->volumes.front()->id().id)},
        {Slic3r::Rest::EntityKind::instance, static_cast<std::uint64_t>(object->instances.front()->id().id)},
        {Slic3r::Rest::EntityKind::layer_range, static_cast<std::uint64_t>(object->id().id), original}
    };
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true, at_hours(1), bindings, ids));
    REQUIRE(ids.size() == bindings.size());
    const std::string range_id = ids.back();
    const Slic3r::Rest::EntityReference resolved = session->resolve_entity(range_id).value();
    CHECK(resolved.kind == Slic3r::Rest::EntityKind::layer_range);
    CHECK(resolved.engine_id == static_cast<std::uint64_t>(object->id().id));
    CHECK(resolved.range == original);
    CHECK(session->list_objects()->at("objects").at(0).at("layer_ranges").at(0).at("id") == range_id);

    const Session::MutationToken stale = session->begin_mutation().value();
    const Session::MutationToken move = session->begin_mutation().value();
    SessionData candidate = session->data_snapshot().value();
    candidate.scene = std::make_shared<Slic3r::Model>(*candidate.scene);
    Slic3r::ModelObject *candidate_object = candidate.scene->object_ptrs().front();
    Slic3r::ModelConfig config = std::move(candidate_object->layer_config_ranges.at(original));
    candidate_object->layer_config_ranges.erase(original);
    candidate_object->layer_config_ranges.emplace(replacement, std::move(config));
    const std::vector<Slic3r::Rest::EntityRebinding> rebindings{{
        Slic3r::Rest::EntityKind::layer_range,
        static_cast<std::uint64_t>(object->id().id),
        static_cast<std::uint64_t>(object->id().id),
        original,
        replacement
    }};
    REQUIRE(session->commit_mutation(move, candidate, true, at_hours(2), {}, ids, false, {}, rebindings));
    const Slic3r::Rest::EntityReference moved = session->resolve_entity(range_id).value();
    CHECK(moved.range == replacement);
    CHECK(moved.engine_id == resolved.engine_id);
    CHECK_FALSE(session->commit_mutation(stale, candidate, true, at_hours(3), {}, ids, false, {}, rebindings));
    CHECK(session->resolve_entity(range_id)->range == replacement);

    const std::optional<std::string> old_key_id = session->bind_entity(
        Slic3r::Rest::EntityKind::layer_range, resolved.engine_id, original);
    REQUIRE(old_key_id.has_value());
    CHECK(old_key_id.value() != range_id);
    REQUIRE(session->unbind_entity(old_key_id.value()));
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

TEST_CASE("scene listings capture matrices and IDs without changing session state", "[rest_session][rest_objects]")
{
    Registry registry(std::chrono::hours(24));
    SessionData data;
    data.scene = std::make_shared<Slic3r::Model>();
    Slic3r::ModelObject *object = data.scene->add_object("pièce.stl", "source.stl", Slic3r::make_cube(10, 10, 10));
    object->volumes.front()->name = "part";
    Slic3r::ModelVolume *modifier = object->add_volume(
        Slic3r::make_cube(2, 2, 2), Slic3r::ModelVolumeType::PARAMETER_MODIFIER);
    modifier->name = "modifier";
    modifier->set_offset(Slic3r::Vec3d(1, 2, 3));
    Slic3r::Transform3d transform = Slic3r::Transform3d::Identity();
    transform.matrix()(0, 0) = 0;
    transform.matrix()(0, 1) = -1;
    transform.matrix()(1, 0) = 1;
    transform.matrix()(1, 1) = 0;
    transform.translation() = Slic3r::Vec3d(10, 20, 30);
    object->add_instance(Slic3r::Geometry::Transformation(transform));
    object->add_instance()->set_offset(Slic3r::Vec3d(40, 50, 60));
    object->printable = false;
    Slic3r::ModelObject *other = data.scene->add_object("other", "other.stl", Slic3r::make_cube(3, 3, 3));
    other->add_instance();

    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    std::vector<Slic3r::Rest::EntityBinding> bindings;
    for (const Slic3r::ModelObject *model_object : data.scene->object_ptrs()) {
        bindings.push_back({Slic3r::Rest::EntityKind::object, model_object->id().id});
        for (const Slic3r::ModelVolume *volume : model_object->volumes)
            bindings.push_back({Slic3r::Rest::EntityKind::volume, volume->id().id});
        for (const Slic3r::ModelInstance *instance : model_object->instances)
            bindings.push_back({Slic3r::Rest::EntityKind::instance, instance->id().id});
    }
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true, at_hours(1), bindings, ids));
    const nlohmann::json listing = session->list_objects().value();
    REQUIRE(listing.at("objects").size() == 2);
    const nlohmann::json &first = listing.at("objects").at(0);
    CHECK(first.at("name") == "pièce.stl");
    CHECK(first.at("matrix") == nlohmann::json::array({1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}));
    REQUIRE(first.at("parts").size() == 2);
    CHECK(first.at("parts").at(1).at("name") == "modifier");
    CHECK(first.at("parts").at(1).at("matrix") == nlohmann::json::array({1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1}));
    REQUIRE(first.at("instances").size() == 2);
    CHECK(first.at("instances").at(0).at("matrix") == nlohmann::json::array({0,-1,0,10,1,0,0,20,0,0,1,30,0,0,0,1}));
    CHECK(first.at("instances").at(1).at("matrix") == nlohmann::json::array({1,0,0,40,0,1,0,50,0,0,1,60,0,0,0,1}));
    CHECK((transform * modifier->get_matrix() * Slic3r::Vec3d(4, 5, 6)).isApprox(Slic3r::Vec3d(3, 25, 39)));

    std::size_t index = 0;
    std::set<std::string> unique_ids;
    for (const nlohmann::json &listed : listing.at("objects")) {
        CHECK(listed.at("id") == ids.at(index++));
        for (const nlohmann::json &part : listed.at("parts"))
            CHECK(part.at("id") == ids.at(index++));
        for (const nlohmann::json &instance : listed.at("instances"))
            CHECK(instance.at("id") == ids.at(index++));
    }
    for (std::size_t i = 0; i < ids.size(); ++i) {
        CHECK(unique_ids.insert(ids[i]).second);
        REQUIRE(session->resolve_entity(ids[i]).has_value());
        CHECK(session->resolve_entity(ids[i])->kind == bindings[i].kind);
        CHECK(session->resolve_entity(ids[i])->engine_id == bindings[i].engine_id);
    }

    const Session::WorkToken work = session->begin_work(at_hours(2)).value();
    const Session::Snapshot busy = session->snapshot();
    CHECK(session->list_objects().value() == listing);
    CHECK(session->snapshot().revision == busy.revision);
    CHECK(session->snapshot().status == busy.status);
    CHECK(session->snapshot().last_activity == busy.last_activity);
    REQUIRE(session->finish_work(work, "result.gcode", viewer_result(), at_hours(3)));
    const Session::Snapshot sliced = session->snapshot();
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> viewer = session->viewer_snapshot();
    CHECK(session->list_objects().value() == listing);
    CHECK(session->snapshot().revision == sliced.revision);
    CHECK(session->current_gcode().value() == std::filesystem::path("result.gcode"));
    CHECK(session->viewer_snapshot() == viewer);

    const Session::WorkToken retry = session->begin_work(at_hours(4)).value();
    REQUIRE(session->fail_work(retry, "failed", at_hours(5)));
    const Session::Snapshot failed = session->snapshot();
    const std::shared_ptr<const Slic3r::Rest::ViewerSnapshot> outdated = session->viewer_snapshot();
    CHECK(session->list_objects().value() == listing);
    CHECK(session->snapshot().revision == failed.revision);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::failed);
    CHECK(session->viewer_snapshot() == outdated);

    const Session::MutationToken replacement = session->begin_mutation().value();
    SessionData empty;
    empty.scene = std::make_shared<Slic3r::Model>();
    std::promise<void> gate;
    const std::shared_future<void> start = gate.get_future();
    std::vector<nlohmann::json> responses;
    std::thread reader([&]() {
        start.wait();
        for (int i = 0; i < 64; ++i)
            responses.push_back(session->list_objects().value());
    });
    gate.set_value();
    const bool replaced = session->commit_mutation(replacement, empty, true, at_hours(6), {}, ids, true);
    reader.join();
    REQUIRE(replaced);
    const nlohmann::json empty_listing{{"objects", nlohmann::json::array()}};
    for (const nlohmann::json &response : responses)
        CHECK((response == listing || response == empty_listing));
    CHECK(session->list_objects().value() == empty_listing);
    CHECK(listing.at("objects").size() == 2);
    for (const std::string &id : unique_ids)
        CHECK_FALSE(session->resolve_entity(id).has_value());
    REQUIRE(registry.erase("alice", session->session_id()) == SessionEraseResult::erased);
    CHECK_FALSE(session->list_objects().has_value());
}

TEST_CASE("scene listings reject missing ID mappings without repairing them", "[rest_session][rest_objects]")
{
    SessionData data;
    data.scene = std::make_shared<Slic3r::Model>();
    data.scene->add_object("part", "part.stl", Slic3r::make_cube(1, 1, 1))->add_instance();
    Session session("session", "alice", std::chrono::hours(24), at_hours(0), data);
    CHECK_THROWS_AS(session.list_objects(), std::out_of_range);
    CHECK(session.snapshot().revision == 0);
}

TEST_CASE("Transform targets resolve preserved IDs only inside the scene copy", "[rest_session][rest_transform]")
{
    Slic3r::Model source;
    Slic3r::ModelObject *object = source.add_object("part", "part.stl", Slic3r::make_cube(1,1,1));
    object->add_instance();
    object->add_instance();
    Slic3r::Model copy(source);
    const Slic3r::Rest::EntityReference entity{Slic3r::Rest::EntityKind::object, object->id().id};
    const std::vector<Slic3r::ModelInstance *> targets = Slic3r::Rest::find_transform_instances(copy, entity);
    REQUIRE(targets.size() == 2);
    CHECK(targets.front() != object->instances.front());
    CHECK(targets.front()->id() == object->instances.front()->id());
    const std::vector<Slic3r::ModelInstance *> single = Slic3r::Rest::find_transform_instances(copy,
        {Slic3r::Rest::EntityKind::instance, object->instances.back()->id().id});
    REQUIRE(single.size() == 1);
    CHECK(single.front() == targets.back());
    CHECK_THROWS_AS(Slic3r::Rest::find_transform_instances(copy,
        {Slic3r::Rest::EntityKind::volume, object->volumes.front()->id().id}), std::logic_error);
    const std::pair<Slic3r::ModelObject *, Slic3r::ModelVolume *> volume = Slic3r::Rest::find_transform_volume(copy,
        {Slic3r::Rest::EntityKind::volume, object->volumes.front()->id().id});
    CHECK(volume.first == copy.object_ptrs().front());
    CHECK(volume.second != object->volumes.front());
    CHECK(volume.second->id() == object->volumes.front()->id());
    CHECK_THROWS_AS(Slic3r::Rest::find_transform_volume(copy, entity), std::logic_error);

    SessionData initial;
    initial.scene = std::make_shared<Slic3r::Model>(source);
    Session session("transform-session", "alice", std::chrono::hours(24), at_hours(0), initial);
    const Session::MutationToken token = session.begin_mutation().value();
    SessionData candidate = session.data_snapshot().value();
    candidate.scene = std::make_shared<Slic3r::Model>(*candidate.scene);
    Slic3r::Rest::find_transform_instances(*candidate.scene, entity).front()->set_offset(Slic3r::Vec3d(10,0,0));
    SessionData competing = session.data_snapshot().value();
    competing.preferences = {{"units", "mm"}};
    REQUIRE(session.commit_mutation(token, competing, false, at_hours(1)));
    CHECK_FALSE(session.commit_mutation(token, candidate, true, at_hours(2)));
    CHECK(session.data_snapshot()->preferences == competing.preferences);
    CHECK(session.data_snapshot()->scene->object_ptrs().front()->instances.front()->get_offset().isZero());
}

TEST_CASE("Instance ID removals share the scene commit and stale commits remove nothing", "[rest_session][rest_instances]")
{
    Registry registry(std::chrono::hours(24));
    SessionData data;
    data.scene = std::make_shared<Slic3r::Model>();
    Slic3r::ModelObject *object = data.scene->add_object("part", "part.stl", Slic3r::make_cube(1,1,1));
    object->add_instance();
    object->add_instance();
    const std::uint64_t removed_engine_id = object->instances.back()->id().id;
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const std::vector<Slic3r::Rest::EntityBinding> bindings{
        {Slic3r::Rest::EntityKind::object, object->id().id},
        {Slic3r::Rest::EntityKind::volume, object->volumes.front()->id().id},
        {Slic3r::Rest::EntityKind::instance, object->instances.front()->id().id},
        {Slic3r::Rest::EntityKind::instance, removed_engine_id}
    };
    std::vector<std::string> ids;
    REQUIRE(session->commit_mutation(session->begin_mutation().value(), data, true, at_hours(1), bindings, ids));
    const Session::MutationToken stale = session->begin_mutation().value();
    REQUIRE(session->instance_ids(stale, object->id().id).has_value());
    CHECK(session->instance_ids(stale, object->id().id).value() == std::vector<std::string>{ids[2], ids[3]});
    SessionData reduced = session->data_snapshot().value();
    reduced.scene = std::make_shared<Slic3r::Model>(*reduced.scene);
    reduced.scene->object_ptrs().front()->delete_last_instance();
    const std::vector<Slic3r::Rest::EntityBinding> removals{{Slic3r::Rest::EntityKind::instance, removed_engine_id}};
    SessionData competing = session->data_snapshot().value();
    competing.preferences = {{"units", "mm"}};
    REQUIRE(session->commit_mutation(stale, competing, false, at_hours(2)));
    std::vector<std::string> ignored{"unchanged"};
    CHECK_FALSE(session->commit_mutation(stale, reduced, true, at_hours(3), {}, ignored, false, removals));
    CHECK(ignored == std::vector<std::string>{"unchanged"});
    CHECK_FALSE(session->instance_ids(stale, object->id().id));
    CHECK(session->resolve_entity(ids[3]).has_value());
    CHECK(session->data_snapshot()->scene->object_ptrs().front()->instances.size() == 2);

    reduced.preferences = competing.preferences;
    const Session::MutationToken current = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(current, reduced, true, at_hours(4), {}, ignored, false, removals));
    CHECK(ignored.empty());
    CHECK_FALSE(session->resolve_entity(ids[3]).has_value());
    for (std::size_t index = 0; index < 3; ++index)
        CHECK(session->resolve_entity(ids[index]).has_value());
    CHECK(session->instance_ids(session->begin_mutation().value(), object->id().id).value() ==
        std::vector<std::string>{ids[2]});
    CHECK(session->data_snapshot()->scene->object_ptrs().front()->instances.size() == 1);
}

TEST_CASE("cancellation without active work changes nothing", "[rest_session][rest_cancel]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::Snapshot before = session->snapshot();
    CHECK(session->request_cancel(at_hours(1)) == Session::CancelResult::not_active);
    CHECK(session->snapshot().revision == before.revision);
    CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
}

TEST_CASE("queued work is cancelled at once and its worker skips it", "[rest_session][rest_cancel]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    int hook_calls = 0;
    const Session::WorkToken work = session->begin_work(at_hours(1), [&hook_calls]() { ++hook_calls; }).value();
    const std::uint64_t revision = session->snapshot().revision;

    CHECK(session->request_cancel(at_hours(2)) == Session::CancelResult::cancelled);
    CHECK(hook_calls == 0);
    const Session::Snapshot cancelled = session->snapshot();
    CHECK(cancelled.status == Slic3r::Rest::SessionStatus::ready);
    CHECK(cancelled.revision == revision + 1);
    CHECK(cancelled.last_activity == at_hours(2));
    CHECK_FALSE(session->start_work(work));
    CHECK_FALSE(session->update_progress(work, false, "late", 10));
    CHECK_FALSE(session->finish_work(work, "late.gcode", viewer_result(), at_hours(3)));
    CHECK_FALSE(session->fail_work(work, "late", at_hours(3)));
    CHECK(session->request_cancel(at_hours(3)) == Session::CancelResult::not_active);
}

TEST_CASE("started work stops once and an accepted cancellation wins", "[rest_session][rest_cancel]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken first = session->begin_work(at_hours(1)).value();
    REQUIRE(session->finish_work(first, "first.gcode", viewer_result(), at_hours(2)));
    const std::string first_result = session->viewer_snapshot()->result_id;

    SECTION("a late success is discarded")
    {
        int hook_calls = 0;
        const Session::WorkToken work = session->begin_work(at_hours(3), [&hook_calls]() { ++hook_calls; }).value();
        REQUIRE(session->start_work(work));
        CHECK(session->request_cancel(at_hours(4)) == Session::CancelResult::stopping);
        CHECK(session->request_cancel(at_hours(4)) == Session::CancelResult::stopping);
        CHECK(hook_calls == 1);
        CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::slicing);

        // The released owner proves the discarded result is not retained.
        std::shared_ptr<int> owner = std::make_shared<int>(1);
        std::weak_ptr<int> watched = owner;
        REQUIRE(session->finish_work(work, "late.gcode", viewer_result(), at_hours(5), std::move(owner)));
        CHECK(watched.expired());
        CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
        CHECK(session->snapshot().last_activity == at_hours(5));
        CHECK_FALSE(session->current_gcode().has_value());
        REQUIRE(session->viewer_snapshot() != nullptr);
        CHECK(session->viewer_snapshot()->result_id == first_result);
        CHECK(session->viewer_snapshot()->outdated);
    }
    SECTION("a late failure ends ready without an error")
    {
        const Session::WorkToken work = session->begin_work(at_hours(3)).value();
        REQUIRE(session->start_work(work));
        CHECK(session->request_cancel(at_hours(4)) == Session::CancelResult::stopping);
        REQUIRE(session->fail_work(work, "engine stopped", at_hours(5)));
        CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::ready);
        CHECK(session->snapshot().error_message.empty());
    }
    SECTION("a restarted run ignores the cancelled run")
    {
        const Session::WorkToken work = session->begin_work(at_hours(3)).value();
        REQUIRE(session->start_work(work));
        CHECK(session->request_cancel(at_hours(4)) == Session::CancelResult::stopping);
        REQUIRE(session->cancel_work(work, at_hours(5)));
        const Session::WorkToken restarted = session->begin_work(at_hours(6)).value();
        REQUIRE(session->start_work(restarted));
        CHECK_FALSE(session->update_progress(work, false, "late", 50));
        CHECK_FALSE(session->finish_work(work, "late.gcode", viewer_result(), at_hours(7)));
        REQUIRE(session->finish_work(restarted, "restarted.gcode", viewer_result(), at_hours(8)));
        CHECK(session->snapshot().status == Slic3r::Rest::SessionStatus::sliced);
        CHECK(session->current_gcode() == std::filesystem::path("restarted.gcode"));
    }
}

TEST_CASE("the cancellation hook runs outside the session lock", "[rest_session][rest_cancel]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    std::optional<Slic3r::Rest::SessionStatus> seen;
    // Reading the session from the hook would deadlock under the lock.
    const Session::WorkToken work = session->begin_work(at_hours(1), [&seen, &session]() {
        seen = session->snapshot().status;
    }).value();
    REQUIRE(session->start_work(work));
    std::future<Session::CancelResult> result = std::async(std::launch::async, [&session]() {
        return session->request_cancel(at_hours(2));
    });
    REQUIRE(result.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    CHECK(result.get() == Session::CancelResult::stopping);
    CHECK(seen == Slic3r::Rest::SessionStatus::slicing);
}

TEST_CASE("a result discarded by cancellation releases its viewer header outside the lock", "[rest_session][rest_cancel]")
{
    Registry registry(std::chrono::hours(24));
    const std::shared_ptr<Session> session = registry.create("alice", at_hours(0));
    const Session::WorkToken work = session->begin_work(at_hours(1)).value();
    REQUIRE(session->start_work(work));
    REQUIRE(session->request_cancel(at_hours(2)) == Session::CancelResult::stopping);

    // The deleter reads the session: under the session lock it would deadlock.
    std::optional<Slic3r::Rest::SessionStatus> seen;
    std::shared_ptr<const Slic3r::Rest::ViewerHeader> header(new Slic3r::Rest::ViewerHeader,
        [&seen, &session](const Slic3r::Rest::ViewerHeader *released) {
            seen = session->snapshot().status;
            delete released;
        });
    std::weak_ptr<const Slic3r::Rest::ViewerHeader> watched = header;
    std::future<bool> finished = std::async(std::launch::async, [&]() {
        return session->finish_work(work, "late.gcode", viewer_result(), at_hours(3), {}, nullptr, std::move(header));
    });
    REQUIRE(finished.wait_for(std::chrono::seconds(10)) == std::future_status::ready);
    CHECK(finished.get());
    CHECK(watched.expired());
    CHECK(seen == Slic3r::Rest::SessionStatus::ready);
    CHECK(session->viewer_snapshot() == nullptr);
}
