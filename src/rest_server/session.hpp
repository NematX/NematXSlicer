/*
 * In-memory session ownership and lifecycle primitives.
 *
 * SessionContext owns the candidate scene/configuration, opaque entity
 * mappings, current G-code state, and the last immutable viewer snapshot. Long
 * mutations use a revision token so imported candidates can be prepared
 * without holding a session lock and rejected if another mutation wins first.
 * Work tokens provide the same stale-commit protection for slicing workers.
 * Profile metadata is kept as core-generated sections beside the flat settings,
 * so a project snapshot can save each selected profile's own metadata.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"

namespace Slic3r::Rest {

enum class SessionStatus {
    ready,
    slicing,
    sliced,
    failed
};

const char *session_status_name(SessionStatus status) noexcept;

// Last active slice progress; -1 is unavailable. TODO(progress rework): replace with the libslic3r progress model.
struct SliceProgress {
    std::string main_message;
    int main_percentage = -1;
    std::string secondary_message;
    int secondary_percentage = -1;
};

struct ProfileSelection {
    std::string print;
    std::string printer;
    std::vector<std::string> extruder;
    std::vector<std::string> filaments;
};

struct SessionData {
    std::shared_ptr<Model> scene;
    DynamicPrintConfig configuration;
    nlohmann::json preferences = nlohmann::json::object();
    ProfileSelection profiles;
    // Optional display name, the GUI project name: it never affects slicing.
    std::optional<std::string> name;
    // Core-generated INI sections of the selected profiles, separate from slicing settings.
    std::string profile_sections;
};

struct ViewerSnapshot {
    std::string result_id;
    std::filesystem::path gcode_file;
    // The last snapshot or download owner removes the result directory.
    std::shared_ptr<const void> gcode_owner;
    // The move list is shared and never mutated: marking a snapshot outdated
    // only copies this small header, not the whole GCodeProcessorResult.
    std::shared_ptr<const GCodeProcessorResult> viewer;
    bool outdated = false;
};

enum class EntityKind {
    object,
    volume,
    instance
};

struct EntityReference {
    EntityKind kind;
    std::uint64_t engine_id;
};

struct EntityBinding {
    EntityKind kind;
    std::uint64_t engine_id;
};

// Resolve a preserved engine ID inside a private scene copy, not the live scene.
std::vector<ModelInstance *> find_transform_instances(Model &scene, const EntityReference &entity);
// Return the parent and volume inside the private scene copy, preserving shared-instance semantics.
std::pair<ModelObject *, ModelVolume *> find_transform_volume(Model &scene, const EntityReference &entity);

class SessionContext {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    struct MutationToken {
        std::uint64_t revision = 0;
    };

    struct WorkToken {
        std::uint64_t generation = 0;
    };

    struct Snapshot {
        std::string session_id;
        std::string owner;
        SessionStatus status = SessionStatus::ready;
        std::uint64_t revision = 0;
        // Latest distributed work generation, not necessarily the active one;
        // compare it with a WorkToken to detect a superseded worker.
        std::uint64_t work_generation = 0;
        bool current_gcode = false;
        bool has_viewer = false;
        bool retired = false;
        TimePoint last_activity;
        std::string error_message;
        SliceProgress progress;
    };

    SessionContext(
        std::string session_id,
        std::string owner,
        Clock::duration expiration,
        TimePoint created_at,
        SessionData data = {});

    const std::string &session_id() const noexcept;
    const std::string &owner() const noexcept;

    std::optional<MutationToken> begin_mutation() const;
    bool commit_mutation(
        MutationToken token,
        SessionData candidate,
        bool affects_slicing,
        TimePoint completed_at);
    // Commit the candidate and bind its new engine entities in the same locked
    // step. `replace_entities` first drops every existing entity ID, for a
    // mutation that replaces the whole scene.
    bool commit_mutation(
        MutationToken token,
        SessionData candidate,
        bool affects_slicing,
        TimePoint completed_at,
        const std::vector<EntityBinding> &bindings,
        std::vector<std::string> &bound_ids,
        bool replace_entities = false,
        const std::vector<EntityBinding> &unbindings = {});

    // Cancellation is arbitrated under the session lock:
    // - queued work (not started) ends at once in `ready`;
    // - started work keeps `slicing`; `cancel_hook` (stop the engine) is called
    //   once, outside the lock, and the worker's later report ends in `ready`;
    // - an accepted cancellation wins over a late success or failure;
    // - without active work nothing changes.
    enum class CancelResult {
        not_active,
        cancelled,
        stopping
    };

    std::optional<WorkToken> begin_work(TimePoint started_at, std::function<void()> cancel_hook = {});
    // Worker entry point: false when the work was cancelled while queued.
    bool start_work(WorkToken token);
    CancelResult request_cancel(TimePoint now);
    // Merge only the active generation's channel, ignoring stale producer callbacks.
    bool update_progress(WorkToken token, bool secondary, std::string message, int percentage);
    // Transfer the displaced viewer to the caller when destruction must also wait for an outer lock.
    bool finish_work(
        WorkToken token,
        std::filesystem::path gcode_file,
        GCodeProcessorResult viewer,
        TimePoint finished_at,
        std::shared_ptr<const void> gcode_owner = {},
        std::shared_ptr<const ViewerSnapshot> *discarded_viewer = nullptr);
    bool fail_work(WorkToken token, std::string message, TimePoint finished_at);
    bool cancel_work(WorkToken token, TimePoint finished_at);

    Snapshot snapshot() const;
    std::optional<SessionData> data_snapshot() const;
    // Read ordered instance IDs only if the mutation's revision still owns the snapshot.
    std::optional<std::vector<std::string>> instance_ids(MutationToken token, std::uint64_t object_id) const;
    // List the scene with its existing IDs, both taken from one committed state.
    // Throws std::out_of_range when an entity has no ID (internal invariant).
    std::optional<nlohmann::json> list_objects() const;
    std::optional<std::filesystem::path> current_gcode() const;
    std::shared_ptr<const ViewerSnapshot> viewer_snapshot() const;

    std::optional<std::string> bind_entity(EntityKind kind, std::uint64_t engine_id);
    std::optional<EntityReference> resolve_entity(const std::string &entity_id) const;
    bool unbind_entity(const std::string &entity_id);

private:
    struct EntityKey {
        EntityKind kind;
        std::uint64_t engine_id;

        bool operator<(const EntityKey &other) const noexcept;
    };

    void mark_viewer_outdated_locked();
    // Transfer the old result without destroying its moves or directory under the session lock.
    bool finish_work_locked(
        WorkToken token,
        SessionStatus status,
        std::filesystem::path gcode_file,
        GCodeProcessorResult &viewer,
        std::string error_message,
        TimePoint completed_at,
        std::shared_ptr<const ViewerSnapshot> &discarded_viewer,
        std::shared_ptr<const void> &gcode_owner);
    bool touch(TimePoint accessed_at);
    bool is_expired(TimePoint now) const;
    bool retire();

    mutable std::mutex mutex_;
    std::string session_id_;
    std::string owner_;
    Clock::duration expiration_;
    TimePoint last_activity_;
    SessionStatus status_ = SessionStatus::ready;
    SliceProgress progress_;
    std::uint64_t revision_ = 0;
    std::uint64_t next_work_generation_ = 0;
    std::optional<std::uint64_t> active_work_generation_;
    bool work_started_ = false;
    bool cancel_requested_ = false;
    std::function<void()> cancel_hook_;
    SessionData data_;
    std::optional<std::filesystem::path> current_gcode_;
    std::shared_ptr<const ViewerSnapshot> viewer_snapshot_;
    std::string error_message_;
    std::map<EntityKey, std::string> entity_ids_by_key_;
    std::unordered_map<std::string, EntityKey> entity_keys_by_id_;
    bool retired_ = false;

    friend class SessionRegistry;
};

enum class SessionEraseResult {
    erased,
    not_found,
    busy
};

class SessionRegistry {
public:
    using Clock = SessionContext::Clock;
    using TimePoint = SessionContext::TimePoint;

    explicit SessionRegistry(Clock::duration expiration);

    std::shared_ptr<SessionContext> create(
        std::string owner,
        TimePoint created_at,
        SessionData data = {});
    std::shared_ptr<SessionContext> find(
        const std::string &owner,
        const std::string &session_id,
        TimePoint accessed_at);
    SessionEraseResult erase(const std::string &owner, const std::string &session_id);
    std::size_t expire(TimePoint now);
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    Clock::duration expiration_;
    std::unordered_map<std::string, std::shared_ptr<SessionContext>> sessions_;
};

} // namespace Slic3r::Rest
