/*
 * In-memory session ownership and lifecycle primitives.
 *
 * The registry only protects its map. Session locks cover individual state
 * transitions, so expiration, lookup, and deletion never hold the registry
 * lock while a caller performs slicing, import, or result serialization.
 */

#include "session.hpp"

#include <atomic>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

// Opaque public identifier: randomness plus a process-wide sequence, so it is
// unique and independent from engine indices. It is not a secret and is never
// used for access control.
std::string make_opaque_id(const std::string &prefix)
{
    static std::atomic<std::uint64_t> sequence{1};
    std::random_device random;
    std::ostringstream output;
    output << prefix << '_';
    output << std::hex << random() << random() << sequence.fetch_add(1);
    return output.str();
}

void validate_session_identity(const std::string &session_id, const std::string &owner)
{
    if (session_id.empty())
        throw std::invalid_argument("session_id must not be empty");
    if (owner.empty())
        throw std::invalid_argument("session owner must not be empty");
}

} // namespace

const char *session_status_name(SessionStatus status) noexcept
{
    switch (status) {
    case SessionStatus::ready:
        return "ready";
    case SessionStatus::slicing:
        return "slicing";
    case SessionStatus::sliced:
        return "sliced";
    case SessionStatus::failed:
        return "failed";
    }
    return "failed";
}

SessionContext::SessionContext(
    std::string session_id,
    std::string owner,
    Clock::duration expiration,
    TimePoint created_at,
    SessionData data)
    : session_id_(std::move(session_id)),
      owner_(std::move(owner)),
      expiration_(expiration),
      last_activity_(created_at),
      data_(std::move(data))
{
    validate_session_identity(session_id_, owner_);
    if (expiration_ <= Clock::duration::zero())
        throw std::invalid_argument("session expiration must be positive");
}

const std::string &SessionContext::session_id() const noexcept
{
    return session_id_;
}

const std::string &SessionContext::owner() const noexcept
{
    return owner_;
}

std::optional<SessionContext::MutationToken> SessionContext::begin_mutation() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value())
        return std::nullopt;
    return MutationToken{revision_};
}

bool SessionContext::commit_mutation(
    MutationToken token,
    SessionData candidate,
    bool affects_slicing,
    TimePoint completed_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value() || token.revision != revision_)
        return false;

    data_ = std::move(candidate);
    ++revision_;
    if (affects_slicing) {
        current_gcode_.reset();
        mark_viewer_outdated_locked();
        status_ = SessionStatus::ready;
        error_message_.clear();
    }
    last_activity_ = completed_at;
    return true;
}

std::optional<SessionContext::WorkToken> SessionContext::begin_work(TimePoint started_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value())
        return std::nullopt;

    const std::uint64_t generation = ++next_work_generation_;
    active_work_generation_ = generation;
    ++revision_;
    status_ = SessionStatus::slicing;
    current_gcode_.reset();
    mark_viewer_outdated_locked();
    error_message_.clear();
    last_activity_ = started_at;
    return WorkToken{generation};
}

bool SessionContext::finish_work(
    WorkToken token,
    std::filesystem::path gcode_file,
    GCodeProcessorResult viewer,
    TimePoint finished_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_work_locked(
        token,
        SessionStatus::sliced,
        std::move(gcode_file),
        std::move(viewer),
        {},
        finished_at);
}

bool SessionContext::fail_work(WorkToken token, std::string message, TimePoint finished_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_work_locked(token, SessionStatus::failed, {}, {}, std::move(message), finished_at);
}

bool SessionContext::cancel_work(WorkToken token, TimePoint finished_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_work_locked(token, SessionStatus::ready, {}, {}, {}, finished_at);
}

SessionContext::Snapshot SessionContext::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot result;
    result.session_id = session_id_;
    result.owner = owner_;
    result.status = status_;
    result.revision = revision_;
    result.work_generation = next_work_generation_;
    result.current_gcode = current_gcode_.has_value();
    result.has_viewer = viewer_snapshot_ != nullptr;
    result.retired = retired_;
    result.last_activity = last_activity_;
    result.error_message = error_message_;
    return result;
}

std::optional<SessionData> SessionContext::data_snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return std::nullopt;
    return data_;
}

std::optional<std::filesystem::path> SessionContext::current_gcode() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || !current_gcode_.has_value())
        return std::nullopt;
    return current_gcode_;
}

std::shared_ptr<const ViewerSnapshot> SessionContext::viewer_snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return nullptr;
    return viewer_snapshot_;
}

std::optional<std::string> SessionContext::bind_entity(EntityKind kind, std::uint64_t engine_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return std::nullopt;

    const EntityKey key{kind, engine_id};
    const std::map<EntityKey, std::string>::const_iterator existing = entity_ids_by_key_.find(key);
    if (existing != entity_ids_by_key_.end())
        return existing->second;

    std::string entity_id;
    do {
        entity_id = make_opaque_id("entity");
    } while (entity_keys_by_id_.find(entity_id) != entity_keys_by_id_.end());
    entity_ids_by_key_.emplace(key, entity_id);
    entity_keys_by_id_.emplace(entity_id, key);
    return entity_id;
}

std::optional<EntityReference> SessionContext::resolve_entity(const std::string &entity_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::unordered_map<std::string, EntityKey>::const_iterator iterator =
        entity_keys_by_id_.find(entity_id);
    if (retired_ || iterator == entity_keys_by_id_.end())
        return std::nullopt;
    return EntityReference{iterator->second.kind, iterator->second.engine_id};
}

bool SessionContext::unbind_entity(const std::string &entity_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return false;
    const std::unordered_map<std::string, EntityKey>::const_iterator iterator =
        entity_keys_by_id_.find(entity_id);
    if (iterator == entity_keys_by_id_.end())
        return false;
    entity_ids_by_key_.erase(iterator->second);
    entity_keys_by_id_.erase(iterator);
    return true;
}

bool SessionContext::EntityKey::operator<(const EntityKey &other) const noexcept
{
    if (kind != other.kind)
        return static_cast<int>(kind) < static_cast<int>(other.kind);
    return engine_id < other.engine_id;
}

void SessionContext::mark_viewer_outdated_locked()
{
    if (viewer_snapshot_ == nullptr || viewer_snapshot_->outdated)
        return;
    std::shared_ptr<ViewerSnapshot> outdated = std::make_shared<ViewerSnapshot>(*viewer_snapshot_);
    outdated->outdated = true;
    viewer_snapshot_ = std::move(outdated);
}

bool SessionContext::finish_work_locked(
    WorkToken token,
    SessionStatus status,
    std::filesystem::path gcode_file,
    GCodeProcessorResult viewer,
    std::string error_message,
    TimePoint completed_at)
{
    if (retired_ || !active_work_generation_.has_value() ||
        active_work_generation_.value() != token.generation)
        return false;

    active_work_generation_.reset();
    ++revision_;
    status_ = status;
    error_message_ = std::move(error_message);
    current_gcode_.reset();
    if (status == SessionStatus::sliced) {
        std::shared_ptr<ViewerSnapshot> result = std::make_shared<ViewerSnapshot>();
        result->result_id = make_opaque_id("result");
        result->gcode_file = std::move(gcode_file);
        result->viewer = std::make_shared<GCodeProcessorResult>(std::move(viewer));
        result->outdated = false;
        viewer_snapshot_ = std::move(result);
        current_gcode_ = viewer_snapshot_->gcode_file;
    } else {
        mark_viewer_outdated_locked();
    }
    last_activity_ = completed_at;
    return true;
}

bool SessionContext::touch(TimePoint accessed_at)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return false;
    if (!active_work_generation_.has_value())
        last_activity_ = accessed_at;
    return true;
}

bool SessionContext::is_expired(TimePoint now) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value() || now < last_activity_)
        return false;
    return now - last_activity_ >= expiration_;
}

bool SessionContext::retire()
{
    // Move the heavy members into locals declared outside the lock scope so
    // any last-reference destruction (scene meshes included) happens after the
    // session mutex is released.
    SessionData discarded_data;
    std::shared_ptr<const ViewerSnapshot> discarded_viewer;
    std::map<EntityKey, std::string> discarded_ids;
    std::unordered_map<std::string, EntityKey> discarded_keys;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (retired_ || active_work_generation_.has_value())
            return false;
        retired_ = true;
        current_gcode_.reset();
        discarded_viewer = std::move(viewer_snapshot_);
        discarded_data = std::move(data_);
        discarded_ids = std::move(entity_ids_by_key_);
        discarded_keys = std::move(entity_keys_by_id_);
    }
    return true;
}

SessionRegistry::SessionRegistry(Clock::duration expiration) : expiration_(expiration)
{
    if (expiration_ <= Clock::duration::zero())
        throw std::invalid_argument("session registry expiration must be positive");
}

std::shared_ptr<SessionContext> SessionRegistry::create(
    std::string owner,
    TimePoint created_at,
    SessionData data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::string session_id;
    do {
        session_id = make_opaque_id("session");
    } while (sessions_.find(session_id) != sessions_.end());

    std::shared_ptr<SessionContext> session = std::make_shared<SessionContext>(
        session_id, std::move(owner), expiration_, created_at, std::move(data));
    sessions_.emplace(session_id, session);
    return session;
}

std::shared_ptr<SessionContext> SessionRegistry::find(
    const std::string &owner,
    const std::string &session_id,
    TimePoint accessed_at)
{
    std::shared_ptr<SessionContext> session;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::unordered_map<std::string, std::shared_ptr<SessionContext>>::const_iterator iterator =
            sessions_.find(session_id);
        if (iterator == sessions_.end() || iterator->second->owner() != owner)
            return nullptr;
        session = iterator->second;
    }
    return session->touch(accessed_at) ? session : nullptr;
}

SessionEraseResult SessionRegistry::erase(const std::string &owner, const std::string &session_id)
{
    // Keep the removed session alive until the registry mutex is released, so
    // its destruction cannot run under the lock.
    std::shared_ptr<SessionContext> removed;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::unordered_map<std::string, std::shared_ptr<SessionContext>>::const_iterator iterator =
            sessions_.find(session_id);
        if (iterator == sessions_.end() || iterator->second->owner() != owner)
            return SessionEraseResult::not_found;
        if (!iterator->second->retire())
            return SessionEraseResult::busy;
        removed = iterator->second;
        sessions_.erase(iterator);
    }
    return SessionEraseResult::erased;
}

// Drop every session whose inactivity delay elapsed. A session with an active
// work is never expired; its delay restarts when the work finishes.
std::size_t SessionRegistry::expire(TimePoint now)
{
    std::vector<std::shared_ptr<SessionContext>> removed;
    std::size_t expired_count = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::unordered_map<std::string, std::shared_ptr<SessionContext>>::iterator iterator = sessions_.begin();
             iterator != sessions_.end();) {
            if (iterator->second->is_expired(now) && iterator->second->retire()) {
                removed.push_back(iterator->second);
                iterator = sessions_.erase(iterator);
                ++expired_count;
            } else {
                ++iterator;
            }
        }
    }
    return expired_count;
}

std::size_t SessionRegistry::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sessions_.size();
}

} // namespace Slic3r::Rest
