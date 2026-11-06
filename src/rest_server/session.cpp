/*
 * In-memory session ownership and lifecycle primitives.
 *
 * The registry only protects its map. Session locks cover individual state
 * transitions, so expiration, lookup, and deletion never hold the registry
 * lock while a caller performs slicing, import, or result serialization.
 */

#include "session.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Slic3r::Rest {

namespace {

// Eigen's storage order must not determine the row-major wire representation.
nlohmann::json matrix_json(const Transform3d &transformation)
{
    nlohmann::json result = nlohmann::json::array();
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column) {
            const double coefficient = transformation.matrix()(row, column);
            if (!std::isfinite(coefficient))
                throw std::logic_error("The scene contains a non-finite transformation.");
            result.push_back(coefficient);
        }
    return result;
}

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

std::vector<ModelInstance *> find_transform_instances(Model &scene, const EntityReference &entity)
{
    for (ModelObject *object : scene.object_ptrs()) {
        if (entity.kind == EntityKind::object && object->id().id == entity.engine_id)
            return object->instances;
        if (entity.kind == EntityKind::instance)
            for (ModelInstance *instance : object->instances)
                if (instance->id().id == entity.engine_id)
                    return {instance};
    }
    throw std::logic_error("The transform target is not in the scene snapshot.");
}

std::pair<ModelObject *, ModelVolume *> find_transform_volume(Model &scene, const EntityReference &entity)
{
    if (entity.kind == EntityKind::volume)
        for (ModelObject *object : scene.object_ptrs())
            for (ModelVolume *volume : object->volumes)
                if (volume->id().id == entity.engine_id)
                    return {object, volume};
    throw std::logic_error("The transform volume is not in the scene snapshot.");
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
    const std::vector<EntityBinding> bindings;
    std::vector<std::string> ignored_ids;
    return commit_mutation(
        token,
        std::move(candidate),
        affects_slicing,
        completed_at,
        bindings,
        ignored_ids);
}

bool SessionContext::commit_mutation(
    MutationToken token,
    SessionData candidate,
    bool affects_slicing,
    TimePoint completed_at,
    const std::vector<EntityBinding> &bindings,
    std::vector<std::string> &bound_ids,
    bool replace_entities,
    const std::vector<EntityBinding> &unbindings)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value() || token.revision != revision_)
        return false;

    if (replace_entities) {
        entity_ids_by_key_.clear();
        entity_keys_by_id_.clear();
    }

    // Remove selected mappings under the scene commit lock, never before the revision check.
    for (const EntityBinding &binding : unbindings) {
        const std::map<EntityKey, std::string>::iterator existing = entity_ids_by_key_.find(
            EntityKey{binding.kind, binding.engine_id});
        if (existing != entity_ids_by_key_.end()) {
            entity_keys_by_id_.erase(existing->second);
            entity_ids_by_key_.erase(existing);
        }
    }

    std::vector<std::string> new_bound_ids;
    new_bound_ids.reserve(bindings.size());
    for (const EntityBinding &binding : bindings) {
        const EntityKey key{binding.kind, binding.engine_id};
        const std::map<EntityKey, std::string>::const_iterator existing = entity_ids_by_key_.find(key);
        if (existing != entity_ids_by_key_.end()) {
            new_bound_ids.push_back(existing->second);
            continue;
        }

        std::string entity_id;
        do {
            entity_id = make_opaque_id("entity");
        } while (entity_keys_by_id_.find(entity_id) != entity_keys_by_id_.end());
        entity_ids_by_key_.emplace(key, entity_id);
        entity_keys_by_id_.emplace(entity_id, key);
        new_bound_ids.push_back(std::move(entity_id));
    }

    data_ = std::move(candidate);
    ++revision_;
    if (affects_slicing) {
        current_gcode_.reset();
        mark_viewer_outdated_locked();
        status_ = SessionStatus::ready;
        error_message_.clear();
    }
    last_activity_ = completed_at;
    bound_ids = std::move(new_bound_ids);
    return true;
}

std::optional<SessionContext::WorkToken> SessionContext::begin_work(
    TimePoint started_at,
    std::function<void()> cancel_hook)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_.has_value())
        return std::nullopt;

    const std::uint64_t generation = ++next_work_generation_;
    active_work_generation_ = generation;
    work_started_ = false;
    cancel_requested_ = false;
    cancel_hook_ = std::move(cancel_hook);
    ++revision_;
    status_ = SessionStatus::slicing;
    progress_ = {};
    current_gcode_.reset();
    mark_viewer_outdated_locked();
    error_message_.clear();
    last_activity_ = started_at;
    return WorkToken{generation};
}

bool SessionContext::start_work(WorkToken token)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || !active_work_generation_ || *active_work_generation_ != token.generation)
        return false;
    work_started_ = true;
    return true;
}

SessionContext::CancelResult SessionContext::request_cancel(TimePoint now)
{
    // Destroyed after the lock is released: an outdated viewer copy and the
    // hook (which may own the engine control) are released outside the lock.
    std::shared_ptr<const ViewerSnapshot> discarded_viewer;
    std::function<void()> hook;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (retired_ || !active_work_generation_)
            return CancelResult::not_active;
        if (!work_started_) {
            // Queued work never reaches the engine: end it now. The worker
            // later finds its token stale and skips the run.
            GCodeProcessorResult no_viewer;
            std::shared_ptr<const void> no_owner;
            finish_work_locked(WorkToken{*active_work_generation_}, SessionStatus::ready, {}, no_viewer, {}, now,
                discarded_viewer, no_owner);
            return CancelResult::cancelled;
        }
        if (cancel_requested_)
            return CancelResult::stopping;
        cancel_requested_ = true;
        hook = std::move(cancel_hook_);
    }
    if (hook)
        hook();
    return CancelResult::stopping;
}

bool SessionContext::update_progress(WorkToken token, bool secondary, std::string message, int percentage)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || !active_work_generation_ || *active_work_generation_ != token.generation)
        return false;
    std::string &channel_message = secondary ? progress_.secondary_message : progress_.main_message;
    int &channel_percentage = secondary ? progress_.secondary_percentage : progress_.main_percentage;
    channel_percentage = std::clamp(percentage, -1, 100);
    // TODO(progress rework): empty nonnegative updates are percent-only heuristically; producers have no strict order.
    if (!message.empty() || percentage < 0)
        channel_message = std::move(message);
    return true;
}

bool SessionContext::finish_work(
    WorkToken token,
    std::filesystem::path gcode_file,
    GCodeProcessorResult viewer,
    TimePoint finished_at,
    std::shared_ptr<const void> gcode_owner,
    std::shared_ptr<const ViewerSnapshot> *discarded_viewer)
{
    // `viewer` and `gcode_owner` stay owned by this frame when the result is
    // discarded (cancelled work), so they are destroyed after the lock.
    std::shared_ptr<const ViewerSnapshot> previous_viewer;
    bool committed;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        committed = finish_work_locked(
            token,
            SessionStatus::sliced,
            std::move(gcode_file),
            viewer,
            {},
            finished_at,
            previous_viewer,
            gcode_owner);
    }
    if (discarded_viewer != nullptr)
        *discarded_viewer = std::move(previous_viewer);
    return committed;
}

bool SessionContext::fail_work(WorkToken token, std::string message, TimePoint finished_at)
{
    std::shared_ptr<const ViewerSnapshot> discarded_viewer;
    GCodeProcessorResult no_viewer;
    std::shared_ptr<const void> no_owner;
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_work_locked(token, SessionStatus::failed, {}, no_viewer, std::move(message), finished_at,
        discarded_viewer, no_owner);
}

bool SessionContext::cancel_work(WorkToken token, TimePoint finished_at)
{
    std::shared_ptr<const ViewerSnapshot> discarded_viewer;
    GCodeProcessorResult no_viewer;
    std::shared_ptr<const void> no_owner;
    std::lock_guard<std::mutex> lock(mutex_);
    return finish_work_locked(token, SessionStatus::ready, {}, no_viewer, {}, finished_at, discarded_viewer, no_owner);
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
    result.progress = progress_;
    return result;
}

std::optional<SessionData> SessionContext::data_snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
        return std::nullopt;
    return data_;
}

std::optional<std::vector<std::string>> SessionContext::instance_ids(MutationToken token, std::uint64_t object_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_ || active_work_generation_ || token.revision != revision_)
        return std::nullopt;
    if (data_.scene != nullptr)
        for (const ModelObject *object : data_.scene->object_ptrs()) {
            if (object->id().id != object_id)
                continue;
            std::vector<std::string> ids;
            ids.reserve(object->instances.size());
            for (const ModelInstance *instance : object->instances)
                ids.push_back(entity_ids_by_key_.at(EntityKey{EntityKind::instance, instance->id().id}));
            return ids;
        }
    throw std::logic_error("The instance-count target is not in the scene snapshot.");
}

// Only the scene pointer and the entity IDs are captured under the lock. A
// published scene is never modified in place (every mutation commits a new
// Model), so the JSON is built afterwards from a consistent snapshot without
// blocking mutations or slicing results on a large scene.
std::optional<nlohmann::json> SessionContext::list_objects() const
{
    std::shared_ptr<const Model> scene;
    // IDs in traversal order: object, its volumes, then its instances.
    std::vector<std::string> ids;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (retired_)
            return std::nullopt;
        scene = data_.scene;
        if (scene != nullptr) {
            // Missing mappings are an internal invariant violation, never a reason to mint IDs on a GET.
            const auto entity_id = [this](EntityKind kind, const ObjectID &id) -> const std::string & {
                return entity_ids_by_key_.at(EntityKey{kind, static_cast<std::uint64_t>(id.id)});
            };
            for (const ModelObject *object : scene->object_ptrs()) {
                ids.push_back(entity_id(EntityKind::object, object->id()));
                for (const ModelVolume *volume : object->volumes)
                    ids.push_back(entity_id(EntityKind::volume, volume->id()));
                for (const ModelInstance *instance : object->instances)
                    ids.push_back(entity_id(EntityKind::instance, instance->id()));
            }
        }
    }

    nlohmann::json objects = nlohmann::json::array();
    if (scene != nullptr) {
        std::vector<std::string>::const_iterator id = ids.begin();
        for (const ModelObject *object : scene->object_ptrs()) {
            const std::string &object_id = *id++;
            nlohmann::json parts = nlohmann::json::array();
            for (const ModelVolume *volume : object->volumes)
                parts.push_back({
                    {"id", *id++},
                    {"name", volume->name},
                    {"matrix", matrix_json(volume->get_matrix())}
                });
            nlohmann::json instances = nlohmann::json::array();
            for (const ModelInstance *instance : object->instances)
                instances.push_back({
                    {"id", *id++},
                    {"matrix", matrix_json(instance->get_matrix())}
                });
            objects.push_back({
                {"id", object_id},
                {"name", object->name},
                {"matrix", matrix_json(Transform3d::Identity())},
                {"parts", std::move(parts)},
                {"instances", std::move(instances)}
            });
        }
    }
    return nlohmann::json{{"objects", std::move(objects)}};
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
    GCodeProcessorResult &viewer,
    std::string error_message,
    TimePoint completed_at,
    std::shared_ptr<const ViewerSnapshot> &discarded_viewer,
    std::shared_ptr<const void> &gcode_owner)
{
    if (retired_ || !active_work_generation_.has_value() ||
        active_work_generation_.value() != token.generation)
        return false;

    // An accepted cancellation wins over a late success or failure: the
    // computed result is discarded and the session returns to ready.
    if (cancel_requested_) {
        status = SessionStatus::ready;
        error_message.clear();
    }
    active_work_generation_.reset();
    work_started_ = false;
    cancel_requested_ = false;
    // The hook only shares the cheap engine control; dropping it here is safe.
    cancel_hook_ = nullptr;
    ++revision_;
    status_ = status;
    progress_ = {};
    error_message_ = std::move(error_message);
    current_gcode_.reset();
    if (status == SessionStatus::sliced) {
        std::shared_ptr<ViewerSnapshot> result = std::make_shared<ViewerSnapshot>();
        result->result_id = make_opaque_id("result");
        result->gcode_file = std::move(gcode_file);
        result->gcode_owner = std::move(gcode_owner);
        result->viewer = std::make_shared<GCodeProcessorResult>(std::move(viewer));
        result->outdated = false;
        discarded_viewer = std::move(viewer_snapshot_);
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
        progress_ = {};
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
