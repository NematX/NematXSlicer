///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ContiguousSeamPlacer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "SeamCandidatePreparation.hpp"
#include "SeamCostModel.hpp"
#include "SeamPlacementConstraints.hpp"
#include "SeamTrackPreparation.hpp"
#include "SeamInitializationCancellation.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

/*
Contiguous seam placement
=========================
The provider tries to keep the beginning of an external perimeter close to the
seam selected on the preceding object layer. "Contiguous" describes this
visual continuity between layers; it does not mean that the printer extrudes
one physically uninterrupted path through several layers.

Initialization performs all expensive work before parallel ordering begins:

1. SeamTrackPreparation scans the PrintingPlan, creates one track for each
   external perimeter and records the direct lower/upper LayerIsland links.
   Internal and technical loops receive bindings but do not become tracks.
2. Each track obtains ordinary geometric candidates. Blockers, enforcers,
   region boundaries, overhang and layer embedding are evaluated by the same
   shared preparation used by the other candidate-based providers.
3. Tracks are finalized from lower to upper layers. A track with no usable
   lower predecessor selects its best local candidate. Otherwise every seam
   finalized on a directly linked lower island is projected onto the current
   perimeter and evaluated as another candidate. A local enforcer deliberately
   starts a new chain instead of inheriting the lower seam.
4. If a projected lower seam survives the constraints, it is preferred. When
   every projection is rejected, the admissible local candidate nearest to a
   lower seam is used; if no lower reference is usable, local scoring provides
   the final fallback. Stable traversal indices resolve exact ties.

Final track points are stored in object coordinates so instances of the same
object can share the prepared result. place_seam() adds the selected instance
translation to convert a point to platter coordinates, then performs a
read-only projection onto the exact line/arc geometry supplied by the caller.
This last projection neither rotates nor otherwise modifies the loop. It only
guarantees that the returned point lies on the current representation of that
loop. Internal loops use the closest finalized external track from their own
LayerRegionIsland, while technical distance-only loops simply project the
incoming start position.

After initialization the session contains only immutable prepared data.
Concurrent place_seam() calls therefore read shared state without locking.
*/

namespace slic3r_api { namespace SeamPlacement { namespace ContiguousPlugin {

namespace CP = CandidatePreparation;
namespace TP = TrackPreparation;

namespace {

const char *const kPluginId = "seam_placer.contiguous";
const char *const kGroupId = "seam_placer_plugin";
const char *const kDependencies[] = { nullptr };

struct TrackState {
    c_point final_point{};
    bool ready = false;
};

struct ProjectionRecord {
    size_t stable_order = size_t(-1);
    size_t source_track = size_t(-1);
    c_point source_target{};
    double distance = std::numeric_limits<double>::infinity();
};

class ContiguousSession;
class ContiguousSeamPlacer;

/* Score an independent local anchor with the same policy as Aligned. */
double local_cost(const CP::Candidate &candidate);
/* Select the locally best candidate, retaining candidate order on exact ties. */
size_t best_local_candidate(const std::vector<CP::Candidate> &candidates);
/* Validate the closed geometry contract required by the SeamPlacer service. */
void validate_loop(const ExtrusionEntity &loop);
/* Return finalized tracks on direct lower graph links in stable order. */
std::vector<size_t> lower_tracks(const TP::Data &data,
                                 const std::vector<TrackState> &states,
                                 size_t track_index);
/* Choose the surviving projected candidate with deterministic graph ties. */
size_t best_surviving_projection(
    const TP::Data &data, const std::vector<CP::Candidate> &candidates,
    const std::vector<ProjectionRecord> &projections);
/* Choose the admissible base candidate nearest to any lower finalized seam. */
size_t nearest_surviving_base(
    const TP::Track &track, const std::vector<CP::Candidate> &candidates,
    const std::vector<ProjectionRecord> &projections,
    size_t projection_begin);
/* Finalize every track from lower to upper layers. */
void build_contiguous_targets(TP::Data &data,
                              const SeamPlacementConstraints &constraints,
                              std::vector<TrackState> &states,
                              const plugin_run_context *run_context);

class ContiguousSession final : public SeamPlacerSession
{
public:
    explicit ContiguousSession(storage_handle *storage,
                               const plugin_run_context *run_context = nullptr);
    void initialize(const PrintingPlan &plan,
                    const std::vector<PrintRegion> &regions) override;
    float get_priority(const ExtrusionEntity &, const LayerRegion &) const override;
    c_point place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                       c_point start_position) const override;

private:
    storage_handle *m_storage;
    const plugin_run_context *m_run_context;
    bool m_initialization_started = false;
    TP::Data m_data;
    std::vector<TrackState> m_states;
};

class ContiguousSeamPlacer final : public PluginBase
{
public:
    static ContiguousSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit ContiguousSeamPlacer(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    // Return the fixed execution priority for seam-placement jobs.
    int32_t priority_impl() const noexcept override;
    // Return the fixed rank among seam-placement alternatives.
    int32_t exclusive_group_priority_impl() const noexcept override;
    void run_impl(const plugin_run_context *run_context) const override;
};

double local_cost(const CP::Candidate &candidate)
{
    const double overhang_penalty =
        candidate.overhang > 0 && candidate.width > 0 ?
            std::clamp(2.0 * (candidate.overhang - candidate.width / 8.0) /
                           candidate.width, 0.0, 3.0) :
            0.0;
    return overhang_penalty + 0.6 *
        Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle));
}

size_t best_local_candidate(const std::vector<CP::Candidate> &candidates)
{
    size_t best = size_t(-1);
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < candidates.size(); ++index) {
        const double cost = local_cost(candidates[index]);
        if (cost < best_cost) {
            best = index;
            best_cost = cost;
        }
    }
    return best;
}

void validate_loop(const ExtrusionEntity &loop)
{
    if (!loop.valid() || !loop.is_loop() || loop.empty())
        throw std::invalid_argument(
            "Contiguous seam placement needs a non-empty closed loop.");
    const std::vector<c_point> points = loop.collect_points();
    if (points.size() < 2 || points.front().x != points.back().x ||
        points.front().y != points.back().y)
        throw std::invalid_argument(
            "Contiguous seam placement rejects open or invalid loop geometry.");
}

std::vector<size_t> lower_tracks(const TP::Data &data,
                                 const std::vector<TrackState> &states,
                                 size_t track_index)
{
    std::vector<size_t> result;
    const TP::Track &track = data.tracks[track_index];
    const std::map<const layer_island_handle *, TP::LayerIslandNode>::const_iterator
        node = data.islands.find(track.layer_island);
    if (node == data.islands.end() || node->second.layer == 0)
        return result;
    const size_t expected_layer = node->second.layer - 1;

    for (size_t candidate_index = 0;
         candidate_index < data.tracks.size(); ++candidate_index) {
        const TP::Track &candidate = data.tracks[candidate_index];
        if (!states[candidate_index].ready ||
            candidate.stable_id.object != track.stable_id.object ||
            candidate.stable_id.instance != track.stable_id.instance ||
            candidate.stable_id.layer != expected_layer)
            continue;
        if (std::find(node->second.lower.begin(), node->second.lower.end(),
                      candidate.layer_island) != node->second.lower.end())
            result.push_back(candidate_index);
    }
    std::sort(result.begin(), result.end(), [&](size_t left, size_t right) {
        return TP::stable_track_less(data.tracks[left], data.tracks[right]);
    });
    return result;
}

size_t best_surviving_projection(
    const TP::Data &data, const std::vector<CP::Candidate> &candidates,
    const std::vector<ProjectionRecord> &projections)
{
    size_t best_candidate = size_t(-1);
    const ProjectionRecord *best_record = nullptr;
    for (size_t candidate_index = 0;
         candidate_index < candidates.size(); ++candidate_index) {
        const CP::Candidate &candidate = candidates[candidate_index];
        const std::vector<ProjectionRecord>::const_iterator record =
            std::find_if(projections.begin(), projections.end(),
                [&](const ProjectionRecord &entry) {
                    return entry.stable_order == candidate.stable_order;
                });
        if (record == projections.end()) continue;
        const bool better = best_record == nullptr ||
            record->distance < best_record->distance ||
            (record->distance == best_record->distance &&
             TP::stable_track_less(data.tracks[record->source_track],
                                   data.tracks[best_record->source_track]));
        if (better) {
            best_candidate = candidate_index;
            best_record = &*record;
        }
    }
    return best_candidate;
}

size_t nearest_surviving_base(
    const TP::Track &track, const std::vector<CP::Candidate> &candidates,
    const std::vector<ProjectionRecord> &projections,
    size_t projection_begin)
{
    size_t best = size_t(-1);
    double best_distance = std::numeric_limits<double>::infinity();
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t candidate_index = 0;
         candidate_index < candidates.size(); ++candidate_index) {
        const CP::Candidate &candidate = candidates[candidate_index];
        if (candidate.stable_order >= projection_begin) continue;
        const c_point platter = {
            candidate.point.x + track.instance_shift.x,
            candidate.point.y + track.instance_shift.y};
        double distance = std::numeric_limits<double>::infinity();
        for (const ProjectionRecord &projection : projections)
            distance = std::min(distance, std::hypot(
                double(platter.x) - projection.source_target.x,
                double(platter.y) - projection.source_target.y));
        const double cost = local_cost(candidate);
        if (distance < best_distance ||
            (distance == best_distance && (cost < best_cost ||
             (cost == best_cost && (best == size_t(-1) ||
              candidate.stable_order < candidates[best].stable_order))))) {
            best = candidate_index;
            best_distance = distance;
            best_cost = cost;
        }
    }
    return best;
}

void build_contiguous_targets(TP::Data &data,
                              const SeamPlacementConstraints &constraints,
                              std::vector<TrackState> &states,
                              const plugin_run_context *run_context)
{
    states.resize(data.tracks.size());
    std::vector<size_t> order(data.tracks.size());
    for (size_t index = 0; index < order.size(); ++index) order[index] = index;
    std::sort(order.begin(), order.end(), [&](size_t left, size_t right) {
        return TP::stable_track_less(data.tracks[left], data.tracks[right]);
    });

    for (const size_t track_index : order) {
        InitializationCancellation::check(run_context);
        TP::Track &track = data.tracks[track_index];
        std::vector<CP::Candidate> local_candidates = track.candidates;
        CP::filter_candidates(local_candidates, run_context);
        if (local_candidates.empty())
            throw std::runtime_error(
                "Contiguous seam filtering removed every perimeter candidate.");
        const size_t local_index = best_local_candidate(local_candidates);
        if (local_index == size_t(-1))
            throw std::runtime_error(
                "Contiguous seam placement could not select a local candidate.");
        TrackState &state = states[track_index];

        const std::vector<size_t> lower =
            lower_tracks(data, states, track_index);
        const bool explicit_enforcer = std::any_of(
            local_candidates.begin(), local_candidates.end(),
            [](const CP::Candidate &candidate) {
                return candidate.enforced;
            });
        if (lower.empty() || explicit_enforcer) {
            state.final_point = local_candidates[local_index].point;
            state.ready = true;
            continue;
        }

        /* Append one temporary projection for each lower seam. The candidates
           retain unique stable_order values so filtering can copy/reorder them
           without losing their source-track association. */
        std::vector<CP::Candidate> combined = track.candidates;
        const size_t projection_begin = combined.size();
        std::vector<ProjectionRecord> projections;
        projections.reserve(lower.size());
        const ExtrusionEntity loop(track.loop);
        const LayerRegionIsland region_island(track.region_island);
        for (const size_t source_track : lower) {
            InitializationCancellation::check(run_context);
            const TP::Track &source = data.tracks[source_track];
            const c_point source_target = {
                states[source_track].final_point.x + source.instance_shift.x,
                states[source_track].final_point.y + source.instance_shift.y};
            const std::pair<c_point, double> projection =
                CP::project_onto_loop(loop, source_target);
            if (!std::isfinite(projection.second)) continue;

            CP::Candidate candidate;
            candidate.point = {
                projection.first.x - track.instance_shift.x,
                projection.first.y - track.instance_shift.y};
            candidate.width = track.candidates.front().width;
            candidate.nozzle = track.candidates.front().nozzle;
            candidate.stable_order = projection_begin + projections.size();
            for (const CP::Candidate &base : track.candidates)
                if (base.point.x == candidate.point.x &&
                    base.point.y == candidate.point.y) {
                    candidate.angle = base.angle;
                    break;
                }
            const SeamPlacementConstraints::Status status =
                constraints.classify_point(
                    loop, projection.first, scale_d(candidate.width));
            candidate.blocked =
                status == SeamPlacementConstraints::Status::Blocked;
            candidate.enforced =
                status == SeamPlacementConstraints::Status::Enforced;
            std::vector<CP::Candidate> enriched{candidate};
            CP::calculate_layer_attributes(
                region_island, enriched, run_context);
            candidate = enriched.front();
            candidate.stable_order = projection_begin + projections.size();
            combined.push_back(candidate);
            projections.push_back({candidate.stable_order, source_track,
                                   source_target, projection.second});
        }

        if (projections.empty()) {
            state.final_point = local_candidates[local_index].point;
            state.ready = true;
            continue;
        }
        CP::filter_candidates(combined, run_context);
        if (combined.empty()) combined = local_candidates;
        size_t selected = best_surviving_projection(
            data, combined, projections);
        if (selected == size_t(-1))
            selected = nearest_surviving_base(
                track, combined, projections, projection_begin);
        if (selected == size_t(-1)) {
            state.final_point = local_candidates[local_index].point;
        } else {
            state.final_point = combined[selected].point;
        }
        state.ready = true;
    }
}

ContiguousSession::ContiguousSession(
    storage_handle *storage, const plugin_run_context *run_context) :
    m_storage(storage), m_run_context(run_context)
{}

void ContiguousSession::initialize(
    const PrintingPlan &plan, const std::vector<PrintRegion> &regions)
{
    if (m_initialization_started)
        throw std::logic_error(
            "Contiguous seam placement sessions may only be initialized once.");
    m_initialization_started = true;
    InitializationCancellation::check(m_run_context);
    if (!plan.valid() || m_storage == nullptr)
        throw std::invalid_argument(
            "Contiguous seam placement needs a valid plan and temporary storage.");
    SeamPlacementConstraints constraints;
    constraints.initialize_for_regions(plan, m_storage, regions, m_run_context);
    m_data = TP::prepare(
        plan, regions, m_storage, constraints, "Contiguous", m_run_context);
    build_contiguous_targets(
        m_data, constraints, m_states, m_run_context);
    m_storage = nullptr;
    m_run_context = nullptr;
}

float ContiguousSession::get_priority(const ExtrusionEntity &,
                                      const LayerRegion &) const
{
    return 50.0f;
}

c_point ContiguousSession::place_seam(const ExtrusionEntity &loop,
                                      const LayerRegion &region,
                                      c_point start_position) const
{
    validate_loop(loop);
    if (!region.valid())
        throw std::invalid_argument(
            "Contiguous seam placement needs a valid LayerRegion.");
    const PrintRegion print_region = region.print_region();
    if (!print_region.valid() ||
        m_data.regions.count(print_region.handle()) == 0)
        throw std::invalid_argument(
            "Contiguous seam LayerRegion was not assigned to this provider session.");
    const std::map<const extrusion_entity_handle *, TP::LoopBinding>::const_iterator
        binding = m_data.bindings.find(loop.handle());
    if (binding == m_data.bindings.end())
        throw std::invalid_argument(
            "Contiguous seam loop was not present in the initialized plan.");
    if (binding->second.distance_only) {
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, start_position);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Contiguous seam could not project the approach position onto a distance-only loop.");
        return projection.first;
    }

    if (binding->second.external_track != size_t(-1)) {
        const size_t track_index = binding->second.external_track;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {
            local.x + binding->second.instance_shift.x,
            local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Contiguous seam could not project onto external loop geometry.");
        return projection.first;
    }

    c_point best{};
    double best_distance = std::numeric_limits<double>::infinity();
    size_t best_track = size_t(-1);
    for (size_t track_index = 0;
         track_index < m_data.tracks.size(); ++track_index) {
        const TP::Track &track = m_data.tracks[track_index];
        if (track.region_island != binding->second.region_island ||
            !TP::same_shift(track.instance_shift,
                            binding->second.instance_shift))
            continue;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {
            local.x + binding->second.instance_shift.x,
            local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (projection.second < best_distance ||
            (projection.second == best_distance &&
             best_track != size_t(-1) &&
             TP::stable_track_less(track, m_data.tracks[best_track]))) {
            best = projection.first;
            best_distance = projection.second;
            best_track = track_index;
        }
    }
    if (best_track == size_t(-1) || !std::isfinite(best_distance))
        throw std::runtime_error(
            "Contiguous seam source island has no external track.");
    return best;
}

ContiguousSeamPlacer &ContiguousSeamPlacer::instance(
    orchestrator_handle *orchestrator)
{
    static ContiguousSeamPlacer plugin(orchestrator);
    return plugin;
}

ContiguousSeamPlacer::ContiguousSeamPlacer(
    orchestrator_handle *orchestrator) : PluginBase(orchestrator)
{}

const char *ContiguousSeamPlacer::id_impl() const noexcept
{
    return kPluginId;
}

const char *ContiguousSeamPlacer::name_impl() const noexcept
{
    return "Contiguous";
}

const char *ContiguousSeamPlacer::description_impl() const noexcept
{
    return "Projects seams from directly connected lower-layer islands.";
}

const char *ContiguousSeamPlacer::exclusive_group_impl() const noexcept
{
    return kGroupId;
}

slicing_step_t ContiguousSeamPlacer::step_impl() const noexcept
{
    return SEAM_PLACER;
}

const char *const *ContiguousSeamPlacer::dependencies_impl() const noexcept
{
    return kDependencies;
}

int32_t ContiguousSeamPlacer::priority_impl() const noexcept
{
    return 600;
}

int32_t ContiguousSeamPlacer::exclusive_group_priority_impl() const noexcept { return 20; }

void ContiguousSeamPlacer::run_impl(
    const plugin_run_context *run_context) const
{
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr ||
        (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument(
            "Contiguous seam placement received an invalid run context.");
    std::vector<PrintRegion> regions;
    regions.reserve(context->region_count);
    for (uint32_t index = 0; index < context->region_count; ++index)
        regions.emplace_back(context->regions[index]);
    InitializationCancellation::check(run_context);
    context->instance = make_seam_placer_instance(
        std::unique_ptr<SeamPlacerSession>(
            new ContiguousSession(run_context->plugin_storage, run_context)),
        PrintingPlan(context->plan), regions);
}

} // namespace

std::unique_ptr<SeamPlacerSession> make_contiguous_seam_session(
    storage_handle *storage)
{
    return std::unique_ptr<SeamPlacerSession>(
        new ContiguousSession(storage));
}

void register_contiguous_seam_placer_plugin(
    orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator,
        ContiguousSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::ContiguousPlugin
