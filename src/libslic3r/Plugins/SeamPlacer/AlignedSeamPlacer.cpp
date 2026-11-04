///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "AlignedSeamPlacer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "SeamCandidatePreparation.hpp"
#include "SeamPlacementConstraints.hpp"
#include "SeamCostModel.hpp"
#include "SeamTrackPreparation.hpp"
#include "SeamInitializationCancellation.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

/*
Aligned seam placement
======================
Initialization chooses one local anchor for every actual external perimeter,
then grows deterministic chains over adjacent LayerIsland graph links. Only a
chain with at least six tracks is spline-smoothed; all other tracks retain the
local anchor selected from their own filtered candidates. The published state
contains values and borrowed loop identities only and is never mutated by
place_seam(), so parallel placement calls are safe while host geometry lives.
*/

namespace slic3r_api { namespace SeamPlacement { namespace AlignedPlugin {

namespace CP = CandidatePreparation;
namespace TP = TrackPreparation;

namespace {

constexpr double kCandidateCostTolerance = 0.3;
constexpr double kDistanceFactor = 4.0;
constexpr size_t kMinimumChainLength = 6;
constexpr double kSharpAngle = 55.0 * PI / 180.0;
const char *const kPluginId = "seam_placer.aligned";
const char *const kGroupId = "seam_placer_plugin";
const char *const kDependencies[] = { nullptr };

using Track = TP::Track;
using LoopBinding = TP::LoopBinding;
using LayerIslandNode = TP::LayerIslandNode;
using AlignedData = TP::Data;

struct TrackState {
    size_t local_candidate = size_t(-1);
    size_t chain_candidate = size_t(-1);
    double local_cost = std::numeric_limits<double>::infinity();
    c_point final_point{};
    bool assigned = false;
};

struct Continuation {
    size_t track = size_t(-1);
    size_t candidate = size_t(-1);
    double distance = std::numeric_limits<double>::infinity();
    double cost = std::numeric_limits<double>::infinity();
};

class AlignedSession;
class AlignedSeamPlacer;

/* Compute the provider's local score without visibility or travel. */
double local_cost(const CP::Candidate &candidate);
/* Compare complete candidate identities for deterministic final ties. */
bool stable_candidate_less(const Track &left_track, const CP::Candidate &left,
                           const Track &right_track, const CP::Candidate &right);
/* Select the local best candidate of every independently filtered perimeter. */
void prepare_local_anchors(AlignedData &data, std::vector<TrackState> &states,
                           const plugin_run_context *run_context);
/* Find one eligible continuation on a linked island at the adjacent level. */
Continuation find_continuation(const AlignedData &data,
                               const std::vector<TrackState> &states,
                               size_t source_track, size_t source_candidate,
                               bool upward,
                               const plugin_run_context *run_context);
/* Walk one graph direction, reserving only the selected branch. */
std::vector<std::pair<size_t, size_t>> walk_chain(
    const AlignedData &data, std::vector<TrackState> &states,
    size_t seed_track, size_t seed_candidate, bool upward,
    const plugin_run_context *run_context);
/* Smooth a long chain and preserve local anchors if fitting is not finite. */
void smooth_chain(const AlignedData &data,
                  const std::vector<std::pair<size_t, size_t>> &chain,
                  std::vector<TrackState> &states);
/* Seed chains by local quality and stable traversal identity. */
void build_aligned_targets(const AlignedData &data,
                           std::vector<TrackState> &states,
                           const plugin_run_context *run_context);
/* Verify that the request is a real non-empty closed geometry loop. */
void validate_loop(const ExtrusionEntity &loop);

class AlignedSession final : public SeamPlacerSession
{
public:
    explicit AlignedSession(storage_handle *storage,
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
    AlignedData m_data;
    std::vector<TrackState> m_states;
};

class AlignedSeamPlacer final : public PluginBase
{
public:
    static AlignedSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit AlignedSeamPlacer(orchestrator_handle *orchestrator);

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
    const double overhang_penalty = candidate.overhang > 0 && candidate.width > 0 ?
        std::clamp(2.0 * (candidate.overhang - candidate.width / 8.0) /
                       candidate.width, 0.0, 3.0) : 0.0;
    return overhang_penalty + 0.6 *
        Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle));
}

bool stable_candidate_less(const Track &left_track, const CP::Candidate &left,
                           const Track &right_track, const CP::Candidate &right)
{
    if (TP::stable_track_less(left_track, right_track)) return true;
    if (TP::stable_track_less(right_track, left_track)) return false;
    return left.stable_order < right.stable_order;
}

void prepare_local_anchors(AlignedData &data, std::vector<TrackState> &states,
                           const plugin_run_context *run_context)
{
    states.resize(data.tracks.size());
    for (size_t track_index = 0; track_index < data.tracks.size(); ++track_index) {
        InitializationCancellation::check(run_context);
        Track &track = data.tracks[track_index];
        CP::filter_candidates(track.candidates, run_context);
        if (track.candidates.empty())
            throw std::runtime_error("Aligned seam filtering removed every perimeter candidate.");
        TrackState &state = states[track_index];
        for (size_t candidate_index = 0;
             candidate_index < track.candidates.size(); ++candidate_index) {
            const double cost = local_cost(track.candidates[candidate_index]);
            if (cost < state.local_cost) {
                state.local_cost = cost;
                state.local_candidate = candidate_index;
            }
        }
        state.chain_candidate = state.local_candidate;
        state.final_point = track.candidates[state.local_candidate].point;
    }
}

Continuation find_continuation(const AlignedData &data,
                               const std::vector<TrackState> &states,
                               size_t source_track, size_t source_candidate,
                               bool upward,
                               const plugin_run_context *run_context)
{
    Continuation best;
    const Track &source = data.tracks[source_track];
    const CP::Candidate &source_point = source.candidates[source_candidate];
    const std::map<const layer_island_handle *, LayerIslandNode>::const_iterator source_node =
        data.islands.find(source.layer_island);
    if (source_node == data.islands.end()) return best;
    const std::vector<const layer_island_handle *> &links = upward ?
        source_node->second.upper : source_node->second.lower;
    const size_t expected_layer = upward ? source_node->second.layer + 1 :
        (source_node->second.layer == 0 ? size_t(-1) : source_node->second.layer - 1);

    for (const layer_island_handle *linked_handle : links) {
        InitializationCancellation::check(run_context);
        const std::map<const layer_island_handle *, LayerIslandNode>::const_iterator linked =
            data.islands.find(linked_handle);
        if (linked == data.islands.end() || linked->second.layer != expected_layer)
            continue;
        for (size_t track_index = 0; track_index < data.tracks.size(); ++track_index) {
            InitializationCancellation::check_periodic(
                run_context, track_index);
            const Track &target = data.tracks[track_index];
            if (states[track_index].assigned || target.layer_island != linked_handle ||
                target.stable_id.object != source.stable_id.object ||
                target.stable_id.instance != source.stable_id.instance)
                continue;
            for (size_t candidate_index = 0;
                 candidate_index < target.candidates.size(); ++candidate_index) {
                const CP::Candidate &candidate = target.candidates[candidate_index];
                const double cost = local_cost(candidate);
                if (cost > states[track_index].local_cost + kCandidateCostTolerance)
                    continue;
                const double distance = unscaled(std::hypot(
                    double(candidate.point.x) - source_point.point.x,
                    double(candidate.point.y) - source_point.point.y));
                const double scale = std::max({source_point.width, source_point.nozzle,
                                               candidate.width, candidate.nozzle});
                if (!std::isfinite(distance) || scale <= 0 ||
                    distance > kDistanceFactor * scale)
                    continue;
                const bool better = distance < best.distance ||
                    (distance == best.distance && (cost < best.cost ||
                    (cost == best.cost && (best.track == size_t(-1) ||
                    stable_candidate_less(target, candidate,
                        data.tracks[best.track],
                        data.tracks[best.track].candidates[best.candidate])))));
                if (better) {
                    best.track = track_index;
                    best.candidate = candidate_index;
                    best.distance = distance;
                    best.cost = cost;
                }
            }
        }
    }
    return best;
}

std::vector<std::pair<size_t, size_t>> walk_chain(
    const AlignedData &data, std::vector<TrackState> &states,
    size_t seed_track, size_t seed_candidate, bool upward,
    const plugin_run_context *run_context)
{
    std::vector<std::pair<size_t, size_t>> result;
    size_t source_track = seed_track;
    size_t source_candidate = seed_candidate;
    while (true) {
        const Continuation next = find_continuation(
            data, states, source_track, source_candidate, upward,
            run_context);
        if (next.track == size_t(-1)) break;
        states[next.track].assigned = true;
        states[next.track].chain_candidate = next.candidate;
        result.push_back({next.track, next.candidate});
        source_track = next.track;
        source_candidate = next.candidate;
    }
    return result;
}

void smooth_chain(const AlignedData &data,
                  const std::vector<std::pair<size_t, size_t>> &chain,
                  std::vector<TrackState> &states)
{
    if (chain.size() < kMinimumChainLength) return;
    std::vector<Slic3r::SeamCostModel::WeightedSplinePoint> observations;
    observations.reserve(chain.size());
    for (const std::pair<size_t, size_t> &entry : chain) {
        const Track &track = data.tracks[entry.first];
        const CP::Candidate &candidate = track.candidates[entry.second];
        double weight = 1.0 / (0.1 +
            Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle)));
        if (candidate.enforced) weight += 3.0;
        observations.push_back({unscaled(candidate.point.x),
                                unscaled(candidate.point.y),
                                unscaled(track.slice_z), weight});
    }
    std::vector<c_vec2d> fitted;
    if (!Slic3r::SeamCostModel::fit_weighted_cubic_bspline(
            observations, fitted))
        return;
    std::vector<c_point> smoothed;
    smoothed.reserve(chain.size());
    for (size_t index = 0; index < chain.size(); ++index) {
        const size_t track_index = chain[index].first;
        const CP::Candidate &candidate =
            data.tracks[track_index].candidates[chain[index].second];
        double blend = std::min(1.0, std::pow(std::abs(candidate.angle) /
                                              kSharpAngle, 3.0));
        if (candidate.enforced) blend = std::max(0.4, blend);
        const double x = blend * unscaled(candidate.point.x) +
                         (1.0 - blend) * fitted[index].x;
        const double y = blend * unscaled(candidate.point.y) +
                         (1.0 - blend) * fitted[index].y;
        if (!std::isfinite(x) || !std::isfinite(y)) return;
        smoothed.push_back({coord_t(scale_d(x)), coord_t(scale_d(y))});
    }
    for (size_t index = 0; index < chain.size(); ++index)
        states[chain[index].first].final_point = smoothed[index];
}

void build_aligned_targets(const AlignedData &data,
                           std::vector<TrackState> &states,
                           const plugin_run_context *run_context)
{
    std::vector<size_t> seeds(data.tracks.size());
    for (size_t index = 0; index < seeds.size(); ++index) seeds[index] = index;
    std::sort(seeds.begin(), seeds.end(), [&](size_t left, size_t right) {
        if (states[left].local_cost != states[right].local_cost)
            return states[left].local_cost < states[right].local_cost;
        return TP::stable_track_less(data.tracks[left], data.tracks[right]);
    });
    for (const size_t seed : seeds) {
        InitializationCancellation::check(run_context);
        if (states[seed].assigned) continue;
        states[seed].assigned = true;
        const size_t candidate = states[seed].local_candidate;
        std::vector<std::pair<size_t, size_t>> upper =
            walk_chain(data, states, seed, candidate, true, run_context);
        std::vector<std::pair<size_t, size_t>> lower =
            walk_chain(data, states, seed, candidate, false, run_context);
        std::vector<std::pair<size_t, size_t>> chain;
        chain.reserve(lower.size() + 1 + upper.size());
        for (std::vector<std::pair<size_t, size_t>>::reverse_iterator iterator =
                 lower.rbegin(); iterator != lower.rend(); ++iterator)
            chain.push_back(*iterator);
        chain.push_back({seed, candidate});
        chain.insert(chain.end(), upper.begin(), upper.end());
        smooth_chain(data, chain, states);
    }
}

void validate_loop(const ExtrusionEntity &loop)
{
    if (!loop.valid() || !loop.is_loop() || loop.empty())
        throw std::invalid_argument(
            "Aligned seam placement needs a non-empty closed loop.");
    const std::vector<c_point> points = loop.collect_points();
    if (points.size() < 2 || points.front().x != points.back().x ||
        points.front().y != points.back().y)
        throw std::invalid_argument(
            "Aligned seam placement rejects open or invalid loop geometry.");
}

AlignedSession::AlignedSession(storage_handle *storage,
                               const plugin_run_context *run_context) :
    m_storage(storage), m_run_context(run_context)
{}

void AlignedSession::initialize(const PrintingPlan &plan,
                                const std::vector<PrintRegion> &regions)
{
    if (m_initialization_started)
        throw std::logic_error(
            "Aligned seam placement sessions may only be initialized once.");
    m_initialization_started = true;
    InitializationCancellation::check(m_run_context);
    if (!plan.valid() || m_storage == nullptr)
        throw std::invalid_argument(
            "Aligned seam placement needs a valid plan and temporary storage.");
    SeamPlacementConstraints constraints;
    constraints.initialize_for_regions(plan, m_storage, regions, m_run_context);
    m_data = TP::prepare(
        plan, regions, m_storage, constraints, "Aligned", m_run_context);
    prepare_local_anchors(m_data, m_states, m_run_context);
    build_aligned_targets(m_data, m_states, m_run_context);
    m_storage = nullptr;
    m_run_context = nullptr;
}

float AlignedSession::get_priority(const ExtrusionEntity &,
                                   const LayerRegion &) const
{
    return 50.0f;
}

c_point AlignedSession::place_seam(const ExtrusionEntity &loop,
                                    const LayerRegion &region,
                                    c_point start_position) const
{
    validate_loop(loop);
    if (!region.valid())
        throw std::invalid_argument(
            "Aligned seam placement needs a valid LayerRegion.");
    const PrintRegion print_region = region.print_region();
    if (!print_region.valid() || m_data.regions.count(print_region.handle()) == 0)
        throw std::invalid_argument(
            "Aligned seam LayerRegion was not assigned to this provider session.");
    const std::map<const extrusion_entity_handle *, LoopBinding>::const_iterator binding =
        m_data.bindings.find(loop.handle());
    if (binding == m_data.bindings.end())
        throw std::invalid_argument(
            "Aligned seam loop was not present in the initialized plan.");
    if (binding->second.distance_only) {
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, start_position);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Aligned seam could not project the approach position onto a distance-only loop.");
        return projection.first;
    }

    if (binding->second.external_track != size_t(-1)) {
        const size_t track_index = binding->second.external_track;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {local.x + binding->second.instance_shift.x,
                                local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Aligned seam could not project onto external loop geometry.");
        return projection.first;
    }

    c_point best{};
    double best_distance = std::numeric_limits<double>::infinity();
    size_t best_track = size_t(-1);
    for (size_t track_index = 0; track_index < m_data.tracks.size(); ++track_index) {
        const Track &track = m_data.tracks[track_index];
        if (track.region_island != binding->second.region_island ||
            !TP::same_shift(track.instance_shift,
                            binding->second.instance_shift))
            continue;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {local.x + binding->second.instance_shift.x,
                                local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (projection.second < best_distance ||
            (projection.second == best_distance && best_track != size_t(-1) &&
             TP::stable_track_less(track, m_data.tracks[best_track]))) {
            best = projection.first;
            best_distance = projection.second;
            best_track = track_index;
        }
    }
    if (best_track == size_t(-1) || !std::isfinite(best_distance))
        throw std::runtime_error(
            "Aligned seam source island has no aligned external track.");
    return best;
}

AlignedSeamPlacer &AlignedSeamPlacer::instance(orchestrator_handle *orchestrator)
{
    static AlignedSeamPlacer plugin(orchestrator);
    return plugin;
}

AlignedSeamPlacer::AlignedSeamPlacer(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{}

const char *AlignedSeamPlacer::id_impl() const noexcept { return kPluginId; }
const char *AlignedSeamPlacer::name_impl() const noexcept { return "Aligned"; }
const char *AlignedSeamPlacer::description_impl() const noexcept
{
    return "Aligns locally suitable seam positions across connected layers.";
}
const char *AlignedSeamPlacer::exclusive_group_impl() const noexcept { return kGroupId; }
slicing_step_t AlignedSeamPlacer::step_impl() const noexcept { return SEAM_PLACER; }
const char *const *AlignedSeamPlacer::dependencies_impl() const noexcept { return kDependencies; }
int32_t AlignedSeamPlacer::priority_impl() const noexcept { return 600; }
int32_t AlignedSeamPlacer::exclusive_group_priority_impl() const noexcept { return 10; }

void AlignedSeamPlacer::run_impl(const plugin_run_context *run_context) const
{
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr ||
        (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument(
            "Aligned seam placement received an invalid run context.");
    std::vector<PrintRegion> regions;
    regions.reserve(context->region_count);
    for (uint32_t index = 0; index < context->region_count; ++index)
        regions.emplace_back(context->regions[index]);
    InitializationCancellation::check(run_context);
    context->instance = make_seam_placer_instance(
        std::unique_ptr<SeamPlacerSession>(
            new AlignedSession(run_context->plugin_storage, run_context)),
        PrintingPlan(context->plan), regions);
}

} // namespace

std::unique_ptr<SeamPlacerSession> make_aligned_seam_session(storage_handle *storage)
{
    return std::unique_ptr<SeamPlacerSession>(new AlignedSession(storage));
}

void register_aligned_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, AlignedSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::AlignedPlugin
